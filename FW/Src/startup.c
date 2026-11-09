/* ============================================================================
 * @file    startup.c
 * @brief   起动各子状态：Charge（预充）、Calib（飞车检测分发）、Align（IPD / 定位）、Startup（I/F 开环拖动），
 *          以及 I/F 与闭环之间的切换（openLoopHandover：开环 -> 闭环；Startup_PrepareOpenLoop / Commit：闭环 / 接管 -> 开环）。
 *          接口说明见 startup.h
 *
 * I/F 开环原理：电流矢量幅值恒定（OPENLOOP_CURRENT_A），角度按设定加速度积分旋转（不依赖转子位置）。
 *   转子自动跟随：电流矢量与转子 d 轴的夹角（负载角 δ）自动调整到 I*sin(δ)*Kt = 负载转矩 + 加速转矩。
 *   只要加速所需转矩小于 I*Kt 就不会失步。观测器与 PLL 在此期间只跟踪不控制，转速一致后切换闭环。
 * ==========================================================================*/
#include "startup.h"
#include "supervisor.h"
#include "foc_core.h"
#include "est.h"
#include "hal.h"
#include "speed_ctrl.h"
#include "protect.h"
#include "qmath.h"
#include "flux_obs.h"
#include "meas.h"
#include "flystart.h"
#include "identify.h"
#include "ipd.h"
#include "param.h"

Startup_TypeDef gStartup;

volatile OpenLoopDiag_TypeDef gOpenLoopDiag;
static volatile int16_t s_alignTheta;      /* 定位角（慢环写，快环读） */
static uint16_t s_alignTicks;
static volatile int32_t s_olTheta;         /* 开环角 Q16.16（快环推进） */
static volatile int32_t s_olSpeed;         /* 开环转速 Q31（快环斜坡） */
static volatile int32_t s_olTarget;        /* 目标转速 Q31，带方向（慢环写） */
static int32_t s_pllFilt;                  /* PLL 转速一阶滤波 Q31（慢环，OPENLOOP_PLL_FILT_TAU_S） */
static uint16_t s_agree, s_hold;
/* 开环加速度按实际开环电流缩放：OPENLOOP_ACC_RPM_S 为 OPENLOOP_CURRENT_A 下最大能力的 1/3，回退 / 反转时电流增大，
 * 保持相同余量时加速度也按电流比例增大，过零更快。s_accScaleQ12 = I / OPENLOOP_CURRENT_A（Q12，冷起动 = 4096） */
static int32_t s_accScaleQ12 = 4096;
static int32_t s_olCur, s_olCurTarget;      /* 开环电流当前值 / 目标值（Q15）；回退时由小升至目标（CURRENT_SLEW_A_S） */
static volatile int32_t s_accStep = OPENLOOP_ACC_STEP_Q31;   /* 快环读 */
static int32_t s_accStepFull;                         /* 开环电流达到目标后的加速度步长 */
static int32_t s_dampQ8;                   /* 定位阻尼增益：Q15 电流 / Q15 电压，Q8 */
static int32_t s_emfFilt;                  /* 定位时 q 轴反电势（滤波），Q15 电压 << ALIGN_EMF_FILT_SH */
static uint16_t s_still;                   /* 定位第二段：连续"静止"的拍数 */
static volatile uint8_t s_ipd;             /* 1 = Align 子状态正在执行 IPD（慢环写，快环读） */

/* 当前开环转速（Q15），供观测器相位补偿等使用 */
int16_t Startup_OpenLoopSpeedQ15(void) { return (int16_t)(s_olSpeed >> 16); }

/* 开环目标转速。|指令| >= OPENLOOP_SPIN_MIN_RPM：朝指令方向拖动，下限 OPENLOOP_HANDOVER_RPM（指令更低时先拖到切换转速、切闭环后再降速），
 * 上限 OPENLOOP_FALLBACK_RPM（观测器在此以上可靠）；超过 OPENLOOP_HANDOVER_RPM 后边加速边检查一致性，一致即切换，无需停顿等待。
 * 否则以指令转速持续开环运行。 */
static int32_t openLoopTarget(void)
{
    int32_t c = gSpd.cmd, a = (c < 0 ? -c : c) << 16;
    if (a < ((int32_t)OPENLOOP_SPIN_MIN_Q15 << 16)) return c << 16;
    if (a < OPENLOOP_HANDOVER_Q31) a = OPENLOOP_HANDOVER_Q31;
    if (a > OPENLOOP_FALLBACK_Q31) a = OPENLOOP_FALLBACK_Q31;
    return c > 0 ? a : -a;
}

/* asin(x)，x 为 Q15（限 ±0.95），返回 Q15 角度（65536 = 360°）。基于正弦表二分查找，慢环使用 */
static int16_t asinQ15(int32_t x)
{
    int32_t lo = -16384, hi = 16384, mid;
    int i;
    if (x > 31130) x = 31130;
    if (x < -31130) x = -31130;
    for (i = 0; i < 15; i++) {
        mid = (lo + hi) / 2;
        if (FocMath_SinCos((int16_t)mid).s16Sin < x) lo = mid; else hi = mid;
    }
    return (int16_t)lo;
}

/* 由 Spin 慢环查询是否需要回退到 I/F：转速已低于 OPENLOOP_FALLBACK_RPM（观测器可靠下限附近），且指令反向（需过零）
 * 或指令本身低于闭环最低转速时返回 1 */
uint8_t Startup_OpenLoopWantsFallback(void)
{
    int32_t filt = gMeas.speed, cmd = gSpd.cmd;
    int32_t af = filt < 0 ? -filt : filt;
    if (af >= OPENLOOP_FALLBACK_Q15) return 0;
    /* 指令反向，或指令低于闭环最低转速：交给 I/F */
    return (uint8_t)((cmd > 0) != (filt > 0) || (cmd < 0 ? -cmd : cmd) < OPENLOOP_SPIN_MIN_Q15);
}

/* 进入 Align（飞车检测判为静止）：清除定位状态。默认先执行 IPD（STARTUP_IPD = 1），IPD 结果不可信时再定位；
 * 辨识模式（IDENT_TEST）下定位到指定角或按辨识流程执行 */
void Startup_OnEnterAlign(void)
{
    s_alignTicks = 0;
    s_emfFilt = 0;
    s_still = 0;
    s_alignTheta = 0;
    FocCore_SetIdqRef(0, 0);
    gOpenLoopDiag.stage = OPENLOOP_STAGE_ALIGN;
    s_ipd = 0;
    if (IDENT_TEST && !gIdentCfg.run) { s_alignTheta = Ident_AlignTheta(); Ident_Reset(); }     /* 参数辨识：定位到指定角 */
    else {
        /* 运行辨识（run = 1）正常起动：上一次静止测量停在 phase 4，需清除，否则 Align 会一直交由测量程序处理 */
        if (IDENT_TEST) Ident_Idle();
        if (STARTUP_IPD) { Ipd_Start(); s_ipd = 1; gOpenLoopDiag.stage = OPENLOOP_STAGE_IPD; }    /* 先检测初始位置，检测失败再定位 */
    }
}

/* 从闭环回退到 I/F 时的开环电流：至少为 OPENLOOP_CURRENT_A；需能提供此刻的 iq 且负载角不超过 δmax（|iq| / sin(δmax)）；
 * 需过零（反转）时至少为 OPENLOOP_REV_CURRENT_A；不超过 OPENLOOP_WARM_MAX_A */
int16_t Startup_OpenLoopWarmCurrent(int16_t iq, uint8_t reversal)
{
    int32_t cur = OPENLOOP_IQ_Q15, aiq = iq < 0 ? -(int32_t)iq : iq;
    int32_t need = (int32_t)(((int64_t)aiq * OPENLOOP_WARM_KI_Q15) >> 15);      /* |iq| / sin(δmax) */
    if (need > cur) cur = need;
    if (reversal && cur < OPENLOOP_REV_IQ_Q15) cur = OPENLOOP_REV_IQ_Q15;
    if (cur > OPENLOOP_WARM_IMAX_Q15) cur = OPENLOOP_WARM_IMAX_Q15;
    return (int16_t)cur;
}

/* 进入 I/F 分为"准备 + 提交"两步：
 *   Startup_PrepareOpenLoop：只计算、不修改任何状态，输入全部作为参数（调用者在临界区外调用，可被快环打断）。
 *     计算开环电流起始值 / 目标值、加速度步长，以及开环角相对估计角的偏移 asin(iq / I)。
 *   Startup_CommitOpenLoop：只赋值（几十个时钟，调用者在临界区内调用）。开环角 = 提交时刻的 PLL 角 + 偏移，
 *     准备期间即使被快环打断、PLL 角已前进也不受影响。
 * 拆分的目的是将耗时的开方查找与除法移出临界区 / 快环，缩短关中断时间。
 *   warm = 1：从运行状态接管。iq = 闭环此刻的 q 轴电流指令，speedQ15 = 开环起始转速；
 *     revCur / revAcc = 开环需过零（目标方向与 PLL 转速 / 起始转速方向相反），分别决定开环电流和加速度缩放。
 *   warm = 0：冷起动（定位结束），开环角 = 定位角、零速。 */
void Startup_PrepareOpenLoop(uint8_t warm, int16_t iqNow, uint8_t revCur, uint8_t revAcc, int16_t speedQ15, OpenLoopEntry_TypeDef *e)
{
    int32_t cur = OPENLOOP_IQ_Q15;
    e->warm = warm;
    e->thetaOffset = 0;
    if (warm) {
        /* 转矩连续：开环电流 I 沿 theta_ol，其在转子 q 轴的分量为 I*sin(theta_ol - theta_est)。
         * 令其等于闭环此刻的 iq：theta_ol = theta_est + asin(iq/I)。
         * 开环转矩 = I·Kt·sin(δ)，δ 超过 90° 即失步。I 的取值使 δ 不超过 OPENLOOP_WARM_DELTA_DEG（45°）：
         * 为磁链 / 转矩系数偏小 25% 以上留出余量（sin 45° / 0.75 = 0.94 < 1）。
         * 反转（开环需过零）时 I 至少为 OPENLOOP_REV_CURRENT_A——过零段需先将转子制动再反向拖动，转矩需求最大。 */
        int32_t iq = iqNow, aiq = iq < 0 ? -iq : iq, i0, sp;
        cur = Startup_OpenLoopWarmCurrent((int16_t)iq, revCur);
        /* 起始电流：不直接给到 cur，而是从 max(|iq| / sin45°, cur / 4) 起、按 CURRENT_SLEW_A_S 升至 cur。
         * 回退前闭环已将 id 渐变到 0，切换时刻 d 轴仅增加 i0·cos δ0 = |iq|（δ0 = 45°）；回退时的减速度已与
         * I/F 衔接（approachFallback），|iq| 较小。
         * δ0 <= 45°，电流增大时 δ 只会减小，不会失步 */
        i0 = (int32_t)(((int64_t)aiq * OPENLOOP_WARM_KI_Q15) >> 15);
        if (i0 < (cur >> 2)) i0 = cur >> 2;
        if (i0 > cur) i0 = cur;
        e->olCurTarget = cur;
        cur = i0;
        e->thetaOffset = (uint32_t)(uint16_t)asinQ15(((int32_t)iq << 15) / cur) << 16;
        /* 开环转速从滤波后的转速（gMeas.speed，SPEED_FB_FILT_HZ）开始，而非 PLL 瞬时值：回退点位于观测器可用下限附近，
         * PLL 瞬时转速噪声较大，若开环转速初值偏差过大，进入 I/F 后易失步。
         * 飞车接管时二者相同（Est_OnCatch 均设为测得的转速） */
        sp = (int32_t)speedQ15 << 16;
        if (sp > OPENLOOP_WARM_MAX_Q31) sp = OPENLOOP_WARM_MAX_Q31;      /* 仅防止异常值，正常情况 <= 回退转速 */
        if (sp < -OPENLOOP_WARM_MAX_Q31) sp = -OPENLOOP_WARM_MAX_Q31;
        e->olSpeed = sp;
    } else {
        e->olSpeed = 0;
        e->olCurTarget = cur;
    }
    e->olCur = cur;
    /* 电流沿开环角方向（开环坐标系 d 轴）：转子滞后 delta，转矩 ∝ I*sin(delta)，
     * 加速时 delta 自然增大。冷起动时与定位电流方向连续，避免像施加在 q 轴那样
     * 一开始就输出满转矩、激起新的摆动。 */
    /* 反转过零时开环电流至少为 OPENLOOP_REV_CURRENT_A：加速度按 OPENLOOP_REV_CURRENT_A / OPENLOOP_CURRENT_A 增大，余量与冷起动相同
     * （按"所需 iq"增加的那部分电流用于已有转矩需求，不用于加速）。冷起动 / 同向 = 1 */
    e->accScaleQ12 = (warm && revAcc) ? (int32_t)(((int64_t)OPENLOOP_REV_IQ_Q15 << 12) / OPENLOOP_IQ_Q15) : 4096;
    e->accStepFull = (int32_t)(((int64_t)OPENLOOP_ACC_STEP_Q31 * e->accScaleQ12) >> 12);
    /* 开环加速度与开环电流成正比：电流仍按 CURRENT_SLEW_A_S 向目标上升时，可产生的转矩相应受限，
     * 加速度按同一比例给出，负载角余量（OPENLOOP_ACC_RPM_S 取最大加速度的 1/3）全程保持不变。
     * accStepFull * 电流 < 2^31 由配置检查 OPENLOOP_ACC_STEP_FITS 保证（32 位乘除） */
    e->accStep = (e->accStepFull * cur) / e->olCurTarget;
}

/* 进入 I/F 的"提交"（在临界区内 / 快环中调用，只赋值）：开环目标、开环角（冷起动 = 定位角；接管 = PLL 角 + 预先计算的偏移）、
 * 开环转速、电流、加速度，电流指令沿开环 d 轴 */
void Startup_CommitOpenLoop(const OpenLoopEntry_TypeDef *e)
{
    s_olTarget = openLoopTarget();
    Protect_PhaseReset();           /* 缺相窗口从进入 I/F 时重新计 */
    s_olTheta = e->warm ? (int32_t)((uint32_t)gEst.pll.s32Theta + e->thetaOffset)
                        : (int32_t)gFoc.theta << 16;        /* 冷起动 = 定位角，电流方向与定位时相同，无冲击 */
    s_olSpeed = e->olSpeed;
    s_olCur = e->olCur;
    s_olCurTarget = e->olCurTarget;
    s_accScaleQ12 = e->accScaleQ12;
    s_accStepFull = e->accStepFull;
    s_accStep = e->accStep;
    FocCore_SetIdqRef((int16_t)e->olCur, 0);
    s_pllFilt = gEst.pll.s32Speed;
    s_agree = 0; s_hold = 0;
    gOpenLoopDiag.stage = OPENLOOP_STAGE_RUN;
}

/* 飞车接管进入 I/F（快环中执行）：接管时闭环电流 iq = 0（Sup_SwitchCalibOpenLoop 先清零），入口参数除起始转速外
 * 均为常数，仅分"开环需过零 / 不过零"两种。上电时（Startup_Init）预先计算，快环中按方向选取一份、填入转速并提交：常数时间 */
static OpenLoopEntry_TypeDef s_catchIf[2];

void Startup_CommitCatchOpenLoop(int16_t spd)
{
    OpenLoopEntry_TypeDef e = s_catchIf[(openLoopTarget() > 0) != (spd > 0)];
    int32_t sp = (int32_t)spd << 16;
    if (sp > OPENLOOP_WARM_MAX_Q31) sp = OPENLOOP_WARM_MAX_Q31;
    if (sp < -OPENLOOP_WARM_MAX_Q31) sp = -OPENLOOP_WARM_MAX_Q31;
    e.olSpeed = sp;
    Startup_CommitOpenLoop(&e);
}

/* 开环需过零（反转）：开环目标方向与当前转速方向相反 */
uint8_t Startup_OpenLoopReversal(int32_t speedSign)
{
    int32_t t = openLoopTarget();
    return (uint8_t)((t > 0) != (speedSign > 0));
}

/* 转矩不变切换：开环时电流沿 theta_ol（开环 d 轴），其在转子 q 轴上的分量为
 * I*sin(theta_ol - theta_r)。 */
static void openLoopHandover(void)
{
    /* 融合切换：控制角由开环角 theta_ol 换为估计角 theta_est，电流矢量与电压矢量在空间中保持不变——
     * I/F 电流（开环 d 轴，幅值 I）在估计坐标系中为 (I cos d, I sin d)，d = theta_ol - theta_est；
     * 电流 PI 积分器（= 电压）同样旋转到估计坐标系。此拍无任何跳变，观测器此刻的角度误差也原样保留。
     * 之后 OPENLOOP_BLEND_S 内：速度 PI（预置为负载部分）照常全额输出，id0 及 q 轴剩余分量叠加其上并按 (1 - w) 淡出；
     * 速度斜坡从开环转速、以 I/F 加速度继续，斜率与加速前馈一起过渡到运行值——
     * 角度误差引起的转矩逐渐淡出，由速度环补偿，而非一次性形成冲击。
     * 关中断：控制角（runState）、电流指令、积分器必须在同一快环拍间隙内同时切换。 */
    int16_t d = (int16_t)((s_olTheta >> 16) - (gEst.pll.s32Theta >> 16));
    sAngle_Trig a = FocMath_SinCos(d);
    int16_t i = gFoc.idqRef.s16D;
    int16_t id0 = QMath_MulQ15(i, a.s16Cos), iq0 = QMath_MulQ15(i, a.s16Sin), iqPi, ff0 = 0;
    int32_t ol = s_olSpeed;
    /* 切换时开环仍在加速：该部分转矩已知（模型 OPENLOOP_ACC_RPM_S / MOTOR_ACC_RPM_S_PER_A），作为加速前馈的起点（速度环中
     * 前馈随斜坡斜率一起过渡到运行值），而非依赖含角度误差的 I sin d 估计。速度 PI 只承接剩余的负载部分。 */
    if (ol > 0 && s_olTarget > ol) ff0 = (int16_t)((gGain.openLoopAccFfIq * s_accScaleQ12) >> 12);
    if (ol < 0 && s_olTarget < ol) ff0 = (int16_t)(-((gGain.openLoopAccFfIq * s_accScaleQ12) >> 12));
    iqPi = (int16_t)(iq0 - ff0);
    /* 速度 PI 预置：切换前转子只承担负载（加速转矩已由 ff0 给出），真实转矩必为电动方向。iq0 若为反向，
     * 只可能来自观测器此刻的角度误差，它与 id0 一样属于"误差坐标系中的分量"，随融合淡出；
     * 速度 PI 只承接电动方向部分（否则融合结束后速度 PI 还需自行补偿该反向分量，低速时易导致失锁）。 */
    if (s_olSpeed > 0 && iqPi < 0) iqPi = 0;
    if (s_olSpeed < 0 && iqPi > 0) iqPi = 0;
    {
        IRQ_LOCK();
        FocCore_SetIdqRef(id0, iqPi);       /* Sup_SwitchStartupSpin -> SpeedCtrl_OnEnterSpin 以此预置速度 PI */
        FocCore_RotatePI(a);
        gOpenLoopDiag.deltaAtHandover = d;
        gOpenLoopDiag.iqAtHandover = iq0;
        gOpenLoopDiag.handovers++;
        gOpenLoopDiag.stage = OPENLOOP_STAGE_HANDOVER;
        Sup_SwitchStartupSpin();
        FocCore_SetIdqRef(id0, iq0);        /* 本拍电流矢量保持不变 */
        gSpd.idqCmd.s16Q = iqPi;            /* Spin 慢环中速度环从 iqPi 开始，iq0 - iqPi 由融合叠加 */
        SpeedCtrl_BlendStart(id0, (int16_t)(iq0 - iqPi - ff0), ol, (int16_t)s_accScaleQ12, ff0);   /* 速度 PI 给出 iqPi、前馈给出 ff0，融合只叠加剩余部分 */
        IRQ_UNLOCK();
    }
}

/* 上电：预充时间、飞车门限及"检测到转动后等待停稳"的时间（含 ln，上电计算一次）、定位阻尼增益及各段时长（摆周期的倍数）、
 * 飞车接管进入 I/F 的两份入口参数（同向 / 过零） */
void Startup_Init(void)
{
    gStartup.preChargeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;

    gFly.s16BemfAmpTh = Q15(FLYCURRENT_TH_PU);
    {   /* 检测到过转动时的"等待停稳"时间：FLYBRAKE_TIME + τ·ln(门限转速 / 残余转速)。ln 用 atanh 级数计算（上电一次，不链接 libm） */
        double x = FLY_STILL_RPM / FLY_STILL_END_RPM, y = (x - 1.0) / (x + 1.0), y2 = y * y, t = y, ln = 0.0, ms;
        int k;
        for (k = 1; k < 60; k += 2) { ln += t / k; t *= y2; }
        ms = FLY_STILL_BLOCKS + FLY_BRAKE_TAU_S * 2.0 * ln * 1000.0;
        gFly.stillSpinMs = (uint16_t)(ms > 60000.0 ? 60000.0 : ms);
    }

    /* 仅上电时计算一次（含开方与浮点） */
    s_dampQ8 = (int32_t)(ALIGN_C1 * (double)QMath_Isqrt64((uint64_t)(ALIGN_K_NM * ALIGN_J * 1099511627776.0)) / 1048576.0);
    Startup_PrepareOpenLoop(1, 0, 0, 0, 0, &s_catchIf[0]);   /* 飞车接管进入 I/F 的入口参数（iq = 0），同向 / 过零两份 */
    Startup_PrepareOpenLoop(1, 0, 1, 1, 0, &s_catchIf[1]);
}

/* 停机复位：预充计数清零 */
void Startup_ResetRun(void)
{
    gStartup.preChargeCnt = 0;
    gStartup.preChargePhase = 0;
}


/* ---------------------------------------------------------------------------
 * Charge：三相下桥轮流导通为自举电容充电，PRECHARGE_TIME 后进入 Calib
 * -------------------------------------------------------------------------*/
/* 预充快环：上下桥均关闭，三相下桥轮流导通（每拍切换一相），为上桥驱动的自举电容充电；到时进入飞车检测 */
void Startup_ChargeFast(void)
{
    HAL_PwmLowOff();
    HAL_PwmHighOff();
    HAL_PwmSetHalfDuty();

    if (++gStartup.preChargePhase > 2) gStartup.preChargePhase = 0;
    if (gStartup.preChargePhase == 0)      HAL_PwmLowOnA();
    else if (gStartup.preChargePhase == 1) HAL_PwmLowOnB();
    else                               HAL_PwmLowOnC();

    if (gStartup.preChargeCnt++ >= gStartup.preChargeCmd) {
        Sup_SwitchChargeCalib();
    }
}

/* 预充慢环：仅做测量滤波 */
void Startup_ChargeSlow(void)
{
    Meas_SlowFilter();
}

/* ---------------------------------------------------------------------------
 * Calib：飞车检测（flystart.c）。正在转动且转速足够 -> Spin；低速 -> I/F；静止 -> Align；制动不停 -> 故障
 * -------------------------------------------------------------------------*/
/* 飞车检测快环：执行 FlyStart_DetectFast，按结论切换到 Align / Spin / I/F 或报故障 */
void Startup_CalibFast(void)
{
    switch (FlyStart_DetectFast(gSpd.cmd)) {
    case FLY_STILL:
        Sup_SwitchCalibAlign();
        break;
    case FLY_CATCH:
        Sup_SwitchCalibSpin();
        break;
    case FLY_CATCH_OPENLOOP:
        Sup_SwitchCalibOpenLoop();
        break;
    case FLY_TIMEOUT:
        Sup_CountRestart();
        Sup_RaiseFault(MC_FAULT_FLY_TIMEOUT);
        break;
    default:
        Meas_SpeedSource(gFly.speed);       /* 仅用于诊断显示 / 滤波器 */
        break;
    }
}

/* 飞车检测慢环：测量滤波；对快环保存的电流平方值开方用于诊断显示（快环不开方） */
void Startup_CalibSlow(void)
{
    Meas_SlowFilter();
    gFly.pulseAmp = (int16_t)QMath_Isqrt32(gFly.pulseM2);     /* 诊断：快环只保存平方 */
    gFly.amp = QMath_Sat16(2 * (int32_t)QMath_Isqrt32(gFly.ampM2));
}

/* ---------------------------------------------------------------------------
 * Align：d 轴电流斜坡升至 ALIGN_A 定位，q 轴按反电势施加阻尼，停稳后进入 I/F
 * -------------------------------------------------------------------------*/
/* Align 快环：辨识模式 / IPD 时由其直接输出电压；定位时电流环按定位角运行（电流指令由慢环给出） */
void Startup_AlignFast(void)
{
    FocCore_Clarke();
    if (IDENT_TEST && Ident_Fast()) return;     /* 参数辨识的脉冲段：调制由辨识程序输出 */
    if (STARTUP_IPD && s_ipd) { Ipd_Fast(); return; }      /* IPD：旁路电流环，直接输出脉冲电压 */
    gFoc.theta = s_alignTheta;
    gFoc.thetaInv = gFoc.theta;
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

/* Align 慢环：
 *   IPD 中：等待 12 个脉冲完成，判断对比度，可信 -> 从 N 极角直接进入 I/F；不可信 -> 退回定位；
 *   定位：d 轴电流按斜坡升至 ALIGN_A，q 轴按反电势施加阻尼（转子摆动时施加反向转矩抑制），
 *         达到最短时间且反电势持续低于门限（已停稳），或超过最长时间 -> 进入 I/F */
void Startup_AlignSlow(void)
{
    int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15;
    int32_t emf, iq;
    if (IDENT_TEST && Ident_Active()) { Ident_Slow(); Meas_SlowFilter(); Meas_SpeedHoldZero(); return; }
    if (STARTUP_IPD && s_ipd) {
        if (gIpd.state == IPD_STATE_DONE) {
            if (Ipd_Evaluate()) {
                Sup_SwitchIpdStartup(gIpd.theta);  /* 从测得的 N 极角开始拖动，无需定位 */
            } else {
                /* 对比度不足（饱和不明显 / 转子在动）：本次退回定位。PWM 已为零电压，电流约为 0。
                 * 先清积分器再清 s_ipd（分别为一次 32 / 8 位写）：快环看到 s_ipd = 0 时积分器已为 0 */
                FocCore_PresetPI(0, 0);
                gOpenLoopDiag.stage = OPENLOOP_STAGE_ALIGN;
                s_ipd = 0;
            }
        }
        Meas_SlowFilter();
        Meas_SpeedHoldZero();
        return;
    }
    if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
    /* 阻尼：q 轴反电势 e_q = w*psi_f，经 ALIGN_EMF_FILT_TAU_S 低通，iq = -k*e_q。
     * e_q 取 q 轴电流 PI 的积分项（= Rs*iq + e_q，稳态电压）减去 Rs*iq_实测，而不用 PI 总输出：
     * 总输出中含 Kp*误差 的瞬态分量，阻尼指令变化时会跳变，环路增益 > 1 时会振荡。 */
    emf = (gFoc.piQ.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, gFoc.idq.s16Q);
    s_emfFilt += emf - (s_emfFilt >> ALIGN_EMF_FILT_SH);
    iq = -((s_emfFilt >> ALIGN_EMF_FILT_SH) * s_dampQ8 >> 8);
    iq = QMath_Clamp(iq, ALIGN_ID_Q15);
    FocCore_SetIdqRef((int16_t)id, (int16_t)iq);
    ++s_alignTicks;
    {   /* 0° 定位，满足最短时间后等待停稳 */
        int32_t e = s_emfFilt >> ALIGN_EMF_FILT_SH;
        if (e < 0) e = -e;
        if (s_alignTicks > ALIGN_MIN_TICKS && e < gGain.alignStillEmfQ15) { if (s_still < 0xFFFF) s_still++; } else s_still = 0;
        if ((s_alignTicks >= ALIGN_MIN_TICKS && s_still >= ALIGN_STILL_TICKS) || s_alignTicks >= ALIGN_MAX_TICKS) {
            gOpenLoopDiag.alignTime = s_alignTicks;
            if (IDENT_TEST && !gIdentCfg.run) Ident_Begin();  /* 参数辨识：不切入 I/F，从此开始测量 */
            else Sup_SwitchAlignStartup();  /* 观测器初始化为 |psi_f|∠0 */
        }
    }
    Meas_SlowFilter();
    Meas_SpeedHoldZero();
}

/* ---------------------------------------------------------------------------
 * Startup（I/F）：电流矢量开环旋转（角度 = 开环积分），观测器 + PLL 只跟踪；转速一致后切换到 Spin。
 * -------------------------------------------------------------------------*/
/* I/F 快环：观测器 + PLL 只跟踪；开环转速按加速度步长趋近目标，开环角积分；电流环按开环角运行 */
void Startup_StartupFast(void)
{
    int32_t sp = s_olSpeed, tgt = s_olTarget, step;
    FocCore_Clarke();
    (void)Est_TrackTick();              /* 观测器 + PLL 只跟踪，不参与控制 */
    if (sp < tgt)      { sp += s_accStep; if (sp > tgt) sp = tgt; }
    else if (sp > tgt) { sp -= s_accStep; if (sp < tgt) sp = tgt; }
    s_olSpeed = sp;
    step = (sp >> 15) * (int32_t)gEst.pll.s16ThGain;           /* 一拍角度增量 Q16.16 */
    gFoc.theta = (int16_t)(s_olTheta >> 16);
    gFoc.thetaInv = gFoc.theta + (int16_t)(((step >> 8) * INVPARK_ADV_Q8) >> 16);
    s_olTheta = (int32_t)((uint32_t)s_olTheta + (uint32_t)step);
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
    Meas_SpeedSource((int16_t)(sp >> 16));          /* 经 Meas_SlowFilter 后 gMeas.speed 跟随开环转速 */

    /* 缺相检测：累加各相电流 */
    Protect_PhaseFast();
}


/* I/F 慢环：
 *   更新目标、PLL 转速滤波；回退进入时开环电流按 CURRENT_SLEW_A_S 升至目标（加速度同比例）；
 *   缺相检查；运行辨识模式下先测磁链；
 *   开环转速超过切换转速 OPENLOOP_HANDOVER_RPM 后，PLL 转速与开环转速一致（相对误差 < OPENLOOP_AGREE_RATIO）
 *   并持续 OPENLOOP_AGREE_S -> 切闭环；拖动超时仍不一致 -> 堵转故障 */
void Startup_StartupSlow(void)
{
    int32_t ol = s_olSpeed;
    s_olTarget = openLoopTarget();
    s_pllFilt += (gEst.pll.s32Speed >> OPENLOOP_PLL_FILT_SH) - (s_pllFilt >> OPENLOOP_PLL_FILT_SH);
    if (s_olCur < s_olCurTarget) {      /* 回退时开环电流升至目标 */
        s_olCur += CURRENT_SLEW_STEP_Q15;
        if (s_olCur > s_olCurTarget) s_olCur = s_olCurTarget;
        s_accStep = (s_accStepFull * s_olCur) / s_olCurTarget;     /* 32 位，见 Startup_PrepareOpenLoop */
        FocCore_SetIdqRef((int16_t)s_olCur, 0);
    }
    gOpenLoopDiag.olSpeed = (int16_t)(ol >> 16);
    gOpenLoopDiag.pllFilt = (int16_t)(s_pllFilt >> 16);

    /* 缺相：与 Spin 相同的判据（protect.h），电周期按开环转速计 */
    if (Protect_PhaseSlow((int16_t)(ol >> 16))) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        Sup_CountRestart();
    } else if (IDENT_TEST && Ident_OpenLoopFluxSlow(ol, s_olTarget)) {
        /* 运行辨识：在开环中先测磁链（identify.c），测得后生效，再按下述一致性判据切闭环 */
    } else if ((s_olTarget >= OPENLOOP_HANDOVER_Q31 && ol >= OPENLOOP_HANDOVER_Q31) ||
               (s_olTarget <= -OPENLOOP_HANDOVER_Q31 && ol <= -OPENLOOP_HANDOVER_Q31)) {
        /* 已超过切换转速且与目标同向（反转时开环需先过零）：边拖动边检查。PLL 滤波 16 ms 在 I/F 加速度下
         * 仅滞后约 2 rpm，远小于 OPENLOOP_AGREE_RATIO 的容差，无需补偿 */
        int64_t d = (int64_t)s_pllFilt - ol;
        int64_t tol = ((int64_t)(ol < 0 ? -ol : ol) * OPENLOOP_AGREE_Q15) >> 15;
        if (d < 0) d = -d;
        if (d < tol) s_agree++; else s_agree = 0;
        gOpenLoopDiag.agreeCnt = s_agree;
        if (s_agree >= OPENLOOP_AGREE_TICKS) {
            openLoopHandover();
        } else if (++s_hold > OPENLOOP_TIMEOUT_TICKS) {
            gOpenLoopDiag.timeouts++;
            gOpenLoopDiag.stage = OPENLOOP_STAGE_TIMEOUT;
            Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
            Sup_CountRestart();
        }
    }
    Meas_SlowFilter();
}
