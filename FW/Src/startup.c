/* ============================================================================
 * @file    startup.c
 * @brief   起动各子状态：Charge（预充）、FlyCheck（飞车检测分发）、Align（IPD / 摆动起动 / 辨识定位）；
 *          I/F 开环（OpenLoop）在 openloop.c。
 *          接口说明见 startup.h
 * ==========================================================================*/
#include "startup.h"
#include "openloop.h"
#include "supervisor.h"
#include "foc_core.h"
#include "hal.h"
#include "speed_ctrl.h"
#include "protect.h"
#include "qmath.h"
#include "flux_obs.h"
#include "meas.h"
#include "flystart.h"
#include "identify.h"
#include "ipd.h"
#include "spin.h"
#include "param.h"

Startup_TypeDef gStartup;

static volatile int16_t s_alignTheta;      /* 定位角（慢环写，快环读） */
static uint16_t s_alignTicks;
#if IDENT_TEST
/* 静止参数辨识定位（见 Startup_AlignSlow）。阻尼增益 gGain.alignDampRQ8（虚拟电阻 = Rs）： */
static int32_t s_dampRunQ8;                /* 实际使用的阻尼（检测到离散颤振后自适应减半，仅本次定位有效；0 = 未衰减） */
static int32_t s_dampPrevIq;               /* 上一拍阻尼电流指令（颤振检测用） */
static uint8_t s_dampFlips;                /* 阻尼指令连续大幅换向的计数 */
/* 观测式等停稳：连续"静止"的拍数、已观测到的最长半周期（拍）；本窗口内 e_q 估计的最小 / 最大值（峰峰值判"在动"，
 * 不受恒定偏置影响）；e_q 极值检测（方向、当前极值、上一个确认极值、确认时刻） */
static uint16_t s_quiet, s_halfMax, s_lastRev;
static int32_t s_wMin, s_wMax, s_xExt, s_xPrevExt;
static int8_t s_xDir;
/* 0 = 预定位轴（目标轴 + 60°）等停稳，1 = 按扫掠角速度转到目标轴，2 = 目标轴等停稳，
 * 3 = 摆动测量：阶跃后自由摆动（无阻尼），4 = 摆动测量：加阻尼等停稳 */
static uint8_t s_identStage;
static uint8_t s_identHold;                /* 1 = 收尾定位（Startup_IdentHold）：停稳后做摆动测量 */
/* 摆动测量：步号、本步确认的极值个数、极值检测方向、确认门限（静止时噪声峰峰值 + 1）、电流环建立后的反电势、当前候选极值及其时刻（Q8 拍）、
 * 第一 / 最后一个极值的时刻（Q8 拍，从阶跃起算）、前两个极值的值、最近三拍反电势（抛物线插值极值时刻） */
static uint8_t s_pendStep, s_pendN;
static int8_t s_pendDir;
static int32_t s_pendTh, s_pendE0, s_pendExtV, s_pendV1, s_pendV2, s_pendY1, s_pendY2;
static uint32_t s_pendExtQ8, s_pendFirstQ8, s_pendLastQ8;
static void identWaitReset(void);
#endif
static int32_t s_emfFilt;                  /* 定位时 q 轴反电势（滤波），Q15 电压 << ALIGN_EMF_FILT_SH */
static int32_t s_dtFilt;                   /* 同一滤波器滤波的逆变器电压误差分量（摆动伪峰值参考） */
static volatile uint8_t s_ipd;             /* 1 = Align 子状态正在执行 IPD（慢环写，快环读） */
/* 摆动中起动：1 = 本次起动采用；转向（+1 / -1）；判定"无摆动"的拍数、最长等待拍数；可接入的最小峰值转速（Q15）；
 * e_q 上升途中记录的最近两个电平（L2 较新）及当时的 d 轴积分项；是否出现过超过门限的反电势 */
static uint8_t s_swing;
static int8_t s_swingDir;
static uint16_t s_swingQuietTicks, s_swingMaxTicks;
static int16_t s_swingMinSpd;
static int32_t s_swingL1, s_swingL2, s_swingVd1, s_swingVd2;
static int32_t s_swingNegL1, s_swingNegL2, s_swingNegVd1, s_swingNegVd2;   /* 逆向峰值用同一套电平记录 */
static uint8_t s_swingMoved;
/* 换轴定位：定位轴（Q15，相对开环角 0）与是否已换轴。
 * 首轮未能按过轴峰值接入时，转子可能已在定位轴上，也可能停在定位轴背面的死点（d 轴电流与 N 极反向，转矩 ≈ 0，
 * 静摩擦即可卡住）；若直接按"在定位轴上"接入，死点情形的开环角会错约 180° 而堵转。
 * 故先把定位轴按扫掠角速度沿转向转 90°：在轴上的转子被拖到新轴，死点上的转子受最大转矩被拉到新轴，再按新轴接入 */
static int16_t s_swingAxis;
static uint8_t s_swingTry;

/* 启动摆动中起动（SWING 主流程或 IPD 失败退回时调用） */
static void beginSwing(void)
{
    s_swing = 1;
    s_alignTicks = 0;
    s_emfFilt = 0;
    s_dtFilt = 0;
    s_swingDir = OpenLoop_Reversal(1) ? -1 : 1;
    s_swingL1 = 0;
    s_swingL2 = 0;
    s_swingNegL1 = 0;
    s_swingNegL2 = 0;
    s_swingMoved = 0;
    s_swingAxis = 0;
    s_swingTry = 0;
    s_alignTheta = (int16_t)(-s_swingDir * ALIGN_SWING_SWEEP_Q15);
}

/* 进入 Align（飞车检测判为静止）：起动方式（gStartup.mode）为 IPD 时先执行 IPD，结果不可信时退回摆动起动；否则直接摆动起动。
 * 参数辨识的静止测量：定位到指定角并等待停稳 */
void Startup_OnEnterAlign(void)
{
    s_alignTicks = 0;
    s_emfFilt = 0;
    s_dtFilt = 0;
    s_alignTheta = 0;
    FocCore_SetIdqRef(0, 0);
    s_ipd = 0;
    s_swing = 0;
#if IDENT_TEST
    s_dampRunQ8 = 0;
    s_dampPrevIq = 0;
    s_dampFlips = 0;
    if (Ident_StaticThisStart()) {
        s_alignTheta = (int16_t)(Ident_AlignTheta() + ANGLE_60_Q15);
        s_identStage = 0;
        s_identHold = 0;
        identWaitReset();
        Ident_Reset();
        return;
    }
    /* 不执行静止测量（运行测量 / 已完成）时正常起动：上一次静止测量停在 phase 4，需清除，否则 Align 会一直交由测量程序处理 */
    Ident_Idle();
#endif
    /* 起动方式仅在此处读取一次：本轮中途修改 gStartup.mode 不影响正在执行的 IPD / 摆动起动（其他值按摆动起动） */
    if (gStartup.mode == STARTUP_MODE_IPD) { Ipd_Start(); s_ipd = 1; }
    else beginSwing();
}

/* 从 gGain 重新装载定位摆周期各段拍数（上电及 param.c 更新参数时） */
void Startup_LoadGains(void)
{
    uint32_t t = gGain.alignPeriodTicks;
    s_swingQuietTicks = (uint16_t)(t * (uint32_t)(ALIGN_SWING_QUIET_T * 256.0 + 0.5) >> 8);
    s_swingMaxTicks = (uint16_t)(t * (uint32_t)(ALIGN_SWING_MAX_T * 256.0 + 0.5) >> 8);
    s_swingMinSpd = gGain.alignSwingMinSpd;
}

/* 上电：预充时间、起动方式默认值、装载定位与摆动参数 */
void Startup_Init(void)
{
    gStartup.preChargeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;
    gStartup.mode = STARTUP_MODE;
    Startup_LoadGains();
}

/* 停机复位：预充计数清零 */
void Startup_ResetRun(void)
{
    gStartup.preChargeCnt = 0;
    gStartup.preChargePhase = 0;
}

/* ---------------------------------------------------------------------------
 * Charge：三相下桥轮流导通为自举电容充电，PRECHARGE_TIME 后进入 FlyCheck
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
        Sup_SwitchChargeFlyCheck();
    }
}

/* 预充慢环：仅做测量滤波 */
void Startup_ChargeSlow(void)
{
    Meas_SlowFilter();
}

/* ---------------------------------------------------------------------------
 * FlyCheck：飞车检测（flystart.c）。正在转动且转速足够 -> Spin；低速 -> I/F；静止 -> Align；制动不停 -> 故障
 * -------------------------------------------------------------------------*/
/* 飞车检测快环：执行 FlyStart_DetectFast，按结论切换到 Align / Spin / I/F 或报故障 */
void Startup_FlyCheckFast(void)
{
    switch (FlyStart_DetectFast(gSpd.cmd)) {
    case FLY_STILL:
        Sup_SwitchFlyCheckAlign();
        break;
    case FLY_CATCH:
        Sup_SwitchFlyCheckSpin();
        break;
    case FLY_CATCH_OPENLOOP:
        Sup_SwitchFlyCheckOpenLoop();
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

/* 飞车检测慢环：测量滤波 */
void Startup_FlyCheckSlow(void)
{
    Meas_SlowFilter();
}

/* ---------------------------------------------------------------------------
 * Align：IPD 初始位置检测 / 摆动中起动（SWING）/ 参数辨识阻尼定位
 * -------------------------------------------------------------------------*/
/* Align 快环：辨识模式 / IPD 时由其直接输出电压；摆动起动 / 辨识定位时电流环按定位角运行（电流指令由慢环给出） */
void Startup_AlignFast(void)
{
    FocCore_Clarke();
    if (IDENT_TEST && Ident_Fast()) return;     /* 参数辨识的脉冲段：调制由辨识程序输出 */
    if (s_ipd) { Ipd_Fast(); return; }      /* IPD：由检测模块输出电压（注入 / 偏置电流环） */
    gFoc.theta = s_alignTheta;
    gFoc.thetaInv = gFoc.theta;
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

/* 摆动中起动的 Align 慢环（原理见 cfg_internal.h）：
 *   电流上升段：d 轴电流按斜坡升至 ALIGN_A，定位角随电流由 -ALIGN_SWING_SWEEP_DEG 扫到 0（按转向取号）；
 *   检测段：e = 顺转向的 q 轴反电势（q 轴电流 PI 积分项，滤波），vd = d 轴电流 PI 积分项（同一滤波）。
 *     |e| 上升途中每升高一档（门限的 1/4 + 当前电平的 1/16）记录一个电平和当时的 vd；最高电平不低于门限、|e| 回落到上一个电平时：
 *     两次经过该电平的 vd 之差的一半 = 此刻的 d 轴反电势 ed（过轴之后为正）；
 *       ed 明显为负：转子在定位轴背面，是假峰值，本次摆动不再判断；
 *       否则 转子越过定位轴的角度 = atan(ed / |e|)，转速 = |反电势| / 磁链，再按测量延迟内转过的角度修正，由此接入 I/F。
 *     顺向 / 逆向峰值各用一套电平记录，两者对称：逆向过轴时以负的转子角、负的开环转速接入，由 I/F 加速过零，
 *     不再等转子摆回来，省去最多一个摆动周期。
 *   无摆动（s_swingQuietTicks 内 |e| 始终低于门限）或等待超过 s_swingMaxTicks：定位轴按扫掠角速度沿转向转 90°，
 *     到达后再等 s_swingQuietTicks，按新轴静止接入（避免转子停在死点时按原定位轴接入而错约 180°）。 */
/* 摆动起动接入：本次可直接闭环（STARTUP_DIRECT_CL，上次未失败、自动辨识不需在 I/F 中测磁链）-> 直接闭环，否则 -> I/F。
 * 逆向过轴同样直接闭环：磁链观测器是电压模型 psi = ∫(v - Rs*i)，不依赖反电势，转速过零时 psi 停转、角度保持，
 * 由速度环减速过零再加速 */
static void swingHandover(int16_t th, int16_t w)
{
    if (Spin_DirectOk() && !(IDENT_TEST && Ident_OpenLoopFluxPending()))
        Sup_SwitchSwingSpin(th, w, s_alignTheta);
    else
        Sup_SwitchSwingOpenLoop(th, w);
}

/* 峰值确认后的接入计算（慢环，每次起动最多执行一次：三次 32 位除法）。
 * e = 该方向上的 |e_q|，ed = 过轴之后的 d 轴反电势，sign = 该方向的角度 / 转速符号。 */
static void swingCatch(int32_t e, int32_t ed, int8_t sign)
{
    int32_t t, th, w;
    if (ed < 0) ed = 0;
    t = (ed << 12) / e;                         /* tan(偏角)，Q12 */
    if (t > 4096) t = 4096;
    /* atan(t) = t*pi/4 + 0.273*t*(1 - t)（0 <= t <= 1，误差 < 0.3°），换成 Q15 角度 */
    th = (((t * 3217) >> 12) + ((((t * 1118) >> 12) * (4096 - t)) >> 12)) * 10430 >> 12;
    w = ((e + (ed * ed) / (2 * e)) << 15) / gGain.decKEmfQ15;   /* sqrt(e^2 + ed^2) 的一阶近似 / 磁链，Q15 转速 */
    th += (w * gGain.swingLagAngQ12) >> 12;
    swingHandover((int16_t)(sign * th), (int16_t)(sign * w));
}
static void alignSwingSlow(void)
{
    int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15, e, vd, thr;
    if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
    FocCore_SetIdqRef((int16_t)id, 0);
    if (s_swingTry == 0u)
        s_alignTheta = (int16_t)(-s_swingDir * (((ALIGN_ID_Q15 - id) * ALIGN_SWING_SWEEP_K_Q15) >> 15));
    if (id < ALIGN_ID_Q15) return;
    s_emfFilt += (gFoc.piQ.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, gFoc.idq.s16Q) - (s_emfFilt >> ALIGN_EMF_FILT_SH);
    s_dtFilt += (gFoc.piD.qdSum >> 15) - (s_dtFilt >> ALIGN_EMF_FILT_SH);
    if (++s_alignTicks <= ALIGN_SWING_SETTLE_TICKS) return;
    {   /* 转向取当前指令方向；检测途中指令反向时重新开始记录 */
        int8_t dir = OpenLoop_Reversal(1) ? -1 : 1;
        if (dir != s_swingDir) {
            s_swingDir = dir;
            s_swingL1 = 0;
            s_swingL2 = 0x3FFF0000;
            s_swingNegL1 = 0;
            s_swingNegL2 = 0x3FFF0000;
        }
    }
    if (s_swingTry != 0u) {
        /* 换轴：定位轴按首轮扫掠的角速度转到新轴（满电流下突跳会使转子冲过并被齿槽 / 摩擦卡在半途），
         * 到达后再等 s_swingQuietTicks，按新轴接入，不再按峰值接入 */
        int32_t d = (int32_t)(int16_t)(s_swingAxis - s_alignTheta);
        int32_t st = ((int32_t)ALIGN_STEP_Q15 * ALIGN_SWING_SWEEP_K_Q15) >> 15;
        if (st < 1) st = 1;
        if (d > st) d = st;
        if (d < -st) d = -st;
        s_alignTheta = (int16_t)(s_alignTheta + d);
        if (s_alignTheta != s_swingAxis) s_alignTicks = ALIGN_SWING_SETTLE_TICKS;
        else if (s_alignTicks >= ALIGN_SWING_SETTLE_TICKS + s_swingQuietTicks) swingHandover(s_swingAxis, 0);
        return;
    }
    e = s_swingDir * (s_emfFilt >> ALIGN_EMF_FILT_SH);
    vd = s_dtFilt >> ALIGN_EMF_FILT_SH;
    thr = ((int32_t)s_swingMinSpd * gGain.decKEmfQ15) >> 15;
    if (e >= thr || e <= -thr) s_swingMoved = 1;
    if (e == 0) {                                   /* 过零：两套记录都复位 */
        s_swingL1 = 0;
        s_swingL2 = 0;
        s_swingVd2 = vd;
        s_swingNegL1 = 0;
        s_swingNegL2 = 0;
        s_swingNegVd2 = vd;
    } else if (e < 0) {                             /* 逆向峰值：对称测量，接入后 I/F 从负转速加速过零 */
        s_swingL1 = 0;
        s_swingL2 = 0;
        s_swingVd2 = vd;
        e = -e;
        if (e >= s_swingNegL2 + (thr >> 2) + (s_swingNegL2 >> 4)) {
            s_swingNegL1 = s_swingNegL2;
            s_swingNegVd1 = s_swingNegVd2;
            s_swingNegL2 = e;
            s_swingNegVd2 = vd;
        } else if (s_swingNegL1 > 0 && s_swingNegL2 >= thr && e <= s_swingNegL1) {
            int32_t ed = (s_swingNegVd1 - vd) >> 1;
            if (ed < -(thr >> 2)) {
                /* 假峰值 */
                s_swingNegL1 = 0;
                s_swingNegL2 = 0x3FFF0000;
            } else {
                swingCatch(e, ed, (int8_t)(-s_swingDir));
                return;
            }
        }
    } else {
        s_swingNegL1 = 0;
        s_swingNegL2 = 0;
        s_swingNegVd2 = vd;
        if (e >= s_swingL2 + (thr >> 2) + (s_swingL2 >> 4)) {         /* 升高一档 */
            s_swingL1 = s_swingL2;
            s_swingVd1 = s_swingVd2;
            s_swingL2 = e;
            s_swingVd2 = vd;
        } else if (s_swingL1 > 0 && s_swingL2 >= thr && e <= s_swingL1) {  /* 峰值已过，回落到上一个电平 */
            int32_t ed = (s_swingVd1 - vd) >> 1;
            if (ed < -(thr >> 2)) {
                /* 假峰值：e 回到 0 之前不再记录 */
                s_swingL1 = 0;
                s_swingL2 = 0x3FFF0000;
            } else {
                swingCatch(e, ed, s_swingDir);
                return;
            }
        }
    }
    if (s_alignTicks >= (s_swingMoved ? s_swingMaxTicks : s_swingQuietTicks)) {
        if (s_swingTry == 0u) {
            /* 首轮未能按过轴峰值接入：换轴定位（电流保持 ALIGN_A），见 s_swingAxis */
            s_swingTry = 1u;
            s_swingAxis = (int16_t)(s_swingDir * ANGLE_90_Q15);
            return;
        }
        swingHandover(s_swingAxis, 0);
    }
}

#if IDENT_TEST
/* 等停稳计时复位（每段定位开始、定位电流未升满时） */
static void identWaitReset(void)
{
    s_alignTicks = 0;
    s_quiet = 0;
    s_halfMax = 0;
    s_lastRev = 0;
    s_wMin = 0x7FFFFFFF;
    s_wMax = -0x7FFFFFFF;
    s_xDir = 0;
    s_xExt = 0;
    s_xPrevExt = 0;
}

/* 观测式等停稳（每慢环拍一次），x = q 轴反电势估计（vq_ss - Rs*iq_ref）。摆动周期不预测（惯量未知）：
 *   "在动"：本窗口内 x 的峰峰值超过静止门限（恒定偏置——电流采样零点误差 x R、死区残差——不是运动，不计入；
 *           单摆的位置有界，反电势必正负往复）；超过即重开窗口。
 *   半周期：x 相邻两个极值（幅度相差超过门限）的间隔。
 *   静止持续时间超过已观测到的最长半周期的 ALIGN_QUIET_HALF_K 倍（摆幅超过门限的单摆每半个周期必出现一次极值），
 *   且不短于 ALIGN_QUIET_MIN_S，判为停稳；ALIGN_WAIT_MAX_S 超时记 SWING_CLAMP（只记录）。返回 1 = 停稳 */
static uint8_t identStill(int32_t x)
{
    uint32_t need;
    if (s_alignTicks < 0xFFFFu) s_alignTicks++;
    if (x < s_wMin) s_wMin = x;
    if (x > s_wMax) s_wMax = x;
    if (s_wMax - s_wMin >= ALIGN_STILL_EMF_Q15) {
        s_quiet = 0;
        s_wMin = s_wMax = x;
    } else if (s_quiet < 0xFFFFu) {
        s_quiet++;
    }
    /* 极值检测（滞回 = 门限的一半），相邻极值相差超过门限才算一次摆动的半周期 */
    if (s_xDir >= 0 && x > s_xExt) s_xExt = x;
    if (s_xDir <= 0 && x < s_xExt) s_xExt = x;
    if ((s_xDir >= 0 && x < s_xExt - (ALIGN_STILL_EMF_Q15 >> 1)) || (s_xDir <= 0 && x > s_xExt + (ALIGN_STILL_EMF_Q15 >> 1))) {
        int32_t d = s_xExt - s_xPrevExt;
        if (d < 0) d = -d;
        if (d >= ALIGN_STILL_EMF_Q15) {
            if (s_lastRev != 0u && (uint16_t)(s_alignTicks - s_lastRev) > s_halfMax) s_halfMax = (uint16_t)(s_alignTicks - s_lastRev);
            s_lastRev = s_alignTicks;
        }
        s_xPrevExt = s_xExt;
        s_xDir = (x < s_xExt) ? -1 : 1;
        s_xExt = x;
    }
    need = (uint32_t)s_halfMax * ALIGN_QUIET_HALF_K;
    if (need < ALIGN_QUIET_MIN_TICKS) need = ALIGN_QUIET_MIN_TICKS;
    if (s_quiet >= need) return 1;
    if (s_alignTicks >= ALIGN_WAIT_MAX_TICKS) {
        gIdent.warn |= IDENT_WARN_SWING_CLAMP;
        return 1;
    }
    return 0;
}

/* 摆动测量：定位角先 +IDENT_PEND_STEP_DEG 逐步走一圈，再 -IDENT_PEND_STEP_DEG 逐步走回目标轴（PEND_N 步） */
#define PEND_HALF   ((uint8_t)(360 / (int)IDENT_PEND_STEP_DEG))
#define PEND_N      ((uint8_t)(2u * PEND_HALF))

/* 开始一步摆动测量：定位角阶跃，关阻尼（stage 3）。确认门限取刚才静止窗口内的噪声峰峰值 + 1（不高于静止门限）：
 * 低磁链电机摆动的反电势可能低于静止门限，测量门限由信噪比决定 */
static void pendStart(void)
{
    int16_t step = (int16_t)(s_pendStep < PEND_HALF ? IDENT_PEND_STEP_Q15 : -IDENT_PEND_STEP_Q15);
    SinCos_TypeDef r = FocMath_SinCos(step);
    s_pendTh = s_wMax - s_wMin + 1;
    if (s_pendTh < 1 || s_pendTh > ALIGN_STILL_EMF_Q15) s_pendTh = ALIGN_STILL_EMF_Q15;
    /* 定位坐标旋转 step 后，电流在新坐标中为 i'(0) = R(-step) i，电流环在其带宽内把它拉到指令。零极点对消的 PI 中，
     * 电流从 i'(0) 变到指令 i_ref 的过程里积分项变化 Rs*(i_ref - i'(0))；积分项若不预置，这部分在电流到位后作为
     * 被对消的电机极点（L/R）的模态慢慢恢复，混在 q 轴电压里，与低磁链电机的摆动反电势同量级。
     * 预置积分项 = 原积分项 - Rs*(i_ref - i'(0))，电流到位时积分项正好是新轴上的稳态电压（死区电压沿电流方向，留在 d 轴） */
    {
        int32_t id = gFoc.idqRef.s16D, iq = gFoc.idqRef.s16Q;
        int32_t nid = (id * r.s16Cos + iq * r.s16Sin) >> 15, niq = (iq * r.s16Cos - id * r.s16Sin) >> 15;
        int32_t vd = (gFoc.piD.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, (int16_t)(id - nid));
        int32_t vq = (gFoc.piQ.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, (int16_t)(iq - niq));
        IRQ_LOCK();
        s_alignTheta = (int16_t)(s_alignTheta + step);
        FocCore_PresetPI(vd << 15, vq << 15);
        IRQ_UNLOCK();
    }
    s_identStage = 3;
    s_alignTicks = 0;
    s_pendN = 0;
    s_pendDir = 0;
    s_pendFirstQ8 = s_pendLastQ8 = 0;
    s_pendV1 = s_pendV2 = 0;
    s_dampPrevIq = 0;
    s_dampFlips = 0;
}

/* 自由摆动（每慢环拍一次）：阶跃时转子静止在摆幅端点（-A），此后转子过轴（δ = 0，转速最大）时 q 轴反电势取极值：
 * 第 k 个极值在 T/4 + (k-1) T/2（加一阶滤波延迟，identify.c 扣除）。用极值而不用过零：极值时刻与幅度差（峰峰值）都不受
 * 恒定偏置（电流采样零点误差 x R、死区残差）影响。极值时刻按相邻三拍抛物线插值；与上一个极值相差超过确认门限才确认。
 * 阶跃后 gGain.alignStepBlank 拍内电流环在建立（反电势按 0 计），不检测。
 * 测到 3 个极值、或阶跃 / 上一个极值后 ALIGN_QUIET_MIN_S 内没有新的极值（没有摆动 / 摩擦使摆动停止）时本步结束，返回 1 */
static uint8_t pendSwing(int32_t e)
{
    uint32_t t;
    if (s_alignTicks < 0xFFFFu) s_alignTicks++;
    t = s_alignTicks;
    if (t <= gGain.alignStepBlank) {                                /* 电流环建立期间（反电势按 0 计）：只记录起点 */
        s_pendE0 = s_pendY1 = s_pendY2 = e;
        return 0;
    }
    if (s_pendDir == 0) {                                           /* 等第一段摆动的方向 */
        if (e >= s_pendE0 + s_pendTh) { s_pendDir = 1; s_pendExtV = e; s_pendExtQ8 = t << 8; }
        else if (e <= s_pendE0 - s_pendTh) { s_pendDir = -1; s_pendExtV = e; s_pendExtQ8 = t << 8; }
    } else if ((s_pendDir > 0 && s_pendY2 >= s_pendExtV) || (s_pendDir < 0 && s_pendY2 <= s_pendExtV)) {
        /* 上一拍是新的候选极值：y1, y2, e 三点抛物线顶点 */
        int32_t den = s_pendY1 - 2 * s_pendY2 + e;
        int32_t off = den ? (int32_t)(((int64_t)(s_pendY1 - e) << 7) / den) : 0;    /* Q8，(y1 - y3) / (2 * den) */
        if (off > 128) off = 128;
        if (off < -128) off = -128;
        s_pendExtV = s_pendY2;
        s_pendExtQ8 = (uint32_t)((int32_t)((t - 1u) << 8) + off);
    }
    if (s_pendDir != 0 && ((s_pendDir > 0 && e <= s_pendExtV - s_pendTh) || (s_pendDir < 0 && e >= s_pendExtV + s_pendTh))) {
        if (s_pendN == 0u) { s_pendFirstQ8 = s_pendExtQ8; s_pendV1 = s_pendExtV; }
        if (s_pendN == 1u) s_pendV2 = s_pendExtV;
        s_pendLastQ8 = s_pendExtQ8;
        s_pendN++;
        s_pendDir = (int8_t)-s_pendDir;
        s_pendExtV = e;
        s_pendExtQ8 = t << 8;
    }
    s_pendY1 = s_pendY2;
    s_pendY2 = e;
    return (uint8_t)(s_pendN >= 3u || t >= ALIGN_WAIT_MAX_TICKS || t >= (s_pendLastQ8 >> 8) + ALIGN_QUIET_MIN_TICKS);
}

/* 辨识收尾定位（identify.c 测完 Ld / Lq 后调用）：直接在定位轴上执行辨识定位的第二步（同一套阻尼与停稳判据），
 * 停稳后经 Ident_Begin 结束静止测量 */
void Startup_IdentHold(void)
{
    s_alignTheta = Ident_AlignTheta();
    s_identStage = 2;
    s_identHold = 1;
    identWaitReset();
    s_emfFilt = 0;
    s_dampRunQ8 = 0;
    s_dampPrevIq = 0;
    s_dampFlips = 0;
    FocCore_PresetPI(0, 0);
    FocCore_SetIdqRef(0, 0);
}
#else
void Startup_IdentHold(void) {}     /* identify.c 始终参与编译：IDENT_TEST = 0 时不会被调用，留空以便链接 */
#endif

/* Align 慢环：
 *   摆动中起动：见 alignSwingSlow；
 *   IPD 中：推进检测（Ipd_Slow），完成后判定，可信 -> 从 N 极角直接进入 I/F；不可信 -> 退回摆动中起动；
 *   参数辨识（IDENT_TEST = 1）：d 轴电流按斜坡升至 ALIGN_A，q 轴按反电势施加阻尼；两步定位：
 *     先在目标轴 + 60° 上等停稳，再按扫掠角速度转到目标轴等停稳（避免死点）；停稳计时从电流升满后开始。
 *     达到最短时间且反电势持续低于门限（已停稳），或超过最长时间 -> 进入下一步 / 开始静止测量（Ident_Begin） */
void Startup_AlignSlow(void)
{
    if (IDENT_TEST && Ident_Active()) {
        Ident_Slow();
        Meas_SlowFilter();
        Meas_SpeedHoldZero();
        return;
    }
    if (s_swing) {
        alignSwingSlow();
        Meas_SlowFilter();
        Meas_SpeedHoldZero();
        return;
    }
    if (s_ipd) {
        Ipd_Slow();
        if (gIpd.state == IPD_STATE_DONE) {
            if (Ipd_Evaluate()) {
                Sup_SwitchIpdOpenLoop(gIpd.theta);  /* 从测得的 N 极角开始拖动，无需定位 */
            } else {
                /* 凸极或极性对比度不足（无凸极电机、转子在动等）：本次退回摆动中起动。偏置电流已回到 0，PWM 输出零电压。
                 * 先清积分器并初始化摆动状态，再清 s_ipd（单字节写）：快环看到 s_ipd = 0 时定位角与积分器已就绪 */
                FocCore_PresetPI(0, 0);
                beginSwing();
                s_ipd = 0;
            }
        }
        Meas_SlowFilter();
        Meas_SpeedHoldZero();
        return;
    }
#if IDENT_TEST
    do {   /* 辨识定位（break 跳出后仍执行下方慢环滤波） */
        int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15;
        int32_t emf, iq = 0, e;
        if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
        /* q 轴电压：取电流环积分项（稳态电压，不含比例项放大的电流采样噪声）减 Rs*iq_ref（阻尼电流指令，无采样噪声）。
         * 摆动测量阶跃后电流在电流环带宽内转到新轴（积分项已预置，见 pendStart），期间积分项随电流误差变化、不是反电势：
         * 这几拍（gGain.alignStepBlank）按 0 计（转子刚从静止释放，反电势还小） */
        emf = (gFoc.piQ.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, gFoc.idqRef.s16Q);
        if (s_identStage == 3u && s_alignTicks < gGain.alignStepBlank) emf = 0;
        /* 阻尼：辨识定位阶段尚未测得真实 Rs 与死区（gGain.focKrsQ15 此时为含死区的 R/L 粗测值，往往显著大于真实 Rs）：
         * 扣除未标定的 Rs*iq 会形成负虚拟电阻而正反馈锁死。改用无源虚拟电阻阻尼 iq = -Kd * vq_ss，Kd = 1 / Rs（当前值）：
         * 耗散功率 -eq*iq = (Rs + 1/Kd)*iq^2 + Vdt*iq >= 0，对任意未知电机无条件稳定且唯一平衡点在 iq = 0，
         * 只用已测得的 R，不需要磁链与惯量。连续时间无源不等于离散时间稳定：极低磁链电机 Kd 相对很大时，一阶滤波 + 采样延迟下
         * 特征根越过 -1，表现为 ~500 Hz 的阻尼指令大幅交替（啸叫）。两级防护：
         *   1) 阻尼分量幅值钳位 ALIGN_A/8（约 0.1 倍额定电流，不影响 d 轴定位电流）；
         *   2) 颤振检测：相邻拍 |Δiq| >= iqMax/2 就 +2、否则 −1（不依赖反号奇偶，可抓周期 4 极限环），
         *      累积 >= 8 => Kd 减半并清滤波，可多次衰减。减半下限取 min(damp, max(damp/2, 256))。
         * 摆动测量的自由摆动段（stage 3）不加阻尼 */
        s_emfFilt += emf - (s_emfFilt >> ALIGN_EMF_FILT_SH);
        if (s_identStage != 3u) {
            int32_t damp = (s_dampRunQ8 > 0) ? s_dampRunQ8 : gGain.alignDampRQ8;
            int32_t iqMax = ALIGN_ID_Q15 / 8;
            int32_t iqCmd = (int32_t)(-(((int64_t)(s_emfFilt >> ALIGN_EMF_FILT_SH) * damp) >> 8));
            int32_t delta;
            if (iqCmd > iqMax) iqCmd = iqMax;
            if (iqCmd < -iqMax) iqCmd = -iqMax;
            delta = iqCmd - s_dampPrevIq;
            if (delta < 0) delta = -delta;
            if (delta >= iqMax / 2) {
                if (s_dampFlips < 0xFEu) s_dampFlips += 2;
            } else {
                if (s_dampFlips > 0u) s_dampFlips--;
            }
            if (s_dampFlips >= 8u) {
                int32_t half = damp >> 1;
                s_dampRunQ8 = (half >= 256) ? (half < damp ? half : damp) : (damp < 256 ? damp : 256);
                s_dampFlips = 0;
                s_emfFilt = 0;
            }
            s_dampPrevIq = iqCmd;
            iq = iqCmd;
        }
        FocCore_SetIdqRef((int16_t)id, (int16_t)iq);
        /* 两步定位：固定一个轴定位时，转子停在该轴背面（死点，转矩 ≈ 0）会被静摩擦卡住、照样判为停稳；
         * 先定位到目标轴 + 60°，再按摆动起动的扫掠角速度转到目标轴：无论在哪一步遇到死点，下一步都偏 60°（转矩 0.87 倍最大值）。
         * 取 60° 而不是 90°：目标轴在相轴上时预定位轴也在相轴上（三相电流都不为零），逆变器死区电压沿电流方向，
         * 不在 q 轴上产生偏置；90° 时有一相电流为零，死区电压的方向不定，q 轴偏置会被当成反电势而一直判为"在动"。
         * 停稳计时从定位电流升满后开始 */
        if (id < ALIGN_ID_Q15) {
            identWaitReset();
            break;
        }
        if (s_identStage == 1u) {
            int32_t d = (int32_t)(int16_t)(Ident_AlignTheta() - s_alignTheta);
            int32_t st = ((int32_t)ALIGN_STEP_Q15 * ALIGN_SWING_SWEEP_K_Q15) >> 15;
            if (st < 1) st = 1;
            if (d > st) d = st;
            if (d < -st) d = -st;
            s_alignTheta = (int16_t)(s_alignTheta + d);
            if (s_alignTheta == Ident_AlignTheta()) s_identStage = 2;
            identWaitReset();
            break;
        }
        e = s_emfFilt >> ALIGN_EMF_FILT_SH;
        if (s_identStage == 3u) {
            if (pendSwing(e)) {
                /* 本步结束：记录，加阻尼等停稳；判停稳的半周期取本步测得值 */
                uint16_t h = (uint16_t)(s_pendN > 1u ? ((s_pendLastQ8 - s_pendFirstQ8) / (s_pendN - 1u)) >> 8 : 0u);
                Ident_PendNote(s_pendFirstQ8, s_pendLastQ8, s_pendN, s_pendV2 - s_pendV1);
                s_identStage = 4;
                identWaitReset();
                s_halfMax = h;
            }
            break;
        }
        if (!identStill(e)) break;
        if (s_identStage == 0u) {               /* 预定位完成：转向目标轴 */
            s_identStage = 1;
            identWaitReset();
            break;
        }
        if (s_identStage == 2u && s_identHold) {    /* 收尾定位停稳：开始摆动测量 */
            s_pendStep = 0;
            pendStart();
            break;
        }
        if (s_identStage == 4u && ++s_pendStep < PEND_N) {
            pendStart();
            break;
        }
        if (s_identStage == 4u) Ident_PendFinish();     /* 已回到目标轴并停稳：计算惯量与磁链 */
        Ident_Begin();
    } while (0);
#endif
    Meas_SlowFilter();
    Meas_SpeedHoldZero();
}
