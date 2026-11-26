/* ============================================================================
 * @file    openloop.c
 * @brief   I/F 开环（Run 子状态 OpenLoop）：开环拖动，以及开环与闭环之间的切换
 *          （openLoopHandover：开环 -> 闭环；OpenLoop_Prepare* / Commit：IPD、摆动起动、闭环回退、飞车接管 -> 开环）。接口见 openloop.h
 *
 * I/F 开环原理：电流矢量幅值恒定（拖动电流 gGain.olIqQ15），角度按设定加速度积分旋转（不依赖转子位置）。
 *   转子自动跟随：电流矢量与转子 d 轴的夹角（负载角 δ）自动调整到 I*sin(δ)*Kt = 负载转矩 + 加速转矩。
 *   只要加速所需转矩小于 I*Kt 就不会失步。观测器与 PLL 在此期间只跟踪不控制，转速一致后切换闭环。
 * ==========================================================================*/
#include "openloop.h"
#include "startup.h"
#include "supervisor.h"
#include "foc_core.h"
#include "hal.h"
#include "speed_ctrl.h"
#include "protect.h"
#include "qmath.h"
#include "flux_obs.h"
#include "meas.h"
#include "identify.h"
#include "param.h"

static volatile int32_t s_olTheta;         /* 开环角 Q16.16（快环推进） */
static volatile int32_t s_olSpeed;         /* 开环转速 Q31（快环斜坡） */
static volatile int32_t s_olTarget;        /* 目标转速 Q31，带方向（慢环写） */
static int32_t s_pllFilt;                  /* PLL 转速一阶滤波 Q31（慢环，OPENLOOP_PLL_FILT_TAU_S） */
static uint16_t s_agree, s_hold;
/* 开环加速度按实际开环电流缩放：gGain.openLoopAccStepQ31 为拖动电流 gGain.olIqQ15 下最大能力的 1/3，回退 / 反转时电流增大，
 * 保持相同余量时加速度也按电流比例增大，过零更快。s_accScaleQ12 = I / 拖动电流（Q12，冷起动 = 4096）。
 * I/F 各电流（拖动 / 反转 / 上限）为配置值按驱动板限幅后的运行时值（param.c openLoopCurrents） */
static int32_t s_accScaleQ12 = 4096;
static int32_t s_olCur, s_olCurTarget;      /* 开环电流当前值 / 目标值（Q15）；回退时由小升至目标（CURRENT_SLEW_STEP_Q15_OF） */
static volatile int32_t s_accStep = OPENLOOP_ACC_STEP_Q31;   /* 快环读 */
static int32_t s_accStepFull;                         /* 开环电流达到目标后的加速度步长 */
/* 冷起动失速自适应：I/F 能否把转子拖进同步取决于接入角误差、电流（转矩能力）与加速度之比，事先无法可靠判断。
 * 冷起动（摆动 / IPD 之后）在 I/F 中堵转超时一次，加强一档：电流按 OPENLOOP_BOOST_MAX 档从拖动电流 gGain.olIqQ15 均匀加到
 * 上限 gGain.olWarmImaxQ15（闭环电流上限 SPD_PI_LIMIT，按驱动板限幅），加速度每档减半；堵转故障释放后由现有的自动重启（SUP_RESTART_LIMIT 次内）以新档位重试。
 * 学到的档位保留到下次上电（之后的起动直接使用），不影响闭环回退 / 飞车接管进入 I/F。正常电机首次即可起动，行为不变 */
#define OPENLOOP_BOOST_MAX  3u
#define OPENLOOP_IDENT_BOOST 2u   /* 自动辨识运行段测得磁链前，冷起动加速度的最低档 */
static uint8_t s_ifBoost;

uint8_t OpenLoop_BoostLevel(void) { return s_ifBoost; }

/* 当前开环转速（Q15），供观测器相位补偿等使用 */
int16_t OpenLoop_SpeedQ15(void) { return (int16_t)(s_olSpeed >> 16); }

/* 开环目标转速。|指令| >= 闭环最低转速（gGain.spdSpinMin）：朝指令方向拖动，下限 切换转速（gGain.spdHandover）（指令更低时先拖到切换转速、切闭环后再降速），
 * 上限 回退转速（gGain.spdFallback）（观测器在此以上可靠）；超过 切换转速（gGain.spdHandover） 后边加速边检查一致性，一致即切换，无需停顿等待。
 * 否则以指令转速持续开环运行。 */
static int32_t openLoopTarget(void)
{
    int32_t c = gSpd.cmd, a = (c < 0 ? -c : c) << 16;
    if (a < ((int32_t)gGain.spdSpinMinQ15 << 16)) return c << 16;
    if (a < gGain.spdHandoverQ31) a = gGain.spdHandoverQ31;
    if (a > gGain.spdFallbackQ31) a = gGain.spdFallbackQ31;
    return c > 0 ? a : -a;
}

/* asin(x)，x 为 Q15（限 ±0.95），返回 Q15 角度（65536 = 360°）。基于正弦表二分查找，慢环使用 */
static int16_t asinQ15(int32_t x)
{
    int32_t lo = -ANGLE_90_Q15, hi = ANGLE_90_Q15, mid;
    int i;
    if (x > 31130) x = 31130;
    if (x < -31130) x = -31130;
    for (i = 0; i < 15; i++) {
        mid = (lo + hi) / 2;
        if (FocMath_SinCos((int16_t)mid).s16Sin < x) lo = mid; else hi = mid;
    }
    return (int16_t)lo;
}

/* 由 Spin 慢环查询是否需要回退到 I/F：转速已低于 回退转速（gGain.spdFallback）（观测器可靠下限附近），且指令反向（需过零）
 * 或指令本身低于闭环最低转速时返回 1 */
uint8_t OpenLoop_WantsFallback(void)
{
    int32_t filt = gMeas.speed, cmd = gSpd.cmd;
    int32_t af = filt < 0 ? -filt : filt;
    if (af >= gGain.spdFallbackQ15) return 0;
    /* 指令反向，或指令低于闭环最低转速：交给 I/F */
    return (uint8_t)((cmd > 0) != (filt > 0) || (cmd < 0 ? -cmd : cmd) < gGain.spdSpinMinQ15);
}

/* 从闭环回退到 I/F 时的开环电流：至少为拖动电流；需能提供此刻的 iq 且负载角不超过 δmax（|iq| / sin(δmax)）；
 * 需过零（反转）时至少为反转电流；不超过上限 gGain.olWarmImaxQ15 */
static int16_t OpenLoop_WarmCurrent(int16_t iq, uint8_t reversal)
{
    int32_t cur = gGain.olIqQ15, aiq = iq < 0 ? -(int32_t)iq : iq;
    int32_t need = (int32_t)(((int64_t)aiq * OPENLOOP_WARM_KI_Q15) >> 15);      /* |iq| / sin(δmax) */
    if (need > cur) cur = need;
    if (reversal && cur < gGain.olRevIqQ15) cur = gGain.olRevIqQ15;
    if (cur > gGain.olWarmImaxQ15) cur = gGain.olWarmImaxQ15;
    return (int16_t)cur;
}

/* 进入 I/F 分为"准备 + 提交"两步：
 *   OpenLoop_Prepare：只计算、不修改任何状态，输入全部作为参数（调用者在临界区外调用，可被快环打断）。
 *     计算开环电流起始值 / 目标值、加速度步长，以及开环角相对估计角的偏移 asin(iq / I)。
 *   OpenLoop_Commit：只赋值（几十个时钟，调用者在临界区内调用）。开环角 = 提交时刻的 PLL 角 + 偏移，
 *     准备期间即使被快环打断、PLL 角已前进也不受影响。
 * 拆分的目的是将耗时的开方查找与除法移出临界区 / 快环，缩短关中断时间。
 *   warm = 1：从运行状态接管。iq = 闭环此刻的 q 轴电流指令，speedQ15 = 开环起始转速；
 *     revCur / revAcc = 开环需过零（目标方向与 PLL 转速 / 起始转速方向相反），分别决定开环电流和加速度缩放。
 *   warm = 0：冷起动（IPD / 摆动起动后），开环角 = 转子角 + 平衡负载角。 */
static void prepareOpenLoop(uint8_t warm, int16_t iqNow, uint8_t revCur, uint8_t revAcc, int16_t speedQ15, OpenLoopEntry_TypeDef *e)
{
    int32_t cur = gGain.olIqQ15;
    uint8_t accBoost = s_ifBoost;
    /* 自动辨识的运行段、磁链尚未测得：加速度系数（∝ psi / J）仍为配置值，可能偏大数倍，冷起动加速度至少按
     * OPENLOOP_IDENT_BOOST 档（1/4），所需转矩随之减为 1/4，留出参数未知时的负载角余量。只降加速度、不加电流：
     * 内嵌式电机 Ld < Lq 时，磁阻转矩使 d 轴附近的刚度按 psi - (Lq - Ld) * i 下降，电流越大越容易失步。
     * 辨识允许耗时，测得 psi 后恢复正常档 */
    if (IDENT_TEST && !warm && accBoost < OPENLOOP_IDENT_BOOST && Ident_OpenLoopFluxPending()) accBoost = OPENLOOP_IDENT_BOOST;
    if (!warm && s_ifBoost && gGain.olWarmImaxQ15 > gGain.olIqQ15)
        cur += ((int32_t)s_ifBoost * (gGain.olWarmImaxQ15 - gGain.olIqQ15)) / (int32_t)OPENLOOP_BOOST_MAX;
    e->warm = warm;
    e->thetaOffset = 0;
    if (warm) {
        /* 转矩连续：开环电流 I 沿 theta_ol，其在转子 q 轴的分量为 I*sin(theta_ol - theta_est)。
         * 令其等于闭环此刻的 iq：theta_ol = theta_est + asin(iq/I)。
         * 开环转矩 = I·Kt·sin(δ)，δ 超过 90° 即失步。I 的取值使 δ 不超过 OPENLOOP_WARM_DELTA_DEG（45°）：
         * 为磁链 / 转矩系数偏小 25% 以上留出余量（sin 45° / 0.75 = 0.94 < 1）。
         * 反转（开环需过零）时 I 至少为 OPENLOOP_REV_CURRENT_A——过零段需先将转子制动再反向拖动，转矩需求最大。 */
        int32_t iq = iqNow, aiq = iq < 0 ? -iq : iq, i0, sp;
        cur = OpenLoop_WarmCurrent((int16_t)iq, revCur);
        /* 起始电流：不直接给到 cur，而是从 max(|iq| / sin45°, cur / 4) 起、按 CURRENT_SLEW_STEP_Q15_OF 升至 cur。
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
        if (sp > gGain.spdWarmMaxQ31) sp = gGain.spdWarmMaxQ31;      /* 仅防止异常值，正常情况 <= 回退转速 */
        if (sp < -gGain.spdWarmMaxQ31) sp = -gGain.spdWarmMaxQ31;
        e->olSpeed = sp;
    } else {
        e->olSpeed = 0;
        e->olCurTarget = cur;
    }
    e->olCur = cur;
    /* 电流沿开环角方向（开环坐标系 d 轴）：转子滞后 delta，转矩 ∝ I*sin(delta)，
     * 加速时 delta 自然增大。冷起动时与定位电流方向连续，避免像施加在 q 轴那样
     * 一开始就输出满转矩、激起新的摆动。 */
    /* 反转过零时开环电流至少为反转电流：加速度按反转电流 / 拖动电流增大，余量与冷起动相同
     * （按"所需 iq"增加的那部分电流用于已有转矩需求，不用于加速）。冷起动 / 同向 = 1 */
    e->accScaleQ12 = (warm && revAcc) ? (int32_t)(((int64_t)gGain.olRevIqQ15 << 12) / gGain.olIqQ15) : 4096;
    e->accStepFull = (int32_t)(((int64_t)gGain.openLoopAccStepQ31 * e->accScaleQ12) >> 12);
    if (!warm) e->accStepFull >>= accBoost;      /* 失速自适应：加速度每档减半 */
    /* 开环加速度与开环电流成正比：电流仍按 CURRENT_SLEW_STEP_Q15_OF 向目标上升时，可产生的转矩相应受限，
     * 加速度按同一比例给出，负载角余量（OPENLOOP_ACC_RPM_S_OF 取最大加速度的 1/3）全程保持不变。
     * accStepFull * 电流用 64 位乘除（大加速度系数、大电流的电机超出 32 位；不在快环中） */
    e->accStep = (int32_t)(((int64_t)e->accStepFull * cur) / e->olCurTarget);
}

/* 冷起动（IPD 之后）：开环角 = 转子角 + 平衡负载角、零速、开环电流从 1/4 起上升（加速度同比例），d 轴电流阶跃较小 */
void OpenLoop_PrepareCold(OpenLoopEntry_TypeDef *e)
{
    prepareOpenLoop(0, 0, 0, 0, 0, e);
    e->olCur = e->olCurTarget >> 2;
    e->accStep = (int32_t)(((int64_t)e->accStepFull * e->olCur) / e->olCurTarget);
}

/* 摆动中起动：转子正以 speedQ15（与目标同向）经过当前控制角。同冷起动（开环角 = 转子角 + 平衡负载角），但开环转速从转子转速开始，
 * 电流从定位电流按 CURRENT_SLEW_STEP_Q15_OF 升至目标（加速度同比例） */
void OpenLoop_PrepareSwing(int16_t speedQ15, OpenLoopEntry_TypeDef *e)
{
    prepareOpenLoop(0, 0, 0, 0, 0, e);
    e->olCur = ALIGN_ID_Q15;
    e->olSpeed = (int32_t)speedQ15 << 16;
    if (e->olSpeed > gGain.spdWarmMaxQ31) e->olSpeed = gGain.spdWarmMaxQ31;      /* 仅防止异常值 */
    if (e->olSpeed < -gGain.spdWarmMaxQ31) e->olSpeed = -gGain.spdWarmMaxQ31;
    e->accStep = (int32_t)(((int64_t)e->accStepFull * e->olCur) / e->olCurTarget);
}

/* 从运行状态接管（回退 / 飞车）：iqNow = 此刻的 q 轴电流指令；revCur / revAcc = 开环需过零（分别决定开环电流、加速度缩放）；
 * speedQ15 = 开环起始转速 */
void OpenLoop_PrepareWarm(int16_t iqNow, uint8_t revCur, uint8_t revAcc, int16_t speedQ15, OpenLoopEntry_TypeDef *e)
{
    prepareOpenLoop(1, iqNow, revCur, revAcc, speedQ15, e);
}

/* 进入 I/F 的"提交"（在临界区内 / 快环中调用，只赋值）：开环目标、开环角（冷起动 = 转子角 + 平衡负载角；接管 = PLL 角 + 预先计算的偏移）、
 * 开环转速、电流、加速度，电流指令沿开环 d 轴 */
void OpenLoop_Commit(const OpenLoopEntry_TypeDef *e)
{
    s_olTarget = openLoopTarget();
    Protect_PhaseReset();           /* 缺相窗口从进入 I/F 时重新计 */
    /* 冷起动：转子角（摆动过轴角 / IPD N 极角）再超前平衡负载角，起动即处于平衡点、负载角不摆动（cfg_internal.h OPENLOOP_ACC_FRAC） */
    s_olTheta = e->warm ? (int32_t)((uint32_t)gEst.pll.s32Theta + e->thetaOffset)
                        : (int32_t)(((uint32_t)(int32_t)gFoc.theta << 16) + (uint32_t)(s_olTarget < 0 ? -OPENLOOP_LEAD_Q31 : OPENLOOP_LEAD_Q31));
    s_olSpeed = e->olSpeed;
    s_olCur = e->olCur;
    s_olCurTarget = e->olCurTarget;
    s_accScaleQ12 = e->accScaleQ12;
    s_accStepFull = e->accStepFull;
    s_accStep = e->accStep;
    FocCore_SetIdqRef((int16_t)e->olCur, 0);
    s_pllFilt = gEst.pll.s32Speed;
    s_agree = 0;
    s_hold = 0;
}

/* 飞车接管进入 I/F（快环中执行）：接管时闭环电流 iq = 0（Sup_SwitchFlyCheckOpenLoop 先清零），入口参数除起始转速外
 * 均为常数，仅分"开环需过零 / 不过零"两种。上电及参数更新时（OpenLoop_Init / OpenLoop_LoadGains）预先计算，快环中按方向选取一份、填入转速并提交：常数时间 */
static OpenLoopEntry_TypeDef s_catchIf[2];

void OpenLoop_CommitCatch(int16_t spd)
{
    OpenLoopEntry_TypeDef e = s_catchIf[(openLoopTarget() > 0) != (spd > 0)];
    int32_t sp = (int32_t)spd << 16;
    if (sp > gGain.spdWarmMaxQ31) sp = gGain.spdWarmMaxQ31;
    if (sp < -gGain.spdWarmMaxQ31) sp = -gGain.spdWarmMaxQ31;
    e.olSpeed = sp;
    OpenLoop_Commit(&e);
}

/* 开环需过零（反转）：开环目标方向与当前转速方向相反 */
uint8_t OpenLoop_Reversal(int32_t speedSign)
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
    SinCos_TypeDef a = FocMath_SinCos(d);
    int16_t i = gFoc.idqRef.s16D;
    int16_t id0 = QMath_MulQ15(i, a.s16Cos), iq0 = QMath_MulQ15(i, a.s16Sin), iqPi, ff0 = 0;
    int32_t ol = s_olSpeed;
    /* 切换时开环仍在加速：该部分转矩已知（模型 OPENLOOP_ACC_RPM_S_OF / MOTOR_ACC_RPM_S_PER_A），作为加速前馈的起点（速度环中
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
        FocCore_SetIdqRef(id0, iq0);        /* 本拍电流矢量保持不变 */
        FocCore_RotatePI(a);
        /* 速度环：PI 积分器预置为 iqPi（前馈给出 ff0），斜坡延续开环转速与加速度；id0 与 iq0 - iqPi - ff0 由融合叠加并淡出 */
        SpeedCtrl_OnEnterSpin(id0, iqPi);
        SpeedCtrl_BlendStart(id0, (int16_t)(iq0 - iqPi - ff0), ol, (int16_t)s_accScaleQ12, ff0);
        Sup_SwitchOpenLoopSpin();
        IRQ_UNLOCK();
    }
}

/* 按 gGain 重新计算飞车接管进入 I/F 的两份入口参数（iq = 0，同向 / 过零；电流与加速度均随参数变化），
 * 上电及 param.c 更新参数时调用。iq = 0 时只有整数运算与一次 asin(0)，可在关中断期间执行 */
void OpenLoop_LoadGains(void)
{
    OpenLoop_PrepareWarm(0, 0, 0, 0, &s_catchIf[0]);
    OpenLoop_PrepareWarm(0, 1, 1, 0, &s_catchIf[1]);
}

/* 上电：飞车接管入口参数快环接管时直接取用（常数时间） */
void OpenLoop_Init(void)
{
    s_ifBoost = 0;
    OpenLoop_LoadGains();
}

/* ---------------------------------------------------------------------------
 * OpenLoop（I/F）：电流矢量开环旋转（角度 = 开环积分），观测器 + PLL 只跟踪；转速一致后切换到 Spin。
 * -------------------------------------------------------------------------*/
/* I/F 快环：观测器 + PLL 只跟踪；开环转速按加速度步长趋近目标，开环角积分；电流环按开环角运行。
 *   [读] gMeas.iabc/vbus/vbusAvg/kvbus, gFoc.idqRef（开环 d 轴电流）, s_olSpeed/s_olTarget/s_olTheta
 *   [写] gMeas.iab/ibusRec, gFluxObs/gEst（仅后台跟踪）, gFoc.theta/thetaInv（开环角）, gSvpwm, EPWM 占空比 */
void OpenLoop_Fast(void)
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
 *   更新目标、PLL 转速滤波；回退进入时开环电流按 CURRENT_SLEW_STEP_Q15_OF 升至目标（加速度同比例）；
 *   缺相检查；运行辨识模式下先测磁链；
 *   开环转速超过切换转速 切换转速（gGain.spdHandover） 后，PLL 转速与开环转速一致（相对误差 < OPENLOOP_AGREE_RATIO）
 *   并持续 OPENLOOP_AGREE_S -> 切闭环；拖动超时仍不一致 -> 堵转故障 */
void OpenLoop_Slow(void)
{
    int32_t ol = s_olSpeed;
    s_olTarget = openLoopTarget();
    s_pllFilt += (gEst.pll.s32Speed >> OPENLOOP_PLL_FILT_SH) - (s_pllFilt >> OPENLOOP_PLL_FILT_SH);
    if (s_olCur < s_olCurTarget) {      /* 回退时开环电流升至目标 */
        s_olCur += gGain.currentSlewStepQ15;
        if (s_olCur > s_olCurTarget) s_olCur = s_olCurTarget;
        s_accStep = (int32_t)(((int64_t)s_accStepFull * s_olCur) / s_olCurTarget);     /* 64 位，见 OpenLoop_Prepare */
        FocCore_SetIdqRef((int16_t)s_olCur, 0);
    }

    /* 缺相：与 Spin 相同的判据（protect.h），电周期按开环转速计 */
    if (Protect_PhaseSlow((int16_t)(ol >> 16))) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        Sup_CountRestart();
    } else if (IDENT_TEST && Ident_OpenLoopFluxSlow(ol, s_olTarget)) {
        /* 运行辨识：在开环中先测磁链（identify.c），测得后生效，再按下述一致性判据切闭环 */
    } else if ((s_olTarget >= gGain.spdHandoverQ31 && ol >= gGain.spdHandoverQ31) ||
               (s_olTarget <= -gGain.spdHandoverQ31 && ol <= -gGain.spdHandoverQ31)) {
        /* 已超过切换转速且与目标同向（反转时开环需先过零）：边拖动边检查。PLL 滤波 16 ms 在 I/F 加速度下
         * 仅滞后约 2 rpm，远小于 OPENLOOP_AGREE_RATIO 的容差，无需补偿 */
        int64_t d = (int64_t)s_pllFilt - ol;
        int64_t tol = ((int64_t)(ol < 0 ? -ol : ol) * OPENLOOP_AGREE_Q15) >> 15;
        if (d < 0) d = -d;
        if (d < tol) s_agree++; else s_agree = 0;
        if (s_agree >= OPENLOOP_AGREE_TICKS) {
            openLoopHandover();
        } else if (++s_hold > OPENLOOP_TIMEOUT_TICKS) {
            if (s_ifBoost < OPENLOOP_BOOST_MAX) s_ifBoost++;    /* 失速自适应：下一次冷起动加强一档 */
            if (IDENT_TEST) Ident_OpenLoopFluxRearm();      /* 运行辨识：本次磁链可能测自未同步的拖动，下次重测 */
            Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
            Sup_CountRestart();
        }
    }
    Meas_SlowFilter();
}
