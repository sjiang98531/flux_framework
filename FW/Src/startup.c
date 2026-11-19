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
#include "param.h"

Startup_TypeDef gStartup;

static volatile int16_t s_alignTheta;      /* 定位角（慢环写，快环读） */
static uint16_t s_alignTicks;
#if IDENT_TEST
/* 静止参数辨识的阻尼定位参数（Startup_LoadGains 按摆动周期 T 计算）：最短 / 判停稳持续 / 最长、阻尼增益 */
static uint16_t s_alignMinTicks, s_alignStillTicks, s_alignMaxTicks;
static int32_t s_dampQ8;                   /* 定位阻尼增益：Q15 电流 / Q15 电压，Q8 */
static int32_t s_dampRunQ8;                /* 实际使用的阻尼（检测到离散颤振后自适应减半，仅本次定位有效；0 = 未衰减） */
static int32_t s_dampPrevIq;               /* 上一拍阻尼电流指令（颤振检测用） */
static uint8_t s_dampFlips;                /* 阻尼指令连续大幅换向的计数 */
static uint16_t s_still;                   /* 定位第二段：连续"静止"的拍数 */
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
    s_still = 0;
    s_dampRunQ8 = 0;
    s_dampPrevIq = 0;
    s_dampFlips = 0;
    if (Ident_StaticThisStart()) {
        s_alignTheta = Ident_AlignTheta();
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
#if IDENT_TEST
    s_alignMinTicks = (uint16_t)(t * (uint32_t)(ALIGN_MIN_T * 256.0 + 0.5) >> 8);
    s_alignStillTicks = (uint16_t)(t * (uint32_t)(ALIGN_STILL_T * 256.0 + 0.5) >> 8);
    s_alignMaxTicks = (uint16_t)(t * (uint32_t)(ALIGN_MAX_T * 256.0 + 0.5) >> 8);
    s_dampQ8 = gGain.alignDampQ8;
#endif
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
 *   无摆动（s_swingQuietTicks 内 |e| 始终低于门限）或等待超过 s_swingMaxTicks：按静止在定位轴上接入。 */
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
    Sup_SwitchSwingOpenLoop((int16_t)(sign * th), (int16_t)(sign * w));
}
static void alignSwingSlow(void)
{
    int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15, e, vd, thr;
    if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
    FocCore_SetIdqRef((int16_t)id, 0);
    s_alignTheta = (int16_t)(-s_swingDir * (((ALIGN_ID_Q15 - id) * ALIGN_SWING_SWEEP_K_Q16) >> 16));
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
    if (s_alignTicks >= (s_swingMoved ? s_swingMaxTicks : s_swingQuietTicks)) Sup_SwitchSwingOpenLoop(0, 0);
}

/* Align 慢环：
 *   摆动中起动：见 alignSwingSlow；
 *   IPD 中：推进检测（Ipd_Slow），完成后判定，可信 -> 从 N 极角直接进入 I/F；不可信 -> 退回摆动中起动；
 *   参数辨识（IDENT_TEST = 1）：d 轴电流按斜坡升至 ALIGN_A，q 轴按反电势施加阻尼，
 *     达到最短时间且反电势持续低于门限（已停稳），或超过最长时间 -> 开始静止测量（Ident_Begin） */
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
    {
        int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15;
        int32_t emf, iq;
        if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
        /* 辨识定位阶段尚未测得真实 Rs 与死区（gGain.focKrsQ15 此时为含死区的 R/L 粗测值，往往显著大于真实 Rs）：
         * 扣除未标定的 Rs*iq 会形成负虚拟电阻而正反馈锁死。改用无源虚拟电阻阻尼 iq = -Kd * vq_ss：
         * 耗散功率 -eq*iq = (Rs + 1/Kd)*iq^2 + Vdt*iq >= 0，对任意未知电机无条件稳定且唯一平衡点在 iq = 0。
         * 连续时间无源不等于离散时间稳定：极低磁链电机（psi ~ 0.4 mWb）理论 Kd 极大，一阶滤波 + 采样延迟下
         * 特征根越过 -1，表现为 ~500 Hz 的阻尼指令大幅交替（啸叫）。两级防护（均不依赖配置 psi/J）：
         *   1) 阻尼分量幅值钳位 ALIGN_A/8（约 0.1 倍额定电流，不影响 d 轴定位电流）；
         *   2) 颤振检测：相邻拍 |Δiq| >= iqMax/2 就 +2、否则 −1（不依赖反号奇偶，可抓周期 4 极限环），
         *      累积 >= 8 => Kd 减半并清滤波，可多次衰减。减半下限取 min(damp, max(damp/2, 256))。 */
        emf = gFoc.piQ.qdSum >> 15;
        s_emfFilt += emf - (s_emfFilt >> ALIGN_EMF_FILT_SH);
        {
            int32_t damp = (s_dampRunQ8 > 0) ? s_dampRunQ8 : s_dampQ8;
            int32_t iqMax = ALIGN_ID_Q15 / 8;
            int32_t iqCmd = -((s_emfFilt >> ALIGN_EMF_FILT_SH) * damp >> 8);
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
        ++s_alignTicks;
        {   /* 参数辨识：停稳判据取 |vq_filt| + |focKrs * iq|（|eq| 的保守上界），静止时两者同时归零 */
            int32_t e = s_emfFilt >> ALIGN_EMF_FILT_SH;
            int32_t ir = QMath_MulQ15(gGain.focKrsQ15, gFoc.idq.s16Q);
            if (e < 0) e = -e;
            if (ir < 0) ir = -ir;
            e += ir;
            if (s_alignTicks > s_alignMinTicks && e < gGain.alignStillEmfQ15) {
                if (s_still < 0xFFFF) s_still++;
            } else {
                s_still = 0;
            }
            if ((s_alignTicks >= s_alignMinTicks && s_still >= s_alignStillTicks) || s_alignTicks >= s_alignMaxTicks) {
                if (s_alignTicks >= s_alignMaxTicks && s_still < s_alignStillTicks)
                    gIdent.warn |= IDENT_WARN_DT_UNSTABLE;   /* 超时未停稳：置告警 */
                Ident_Begin();
            }
        }
    }
#endif
    Meas_SlowFilter();
    Meas_SpeedHoldZero();
}
