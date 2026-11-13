/* ============================================================================
 * @file    startup.c
 * @brief   起动各子状态：Charge（预充）、FlyCheck（飞车检测分发）、Align（IPD / 定位）；
 *          I/F 开环（OpenLoop）在 openloop.c。
 *          接口说明见 startup.h
 * ==========================================================================*/
#include "startup.h"
#include "openloop.h"
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

static volatile int16_t s_alignTheta;      /* 定位角（慢环写，快环读） */
static uint16_t s_alignTicks;
/* 定位各段拍数（Startup_Init 按摆动周期 T 计算一次）：最短 / 判停稳持续 / 最长 */
static uint16_t s_alignMinTicks, s_alignStillTicks, s_alignMaxTicks;
static uint16_t s_alignGoTicks;            /* 定位后立即加速：定位总拍数（电流上升 + ALIGN_GO_T 个 T） */
static uint8_t s_alignGo;                  /* 1 = 本次定位到时间即拖动（起动用）；0 = 等停稳（参数辨识） */
static int32_t s_dampQ8;                   /* 定位阻尼增益：Q15 电流 / Q15 电压，Q8 */
static int32_t s_emfFilt;                  /* 定位时 q 轴反电势（滤波），Q15 电压 << ALIGN_EMF_FILT_SH */
static int32_t s_dtFilt;                   /* 同一滤波器滤波的逆变器电压误差 q 分量（判停稳用） */
static uint16_t s_still;                   /* 定位第二段：连续"静止"的拍数 */
static volatile uint8_t s_ipd;             /* 1 = Align 子状态正在执行 IPD（慢环写，快环读） */
static uint8_t s_drag;                     /* 1 = 起动方式为直接拖动：下一个慢环拍进入 I/F */
/* 摆动中起动：1 = 本次起动采用；转向（+1 / -1）；判定"无摆动"的拍数、最长等待拍数；可接入的最小峰值转速（Q15）；
 * e_q 上升途中记录的最近两个电平（L2 较新）及当时的 d 轴积分项；是否出现过超过门限的反电势 */
static uint8_t s_swing;
static int8_t s_swingDir;
static uint16_t s_swingQuietTicks, s_swingMaxTicks;
static int16_t s_swingMinSpd;
static int32_t s_swingL1, s_swingL2, s_swingVd1, s_swingVd2;
static int32_t s_swingNegL1, s_swingNegL2, s_swingNegVd1, s_swingNegVd2;   /* 逆向峰值用同一套电平记录 */
static uint8_t s_swingMoved;

/* 进入 Align（飞车检测判为静止）：清除定位状态。起动方式（gStartup.mode）为 IPD 时先执行 IPD，结果不可信时再定位；否则直接拖动。
 * 参数辨识的静止测量：定位到指定角 */
void Startup_OnEnterAlign(void)
{
    s_alignTicks = 0;
    s_emfFilt = 0;
    s_dtFilt = 0;
    s_still = 0;
    s_alignTheta = 0;
    FocCore_SetIdqRef(0, 0);
    s_ipd = 0;
    s_drag = 0;
    s_swing = 0;
    s_alignGo = 1;
    if (IDENT_TEST && Ident_StaticThisStart()) {
        s_alignGo = 0;     /* 参数辨识：定位到指定角 */
        s_alignTheta = Ident_AlignTheta();
        Ident_Reset();
    } else {
        /* 不执行静止测量（运行测量 / 已完成）时正常起动：上一次静止测量停在 phase 4，需清除，否则 Align 会一直交由测量程序处理 */
        if (IDENT_TEST) Ident_Idle();
        /* 起动方式仅在此处读取一次：本轮中途修改 gStartup.mode 不影响正在执行的 IPD / 定位 */
        if (gStartup.mode == STARTUP_MODE_IPD) { Ipd_Start(); s_ipd = 1; }    /* 先检测初始位置，检测失败再定位 */
        else if (gStartup.mode == STARTUP_MODE_SWING) {             /* 摆动中起动：电流矢量从定位轴之后扫向定位轴 */
            s_swing = 1;
            s_swingDir = OpenLoop_Reversal(1) ? -1 : 1;
            s_swingL1 = 0;
            s_swingL2 = 0;
            s_swingNegL1 = 0;
            s_swingNegL2 = 0;
            s_swingMoved = 0;
            s_alignTheta = (int16_t)(-s_swingDir * ALIGN_SWING_SWEEP_Q15);
        }
        else if (gStartup.mode != STARTUP_MODE_ALIGN) s_drag = 1;    /* 直接拖动；STARTUP_MODE_ALIGN 时定位后立即加速 */
    }
}

/* 上电：预充时间、定位阻尼增益、起动方式默认值 */
void Startup_Init(void)
{
    gStartup.preChargeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;
    gStartup.mode = STARTUP_MODE;
    {   /* 定位各段 = 摆动周期 T 的倍数（cfg_internal.h ALIGN_*_T） */
        uint32_t t = QMath_Isqrt64(ALIGN_T2_TICKS2);
        s_alignMinTicks = (uint16_t)(t * (uint32_t)(ALIGN_MIN_T * 256.0 + 0.5) >> 8);
        s_alignStillTicks = (uint16_t)(t * (uint32_t)(ALIGN_STILL_T * 256.0 + 0.5) >> 8);
        s_alignMaxTicks = (uint16_t)(t * (uint32_t)(ALIGN_MAX_T * 256.0 + 0.5) >> 8);
        s_alignGoTicks = (uint16_t)(ALIGN_A / ALIGN_RAMP_A_S * SLOWLOOP_FREQ + 0.5) + (uint16_t)(t * (uint32_t)(ALIGN_GO_T * 256.0 + 0.5) >> 8);
        s_swingQuietTicks = (uint16_t)(t * (uint32_t)(ALIGN_SWING_QUIET_T * 256.0 + 0.5) >> 8);
        s_swingMaxTicks = (uint16_t)(t * (uint32_t)(ALIGN_SWING_MAX_T * 256.0 + 0.5) >> 8);
        s_swingMinSpd = (int16_t)(ALIGN_SWING_MIN_SPD_T / (double)(t ? t : 1) + 0.5);
    }

    /* 仅上电时计算一次（含开方与浮点） */
    s_dampQ8 = (int32_t)(ALIGN_C1 * (double)QMath_Isqrt64((uint64_t)(ALIGN_K_NM * ALIGN_J * 1099511627776.0)) / 1048576.0);
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
 * Align：d 轴电流斜坡升至 ALIGN_A 定位，q 轴按反电势施加阻尼，停稳后进入 I/F
 * -------------------------------------------------------------------------*/
/* Align 快环：辨识模式 / IPD 时由其直接输出电压；定位时电流环按定位角运行（电流指令由慢环给出） */
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
    th += (w * ALIGN_SWING_LAG_ANG_Q12) >> 12;
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
        if (dir != s_swingDir) { s_swingDir = dir; s_swingL1 = 0; s_swingL2 = 0x3FFF0000;
                                s_swingNegL1 = 0; s_swingNegL2 = 0x3FFF0000; }
    }
    e = s_swingDir * (s_emfFilt >> ALIGN_EMF_FILT_SH);
    vd = s_dtFilt >> ALIGN_EMF_FILT_SH;
    thr = ((int32_t)s_swingMinSpd * gGain.decKEmfQ15) >> 15;
    if (e >= thr || e <= -thr) s_swingMoved = 1;
    if (e == 0) {                                   /* 过零：两套记录都复位 */
        s_swingL1 = 0; s_swingL2 = 0; s_swingVd2 = vd;
        s_swingNegL1 = 0; s_swingNegL2 = 0; s_swingNegVd2 = vd;
    } else if (e < 0) {                             /* 逆向峰值：对称测量，接入后 I/F 从负转速加速过零 */
        s_swingL1 = 0; s_swingL2 = 0; s_swingVd2 = vd;
        e = -e;
        if (e >= s_swingNegL2 + (thr >> 2) + (s_swingNegL2 >> 4)) {
            s_swingNegL1 = s_swingNegL2; s_swingNegVd1 = s_swingNegVd2;
            s_swingNegL2 = e; s_swingNegVd2 = vd;
        } else if (s_swingNegL1 > 0 && s_swingNegL2 >= thr && e <= s_swingNegL1) {
            int32_t ed = (s_swingNegVd1 - vd) >> 1;
            if (ed < -(thr >> 2)) {
                s_swingNegL1 = 0; s_swingNegL2 = 0x3FFF0000;   /* 假峰值 */
            } else {
                swingCatch(e, ed, (int8_t)(-s_swingDir));
                return;
            }
        }
    } else {
        s_swingNegL1 = 0; s_swingNegL2 = 0; s_swingNegVd2 = vd;
        if (e >= s_swingL2 + (thr >> 2) + (s_swingL2 >> 4)) {         /* 升高一档 */
            s_swingL1 = s_swingL2; s_swingVd1 = s_swingVd2;
            s_swingL2 = e; s_swingVd2 = vd;
        } else if (s_swingL1 > 0 && s_swingL2 >= thr && e <= s_swingL1) {  /* 峰值已过，回落到上一个电平 */
            int32_t ed = (s_swingVd1 - vd) >> 1;
            if (ed < -(thr >> 2)) {
                s_swingL1 = 0; s_swingL2 = 0x3FFF0000;      /* 假峰值：e 回到 0 之前不再记录 */
            } else {
                swingCatch(e, ed, s_swingDir);
                return;
            }
        }
    }
    if (s_alignTicks >= (s_swingMoved ? s_swingMaxTicks : s_swingQuietTicks)) Sup_SwitchSwingOpenLoop(0, 0);
}

/* Align 慢环：
 *   直接拖动：立即进入 I/F（openloop.c 牵入段）；
 *   摆动中起动：见 alignSwingSlow；
 *   IPD 中：推进检测（Ipd_Slow），完成后判定，可信 -> 从 N 极角直接进入 I/F；不可信 -> 退回定位；
 *   定位：d 轴电流按斜坡升至 ALIGN_A，q 轴按反电势施加阻尼（转子摆动时施加反向转矩抑制）；
 *         起动（STARTUP_MODE_ALIGN、IPD 检测失败）：到达固定时间（电流上升 + ALIGN_GO_T 个 T）即进入 I/F；
 *         参数辨识：达到最短时间且反电势持续低于门限（已停稳），或超过最长时间 -> 开始测量 */
void Startup_AlignSlow(void)
{
    int32_t id = (int32_t)gFoc.idqRef.s16D + ALIGN_STEP_Q15;
    int32_t emf, iq;
    if (IDENT_TEST && Ident_Active()) { Ident_Slow(); Meas_SlowFilter(); Meas_SpeedHoldZero(); return; }
    if (s_drag) { Sup_SwitchDragOpenLoop(); Meas_SlowFilter(); Meas_SpeedHoldZero(); return; }
    if (s_swing) { alignSwingSlow(); Meas_SlowFilter(); Meas_SpeedHoldZero(); return; }
    if (s_ipd) {
        Ipd_Slow();
        if (gIpd.state == IPD_STATE_DONE) {
            if (Ipd_Evaluate()) {
                Sup_SwitchIpdOpenLoop(gIpd.theta);  /* 从测得的 N 极角开始拖动，无需定位 */
            } else {
                /* 凸极或极性对比度不足（转子在动等）：本次退回定位。偏置电流已回到 0，PWM 输出零电压。
                 * 先清积分器再清 s_ipd（分别为一次 32 / 8 位写）：快环看到 s_ipd = 0 时积分器已为 0 */
                FocCore_PresetPI(0, 0);
                s_ipd = 0;
            }
        }
        Meas_SlowFilter();
        Meas_SpeedHoldZero();
        return;
    }
    if (id > ALIGN_ID_Q15) id = ALIGN_ID_Q15;
    /* 阻尼：q 轴反电势 e_q = w*psi_f，经 ALIGN_EMF_FILT_TAU_S 低通，iq = -k*e_q。
     * e_q 取 q 轴电流 PI 的积分项（= Rs*iq + 逆变器电压误差 + e_q，稳态电压）减去 Rs*iq_实测，而不用 PI 总输出：
     * 总输出中含 Kp*误差 的瞬态分量，阻尼指令变化时会跳变，环路增益 > 1 时会振荡。 */
    emf = (gFoc.piQ.qdSum >> 15) - QMath_MulQ15(gGain.focKrsQ15, gFoc.idq.s16Q);
    s_emfFilt += emf - (s_emfFilt >> ALIGN_EMF_FILT_SH);
    /* 另外扣除逆变器电压误差。定位时 d 轴电流在另两相上各为 -ALIGN_A/2，摆动产生的 iq 会把其中一相推入
     * 死区误差的线性段（INVERTER_DT_I0_A 以内，等效电阻约为 Rs 的 2 倍以上），该电压随 iq 同步变化：不扣除时
     * 反电势只估出真值的一半左右，阻尼只有设计值的一半，判停稳也会在转子仍在摆动时成立（容差见 cfg_internal.h ALIGN_ZETA）。 */
    s_dtFilt += FluxObs_DtErr(gMeas.iab, gFoc.ang).s16Q - (s_dtFilt >> ALIGN_EMF_FILT_SH);
    iq = -(((s_emfFilt - s_dtFilt) >> ALIGN_EMF_FILT_SH) * s_dampQ8 >> 8);
    iq = QMath_Clamp(iq, ALIGN_ID_Q15);
    FocCore_SetIdqRef((int16_t)id, (int16_t)iq);
    ++s_alignTicks;
    {   /* 起动：到时间即拖动；参数辨识：满足最短时间后等待停稳 */
        int32_t e = (s_emfFilt - s_dtFilt) >> ALIGN_EMF_FILT_SH;
        if (e < 0) e = -e;
        if (s_alignTicks > s_alignMinTicks && e < gGain.alignStillEmfQ15) { if (s_still < 0xFFFF) s_still++; } else s_still = 0;
        if (s_alignGo ? s_alignTicks >= s_alignGoTicks
                      : ((s_alignTicks >= s_alignMinTicks && s_still >= s_alignStillTicks) || s_alignTicks >= s_alignMaxTicks)) {
            if (IDENT_TEST && Ident_StaticThisStart()) Ident_Begin();  /* 参数辨识：不切入 I/F，从此开始测量 */
            else Sup_SwitchAlignOpenLoop();  /* 定位后立即加速：观测器初始化为 |psi_f|∠0 */
        }
    }
    Meas_SlowFilter();
    Meas_SpeedHoldZero();
}
