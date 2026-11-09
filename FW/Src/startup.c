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
static int32_t s_dampQ8;                   /* 定位阻尼增益：Q15 电流 / Q15 电压，Q8 */
static int32_t s_emfFilt;                  /* 定位时 q 轴反电势（滤波），Q15 电压 << ALIGN_EMF_FILT_SH */
static uint16_t s_still;                   /* 定位第二段：连续"静止"的拍数 */
static volatile uint8_t s_ipd;             /* 1 = Align 子状态正在执行 IPD（慢环写，快环读） */

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

/* 上电：预充时间、定位阻尼增益 */
void Startup_Init(void)
{
    gStartup.preChargeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;

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

/* 飞车检测慢环：测量滤波；对快环保存的电流平方值开方用于诊断显示（快环不开方） */
void Startup_FlyCheckSlow(void)
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
                Sup_SwitchIpdOpenLoop(gIpd.theta);  /* 从测得的 N 极角开始拖动，无需定位 */
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
            else Sup_SwitchAlignOpenLoop();  /* 观测器初始化为 |psi_f|∠0 */
        }
    }
    Meas_SlowFilter();
    Meas_SpeedHoldZero();
}
