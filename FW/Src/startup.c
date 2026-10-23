/* 启动流程各子状态，见 startup.h。原厂模式的逻辑与 test33 的 StateMachine.c 逐行对应。 */
#include "startup.h"
#include "supervisor.h"
#include "foc_core.h"
#include "est.h"
#include "hal.h"
#include "speed_ctrl.h"
#include "protect.h"
#include "mathx.h"
#include "flux_obs.h"
#include "meas.h"
#include "flystart.h"

Startup_TypeDef gStartup;

#if (STARTUP_MODE == STARTUP_IF)
volatile StartupIfDiag_TypeDef gStartupIF;
static volatile int16_t s_alignTheta;      /* 定位角（慢环写，快环读） */
static uint16_t s_alignTicks;
static volatile int32_t s_olTheta;         /* 开环角 Q16.16（快环推进） */
static volatile int32_t s_olSpeed;         /* 开环转速 Q31（快环斜坡） */
static volatile int32_t s_olTarget;        /* 目标转速 Q31，带方向（慢环写） */
static int32_t s_pllFilt;                  /* PLL 转速一阶滤波 Q31（慢环，tau 16 ms） */
static uint16_t s_agree, s_hold, s_time;
static int32_t s_dampQ8;                   /* 定位阻尼增益：Q15 电流 / Q15 电压，Q8 */
static int32_t s_emfFilt;                  /* 定位时 q 轴反电势（滤波），Q15 电压 << 3 */
static uint16_t s_still;                   /* 定位第二段：连续"静止"的拍数 */

int16_t Startup_IfSpeedQ15(void) { return (int16_t)(s_olSpeed >> 16); }

/* 开环目标转速：|指令| >= IF_SPIN_MIN_RPM 时拖到切换转速（之后切闭环，再由速度环
 * 走到指令）；否则就以指令转速一直开环跑 */
static int32_t ifTarget(void)
{
    int32_t c = gSpd.cmd;
    if (c >= IF_SPIN_MIN_Q15)  return IF_HANDOVER_Q31;
    if (c <= -IF_SPIN_MIN_Q15) return -IF_HANDOVER_Q31;
    return c << 16;
}

/* asin(x)，x 为 Q15（限 ±0.95），返回 Q15 角度（65536 = 360°）。用正弦表二分，慢环用 */
static int16_t asinQ15(int32_t x)
{
    int32_t lo = -16384, hi = 16384, mid;
    int i;
    if (x > 31130) x = 31130;
    if (x < -31130) x = -31130;
    for (i = 0; i < 15; i++) {
        mid = (lo + hi) / 2;
        if (Mc_SinCos((int16_t)mid).s16Sin < x) lo = mid; else hi = mid;
    }
    return (int16_t)lo;
}

uint8_t Startup_IfWantsFallback(void)
{
    int32_t filt = gMeas.speed, cmd = gSpd.cmd;
    int32_t af = filt < 0 ? -filt : filt;
    if (af >= IF_FALLBACK_Q15) return 0;
    /* 指令反向，或指令低于闭环最低转速：交给 I/F */
    return (uint8_t)((cmd > 0) != (filt > 0) || (cmd < 0 ? -cmd : cmd) < IF_SPIN_MIN_Q15);
}

void Startup_OnEnterAlign(void)
{
    s_alignTicks = 0;
    s_emfFilt = 0;
    s_still = 0;
    s_alignTheta = (IF_ALIGN_STAGES == 2) ? IF_ALIGN1_THETA : 0;
    gFoc.idqRef.s16D = 0;
    gFoc.idqRef.s16Q = 0;
    gStartupIF.stage = 1;
}

void Startup_OnEnterIF(uint8_t warm)
{
    int32_t sp;
    s_olTarget = ifTarget();
    if (warm) {
        /* 转矩连续：开环电流 I 沿 theta_ol，转子 q 轴分量 I*sin(theta_ol - theta_est)。
         * 让它等于闭环此刻的 iq：theta_ol = theta_est + asin(iq/I) */
        int32_t r = ((int32_t)gFoc.idqRef.s16Q << 15) / IF_IQ_Q15;
        s_olTheta = (int32_t)((uint32_t)gEst.pll.s32Theta + ((uint32_t)(uint16_t)asinQ15(r) << 16));
        sp = gEst.pll.s32Speed;
        if (sp > IF_WARM_MAX_Q31) sp = IF_WARM_MAX_Q31;      /* 只防异常值，正常 <= 回退转速 */
        if (sp < -IF_WARM_MAX_Q31) sp = -IF_WARM_MAX_Q31;
        s_olSpeed = sp;
        gProt.sIabc_Max.s16A = 0;    /* 缺相判定重新计 */
        gProt.sIabc_Max.s16B = 0;
        gProt.sIabc_Max.s16C = 0;
    } else {
        s_olTheta = (int32_t)gFoc.theta << 16;     /* = 定位角，电流方向与定位时相同，无冲击 */
        s_olSpeed = 0;
    }
    /* 电流沿开环角方向（开环坐标的 d 轴）：转子落后它 delta，转矩 ∝ I*sin(delta)，
     * 加速时 delta 自己长出来。冷启动时与定位电流方向连续，不像放在 q 轴那样一上来
     * 就满转矩踢一脚、激起新的摆动。 */
    gFoc.idqRef.s16D = IF_IQ_Q15;
    gFoc.idqRef.s16Q = 0;
    s_pllFilt = gEst.pll.s32Speed;
    s_agree = 0; s_hold = 0; s_time = 0;
    gStartupIF.stage = 3;
}

/* 转矩不变切换：开环时电流沿 theta_ol（开环 d 轴），在转子 q 轴上的分量是
 * I*sin(theta_ol - theta_r)。闭环第一拍就给这个值，id = 0。 */
static void ifHandover(void)
{
    int16_t d = (int16_t)((s_olTheta >> 16) - (gEst.pll.s32Theta >> 16));
    sAngle_Trig a = Mc_SinCos(d);
    gFoc.idqRef.s16Q = Mx_MulQ15(gFoc.idqRef.s16D, a.s16Sin);
    gFoc.idqRef.s16D = 0;
    gEst.thetaOffset = 0;
    gStartupIF.deltaAtHandover = d;
    gStartupIF.iqAtHandover = gFoc.idqRef.s16Q;
    gStartupIF.handovers++;
    gStartupIF.stage = 5;
    Sup_SwitchStartupSpin();
}
#else
void Startup_OnEnterAlign(void) { }
void Startup_OnEnterIF(uint8_t warm) { (void)warm; }
#endif

static uint16_t s_u16BemfCheckCnt;     /* 飞车检测：反电势电流超门限的持续拍数 */
static uint16_t s_u16BlockTime;        /* Startup 堵转/缺相计时（慢环拍，见下方说明） */

void Startup_Init(void)
{
    gStartup.preChargeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;
    gStartup.spdMax = Q15(MAXSTARTUP_SPEED / SPEED_MAX);
    gStartup.iqCmd = Q15(IQ_STARTUP / I_MAX);
    gStartup.timeCmd = STARTUP_TIME * SLOWLOOP_FREQ;
    gStartup.tqTimeCmd = TORQUELOOP_TIME * SLOWLOOP_FREQ;
    gStartup.alignTimeCmd = ALIGN_TIME * SLOWLOOP_FREQ;
    gStartup.alignCurAcc = Q15(ALIGN_CURRENT_RAMP / I_MAX / SLOWLOOP_FREQ);

    gFly.s16BemfAmpTh = Q15(FLYCURRENT_TH_PU);
    gFly.u32FlyBrakeTimeCmd = PWM_FREQ * FLYBRAKE_TIME;

    gStartup.detSpd = abs(Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX));
    gStartup.detTime = 10;
#if (STARTUP_MODE == STARTUP_IF)
    /* 只在上电算一次（含开方和浮点） */
    s_dampQ8 = (int32_t)(IF_ALIGN_C1 * (double)Mx_Isqrt64((uint64_t)(IF_ALIGN_K_NM * IF_ALIGN_J * 1099511627776.0)) / 1048576.0);
#endif
}

void Startup_ResetFlyCheck(void)  { s_u16BemfCheckCnt = 0; }
void Startup_ResetBlockTimer(void) { s_u16BlockTime = 0; }

/* ---------------------------------------------------------------------------
 * Charge：三相下桥轮流导通给自举电容充电，PRECHARGE_TIME 后进 Calib
 * -------------------------------------------------------------------------*/
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

void Startup_ChargeSlow(void)
{
    Meas_SlowFilter();
}

/* ---------------------------------------------------------------------------
 * Calib：飞车检测。下桥短接，测短路电流幅值（= 反电势/阻抗）。
 * 超门限持续 320 拍（20 ms）-> 转子在转 -> Torqueloop；
 * FLYBRAKE_TIME（100 ms）内没超 -> 静止 -> Align。
 * -------------------------------------------------------------------------*/
void Startup_CalibFast(void)
{
    FlyStart_CalibFast();
    gMeas.speedRaw = gEst.pll.s32Speed >> 16;

    if (gFly.s16BemfAmp > gFly.s16BemfAmpTh) {
#if (STARTUP_MODE == STARTUP_IF)
        if (s_u16BemfCheckCnt > 320 && ((gSpd.cmd > 0) != (gMeas.speed > 0))) {
            /* 转子逆着指令方向在转：不去追它（I/F 接不住反转的转子），留在 Calib 继续
             * 下桥短接制动，短路电流低于门限（本电机约 30 rpm）后按静止处理 -> 定位 */
            gFly.u32FlyBrakeTimeCnt = 0;
        } else
#endif
        if (s_u16BemfCheckCnt > 320) {
            s_u16BemfCheckCnt = 0;
            gFly.u32FlyBrakeTimeCnt = 0;
            Sup_SwitchCalibTorqueloop();
        } else {
            s_u16BemfCheckCnt++;
        }
    }

    gFly.u32FlyBrakeTimeCnt++;
    if (gFly.u32FlyBrakeTimeCnt > gFly.u32FlyBrakeTimeCmd) {
        s_u16BemfCheckCnt = 0;
        gFly.u32FlyBrakeTimeCnt = 0;
        Sup_SwitchCalibAlign();
    }
}

void Startup_CalibSlow(void)
{
    Meas_SlowFilter();
}

/* ---------------------------------------------------------------------------
 * Align：原厂快速启动下只是 Iq = IQ_STARTUP/4、角度 0 打 ALIGN_TIME（1 ms）
 * -------------------------------------------------------------------------*/
void Startup_AlignFast(void)
{
    FocCore_Clarke();
#if (STARTUP_MODE == STARTUP_IF)
    gFoc.theta = s_alignTheta;
#else
    gFoc.theta = 0;
#endif
    gFoc.thetaInv = gFoc.theta;
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

void Startup_AlignSlow(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t id = (int32_t)gFoc.idqRef.s16D + IF_ALIGN_STEP_Q15;
    int32_t emf, iq;
    if (id > IF_ALIGN_ID_Q15) id = IF_ALIGN_ID_Q15;
    gFoc.idqRef.s16D = (int16_t)id;
    /* 阻尼：q 轴反电势 e_q = w*psi_f，8 ms 低通，iq = -k*e_q。
     * e_q 取 q 轴电流 PI 的积分项（= Rs*iq + e_q，稳态电压）减 Rs*iq_实测，不用 PI 总输出：
     * 总输出里有 Kp*误差 的瞬态（Kp = 86 V/A），阻尼指令一变它就跳，环路增益 > 1 会振荡。 */
    emf = (gFoc.piQ.qdSum >> 15) - Mx_MulQ15(sFluxObs.s16KRs, gFoc.idq.s16Q);
    s_emfFilt += emf - (s_emfFilt >> 3);
    iq = -((s_emfFilt >> 3) * s_dampQ8 >> 8);
    iq = Mx_Clamp(iq, IF_ALIGN_ID_Q15);
    gFoc.idqRef.s16Q = (int16_t)iq;
    ++s_alignTicks;
#if (IF_ALIGN_STAGES == 1)
    {   /* 一段：0° 定位，打满最短时间后等停稳 */
        int32_t e = s_emfFilt >> 3;
        if (e < 0) e = -e;
        if (s_alignTicks > IF_ALIGN1_TICKS && e < IF_ALIGN_STILL_EMF_Q15) { if (s_still < 0xFFFF) s_still++; } else s_still = 0;
        if ((s_alignTicks >= IF_ALIGN1_TICKS && s_still >= IF_ALIGN_STILL_TICKS) || s_alignTicks >= IF_ALIGN2_MAX_TICKS) {
            gStartupIF.alignTime = s_alignTicks;
            Sup_SwitchAlignStartup();       /* 观测器种在 |psi_f|∠0 */
        }
    }
#else
    if (s_alignTicks > IF_ALIGN1_TICKS) {
        uint16_t k = s_alignTicks - IF_ALIGN1_TICKS;            /* 第二段已过的拍数 */
        int32_t e = s_emfFilt >> 3;
        gStartupIF.stage = 2;
        /* 120° -> 0° 慢慢转 */
        s_alignTheta = (k < IF_ALIGN_SWEEP_TICKS) ? (int16_t)(IF_ALIGN1_THETA - (int32_t)IF_ALIGN1_THETA * k / IF_ALIGN_SWEEP_TICKS) : 0;
        if (e < 0) e = -e;
        if (k > IF_ALIGN_SWEEP_TICKS && e < IF_ALIGN_STILL_EMF_Q15) { if (s_still < 0xFFFF) s_still++; } else s_still = 0;
        if ((k >= IF_ALIGN_SWEEP_TICKS + IF_ALIGN2_TICKS && s_still >= IF_ALIGN_STILL_TICKS) ||
            k >= IF_ALIGN2_MAX_TICKS) {
            gStartupIF.alignTime = s_alignTicks;
            Sup_SwitchAlignStartup();       /* 观测器种在 |psi_f|∠0（转子已被定住） */
        }
    }
#endif
    Meas_SlowFilter();
    gMeas.speed = 0;
#else
    if (gSpd.cmd > 0) gFoc.idqRef.s16Q = gStartup.iqCmd >> 2;
    else                   gFoc.idqRef.s16Q = -gStartup.iqCmd >> 2;

    if (++gStartup.alignTimeCnt > gStartup.alignTimeCmd) {
        Sup_SwitchAlignStartup();
        gStartup.alignTimeCnt = 0;
    }
    Meas_SlowFilter();
    gMeas.speed = 0;
#endif
}

/* ---------------------------------------------------------------------------
 * Startup
 *   原厂：固定 Iq 拖动，观测器 + PLL 从第一拍就闭环（PLL 带 12.8 rpm 下限），
 *         vendorStartupDone 每 11 个慢环拍查一次 gMeas.speed >= 80 rpm 就切 Spin。
 *   I/F： 电流矢量开环旋转（角度 = 开环积分），观测器 + PLL 只跟踪；转速一致后切 Spin。
 * -------------------------------------------------------------------------*/
void Startup_StartupFast(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t sp = s_olSpeed, tgt = s_olTarget, step;
    FocCore_Clarke();
    (void)Est_TrackTick();              /* 观测器 + PLL 只跟踪，不用来控制 */
    if (sp < tgt)      { sp += IF_ACC_STEP_Q31; if (sp > tgt) sp = tgt; }
    else if (sp > tgt) { sp -= IF_ACC_STEP_Q31; if (sp < tgt) sp = tgt; }
    s_olSpeed = sp;
    step = (sp >> 15) * (int32_t)gEst.pll.s16ThGain;           /* 一拍角度增量 Q16.16 */
    gFoc.theta = (int16_t)(s_olTheta >> 16);
#if CTRL_ANGLE_COMP
    gFoc.thetaInv = gFoc.theta + (int16_t)(((step >> 8) * INVPARK_ADV_Q8) >> 16);
#else
    gFoc.thetaInv = gFoc.theta;
#endif
    s_olTheta = (int32_t)((uint32_t)s_olTheta + (uint32_t)step);
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
    gMeas.speedRaw = (int16_t)(sp >> 16);           /* Meas_SlowFilter -> gMeas.speed 跟开环转速 */
#else
    FocCore_Clarke();
    Est_StartupTick();
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
    Est_StartupReportSpeed();
#endif

    /* 缺相检测：记录各相电流峰值 */
    gProt.sIabc_Max.s16A = (gProt.sIabc_Max.s16A > gMeas.iabc.s16A) ? gProt.sIabc_Max.s16A : gMeas.iabc.s16A;
    gProt.sIabc_Max.s16B = (gProt.sIabc_Max.s16B > gMeas.iabc.s16B) ? gProt.sIabc_Max.s16B : gMeas.iabc.s16B;
    gProt.sIabc_Max.s16C = (gProt.sIabc_Max.s16C > gMeas.iabc.s16C) ? gProt.sIabc_Max.s16C : gMeas.iabc.s16C;
}

#if (STARTUP_MODE == STARTUP_VENDOR)
/* 原厂启动的切 Spin 判据（原 Fast_Boot_StartUp）：每 DetcTime 个慢环拍查一次，
 * 估计转速与指令同向且 |转速| >= DetcSpd（80 rpm）就切。 */
static uint8_t vendorStartupDone(void)
{
    if (gStartup.timeCnt <= gStartup.detTime) return 0;
    gStartup.timeCnt = 0;
    if (gMeas.speed >= gStartup.detSpd && gSpd.cmd > 0) return 1;
    if (gMeas.speed <= -gStartup.detSpd && gSpd.cmd < 0) return 1;
    return 0;
}
#endif

void Startup_StartupSlow(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t ol = s_olSpeed;
    s_olTarget = ifTarget();
    s_pllFilt += (gEst.pll.s32Speed >> 4) - (s_pllFilt >> 4);
    if (s_time < 0xFFFF) s_time++;
    gStartupIF.olSpeed = (int16_t)(ol >> 16);
    gStartupIF.pllFilt = (int16_t)(s_pllFilt >> 16);

    /* 缺相：拖了 STARTUP_TIME 之后，任一相的正向电流峰值没到门限 */
    if (s_time > gStartup.timeCmd &&
        ((gProt.sIabc_Max.s16A < gProt.s16I_Phaseless_threshold) ||
         (gProt.sIabc_Max.s16B < gProt.s16I_Phaseless_threshold) ||
         (gProt.sIabc_Max.s16C < gProt.s16I_Phaseless_threshold))) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        if (gSup.faultRestarts < 10) gSup.faultRestarts++;
    } else if (ol == s_olTarget && (ol == IF_HANDOVER_Q31 || ol == -IF_HANDOVER_Q31)) {
        int64_t d = (int64_t)s_pllFilt - ol;
        int64_t tol = ((int64_t)(ol < 0 ? -ol : ol) * IF_AGREE_Q15) >> 15;
        if (d < 0) d = -d;
        if (d < tol) s_agree++; else s_agree = 0;
        gStartupIF.agreeCnt = s_agree;
        if (s_agree >= IF_AGREE_TICKS) {
            ifHandover();
        } else if (++s_hold > IF_TIMEOUT_TICKS) {
            gStartupIF.timeouts++;
            gStartupIF.stage = 6;
            Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
            if (gSup.faultRestarts < 10) gSup.faultRestarts++;
        }
    }
    Meas_SlowFilter();
#else
    gStartup.timeCnt++;
    /* 注意：原厂代码在这里和下面各加一次，所以 s_u16BlockTime 每个慢环拍 +2：
     * 缺相判定实际在 STARTUP_TIME/2 = 1 s、堵转判定在 3*STARTUP_TIME/2 = 3 s。
     * 保持原样（台架上的保护时间就是这样标定的）。 */
    s_u16BlockTime++;

    /* Iq 斜坡升到 IQ_STARTUP */
    if (gSpd.cmd > 0) {
        gFoc.idqRef.s16Q += gStartup.alignCurAcc;
        if (gFoc.idqRef.s16Q > gStartup.iqCmd) gFoc.idqRef.s16Q = gStartup.iqCmd;
    } else if (gSpd.cmd < 0) {
        gFoc.idqRef.s16Q -= gStartup.alignCurAcc;
        if (gFoc.idqRef.s16Q < -gStartup.iqCmd) gFoc.idqRef.s16Q = -gStartup.iqCmd;
    }

    /* 速度斜坡（STARTUP_SPEED_RAMP，限 MAXSTARTUP_SPEED）——只给保护/切换参考，不驱动角度 */
    gSpd.rampQ31 = Mc_RampQ31((gSpd.cmd << 16), &gSpd.speedRampSt);
    if (gSpd.rampQ31 > (gStartup.spdMax << 16)) {
        gSpd.rampQ31 = (gStartup.spdMax << 16);
    } else if (gSpd.rampQ31 < -(gStartup.spdMax << 16)) {
        gSpd.rampQ31 = -(gStartup.spdMax << 16);
    }
    gSpd.ramp = gSpd.rampQ31 >> 16;

    s_u16BlockTime++;

    if (vendorStartupDone()) {
        Sup_SwitchStartupSpin();
        s_u16BlockTime = 0;
    } else {
        if (s_u16BlockTime > (gStartup.timeCmd * 3)) {
            Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
            s_u16BlockTime = 0;
            if (gSup.faultRestarts < 10) gSup.faultRestarts++;
        }
        if (s_u16BlockTime > gStartup.timeCmd) {
            if ((gProt.sIabc_Max.s16A < gProt.s16I_Phaseless_threshold) ||
                (gProt.sIabc_Max.s16B < gProt.s16I_Phaseless_threshold) ||
                (gProt.sIabc_Max.s16C < gProt.s16I_Phaseless_threshold)) {
                Sup_RaiseFault(MC_FAULT_PHASELESS);
                s_u16BlockTime = 0;
                if (gSup.faultRestarts < 10) gSup.faultRestarts++;
            }
        }
    }
    Meas_SlowFilter();
#endif
}

/* ---------------------------------------------------------------------------
 * Torqueloop：飞车启动。按 Calib 测到的角度/转速直接进力矩闭环 TORQUELOOP_TIME
 * -------------------------------------------------------------------------*/
void Startup_TorqueloopFast(void)
{
    FocCore_Clarke();
    Est_SpinTick();
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

void Startup_TorqueloopSlow(void)
{
    gStartup.tqTimeCnt++;
    if (gStartup.tqTimeCnt > gStartup.tqTimeCmd) {
        Sup_SwitchTorqueloopSpin();
        gStartup.tqTimeCnt = 0;
    }
    Meas_SlowFilter();
}
