/* 启动流程各子状态，见 startup.h。原厂模式的逻辑与 test33 的 StateMachine.c 逐行对应。 */
#include "startup.h"
#include "supervisor.h"
#include "foc_core.h"
#include "est.h"
#include "hal.h"
#include "ctrl_globals.h"
#include "mathx.h"
#include "flux_obs.h"
#include "meas.h"
#include "flystart.h"

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
    int32_t c = FOC.s16SpdCmd;
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
    int32_t filt = FOC.s16SpdFilt, cmd = FOC.s16SpdCmd;
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
    FOC.sIdq_cmd.s16D = 0;
    FOC.sIdq_cmd.s16Q = 0;
    gStartupIF.stage = 1;
}

void Startup_OnEnterIF(uint8_t warm)
{
    int32_t sp;
    s_olTarget = ifTarget();
    if (warm) {
        /* 转矩连续：开环电流 I 沿 theta_ol，转子 q 轴分量 I*sin(theta_ol - theta_est)。
         * 让它等于闭环此刻的 iq：theta_ol = theta_est + asin(iq/I) */
        int32_t r = ((int32_t)FOC.sIdq_cmd.s16Q << 15) / IF_IQ_Q15;
        s_olTheta = (int32_t)((uint32_t)FOC.sPLL.s32Theta + ((uint32_t)(uint16_t)asinQ15(r) << 16));
        sp = FOC.sPLL.s32Speed;
        if (sp > IF_WARM_MAX_Q31) sp = IF_WARM_MAX_Q31;      /* 只防异常值，正常 <= 回退转速 */
        if (sp < -IF_WARM_MAX_Q31) sp = -IF_WARM_MAX_Q31;
        s_olSpeed = sp;
        FOC.sProtect.sIabc_Max.s16A = 0;    /* 缺相判定重新计 */
        FOC.sProtect.sIabc_Max.s16B = 0;
        FOC.sProtect.sIabc_Max.s16C = 0;
    } else {
        s_olTheta = (int32_t)FOC.s16CmdTheta << 16;     /* = 定位角，电流方向与定位时相同，无冲击 */
        s_olSpeed = 0;
    }
    /* 电流沿开环角方向（开环坐标的 d 轴）：转子落后它 delta，转矩 ∝ I*sin(delta)，
     * 加速时 delta 自己长出来。冷启动时与定位电流方向连续，不像放在 q 轴那样一上来
     * 就满转矩踢一脚、激起新的摆动。 */
    FOC.sIdq_cmd.s16D = IF_IQ_Q15;
    FOC.sIdq_cmd.s16Q = 0;
    s_pllFilt = FOC.sPLL.s32Speed;
    s_agree = 0; s_hold = 0; s_time = 0;
    gStartupIF.stage = 3;
}

/* 转矩不变切换：开环时电流沿 theta_ol（开环 d 轴），在转子 q 轴上的分量是
 * I*sin(theta_ol - theta_r)。闭环第一拍就给这个值，id = 0。 */
static void ifHandover(void)
{
    int16_t d = (int16_t)((s_olTheta >> 16) - (FOC.sPLL.s32Theta >> 16));
    sAngle_Trig a = Mc_SinCos(d);
    FOC.sIdq_cmd.s16Q = Mx_MulQ15(FOC.sIdq_cmd.s16D, a.s16Sin);
    FOC.sIdq_cmd.s16D = 0;
    FOC.s16ErrTheta = 0;
    gStartupIF.deltaAtHandover = d;
    gStartupIF.iqAtHandover = FOC.sIdq_cmd.s16Q;
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
    USER.u16PreChargeTimeCmd = PWM_FREQ / 1000.0 * PRECHARGE_TIME;
    USER.s16StartupSpdCmdMax = Q15(MAXSTARTUP_SPEED / SPEED_MAX);
    USER.s16StartupIqCmd = Q15(IQ_STARTUP / I_MAX);
    USER.u16StartupTimeCmd = STARTUP_TIME * SLOWLOOP_FREQ;
    USER.u16TorqueloopTimeCmd = TORQUELOOP_TIME * SLOWLOOP_FREQ;
    USER.s16AlignIqCmd = Q15(IQ_ALIGN / I_MAX);
    USER.u16AlignTimeCmd = ALIGN_TIME * SLOWLOOP_FREQ;
    USER.s16AlignCurAcc = Q15(ALIGN_CURRENT_RAMP / I_MAX / SLOWLOOP_FREQ);

    FOC.sFly_Start.s16BemfAmpTh = Q15(FLYCURRENT_TH_PU);
    FOC.sFly_Start.u32FlyBrakeTimeCmd = PWM_FREQ * FLYBRAKE_TIME;

    FastBoot_Para.DetcSpd = abs(Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX));
    FastBoot_Para.DetcTime = 10;
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

    if (++USER.u8PreChargeCnt > 2) USER.u8PreChargeCnt = 0;
    if (USER.u8PreChargeCnt == 0)      HAL_PwmLowOnA();
    else if (USER.u8PreChargeCnt == 1) HAL_PwmLowOnB();
    else                               HAL_PwmLowOnC();

    if (USER.u16PreChargeTime++ >= USER.u16PreChargeTimeCmd) {
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
    FlyStart_CalibFast(&FOC);
    FOC.FluxFeedback.s16Speed = FOC.sPLL.s32Speed >> 16;

    if (FOC.sFly_Start.s16BemfAmp > FOC.sFly_Start.s16BemfAmpTh) {
#if (STARTUP_MODE == STARTUP_IF)
        if (s_u16BemfCheckCnt > 320 && ((FOC.s16SpdCmd > 0) != (FOC.s16SpdFilt > 0))) {
            /* 转子逆着指令方向在转：不去追它（I/F 接不住反转的转子），留在 Calib 继续
             * 下桥短接制动，短路电流低于门限（本电机约 30 rpm）后按静止处理 -> 定位 */
            FOC.sFly_Start.u32FlyBrakeTimeCnt = 0;
        } else
#endif
        if (s_u16BemfCheckCnt > 320) {
            s_u16BemfCheckCnt = 0;
            FOC.sFly_Start.u32FlyBrakeTimeCnt = 0;
            Sup_SwitchCalibTorqueloop();
        } else {
            s_u16BemfCheckCnt++;
        }
    }

    FOC.sFly_Start.u32FlyBrakeTimeCnt++;
    if (FOC.sFly_Start.u32FlyBrakeTimeCnt > FOC.sFly_Start.u32FlyBrakeTimeCmd) {
        s_u16BemfCheckCnt = 0;
        FOC.sFly_Start.u32FlyBrakeTimeCnt = 0;
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
    FocCore_Clarke(&FOC);
#if (STARTUP_MODE == STARTUP_IF)
    FOC.s16CmdTheta = s_alignTheta;
#else
    FOC.s16CmdTheta = 0;
#endif
    FOC.s16CmdTheta_Update = FOC.s16CmdTheta;
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
}

void Startup_AlignSlow(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t id = (int32_t)FOC.sIdq_cmd.s16D + IF_ALIGN_STEP_Q15;
    int32_t emf, iq;
    if (id > IF_ALIGN_ID_Q15) id = IF_ALIGN_ID_Q15;
    FOC.sIdq_cmd.s16D = (int16_t)id;
    /* 阻尼：q 轴反电势 e_q = w*psi_f，8 ms 低通，iq = -k*e_q。
     * e_q 取 q 轴电流 PI 的积分项（= Rs*iq + e_q，稳态电压）减 Rs*iq_实测，不用 PI 总输出：
     * 总输出里有 Kp*误差 的瞬态（Kp = 86 V/A），阻尼指令一变它就跳，环路增益 > 1 会振荡。 */
    emf = (FOC.CurIQ.qdSum >> 15) - Mx_MulQ15(sFluxObs.s16KRs, FOC.sIdq.s16Q);
    s_emfFilt += emf - (s_emfFilt >> 3);
    iq = -((s_emfFilt >> 3) * s_dampQ8 >> 8);
    iq = Mx_Clamp(iq, IF_ALIGN_ID_Q15);
    FOC.sIdq_cmd.s16Q = (int16_t)iq;
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
    FOC.s16SpdFilt = 0;
#else
    if (FOC.s16SpdCmd > 0) FOC.sIdq_cmd.s16Q = USER.s16StartupIqCmd >> 2;
    else                   FOC.sIdq_cmd.s16Q = -USER.s16StartupIqCmd >> 2;

    if (++USER.u16AlignTimeCnt > USER.u16AlignTimeCmd) {
        Sup_SwitchAlignStartup();
        USER.u16AlignTimeCnt = 0;
    }
    Meas_SlowFilter();
    FOC.s16SpdFilt = 0;
#endif
}

/* ---------------------------------------------------------------------------
 * Startup
 *   原厂：固定 Iq 拖动，观测器 + PLL 从第一拍就闭环（PLL 带 12.8 rpm 下限），
 *         vendorStartupDone 每 11 个慢环拍查一次 s16SpdFilt >= 80 rpm 就切 Spin。
 *   I/F： 电流矢量开环旋转（角度 = 开环积分），观测器 + PLL 只跟踪；转速一致后切 Spin。
 * -------------------------------------------------------------------------*/
void Startup_StartupFast(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t sp = s_olSpeed, tgt = s_olTarget, step;
    FocCore_Clarke(&FOC);
    (void)Est_TrackTick(&FOC);              /* 观测器 + PLL 只跟踪，不用来控制 */
    if (sp < tgt)      { sp += IF_ACC_STEP_Q31; if (sp > tgt) sp = tgt; }
    else if (sp > tgt) { sp -= IF_ACC_STEP_Q31; if (sp < tgt) sp = tgt; }
    s_olSpeed = sp;
    step = (sp >> 15) * (int32_t)FOC.sPLL.s16ThGain;           /* 一拍角度增量 Q16.16 */
    FOC.s16CmdTheta = (int16_t)(s_olTheta >> 16);
#if CTRL_ANGLE_COMP
    FOC.s16CmdTheta_Update = FOC.s16CmdTheta + (int16_t)(((step >> 8) * INVPARK_ADV_Q8) >> 16);
#else
    FOC.s16CmdTheta_Update = FOC.s16CmdTheta;
#endif
    s_olTheta = (int32_t)((uint32_t)s_olTheta + (uint32_t)step);
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
    FOC.FluxFeedback.s16Speed = (int16_t)(sp >> 16);           /* Meas_SlowFilter -> s16SpdFilt 跟开环转速 */
#else
    FocCore_Clarke(&FOC);
    Est_StartupTick(&FOC);
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
    Est_StartupReportSpeed(&FOC);
#endif

    /* 缺相检测：记录各相电流峰值 */
    FOC.sProtect.sIabc_Max.s16A = (FOC.sProtect.sIabc_Max.s16A > FOC.sIabc.s16A) ? FOC.sProtect.sIabc_Max.s16A : FOC.sIabc.s16A;
    FOC.sProtect.sIabc_Max.s16B = (FOC.sProtect.sIabc_Max.s16B > FOC.sIabc.s16B) ? FOC.sProtect.sIabc_Max.s16B : FOC.sIabc.s16B;
    FOC.sProtect.sIabc_Max.s16C = (FOC.sProtect.sIabc_Max.s16C > FOC.sIabc.s16C) ? FOC.sProtect.sIabc_Max.s16C : FOC.sIabc.s16C;
}

#if (STARTUP_MODE == STARTUP_VENDOR)
/* 原厂启动的切 Spin 判据（原 Fast_Boot_StartUp）：每 DetcTime 个慢环拍查一次，
 * 估计转速与指令同向且 |转速| >= DetcSpd（80 rpm）就切。 */
static uint8_t vendorStartupDone(const FOC_TypeDef *f)
{
    if (USER.u16StartupTimeCnt <= FastBoot_Para.DetcTime) return 0;
    USER.u16StartupTimeCnt = 0;
    if (f->s16SpdFilt >= FastBoot_Para.DetcSpd && f->s16SpdCmd > 0) return 1;
    if (f->s16SpdFilt <= -FastBoot_Para.DetcSpd && f->s16SpdCmd < 0) return 1;
    return 0;
}
#endif

void Startup_StartupSlow(void)
{
#if (STARTUP_MODE == STARTUP_IF)
    int32_t ol = s_olSpeed;
    s_olTarget = ifTarget();
    s_pllFilt += (FOC.sPLL.s32Speed >> 4) - (s_pllFilt >> 4);
    if (s_time < 0xFFFF) s_time++;
    gStartupIF.olSpeed = (int16_t)(ol >> 16);
    gStartupIF.pllFilt = (int16_t)(s_pllFilt >> 16);

    /* 缺相：拖了 STARTUP_TIME 之后，任一相的正向电流峰值没到门限 */
    if (s_time > USER.u16StartupTimeCmd &&
        ((FOC.sProtect.sIabc_Max.s16A < FOC.sProtect.s16I_Phaseless_threshold) ||
         (FOC.sProtect.sIabc_Max.s16B < FOC.sProtect.s16I_Phaseless_threshold) ||
         (FOC.sProtect.sIabc_Max.s16C < FOC.sProtect.s16I_Phaseless_threshold))) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
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
            if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
        }
    }
    Meas_SlowFilter();
#else
    USER.u16StartupTimeCnt++;
    /* 注意：原厂代码在这里和下面各加一次，所以 s_u16BlockTime 每个慢环拍 +2：
     * 缺相判定实际在 STARTUP_TIME/2 = 1 s、堵转判定在 3*STARTUP_TIME/2 = 3 s。
     * 保持原样（台架上的保护时间就是这样标定的）。 */
    s_u16BlockTime++;

    /* Iq 斜坡升到 IQ_STARTUP */
    if (FOC.s16SpdCmd > 0) {
        FOC.sIdq_cmd.s16Q += USER.s16AlignCurAcc;
        if (FOC.sIdq_cmd.s16Q > USER.s16StartupIqCmd) FOC.sIdq_cmd.s16Q = USER.s16StartupIqCmd;
    } else if (FOC.s16SpdCmd < 0) {
        FOC.sIdq_cmd.s16Q -= USER.s16AlignCurAcc;
        if (FOC.sIdq_cmd.s16Q < -USER.s16StartupIqCmd) FOC.sIdq_cmd.s16Q = -USER.s16StartupIqCmd;
    }

    /* 速度斜坡（STARTUP_SPEED_RAMP，限 MAXSTARTUP_SPEED）——只给保护/切换参考，不驱动角度 */
    FOC.s32SpeedRamp = Mc_RampQ31((FOC.s16SpdCmd << 16), &USER.sSpeedRamp);
    if (FOC.s32SpeedRamp > (USER.s16StartupSpdCmdMax << 16)) {
        FOC.s32SpeedRamp = (USER.s16StartupSpdCmdMax << 16);
    } else if (FOC.s32SpeedRamp < -(USER.s16StartupSpdCmdMax << 16)) {
        FOC.s32SpeedRamp = -(USER.s16StartupSpdCmdMax << 16);
    }
    FOC.s16SpeedRamp = FOC.s32SpeedRamp >> 16;

    s_u16BlockTime++;

    if (vendorStartupDone(&FOC)) {
        Sup_SwitchStartupSpin();
        s_u16BlockTime = 0;
    } else {
        if (s_u16BlockTime > (USER.u16StartupTimeCmd * 3)) {
            Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
            s_u16BlockTime = 0;
            if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
        }
        if (s_u16BlockTime > USER.u16StartupTimeCmd) {
            if ((FOC.sProtect.sIabc_Max.s16A < FOC.sProtect.s16I_Phaseless_threshold) ||
                (FOC.sProtect.sIabc_Max.s16B < FOC.sProtect.s16I_Phaseless_threshold) ||
                (FOC.sProtect.sIabc_Max.s16C < FOC.sProtect.s16I_Phaseless_threshold)) {
                Sup_RaiseFault(MC_FAULT_PHASELESS);
                s_u16BlockTime = 0;
                if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
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
    FocCore_Clarke(&FOC);
    Est_SpinTick(&FOC);
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
}

void Startup_TorqueloopSlow(void)
{
    USER.u16TorqueloopTimeCnt++;
    if (USER.u16TorqueloopTimeCnt > USER.u16TorqueloopTimeCmd) {
        Sup_SwitchTorqueloopSpin();
        USER.u16TorqueloopTimeCnt = 0;
    }
    Meas_SlowFilter();
}
