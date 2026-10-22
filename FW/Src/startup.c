/* 启动流程各子状态，见 startup.h。逻辑与 test33 的 StateMachine.c 逐行对应。 */
#include "startup.h"
#include "supervisor.h"
#include "foc_core.h"
#include "est.h"
#include "hal.h"
#include "ctrl_globals.h"

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
    Var_Filt();
}

/* ---------------------------------------------------------------------------
 * Calib：飞车检测。下桥短接，测短路电流幅值（= 反电势/阻抗）。
 * 超门限持续 320 拍（20 ms）-> 转子在转 -> Torqueloop；
 * FLYBRAKE_TIME（100 ms）内没超 -> 静止 -> Align。
 * -------------------------------------------------------------------------*/
void Startup_CalibFast(void)
{
#if (FLY_CHECK_0BEMF == 1)
    MCF_FlyStart_0BEMF(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
#else
    MCF_FlyStart_2R(&FOC);
#endif
    FOC.FluxFeedback.s16Speed = FOC.sPLL.s32Speed >> 16;

    if (FOC.sFly_Start.s16BemfAmp > FOC.sFly_Start.s16BemfAmpTh) {
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
    Var_Filt();
}

/* ---------------------------------------------------------------------------
 * Align：原厂快速启动下只是 Iq = IQ_STARTUP/4、角度 0 打 ALIGN_TIME（1 ms）
 * -------------------------------------------------------------------------*/
void Startup_AlignFast(void)
{
    FocCore_Clarke(&FOC);
    FOC.s16CmdTheta = 0;
    FOC.s16CmdTheta_Update = FOC.s16CmdTheta;
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
}

void Startup_AlignSlow(void)
{
    if (FOC.s16SpdCmd > 0) FOC.sIdq_cmd.s16Q = USER.s16StartupIqCmd >> 2;
    else                   FOC.sIdq_cmd.s16Q = -USER.s16StartupIqCmd >> 2;

    if (++USER.u16AlignTimeCnt > USER.u16AlignTimeCmd) {
        Sup_SwitchAlignStartup();
        USER.u16AlignTimeCnt = 0;
    }
    Var_Filt();
    FOC.s16SpdFilt = 0;
}

/* ---------------------------------------------------------------------------
 * Startup：固定 Iq 拖动，观测器 + PLL 从第一拍就闭环（PLL 带 12.8 rpm 下限），
 * Fast_Boot_StartUp 每 11 个慢环拍查一次 s16SpdFilt >= 80 rpm 就切 Spin。
 * -------------------------------------------------------------------------*/
void Startup_StartupFast(void)
{
    FocCore_Clarke(&FOC);
    Est_StartupTick(&FOC);
    FOC_Current_Controller(&FOC);
    FocCore_Modulate(&FOC, SEVENMODE);
    Est_StartupReportSpeed(&FOC);

    /* 缺相检测：记录各相电流峰值 */
    FOC.sProtect.sIabc_Max.s16A = (FOC.sProtect.sIabc_Max.s16A > FOC.sIabc.s16A) ? FOC.sProtect.sIabc_Max.s16A : FOC.sIabc.s16A;
    FOC.sProtect.sIabc_Max.s16B = (FOC.sProtect.sIabc_Max.s16B > FOC.sIabc.s16B) ? FOC.sProtect.sIabc_Max.s16B : FOC.sIabc.s16B;
    FOC.sProtect.sIabc_Max.s16C = (FOC.sProtect.sIabc_Max.s16C > FOC.sIabc.s16C) ? FOC.sProtect.sIabc_Max.s16C : FOC.sIabc.s16C;
}

void Startup_StartupSlow(void)
{
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
    FOC.s32SpeedRamp = MLIB_Ramp_Q31((FOC.s16SpdCmd << 16), &USER.sSpeedRamp);
    if (FOC.s32SpeedRamp > (USER.s16StartupSpdCmdMax << 16)) {
        FOC.s32SpeedRamp = (USER.s16StartupSpdCmdMax << 16);
    } else if (FOC.s32SpeedRamp < -(USER.s16StartupSpdCmdMax << 16)) {
        FOC.s32SpeedRamp = -(USER.s16StartupSpdCmdMax << 16);
    }
    FOC.s16SpeedRamp = FOC.s32SpeedRamp >> 16;

    s_u16BlockTime++;

    if (Fast_Boot_StartUp(&FOC) == 1) {
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
    Var_Filt();
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
    Var_Filt();
}
