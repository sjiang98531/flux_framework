/* ============================================================================
 * supervisor.c —— 状态机、子状态切换、保护、初始化编排
 *
 * 由原厂 StateMachine.c（test33 版）拆出：启动相关子状态的处理函数移到
 * startup.c，速度环移到 speed_ctrl.c，观测器/PLL 编排移到 est.c。
 * 状态、切换条件、切换时做的事和顺序都没变。
 * ==========================================================================*/
#include "supervisor.h"
#include "ctrl_globals.h"
#include "hal.h"
#include "foc_core.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "est.h"
#include "startup.h"
#include "speed_ctrl.h"
#include "diag.h"
#include <stdlib.h>

m1_app_mainstate_t eM1_MainState = MainState_Init;
m1_run_substate_t  eM1_RunSubState = RunState_Charge;

static uint16_t s_u16SpinCheckTime;     /* Spin 里 Fast_Boot_Spin 的轮询分频（原 NewStartUpCloseTime） */

static void M1_Fault_Fast(void);
static void M1_Init_Fast(void);
static void M1_Stop_Fast(void);
static void M1_Run_Fast(void);
static void M1_Fault_Slow(void);
static void M1_Init_Slow(void);
static void M1_Stop_Slow(void);
static void M1_Run_Slow(void);
static void M1_RunSpinFast(void);
static void M1_RunSpinSlow(void);
static void M1_FaultDetection(void);
static void Fault_LED_Disp(void);

const pfcn_void_void s_STATE_FAST[4] = {M1_Fault_Fast, M1_Init_Fast, M1_Stop_Fast, M1_Run_Fast};
const pfcn_void_void s_STATE_SLOW[4] = {M1_Fault_Slow, M1_Init_Slow, M1_Stop_Slow, M1_Run_Slow};

/* Run 子状态表，顺序 = m1_run_substate_t */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_FAST[6] = {
    Startup_ChargeFast, Startup_CalibFast, Startup_AlignFast,
    Startup_StartupFast, M1_RunSpinFast, Startup_TorqueloopFast
};
static const pfcn_void_void s_M1_STATE_RUN_TABLE_SLOW[6] = {
    Startup_ChargeSlow, Startup_CalibSlow, Startup_AlignSlow,
    Startup_StartupSlow, M1_RunSpinSlow, Startup_TorqueloopSlow
};

/* ===========================================================================
 * 入口
 * =========================================================================*/
static void initProtection(void)
{
    USER.s16OVPCmd = Q15(DCBUS_OVER / UDC_MAX);
    USER.s16UVPCmd = Q15(DCBUS_UNDER / UDC_MAX);
    USER.u16FreeWheelTimeCmd = FREEWHEEL_TIME * SLOWLOOP_FREQ;
    USER.u16FaultReleaseTimeCmd = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    USER.u8FaultReleaseCnt = 0;

    FOC.sProtect.sIabc_Max.s16A = 0;
    FOC.sProtect.sIabc_Max.s16B = 0;
    FOC.sProtect.sIabc_Max.s16C = 0;
    FOC.sProtect.u32Ia_Integral = 0;
    FOC.sProtect.u32Ib_Integral = 0;
    FOC.sProtect.u32Ic_Integral = 0;
    FOC.sProtect.bRotor_block = 0;
    FOC.sProtect.bErr_Spin = 0;
    FOC.sProtect.bSpeed_OC = 0;
    FOC.sProtect.bPhaseless = 0;
    FOC.sProtect.u16PhaselessTimeCmd = PHASSLESS_TIME * SLOWLOOP_FREQ;
    FOC.sProtect.u16Speed_OC_cmd = Q15(SPEED_OC_RATIO);
    FOC.sProtect.u16BlockSpeed = Q15(BLOCK_SPEED_THRESHOLD / SPEED_MAX);
    FOC.sProtect.u16BlockTimeCmd = BLOCK_TIME * SLOWLOOP_FREQ;
    FOC.sProtect.u16BlockVoltageAmp_square = MLIB_Mul_Q15(Q15(BLOCK_VOLTAGE_THRESHOLD / U_MAX), Q15(BLOCK_VOLTAGE_THRESHOLD / U_MAX));
    FOC.sProtect.s16I_Phaseless_threshold = Q15(I_PHASSLESS_THRESHOLD / I_MAX);
}

void Ctrl_Init(void)
{
    HAL_InitParams();

    /* Var_Filt（闭源库）用的三个测量滤波器：母线电压、转速、母线电流，窗口 2^3 */
    USER.sSpeed_Filt.u16Sh = 3; USER.sSpeed_Filt.s32Acc = 0;
    USER.sVbus_Filt.u16Sh = 3;  USER.sVbus_Filt.s32Acc = 0;
    USER.sIbus_Filt.u16Sh = 3;  USER.sIbus_Filt.s32Acc = 0;

    SpeedCtrl_Init();
    initProtection();
    Startup_Init();
    FocCore_Init();
    FluxPLL_Init(&FOC.sPLL);
    Diag_Init();
    FluxObs_Init();
}

void Ctrl_ResetRunVariables(void)
{
    FOC.Speed.qdSum = 0;
    FOC.Power.qdSum = 0;
    FOC.CurID.qdSum = 0;
    FOC.CurIQ.qdSum = 0;
    FOC.sVdq.s16Q = 0;
    FOC.sVdq.s16D = 0;
    FOC.sIdq_cmd.s16Q = 0;
    FOC.sIdq_cmd.s16D = 0;
    USER.sSpeed_Filt.s32Acc = 0;
    USER.sSpeedRamp.s32State = 0;

    FOC.sPLL.s32I_1 = 0;
    FOC.sPLL.s32Speed = 0;

    FOC.FluxFeedback.s16Speed = 0;
    FOC.s16SpeedRamp = 0;
    FOC.s32SpeedRamp = 0;
    FOC.s16SpdFilt = 0;
    USER.u16AlignTimeCnt = 0;
    USER.u16PreChargeTime = 0;
    USER.u8PreChargeCnt = 0;
    USER.u16StartupTimeCnt = 0;
    USER.u16TorqueloopTimeCnt = 0;
    USER.u16SpinToStartCnt = 0;
    FOC.s16CmdTheta = 0;
    FOC.s16ErrTheta = 0;
    FOC.s16ThetaComp = 0;

    FOC.sProtect.sIabc_Max.s16A = 0;
    FOC.sProtect.sIabc_Max.s16B = 0;
    FOC.sProtect.sIabc_Max.s16C = 0;
    FOC.sProtect.u32Ia_Integral = 0;
    FOC.sProtect.u32Ib_Integral = 0;
    FOC.sProtect.u32Ic_Integral = 0;
    FOC.sProtect.bRotor_block = 0;
    FOC.sProtect.bErr_Spin = 0;
    FOC.sProtect.bSpeed_OC = 0;
    FOC.sProtect.bPhaseless = 0;
    FOC.sProtect.u16PhaselessTimeCnt = 0;
}

void Ctrl_FastTick(void)
{
    /* 观测器用"上一拍"的比较值重建电压：在本拍生成新比较值之前抓快照。
     * 只有 Run 且已对齐、上下桥全部输出 PWM 时重建电压才可信。 */
    FluxObs_Capture(&FOC, (uint8_t)(eM1_MainState == MainState_Run &&
                                     eM1_RunSubState >= RunState_Align &&
                                     HAL_PwmAllEnabled()));
    s_STATE_FAST[eM1_MainState]();
    if (eM1_MainState != MainState_Run) FluxObs_Reset();
    gFluxDiag.mainState = (uint16_t)eM1_MainState;
    gFluxDiag.runState = (uint16_t)eM1_RunSubState;
}

void Ctrl_SlowTick(void)
{
    s_STATE_SLOW[eM1_MainState]();
    Diag_SlowTick();
}

/* ===========================================================================
 * 主状态
 * =========================================================================*/
static void M1_SwitchRunFault(void)
{
    DISABLE_PWMOUT();
    eM1_MainState = MainState_Fault;
    eM1_RunSubState = RunState_Charge;
    PWMABC_Dis;
    PWMABCN_Dis;
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
}

void Sup_RaiseFault(uint16_t faultId)
{
    MC_FAULT_SET(M1FaultID, faultId);
    M1FaultID_Record |= M1FaultID;
    M1_SwitchRunFault();
}

static void M1_SwitchRunStop(void)
{
    eM1_MainState = MainState_Stop;
    eM1_RunSubState = RunState_Charge;
    PWMABC_Dis;
    PWMABCN_Dis;
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
}

static void M1_SwitchStopRun(void)
{
    eM1_MainState = MainState_Run;
    eM1_RunSubState = RunState_Charge;
    HAL_FgSetDuty(0);
}

static void M1_Fault_Fast(void)
{
    DISABLE_PWMOUT();
    M1_FaultDetection();
}

static void M1_Init_Fast(void)
{
    DISABLE_PWMOUT();
    Ctrl_ResetRunVariables();
    /* Init -> Stop */
    eM1_MainState = MainState_Stop;
    Ctrl_ResetRunVariables();
}

static void M1_Stop_Fast(void)
{
    DISABLE_PWMOUT();
    HAL_PwmSetHalfDuty();
    FOC.s16VbusAvg = FOC.s16Vbus;
    M1_FaultDetection();
    if (M1FaultID) eM1_MainState = MainState_Fault;
}

static void M1_Run_Fast(void)
{
    s_M1_STATE_RUN_TABLE_FAST[eM1_RunSubState]();
    M1_FaultDetection();
    if (M1FaultID) M1_SwitchRunFault();
}

static void M1_Fault_Slow(void)
{
    static uint16_t u16FaultTimeCnt;

    Var_Filt();
    Fault_LED_Disp();
    /* 故障消失后等 FAULTRELEASE_TIME 再回 Stop */
    if (!M1FaultID) {
        u16FaultTimeCnt++;
        if (u16FaultTimeCnt > USER.u16FaultReleaseTimeCmd) {
            eM1_MainState = MainState_Stop;
            u16FaultTimeCnt = 0;
        }
    }
}

static void M1_Init_Slow(void)
{
    Var_Filt();
}

static void M1_Stop_Slow(void)
{
    FOC.s16SpdCmd = USER.s16VspCmd;
#if (CLOSE_LOOP == TORQUE_LOOP)
    if ((abs(FOC.s16SpdCmd) > Q15(STOP_TO_RUN_IQ / I_MAX)) && (USER.u8FaultReleaseCnt < 5)) M1_SwitchStopRun();
    else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_IQ / I_MAX)) USER.u8FaultReleaseCnt = 0;
#elif (CLOSE_LOOP == SPEED_LOOP)
    if (abs(FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && (USER.u8FaultReleaseCnt < 5)) M1_SwitchStopRun();
    else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_SPEED / SPEED_MAX)) USER.u8FaultReleaseCnt = 0;
#elif (CLOSE_LOOP == POWER_LOOP)
    if (abs(FOC.s16SpdCmd) > Q15(STOP_TO_RUN_POWER / POWER_MAX) && (USER.u8FaultReleaseCnt < 5)) M1_SwitchStopRun();
    else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_POWER / POWER_MAX)) USER.u8FaultReleaseCnt = 0;
#endif
    Var_Filt();
}

static void M1_Run_Slow(void)
{
    static uint8_t u8SpdUpCnt = 0;

    FOC.s16SpdCmd = USER.s16VspCmd;
    SpeedCtrl_UpdatePowerFdk();

#if (CLOSE_LOOP == TORQUE_LOOP)
    if (abs(FOC.s16SpdCmd) < Q15(RUN_TO_STOP_IQ / I_MAX)) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#elif (CLOSE_LOOP == SPEED_LOOP)
    if (abs(FOC.s16SpdCmd) < abs(Q15(RUN_TO_STOP_SPEED / SPEED_MAX))) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#elif (CLOSE_LOOP == POWER_LOOP)
    if (abs(FOC.s16SpdCmd) < Q15(RUN_TO_STOP_POWER / POWER_MAX)) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#endif
    else if (++u8SpdUpCnt >= USER.u8SpeedUpDatePeriod) {
        u8SpdUpCnt = 0;
        HAL_FgSetDuty(0);          /* FG 输出原厂就是关的 */
    }
    s_M1_STATE_RUN_TABLE_SLOW[eM1_RunSubState]();
}

/* ===========================================================================
 * Spin（闭环运行）
 * =========================================================================*/
static void M1_RunSpinFast(void)
{
    FocCore_Clarke(&FOC);

    /* 飞车启动交接角差，每拍消 9 LSB */
    if (FOC.s16ErrTheta > 9) FOC.s16ErrTheta -= 9;
    else if (FOC.s16ErrTheta < -9) FOC.s16ErrTheta += 9;
    else FOC.s16ErrTheta = 0;

    Est_SpinTick(&FOC);
    FOC_Current_Controller(&FOC);

    /* 缺相检测：三相电流绝对值积分（MCF_ADV_Protect 比较） */
    FOC.sProtect.u32Ia_Integral += MLIB_Abs_Q15(FOC.sIabc.s16A);
    FOC.sProtect.u32Ib_Integral += MLIB_Abs_Q15(FOC.sIabc.s16B);
    FOC.sProtect.u32Ic_Integral += MLIB_Abs_Q15(FOC.sIabc.s16C);

    /* 低速七段式，高速（> 0.2 * SPEED_MAX = 640 rpm）五段式 */
    FocCore_Modulate(&FOC, (FOC.s16SpdFilt < Q15(0.2)) ? SEVENMODE : FIVEMODE);
}

static void M1_RunSpinSlow(void)
{
    SpeedCtrl_SpinSlow();

    /* 指令反向：记下来（非 Fast Boot 模式用） */
    if ((MLIB_Mul_Q15(FOC.s16PreSpdCmd, FOC.s16SpdCmd)) < 0)
        USER.u16SpinToStartCnt = PWM_FREQ;
    FOC.s16PreSpdCmd = FOC.s16SpdCmd;

    /* 回退到 Startup 的两条路：
     *  1) 同向时每 DetcTime 拍查一次 Fast_Boot_Spin（filt 掉到 40 rpm 以下）
     *  2) 反向（指令与 filt 异号）且 |filt| < 50 rpm：立即回退 */
    if (MLIB_Mul_Q15(FOC.s16SpdCmd, FOC.s16SpdFilt) > 0) {
        if (++s_u16SpinCheckTime >= FastBoot_Para.DetcTime) {
            s_u16SpinCheckTime = 0;
            if (Fast_Boot_Spin(&FOC) == 1) Sup_SwitchSpinStartup();
        }
    } else {
        if ((MLIB_Mul_Q15(FOC.s16SpdCmd, FOC.s16SpdFilt) < 0) && (abs(FOC.s16SpdFilt) < Q15(50.0 / SPEED_MAX)))
            Sup_SwitchSpinStartup();
    }

    /* 不用原厂 SMO 的高速角度补偿（磁链 PLL 没有滞后，test25/26） */
    FOC.s16ThetaComp = 0;

    Var_Filt();

    /* 原厂高级保护：堵转 / 反转失控 / 缺相 / 超速 */
    MCF_ADV_Protect(&FOC);
    if (FOC.sProtect.bErr_Spin) {
        FOC.sProtect.bErr_Spin = 0;
        Sup_SwitchSpinStartup();
    }
    if (FOC.sProtect.bRotor_block) {
        FOC.sProtect.u16BlockCounter = 0;
        Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
        if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
    }
    if (FOC.sProtect.bPhaseless) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
    }
    if (FOC.sProtect.bSpeed_OC) {
        Sup_RaiseFault(MC_FAULT_SPEED_OC);
        if (USER.u8FaultReleaseCnt < 10) USER.u8FaultReleaseCnt++;
    }
}

/* ===========================================================================
 * Run 子状态切换
 * =========================================================================*/
void Sup_SwitchChargeCalib(void)
{
    DISABLE_PWMOUT();
#if (FLY_CHECK_0BEMF == 1)
    PWMABC_Enb;
    PWMABCN_Enb;
#else
    PWMABC_Dis;
    PWMABCN_Enb;
#endif
    FocCore_ModulateZero();
    ENABLE_PWMOUT();

    Ctrl_ResetRunVariables();
    Startup_ResetFlyCheck();
    USER.u16PreChargeTime = 0;
    eM1_RunSubState = RunState_Calib;
}

void Sup_SwitchCalibAlign(void)
{
    eM1_RunSubState = RunState_Align;
    DISABLE_PWMOUT();
    PWMABC_Enb;
    PWMABCN_Enb;
    FocCore_ModulateZero();
    ENABLE_PWMOUT();
}

void Sup_SwitchCalibTorqueloop(void)
{
    /* 飞车：按测到的转速给初始转矩，按测到的角度接管 */
    FOC.CurIQ.qdSum = FOC.s16SpdFilt << 14;
    FOC.sIdq_cmd.s16Q = abs(FOC.s16SpdFilt);
    FOC.s16CmdTheta = FOC.sFly_Start.s16PosElEst_fly;
    eM1_RunSubState = RunState_Torqueloop;

    DISABLE_PWMOUT();
    PWMABC_Enb;
    PWMABCN_Enb;
    FocCore_ModulateZero();
    ENABLE_PWMOUT();

    /* 飞车检测借用过 sPLL（MLIB_PLL），这里换回磁链 PLL 的增益并清状态 */
    FluxPLL_Init(&FOC.sPLL);
}

void Sup_SwitchAlignStartup(void)
{
    Est_OnEnterStartupCold(&FOC);
    eM1_RunSubState = RunState_Startup;
    SpeedCtrl_UseStartupRamp();
}

void Sup_SwitchStartupSpin(void)
{
    SpeedCtrl_OnEnterSpin();
    FOC.sProtect.u16PhaselessTimeCnt = 0;
    FOC.s16PreSpdCmd = FOC.s16SpdCmd;
    eM1_RunSubState = RunState_Spin;
    s_u16SpinCheckTime = 0;
}

void Sup_SwitchTorqueloopSpin(void)
{
    SpeedCtrl_OnEnterSpin();
    FOC.s16ErrTheta = 0;
    FOC.sProtect.u16PhaselessTimeCnt = 0;
    FOC.s16PreSpdCmd = FOC.s16SpdCmd;
    if ((MLIB_Mul_Q15(FOC.s16SpdFilt, FOC.s16SpdCmd)) < 0)
        USER.u16SpinToStartCnt = 20000;
    eM1_RunSubState = RunState_Spin;
}

void Sup_SwitchSpinStartup(void)
{
    eM1_RunSubState = RunState_Startup;
    Est_OnEnterStartupFromSpin(&FOC);       /* 转子还在转：上报转速从当前值开始 */
    USER.sSpeedRamp.s32State = FOC.s16SpdFilt << 16;
    SpeedCtrl_UseStartupRamp();
}

/* ===========================================================================
 * 保护
 * =========================================================================*/
static void M1_FaultDetection(void)
{
    static uint16_t u16OvCntr, u16UvCntr;

    MC_FAULT_CLEAR_ALL(M1FaultID);

    if (FOC.s16VbusAvg > USER.s16OVPCmd) {
        if (u16OvCntr <= 1600) u16OvCntr++;          /* 0.1 s */
        else {
            MC_FAULT_SET(M1FaultID, MC_FAULT_U_DCBUS_OVER);
            DISABLE_PWMOUT();
        }
    } else {
        u16OvCntr = 0;
    }

    if (FOC.s16VbusAvg < USER.s16UVPCmd) {
        if (u16UvCntr <= 1600) u16UvCntr++;
        else {
            MC_FAULT_SET(M1FaultID, MC_FAULT_U_DCBUS_UNDER);
            DISABLE_PWMOUT();
        }
    } else {
        u16UvCntr = 0;
    }

    M1FaultID_Record |= M1FaultID;
}

/* 故障码闪灯（LED 驱动原厂就注释掉了，只保留计时） */
static void Fault_LED_Disp(void)
{
    static uint16_t u16FaultTimeCnt = 0;
    static uint16_t u16FaultCnt = 0;
    static uint16_t u16FalultNumber = 0;
    int i;

    if (M1FaultID) {
        for (i = 0; i < 15; i++) {
            u16FalultNumber = M1FaultID & (0x01 << i) ? i : u16FalultNumber;
        }
        u16FaultTimeCnt++;
        if (u16FaultTimeCnt == 500) {
            /* LED1_ON(); */
        } else if (u16FaultTimeCnt == 1000) {
            u16FaultCnt++;
            /* LED1_OFF(); */
            if (u16FaultCnt == u16FalultNumber) u16FaultTimeCnt = 2000;
            else u16FaultTimeCnt = 0;
        } else if (u16FaultTimeCnt == 4000) {
            u16FaultCnt = 0;
            u16FaultTimeCnt = 0;
        }
    } else {
        u16FaultCnt = 0;
        u16FaultTimeCnt = 0;
    }
}
