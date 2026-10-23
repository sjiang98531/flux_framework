/* ============================================================================
 * supervisor.c —— 状态机、子状态切换、保护、初始化编排
 *
 * 由原厂 StateMachine.c（test33 版）拆出：启动相关子状态的处理函数移到
 * startup.c，速度环移到 speed_ctrl.c，观测器/PLL 编排移到 est.c。
 * 状态、切换条件、切换时做的事和顺序都没变。
 * ==========================================================================*/
#include "supervisor.h"
#include "hal.h"
#include "foc_core.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "est.h"
#include "startup.h"
#include "speed_ctrl.h"
#include "diag.h"
#include "mtpa_fw.h"
#include "meas.h"
#include "protect.h"
#include "flystart.h"
#include <stdlib.h>

Supervisor_TypeDef gSup = { MainState_Init, RunState_Charge };

static uint16_t s_u16SpinCheckTime;     /* Spin 里 spinTooSlow 的轮询分频（原 NewStartUpCloseTime） */

/* Spin 回退判据（原 Fast_Boot_Spin）：估计转速掉到 DetcSpd/2（40 rpm）以下，
 * 或与指令反向，就回到 Startup。 */
static uint8_t spinTooSlow(void)
{
    int32_t half = gStartup.detSpd >> 1;
    if (gMeas.speed >= half && gSpd.cmd > 0) return 0;
    if (gMeas.speed <= -half && gSpd.cmd < 0) return 0;
    gStartup.timeCnt = 0;
    return 1;
}

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
    gProt.ovpCmd = Q15(DCBUS_OVER / UDC_MAX);
    gProt.uvpCmd = Q15(DCBUS_UNDER / UDC_MAX);
    gSup.faultReleaseTicks = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    gSup.faultRestarts = 0;

    gProt.sIabc_Max.s16A = 0;
    gProt.sIabc_Max.s16B = 0;
    gProt.sIabc_Max.s16C = 0;
    gProt.u32Ia_Integral = 0;
    gProt.u32Ib_Integral = 0;
    gProt.u32Ic_Integral = 0;
    gProt.bRotor_block = 0;
    gProt.bErr_Spin = 0;
    gProt.bSpeed_OC = 0;
    gProt.bPhaseless = 0;
    gProt.u16PhaselessTimeCmd = PHASSLESS_TIME * SLOWLOOP_FREQ;
    gProt.u16Speed_OC_cmd = Q15(SPEED_OC_RATIO);
    gProt.u16BlockSpeed = Q15(BLOCK_SPEED_THRESHOLD / SPEED_MAX);
    gProt.u16BlockTimeCmd = BLOCK_TIME * SLOWLOOP_FREQ;
    gProt.u16BlockVoltageAmp_square = MLIB_Mul_Q15(Q15(BLOCK_VOLTAGE_THRESHOLD / U_MAX), Q15(BLOCK_VOLTAGE_THRESHOLD / U_MAX));
    gProt.s16I_Phaseless_threshold = Q15(I_PHASSLESS_THRESHOLD / I_MAX);
}

void Ctrl_Init(void)
{
    HAL_InitParams();

    /* Meas_SlowFilter 用的三个测量滤波器：母线电压、转速、母线电流，窗口 2^3 */
    gMeas.speedFilt.u16Sh = 3; gMeas.speedFilt.s32Acc = 0;
    gMeas.vbusFilt.u16Sh = 3;  gMeas.vbusFilt.s32Acc = 0;
    gMeas.ibusFilt.u16Sh = 3;  gMeas.ibusFilt.s32Acc = 0;

    SpeedCtrl_Init();
    initProtection();
    Startup_Init();
    FocCore_Init();
    FluxPLL_Init(&gEst.pll);
    Diag_Init();
    FluxObs_Init();
    MtpaFw_Init();
}

void Ctrl_ResetRunVariables(void)
{
    gSpd.pi.qdSum = 0;
    gSpd.piPower.qdSum = 0;
    gFoc.piD.qdSum = 0;
    gFoc.piQ.qdSum = 0;
    gFoc.vdq.s16Q = 0;
    gFoc.vdq.s16D = 0;
    gFoc.idqRef.s16Q = 0;
    gFoc.idqRef.s16D = 0;
    gMeas.speedFilt.s32Acc = 0;
    gSpd.speedRampSt.s32State = 0;

    gEst.pll.s32I_1 = 0;
    gEst.pll.s32Speed = 0;

    gMeas.speedRaw = 0;
    gSpd.ramp = 0;
    gSpd.rampQ31 = 0;
    gMeas.speed = 0;
    gStartup.alignTimeCnt = 0;
    gStartup.preChargeCnt = 0;
    gStartup.preChargePhase = 0;
    gStartup.timeCnt = 0;
    gStartup.tqTimeCnt = 0;
    gFoc.theta = 0;
    gEst.thetaOffset = 0;

    gProt.sIabc_Max.s16A = 0;
    gProt.sIabc_Max.s16B = 0;
    gProt.sIabc_Max.s16C = 0;
    gProt.u32Ia_Integral = 0;
    gProt.u32Ib_Integral = 0;
    gProt.u32Ic_Integral = 0;
    gProt.bRotor_block = 0;
    gProt.bErr_Spin = 0;
    gProt.bSpeed_OC = 0;
    gProt.bPhaseless = 0;
    gProt.u16PhaselessTimeCnt = 0;
    MtpaFw_Reset();         /* MTPA/弱磁关闭时是空函数 */
    FocCore_ResetDecouple();        /* 积分器上面已清零，只复位开关，不搬前馈 */
}

void Ctrl_FastTick(void)
{
    /* 观测器用"上一拍"的比较值重建电压：在本拍生成新比较值之前抓快照。
     * 只有 Run 且已对齐、上下桥全部输出 PWM 时重建电压才可信。 */
    FluxObs_Capture((uint8_t)(gSup.mainState == MainState_Run &&
                                     gSup.runState >= RunState_Align &&
                                     HAL_PwmAllEnabled()));
    s_STATE_FAST[gSup.mainState]();
    if (gSup.mainState != MainState_Run) FluxObs_Reset();
    gFluxDiag.mainState = (uint16_t)gSup.mainState;
    gFluxDiag.runState = (uint16_t)gSup.runState;
}

/* 观测器去直流相位补偿用的转速：Spin 用 PLL 转速；I/F 拖动用开环转速（转子同步）；
 * 其余（原厂 Startup、飞车 Torqueloop、停机）不补——那时 PLL 转速不可信。 */
static int16_t compSpeed(void)
{
    if (gSup.mainState != MainState_Run) return 0;
    if (gSup.runState == RunState_Spin) return (int16_t)(gEst.pll.s32Speed >> 16);
#if (STARTUP_MODE == STARTUP_IF)
    if (gSup.runState == RunState_Startup) return Startup_IfSpeedQ15();
#endif
    return 0;
}

void Ctrl_SlowTick(void)
{
    s_STATE_SLOW[gSup.mainState]();
    FluxObs_SetSpeed(compSpeed());      /* 关闭补偿时是空函数 */
    Diag_SlowTick();
}

/* ===========================================================================
 * 主状态
 * =========================================================================*/
static void M1_SwitchRunFault(void)
{
    DISABLE_PWMOUT();
    gSup.mainState = MainState_Fault;
    gSup.runState = RunState_Charge;
    PWMABC_Dis;
    PWMABCN_Dis;
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
}

void Sup_RaiseFault(uint16_t faultId)
{
    MC_FAULT_SET(gSup.fault, faultId);
    gSup.faultRecord |= gSup.fault;
    M1_SwitchRunFault();
}

static void M1_SwitchRunStop(void)
{
    gSup.mainState = MainState_Stop;
    gSup.runState = RunState_Charge;
    PWMABC_Dis;
    PWMABCN_Dis;
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
}

static void M1_SwitchStopRun(void)
{
    gSup.mainState = MainState_Run;
    gSup.runState = RunState_Charge;
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
    gSup.mainState = MainState_Stop;
    Ctrl_ResetRunVariables();
}

static void M1_Stop_Fast(void)
{
    DISABLE_PWMOUT();
    HAL_PwmSetHalfDuty();
    gMeas.vbusAvg = gMeas.vbus;
    M1_FaultDetection();
    if (gSup.fault) gSup.mainState = MainState_Fault;
}

static void M1_Run_Fast(void)
{
    s_M1_STATE_RUN_TABLE_FAST[gSup.runState]();
    M1_FaultDetection();
    if (gSup.fault) M1_SwitchRunFault();
}

static void M1_Fault_Slow(void)
{
    static uint16_t u16FaultTimeCnt;

    Meas_SlowFilter();
    Fault_LED_Disp();
    /* 故障消失后等 FAULTRELEASE_TIME 再回 Stop */
    if (!gSup.fault) {
        u16FaultTimeCnt++;
        if (u16FaultTimeCnt > gSup.faultReleaseTicks) {
            gSup.mainState = MainState_Stop;
            u16FaultTimeCnt = 0;
        }
    }
}

static void M1_Init_Slow(void)
{
    Meas_SlowFilter();
}

static void M1_Stop_Slow(void)
{
    gSpd.cmd = gSup.speedCmd;
#if (CLOSE_LOOP == TORQUE_LOOP)
    if ((abs(gSpd.cmd) > Q15(STOP_TO_RUN_IQ / I_MAX)) && (gSup.faultRestarts < 5)) M1_SwitchStopRun();
    else if (abs(gSpd.cmd) < Q15(STOP_TO_RUN_IQ / I_MAX)) gSup.faultRestarts = 0;
#elif (CLOSE_LOOP == SPEED_LOOP)
    if (abs(gSpd.cmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && (gSup.faultRestarts < 5)) M1_SwitchStopRun();
    else if (abs(gSpd.cmd) < Q15(STOP_TO_RUN_SPEED / SPEED_MAX)) gSup.faultRestarts = 0;
#elif (CLOSE_LOOP == POWER_LOOP)
    if (abs(gSpd.cmd) > Q15(STOP_TO_RUN_POWER / POWER_MAX) && (gSup.faultRestarts < 5)) M1_SwitchStopRun();
    else if (abs(gSpd.cmd) < Q15(STOP_TO_RUN_POWER / POWER_MAX)) gSup.faultRestarts = 0;
#endif
    Meas_SlowFilter();
}

static void M1_Run_Slow(void)
{
    static uint8_t u8SpdUpCnt = 0;

    gSpd.cmd = gSup.speedCmd;
    SpeedCtrl_UpdatePowerFdk();

#if (CLOSE_LOOP == TORQUE_LOOP)
    if (abs(gSpd.cmd) < Q15(RUN_TO_STOP_IQ / I_MAX)) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#elif (CLOSE_LOOP == SPEED_LOOP)
    if (abs(gSpd.cmd) < abs(Q15(RUN_TO_STOP_SPEED / SPEED_MAX))) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#elif (CLOSE_LOOP == POWER_LOOP)
    if (abs(gSpd.cmd) < Q15(RUN_TO_STOP_POWER / POWER_MAX)) {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();
    }
#endif
    else if (++u8SpdUpCnt >= gHal.speedUpdateDiv) {
        u8SpdUpCnt = 0;
        HAL_FgSetDuty(0);          /* FG 输出原厂就是关的 */
    }
    s_M1_STATE_RUN_TABLE_SLOW[gSup.runState]();
}

/* ===========================================================================
 * Spin（闭环运行）
 * =========================================================================*/
static void M1_RunSpinFast(void)
{
    FocCore_Clarke();

    /* 飞车启动交接角差，每拍消 9 LSB */
    if (gEst.thetaOffset > 9) gEst.thetaOffset -= 9;
    else if (gEst.thetaOffset < -9) gEst.thetaOffset += 9;
    else gEst.thetaOffset = 0;

    Est_SpinTick();
    FocCore_CurrentLoop();

    /* 缺相检测：三相电流绝对值积分（Protect_SpinSlow 比较） */
    gProt.u32Ia_Integral += MLIB_Abs_Q15(gMeas.iabc.s16A);
    gProt.u32Ib_Integral += MLIB_Abs_Q15(gMeas.iabc.s16B);
    gProt.u32Ic_Integral += MLIB_Abs_Q15(gMeas.iabc.s16C);

    /* 低速七段式，高速（> 0.2 * SPEED_MAX = 640 rpm）五段式 */
#if CTRL_FIX_PWMMODE_ABS
    FocCore_Modulate((abs(gMeas.speed) < Q15(0.2)) ? SEVENMODE : FIVEMODE);
#else
    FocCore_Modulate((gMeas.speed < Q15(0.2)) ? SEVENMODE : FIVEMODE);   /* 原厂：反转永远七段 */
#endif
}

static void M1_RunSpinSlow(void)
{
    SpeedCtrl_SpinSlow();
    MtpaFw_Slow();          /* id 指令（MTPA + 弱磁）与 iq 限幅；关闭时是空函数 */


    /* 回退到 Startup 的两条路：
     *  1) 同向时每 DetcTime 拍查一次 spinTooSlow（filt 掉到 40 rpm 以下）
     *  2) 反向（指令与 filt 异号）且 |filt| < 50 rpm：立即回退 */
#if (STARTUP_MODE == STARTUP_IF)
    /* I/F 模式：低速区（含过零反向）归开环管，在观测器还锁着的转速就交回去 */
    if (Startup_IfWantsFallback()) {
        Sup_SwitchSpinStartup();
    } else
#endif
    if (MLIB_Mul_Q15(gSpd.cmd, gMeas.speed) > 0) {
        if (++s_u16SpinCheckTime >= gStartup.detTime) {
            s_u16SpinCheckTime = 0;
            if (spinTooSlow()) Sup_SwitchSpinStartup();
        }
    } else {
        if ((MLIB_Mul_Q15(gSpd.cmd, gMeas.speed) < 0) && (abs(gMeas.speed) < Q15(50.0 / SPEED_MAX)))
            Sup_SwitchSpinStartup();
    }

    /* 不用原厂 SMO 的高速角度补偿（磁链 PLL 没有滞后，test25/26） */

    Meas_SlowFilter();

    /* 原厂高级保护：堵转 / 反转失控 / 缺相 / 超速 */
    Protect_SpinSlow();
    if (gProt.bErr_Spin) {
        gProt.bErr_Spin = 0;
        Sup_SwitchSpinStartup();
    }
    if (gProt.bRotor_block) {
        gProt.u16BlockCounter = 0;
        Sup_RaiseFault(MC_FAULT_ROTOR_BLOCKED);
        if (gSup.faultRestarts < 10) gSup.faultRestarts++;
    }
    if (gProt.bPhaseless) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        if (gSup.faultRestarts < 10) gSup.faultRestarts++;
    }
    if (gProt.bSpeed_OC) {
        Sup_RaiseFault(MC_FAULT_SPEED_OC);
        if (gSup.faultRestarts < 10) gSup.faultRestarts++;
    }
}

/* ===========================================================================
 * Run 子状态切换
 * =========================================================================*/
void Sup_SwitchChargeCalib(void)
{
    DISABLE_PWMOUT();
    PWMABC_Dis;                     /* 飞车检测：上桥关、下桥开（短接） */
    PWMABCN_Enb;
    FocCore_ModulateZero();
    ENABLE_PWMOUT();

    Ctrl_ResetRunVariables();
    Startup_ResetFlyCheck();
    gStartup.preChargeCnt = 0;
    gSup.runState = RunState_Calib;
}

void Sup_SwitchCalibAlign(void)
{
    Startup_OnEnterAlign();         /* I/F：定位计时、角度清零；原厂模式空函数 */
    gSup.runState = RunState_Align;
    DISABLE_PWMOUT();
    PWMABC_Enb;
    PWMABCN_Enb;
    FocCore_ModulateZero();
    ENABLE_PWMOUT();
}

void Sup_SwitchCalibTorqueloop(void)
{
    /* 飞车：按测到的转速给初始转矩，按测到的角度接管 */
    gFoc.piQ.qdSum = gMeas.speed << 14;
    gFoc.idqRef.s16Q = abs(gMeas.speed);
    gFoc.theta = gFly.s16PosElEst_fly;
    gSup.runState = RunState_Torqueloop;

    DISABLE_PWMOUT();
    PWMABC_Enb;
    PWMABCN_Enb;
    FocCore_ModulateZero();
    ENABLE_PWMOUT();

    /* 飞车检测借用过 gEst.pll（MLIB_PLL 语义），这里换回磁链 PLL 的增益并清状态 */
    FluxPLL_Init(&gEst.pll);
}

void Sup_SwitchAlignStartup(void)
{
    Est_OnEnterStartupCold();
    Startup_OnEnterIF(0);           /* I/F：从定位角、零速开始拖；原厂模式空函数 */
    gSup.runState = RunState_Startup;
    SpeedCtrl_UseStartupRamp();
}

void Sup_SwitchStartupSpin(void)
{
    SpeedCtrl_OnEnterSpin();
    gProt.u16PhaselessTimeCnt = 0;
    gSup.runState = RunState_Spin;
    s_u16SpinCheckTime = 0;
    /* 先切状态再请求解耦：快环在 Spin 的第一拍里搬前馈量。反过来的话，请求可能在
     * 还是 Startup 的那一拍执行，d 轴积分器随即被原厂启动预置覆盖，搬进去的量丢掉。 */
    FocCore_SetDecouple(1);
}

void Sup_SwitchTorqueloopSpin(void)
{
    SpeedCtrl_OnEnterSpin();
    gEst.thetaOffset = 0;
    gProt.u16PhaselessTimeCnt = 0;
    gSup.runState = RunState_Spin;
    FocCore_SetDecouple(1);
}

void Sup_SwitchSpinStartup(void)
{
    MtpaFw_Reset();         /* 回到原厂启动：id = 0 */
    FocCore_SetDecouple(0);
    Startup_OnEnterIF(1);           /* I/F：从当前估计角/转速接管开环拖动（必须在切状态之前）；原厂模式空函数 */
    gSup.runState = RunState_Startup;
    Est_OnEnterStartupFromSpin();       /* 转子还在转：上报转速从当前值开始 */
    gSpd.speedRampSt.s32State = gMeas.speed << 16;
    SpeedCtrl_UseStartupRamp();
}

/* ===========================================================================
 * 保护
 * =========================================================================*/
static void M1_FaultDetection(void)
{
    static uint16_t u16OvCntr, u16UvCntr;

    MC_FAULT_CLEAR_ALL(gSup.fault);

    if (gMeas.vbusAvg > gProt.ovpCmd) {
        if (u16OvCntr <= 1600) u16OvCntr++;          /* 0.1 s */
        else {
            MC_FAULT_SET(gSup.fault, MC_FAULT_U_DCBUS_OVER);
            DISABLE_PWMOUT();
        }
    } else {
        u16OvCntr = 0;
    }

    if (gMeas.vbusAvg < gProt.uvpCmd) {
        if (u16UvCntr <= 1600) u16UvCntr++;
        else {
            MC_FAULT_SET(gSup.fault, MC_FAULT_U_DCBUS_UNDER);
            DISABLE_PWMOUT();
        }
    } else {
        u16UvCntr = 0;
    }

    gSup.faultRecord |= gSup.fault;
}

/* 故障码闪灯（LED 驱动原厂就注释掉了，只保留计时） */
static void Fault_LED_Disp(void)
{
    static uint16_t u16FaultTimeCnt = 0;
    static uint16_t u16FaultCnt = 0;
    static uint16_t u16FalultNumber = 0;
    int i;

    if (gSup.fault) {
        for (i = 0; i < 15; i++) {
            u16FalultNumber = gSup.fault & (0x01 << i) ? i : u16FalultNumber;
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
