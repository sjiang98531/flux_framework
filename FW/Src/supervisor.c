/* ============================================================================
 * @file    supervisor.c
 * @brief   状态机、子状态切换、保护、初始化调度
 *
 * 模块划分：起动相关子状态的处理函数在 startup.c，闭环 Spin（含零电流跟踪、失锁、回退融合）在 spin.c，
 * 速度环在 speed_ctrl.c，观测器 / PLL 调度在 est.c，母线过欠压在 protect.c。
 *
 * 状态机分两层：
 *   主状态 gSup.mainState：Init（上电一拍）-> Stop（PWM 关闭、等待指令）-> Run（运行）；任何时刻出现故障 -> Fault，
 *     故障消失 FAULTRELEASE_TIME 后回到 Stop。
 *   Run 子状态 gSup.runState：Charge（自举电容预充）-> Calib（飞车检测）-> Align（IPD / 定位）
 *     -> Startup（I/F 开环拖动）-> Spin（闭环）。飞车检测可直接跳到 Startup 或 Spin；Spin 转速过低时回退到 Startup。
 * 每个状态有一个快环函数（每 PWM 周期，ADC 中断）和一个慢环函数（1 kHz，PendSV），由下方函数表分发。
 * 状态切换函数统一命名为 Sup_Switch<源><目标>，切换所需操作（开关桥臂、预置积分器、切换角度来源）在其中一次完成。
 *
 * 函数一览：
 *   Ctrl_Init / Ctrl_ResetRunVariables   上电初始化 / 每次停机、故障时清除运行状态
 *   Ctrl_FastTick / Ctrl_SlowTick        快环 / 慢环入口（在 hal_sc32r602.c 的中断中调用）
 *   M1_<主状态>_Fast / _Slow             各主状态每拍的处理
 *   softStopSlow                         软停机：电流矢量按限定速率降到 0
 *   Sup_Switch*                          状态切换
 *   M1_FaultDetection                    每拍汇总故障位（硬件过流、配置检查失败、母线过欠压）
 *   Sup_Log                              事件记录（gSupLog，记录最近的状态变化）
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
#include "id_ref.h"
#include "meas.h"
#include "protect.h"
#include "flystart.h"
#include "spin.h"
#include "identify.h"
#include "param.h"
#include "qmath.h"
#include <stdlib.h>
#include "compiler.h"

Supervisor_TypeDef gSup = { MainState_Init, RunState_Charge };
volatile SupLog_TypeDef gSupLog;
/* 状态切换的临界区 IRQ_LOCK / IRQ_UNLOCK 见 hal.h（例：若 Spin -> I/F 回退时先写入 I/F 电流而 runState 仍为 Spin，
 * 快环会按估计角将开环电流施加到 d 轴）。 */

/* 事件记录：每次状态切换、故障、接管等记录一条（事件码 SUP_EV_*，arg 为附加参数，例如接管角），
 * 同时记录当时的时间、状态、转速、指令、母线、iq。环形缓冲 SUP_LOG_N 条，gSupLog.lastCode 为最近一条的事件码。
 * 出现异常时查看 gSupLog.e[] 即可追溯发生过程。 */
static uint8_t s_softStop;                      /* 软停机进行中 */
void Sup_Log(uint16_t code, uint16_t arg)
{
    /* 快环与慢环均会调用：关中断以保证单条记录完整 */
    volatile SupEvent_TypeDef *e;
    IRQ_LOCK();
    e = &gSupLog.e[gSupLog.count % SUP_LOG_N];
    e->ms = gSup.ms;
    e->code = code;
    e->arg = arg;
    e->main = (uint8_t)gSup.mainState;
    e->run = (uint8_t)gSup.runState;
    e->speed = gMeas.speed;
    e->pll = (int16_t)(gEst.pll.s32Speed >> 16);
    e->cmd = gSpd.cmd;
    e->vbus = gMeas.vbusAvg;
    e->iq = gFoc.idq.s16Q;
    gSupLog.lastCode = code;
    gSupLog.count++;
    IRQ_UNLOCK();
}

/* 主状态处理函数：快环、慢环各一组，下标 = gSup.mainState（Fault = 0, Init = 1, Stop = 2, Run = 3） */
static void M1_Fault_Fast(void);
static void M1_Init_Fast(void);
static void M1_Stop_Fast(void);
static void M1_Run_Fast(void);
static void M1_Fault_Slow(void);
static void M1_Init_Slow(void);
static void M1_Stop_Slow(void);
static void M1_Run_Slow(void);
static void M1_FaultDetection(void);

const pfcn_void_void s_STATE_FAST[4] = {M1_Fault_Fast, M1_Init_Fast, M1_Stop_Fast, M1_Run_Fast};
const pfcn_void_void s_STATE_SLOW[4] = {M1_Fault_Slow, M1_Init_Slow, M1_Stop_Slow, M1_Run_Slow};

/* Run 子状态表，顺序 = m1_run_substate_t */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_FAST[5] = {
    Startup_ChargeFast, Startup_CalibFast, Startup_AlignFast,
    Startup_StartupFast, Spin_Fast
};
static const pfcn_void_void s_M1_STATE_RUN_TABLE_SLOW[5] = {
    Startup_ChargeSlow, Startup_CalibSlow, Startup_AlignSlow,
    Startup_StartupSlow, Spin_Slow
};

/* ===========================================================================
 * 入口
 * =========================================================================*/
/* 保护相关初值：故障消失后回到 Stop 的等待时间（换算为慢环拍数）、连续重启计数清零、各保护模块初始化 */
static void initProtection(void)
{
    gSup.faultReleaseTicks = FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    gSup.faultRestarts = 0;
    Protect_Init();
}

/* 上电初始化（main.c 在开中断之前调用一次）。顺序要求：
 *   HAL 参数（PWM 周期、慢环分频）最先；运行时电机参数 Param_Init 须在各控制模块 Init 之前（各模块读取 gGain）；
 *   之后各模块按依赖顺序初始化。 */
void Ctrl_Init(void)
{
    HAL_InitParams();
    Param_Init();           /* 运行时电机参数（配置标称值），以下各模块 Init 读取 gGain */

    Meas_Init();

    SpeedCtrl_Init();
    initProtection();
    Startup_Init();
    FocCore_Init();
    FluxPLL_Init(&gEst.pll);
    Diag_Init();
    FluxObs_Init();
    IdRef_Init();
}

/* 清除运行状态：每次停机、故障、从 Stop 进入 Run（Charge -> Calib）时调用。
 * 积分器、斜坡、观测器、PLL、各计数器恢复初值，保证每次起动都从相同的初始状态开始 */
void Ctrl_ResetRunVariables(void)
{
    /* 各模块清除自身状态 */
    SpeedCtrl_ResetRun();
    FocCore_ResetRun();
    Meas_SpeedReset();
    Est_ResetRun();
    Startup_ResetRun();
    Protect_PhaseReset();
    IdRef_Reset();         /* MTPA/弱磁关闭时为空函数 */
    Spin_Reset();
    s_softStop = 0;
}

/* 快环入口：在 ADC 中断中每个 PWM 周期调用一次（16 kHz，62.5 us），此时采样已完成（gMeas.iabc / vbus）。
 * 按主状态分发到 M1_<状态>_Fast；非 Run 状态下观测器保持复位 */
RAMFUNC void Ctrl_FastTick(void)
{
    s_STATE_FAST[gSup.mainState]();
    if (gSup.mainState != MainState_Run) FluxObs_Reset();
}

/* 观测器去直流相位补偿所用转速：Spin 使用 PLL 转速；I/F 拖动使用开环转速（转子同步）；
 * 其余状态（定位、飞车检测、停机）不补偿。 */
static int16_t compSpeed(void)
{
    if (gSup.mainState != MainState_Run) return 0;
    if (gSup.runState == RunState_Spin) return (int16_t)(gEst.pll.s32Speed >> 16);
    if (gSup.runState == RunState_Startup) return Startup_OpenLoopSpeedQ15();
    return 0;
}

/* 慢环入口：在 PendSV 中断中调用，由快环每 16 拍挂起一次（1 kHz），优先级低于快环（可被快环打断）。
 * 先统计慢环间隔（slowGap，名义 16 拍）与毫秒时间戳，再按主状态分发到 M1_<状态>_Slow，
 * 然后更新观测器慢变量（相位补偿、死区幅值）、母线过欠压判定、置信度诊断，最后记录慢环时长（slowDur）。 */
void Ctrl_SlowTick(void)
{
    static uint32_t s_prev;
    uint32_t n0 = gFluxDiag.sampleCount;
    gFluxDiag.slowGap = (uint16_t)(n0 - s_prev);
    if (gFluxDiag.slowGap > gFluxDiag.slowGapMax) gFluxDiag.slowGapMax = gFluxDiag.slowGap;
    s_prev = n0;
    {   /* 毫秒时间戳（用于事件记录），慢环频率不是 1 kHz 时同样正确：每拍加 1000 / f ms，余数累计 */
        static uint16_t s_msFrac;
        s_msFrac += 1000u;
        while (s_msFrac >= (uint16_t)SLOWLOOP_FREQ) { s_msFrac -= (uint16_t)SLOWLOOP_FREQ; gSup.ms++; }
    }
    Diag_SlowBegin();
    s_STATE_SLOW[gSup.mainState]();
    FluxObs_SlowTick(compSpeed(), gMeas.vbusAvg);
    Protect_BusSlow();                  /* 判定结果由快环 M1_FaultDetection 读取 */
    gFluxDiag.mainState = (uint16_t)gSup.mainState;     /* 状态诊断（1 ms 分辨率，精确时刻见 gSupLog） */
    gFluxDiag.runState = (uint16_t)gSup.runState;
    Diag_SlowTick();
    gFluxDiag.slowDur = (uint16_t)(gFluxDiag.sampleCount - n0);
    if (gFluxDiag.slowDur > gFluxDiag.slowDurMax) gFluxDiag.slowDurMax = gFluxDiag.slowDur;
}

/* ===========================================================================
 * 主状态
 * =========================================================================*/
/* Run -> Fault：关闭全部桥臂、记录事件、清除运行状态。全过程关中断（可能在慢环中调用，不能被快环在中间状态打断） */
static void M1_SwitchRunFault(void)
{
    IRQ_LOCK();
    HAL_PwmAllOff();
    if (gSup.runState >= RunState_Startup) FlyStart_NoteRunEnd();     /* 转子可能仍在滑行 */
    Sup_Log(SUP_EV_FAULT, (uint16_t)gSup.fault);
    gSup.mainState = MainState_Fault;
    gSup.runState = RunState_Charge;
    HAL_PwmHighOff();
    HAL_PwmLowOff();
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
    IRQ_UNLOCK();
}

/* 连续重启计数（飞车超时等可恢复故障时加 1，最多 10）。M1_Stop_Slow 中计数 >= 5 时不再自动起动，
 * 指令归零才清零：防止"故障 -> 重启 -> 故障"无限循环 */
void Sup_CountRestart(void)
{
    if (gSup.faultRestarts < 10) gSup.faultRestarts++;
}

/* 报故障：置故障位（gSup.fault，同时累计到 faultRecord 便于追溯），立即切换到 Fault（关闭 PWM） */
void Sup_RaiseFault(uint16_t faultId)
{
    MC_FAULT_SET(gSup.fault, faultId);
    gSup.faultRecord |= gSup.fault;
    M1_SwitchRunFault();
}

/* Run -> Stop：正常停机（软停机完成、转速过低结束跟踪，或指令归零）。关闭桥臂、清除运行状态。调用方已关闭 PWM 输出 */
void Sup_SwitchRunStop(void)
{
    IRQ_LOCK();
    if (gSup.runState >= RunState_Startup) FlyStart_NoteRunEnd();     /* 转子可能仍在滑行 */
    Sup_Log(SUP_EV_STOP, 0);
    gSup.mainState = MainState_Stop;
    gSup.runState = RunState_Charge;
    HAL_PwmHighOff();
    HAL_PwmLowOff();
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
    IRQ_UNLOCK();
}

/* Stop -> Run：收到起动指令。从 Charge（预充）开始；停机期间的硬件过流锁存不带入本次运行 */
static void M1_SwitchStopRun(void)
{
    Sup_Log(SUP_EV_START, gSup.faultRestarts);
    gSup.hwOcLatch = 0;             /* 停机期间的过流标志不带入新的运行 */
    gSup.mainState = MainState_Run;
    gSup.runState = RunState_Charge;
    HAL_FgSetDuty(0);
}

/* Fault 快环：保持 PWM 关闭，持续检测故障（故障消失后由慢环计时回到 Stop） */
static void M1_Fault_Fast(void)
{
    HAL_PwmAllOff();
    M1_FaultDetection();
}

/* Init 快环：上电后第一拍，清除状态后直接进入 Stop */
static void M1_Init_Fast(void)
{
    HAL_PwmAllOff();
    Ctrl_ResetRunVariables();
    gSup.mainState = MainState_Stop;    /* Init -> Stop */
}

/* Stop 快环：PWM 关闭，比较值设为半占空比（下次开启 PWM 时三相电压平衡）；母线平均值直接跟随采样
 * （Stop 时不滤波，保证起动时刻母线归一化准确）；检测故障 */
static void M1_Stop_Fast(void)
{
    HAL_PwmAllOff();
    HAL_PwmSetHalfDuty();
    gMeas.vbusAvg = gMeas.vbus;
    M1_FaultDetection();
    if (gSup.fault) gSup.mainState = MainState_Fault;
}

/* Run 快环：按子状态分发（Charge / Calib / Align / Startup / Spin 的 *_Fast），然后检测故障，有故障立即切换到 Fault */
RAMFUNC static void M1_Run_Fast(void)
{
    s_M1_STATE_RUN_TABLE_FAST[gSup.runState]();
    M1_FaultDetection();
    if (gSup.fault) M1_SwitchRunFault();
}

/* Fault 慢环：照常滤波（母线电压等须始终有效）；故障连续消失 FAULTRELEASE_TIME 后回到 Stop。
 * 只计连续无故障的时间：故障出现即清零 */
static void M1_Fault_Slow(void)
{
    static uint16_t s_releaseTicks;

    Meas_SlowFilter();
    if (gSup.fault) {
        s_releaseTicks = 0;
    } else if (++s_releaseTicks > gSup.faultReleaseTicks) {
        gSup.mainState = MainState_Stop;
        s_releaseTicks = 0;
    }
}

/* Init 慢环：仅做测量滤波 */
static void M1_Init_Slow(void)
{
    Meas_SlowFilter();
}

/* Stop 慢环：读取速度指令（gSup.speedCmd -> gSpd.cmd）；指令超过 STOP_TO_RUN_SPEED 且连续重启次数未超限时起动；
 * 指令回到门限以下时清除重启计数 */
static void M1_Stop_Slow(void)
{
    gSpd.cmd = gSup.speedCmd;
    if (abs(gSpd.cmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && (gSup.faultRestarts < 5)) M1_SwitchStopRun();
    else if (abs(gSpd.cmd) < Q15(STOP_TO_RUN_SPEED / SPEED_MAX)) gSup.faultRestarts = 0;
    Meas_SlowFilter();
}

/* 软停机：收到停机指令后不直接关闭 PWM，而是先将电流矢量按 SOFT_STOP_STEP_Q15 的幅值变化率
 * 降到 0（方向不变），再关闭。若直接关闭，绕组电流会在几毫秒内经二极管泄放，转矩突变产生噪声。
 * 变化率与 S 形斜坡允许的最大转矩变化率相同（SPEED_RAMP_UP / b / SPEED_RAMP_ROUND_S）：起动、调速、停机
 * 全程统一限制转矩变化率。此期间不执行子状态慢环（速度环、I/F 切换），快环照常运行（电流环、观测器 / 开环角）。
 * 返回 1 = 电流已降到 0。 */
static uint8_t softStopSlow(void)
{
    int32_t d = gFoc.idqRef.s16D, q = gFoc.idqRef.s16Q;
    int32_t m = (int32_t)QMath_Isqrt32((uint32_t)(d * d + q * q));
    if (m <= SOFT_STOP_STEP_Q15) { FocCore_SetIdqRef(0, 0); return 1; }
    d = d * (m - SOFT_STOP_STEP_Q15) / m;
    q = q * (m - SOFT_STOP_STEP_Q15) / m;
    FocCore_SetIdqRef((int16_t)d, (int16_t)q);
    return 0;
}

/* Run 慢环，按优先级：
 *   1. 参数辨识模式（IDENT_TEST）的慢环钩子；
 *   2. 零电流跟踪中（闭环中停机后转子仍在转动）：只执行 Spin_TrackSlow；
 *   3. 停机指令（|指令| < RUN_TO_STOP_SPEED）或软停机进行中：有电流的子状态先软停（电流降到 0），
 *      完成后若处于闭环且转速足够 -> 零电流跟踪，否则关闭 PWM 回到 Stop。闭环软停期间指令恢复 -> 取消软停继续闭环；
 *   4. 正常情况：按子状态分发慢环函数（预充计时、飞车判定、定位、I/F、速度环）。 */
static void M1_Run_Slow(void)
{
    gSpd.cmd = gSup.speedCmd;
    if (IDENT_TEST) Ident_RunSlow((uint8_t)(gSup.runState == RunState_Spin && !s_softStop), Spin_Tracking(), gSpd.cmd, gSpd.ramp);
    if (Spin_Tracking()) { Spin_TrackSlow(); return; }
    {   /* 有电流的子状态（定位、I/F、闭环）执行软停机；软停机开始后，仅当闭环中指令恢复时才取消
         * （从当前电流和转速继续闭环），定位 / I/F 中一律完成后再停 */
        uint8_t live = (gSup.runState == RunState_Spin || gSup.runState == RunState_Startup || gSup.runState == RunState_Align);
        uint8_t req = (uint8_t)(abs(gSpd.cmd) < Q15(RUN_TO_STOP_SPEED / SPEED_MAX));
        if (s_softStop && !req && gSup.runState == RunState_Spin) {
            s_softStop = 0;
            IRQ_LOCK();
            SpeedCtrl_OnEnterSpin();        /* 斜坡从当前转速、积分器从当前 iq 继续 */
            IRQ_UNLOCK();
        }
        if (live && (req || s_softStop)) {
            if (!s_softStop) SpeedCtrl_BlendStop();     /* 软停期间不执行速度环，未完成的 I/F 切换融合作废 */
            s_softStop = 1;
            if (softStopSlow()) {
                s_softStop = 0;
                if (gSup.runState == RunState_Spin && abs(gMeas.speed) >= TRACK_END_Q15) {
                    /* 闭环中停机：不关闭 PWM，进入零电流跟踪（保持转子状态，滑行中重新给指令无需飞车检测） */
                    Sup_Log(SUP_EV_TRACK, (uint16_t)gMeas.speed);
                    Spin_TrackBegin();
                } else {
                    HAL_PwmAllOff();
                    Sup_SwitchRunStop();
                }
            }
            Meas_SlowFilter();
            return;
        }
        s_softStop = 0;
    }

    if (abs(gSpd.cmd) < Q15(RUN_TO_STOP_SPEED / SPEED_MAX)) {     /* 无电流的子状态（预充、飞车检测）直接停机 */
        HAL_PwmAllOff();
        Sup_SwitchRunStop();
        return;
    }
    s_M1_STATE_RUN_TABLE_SLOW[gSup.runState]();
}

/* ===========================================================================
 * Run 子状态切换
 * =========================================================================*/
/* Charge -> Calib（预充完成，开始飞车检测）：上桥关闭、下桥导通（三相短接），比较值为零矢量。
 * 清除运行状态与飞车检测状态 */
void Sup_SwitchChargeCalib(void)
{
    HAL_PwmAllOff();
    HAL_PwmHighOff();                     /* 飞车检测：上桥关闭、下桥导通（短接） */
    HAL_PwmLowOn();
    FocCore_ModulateZero();
    HAL_PwmAllOn();

    Ctrl_ResetRunVariables();
    Startup_ResetFlyCheck();
    gStartup.preChargeCnt = 0;
    gSup.runState = RunState_Calib;
}

/* Calib -> Align（飞车检测判为静止）：开启上下桥互补输出，进入 IPD / 定位（由 Startup_OnEnterAlign 决定先做 IPD 还是直接定位） */
void Sup_SwitchCalibAlign(void)
{
    Sup_Log(SUP_EV_STILL, 0);
    Startup_OnEnterAlign();         /* 定位计时、角度清零 */
    gSup.runState = RunState_Align;
    HAL_PwmAllOff();
    HAL_PwmHighOn();
    HAL_PwmLowOn();
    FocCore_ModulateZero();
    HAL_PwmAllOn();
}

/* Calib -> Spin（飞车检测：转子转速足够，直接接入闭环）。在快环中调用（常数时间赋值）：
 *   用测得的角度 / 转速预置 PLL、观测器（Est_OnCatch），电流指令置 0、q 轴积分器预置为反电势，
 *   开启 PWM（本周期零矢量），进入零电流跟踪（Spin_OnEnterCatch），PLL 收敛后再闭合速度环。 */
void Sup_SwitchCalibSpin(void)
{
    int16_t spd = gFly.speedCatch;
    Sup_Log(SUP_EV_CATCH_SPIN, (uint16_t)gFly.thetaCatch);
    Est_OnCatch(gFly.thetaCatch, spd);
    /* 电流环：指令为 0；q 轴积分器预置为反电势电压，d 轴为 0（id = iq = 0 时的稳态电压）。
     * 短接时的短路电流由电流环在几毫秒内拉到 0。打开解耦时 SetDecouple 会将该前馈量
     * 移出积分器，总输出不变。 */
    FocCore_SetIdqRef(0, 0);
    FocCore_PresetPIForSpeed(spd);

    HAL_PwmAllOff();
    HAL_PwmHighOn();
    HAL_PwmLowOn();
    FocCore_ModulateZero();         /* 本周期仍为零矢量（= 短接），下一拍由电流环接手 */
    HAL_PwmAllOn();

    SpeedCtrl_BlendStop();
    Spin_OnEnterCatch();            /* 先跟踪后控制：零电流跟踪，估计器收敛后再闭合速度环；速度环初始化推迟到下一个慢环周期 */
    gSup.runState = RunState_Spin;
    FocCore_SetDecouple(1);
}

/* Calib -> Startup（飞车检测：转子在转但低于直接闭环门限，接入 I/F 开环）。在快环中调用：
 *   预置 PLL / 观测器，开环角与开环转速从测得值开始，电流沿转子 d 轴（转矩从 0 开始），随后按 I/F 正常拖动 */
void Sup_SwitchCalibOpenLoop(void)
{
    int16_t spd = gFly.speedCatch;
    Sup_Log(SUP_EV_CATCH_OPENLOOP, (uint16_t)gFly.thetaCatch);
    Est_OnCatch(gFly.thetaCatch, spd);
    /* 开环电流沿转子 d 轴（iq = 0 -> 开环角 = 转子角，转矩从 0 开始），q 轴积分器预置为反电势 */
    FocCore_SetIdqRef(gFoc.idqRef.s16D, 0);
    FocCore_PresetPIForSpeed(spd);
    Startup_CommitCatchOpenLoop(spd);     /* 从 PLL 角（= 接管角外推一拍）和转速开始拖动；入口参数已在上电时计算，常数时间 */

    HAL_PwmAllOff();
    HAL_PwmHighOn();
    HAL_PwmLowOn();
    FocCore_ModulateZero();
    HAL_PwmAllOn();

    gSup.runState = RunState_Startup;
    SpeedCtrl_PresetRamp(gMeas.speed);
    SpeedCtrl_UseStartupRamp();
}

/* 慢环中的切换：耗时计算在临界区外"准备"（Startup_PrepareOpenLoop，只计算、不修改状态），
 * 临界区内只"提交"（赋值 + 切换状态），以缩短关中断时间。 */
void Sup_SwitchAlignStartup(void)
{
    OpenLoopEntry_TypeDef e;
    Startup_PrepareOpenLoop(0, 0, 0, 0, 0, &e);
    {
    IRQ_LOCK();
    Sup_Log(SUP_EV_ALIGN_DONE, 0);
    Est_OnEnterStartupCold();
    Startup_CommitOpenLoop(&e);           /* I/F：从定位角、零速开始拖动 */
    gSup.runState = RunState_Startup;
    SpeedCtrl_UseStartupRamp();
    IRQ_UNLOCK();
    }
}

/* Align（IPD 成功）-> Startup：在慢环中调用，先在临界区外准备 I/F 入口参数，临界区内只赋值。
 * theta = IPD 测得的 N 极方向（Q15 角度），作为开环起始角 */
void Sup_SwitchIpdStartup(int16_t theta)
{
    OpenLoopEntry_TypeDef e;
    Startup_PrepareOpenLoop(0, 0, 0, 0, 0, &e);
    /* 定位后电流本就在 I/F 方向上；IPD 后电流为 0，开环电流从 1/4 起按 CURRENT_SLEW_A_S 上升（与回退相同），
     * 加速度按电流比例给出，负载角余量不变；d 轴电流阶跃较小 */
    e.olCur = e.olCurTarget >> 2;
    e.accStep = (e.accStepFull * e.olCur) / e.olCurTarget;
    {
    IRQ_LOCK();
    Sup_Log(SUP_EV_IPD_DONE, (uint16_t)theta);
    gFoc.theta = theta;
    gFoc.thetaInv = theta;
    FocCore_PresetPI(0, 0);         /* 电流环已旁路一轮（直接输出电压），积分器从 0 开始 */
    Est_OnEnterStartupCold();       /* 观测器初始化为 |psi_f|∠theta（N 极），PLL 角 = theta */
    Startup_CommitOpenLoop(&e);
    gSup.runState = RunState_Startup;
    SpeedCtrl_UseStartupRamp();
    IRQ_UNLOCK();
    }
}

/* Startup -> Spin（I/F 开环与观测器转速一致，切换闭环）：速度环从当前开环电流继续（转矩不变），
 * Spin 开始融合（I/F 的 d 轴电流渐变到 0，speed_ctrl.c Blend），打开 dq 解耦 */
void Sup_SwitchStartupSpin(void)
{
    IRQ_LOCK();
    Sup_Log(SUP_EV_HANDOVER, 0);
    SpeedCtrl_OnEnterSpin();
    Spin_OnEnter();
    gSup.runState = RunState_Spin;
    /* 先切换状态再请求解耦：保证快环在 Spin 的第一拍转移前馈量。若顺序相反，
     * 请求可能在仍为 Startup 的一拍中执行，转移的前馈量可能丢失。 */
    FocCore_SetDecouple(1);
    IRQ_UNLOCK();
}

/* Spin -> Startup（回退：闭环转速低于 OPENLOOP_FALLBACK_RPM，或正反转需过零）：
 * 准备 I/F 入口（开环角延续 PLL 角、开环电流从当前 iq 延续、判断是否反转），临界区内关闭解耦、切换状态 */
void Sup_SwitchSpinStartup(void)
{
    OpenLoopEntry_TypeDef e;
    /* 准备：输入均为慢环写入的量（iq 指令、滤波转速）或变化很慢的量（PLL 转速符号），可在临界区外读取 */
    Startup_PrepareOpenLoop(1, gFoc.idqRef.s16Q, Startup_OpenLoopReversal(gEst.pll.s32Speed), Startup_OpenLoopReversal(gMeas.speed), gMeas.speed, &e);
    {
    IRQ_LOCK();
    IdRef_Reset();         /* 回到起动模式：id = 0 */
    FocCore_SetDecouple(0);
    SpeedCtrl_BlendStop();
    Startup_CommitOpenLoop(&e);           /* I/F：开环角 = 此刻的 PLL 角 + 预先计算的偏移 */
    gSup.runState = RunState_Startup;
    SpeedCtrl_PresetRamp(gMeas.speed);
    SpeedCtrl_UseStartupRamp();
    IRQ_UNLOCK();
    }
}

/* ===========================================================================
 * 保护
 * =========================================================================*/
/* 每拍（快环）汇总故障位：先清除，再按当前情况重新置位（故障消失后 gSup.fault 自动为 0，faultRecord 保留历史）。
 * 来源：硬件过流（EPWM 刹车中断置位的锁存）、配置检查失败、母线过压 / 欠压。缺相、超速等由 Spin 在慢环中报告 */
RAMFUNC static void M1_FaultDetection(void)
{
    MC_FAULT_CLEAR_ALL(gSup.fault);
    /* 硬件过流：中断中只置锁存，在此转换为故障位。若由中断直接写故障位，会被上一行的清除操作覆盖，
     * 导致 PWM 已被中断关闭而状态机仍认为在运行。 */
    if (gSup.hwOcLatch) {
        gSup.hwOcLatch = 0;
        MC_FAULT_SET(gSup.fault, MC_FAULT_I_DCBUS_OVER);
    }

    /* 配置检查未通过：持续报故障，电机不起动（cfg_check.h） */
    if (gFluxDiag.cfgFailCount) MC_FAULT_SET(gSup.fault, MC_FAULT_CFG);

    {   /* 母线过压 / 欠压（由 protect.c 慢环判定，越限持续 DCBUS_FILT_S 后已关闭 PWM） */
        uint8_t b = gProt.busFlags;
        if (b & PROTECT_BUS_OV) MC_FAULT_SET(gSup.fault, MC_FAULT_U_DCBUS_OVER);
        if (b & PROTECT_BUS_UV) MC_FAULT_SET(gSup.fault, MC_FAULT_U_DCBUS_UNDER);
    }

    gSup.faultRecord |= gSup.fault;
}
