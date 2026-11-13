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
 *   Run 子状态 gSup.runState：Charge（自举电容预充）-> FlyCheck（飞车检测）-> Align（IPD / 定位）
 *     -> OpenLoop（I/F 开环拖动）-> Spin（闭环）。飞车检测可直接跳到 OpenLoop 或 Spin；Spin 转速过低时回退到 OpenLoop。
 * 每个状态有一个快环函数（每 PWM 周期，ADC 中断）和一个慢环函数（1 kHz，PendSV），由下方函数表分发。
 * 状态切换函数统一命名为 Sup_Switch<源><目标>，切换所需操作（开关桥臂、预置积分器、切换角度来源）在其中一次完成。
 *
 * 函数一览：
 *   Ctrl_Init / Ctrl_ResetRunVariables   上电初始化 / 每次停机、故障时清除运行状态
 *   Ctrl_FastTick / Ctrl_SlowTick        快环 / 慢环入口（在 hal_sc32r602.c 的中断中调用）
 *   M1_<主状态>_Fast / _Slow             各主状态每拍的处理
 *   softStopSlow                         软停机：电流矢量按限定速率降到 0
 *   stopRampSlow                         受控减速停机：闭环中收到停机指令时先由速度环减速到 TRACK_END_RPM
 *   Sup_Switch*                          状态切换
 *   faultDetect                    每拍汇总故障位（硬件过流、配置检查失败、母线过欠压）
 * ==========================================================================*/
#include "supervisor.h"
#include "hal.h"
#include "foc_core.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "est.h"
#include "startup.h"
#include "openloop.h"
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
/* 状态切换的临界区 IRQ_LOCK / IRQ_UNLOCK 见 hal.h（例：若 Spin -> I/F 回退时先写入 I/F 电流而 runState 仍为 Spin，
 * 快环会按估计角将开环电流施加到 d 轴）。 */

static uint8_t s_softStop;                      /* 软停机进行中 */
/* 主状态处理函数：快环、慢环各一组，下标 = gSup.mainState（Fault = 0, Init = 1, Stop = 2, Run = 3） */
static void faultFast(void);
static void initFast(void);
static void stopFast(void);
static void runFast(void);
static void faultSlow(void);
static void initSlow(void);
static void stopSlow(void);
static void runSlow(void);
static void faultDetect(void);

static const StateFn_t s_mainFast[4] = {faultFast, initFast, stopFast, runFast};
static const StateFn_t s_mainSlow[4] = {faultSlow, initSlow, stopSlow, runSlow};

/* Run 子状态表，顺序 = RunState_TypeDef */
static const StateFn_t s_runFast[5] = {
    Startup_ChargeFast, Startup_FlyCheckFast, Startup_AlignFast,
    OpenLoop_Fast, Spin_Fast
};
static const StateFn_t s_runSlow[5] = {
    Startup_ChargeSlow, Startup_FlyCheckSlow, Startup_AlignSlow,
    OpenLoop_Slow, Spin_Slow
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
    gSup.stopMode = STOP_MODE;
    Param_Init();           /* 运行时电机参数（配置标称值），以下各模块 Init 读取 gGain */

    Meas_Init();

    SpeedCtrl_Init();
    initProtection();
    Startup_Init();
    OpenLoop_Init();
    FlyStart_Init();
    FocCore_Init();
    FluxPLL_Init(&gEst.pll);
    Diag_Init();
    FluxObs_Init();
    IdRef_Init();
}

/* 清除运行状态：每次停机、故障、从 Stop 进入 Run（Charge -> FlyCheck）时调用。
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
    s_mainFast[gSup.mainState]();
    if (gSup.mainState != MainState_Run) FluxObs_Reset();
}

/* 观测器去直流相位补偿所用转速：Spin 使用 PLL 转速；I/F 拖动使用开环转速（转子同步）；
 * 其余状态（定位、飞车检测、停机）不补偿。 */
static int16_t compSpeed(void)
{
    if (gSup.mainState != MainState_Run) return 0;
    if (gSup.runState == RunState_Spin) return (int16_t)(gEst.pll.s32Speed >> 16);
    if (gSup.runState == RunState_OpenLoop) return OpenLoop_SpeedQ15();
    return 0;
}

/* 慢环入口：在 PendSV 中断中调用，由快环每 16 拍挂起一次（1 kHz），优先级低于快环（可被快环打断）。
 * 按主状态分发到 M1_<状态>_Slow，然后更新观测器慢变量（相位补偿、死区幅值）与母线过欠压判定。 */
void Ctrl_SlowTick(void)
{
    gSup.ms++;                          /* 1 kHz 毫秒时基（Stop 状态也在走）：飞车检测判断停机时长 */
    s_mainSlow[gSup.mainState]();
    FluxObs_SlowTick(compSpeed(), gMeas.vbusAvg);
    Protect_BusSlow();                  /* 判定结果由快环 faultDetect 读取 */
}

/* ===========================================================================
 * 主状态
 * =========================================================================*/
/* Run -> Fault：关闭全部桥臂、记录事件、清除运行状态。全过程关中断（可能在慢环中调用，不能被快环在中间状态打断） */
static void switchRunFault(void)
{
    IRQ_LOCK();
    HAL_PwmAllOff();
    if (gSup.runState >= RunState_OpenLoop) FlyStart_NoteRunEnd();     /* 转子可能仍在滑行 */
    gSup.mainState = MainState_Fault;
    gSup.runState = RunState_Charge;
    HAL_PwmHighOff();
    HAL_PwmLowOff();
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
    IRQ_UNLOCK();
}

/* 连续重启计数（飞车超时等可恢复故障时加 1，最多 10）。stopSlow 中计数 >= 5 时不再自动起动，
 * 指令归零才清零：防止"故障 -> 重启 -> 故障"无限循环 */
void Sup_CountRestart(void)
{
    if (gSup.faultRestarts < SUP_RESTART_COUNT_MAX) gSup.faultRestarts++;
}

/* 指令归零：清除连续重启计数（Stop 状态、零电流跟踪中） */
void Sup_ClearRestarts(void)
{
    gSup.faultRestarts = 0;
}

/* 报故障：置故障位（gSup.fault，同时累计到 faultRecord 便于追溯），立即切换到 Fault（关闭 PWM） */
void Sup_RaiseFault(uint16_t faultId)
{
    MC_FAULT_SET(gSup.fault, faultId);
    gSup.faultRecord |= gSup.fault;
    switchRunFault();
}

/* Run -> Stop：正常停机（软停机完成、转速过低结束跟踪，或指令归零）。关闭桥臂、清除运行状态。调用方已关闭 PWM 输出 */
void Sup_SwitchRunStop(void)
{
    IRQ_LOCK();
    if (gSup.runState >= RunState_OpenLoop) FlyStart_NoteRunEnd();     /* 转子可能仍在滑行 */
    gSup.mainState = MainState_Stop;
    gSup.runState = RunState_Charge;
    HAL_PwmHighOff();
    HAL_PwmLowOff();
    Ctrl_ResetRunVariables();
    HAL_FgSetDuty(0);
    IRQ_UNLOCK();
}

/* Stop -> Run：收到起动指令。从 Charge（预充）开始；停机期间的硬件过流锁存不带入本次运行 */
static void switchStopRun(void)
{
    HAL_ClearOcLatch();             /* 停机期间的过流标志不带入新的运行 */
    gSup.mainState = MainState_Run;
    gSup.runState = RunState_Charge;
    HAL_FgSetDuty(0);
}

/* Fault 快环：保持 PWM 关闭，持续检测故障（故障消失后由慢环计时回到 Stop） */
static void faultFast(void)
{
    HAL_PwmAllOff();
    faultDetect();
}

/* Init 快环：上电后第一拍，清除状态后直接进入 Stop */
static void initFast(void)
{
    HAL_PwmAllOff();
    Ctrl_ResetRunVariables();
    gSup.mainState = MainState_Stop;    /* Init -> Stop */
}

/* Stop 快环：PWM 关闭，比较值设为半占空比（下次开启 PWM 时三相电压平衡）；母线平均值直接跟随采样
 * （Stop 时不滤波，保证起动时刻母线归一化准确）；检测故障 */
static void stopFast(void)
{
    HAL_PwmAllOff();
    HAL_PwmSetHalfDuty();
    Meas_BusFollowSample();
    faultDetect();
    if (gSup.fault) gSup.mainState = MainState_Fault;
}

/* Run 快环：按子状态分发（Charge / FlyCheck / Align / OpenLoop / Spin 的 *_Fast），然后检测故障，有故障立即切换到 Fault */
RAMFUNC static void runFast(void)
{
    s_runFast[gSup.runState]();
    faultDetect();
    if (gSup.fault) switchRunFault();
}

/* Fault 慢环：照常滤波（母线电压等须始终有效）；电平型故障连续消失 FAULTRELEASE_TIME 后清除本次故障位、回到 Stop。
 * 只计连续无电平型故障的时间：出现即清零。事件型故障（已关 PWM）不阻止释放 */
static void faultSlow(void)
{
    static uint16_t s_releaseTicks;

    Meas_SlowFilter();
    if (gSup.faultActive) {
        s_releaseTicks = 0;
    } else if (++s_releaseTicks > gSup.faultReleaseTicks) {
        gSup.fault = 0;
        gSup.mainState = MainState_Stop;
        s_releaseTicks = 0;
    }
}

/* Init 慢环：仅做测量滤波 */
static void initSlow(void)
{
    Meas_SlowFilter();
}

/* Stop 慢环：读取速度指令（gSup.speedCmd -> gSpd.cmd）；指令超过 STOP_TO_RUN_SPEED 且连续重启次数未超限时起动；
 * 指令回到门限以下时清除重启计数 */
static void stopSlow(void)
{
    gSpd.cmd = gSup.speedCmd;
    /* 存在待装入的辨识参数（gParamStat.pending）时暂不起动：主循环 Param_Background 仅在 Stop 中装入，装入后再起动 */
    if (abs(gSpd.cmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && (gSup.faultRestarts < SUP_RESTART_LIMIT) && !gParamStat.pending) switchStopRun();
    else if (abs(gSpd.cmd) < Q15(STOP_TO_RUN_SPEED / SPEED_MAX)) Sup_ClearRestarts();
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

/* Run 中的停机处理（有电流的子状态：定位、I/F、闭环）：软停机把电流降到 0 后，闭环且转速够高 -> 零电流跟踪，
 * 否则关 PWM 回 Stop。软停开始后只有闭环中指令恢复才取消（从当前电流和转速继续闭环），定位 / I/F 中一律做完再停。
 * 返回 1 = 本拍由软停机处理（调用方不再执行子状态慢环） */
static uint8_t runStopSlow(uint8_t stopReq)
{
    uint8_t live = (uint8_t)(gSup.runState == RunState_Spin || gSup.runState == RunState_OpenLoop || gSup.runState == RunState_Align);
    if (s_softStop && !stopReq && gSup.runState == RunState_Spin) {
        s_softStop = 0;
        {
            IRQ_LOCK();
            SpeedCtrl_OnEnterSpin(gFoc.idqRef.s16D, gFoc.idqRef.s16Q);        /* 斜坡从当前转速、积分器从当前 iq 继续 */
            IRQ_UNLOCK();
        }
    }
    if (!(live && (stopReq || s_softStop))) {
        s_softStop = 0;
        return 0;
    }
    if (!s_softStop) SpeedCtrl_BlendStop();     /* 软停期间不执行速度环，未完成的 I/F 切换融合作废 */
    s_softStop = 1;
    if (softStopSlow()) {
        s_softStop = 0;
        if (gSup.runState == RunState_Spin && abs(gMeas.speed) >= TRACK_END_Q15) {
            /* 闭环中停机：不关 PWM，进入零电流跟踪（保持转子状态，滑行中再给指令无需飞车检测） */
            Spin_TrackBegin();
        } else {
            HAL_PwmAllOff();
            Sup_SwitchRunStop();
        }
    }
    Meas_SlowFilter();
    return 1;
}

/* 受控减速停机（STOP_MODE_RAMP）：闭环运行中收到停机指令时，不立即软停，而是把速度环的目标改为
 * "同向的 TRACK_END_RPM"，按 SPEED_RAMP_DOWN 减速（与平时同向调速完全相同的路径：S 形斜坡、轨迹前馈、
 * 母线电压控制器限制回馈）。斜坡到达 TRACK_END_RPM 后返回 0，由调用方按原流程软停、关闭 PWM。
 * 目标取同向的 TRACK_END_RPM 而不是 0：闭环（观测器）只在该转速以上可靠，更低的转速不做闭环制动。
 * 返回 1 = 本拍继续闭环减速（gSpd.cmd 已改写为减速目标）。
 * 不适用的情况（返回 0，按原流程软停）：停机方式为滑行、运行辨识进行中（依赖零电流滑行）、
 * 非闭环子状态（定位、I/F 电流沿开环角，直接软停）、软停已开始、斜坡已不高于 TRACK_END_RPM。 */
static uint8_t stopRampSlow(void)
{
    if ((IDENT_TEST && Ident_RunActive()) || gSup.stopMode != STOP_MODE_RAMP) return 0;
    if (gSup.runState != RunState_Spin || s_softStop) return 0;
    if (abs(gSpd.ramp) <= TRACK_END_Q15) return 0;
    gSpd.cmd = (gSpd.ramp > 0) ? TRACK_END_Q15 : -TRACK_END_Q15;
    return 1;
}

/* Run 慢环，按优先级：
 *   1. 参数辨识模式（IDENT_TEST）的慢环钩子；
 *   2. 零电流跟踪中（闭环中停机后转子仍在转动）：只执行 Spin_TrackSlow；
 *   3. 停机指令（|指令| < RUN_TO_STOP_SPEED）或软停机进行中：闭环且停机方式为受控减速时先减速到 TRACK_END_RPM（stopRampSlow）；
 *      有电流的子状态软停（电流降到 0），
 *      完成后若处于闭环且转速足够 -> 零电流跟踪，否则关闭 PWM 回到 Stop。闭环软停期间指令恢复 -> 取消软停继续闭环；
 *   4. 正常情况：按子状态分发慢环函数（预充计时、飞车判定、定位、I/F、速度环）。 */
static void runSlow(void)
{
    uint8_t stopReq;
    gSpd.cmd = IDENT_TEST ? Ident_SpeedCmd(gSup.speedCmd) : gSup.speedCmd;
    stopReq = (uint8_t)(abs(gSpd.cmd) < Q15(RUN_TO_STOP_SPEED / SPEED_MAX));
    if (IDENT_TEST && Ident_TakeRestart()) {    /* 自动辨识：静止测量完成，停机（参数在 Stop 中装入）后自动重新起动执行运行测量 */
        HAL_PwmAllOff();
        Sup_SwitchRunStop();
        return;
    }
    if (IDENT_TEST) Ident_RunSlow((uint8_t)(gSup.runState == RunState_Spin && !s_softStop), Spin_Tracking(), gSpd.cmd, gSpd.ramp);
    if (Spin_Tracking()) { Spin_TrackSlow(); return; }
    if (stopReq && stopRampSlow()) stopReq = 0;     /* 受控减速停机：本拍照常执行速度环，目标 = 同向 TRACK_END_RPM */
    if (runStopSlow(stopReq)) return;
    if (stopReq) {                      /* 无电流的子状态（预充、飞车检测）直接停机 */
        HAL_PwmAllOff();
        Sup_SwitchRunStop();
        return;
    }
    s_runSlow[gSup.runState]();
}

/* ===========================================================================
 * Run 子状态切换
 * =========================================================================*/
/* 打开上下桥互补输出，本周期输出零矢量（下一拍起由电流环 / 开环接手）：先全关、使能上下桥、写零矢量比较值，再开输出 */
static void bridgeOnZeroVector(void)
{
    HAL_PwmAllOff();
    HAL_PwmHighOn();
    HAL_PwmLowOn();
    FocCore_ModulateZero();
    HAL_PwmAllOn();
}

/* Charge -> FlyCheck（预充完成，开始飞车检测）：上桥关闭、下桥导通（三相短接），比较值为零矢量。
 * 清除运行状态与飞车检测状态 */
void Sup_SwitchChargeFlyCheck(void)
{
    HAL_PwmAllOff();
    HAL_PwmHighOff();                     /* 飞车检测：上桥关闭、下桥导通（短接） */
    HAL_PwmLowOn();
    FocCore_ModulateZero();
    HAL_PwmAllOn();

    Ctrl_ResetRunVariables();
    FlyStart_Begin();
    gSup.runState = RunState_FlyCheck;
}

/* FlyCheck -> Align（飞车检测判为静止）：开启上下桥互补输出，进入 IPD / 定位（由 Startup_OnEnterAlign 决定先做 IPD 还是直接定位） */
void Sup_SwitchFlyCheckAlign(void)
{
    Startup_OnEnterAlign();         /* 定位计时、角度清零 */
    gSup.runState = RunState_Align;
    bridgeOnZeroVector();
}

/* FlyCheck -> Spin（飞车检测：转子转速足够，直接接入闭环）。在快环中调用（常数时间赋值）：
 *   用测得的角度 / 转速预置 PLL、观测器（Est_OnCatch），电流指令置 0、q 轴积分器预置为反电势，
 *   开启 PWM（本周期零矢量），进入零电流跟踪（Spin_OnEnterCatch），PLL 收敛后再闭合速度环。 */
void Sup_SwitchFlyCheckSpin(void)
{
    int16_t spd = gFly.speedCatch;
    Est_OnCatch(gFly.thetaCatch, spd);
    /* 电流环：指令为 0；q 轴积分器预置为反电势电压，d 轴为 0（id = iq = 0 时的稳态电压）。
     * 短接时的短路电流由电流环在几毫秒内拉到 0。打开解耦时 SetDecouple 会将该前馈量
     * 移出积分器，总输出不变。 */
    FocCore_SetIdqRef(0, 0);
    FocCore_PresetPIForSpeed(spd);

    bridgeOnZeroVector();         /* 本周期仍为零矢量（= 短接），下一拍由电流环接手 */

    SpeedCtrl_BlendStop();
    Spin_OnEnterCatch();            /* 先跟踪后控制：零电流跟踪，估计器收敛后再闭合速度环；速度环由零电流跟踪的慢环跟随转速 */
    gSup.runState = RunState_Spin;
    FocCore_SetDecouple(1);
}

/* FlyCheck -> OpenLoop（飞车检测：转子在转但低于直接闭环门限，接入 I/F 开环）。在快环中调用：
 *   预置 PLL / 观测器，开环角与开环转速从测得值开始，电流沿转子 d 轴（转矩从 0 开始），随后按 I/F 正常拖动 */
void Sup_SwitchFlyCheckOpenLoop(void)
{
    int16_t spd = gFly.speedCatch;
    Est_OnCatch(gFly.thetaCatch, spd);
    /* 开环电流沿转子 d 轴（iq = 0 -> 开环角 = 转子角，转矩从 0 开始），q 轴积分器预置为反电势 */
    FocCore_SetIdqRef(gFoc.idqRef.s16D, 0);
    FocCore_PresetPIForSpeed(spd);
    OpenLoop_CommitCatch(spd);     /* 从 PLL 角（= 接管角外推一拍）和转速开始拖动；入口参数已在上电时计算，常数时间 */

    bridgeOnZeroVector();

    gSup.runState = RunState_OpenLoop;
    SpeedCtrl_PresetRamp(gMeas.speed);
    SpeedCtrl_UseOpenLoopRamp();
}

/* 慢环中的切换：耗时计算在临界区外"准备"（OpenLoop_Prepare，只计算、不修改状态），
 * 临界区内只"提交"（赋值 + 切换状态），以缩短关中断时间。 */
void Sup_SwitchAlignOpenLoop(void)
{
    OpenLoopEntry_TypeDef e;
    OpenLoop_PrepareCold(&e);
    e.olCur = e.olCurTarget;           /* 定位后电流已在开环方向上：从额定开环电流开始 */
    e.accStep = e.accStepFull;
    {
        IRQ_LOCK();
        Est_OnEnterOpenLoopCold();
        OpenLoop_Commit(&e);           /* I/F：从定位角、零速开始拖动 */
        gSup.runState = RunState_OpenLoop;
        SpeedCtrl_UseOpenLoopRamp();
        IRQ_UNLOCK();
    }
}

/* 直接拖动（STARTUP_MODE_DRAG）-> I/F：转子位置未知，从 0° 以牵入转速 f0 开始（openloop.c） */
void Sup_SwitchDragOpenLoop(void)
{
    OpenLoopEntry_TypeDef e;
    OpenLoop_PrepareDrag(&e);
    {
        IRQ_LOCK();
        gFoc.theta = 0;
        gFoc.thetaInv = 0;
        FocCore_PresetPI(0, 0);
        Est_OnEnterOpenLoopCold();       /* 观测器初始化为 |psi_f|∠0（转子位置未知，转动后收敛） */
        OpenLoop_Commit(&e);           /* I/F：先以 f0 匀速牵入（电压型驱动），之后加速 */
        gSup.runState = RunState_OpenLoop;
        SpeedCtrl_UseOpenLoopRamp();
        IRQ_UNLOCK();
    }
}

/* 摆动中起动（STARTUP_MODE_SWING）-> I/F：转子正以 speedQ15 经过 theta（Q15 角度），开环角、开环转速从转子的角度和转速开始。
 * 电流环积分器保持（电流矢量连续），观测器初始化为 |psi_f|∠theta */
void Sup_SwitchSwingOpenLoop(int16_t theta, int16_t speedQ15)
{
    OpenLoopEntry_TypeDef e;
    OpenLoop_PrepareSwing(speedQ15, &e);
    {
        IRQ_LOCK();
        gFoc.theta = theta;
        gFoc.thetaInv = theta;
        Est_OnEnterOpenLoopCold();
        OpenLoop_Commit(&e);
        gSup.runState = RunState_OpenLoop;
        SpeedCtrl_UseOpenLoopRamp();
        IRQ_UNLOCK();
    }
}

/* Align（IPD 成功）-> OpenLoop：在慢环中调用，先在临界区外准备 I/F 入口参数，临界区内只赋值。
 * theta = IPD 测得的 N 极方向（Q15 角度），作为开环起始角 */
void Sup_SwitchIpdOpenLoop(int16_t theta)
{
    OpenLoopEntry_TypeDef e;
    OpenLoop_PrepareCold(&e);           /* 开环电流从 1/4 起按 CURRENT_SLEW_A_S 上升，加速度同比例 */
    {
        IRQ_LOCK();
        gFoc.theta = theta;
        gFoc.thetaInv = theta;
        FocCore_PresetPI(0, 0);         /* 电流环已旁路一轮（直接输出电压），积分器从 0 开始 */
        Est_OnEnterOpenLoopCold();       /* 观测器初始化为 |psi_f|∠theta（N 极），PLL 角 = theta */
        OpenLoop_Commit(&e);
        gSup.runState = RunState_OpenLoop;
        SpeedCtrl_UseOpenLoopRamp();
        IRQ_UNLOCK();
    }
}

/* OpenLoop -> Spin（I/F 开环与观测器转速一致，切换闭环）：速度环从当前开环电流继续（转矩不变），
 * Spin 开始融合（I/F 的 d 轴电流渐变到 0，speed_ctrl.c Blend），打开 dq 解耦 */
void Sup_SwitchOpenLoopSpin(void)
{
    IRQ_LOCK();
    Spin_OnEnter();
    gSup.runState = RunState_Spin;
    /* 先切换状态再请求解耦：保证快环在 Spin 的第一拍转移前馈量。若顺序相反，
     * 请求可能在仍为 OpenLoop 的一拍中执行，转移的前馈量可能丢失。 */
    FocCore_SetDecouple(1);
    IRQ_UNLOCK();
}

/* Spin -> OpenLoop（回退：闭环转速低于 OPENLOOP_FALLBACK_RPM，或正反转需过零）：
 * 准备 I/F 入口（开环角延续 PLL 角、开环电流从当前 iq 延续、判断是否反转），临界区内关闭解耦、切换状态 */
void Sup_SwitchSpinOpenLoop(void)
{
    OpenLoopEntry_TypeDef e;
    /* 准备：输入均为慢环写入的量（iq 指令、滤波转速）或变化很慢的量（PLL 转速符号），可在临界区外读取 */
    OpenLoop_PrepareWarm(gFoc.idqRef.s16Q, OpenLoop_Reversal(gEst.pll.s32Speed), OpenLoop_Reversal(gMeas.speed), gMeas.speed, &e);
    {
        IRQ_LOCK();
        IdRef_Reset();         /* 回到起动模式：id = 0 */
        FocCore_SetDecouple(0);
        SpeedCtrl_BlendStop();
        OpenLoop_Commit(&e);           /* I/F：开环角 = 此刻的 PLL 角 + 预先计算的偏移 */
        gSup.runState = RunState_OpenLoop;
        SpeedCtrl_PresetRamp(gMeas.speed);
        SpeedCtrl_UseOpenLoopRamp();
        IRQ_UNLOCK();
    }
}

/* ===========================================================================
 * 保护
 * =========================================================================*/
/* 每拍（快环）故障检测：
 *   电平型（配置检查失败；母线过压 / 欠压，由 protect.c 慢环判定，越限持续 DCBUS_FILT_S 后已关 PWM）重算到 gSup.faultActive；
 *   硬件过流（事件型）：EPWM 刹车中断只置锁存（gHal.ocLatch），这里取走。若中断直接写故障位，
 *   PWM 已被中断关掉而状态机不知道；
 *   两类都并入 gSup.fault（Run 中据此切到 Fault），并累计到 faultRecord。缺相、超速等事件型故障由各子状态在慢环中报（Sup_RaiseFault） */
RAMFUNC static void faultDetect(void)
{
    FaultBits_t a = 0;
    if (gFluxDiag.cfgFailCount) MC_FAULT_SET(a, MC_FAULT_CFG);      /* 配置检查未通过：持续报故障，电机不起动（cfg_check.h） */
    {
        uint8_t b = gProt.busFlags;
        if (b & PROTECT_BUS_OV) MC_FAULT_SET(a, MC_FAULT_U_DCBUS_OVER);
        if (b & PROTECT_BUS_UV) MC_FAULT_SET(a, MC_FAULT_U_DCBUS_UNDER);
    }
    gSup.faultActive = a;
    gSup.fault |= a;
    if (HAL_TakeOcLatch()) MC_FAULT_SET(gSup.fault, MC_FAULT_I_DCBUS_OVER);
    gSup.faultRecord |= gSup.fault;
}
