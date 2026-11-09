/* ============================================================================
 * @file    supervisor.h
 * @brief   状态机 + 保护 + 初始化调度
 *
 * 主状态：Fault / Init / Stop / Run；Run 子状态：Charge / Calib / Align / Startup / Spin。
 *
 * 快环：HAL 的 ADC 中断每拍调用 Ctrl_FastTick()。
 * 慢环：PendSV 中断（快环每 16 拍挂起一次，最低优先级，可被快环抢占）中调用 Ctrl_SlowTick()。
 * ==========================================================================*/
#ifndef SUPERVISOR_H
#define SUPERVISOR_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

typedef void (*pfcn_void_void)(void);    /* 无参数无返回值的函数指针（用于状态处理函数表） */
typedef uint16_t mcdef_fault_t;    /* 故障位集合，bit n = MC_FAULT_x */

/* 主状态编号 */
typedef enum _m1_app_mainstate_t {
    MainState_Fault = 0,    /* 故障：PWM 关闭，故障消失 FAULTRELEASE_TIME 后回到 Stop */
    MainState_Init  = 1,    /* 上电后第一拍 */
    MainState_Stop  = 2,    /* 停机：PWM 关闭，等待速度指令 */
    MainState_Run   = 3,    /* 运行，具体阶段见 runState */
} m1_app_mainstate_t;

typedef enum _m1_run_substate_t {
    RunState_Charge     = 0,    /* 下桥导通为自举电容预充（PRECHARGE_TIME） */
    RunState_Calib      = 1,    /* 飞车检测：下桥短接检测转子是否在转（flystart.c） */
    RunState_Align      = 2,    /* 静止：IPD 测初始角，失败则定位（ipd.c / startup.c） */
    RunState_Startup    = 3,    /* I/F 开环拖动（startup.c） */
    RunState_Spin       = 4,    /* 闭环（spin.c），含零电流跟踪 */
} m1_run_substate_t;

#define MC_FAULT_I_DCBUS_OVER   1   /* 硬件过流 */
#define MC_FAULT_U_DCBUS_UNDER  2   /* 欠压 */
#define MC_FAULT_U_DCBUS_OVER   3   /* 过压 */
#define MC_FAULT_ROTOR_BLOCKED  4   /* 堵转 */
#define MC_FAULT_PHASELESS      5   /* 缺相 */
#define MC_FAULT_SPEED_OC       6   /* 超速 */
#define MC_FAULT_FLY_TIMEOUT    7   /* 飞车检测：短接制动超过 FLY_BRAKE_TIMEOUT_S 仍未停止 */
#define MC_FAULT_CFG            8   /* 配置检查未通过（cfg_check_list.h，行号见 gFluxDiag.cfgFailLine），不起动 */

/* 故障位操作：置位、全部清除 */
#define MC_FAULT_SET(faults, faultid)   (faults |= ((mcdef_fault_t)1 << faultid))
#define MC_FAULT_CLEAR_ALL(faults)      (faults = 0)

/* 连续重启（Sup_CountRestart）：计数达到 SUP_RESTART_LIMIT 后 Stop 不再自动起动，指令归零时清零；计数本身封顶 SUP_RESTART_COUNT_MAX */
#define SUP_RESTART_LIMIT       5
#define SUP_RESTART_COUNT_MAX   10

/* 状态机、速度指令、故障 */
typedef struct {
    m1_app_mainstate_t mainState;       /* 主状态 */
    m1_run_substate_t  runState;        /* Run 子状态 */
    volatile int16_t   speedCmd;        /* 转速指令 Q15 = rpm * 32768 / SPEED_MAX，由调试器或通信接口写入 */
    int16_t            fault;           /* 当前故障位（bit n = MC_FAULT_x） */
    int16_t            faultRecord;     /* 上电以来出现过的故障位 */
    uint8_t            faultRestarts;   /* 连续故障 / 起动失败次数；>= 5 时不再自动重启，指令归零后清除 */
    uint16_t           faultReleaseTicks;   /* 故障后自动恢复的等待时间（慢环拍） */
    uint32_t           ms;              /* 上电后的毫秒数（慢环拍），用于事件记录 */
} Supervisor_TypeDef;

extern Supervisor_TypeDef gSup;

/* ---- 事件记录：每次状态切换 / 故障记录一条，保留最近 SUP_LOG_N 条（循环覆盖） ----
 * gSupLog.count = 总条数，最新一条位于 e[(count-1) % SUP_LOG_N]；gSupLog.lastCode = 最近一条的事件码。
 * 编号 7、10 及 SUP_FB_x 中的 2、4 为保留值，不使用。 */
#define SUP_EV_START        1   /* Stop -> Run（收到指令） */
#define SUP_EV_STOP         2   /* Run -> Stop（指令归零） */
#define SUP_EV_FAULT        3   /* 进入 Fault，arg = 故障位 */
#define SUP_EV_STILL        4   /* 飞车检测：静止 -> 定位 */
#define SUP_EV_CATCH_SPIN   5   /* 飞车检测：直接接管进入 Spin，arg = 接管角 */
#define SUP_EV_CATCH_OPENLOOP     6   /* 飞车检测：直接进入 I/F，arg = 接管角 */
#define SUP_EV_ALIGN_DONE   8   /* 定位完成 -> Startup */
#define SUP_EV_HANDOVER     9   /* Startup -> Spin */
#define SUP_EV_FALLBACK     11  /* Spin -> Startup，arg = 原因（SUP_FB_x） */
#define SUP_FB_OPENLOOP           1   /* Startup_OpenLoopWantsFallback：低于回退转速且指令反向 / 过低 */
#define SUP_FB_REVERSE      3   /* 兜底：指令反向且已低于 OPENLOOP_SPIN_MIN_RPM 仍未回退 */
#define SUP_EV_LOST_LOCK    12  /* Spin 中失锁：指令方向未变，估计转速却降为 0 / 反号，或同向降到 SPIN_LOST_RPM 以下。关闭 PWM 回到 Stop，
                                 * 若指令仍存在则立即重新起动（重新进行飞车检测），arg = 当时的 gMeas.speed */
#define SUP_EV_TRACK        13  /* 软停机电流降到 0 后进入零电流跟踪（PWM 不关闭，观测器持续跟踪），arg = gMeas.speed */
#define SUP_EV_TRACK_END    14  /* 零电流跟踪结束：arg = 1 指令恢复、闭合速度环；0 = 转速低于结束转速，关闭 PWM 回到 Stop */
#define SUP_EV_IPD_DONE     15  /* IPD 完成 -> Startup，arg = N 极角（Q15 角度） */
#define SUP_LOG_N           16
typedef struct {
    uint32_t ms;            /* gSup.ms */
    uint16_t code;          /* SUP_EV_x */
    uint16_t arg;    /* 附加参数，含义见各 SUP_EV_x */
    uint8_t  main, run;     /* 切换前的主状态 / 子状态 */
    int16_t  speed;         /* gMeas.speed（Q15） */
    int16_t  pll;           /* PLL 转速 >> 16（Q15） */
    int16_t  cmd;           /* gSpd.cmd */
    int16_t  vbus;          /* gMeas.vbusAvg */
    int16_t  iq;            /* gFoc.idq.s16Q */
} SupEvent_TypeDef;
typedef struct {
    uint16_t count;    /* 累计记录条数（最新一条位于 e[(count-1) % SUP_LOG_N]） */
    uint16_t lastCode;    /* 最近一条的事件码 */
    SupEvent_TypeDef e[SUP_LOG_N];
} SupLog_TypeDef;
extern volatile SupLog_TypeDef gSupLog;
void Sup_Log(uint16_t code, uint16_t arg);

/* ---- 入口 ---- */
void Ctrl_Init(void);           /* 上电初始化全部模块 */
void Ctrl_ResetRunVariables(void); /* 停机 / 故障 / 重新起动前复位控制状态 */
void Ctrl_FastTick(void);       /* 快环：观测器输入快照 + 状态机快环 */
void Ctrl_SlowTick(void);       /* 慢环：状态机慢环 + 诊断 */

/* ---- 子状态切换（由 startup.c 等在条件满足时调用） ---- */
void Sup_SwitchChargeCalib(void);    /* 预充完成 -> 飞车检测（下桥短接） */
void Sup_SwitchCalibAlign(void);    /* 飞车检测判为静止 -> IPD / 定位 */
void Sup_SwitchCalibSpin(void);     /* 飞车直接接管进入 Spin */
void Sup_SwitchCalibOpenLoop(void);       /* 飞车低速，直接进入 I/F */
void Sup_SwitchAlignStartup(void);    /* 定位完成 -> I/F（从定位角、零速开始拖动） */
/* IPD 完成：从 theta（N 极角）冷起动 I/F，开环电流从 1/4 起上升 */
void Sup_SwitchIpdStartup(int16_t theta);
void Sup_SwitchStartupSpin(void);    /* I/F 与观测器转速一致 -> 闭环（转矩不变切换） */
void Sup_SwitchSpinStartup(void);    /* 闭环转速过低 / 正反转过零 -> 回退 I/F */
/* Run -> Stop（关闭 PWM 后调用：复位控制状态） */
void Sup_SwitchRunStop(void);
/* 置故障位并进入 Fault（关闭 PWM、复位控制状态） */
void Sup_RaiseFault(uint16_t faultId);
/* 连续重启计数 +1（上限 10）：会自动重启的故障 / 失锁各计一次，Stop 中达到 5 次后不再自动起动，指令归零时清零 */
void Sup_CountRestart(void);
void Sup_ClearRestarts(void);    /* 指令归零时清除连续重启计数 */

#endif
