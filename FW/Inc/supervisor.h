/* ============================================================================
 * @file    supervisor.h
 * @brief   状态机 + 保护 + 初始化调度
 *
 * 主状态：Fault / Init / Stop / Run；Run 子状态：Charge / FlyCheck / Align / OpenLoop / Spin。
 *
 * 快环：HAL 的 ADC 中断每拍调用 Ctrl_FastTick()。
 * 慢环：PendSV 中断（快环每 16 拍挂起一次，最低优先级，可被快环抢占）中调用 Ctrl_SlowTick()。
 * ==========================================================================*/
#ifndef SUPERVISOR_H
#define SUPERVISOR_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

typedef void (*StateFn_t)(void);    /* 无参数无返回值的函数指针（用于状态处理函数表） */
typedef uint16_t FaultBits_t;    /* 故障位集合，bit n = MC_FAULT_x */

/* 主状态编号 */
typedef enum {
    MainState_Fault = 0,    /* 故障：PWM 关闭，故障消失 FAULTRELEASE_TIME 后回到 Stop */
    MainState_Init  = 1,    /* 上电后第一拍 */
    MainState_Stop  = 2,    /* 停机：PWM 关闭，等待速度指令 */
    MainState_Run   = 3,    /* 运行，具体阶段见 runState */
} MainState_TypeDef;

typedef enum {
    RunState_Charge     = 0,    /* 下桥导通为自举电容预充（PRECHARGE_TIME） */
    RunState_FlyCheck      = 1,    /* 飞车检测：下桥短接检测转子是否在转（flystart.c） */
    RunState_Align      = 2,    /* 静止：IPD 测初始角（失败则退回摆动起动）/ 摆动起动 / 辨识定位（ipd.c / startup.c） */
    RunState_OpenLoop    = 3,    /* I/F 开环拖动（openloop.c） */
    RunState_Spin       = 4,    /* 闭环（spin.c），含零电流跟踪 */
} RunState_TypeDef;

/* 故障位（bit n = MC_FAULT_x）分两类：
 *   电平型：条件存在期间一直成立——配置检查失败（CFG）、母线过压 / 欠压。每快环拍重算到 gSup.faultActive。
 *   事件型：发生一次即报——硬件过流、堵转、缺相、超速、飞车超时。由 Sup_RaiseFault（或过流锁存）置入 gSup.fault。
 * gSup.fault = 本次 Fault 的故障位：进入 Fault 时两类都锁存，Fault 中电平型全部消失 FAULTRELEASE_TIME 后清零、回到 Stop
 * （之后由 Stop 按连续重启计数决定是否自动起动）。faultRecord 累计上电以来出现过的全部故障位。 */
#define MC_FAULT_I_DCBUS_OVER   1   /* 硬件过流 */
#define MC_FAULT_U_DCBUS_UNDER  2   /* 欠压 */
#define MC_FAULT_U_DCBUS_OVER   3   /* 过压 */
#define MC_FAULT_ROTOR_BLOCKED  4   /* 堵转 */
#define MC_FAULT_PHASELESS      5   /* 缺相 */
#define MC_FAULT_SPEED_OC       6   /* 超速 */
#define MC_FAULT_FLY_TIMEOUT    7   /* 飞车检测：短接制动超过 FLY_BRAKE_TIMEOUT_S 仍未停止 */
#define MC_FAULT_CFG            8   /* 配置检查未通过（cfg_check_list.h，行号见 gFluxDiag.cfgFailLine），不起动 */

/* 置故障位 */
#define MC_FAULT_SET(faults, faultid)   (faults |= ((FaultBits_t)1 << faultid))

/* 连续重启（Sup_CountRestart）：计数达到 SUP_RESTART_LIMIT 后 Stop 不再自动起动，指令归零时清零；计数本身封顶 SUP_RESTART_COUNT_MAX */
#define SUP_RESTART_LIMIT       5
#define SUP_RESTART_COUNT_MAX   10

/* 状态机、速度指令、故障 */
typedef struct {
    MainState_TypeDef mainState;       /* 主状态 */
    RunState_TypeDef  runState;        /* Run 子状态 */
    volatile int16_t   speedCmd;        /* 转速指令 Q15 = rpm * 32768 / SPEED_MAX，由调试器或通信接口写入 */
    volatile uint8_t   stopMode;        /* 停机方式 STOP_MODE_RAMP / STOP_MODE_COAST（上电默认 STOP_MODE，调试器或上位机可改；其他值按滑行处理） */
    FaultBits_t      fault;           /* 本次 Fault 的故障位（进入 Fault 时锁存，回到 Stop 时清零） */
    FaultBits_t      faultActive;     /* 当前存在的电平型故障（每快环拍重算） */
    FaultBits_t      faultRecord;     /* 上电以来出现过的故障位 */
    uint8_t            faultRestarts;   /* 连续故障 / 起动失败次数；>= 5 时不再自动重启，指令归零后清除 */
    uint16_t           faultReleaseTicks;   /* 故障后自动恢复的等待时间（慢环拍） */
    volatile uint32_t  ms;              /* 上电以来的毫秒计数（慢环 1 kHz 递增，49.7 天回绕）。飞车检测用它判断停机已多久 */
} Supervisor_TypeDef;

extern Supervisor_TypeDef gSup;

/* ---- 入口 ---- */
void Ctrl_Init(void);           /* 上电初始化全部模块 */
void Ctrl_FastTick(void);       /* 快环：观测器输入快照 + 状态机快环 */
void Ctrl_SlowTick(void);       /* 慢环：状态机慢环 */

/* ---- 子状态切换（由 startup.c 等在条件满足时调用） ---- */
void Sup_SwitchChargeFlyCheck(void);    /* 预充完成 -> 飞车检测（下桥短接） */
void Sup_SwitchFlyCheckAlign(void);    /* 飞车检测判为静止 -> IPD / 摆动起动（参数辨识：定位） */
void Sup_SwitchFlyCheckSpin(void);     /* 飞车直接接管进入 Spin */
void Sup_SwitchFlyCheckOpenLoop(void);       /* 飞车低速，直接进入 I/F */
void Sup_SwitchSwingOpenLoop(int16_t theta, int16_t speedQ15);
void Sup_SwitchSwingSpin(int16_t theta, int16_t speedQ15, int16_t alignTheta);    /* 摆动中起动直接闭环（STARTUP_DIRECT_CL） */    /* 摆动中起动：转子正以 speedQ15 经过 theta，I/F 从该角度和转速接入 */
/* IPD 完成：从 theta（N 极角）冷起动 I/F，开环电流从 1/4 起上升 */
void Sup_SwitchIpdOpenLoop(int16_t theta);
void Sup_SwitchOpenLoopSpin(void);    /* I/F 与观测器转速一致 -> 闭环（转矩不变切换） */
void Sup_SwitchSpinOpenLoop(void);    /* 闭环转速过低 / 正反转过零 -> 回退 I/F */
/* Run -> Stop（关闭 PWM 后调用：复位控制状态） */
void Sup_SwitchRunStop(void);
/* 置故障位并进入 Fault（关闭 PWM、复位控制状态） */
void Sup_RaiseFault(uint16_t faultId);
/* 连续重启计数 +1（上限 10）：会自动重启的故障 / 失锁各计一次，Stop 中达到 5 次后不再自动起动，指令归零时清零 */
void Sup_CountRestart(void);
void Sup_ClearRestarts(void);    /* 指令归零时清除连续重启计数 */

#endif
