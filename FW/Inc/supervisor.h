/* ============================================================================
 * supervisor.h —— 状态机 + 保护 + 初始化编排
 *
 * 主状态：Fault / Init / Stop / Run；Run 子状态：Charge / Calib / Align /
 * Startup / Spin / Torqueloop（编号与原厂相同，J-Scope 的 runState 照旧看）。
 *
 * 快环：hal 的 ADC 中断每拍调 Ctrl_FastTick()。
 * 慢环：main() 轮询，USER.bSlowLoopFlag 置位时调 Ctrl_SlowTick()。
 * ==========================================================================*/
#ifndef SUPERVISOR_H
#define SUPERVISOR_H
#include "fw_config.h"
#include "vendor_lib.h"

typedef void (*pfcn_void_void)(void);
typedef uint16_t mcdef_fault_t;

/* 编号不能改：闭源库 Fast_Boot_Current_Controller 用 eM1_RunSubState == 3 判 Startup */
typedef enum _m1_app_mainstate_t {
    MainState_Fault = 0,
    MainState_Init  = 1,
    MainState_Stop  = 2,
    MainState_Run   = 3,
} m1_app_mainstate_t;

typedef enum _m1_run_substate_t {
    RunState_Charge     = 0,
    RunState_Calib      = 1,
    RunState_Align      = 2,
    RunState_Startup    = 3,
    RunState_Spin       = 4,
    RunState_Torqueloop = 5,
} m1_run_substate_t;

#define MC_FAULT_I_DCBUS_OVER   1   /* 硬件过流 */
#define MC_FAULT_U_DCBUS_UNDER  2   /* 欠压 */
#define MC_FAULT_U_DCBUS_OVER   3   /* 过压 */
#define MC_FAULT_ROTOR_BLOCKED  4   /* 堵转 */
#define MC_FAULT_PHASELESS      5   /* 缺相 */
#define MC_FAULT_SPEED_OC       6   /* 超速 */

#define MC_FAULT_SET(faults, faultid)   (faults |= ((mcdef_fault_t)1 << faultid))
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((mcdef_fault_t)1 << faultid))
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((mcdef_fault_t)1 << faultid)) >> faultid)
#define MC_FAULT_CLEAR_ALL(faults)      (faults = 0)

extern m1_app_mainstate_t eM1_MainState;
extern m1_run_substate_t  eM1_RunSubState;
extern const pfcn_void_void s_STATE_FAST[4];
extern const pfcn_void_void s_STATE_SLOW[4];

/* ---- 入口 ---- */
void Ctrl_Init(void);           /* 上电初始化全部模块（原 Init_Parameter） */
void Ctrl_ResetRunVariables(void); /* 停机/故障/重新启动前复位控制状态（原 variable_reset） */
void Ctrl_FastTick(void);       /* 快环：观测器输入快照 + 状态机快环 */
void Ctrl_SlowTick(void);       /* 慢环：状态机慢环 + 诊断 */

/* ---- 子状态切换（startup.c 等在条件满足时调用） ---- */
void Sup_SwitchChargeCalib(void);
void Sup_SwitchCalibAlign(void);
void Sup_SwitchCalibTorqueloop(void);
void Sup_SwitchAlignStartup(void);
void Sup_SwitchStartupSpin(void);
void Sup_SwitchTorqueloopSpin(void);
void Sup_SwitchSpinStartup(void);
/* 置故障位并进 Fault（停 PWM、复位控制状态） */
void Sup_RaiseFault(uint16_t faultId);

#endif
