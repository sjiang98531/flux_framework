/* ============================================================================
 * speed_ctrl.h —— 外环：速度斜坡 + 速度 PI + 加速度前馈（+ 母线电流/功率限制）
 *
 *   iq_cmd = SpeedPI(ramp - filt) + PowerPI(Ibus 限) + iq_ff
 *   iq_ff  = SPEED_ACC_FF_IQ（斜坡离开 0 的方向上），= 斜坡率 / MOTOR_ACC_RPM_S_PER_A
 *
 * 速度 PI 增益由 cfg_ctrl.h 的 SPEED_WN_HZ / SPEED_ZETA 和电机的
 * MOTOR_ACC_RPM_S_PER_A 自动算。CLOSE_LOOP 选 TORQUE/SPEED/POWER（原厂结构）。
 * ==========================================================================*/
#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H
#include "fw_config.h"
#include "vendor_lib.h"

extern int16_t s16PowerFdk;     /* 实测功率（Q15，POWER_MAX 基准），慢环更新 */

void SpeedCtrl_Init(void);
/* 每个 Run 慢环拍开头：更新功率反馈 */
void SpeedCtrl_UpdatePowerFdk(void);
/* Spin 慢环：斜坡 + PI + 前馈，写 FOC.sIdq_cmd.s16Q */
void SpeedCtrl_SpinSlow(void);
/* 进入 Spin（从 Startup 或 Torqueloop）：恢复运行斜坡率，按当前状态预置斜坡与 PI 积分器 */
void SpeedCtrl_OnEnterSpin(void);
/* 进入 Startup：速度斜坡改用 STARTUP_SPEED_RAMP */
void SpeedCtrl_UseStartupRamp(void);

#endif
