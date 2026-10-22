/* ============================================================================
 * startup.h —— 启动流程（原厂 Fast Boot）各子状态的处理函数
 *
 *   Charge 20 ms（自举预充）
 *   -> Calib 100 ms（MCF_FlyStart_2R 短接下桥测电流，判断转子是否在转）
 *        ├─ 在转 -> Torqueloop 0.3 s（按测到的角度/转速直接力矩闭环）-> Spin
 *        └─ 静止 -> Align 1 ms -> Startup（Iq 0.5 A，观测器+PLL 自由运行，
 *                   PLL 12.8 rpm 下限推着走）-> Fast_Boot_StartUp 判 80 rpm -> Spin
 *
 * 子状态之间的切换函数在 supervisor.c（Sup_Switch*），这里的处理函数在
 * 条件满足时调用它们，调用顺序与 test33 完全一致。
 *
 * 待扩展（P4 阶段）：
 *   IPD 初始位置检测 + I/f 拖动 + 磁链接管 —— 零速带载启动
 *   HFI 高频注入（需要凸极，IPM 才有意义）
 * ==========================================================================*/
#ifndef STARTUP_H
#define STARTUP_H
#include "fw_config.h"
#include "vendor_lib.h"

#if (FAST_BOOT_METHOD != 1)
#error "只支持原厂快速启动（FAST_BOOT_METHOD = 1）。原厂的 0 分支在 test33 里本来就是半截注释掉的，已删除。"
#endif

void Startup_Init(void);

/* 进入 Calib 时清飞车检测计数 */
void Startup_ResetFlyCheck(void);
/* Startup->Spin 时清堵转计时 */
void Startup_ResetBlockTimer(void);

/* ---- 子状态处理：Fast = ADC 中断里每拍，Slow = 主循环每 1 ms ---- */
void Startup_ChargeFast(void);
void Startup_ChargeSlow(void);
void Startup_CalibFast(void);
void Startup_CalibSlow(void);
void Startup_AlignFast(void);
void Startup_AlignSlow(void);
void Startup_StartupFast(void);
void Startup_StartupSlow(void);
void Startup_TorqueloopFast(void);
void Startup_TorqueloopSlow(void);

#endif
