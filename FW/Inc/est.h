/* ============================================================================
 * est.h —— 无感角度估计：观测器 + PLL 的编排（每拍调用一次）
 *
 *   flux_obs（磁链） -> FluxPLL_PhaseError -> flux_pll -> gFoc.theta
 *
 * 也负责把 PLL 转速写进 gMeas.speedRaw（Meas_SlowFilter 滤波后得到 gMeas.speed，
 * 速度环、保护和状态切换都用它）。
 * ==========================================================================*/
#ifndef EST_H
#define EST_H
#include "fw_config.h"
#include "mc_types.h"

/* 估计模块的状态 */
typedef struct {
    PLL_TypeDef pll;            /* 磁链 PLL（flux_pll.c 读写；飞车检测也借用它） */
    int16_t     thetaPll;       /* PLL 输出角（已推进到下一拍） */
    int16_t     thetaOffset;    /* 飞车接管时的角度差，Spin 里每拍消 9 LSB */
} Est_TypeDef;

extern Est_TypeDef gEst;

/* 观测器 + 运行期 PLL 跟踪一拍，不写 gFoc.theta（I/F 拖动期间用）。
 * 返回采样时刻 k 的 PLL 角；返回时 PLL 已推进到 k+1。 */
int16_t Est_TrackTick(void);

/* Spin（闭环运行）：观测器 + 运行期 PLL + 角度合成，写 gFoc.theta / thetaInv、gMeas.speedRaw */
void Est_SpinTick(void);

/* 原厂启动：转向保护 + 观测器 + 启动期 PLL（带速度下限），写 gFoc.theta */
void Est_StartupTick(void);

/* 原厂启动期间上报转速（加速度限幅，test31）。在本拍 PWM 更新之后调用。 */
void Est_StartupReportSpeed(void);

/* 进入 Startup：冷启动（Align 之后）从 0 开始上报 */
void Est_OnEnterStartupCold(void);
/* 进入 Startup：从 Spin 回退，转子还在转，从当前上报值开始 */
void Est_OnEnterStartupFromSpin(void);

#endif
