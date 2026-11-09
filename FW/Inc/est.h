/* ============================================================================
 * @file    est.h
 * @brief   无感角度估计：观测器 + PLL 的调度（每拍调用一次）
 *
 *   flux_obs（磁链） -> FluxPLL_PhaseError -> flux_pll -> gFoc.theta
 *
 * 同时负责将 PLL 转速写入 gMeas.speedRaw（经 Meas_SlowFilter 滤波后得到 gMeas.speed，
 * 供速度环、保护和状态切换使用）。
 * ==========================================================================*/
#ifndef EST_H
#define EST_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

/* 估计模块的状态 */
typedef struct {
    PLL_TypeDef pll;            /* 磁链 PLL（flux_pll.c 读写） */
    int16_t     thetaPll;       /* PLL 输出角（已推进到下一拍） */
} Est_TypeDef;

extern Est_TypeDef gEst;

/* 观测器 + 运行期 PLL 跟踪一拍，不写 gFoc.theta（I/F 拖动期间使用）。
 * 返回采样时刻 k 的 PLL 角；返回时 PLL 已推进到 k+1。 */
int16_t Est_TrackTick(void);

/* Spin（闭环运行）：观测器 + 运行期 PLL + 角度合成，写 gFoc.theta / thetaInv、gMeas.speedRaw */
void Est_SpinTick(void);

/* 进入 I/F 冷起动（定位之后）：观测器复位、PLL 角度对齐定位角 */
void Est_OnEnterStartupCold(void);
/* 飞车直接接管进入 Spin：theta = 本拍采样时刻的转子角，spd = 转速 Q15。
 * 预置观测器磁链、PLL（角度推进到下一拍、转速、积分器）以及转速反馈 / 滤波器。 */
void Est_OnCatch(int16_t theta, int16_t spd);
/* 停机 / 故障复位：PLL 转速和积分器清零（角度、增益保留） */
void Est_ResetRun(void);

#endif
