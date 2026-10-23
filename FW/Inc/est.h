/* ============================================================================
 * est.h —— 无感角度估计：观测器 + PLL 的编排（每拍调用一次）
 *
 *   flux_obs（磁链） -> FluxPLL_PhaseError -> flux_pll -> FOC.s16CmdTheta
 *
 * 也负责把 PLL 转速写进 FOC.FluxFeedback.s16Speed（闭源库 Var_Filt 从这里读，
 * 滤波后得到 s16SpdFilt，速度环和 Fast_Boot 都用它）。
 * ==========================================================================*/
#ifndef EST_H
#define EST_H
#include "fw_config.h"
#include "vendor_lib.h"

/* 观测器 + 运行期 PLL 跟踪一拍，不写 s16CmdTheta（I/F 拖动期间用）。
 * 返回采样时刻 k 的 PLL 角；返回时 PLL 已推进到 k+1。 */
int16_t Est_TrackTick(FOC_TypeDef *f);

/* Spin（闭环运行）：观测器 + 运行期 PLL + 角度合成（原 PMSM_Obs） */
void Est_SpinTick(FOC_TypeDef *f);

/* Startup：转向保护 + 观测器 + 启动期 PLL（带速度下限），写 s16CmdTheta */
void Est_StartupTick(FOC_TypeDef *f);

/* Startup 期间上报转速（加速度限幅，test31）。在本拍 PWM 更新之后调用。 */
void Est_StartupReportSpeed(FOC_TypeDef *f);

/* 进入 Startup：冷启动（Align 之后）从 0 开始上报 */
void Est_OnEnterStartupCold(FOC_TypeDef *f);
/* 进入 Startup：从 Spin 回退，转子还在转，从当前上报值开始 */
void Est_OnEnterStartupFromSpin(FOC_TypeDef *f);

#endif
