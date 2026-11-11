/* ============================================================================
 * @file    flux_pll.h
 * @brief   转子磁链锁相环
 *
 * 鉴相：err = psi_beta*cos(th^) - psi_alpha*sin(th^) = |psi|*sin(th - th^)
 *   磁链矢量方向不随转向翻转，因此无需按转速指令翻转符号，也无需 rot90 与 omega。
 *   |psi| 恒定，全速域使用同一组增益（cfg_internal.h FLUXPLL_WN_HZ / ZETA）。
 *
 * 状态存储于 gEst.pll（PLL_TypeDef），字段含义：
 *   s32Theta  Q16.16 电角度（>>16 得 0..65535 一圈）
 *   s32Speed  Q31 速度，0x80000000 = SPEED_MAX
 *   s32I_1    Q31 速度，积分器
 *   s16PGain / s16IGain  每 1 LSB 误差对应的 Q31 速度增量（比例 / 每拍积分）
 *   s16ThGain SPEED_TO_THETA
 * ==========================================================================*/
#ifndef FLUX_PLL_H
#define FLUX_PLL_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

/* 装载增益（由配置自动计算）并清零全部状态 */
void FluxPLL_Init(PLL_TypeDef *p);
void FluxPLL_LoadGains(PLL_TypeDef *p);    /* 仅重新装载 Kp / Ki（param.c 更新参数时调用） */

/* 鉴相：转子磁链与当前估计角的叉积 */
int16_t FluxPLL_PhaseError(int16_t psiAlpha, int16_t psiBeta, int16_t theta);

/* 按当前 PLL 转速计算 advQ8/256 拍对应的角度增量（Q15 角度，带符号），用于角度时序补偿 */
int16_t FluxPLL_StepQ15(const PLL_TypeDef *p, int32_t advQ8);

/* 运行期 PLL，返回 Q15 角度 */
int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p);

#endif
