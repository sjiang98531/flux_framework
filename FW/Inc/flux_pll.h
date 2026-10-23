/* ============================================================================
 * flux_pll.h —— 转子磁链锁相环（替代原厂 MLIB_PLL / MLIB_PLL1）
 *
 * 鉴相：err = psi_beta*cos(th^) - psi_alpha*sin(th^) = |psi|*sin(th - th^)
 *   磁链矢量不随转向翻转，所以不按转速指令翻符号，也不需要 rot90 和 omega。
 *   |psi| 恒定，所以全速域一组增益（cfg_ctrl.h 的 FLUXPLL_WN_HZ / ZETA）。
 *
 * 存储在 gEst.pll（PLL_TypeDef，结构体沿用原厂开源头文件），字段含义：
 *   s32Theta  Q16.16 电角度（>>16 得 0..65535 一圈）
 *   s32Speed  Q31 速度，0x80000000 = SPEED_MAX
 *   s32I_1    Q31 速度，积分器（启动下限加在这里和输出上）
 *   s16PGain / s16IGain  "每 1 LSB 误差的 Q31 速度增量"（比例 / 每拍积分）
 *   s16ThGain SPEED_TO_THETA；各 Sh 字段不用，保持 0
 * ==========================================================================*/
#ifndef FLUX_PLL_H
#define FLUX_PLL_H
#include "fw_config.h"
#include "mc_types.h"

typedef struct {
    int16_t  errLast;
    int16_t  speedLast;
    uint16_t thetaLast;
    uint32_t satCount;
    uint32_t floorCount;
    uint32_t buildTag;
} FluxPLL_Diag_TypeDef;

extern volatile FluxPLL_Diag_TypeDef gFluxPLLDiag;

/* 装载增益（cfg 自动算）并清零全部状态 */
void FluxPLL_Init(PLL_TypeDef *p);

/* 鉴相：转子磁链与当前估计角的叉积 */
int16_t FluxPLL_PhaseError(int16_t psiAlpha, int16_t psiBeta, int16_t theta);

/* 按当前 PLL 转速，advQ8/256 拍对应的角度增量（Q15 角度，带符号）。角度时序补偿用 */
int16_t FluxPLL_StepQ15(const PLL_TypeDef *p, int32_t advQ8);

/* 运行期 PLL，返回 Q15 角度 */
int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p);

/* 启动期 PLL：同 Run，另加速度下限（积分器 + 输出转速，且在推进角度之前）。
 * floorQ31 与 s32I_1 同单位；spdCmd 决定方向。 */
int16_t FluxPLL_RunStartup(int16_t err, PLL_TypeDef *p, int16_t spdCmd, int32_t floorQ31);

#endif
