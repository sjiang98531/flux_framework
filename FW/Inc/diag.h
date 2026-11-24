/* ============================================================================
 * @file    diag.h
 * @brief   固件标识与配置检查结果（供调试器 / 上位机读取）
 *
 * 运行状态与故障码见 supervisor.h（gSup.mainState / runState / fault / faultRecord）。
 * ==========================================================================*/
#ifndef DIAG_H
#define DIAG_H
#include <stdint.h>

typedef struct {
    uint32_t buildTag;                 /* 固件版本标识（= BUILD_TAG） */
    uint16_t cfgWarn;                  /* 配置警告：bit0 未用；bit1 = 电流环增益超 65535 被封顶；
                                        * bit2 = PLL 增益 / 角度增益超 int16 被封顶（PLL 带宽太高或 psi_f 太小）；
                                        * bit3 = SPEED_MAX 下反电势超过电压基准（DECOUPLE_K_EMF_Q15 饱和） */
    uint16_t cfgFailLine;              /* 配置检查（cfg_check.h）：第一个不满足项在 cfg_check_list.h 的行号，0 = 全部通过 */
    uint16_t cfgFailCount;             /* 不满足的项数；非 0 时报 MC_FAULT_CFG，电机不起动 */
} FluxDiag_TypeDef;

extern volatile FluxDiag_TypeDef gFluxDiag;

void Diag_Init(void);    /* 上电：写入版本号、记录配置检查结果 */

#endif
