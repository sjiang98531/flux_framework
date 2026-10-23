/* ============================================================================
 * diag.h —— 诊断量（只写不读，不参与控制；J-Scope 看这里）
 * ==========================================================================*/
#ifndef DIAG_H
#define DIAG_H
#include <stdint.h>

typedef struct {
    uint16_t isrLast, isrMax;          /* 快环耗时，TIM3 counts（2250 = 62.5 us = 一个 PWM 周期） */
    uint32_t overBudget, sampleCount;  /* 超预算次数 / 总拍数 */
    uint16_t mainState, runState;      /* gSup.mainState / runState 的拷贝 */
    int16_t  voltageAlpha, voltageBeta;/* 观测器用的重构电压（Q15，475 V 基准） */
    uint32_t inputSatCount, fluxSatCount;
    uint32_t buildTag;                 /* = FW_BUILD_TAG */
    /* ---- flux_framework 新增 ---- */
    uint16_t cfgWarn;                  /* 配置检查：bit0 = MOTOR_IPM=0 但 Ld != Lq；bit1 = 电流环增益超 65535 被封顶；
                                        * bit2 = PLL 增益/角度增益超 int16 被封顶（PLL 带宽太高或 psi_f 太小）；
                                        * bit3 = SPEED_MAX 下反电势超过电压基准（DEC_K_EMF 饱和） */
    int16_t  confidence;               /* 观测器置信度 Q15：1 - | |psi_r| - 目标 | / 目标（慢环算） */
} FluxDiag_TypeDef;

extern volatile FluxDiag_TypeDef gFluxDiag;

void Diag_Init(void);
/* 快环结尾调用：start/end 为 TIM3 计数 */
void Diag_IsrTiming(uint32_t start, uint32_t end);
/* 慢环调用：更新置信度等慢变量 */
void Diag_SlowTick(void);

#endif
