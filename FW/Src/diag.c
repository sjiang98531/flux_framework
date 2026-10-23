#include "diag.h"
#include "fw_config.h"
#include "Hardware_config.h"
#include "flux_obs.h"
#include "foc_core.h"

volatile FluxDiag_TypeDef gFluxDiag;

void Diag_Init(void)
{
    gFluxDiag.buildTag = FW_BUILD_TAG;
    gFluxDiag.isrLast = 0; gFluxDiag.isrMax = 0;
    gFluxDiag.overBudget = 0; gFluxDiag.sampleCount = 0;
    gFluxDiag.inputSatCount = 0; gFluxDiag.fluxSatCount = 0;
    gFluxDiag.confidence = 0;
    gFluxDiag.cfgWarn = 0;
#if !MOTOR_IPM
    if (MOTOR_LD_H != MOTOR_LQ_H) gFluxDiag.cfgWarn |= 1u;
#endif
    if (CURRENT_KP_D > 65535 || CURRENT_KP_Q > 65535 || CURRENT_KI_D > 65535 || CURRENT_KI_Q > 65535)
        gFluxDiag.cfgWarn |= 2u;        /* 电流环带宽太高，增益被封顶 */
    if (FLUXPLL_KP_Q31_PER_ERR > 32767 || FLUXPLL_KI_Q31_PER_ERR > 32767 || SPEED_TO_THETA >= 32767)
        gFluxDiag.cfgWarn |= 4u;        /* PLL 增益放不进 int16（FluxPLL_Init 已封顶，带宽会变低） */
    if (W_EMAX_RAD_S * MOTOR_PSI_F_WB / UDC_MAX >= 1.0)
        gFluxDiag.cfgWarn |= 8u;
}

void Diag_IsrTiming(uint32_t start, uint32_t end)
{
    uint32_t period = 65536UL - (uint32_t)TASK_BASETIM_PRIOD_LOAD;
    uint32_t elapsed = end >= start ? end - start : period - start + end;
    gFluxDiag.sampleCount++;
    gFluxDiag.isrLast = (uint16_t)elapsed;
    if (elapsed > gFluxDiag.isrMax) gFluxDiag.isrMax = (uint16_t)elapsed;
    if (elapsed >= FLUX_ISR_BUDGET) gFluxDiag.overBudget++;
}

void Diag_SlowTick(void)
{
    int32_t mag = sFluxObs.s16PsiRMag;
    int32_t tgt = FluxObs_TargetMag();
    int32_t dev, c;
    if (mag == 0 || tgt <= 0) { gFluxDiag.confidence = 0; return; }
    dev = mag - tgt;
    if (dev < 0) dev = -dev;
    c = 32767 - (int32_t)(((int32_t)dev * 32767) / tgt);   /* 慢环，允许除法 */
    gFluxDiag.confidence = (int16_t)(c < 0 ? 0 : c);
}
