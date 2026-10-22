#ifndef FLUXOBS_H
#define FLUXOBS_H
#include "motor_structure.h"
#include "FluxConfig.h"

/* Rotor-flux voltage model that replaces MLIB_SMO.
 *   e   = v - Rs*i_avg                 v from the previous PWM compares x Vbus
 *   e  -= MA(e)                        high-pass, removes the DC offset
 *   psi += 2*(e*KTs + NLFO corr)       corr = psi_r*(psi_f^2-|psi_r|^2), radial only
 *   psi_r = psi - Ls*i                 -> read directly by the PLL phase detector
 * Inputs are measurements and the previous PWM only; nothing comes from SMO. */

typedef struct {
    int16_t ia, ib, ic, bus;
    int16_t compareA, compareB, compareC;
    uint16_t period;
    uint8_t pwmValid, pwmMode;
} FluxInput_TypeDef;

typedef struct {
    int16_t s16KRs, s16KLs, s16KTs;          /* model constants, Q15 */
    int32_t s32PsiAlF, s32PsiBeF;            /* stator flux, Q16.16 */
    MLIB_FILTER_MA_T sEalAvgMA, sEbeAvgMA;   /* DC removal of e */
    int16_t s16PsiAl, s16PsiBe;              /* stator flux, Q15 */
    int16_t s16PsiRAl, s16PsiRBe, s16PsiRMag;/* rotor flux, Q15 (RMag every 16 ticks) */
    int16_t s16Eal, s16Ebe;                  /* v - Rs*i before the high-pass (J-Scope) */
    int16_t previousIa, previousIb;
    uint8_t u8DivCnt, haveCurrent;
} FluxObs_TypeDef;

typedef struct {
    uint16_t isrLast, isrMax;                /* TIM3 counts, 2250 = 62.5 us = one PWM period */
    uint32_t overBudget, sampleCount;
    uint16_t mainState, runState;
    int16_t voltageAlpha, voltageBeta;
    uint32_t inputSatCount, fluxSatCount;
    uint32_t buildTag;
} FluxDiag_TypeDef;

extern FluxObs_TypeDef sFluxObs;
extern volatile FluxDiag_TypeDef gFluxDiag;
extern FluxInput_TypeDef gFluxInput;

void FluxObs_Init(void);
void FluxObs_Reset(void);
/* Align -> Startup: clear the observer and the PLL speed, seed psi at psi_f on
 * the aligned angle. */
void FluxObs_BeginStartup(FOC_TypeDef *f, MLIB_FILTER_MA_T *speedFilter);
/* Snapshot the previous PWM and the currents at the top of the ISR. */
void FluxObs_Capture(const FOC_TypeDef *f, uint8_t valid);
/* Run the model for this tick. Caller must have computed f->sIAlBe. */
void FluxObs_UpdateFeedback(FOC_TypeDef *f);
void FluxObs_Timing(uint32_t start, uint32_t end);
#endif
