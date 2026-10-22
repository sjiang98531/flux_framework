#ifndef FLUXPLL_H
#define FLUXPLL_H
#include "motor_structure.h"

/* Self-contained rotor-flux PLL.
 *
 * Replaces MLIB_PLL() / MLIB_PLL1() without touching the closed-source library.
 * FOC.sPLL is reused purely as storage, so the FOC_TypeDef byte layout stays
 * identical for Fast_Boot_Current_Controller / MCF_ADV_Protect / Var_Filt.
 *
 * Field convention (deliberately different from the vendor one):
 *   s32Theta   Q16.16 electrical angle: the integer half (s32Theta>>16) is
 *              0..65535 for one electrical revolution, the low half is the
 *              fraction. A tick at 3200 rpm therefore adds 1092<<16, not 1092.
 *              (Same layout as the vendor: M1_SwitchRunAlignStartup seeds it
 *              with s16CmdTheta<<16 and MLIB_PLL returns s32Theta>>16.)
 *   s32Speed   Q31 speed, 0x80000000 = SPEED_MAX
 *   s32I_1     Q31 speed, integrator state (this is what the startup floor clamps)
 *   s16PGain   Q31 speed increment per err LSB        (proportional)
 *   s16IGain   Q31 speed increment per err LSB per tick (integral)
 *   s16ThGain  SPEED_TO_THETA (Q15), turns s32Speed into an angle step
 *   i16PGainSh / i16IGainSh / i16ThGainSh are unused here and kept at 0.
 *
 * The phase detector input is the rotor-flux cross product, so no rot90 and no
 * omega factor are needed:
 *   err = psi_beta*cos(theta_hat) - psi_alpha*sin(theta_hat) = |psi|*sin(theta-theta_hat)
 * err is therefore inherently correct for both rotation directions; the vendor
 * sign flip on s16SpdCmd must NOT be applied to it.
 */

typedef struct {
    int16_t  errLast;
    int16_t  speedLast;
    uint16_t thetaLast;
    uint32_t satCount;
    uint32_t floorCount;
    uint32_t buildTag;
} FluxPLL_Diag_TypeDef;

extern volatile FluxPLL_Diag_TypeDef gFluxPLLDiag;

/* Load gains from FluxConfig.h and clear all state. */
void FluxPLL_Init(PLL_TypeDef *p);

/* Run-time PLL. Returns the Q15 angle. */
int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p);

/* Startup PLL: identical to FluxPLL_Run plus the integrator speed floor that
 * MLIB_PLL1 provides. floorQ31 is in the same Q31 speed unit as s32I_1, so the
 * vendor call site Q31(0.004) keeps its exact meaning (12.8 rpm). */
int16_t FluxPLL_RunStartup(int16_t err, PLL_TypeDef *p, int16_t spdCmd, int32_t floorQ31);

#endif
