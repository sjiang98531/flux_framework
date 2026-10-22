/* Self-contained rotor-flux PLL. See FluxPLL.h for the unit conventions.
 *
 * Why this exists: the vendor MLIB_PLL was tuned against SMO's Ea/Eb, whose
 * amplitude is proportional to speed. A rotor-flux vector has a constant
 * amplitude, so reusing the vendor gains makes the loop bandwidth collapse at
 * high speed and explode at low speed. Owning the loop lets both gains be
 * scaled against the actual flux magnitude once, for the whole speed range.
 *
 * Gains are derived from a second-order loop model:
 *   err    = |psi| * sin(theta - theta_hat)  ->  Kd = |psi| (Q15 counts per rad)
 *   v      = Kp*err + Ki*integral(err)       ->  wn = sqrt(Kd*Ki), 2*zeta*wn = Kd*Kp
 * Kp and Ki below are already in "Q31 speed increment per err LSB" units.
 *
 * Unit chain, verified against the vendor code rather than assumed:
 *   FOC.sPLL.s32Speed >> 16 is the Q15 speed used everywhere else in the
 *   project, so 2^31 corresponds to SPEED_MAX = 3200 rpm. That gives
 *   1 rad/s = 2^31 / (2*pi*3200*5/60) = 1281700 in Q31 speed units, which is
 *   the factor behind both gain constants.
 */
#include "FluxPLL.h"
#include "FluxConfig.h"

volatile FluxPLL_Diag_TypeDef gFluxPLLDiag;

/* The vendor MLIB_PLL applies no clamp at all: a Q31 speed accumulator simply
 * wraps once it passes 2^31, which is why the stock firmware breaks at exactly
 * 3200 rpm. Clamping at 0x7FFF8000 keeps 3199.9 rpm reachable while stopping
 * the wrap one LSB earlier than the natural boundary. */
#define SPEED_LIMIT 0x7FFF8000L

void FluxPLL_Init(PLL_TypeDef *p)
{
    p->s16PGain    = FLUXPLL_KP_Q31_PER_ERR;
    p->s16IGain    = FLUXPLL_KI_Q31_PER_ERR;
    p->i16PGainSh  = 0;
    p->i16IGainSh  = 0;
    p->s16ThGain   = (int16_t)SPEED_TO_THETA;
    p->i16ThGainSh = 0;
    p->s32Theta    = 0;
    p->s32Speed    = 0;
    p->s32I_1      = 0;
    gFluxPLLDiag.errLast   = 0;
    gFluxPLLDiag.speedLast = 0;
    gFluxPLLDiag.thetaLast = 0;
    gFluxPLLDiag.satCount  = 0;
    gFluxPLLDiag.floorCount= 0;
    gFluxPLLDiag.buildTag  = FLUXPLL_BUILD_TAG;
}

/* Saturating add. The plain "+" here was the second half of the wrap problem:
 * with the integrator already at the clamp, an error above 656 counts (5.6 deg)
 * would have flipped s32I_1 + kpTerm straight to a large negative value and
 * reversed the angle. Raising the limit without this would only have widened
 * the window in which it could happen. */
static __forceinline int32_t addSat(int32_t a, int32_t b)
{
    if(b > 0 && a > (int32_t)0x7FFFFFFF - b) return  (int32_t)0x7FFFFFFF;
    if(b < 0 && a < -(int32_t)0x7FFFFFFF - b) return -(int32_t)0x7FFFFFFF;
    return a + b;
}

static __forceinline int32_t speedClamp(int32_t v)
{
    if(v >  SPEED_LIMIT){gFluxPLLDiag.satCount++;return  SPEED_LIMIT;}
    if(v < -SPEED_LIMIT){gFluxPLLDiag.satCount++;return -SPEED_LIMIT;}
    return v;
}

/* s32Theta is Q16.16: the integer half (>>16) is the angle in 0..65535 for one
 * electrical revolution. One tick at full speed (3200 rpm -> 266.67 Hz elec,
 * 16 kHz) must therefore advance s32Theta by 1092 << 16, not by 1092:
 *
 *   step = (s32Speed * SPEED_TO_THETA) >> 15        <-- vendor, and correct
 *
 * Confirmed against the shipped library: MLIB_PLL at 0x08002BF1 calls the
 * 32x32->32 helper at 0x08002BE0, whose body is
 *     uxth/asrs split speed, muls by kTheta, then (hi<<1) + (lo>>15)
 *   == (speed * kTheta) >> 15
 * followed by an i16ThGainSh shift and s32Theta += result. An earlier version
 * of this function shifted 16 more bits, which froze the angle and caused the
 * "11 Hz oscillation" of test2/test3.
 *
 * (speed>>15)*kTheta keeps the multiply in 32 bits (max 65534*1092 = 7.16e7)
 * and drops at most kTheta LSB of the Q16.16 accumulator, which the integrator
 * absorbs; there is no cumulative drift. */
static __forceinline int32_t angleStep(int32_t speed, int16_t kTheta)
{
    return (int32_t)(speed >> 15) * (int32_t)kTheta;
}

static __forceinline int16_t advance(PLL_TypeDef *p, int32_t kpTerm, int32_t kiTerm, int32_t minSpeed)
{
    p->s32I_1   = speedClamp(addSat(p->s32I_1, kiTerm));
    p->s32Speed = speedClamp(addSat(p->s32I_1, kpTerm));
    /* test29: Startup only (minSpeed == 0 in Spin, so this is a no-op there).
     *
     * With the vendor SMO, err is ~0 at standstill, so flooring the INTEGRATOR
     * is enough to keep theta advancing. The flux detector has full amplitude
     * at zero speed instead: |psi| = 6111, so err = -336 counts already cancels
     * the floor (floorQ31/Kp = 8.59e6/25583 = 336), the PLL stalls, and the
     * drive never leaves Startup.
     *
     * The clamp MUST sit here, between computing s32Speed and advancing
     * s32Theta. Clamping s32Speed after advance() returns would be cosmetic:
     * this tick's angle has already been advanced at the unclamped speed and
     * s32Speed is recomputed from scratch next tick, so theta would stay frozen
     * while s16Speed/filt merely REPORT 12.8 rpm.
     *
     * This does not deviate from the vendor behaviour, it restores it: vendor
     * PLL1 + SMO effectively never ran below the floor either. */
    if(minSpeed > 0 && p->s32Speed < minSpeed) p->s32Speed = minSpeed;
    if(minSpeed < 0 && p->s32Speed > minSpeed) p->s32Speed = minSpeed;
    /* The Q16.16 accumulator wraps once per electrical revolution at full speed
     * (every 60 ticks), so add through uint32 to keep the wrap defined. */
    p->s32Theta = (int32_t)((uint32_t)p->s32Theta
                          + (uint32_t)angleStep(p->s32Speed, p->s16ThGain));
    gFluxPLLDiag.speedLast = (int16_t)(p->s32Speed >> 16);
    gFluxPLLDiag.thetaLast = (uint16_t)(p->s32Theta >> 16);
    return (int16_t)(p->s32Theta >> 16);
}

int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p)
{
    int32_t kpTerm = (int32_t)err * p->s16PGain;
    int32_t kiTerm = (int32_t)err * p->s16IGain;
    gFluxPLLDiag.errLast = err;
    return advance(p, kpTerm, kiTerm, 0);          /* test29: no extra floor in Spin */
}

int16_t FluxPLL_RunStartup(int16_t err, PLL_TypeDef *p, int16_t spdCmd, int32_t floorQ31)
{
    int32_t kpTerm = (int32_t)err * p->s16PGain;
    int32_t kiTerm = (int32_t)err * p->s16IGain;
    int32_t integ  = addSat(p->s32I_1, kiTerm);

    gFluxPLLDiag.errLast = err;
    /* Vendor MLIB_PLL1 semantics, recovered from the disassembly: the floor is
     * applied to the integrator state, not to the output speed, so a zero error
     * still leaves a minimum speed that drags the rotor along. The negative
     * branch also lifts a negative integrator straight to +floor, which is what
     * the vendor code does and what makes a direction reversal restart cleanly. */
    if(spdCmd > 0){
        if(integ < floorQ31){integ = floorQ31;gFluxPLLDiag.floorCount++;}
    }else{
        if(integ > -floorQ31){integ = -floorQ31;gFluxPLLDiag.floorCount++;}
    }
    p->s32I_1 = integ;

    return advance(p, kpTerm, 0, spdCmd > 0 ? floorQ31 : -floorQ31);  /* test29 */
}
