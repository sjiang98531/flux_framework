/* Glue for the OLD (test33) user code. */
#include <string.h>
#include "motor_structure.h"
#include "user_function.h"
#include "StateMachine.h"
#include "FluxObs.h"
#include "FluxPLL.h"
void ADC_IRQHandler(void);
void glue_init(void) { Init_Parameter(); }
void glue_cfg(int sc) { (void)sc; }
void glue_isr(void) { ADC_IRQHandler(); }
void glue_main_poll(void)
{
    if (USER.bSlowLoopFlag == 1) { USER.bSlowLoopFlag = 0; s_STATE_SLOW[eM1_MainState](); }
}
int16_t glue_psi_mag(void) { return sFluxObs.s16PsiRMag; }
size_t glue_export(uint8_t *b)
{
    size_t n = 0;
#define PUT(x) do { memcpy(b + n, &(x), sizeof(x)); n += sizeof(x); } while (0)
    PUT(sFluxObs.s32PsiAlF); PUT(sFluxObs.s32PsiBeF);
    PUT(sFluxObs.sEalAvgMA.s32Acc); PUT(sFluxObs.sEbeAvgMA.s32Acc);
    PUT(sFluxObs.s16PsiAl); PUT(sFluxObs.s16PsiBe);
    PUT(sFluxObs.s16PsiRAl); PUT(sFluxObs.s16PsiRBe); PUT(sFluxObs.s16PsiRMag);
    PUT(sFluxObs.s16Eal); PUT(sFluxObs.s16Ebe);
    PUT(sFluxObs.previousIa); PUT(sFluxObs.previousIb);
    PUT(sFluxObs.u8DivCnt); PUT(sFluxObs.haveCurrent);
    PUT(gFluxDiag.voltageAlpha); PUT(gFluxDiag.voltageBeta);
    PUT(gFluxDiag.inputSatCount); PUT(gFluxDiag.fluxSatCount);
    PUT(gFluxDiag.mainState); PUT(gFluxDiag.runState);
    PUT(gFluxPLLDiag.errLast); PUT(gFluxPLLDiag.speedLast); PUT(gFluxPLLDiag.thetaLast);
    PUT(gFluxPLLDiag.satCount); PUT(gFluxPLLDiag.floorCount);
    PUT(M1FaultID); PUT(M1FaultID_Record);
    return n;
}
