/* Glue for the NEW (flux_framework) code. */
#include <string.h>
#include "sim_glue.h"
#include "ctrl_globals.h"
#include "supervisor.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "diag.h"
#include "svpwm.h"
void ADC_IRQHandler(void);
void glue_init(void) { Ctrl_Init(); }
void glue_cfg(int sc) { (void)sc; }
void glue_isr(void) { ADC_IRQHandler(); }
void glue_main_poll(void)
{
    if (USER.bSlowLoopFlag == 1) { USER.bSlowLoopFlag = 0; Ctrl_SlowTick(); }
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

void glue_set_speed_cmd(int16_t q15) { USER.s16VspCmd = q15; }
void glue_probe(SimProbe *p)
{
    p->theta = FOC.s16CmdTheta; p->vd = FOC.sVdq.s16D; p->vq = FOC.sVdq.s16Q;
    p->id = FOC.sIdq.s16D; p->iq = FOC.sIdq.s16Q; p->idRef = FOC.sIdq_cmd.s16D; p->iqRef = FOC.sIdq_cmd.s16Q;
    p->spdFilt = FOC.s16SpdFilt; p->pllSpeed16 = (int16_t)(FOC.sPLL.s32Speed >> 16);
    p->mainState = (unsigned)eM1_MainState; p->runState = (unsigned)eM1_RunSubState;
}
const void *glue_state_ptr(int i)
{
    static const unsigned z = 0;
    switch (i) { case 0: return &FOC; case 1: return &USER; case 2: return &pwm_gen; default: return &z; }
}
size_t glue_state_size(int i)
{
    switch (i) { case 0: return sizeof FOC; case 1: return sizeof USER; case 2: return sizeof pwm_gen; default: return sizeof(unsigned); }
}
