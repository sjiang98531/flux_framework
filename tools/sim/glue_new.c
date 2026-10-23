/* Glue for the NEW (flux_framework) code. */
#include <string.h>
#include "sim_glue.h"
#include "meas.h"
#include "foc_core.h"
#include "est.h"
#include "speed_ctrl.h"
#include "protect.h"
#include "flystart.h"
#include "startup.h"
#include "hal.h"
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
    if (gHal.slowLoopFlag == 1) { gHal.slowLoopFlag = 0; Ctrl_SlowTick(); }
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
    PUT(gSup.fault); PUT(gSup.faultRecord);
    return n;
}

void glue_set_speed_cmd(int16_t q15) { gSup.speedCmd = q15; }
void glue_probe(SimProbe *p)
{
    p->theta = gFoc.theta; p->vd = gFoc.vdq.s16D; p->vq = gFoc.vdq.s16Q;
    p->id = gFoc.idq.s16D; p->iq = gFoc.idq.s16Q; p->idRef = gFoc.idqRef.s16D; p->iqRef = gFoc.idqRef.s16Q;
    p->spdFilt = gMeas.speed; p->pllSpeed16 = (int16_t)(gEst.pll.s32Speed >> 16);
    p->mainState = (unsigned)gSup.mainState; p->runState = (unsigned)gSup.runState;
}
const void *glue_state_ptr(int i)
{
    static const unsigned z = 0;
    switch (i) { case 0: return &gMeas; case 1: return &gFoc; case 2: return &pwm_gen; case 3: return &gSpd; default: return &z; }
}
size_t glue_state_size(int i)
{
    switch (i) { case 0: return sizeof gMeas; case 1: return sizeof gFoc; case 2: return sizeof pwm_gen; case 3: return sizeof gSpd; default: return sizeof(unsigned); }
}
