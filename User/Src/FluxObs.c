/* Rotor-flux observer feeding the self-written FluxPLL. See FluxObs.h.
 * Experiment history behind every choice below: 实验记录.md / README.md. */
#include "FluxObs.h"
#include "hardware_config.h"
#include "pwm_gen.h"
FluxObs_TypeDef sFluxObs;
volatile FluxDiag_TypeDef gFluxDiag;
FluxInput_TypeDef gFluxInput;

#define FMAX ((int32_t)29491L * 65536L)
#define PSI_LIMIT 23170
#define COUNT_SCALE ((int32_t)(1073741824.0f / PWM_PERIOD))
/* Measured rotor flux 0.141 Wb in the model's Q15 base: 0.141/0.756 -> 6111. */
#define NOMINAL_Q15 Q15(FLUX_NOMINAL_WB/FLUX_PSI_N)
/* |psi_f|^2 in squared flux LSB, for the NLFO radial correction. */
#define FLUX_NOM2   ((int32_t)NOMINAL_Q15*(int32_t)NOMINAL_Q15)

static __forceinline int16_t mul(int16_t a,int16_t b)
{return (int16_t)(((int32_t)a*b)>>15);}
static __forceinline int32_t clamp(int32_t x,int32_t n)
{return x>n?n:(x<-n?-n:x);}
static __forceinline int16_t inputClamp(int32_t x)
{
    if(x>29491||x<-29491)gFluxDiag.inputSatCount++;
    return (int16_t)clamp(x,29491);
}
static __forceinline int32_t addFlux(int32_t x,int32_t dx)
{
    int32_t sum=x+dx;
    if(sum>FMAX||sum<-FMAX)gFluxDiag.inputSatCount++;
    return clamp(sum,FMAX);
}

void FluxObs_Capture(const FOC_TypeDef *f,uint8_t valid)
{
    const PWM_GEN_T *p=&pwm_gen;
    gFluxInput.ia=f->sIabc.s16A;gFluxInput.ib=f->sIabc.s16B;gFluxInput.ic=f->sIabc.s16C;
    gFluxInput.bus=f->s16Vbus;
    gFluxInput.compareA=p->CompA;gFluxInput.compareB=p->CompB;gFluxInput.compareC=p->CompC;
    gFluxInput.period=p->N_halfPeriod;gFluxInput.pwmMode=p->Mode;gFluxInput.pwmValid=valid;
}
void FluxObs_Reset(void)
{
    sFluxObs.s32PsiAlF=0;sFluxObs.s32PsiBeF=0;
    sFluxObs.sEalAvgMA.s32Acc=0;sFluxObs.sEbeAvgMA.s32Acc=0;
    sFluxObs.s16PsiAl=0;sFluxObs.s16PsiBe=0;
    sFluxObs.s16PsiRAl=0;sFluxObs.s16PsiRBe=0;sFluxObs.s16PsiRMag=0;
    sFluxObs.s16Eal=0;sFluxObs.s16Ebe=0;
    sFluxObs.u8DivCnt=0;sFluxObs.haveCurrent=0;
    gFluxDiag.voltageAlpha=0;gFluxDiag.voltageBeta=0;
}
void FluxObs_Init(void)
{
    gFluxDiag.buildTag=FLUXPLL_BUILD_TAG;
    sFluxObs.s16KRs=Q15(Rs*I_MAX/FLUX_V_BASE);
    sFluxObs.s16KLs=Q15(Ls*I_MAX/FLUX_PSI_N);
    sFluxObs.s16KTs=Q15(2.0f*3.14159265f*FLUX_WN_HZ/PWM_FREQ);
    sFluxObs.sEalAvgMA.u16Sh=12;sFluxObs.sEbeAvgMA.u16Sh=12;
    gFluxDiag.isrLast=0;gFluxDiag.isrMax=0;gFluxDiag.overBudget=0;gFluxDiag.sampleCount=0;
    gFluxDiag.inputSatCount=0;gFluxDiag.fluxSatCount=0;
    FluxObs_Reset();
}
void FluxObs_BeginStartup(FOC_TypeDef *f, MLIB_FILTER_MA_T *speedFilter)
{
    /* Cold Align -> Startup only. The preceding fly-start detector also uses
     * the shared PLL; its residual speed must not qualify this new observer.
     * The caller's aligned angle, PLL gains and all control-loop PIs are kept. */
    FluxObs_Reset();
    f->sPLL.s32Speed=0;f->sPLL.s32I_1=0;
    f->FluxFeedback.s16Speed=0;
    f->FluxFeedback.s16Ealpha_est=0;f->FluxFeedback.s16Ebeta_est=0;
    f->s16SpdFilt=0;speedFilter->s32Acc=0;
    /* test23: seed psi at |psi_f| on the PLL angle instead of 0.
     * With psi=0 the NLFO grows the magnitude back within ~30 ms but in the
     * direction of whatever the Rs/dead-time error happens to be, so the PLL
     * error is near full scale and theta_hat swings ~half a turn: test22 saw a
     * 177-191 rpm false speed 32-34 ms after every Startup entry, Fast_Boot
     * switched to Spin on it and fell back 30-40 ms later.
     * The caller has already set sPLL.s32Theta = s16CmdTheta<<16, so seeding
     * on s16CmdTheta makes the first PLL error exactly 0. psi_r is recomputed
     * from these accumulators every tick. */
    {
        sAngle_Trig a = MLIB_SinCos(f->s16CmdTheta);
        sFluxObs.s32PsiAlF = (int32_t)mul(NOMINAL_Q15, a.s16Cos) << 16;
        sFluxObs.s32PsiBeF = (int32_t)mul(NOMINAL_Q15, a.s16Sin) << 16;
    }
}
/* Smaller compare = higher pole voltage in this PWM implementation.
 * Reconstruct the commanded average of the previous period from the compares
 * and the measured bus. Dead time is NOT included (DTC tried twice, test12 and
 * test16, both worse; the effective error is ~2 V, see 实验记录 C2). */
static void voltage(int16_t *va,int16_t *vb)
{
    int32_t a,b,c,t;
    a=((int32_t)(PWM_PERIOD-gFluxInput.compareA)*COUNT_SCALE)>>15;
    b=((int32_t)(PWM_PERIOD-gFluxInput.compareB)*COUNT_SCALE)>>15;
    c=((int32_t)(PWM_PERIOD-gFluxInput.compareC)*COUNT_SCALE)>>15;
    a=(a*gFluxInput.bus)>>15;b=(b*gFluxInput.bus)>>15;c=(c*gFluxInput.bus)>>15;
    t=2*a-b-c;
    *va=(int16_t)((t*21846L+32768L)>>16);
    *vb=(int16_t)(((b-c)*18919L)>>15);
}
static void FluxObs_Run(const FOC_TypeDef *f)
{
    MCFLIB_2_ALBE_T_S16 current;
    int16_t va,vb,ea,eb,ia,ib;
    int32_t x,y,corrA,corrB;
    if(!gFluxInput.pwmValid||gFluxInput.period!=PWM_PERIOD||gFluxInput.bus<Q15(FLUX_BUS_MIN_V/UDC_MAX)||
       gFluxInput.compareA<0||gFluxInput.compareA>PWM_PERIOD||
       gFluxInput.compareB<0||gFluxInput.compareB>PWM_PERIOD||
       gFluxInput.compareC<0||gFluxInput.compareC>PWM_PERIOD){
        FluxObs_Reset();
        return;
    }
    /* The caller already computed this sample's Clarke. */
    current=f->sIAlBe;
    voltage(&va,&vb);gFluxDiag.voltageAlpha=va;gFluxDiag.voltageBeta=vb;
    /* Rs drop uses the interval-average current; Ls*i uses the endpoint. */
    ia=current.s16Alpha;ib=current.s16Beta;
    if(sFluxObs.haveCurrent){ia=((int32_t)ia+sFluxObs.previousIa)/2;ib=((int32_t)ib+sFluxObs.previousIb)/2;}
    sFluxObs.previousIa=current.s16Alpha;sFluxObs.previousIb=current.s16Beta;sFluxObs.haveCurrent=1;
    ea=inputClamp((int32_t)va-mul(sFluxObs.s16KRs,ia));
    eb=inputClamp((int32_t)vb-mul(sFluxObs.s16KRs,ib));
    sFluxObs.s16Eal=ea;sFluxObs.s16Ebe=eb;
    ea=inputClamp((int32_t)ea-MLIB_FilterMA_Q15(ea,&sFluxObs.sEalAvgMA));
    eb=inputClamp((int32_t)eb-MLIB_FilterMA_Q15(eb,&sFluxObs.sEbeAvgMA));
    if(++sFluxObs.u8DivCnt>=16){ sFluxObs.u8DivCnt=0; }
    x=(sFluxObs.s32PsiAlF+32768)>>16;y=(sFluxObs.s32PsiBeF+32768)>>16;
    /* ---- NLFO radial correction (after SguanFOC_Library Sguan_NLFO.c, MIT) --
     *     corr = Gain * psi_r * (|psi_f|^2 - |psi_r|^2)
     * Purely along psi_r, so a voltage error can only change the magnitude,
     * never steer the angle. (The older e/omega reference pull had a tangential
     * component and dragged the angle: test4/5/7/8.)
     * Small signal: first order, tau ~ 16 ms at FLUX_NLFO_SH = 9.
     * Scaling, in flux LSB (6111 = |psi_f|):
     *   ferr = |psi_f|^2 - |psi_r|^2        |.| <= 1.07e9
     *   fq   = ferr/|psi_f|^2 in Q15 = (ferr>>12)*3, clamped to +-32768
     *          ((>>12)*3 was sized for 0.154 Wb; at 0.141 the gain is ~17%
     *          weaker, tau ~19 ms - left as is, it works)
     *   corr = (psi_r*fq) >> FLUX_NLFO_SH   same scale as e*KTs
     * Do not tighten FLUX_NLFO_FQ_LIMIT to damp transients: test11 (8192) was
     * strictly worse. */
    {
        int32_t g0=(int32_t)x-mul(sFluxObs.s16KLs,current.s16Alpha);
        int32_t g1=(int32_t)y-mul(sFluxObs.s16KLs,current.s16Beta);
        int32_t ferr,fq;
        /* Clamp before squaring, otherwise g0*g0 can reach 3.1e9 and wrap. */
        if(g0> PSI_LIMIT)g0= PSI_LIMIT;
        if(g0<-PSI_LIMIT)g0=-PSI_LIMIT;
        if(g1> PSI_LIMIT)g1= PSI_LIMIT;
        if(g1<-PSI_LIMIT)g1=-PSI_LIMIT;
        ferr=FLUX_NOM2-((int32_t)g0*g0+(int32_t)g1*g1);
        fq=(ferr>>12)*3;
        if(fq> FLUX_NLFO_FQ_LIMIT)fq= FLUX_NLFO_FQ_LIMIT;
        if(fq<-FLUX_NLFO_FQ_LIMIT)fq=-FLUX_NLFO_FQ_LIMIT;
        corrA=((int32_t)g0*fq)>>FLUX_NLFO_SH;
        corrB=((int32_t)g1*fq)>>FLUX_NLFO_SH;
    }
    sFluxObs.s32PsiAlF=addFlux(sFluxObs.s32PsiAlF,2*((int32_t)ea*sFluxObs.s16KTs+corrA));
    sFluxObs.s32PsiBeF=addFlux(sFluxObs.s32PsiBeF,2*((int32_t)eb*sFluxObs.s16KTs+corrB));
    sFluxObs.s16PsiAl=(sFluxObs.s32PsiAlF+32768)>>16;sFluxObs.s16PsiBe=(sFluxObs.s32PsiBeF+32768)>>16;
    x=(int32_t)sFluxObs.s16PsiAl-mul(sFluxObs.s16KLs,current.s16Alpha);
    y=(int32_t)sFluxObs.s16PsiBe-mul(sFluxObs.s16KLs,current.s16Beta);
    if(x>PSI_LIMIT||x<-PSI_LIMIT||y>PSI_LIMIT||y<-PSI_LIMIT)gFluxDiag.fluxSatCount++;
    sFluxObs.s16PsiRAl=(int16_t)clamp(x,PSI_LIMIT);sFluxObs.s16PsiRBe=(int16_t)clamp(y,PSI_LIMIT);
    /* |psi_r| is a J-Scope channel only; one sqrt per 16 ticks. */
    if(sFluxObs.u8DivCnt==8){
        sFluxObs.s16PsiRMag=Sqrt_Q15((int32_t)sFluxObs.s16PsiRAl*sFluxObs.s16PsiRAl+(int32_t)sFluxObs.s16PsiRBe*sFluxObs.s16PsiRBe);
    }
}
void FluxObs_UpdateFeedback(FOC_TypeDef *f)
{
    FluxObs_Run(f);
    /* The closed-source library only reads FluxFeedback.s16Speed (Var_Filt,
     * FOC+0x90); s16Speed is written by PMSM_Obs()/M1_RunStartupFast(). The
     * two EMF fields have no reader and are held at 0 (test27 disassembly). */
    f->FluxFeedback.s16Ealpha_est=0;
    f->FluxFeedback.s16Ebeta_est=0;
}
void FluxObs_Timing(uint32_t start,uint32_t end)
{
    uint32_t period=65536UL-(uint32_t)TASK_BASETIM_PRIOD_LOAD;
    uint32_t elapsed=end>=start?end-start:period-start+end;
    gFluxDiag.sampleCount++;gFluxDiag.isrLast=(uint16_t)elapsed;
    if(elapsed>gFluxDiag.isrMax)gFluxDiag.isrMax=(uint16_t)elapsed;
    if(elapsed>=FLUX_ISR_BUDGET)gFluxDiag.overBudget++;
}
