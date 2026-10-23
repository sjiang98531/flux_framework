/* Random-input fuzz of observer + PLL (old vs new built separately; hash stream compared). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "motor_structure.h"
#ifdef NEWCODE
#include "vendor_lib.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "diag.h"
#define PLLRUN FluxPLL_Run
#define PLLST  FluxPLL_RunStartup
static int16_t perr(int16_t a,int16_t b,int16_t th){return FluxPLL_PhaseError(a,b,th);}
#else
#include "pwm_gen.h"
#include "FluxObs.h"
#include "FluxPLL.h"
#define PLLRUN FluxPLL_Run
#define PLLST  FluxPLL_RunStartup
static int16_t perr(int16_t a,int16_t b,int16_t th){sAngle_Trig s=MLIB_SinCos(th);return MLIB_Mul_Q15(b,s.s16Cos)-MLIB_Mul_Q15(a,s.s16Sin);}
#endif
EPWM_TypeDef sim_EPWM; TIM_TypeDef sim_TIM0, sim_TIM3; ADC_TypeDef sim_ADC; WDT_TypeDef sim_WDT; GPIO_TypeDef sim_GPIOA;
USER_TypeDef USER; FOC_TypeDef FOC; FastBoot_TypeDef FastBoot_Para; volatile unsigned int eM1_RunSubState;
static uint32_t rs=1;
static uint32_t rnd(void){rs^=rs<<13;rs^=rs>>17;rs^=rs<<5;return rs;}
static int16_t r16(int range){return (int16_t)((int32_t)(rnd()%(2*range+1))-range);}
static uint64_t fnv(uint64_t h,const void*p,size_t n){const uint8_t*b=p;while(n--){h^=*b++;h*=1099511628211ULL;}return h;}
int main(int argc,char**argv){
    long N=argc>1?atol(argv[1]):400000; FILE*f=fopen(argc>2?argv[2]:"fz.hash","wb");
    FluxPLL_Init(&FOC.sPLL); FluxObs_Init();
#ifdef NEWCODE
    Diag_Init();
#endif
    for(long k=0;k<N;k++){
        int mode=(k/5000)%4; int big=(k/20000)%3;   /* amplitude classes */
        int amp=big==0?3000:(big==1?20000:32767);
        FOC.sIabc.s16A=r16(amp);FOC.sIabc.s16B=r16(amp);FOC.sIabc.s16C=-FOC.sIabc.s16A-FOC.sIabc.s16B;
        FOC.sIAlBe.s16Alpha=r16(amp);FOC.sIAlBe.s16Beta=r16(amp);
        FOC.s16Vbus=(rnd()%10==0)?(int16_t)(rnd()%8000):(int16_t)(15000+rnd()%17000);
        pwm_gen.CompA=(int16_t)((int32_t)(rnd()%2600)-150);pwm_gen.CompB=(int16_t)(rnd()%2251);pwm_gen.CompC=(int16_t)(rnd()%2251);
        pwm_gen.N_halfPeriod=(rnd()%1000==0)?2000:2250; pwm_gen.Mode=rnd()%2;
        if((k/30000)%4==3){pwm_gen.CompA=(int16_t)(rnd()%40);pwm_gen.CompB=2250;pwm_gen.CompC=(int16_t)(2200+rnd()%50);FOC.s16Vbus=32000;}
        FluxObs_Capture(&FOC,(uint8_t)(rnd()%50!=0));
        if(rnd()%30000==0){FOC.s16CmdTheta=r16(32767);FOC.sPLL.s32Theta=FOC.s16CmdTheta<<16;FluxObs_BeginStartup(&FOC,&USER.sSpeed_Filt);}
        if(rnd()%40000==0)FluxObs_Reset();
        FluxObs_UpdateFeedback(&FOC);
        {int16_t e=perr(sFluxObs.s16PsiRAl,sFluxObs.s16PsiRBe,(int16_t)(FOC.sPLL.s32Theta>>16));
         if(mode&1){FOC.s16SpdCmd=(mode&2)?1000:-1000; FOC.s16CmdTheta=PLLST(e,&FOC.sPLL,FOC.s16SpdCmd,Q31(0.004));}
         else FOC.s16ThetaPLL=PLLRUN(e,&FOC.sPLL);}
        if(rnd()%7000==0){FOC.sPLL.s32I_1=(int32_t)rnd();FOC.sPLL.s32Speed=(int32_t)rnd();}
        uint64_t h=1469598103934665603ULL;
        h=fnv(h,&sFluxObs.s32PsiAlF,4);h=fnv(h,&sFluxObs.s32PsiBeF,4);h=fnv(h,&sFluxObs.sEalAvgMA.s32Acc,4);h=fnv(h,&sFluxObs.sEbeAvgMA.s32Acc,4);
        h=fnv(h,&sFluxObs.s16PsiRAl,2);h=fnv(h,&sFluxObs.s16PsiRBe,2);h=fnv(h,&sFluxObs.s16PsiRMag,2);h=fnv(h,&sFluxObs.s16Eal,2);h=fnv(h,&sFluxObs.s16Ebe,2);
        h=fnv(h,(const void*)&gFluxDiag.inputSatCount,4);h=fnv(h,(const void*)&gFluxDiag.fluxSatCount,4);h=fnv(h,(const void*)&gFluxDiag.voltageAlpha,2);h=fnv(h,(const void*)&gFluxDiag.voltageBeta,2);
        h=fnv(h,&FOC.sPLL,sizeof FOC.sPLL);h=fnv(h,&FOC.FluxFeedback,sizeof FOC.FluxFeedback);h=fnv(h,(const void*)&gFluxPLLDiag.satCount,4);h=fnv(h,(const void*)&gFluxPLLDiag.floorCount,4);
        fwrite(&h,8,1,f);
    }
    fclose(f);
    printf("fluxSat=%u inputSat=%u pllSat=%u floor=%u\n",(unsigned)gFluxDiag.fluxSatCount,(unsigned)gFluxDiag.inputSatCount,(unsigned)gFluxPLLDiag.satCount,(unsigned)gFluxPLLDiag.floorCount);
    return 0;
}
