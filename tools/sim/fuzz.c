/* Random-input fuzz of observer + PLL (old vs new built separately; hash stream compared). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#ifdef NEWCODE
/* 新代码（0xF20E0007 起）：状态按模块分开（gMeas / gFoc / gEst …） */
#include "mc_types.h"
#include "meas.h"
#include "foc_core.h"
#include "est.h"
#include "speed_ctrl.h"
#include "supervisor.h"
#include "svpwm.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "diag.h"
#define PLLRUN FluxPLL_Run
#define PLLST  FluxPLL_RunStartup
static int16_t perr(int16_t a,int16_t b,int16_t th){return FluxPLL_PhaseError(a,b,th);}
Meas_TypeDef gMeas; Est_TypeDef gEst; SpeedCtrl_TypeDef gSpd; Supervisor_TypeDef gSup;
#define V_IABC   gMeas.iabc
#define V_IAB    gMeas.iab
#define V_VBUS   gMeas.vbus
#define V_THETA  gFoc.theta
#define V_SPDCMD gSpd.cmd
#define V_THPLL  gEst.thetaPll
#define V_PLL    gEst.pll
#define CAPTURE(v)      FluxObs_Capture(v)
#define BEGINSTARTUP()  FluxObs_BeginStartup()
#define UPDATEFB()      FluxObs_UpdateFeedback()
/* 原厂结构体里的 FluxFeedback（SMOType）只剩 s16Speed 有意义，其余恒 0 */
static SMOType feedbackView(void){SMOType v;memset(&v,0,sizeof v);v.s16Speed=gMeas.speedRaw;return v;}
#else
#include "motor_structure.h"
#include "pwm_gen.h"
#include "FluxObs.h"
#include "FluxPLL.h"
#define PLLRUN FluxPLL_Run
#define PLLST  FluxPLL_RunStartup
static int16_t perr(int16_t a,int16_t b,int16_t th){sAngle_Trig s=MLIB_SinCos(th);return MLIB_Mul_Q15(b,s.s16Cos)-MLIB_Mul_Q15(a,s.s16Sin);}
USER_TypeDef USER; FOC_TypeDef FOC; FastBoot_TypeDef FastBoot_Para; volatile unsigned int eM1_RunSubState;
#define V_IABC   FOC.sIabc
#define V_IAB    FOC.sIAlBe
#define V_VBUS   FOC.s16Vbus
#define V_THETA  FOC.s16CmdTheta
#define V_SPDCMD FOC.s16SpdCmd
#define V_THPLL  FOC.s16ThetaPLL
#define V_PLL    FOC.sPLL
#define CAPTURE(v)      FluxObs_Capture(&FOC,v)
#define BEGINSTARTUP()  FluxObs_BeginStartup(&FOC,&USER.sSpeed_Filt)
#define UPDATEFB()      FluxObs_UpdateFeedback(&FOC)
static SMOType feedbackView(void){return FOC.FluxFeedback;}
#endif
EPWM_TypeDef sim_EPWM; TIM_TypeDef sim_TIM0, sim_TIM3; ADC_TypeDef sim_ADC; WDT_TypeDef sim_WDT; GPIO_TypeDef sim_GPIOA;
static uint32_t rs=1;
static uint32_t rnd(void){rs^=rs<<13;rs^=rs>>17;rs^=rs<<5;return rs;}
static int16_t r16(int range){return (int16_t)((int32_t)(rnd()%(2*range+1))-range);}
static uint64_t fnv(uint64_t h,const void*p,size_t n){const uint8_t*b=p;while(n--){h^=*b++;h*=1099511628211ULL;}return h;}
int main(int argc,char**argv){
    long N=argc>1?atol(argv[1]):400000; FILE*f=fopen(argc>2?argv[2]:"fz.hash","wb");
    FluxPLL_Init(&V_PLL); FluxObs_Init();
#ifdef NEWCODE
    Diag_Init();
#endif
    for(long k=0;k<N;k++){
        int mode=(k/5000)%4; int big=(k/20000)%3;   /* amplitude classes */
        int amp=big==0?3000:(big==1?20000:32767);
        V_IABC.s16A=r16(amp);V_IABC.s16B=r16(amp);V_IABC.s16C=-V_IABC.s16A-V_IABC.s16B;
        V_IAB.s16Alpha=r16(amp);V_IAB.s16Beta=r16(amp);
        V_VBUS=(rnd()%10==0)?(int16_t)(rnd()%8000):(int16_t)(15000+rnd()%17000);
        pwm_gen.CompA=(int16_t)((int32_t)(rnd()%2600)-150);pwm_gen.CompB=(int16_t)(rnd()%2251);pwm_gen.CompC=(int16_t)(rnd()%2251);
        pwm_gen.N_halfPeriod=(rnd()%1000==0)?2000:2250; pwm_gen.Mode=rnd()%2;
        if((k/30000)%4==3){pwm_gen.CompA=(int16_t)(rnd()%40);pwm_gen.CompB=2250;pwm_gen.CompC=(int16_t)(2200+rnd()%50);V_VBUS=32000;}
        CAPTURE((uint8_t)(rnd()%50!=0));
        if(rnd()%30000==0){V_THETA=r16(32767);V_PLL.s32Theta=V_THETA<<16;BEGINSTARTUP();}
        if(rnd()%40000==0)FluxObs_Reset();
        UPDATEFB();
        {int16_t e=perr(sFluxObs.s16PsiRAl,sFluxObs.s16PsiRBe,(int16_t)(V_PLL.s32Theta>>16));
         if(mode&1){V_SPDCMD=(mode&2)?1000:-1000; V_THETA=PLLST(e,&V_PLL,V_SPDCMD,Q31(0.004));}
         else V_THPLL=PLLRUN(e,&V_PLL);}
        if(rnd()%7000==0){V_PLL.s32I_1=(int32_t)rnd();V_PLL.s32Speed=(int32_t)rnd();}
        uint64_t h=1469598103934665603ULL;
        h=fnv(h,&sFluxObs.s32PsiAlF,4);h=fnv(h,&sFluxObs.s32PsiBeF,4);h=fnv(h,&sFluxObs.sEalAvgMA.s32Acc,4);h=fnv(h,&sFluxObs.sEbeAvgMA.s32Acc,4);
        h=fnv(h,&sFluxObs.s16PsiRAl,2);h=fnv(h,&sFluxObs.s16PsiRBe,2);h=fnv(h,&sFluxObs.s16PsiRMag,2);h=fnv(h,&sFluxObs.s16Eal,2);h=fnv(h,&sFluxObs.s16Ebe,2);
        h=fnv(h,(const void*)&gFluxDiag.inputSatCount,4);h=fnv(h,(const void*)&gFluxDiag.fluxSatCount,4);h=fnv(h,(const void*)&gFluxDiag.voltageAlpha,2);h=fnv(h,(const void*)&gFluxDiag.voltageBeta,2);
        h=fnv(h,&V_PLL,sizeof V_PLL);{SMOType fb=feedbackView();h=fnv(h,&fb,sizeof fb);}h=fnv(h,(const void*)&gFluxPLLDiag.satCount,4);h=fnv(h,(const void*)&gFluxPLLDiag.floorCount,4);
        fwrite(&h,8,1,f);
    }
    fclose(f);
    printf("fluxSat=%u inputSat=%u pllSat=%u floor=%u\n",(unsigned)gFluxDiag.fluxSatCount,(unsigned)gFluxDiag.inputSatCount,(unsigned)gFluxPLLDiag.satCount,(unsigned)gFluxPLLDiag.floorCount);
    return 0;
}
