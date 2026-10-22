/*-------------------- Includes -----------------------*/
#include <stdint.h>
#include "basic.h"
#include "FOC_PMSM.h"

/*------------------- Private variables ---------------*/
#define 	SEVENMODE		1
#define 	FIVEMODE		0

typedef struct {
			int32_t	Alpha;
			int32_t	Beta;
			int16_t	CompA;
			int16_t	CompB;
			int16_t	CompC;
			uint8_t	Sector;
			uint8_t	Mode;
			int16_t	N_halfPeriod;
}PWM_GEN_T;


extern PWM_GEN_T pwm_gen;

extern void PWM_GEN_init(PWM_GEN_T *u1);
extern void PWM_GEN_calc(PWM_GEN_T *u1);
extern void Update_PWM(PWM_GEN_T *u1);
extern void Var_Filt(void);

extern void MCF_FlyStart_2R(FOC_TypeDef *pFOC);
extern void MCF_FlyStart_0BEMF(FOC_TypeDef *pFOC);

extern void MCF_ADV_Protect(FOC_TypeDef *pFOC);
extern void Fast_Boot_Current_Controller(FOC_TypeDef *pFOC);
extern uint8_t Fast_Boot_StartUp(FOC_TypeDef *pFOC);
extern uint8_t Fast_Boot_Spin(FOC_TypeDef *pFOC);
extern void DcBus_Ripple_suppress(MCFLIB_2_ALBE_T_S16 *sVAlBeIn,Q15_t s16Vbus, MCFLIB_2_ALBE_T_S16 *sVAlBe_Out);


