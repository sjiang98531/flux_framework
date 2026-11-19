/* Host-side stand-in for the SC32 device headers (simulation only). */
#ifndef SIM_SC32_CONF_H
#define SIM_SC32_CONF_H
#include <stdint.h>
#include "basic.h"
#ifndef __IO
#define __IO volatile
#endif
#define __I volatile const
#define __O volatile
typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;
typedef struct { __IO uint32_t EPWM_CON; __IO uint32_t EPWM_CHN; __IO uint32_t EPWM_STS; uint32_t pad[9]; __IO uint32_t EPWM_DT[8]; } EPWM_TypeDef;
typedef struct { __IO uint32_t TIM_CON; __IO uint32_t TIM_CNT; __IO uint32_t TIM_RLD; } TIM_TypeDef;
typedef struct { __IO uint32_t ADC_CON; __IO uint32_t ADC_STS; } ADC_TypeDef;   /* ADC_STS：glue_isr 每拍置转换完成位（0xF20E004D 起快环直接读写标志） */
typedef struct { uint32_t pad[3]; __IO uint32_t WDT_CON; } WDT_TypeDef;
typedef struct { __IO uint32_t PIN; } GPIO_TypeDef;
typedef struct { __IO uint32_t ICSR; } SCB_TypeDef;     /* 只模拟 PendSV 挂起位 */
extern SCB_TypeDef sim_SCB;
#define SCB (&sim_SCB)
#define SCB_ICSR_PENDSVSET_Msk (1UL << 28)
typedef enum { PendSV_IRQn = -2 } IRQn_Type_sim;
#define NVIC_SetPriority(irq, pri) ((void)(irq), (void)(pri))
extern EPWM_TypeDef sim_EPWM; extern TIM_TypeDef sim_TIM0, sim_TIM3; extern ADC_TypeDef sim_ADC; extern WDT_TypeDef sim_WDT; extern GPIO_TypeDef sim_GPIOA;
#define EPWM (&sim_EPWM)
#define TIM0 (&sim_TIM0)
#define TIM3 (&sim_TIM3)
#define ADC  (&sim_ADC)
#define WDT  (&sim_WDT)
#define GPIOA (&sim_GPIOA)
#define EPWM_CHN_ENEPWM0 (0x1UL<<0)
#define EPWM_CHN_ENEPWM1 (0x1UL<<1)
#define EPWM_CHN_ENEPWM2 (0x1UL<<2)
#define EPWM_CHN_ENEPWM3 (0x1UL<<3)
#define EPWM_CHN_ENEPWM4 (0x1UL<<4)
#define EPWM_CHN_ENEPWM5 (0x1UL<<5)
#define EPWM_STS_CBCIF   (0x1UL<<2)
#define ADC_Flag_EOSIF0  1
#define TIM_Flag_TI      1
#define TIM_PWMChannel_PWMA 0
#define EPWM_SWAP_MODE4  4
#define PO_HIGH__NE_HIGH 0
FlagStatus TIM_GetFlagStatus(TIM_TypeDef *t, uint32_t f);
void TIM_ClearFlag(TIM_TypeDef *t, uint32_t f);
void TIM_PWMSetDuty(TIM_TypeDef *t, uint32_t ch, uint32_t duty);
#endif
