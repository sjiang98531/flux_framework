//************************************************************
//  Copyright (c)
//	FileName	: SC_Init.h
//	Function	: Contains the MCU initialization function and its H file
//  Instructions        :  
//*************************************************************

#ifndef _SC_Init_H_
#define _SC_Init_H_

#include "sc32_conf.h"

void SC_Init(void);
void SystemClock_Init(void);
void SC_OPTION_Init(void);
void SC_GPIO_Init(void);
void SC_UART1_Init(void);
void SC_UART2_Init(void);
void SC_UART3_Init(void);
void SC_TIM0_Init(void);
void SC_TIM1_Init(void);
void SC_TIM2_Init(void);
void SC_TIM3_Init(void);
void SC_TIM4_Init(void);
void SC_TIM5_Init(void);
void SC_TIM6_Init(void);
void SC_TIM7_Init(void);
void SC_PWM_Init(void);
void SC_PWM1_Init(void);
void SC_EPWM_Init(void);
void SC_INT_Init(void);
void SC_VREF_Init(void);
void SC_ADC_Init(void);
void SC_OP0_Init(void);
void SC_OP1_Init(void);
void SC_OP2_Init(void);
void SC_IAP_Init(void);
void SC_SPI0_Init(void);
void SC_SPI1_Init(void);
void SC_TWI0_Init(void);
void SC_TWI1_Init(void);
void SC_BTM_Init(void);
void SC_CRC_Init(void);
void SC_WDT_Init(void);
void SC_LCD_LED_Init(void);
void SC_PCAP_Init(void);
void SC_CMP0_Init(void);
void SC_CMP3_Init(void);
void SC_DAC_Init(void);
void SC_MR_Init(void);
void SC_QEP_Init(void);
void WDG_init(void); 

void Motor_Hardware_Init(void);
void Motor_NVIC_Config(void);

void SoftDelay_50MS(void);


#endif

/***********************user_code_area*********************/
/*<UserCodeStart>*/
/*<UserCodeEnd>*/
/**********************.user_code_area.********************/
