//************************************************************
//  Copyright (c)  
//	FileName	   : SC_it.h
//	Author		   : 
//	Function	   : Interrupt service program header file
//	Local Functions:
//  Date           : 2022年5月24日
// 	Version		   : V1.000
//*************************************************************
#ifndef _SC_IT_H_
#define	_SC_IT_H_

#include "sc32_conf.h"

void INT0_IRQHandler(void);
void INT1_7_IRQHandler(void);
void INT8_11_IRQHandler(void);
void INT12_15_IRQHandler(void);
void RCC_IRQHandler(void);
void BTM_IRQHandler(void);
void UART0_2_IRQHandler(void);
void UART1_3_IRQHandler(void);
void SPI0_IRQHandler(void);
void SPI1_IRQHandler(void);
void DMA0_IRQHandler(void);
void DMA1_IRQHandler(void);
void DMA2_IRQHandler(void);
void DMA3_IRQHandler(void);
void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
void TIMER2_IRQHandler(void);
void TIMER3_IRQHandler(void);
void TIMER4_5_IRQHandler(void);
void TIMER6_7_IRQHandler(void);
void PWM0_IRQHandler(void);
void LEDPWM_IRQHandler(void);
void TWI0_IRQHandler(void);
void TWI1_IRQHandler(void);
void ADC_IRQHandler(void);
void CMP_IRQHandler(void);
void TK_IRQHandler(void);

#endif

/***********************user_code_area*********************/
/*<UserCodeStart>*/
/*<UserCodeEnd>*/
/**********************.user_code_area.********************/
