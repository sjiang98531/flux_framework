/************************************************************
//  Copyright (c)
//	FileName	 : main.c
//	Module Function	:
//  Instructions : Contains the MCU initialization function and its H file
*******************Includes************************************************************************/
#include "sc32_conf.h"
#include "SC_Init.h"
#include "SC_it.h"
#include "stdio.h"
#include "includes.h"
#include "user_function.h"
#include "StateMachine.h"
#include "mc_customize.h"

uint16_t HoldTime = 0;
uint16_t Run_Curr_Target_A = 0;
uint16_t Run_Speed_Target_rpm = 0;
uint16_t Run_Power_Target_W = 0;
float Target_Speed = 2.0f;
int main(void)
{
  Motor_Hardware_Init();  /* 硬件外设初始化 */
	Init_Parameter();

	#if (PWM_DEBUG == 0)
	{
		Motor_NVIC_Config();    /* 中断优先级使能 2057 15222 */
	}
    #else
	PWMABCN_Dis;
	PWMABC_Dis;

	EPWM->EPWM_DT[0] = USER.u16PWMPeriod>>2; //对应板子V相
//			EPWM->EPWM_DT[1] = USER.u16PWMPeriod>>2;
	EPWM->EPWM_DT[2] = USER.u16PWMPeriod>>2;//对应板子V相
//			EPWM->EPWM_DT[3] = USER.u16PWMPeriod>>2;
	EPWM->EPWM_DT[4] = USER.u16PWMPeriod>>2;//对应板子U相
//			EPWM->EPWM_DT[5] = USER.u16PWMPeriod>>2;

	PWMAN_Enb;
	PWMBN_Enb;
	PWMCN_Enb;
	
	PWMA_Enb;
	PWMB_Enb;
	PWMC_Enb;
	
	while(1)
	{
		/*IWDG_ReloadCounter*/
		WDT->WDT_CON |= 0X01;//喂狗
	}
	#endif

	while(1)
	{
		/*IWDG_ReloadCounter*/
//		WDT->WDT_CON |=0X01;//喂狗
	
		if(USER.bSlowLoopFlag == 1)
		{
			/* Slow Loop Statemachine */
			USER.bSlowLoopFlag =  0;
		
			s_STATE_SLOW[eM1_MainState]();
//			HoldTime++;
//            if(HoldTime >= 15000)
//			{
//				HoldTime = 0;
//			}
//			
//			if(HoldTime < 4000)
//			{
//				USER.s16VspCmd = Q15(100.0/POWER_MAX);
//			}
//			else
//			{
//				USER.s16VspCmd = 0;
//			}
            
//			if(HoldTime >= 35000)
//			{
//				HoldTime = 0;
//			}
//			
//			if(HoldTime < 10000)
//			{
//				USER.s16VspCmd = Q15(920.0/SPEED_MAX);
//			}
//			else if(HoldTime < 14500)
//			{
//				USER.s16VspCmd = Q15(360.0/SPEED_MAX);
//			}
//			else if(HoldTime < 15000)
//			{
//				USER.s16VspCmd = 0;
//			}
//			else if(HoldTime < 25000)
//			{
//				USER.s16VspCmd = -Q15(400.0/SPEED_MAX);
//			}
//			else if(HoldTime < 30000)
//			{
//				USER.s16VspCmd = -Q15(200.0/SPEED_MAX);
//			}
//			else
//			{
//				USER.s16VspCmd = 0;
//			}
		}
		Run_Curr_Target_A = Q15(Target_Speed/I_MAX);// I_MAX SPEED_MAX POWER_MAX
        Run_Speed_Target_rpm = Q15(Target_Speed/SPEED_MAX);// I_MAX SPEED_MAX POWER_MAX
        Run_Power_Target_W = Q15(Target_Speed/POWER_MAX);// I_MAX SPEED_MAX POWER_MAX
	}
}
