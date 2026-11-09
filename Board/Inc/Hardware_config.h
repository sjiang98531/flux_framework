/*******************************************************************************
 * 版权所有 (C) 2023, COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD
 *
 * 文件名称： Hardware_config.h
 * 内容摘要： 硬件配置相关文件
 * 其它说明： 无
 * 当前版本： V1.0
 * 完成日期: 2023年6月28日
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------- Includes -----------------------*/
#include "includes.h"
#include "cfg_board.h"      /* 驱动板物理参数（时钟、PWM 频率、死区、ADC 基准、采样电阻、运放增益）统一定义在 cfg_board.h */
/* 本文件只描述驱动板的连接方式：EPWM / 定时器装载值、硬件过流 DAC 门限、ADC 通道映射、PWM 通道开关宏（供 hal_sc32r602.c 使用）。 */

/******************************* 硬件用户配置参数 ********************************/
/*------------------------控制器参数相关定义------------------------*/

#define MCU_FREQUENCY                ((int32_t)SYSCLK_HSI_72MHz)      /* Hz 系统时钟（取自 cfg_board.h，转为整数：死区 / 定时器装载值的整数除法依赖它） */
#define EPWM_FREQUENCY               (PWM_FREQ)         /* Hz 赫兹 电机驱动载波EPWM频率  		*/
#define ADC_Vref                     (ADC_REFV)      /* V  ADC 基准电压（取自 cfg_board.h） */
#define IBUS_HOCP_Value              (4.3278)         /* V  伏 母线电流硬件母线电阻电压值  	*/

/*------------------------系统定时器参数相关定义------------------------*/
#define EPWM_PRIOD_LOAD              (u16)(MCU_FREQUENCY/(2*EPWM_FREQUENCY) - 1)     /* EPWM 电机驱动载波EPWM周期装载值 中心对齐计数模式 */

#define EPWM_DEADTIME                (float)(DEAD_TIME)                                    /* EPWM 电机驱动载波EPWM死区时间 单位：ns 建议范围：800~3000nS */
#define EPWM_DEADTIME_LOAD           (u16)((EPWM_DEADTIME/(4*(1000000000/MCU_FREQUENCY)))+1)	 /* EPWM 电机驱动载波EPWM死区时间装载值 */

#define TASK_BASETIM_FREQUENCY       (u16)(1000)     /* 系统任务时基定时器频率 单位:Hz */
#define DAC_Range                    (1024)          /* AD/mV AD每毫伏 MCU DAC量程10位    */						         
#define IBUS_HOCP_DACValue           (u16)((DAC_Range*IBUS_HOCP_Value)/ADC_Vref)      

/*------------------------ADC采样及运算放大器相关定义------------------------*/
#define ADC_Channel_OP0		    ADC_Channel_0       /* OP0输出采样通道   芯片默认不可更改 */ 
#define ADC_Channel_OP1		    ADC_Channel_1       /* OP1输出采样通道   芯片默认不可更改 */
#define ADC_Channel_OP2		    ADC_Channel_6       /* OP2输出采样通道   芯片默认不可更改 */ 

#define ADC_Channel_IU		    ADC_Channel_OP1     /* U相电流采样通道           --根据电路实际情况选择 */ 
#define ADC_Channel_IV		    ADC_Channel_OP0     /* V相电流采样通道           --根据电路实际情况选择 */
#define ADC_Channel_IAVG		ADC_Channel_7
#define ADC_Channel_VSP			ADC_Channel_11       /* 电位器电压采样通道        --根据电路实际情况选择 */
#define ADC_Channel_VBUS		ADC_Channel_3       /* 母线电压采样通道          --根据电路实际情况选择 */

/******************************* 硬件配置内部参数 ********************************/
/*------------------------ 定时器相关定义 ------------------------*/
#define TIM3_PSC_LOAD           (u16)(1)                                                               /* TIM3 时钟分频装载值     0表示不分频    */
#define TASK_BASETIM_PRIOD_LOAD (u16)(65536-((MCU_FREQUENCY/((TIM3_PSC_LOAD + 1)*TASK_BASETIM_FREQUENCY))-1))  /* TIM3  周期装载值 16bit max：65535      */

/*------------------------ PWM驱动相关定义 ------------------------*/
#define PWMA_Enb							(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM0)    /* PWM 互补模式下，U 相上桥使能输出 */
#define PWMA_Dis   		 					(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM0)    /* PWM 互补模式下，U 相上桥关闭输出 */
#define PWMB_Enb							(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM2)    /* PWM 互补模式下，V 相上桥使能输出 */
#define PWMB_Dis    						(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM2)    /* PWM 互补模式下，V 相上桥关闭输出 */
#define PWMC_Enb    						(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM4)    /* PWM 互补模式下，W 相上桥使能输出 */
#define PWMC_Dis    						(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM4)    /* PWM 互补模式下，W 相上桥关闭输出 */

#define PWMABC_Enb   						(EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)(EPWM_CHN_ENEPWM0|EPWM_CHN_ENEPWM2|EPWM_CHN_ENEPWM4))))   /* PWM 互补模式下，UVW 三相上桥使能输出 */
#define PWMABC_Dis  						(EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)(EPWM_CHN_ENEPWM0|EPWM_CHN_ENEPWM2|EPWM_CHN_ENEPWM4))))  /* PWM 互补模式下，UVW 三相上桥关闭输出 */

#define PWMAN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM1))     /* PWM 互补模式下，U 相下桥使能输出 */
#define PWMAN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM1))    /* PWM 互补模式下，U 相下桥关闭输出 */
#define PWMBN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM3))     /* PWM 互补模式下，V 相下桥使能输出 */
#define PWMBN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM3))    /* PWM 互补模式下，V 相下桥关闭输出 */
#define PWMCN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM5))     /* PWM 互补模式下，W 相下桥使能输出 */
#define PWMCN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM5))    /* PWM 互补模式下，W 相下桥关闭输出 */

#define PWMABCN_Enb  						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)(EPWM_CHN_ENEPWM1|EPWM_CHN_ENEPWM3|EPWM_CHN_ENEPWM5)))   /* PWM 互补模式下，UVW 三相下桥使能输出 */
#define PWMABCN_Dis 						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)(EPWM_CHN_ENEPWM1|EPWM_CHN_ENEPWM3|EPWM_CHN_ENEPWM5)))  /* PWM 互补模式下，UVW 三相下桥关闭输出 */

#define DISABLE_PWMOUT() 				PWMABC_Dis;	PWMABCN_Dis  /* 上桥、下桥全部关闭输出 */
#define ENABLE_PWMOUT() 				PWMABC_Enb;	PWMABCN_Enb  /* 上桥、下桥全部使能输出（互补） */

#endif /* __HARDWARE_CONFIG_H */
/*********** (C) COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD  *************/
/* ------------------------------END OF FILE------------------------------------ */
