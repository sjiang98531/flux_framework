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
#include "cfg_board.h"      /* flux_framework: PWM_FREQ / DEAD_TIME */

/******************************* 硬件用户配置参数 ********************************/
/*------------------------控制器参数相关定义------------------------*/

#define  EPWM_SWAP_FUNCTION            EPWM_SWAP_MODE4                    /* 芯片内置预驱 管脚交换 */
#define  PRE_DRIVER_POLARITY           PO_HIGH__NE_HIGH                   /* 预驱预动极性伞よ置 上管高电平有效，下管高电平有效 */

#define MCU_FREQUENCY                (72000000)      /* Hz 赫兹 MCU系统时钟频率       	*/
#define EPWM_FREQUENCY               (PWM_FREQ)         /* Hz 赫兹 电机驱动载波EPWM频率  		*/
#define POWER_DCVoltage              (12.0)          /* V  伏   驱动器输入直流电源电压 		*/
#define MCU_VDD                      (5.0)           /* V  伏   MCU供电电压          	*/
#define ADC_Vref                     (5.0)           /* V  伏   ADC基准值        			*/
#define OPA_Gain                     (3.139)//(14.70588)        /* 运放放大倍数        	*/
#define SAMPLE_RShunt                (0.10)           /* Ω 欧姆 母线电流采样电阻      	*/
#define IBUS_HOCP_Value              (4.3278)         /* V  伏 母线电流硬件母线电阻电压值  	*/
#define PHASE_CURRENT_PEAK           (8.5)           /* A  安培 相线峰值电流最大值     	*/


/*------------------------系统定时器参数相关定义------------------------*/
#define MCU_PER_TIME                 (float)(1000000000.0/MCU_FREQUENCY)             /* MCU工作时间基准 单位:ns  */
							         
#define EPWM_PRIOD_LOAD              (u16)(MCU_FREQUENCY/(2*EPWM_FREQUENCY) - 1)     /* EPWM 电机驱动载波EPWM周期装载值 中心对齐计数模式 */

#define EPWAM_TIME_C                 (float)(1000000000.0/EPWM_FREQUENCY/2/(EPWM_PRIOD_LOAD+1)) //单位PWM装载值表示的时间  单位：ns
							         
#define EPWM_DEADTIME                (float)(DEAD_TIME)                                    /* EPWM 电机驱动载波EPWM死区时间 单位：ns 建议范围：800~3000nS */
//#define EPWM_DEADTIME_LOAD           (u16)((EPWM_DEADTIME/((4*1000000000)/MCU_FREQUENCY))+1)	 /* EPWM 电机驱动载波EPWM死区时间装载值 */
//#define EPWM_DEADTIME_LOAD           (u16)((EPWM_DEADTIME/((4*100000)/(MCU_FREQUENCY/1000)))+1)	 /* EPWM 电机驱动载波EPWM死区时间装载值 */
#define EPWM_DEADTIME_LOAD           (u16)((EPWM_DEADTIME/(4*(1000000000/MCU_FREQUENCY)))+1)	 /* EPWM 电机驱动载波EPWM死区时间装载值 */
#define SOFTWARE_DEADTIME            (float)(1000)       //软件死驱时间 单位：ns
#define SOFTWARE_TNOISE_TIME         (float)(500)      //振荡时间  单位：ns
#define SAMPLE_OVERTIME              (float)(9*MCU_PER_TIME)          //采样加转换完成时间
#define ADCTRIG_FIRST_TIME           (u16)(EPWM_PRIOD_LOAD-(u16)((SAMPLE_OVERTIME*2+SOFTWARE_TNOISE_TIME+SOFTWARE_DEADTIME)/EPWAM_TIME_C + 1))
#define ADCTRIG_SECOND_TIME          (u16)(EPWM_PRIOD_LOAD-(u16)(SAMPLE_OVERTIME/EPWAM_TIME_C + 1))
#define ADCTRIG_THIRD_TIME           (u16)(EPWM_PRIOD_LOAD-(u16)((SOFTWARE_TNOISE_TIME+SOFTWARE_DEADTIME)/EPWAM_TIME_C + 1))
#define ADCTRIG_FOURTH_TIME          (u16)(EPWM_PRIOD_LOAD-(u16)((SAMPLE_OVERTIME*2+SOFTWARE_TNOISE_TIME+SOFTWARE_DEADTIME)/EPWAM_TIME_C + 1))
#define Half_Window_Time             (u16)(((SAMPLE_OVERTIME+SOFTWARE_TNOISE_TIME)/EPWAM_TIME_C + 1)/2)

#define EPWM_LIMITDUTY               (u16)(1000)	                                 /* EPWM 电机驱动载波EPWM最小输出占空比限制值 单位：ns 建议范围：800~3000nS*/
#define EPWM_MINIDUTY                (u16)(EPWM_LIMITDUTY+EPWM_DEADTIME)	         /* EPWM 电机驱动载波EPWM最小输出占空比 单位：ns */
#define EPWM_MINIDUTYCNT             (u16)(EPWM_MINIDUTY/MCU_PER_TIME)	             /* EPWM 电机驱动载波EPWM最小输出占空比计数值  */

#define TASK_BASETIM_FREQUENCY       (u16)(1000)     /* 系统任务时基定时器频率 单位:Hz */
#define HALL_PCAP_FREQUENCY          (u16)(20)       /* PCAP Hall定时器捕获频率     单位:Hz */
#define DAC_Range                    (1024)          /* AD/mV AD每毫伏 MCU DAC量程10位    */						         
#define IBUS_HOCP_DACValue           (u16)((DAC_Range*IBUS_HOCP_Value)/ADC_Vref)      



/*------------------------ADC采样及运算放大器相关定义------------------------*/
#define ADC_Channel_OP0		    ADC_Channel_0       /* OP0输出采样通道   芯片默认不可更改 */ 
#define ADC_Channel_OP1		    ADC_Channel_1       /* OP1输出采样通道   芯片默认不可更改 */
#define ADC_Channel_OP2		    ADC_Channel_6       /* OP2输出采样通道   芯片默认不可更改 */ 


#define ADC_Channel_IU		    ADC_Channel_OP1     /* U相电流采样通道           --根据电路实际情况选择 */ 
#define ADC_Channel_IV		    ADC_Channel_OP0     /* V相电流采样通道           --根据电路实际情况选择 */
#define ADC_Channel_IW		    ADC_Channel_OP0       /* W相电流采样通道           --根据电路实际情况选择 */
#define ADC_Channel_IPEAK	·	ADC_Channel_OP2     /* 母线峰值电流采样通道      --根据电路实际情况选择 */
#define ADC_Channel_IAVG		ADC_Channel_7
#define ADC_Channel_VSP			ADC_Channel_11       /* 电位器电压采样通道        --根据电路实际情况选择 */
#define ADC_Channel_VBUS		ADC_Channel_3       /* 母线电压采样通道          --根据电路实际情况选择 */
#define	ADC_Channel_IBUS		ADC_Channel_8


/******************************* 硬件配置内部参数 ********************************/
/*------------------------ 定时器相关定义 ------------------------*/
#define TIM3_PSC_LOAD           (u16)(1)                                                               /* TIM3 时钟分频装载值     0表示不分频    */
#define TASK_BASETIM_PRIOD_LOAD (u16)(65536-((MCU_FREQUENCY/((TIM3_PSC_LOAD + 1)*TASK_BASETIM_FREQUENCY))-1))  /* TIM3  周期装载值 16bit max：65535      */

/*------------------------ PWM驱动相关定义 ------------------------*/
#define PWMA_Enb							(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM0)    /* PWM上下桥互补模式下，U相下桥关闭输出 */
#define PWMA_Dis   		 					(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM0)    /* PWM上下桥互补模式下，U相下桥打开输出 */
#define PWMB_Enb							(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM2)    /* PWM上下桥互补模式下，V相下桥关闭输出 */
#define PWMB_Dis    						(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM2)    /* PWM上下桥互补模式下，V相下桥打开输出 */
#define PWMC_Enb    						(EPWM->EPWM_CHN |=  EPWM_CHN_ENEPWM4)    /* PWM上下桥互补模式下，W相下桥关闭输出 */
#define PWMC_Dis    						(EPWM->EPWM_CHN &= ~EPWM_CHN_ENEPWM4)    /* PWM上下桥互补模式下，W相下桥打开输出 */

#define PWMABC_Enb   						(EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)(EPWM_CHN_ENEPWM0|EPWM_CHN_ENEPWM2|EPWM_CHN_ENEPWM4))))   /* PWM上下桥互补模式下，WVW下桥打开输出 */
#define PWMABC_Dis  						(EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)(EPWM_CHN_ENEPWM0|EPWM_CHN_ENEPWM2|EPWM_CHN_ENEPWM4))))  /* PWM上下桥互补模式下，UVW下桥关闭输出 */

#define PWMAN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM1))     /* PWM上下桥互补模式下，u相下桥使能 */
#define PWMAN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM1))    /* PWM上下桥互补模式下，U相下桥关闭 */
#define PWMBN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM3))     /* PWM上下桥互补模式下，b相下桥使能 */
#define PWMBN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM3))    /* PWM上下桥互补模式下，b相下桥关闭 */
#define PWMCN_Enb   						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)EPWM_CHN_ENEPWM5))     /* PWM上下桥互补模式下，W相下桥关闭输出 */
#define PWMCN_Dis   						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)EPWM_CHN_ENEPWM5))    /* PWM上下桥互补模式下，W相下桥打开输出 */

#define PWMABCN_Enb  						EPWM->EPWM_CHN |= (uint16_t)(((uint16_t)(EPWM_CHN_ENEPWM1|EPWM_CHN_ENEPWM3|EPWM_CHN_ENEPWM5)))   /* PWM上下桥互补模式下，WVW下桥打开输出 */
#define PWMABCN_Dis 						EPWM->EPWM_CHN &= (uint16_t)(~((uint16_t)(EPWM_CHN_ENEPWM1|EPWM_CHN_ENEPWM3|EPWM_CHN_ENEPWM5)))  /* PWM上下桥互补模式下，UVW下桥关闭输出 */

#define DISABLE_PWMOUT() 				PWMABC_Dis;	PWMABCN_Dis  /* PWM上下桥互补模式下，上桥关闭输出，下桥关闭输出 */
#define ENABLE_PWMOUT() 				PWMABC_Enb;	PWMABCN_Enb  /* PWM上下桥互补模式下，上桥打开输出，下桥打开输出 */
//------------------------------------------------------------------------               
#define IO_UL_OFF() 						(GPIOA->PIN   = (1<<4))    /* 上桥PWM下桥IO模式下，U相下桥关闭输出 */
#define IO_UL_ON()							(GPIOA->PIN   = (1<<4))    /* 上桥PWM下桥IO模式下，U相下桥打开输出 */
#define IO_VL_OFF() 						(GPIOA->PIN   = (1<<6))    /* 上桥PWM下桥IO模式下，V相下桥关闭输出 */
#define IO_VL_ON()							(GPIOA->PIN   = (1<<6))    /* 上桥PWM下桥IO模式下，V相下桥打开输出 */
#define IO_WL_OFF() 						(GPIOA->PIN   = (1<<8))    /* 上桥PWM下桥IO模式下，W相下桥关闭输出 */
#define IO_WL_ON()							(GPIOA->PIN   = (1<<8))    /* 上桥PWM下桥IO模式下，W相下桥打开输出 */

#define IO_UVWL_ON()    				IO_UL_ON();IO_VL_ON();IO_WL_ON()      /* 上桥PWM下桥IO模式下，WVW下桥打开输出 */
#define IO_UVWL_OFF()   				IO_UL_OFF();IO_VL_OFF();IO_WL_OFF()   /* 上桥PWM下桥IO模式下，UVW下桥关闭输出 */

#define IO_PWM_DRIVE_OFF()               UVWH_OFF();IO_UVWL_OFF()    /* 上桥PWM下桥IO模式下，WVW上桥关闭输出，WVW下桥关闭输出 */
#define IO_PWM_DRIVE_BRAKR()             UVWH_OFF();IO_UVWL_ON()     /* 上桥PWM下桥IO模式下，WVW上桥关闭输出，WVW下桥打开输出 */
#define IO_PWM_DRIVE_UVWH_ON_UVWL_OFF()  UVWH_ON();IO_UVWL_OFF()     /* 上桥PWM下桥IO模式下，WVW上桥打开输出，WVW下桥关闭输出 */
#define IO_PWM_DRIVE_UVWH_ON_UVWL_ON()   UVWH_ON();IO_UVWL_ON()      /* 上桥PWM下桥IO模式下，WVW上桥打开输出，WVW下桥打开输出 */


#endif /* __HARDWARE_CONFIG_H */
/*********** (C) COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD  *************/
/* ------------------------------END OF FILE------------------------------------ */
