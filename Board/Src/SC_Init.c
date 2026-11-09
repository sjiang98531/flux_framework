//************************************************************
//  Copyright (c)
//	FileName	   : SC_Init.c
//	Function	   : Contains the MCU initialization function and its C file
//  Instructions   :
//*************************************************************

#include "SC_Init.h"	// MCU initialization header file, including all firmware library header files

#include "SC_itExtern.h"

#include "includes.h"

//*************************************************************

/*****************************************************
*函数名称: SC_Init
*函数功能: MCU初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_Init(void)
{
    /*write initial function here*/
}

/****************************************************************
	函数名: Motor_Hardware_Init
	描述:   电机控制硬件参数初始化
	输入:   无
	输出:   无
****************************************************************/
void Motor_Hardware_Init(void)
{
    SystemClock_Init();		         /* 初始化系统时钟HSI 72M */

    SC_VREF_Init();              		/* 初始化基准电压 */
    SC_ADC_Init();                 	/* 初始化ADC */
    SC_CMP3_Init();                	/* 初始化CMPx */
    SC_DAC_Init();                 	/* 初始化DAC */
    SC_OP0_Init();                 	/* 初始化OP0 */
    SC_OP1_Init();                	/* 初始化OP1 */
	SC_OP2_Init();
	
    SC_EPWM_Init();									/* 初始化EPWM */
	SC_TIM3_Init();
	SC_TIM0_Init();                	/* FG-PWM输出*/
	SC_GPIO_Init();	               	/* 初始化GPIO */
    SC_MR_Init();                  	/* 开启硬件加速模块 */
	
	SoftDelay_50MS();              	/* 延时50ms等待系统稳定*/
	SoftDelay_50MS();             	/* 延时50ms等待系统稳定*/	
	SoftDelay_50MS();              	/* 延时50ms等待系统稳定*/
	SoftDelay_50MS();              	/* 延时50ms等待系统稳定*/
	
//	WDG_init();       							/* 看门狗初始化 */
}



/*******************************************************************************
* 函数名称: Motor_NVIC_Config
* 功能描述: 电机中断配置
* 输入参数: void
* 返回参数: 无
********************************************************************************/
void Motor_NVIC_Config(void)
{
    NVIC_SetPriority(EPWM_IRQn, 0);             /* 配置EPWM 故障中断优先级 */
	NVIC_SetPriority(ADC_IRQn, 1);              /* 配置ADC中断优先级 */
	
    /* 设置中断使能 */
    NVIC_EnableIRQ(EPWM_IRQn);                  /* 使能EPWM中断 */
  	NVIC_EnableIRQ(ADC_IRQn);                  	/* 使能ADC中断 */
}

/*****************************************************
*函数名称: SystemClock_Init
*函数功能: MCU初始化函数
*入口参数：void
*出口参数：void
*****************************************************/

void SystemClock_Init(void)
{
    RCC->RCC_KEY = 0XFF;
    RCC->RCC_CFG0 |= (2 << RCC_CFG0_WAIT_Pos); // Flash 等待周期 = 2 
    RCC->RCC_CFG0 &= 0xFFFFEFFF; // Flash 等待周期配置 	
    RCC->RCC_CFG0  |= 0x00004000;  //切换成72MHz开关使能
	RCC->RCC_CFG0  |= 0x00000200;  //分频系数为1
    RCC->RCC_KEY = 0XFF;
	RCC->RCC_CFG0  |= 0x00000080; // 上面设置使能
	while((RCC->RCC_CFG0&0x00000080)==0x00000000){};// 等待时钟切换完成
    RCC_HCLKConfig(RCC_SYSCLK_Div1);            //HLCK 1分频
}

/*****************************************************
*函数名称: SC_OPTION_Init
*函数功能: OPTION配置初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_OPTION_Init(void)
{
    /*OPTION_Init write here*/
}

/*****************************************************
*函数名称: SC_GPIO_Init
*函数功能: GPIO初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_GPIO_Init(void)
{
		//----------------------配置EPWM口-----------------------------//
    /*GPIO_Init write here*/
    GPIO_InitTypeDef GPIO_InitStruct;									//GPIO结构体,PWM需要EPWM产生，通过GPIO口输出
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;   //EPWM
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
		
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;   	//T0PWMA
	GPIO_Init(GPIOC, &GPIO_InitStruct);

    PA_BIT(3) = 0; 
	PA_BIT(4) = 0; 
	PA_BIT(5) = 0; 
	PA_BIT(6) = 0; 
	PA_BIT(7) = 0; 
	PA_BIT(8) = 0;
	PC_BIT(6) = 0;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;   //TX2
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
    PA_BIT(0) = 0;
}


/*****************************************************
*函数名称: SC_UART1_Init
*函数功能: UART1初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_UART1_Init(void)
{

}

/*****************************************************
*函数名称: SC_UART2_Init
*函数功能: UART2初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_UART2_Init(void)
{

}


/*****************************************************
*函数名称: SC_UART3_Init
*函数功能: UART3初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_UART3_Init(void)
{
    /*UART3_Init write here*/
}

/*****************************************************
*函数名称: SC_TIM0_Init
*函数功能: TIMER0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM0_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_PWM_InitTypeDef TIM_PWM_InitStruct;

	/* 使能TIM0时钟 */
	RCC_APB0Cmd(ENABLE);
	RCC_APB0PeriphClockCmd(RCC_APB0Periph_TIM0, ENABLE);

	/* TIM配置结构体初始化 */
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_PWMStructInit(&TIM_PWM_InitStruct);

	/* 设置TIM时基配置结构体 */
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM工作在向上计数模式
	TIM_TimeBaseInitStruct.TIM_EXENX = TIM_EXENX_Disable;					//外部重加载管脚失能
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM_PRESCALER_128;			//TIM分频为128分频	562.5KHZ
	TIM_TimeBaseInitStruct.TIM_Preload = 5625;										//设置TIM的定时器频率为562.5KHZ/100HZ
	TIM_TimeBaseInitStruct.TIM_WorkMode = TIM_WorkMode_Timer;			//TIM工作在定时器模式，时钟源为APB0时钟

	/* TIM时基配置 */
	TIM_TIMBaseInit(TIM0, &TIM_TimeBaseInitStruct);

	/* 设置TIM PWM配置结构体 */
	TIM_PWM_InitStruct.TIM_PWMOutputChannl = TIM_PWMChannel_PWMA;	//TIM1 PWMB输出使能

	/* TIM PWM配置 */
	TIM_PWMInit(TIM0,&TIM_PWM_InitStruct);
	TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,0);				//占空比设置为0%

	/* TIM外设初始化 */
	TIM0->TIM_CON &= ~TIM_CON_SPOS;
	TIM0->TIM_CON |= 0x00400000;
	TIM_Cmd(TIM0, ENABLE); 														//使能T0PWMA，比较值0输出低电平
}

/*****************************************************
*函数名称: SC_TIM1_Init
*函数功能: TIMER1初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM1_Init(void)
{
    /*TIM1_Init write here*/
}

/*****************************************************
*函数名称: SC_TIM2_Init
*函数功能: TIMER2初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM2_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_PWM_InitTypeDef TIM_PWM_InitStruct;
  /* 使能TIM1时钟 */
  RCC_APB1Cmd(ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* TIM配置结构体初始化 */
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_PWMStructInit(&TIM_PWM_InitStruct);
  
  /* 设置TIM时基配置结构体 */
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM工作在向上计数模式
  TIM_TimeBaseInitStruct.TIM_EXENX = TIM_EXENX_Disable;			//外部重加载管脚失能
  TIM_TimeBaseInitStruct.TIM_Preload = 72000000 / 2000;			//设置TIM的定时器频率为1000Hz@APB0Clock=36M
  TIM_TimeBaseInitStruct.TIM_Prescaler = TIM_PRESCALER_1;		//TIM分频为1分频
  TIM_TimeBaseInitStruct.TIM_WorkMode = TIM_WorkMode_Timer;		//TIM工作在定时器模式，时钟源为APB0时钟
  
  /* TIM时基配置 */
  TIM_TIMBaseInit(TIM2, &TIM_TimeBaseInitStruct);

  /* 设置TIM PWM配置结构体 */
  TIM_PWM_InitStruct.TIM_PWMOutputChannl = TIM_PWMChannel_PWMA;	//TIM1 PWMB输出使能
  
  /* TIM PWM配置 */
  TIM_PWMInit(TIM2,&TIM_PWM_InitStruct);
  TIM_PWMSetDuty(TIM2,TIM_PWMChannel_PWMA,TIM_TimeBaseInitStruct.TIM_Preload* 50/100);//占空比设置为50%
  
  /* TIM外设初始化 */
  TIM_Cmd(TIM2, ENABLE); // 若使能 T2PWMA会输出占空比设置为50%的方波
 //TIM_Cmd(TIM2, DISABLE);
}

/*****************************************************
*函数名称: SC_TIM3_Init
*函数功能: TIMER3初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    /* 使能TIM0时钟 */
    RCC_APB1Cmd(ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM配置结构体初始化 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);

    /* 设置TIM配置结构体 */
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM工作在向上计数模式
    TIM_TimeBaseInitStruct.TIM_EXENX = TIM_EXENX_Disable;	        //外部重加载管脚失能
    TIM_TimeBaseInitStruct.TIM_Preload = TASK_BASETIM_PRIOD_LOAD;	//设置TIM的定时器周期装载值
    TIM_TimeBaseInitStruct.TIM_Prescaler = TIM_PRESCALER_2;	        //TIM分频为1分频
    TIM_TimeBaseInitStruct.TIM_WorkMode = TIM_WorkMode_Timer;	    //TIM工作在定时器模式，时钟源为APB0时钟

    /* TIM时基配置 */
    TIM_TIMBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    /* 使能TIM0中断 */
    TIM_ITConfig(TIM3, TIM_IT_TI | TIM_IT_INTEN, ENABLE);

    /* TIM外设初始化 */
    TIM_Cmd(TIM3, ENABLE);
}

/*****************************************************
*函数名称: SC_TIM4_Init
*函数功能: TIMER4初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM4_Init(void)
{
    /*TIM4_Init write here*/
}

/*****************************************************
*函数名称: SC_TIM5_Init
*函数功能: TIMER0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM5_Init(void)
{
    /*TIM5_Init write here*/
}

/*****************************************************
*函数名称: SC_TIM6_Init
*函数功能: TIMER0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM6_Init(void)
{
    /*TIM6_Init write here*/
}

/*****************************************************
*函数名称: SC_TIM7_Init
*函数功能: TIMER0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM7_Init(void)
{
    /*TIM7_Init write here*/
}

/****************************************************************
	函数名：SC_TIM14_Init
	描述：配置TIM14定时器
	输入：无
	输出：无
****************************************************************/
void SC_TIM14_Init(void)
{
    /* TIM14 Peripheral Configuration -------------------------------------------*/
}


/*****************************************************
*函数名称: SC_TIM16_Init
*函数功能: TIMER0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TIM16_Init(void)
{
    /* TIM16 Peripheral Configuration -------------------------------------------*/
}

/*****************************************************
*函数名称: SC_EPWM_Init
*函数功能: 运放初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_EPWM_Init(void)
{
    /*EPWM_Init write here*/
    EPWM_InitTypeDef EPWM_InitStruct;
    EPWM_FDInitTypeDef EPWM_FDInitStruct;

    /* 使能PWM0时钟 */
    RCC_APB2Cmd(ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_EPWM, ENABLE);

    /* PWM配置结构体初始化 */
    EPWM_StructInit(&EPWM_InitStruct);

    /* 设置PWM配置结构体 */
    EPWM_InitStruct.EPWM_AlignedMode       = EPWM_AlignmentMode_Center;	//PWM对齐模式为边沿对齐
    EPWM_InitStruct.EPWM_Cycle             = EPWM_PRIOD_LOAD;						//PWM的频率为16000Hz

    EPWM_InitStruct.EPWM_OutputChannel     = EPWM_Channel_0 | EPWM_Channel_1 | EPWM_Channel_2 | EPWM_Channel_3 | EPWM_Channel_4 | EPWM_Channel_5;	//使能PWM00输出
    EPWM_InitStruct.EPWM_LowPolarityChannl = EPWM_Channel_1 | EPWM_Channel_3 | EPWM_Channel_5;	//PWM01输出反向

    EPWM_InitStruct.EPWM_Prescaler         = EPWM_PRESCALER_DIV1;					//PWM0时钟分频为1分配
    EPWM_InitStruct.EPWM_WorkMode          = EPWM_WorkMode_Complementary;	//PWM0工作模式为互补模式
    EPWM_Init(EPWM, &EPWM_InitStruct);	     //配置PWM0
	

    EPWM_SWAPConfig(EPWM,EPWM_SWAP_Alternate);    		//EPWM端口组合2	
	EPWM_MAPConfig( EPWM,EPWM_MAP_Alternate);			//EPWM端口组合3	
//    EPWM_SWAPConfig(EPWM, EPWM_SWAP_Alternate);   	//EPWM端口组合4

    EPWM_RisingDeadTimeConfig(EPWM,  EPWM_DEADTIME_LOAD);	/* 配置EPMW死区 */
    EPWM_FallingDeadTimeConfig(EPWM, EPWM_DEADTIME_LOAD);

    EPWM_SetADCTrigDir(EPWM, EPWM_ADCTrigGroup0, EPWM_ADCTrigDir_Up); /* 配置ADC组0在PWM向上/下计数时触发 */
    EPWM_SetADCTrigDir(EPWM, EPWM_ADCTrigGroup1, EPWM_ADCTrigDir_Up); /* 配置ADC组1在PWM向上/下计数时触发 */
    EPWM_SetADCTrigDir(EPWM, EPWM_ADCTrigGroup2, EPWM_ADCTrigDir_Up); /* 配置ADC组2在PWM向上/下计数时触发 */
    EPWM_SetADCTrigDir(EPWM, EPWM_ADCTrigGroup3, EPWM_ADCTrigDir_Up); /* 配置ADC组2在PWM向上/下计数时触发 */

    EPWM_SetADCTrigCnt(EPWM, EPWM_ADCTrigGroup0, 1);                  /* 配置ADC组0在PWM触发的计数值 */
    EPWM_SetADCTrigCnt(EPWM, EPWM_ADCTrigGroup1, 0xFFFF);             /* 配置ADC组1在PWM触发的计数值 */

    EPWM_SetADCTrigCnt(EPWM, EPWM_ADCTrigGroup2, 0xFFFF);             /* 配置ADC组2在PWM触发的计数值 */
    EPWM_SetADCTrigCnt(EPWM, EPWM_ADCTrigGroup3, 0xFFFF);             /* 配置ADC组3在PWM触发的计数值 */

    EPWM_FDInitStruct.EPWM_FDFilteringTime = EPWM_FilteringTime_0_8us;   	/* 刹车信号滤波时间为0.25uS */
    EPWM_FDInitStruct.EPWM_FDSoure = EPWM_FDSoure_CMP3;                 	/* 刹车故障源为软件的CBC */
	EPWM_FDInitStruct.EPWM_FDMode = EPWM_FDMODE_CBC;                 			/* 刹车故障电平为高 */
	EPWM_FDInitStruct.EPWM_FDTriggerLV = EPWM_FDTriggerLV_Hight;      		/* 刹车故障源为软件的CBC */
    EPWM_FDInitStruct.EPWM_FDSoureCmd = ENABLE;                         	/* 使能刹车源 */
    EPWM_FDInit(EPWM, &EPWM_FDInitStruct);

    EPWM_FD_HALTMSCmd(EPWM, ENABLE);                                    /* Debug模式是否刹车 */
    EPWM_FDOutputStateConfig(EPWM, EPWM_FDOutputState_PP);              /* 刹车后输出管脚为悬空/推挽 */ 

    EPWM_FDOutputLevelConfig(EPWM, EPWM_Channel_0 | EPWM_Channel_2 | EPWM_Channel_4, EPWM_FDOutputLevel_Low);  /* 刹车故障后上桥输出管脚电平为低电平 */
    EPWM_FDOutputLevelConfig(EPWM, EPWM_Channel_1 | EPWM_Channel_3 | EPWM_Channel_5, EPWM_FDOutputLevel_Low);  /* 刹车故障后下桥输出管脚电平为低电平 */

    EPWM_FDCmd(EPWM, ENABLE);   /* 使能EPWM故障检测功能 */
    EPWM_ITConfig(EPWM, EPWM_IT_INTEN |  EPWM_IT_CBC, ENABLE);  /* 失/使能 EPWM OneShot故障中断 */
    DISABLE_PWMOUT();
		
    /* PWM外设初始化 */
    EPWM_Cmd(EPWM, ENABLE);
}


/*****************************************************
*函数名称: SC_INT_Init
*函数功能: INT初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_INT_Init(void)
{
    /*INT_Init write here*/
}

#define VREF_INTERNAL 0  /* 选择内部VREF源*/
#define VREF_EXTERNAL 1  /* 选择外部输入VREF */
#define VREF_SELECT   (VREF_INTERNAL)   /* VREF源选择 */

/*****************************************************
*函数名称: SC_VREF_Init
*函数功能: VREF初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_VREF_Init(void)
{
#if (VREF_SELECT == VREF_INTERNAL)
    RCC_APB2Cmd(ENABLE);
    Vref_ReferenceVoltageConfig(VREF, Vref_InReferenceVoltage_2_4); /* VREF为内部设置 */
	
	Vref_VMIDCmd(VREF, ENABLE);    /* 使能1/2Vref管脚输出 */
	
#if ((OP_MODE_SELECT == INTERNAL_OP_MODE)||(OP_MODE_SELECT ==INTERNAL_PGA_MODE))	
    Vref_DIVENCmd(VREF, ENABLE);   /* 使能1/2Vref */
#if (OP_MODE_SELECT != INTERNAL_PGA_MODE)
    Vref_VMIDCmd(VREF, ENABLE);    /* 使能1/2Vref管脚输出 */
#endif
#endif
#else
	RCC_APB2Cmd(ENABLE);
    Vref_ReferenceVoltageConfig(VREF, Vref_ExReferenceVoltage); /* VREF由外部输入 */
    Vref_DIVENCmd(VREF, ENABLE);   /* 使能1/2Vref */
    Vref_VMIDCmd(VREF, ENABLE);    /* 使能1/2Vref管脚输出 */
#endif
}

/*****************************************************
*函数名称: SC_ADC_Init
*函数功能: ADC初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
#define ADC_REV *((volatile unsigned int *) 0x4002213C)
void SC_ADC_Init(void)
{
   /*ADC_Init write here*/
    ADC_InitTypeDef ADC_InitStruct;
    DMA_InitTypeDef DMA_InitStruct; //定义DMA初始化结构体变量

    /* ADC Periph clock enable */
    RCC_APB2Cmd(ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);

    /* ADC1 DeInit */
    ADC_DeInit(ADC);	
    ADC_REV = 0xA05F0F00; /* 修改ADC转换时间  F00快速模式400ns  D00快速模式350ns  ADC校准用*/
    ADC_StructInit(&ADC_InitStruct);

    /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
    ADC_InitStruct.ADC_ConvMode =  ADC_ConvMode_Sequence;   /* 连续转换模式 */
    ADC_InitStruct.ADC_SPMODE = ADC_SampleMode_Single;      /* 单采保模式 */
    ADC_InitStruct.ADC_Prescaler = ADC_Prescaler_9CLOCK;    /* 9个采样周期 */
	
	
    ADC_InitStruct.ADC_EAIN =  ADC_EAIN_11 | ADC_EAIN_7 | ADC_EAIN_3; /* 使能ADC通道 */

    ADC_InitStruct.ADC_VREF = ADC_RefSource_VDD;   /* 配置ADC参考电压 */

    ADC_Init(ADC, &ADC_InitStruct);

    /* 配置ADC序列通道 */
    ADC_SequenceChannelConfig(ADC, ADC_SQRank_0, ADC_Channel_IU);           /* U相电流  */
    ADC_SequenceChannelConfig(ADC, ADC_SQRank_1, ADC_Channel_IV);           /* V相电流  */
    ADC_SequenceChannelConfig(ADC, ADC_SQRank_2, ADC_Channel_IAVG);         /* V相电流  */
    ADC_SequenceChannelConfig(ADC, ADC_SQRank_3, ADC_Channel_VBUS);         /* 母线电压 */
    ADC_SequenceChannelConfig(ADC, ADC_SQRank_4, ADC_Channel_VSP);          /* 母线电压 */

    ADC_SequenceGroupConfig(ADC, ADC_SQGroup_0, 0, 5); /* 配置ADC序列0通道总数 */

    ADC_PWMTrigSQConvCongfig(ADC, ADC_SQGroup_0, ENABLE); /* 使能ADC序列0 */

    ADC_ITConfig(ADC, ADC_IT_INTEN | ADC_IT_EOSIE0, ENABLE);   /* 使能ADC序列0中断 */
    ADC_DMACmd(ADC, ENABLE);   /* 使能ADC DMA */

    /* 配置ADC DMA数据采集 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);
    *(uint32_t*)0x4001083C = 0xA05F0000;
    /* DMA外设配置 */
    DMA_StructInit(&DMA_InitStruct);

    DMA_InitStruct.DMA_CircularMode = DMA_CircularMode_Enable;  /* 使能DMA循环模式 */
    DMA_InitStruct.DMA_DataSize   = DMA_DataSize_HalfWord;     	/* 传输宽度为字 */
    DMA_InitStruct.DMA_TargetMode = DMA_TargetMode_INC_CIRC;    /* 目标地址递增 */
    DMA_InitStruct.DMA_SourceMode = DMA_SourceMode_FIXED;       /* 源地址不变 */
    DMA_InitStruct.DMA_SrcAddress = (uint32_t)&ADC->ADC_VALUE;  /* 源地址为ADC结果寄存器 */
    DMA_InitStruct.DMA_DstAddress = (uint32_t)ADC_DMABuff;         /* 目标地址为用户ADC结果数组 */
    DMA_InitStruct.DMA_Request = DMA_Request_ADC;               /* DMA触发源为ADC */
    DMA_InitStruct.DMA_BufferSize = 5;                          /* DMA数据量 */

    DMA_Init(DMA0, &DMA_InitStruct); /* 配置DMA0 */
    DMA_Cmd(DMA0, ENABLE);           /* 使能DMA0 */

    ADC_Cmd(ADC, ENABLE);            /* 使能ADC */
}


/*****************************************************
*函数名称: SC_PGA_Init
*函数功能: PGA初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_OP0_Init(void)
{
    /*PGA_Init write here*/
    OP_InitTypeDef OP_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;     /* OP0_P OP0_N OP0_OUT */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_HI;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    /* ADC Periph clock enable */
    RCC_APB2Cmd(ENABLE);

//    OP_DeInit(OP_0);

//    OP_InitStruct.OP_GAN = OP_GAN_Invert3;          /* 放大倍数为3倍 */
//    OP_InitStruct.OP_InvertInput = OP_InvertInput_Res;  /* 反向输入为 */
//    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_VREF_Div2; /* 正向输入为外部输入 */
//    OP_InitStruct.OP_OutputPin = OP_OutputPin_Disable;  /* OP管脚输出使能 */
//    OP_InitStruct.OP_OPRF = 0x00;
//    OP_InitStruct.OP_FDBR = OP_FDBR_OPxN;
//    OP_InitStruct.OP_CMPIM1 = 0;
//    OP_InitStruct.OP_CMPIM2 = 0;
//    OP_Init(OP_0, &OP_InitStruct);

//    OP_Cmd(OP_0, ENABLE);
	
    OP_StructInit(&OP_InitStruct);    /* 初始化参数 */
    OP_DeInit(OP_0);
	
	OP_InitStruct.OP_REFSEL = OP_RefSource_VREF;         /* 设置OP参考电压 */
    OP_InitStruct.OP_InvertInput = OP_InvertInput_OPxN;  /* 反向输入为 */
    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_OPxP; /* 正向输入为外部输入 */
    OP_InitStruct.OP_OutputPin = OP_OutputPin_Enable;  /* OP管脚输出使能 */
    OP_InitStruct.OP_FDBR = OP_FDBR_VSS;
    OP_Init(OP_0, &OP_InitStruct);

    OP_Cmd(OP_0, ENABLE);
}

/*****************************************************
*函数名称: SC_PGA_Init
*函数功能: PGA初始化函数
*入口参数：void
*出口参数：void·
+*****************************************************/
void SC_OP1_Init(void)
{
    /*PGA_Init write here*/
    OP_InitTypeDef OP_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; /* OP1_P OP1_N OP1_OUT */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_HI;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC Periph clock enable */
//    RCC_APB2Cmd(ENABLE);

//    OP_DeInit(OP_1);

//    OP_InitStruct.OP_GAN = OP_GAN_Invert3;           /* 放大倍数为3倍 */
//    OP_InitStruct.OP_InvertInput = OP_InvertInput_Res;  /* 反向输入为 */
//    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_VREF_Div2; /* 正向输入为外部输入 */
//    OP_InitStruct.OP_OutputPin = OP_OutputPin_Disable;  /* OP管脚输出使能 */
//    OP_InitStruct.OP_OPRF = 0x00;
//    OP_InitStruct.OP_FDBR = OP_FDBR_OPxN;
//    OP_InitStruct.OP_CMPIM1 = 0;
//    OP_InitStruct.OP_CMPIM2 = 0;
//    OP_Init(OP_1, &OP_InitStruct);

//    OP_Cmd(OP_1, ENABLE);
	
    /* ADC Periph clock enable */
    RCC_APB2Cmd(ENABLE);
	
    OP_StructInit(&OP_InitStruct);    /* 初始化参数 */
    OP_DeInit(OP_1);
	OP_InitStruct.OP_REFSEL = OP_RefSource_VREF;         /* 设置OP参考电压 */
    OP_InitStruct.OP_InvertInput = OP_InvertInput_OPxN;  /* 反向输入为 */
    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_OPxP; /* 正向输入为外部输入 */
    OP_InitStruct.OP_OutputPin = OP_OutputPin_Enable;  /* OP管脚输出使能 */
    OP_InitStruct.OP_FDBR = OP_FDBR_VSS;
    OP_Init(OP_1, &OP_InitStruct);

    OP_Cmd(OP_1, ENABLE);
}
/*****************************************************
*函数名称: SC_PGA_Init
*函数功能: PGA初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_OP2_Init(void)
{
	/*PGA_Init write here*/
    OP_InitTypeDef OP_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; /* OP2_P OP2_N OP2_OUT */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_HI;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
    /* ADC Periph clock enable */
//    RCC_APB2Cmd(ENABLE);

//    OP_DeInit(OP_2);

//    OP_InitStruct.OP_GAN = OP_GAN_Invert3;          /* 放大倍数为3倍 */
//    OP_InitStruct.OP_InvertInput = OP_InvertInput_Res;  /* 反向输入为 OP_InvertInput_OPxN */
//    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_VREF_Div2; /* 正向输入为外部输入 OP_InvertInput_OPxP*/
//    OP_InitStruct.OP_OutputPin = OP_OutputPin_Enable;  /* OP管脚输出使能 OP_OutputPin_Enable*/
//    OP_InitStruct.OP_OPRF = 0x00;
//    OP_InitStruct.OP_FDBR = OP_FDBR_OPxN;
//    OP_InitStruct.OP_CMPIM1 = 0;
//    OP_InitStruct.OP_CMPIM2 = 0;
//    OP_Init(OP_2, &OP_InitStruct);

//    OP_Cmd(OP_2, ENABLE);
	
	
    /* ADC Periph clock enable */
    RCC_APB2Cmd(ENABLE);
	
    OP_StructInit(&OP_InitStruct);    /* 初始化参数 */
    OP_DeInit(OP_2);
    OP_InitStruct.OP_REFSEL = OP_RefSource_VREF;         /* 设置OP参考电压 */
    OP_InitStruct.OP_InvertInput = OP_InvertInput_OPxN;  /* 反向输入为 */
    OP_InitStruct.OP_NonInvertInput = OP_NonInvertInput_OPxP; /* 正向输入为外部输入 */
    OP_InitStruct.OP_OutputPin = OP_OutputPin_Enable;  /* OP管脚输出使能 */
    OP_InitStruct.OP_FDBR = OP_FDBR_VSS;
    OP_Init(OP_2, &OP_InitStruct);

    OP_Cmd(OP_2, ENABLE);
}

/*****************************************************
*函数名称: SC_IAP_Init
*函数功能: IAP初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_IAP_Init(void)
{
    /*IAP_Init write here*/
}

/*****************************************************
*函数名称: SC_SPI0_Init
*函数功能: SPI0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_SPI0_Init(void)
{
    /*SPI0_Init write here*/
}

/*****************************************************
*函数名称: SC_SPI1_Init
*函数功能: SPI1初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_SPI1_Init(void)
{
    /*SPI1_Init write here*/
}

/*****************************************************
*函数名称: SC_TWI0_Init
*函数功能: TWI0初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TWI0_Init(void)
{
    /*TWI0_Init write here*/
}

/*****************************************************
*函数名称: SC_TWI1_Init
*函数功能: TWI1初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_TWI1_Init(void)
{
    /*TWI1_Init write here*/
}

/*****************************************************
*函数名称: SC_BTM_Init
*函数功能: 低频时钟定时器初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_BTM_Init(void)
{
    /*BTM_Init write here*/
}

/*****************************************************
*函数名称: SC_CRC_Init
*函数功能: check sum 初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_CRC_Init(void)
{
    /*CRC_Init write here*/
}

/*****************************************************
*函数名称: SC_WDT_Init
*函数功能: 看门狗初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_WDT_Init(void)
{
    /*WDT_Init write here*/
}

/*****************************************************
*函数名称: SC_LCD_LED_Init
*函数功能: 显示控制初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_LCD_LED_Init(void)
{
    /*LCD_LED_Init write here*/
}

/*****************************************************
*函数名称: SC_PCAP_Init
*函数功能: 三相初始化
*入口参数：void
*出口参数：void
*****************************************************/
void SC_PCAP_Init(void)
{
 
}

/*****************************************************
*函数名称: SC_CMP0_Init
*函数功能: 模拟比较器0初始化
*入口参数：void
*出口参数：void
*****************************************************/
void SC_CMP0_Init(void)
{
    CMP_InitTypeDef  CMP_InitStruct;

    CMP_DeInit(CMP_0);
    CMP_StructInit(&CMP_InitStruct);
//    CMP_InitStruct.CMP_CMPRF       = CMP_CMPRF_1D16CMP_VREF;    /* 负端输入VREF分压比 */
//    CMP_InitStruct.CMP_Negative    = CMP_Negative_CMPRF;        /* 负端输入       */
    CMP_InitStruct.CMP_Negative    = CMP0_1_2NegativeSelect_DAC;            /* 负端输入       */
    CMP_InitStruct.CMP_Positive    = CMP0_Positive_CMP0P;          /* 正端为外部输入 */
    CMP_InitStruct.CMP_TriggerMode = CMP0_1_2TriggerMode_RISE;        /* CMP上升沿触发  */
    CMP_InitStruct.CMP_HYS         = CMP_HYS_0mV;                 /* 迟滞电压为20mV */

//    CMP_ITConfig(CMP_0, CMP0_1_2IT_CMP0, ENABLE); /* 使能CMP3中断 */
    CMP_Init(CMP_0, &CMP_InitStruct);
    CMP_ClearFlag(CMP_0, CMP_FLAG_CMP0);  /* 使能CMP3中断 */
    CMP_Cmd(CMP_0, ENABLE);
}

/*****************************************************
*函数名称: SC_CMP3_Init
*函数功能: 模拟比较器3初始化
*入口参数：void
*出口参数：void
*****************************************************/
void SC_CMP3_Init(void)
{
    CMP_InitTypeDef  CMP_InitStruct;

    CMP_DeInit(CMP3);
    CMP_StructInit(&CMP_InitStruct);
//    CMP_InitStruct.CMP_CMPRF       = CMP_CMPRF_1D16CMP_VREF;    /* 负端输入VREF分压比 */
//    CMP_InitStruct.CMP_Negative    = CMP_Negative_CMPRF;        /* 负端输入       */
    CMP_InitStruct.CMP_Negative    = CMP3_NegativeSelect_DAC;     /* 负端输入       */
    CMP_InitStruct.CMP_Positive    = CMP3_Positive_CMP3P;         /* 正端为外部输入 */
    CMP_InitStruct.CMP_TriggerMode = CMP3_TriggerMode_RISE;       /* CMP上升沿触发  */
    CMP_InitStruct.CMP_HYS         = CMP_HYS_0mV;                 /* 迟滞电压为20mV */

//    CMP_ITConfig(CMP3, CMP3_IT_INT, ENABLE); /* 使能CMP3中断 */
    CMP_Init(CMP3, &CMP_InitStruct);
    CMP_ClearFlag(CMP3, CMP_FLAG_CMP3);  /* 使能CMP3 */
    CMP_Cmd(CMP3, ENABLE);
}

/*****************************************************
*函数名称: SC_DAC_Init
*函数功能: DAC初始化函数
*入口参数：void
*出口参数：void
*****************************************************/
void SC_DAC_Init(void)
{
    DAC_DeInit(DAC);   /* DAC外设复位 */
	DAC_VrefConfig(DAC, DAC_RefSource_VDD);        /* 设置DAC参考电压 */
    DAC_SetConversionValue(DAC,IBUS_HOCP_DACValue); /* 设置DAC输出值 0-1023 */
    DAC_ChannelConfig(DAC, DAC_Channel_1, DISABLE);   /* DAC输出到DAC通道1 */
    DAC_Cmd(DAC, ENABLE); /* 使能DAC */
}

/****************************************************************
	函数名: Motor_MR_Init
	描述:   硬件加速单元初始化函数
	输入:   无
	输出:   无
****************************************************************/
void SC_MR_Init(void)
{
    MR_InitTypeDef MR_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_MR, ENABLE); /* 开启硬件加速模块时钟 */
    MR_DeInit();

    MR_InitStruct.MR_ModulationMode = MR_SevenSVPWM;//MR_SevenSVPWM MR_Five SVPWM SPWM
    MR_InitStruct.MR_PwmPriodLoad   = EPWM_PRIOD_LOAD + 1;
    MR_InitStruct.MR_MulMode        = MR_SingedMul;//Signed
    MR_InitStruct.MR_SequenceMode   = ENABLE;//DISABLE
    MR_Init(&MR_InitStruct);
}

/****************************************************************
	函数名: WDG_init
	描述:   看门狗初始化函数
	输入:   无
	输出:   无
****************************************************************/
void WDG_init(void)
{
	WDT_SetOverTime(WDT_OverTime_62_5MS);//设置WDT时间为62.5ms
		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_IFB,ENABLE);//开启IFB时钟
	
	OPTION_WDTCmd(ENABLE);//开启WDT
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_IFB,DISABLE);//IFB时钟关闭	 
}

/****************************************************************
	函数名: delay_ms
	描述:   软件延时ms
	输入:   无
	输出:   无
****************************************************************/
void SoftDelay_50MS(void)
{
	u16 i,j,k;

	for(i=30;i>0;i--)

	for(j=100;j>0;j--)

	for(k=200;k>0;k--);
}

