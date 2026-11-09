/**
 ******************************************************************************
 * @file    sc32m1xxx_adc.h
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   Header file of ADC module.
 ******************************************************************************
 * @attention
 *
 *1.This software is supplied by SinOne Microelectronics Co.,Ltd. and is only 
 *intended for use with SinOne products. No other uses are authorized. This 
 *software is owned by SinOne Microelectronics Co.,Ltd. and is protected under 
 *all applicable laws, including copyright laws. 
 *2.The software which is for guidance only aims at providing customers with 
 *coding information regarding their products in order for them to save time. 
 *As a result, SinOne shall not be held liable for any direct, indirect or 
 *consequential damages with respect to any claims arising from the content of 
 *such software and/or the use made by customers of the coding information 
 *contained herein in connection with their products.
 *
 *  COPYRIGHT 2024 SinOne Microelectronics
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __sc32m1xxx_ADC_H
#define __sc32m1xxx_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"



/* Exported enumerations ------------------------------------------------------------*/
/** @defgroup ADC_Exported_Enumerations ADC Exported Enumerations
 * @{
 */

/** @defgroup ADC_SampleMode ADC Sample Mode
 * @{
 */
typedef enum
{
    ADC_SampleMode_Single    = (uint16_t)(0x00 << ADC_CON_SPMODE_Pos),   /* Single sampling mode */
    ADC_SampleMode_Double    = (uint16_t)(0x01 << ADC_CON_SPMODE_Pos),   /* Double sampling mode */
} ADC_SampleMode_TypeDef;

#define IS_ADC_SAMPLEMODE(MODE) (((MODE) == ADC_SampleMode_Single) ||  \
                                 ((MODE) == ADC_SampleMode_Double))
/**
 * @}
 */


/** @defgroup ADC_ConvMode ADC Conversion Mode
 * @{
 */
typedef enum
{
    ADC_ConvMode_Single      = (uint16_t)(0x00 << ADC_CON_CONT_Pos),   /* Single conversion mode */
    ADC_ConvMode_Sequence    = (uint16_t)(0x01 << ADC_CON_CONT_Pos),   /* Sequence conversion mode */
} ADC_ConvMode_TypeDef;

#define IS_ADC_CONVMODE(MODE) (((MODE) == ADC_ConvMode_Single) ||  \
                               ((MODE) == ADC_ConvMode_Sequence))
/**
 * @}
 */

/** @defgroup ADC_Prescaler ADC Prescaler
 * @{
 */
typedef enum
{
    ADC_Prescaler_3CLOCK   = (uint32_t)(0x00 << ADC_CON_LOWSP_Pos),  /* The sampling time is 3   APB2 clocks */
    ADC_Prescaler_6CLOCK   = (uint32_t)(0x01 << ADC_CON_LOWSP_Pos),  /* The sampling time is 6   APB2 clocks */
    ADC_Prescaler_9CLOCK   = (uint32_t)(0x02 << ADC_CON_LOWSP_Pos),  /* The sampling time is 9   APB2 clocks */
    ADC_Prescaler_15CLOCK  = (uint32_t)(0x03 << ADC_CON_LOWSP_Pos),  /* The sampling time is 15  APB2 clocks */
    ADC_Prescaler_30CLOCK  = (uint32_t)(0x04 << ADC_CON_LOWSP_Pos),  /* The sampling time is 30  APB2 clocks */
    ADC_Prescaler_60CLOCK  = (uint32_t)(0x05 << ADC_CON_LOWSP_Pos),  /* The sampling time is 60  APB2 clocks */
    ADC_Prescaler_120CLOCK = (uint32_t)(0x06 << ADC_CON_LOWSP_Pos),  /* The sampling time is 120 APB2 clocks */
    ADC_Prescaler_480CLOCK = (uint32_t)(0x07 << ADC_CON_LOWSP_Pos),  /* The sampling time is 480 APB2 clocks */
} ADC_Prescaler_TypeDef;

#define IS_ADC_PRESCALER(PRESCALER) (((PRESCALER) == ADC_Prescaler_3CLOCK)   || \
                                     ((PRESCALER) == ADC_Prescaler_6CLOCK)   || \
                                     ((PRESCALER) == ADC_Prescaler_9CLOCK)   || \
                                     ((PRESCALER) == ADC_Prescaler_15CLOCK)  || \
                                     ((PRESCALER) == ADC_Prescaler_30CLOCK)  || \
                                     ((PRESCALER) == ADC_Prescaler_60CLOCK)  || \
                                     ((PRESCALER) == ADC_Prescaler_120CLOCK) || \
                                     ((PRESCALER) == ADC_Prescaler_480CLOCK))
/**
 * @}
 */

/** @defgroup ADC_ChannelTypedef ADC input channel
 * @{
 */
typedef enum
{
    ADC_Channel_0      = (uint16_t)0x00,		 /*!< ADC input channel 0 */
    ADC_Channel_1      = (uint16_t)0x01,		 /*!< ADC input channel 1 */
    ADC_Channel_2      = (uint16_t)0x02,		 /*!< ADC input channel 2 */
    ADC_Channel_3      = (uint16_t)0x03,		 /*!< ADC input channel 3 */
    ADC_Channel_4      = (uint16_t)0x04,		 /*!< ADC input channel 4 */
    ADC_Channel_5      = (uint16_t)0x05,		 /*!< ADC input channel 5 */
    ADC_Channel_6      = (uint16_t)0x06,		 /*!< ADC input channel 6 */
    ADC_Channel_7      = (uint16_t)0x07,		 /*!< ADC input channel 7 */
    ADC_Channel_8      = (uint16_t)0x08,		 /*!< ADC input channel 8 */
    ADC_Channel_9      = (uint16_t)0x09,		 /*!< ADC input channel 9 */
    ADC_Channel_10     = (uint16_t)0x0A,		 /*!< ADC input channel 10 */
    ADC_Channel_11     = (uint16_t)0x0B,		 /*!< ADC input channel 11 */
    ADC_Channel_12     = (uint16_t)0x0C,		 /*!< ADC input channel 12 */
    ADC_Channel_13     = (uint16_t)0x0D,		 /*!< ADC input channel 13 */
    ADC_Channel_14     = (uint16_t)0x0E,		 /*!< ADC input channel 14 */
    ADC_Channel_15     = (uint16_t)0x0F,		 /*!< ADC input channel 15 */
    ADC_Channel_VDD_D4 = (uint16_t)0x10,		 /*!< ADC_CHANNEL VDD Divide 4 */
    ADC_Channel_TEMP   = (uint16_t)0x11,		 /*!< ADC input channel TEMP */
} ADC_ChannelTypedef;

#define IS_ADC_CHANNEL(CHANNEL) ((CHANNEL) != 0)

/** @defgroup ADC_ChannelATypedef ADC input channel
 * @{
 */
typedef enum
{
    ADC_ChannelA_0      = (uint16_t)0x00,		 /*!< ADC input channel 0 */ 
    ADC_ChannelA_2      = (uint16_t)0x02,		 /*!< ADC input channel 2 */
    ADC_ChannelA_4      = (uint16_t)0x04,		 /*!< ADC input channel 4 */ 
    ADC_ChannelA_6      = (uint16_t)0x06,		 /*!< ADC input channel 6 */
    ADC_ChannelA_8      = (uint16_t)0x08,		 /*!< ADC input channel 8 */
    ADC_ChannelA_10     = (uint16_t)0x0A,		 /*!< ADC input channel 10 */
    ADC_ChannelA_12     = (uint16_t)0x0C,		 /*!< ADC input channel 12 */
    ADC_ChannelA_14     = (uint16_t)0x0E,		 /*!< ADC input channel 14 */
    ADC_ChannelA_VDD_D4 = (uint16_t)0x10,		 /*!< ADC_CHANNEL VDD Divide 4 */
} ADC_ChannelATypedef;

#define IS_ADC_CHANNEL(CHANNEL) ((CHANNEL) != 0)
/** @defgroup ADC_ChannelBTypedef ADC input channel
 * @{
 */
typedef enum
{
   
    ADC_ChannelB_1      = (uint16_t)0x01,		 /*!< ADC input channel 1 */   
    ADC_ChannelB_3      = (uint16_t)0x03,		 /*!< ADC input channel 3 */
    ADC_ChannelB_5      = (uint16_t)0x05,		 /*!< ADC input channel 5 */
    ADC_ChannelB_7      = (uint16_t)0x07,		 /*!< ADC input channel 7 */  
    ADC_ChannelB_9      = (uint16_t)0x09,		 /*!< ADC input channel 9 */ 
    ADC_ChannelB_11     = (uint16_t)0x0B,		 /*!< ADC input channel 11 */ 
    ADC_ChannelB_13     = (uint16_t)0x0D,		 /*!< ADC input channel 13 */ 
    ADC_ChannelB_15     = (uint16_t)0x0F,		 /*!< ADC input channel 15 */ 
    ADC_ChannelB_TEMP   = (uint16_t)0x11,		 /*!< ADC input channel TEMP */
} ADC_ChannelBTypedef;

#define IS_ADC_CHANNEL(CHANNEL) ((CHANNEL) != 0)
/**
 * @}
 */

/** @defgroup ADC_EAIN ADC Enable Input
 * @{
 */
typedef enum
{
    ADC_EAIN_Less = (uint32_t)0x00000000, /*!< ADC port Less */
    ADC_EAIN_0   = (uint32_t)0x00000001,  /*!< ADC port 0  */
    ADC_EAIN_1   = (uint32_t)0x00000002,  /*!< ADC port 1  */
    ADC_EAIN_2   = (uint32_t)0x00000004,  /*!< ADC port 2  */
    ADC_EAIN_3   = (uint32_t)0x00000008,  /*!< ADC port 3  */
    ADC_EAIN_4   = (uint32_t)0x00000010,  /*!< ADC port 4  */
    ADC_EAIN_5   = (uint32_t)0x00000020,  /*!< ADC port 5  */
    ADC_EAIN_6   = (uint32_t)0x00000040,  /*!< ADC port 6  */
    ADC_EAIN_7   = (uint32_t)0x00000080,  /*!< ADC port 7  */
    ADC_EAIN_8   = (uint32_t)0x00000100,  /*!< ADC port 8  */
    ADC_EAIN_9   = (uint32_t)0x00000200,  /*!< ADC port 9  */
    ADC_EAIN_10  = (uint32_t)0x00000400,  /*!< ADC port 10 */
    ADC_EAIN_11  = (uint32_t)0x00000800,  /*!< ADC port 11 */
    ADC_EAIN_12  = (uint32_t)0x00001000,  /*!< ADC port 12 */
    ADC_EAIN_13  = (uint32_t)0x00002000,  /*!< ADC port 13 */
    ADC_EAIN_14  = (uint32_t)0x00004000,  /*!< ADC port 14 */
    ADC_EAIN_15  = (uint32_t)0x00008000,  /*!< ADC port 15 */
    ADC_EAIN_All = (uint32_t)0x0000FFFF,  /*!< ADC port All */
} ADC_EAIN_Typedef;

#define IS_ADC_EAIN(EAIN) ((EAIN | ADC_EAIN_All) != 0)
/** @defgroup ADC_Thresholds_Channel ADC Thresholds Channel 
 * @{
 */
typedef enum
{
    ADC_Thresholds_Channel_Less = (uint32_t)0x00000000, /*!< ADC port Less */
    ADC_Thresholds_Channel_0   = (uint32_t)0x00000001,  /*!< ADC port 0  */
    ADC_Thresholds_Channel_1   = (uint32_t)0x00000002,  /*!< ADC port 1  */
    ADC_Thresholds_Channel_2   = (uint32_t)0x00000004,  /*!< ADC port 2  */
    ADC_Thresholds_Channel_3   = (uint32_t)0x00000008,  /*!< ADC port 3  */
    ADC_Thresholds_Channel_4   = (uint32_t)0x00000010,  /*!< ADC port 4  */
    ADC_Thresholds_Channel_5   = (uint32_t)0x00000020,  /*!< ADC port 5  */
    ADC_Thresholds_Channel_6   = (uint32_t)0x00000040,  /*!< ADC port 6  */
    ADC_Thresholds_Channel_7   = (uint32_t)0x00000080,  /*!< ADC port 7  */
    ADC_Thresholds_Channel_8   = (uint32_t)0x00000100,  /*!< ADC port 8  */
    ADC_Thresholds_Channel_9   = (uint32_t)0x00000200,  /*!< ADC port 9  */
    ADC_Thresholds_Channel_10  = (uint32_t)0x00000400,  /*!< ADC port 10 */
    ADC_Thresholds_Channel_11  = (uint32_t)0x00000800,  /*!< ADC port 11 */
    ADC_Thresholds_Channel_12  = (uint32_t)0x00001000,  /*!< ADC port 12 */
    ADC_Thresholds_Channel_13  = (uint32_t)0x00002000,  /*!< ADC port 13 */
    ADC_Thresholds_Channel_14  = (uint32_t)0x00004000,  /*!< ADC port 14 */
    ADC_Thresholds_Channel_15  = (uint32_t)0x00008000,  /*!< ADC port 15 */
    ADC_Thresholds_Channel_All = (uint32_t)0x0000FFFF,  /*!< ADC port All */
} ADC_Thresholds_Channel_Typedef;

#define IS_ADC_Thresholds_Channel(Channel) ((Channel | ADC_Thresholds_Channel_All) != 0)
/** @defgroup ADC_SQGroup ADC SQGroup
 * @{
 */
typedef enum
{
    ADC_SQGroup_0    = (uint32_t)0x0000,   /*!< ADC sequence group 0 */
    ADC_SQGroup_1    = (uint32_t)0x0001,   /*!< ADC sequence group 1 */
    ADC_SQGroup_2    = (uint32_t)0x0002,   /*!< ADC sequence group 2 */
    ADC_SQGroup_3    = (uint32_t)0x0003,   /*!< ADC sequence group 3 */
} ADC_SQGroup_Typedef;

#define IS_ADC_SQGROUP(GROUP) ((GROUP) != 0)
/**
 * @}
 */

/** @defgroup ADC_SQRank ADC SQRank
 * @{
 */
typedef enum
{
    ADC_SQRank_0    = (uint32_t)0x0000,   /*!< ADC sequence Channel 0 */
    ADC_SQRank_1    = (uint32_t)0x0001,   /*!< ADC sequence Channel 1 */
    ADC_SQRank_2    = (uint32_t)0x0002,   /*!< ADC sequence Channel 2 */
    ADC_SQRank_3    = (uint32_t)0x0003,   /*!< ADC sequence Channel 3 */
    ADC_SQRank_4    = (uint32_t)0x0004,   /*!< ADC sequence Channel 4 */
    ADC_SQRank_5    = (uint32_t)0x0005,   /*!< ADC sequence Channel 5 */
    ADC_SQRank_6    = (uint32_t)0x0006,   /*!< ADC sequence Channel 6 */
    ADC_SQRank_7    = (uint32_t)0x0007,   /*!< ADC sequence Channel 7 */
    ADC_SQRank_8    = (uint32_t)0x0008,   /*!< ADC sequence Channel 8 */
    ADC_SQRank_9    = (uint32_t)0x0009,   /*!< ADC sequence Channel 9 */
    ADC_SQRank_10   = (uint32_t)0x000A,   /*!< ADC sequence Channel 10 */
    ADC_SQRank_11   = (uint32_t)0x000B,   /*!< ADC sequence Channel 11 */
    ADC_SQRank_12   = (uint32_t)0x000C,   /*!< ADC sequence Channel 12 */
    ADC_SQRank_13   = (uint32_t)0x000D,   /*!< ADC sequence Channel 13 */
    ADC_SQRank_14   = (uint32_t)0x000E,   /*!< ADC sequence Channel 14 */
    ADC_SQRank_15   = (uint32_t)0x000F,   /*!< ADC sequence Channel 15 */
} ADC_SQRank_Typedef;

#define IS_ADC_SQRANK(RANK) ((RANK) != 0)
/**
 * @}
 */

/** @defgroup ADC_TriggerOut ADC TriggerOut
 * @{
 */
typedef enum
{
    ADC_TriggerOut_Less  = (uint32_t)(0x00 << ADC_CON_TRIG_OUT_Pos),	  /*!< ADC Trigger Out  Less  */
    ADC_TriggerOut_0     = (uint32_t)(0x01 << ADC_CON_TRIG_OUT_Pos),	  /*!< Output from ADC_trigger0 */
    ADC_TriggerOut_1     = (uint32_t)(0x02 << ADC_CON_TRIG_OUT_Pos),	  /*!< Output from ADC_trigger1 */
} ADC_TriggerOut_TypeDef;

#define IS_ADC_TRIGGEROUT(OUT) (((OUT) == ADC_TriggerOut_Less)  || \
                                ((OUT) == ADC_TriggerOut_0)     || \
                                ((OUT) == ADC_TriggerOut_1))
/**
 * @}
 */

/** @defgroup ADC_VREF ADC Reference Voltage
 * @{
 */
typedef enum
{
    ADC_RefSource_VDD  = (uint32_t)(0x00 << ADC_CON_REFSEL_Pos),	  /*!< ADC reference voltage is VDD    */
    ADC_RefSource_VREF = (uint32_t)(0x01 << ADC_CON_REFSEL_Pos),	  /*!< ADC reference voltage is Vref */
} ADC_VREF_TypeDef;

#define IS_ADC_VREF(VREF) (((VREF) == ADC_RefSource_VDD)    || \
                           ((VREF) == ADC_RefSource_VREF))
/**
 * @}
 */

/** @defgroup ADC_IT ADC Interrupt
 * @{
 */
typedef enum
{
    ADC_IT_INTEN    = (uint8_t)ADC_IDE_INTEN,	    /*!< ADC interrupt request Flag */
    ADC_IT_EOCIE    = (uint8_t)ADC_IDE_EOCIE,	/*!< ADC sequence 0 sampling + conversion complete interrupt Flag */
    ADC_IT_EOSIE0   = (uint8_t)ADC_IDE_EOSIE0,	/*!< ADC sequence 0 sampling + conversion complete interrupt Flag */
    ADC_IT_EOSIE1   = (uint8_t)ADC_IDE_EOSIE1,	/*!< ADC sequence 1 sampling + conversion complete interrupt Flag */
    ADC_IT_EOSIE2   = (uint8_t)ADC_IDE_EOSIE2,	/*!< ADC sequence 2 sampling + conversion complete interrupt Flag */
    ADC_IT_EOSIE3   = (uint8_t)ADC_IDE_EOSIE3,	/*!< ADC sequence 3 sampling + conversion complete interrupt Flag */
    ADC_IT_UPTHIE   = (uint8_t)ADC_IDE_UPTHIE,	/*!< ADC upper threshold overflow Flag */
    ADC_IT_LOWTHIE  = (uint8_t)ADC_IDE_LOWTHIE,	/*!< ADC lower threshold overflow Flag */
} ADC_IT_TypeDef;

#define IS_ADC_IT(IT) ((((IT) & (uint16_t)0xFEFF) == 0x00) && ((IT) != (uint16_t)0x0000))
/**
 * @}
 */

/** @defgroup ADC_FLAG ADC Flag
 * @{
 */
typedef enum
{
    ADC_Flag_ADCIF    = (uint8_t)ADC_STS_EOCIF,   /*!< ADC interrupt request Flag */
    ADC_Flag_EOSIF0   = (uint8_t)ADC_STS_EOSIF0,	/*!< ADC sequence 0 sampling + conversion complete interrupt Flag */
    ADC_Flag_EOSIF1   = (uint8_t)ADC_STS_EOSIF1,	/*!< ADC sequence 1 sampling + conversion complete interrupt Flag */
    ADC_Flag_EOSIF2   = (uint8_t)ADC_STS_EOSIF2,	/*!< ADC sequence 2 sampling + conversion complete interrupt Flag */
    ADC_Flag_EOSIF3   = (uint8_t)ADC_STS_EOSIF3,	/*!< ADC sequence 3 sampling + conversion complete interrupt Flag */
    ADC_Flag_BUSY     = (uint8_t)ADC_STS_BUSY,    /*!< ADC busy Flag */
    ADC_Flag_UPTHIF   = (uint8_t)ADC_STS_UPTHIF,	/*!< ADC upper threshold overflow Flag */
    ADC_Flag_LOWTHIF  = (uint8_t)ADC_STS_LOWTHIF,	/*!< ADC lower threshold overflow Flag */
} ADC_FLAG_TypeDef;

#define IS_ADC_FLAG(FLAG) ((((FLAG) & (uint16_t)0xFFFE) == 0x00) && ((FLAG) != (uint16_t)0x0000))
#define IS_GET_ADC_FLAG(FLAG) (((FLAG) == ADC_Flag_ADCIF)  || \
                               ((FLAG) == ADC_Flag_EOSIF0) || \
                               ((FLAG) == ADC_Flag_EOSIF1) || \
                               ((FLAG) == ADC_Flag_EOSIF2) || \
                               ((FLAG) == ADC_Flag_EOSIF3) || \
                               ((FLAG) == ADC_Flag_BUSY)   || \
                               ((FLAG) == ADC_Flag_UPTHIF) || \
                               ((FLAG) == ADC_Flag_LOWTHIF))

/**
 * @}
 */

/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/


#define IS_ADC_ALL_PERIPH(PERIPH) ((PERIPH) == ADC)

/**
 * @}
 */
/* End of constants -----------------------------------------------------*/

/** @defgroup ADC_Exported_Struct ADC Exported Struct
 * @{
 */

/** @defgroup ADC Time base Configuration Structure definition
 * @{
 */
typedef struct
{
   

    uint32_t ADC_EAIN; /*!< Specifies Input channel of ADC.
                                                                    This parameter can be a value of @ref ADC_EAIN_Typedef */

    uint32_t ADC_VREF; /*!<  Specifies reference voltage of ADC.
                                                                    This parameter can be a value of @ref ADC_VREF_TypeDef */
	
    uint16_t ADC_ConvMode; /*!< Specifies ADC work conversion mode.
                                                                    This parameter can be a value of @ref ADC_ConvMode_TypeDef */

    uint16_t ADC_SPMODE; /*!<  Specifies Sampling mode of ADC.
                                                                    This parameter can be a value of @ref ADC_SampleMode_TypeDef */

    uint16_t ADC_Prescaler; /*!< Specifies Prescaler of ADC.
                                                                  This parameter can be a value of @ref ADC_Prescaler_TypeDef */
} ADC_InitTypeDef;
/**
 * @}
 */

/**
 * @}
 */
/* End of struct -----------------------------------------------------*/

/** @addtogroup ADC_Exported_Functions ADC Exported Functions
 * @{
 */

/* ADC Base functions ********************************************************/
void ADC_DeInit(ADC_TypeDef* ADCx);
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState);
void ADC_SetChannel(ADC_TypeDef* ADCx, ADC_ChannelTypedef ADC_Channel);
ADC_ChannelTypedef ADC_GetChannel(ADC_TypeDef* ADCx);
/* ADC Conversion management functions ********************************************************/
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx);
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx);
/* Interrupts and flags  ***********************************************/
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint16_t ADC_FLAG);
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint16_t ADC_FLAG);
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState);
/* Common interface functions of motors******************************************************/
void ADC_SampleModeConfig(ADC_TypeDef* ADCx, ADC_SampleMode_TypeDef ADC_SampleMode);
void ADC_ConvModeConfig(ADC_TypeDef* ADCx, ADC_ConvMode_TypeDef ADC_ConvMode);
void ADC_SequenceChannelConfig(ADC_TypeDef* ADCx,ADC_SQRank_Typedef ADC_SQRank,ADC_ChannelTypedef ADC_Channel);
void ADC_SetDoubleSample(ADC_TypeDef* ADCx, ADC_ChannelATypedef ADC_ChannelA,ADC_ChannelBTypedef ADC_ChannelB);
void ADC_SequenceGroupConfig(ADC_TypeDef* ADCx,ADC_SQGroup_Typedef ADC_SQGroup,uint8_t SQStr,uint8_t SQCnt);
void ADC_PWMTrigSQConvCongfig(ADC_TypeDef* ADCx,ADC_SQGroup_Typedef ADC_SQGroup,FunctionalState NewState);
void ADC_RrigOutCongfig(ADC_TypeDef* ADCx,ADC_TriggerOut_TypeDef ADC_TriggerOut);
void ADC_SetThresholds(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold);
void ADC_ThresholdsChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_Thresholds_Channel, FunctionalState NewState);
uint32_t ADC_GetDSConversionValueA(ADC_TypeDef* ADCx);
uint32_t ADC_GetDSConversionValueB(ADC_TypeDef* ADCx);


/**
 * @}
 */
/* End of exported functions --------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
