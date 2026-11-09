/**
 ******************************************************************************
 * @file    sc32m1xxx_epwm.h
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   Header file of EPWM module.
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
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __sc32m1xxx_EPWM_H
#define __sc32m1xxx_EPWM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"


/** @defgroup EPWM_Prescaler PWM Prescaler
 * @{
 */
typedef enum
{
    EPWM_PRESCALER_DIV1	   = (uint32_t)(0x00U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/1    */
    EPWM_PRESCALER_DIV2	   = (uint32_t)(0x01U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/2    */
    EPWM_PRESCALER_DIV4	   = (uint32_t)(0x02U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/4    */
    EPWM_PRESCALER_DIV8	   = (uint32_t)(0x03U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/8    */
    EPWM_PRESCALER_DIV16   = (uint32_t)(0x04U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/16    */
    EPWM_PRESCALER_DIV32   = (uint32_t)(0x05U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/32    */
    EPWM_PRESCALER_DIV64   = (uint32_t)(0x06U << EPWM_CON_PWMCLK_Pos),	/*!< Clock division: Fsource/64    */
    EPWM_PRESCALER_DIV128  = (uint32_t)(0x07U << EPWM_CON_PWMCLK_Pos),    /*!< Clock division: Fsource/128    */
} EPWM_Prescaler_TypeDef;

#define IS_EPWM_PRESCALER(PRESCALER) (((PRESCALER) == EPWM_PRESCALER_DIV1)  ||   \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV2)   ||   \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV4)   ||   \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV8)   ||   \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV16)  ||  \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV32)  ||  \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV64)  ||  \
                                      ((PRESCALER) == EPWM_PRESCALER_DIV128))
/**
 * @}
 */

/** @defgroup EPWM_AlignedMode EPWM AlignedMode
 * @{
 */
typedef enum
{
    EPWM_AlignmentMode_Edge   = (uint16_t)(0x00U << EPWM_CON_PWMMD0_Pos),	/*!< Edge Alignment mode */
    EPWM_AlignmentMode_Center = (uint16_t)(0x01U << EPWM_CON_PWMMD0_Pos), /*!< Center Alignment mode */
} EPWM_AlignedMode_TypeDef;

#define IS_EPWM_AlignedMode(MODE) (((MODE) == EPWM_AlignmentMode_Edge) || \
                                   ((MODE) == EPWM_AlignmentMode_Center))
/**
 * @}
 */

/** @defgroup EPWM_WordMode EPWM WordMode
 * @{
 */
typedef enum
{
    EPWM_WorkMode_Independent   = (uint16_t)(0x00U << EPWM_CON_PWMMD1_Pos),	 /*!< Independent working mode */
    EPWM_WorkMode_Complementary = (uint16_t)(0x01U << EPWM_CON_PWMMD1_Pos),  /*!< Complementary working mode*/
} EPWM_WorkMode_TypeDef;

#define IS_EPWM_WORKMode(MODE)	(((MODE) == EPWM_WorkMode_Independent) || \
                                 ((MODE) == EPWM_WorkMode_Complementary))
/**
 * @}
 */

/** @defgroup EPWM_Channel EPWM Channel
 * @{
 */
typedef enum
{
    EPWM_Channel_Less = (int32_t)0x00000000UL, /*!< No channels are selected */
    EPWM_Channel_0    = (int32_t)0x00000001UL, /*!< EPWM output channel 0 */
    EPWM_Channel_1    = (int32_t)0x00000002UL, /*!< EPWM output channel 1 */
    EPWM_Channel_2    = (int32_t)0x00000004UL, /*!< EPWM output channel 2 */
    EPWM_Channel_3    = (int32_t)0x00000008UL, /*!< EPWM output channel 3 */
    EPWM_Channel_4    = (int32_t)0x00000010UL, /*!< EPWM output channel 4 */
    EPWM_Channel_5    = (int32_t)0x00000020UL, /*!< EPWM output channel 5 */
    EPWM_Channel_6    = (int32_t)0x00000040UL, /*!< EPWM output channel 6 */
    EPWM_Channel_7    = (int32_t)0x00000080UL, /*!< EPWM output channel 7 */
    EPWM_Channel_All  = (int32_t)0x000000FFUL, /*!< EPWM output channel ALL */
} EPWM_Channel_Typedef;

#define IS_EPWM_CHANNEL(CHANNEL) (((CHANNEL) & (uint32_t)0xFFFFFF00) == 0x00)
/**
 * @}
 */

/** @defgroup EPWM_MAP EPWM MAP
 * @{
 */
typedef enum
{
    EPWM_MAP_Normal    = (uint16_t)(0x00U << EPWM_CON_MAP_Pos), /*!< EPWM MAP Normal */
    EPWM_MAP_Alternate = (uint16_t)(0x01U << EPWM_CON_MAP_Pos),	/*!< EPWM MAP Alternate */
} EPWM_MAP_Typedef;

#define IS_EPWM_MAP(MAP) (((MAP) & (uint32_t)0xFFFFFF00) == 0x00)
/**
 * @}
 */

/** @defgroup EPWM_SWAP EPWM SWAP
 * @{
 */
typedef enum
{
    EPWM_SWAP_Normal    = (uint16_t)(0x00U << EPWM_CON_SWAP_Pos), /*!<EPWM SWAP Normal */
    EPWM_SWAP_Alternate = (uint16_t)(0x01U << EPWM_CON_SWAP_Pos),	/*!< EPWM SWAP Alternate */
} EPWM_SWAP_Typedef;

#define IS_EPWM_SWAP(SWAP) (((SWAP) & (uint32_t)0xFFFFFF00) == 0x00)
/**
 * @}
 */

/** @defgroup EPWM_FDSoure EPWM FDMode
 * @{
 */
typedef enum
{
    EPWM_FDSoure_SW   = (uint16_t)EPWM_FLTCFG1_SWTRG,/*!< EPWM fault detection soure: SW One-shot */
    EPWM_FDSoure_OP2  = (uint16_t)EPWM_FLTCFG1_OP2,   /*!< EPWM fault detection soure: OP2 One-shot */
    EPWM_FDSoure_OP1  = (uint16_t)EPWM_FLTCFG1_OP1,    /*!< EPWM fault detection soure: OP1 One-shot */
    EPWM_FDSoure_CMP3  = (uint16_t)EPWM_FLTCFG1_CMP3,   /*!< EPWM fault detection soure: CMP3 One-shot*/
    EPWM_FDSoure_CMP0 = (uint16_t)EPWM_FLTCFG1_CMP0,   /*!< EPWM fault detection soure: CMP0 One-shot */
    EPWM_FDSoure_FLTPIN = (uint16_t)EPWM_FLTCFG1_FLTPIN, /*!< EPWM fault detection soure: External FLT One-shot */
} EPWM_FDSoure_TypeDef;

#define IS_EPWM_FDSOURE(SOURE) (((SOURE) == EPWM_FDSoure_SW)   || \
                                ((SOURE) == EPWM_FDSoure_OP2)  || \
                                ((SOURE) == EPWM_FDSoure_OP1)  || \
                                ((SOURE) == EPWM_FDSoure_CMP3) || \
                                ((SOURE) == EPWM_FDSoure_CMP0) || \
                                ((SOURE) == EPWM_FDSoure_FLTPIN))
/**
 * @}
 */


/** @defgroup EPWM_FDSoure EPWM FDMode
 * @{
 */
typedef enum
{
    EPWM_FDMODE_OS   = (uint16_t)0x00<<EPWM_FLTCFG1_SWEVT_Pos,   /*!< EPWM fault detection soure: SW One-shot */
    EPWM_FDMODE_CBC  = (uint16_t)0x01<<EPWM_FLTCFG1_SWEVT_Pos,   /*!< EPWM fault detection soure: SW cycle by cycle */
} EPWM_FDMode_TypeDef;

#define IS_EPWM_FDMODE(FDMODE) (((FDMODE) == EPWM_FDMode_OS) || \
                                 ((FDMODE) == EPWM_FDMode_CBC))
/**
 * @}
 */

/** @defgroup EPWM_FDSoure EPWM FDMode
 * @{
 */
typedef enum
{
    EPWM_FDTriggerLV_Low      = (uint32_t)0x00<<EPWM_FLTCFG1_OP2LV_Pos,/*!< EPWM fault detection soure: OP2 Low level fault is valid   */
    EPWM_FDTriggerLV_Hight    = (uint32_t)0x01<<EPWM_FLTCFG1_OP2LV_Pos, /*!< EPWM fault detection soure: OP2 hight level fault is valid */
} EPWM_FDTriggerLV_TypeDef;

#define IS_EPWM_TriggerLV(TriggerLV) (((TriggerLV) == EPWM_FDTriggerLV_LOW) || \
                                      ((TriggerLV) == EPWM_FDTriggerLV_Hight))
/**
 * @}
 */


/** @defgroup EPWM_PinRemap EPWM PinRemap
 * @{
 */

typedef enum
{
	  EPWM_PinRemap_Default	= (uint16_t)(0x00U)<<EPWM_CON_SPOS_Pos,  /*!< EPWM PinRemap Default */
    EPWM_PinRemap_A  = (uint16_t)(0x01U)<<EPWM_CON_SPOS_Pos,  /*!< EPWM PinRemap A  */
} EPWM_PinRemap_TypeDef;

#define IS_EPWM_PinRemap(PinRemap) (((PinRemap) == EPWM_PinRemap_Default) || \
                                   ((PinRemap) == EPWM_PinRemap_A))

/**
 * @}
 */

/** @defgroup EPWM_FDVoltage EPWM FDVoltage
 * @{
 */
typedef enum
{
    EPWM_FDOutputState_HI = (uint32_t)(0x00U << EPWM_FLTCFG1_FLTBRK_Pos),  /*!< EPWM fault detection Voltage:Low level */
    EPWM_FDOutputState_PP = (uint32_t)(0x01U << EPWM_FLTCFG1_FLTBRK_Pos),  /*!< EPWM fault detection Voltage:High level */
} EPWM_FDOutputState_TypeDef;

#define IS_EPWM_FDOUTPUTSTATE(STATE) (((STATE) == EPWM_FDOutputState_HI) || \
                                      ((STATE) == EPWM_FDOutputState_PP))
/**
 * @}
 */

/** @defgroup EPWM_FDVoltage EPWM FDVoltage
 * @{
 */
typedef enum
{
    EPWM_FDOutputLevel_Low  = (uint16_t)(0x00U),	/*!< EPWM fault detection Voltage:Low level */
    EPWM_FDOutputLevel_High = (uint16_t)(0x01U),  /*!< EPWM fault detection Voltage:High level */
} EPWM_FDOutputLevel_TypeDef;

#define IS_EPWM_FDOUTPUTLEVEL(LEVEL) (((LEVEL) == EPWM_FDOutputLevel_Low) || \
                                      ((LEVEL) == EPWM_FDOutputLevel_High))
/**
 * @}
 */


/** @defgroup EPWM_FDFilteringTime EPWM FDFilteringTime
 * @{
 */
typedef enum
{
    EPWM_FilteringTime_0us      = (uint16_t)(0x00U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 0us */
    EPWM_FilteringTime_0_2us   = (uint16_t)(0x01U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 0.25us */
    EPWM_FilteringTime_0_4us    = (uint16_t)(0x02U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 0.5us */
    EPWM_FilteringTime_0_8us      = (uint16_t)(0x03U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 1us */
    EPWM_FilteringTime_1_7us      = (uint16_t)(0x04U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 2us */
    EPWM_FilteringTime_3_5us      = (uint16_t)(0x05U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 4us */
    EPWM_FilteringTime_7us      = (uint16_t)(0x06U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 8us */
    EPWM_FilteringTime_14_1us      = (uint16_t)(0x07U << EPWM_FLTCFG0_FILTER_Pos), /*!< EPWM fault detection input signal filtering time is 16us */
} EPWM_FDFilteringTime_TypeDef;

#define IS_EPWM_FDFILTERINGTIME(TIME) (((TIME) == EPWM_FilteringTime_0us)    || \
                                       ((TIME) == EPWM_FilteringTime_0_2us) || \
                                       ((TIME) == EPWM_FilteringTime_0_4us)  || \
                                       ((TIME) == EPWM_FilteringTime_0_8us)    || \
                                       ((TIME) == EPWM_FilteringTime_1_7us)    || \
                                       ((TIME) == EPWM_FilteringTime_3_5us)    || \
                                       ((TIME) == EPWM_FilteringTime_7us)    || \
                                       ((TIME) == EPWM_FilteringTime_14_1us))
/**
 * @}
 */

/** @defgroup EPWM DeadTime
 * @{
 */
typedef enum
{
    EPWM_DeadTime_0us        = (uint8_t)0x00U, /*!< EPWM fault detection input signal filtering time is 0us */
    EPWM_DeadTime_0_0625us   = (uint8_t)0x01U, /*!< EPWM fault detection input signal filtering time is 0.0625us */
    EPWM_DeadTime_0_125us    = (uint8_t)0x02U, /*!< EPWM fault detection input signal filtering time is 0.125us */
    EPWM_DeadTime_0_1875us   = (uint8_t)0x03U, /*!< EPWM fault detection input signal filtering time is 0.1875us */
    EPWM_DeadTime_0_25us     = (uint8_t)0x04U, /*!< EPWM fault detection input signal filtering time is 0.25us */
    EPWM_DeadTime_0_3125us   = (uint8_t)0x05U, /*!< EPWM fault detection input signal filtering time is 0.3125us */
    EPWM_DeadTime_0_375us    = (uint8_t)0x06U, /*!< EPWM fault detection input signal filtering time is 0.375us */
    EPWM_DeadTime_0_4375us   = (uint8_t)0x07U, /*!< EPWM fault detection input signal filtering time is 0.4375us */
    EPWM_DeadTime_3_9375us   = (uint8_t)0x3FU, /*!< EPWM fault detection input signal filtering time is 3.975us */
} EPWM_DeadTime_TypeDef;
/**
 * @}
 */




/** @defgroup EPWM_IT EPWM direction
 * @{
 */
typedef enum
{
    EPWM_ADCTrigDir_Up = (uint16_t)0x00,	/*!<Up-count trigger */
    EPWM_ADCTrigDir_Down = (uint16_t)0x01,	/*!< Down-count trigger */
} EPWM_ADCTrigDir_TypeDef;

#define IS_EPWM_TRIGDIR(DIR) ( ((DIR) == EPWM_ADCTrigDir_Up)   || \
                               ((DIR) == EPWM_ADCTrigDir_Down))

/**
 * @}
 */

/** @defgroup EPWM_IT EPWM Interrupt
 * @{
 */
typedef enum
{
    EPWM_ADCTrigGroup0 = (uint16_t)0x00,	/*!< EPWM trigger Group0 */
    EPWM_ADCTrigGroup1 = (uint16_t)0x01,	/*!< EPWM trigger Group1 */
    EPWM_ADCTrigGroup2 = (uint16_t)0x02,	/*!< EPWM trigger Group2 */
    EPWM_ADCTrigGroup3 = (uint16_t)0x03,	/*!< EPWM trigger Group3 */
} EPWM_ADCTrigGroup_TypeDef;

#define IS_EPWM_ADCTrigGroup(ADCTrigGroup)  (((ADCTrigGroup) == EPWM_ADCTrigGroup0) || \
                                            ((ADCTrigGroup) == EPWM_ADCTrigGroup1) ||\
                                            ((ADCTrigGroup) == EPWM_ADCTrigGroup2) ||\
                                            ((ADCTrigGroup) == EPWM_ADCTrigGroup3))

/**
 * @}
 */

/** @defgroup EPWM_IT EPWM Interrupt
 * @{
 */
typedef enum
{
    EPWM_IT_OVF = (uint16_t)EPWM_IDE_OVFIE,	/*!< EPWM Interrupt: EPWM OVF */
    EPWM_IT_UNF = (uint16_t)EPWM_IDE_UNFIE,	/*!< EPWM Interrupt: EPWM UNF */
    EPWM_IT_CBC = (uint16_t)EPWM_IDE_CBCIE,	/*!< EPWM Interrupt: EPWM CBC */
    EPWM_IT_OST = (uint16_t)EPWM_IDE_OSTIE,	/*!< EPWM Interrupt: EPWM OST */
    EPWM_IT_INTEN = (uint16_t)EPWM_IDE_INTEN,	/*!< EPWM Interrupt: EPWM INTEN */
} EPWM_IT_TypeDef;

#define IS_EPWM_IT(IT) ((((IT) & (uint16_t)0xFEFF) == 0x00) && ((IT) != (uint16_t)0x0000))

/**
 * @}
 */

/** @defgroup EPWM_Flag EPWM Flag
 * @{
 */
typedef enum
{
    EPWM_Flag_OVF = (uint8_t)EPWM_STS_OVFIF,  /*!< EPWM OVF Interrupt Flag*/
    EPWM_Flag_UNF = (uint8_t)EPWM_STS_UNFIF,  /*!< EPWM UNF Interrupt Flag*/
    EPWM_Flag_CBC = (uint16_t)EPWM_STS_CBCIF, /*!< EPWM CBC Interrupt Flag*/
    EPWM_Flag_OST = (uint16_t)EPWM_STS_OSTIF, /*!< EPWM OST Interrupt Flag*/

} EPWM_Flag_TypeDef;

#define IS_EPWM_FLAG(FLAG) ((((FLAG) & (uint8_t)0xFC) == 0x00) && ((FLAG) != (uint8_t)0x00))

#define IS_GET_EPWM_FLAG(FLAG) (((FLAG) == EPWM_Flag_PWMIF) || \
                                ((FLAG) == EPWM_Flag_FLTSTA))
/**
 * @}
 */


/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/

/** @defgroup EPWM_Constants EPWM Constants
  * @{
  */
#define IS_EPWM_ALL_PERIPH(PERIPH) ((PERIPH) == EPWM)

/**
 * @}
 */

/* Exported struct ------------------------------------------------------------*/
/** @defgroup EPWM_Exported_Struct EPWM Exported Struct
 * @{
 */

/** @defgroup PWM Time base Configuration Structure definition
 * @{
 */
typedef struct
{
    uint16_t EPWM_Prescaler; /*!<  Specifies the predivision.
																			This parameter can be a value of @ref EPWM_Prescaler_TypeDef */

    uint16_t EPWM_AlignedMode; /*!< Specifies the aligne mode.
																			This parameter can be a value of @ref EPWM_AlignedMode_TypeDef */

    uint16_t EPWM_WorkMode; /*!< Specifies the work mode.
																					This parameter can be a value of @ref EPWM_WorkMode_TypeDef */

    uint16_t EPWM_Cycle;   /*!< Specifies the cycle.*/

    uint32_t EPWM_OutputChannel; /*!< Specifies the output Channel.
																			 This parameter can be a value of @ref EPWM_Channel_Typedef */

    uint32_t EPWM_LowPolarityChannl; /*!< Specifies the invert Channel.
																				 This parameter can be a value of @ref EPWM_Channel_Typedef */
} EPWM_InitTypeDef;
/**
 * @}
 */

/** @defgroup EPWM_FDInitTypeDef
 * @{
 */
typedef struct
{
    uint16_t EPWM_FDSoure;			/*!<  Specifies the EPWM fault detection Soure.
																										 This parameter can be a value of @ref EPWM_FDSoure_TypeDef */
	
	  uint16_t EPWM_FDMode;			/*!<  Specifies the EPWM fault detection mode.
																										 This parameter can be a value of @ref EPWM_FDMode_TypeDef */

    uint32_t EPWM_FDTriggerLV;		    /*!<  Specifies the EPWM fault detection active level.
																							 This parameter can be a value of @ref EPWM_FDTriggerLV_TypeDef */

    uint32_t EPWM_FDFilteringTime;    /*!<  Specifies the EPWM fault detection filtering time.
																			 This parameter can be a value of @ref EPWM_FDFilteringTime_TypeDef */
	
	  FunctionalState EPWM_FDSoureCmd;			/*!<  Specifies the EPWM fault detection mode. */

} EPWM_FDInitTypeDef;
/**
 * @}
 */

/**
 * @}
 */
/* End of struct -----------------------------------------------------*/

/** @addtogroup EPWM_Functions PWM Functions
 * @{
 */

#define EPWM_SetDuty_Fast(ChannelNum,PWM_Duty) EPWM->EPWM_DT[ChannelNum] = PWM_Duty;
#define EPWM_Asymmetry_SetDuty_Fast(ChannelNum,EPWM_CmpUp,EPWM_CmpDown) EPWM->EPWM_DT[ChannelNum] = EPWM_CmpUp;EPWM->EPWM_DT[ChannelNum+1] = EPWM_CmpDown;
#define EPWM_FDSoftwareTrigger() EPWM->EPWM_FLT_CFG1 |= EPWM_FLTCFG1_SWTRG

//#define EPWM_SetDuty_Fast(EPWMx,ChannelNum,PWM_Duty) EPWMx->EPWM_DT[ChannelNum] = PWM_Duty;
//#define EPWM_Asymmetry_SetDuty_Fast(EPWMx,ChannelNum,EPWM_CmpUp,EPWM_CmpDown) EPWMx->EPWM_DT[ChannelNum] = EPWM_CmpUp;EPWMx->EPWM_DT[ChannelNum+1] = EPWM_CmpDown;
//#define EPWM_FDSoftwareTrigger(EPWMx) EPWMx->EPWM_FLT_CFG1 |= EPWM_FLTCFG1_SWTRG
#define EPWM_SetADCTrig_Fast(EPWMx,Group,Dir,Cnt) ((Dir)?(EPWMx->EPWM_ADCTRGDT[Group] = EPWM_ADCTRG_DIR | Cnt):(EPWMx->EPWM_ADCTRGDT[Group] = Cnt))
#define EPWM_SetADCTrigCnt_Fast(EPWMx,Group,Cnt) (*((uint16_t *)&EPWMx->EPWM_ADCTRGDT[Group]) = Cnt)
/**
 * @}
 */
/* End of functions --------------------------------------------------*/

/** @addtogroup EPWM_Functions PWM Functions
 * @{
 */

/* PWM Base functions ********************************************************/
void EPWM_DeInit(EPWM_TypeDef* EPWMx);
void EPWM_StructInit(EPWM_InitTypeDef* EPWM_InitStruct);
void EPWM_Init(EPWM_TypeDef* EPWMx, EPWM_InitTypeDef* EPWM_InitStruct);
void EPWM_RisingDeadTimeConfig(EPWM_TypeDef* EPWMx, uint8_t EPWM_RisingDeadTime);
void EPWM_FallingDeadTimeConfig(EPWM_TypeDef* EPWMx, uint8_t EPWM_FallingDeadTime);
void EPWM_Cmd(EPWM_TypeDef* EPWMx, FunctionalState NewState);
void EPWM_MAPConfig(EPWM_TypeDef* EPWMx, EPWM_MAP_Typedef EPWM_MAP);
void EPWM_SWAPConfig(EPWM_TypeDef* EPWMx, EPWM_SWAP_Typedef EPWM_SWAP);
void EPWM_SetPrescaler(EPWM_TypeDef* EPWMx, EPWM_Prescaler_TypeDef EPWM_Prescaler);
EPWM_Prescaler_TypeDef EPWM_GetPrescaler(EPWM_TypeDef* EPWMx);
void EPWM_SetCycle(EPWM_TypeDef* EPWMx, uint32_t EPWM_Cycle);
uint16_t EPWM_GetCycle(EPWM_TypeDef* EPWMx);
void EPWM_SetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel, uint16_t EPWM_Duty);
void EPWM_AsymmetryCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState);
void EPWM_Asymmetry_SetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel, uint16_t EPWM_CmpUp, uint16_t EPWM_CmpDown);
uint16_t EPWM_GetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel);


/* PWM Falut Dectection functions **********************************************/
void EPWM_FDStructInit(EPWM_FDInitTypeDef* EPWM_FDInitStruct);
void EPWM_FDInit(EPWM_TypeDef* EPWMx, EPWM_FDInitTypeDef* EPWM_FDInitStruct);
void EPWM_FDSoftwareStart(EPWM_TypeDef* EPWMx);
void EPWM_FD_HALTMSCmd(EPWM_TypeDef* EPWMx,FunctionalState NewState);
void EPWM_FDCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState);
void EPWM_FD_HALTMSCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState);
void EPWM_FDOutputStateConfig(EPWM_TypeDef* EPWMx,EPWM_FDOutputState_TypeDef EPWM_FDOutputState);
void EPWM_FDOutputLevelConfig(EPWM_TypeDef* EPWMx,uint16_t EPWM_Channel,EPWM_FDOutputLevel_TypeDef EPWM_FDOutputLevel);
void EPWM_FDModeConfig(EPWM_TypeDef* EPWMx, EPWM_FDMode_TypeDef EPWM_FDMode, FunctionalState NewState);
void EPWM_TriggerLVConfig(EPWM_TypeDef* EPWMx, EPWM_FDTriggerLV_TypeDef EPWM_FDTriggerLV);
/* PWM ADC Trig functions **********************************************/
void EPWM_SetADCTrigDir(EPWM_TypeDef* EPWMx,EPWM_ADCTrigGroup_TypeDef EPWM_ADCTrigGroup, EPWM_ADCTrigDir_TypeDef EPWM_TrigDir);
void EPWM_SetADCTrigCnt(EPWM_TypeDef* EPWMx,EPWM_ADCTrigGroup_TypeDef EPWM_ADCTrigGroup, uint32_t ADC_TrigCnt);

/* Interrupt Handler functions  ***********************************************/
void EPWM_PinRemapConfig(EPWM_TypeDef* EPWMx, EPWM_PinRemap_TypeDef EPWM_Remap);
void EPWM_ITConfig(EPWM_TypeDef* EPWMx, uint16_t EPWM_IT, FunctionalState NewState);
FlagStatus EPWM_GetFlagStatus(EPWM_TypeDef* EPWMx, uint16_t EPWM_FLAG);
void EPWM_ClearFlag(EPWM_TypeDef* EPWMx, uint16_t EPWM_FLAG);

/**
 * @}
 */
/* End of functions --------------------------------------------------*/

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
