/**
 ******************************************************************************
 * @file    sc32m1xxx_OP.h
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   Header file of OP module.
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
#ifndef __sc32m1xxx_OP_H
#define __sc32m1xxx_OP_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"

/** @addtogroup sc32m1xxx_StdPeriph_Driver
 * @{
 */

/** @addtogroup OP
 * @{
 */

/* Exported enumerations ------------------------------------------------------------*/
/** @defgroup OP_Exported_Enumerations OP Exported Enumerations
 * @{
 */

/**
 * @}
 */

/** @defgroup OP_OutputPin OP Outout Pin
* @{
*/
typedef enum
{
    OP_OutputPin_Disable  = (uint32_t)(0x00U << OP_CON_OPOSEL_Pos),     /*!< The OP output pin is Enable   */  
    OP_OutputPin_Enable   = (uint32_t)(0x01U << OP_CON_OPOSEL_Pos),     /*!< The OP output pin is Disable  */  
} OP_OutputPin_TypeDef;

#define IS_OP_OUTPUTPIN(PIN) (((PIN) == OP_OutputPin_Enable) ||   \
											        ((PIN) == OP_OutputPin_Disable))
/**
 * @}
 */

/** @defgroup OP_PhaseErrorDet Phase error detection
* @{
*/
typedef enum
{
    OP_InvertInput_OPxN  = (uint32_t)(0x00U << OP_CON_OPNSEL_Pos),  /*!< The Invert input is OPxN  */
    OP_InvertInput_DAC   = (uint32_t)(0x01U << OP_CON_OPNSEL_Pos),  /*!< The Invert input is DAC   */
    OP_InvertInput_OPRF  = (uint32_t)(0x02U << OP_CON_OPNSEL_Pos),  /*!< The Invert input is OPRF  */
    OP_InvertInput_Res   = (uint32_t)(0x03U << OP_CON_OPNSEL_Pos),  /*!< The Invert input is Res   */
} OP_InvertInput_TypeDef;

#define IS_OP_INVERTINPUT(INPUT) (((INPUT) == OP_InvertInput_OPxN) || \
                                    ((INPUT) == OP_InvertInput_DAC) || \
                                    ((INPUT) == OP_InvertInput_OPRF) || \
                                    ((INPUT) == OP_InvertInput_Res))
/**
 * @}
 */

/** @defgroup OP_PhaseErrorDet Phase error detection
* @{
*/
typedef enum
{
    OP_NonInvertInput_VSS       = (uint32_t)(0x00U << OP_CON_OPPSEL_Pos),  /*!< The no Invert input is VSS  */
    OP_NonInvertInput_VREF_Div2 = (uint32_t)(0x02U << OP_CON_OPPSEL_Pos),  /*!< The no Invert input is VREF_Div2  */
    OP_NonInvertInput_OPxP      = (uint32_t)(0x03U << OP_CON_OPPSEL_Pos),  /*!< The no Invert input is OPxP    */
} OP_NonInvertInput_TypeDef;

#define IS_OP_NonInvertInput(NonInvertInput) (((NonInvertInput) == OP_NonInvertInput_VSS) || \
                                    ((NonInvertInput) == OP_NonInvertInput_VREF_Div2) || \
                                    ((NonInvertInput) == OP_NonInvertInput_OPxP))
/**
 * @}
 */

/** @defgroup OP_Prescaler OP Prescaler
* @{
*/
typedef enum
{
    OP_GAN_Invert3  = (uint32_t)(0x00U << OP_CON_PGAGAN_Pos),     /*!< The OP inverting input gain is 3  times  */  
    OP_GAN_Invert7  = (uint32_t)(0x01U << OP_CON_PGAGAN_Pos),     /*!< The OP inverting input gain is 7  times  */  
    OP_GAN_Invert15 = (uint32_t)(0x02U << OP_CON_PGAGAN_Pos),     /*!< The OP inverting input gain is 15 times  */  
    OP_GAN_Invert31 = (uint32_t)(0x03U << OP_CON_PGAGAN_Pos),     /*!< The OP inverting input gain is 31 times  */  

    OP_GAN_NonInvert4    = (uint32_t)(0x00U << OP_CON_PGAGAN_Pos),  /*!< The OP in-phase input gain is 3  times  */
    OP_GAN_NonInvert8    = (uint32_t)(0x01U << OP_CON_PGAGAN_Pos),  /*!< The OP in-phase input gain is 7  times  */
    OP_GAN_NonInvert16   = (uint32_t)(0x02U << OP_CON_PGAGAN_Pos),  /*!< The OP in-phase input gain is 15 times  */
    OP_GAN_NonInvert32   = (uint32_t)(0x03U << OP_CON_PGAGAN_Pos),  /*!< The OP in-phase input gain is 31 times  */
} OP_GAN_TypeDef;

#define IS_OP_GAN(GAN) (((GAN) == OP_GAN_Invert3) ||   \
												((GAN) == OP_GAN_Invert7) ||   \
												((GAN) == OP_GAN_Invert15) ||   \
												((GAN) == OP_GAN_Invert31))
/**
 * @}
 */

/** @defgroup OP_FailingCap Failing edge capture
* @{
*/
typedef enum
{
	OP_OPRF_1D16OP_VREF   = (uint32_t)(0x01U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 1D16OP_VREF   */
    OP_OPRF_2D16OP_VREF   = (uint32_t)(0x02U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 2D16OP_VREF   */
    OP_OPRF_3D16OP_VREF   = (uint32_t)(0x03U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 3D16OP_VREF   */
    OP_OPRF_4D16OP_VREF   = (uint32_t)(0x04U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 4D16OP_VREF   */
    OP_OPRF_5D16OP_VREF   = (uint32_t)(0x05U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 5D16OP_VREF   */
    OP_OPRF_6D16OP_VREF   = (uint32_t)(0x06U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 6D16OP_VREF   */
    OP_OPRF_7D16OP_VREF   = (uint32_t)(0x07U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 7D16OP_VREF   */
    OP_OPRF_8D16OP_VREF   = (uint32_t)(0x08U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 8D16OP_VREF  */
    OP_OPRF_9D16OP_VREF   = (uint32_t)(0x09U << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 9D16OP_VREF   */
    OP_OPRF_10D16OP_VREF  = (uint32_t)(0x0AU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 10D16OP_VREF   */
    OP_OPRF_11D16OP_VREF  = (uint32_t)(0x0BU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 11D16OP_VREF  */
    OP_OPRF_12D16OP_VREF  = (uint32_t)(0x0CU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 12D16OP_VREF   */
    OP_OPRF_13D16OP_VREF  = (uint32_t)(0x0DU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 13D16OP_VREF   */
    OP_OPRF_14D16OP_VREF  = (uint32_t)(0x0EU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 14D16OP_VREF  */
    OP_OPRF_15D16OP_VREF  = (uint32_t)(0x0FU << OP_CON_OPRF_Pos),  /*!< Opamp inverter input voltage select 15D16OP_VREF  */
} OP_OPRF_TypeDef;

#define IS_OP_FAILINGCAP(OPRF) (((OPRF) == OP_OPRF_1D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_2D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_3D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_4D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_5D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_6D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_7D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_8D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_9D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_10D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_11D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_12D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_13D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_14D16OP_VREF) || \
                                  ((OPRF) == OP_OPRF_15D16OP_VREF))
/**
 * @}
 */

/** @defgroup OP_VREF OP Reference Voltage
 * @{
 */
typedef enum
{
    OP_RefSource_VDD  = (uint32_t)(0x00 << OP_CFG_REFSEL_Pos),	  /*!< OP reference voltage is VDD    */
    OP_RefSource_VREF = (uint32_t)(0x01 << OP_CFG_REFSEL_Pos),	  /*!< OP reference voltage is Vref */
} OP_VREF_TypeDef;

#define IS_OP_VREF(VREF) (((VREF) == OP_RefSource_VDD) || \
                          ((VREF) == OP_RefSource_VREF))
/**
 * @}
 */

/** @defgroup OP_Negative OP NEGATIVE
 * @{
 */
typedef enum
{
    OP_FDBR_VSS   = (uint32_t)(0x00U << OP_CON_FDBRSEL_Pos),    /*!< Feedback resistor R1 connection select  VSS */
    OP_FDBR_OPxN = (uint32_t)(0x01U << OP_CON_FDBRSEL_Pos),    /*!<  Feedback resistor R1 connection select  OPxN  */
}OP_FDBR_Typedef;

#define IS_OP_FDBR(FDBR) (((FDBR) == OP_FDBR_VSS) || \
                                 ((FDBR) == OP_FDBR_OPxN))

/**
 * @}
 */

/** @defgroup OP_Negative OP NEGATIVE
 * @{
 */
typedef enum
{
    OP_TriggerMode_Disable   = (uint32_t)(0x00U),    /*!< the trigger mode is disable  */
    OP_TriggerMode_RISE      = (uint32_t)(0x01U),    /*!< The trigger mode of the simulated comparator is rising edge  */
    OP_TriggerMode_FALL      = (uint32_t)(0x02U),    /*!< The trigger mode of simulated comparator is falling edge   */
    OP_TriggerMode_RISE_FALL = (uint32_t)(0x03U),    /*!< The trigger mode of the simulated comparator is rising edge and falling edge  */
}OP_TriggerMode_TypeDef;

#define IS_OP_TRIGGER(TRIGGER) (((TRIGGER) == OP_TriggerMode_Disable) || \
                                 ((TRIGGER) == OP_TriggerMode_RISE) || \
                                 ((TRIGGER) == OP_TriggerMode_FALL) || \
                                 ((TRIGGER) == OP_TriggerMode_RISE_FALL))

/**
 * @}
 */

/** @defgroup CMP_Flag CMP Flag
 * @{
 */
typedef enum
{
	  OP_CMPSTA_Low		= (uint32_t)(0x00), /*!< CMP Flag: The non-inverting input is at a lower voltage than the inverting input */
    OP_CMPSTA_High  = (uint32_t)(0x01), /*!< CMP Flag: The non-inverting input is at a higher voltage than the inverting input */
} OP_CMPSTA_TypeDef;

#define IS_OP_CMPSTA(STA) (((STA) == OP_CMPSTA_Low) || \
                           ((STA) == OP_CMPSTA_High))
/**
 * @}
 */

/** @defgroup OP_IT OP Interrupt
 * @{
 */
typedef enum
{
    OP_IT_INT       = (uint8_t)OP_IDE_INTEN,      /*!< OP Interrupt: OP Interrupt */
    OP_IT_OPCMP1    = (uint8_t)OP_IDE_OPCMP1IE,   /*!< OP Interrupt: OPCMP1 interrupt */
    OP_IT_OPCMP2    = (uint8_t)OP_IDE_OPCMP2IE,   /*!< OP Interrupt: OPCMP2 interrupt*/
} OP_IT_TypeDef;

#define IS_OP_IT(IT) ((((IT) & (uint8_t)0xF7) == 0x00) && ((IT) != 0x00))
/**
 * @}
 */

/** @defgroup OP_FLAG OP Flag
 * @{
 */
typedef enum
{
    OP_FLAG_OPCMP1    = (uint8_t)OP_STS_OPCMP1IF,     /*!< OP Flag: OPCMP1 flag */
    OP_FLAG_OPCMP2    = (uint8_t)OP_STS_OPCMP2IF,     /*!< OP Flag: OPCMP2 flag */
} OP_FLAG_TypeDef;

#define IS_OP_FLAG(FLAG) ((((FLAG) & (uint8_t)0xFC) == 0x00) && ((FLAG) != 0x00))

#define IS_GET_OP_FLAG(FLAG) (((FLAG) == OP_FLAG_OPCMP1) || \
                               ((FLAG) == OP_FLAG_OPCMP2))
/**
 * @}
 */
#define IS_OP_ALL_PERIPH(PERIPH) (((PERIPH) == OP0) ||\
                                   ((PERIPH) == OP1) ||\
                                   ((PERIPH) == OP2))
/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/

/** @defgroup OP_Constants OP Constants
  * @{
  */

/**
 * @}
 */
/* End of constants -----------------------------------------------------*/

/** @defgroup OP Time base Configuration Structure definition
 * @{
 */
typedef struct
{
	  uint8_t OP_REFSEL;     /*!<  This member configures OP REFSEL.
																				This parameter can be a value of @ref OP_VREF_TypeDef */
	
	  uint8_t OP_CMPIM2;     /*!<   This member configures OP CMPIM2.
																				This parameter can be a value of @ref OP_TriggerMode_TypeDef */
	
	  uint8_t OP_CMPIM1;     /*!<   This member configures OP CMPIM1.
																				This parameter can be a value of @ref OP_TriggerMode_TypeDef */
	
    uint8_t OP_OutputPin;     /*!<  This member configures OP OutputPin.
																				This parameter can be a value of @ref OP_OutputPin_TypeDef */

    uint8_t OP_InvertInput; /*!<  This member configures  OP InvertInput.
											               When OP_Mode is mode 0 This parameter can be a value of @ref OP_InvertInput_TypeDef. */

    uint8_t OP_NonInvertInput; /*!< This member configures OP NonInvertInput.
											                  When OP_Mode is mode 0 This parameter can be a value of @ref OP_NonInvertInput_TypeDef. */

    uint16_t OP_GAN; /*!<   This member configures OP GAN.
																						This parameter can be a value of @ref OP_GAN_TypeDef */

    uint32_t OP_OPRF; /*!<  This member configures OP OPRF.
																						This parameter can be a value of @ref OP_OPRF_TypeDef */
	
	  uint32_t OP_FDBR; /*!<  This member configures OP FDBR.
																						This parameter can be a value of @ref OP_FDBR_Typedef*/
	
} OP_InitTypeDef;
/**
 * @}
 */

/**
 * @}
 */
/* End of exported enumerations -----------------------------------------------------*/

/** @addtogroup OP_Functions OP Functions
 * @{
 */

/* OP Base functions ********************************************************/
void OP_DeInit(OP_TypeDef* OPx);
void OP_Init(OP_TypeDef* OPx, OP_InitTypeDef* OP_InitStruct);
void OP_Cmd(OP_TypeDef* OPx, FunctionalState NewState);
void OP_OffsetTrimConfig(OP_TypeDef* OPx,  uint32_t OP_TrimValueH, uint32_t OP_TrimValueL);
void OP_StructInit(OP_InitTypeDef* OP_InitStruet);
void OP_InputSelection(OP_TypeDef* OPx,OP_InvertInput_TypeDef OPInvertInput);
void OP_ReferenceVref(OP_TypeDef* OPx,OP_VREF_TypeDef OPVREF);
void OP_GainSelection(OP_TypeDef* OPx, OP_GAN_TypeDef OPGain);
void OP_OutputSelection(OP_TypeDef* OPx, OP_OutputPin_TypeDef OPOutput);
void OP_NonInvertInputSelection(OP_TypeDef* OPx, OP_NonInvertInput_TypeDef OPNonInvertInput);
void OP_InverInputVoltage(OP_TypeDef* OPx, OP_OPRF_TypeDef OP_OPRF);
/* Data transfers functions ********************************************************/
void OP_CMPTriggerCmd(OP_TypeDef* OPx, FunctionalState NewState);
void OP_CMPTriggerConfig(OP_TypeDef* OPx, OP_TriggerMode_TypeDef TriggerMode);
/*  Read check data functions **********************************************/
uint8_t readOP0P_Offset(void);
uint8_t readOP0N_Offset(void);
uint8_t readOP1P_Offset(void);
uint8_t readOP1N_Offset(void);
uint8_t readOP2P_Offset(void);
uint8_t readOP2N_Offset(void);
/* Interrupts, DMA and flags management functions  **********************************************/
OP_CMPSTA_TypeDef OP_GetCMPSTA(OP_TypeDef* OPx);
void OP_ITConfig(uint16_t OP_IT, FunctionalState NewState);
FlagStatus OP_GetFlagStatus(OP_FLAG_TypeDef OP_FLAG);
void OP_ClearFlag(uint16_t OP_FLAG);

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
