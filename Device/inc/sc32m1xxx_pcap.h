/**
 ******************************************************************************
 * @file    sc32m1xxx_PCAP.h
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   Header file of PCAP module.
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
#ifndef __sc32m1xxx_PCAP_H
#define __sc32m1xxx_PCAP_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"



/* Exported enumerations ------------------------------------------------------------*/
/** @defgroup PCAP_Exported_Enumerations PCAP Exported Enumerations
 * @{
 */

/** @defgroup PCAP_Prescaler PCAP Prescaler
* @{
*/
typedef enum
{
  PCAP_PRESCALER_DIV1	   = (uint16_t)(0x00U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/1    */
  PCAP_PRESCALER_DIV2	   = (uint16_t)(0x01U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/2    */
  PCAP_PRESCALER_DIV4	   = (uint16_t)(0x02U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/4    */
  PCAP_PRESCALER_DIV8	   = (uint16_t)(0x03U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/8    */
  PCAP_PRESCALER_DIV16	 = (uint16_t)(0x04U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/16    */
  PCAP_PRESCALER_DIV32	 = (uint16_t)(0x05U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/32    */
  PCAP_PRESCALER_DIV64	 = (uint16_t)(0x06U << PCAP_CON_PCAPCK_Pos),	/*!< Clock division: Fsource/64    */
  PCAP_PRESCALER_DIV128  = (uint16_t)(0x07U << PCAP_CON_PCAPCK_Pos),  /*!< Clock division: Fsource/128    */
} PCAP_PRESCALER_TypeDef;

#define IS_PCAP_PRESCALER(PRESCALER) (((PRESCALER) == PCAP_PRESCALER_DIV1) ||   \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV2) ||   \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV4) ||   \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV8) ||   \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV16) ||  \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV32) ||  \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV64) ||  \
																		 ((PRESCALER) == PCAP_PRESCALER_DIV128))
/**
 * @}
 */

/** @defgroup PCAP_PhaseErrorDet Phase error detection
* @{
*/
typedef enum
{
  PCAP_PhaseErrorDet_Disable   = (uint32_t)(0x00U << PCAP_CON_PHASEMODE_Pos),  /*!< The phase error detection function was disable    */
  PCAP_PhaseErrorDet_Enable    = (uint32_t)(0x01U << PCAP_CON_PHASEMODE_Pos),  /*!< The phase error detection function was enabled   */
} PCAP_PhaseErrorDet_TypeDef;

#define IS_PCAP_PHASEERRORDET(DET) (((DET) == PCAP_PhaseErrorDet_Disable) || \
                                    ((DET) == PCAP_PhaseErrorDet_Enable))
/**
 * @}
 */

/** @defgroup PCAP_RisingCap Rising edge capture
* @{
*/
typedef enum
{
  PCAP_RisingCap_Disable   = (uint32_t)(0x00U << PCAP_CON_RCAPEN_Pos),  /*!< The PCAP rising edge capture module was disable   */
  PCAP_RisingCap_Enable    = (uint32_t)(0x01U << PCAP_CON_RCAPEN_Pos),  /*!< The PCAP rising edge capture module was enabled   */
} PCAP_RisingCap_TypeDef;

#define IS_PCAP_RISINGCAP(RCAP) (((RCAP) == PCAP_RisingCap_Disable) || \
                                  ((RCAP) == PCAP_RisingCap_Enable))
/**
 * @}
 */

/** @defgroup PCAP_FailingCap Failing edge capture
* @{
*/
typedef enum
{
  PCAP_FailingCap_Disable   = (uint32_t)(0x00U << PCAP_CON_FCAPEN_Pos),  /*!< The PCAP failing edge capture module was disable   */
  PCAP_FailingCap_Enable    = (uint32_t)(0x01U << PCAP_CON_FCAPEN_Pos),  /*!< The PCAP failing edge capture module was enabled   */
} PCAP_FailingCap_TypeDef;

#define IS_PCAP_FAILINGCAP(RCAP) (((RCAP) == PCAP_RisingCap_Disable) || \
                                  ((RCAP) == PCAP_RisingCap_Enable))
/**
 * @}
 */

/** @defgroup PCAP_FailingCap Failing edge capture
* @{
*/
typedef enum
{
  PCAP_INV_None = (uint32_t)(0x00U),          /*!< No signal to flip   */
  PCAP_INV_CAP0 = (uint32_t)(PCAP_CON_INV0),  /*!< The signal captured by port CAP0 is flipped   */
	PCAP_INV_CAP1 = (uint32_t)(PCAP_CON_INV1),  /*!< The signal captured by port CAP1 is flipped   */
	PCAP_INV_CAP2 = (uint32_t)(PCAP_CON_INV2),  /*!< The signal captured by port CAP2 is flipped    */
} PCAP_INV_TypeDef;

#define IS_PCAP_INV(CAP) (((RCAP) == PCAP_INV_None) || \
													((RCAP) == PCAP_INV_CAP0) || \
													((RCAP) == PCAP_INV_CAP1) || \
                          ((RCAP) == PCAP_INV_CAP2))
/**
 * @}
 */

/** @defgroup PCAP_Prescaler PCAP Prescaler
 * @{
 */
typedef enum
{
  PCAP_SEL_PCAP0O = (uint32_t)0x00 <<(PCAP_CON_CAP0SEL_Pos),  /*!< CAP0 Input signal Select PCMP0 port   */
	PCAP_SEL_PCAP1O = (uint32_t)0x00 <<(PCAP_CON_CAP1SEL_Pos),  /*!< CAP1 Input signal Select PCMP1 port   */
	PCAP_SEL_PCAP2O = (uint32_t)0x00 <<(PCAP_CON_CAP2SEL_Pos),  /*!< CAP2 Input signal Select PCMP2 port    */
  PCAP_SEL_CMP0O = (uint32_t) 0x01 <<(PCAP_CON_CAP0SEL_Pos),  /*!< CAP0 Input signal Select CMP0O port   */
	PCAP_SEL_CMP1O = (uint32_t) 0x01 <<(PCAP_CON_CAP1SEL_Pos),  /*!< CAP1 Input signal Select CMP1O port   */
	PCAP_SEL_CMP2O = (uint32_t) 0x01 <<(PCAP_CON_CAP2SEL_Pos),  /*!< CAP2 Input signal Select CMP2O port    */
} PCAP_SEL_TypeDef;

#define IS_PCAP_SEL(SEL) (((SEL) == PCAP_SEL_PCAP0O) || \
													((SEL) == PCAP_SEL_PCAP1O) || \
													((SEL) == PCAP_SEL_PCAP2O) || \
                          ((SEL) == PCAP_SEL_CMP0O)  || \
                          ((SEL) == PCAP_SEL_CMP1O)  || \
                          ((SEL) == PCAP_SEL_CMP2O))
/**
 * @}
 */

/** @defgroup PCAP_FILPrescaler PCAP FIL Prescaler
 * @{
 */
typedef enum
{
  PCAP_FILPrescaler_DIV1	   = (uint32_t)(0x00U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/1    */
  PCAP_FILPrescaler_DIV16	   = (uint32_t)(0x01U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/2    */
  PCAP_FILPrescaler_DIV32	   = (uint32_t)(0x02U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/4    */
  PCAP_FILPrescaler_DIV64	   = (uint32_t)(0x03U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/8    */
  PCAP_FILPrescaler_DIV128	 = (uint32_t)(0x04U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/16    */
  PCAP_FILPrescaler_DIV256	 = (uint32_t)(0x05U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/32    */
  PCAP_FILPrescaler_DIV512	 = (uint32_t)(0x06U << PCAP_CON_FILCK_Pos),	/*!< Filter Clock division: Fsource/64    */
  PCAP_FILPrescaler_DIV1024  = (uint32_t)(0x07U << PCAP_CON_FILCK_Pos), /*!< Filter Clock division: Fsource/128    */
} PCAP_FILPrescaler_TypeDef;

#define IS_PCAP_FILPRESCALER(PRESCALER) (((PRESCALER) == PCAP_FILPrescaler_DIV1) ||   \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV16) ||   \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV32) ||   \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV64) ||   \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV128) ||  \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV256) ||  \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV512) ||  \
													    					 ((PRESCALER) == PCAP_FILPrescaler_DIV1024))
/**
 * @}
 */

/** @defgroup PCAP_Mode PCAP Mode
 * @{
 */
typedef enum
{
  PCAP_FIL_75_Dsiable  = (uint32_t)(0X00U << PCAP_CON_FIL75_Pos),   /*!< PCAP FIL_75 Mode:Disable    */
  PCAP_FIL_75_Enable   = (uint32_t)(0X01U << PCAP_CON_FIL75_Pos),   /*!< PCAP FIL_75 Mode:Enable    */
} PCAP_FIL_75_Typedef;

#define IS_PCAP_PCRM(PCRM) (((PCRM) ==PCAP_FIL_75_Dsiable) ||  \
                           ((PCRM) == PCAP_FIL_75_Enable))
/**
 * @}
 */

/** @defgroup PCAP_IT PCAP Interrupt
 * @{
 */
typedef enum
{
 
  PCAP_IT_TIE       = (uint8_t)PCAP_IDE_TIE,       /*!< PCAP Interrupt: Timer overflow Interrupt */
  PCAP_IT_RCAP      = (uint8_t)PCAP_IDE_RCAPIE,    /*!< PCAP Interrupt: External rising edge input Interrupt */
  PCAP_IT_FCAP      = (uint8_t)PCAP_IDE_FCAPIE,    /*!< PCAP Interrupt: External faling edge input Interrupt */
  PCAP_IT_PHASEE    = (uint8_t)PCAP_IDE_PHASEEIE,  /*!< PCAP Interrupt: PHASEE Interrupt */
  PCAP_IT_TIDE       = (uint8_t)PCAP_IDE_TIDE,     /*!< PCAP Interrupt: TIDE Interrupt */
  PCAP_IT_CAPDE      = (uint8_t)PCAP_IDE_CAPDE,   /*!< PCAP Interrupt: CAPDE Interrupt */
  PCAP_IT_INT        = (uint8_t)PCAP_IDE_INTEN,     /*!< PCAP Interrupt: PCAP Interrupt */
} PCAP_IT_TypeDef;

#define IS_PCAP_IT(IT) ((IT) ==PCAP_IT_TIE ||\
                        (IT) ==PCAP_IT_RCAP ||\
                        (IT) ==PCAP_IT_FCAP ||\
                        (IT) ==PCAP_IT_PHASEE ||\
                        (IT) ==PCAP_IT_TIDE ||\
                        (IT) ==PCAP_IT_CAPDE ||\
                        (IT) ==PCAP_IT_INT)
/**
 * @}
 */
typedef enum
{
 
  PCAP_Mode_RisingCapture      = (uint8_t)0x01<<PCAP_CON_RCAPEN_Pos,       /*!< PCAP Mode: Rising Capture  */
  PCAP_Mode_FallinCapture      = (uint8_t)0x01<<PCAP_CON_FCAPEN_Pos,    /*!< PCAP Mode: Falling Capture */
  PCAP_Mode_PhaseJudgment      = (uint8_t)0x01<<PCAP_CON_PHASEMODE_Pos,    /*!< PCAP Mode:Phase Judgment */
} PCAP_Mode_TypeDef;

#define IS_PCAP_Mode(Mode) ((Mode) ==PCAP_Mode_RisingCapture ||\
                        (Mode) ==PCAP_Mode_FallinCapture ||\
                        (Mode) ==PCAP_Mode_PhaseJudgment )
/** @defgroup PCAP_FLAG PCAP Flag
 * @{
 */
typedef enum
{
  PCAP_FLAG_TIF    = (uint8_t)PCAP_STS_TIF,     /*!< PCAP Flag: TIF flag */
	PCAP_FLAG_CAP    = (uint8_t)PCAP_STS_CAPIF,     /*!< PCAP Flag: Capture flag */
  PCAP_FLAG_RCAP   = (uint8_t)PCAP_STS_RCAPIF,    /*!< PCAP Flag: Rising Capture  flag */
  PCAP_FLAG_FCAP   = (uint8_t)PCAP_STS_FCAPIF,    /*!< PCAP Flag: Falling Capture flag */
  PCAP_FLAG_PHASEE = (uint8_t)PCAP_STS_PHASEEIF, /*!< PCAP Flag: PHASEE flag */
} PCAP_FLAG_TypeDef;

#define IS_PCAP_FLAG(FLAG) ((((FLAG) & (uint8_t)0xE4) == 0x00) && ((FLAG) != 0x00))

#define IS_GET_PCAP_FLAG(FLAG) (((FLAG) == PCAP_FLAG_TIF) || \
                               ((FLAG) == PCAP_FLAG_RCAP) || \
                               ((FLAG) == PCAP_FLAG_FCAP) || \
                               ((FLAG) == PCAP_FLAG_CAP)  ||\
                                ((FLAG) == PCAP_FLAG_PHASEE)  )
/**
 * @}
 */

/** @defgroup PCAP_FLAG PCAP Flag
 * @{
 */
typedef enum
{
  PCAP_Direction_Firection  = (uint32_t)0x00,    /*!<PCAP Direction select Firection */
	PCAP_Direction_Reversal   = (uint32_t)0x01,    /*!< PCAP Direction select Reversal */
} PCAP_Direction_TypeDef;

#define IS_GET_PCAP_DIR(DIR) (((DIR) == PCAP_Direction_Firection) || \
                              ((DIR) == PCAP_Direction_Reversal))
/**
 * @}
 */

/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/

/** @defgroup PCAP_Constants PCAP Constants
  * @{
  */

#define IS_PCAP_ALL_PERIPH(PERIPH) (((PERIPH) == PCAP))

/**
 * @}
 */
/* End of constants -----------------------------------------------------*/

/** @defgroup PCAP Time base Configuration Structure definition
 * @{
 */
typedef struct
{
  uint32_t PCAP_Prescaler;     /*!<   This member configures PCAP Prescaler.
																				This parameter can be a value of @ref PCAP_PRESCALER_TypeDef */

  uint32_t PCAP_PhaseErrorDet; /*!<  This member configures PCAP PhaseErrorDet.
											                   When PCAP_Mode is mode 0 This parameter can be a value of @ref PCAP_PhaseErrorDet_TypeDef. */

  uint32_t PCAP_RCAPEN; /*!<  This member configures PCAP RCAPEN.
											                   When PCAP_Mode is mode 0 This parameter can be a value of @ref PCAP_RisingCap_TypeDef. */

  uint32_t PCAP_FCAPEN; /*!<  This member configures PCAP FCAPEN.
																						This parameter can be a value of @ref PCAP_FailingCap_TypeDef */

  uint32_t PCAP_INV; /*!<  This member configures PCAP INV.
																						This parameter can be a value of @ref PCAP_INV_TypeDef */

  uint32_t PCAP_SEL; /*!<  This member configures  PCAP SEL.
																						This parameter can be a value of @ref PCAP_SEL_TypeDef */

  uint32_t PCAP_FIL_CK; /*!<  This member configures PCAP FILCK.
																						This parameter can be a value of @ref PCAP_FILPrescaler_TypeDef */

  uint32_t PCAP_FIL_LEN; /*!<  This member configures PCAP FILLEN.
																			 */

  uint32_t PCAP_FIL_75; /*!<  This member configures PCAP FIL75.
																						This parameter can be a value of @ref PCAP_FIL_75_Typedef */
																						
	uint32_t PCAP_Threshold; /*!<  This member configures PCAP Threshold.*/
																					
} PCAP_InitTypeDef;
/**
 * @}
 */

/**
 * @}
 */
/* End of exported enumerations -----------------------------------------------------*/

/** @addtogroup PCAP_Functions PCAP Functions
 * @{
 */

/* PCAP Base functions ********************************************************/
void PCAP_DeInit(PCAP_TypeDef* PCAPx);
void PCAP_Init(PCAP_TypeDef* PCAPx, PCAP_InitTypeDef* PCAP_InitStruct);
void PCAP_Cmd(PCAP_TypeDef* PCAPx, FunctionalState NewState);
void PCAP_RXCmd(PCAP_TypeDef* PCAPx, FunctionalState NewState);
void PCAP_StructInit(PCAP_InitTypeDef* PCAP_InitStruct);
void PCAP_FIL75Cmd(PCAP_TypeDef* PCAPx, FunctionalState NewState);
void PCAP_SetCounthreshold(PCAP_TypeDef* PCAPx,uint32_t Counterthreshold);
/* Data transfers functions ********************************************************/
uint32_t PCAP_GetCnt(PCAP_TypeDef* PCAPx);
uint8_t PCAP_GetCapLevel(PCAP_TypeDef* PCAPx);
uint8_t PCAP_GetLastLevel(PCAP_TypeDef* PCAPx);
uint8_t PCAP_GetRawData(PCAP_TypeDef* PCAPx);
PCAP_Direction_TypeDef PCAP_GetDirection(PCAP_TypeDef* PCAPx);
uint32_t PCAP_GetCaptureValue(PCAP_TypeDef* PCAPx);

/* Interrupts, DMA and flags management functions  **********************************************/
void PCAP_ModeEnable(PCAP_TypeDef* PCAPx, uint16_t PCAP_Mode, FunctionalState NewState);
void PCAP_ITConfig(PCAP_TypeDef* PCAPx, uint16_t PCAP_IT, FunctionalState NewState);
FlagStatus PCAP_GetFlagStatus(PCAP_TypeDef* PCAPx, uint16_t PCAP_FLAG);
void PCAP_ClearFlag(PCAP_TypeDef* PCAPx, uint16_t PCAP_FLAG);



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
