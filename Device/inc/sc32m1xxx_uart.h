/**
 ******************************************************************************
 * @file    sc32m1xxx_uart.h
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   Header file of UART module.
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
#ifndef __sc32m1xxx_UART_H
#define __sc32m1xxx_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"
#include "stdio.h"
/** @addtogroup sc32m1xxx_StdPeriph_Driver
 * @{
 */

/** @addtogroup UART
 * @{
 */

/* Exported enumerations ------------------------------------------------------------*/
/** @defgroup UART_Exported_Enumerations UART Exported Enumerations
 * @{
 */

/** @defgroup UART_Prescaler UART Prescaler
 * @{
 */
typedef enum
{
  UART_PRESCALER_12 = 0x00U << UART_CON_PERSCALER_Pos, /*!< Clock division: Fapb/12    */
  UART_PRESCALER_4 = 0x01U << UART_CON_PERSCALER_Pos,  /*!< Clock division: Fapb/4   */
} UART_Prescaler_TypeDef;

#define IS_UART_PRESCALER(PRESCALER) (((PRESCALER) == UART_PRESCALER_12) || \
                                      ((PRESCALER) == UART_PRESCALER_4))
/**
 * @}
 */

/** @defgroup UART_Mode UART Mode
 * @{
 */
typedef enum
{
  UART_Mode_8B = (uint32_t)(0X00U << UART_CON_SM01_Pos),   /*!< UART Mode:8-bit half duplex    */
  UART_Mode_10B = (uint32_t)(0X01U << UART_CON_SM01_Pos), /*!< UART Mode:10-bit full duplex    */
  UART_Mode_11B = (uint32_t)(0X03U << UART_CON_SM01_Pos), /*!< UART Mode:11-bit full duplex    */
} UART_Mode_Typedef;

#define IS_UART_Mode(MODE) (((MODE) == UART_Mode_8B) ||  \
                            ((MODE) == UART_Mode_10B) || \
                            ((MODE) == UART_Mode_11B))
/**
 * @}
 */

/** @defgroup UART_PinRemap TIM Pin Remap
 * @{
 */
typedef enum
{
  UART_PinRemap_Default = (uint32_t)0x00 << UART_CON_SPOS_Pos,  /*!< TIM Pin Remap: Disable */
  UART_PinRemap_A       = (uint32_t)(0x01 << TWI_CON_SPOS_Pos),  /*!< TIM Pin Remap: Remap mode A */
} UART_PinRemap_TypeDef;

#define IS_UART_PINREMAP(PINREMAP) (((PINREMAP) == UART_PinRemap_Default) ||  \
																    ((PINREMAP) == UART_PinRemap_A))
/**
 * @}
 */

/** @defgroup UART_IT UART Interrupt
 * @{
 */
typedef enum
{
  UART_IT_EN = (uint8_t)UART_IDE_INTEN, /*!< UART Interrupt: UART Interrupt */
  UART_IT_TX = (uint8_t)UART_IDE_TXIE,  /*!< UART Interrupt: Transmit Interrupt */
  UART_IT_RX = (uint8_t)UART_IDE_RXIE,  /*!< UART Interrupt: Receive Interrupt */
	UART_IT_WK = (uint8_t)UART_IDE_WKIE, /*!< UART Interrupt: Wake up Interrupt */
	UART_IT_BK = (uint16_t)UART_IDE_BKIE, /*!< UART Interrupt: Break Interrupt */
	UART_IT_SL = (uint16_t)UART_IDE_SLVHEIE,  /*!< UART Interrupt: SLVHEIE Interrupt */
	UART_IT_SY = (uint16_t)UART_IDE_SYNCIE, /*!< UART Interrupt: SYNCIE Interrupt */
} UART_IT_TypeDef;

#define IS_UART_IT(IT) ((((IT) & (uint8_t)0xE0) == 0x00) && ((IT) != 0x00))
/**
 * @}
 */

/** @defgroup UART_FLAG UART Flag
 * @{
 */
typedef enum
{
  UART_Flag_RX = (uint8_t)UART_STS_RXIF, /*!< UART Flag: Receive flag */
  UART_Flag_TX = (uint8_t)UART_STS_TXIF, /*!< UART Flag: Transmit flag */
  UART_Flag_BK = (uint16_t)UART_STS_BKIF,/*!< UART Flag: Break flag */
	UART_Flag_SY = (uint16_t)UART_STS_SYNCIF,/*!< UART Flag: SYNCIE flag */
  UART_Flag_SLVYN  = (uint16_t)UART_STS_SLVSYNIF,/*!< UART Flag: Break flag */
	UART_Flag_SLVHE  =  (uint16_t)UART_STS_SLVHEIF,/*!< UART Flag: SYNCIE flag */
} UART_FLAG_TypeDef;

#define IS_UART_FLAG(FLAG) ((((FLAG) & (uint8_t)0xE4) == 0x00) && ((FLAG) != 0x00))

#define IS_GET_UART_FLAG(FLAG) (((FLAG) == UART_Flag_RX) || \
                                ((FLAG) == UART_Flag_TX) || \
                                ((FLAG) == UART_Flag_BK) || \
                                ((FLAG) == UART_Flag_SLVYN) || \
                                ((FLAG) == UART_Flag_SLVHE) || \
                                ((FLAG) == UART_Flag_WK))
/**
 * @}
 */

/** @defgroup UART_DMAReq UART DMA Request
 * @{
 */
typedef enum
{
  UART_DMAReq_RX = (uint8_t)UART_IDE_RXDMAEN, /*!< UART DMA Request: Receive */
  UART_DMAReq_TX = (uint8_t)UART_IDE_TXDMAEN, /*!< UART DMA Request: Transmit */
} UART_DMAReq_TypeDef;

#define IS_UART_DMAREQ(DMAREQ) ((((DMAREQ) & (uint8_t)0x3F) == 0x00) && ((DMAREQ) != 0x00))
/**
 * @}
 */
typedef enum
{
  UART_MASTER = (uint32_t)(0x00 << UART_CON_SLVEN_Pos|(0x01 << UART_CON_FUNCSEL_Pos)),  /*!< UART_LINMODE: UART_MASTER */
  UART_SLAVER = (uint32_t)(0x01 << UART_CON_SLVEN_Pos|(0x01 << UART_CON_FUNCSEL_Pos)),  /*!< UART_LINMODE: UART_SLAVER*/
} UART_LINMODE_TypeDef;

#define IS_UART_LINMODE(LINMODE) (((LINMODE) == UART_MASTER) ||  \
																    ((LINMODE) == UART_SLAVER))
/**
 * @}
 */
/** @brief UART_BKSIZE UART BKSIZE
 * @{
 */
typedef enum
{
  UART_BKSIZE_10 = (uint32_t)0x00 << UART_CON_BKSIZE_Pos,  /*!< UART_BKSIZE: UART_BKSIZE_10 */
  UART_BKSIZE_13 = (uint32_t)(0x01 << UART_CON_BKSIZE_Pos),  /*!< UART_BKSIZE:UART_BKSIZE_13 */
} UART_BKSIZE_TypeDef;

#define IS_UART_BKSIZE(BKSIZE) (((BKSIZE) == UART_BKSIZE_10) ||  \
																    ((BKSIZE) == UART_BKSIZE_13))

/** @brief UART_LBDL UART LBDL
 * @{
 */
typedef enum
{
  UART_LBDL_10 = (uint32_t)0x00 << UART_CON_LBDL_Pos,  /*!< UART_LBDL: UART_LBDL_10 */
  UART_LBDL_11 = (uint32_t)(0x01 << UART_CON_LBDL_Pos),  /*!< UART_LBDL:UART_LBDL_11 */
} UART_LBDL_TypeDef;

#define IS_UART_LBDL(LBDL) (((LBDL) == UART_LBDL_10) ||  \
																    ((LBDL) == UART_LBDL_11))
/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/

/** @defgroup UART_Constants UART Constants
  * @{
  */

#define IS_UART_ALL_PERIPH(PERIPH) (((PERIPH) == UART0) || \
                                    ((PERIPH) == UART1) || \
                                    ((PERIPH) == UART2))




/**
 * @}
 */
/* End of constants -----------------------------------------------------*/

/** @defgroup UART Time base Configuration Structure definition
 * @{
 */
typedef struct
{
  uint32_t UART_ClockFrequency; /*!<  Specifies the clock division.
																				This parameter can be a value of @ref UART_Prescaler_TypeDef */

  uint32_t UART_BaudRate; /*!< This member configures the UART communication baud rate.
											   */

  uint32_t UART_Mode; /*!< Example Set the uart mode.
																						This parameter can be a value of @ref UART_Mode_Typedef */
} UART_InitTypeDef;
/**
 * @}
 */
#define BIT(A,B)       ((A >> B) & 0x01)
#define AvoidSemiHostEable 1
/**
 * @}
 */
/* End of exported enumerations -----------------------------------------------------*/

/** @addtogroup UART_Functions UART Functions
 * @{
 */

/* UART Base functions ********************************************************/
void UART_DeInit(UART_TypeDef* UARTx);
void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct);
void UART_TXCmd(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_RXCmd(UART_TypeDef* UARTx, FunctionalState NewState);

/* Data transfers functions ********************************************************/
void UART_SendData(UART_TypeDef* UARTx, uint16_t Data);
uint16_t UART_ReceiveData(UART_TypeDef* UARTx);

/* Pin remap management functions  **********************************************/
void UART_PinRemapConfig(UART_TypeDef* UARTx, UART_PinRemap_TypeDef UART_Remap);

/* Interrupts, DMA and flags management functions  **********************************************/
void UART_ITConfig(UART_TypeDef* UARTx, uint16_t UART_IT, FunctionalState NewState);
FlagStatus UART_GetFlagStatus(UART_TypeDef* UARTx, UART_FLAG_TypeDef UART_FLAG);
void UART_ClearFlag(UART_TypeDef* UARTx, uint16_t UART_FLAG);
void UART_DMACmd(UART_TypeDef* UARTx, uint16_t UART_DMAReq, FunctionalState NewState);
void UART_LIN_MODE(UART_TypeDef* UARTx, UART_LINMODE_TypeDef UART_LINMODE);
void UART_LIN_BKSIZE(UART_TypeDef* UARTx, UART_BKSIZE_TypeDef BKSIZE);
void UART_SendBreak(void);
void UART_LIN_SLVARENE(UART_TypeDef* UARTx, FunctionalState NewState);
void UART_LIN_LBDL(UART_TypeDef* UARTx, UART_LBDL_TypeDef LBDL);
uint8_t LIN_CalID(uint8_t id);
uint8_t LINCalChecksum(uint8_t id, uint8_t *data, uint8_t len);
void Printf_UartInit(UART_TypeDef* UARTx);
int fputc(int c, FILE* f);
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
