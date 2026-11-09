/**
 ******************************************************************************
 * @file    sc32m1xxx_PCAP.c
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief   PCAP function module
 *
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

/* Includes ------------------------------------------------------------------*/
#include "sc32m1xxx_pcap.h"



/** @defgroup PCAP_Exported_Functions_Group1 Configuration of the PCAP computation unit functions
 *  @brief   Configuration of the PCAP computation unit functions
 *
@verbatim
 ===============================================================================
                     ##### PCAP configuration functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
 * @brief  DeInitializes the PCAP peripheral
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval None
 */
void PCAP_DeInit(PCAP_TypeDef* PCAPx)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

  PCAPx->PCAP_CON = (uint32_t)0x00000000U;
  PCAPx->PCAP_CNT = (uint32_t)0x00000000U;
  PCAPx->PCAP_TH = (uint32_t)0x00000000U;
  PCAPx->PCAP_RESULT = (uint32_t)0x00000001U;
	PCAPx->PCAP_STS = (uint32_t)0x0000001FU;
  PCAPx->PCAP_IDE = (uint32_t)0x00000000U;
}

/**
 * @brief  Initializes the PCAP peripheral
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  PCAP_InitStruct[out]: Pointer to structure PCAP_InitTypeDef, to be initialized.
 * @retval None
 */
void PCAP_Init(PCAP_TypeDef* PCAPx, PCAP_InitTypeDef* PCAP_InitStruct)
{
    uint32_t tmpreg;

    /* Check the parameters */
    assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

    tmpreg = PCAPx->PCAP_CON;
    tmpreg &= ~(PCAP_CON_PCAPCK | PCAP_CON_PHASEMODE | PCAP_CON_RCAPEN
              | PCAP_CON_FCAPEN | PCAP_CON_INV0 | PCAP_CON_INV1 | PCAP_CON_INV2
              | PCAP_CON_CAP0SEL | PCAP_CON_CAP1SEL | PCAP_CON_CAP2SEL
               | PCAP_CON_FILCK | PCAP_CON_FILLEN | PCAP_CON_FIL75);

    tmpreg |= (PCAP_InitStruct->PCAP_Prescaler | PCAP_InitStruct->PCAP_PhaseErrorDet | PCAP_InitStruct->PCAP_RCAPEN |
             PCAP_InitStruct->PCAP_FCAPEN | PCAP_InitStruct->PCAP_INV | PCAP_InitStruct->PCAP_SEL |
             PCAP_InitStruct->PCAP_FIL_CK | PCAP_InitStruct->PCAP_FIL_75 );

    tmpreg |= (PCAP_InitStruct->PCAP_FIL_LEN  << PCAP_CON_FILLEN_Pos);

    PCAPx->PCAP_CON = tmpreg;
	
	PCAPx->PCAP_TH = PCAP_InitStruct->PCAP_Threshold;
}

/**
  * @brief  Fills each PCAP_InitStruct member with its default value.
  * @param  PCAP_InitStruct[out]: Pointer to structure PCAP_InitTypeDef, to be initialized.
  * @retval None
  */
void PCAP_StructInit(PCAP_InitTypeDef* PCAP_InitStruct)
{
  /* Set the default configuration */
	PCAP_InitStruct->PCAP_Prescaler     = PCAP_PRESCALER_DIV1;
	PCAP_InitStruct->PCAP_PhaseErrorDet = PCAP_PhaseErrorDet_Enable;
	PCAP_InitStruct->PCAP_RCAPEN        = PCAP_RisingCap_Disable;
	PCAP_InitStruct->PCAP_FCAPEN        = PCAP_FailingCap_Disable;
	PCAP_InitStruct->PCAP_INV           = PCAP_INV_None;
	PCAP_InitStruct->PCAP_SEL           = PCAP_SEL_PCAP0O;
	PCAP_InitStruct->PCAP_FIL_CK        = PCAP_FILPrescaler_DIV1024;
	PCAP_InitStruct->PCAP_FIL_LEN       = 0;
	PCAP_InitStruct->PCAP_FIL_75        = PCAP_FIL_75_Dsiable ;
}

/**
 * @brief  Enables or disables the specified PCAP peripheral.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  NewState: new state of the PCAPx peripheral.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable 
 * @retval None
 */
void PCAP_Cmd(PCAP_TypeDef* PCAPx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the PCAP Counter */
    PCAPx->PCAP_CON |= PCAP_CON_PCAPEN;
  }
  else
  {
    /* Disable the PCAP Counter */
    PCAPx->PCAP_CON &= (uint32_t)~PCAP_CON_PCAPEN;
  }
}

/**
 * @brief  Enables or disables the specified PCAP peripheral.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  NewState[in]: new state of the PCAPx FIL75.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable 
 * @retval None
 */
void PCAP_FIL75Cmd(PCAP_TypeDef* PCAPx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the PCAP Counter */
    PCAPx->PCAP_CON |= PCAP_CON_FIL75;
  }
  else
  {
    /* Disable the PCAP Counter */
    PCAPx->PCAP_CON &= (uint32_t)~PCAP_CON_FIL75;
  }
}

/**
 * @brief  PCAP Counter threshold Setting.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  Counterthreshold[in]: PCAP Counter threshold.
 * @retval None
 */
void PCAP_SetCounthreshold(PCAP_TypeDef* PCAPx,uint32_t Counterthreshold)
{
	assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
	
	 PCAPx->PCAP_TH &= ~PCAP_TH_TH_Msk;
	 PCAPx->PCAP_TH |= Counterthreshold;
	
}
/**
 * @brief  Gets the PCAP Count value.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval PCAP CNT 
 */
uint32_t PCAP_GetCnt(PCAP_TypeDef* PCAPx)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

  return PCAPx->PCAP_CNT;
}

/**
 * @brief  Gets the PCAP Count value.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval The signal level state read after capturing the edge
 */
uint8_t PCAP_GetCapLevel(PCAP_TypeDef* PCAPx)
{
	uint8_t TmpLevel;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

	TmpLevel = ((PCAPx->PCAP_RESULT & PCAP_RESULT_CAPLEVEL) >>  PCAP_RESULT_CAPLEVEL_Pos);
  return TmpLevel;
}

/**
 * @brief  Gets the PCAP Last Level.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval Capture the signal level state read before the edge
 */
uint8_t PCAP_GetLastLevel(PCAP_TypeDef* PCAPx)
{
	uint8_t TmpLevel;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

	TmpLevel = ((PCAPx->PCAP_RESULT & PCAP_RESULT_LASTLEVEL) >>  PCAP_RESULT_LASTLEVEL_Pos);
  return TmpLevel;
}

/**
 * @brief  Gets the PCAP RawData.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval Signal level state read after capture edge, unfiltered result
 */
uint8_t PCAP_GetRawData(PCAP_TypeDef* PCAPx)
{
	uint8_t TmpLevel;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

	TmpLevel = ((PCAPx->PCAP_STS & PCAP_STS_RAWDATA) >>  PCAP_STS_RAWDATA_Pos);
  return TmpLevel;
}

/**
 * @brief  Gets the PCAP Direction status.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval PCAP Direction status
 *               - PCAP_Direction_Firection:PCAP Direction select Firection 
 *               - PCAP_Direction_Reversal:PCAP Direction select Reversal 
 */
PCAP_Direction_TypeDef PCAP_GetDirection(PCAP_TypeDef* PCAPx)
{
	PCAP_Direction_TypeDef TmpDirection;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

	if((PCAPx->PCAP_RESULT & PCAP_RESULT_DIR) != RESET)
	{
		TmpDirection = PCAP_Direction_Reversal;
	}
	else
	{
		TmpDirection = PCAP_Direction_Firection;
	}
	
  return TmpDirection;
}

/**
 * @brief  Gets the PCAP Capture value.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @retval PCAP Capture type
 */
uint32_t PCAP_GetCaptureValue(PCAP_TypeDef* PCAPx)
{
	uint32_t TmpValue;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

	TmpValue = (uint32_t)(PCAPx->PCAP_RESULT & PCAP_RESULT_CAPCNT);
	
  return TmpValue;
}



/**
 * @}
 */
/* End of PCAP_Group1.	*/


/** @defgroup PCAP_Group3 Interrupts, DMA and flags management functions
 *  @brief    Interrupts, DMA and flags management functions
 *
@verbatim
 ===============================================================================
                     ##### Interrupts, DMA and flags management functions #####
 ===============================================================================
@endverbatim
  * @{
  */
/**
 * @brief  Enables or disables the specified PCAP interrupts.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  PCAP_IT[in]: specifies the PCAP interrupts sources to be enabled or disabled.
 *               - PCAP_IT_TIE: Timer overflow Interrupt 
 *               - PCAP_IT_RCAP: External rising edge input Interrupt 
 *               - PCAP_IT_FCAP: External faling edge input Interrupt 
 *               - PCAP_IT_PHASEE: PHASEE Interrupt 
 *               - PCAP_IT_TIDE: TIDE Interrupt 
 *               - PCAP_IT_CAPDE: CAPDE Interrupt 
 *               - PCAP_IT_INT: PCAP Interrupt 
 * @param  NewState[in]: new state of the PCAP interrupts.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable 
 * @retval None
 */
void PCAP_ITConfig(PCAP_TypeDef* PCAPx, uint16_t PCAP_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
  assert_param(IS_PCAP_IT(PCAP_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    PCAPx->PCAP_IDE |= PCAP_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    PCAPx->PCAP_IDE &= (uint32_t)~PCAP_IT;
  }
}
/**
 * @brief  Enables or disables the specified PCAP interrupts.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  PCAP_Mode[in]: specifies the PCAP interrupts sources to be enabled or disabled.
 *               - PCAP_Mode_RisingCapture: Rising Capture  
 *               - PCAP_Mode_FallinCapture: Falling Capture 
 *               - PCAP_Mode_PhaseJudgment:Phase Judgment 
 * @param  NewState[in]: new state of the PCAP Mode.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable 
 * @retval None
 */
void PCAP_ModeEnable(PCAP_TypeDef* PCAPx, uint16_t PCAP_Mode, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
  assert_param(IS_PCAP_Mode(PCAP_Mode));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the PCAP Mode */
    PCAPx->PCAP_IDE |= PCAP_Mode;
  }
  else
  {
    /* Disable the PCAP Mode */
    PCAPx->PCAP_IDE &= (uint32_t)~PCAP_Mode;
  }
}
/**
 * @brief  Checks whether the specified PCAP flag is set or not.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  PCAP_FLAG[in]: specifies the flag to check.
 *               - PCAP_FLAG_TIF: TIF flag  
 *               - PCAP_FLAG_CAP: Capture flag  
 *               - PCAP_FLAG_RCAP: Rising Capture  flag 
 *               - PCAP_FLAG_FCAP: Falling Capture flag 
 *               - PCAP_FLAG_PHASEE: PHASEE flag 
 * @retval The new state of PCAP_FLAG (SET or RESET).
 *                  -  RESET:Flag reset
 *                  -  SET :Flag up
 */
FlagStatus PCAP_GetFlagStatus(PCAP_TypeDef* PCAPx, uint16_t PCAP_FLAG)
{
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));
  assert_param(IS_GET_PCAP_FLAG(PCAP_FLAG));

  if((PCAPx->PCAP_STS & PCAP_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
 * @brief  Clears the PCAPx's pending flags.
 * @param  PCAPx[out]: Only PCAP can be select the PCAPx peripheral.
 *               - PCAP: select the PCAP peripheral
 * @param  PCAP_FLAG[in]: specifies the flag bit to clear.
 *               - PCAP_FLAG_TIF: TIF flag  
 *               - PCAP_FLAG_CAP: Capture flag  
 *               - PCAP_FLAG_RCAP: Rising Capture  flag 
 *               - PCAP_FLAG_FCAP: Falling Capture flag 
 *               - PCAP_FLAG_PHASEE: PHASEE flag 
 * @retval None
 */
void PCAP_ClearFlag(PCAP_TypeDef* PCAPx, uint16_t PCAP_FLAG)
{
  /* Check the parameters */
  assert_param(IS_PCAP_ALL_PERIPH(PCAPx));

  /* Clear the flags */
	if(PCAP_FLAG == PCAP_FLAG_TIF)
	{
		 PCAPx->PCAP_STS &= (uint8_t)0xE1;
	}
  else if(PCAP_FLAG == PCAP_FLAG_CAP)
	{
		PCAPx->PCAP_STS &= (uint8_t)0xE2;
	}
  else if(PCAP_FLAG == PCAP_FLAG_RCAP)
	{
		PCAPx->PCAP_STS &= (uint8_t)0xE4;
	}
  else if(PCAP_FLAG == PCAP_FLAG_FCAP)
	{
		PCAPx->PCAP_STS &= (uint8_t)0xE8;
	}
  else if(PCAP_FLAG == PCAP_FLAG_FCAP)
	{
		PCAPx->PCAP_STS &= (uint8_t)0xF0;
	}
}


/**
 * @}
 */
/* End of PCAP_Group3.	*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT SOC Microelectronics *****END OF FILE****/

