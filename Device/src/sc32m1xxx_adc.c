/*
 ******************************************************************************
 * @file    sc32m1xxx_adc.c
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief  ADC function module
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
#include "sc32m1xxx_adc.h"


/** @defgroup ADC_Exported_Functions_Group1 Configuration of the ADC computation unit functions
 *  @brief   Configuration of the ADC computation unit functions
 *
@verbatim
 ===============================================================================
                     ##### ADC configuration functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
 * @brief  DeInitializes the ADC peripheral
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.               
 * @retval None
 */
void ADC_DeInit(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  if(ADCx == ADC)
  	{
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC, ENABLE);
	  RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC, DISABLE);
  	}
}

/**
 * @brief  Initializes the ADC peripheral
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_InitStruct[out]: Pointer to structure ADC_InitTypeDef, 
 *                              to be initialized.
 * @retval None
 */
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  uint32_t tmpreg;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CONVMODE(ADC_InitStruct->ADC_ConvMode));
  assert_param(IS_ADC_SAMPLEMODE(ADC_InitStruct->ADC_SPMODE));
  assert_param(IS_ADC_PRESCALER(ADC_InitStruct->ADC_Prescaler));
  assert_param(IS_ADC_EAIN(ADC_InitStruct->ADC_EAIN));
  assert_param(IS_ADC_VREF(ADC_InitStruct->ADC_VREF));
  
  tmpreg = ADCx->ADC_CON;
  tmpreg &= ~(ADC_CON_SPMODE | ADC_CON_CONT | ADC_CON_LOWSP | ADC_CON_REFSEL );
  tmpreg |= (ADC_InitStruct->ADC_SPMODE | ADC_InitStruct->ADC_Prescaler |
  	          ADC_InitStruct->ADC_VREF | ADC_InitStruct->ADC_ConvMode);
  ADCx->ADC_CON = tmpreg;
  
  ADCx->ADC_CFG = ADC_InitStruct->ADC_EAIN;
}

/**
  * @brief  Fills each ADC_InitStruct member with its default value.
  * @param  ADC_InitStruct[out]: Pointer to structure ADC_InitTypeDef, 
  *                              to be initialized.
  * @retval None
  */
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
    /* Set the default configuration */
    ADC_InitStruct->ADC_SPMODE    = ADC_SampleMode_Single;
    ADC_InitStruct->ADC_ConvMode  = ADC_ConvMode_Single;
    ADC_InitStruct->ADC_Prescaler = ADC_Prescaler_3CLOCK;
    ADC_InitStruct->ADC_VREF      = ADC_RefSource_VDD;
    ADC_InitStruct->ADC_EAIN      = ADC_EAIN_Less;
}

/**
 * @brief  Enables or disables the specified ADC peripheral.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  NewState[in]: new state of the ADCx peripheral.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable  
 * @retval None
 */
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
    /* Enable the ADC Counter */
    ADCx->ADC_CON |= ADC_CON_ADCEN;
    }
    else
    {
    /* Disable the ADC Counter */
    ADCx->ADC_CON &= (uint32_t)~ADC_CON_ADCEN;
    }
}



/**
 * @brief  Configure the ADC single/serial conversion mode selection.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_ConvMode[in]: new conversion mode of the ADCx peripheral.
 *                  - ADC_ConvMode_Single:Single conversion mode
 *                  - ADC_ConvMode_Sequence:Continuous conversion mode
 * @retval None
 */
void ADC_ConvModeConfig(ADC_TypeDef* ADCx, ADC_ConvMode_TypeDef ADC_ConvMode)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    if(ADC_ConvMode != ADC_ConvMode_Single)
    {
        /* Set the ADC conversion mode to Sequence */
        ADCx->ADC_CON |= ADC_CON_CONT;
    }
    else
    {
        /* Set the ADC conversion mode to Single */
        ADCx->ADC_CON &= (uint32_t)~ADC_CON_CONT;
    }
}

/**
 * @brief  Configure the ADC input channel
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_Channel[in]: ADC input channel selection.
 *                         - ADC_Channel_0 : ADC input channel 0 
 *                         - ADC_Channel_1 : ADC input channel 1 
 *                         - ADC_Channel_2 : ADC input channel 2 
 *                         - ADC_Channel_3 : ADC input channel 3  
 *                         - ADC_Channel_4 : ADC input channel 4  
 *                         - ADC_Channel_5 : ADC input channel 5  
 *                         - ADC_Channel_6 : ADC input channel 6  
 *                         - ADC_Channel_7 : ADC input channel 7  
 *                         - ADC_Channel_8 : ADC input channel 8 
 *                         - ADC_Channel_9 : ADC input channel 9  
 *                         - ADC_Channel_10 :ADC input channel 10  
 *                         - ADC_Channel_11 :ADC input channel 11 
 *                         - ADC_Channel_12 :ADC input channel 12  
 *                         - ADC_Channel_13 :ADC input channel 13 
 *                         - ADC_Channel_14 :ADC input channel 14
 *                         - ADC_Channel_15 :ADC input channel 15 
 *                         - ADC_Channel_VDD_D4:ADC_CHANNEL VDD Divide 4
 *                         - ADC_Channel_TEMP:ADC_Channel_TEMP 
 * @retval None
 */
void ADC_SetChannel(ADC_TypeDef* ADCx, ADC_ChannelTypedef ADC_Channel)
{

    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    ADCx->ADC_CON &=(uint32_t) ~(ADC_CON_ADCISA);
    ADCx->ADC_CON  |= ADC_Channel<<16;
}



/**
 * @brief  Gets the ADC input channel.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @retval ADC channel type: The selected ADC channel
 *                         - ADC_Channel_0 : ADC input channel 0 
 *                         - ADC_Channel_1 : ADC input channel 1 
 *                         - ADC_Channel_2 : ADC input channel 2 
 *                         - ADC_Channel_3 : ADC input channel 3  
 *                         - ADC_Channel_4 : ADC input channel 4  
 *                         - ADC_Channel_5 : ADC input channel 5  
 *                         - ADC_Channel_6 : ADC input channel 6  
 *                         - ADC_Channel_7 : ADC input channel 7  
 *                         - ADC_Channel_8 : ADC input channel 8 
 *                         - ADC_Channel_9 : ADC input channel 9  
 *                         - ADC_Channel_10 :ADC input channel 10  
 *                         - ADC_Channel_11 :ADC input channel 11 
 *                         - ADC_Channel_12 :ADC input channel 12  
 *                         - ADC_Channel_13 :ADC input channel 13 
 *                         - ADC_Channel_14 :ADC input channel 14
 *                         - ADC_Channel_15 :ADC input channel 15 
 *                         - ADC_Channel_VDD_D4:ADC_CHANNEL VDD Divide 4
 *                         - ADC_Channel_TEMP:ADC_CHANNEL_TEMP 
 */
ADC_ChannelTypedef ADC_GetChannel(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Get the ADC channel type */
  return (ADC_ChannelTypedef)(ADCx->ADC_CON  & ( ADC_CON_ADCISA));
}

/**
 * @}
 */
/* End of ADC_Group1.	*/

/** @defgroup ADC_Group2 Conversion management functions
 *  @brief   Conversion management functions
 *
@verbatim
 ===============================================================================
                     ##### ADC Conversion management functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
 * @brief  Enables or disables the software conversion enable function of the specified ADC.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @retval None
 */
void ADC_SoftwareStartConv(ADC_TypeDef* ADCx)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    /* Set the ADCS Bit */

    ADC->ADC_CON |= ADC_CON_ADCS;
}

/**
 * @brief  Returns the last ADCx conversion result data for regular channel.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @retval the result of the latest ADCx conversion
 */
uint16_t ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  /*Returns the result of the latest ADCx conversion */
  return (uint16_t)ADCx->ADC_VALUE;
}

/**
 * @}
 */
/* End of ADC_Group2.	*/

/** @defgroup ADC_Group3 Interrupts, DMA and flags management functions
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
 * @brief  Enables or disables the specified ADC interrupts.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_IT[in]: specifies the ADC interrupts sources to be enabled or disabled.
 *                  - ADC_IT_INTEN:ADC Interrupt: ADC  Interrupt 
 *                  - ADC_IT_EOCIE:ADC sequence 0 sampling + conversion complete interrupt Flag 
 *                  - ADC_IT_EOSIE0:ADC sequence 0 sampling + conversion complete interrupt Flag
 *                  - ADC_IT_EOSIE1:ADC sequence 1 sampling + conversion complete interrupt Flag 
 *                  - ADC_IT_EOSIE2:ADC sequence 2 sampling + conversion complete interrupt Flag 
 *                  - ADC_IT_EOSIE3:ADC sequence 3 sampling + conversion complete interrupt Flag 
 *                  - ADC_IT_UPTHIE:ADC upper threshold overflow Flag 
 *                  - ADC_IT_LOWTHIE:ADC lower threshold overflow Flag 
 * @param  NewState[in]: new state of the ADC interrupts.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable   
 * @retval None
 */
void ADC_ITConfig(ADC_TypeDef* ADCx, uint16_t ADC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_IT(ADC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    ADCx->ADC_IDE |= ADC_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    ADCx->ADC_IDE &= (uint32_t)~ADC_IT;
  }
}

/**
 * @brief  Checks whether the specified ADC flag is set or not.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_FLAG[in]:specifies the flag to check.
 *                  - ADC_Flag_ADCIF :ADC Flag 
 *                  - ADC_Flag_EOSIF0:ADC sequence 0 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF1:ADC sequence 1 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF2:ADC sequence 2 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF3:ADC sequence 3 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_BUSY:ADC busy Flag 
 *                  - ADC_Flag_UPTHIF:ADC upper threshold overflow Flag 
 *                  - ADC_Flag_LOWTHIF:ADC lower threshold overflow Flag
 * @retval The new state of ADC_FLAG (SET or RESET).
 *                  -  RESET:Flag reset
 *                  -  SET :Flag up
 */
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, uint16_t ADC_FLAG)
{
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_GET_ADC_FLAG(ADC_FLAG));

  if((ADCx->ADC_STS & ADC_FLAG) != (uint16_t)RESET)
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
 * @brief  Clears the ADCx's pending flags.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_FLAG[in]:specifies the flag bit to clear.
 *                  - ADC_Flag_ADCIF :ADC Flag 
 *                  - ADC_Flag_EOSIF0:ADC sequence 0 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF1:ADC sequence 1 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF2:ADC sequence 2 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_EOSIF3:ADC sequence 3 sampling + conversion complete interrupt Flag 
 *                  - ADC_Flag_BUSY:ADC busy Flag 
 *                  - ADC_Flag_UPTHIF:ADC upper threshold overflow Flag 
 *                  - ADC_Flag_LOWTHIF:ADC lower threshold overflow Flag
 * @retval None
 */
void ADC_ClearFlag(ADC_TypeDef* ADCx, uint16_t ADC_FLAG)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    /* Clear the flags */
    ADCx->ADC_STS = (uint16_t)ADC_FLAG;
}
 
/**
 * @brief  Enables or disables the ADC's DMA interface.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  NewState[in]: new state of the DMA Request sources.
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable   
 * @retval None
 */
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    if(NewState != DISABLE)
    {
    /* Enable the selected ADC DMA requests */
    ADCx->ADC_IDE |= (uint32_t)0x40;
    }
    else
    {
    /* Disable the selected ADC DMA requests */
    ADCx->ADC_IDE &= (uint32_t)~0x40;
    }
}
/**
 * @}
 */
/* End of ADC_Group3.	*/
/** @defgroup ADC_Group4 Common interface functions of motors
 *  @brief    multi-channel, continuous sampling and other functions 
 *
@verbatim
 ===============================================================================
   #####  multi-channel, continuous sampling and other functions  #####
 ===============================================================================
@endverbatim
  * @{
  */
/**
 * @brief  Configure the sample mode of the ADC.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_SampleMode[in]: new sample mode of the ADCx peripheral.
 *          -ADC_SampleMode_Single:Single sampling mode 
 *          -ADC_SampleMode_Double:Double sampling mode 
 * @retval None
 */
void ADC_SampleModeConfig(ADC_TypeDef* ADCx, ADC_SampleMode_TypeDef ADC_SampleMode)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    if(ADC_SampleMode != ADC_SampleMode_Single)
    {
        /* Set the ADC conversion mode to double */
        ADCx->ADC_CON |= ADC_CON_SPMODE;
    }
    else
    {
        /* Set the ADC conversion mode to single */
        ADCx->ADC_CON &= (uint32_t)~ADC_CON_SPMODE;
    }
}


/**
 * @brief Configure ADC dual sampling channels.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_ChannelA[in]: Group A sampling channel selection.
 *                         - ADC_ChannelA_0 : ADC input channel 0 
 *                         - ADC_ChannelA_2 : ADC input channel 2 
 *                         - ADC_ChannelA_4 : ADC input channel 4  
 *                         - ADC_ChannelA_6 : ADC input channel 6  
 *                         - ADC_ChannelA_8 : ADC input channel 8 
 *                         - ADC_ChannelA_10 :ADC input channel 10  
 *                         - ADC_ChannelA_12 :ADC input channel 12  
 *                         - ADC_ChannelA_14 :ADC input channel 14
 *                         - ADC_ChannelA_VDD_D4:ADC_CHANNEL VDD Divide 4
 * @param  ADC_ChannelB[in]: Group B sampling channel selection.
 *                         - ADC_ChannelB_1 : ADC input channel 1 
 *                         - ADC_ChannelB_3 : ADC input channel 3  
 *                         - ADC_ChannelB_5 : ADC input channel 5  
 *                         - ADC_ChannelB_7 : ADC input channel 7  
 *                         - ADC_ChannelB_9 : ADC input channel 9  
 *                         - ADC_ChannelB_11 :ADC input channel 11 
 *                         - ADC_ChannelB_13 :ADC input channel 13 
 *                         - ADC_ChannelB_15 :ADC input channel 15 
 *                         - ADC_ChannelB_TEMP:ADC_CHANNEL_TEMP
 * @retval None
 */
void ADC_SetDoubleSample(ADC_TypeDef* ADCx, ADC_ChannelATypedef ADC_ChannelA,ADC_ChannelBTypedef ADC_ChannelB)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    ADCx->ADC_CON &= (uint32_t)~(ADC_CON_ADCISA|ADC_CON_ADCISB);
    ADCx->ADC_CON |= ADC_ChannelA<<16;
    ADCx->ADC_CON |= ADC_ChannelB<<24;
}

/**
 * @brief  Configure ADC sampling sequence signal selection.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_SQRank[in]:  ADC sample sequence rank.
 *         -ADC_SQRank_0:ADC sequence Channel 0 
 *         -ADC_SQRank_1:ADC sequence Channel 1 
 *         -ADC_SQRank_2:ADC sequence Channel 2
 *         -ADC_SQRank_3:ADC sequence Channel 3
 *         -ADC_SQRank_4:ADC sequence Channel 4 
 *         -ADC_SQRank_5:ADC sequence Channel 5 
 *         -ADC_SQRank_6:ADC sequence Channel 6
 *         -ADC_SQRank_7:ADC sequence Channel 7
 *         -ADC_SQRank_8:ADC sequence Channel 8 
 *         -ADC_SQRank_9:ADC sequence Channel 9 
 *         -ADC_SQRank_10:ADC sequence Channel 10 
 *         -ADC_SQRank_11:ADC sequence Channel 11
 *         -ADC_SQRank_12:ADC sequence Channel 12 
 *         -ADC_SQRank_13:ADC sequence Channel 13 
 *         -ADC_SQRank_14:ADC sequence Channel 14
 *         -ADC_SQRank_15:ADC sequence Channel 15 
 * @param  ADC_Channel[in]: ADC sampling channel selection.
 *         - ADC_Channel_0 : ADC input channel 0 
 *         - ADC_Channel_1 : ADC input channel 1 
 *         - ADC_Channel_2 : ADC input channel 2 
 *         - ADC_Channel_3 : ADC input channel 3  
 *         - ADC_Channel_4 : ADC input channel 4  
 *         - ADC_Channel_5 : ADC input channel 5  
 *         - ADC_Channel_6 : ADC input channel 6  
 *         - ADC_Channel_7 : ADC input channel 7  
 *         - ADC_Channel_8 : ADC input channel 8 
 *         - ADC_Channel_9 : ADC input channel 9  
 *         - ADC_Channel_10 :ADC input channel 10  
 *         - ADC_Channel_11 :ADC input channel 11 
 *         - ADC_Channel_12 :ADC input channel 12  
 *         - ADC_Channel_13 :ADC input channel 13 
 *         - ADC_Channel_14 :ADC input channel 14
 *         - ADC_Channel_15 :ADC input channel 15 
 *         - ADC_Channel_VDD_D4:ADC_CHANNEL VDD Divide 4
 *         -  ADC_Channel_TEMP:ADC_Channel_TEMP 
 * @retval None
 */
void ADC_SequenceChannelConfig(ADC_TypeDef* ADCx,ADC_SQRank_Typedef ADC_SQRank,ADC_ChannelTypedef ADC_Channel)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_SQRANK(ADC_SQRank));
    assert_param(IS_ADC_CHANNEL(ADC_Channel));

    *(((uint8_t *)&ADCx->ADC_SQx[0]) + ADC_SQRank) = ADC_Channel;
}

/**
 * @brief  Configure the ADC sequence sampling position and sampling number.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_SQGroup[in]:  ADC sample sequence group.
 *                  -ADC_SQGroup_0:ADC sequence group 0 
 *                  -ADC_SQGroup_1:ADC sequence group 1
 *                  -ADC_SQGroup_2:ADC sequence group 2
 *                  -ADC_SQGroup_3:ADC sequence group 3
 * @param  SQStr[in]: ADC sampling channel selection.
 * @param  SQCnt[in]: ADC Start channel selection.
 * @retval None
 */
void ADC_SequenceGroupConfig(ADC_TypeDef* ADCx,ADC_SQGroup_Typedef ADC_SQGroup,uint8_t SQStr,uint8_t SQCnt)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_SQGROUP(ADC_SQGroup));

    *((uint8_t *)& ADCx->ADC_SQCNT + ADC_SQGroup) = (SQCnt-1) |(SQStr<<4);
}

/**
 * @brief  Enabled or Disable the ADC sampling sequencen 
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_SQGroup[in]:  ADC sample sequence group.
 *                  -ADC_SQGroup_0:ADC sequence group 0 
 *                  -ADC_SQGroup_1:ADC sequence group 1
 *                  -ADC_SQGroup_2:ADC sequence group 2
 *                  -ADC_SQGroup_3:ADC sequence group 3
 * @param  NewState[in]: new state of the ADCx PWM trigger.
 *         - ENABLE: Function enable   
 *         - DISABLE:Function disable
 * @retval None
 */
void ADC_PWMTrigSQConvCongfig(ADC_TypeDef* ADCx,ADC_SQGroup_Typedef ADC_SQGroup,FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_SQGROUP(ADC_SQGroup));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
      ADCx->ADC_CON |= (ADC_CON_PWM_TRIG0 << ADC_SQGroup);
    }
    else
    {
      ADCx->ADC_CON &= ~(ADC_CON_PWM_TRIG0 << ADC_SQGroup);
    }
}

/**
 * @brief  Configure PWM trigger ADC sequence trigger edge signal output selector bit.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_TriggerOut[in]: ADC trigger signal output channel.
 *                  - ADC_TriggerOut_Less:ADC Trigger Out  Less  
 *                  - ADC_TriggerOut_0:Output from ADC_trigger0
 *                  - ADC_TriggerOut_1:Output from ADC_trigger1 
 * @retval None
 */
void ADC_RrigOutCongfig(ADC_TypeDef* ADCx,ADC_TriggerOut_TypeDef ADC_TriggerOut)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
    assert_param(IS_ADC_TRIGGEROUT(ADC_TriggerOut));

    ADCx->ADC_CON &= (uint32_t)(~ADC_CON_TRIG_OUT);
    ADCx->ADC_CON |= (ADC_TriggerOut);
}


/**
 * @brief  Obtain the upper and lower thresholds 
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  HighThreshold[in]: ADC high threshold.
 * @param  LowThreshold[in]: ADC low threshold.
 * @retval None
 */
void ADC_SetThresholds(ADC_TypeDef* ADCx, uint16_t HighThreshold,uint16_t LowThreshold)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));

    ADCx->ADC_UPTH = HighThreshold;
    ADCx->ADC_LOWTH = LowThreshold;
}

/**
 * @brief  The ADC channel threshold was enabled.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @param  ADC_Thresholds_Channel[in]: ADC thresholds channel.
 *                  - ADC_Thresholds_Channel_Less :Select ADC Channel Less 
 *                  - ADC_Thresholds_Channel_0: Select ADC  Channel 0 
 *                  - ADC_Thresholds_Channel_1: Select ADC  Channel 1 
 *                  - ADC_Thresholds_Channel_2: Select ADC  Channel 2
 *                  - ADC_Thresholds_Channel_3: Select ADC  Channel 3
 *                  - ADC_Thresholds_Channel_4: Select ADC  Channel 4
 *                  - ADC_Thresholds_Channel_5: Select ADC  Channel 5
 *                  - ADC_Thresholds_Channel_6: Select ADC  Channel 6
 *                  - ADC_Thresholds_Channel_7: Select ADC  Channel 7
 *                  - ADC_Thresholds_Channel_8: Select ADC  Channel 8
 *                  - ADC_Thresholds_Channel_9: Select ADC  Channel 9
 *                  - ADC_Thresholds_Channel_10: Select ADC  Channel 10
 *                  - ADC_Thresholds_Channel_11: Select ADC  Channel 11
 *                  - ADC_Thresholds_Channel_12: Select ADC  Channel 12
 *                  - ADC_Thresholds_Channel_13: Select ADC  Channel 13
 *                  - ADC_Thresholds_Channel_14: Select ADC  Channel 14
 *                  - ADC_Thresholds_Channel_15: Select ADC  Channel 15
 *                  - ADC_Thresholds_Channel_All: Select ADC  Channel All
 * @param  NewState[in]: new state of the ADCx Thresholds Channel.
 *         - ENABLE: Function enable   
 *         - DISABLE:Function disable
 * @retval None
 */
void ADC_ThresholdsChannelConfig(ADC_TypeDef* ADCx, uint32_t ADC_Thresholds_Channel, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(ADCx));
	  assert_param(IS_ADC_Thresholds_Channel(ADC_Thresholds_Channel));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
        ADCx->ADC_TH_CFG |= ADC_Thresholds_Channel;
    }
    else
    {
        ADCx->ADC_TH_CFG &= (~(ADC_Thresholds_Channel));
    }
}

/**
 * @brief  Returns the last ADCx double sample conversion result data for regular channel.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @retval the result of the latest ADCx conversion
 */
uint32_t ADC_GetDSConversionValueA(ADC_TypeDef* ADCx)
{
	uint16_t ADC_ValueA;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  /*Returns the result of the latest ADCx conversion */
	
   ADC_ValueA = (uint16_t)ADCx->ADC_VALUE;
  return ADC_ValueA;
}
/**
 * @brief  Returns the last ADCx double sample conversion result data for regular channel.
 * @param  ADCx[out]: select the ADCx peripheral.
 *                  - ADC: Only ADC can be select the ADCx peripheral.
 * @retval the result of the latest ADCx conversion
 */
uint32_t ADC_GetDSConversionValueB(ADC_TypeDef* ADCx)
{
	uint16_t ADC_ValueB;
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  /*Returns the result of the latest ADCx conversion */
	
   ADC_ValueB = (((uint16_t)ADCx->ADC_VALUE&0x7fff) >>16);
	return ADC_ValueB;
}
/**
 * @}
 */
/* End of ADC_Group4.	*/

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
