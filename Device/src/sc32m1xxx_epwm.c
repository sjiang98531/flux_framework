/**
 ******************************************************************************
 * @file    sc32m1xxx_epwm.c
 * @author  SOC AE Team
 * @version V0.1
 * @date    06-07-2024
 * @brief  EPWM function module
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
#include "sc32m1xxx_epwm.h"

/** @defgroup EPWM_Exported_Functions_Group1 Configuration of the EPWM computation unit functions
 *  @brief   Configuration of the EPWM computation unit functions.Common functions of motor
 *
@verbatim
 ===============================================================================
                     ##### EPWM configuration functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
 * @brief  DeInitializes the EPWMx peripheral
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
 * @retval None
 */
void EPWM_DeInit(EPWM_TypeDef* EPWMx)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    if(EPWMx == EPWM)
    {
        /* Enable EPWM reset state */
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_EPWM, ENABLE);
        /* Enable EPWM reset state */
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_EPWM, DISABLE);
    }
}

/**
  * @brief  Fills each EPWM_InitStruct member with its default value.
  * @param  EPWM_InitStruct[out]:Pointer to structure EPWM_InitTypeDef, to be initialized.
  * @retval None
  */
void EPWM_StructInit(EPWM_InitTypeDef* EPWM_InitStruct)
{
    /* Set the default configuration */
    EPWM_InitStruct->EPWM_Prescaler = EPWM_PRESCALER_DIV1;
    EPWM_InitStruct->EPWM_AlignedMode = EPWM_AlignmentMode_Edge;
    EPWM_InitStruct->EPWM_WorkMode = EPWM_WorkMode_Independent;
    EPWM_InitStruct->EPWM_Cycle = 0x0000;
    EPWM_InitStruct->EPWM_OutputChannel = EPWM_Channel_Less;
    EPWM_InitStruct->EPWM_LowPolarityChannl = EPWM_Channel_Less;
}

/**
  * @brief  Initializes the EPWMx peripheral according to
  *         the specified parameters in the EPWM_Base_InitStruct.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
  * @param  EPWM_InitStruct[out]:Pointer to structure EPWM_InitTypeDef, to be initialized.
 * @retval None
 */
void EPWM_Init(EPWM_TypeDef* EPWMx, EPWM_InitTypeDef* EPWM_InitStruct)
{
    uint32_t tmpreg;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /*---------------------------- PWMx EPWM_CON Configuration ------------------------*/
    /* Get the PWMx EPWM_CON value */
    tmpreg = EPWMx->EPWM_CON;
    /* Clear PWMCLK, PWMMD0 and PWMMD1 SPR bits */
    tmpreg &= (uint32_t)~(EPWM_CON_PWMCLK | EPWM_CON_PWMMD0 | EPWM_CON_PWMMD1);
    /* Configure PWMx: Prescaler, AlignedMode and WorkMode */
    /* Set EPWMCLK bits according to Prescaler value */
    /* Set EPWMMD0 bit according to AlignedMode value */
    /* Set EPWMMD1 bit according to WorkMode value */
    tmpreg |= (uint32_t)(EPWM_InitStruct->EPWM_Prescaler | EPWM_InitStruct->EPWM_AlignedMode |
                         EPWM_InitStruct->EPWM_WorkMode);

    /* Write to EPWM EPWM_CON */
    EPWMx->EPWM_CON = tmpreg;

    /* Write to EPWM EPWM_CHN */
    EPWMx->EPWM_CHN = EPWM_InitStruct->EPWM_OutputChannel;

    /* Write to EPWM EPWM_INV */
    EPWMx->EPWM_INV = EPWM_InitStruct->EPWM_LowPolarityChannl;
//    EPWMx->EPWM_INV  = 0x2A;

    /* Write to PWMx EPWM_CYCLE */
    EPWMx->EPWM_CYCLE = EPWM_InitStruct->EPWM_Cycle;
}


/**
 * @brief  Configures the EPWMx Rising Dead Time Register value
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
 * @param  EPWM_RisingDeadTime[in]: specifies the Rising Dead Time register new value
 * @retval None.
 */
void EPWM_RisingDeadTimeConfig(EPWM_TypeDef* EPWMx, uint8_t EPWM_RisingDeadTime)
{
    uint32_t tmpreg;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /* Get the EPWM EPWM_DFR value */
    tmpreg = EPWMx->EPWM_DFR;

    /* Clear PDR bits */
    tmpreg &= (uint32_t) ~(EPWM_DFR_PDR);

    /* Set PDR bits to Rising Dead Time value */
    tmpreg |= (uint32_t)(EPWM_RisingDeadTime << EPWM_DFR_PDR_Pos);

    /* Write to EPWM EPWM_DFR */
    EPWMx->EPWM_DFR = tmpreg;
}

/**
 * @brief  Configures the EPWMx Dead Falling Time Register value
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
 * @param  EPWM_fallingDeadTime[in]: specifies the Falling Dead Time register new value.
 * @retval None.
 */
void EPWM_FallingDeadTimeConfig(EPWM_TypeDef* EPWMx, uint8_t EPWM_FallingDeadTime)
{
    uint32_t tmpreg;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /* Get the EPWM EPWM_DFR value */
    tmpreg = EPWMx->EPWM_DFR;

    /* Clear PDF bits */
    tmpreg &= (uint32_t) ~(EPWM_DFR_PDF);

    /* Set PDF bits to Rising Dead Time value */
    tmpreg |= (uint32_t)(EPWM_FallingDeadTime << EPWM_DFR_PDF_Pos);

    /* Write to EPWM EPWM_DFR */
    EPWMx->EPWM_DFR = tmpreg;
}

/**
 * @brief  Enables or disables the specified EPWMx peripheral.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
 * @param  NewState[in]: new state of the EPWM peripheral.
  *                  - DISABLE:Function disable
  *                  - ENABLE:Function enable
 * @retval None
 */
void EPWM_Cmd(EPWM_TypeDef* EPWMx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
        /* Enable the EPWM Counter */
        EPWMx->EPWM_CON |= EPWM_CON_ENPWM;
    }
    else
    {
        /* Disable the EPWM Counter */
        EPWMx->EPWM_CON &= (uint16_t)~EPWM_CON_ENPWM;
    }
}

/**
 * @brief  Enables or disables the specified EPWMx peripheral.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             -  EPWM:Select EPWM peripherals
 * @param  EPWM_MAP[in]: Switchover of the output port mapping.
 *             -  EPWM_MAP_Normal:EPWM MAP Normal
 *             -  EPWM_MAP_Alternate:EPWM MAP Alternate
 * @retval None
 */
void EPWM_MAPConfig(EPWM_TypeDef* EPWMx, EPWM_MAP_Typedef EPWM_MAP)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_CHANNEL(EPWM_MAP));

    EPWMx->EPWM_CON &= (uint16_t)~EPWM_CON_MAP;

    EPWMx->EPWM_CON |= EPWM_MAP;
}

/**
* @brief  Enables or disables the specified EPWMx peripheral.
* @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
*             -  EPWM:Select EPWM peripherals
* @param  EPWM_SWAP[in]: Output port switching.
*             -  EPWM_SWAP_Normal:EPWM SWAP Normal
*             -  EPWM_SWAP_Alternate:EPWM SWAP Alternate
* @retval None
*/
void EPWM_SWAPConfig(EPWM_TypeDef* EPWMx, EPWM_SWAP_Typedef EPWM_SWAP)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_SWAP(EPWM_SWAP));

    EPWMx->EPWM_CON &= (uint16_t)~EPWM_CON_SWAP;

    EPWMx->EPWM_CON |= EPWM_SWAP;
}

/**
 * @brief  Sets the EPWMx Clock Division value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Prescaler[in]: specifies the clock division value.
 *             - EPWM_PRESCALER_DIV1:Clock division: Fsource/1
 *             - EPWM_PRESCALER_DIV2:Clock division: Fsource/2
 *             - EPWM_PRESCALER_DIV4:Clock division: Fsource/4
 *             - EPWM_PRESCALER_DIV8:Clock division: Fsource/8
 *             - EPWM_PRESCALER_DIV16:Clock division: Fsource/16
 *             - EPWM_PRESCALER_DIV32:Clock division: Fsource/32
 *             - EPWM_PRESCALER_DIV64:Clock division: Fsource/64
 *             - EPWM_PRESCALER_DIV128:Clock division: Fsource/128
 * @retval None
 */
void EPWM_SetPrescaler(EPWM_TypeDef* EPWMx, EPWM_Prescaler_TypeDef EPWM_Prescaler)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_PRESCALER(EPWM_Prescaler));

    /* Reset the CKD Bits */
    EPWMx->EPWM_CON &= (uint16_t)~(EPWM_CON_PWMCLK);

    /* Set the CKD value */
    EPWMx->EPWM_CON |= EPWM_Prescaler;
}

/**
 * @brief  Gets the EPWMx Clock Division value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @retval The clock division value.
 *             - EPWM_PRESCALER_DIV1:Clock division: Fsource/1
 *             - EPWM_PRESCALER_DIV2:Clock division: Fsource/2
 *             - EPWM_PRESCALER_DIV4:Clock division: Fsource/4
 *             - EPWM_PRESCALER_DIV8:Clock division: Fsource/8
 *             - EPWM_PRESCALER_DIV16:Clock division: Fsource/16
 *             - EPWM_PRESCALER_DIV32:Clock division: Fsource/32
 *             - EPWM_PRESCALER_DIV64:Clock division: Fsource/64
 *             - EPWM_PRESCALER_DIV128:Clock division: Fsource/128
 */
EPWM_Prescaler_TypeDef EPWM_GetPrescaler(EPWM_TypeDef* EPWMx)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /* Get the CKD value */
    return (EPWM_Prescaler_TypeDef)(EPWMx->EPWM_CON & EPWM_CON_PWMCLK);
}


/**
 * @brief  Sets the EPWMx Cycle Register value
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Cycle[in]: specifies the ReloadData register new value.
 * @retval None
 */
void EPWM_SetCycle(EPWM_TypeDef* EPWMx, uint32_t EPWM_Cycle)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /* Set the ReloadData Register value */
    EPWMx->EPWM_CYCLE = EPWM_Cycle;
}

/**
 * @brief  Gets the EPWM Cycle Register value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @retval Period Cycle of EPWM
 */
uint16_t EPWM_GetCycle(EPWM_TypeDef* EPWMx)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    /* Get the ReloadData Register value */
    return (uint16_t)EPWMx->EPWM_CYCLE;
}

/**
 * @brief  Sets the EPWMx Duty value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Channel[in]: specifies the EPWM channel to check.
 *             - EPWM_Channel_Less:No channels are selected
 *             - EPWM_Channel_0:EPWM output channel 0
 *             - EPWM_Channel_1:EPWM output channel 1
 *             - EPWM_Channel_2:EPWM output channel 2
 *             - EPWM_Channel_3:EPWM output channel 3
 *             - EPWM_Channel_4:EPWM output channel 4
 *             - EPWM_Channel_5:EPWM output channel 5
 *             - EPWM_Channel_6:EPWM output channel 6
 *             - EPWM_Channel_7:EPWM output channel 7
 *             - EPWM_Channel_All:EPWM output channel ALL
 * @param  EPWM_Duty[in]: specifies the Duty register new value.
 * @retval None
 */
void EPWM_SetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel, uint16_t EPWM_Duty)
{
    uint8_t tmpvalue;
    uint32_t tmpchannel;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_CHANNEL(EPWM_Channel));

    tmpchannel = 1;

    for(tmpvalue = 0; tmpvalue < 8; tmpvalue++)
    {
        if((uint32_t)EPWM_Channel & tmpchannel)
        {
            EPWMx->EPWM_DT[tmpvalue] = EPWM_Duty;
        }
        tmpchannel = tmpchannel << 1;
    }
}
/**
 * @brief  Sets the EPWMx Duty value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  NewState[in]: new state of the EPWM Asymmetry .
 *                  - DISABLE:Function disable
 *                  - ENABLE:Function enable
 * @retval None
 */
void EPWM_AsymmetryCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState)
{
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    /* The prerequisite for enabling asymmetric mode is to enable center alignment mode and complementary mode */
    if((EPWMx->EPWM_CON & (EPWM_CON_PWMMD0 | EPWM_CON_PWMMD1)))
    {
        if(NewState != DISABLE)
        {
            /* Enable the PWM Counter */
            EPWMx->EPWM_CON |= EPWM_CON_ASYMEN;
        }
        else
        {
            /* Disable the PWM Counter */
            EPWMx->EPWM_CON &= (uint16_t)~EPWM_CON_ASYMEN;
        }
    }
}

/**
 * @brief  Sets the EPWM Duty value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Channel[in]: specifies the PWM channel to check.
 *             - EPWM_Channel_Less:No channels are selected
 *             - EPWM_Channel_0:EPWM output channel 0
 *             - EPWM_Channel_1:EPWM output channel 1
 *             - EPWM_Channel_2:EPWM output channel 2
 *             - EPWM_Channel_3:EPWM output channel 3
 *             - EPWM_Channel_4:EPWM output channel 4
 *             - EPWM_Channel_5:EPWM output channel 5
 *             - EPWM_Channel_6:EPWM output channel 6
 *             - EPWM_Channel_7:EPWM output channel 7
 *             - EPWM_Channel_All:EPWM output channel ALL
 * @param  EPWM_CmpUp[in]:EPWMx Upcount waveform upturn value Set.
 * @param  EPWM_CmpDown[in]:EPWMx Waveform downward count flip value Set
 * @retval None
 */
void EPWM_Asymmetry_SetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel, uint16_t EPWM_CmpUp, uint16_t EPWM_CmpDown)
{
    uint8_t tmpvalue;
    uint32_t tmpchannel;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_CHANNEL(EPWM_Channel));

    tmpchannel = 1;
    if((EPWMx->EPWM_CON & 0x10) != 0)
    {
        for(tmpvalue = 0; tmpvalue < 8; tmpvalue++)
        {
            if((uint32_t)EPWM_Channel & tmpchannel)
            {
                if((tmpvalue & 0x01) != 0)
                {
                    EPWMx->EPWM_DT[tmpvalue - 1] = EPWM_CmpUp;
                    EPWMx->EPWM_DT[tmpvalue] = EPWM_CmpDown;
                }
                else
                {
                    EPWMx->EPWM_DT[tmpvalue] = EPWM_CmpUp;
                    EPWMx->EPWM_DT[tmpvalue + 1] = EPWM_CmpDown;
                }

            }
            tmpchannel = tmpchannel << 1;
        }

    }

}

/**
 * @brief  Gets the EPWMx Duty value.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Channel[in]: specifies the EPWM channel to check.
 *             - EPWM_Channel_Less:No channels are selected
 *             - EPWM_Channel_0:EPWM output channel 0
 *             - EPWM_Channel_1:EPWM output channel 1
 *             - EPWM_Channel_2:EPWM output channel 2
 *             - EPWM_Channel_3:EPWM output channel 3
 *             - EPWM_Channel_4:EPWM output channel 4
 *             - EPWM_Channel_5:EPWM output channel 5
 *             - EPWM_Channel_6:EPWM output channel 6
 *             - EPWM_Channel_7:EPWM output channel 7
 *             - EPWM_Channel_All:EPWM output channel ALL
 * @retval the Duty register new value.
 */
uint16_t EPWM_GetDuty(EPWM_TypeDef* EPWMx, EPWM_Channel_Typedef EPWM_Channel)
{
    uint8_t tmpvalue ;
    uint32_t tmpchannel;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_CHANNEL(EPWM_Channel));

    tmpchannel = 1;
    for(tmpvalue = 0; tmpvalue < 8; tmpvalue++)
    {
        if((uint32_t)EPWM_Channel & tmpchannel)
        {
            return (uint16_t)(EPWMx->EPWM_DT[tmpvalue]);
        }
        tmpchannel = tmpchannel << 1;
    }
    return 0;
}

/**
 * @}
 */
/* End of EPWM_Group1.	*/



/** @defgroup EPWM_Group2 Fault Dectection management functions
 *  @brief   Fault Dectection management functions
 *
@verbatim
 ===============================================================================
                     ##### EPWM Falut Dectection management functions #####
 ===============================================================================
@endverbatim
  * @{
  */

/**
  * @brief  Fill each parameter in EPWM_FDInitStruct with its default value.
  * @param  EPWM_FDInitStruct[out] : pointer to a EPWM_FDInitStruct
  *         structure which will be initialized.
  * @retval None
  */
void EPWM_FDStructInit(EPWM_FDInitTypeDef* EPWM_FDInitStruct)
{
    /* Set the default configuration */

    EPWM_FDInitStruct->EPWM_FDSoure = 0;
    EPWM_FDInitStruct->EPWM_FDSoureCmd = DISABLE;
    EPWM_FDInitStruct->EPWM_FDMode = EPWM_FDMODE_CBC;
	  EPWM_FDInitStruct->EPWM_FDTriggerLV = EPWM_FDTriggerLV_Low;
    EPWM_FDInitStruct->EPWM_FDFilteringTime = EPWM_FilteringTime_0us;
}

/**
 * @brief  Set EPWM fault detection mode.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
  * @param  EPWM_FDInitStruct[out] : pointer to a EPWM_FDInitStruct
  *         structure which will be initialized.
 * @retval None
 */
void EPWM_FDInit(EPWM_TypeDef* EPWMx, EPWM_FDInitTypeDef* EPWM_FDInitStruct)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    uint32_t Tmpreg0;
	
    union
    {  
        u8  Str[4];
			  u32 Reg;
    } TmpUnion;
		
		/* Set FILTER[2:0] bits according to Prescaler value */
		Tmpreg0 =  EPWMx->EPWM_FLT_CFG0;
		Tmpreg0 &= EPWM_FLTCFG0_FILTER;
		Tmpreg0 |= EPWM_FDInitStruct->EPWM_FDFilteringTime;
		
		TmpUnion.Reg = EPWMx->EPWM_FLT_CFG1;
		/* Set EN bits according to Prescaler value */
		if(EPWM_FDInitStruct->EPWM_FDSoureCmd == ENABLE)
		{
			TmpUnion.Str[0] |= EPWM_FDInitStruct->EPWM_FDSoure;
		}
		else
		{
			TmpUnion.Str[0] &= EPWM_FDInitStruct->EPWM_FDSoure;
		}		
		
		/* Set EVT bits according to Prescaler value */
		if(EPWM_FDInitStruct->EPWM_FDMode == EPWM_FDMODE_CBC)
		{
			TmpUnion.Str[1] |= EPWM_FDInitStruct->EPWM_FDSoure;
		}
		else
		{
			TmpUnion.Str[1] &= EPWM_FDInitStruct->EPWM_FDSoure;
		}
		
		/* Set LV bits according to Prescaler value */
		if(EPWM_FDInitStruct->EPWM_FDTriggerLV == EPWM_FDTriggerLV_Hight)
		{
			TmpUnion.Str[2] |= EPWM_FDInitStruct->EPWM_FDSoure;
		}
		else
		{
			TmpUnion.Str[2] &= EPWM_FDInitStruct->EPWM_FDSoure;
		}

    EPWMx->EPWM_FLT_CFG1 = TmpUnion.Reg;
}

/**
 * @brief  The HALT status control of all enabled channels of EPWM function is enabled or disabled.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  NewState: new state of the EPWM HALT status control.
 *         - ENABLE: Function enable
 *         - DISABLE:Function disable
 * @retval None
 */

void EPWM_FDSoftwareStart(EPWM_TypeDef* EPWMx)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));


    EPWMx->EPWM_FLT_CFG1 |= (uint16_t)EPWM_FLTCFG1_SWTRG;


}


/**
 * @brief  The HALT status control of all enabled channels of EPWM function is enabled or disabled.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  NewState: new state of the EPWM HALT status control.
 *         - ENABLE: Function enable
 *         - DISABLE:Function disable
 * @retval None
 */

void EPWM_FD_HALTMSCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));

    if(NewState != DISABLE)
    {
        EPWMx->EPWM_FLT_CFG0 |= (uint16_t)EPWM_FLTCFG0_HALTMS;
    }
    else
    {
        EPWMx->EPWM_FLT_CFG0 &= (uint16_t)~EPWM_FLTCFG0_HALTMS;
    }

}

/**
 * @brief  This section describes how to configure the port status for each EPWM fault.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_FDOutputState[in]: The port status of each EPWM when the end that triggers a fault triggers a fault.
*             - EPWM_FDOutputState_HI:PWM fault detection Voltage:Low level
*             - EPWM_FDOutputState_PP:PWM fault detection Voltage:High level
 * @retval None
 */
void EPWM_FDOutputStateConfig(EPWM_TypeDef* EPWMx, EPWM_FDOutputState_TypeDef EPWM_FDOutputState)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_FDOUTPUTSTATE(EPWM_FDOutputState));

    if(EPWM_FDOutputState != EPWM_FDOutputState_HI)
    {
        EPWMx->EPWM_FLT_CFG1 |= EPWM_FLTCFG1_FLTBRK;
    }
    else
    {
        EPWMx->EPWM_FLT_CFG1 &= (uint32_t)(~EPWM_FLTCFG1_FLTBRK);
    }
}

/**
 * @brief  This section describes how to configure the port level for each EPWM fault.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_Channel[in]: specifies the EPWM channel to check.
 *             - EPWM_Channel_Less:No channels are selected
 *             - EPWM_Channel_0:EPWM output channel 0
 *             - EPWM_Channel_1:EPWM output channel 1
 *             - EPWM_Channel_2:EPWM output channel 2
 *             - EPWM_Channel_3:EPWM output channel 3
 *             - EPWM_Channel_4:EPWM output channel 4
 *             - EPWM_Channel_5:EPWM output channel 5
 *             - EPWM_Channel_6:EPWM output channel 6
 *             - EPWM_Channel_7:EPWM output channel 7
 *             - EPWM_Channel_All:EPWM output channel ALL
 * @param  EPWM_FDOutputLevel[in]: The port level of each EPWM when the end that triggers a fault triggers a fault.
 * @note   Need to set the port status to push-pull after the fault is triggered.
 * @retval None
 */
void EPWM_FDOutputLevelConfig(EPWM_TypeDef* EPWMx, uint16_t EPWM_Channel, EPWM_FDOutputLevel_TypeDef EPWM_FDOutputLevel)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_CHANNEL(EPWM_Channel));
    assert_param(IS_EPWM_FDOUTPUTLEVEL(EPWM_FDOutputLevel));

    if(EPWM_FDOutputLevel != EPWM_FDOutputLevel_Low)
    {
        EPWMx->EPWM_FLT_CFG1 |= (((uint32_t)EPWM_Channel) << (EPWM_FLTCFG1_PWMFLT0_Pos));
    }
    else
    {
        EPWMx->EPWM_FLT_CFG1 &= ~(((uint32_t)EPWM_Channel) << (EPWM_FLTCFG1_PWMFLT0_Pos));
    }

}

/**
 * @brief  The EPWM fault detection function is enabled or disabled.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  NewState[in]: new state of the EPWM fault detection .
 *         - ENABLE: Function enable
 *         - DISABLE:Function disable
 * @retval None
 */
void EPWM_FDCmd(EPWM_TypeDef* EPWMx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
        /* Enable the EPWM Fault Dectection */
        EPWMx->EPWM_FLT_CFG0 |= EPWM_FLTCFG0_FLTEN;
    }
    else
    {
        /* Disable the EPWM Fault Dectection */
        EPWMx->EPWM_FLT_CFG0 &= (uint16_t)~EPWM_FLTCFG0_FLTEN;
    }
}

/**
 * @brief  Select the EPWM fault detection function mode.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_FDSoure: Trigger event Settings for each trigger source.
 *             - EPWM_FDMODE_SWOS:EPWM fault detection soure: SW One-shot
 *             - EPWM_FDMODE_SWCBC:EPWM fault detection soure: SW cycle by cycle
 *             - EPWM_FDMODE_OP2OS:EPWM fault detection soure: OP2 One-shot
 *             - EPWM_FDMODE_OP2CBC:EPWM fault detection soure: OP2 cycle by cycle
 *             - EPWM_FDMODE_OP1OS:EPWM fault detection soure: OP1 One-shot
 *             - EPWM_FDMODE_OP1CBC:EPWM fault detection soure: OP1 cycle by cycle
 *             - EPWM_FDMODE_CMP3OS:EPWM fault detection soure: CMP One-shot
 *             - EPWM_FDMODE_CMP3CBC:EPWM fault detection soure: CMP cycle by cycle
 *             - EPWM_FDMODE_CMP0OS:EPWM fault detection soure: CMP0 One-shot
 *             - EPWM_FDMODE_CMP0CBC:EPWM fault detection soure: CMP0 cycle by cycle
 *             - EPWM_FDMODE_FLTPINOS:EPWM fault detection soure: External FLT One-shot
 *             - EPWM_FDMODE_FLTPINCBC:EPWM fault detection soure: External FLT cycle by cycle
 * @retval None
 */
void EPWM_FDModeCongfig(EPWM_TypeDef* EPWMx,EPWM_FDSoure_TypeDef EPWM_FDSoure,EPWM_FDMode_TypeDef EPWM_FDMode)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_FDMODE(EPWM_FDMode));

    union
    {  
        u8  Str[4];
			  u32 Reg;
    } TmpUnion;
		
		TmpUnion.Reg = EPWMx->EPWM_FLT_CFG1;
		/* Set LV bits according to Prescaler value */
		if(EPWM_FDMode == EPWM_FDMODE_OS)
		{
			TmpUnion.Str[1] |= EPWM_FDSoure;
		}
		else
		{
			TmpUnion.Str[1] &= EPWM_FDSoure;
		}

    EPWMx->EPWM_FLT_CFG1 = TmpUnion.Reg;
}

/**
 * @brief  Select the EPWM fault detection function mode.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_TriggerLV:Fault level selection
 *             - EPWM_OP2LV_Hight: OP2 hight level fault is valid
 *             - EPWM_OP1LV_LOW: OP1 Low level fault is valid
 *             - EPWM_OP1LV_Hight: OP1 hight level fault is valid
 *             - EPWM_CMP3LV_LOW: CMP3 Low level fault is valid
 *             - EPWM_CMP3LV_Hight: CMP3 hight level fault is valid
 *             - EPWM_CMP0LV_LOW: CMP0 Low level fault is valid
 *             - EPWM_CMP0LV_Hight: CMP0 hight level fault is valid
 *             - EPWM_FLTPINLV_LOW: FLTPIN Low level fault is valid
 *             - EPWM_FLTPINLV_Hight: FLTPIN hight level fault is valid
 * @retval None
 */
void EPWM_FDTriggerLVCongfig(EPWM_TypeDef* EPWMx, EPWM_FDSoure_TypeDef EPWM_FDSoure,EPWM_FDTriggerLV_TypeDef EPWM_TriggerLV)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_TriggerLV(EPWM_TriggerLV));
	
    union
    {  
        u8  Str[4];
			  u32 Reg;
    } TmpUnion;
		
		TmpUnion.Reg = EPWMx->EPWM_FLT_CFG1;
		/* Set LV bits according to Prescaler value */
		if(EPWM_TriggerLV == EPWM_FDTriggerLV_Hight)
		{
			TmpUnion.Str[2] |= EPWM_FDSoure;
		}
		else
		{
			TmpUnion.Str[2] &= EPWM_FDSoure;
		}

    EPWMx->EPWM_FLT_CFG1 = TmpUnion.Reg;
}
/**
 * @}
 */
/* End of EPWM_Group2.	*/

/** @defgroup EPWM_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim
 ===============================================================================
                     ##### Interrupts and flags management functions #####
 ===============================================================================
@endverbatim
  * @{
  */
/**
 * @brief  Set the count trigger mode.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_ADCTrigGroup[in]: Select group number
 *             - EPWM_ADCTrigGroup0:EPWM trigger Group0
 *             - EPWM_ADCTrigGroup1:EPWM trigger Group1
 *             - EPWM_ADCTrigGroup2:EPWM trigger Group2
 *             - EPWM_ADCTrigGroup3:EPWM trigger Group3
 * @param  EPWM_TrigDir[in]: Counting mode triggers selection.
 *             - EPWM_ADCTrigDir_Up:Up-count trigger
 *             - EPWM_ADCTrigDir_Down:Down-count trigger
 * @retval None
 */
void EPWM_SetADCTrigDir(EPWM_TypeDef* EPWMx, EPWM_ADCTrigGroup_TypeDef EPWM_ADCTrigGroup, EPWM_ADCTrigDir_TypeDef EPWM_TrigDir)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_ADCTrigGroup(EPWM_ADCTrigGroup));
    assert_param(IS_EPWM_TRIGDIR(EPWM_TrigDir));
    if(EPWM_TrigDir != EPWM_ADCTrigDir_Up)
    {
        /* Disable the Interrupt sources */
        EPWMx->EPWM_ADCTRGDT[EPWM_ADCTrigGroup] |= (uint32_t)EPWM_ADCTRG_DIR;
    }
    else
    {

        /* Enable the Interrupt sources */
        EPWMx->EPWM_ADCTRGDT[EPWM_ADCTrigGroup] &= (uint32_t)~EPWM_ADCTRG_DIR;
    }
}

/**
 * @brief  Enables or disables the specified PWM interrupts.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_ADCTrigGroup[in]: Select group number
 *             - EPWM_ADCTrigGroup0:EPWM trigger Group0
 *             - EPWM_ADCTrigGroup1:EPWM trigger Group1
 *             - EPWM_ADCTrigGroup2:EPWM trigger Group2
 *             - EPWM_ADCTrigGroup3:EPWM trigger Group3
 * @param  ADC_TrigCnt[in]:EPWM triggers the timing configuration of the ADC sampling sequence n
 * @retval None
 */
void EPWM_SetADCTrigCnt(EPWM_TypeDef* EPWMx, EPWM_ADCTrigGroup_TypeDef EPWM_ADCTrigGroup, uint32_t ADC_TrigCnt)
{
    uint32_t tmpreg;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_ADCTrigGroup(EPWM_ADCTrigGroup));

    tmpreg = EPWMx->EPWM_ADCTRGDT[EPWM_ADCTrigGroup];

    tmpreg &= (uint32_t)(~EPWM_ADCTRG_SOCCNT);

    tmpreg |= (uint32_t)ADC_TrigCnt;

    EPWMx->EPWM_ADCTRGDT[EPWM_ADCTrigGroup] = tmpreg;
}

/** 
 * @}
 */
/* End of EPWM_Group3.	*/

/**
 * @}
 */
/* End of functions ---------------------------------------------------------*/


/** @defgroup EPWM_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim
 ===============================================================================
                     ##### Interrupts and flags management functions #####
 ===============================================================================
@endverbatim
  * @{
  */
/**
* @brief  Configure remapping pins for SPIx
* @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
*             - EPWM:Select EPWM peripherals
* @param  EPWM_Remap[in]: specifies the EPWM  remapping source.
*             - EPWM_PinRemap_Default:EPWM PinRemap Default
*             - EPWM_PinRemap_A:EPWM PinRemap A
* @retval None
*/
void EPWM_PinRemapConfig(EPWM_TypeDef* EPWMx, EPWM_PinRemap_TypeDef EPWM_Remap)
{
    uint32_t tmpreg ;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_PinRemap(EPWM_Remap));

    tmpreg = EPWMx->EPWM_CON;

    tmpreg &= (uint32_t)(~EPWM_CON_SPOS);

    tmpreg |= EPWM_Remap;

    EPWMx->EPWM_CON = tmpreg;
}
/**
 * @brief  Enables or disables the specified EPWMx interrupts.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_IT[in]: specifies the PWM interrupts sources to be enabled or disabled.
 *             - EPWM_IT_OVF:EPWM Interrupt: EPWM OVF
 *             - EPWM_IT_UNF:EPWM Interrupt: EPWM UNF
 *             - EPWM_IT_CBC:EPWM Interrupt: EPWM CBC
 *             - EPWM_IT_OST:EPWM Interrupt: EPWM OST
 *             - EPWM_IT_INTEN:EPWM Interrupt: EPWM INTEN
 * @param  NewState: new state of the EPWM interrupts.
 *         - ENABLE: Function enable
 *         - DISABLE:Function disable
 * @retval None
 */
void EPWM_ITConfig(EPWM_TypeDef* EPWMx, uint16_t EPWM_IT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_IT(EPWM_IT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if(NewState != DISABLE)
    {
        /* Enable the Interrupt sources */
        EPWMx->EPWM_IDE |= EPWM_IT;
    }
    else
    {
        /* Disable the Interrupt sources */
        EPWMx->EPWM_IDE &= (uint16_t)~EPWM_IT;
    }
}

/**
 * @brief  Checks whether the specified EPWM flag is set or not.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_FLAG[in]: specifies the flag to check.
 *             - EPWM_Flag_OVF:EPWM OVF Interrupt Flag
 *             - EPWM_Flag_UNF:EPWM UNF Interrupt Flag
 *             - EPWM_Flag_CBC:EPWM CBC Interrupt Flag
 *             - EPWM_Flag_OST:EPWM OST Interrupt Flag
 * @retval The new state of EPWM_FLAG (SET or RESET).
 *                  -  RESET:Flag reset
 *                  -  SET :Flag up
 */
FlagStatus EPWM_GetFlagStatus(EPWM_TypeDef* EPWMx, uint16_t EPWM_FLAG)
{
    ITStatus bitstatus = RESET;
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_FLAG(EPWM_FLAG));

    if((EPWMx->EPWM_STS & EPWM_FLAG) != (uint16_t)RESET)
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
 * @brief  Clears the EPWMx's pending flags.
 * @param  EPWMx[out]: where x can be to select the EPWMx peripheral.
 *             - EPWM:Select EPWM peripherals
 * @param  EPWM_FLAG[in]: specifies the flag bit to clear.
 *             - EPWM_Flag_OVF:EPWM OVF Interrupt Flag
 *             - EPWM_Flag_UNF:EPWM UNF Interrupt Flag
 *             - EPWM_Flag_CBC:EPWM CBC Interrupt Flag
 *             - EPWM_Flag_OST:EPWM OST Interrupt Flag
 * @retval None
 */
void EPWM_ClearFlag(EPWM_TypeDef* EPWMx, uint16_t EPWM_FLAG)
{
    /* Check the parameters */
    assert_param(IS_EPWM_ALL_PERIPH(EPWMx));
    assert_param(IS_EPWM_FLAG(EPWM_FLAG));

    /* Clear the flags */
    EPWMx->EPWM_STS = (uint16_t)EPWM_FLAG;
}

/**
 * @}
 */
/* End of EPWM_Group3.	*/

/**
 * @}
 */
/* End of functions ---------------------------------------------------------*/



/************************ (C) COPYRIGHT SOC Microelectronics *****END OF FILE****/
