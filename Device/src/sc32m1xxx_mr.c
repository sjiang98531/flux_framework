/**
 ******************************************************************************
 * @file    sc32m1xxx_mr.c
 * @author  SOC Engineering Team
 * @version V0.1
 * @date    15-05-2024
 * @brief  MR function module
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
#include "sc32m1xxx_mr.h"

/** @addtogroup sc32m1xxx_StdPeriph_Driver
 * @{
 */

/** @defgroup mr
 * @brief mr driver modules
 * @{
 */

/** @defgroup MR_Exported_Functions
 * @{
 */

/** @defgroup MR_Exported_Functions_Group1 Configuration of the mr computation unit functions
 *  @brief   Configuration of the mr computation unit functions
 *
@verbatim
 ===============================================================================
                     ##### MR configuration functions #####
 ===============================================================================
@endverbatim
  * @{
  */
 MR_VAL MR_foc;
unsigned int Instr_PARK[3] ;
unsigned int Instr_InvPARK_to_SVPWM[4];

/**
 * @brief System analog circuit mr selection.
 * @param  mrx[out]: Only mr can be select the mrx peripheral.
 *                  - mr:select mr peripheral
 * @param  Peripheral_Voltage[in]: specifies the Peripheral_Voltage to check.
 *                  - mr_ReferenceVoltage_Default:mr Reference Voltage Default
 *                  - mr_ReferenceVoltage_2_048:mr Reference Voltage 2.048
 *                  - mr_ReferenceVoltage_1_024:mr Reference Voltage 1.024
 *                  - mr_ReferenceVoltage_2_4:mr Reference Voltage 2.4
 * @retval TNone
 */
 
 /**
 * @brief  DeInitializes the MR peripheral
 * @retval None
 */
void MR_DeInit(void)
{
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_MR,ENABLE);
}


 
/**
 * @brief  Initializes the MR peripheral
 * @param  ADC_InitStruct[out]: Pointer to structure MR_InitTypeDef,
 *                              to be initialized.
 * @retval None
 */
void MR_Init(MR_InitTypeDef* MR_InitStruct)
{
    uint32_t tmpreg;
    tmpreg = MR->CONFIG;
    tmpreg &= ~0xC1;
    tmpreg |= (MR_InitStruct->MR_ModulationMode | MR_InitStruct->MR_MulMode);             /* Ñ¡ÔñÓÐ·ûºÅ³ý·¨ */\
    if(MR_InitStruct->MR_SequenceMode == ENABLE)
    {
        tmpreg |= 0x01;
    }
    MR->CONFIG = tmpreg;
    MR->PWMPRD = MR_InitStruct->MR_PwmPriodLoad;

    MR->SADDR  = (unsigned int)&MR_foc;
    MR_foc.PI_AxisD.Ui        = 0      ;
    MR_foc.PI_AxisQ.Ui        = 0      ;
    MR_foc.PI_General.Sum     = 0      ;
    MR_foc.Math.Theta         = 0      ;
    MR_foc.Math.IniX         = 32767  ;
    MR_foc.Math.IniY         = 0      ;
    Instr_PARK[0]             = ((MR_INSTR_NEGATIVE) + ((MR_I_Theta_AddOffset   | W16) << HL) + ((FILLER) << LH) + (MR_I_Theta_AddOffset | W16));
    Instr_PARK[1]             = ((MR_INSTR_CORDIC_R) + ((MR_I_AxisD_AddOffset) << HL) + ((MR_I_Alpha_AddOffset) << LH) + (MR_I_Theta_AddOffset));
    Instr_PARK[2]             = 0x00000000;
    Instr_InvPARK_to_SVPWM[0] = ((MR_INSTR_CORDIC_R) + ((MR_V_Alpha_AddOffset   | W16) << HL) + ((MR_V_AxisD_AddOffset  | W16) << LH) + (MR_V_Theta_AddOffset | W16));
    Instr_InvPARK_to_SVPWM[1] = ((MR_INSTR_INVCLARK) + ((MR_V_AxisA_AddOffset  | W16) << HL) + ((MR_V_Beta_AddOffset   | W16) << LH) + (MR_V_Alpha_AddOffset | W16));
    Instr_InvPARK_to_SVPWM[2] = ((MR_INSTR_SVPWM)    + ((MR_V_DutyA_AddOffset   | W16) << HL) + ((MR_V_AxisA_AddOffset | W16) << LH) + (MR_V_AxisC_AddOffset  | W16));
    Instr_InvPARK_to_SVPWM[3] = 0x00000000;
}



/**
 * @}
 */
/* End of MR_Group1.	*/

/**
 * @}
 */
/* End of functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT SOC Microelectronics *****END OF FILE****/
