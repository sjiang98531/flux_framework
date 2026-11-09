/**
 ******************************************************************************
 * @file    sc32m1xxx_mr.h
 * @author  SOC Engineering Team
 * @version V1.0
 * @date    15-05-2024
 * @brief   Header file of MR module.
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
 
#ifndef __sc32f10xx_MR_H
#define __sc32f10xx_MR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "sc32m156x.h"
#include "sc32.h"
#include "sc32m1xxx_rcc.h"

/* Exported enumerations ------------------------------------------------------------*/
/** @defgroup MR_Exported_Enumerations MR Exported Enumerations
 * @{
 */
 
 /** @defgroup MR_ModulationMode MR Modulation Mode
 * @{
 */
 typedef enum
{
    MR_SevenSVPWM  = (uint16_t)(0x00),   /* SEVEN-segment Space Vector Pulse Width Modulation */
    MR_FiveSVPWM   = (uint16_t)(0x04),   /* Five-segment Space Vector Pulse Width Modulation */
	MR_SPWM        = (uint16_t)(0x08),   /* Space Pulse Width Modulation */
} MR_ModulationMode_TypeDef;

#define IS_MR_MODULATIONMODE(MODE) (((MODE) == MR_SevenSVPWM) ||  \
                                    ((MODE) == MR_FiveSVPWM) ||  \
                                    ((MODE) == MR_SPWM))
 /**
 * @}
 */

 /** @defgroup MR_ModulationMode MR Modulation Mode
 * @{
 */
 typedef enum
{
    MR_UnsingedMul  = (uint16_t)(0x00),   /* Unsigned division */
    MR_SingedMul    = (uint16_t)(0x20),   /* Signed division */
} MR_MulMode_TypeDef;

#define IS_MR_MULMODE(MODE) (((MODE) == MR_SevenSVPWM) ||  \
                             ((MODE) == MR_SPWM))
 /**
 * @}
 */

/**
 * @}
 */
/* End of enumerations -----------------------------------------------------*/

/** @defgroup MR_Exported_Struct MR Exported Struct
 * @{
 */

/** @defgroup MR Clarl and Park Structure definition
 * @{
 */
typedef struct
{
    volatile short int PhaseU;
    volatile short int PhaseV;
    volatile short int PhaseW;
    volatile short int Theta;
    volatile short int Alpha;
    volatile short int Beta;
    volatile short int Arctan;
    volatile short int Direct;
    volatile short int Quadrature;
    volatile short int Filler;
} CLARK_AND_PARK, *CLARK_AND_PARK_handle;
/**
 * @}
 */

/** @defgroup MR InvPark and SVPWM Structure definition
 * @{
 */
typedef struct
{
    volatile short int Direct;
    volatile short int Quadrature;
    volatile short int Theta;
    volatile short int Alpha;
    volatile short int Beta;
    volatile short int Arctan;
    volatile short int AxisA;
    volatile short int AxisB;
    volatile short int AxisC;
    volatile short int Sector;
    volatile short int DutyA;
    volatile short int DutyB;
    volatile short int DutyC;
    volatile short int DutyD;
} InvPARK_AND_SVPWM, *InvPARK_AND_SVPWM_handle;
/**
 * @}
 */

/** @defgroup MR Axis PI Control Structure definition
 * @{
 */
typedef struct
{
    volatile short int InMeas;
    volatile short int InRef;
    volatile short int Ui;
    volatile short int Out;
} PI_AxisDQ, *PI_AxisDQ_handle;
/**
 * @}
 */

/** @defgroup MR General PI Control Structure definition
 * @{
 */
typedef struct
{
    volatile short int InMeas;
    volatile short int InRef;
    volatile short int Sum;
    volatile short int Out;
} PI_General, *PI_General_handle;
/**
 * @}
 */

/** @defgroup MR Math Structure definition
 * @{
 */
typedef struct
{
    volatile short int Cos;
    volatile short int Sin;
    volatile short int IniX;
    volatile short int IniY;
    volatile short int Theta;
    volatile unsigned short int SqrtOut;
    volatile unsigned int SqrtIn;
    volatile int AluA;
    volatile int AluB;
    volatile int AluC;
    volatile int AluD;
} MATHS, *MATHS_handle;
/**
 * @}
 */

/** @defgroup MR Structure definition
 * @{
 */
typedef struct
{
    CLARK_AND_PARK I;
    InvPARK_AND_SVPWM V;
    PI_AxisDQ PI_AxisD;
    PI_AxisDQ PI_AxisQ;
    PI_General PI_General;
    MATHS Math;
} MR_VAL, *MR_VAL_handle;
/**
 * @}
 */

/** @defgroup MR Structure definition
 * @{
 */
typedef struct
{
    uint16_t MR_ModulationMode;   
	  uint16_t MR_MulMode;   
    uint16_t MR_PwmPriodLoad;
	  uint16_t MR_SequenceMode;
} MR_InitTypeDef;
/**
 * @}
 */

/**
 * @}
 */
/* End of struct -----------------------------------------------------*/

#define MR_I_PhaseU_AddOffset     0x00
#define MR_I_PhaseV_AddOffset     0x02
#define MR_I_PhaseW_AddOffset     0x04
#define MR_I_Theta_AddOffset      0x06
#define MR_I_Alpha_AddOffset      0x08
#define MR_I_Beta_AddOffset       0x0A
#define MR_I_Arctan_AddOffset     0x0C
#define MR_I_AxisD_AddOffset      0x0E
#define MR_I_AxisQ_AddOffset      0x10
#define MR_I_Filler_AddOffset     0x12
#define MR_V_AxisD_AddOffset      0x14
#define MR_V_AxisQ_AddOffset      0x16
#define MR_V_Theta_AddOffset      0x18
#define MR_V_Alpha_AddOffset      0x1A
#define MR_V_Beta_AddOffset       0x1C
#define MR_V_Arctan_AddOffset     0x1E
#define MR_V_AxisA_AddOffset     0x20
#define MR_V_AxisB_AddOffset     0x22
#define MR_V_AxisC_AddOffset     0x24
#define MR_V_Sector_AddOffset     0x26
#define MR_V_DutyA_AddOffset      0x28
#define MR_V_DutyB_AddOffset      0x2A
#define MR_V_DutyC_AddOffset      0x2C
#define MR_V_DutyD_AddOffset      0x2E
#define MR_PI_AxisD_InMeas_AddOffset 0x30
#define MR_PI_AxisD_InRef_AddOffset  0x32
#define MR_PI_AxisD_Ui_AddOffset     0x34
#define MR_PI_AxisD_Out_AddOffset    0x36
#define MR_PI_AxisQ_InMeas_AddOffset 0x38
#define MR_PI_AxisQ_InRef_AddOffset  0x3A
#define MR_PI_AxisQ_Ui_AddOffset     0x3C
#define MR_PI_AxisQ_Out_AddOffset    0x3E
#define MR_PI_P_InMeas_AddOffset  0x40
#define MR_PI_P_InRef_AddOffset   0x42
#define MR_PI_P_Sum_AddOffset     0x44
#define MR_PI_P_Out_AddOffset     0x46
#define MR_Math_Cos_AddOffset     0x48
#define MR_Math_Sin_AddOffset     0x4A
#define MR_Math_IniX_AddOffset    0x4C
#define MR_Math_IniY_AddOffset    0x4E
#define MR_Math_Theta_AddOffset   0x50
#define MR_Math_SqrtOut_AddOffset 0x52
#define MR_Math_SqrtIn_AddOffset  0x54
#define MR_Math_AluA_AddOffset    0x58
#define MR_Math_AluB_AddOffset    0x5C
#define MR_Math_AluC_AddOffset    0x60
#define MR_Math_AluD_AddOffset    0x64

#define W16 1
#define W32 0
#define HH 24
#define HL 16
#define LH 8
#define LL 0
#define FILLER     0x00
#define SERIES_D_M 0xAD
#define SERIES_D_0 0x5D
#define SERIES_Q_M 0xA9
#define SERIES_Q_0 0x59
#define PARALLEL_M 0x0A
#define PARALLEL_0 0x05
#define MR_INSTR_SIGNED_EXT 0xE1000000
#define MR_INSTR_NEGATIVE   0xD2000000
#define MR_INSTR_SHIFTER    0xC3000000
#define MR_INSTR_ADD        0xB4000000
#define MR_INSTR_SUB        0xA5000000
#define MR_INSTR_MUL        0x96000000
#define MR_INSTR_DIV        0x87000000
#define MR_INSTR_PID        0x78000000
#define MR_INSTR_INVCLARK   0x69000000
#define MR_INSTR_SQRT       0x5A000000
#define MR_INSTR_CORDIC_T   0x4B000000
#define MR_INSTR_CORDIC_R   0x3C000000
#define MR_INSTR_CLARK      0x2D000000
#define MR_INSTR_SVPWM      0x1E000000


extern MR_VAL MR_foc;
extern unsigned int Instr_PARK[3] ;
extern unsigned int Instr_InvPARK_to_SVPWM[4];



/** @addtogroup ADC_Exported_Functions ADC Exported Functions
 * @{
 */

#define MR_SVPWMConfig(ModulationMode) MR->CONFIG = MR->CONFIG & (~0x0C);  MR->CONFIG = MR->CONFIG | (ModulationMode);                      

#define MR_SinCos()               __disable_irq();MR->INSTR  = ( ( MR_INSTR_CORDIC_R ) + ( ( MR_Math_Cos_AddOffset     | W16 ) << HL ) + ( ( MR_Math_IniX_AddOffset        | W16 ) << LH ) + ( MR_Math_Theta_AddOffset     | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_InvPARK()              __disable_irq();MR->INSTR  = ( ( MR_INSTR_CORDIC_R ) + ( ( MR_V_Alpha_AddOffset            ) << HL ) + ( ( MR_V_AxisD_AddOffset                ) << LH ) + ( MR_V_Theta_AddOffset              ) ) ; MR_NOP(); __enable_irq()
#define MR_CLARK()                __disable_irq();MR->INSTR  = ( ( MR_INSTR_CLARK    ) + ( ( MR_I_Alpha_AddOffset      | W16 ) << HL ) + ( ( MR_I_PhaseU_AddOffset         | W16 ) << LH ) + ( MR_I_PhaseW_AddOffset       | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_InvCLARK()             __disable_irq();MR->INSTR  = ( ( MR_INSTR_INVCLARK ) + ( ( MR_V_AxisA_AddOffset     | W16 ) << HL ) + ( ( MR_V_Beta_AddOffset           | W16 ) << LH ) + ( MR_V_Alpha_AddOffset        | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_SVPWM()                __disable_irq();MR->INSTR  = ( ( MR_INSTR_SVPWM    ) + ( ( MR_V_DutyA_AddOffset      | W16 ) << HL ) + ( ( MR_V_AxisA_AddOffset         | W16 ) << LH ) + ( MR_V_AxisC_AddOffset       | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_SPWM()                 __disable_irq();MR->INSTR  = ( ( MR_INSTR_SVPWM    ) + ( ( MR_V_DutyA_AddOffset      | W16 ) << HL ) + ( ( MR_V_Alpha_AddOffset          | W16 ) << LH ) + ( FILLER                      | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_MUL()                  __disable_irq();MR->INSTR  = ( ( MR_INSTR_MUL      ) + ( ( MR_Math_AluC_AddOffset          ) << HL ) + ( ( MR_Math_AluB_AddOffset              ) << LH ) + ( MR_Math_AluA_AddOffset            ) ) ; MR_NOP(); __enable_irq()
#define MR_DIV()                  __disable_irq();MR->INSTR  = ( ( MR_INSTR_DIV      ) + ( ( MR_Math_AluC_AddOffset    | W32 ) << HL ) + ( ( MR_Math_AluA_AddOffset        | W32 ) << LH ) + ( MR_Math_AluB_AddOffset      | W32 ) ) ; MR_NOP(); __enable_irq() /* a/b = c...d */
#define MR_PI_AxisD_M()           __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisD_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisD_InMeas_AddOffset        ) << LH ) + ( SERIES_D_M                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_AxisD_0()           __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisD_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisD_InMeas_AddOffset        ) << LH ) + ( SERIES_D_0                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_AxisQ_M()           __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisQ_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisQ_InMeas_AddOffset        ) << LH ) + ( SERIES_Q_M                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_AxisQ_0()           __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisQ_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisQ_InMeas_AddOffset        ) << LH ) + ( SERIES_Q_0                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_General_M()         __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_P_Sum_AddOffset           ) << HL ) + ( ( MR_PI_P_InMeas_AddOffset            ) << LH ) + ( PARALLEL_M                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_General_0()         __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_P_Sum_AddOffset           ) << HL ) + ( ( MR_PI_P_InMeas_AddOffset            ) << LH ) + ( PARALLEL_0                        ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_AxisD_RESET()       __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisD_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisD_InMeas_AddOffset        ) << LH ) + ( FILLER                            ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_AxisQ_RESET()       __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_AxisQ_Ui_AddOffset        ) << HL ) + ( ( MR_PI_AxisQ_InMeas_AddOffset        ) << LH ) + ( FILLER                            ) ) ; MR_NOP(); __enable_irq()
#define MR_PI_General_RESET()     __disable_irq();MR->INSTR  = ( ( MR_INSTR_PID      ) + ( ( MR_PI_P_Sum_AddOffset           ) << HL ) + ( ( MR_PI_P_InMeas_AddOffset            ) << LH ) + ( FILLER                            ) ) ; MR_NOP(); __enable_irq()
#define MR_ArcTan()               __disable_irq();MR->INSTR  = ( ( MR_INSTR_CORDIC_T ) + ( ( MR_Math_Theta_AddOffset    | W16 ) << HL ) + ( ( MR_Math_Sin_AddOffset     | W16 ) << LH ) + ( MR_Math_Cos_AddOffset          | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_ArcTan_I()             __disable_irq();MR->INSTR  = ( ( MR_INSTR_CORDIC_T ) + ( ( MR_I_Arctan_AddOffset      | W16 ) << HL ) + ( ( MR_I_Beta_AddOffset       | W16 ) << LH ) + ( MR_I_Alpha_AddOffset           | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_ArcTan_V()             __disable_irq();MR->INSTR  = ( ( MR_INSTR_CORDIC_T ) + ( ( MR_V_Arctan_AddOffset      | W16 ) << HL ) + ( ( MR_V_Beta_AddOffset           | W16 ) << LH ) + ( MR_V_Alpha_AddOffset       | W16 ) ) ; MR_NOP(); __enable_irq()
#define MR_SQRT()                 __disable_irq(); MR->INSTR  = ( ( MR_INSTR_SQRT     ) + ( ( MR_Math_SqrtOut_AddOffset  | W16 ) << HL ) + ( ( FILLER                              ) << LH ) + ( MR_Math_SqrtIn_AddOffset        ) ) ; MR_NOP(); __enable_irq()
#define MR_NOP()                  MR->INSTR  = 0x00000000

#define MR_Single()               MR->CONFIG = MR->CONFIG & (~0x01);
#define MR_SeqMode()              MR->CONFIG = MR->CONFIG | ( 0x01);
#define MR_PARK()                 MR->MADDR = (unsigned int)&Instr_PARK[0]             ; MR_NOP()
#define MR_InvPARK_to_SVPWM()     MR->MADDR = (unsigned int)&Instr_InvPARK_to_SVPWM[0] ; MR_NOP()

void MR_DeInit(void);
void MR_Init(MR_InitTypeDef* MR_InitStruct);
/**
 * @}
 */
/* End of exported functions --------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif


