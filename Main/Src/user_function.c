/*******************************************************************************
Copyright: SOC32 Microelectronics Co. Ltd.
FileName: user_function.c
Author: 
Date:   2024.08
Description: FOC 
Version: v1.0.1

Function List: 
1.Init_Parameter
2.variable_reset
3.Var_Filt
4.PWM_GEN_init
5.Update_PWM

*******************************************************************************/

/** Define to prevent recursive inclusion */
#define _USER_FUNCTION_C_

/** Files includes */
#include "user_function.h"
#include "motor_structure.h"
#include "mc_customize.h"
#include "pwm_gen.h"
#include "FluxObs.h"
#include "FluxPLL.h"

USER_TypeDef	USER;
FOC_TypeDef		FOC;
FastBoot_TypeDef FastBoot_Para;
int16_t M1FaultID;
int16_t M1FaultID_Record;
uint16_t PeriodTime = 0;

/**
  * @brief  Initializes the Motor control parameters
  * @retval None
  */

void Init_Parameter(void)
{
    /* PWM setting */
    USER.u16PWMFreq										= PWM_FREQ;   /* PWM  Frequence*/
//    USER.u16DeadTime									= SYSCLK_HSI_72MHz / 1000000 * DEAD_TIME;  /* Dead time */

    USER.sSpeed_Filt.u16Sh					= 3;
    USER.sSpeed_Filt.s32Acc					= 0;
    USER.sVbus_Filt.u16Sh					= 3;
    USER.sVbus_Filt.s32Acc					= 0;
	USER.sIbus_Filt.u16Sh					= 3;
    USER.sIbus_Filt.s32Acc					= 0;
	USER.sCurrentRamp.s32RampUp			= Q31(TORQUE_RAMP_UP / I_MAX / SLOWLOOP_FREQ);
    USER.sCurrentRamp.s32RampDown		= Q31(TORQUE_RAMP_DOWN / I_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampUp			= Q31(SPEED_RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampDown			= Q31(SPEED_RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);
	USER.sPowerRamp.s32RampUp			= Q31(POWER_RAMP_UP / POWER_MAX / SLOWLOOP_FREQ);
    USER.sPowerRamp.s32RampDown			= Q31(POWER_RAMP_DOWN / POWER_MAX / SLOWLOOP_FREQ);

    /* Protection  */
    USER.s16OVPCmd								= Q15(DCBUS_OVER / UDC_MAX);
    USER.s16UVPCmd								= Q15(DCBUS_UNDER / UDC_MAX);

    /* Iq Limit  */
    USER.s16IqLimitMax							= Q15(SPD_PI_LIMIT / I_MAX);

    /* Precharge */
    USER.u16PreChargeTimeCmd					= PWM_FREQ / 1000.0 * PRECHARGE_TIME;

    /* Alignment setting */
    USER.s16StartupSpdCmdMax					= Q15(MAXSTARTUP_SPEED / SPEED_MAX);
    USER.s16StartupIqCmd						= Q15(IQ_STARTUP / I_MAX);
    USER.u16StartupTimeCmd						= STARTUP_TIME * SLOWLOOP_FREQ;
    USER.u16TorqueloopTimeCmd					= TORQUELOOP_TIME * SLOWLOOP_FREQ;

	USER.s16AlignIqCmd							= Q15(IQ_ALIGN / I_MAX);
    USER.u16AlignTimeCmd						= ALIGN_TIME * SLOWLOOP_FREQ;
	USER.s16AlignCurAcc              			= Q15(ALIGN_CURRENT_RAMP / I_MAX / SLOWLOOP_FREQ);


    USER.u16FreeWheelTimeCmd					= FREEWHEEL_TIME * SLOWLOOP_FREQ;
    USER.u16FaultReleaseTimeCmd					= FAULTRELEASE_TIME * SLOWLOOP_FREQ;
    USER.u8FaultReleaseCnt						= 0;
    FOC.s16ThetaComp							= 0;

    /* VS Limit */
    USER.s16DutyLimitCmd							= Q15(MAX_DUTY);

    USER.u16PWMPeriod								= SYSCLK_HSI_72MHz / 2 / USER.u16PWMFreq;	
    USER.u16SlowLoopDiv								= PWM_FREQ / SLOWLOOP_FREQ;
    USER.u16CaliPWMDuty								= (USER.u16PWMPeriod*CALIPWMPERCENT);	

    USER.u8SpeedUpDatePeriod						= SLOWLOOP_FREQ / SPEEDUPDATE_FREQ;
    USER.u32FGNum									= (SYSCLK_HSI_72MHz*60*256)/(SPEED_MAX*FG_NUM);		//(72M*60*32768)/(SpdMax*FG_Num*Prescaler)	Prescaler=128
	
	FOC.s16KVbus									= Q15(0.866*UDC_MAX/UDC_RATE/4);
		
		//Speed Loop
    FOC.Speed.qKp									= SPEED_KP;				//Q15
    FOC.Speed.qKi									= SPEED_KI;				//Q15
    FOC.Speed.qdSum									= 0;
    FOC.Speed.qInMeas								= 0;
    FOC.Speed.qOut									= 0;
		
		//Power Loop
    FOC.Power.qKp									= POWER_KP;				//Q15
    FOC.Power.qKi									= POWER_KI;				//Q15
    FOC.Power.qdSum									= 0;
    FOC.Power.qInMeas								= 0;
    FOC.Power.qOut									= 0;
		
		#if (CLOSE_LOOP == SPEED_LOOP)  					/* 转速环 			*/
    FOC.Speed.qOutMax								= USER.s16IqLimitMax;
    FOC.Speed.qOutMin								= -USER.s16IqLimitMax;		//0
		FOC.Power.qOutMax								= 0;
    FOC.Power.qOutMin								= -USER.s16IqLimitMax;
		#elif (CLOSE_LOOP == POWER_LOOP)   			/* 功率环 */
    FOC.Speed.qOutMax								= 0;
    FOC.Speed.qOutMin								= -USER.s16IqLimitMax;	
	FOC.Power.qOutMax								=  USER.s16IqLimitMax;
    FOC.Power.qOutMin								= -USER.s16IqLimitMax;		
		#endif
		
		//Flux Loop
    FOC.CurID.qKp									= CURRENT_KP;				//Q15
    FOC.CurID.qKi									= CURRENT_KI;
    FOC.CurID.qOutMax								= CURRENT_OUTMAX;
    FOC.CurID.qOutMin								= -FOC.CurID.qOutMax;
    FOC.CurID.qdSum									= 0;
    FOC.CurID.qInMeas								= 0;
    FOC.CurID.qOut									= 0;
		
    //Torque Loop
    FOC.CurIQ.qKp									= CURRENT_KP;
    FOC.CurIQ.qKi									= CURRENT_KI;
    FOC.CurIQ.qOutMax								= CURRENT_OUTMAX;
    FOC.CurIQ.qOutMin								= -FOC.CurIQ.qOutMax;
    FOC.CurIQ.qdSum									= 0;
    FOC.CurIQ.qInMeas								= 0;
    FOC.CurIQ.qOut									= 0;
		
    /* Flux feedback storage used by the original library ABI. */
    FOC.FluxFeedback.s16Ealpha_est						= 0;
    FOC.FluxFeedback.s16Ebeta_est						= 0;
    FOC.FluxFeedback.s16Speed								= 0;

    FOC.sFly_Start.s16BemfAmpTh				= FLYCURRENT_TH;
    FOC.sFly_Start.u32FlyBrakeTimeCmd	= PWM_FREQ * FLYBRAKE_TIME;//100ms

    FOC.sProtect.sIabc_Max.s16A			= 0;
    FOC.sProtect.sIabc_Max.s16B			= 0;
    FOC.sProtect.sIabc_Max.s16C			= 0;
    FOC.sProtect.u32Ia_Integral			= 0;
    FOC.sProtect.u32Ib_Integral			= 0;
    FOC.sProtect.u32Ic_Integral			= 0;
    FOC.sProtect.bRotor_block				= 0;
    FOC.sProtect.bErr_Spin					= 0;
    FOC.sProtect.bSpeed_OC					= 0;
    FOC.sProtect.bPhaseless					= 0;
    FOC.sProtect.u16PhaselessTimeCmd	= PHASSLESS_TIME * SLOWLOOP_FREQ;
    FOC.sProtect.u16Speed_OC_cmd			= SPEED_OC_THRESHOLD;
    FOC.sProtect.u16BlockSpeed				= Q15(BLOCK_SPEED_THRESHOLD/SPEED_MAX);
    FOC.sProtect.u16BlockTimeCmd			= BLOCK_TIME * SLOWLOOP_FREQ;
    FOC.sProtect.u16BlockVoltageAmp_square	= MLIB_Mul_Q15(Q15(BLOCK_VOLTAGE_THRESHOLD/U_MAX),Q15(BLOCK_VOLTAGE_THRESHOLD/U_MAX));
    FOC.sProtect.s16I_Phaseless_threshold	= Q15(I_PHASSLESS_THRESHOLD / I_MAX);

	FastBoot_Para.DetcSpd = abs(Q15(STARTUP_TO_SPIN_SPEED/SPEED_MAX));
	FastBoot_Para.DetcTime = 10;
    PWM_GEN_init(&pwm_gen);
	
    /* Self-written flux PLL: gains come from FluxConfig.h and are sized for the
     * constant rotor-flux magnitude, not for SMO's speed-proportional EMF.
     * FluxPLL_Init also clears theta/speed, which variable_reset() already did. */
    FluxPLL_Init(&FOC.sPLL);

    /* shadow flux observer */
    FluxObs_Init();
}

/**
  * @brief  Reset the Motor control parameters
  * @retval None
  */


void variable_reset(void)
{
    FOC.Speed.qdSum										= 0;
	FOC.Power.qdSum										= 0;
    FOC.CurID.qdSum										= 0;
    FOC.CurIQ.qdSum										= 0;
    FOC.sVdq.s16Q                    	= 0;
    FOC.sVdq.s16D                    	= 0;
    FOC.sIdq_cmd.s16Q                	= 0;
    FOC.sIdq_cmd.s16D                	= 0;
    USER.sSpeed_Filt.s32Acc         	= 0;
	USER.sSpeedRamp.s32State         	= 0;
	
    /*Shared observer PLL parameter*/
    FOC.sPLL.s32I_1                 	= 0;
    FOC.sPLL.s32Speed               	= 0;
	
    FOC.FluxFeedback.s16Speed                	= 0;
    FOC.s16SpeedRamp                	= 0;
    FOC.s32SpeedRamp                	= 0;
    FOC.s16SpdFilt                  	= 0;
    USER.u16AlignTimeCnt            	= 0;
    USER.u16PreChargeTime							= 0;
    USER.u8PreChargeCnt								= 0;
	USER.u16StartupTimeCnt						= 0;
	USER.u16TorqueloopTimeCnt					= 0;
	USER.u16SpinToStartCnt						= 0;
    FOC.s16CmdTheta                		= 0;
    FOC.s16ErrTheta                 	= 0;
    FOC.s16ThetaComp                	= 0;
		
    FOC.sProtect.sIabc_Max.s16A				= 0;
    FOC.sProtect.sIabc_Max.s16B				= 0;
    FOC.sProtect.sIabc_Max.s16C				= 0;
    FOC.sProtect.u32Ia_Integral				= 0;
    FOC.sProtect.u32Ib_Integral				= 0;
    FOC.sProtect.u32Ic_Integral				= 0;
    FOC.sProtect.bRotor_block					= 0;
    FOC.sProtect.bErr_Spin						= 0;
    FOC.sProtect.bSpeed_OC						= 0;
    FOC.sProtect.bPhaseless						= 0;
	FOC.sProtect.u16PhaselessTimeCnt	= 0;
}


/*************************************
	函数名：PWM_GEN_init
	描述：初始化SVPWM结构体
	输入：*u1---PWM_GEN_T结构体变量
	输出：无
**************************************/
void PWM_GEN_init(PWM_GEN_T *u1)
{
	u1->Alpha = 0;
	u1->Beta	= 0;
	u1->CompA	= 0;
	u1->CompB	= 0;
	u1->CompC	= 0;
	u1->Mode	= SEVENMODE;
	u1->Sector	= 0;
	u1->N_halfPeriod	= SYSCLK_HSI_72MHz/PWM_FREQ/2;
}

/****************************************************************
	函数名：Update_PWM
	描述：更新占空比
	输入: *u1 --- PWM_GEN_T结构体变量
	输出：无
****************************************************************/
void Update_PWM(PWM_GEN_T *u1)
{
//    PA_BIT(0) = 1;
	#if (RUN_DIR == RUN_CW) 
		EPWM->EPWM_DT[4] = pwm_gen.CompA;
		EPWM->EPWM_DT[2] = pwm_gen.CompB;
	#elif (RUN_DIR == RUN_CCW)   
    	EPWM->EPWM_DT[2] = pwm_gen.CompA;
		EPWM->EPWM_DT[4] = pwm_gen.CompB;
	#endif
		EPWM->EPWM_DT[0] = pwm_gen.CompC;
}

