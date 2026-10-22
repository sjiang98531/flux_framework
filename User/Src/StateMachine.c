/**
 * @file     STATEMACHINE.c
 * @author   MindMotion Motor Team : Wesson
 * @brief    This file provides the STATEMACHINE functions and test samples.
 *
 * @attention
 *
 * THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
 * CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
 * TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
 * HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
 * CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
 *
 * <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
 */

/** Define to prevent recursive inclusion */
#define _STATEMACHINE_C_
/** Files includes */

#include "StateMachine.h"
#include "mc_customize.h"
#include "FOC_PMSM.h"
#include "FluxObs.h"
#include "FluxPLL.h"
#include "pwm_gen.h"
#include "mlib.h"
#include "hardware_config.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
uint16_t NewStartUpCloseTime = 0;
static int32_t s32StartupSpdRep;	/* test31: rate-limited PLL speed reported during Startup */
Q15_t    s16PosElEst_Flux;
uint16_t hRemainder;
uint16_t u32FGPeriod;
uint16_t u16AbsSpdfilt;
int16_t	 s16PowerFdk;
uint16_t u16LWCntr;
uint16_t NewStartUpPhaselessTime = 0;
uint16_t NewStartUpBlockTime = 0;
uint16_t	ReverseFlag = 0;
uint16_t Bemf_Check_cnt = 0;
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief M1 structure */
m1_app_mainstate_t eM1_MainState = MainState_Init;                 /* Main State */
m1_run_substate_t eM1_RunSubState = RunState_Charge;                             /* Run Sub State */

/*! @brief Application sub-state function field - fast */
const pfcn_void_void s_STATE_FAST[4] = {M1_Fault_Fast, M1_Init_Fast, M1_Stop_Fast, M1_Run_Fast};

/*! @brief Application sub-state function field - slow */
const pfcn_void_void s_STATE_SLOW[4] = {M1_Fault_Slow, M1_Init_Slow, M1_Stop_Slow, M1_Run_Slow};


/*! @brief Application sub-state function field - fast */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_FAST[6] = {M1_RunChargeFast, M1_RunCalibFast,
                                                            M1_RunAlignFast, M1_RunStartupFast,
                                                            M1_RunSpinFast,M1_RunTorqueloopFast
                                                           };

/*! @brief Application sub-state function field - slow */
static const pfcn_void_void s_M1_STATE_RUN_TABLE_SLOW[6] = {M1_RunChargeSlow, M1_RunCalibSlow,
                                                            M1_RunAlignSlow, M1_RunStartupSlow,
                                                            M1_RunSpinSlow,M1_RunTorqueloopSlow
                                                           };

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
void M1_Fault_Fast(void)
{
    DISABLE_PWMOUT();
    M1_FaultDetection();
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Init_Fast(void)
{
    DISABLE_PWMOUT();
    variable_reset();

    /*Switch from Init to Stop state*/
    M1_SwitchInitStop();
}

/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Stop_Fast(void)
{
    DISABLE_PWMOUT();

		EPWM->EPWM_DT[0] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[1] = USER.u16PWMPeriod>>1;
		EPWM->EPWM_DT[2] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[3] = USER.u16PWMPeriod>>1;
		EPWM->EPWM_DT[4] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[5] = USER.u16PWMPeriod>>1;
	
    FOC.s16VbusAvg = FOC.s16Vbus;
    M1_FaultDetection();

    /* If a fault occurred */
    if (M1FaultID)
    {
        /* Switches to the FAULT state */
        M1_SwitchStopFault();
    }
}

/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Run_Fast(void)
{
    /*Run fast state machine*/
    s_M1_STATE_RUN_TABLE_FAST[eM1_RunSubState]();
    M1_FaultDetection();

    /* If a fault occurred */
    if (M1FaultID)
    {
        /* Switches to the FAULT state */
        M1_SwitchRunFault();
    }
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Fault_Slow(void)
{
    static    uint16_t u16FaultTimeCnt;

    Var_Filt();
    Fault_LED_Disp();

    /* If no fault occurred */
    if (!M1FaultID)
    {
        u16FaultTimeCnt++;
        if (u16FaultTimeCnt > USER.u16FaultReleaseTimeCmd)
        {
            /* Switch to STOP state */
            M1_SwitchFaultStop();
            u16FaultTimeCnt = 0;
        }
    }
    /* after fault condition ends wait defined time to clear fault state */
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Init_Slow(void)
{
    Var_Filt();
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Stop_Slow(void)
{
    /* Get the speed command*/
    FOC.s16SpdCmd = USER.s16VspCmd;
	
    #if (CLOSE_LOOP == TORQUE_LOOP)  					/* ������ */
    if ((abs(FOC.s16SpdCmd) > Q15(STOP_TO_RUN_IQ / I_MAX)) && (USER.u8FaultReleaseCnt < 5))
    {
        /* Switch from Stop to Run state*/
        M1_SwitchStopRun();
    }
    else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_IQ / I_MAX))
    {
        USER.u8FaultReleaseCnt = 0;
    }
	#elif (CLOSE_LOOP == SPEED_LOOP)  					/* �ٶȻ� */
    if (abs(FOC.s16SpdCmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX)&&(USER.u8FaultReleaseCnt < 5))
    {
        /* Switch from Stop to Run state*/
        M1_SwitchStopRun();
    }
	else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_SPEED / SPEED_MAX))
	{
		USER.u8FaultReleaseCnt = 0;
	}
	#elif (CLOSE_LOOP == POWER_LOOP)  					/* ���ʻ� */
    if (abs(FOC.s16SpdCmd) >  Q15(STOP_TO_RUN_POWER / POWER_MAX) && (USER.u8FaultReleaseCnt < 5))
    {
        /* Switch from Stop to Run state*/
        M1_SwitchStopRun();
    }	
    else if (abs(FOC.s16SpdCmd) < Q15(STOP_TO_RUN_POWER / POWER_MAX))
    {
        USER.u8FaultReleaseCnt = 0;
    }		
		#endif
    Var_Filt();
}

/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_Run_Slow(void)
{
		static uint8_t u8SpdUpCnt = 0;
	
    /* Get the speed command*/
    FOC.s16SpdCmd = USER.s16VspCmd;
	
		//Calculate actual Power
		s16PowerFdk = 2*MLIB_Mul_Q15(Q15(0.866),MLIB_Mul_Q15(FOC.s16VbusAvg,FOC.s16IbusAvg));
	
		 /* command  Run to Stop */
		#if (CLOSE_LOOP == TORQUE_LOOP)  					/* ������ 			*/
    if (abs(FOC.s16SpdCmd) < Q15(RUN_TO_STOP_IQ / I_MAX))
    {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();                                    
    }
		#elif (CLOSE_LOOP == SPEED_LOOP)  					/* �ٶȻ� 			*/
	
    if (abs(FOC.s16SpdCmd) < abs(Q15(RUN_TO_STOP_SPEED / SPEED_MAX)))
    {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();        
    }
	
		#elif (CLOSE_LOOP == POWER_LOOP)  					/* ���ʻ� 			*/
    if (abs(FOC.s16SpdCmd) < Q15(RUN_TO_STOP_POWER / POWER_MAX))
    {
        DISABLE_PWMOUT();
        M1_SwitchRunStop();        
    }	
	#endif
	else if(++u8SpdUpCnt >= USER.u8SpeedUpDatePeriod)   //For slow loop state machine
	{
		u8SpdUpCnt = 0;
		
//			if(eM1_RunSubState > RunState_Startup)
//			{
//				u16AbsSpdfilt = MLIB_Abs_Q15(FOC.s16SpdFilt);
//				TIM0->TIM_RLD = USER.u32FGNum/u16AbsSpdfilt;	
//				TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,TIM0->TIM_RLD>>1);
//			}
//			else
		{
			TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,0);
		}
	}
    /* Run sub-state function */
    s_M1_STATE_RUN_TABLE_SLOW[eM1_RunSubState]();
}

/*!
 * @brief Switch from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchFaultStop(void)
{
    /* Type the code to do when going from the FAULT to the INIT state */
    eM1_MainState = MainState_Stop;
}


/*!
 * @brief Switch from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchInitStop(void)
{
    /* type the code to do when going from the RUN to the STOP state */
    eM1_MainState = MainState_Stop;

    variable_reset();
}

/*!
 * @brief Transition from Stop to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchStopFault(void)
{
    /* type the code to do when going from the STOP to the FAULT state */
    eM1_MainState = MainState_Fault;
}

/*!
 * @brief Switch from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchStopRun(void)
{
    /* type the code to do when going from the STOP to the RUN state */
    eM1_MainState = MainState_Run;
    eM1_RunSubState = RunState_Charge;
	
	TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,0);
}

/*!
 * @brief Switch from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunFault(void)
{
    /* type the code to do when going from the RUN to the FAULT state */
    DISABLE_PWMOUT();
    eM1_MainState = MainState_Fault;
    eM1_RunSubState = RunState_Charge;
	PWMABC_Dis;	
	PWMABCN_Dis;
    variable_reset();
		
	TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,0);
}

/*!
 * @brief Switch from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunStop(void)
{
	/* type the code to do when going from the RUN to the STOP state */

    eM1_MainState = MainState_Stop;
    eM1_RunSubState = RunState_Charge;

	PWMABC_Dis;	
	PWMABCN_Dis;
    variable_reset();	
	
	TIM_PWMSetDuty(TIM0,TIM_PWMChannel_PWMA,0);
}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunCalibFast(void)
{
		#if	(FLY_CHECK_0BEMF == 1)
	
			MCF_FlyStart_0BEMF(&FOC);
		
			pwm_gen.Mode    = SEVENMODE;
			pwm_gen.Alpha   = FOC.sVAlBe_cmd.s16Alpha;
			pwm_gen.Beta    = FOC.sVAlBe_cmd.s16Beta;
			PWM_GEN_calc(&pwm_gen);

			Update_PWM(&pwm_gen);
		
		#else
			MCF_FlyStart_2R(&FOC);	
		#endif
		FOC.FluxFeedback.s16Speed = FOC.sPLL.s32Speed >> 16;

		if (FOC.sFly_Start.s16BemfAmp > FOC.sFly_Start.s16BemfAmpTh)
		{
			if(Bemf_Check_cnt > 320)
			{
				Bemf_Check_cnt = 0;
				FOC.sFly_Start.u32FlyBrakeTimeCnt = 0;
				M1_SwitchRunCalibTorqueloop();
			}
			else
			{
				Bemf_Check_cnt++;
			}
		}
		
		FOC.sFly_Start.u32FlyBrakeTimeCnt++ ;
		if (FOC.sFly_Start.u32FlyBrakeTimeCnt > FOC.sFly_Start.u32FlyBrakeTimeCmd)
		{
				Bemf_Check_cnt = 0;
				FOC.sFly_Start.u32FlyBrakeTimeCnt = 0;

				M1_SwitchRunCalibAlign();
		}
}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunChargeFast(void)
{	
	PWMABCN_Dis;
	PWMABC_Dis;

	EPWM->EPWM_DT[0] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[1] = USER.u16PWMPeriod>>1;
	EPWM->EPWM_DT[2] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[3] = USER.u16PWMPeriod>>1;
	EPWM->EPWM_DT[4] = USER.u16PWMPeriod>>1;
//	EPWM->EPWM_DT[5] = USER.u16PWMPeriod>>1;
	
	if(++USER.u8PreChargeCnt>2)
		USER.u8PreChargeCnt = 0;

	if(USER.u8PreChargeCnt==0)
	{
		PWMAN_Enb;
	}
	else if(USER.u8PreChargeCnt==1)
	{
		PWMBN_Enb;
	}
	else                                              
	{
		PWMCN_Enb;
	}

    if (USER.u16PreChargeTime++ >= USER.u16PreChargeTimeCmd)
    {		
        /* Switch from Ready to Align state*/
        M1_SwitchChargeCalib();
    }
	
    /* PreCharge Duty 50%*/
//    ENABLE_PWMOUT();
}

/*!
 * @brief Torqueloop state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunTorqueloopFast(void)
{
    /* type the code to do when in the RUN STARTUP sub-state */
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&FOC.sIabc, &FOC.sIAlBe);
	
    PMSM_Obs(&FOC);
	
    FOC_Current_Controller(&FOC);

    pwm_gen.Mode = SEVENMODE;
    pwm_gen.Alpha = FOC.sVAlBe_cmd.s16Alpha;
    pwm_gen.Beta    = FOC.sVAlBe_cmd.s16Beta;
    PWM_GEN_calc(&pwm_gen);

    Update_PWM(&pwm_gen);
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunAlignFast(void)
{
    /* type the code to do when in the RUN ALIGN sub-state */
    MCFLIB_Clark_Sat_S16(&FOC.sIabc, &FOC.sIAlBe);
		
	FOC.s16CmdTheta = 0;

    FOC.s16CmdTheta_Update = FOC.s16CmdTheta;
    FOC_Current_Controller(&FOC);

    pwm_gen.Mode = SEVENMODE;
    pwm_gen.Alpha = FOC.sVAlBe_cmd.s16Alpha;
    pwm_gen.Beta    = FOC.sVAlBe_cmd.s16Beta;
    PWM_GEN_calc(&pwm_gen);

    Update_PWM(&pwm_gen);
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunStartupFast(void)
{
    static    int16_t s16PosElEst;
    static    int16_t s16ThetaErr;
    static    sAngle_Trig     sAngle_temp;

    /* type the code to do when in the RUN STARTUP sub-state */
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&FOC.sIabc, &FOC.sIAlBe);
		
		/*use forced direction for startup */
    if (FOC.s16SpdCmd > 0)
    {
        if (FOC.FluxFeedback.s16Speed < 0)
        {
            FOC.FluxFeedback.s16Speed = 1;
        }
    }
    else
    {
        if (FOC.FluxFeedback.s16Speed > 0)
        {
            FOC.FluxFeedback.s16Speed = -1;
        }
    }
		
    FluxObs_UpdateFeedback(&FOC);
		
	/* test32: phase-detect against the PLL's current angle, not against the
	 * copy of its previous output. s16PosElEst is only refreshed after the PLL
	 * runs, so on entering Startup it still holds the angle from before the
	 * Align->Startup seed (0 after a cold boot) and the first error is full
	 * scale (+|psi| ~ +6111) instead of 0. On every other tick, and on the
	 * Spin->Startup fallback, the two are bit-identical. */
	sAngle_temp = MLIB_SinCos((int16_t)(FOC.sPLL.s32Theta >> 16));

    /* Direct rotor-flux phase detector, identical to the run-time path:
     * err = |psi| * sin(theta - theta_hat). No sign flip, no rot90, no omega. */
    s16ThetaErr = MLIB_Mul_Q15(sFluxObs.s16PsiRBe, sAngle_temp.s16Cos)
                - MLIB_Mul_Q15(sFluxObs.s16PsiRAl, sAngle_temp.s16Sin);
	
	#if( FAST_BOOT_METHOD == 1)

	/* Q31(0.004) is the stock floor: 0.004 * 3200 rpm = 12.8 rpm. Kept verbatim
	 * so the startup ramp behaves exactly like the factory one. */
	s16PosElEst = FluxPLL_RunStartup(s16ThetaErr, &FOC.sPLL,FOC.s16SpdCmd,Q31(0.004));
	s16PosElEst_Flux = s16PosElEst;
	FOC.s16CmdTheta = s16PosElEst_Flux;
	
	#else
		
    s16PosElEst = FluxPLL_Run(s16ThetaErr, &FOC.sPLL);
    s16PosElEst_Flux = s16PosElEst;
		
	/* open loop startup angle calcation*/
    FOC.s16CmdTheta +=  MLIB_Mul_Q15(FOC.s16SpeedRamp, SPEED_TO_THETA);
	FOC.s16ErrTheta = FOC.s16CmdTheta - s16PosElEst_Flux;
	
	#endif
    
    /* no angle compensation*/
    FOC.s16CmdTheta_Update = FOC.s16CmdTheta;
    FOC_Current_Controller(&FOC);

    pwm_gen.Mode = SEVENMODE;
    pwm_gen.Alpha = FOC.sVAlBe_cmd.s16Alpha;
    pwm_gen.Beta    = FOC.sVAlBe_cmd.s16Beta;
    PWM_GEN_calc(&pwm_gen);

    Update_PWM(&pwm_gen);

    /* test31: during Startup the speed REPORTED to the vendor code is the PLL
     * speed with an acceleration limit (FLUX_STARTUP_ACC_LIMIT, FluxConfig.h).
     * The PLL itself (angle, s32Speed) is untouched. Fast_Boot_StartUp() only
     * reads s16SpdFilt, which Var_Filt() builds from this field, and switches to
     * Spin once it is above 80 rpm for 10 ms. When the rotor breaks free the
     * flux PLL catches up a large phase error and its speed overshoots for
     * 25-40 ms (test30: 165-279 rpm in 25 ms = 5000-11000 rpm/s while the rotor
     * was at 54-110 rpm), which was enough to pass that test, switch, collapse
     * and fall back (4/4 starts). 0.5 A can accelerate the rotor by ~250 rpm/s,
     * so a 1000 rpm/s limit never delays a real start (checked against the 17
     * starts of test28_2/test29) but caps a 30 ms spike at +30 rpm. */
    {
        int32_t s32Step = FOC.sPLL.s32Speed - s32StartupSpdRep;
        if (s32Step >  FLUX_STARTUP_ACC_STEP) s32Step =  FLUX_STARTUP_ACC_STEP;
        if (s32Step < -FLUX_STARTUP_ACC_STEP) s32Step = -FLUX_STARTUP_ACC_STEP;
        s32StartupSpdRep += s32Step;
    }
    FOC.FluxFeedback.s16Speed = (int16_t)(s32StartupSpdRep >> 16);

	/*PhaseLess Detection*/
    FOC.sProtect.sIabc_Max.s16A = (FOC.sProtect.sIabc_Max.s16A > FOC.sIabc.s16A) ? FOC.sProtect.sIabc_Max.s16A : FOC.sIabc.s16A;
    FOC.sProtect.sIabc_Max.s16B = (FOC.sProtect.sIabc_Max.s16B > FOC.sIabc.s16B) ? FOC.sProtect.sIabc_Max.s16B : FOC.sIabc.s16B;
    FOC.sProtect.sIabc_Max.s16C = (FOC.sProtect.sIabc_Max.s16C > FOC.sIabc.s16C) ? FOC.sProtect.sIabc_Max.s16C : FOC.sIabc.s16C;
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunSpinFast(void)
{
    /* Type the code to do when in the RUN SPIN sub-state */
    /* Clark transform */
    MCFLIB_Clark_Sat_S16(&FOC.sIabc, &FOC.sIAlBe);

	if(FOC.s16ErrTheta > 9) FOC.s16ErrTheta -= 9;
	else if(FOC.s16ErrTheta < -9) FOC.s16ErrTheta += 9;
	else FOC.s16ErrTheta = 0;
	
    PMSM_Obs(&FOC);
	
    FOC_Current_Controller(&FOC);
		
    FOC.sProtect.u32Ia_Integral += MLIB_Abs_Q15(FOC.sIabc.s16A);
    FOC.sProtect.u32Ib_Integral += MLIB_Abs_Q15(FOC.sIabc.s16B);
    FOC.sProtect.u32Ic_Integral += MLIB_Abs_Q15(FOC.sIabc.s16C);

	if(FOC.s16SpdFilt < Q15(0.2))
	{
		pwm_gen.Mode = SEVENMODE;
	}
	else
	{
		pwm_gen.Mode = FIVEMODE;
	}
		
    pwm_gen.Alpha = FOC.sVAlBe_cmd.s16Alpha;
    pwm_gen.Beta    = FOC.sVAlBe_cmd.s16Beta;
    PWM_GEN_calc(&pwm_gen);

    Update_PWM(&pwm_gen);
}

/*!
 * @brief Calibration process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunCalibSlow(void)
{
    Var_Filt();
}

/*!
 * @brief Ready state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunChargeSlow(void)
{
    Var_Filt();
}

static void M1_RunTorqueloopSlow(void)
{
	//Set torque from function M1_SwitchRunCalibTorqueloop()	
	USER.u16TorqueloopTimeCnt++;
	if (USER.u16TorqueloopTimeCnt > USER.u16TorqueloopTimeCmd)
	{
		M1_SwitchRunTorqueloopSpin();
		USER.u16TorqueloopTimeCnt = 0;
	}
	
	Var_Filt();
}

/*!
 * @brief Alignment process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunAlignSlow(void)
{
    /* Iq command for align */
    #if( FAST_BOOT_METHOD == 1)
		if (FOC.s16SpdCmd > 0)
		{			
			FOC.sIdq_cmd.s16Q	= USER.s16StartupIqCmd>>2;
		}
		else
		{
			FOC.sIdq_cmd.s16Q	= -USER.s16StartupIqCmd>>2;
		}
//			if (FOC.s16SpdCmd > 0)
//			{
//					FOC.sIdq_cmd.s16Q   += USER.s16AlignCurAcc;
//					if (FOC.sIdq_cmd.s16Q > USER.s16AlignIqCmd)
//					{
//							FOC.sIdq_cmd.s16Q = USER.s16AlignIqCmd;
//					}
//			}
//			else if (FOC.s16SpdCmd < 0)
//			{
//					FOC.sIdq_cmd.s16Q     -= USER.s16AlignCurAcc;
//					if (FOC.sIdq_cmd.s16Q < -USER.s16AlignIqCmd)
//					{
//							FOC.sIdq_cmd.s16Q = -USER.s16AlignIqCmd;
//					}
//			}
		#else
			if (FOC.s16SpdCmd > 0)
			{
					FOC.sIdq_cmd.s16Q   += USER.s16AlignCurAcc;
					if (FOC.sIdq_cmd.s16Q > USER.s16AlignIqCmd)
					{
							FOC.sIdq_cmd.s16Q = USER.s16AlignIqCmd;
					}
			}
			else if (FOC.s16SpdCmd < 0)
			{
					FOC.sIdq_cmd.s16Q     -= USER.s16AlignCurAcc;
					if (FOC.sIdq_cmd.s16Q < -USER.s16AlignIqCmd)
					{
							FOC.sIdq_cmd.s16Q = -USER.s16AlignIqCmd;
					}
			}
		#endif

    if (++USER.u16AlignTimeCnt > USER.u16AlignTimeCmd)
    {
        /*Switch from Align to Startup state*/
        M1_SwitchRunAlignStartup();
        USER.u16AlignTimeCnt = 0;
    }
	
    Var_Filt();
    FOC.s16SpdFilt = 0;
}

/*!
 * @brief Start-up process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunStartupSlow(void)
{
		USER.u16StartupTimeCnt++;
		NewStartUpPhaselessTime++;
		NewStartUpBlockTime++;

		/* Iq command for startup */
		if (FOC.s16SpdCmd > 0)
		{
			FOC.sIdq_cmd.s16Q   += USER.s16AlignCurAcc;
			if (FOC.sIdq_cmd.s16Q > USER.s16StartupIqCmd)
			{
				FOC.sIdq_cmd.s16Q = USER.s16StartupIqCmd;
			}
		}
		else if (FOC.s16SpdCmd < 0)
		{
			FOC.sIdq_cmd.s16Q     -= USER.s16AlignCurAcc;
			if (FOC.sIdq_cmd.s16Q < -USER.s16StartupIqCmd)
			{
				FOC.sIdq_cmd.s16Q = -USER.s16StartupIqCmd;
			}
		}

		/* acclelerate for startup speed command */
		FOC.s32SpeedRamp =   MLIB_Ramp_Q31((FOC.s16SpdCmd<<16), &USER.sSpeedRamp);
		
		if (FOC.s32SpeedRamp > (USER.s16StartupSpdCmdMax<<16))
		{
			FOC.s32SpeedRamp = (USER.s16StartupSpdCmdMax<<16);
		}
		else if (FOC.s32SpeedRamp < -(USER.s16StartupSpdCmdMax<<16))
		{
			FOC.s32SpeedRamp = -(USER.s16StartupSpdCmdMax<<16);
		}
		
		FOC.s16SpeedRamp = FOC.s32SpeedRamp>>16;
	
	#if( FAST_BOOT_METHOD == 1)
		NewStartUpBlockTime++;
	
		if(Fast_Boot_StartUp(&FOC) == 1)
		{
			M1_SwitchRunStartupSpin();
			NewStartUpBlockTime = 0;
		}
		else
		{
			if(NewStartUpBlockTime > (USER.u16StartupTimeCmd*3))
			{
				MC_FAULT_SET(M1FaultID, MC_FAULT_ROTOR_BLOCKED);
				M1FaultID_Record |= M1FaultID;
				M1_SwitchRunFault();
				NewStartUpBlockTime = 0;
				
				if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
			}
			
			if(NewStartUpBlockTime > USER.u16StartupTimeCmd)
			{
				if ((FOC.sProtect.sIabc_Max.s16A < FOC.sProtect.s16I_Phaseless_threshold)||\
				(FOC.sProtect.sIabc_Max.s16B < FOC.sProtect.s16I_Phaseless_threshold)||\
				(FOC.sProtect.sIabc_Max.s16C < FOC.sProtect.s16I_Phaseless_threshold))
				{
					MC_FAULT_SET(M1FaultID, MC_FAULT_PHASELESS);
					M1FaultID_Record |= M1FaultID;
					M1_SwitchRunFault();
					NewStartUpBlockTime = 0;
					
					if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
				}	/*PhaseLess Detect*/
			}
		}
		
	#else
//		if(USER.u16StartupTimeCnt > USER.u16StartupTimeCmd)
//		{
//			if ((FOC.sProtect.sIabc_Max.s16A < FOC.sProtect.s16I_Phaseless_threshold)||\
//			(FOC.sProtect.sIabc_Max.s16B < FOC.sProtect.s16I_Phaseless_threshold)||\
//			(FOC.sProtect.sIabc_Max.s16C < FOC.sProtect.s16I_Phaseless_threshold))
//			{
//				MC_FAULT_SET(M1FaultID, MC_FAULT_PHASELESS);
//				M1FaultID_Record |= M1FaultID;
//				M1_SwitchRunFault();
//				USER.u16StartupTimeCnt = 0;
//				
//				if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
//			}	/*PhaseLess Detect*/
//			else if(((FOC.s16SpdFilt >= Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX)) && (FOC.s16SpdCmd > 0)) || \
//			((FOC.s16SpdFilt <= -Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX)) && (FOC.s16SpdCmd < 0)))
//			{
//				M1_SwitchRunStartupSpin();
//				USER.u16StartupTimeCnt = 0;
//			}	/* if speed filter higher than STARTUP_TO_SPIN_SPEED and speed command in the right direction , it will switch startup to spin*/

//			if(USER.u16StartupTimeCnt > (USER.u16StartupTimeCmd*3))
//			{
//				MC_FAULT_SET(M1FaultID, MC_FAULT_ROTOR_BLOCKED);
//				M1FaultID_Record |= M1FaultID;
//				M1_SwitchRunFault();
//				
//				if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
//			}
//		}
	#endif

	Var_Filt();
}


/*!
 * @brief Spin state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_RunSpinSlow(void)
{	
	int32_t s32RampPrev, s32IqCmd;
	int16_t s16AccFF;
	(void)s32RampPrev; (void)s32IqCmd; (void)s16AccFF;
	FOC.s16SpdRef = FOC.s16SpdCmd;
	
	#if (CLOSE_LOOP == TORQUE_LOOP)						/* ������ */
		/* Execute the current ramp*/
		FOC.s32CurRamp = MLIB_Ramp_Q31(FOC.s16SpdRef<<16, &USER.sCurrentRamp);
		FOC.s16CurRamp = FOC.s32CurRamp>>16;

		FOC.sIdq_cmd.s16Q = FOC.s16CurRamp ;
			
	#elif (CLOSE_LOOP == SPEED_LOOP)     				/* �ٶȻ� */
	
		/* Execute the speed ramp*/
		s32RampPrev = FOC.s32SpeedRamp;
		FOC.s32SpeedRamp = MLIB_Ramp_Q31(FOC.s16SpdRef<<16, &USER.sSpeedRamp);
		FOC.s16SpeedRamp = FOC.s32SpeedRamp>>16;
		
		/* test30: acceleration feedforward, see SPEED_ACC_FF_IQ in mc_customize.h.
		 * Only while |ramp| is increasing; deceleration stays vendor. */
		s16AccFF = 0;
		#if (SPEED_ACC_FF_ENABLE)
		if ((FOC.s32SpeedRamp > s32RampPrev) && (FOC.s32SpeedRamp > 0))
			s16AccFF = SPEED_ACC_FF_IQ;
		else if ((FOC.s32SpeedRamp < s32RampPrev) && (FOC.s32SpeedRamp < 0))
			s16AccFF = -SPEED_ACC_FF_IQ;
		#endif
		
		/* Speed PI compute the Iq reference*/	
		FOC.Speed.qInRef = FOC.s16SpeedRamp;
		FOC.Speed.qInMeas = FOC.s16SpdFilt;
		CalcPI(&FOC.Speed);
		
		FOC.Power.qInRef = IBUS_LIMIT_VALUE;			//��λA
		FOC.Power.qInMeas = FOC.s16IbusAvg;			
		CalcPI(&FOC.Power);
		
		s32IqCmd = (int32_t)FOC.Speed.qOut + FOC.Power.qOut + s16AccFF;
		if (s32IqCmd > USER.s16IqLimitMax)  s32IqCmd = USER.s16IqLimitMax;
		if (s32IqCmd < -USER.s16IqLimitMax) s32IqCmd = -USER.s16IqLimitMax;
		FOC.sIdq_cmd.s16Q = (int16_t)s32IqCmd;
	
	#else
	
		/* Execute the Power ramp*/
		FOC.s32PowerRamp = MLIB_Ramp_Q31(FOC.s16SpdRef<<16, &USER.sPowerRamp);
		FOC.s16PowerRamp = FOC.s32PowerRamp>>16;

		/* Speed PI compute the Iq reference*/	
		FOC.Speed.qInRef = SPEED_LIMIT_VALUE;
		FOC.Speed.qInMeas = FOC.s16SpdFilt;
		CalcPI(&FOC.Speed);
		
		/* Speed PI compute the Iq reference*/
		FOC.Power.qInRef = FOC.s16PowerRamp;		
		FOC.Power.qInMeas = s16PowerFdk;	
		CalcPI(&FOC.Power);
		
		FOC.sIdq_cmd.s16Q = FOC.Power.qOut + FOC.Speed.qOut;
	
	#endif

	/*�������л�*/
	if((MLIB_Mul_Q15(FOC.s16PreSpdCmd, FOC.s16SpdCmd)) < 0 )
		USER.u16SpinToStartCnt = PWM_FREQ;
	
	FOC.s16PreSpdCmd = FOC.s16SpdCmd;
	
	#if( FAST_BOOT_METHOD == 1)
	
	if (MLIB_Mul_Q15(FOC.s16SpdCmd, FOC.s16SpdFilt) > 0)		//ͬ��
	{
		if(++NewStartUpCloseTime >= FastBoot_Para.DetcTime)
		{
				NewStartUpCloseTime = 0;
				if(Fast_Boot_Spin(&FOC) == 1)
				{
					M1_SwitchRunSpinStartup();
				}
		}
	}
	else		//����
	{
		if((MLIB_Mul_Q15(FOC.s16SpdCmd, FOC.s16SpdFilt) < 0) && (abs(FOC.s16SpdFilt) < Q15(50.0 / SPEED_MAX)))
		{
				M1_SwitchRunSpinStartup();
		}
	}
	
	#else
	
	if(USER.u16SpinToStartCnt > 0)
	{
		USER.u16SpinToStartCnt--;
		if ((MLIB_Mul_Q15(FOC.s16SpdCmd, FOC.s16SpdFilt) < 0) && (abs(FOC.s16SpdFilt) < Q15(STARTUP_TO_SPIN_SPEED / SPEED_MAX)))
		{
			M1_SwitchRunSpinStartup();
		}
	}
	
	#endif

	
//	/*���ٽǶȲ���*/
//	FOC.s16ThetaComp = MLIB_Mul_Q15(Q15(0.5), FOC.s16SpdFilt);

//	if (FOC.s16ThetaComp < -Q15(0.4))
//	{
//		FOC.s16ThetaComp = -Q15(0.4);
//	}
//	else if (FOC.s16ThetaComp > Q15(0.4))
//	{
//		FOC.s16ThetaComp = Q15(0.4);
//	}
	
    /*���ٽǶȲ���*/
    /* The vendor SMO phase advance (CompTemp*2) is not used: the flux PLL reads
     * psi_r directly and has no observer lag to compensate (test25/26). */
    FOC.s16ThetaComp = 0;
	
	Var_Filt();
		
	MCF_ADV_Protect(&FOC);
	
	if (FOC.sProtect.bErr_Spin)
	{
		FOC.sProtect.bErr_Spin = 0;
		M1_SwitchRunSpinStartup();
	}
	
	if (FOC.sProtect.bRotor_block)
	{
		FOC.sProtect.u16BlockCounter = 0;
		MC_FAULT_SET(M1FaultID, MC_FAULT_ROTOR_BLOCKED);
		M1FaultID_Record |= M1FaultID;
		M1_SwitchRunFault();
			
		if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
	}
	if (FOC.sProtect.bPhaseless)
	{
		MC_FAULT_SET(M1FaultID, MC_FAULT_PHASELESS);
		M1FaultID_Record |= M1FaultID;
		M1_SwitchRunFault();
			
		if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
	}  
	if (FOC.sProtect.bSpeed_OC)
	{
		MC_FAULT_SET(M1FaultID, MC_FAULT_SPEED_OC);
		M1FaultID_Record |= M1FaultID;
		M1_SwitchRunFault();
		
		if(USER.u8FaultReleaseCnt<10) USER.u8FaultReleaseCnt++;
	}
}


/*!
 * @brief Switch from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunCalibAlign(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */
    eM1_RunSubState = RunState_Align;
	
	DISABLE_PWMOUT();
	PWMABC_Enb;
	PWMABCN_Enb;
	
	pwm_gen.Mode = SEVENMODE;
	pwm_gen.Alpha = 0;
	pwm_gen.Beta    = 0;
	PWM_GEN_calc(&pwm_gen);
	Update_PWM(&pwm_gen);
    ENABLE_PWMOUT();
}

/*!
 * @brief Switch from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchChargeCalib(void)
{
	DISABLE_PWMOUT();
	
	#if (FLY_CHECK_0BEMF == 1)
		PWMABC_Enb;
		PWMABCN_Enb;
	#else
		PWMABC_Dis;
		PWMABCN_Enb;
	#endif
	
    pwm_gen.Mode    = SEVENMODE;
    pwm_gen.Alpha   = 0;
    pwm_gen.Beta    = 0;
    PWM_GEN_calc(&pwm_gen);

    Update_PWM(&pwm_gen);
	
    /* PreCharge Duty 50%*/
    ENABLE_PWMOUT();

    /* Type the code to do when going from the RUN kRunState_Ready to the RUN kRunState_Align sub-state */
    variable_reset();

		Bemf_Check_cnt = 0;
    USER.u16PreChargeTime  = 0;
    eM1_RunSubState = RunState_Calib;
}

/*!
 * @brief Switch from Cali to Torqueloop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunCalibTorqueloop(void)
{
	/* TorqueloopSpd */	
	FOC.CurIQ.qdSum = FOC.s16SpdFilt<<14;
	FOC.sIdq_cmd.s16Q = abs(FOC.s16SpdFilt);
	FOC.s16CmdTheta = FOC.sFly_Start.s16PosElEst_fly;
	eM1_RunSubState = RunState_Torqueloop;
	
	DISABLE_PWMOUT();
	PWMABC_Enb;
	PWMABCN_Enb;
	
	pwm_gen.Mode = SEVENMODE;
	pwm_gen.Alpha = 0;
	pwm_gen.Beta    = 0;
	PWM_GEN_calc(&pwm_gen);
	Update_PWM(&pwm_gen);
	ENABLE_PWMOUT();
	
	/* Self-written PLL: one gain pair for the whole speed range, so the vendor
	 * per-substate gain shifts are gone. FluxPLL_Init clears theta as well;
	 * M1_SwitchRunAlignStartup re-seeds it from the aligned command angle. */
	FluxPLL_Init(&FOC.sPLL);
}

/*!
 * @brief Switch from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Startup sub-state */
    FOC.sPLL.s32Theta = FOC.s16CmdTheta << 16;
    FluxObs_BeginStartup(&FOC, &USER.sSpeed_Filt);
    s32StartupSpdRep = 0;	/* test31: BeginStartup also zeroes the PLL speed */
    eM1_RunSubState = RunState_Startup;
    USER.sSpeedRamp.s32RampUp         = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampDown       = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
}


/* test30: feedforward the first Spin tick will apply. The vendor presets the
 * speed-PI integrator with the whole present Iq command; with the acceleration
 * feedforward on top that would count the acceleration current twice (the
 * startup Iq already accelerates the rotor). Spin starts its ramp at s16SpdFilt,
 * so it accelerates exactly when the command is further from zero than filt. */
static int16_t M1_AccFFAtSpinEntry(void)
{
#if (SPEED_ACC_FF_ENABLE) && (CLOSE_LOOP == SPEED_LOOP)
	if ((FOC.s16SpdFilt >= 0) && (FOC.s16SpdCmd > FOC.s16SpdFilt)) return SPEED_ACC_FF_IQ;
	if ((FOC.s16SpdFilt <= 0) && (FOC.s16SpdCmd < FOC.s16SpdFilt)) return -SPEED_ACC_FF_IQ;
#endif
	return 0;
}

/*!
 * @brief Switch from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    USER.sCurrentRamp.s32RampUp		= Q31(TORQUE_RAMP_UP / I_MAX / SLOWLOOP_FREQ);
    USER.sCurrentRamp.s32RampDown	= Q31(TORQUE_RAMP_DOWN / I_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampUp		= Q31(SPEED_RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampDown		= Q31(SPEED_RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sPowerRamp.s32RampUp		= Q31(POWER_RAMP_UP / POWER_MAX / SLOWLOOP_FREQ);
    USER.sPowerRamp.s32RampDown		= Q31(POWER_RAMP_DOWN / POWER_MAX / SLOWLOOP_FREQ);	
    USER.sSpeedRamp.s32State		= FOC.s16SpdFilt << 16;
	USER.sPowerRamp.s32State		= s16PowerFdk << 16;
	USER.sCurrentRamp.s32State		= FOC.sIdq_cmd.s16Q << 16;
	FOC.Speed.qdSum					= ((int32_t)FOC.sIdq_cmd.s16Q - M1_AccFFAtSpinEntry()) * 32768;
    FOC.Power.qdSum					= FOC.sIdq_cmd.s16Q << 15;
	FOC.sProtect.u16PhaselessTimeCnt	= 0;
	FOC.s16PreSpdCmd					= FOC.s16SpdCmd;
    eM1_RunSubState = RunState_Spin;
	
	NewStartUpCloseTime = 0;
}

/*!
 * @brief Switch from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunTorqueloopSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    USER.sCurrentRamp.s32RampUp		= Q31(TORQUE_RAMP_UP / I_MAX / SLOWLOOP_FREQ);
    USER.sCurrentRamp.s32RampDown	= Q31(TORQUE_RAMP_DOWN / I_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampUp			= Q31(SPEED_RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampDown		= Q31(SPEED_RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sPowerRamp.s32RampUp			= Q31(POWER_RAMP_UP / POWER_MAX / SLOWLOOP_FREQ);
    USER.sPowerRamp.s32RampDown		= Q31(POWER_RAMP_DOWN / POWER_MAX / SLOWLOOP_FREQ);	
    USER.sSpeedRamp.s32State			= FOC.s16SpdFilt << 16;
		USER.sPowerRamp.s32State			= s16PowerFdk << 16;
		USER.sCurrentRamp.s32State		= FOC.sIdq_cmd.s16Q << 16;
		FOC.Speed.qdSum								= ((int32_t)FOC.sIdq_cmd.s16Q - M1_AccFFAtSpinEntry()) * 32768;
    FOC.Power.qdSum								= FOC.sIdq_cmd.s16Q << 15;
		FOC.s16ErrTheta								= 0;
		FOC.sProtect.u16PhaselessTimeCnt	= 0;
	
		FOC.s16PreSpdCmd = FOC.s16SpdCmd;	
		
		if((MLIB_Mul_Q15(FOC.s16SpdFilt, FOC.s16SpdCmd)) < 0 )
			USER.u16SpinToStartCnt = 20000;

		/* FluxPLL has a single gain pair and its integrator must survive the
		 * handover, so no gain restoration is needed here. */
    eM1_RunSubState = RunState_Spin;
}


/*!
 * @brief Switch from Spin to startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_SwitchRunSpinStartup(void)
{
    /* Type the code to do when going from the RUN SPIN to the RUN FREEWHEEL sub-state */
    eM1_RunSubState = RunState_Startup;
    /* test31: the rotor is still turning after a Spin->Startup fallback; start
     * the limited report from the speed Spin was reporting, not from 0. */
    s32StartupSpdRep = (int32_t)FOC.FluxFeedback.s16Speed * 65536;
    USER.sSpeedRamp.s32State          = FOC.s16SpdFilt << 16;
    USER.sSpeedRamp.s32RampUp         = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
    USER.sSpeedRamp.s32RampDown       = Q31(STARTUP_SPEED_RAMP / SPEED_MAX / SLOWLOOP_FREQ);
}

/*!
 * @brief Fault detention routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void M1_FaultDetection(void)
{
	static uint16_t u16OvCntr, u16UvCntr;
	/* Clearing actual faults before detecting them again  */
	/* Clear all faults */
	MC_FAULT_CLEAR_ALL(M1FaultID);

	if (FOC.s16VbusAvg > USER.s16OVPCmd)
	{
		if(u16OvCntr <= 1600)  //0.1s
		{
			u16OvCntr++;
		}
		else  
		{
			MC_FAULT_SET(M1FaultID, MC_FAULT_U_DCBUS_OVER);
			DISABLE_PWMOUT();
		}
	}
	else
	{
		u16OvCntr = 0;
	}

	if (FOC.s16VbusAvg < USER.s16UVPCmd)
	{
		if (u16UvCntr <= 1600) //0.1s
		{
			u16UvCntr ++;
		}
		else 
		{
			MC_FAULT_SET(M1FaultID, MC_FAULT_U_DCBUS_UNDER);
			DISABLE_PWMOUT();
		}
	}
	else
	{
		u16UvCntr = 0;
	}
	
		/*  Trigger hardware I_DCBUS overcurrent protection */
//		if(EPWM->STOP_SR & EPWM_STOP_SR_ACMP_TRIG_Msk)		// ������־
//    {
//				EPWM->STOP_SR &= (uint32_t)~EPWM_STOP_SR_STOP_Msk;	//���Ӳ��������׼
//			
//        MC_FAULT_SET(M1FaultID, MC_FAULT_I_DCBUS_OVER);
//        DISABLE_PWMOUT();

//        if (USER.u8FaultReleaseCnt < 10)
//        {
//            USER.u8FaultReleaseCnt++;
//        }
//    }

	M1FaultID_Record |= M1FaultID;
}

/*!
 * @brief Fault Display routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
static void Fault_LED_Disp(void)
{
    static uint16_t u16FaultTimeCnt = 0;
    static uint16_t u16FaultCnt = 0;
    static uint16_t u16FalultNumber = 0;

    if (M1FaultID)
    {
        for (int i = 0; i < 15; i++)
        {
            u16FalultNumber = M1FaultID & (0x01 << i) ? i : u16FalultNumber;
        }
        u16FaultTimeCnt++;
        if (u16FaultTimeCnt == 500) //0.5sec
        {
//            LED1_ON();
        }
        else if (u16FaultTimeCnt == 1000) //0.5sec
        {
            u16FaultCnt++;
//            LED1_OFF();
            if (u16FaultCnt == u16FalultNumber)
            {
                u16FaultTimeCnt = 2000;
            }
            else u16FaultTimeCnt = 0;
        }
        else if (u16FaultTimeCnt == 4000) //2000~4000, 2sec
        {
            u16FaultCnt = 0;
            u16FaultTimeCnt = 0;
        }
    }
    else
    {
        u16FaultCnt = 0;
        u16FaultTimeCnt = 0;
//        LED1_OFF();
    }
}

