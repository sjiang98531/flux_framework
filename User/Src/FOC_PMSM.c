/*******************************************************************************
Copyright: SOC32
FileName: FOC_PMSM.c
Author: Hunter
Date:   2024.08
Description: FOC 
Version: v1.0.1

Function List: 
1.FOC_Current_Controller
2.PMSM_Obs

*******************************************************************************/

#define _FOC_PMSM_C_

/** Files includes */

#include "FOC_PMSM.h"
#include "FluxObs.h"
#include "FluxPLL.h"
#include "mc_customize.h"
#include "pwm_gen.h"
#include "StateMachine.h"


/**
  * @brief  Sensorless flux observer and calls
  *         PLL with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle
  * @param  Motor
  * @retval None
  */

void PMSM_Obs(FOC_TypeDef *pFOC)
{
    int16_t s16ThetaErr;
    sAngle_Trig     sAngle_pll;               /* the sin cos of pll angle*/

    /* Flux observer in AB system */
    FluxObs_UpdateFeedback(pFOC);

    sAngle_pll = MLIB_SinCos(pFOC->s16ThetaPLL);

    /* Direct rotor-flux phase detector:
     *   err = psi_beta*cos(theta_hat) - psi_alpha*sin(theta_hat)
     *       = |psi| * sin(theta - theta_hat)
     * The flux vector keeps its orientation whichever way the rotor turns, so
     * unlike the back-EMF formula there is NO sign flip on s16SpdCmd, and no
     * rot90 or omega factor is needed. |psi| is constant, which is what lets
     * FluxPLL use one fixed pair of gains across the whole speed range. */
    s16ThetaErr = MLIB_Mul_Q15(sFluxObs.s16PsiRBe, sAngle_pll.s16Cos)
                - MLIB_Mul_Q15(sFluxObs.s16PsiRAl, sAngle_pll.s16Sin);

    /* Calculate the motor angle  */
    pFOC->s16ThetaPLL = FluxPLL_Run(s16ThetaErr,&pFOC->sPLL);

    /* update the motor angle  */
    pFOC->s16CmdTheta = pFOC->s16ThetaPLL + pFOC->s16ErrTheta;
    /* No SMO phase advance: the flux PLL has no observer lag to compensate. */
    pFOC->s16ThetaComp = 0;
    pFOC->s16CmdTheta = pFOC->s16CmdTheta + pFOC->s16ThetaComp;

    /* PLL calculation */
    pFOC->s16CmdTheta_Update = pFOC->s16CmdTheta + MLIB_Mul_Q15(pFOC->s16SpdFilt, SPEED_TO_THETA);
    pFOC->FluxFeedback.s16Speed = pFOC->sPLL.s32Speed >> 16;
}

/**
  * @brief It executes the core of FOC drive that is the controllers for Iqd
  *        currents regulation. It must be called periodically
  *        when new motor currents have been converted
  * @param Motor.
  * @retval None
  */
void FOC_Current_Controller(FOC_TypeDef *pFOC)
{
	int32_t s32Temp;
	int16_t s16Temp;

	/* Calculate the sincos of  s16CmdTheta */
    pFOC->sAngle_cmd = MLIB_SinCos(pFOC->s16CmdTheta);
	
	/* PARK  transform */
    MCFLIB_Park_S16(&pFOC->sIAlBe, &pFOC->sAngle_cmd, &pFOC->sIdq);
	
	s16Temp = MLIB_Mul_Q15(USER.s16DutyLimitCmd, pFOC->s16VbusAvg);
	pFOC->s16VMAX = MLIB_Mul_Q15(s16Temp, Q15(0.6667)); 		//Amplitude invariant transformation

	#if( FAST_BOOT_METHOD == 1)

		Fast_Boot_Current_Controller(&FOC);
		
	#else
		 
		pFOC->CurID.qOutMax = pFOC->s16VMAX;
		pFOC->CurID.qOutMin = -pFOC->CurID.qOutMax;
		
		pFOC->CurID.qInRef = pFOC->sIdq_cmd.s16D;
		pFOC->CurID.qInMeas = pFOC->sIdq.s16D;
		CalcPI(&pFOC->CurID);
		pFOC->sVdq.s16D = pFOC->CurID.qOut;
	
		/* Calculate the PI saturation for Vq    */	
		s32Temp = pFOC->s16VMAX * pFOC->s16VMAX - pFOC->sVdq.s16D * pFOC->sVdq.s16D;
		pFOC->CurIQ.qOutMax =  Sqrt_Q15(s32Temp);
		pFOC->CurIQ.qOutMin = -pFOC->CurIQ.qOutMax;
		
		pFOC->CurIQ.qInRef = pFOC->sIdq_cmd.s16Q;
		pFOC->CurIQ.qInMeas = pFOC->sIdq.s16Q;
		CalcPI(&pFOC->CurIQ);
		pFOC->sVdq.s16Q = 1000;//pFOC->CurIQ.qOut;
		
	#endif
	
	/* Update the sincos for invpark */
	pFOC->sAngle_cmd_update = pFOC->sAngle_cmd;
	
		/* Inv PARK transform*/
    MCFLIB_InvPark_S16(&pFOC->sVdq, &pFOC->sAngle_cmd_update, &pFOC->sVAlBe);
		
		/* Elimination of the DC-bus voltage ripple  */
    DcBus_Ripple_suppress(&pFOC->sVAlBe, pFOC->s16VbusAvg, &pFOC->sVAlBe_cmd);
}
