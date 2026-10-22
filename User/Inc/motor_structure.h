/*******************************************************************************
 * 版权所有 (C) 2023, COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD
 *
 * 文件名称: GlobalVarrables.h
 * 内容摘要: 全局变量定义相关头文件
 * 其它说明: 无
 * 当前版本: V1.0
 * 完成日期: 2023年6月28日
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __MOTOR_STRUCTURE_H
#define __MOTOR_STRUCTURE_H

#ifdef __cplusplus
extern "C" {
#endif

/*-------------------- Includes -----------------------*/
#include "includes.h"
#include "mcflib.h"
#include "mlib.h"

typedef struct
{
    __IO int16_t            s16PosElEst_fly;
    __IO int16_t            s16ThetaErr_fly;
    __IO sAngle_Trig        sAngle_fly;
    __IO int32_t            s32BemfAmp2;
    __IO int16_t            s16BemfAmp;
    __IO int16_t            s16BemfAmpTh;

    __IO uint32_t           u32FlyTimeCnt;      /* fly start time cnt */
    __IO uint32_t           u32FlyTimeCmd;      /* fly start time cmd */
    __IO uint32_t           u32FlyBrakeTimeCnt;      /* fly start brake time cnt */
    __IO uint32_t           u32FlyBrakeTimeCmd;      /* fly start brake time cmd */
    __IO int16_t            s16Bemf_th;
  
} FLY_START;

typedef struct
{
     uint16_t           u16BlockTimeCmd;            /* the commond of block time */
     uint16_t           u16BlockVoltageAmp_square ; /* the commond of voltage amplitude square value */
     uint16_t           u16BlockSpeed ;             /* the commond of block speed */
     uint16_t           u16VoltageTemp;             /* the actural voltage amplitude square value */
     uint16_t           u16Speed_OC_cmd ;             /* the commond of speed out of range*/
     uint16_t           u16BlockCounter;            /* block counter */
     uint16_t           u16FspinCounter;            /* fault spin counter*/
     MCFLIB_3_ABC_T_S16      sIabc_Max;                  /* Phaseless fault detection I max value */
     uint32_t           u32Ia_Integral;             /* Phaseless fault detection I Integral value */
     uint32_t           u32Ib_Integral;             /* Phaseless fault detection I Integral value */
     uint32_t           u32Ic_Integral;             /* Phaseless fault detection I Integral value */
     uint32_t           u32PhaselessMinTemp;        /* Phaseless fault detection I Integral Min value */
     uint32_t           u32PhaselessMaxTemp;        /* Phaseless fault detection I Integral Max value */    
     uint16_t           u16PhaselessTimeCnt;        /* Phaseless detection time cnt */
     uint16_t           u16PhaselessTimeCmd;        /* Phaseless detection time cnt cmd*/
     int16_t            s16I_Phaseless_threshold;   /* Phaseless fault detection current threshold value */
     bool_t             bPhaseless;
     bool_t             bSpeed_OC;
     bool_t             bRotor_block;
     bool_t             bErr_Spin;
} ADV_PROTECT;

//-----------------------------------------------------------------------------------------------

typedef struct
{
    MCFLIB_2_DQ_T_S16   sVdq;                 /* dq axis voltage */
    MCFLIB_2_ALBE_T_S16 sVAlBe;               /* alpha beta axis voltage */
    MCFLIB_2_ALBE_T_S16 sVAlBe_cmd;           /* the commond of alpha beta axis voltage */

    MCFLIB_3_ABC_T_S16  sIabc;                /* dq axis voltage */
    MCFLIB_3_ABC_T_S16  sIabc_offset;         /* current offset */

    MCFLIB_2_ALBE_T_S16 sIAlBe;               /* alpha beta axis current */
    MCFLIB_2_DQ_T_S16   sIdq;                 /* dq axis current */
    MCFLIB_2_DQ_T_S16   sIdq_cmd;             /* the commond of dq axis current */
    MCFLIB_2_DQ_T_S16   sIdq_err;             /* the error of dq axis current */

    __IO int16_t    s16CmdTheta;              /* the commond of theta*/
    __IO int16_t    s16ErrTheta;              /* the commond of theta*/
    __IO int16_t    s16ThetaPLL;              /* the pll output theta*/
    __IO int16_t    s16ThetaComp;             /* the output theta compensate value*/
  
    __IO int16_t    s16CmdTheta_Update;       /* the commond of theta update*/
    __IO int16_t    s16ThetaErr;              /* theta error*/
    sAngle_Trig     sAngle_cmd;               /* the commond of angle*/
    sAngle_Trig     sAngle_cmd_update;        /* the commond of angle update*/

    __IO int16_t    s16VMAX;                  /* V max value*/  
	__IO int16_t    s16KVbus;                 /* Vbus slide*/ 
		
    __IO int16_t   	s16Vbus;                  /* DC bus voltage*/
    __IO int16_t   	s16VbusAvg;               /* the average of DC bus voltage*/
	__IO int16_t   	s16Ibusreconstructed;     /* DC bus current reconstructed*/	
		
	__IO int16_t   	s16Ibus;                  /* DC bus current*/
    __IO int16_t   	s16IbusAvg;               /* the average of DC bus current*/
		
	__IO uint16_t   s16BemfU;                  /* BEMF U*/
	__IO uint16_t   s16BemfV;                  /* BEMF V*/
	__IO uint16_t   s16BemfW;                  /* BEMF W*/

    __IO int16_t    s16VqLimit;               /* the limit value of Vq*/

    __IO int16_t    s16SpdCmd;                /* the commond of speed*/
    __IO int16_t    s16SpdRef;                /* the commond of speed*/
    __IO int16_t    s16PreSpdCmd;                /* the commond of speed*/
    __IO int16_t    s16SpdFilt;               /* the filtered value of speed*/
	__IO int16_t    s16CurRamp;            	 	/* the ramp value of torque*/
    __IO int32_t    s32CurRamp;             	/* the ramp value of torque*/
    __IO int16_t    s16SpeedRamp;             /* the ramp value of speed*/
    __IO int32_t    s32SpeedRamp;             /* the ramp value of speed*/
	__IO int16_t    s16PowerRamp;             /* the ramp value of power*/
    __IO int32_t    s32PowerRamp;             /* the ramp value of power*/

    SMOType         FluxFeedback; /* Vendor ABI storage; layout must stay unchanged. */                      
    PLL_TypeDef   	sPLL;
    FLY_START       sFly_Start;
    tPIParm   			CurID;		//ID Current Loop
    tPIParm   			CurIQ;		//IQ Current Loop
    tPIParm   			Speed;		//Speed Loop
	tPIParm  	   		Power;		//Power Loop
	tPIParm   			FWCTL;		//field weakening control

		ADV_PROTECT 		sProtect;
} FOC_TypeDef;
//-----------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------

typedef struct
{
    __IO uint16_t           u16PWMFreq;                 /* PWM frequency*/
    __IO uint16_t           u16DeadTime;                /* dead time */
    __IO int16_t            s16OVPCmd;                  /* the commond of over voltage protect  */
    __IO int16_t            s16UVPCmd;                  /* the commond of under voltage protect */
    __IO int16_t            s16DutyLimitCmd;            /* the commond of VS limit */
    __IO int16_t            s16IqLimitMax;              /* the max value of Iq limit  */

    __IO uint16_t           s16AlignCurAcc;             /* the acceleration value of align current*/
    __IO uint16_t           s16StartupSpdCmdMax;        /* the commond of startup speed max value */
    __IO uint16_t           u16AlignTimeCnt;            /* align time cnt */
    __IO uint16_t           u16StartupTimeCnt;          /* startup time cnt */
    __IO uint16_t           u16TorqueloopTimeCnt;          /* Torqueloop time cnt */

    __IO int16_t            s16AlignIqCmd;              /* the commond of align Iq */
    __IO uint16_t           u16AlignTimeCmd;            /* the commond of align time */
    __IO uint16_t           u16StartupTimeCmd;          /* the commond of startup time */
    __IO uint16_t           u16TorqueloopTimeCmd;		/* the commond of Torqueloop time */
    __IO uint16_t           u16FreeWheelTimeCmd;        /* the commond of freewheel time */
    __IO uint16_t           u16FaultReleaseTimeCmd;     /* the commond of fault release time */
    __IO uint8_t           	u8FaultReleaseCnt;          /* if u8FaultReleaseCnt >= 5 , Motor will not restart,
	while Speed command = 0，u8FaultReleaseCnt will be equal 0，then Speed command > STOP_TO_RUN_SPEED,Motor will restart*/
    __IO uint16_t           u16BlockTimeCmd;            /* the commond of block time */
    __IO uint16_t           u16BlockCounter;            /* block counter */

    __IO int16_t            s16StartupIqCmd;            /* the commond of startup Iq */
    __IO uint16_t           u16SyncRampSpdCmd;          /* the commond of sync ramp speed */
 
    __IO uint16_t           u16PreChargeTimeCmd;        /* the commond of precharge time */

    __IO int16_t            s16VspCmd;                  /* the speed commond of VSP */
    __IO uint16_t           s16VspCmdAvg;               /* the average speed commond of VSP */
	__IO uint16_t           s16VspReception;               /* the average speed commond of VSP */

    __IO uint16_t           u16PWMPeriod;               /* PWM period */
    __IO uint16_t           u16CaliPWMDuty;               /* PWM period */
    __IO uint16_t           u16MaxDutyValue;            /* max duty value */
    __IO uint16_t           u16MinDutyValue;            /* min duty value */

    __IO uint16_t           u16SlowLoopDiv;             /* slow loop div */
    __IO bool_t           	bSlowLoopFlag;              /* slow loop flag */
	
    __IO uint8_t						u8SpeedUpDatePeriod;				/* slow loop div */
    __IO uint32_t						u32FGNum;										/* slow loop div */

    __IO uint16_t           u16Align0Time;              /* align0 time */
    __IO uint16_t           u16Align1Time;              /* align1 time */
    __IO uint16_t           u16Ramp0Time;               /* ramp0 time */
    __IO uint16_t           u16Ramp1Time;               /* ramp1 time */
    __IO uint16_t           u16PreChargeTime;           /* precharge time */
    __IO uint8_t						u8PreChargeCnt;           /* precharge time */
    __IO uint16_t           u16SoftConectTime;          /* soft conect time */
    __IO uint8_t            u8MainState;                /* main state */
    __IO uint16_t						u16SpinToStartCnt;           /* precharge time */

    MLIB_FILTER_MA_T        sSpeed_Filt;
    MLIB_FILTER_MA_T        sVbus_Filt;
		MLIB_FILTER_MA_T        sIbus_Filt;
		MLIB_RAMP_T_Q31         sCurrentRamp;
    MLIB_RAMP_T_Q31         sSpeedRamp;
		MLIB_RAMP_T_Q31         sPowerRamp;
} USER_TypeDef;


typedef struct
{
    int16_t	DetcSpd;
		uint16_t	DetcTime;
} FastBoot_TypeDef;

extern __align(4)u16 ADC_DMABuff[14];

#endif /* __GLOBAL_VARIABLES_H */
/*********** (C) COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD  *************/
/* ------------------------------END OF FILE------------------------------------ */


