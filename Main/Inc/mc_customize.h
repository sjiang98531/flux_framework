/**
 ***********************************************************************
 * @file        : mc_customize.h
 * @brief       : 用户定制头文件。用户可以利用该文件完成自己工程的定制，
                    通过选取相应的功能以及配置对应的参数可以以最快的速度
                    转动电机。（*）代表该项为必填项；（-）代表该项为选填
                    项；（/）代表母项被勾选时必填。
 * @author      : Hunter
 * @date        : 2025/12/01
 * @copyright   : SOC
 * @version     : ver 0.0.1 
 * @revision    : rev 0.1
 ***********************************************************************	
 */

#ifndef __MC_CUSTOMIZE_H_
#define __MC_CUSTOMIZE_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sc32_conf.h"
#include "basic.h"

// ----------------------------------------- 系统配置 ------------------------------------------ //
#define RUN_CW										(0)				  
#define RUN_CCW										(1)
#define RUN_DIR										RUN_CCW

#define	PWM_DEBUG									0		//Disable PWM test
#define	FAST_BOOT_METHOD              				1		//enable fast boot satrtup method

#define	IBUS_RECONSTRUCTED							1

#define FLY_CHECK_0BEMF                             (0)

/*Hardware setting*/
#define SYSCLK_HSI_72MHz                            72000000.0	//  unit:Hz

#define PWM_FREQ                                   	16000		//  unit:Hz
#define PWM_PERIOD 									((uint16_t)	(SYSCLK_HSI_72MHz/(2*PWM_FREQ)))  // EPWM计数周期
#define DEAD_TIME                                   (1200)			//  unit:ns
#define SLOWLOOP_FREQ                               1000		//  unit:Hz

// ---------------------------------------- 硬件参数配置 --------------------------------------- //
// Current Sampling
#define ADC_REFV                                     5.0
#define R_SHUNT                                     (0.1)     //  unit:ohm
#define CURRENT_OPA_GAIN                            (3.139)     //  OPA_GAIN  
#define I_MAX                                       (ADC_REFV/2/CURRENT_OPA_GAIN/R_SHUNT)  //  unit:A   Max current of Hardware  3.93A        

// Voltage Sampling
#define	UDC_RATE									(310.0)			//  unit:V
#define UDC_MAX                                     (475.0)     //  unit:V      Max DC Voltage of Hardware 
#define U_MAX                                       (UDC_MAX/1.732)  // unit:V      Max Voltage  410V
#define DCBUS_OVER                                  (424.0)     //  unit:V        if DC Bus voltage over this, it will stop motor
#define DCBUS_UNDER                                 (0.0)     //  unit:V        if DC Bus voltage under this, it will stop motor 248V

// ----------------------------------- 环路选择 ---------------------------------- //
#define TORQUE_LOOP									(0)				  /* 电流环 */
#define SPEED_LOOP									(1)				  /* 速度环 */
#define POWER_LOOP									(2)				  /* 功率环 */
#define CLOSE_LOOP									SPEED_LOOP	/* 环路选择 */

#define STOP_TO_RUN_IQ								(0.3)
#define RUN_TO_STOP_IQ								(0.2)

#define STOP_TO_RUN_SPEED                           15.0      	//  unit:RPM     Switch from Stop to Run state
#define RUN_TO_STOP_SPEED                           12.0      	//  unit:RPM     Switch from Stop to Run state

#define POWER_MAX									(I_MAX*U_MAX) //
#define STOP_TO_RUN_POWER                           3.0			//  unit:Watt     Switch from Stop to Run state
#define RUN_TO_STOP_POWER                           2.0   	//  unit:Watt     Switch from Stop to Run state

#define MAX_DUTY                                    (0.84)    //   unit

#if (CLOSE_LOOP == SPEED_LOOP)  					
	#define IBUS_LIMIT_VALUE 						Q15(3.5/I_MAX)			//速度环限Ibus    
#elif (CLOSE_LOOP == POWER_LOOP)   		
	#define SPEED_LIMIT_VALUE 						Q15(3000.0/SPEED_MAX)	//功率环环限Speed 		
#endif

// CURRENT LOOP PI PARAMETER
#define CURRENT_KP									(13747)
#define CURRENT_KI									(274)
#define CURRENT_OUTMAX								(3000) //   (Q15(1.0/I_MAX))

// SPEED LOOP PI PARAMETER
#define SPEED_KP                                 	(8000)			//   KP Value of speed loop (vendor value; the "15000" in the old comment is stale)
/* test22: back to the vendor value 8000 (test20 had 15000). */
#define SPEED_KI                                 	(100)			//   KI Value of speed loop  100
/* test30: speed-ramp acceleration feedforward (not in the vendor code).
 * While the Spin speed ramp is moving AWAY from zero, Iq gets a fixed extra
 * SPEED_ACC_FF_IQ = the current that produces SPEED_RAMP_UP on its own, so the
 * speed PI only has to supply the load current. Deceleration gets no feedforward
 * (vendor behaviour, no extra braking current).
 * SPEED_ACC_PER_A was identified from test29 (293->1172 ramps, 20 ms bins):
 *   d(speed)/dt = 1.247 Q15-speed/s per measured-Iq count = 501 rpm/s per A
 *   (fan drag and friction fitted separately, R^2 = 0.89). The same model with
 *   the vendor PI reproduces the measured 3.2 s ringing, the +110 rpm overshoot
 *   after 293->1172 and the +84 rpm overshoot after a direct start.
 * 200 rpm/s / 501 = 0.40 A. Set SPEED_ACC_FF_ENABLE to 0 to get the vendor loop back. */
#define SPEED_ACC_FF_ENABLE                         (1)
#define SPEED_ACC_PER_A                             (501.0)   	//   unit:rpm/s per A of q current (identified, test29)
#define SPEED_ACC_FF_IQ                             Q15((SPEED_RAMP_UP / SPEED_ACC_PER_A) / I_MAX)
#define SPD_PI_LIMIT                                (4.0)     	//   unit:A	Speed PI output limitation 

#define POWER_KP   									(14000)			//   KP Value of Power loop 24000
#define POWER_KI   									(140)				//   KI Value of Power loop 300

//----------------------------------- 电机启动参数设置 ---------------------------------- //
// PreCharge stage
#define PRECHARGE_TIME                              20        //  unit:ms       precharge time setup

// Align state
#define IQ_ALIGN                                    0.5        //  unit:A        q axis current for align      

#if( FAST_BOOT_METHOD == 1)
	#define ALIGN_TIME                          	0.001      //  unit:s        align time (vendor value)
	/* test22: back to the vendor value 0.001 (test18-21 had 0.5). */
	#define ALIGN_CURRENT_RAMP                      5.0        //  unit:A/s      ramp of  align current
#else
	#define ALIGN_TIME                              1.0        //  unit:s        align time
	#define ALIGN_CURRENT_RAMP                      2.0        //  unit:A/s      ramp of  align current
#endif

// Startup state
#define IQ_STARTUP                                  0.5        //  unit:A        q axis current for startup (vendor value)
/* test22: back to the vendor value 0.5 A (test19-21 had 1.0 A for the I-F drag, now removed). */
#define STARTUP_SPEED_RAMP							60.0      //  unit:RPM/s    ramp of startup speed 			100
#define STARTUP_TIME                                2.0        //  unit:s
#define MAXSTARTUP_SPEED                            100.0      //  unit:RPM      max startup speed 
#define STARTUP_TO_SPIN_SPEED                       80.0      	//  unit:RPM     Switch from Startup to Spin state
#define SPEED_TO_THETA                              Q15(2*SPEED_MAX*POLEPAIRS/60.0/PWM_FREQ) //unit: speed to theta Integral constant

// run state
#define TORQUE_RAMP_UP                            	0.5     //  A/s            	ramp of torque command increase
#define TORQUE_RAMP_DOWN                            0.5     //  A/s            	ramp of torque command decrease
#define SPEED_RAMP_UP                            	200.0     //  RPM/s           ramp of speed command increase
#define SPEED_RAMP_DOWN                            	200.0     //  RPM/s           ramp of speed command decrease
#define POWER_RAMP_UP                            	100.0     	//  Watt/s          ramp of power command increase 正转50
#define POWER_RAMP_DOWN                            	100.0     	//  Watt/s          ramp of power command decrease

// ----------------------------------- 顺逆风参数设置 ---------------------------------- //
// Fly Check state
#define FLYBRAKE_TIME								0.1				//  unit:s           Fly Check time
#define FLYCURRENT_TH								Q15(0.02)	//  unit:A        	Fly Check Current Threshold
#define CALIPWMPERCENT								(0.99)		//  unit:s           TORQUELOOP time 
#define TORQUELOOP_TIME								0.3				//  unit:s           TORQUELOOP time 

//----------------------------------- 保护参数设置 ---------------------------------- //
// freewheel state
#define FREEWHEEL_TIME                              0.0				//  unit:s           freewheel time
#define FAULTRELEASE_TIME                           0.0				//  unit:s           fault release time 

// fault setting
#define BLOCK_TIME                                   2.0			//  unit:s           block detect time 
#define BLOCK_VOLTAGE_THRESHOLD                      U_MAX*0.05		//  unit:v           block detect voltage threshold
#define BLOCK_SPEED_THRESHOLD                        STARTUP_TO_SPIN_SPEED       //  unit:rpm      block detect speed threshold
#define I_PHASSLESS_THRESHOLD                        0.3			//  unit:A	Type:4%*IMAX	Phasless detect current threshold  
#define SPEED_OC_THRESHOLD                           Q15(0.90)	//  unit            SPEED out of control detect threshold  relay to speed max vaule
#define PHASSLESS_TIME                               3.5 			//s 

// ----------------------------------- 电机本体参数设置 ---------------------------------- //
//Motor parameter
#define Rs                                          14.7		//8.2						//  unit:ohm
#define Ls                                          0.046		//0.059				  //  unit:H
#define POLEPAIRS                                   5.0			      			//  unit
#define SPEED_MAX                                   3200.0	  					//  unit:rpm    max speed of motor

// ----------------------------------- 磁链观测器参数 ---------------------------------- //
#define FLUX_V_BASE                                 (UDC_MAX)                        // 电压标幺基准
#define FLUX_WN_HZ                                  100.0                            // 角频率标幺基准 wN=2*pi*100
#define FLUX_PSI_N                                  (FLUX_V_BASE/(2*3.14159265359*FLUX_WN_HZ)) // 磁链基准 ~0.756 Wb
#define FLUX_ISR_BUDGET                             2250                             // TIM3 计数，62.5us@36MHz

//AO 四对极
//#define Rs                                          8.02		//0.08						//  unit:ohm  AO 
//#define Ls                                          0.0336		//0.00011	   	//  unit:H
//#define POLEPAIRS                                   4.0        	//  unit
//#define SPEED_MAX                                   3500.0  		//  unit:rpm    max speed of motor


// Shared PLL parameters (original values)
// track observer
#define M1_TO_KP_GAIN                               Q15(0.675)  		//   KP Value of track observer
#define M1_TO_KP_SHIFT                              (0)         	//   KP Shift Value of track observer
#define M1_TO_KI_GAIN                               Q15(0.9424)  	//   KI Value of track observer
#define M1_TO_KI_SHIFT                              (-6)         	//   KI Shift Value of track observer
#define M1_TO_THETA_GAIN                            Q15(0.5333)			//   Theta gain value
#define M1_TO_THETA_SHIFT                           (-4)         	//   Theta gain shift value

/*FG*/
#define SPEEDUPDATE_FREQ							100			//  unit:Hz
#define FG_NUM										5			//  unit:To be output 12 pulses/1 revolution from FG terminal 


#ifdef __cplusplus
}
#endif

#endif
