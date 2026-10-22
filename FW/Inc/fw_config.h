/* ============================================================================
 * fw_config.h —— 配置总入口 + 自动算出的定点常量
 *
 *   cfg_board.h  驱动板（HAL）     cfg_motor.h  电机本体     cfg_ctrl.h  控制规格
 *
 * 本文件只做换算，正常调参不用改它。所有增益用"截断取整"（与原厂 Q15() 宏
 * 一致），默认参数下得到的整数与 test33 逐位相同（见 tools/sim 的 gain_check）。
 * ==========================================================================*/
#ifndef FW_CONFIG_H
#define FW_CONFIG_H

#include "cfg_board.h"
#include "cfg_motor.h"
#include "cfg_ctrl.h"
#include "mlib_types.h"

/* ---------------------------------------------------------------------------
 * 固件标识：接 J-Scope 读 gFluxDiag.buildTag 确认板上是哪一版。
 * 0xF10E0001 ~ 0xF10E0012 是 flux_selfpll（test1~test33）的历史，见那边的
 * FluxConfig.h。重构后的框架从 0xF20E0001 开始。
 * 0xF20E0001  flux_framework P1+P2：模块化重构，默认行为与 test33（0xF10E0012）逐位相同
 * -------------------------------------------------------------------------*/
#define FW_BUILD_TAG            0xF20E0001UL
#define FLUXPLL_BUILD_TAG       FW_BUILD_TAG

#define FW_PI                   3.14159265358979

/* ---- 通用标幺 ------------------------------------------------------------ */
#define POWER_MAX               (I_MAX * U_MAX)
/* Q15 速度 -> 每拍角度增量（原厂定义） */
#define SPEED_TO_THETA          Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ)

/* ---- 电流环（CalcPI：out[Q15 V] = Kp*e>>15 + sum(Ki*e)>>15，e 为 Q15 电流） */
#define CUR_GAIN_SCALE          (I_MAX / UDC_MAX * 32768.0)
#define CURRENT_KP_D            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_LD_H * CUR_GAIN_SCALE))
#define CURRENT_KI_D            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_RS_OHM * CUR_GAIN_SCALE / PWM_FREQ))
#define CURRENT_KP_Q            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_LQ_H * CUR_GAIN_SCALE))
#define CURRENT_KI_Q            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_RS_OHM * CUR_GAIN_SCALE / PWM_FREQ))

/* ---- 速度环（CalcPI，慢环；e 为 Q15 速度，out 为 Q15 电流） --------------- */
#define SPD_GAIN_SCALE          (SPEED_MAX / I_MAX * 32768.0 / MOTOR_ACC_RPM_S_PER_A)
#define SPEED_KP                ((int32_t)(2 * SPEED_ZETA * 2 * FW_PI * SPEED_WN_HZ * SPD_GAIN_SCALE))
#define SPEED_KI                ((int32_t)((2 * FW_PI * SPEED_WN_HZ) * (2 * FW_PI * SPEED_WN_HZ) * SPD_GAIN_SCALE / SLOWLOOP_FREQ))
#define SPEED_ACC_FF_IQ         Q15((SPEED_RAMP_UP / MOTOR_ACC_RPM_S_PER_A) / I_MAX)

#define IBUS_LIMIT_VALUE        Q15(IBUS_LIMIT_A / I_MAX)
#define SPEED_LIMIT_VALUE       Q15(POWER_LIMIT_SPEED_RPM / SPEED_MAX)

/* ---- 磁链观测器 ---------------------------------------------------------- */
/* psi_f 在观测器 Q15 磁链单位里的值（0.141 Wb -> 6111） */
#define FLUX_NOMINAL_Q15        Q15(MOTOR_PSI_F_WB / FLUX_PSI_N)
/* 以下三个在 FluxObs_Init() 里运行时求值（与 test33 写法相同） */
#define FLUX_KRS_Q15            Q15(MOTOR_RS_OHM * I_MAX / FLUX_V_BASE)
#define FLUX_KLQ_Q15            Q15(MOTOR_LQ_H * I_MAX / FLUX_PSI_N)
#define FLUX_KTS_Q15            Q15(2.0f * 3.14159265f * FLUX_WN_HZ / PWM_FREQ)
#if MOTOR_IPM
/* (Ld - Lq) 在"Q15 电流 -> Q15 磁链"单位下，有符号 Q15 */
#define FLUX_KDL_Q15            Q15((MOTOR_LD_H - MOTOR_LQ_H) * I_MAX / FLUX_PSI_N)
#endif

/* ---- FluxPLL ------------------------------------------------------------- */
/* err = |psi|*sin(dtheta)，鉴相增益 Kd = FLUX_NOMINAL_Q15（LSB/rad）；
 * 1 rad/s 电角速度 = PLL_KS（Q31 速度单位）。
 *   2*zeta*wn = Kd*Kp/Ks，wn^2 = Kd*Ki*fs/Ks */
#define PLL_KS                  (2147483648.0 / (2 * FW_PI * SPEED_MAX * POLEPAIRS / 60.0))
#define FLUXPLL_KP_Q31_PER_ERR  ((int32_t)(2 * FLUXPLL_ZETA * 2 * FW_PI * FLUXPLL_WN_HZ * PLL_KS / FLUX_NOMINAL_Q15))
#define FLUXPLL_KI_Q31_PER_ERR  ((int32_t)((2 * FW_PI * FLUXPLL_WN_HZ) * (2 * FW_PI * FLUXPLL_WN_HZ) * PLL_KS / FLUX_NOMINAL_Q15 / PWM_FREQ))

/* 启动期速度下限（Q31 速度） */
#define STARTUP_PLL_FLOOR_Q31   Q31(STARTUP_PLL_FLOOR_RPM / SPEED_MAX)
/* Startup 上报转速每拍允许的变化量（Q31 速度/拍） */
#define FLUX_STARTUP_ACC_STEP   ((int32_t)(STARTUP_REPORT_ACC_LIMIT / SPEED_MAX / PWM_FREQ * 2147483648.0f))

#endif
