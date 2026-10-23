/* ============================================================================
 * fw_config.h —— 配置总入口 + 自动算出的定点常量
 *
 *   cfg_board.h  驱动板（HAL）     cfg_motor.h  电机本体     cfg_ctrl.h  控制规格
 *
 * 本文件只做换算，正常调参不用改它。所有增益用"截断取整"（与原厂 Q15() 宏
 * 一致）。按 test33 的参数（tools/sim/legacy_cfg.sh）算出的整数与 test33 逐位相同。
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
 * 0xF20E0002  加 MTPA / 弱磁（默认关）与 IPM 有效磁链观测（MOTOR_IPM=1 时）；默认配置行为不变
 * 0xF20E0003  按理论取参：电流环 300 Hz + dq 解耦，速度环 0.5 Hz/ζ0.7，PLL 15 Hz/0.707，
 *             自写防溢出 PI，观测器去直流相位补偿，电流零点 256 点。不再与 test33 逐位相同
 * 0xF20E0004  代码审核后的修复：I/F 开环启动（STARTUP_MODE，可切回原厂）、相位补偿只在
 *             Spin、角度时序补偿、PLL 增益溢出检查、POWER_LOOP 下 MTPA/弱磁限幅、
 *             解耦切换移进快环、反转五段式
 * 0xF20E0005  I/F 定位改成一段（0°，最短 0.8 s，等停稳），IF_ALIGN_STAGES = 2 可切回两段；
 *             停稳门限 5 -> 8 rpm
 * 0xF20E0006  去掉原厂闭源库 SOC_MCI_2R_V102.lib：库里的函数全部自写（mc_math、svpwm、
 *             meas、protect、flystart），行为与 0xF20E0005 逐位相同（仿真 40 组 + 逐函数对比）
 * -------------------------------------------------------------------------*/
#define FW_BUILD_TAG            0xF20E0008UL
#define FLUXPLL_BUILD_TAG       FW_BUILD_TAG

#define FW_PI                   3.14159265358979

/* ---- 通用标幺 ------------------------------------------------------------ */
#define POWER_MAX               (I_MAX * U_MAX)
/* Q15 速度 -> 每拍角度增量（原厂定义） */
#define SPEED_TO_THETA          Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ)

/* ---- 电流环（Pi_Run32：out[Q15 V] = Kp*e>>15 + sum(Ki*e)>>15，e 为 Q15 电流） */
#define CUR_GAIN_SCALE          (I_MAX / UDC_MAX * 32768.0)
#define CURRENT_KP_D            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_LD_H * CUR_GAIN_SCALE))
#define CURRENT_KI_D            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_RS_OHM * CUR_GAIN_SCALE / PWM_FREQ))
#define CURRENT_KP_Q            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_LQ_H * CUR_GAIN_SCALE))
#define CURRENT_KI_Q            ((int32_t)(2 * FW_PI * CURRENT_BW_HZ * MOTOR_RS_OHM * CUR_GAIN_SCALE / PWM_FREQ))

/* ---- 速度环（Pi_Run64，慢环；e 为 Q15 速度，out 为 Q15 电流） --------------- */
#define SPD_GAIN_SCALE          (SPEED_MAX / I_MAX * 32768.0 / MOTOR_ACC_RPM_S_PER_A)
#define SPEED_KP                ((int32_t)(2 * SPEED_ZETA * 2 * FW_PI * SPEED_WN_HZ * SPD_GAIN_SCALE))
#define SPEED_KI                ((int32_t)((2 * FW_PI * SPEED_WN_HZ) * (2 * FW_PI * SPEED_WN_HZ) * SPD_GAIN_SCALE / SLOWLOOP_FREQ))
#define SPEED_ACC_FF_IQ         Q15((SPEED_RAMP_UP / MOTOR_ACC_RPM_S_PER_A) / I_MAX)

#define IBUS_LIMIT_VALUE        Q15(IBUS_LIMIT_A / I_MAX)
#define SPEED_LIMIT_VALUE       Q15(POWER_LIMIT_SPEED_RPM / SPEED_MAX)

/* ---- dq 解耦前馈 --------------------------------------------------------- */
/* 电角速度满量程（Q15 速度 32768 对应）rad/s */
#define W_EMAX_RAD_S            (2 * FW_PI * SPEED_MAX / 60.0 * POLEPAIRS)
/* vq_ff = w*psi_f：Q15 速度 -> Q15 电压 */
#define DEC_K_EMF_Q15           Q15(W_EMAX_RAD_S * MOTOR_PSI_F_WB / UDC_MAX)
/* w*L*i：(Q15 速度 * Q15 电流 >> 15) * K >> 12 -> Q15 电压；K 可能 > 1，用 Q12 */
#define DEC_K_LQ_Q12            ((int32_t)(W_EMAX_RAD_S * MOTOR_LQ_H * I_MAX / UDC_MAX * 4096.0))
#define DEC_K_LD_Q12            ((int32_t)(W_EMAX_RAD_S * MOTOR_LD_H * I_MAX / UDC_MAX * 4096.0))

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

/* ---- MTPA / 弱磁 ---------------------------------------------------------- */
#if MOTOR_IPM
/* a = (Ld - Lq) * I_MAX / psi_f（无量纲）。MTPA 在标幺下：
 *   id_n = (sqrt(1 + (2a*iq_n)^2) - 1) / (2a)
 * 定点：x = 2a*iq_n 用 Q16；id_q15 = (s - 65536) * MTPA_KINV >> 16，MTPA_KINV = 16384/a */
#define MTPA_A                  ((MOTOR_LD_H - MOTOR_LQ_H) * I_MAX / MOTOR_PSI_F_WB)
#define MTPA_K2A_Q16            ((int32_t)(2.0 * MTPA_A * 65536.0))
#define MTPA_KINV               ((int32_t)(16384.0 / MTPA_A))
#endif
/* 弱磁积分增益：fwInteg（id 的 Q31）每慢环拍 += (FW_KI_Q20 * e_q15) >> 4 */
#define FW_KI_Q20               ((int32_t)(FW_KI_A_PER_S / I_MAX / SLOWLOOP_FREQ * 1048576.0))
#define FW_VLIM_Q15             Q15(1.0 - FW_VOLTAGE_MARGIN)
#define FW_ID_MIN_Q15           Q15(FW_ID_MIN_A / I_MAX)

/* 去直流高通的截止角频率 wc = fs / 2^SH（rad/s），换成 Q15 速度单位；补偿起点 */
#define HPF_WC_Q15SPD           ((int32_t)((double)PWM_FREQ / (double)(1L << FLUX_HPF_SH) / W_EMAX_RAD_S * 32768.0 + 0.5))
#define HPF_COMP_W0_Q15SPD      ((int32_t)(FLUX_HPF_COMP_MIN_RPM / SPEED_MAX * 32768.0))

/* ---- 角度时序补偿 --------------------------------------------------------- */
#define INVPARK_ADV_Q8          ((int32_t)(CTRL_INVPARK_ADVANCE * 256.0 + 0.5))

/* ---- I/F 开环启动 ---------------------------------------------------------- */
#define IF_ALIGN_ID_Q15         Q15(IF_ALIGN_A / I_MAX)
#define IF_ALIGN_STEP_Q15       ((int16_t)(IF_ALIGN_RAMP_A_S / I_MAX / SLOWLOOP_FREQ * 32768.0 + 0.5))   /* 每慢环拍 */
/* 定位阻尼增益（A/V）：摆的刚度 K = 1.5*p^2*psi_f*I（N·m/rad，机械角），
 * J = 1.5*p*psi_f*60/(2*pi*b)；B = 2*zeta*sqrt(K*J)；iq = -B*w_m/(1.5*p*psi_f)，
 * 反电势 e_q = p*psi_f*w_m  ->  iq = -[B/(1.5*p^2*psi_f^2)] * e_q。换成 Q15 电流/Q15 电压，Q8 */
#define IF_ALIGN_K_NM           (1.5 * MOTOR_POLE_PAIRS * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB * IF_ALIGN_A)
#define IF_ALIGN_J              (1.5 * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB * 60.0 / (2 * FW_PI * MOTOR_ACC_RPM_S_PER_A))
/* sqrt(K*J) 要开方，在 Startup_Init 里算一次：gain_q8 = IF_ALIGN_C1 * sqrt(K*J) */
#define IF_ALIGN_C1             (2 * IF_ALIGN_ZETA / (1.5 * MOTOR_POLE_PAIRS * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB * MOTOR_PSI_F_WB) \
                                 * UDC_MAX / I_MAX * 256.0)
#define IF_ALIGN1_TICKS         ((uint16_t)(IF_ALIGN1_S * SLOWLOOP_FREQ))
#define IF_ALIGN2_TICKS         ((uint16_t)(IF_ALIGN2_S * SLOWLOOP_FREQ))
/* 第一段定位角 120°（Q15 角度）。必须落在某一相的轴线上（0/60/120°...）：在 30/90/150°
 * 等位置有一相电流正好为 0，死区让这一相的电压来回跳（零电流钳位），正好吞掉 q 轴上的
 * 反电势，阻尼环看不到转子在动（仿真里 90° 时阻尼完全失效）。 */
#define IF_ALIGN1_THETA         21845
#define IF_ALIGN_SWEEP_TICKS    ((uint16_t)(IF_ALIGN_SWEEP_S * SLOWLOOP_FREQ))
#define IF_ALIGN2_MAX_TICKS     ((uint16_t)(IF_ALIGN2_MAX_S * SLOWLOOP_FREQ))
#define IF_ALIGN_STILL_TICKS    ((uint16_t)(IF_ALIGN_STILL_S * SLOWLOOP_FREQ))
/* "静止"门限：反电势 p*psi_f*w_m（Q15 电压） */
#define IF_ALIGN_STILL_EMF_Q15  ((int32_t)(IF_ALIGN_STILL_RPM * 2 * FW_PI / 60.0 * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB / UDC_MAX * 32768.0 + 0.5))
#define IF_IQ_Q15               Q15(IF_CURRENT_A / I_MAX)
#define IF_ACC_STEP_Q31         ((int32_t)(IF_ACC_RPM_S / SPEED_MAX / PWM_FREQ * 2147483648.0))      /* 每快环拍 */
#define IF_HANDOVER_Q31         ((int32_t)(IF_HANDOVER_RPM / SPEED_MAX * 2147483648.0))
#define IF_SPIN_MIN_Q15         Q15(IF_SPIN_MIN_RPM / SPEED_MAX)
#define IF_FALLBACK_Q15         Q15(IF_FALLBACK_RPM / SPEED_MAX)

/* ---- 飞车检测（FLY_PHASOR，flystart.c） ---- */
#define FLY_BLOCK_TICKS         (PWM_FREQ / 1000)           /* 每块 1 ms */
#define FLY_SETTLE_BLOCKS       ((uint16_t)(FLY_SETTLE_S * 1000.0 + 0.5))
#define FLY_STILL_BLOCKS        ((uint16_t)(FLYBRAKE_TIME * 1000.0 + 0.5))
#define FLY_TIMEOUT_BLOCKS      ((uint32_t)(FLY_BRAKE_TIMEOUT_S * 1000.0 + 0.5))
#define FLY_CATCH_Q15           Q15(FLY_CATCH_RPM / SPEED_MAX)
#if (FLY_LOW_MODE == FLY_LOW_IF) && (STARTUP_MODE == STARTUP_IF)
#define FLY_STABLE_MIN_Q15      Q15(FLY_LOW_MIN_RPM / SPEED_MAX)    /* 从这个转速起就要测准（低速同向进 I/F） */
#else
#define FLY_STABLE_MIN_Q15      FLY_CATCH_Q15
#endif
/* 每块转过的电角度（2^32 = 一圈）-> Q15 转速：spd = dphi * K >> 32，
 * K = 60 * 32768 / (Tblock * 极对数 * SPEED_MAX) */
#define FLY_DPHI_K              ((int32_t)(60.0 * 32768.0 * PWM_FREQ / FLY_BLOCK_TICKS / MOTOR_POLE_PAIRS / SPEED_MAX + 0.5))
/* Q15 转速 -> w*Lq/Rs（Q12）：阻抗角 = atan(w*Lq/Rs) */
#define FLY_WLR_Q12             ((int32_t)(SPEED_MAX * 2 * FW_PI / 60.0 * MOTOR_POLE_PAIRS * MOTOR_LQ_H / MOTOR_RS_OHM * 4096.0 + 0.5))
#if (PWM_FREQ % 1000) != 0
#error "FLY_PHASOR 按 1 ms 分块，PWM_FREQ 要是 1 kHz 的整数倍"
#endif
#define IF_WARM_MAX_Q31         ((int32_t)(1.5 * IF_FALLBACK_RPM / SPEED_MAX * 2147483648.0))
#define IF_AGREE_Q15            Q15(IF_AGREE_RATIO)
#define IF_AGREE_TICKS          ((uint16_t)(IF_AGREE_S * SLOWLOOP_FREQ))
#define IF_TIMEOUT_TICKS        ((uint16_t)(IF_TIMEOUT_S * SLOWLOOP_FREQ))

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
