/* ============================================================================
 * @file    cfg_derived.h
 * @brief   配置总入口及自动推导的定点常量
 *
 *   cfg_board.h  驱动板（HAL）     cfg_motor.h  电机（必改）     cfg_app.h  应用（按需改）与调试项     cfg_internal.h  内部（无需修改）
 *
 * 本文件只做单位换算与定点化，正常调参无需修改。增益一律截断取整（与 Q15() 宏一致）。
 * ==========================================================================*/
#ifndef CFG_DERIVED_H
#define CFG_DERIVED_H

#include "cfg_board.h"
#include "cfg_motor.h"
#include "cfg_app.h"
#include "cfg_internal.h"
#include "qtypes.h"

/* ---------------------------------------------------------------------------
 * 固件版本标识：可通过调试器读取 gFluxDiag.buildTag 确认板上运行的固件版本。
 * -------------------------------------------------------------------------*/
#define BUILD_TAG            0xF20E0069UL

#define MATH_PI                   3.14159265358979    /* 圆周率（编译期计算用，double） */
/* 编译期 sin（度，0～90°，泰勒到 7 次，误差 < 2e-4） */
#define MATH_RAD_(d)              ((d) * MATH_PI / 180.0)
#define MATH_SIN_DEG(d)           (MATH_RAD_(d) - MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) / 6.0 + \
                                 MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) / 120.0 - \
                                 MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) * MATH_RAD_(d) / 5040.0)

/* 转速反馈滤波窗口：2^n 拍 >= SLOWLOOP_FREQ / (2*pi*SPEED_FB_FILT_HZ)（向上取整，实际拐点 <= SPEED_FB_FILT_HZ） */
#define SPEED_FB_FILT_TICKS     (SLOWLOOP_FREQ / (2.0 * MATH_PI * SPEED_FB_FILT_HZ))
#define SPEED_FB_FILT_SH        ((SPEED_FB_FILT_TICKS) <= 1.0 ? 0 : (SPEED_FB_FILT_TICKS) <= 2.0 ? 1 : \
                                 (SPEED_FB_FILT_TICKS) <= 4.0 ? 2 : (SPEED_FB_FILT_TICKS) <= 8.0 ? 3 : \
                                 (SPEED_FB_FILT_TICKS) <= 16.0 ? 4 : (SPEED_FB_FILT_TICKS) <= 32.0 ? 5 : \
                                 (SPEED_FB_FILT_TICKS) <= 64.0 ? 6 : 7)
#define SPEED_FB_FILT_REAL_HZ   (SLOWLOOP_FREQ / (2.0 * MATH_PI * (double)(1 << SPEED_FB_FILT_SH)))    /* 取整后实际的转速滤波截止频率 Hz */

/* 母线门限排布（cfg_internal.h"回馈减速：母线电压控制"）：测量延迟 Td 内最大回馈能量对应的电压升 dV_d，从过压门限往下每 dV_d 一档 */
/* 时间常数 -> 一阶移位滤波的位数：窗口 2^SH 拍取最接近 tau * f 的 2 的幂（对数意义四舍五入），0~13 */
#define SH_FOR_TICKS(t)      ((t) < 1.414 ? 0 : (t) < 2.828 ? 1 : (t) < 5.657 ? 2 : (t) < 11.31 ? 3 : (t) < 22.63 ? 4 : \
                                 (t) < 45.25 ? 5 : (t) < 90.51 ? 6 : (t) < 181.0 ? 7 : (t) < 362.0 ? 8 : (t) < 724.1 ? 9 : \
                                 (t) < 1448.0 ? 10 : (t) < 2896.0 ? 11 : (t) < 5793.0 ? 12 : 13)
#define MEAS_VBUS_FILT_SH       SH_FOR_TICKS(MEAS_VBUS_FILT_TAU_S * SLOWLOOP_FREQ)    /* 母线电压慢环滤波的移位位数（时间常数 MEAS_VBUS_FILT_TAU_S） */
#define MEAS_IBUS_FILT_SH       SH_FOR_TICKS(MEAS_IBUS_FILT_TAU_S * SLOWLOOP_FREQ)    /* 母线电流慢环滤波的移位位数 */
#define OPENLOOP_PLL_FILT_SH          SH_FOR_TICKS(OPENLOOP_PLL_FILT_TAU_S * SLOWLOOP_FREQ)    /* I/F 期间 PLL 转速滤波（判断能不能切闭环）的移位位数 */
#define ALIGN_EMF_FILT_SH    SH_FOR_TICKS(ALIGN_EMF_FILT_TAU_S * SLOWLOOP_FREQ)    /* 定位判静止用的反电势滤波移位位数 */
/* Spin 里失锁判断要持续的慢环拍数（至少 1） */
#define SPIN_CHECK_TICKS        ((uint16_t)(SPIN_CHECK_S * SLOWLOOP_FREQ + 0.5) < 1 ? 1 : \
                                 (uint16_t)(SPIN_CHECK_S * SLOWLOOP_FREQ + 0.5))
#define MOTOR_KT_NM_A_OF(psi)       (1.5 * MOTOR_POLE_PAIRS * (psi))    /* 转矩常数 N·m/A = 1.5 * 极对数 * psi_f（本电机 1.06） */
#define MOTOR_KT_NM_A               MOTOR_KT_NM_A_OF(MOTOR_PSI_F_WB)
#define BUS_DELAY_S                 ((double)((1 << MEAS_VBUS_FILT_SH) + 1) / SLOWLOOP_FREQ)    /* 母线电压测量延迟 Td s（滤波窗口 + 1 拍）：控制器"看到"电压升高之前已经过去的时间 */
#define BUS_IDLE_MAX_V              (BOARD_VBUS_IN_MAX_V * (1.0 + BOARD_VBUS_TOL))    /* 空载母线可能的最高读数（输入上限 + 采样误差），调节点必须高于它 */
/* 单档最大允许压升：保证三档之下的 BUS_VDC_REF_V 始终高于空载母线上限 BUS_IDLE_MAX_V（留 10% 裕量）。
 * 大功率电机在 SPD_PI_LIMIT 下的回馈压升超出此值时，自动按比例下调回馈方向的 iq 上限 BUS_REGEN_IQ_MAX_A */
#define BUS_DV_MAX_V                (0.9 * (BOARD_DCBUS_OVP_V - BUS_IDLE_MAX_V) / 3.0)
/* 最大回馈功率 W = 最大转矩 * 额定转速（最坏情况的估计）；测量延迟 Td 内最大回馈功率把母线推高的电压 */
#define BUS_REGEN_P_MAX_W_OF(psi)   (MOTOR_KT_NM_A_OF(psi) * SPD_PI_LIMIT * MOTOR_SPEED_RATED_RPM * 2.0 * MATH_PI / 60.0)
#define BUS_REGEN_P_MAX_W           BUS_REGEN_P_MAX_W_OF(MOTOR_PSI_F_WB)
#define BUS_DV_RAW_V_OF(psi)        (BUS_REGEN_P_MAX_W_OF(psi) * BUS_DELAY_S / (BOARD_CBUS_UF * 1e-6 * BOARD_DCBUS_OVP_V))
#define BUS_DV_DELAY_V_OF(psi)      (BUS_DV_RAW_V_OF(psi) < BUS_DV_MAX_V ? BUS_DV_RAW_V_OF(psi) : BUS_DV_MAX_V)    /* 门限之间至少隔这么多 */
#define BUS_DV_DELAY_V              BUS_DV_DELAY_V_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_IQ_MAX_A_OF(psi)  (BUS_DV_RAW_V_OF(psi) > BUS_DV_MAX_V ? SPD_PI_LIMIT * (BUS_DV_MAX_V / BUS_DV_RAW_V_OF(psi)) : SPD_PI_LIMIT)
#define BUS_REGEN_IQ_MAX_A          BUS_REGEN_IQ_MAX_A_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_IQ_MAX_Q15_OF(psi) Q15(BUS_REGEN_IQ_MAX_A_OF(psi) / I_MAX)
#define BUS_REGEN_IQ_MAX_Q15        BUS_REGEN_IQ_MAX_Q15_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_FULL_V_OF(psi)    (BOARD_DCBUS_OVP_V - BUS_DV_DELAY_V_OF(psi))    /* 回馈兜底"全力制动"门限 = 过压保护往下一档 */
#define BUS_REGEN_START_V_OF(psi)   (BUS_REGEN_FULL_V_OF(psi) - BUS_DV_DELAY_V_OF(psi))    /* 回馈兜底开始介入的门限 = 再往下一档 */
#define BUS_VDC_REF_V_OF(psi)       (BUS_REGEN_START_V_OF(psi) - BUS_DV_DELAY_V_OF(psi))    /* 回馈减速时母线电压控制器的调节点 = 再往下一档（本板约 387 V） */
#define BUS_REGEN_FULL_V            BUS_REGEN_FULL_V_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_START_V           BUS_REGEN_START_V_OF(MOTOR_PSI_F_WB)
#define BUS_VDC_REF_V               BUS_VDC_REF_V_OF(MOTOR_PSI_F_WB)
/* 保护（protect.h）：缺相窗口 = PHASELESS_PERIODS 个电周期，慢环每拍累加 |转速 Q15|，满一个电周期的累加值
 * = 32768 * 60 * SLOWLOOP_FREQ / (SPEED_MAX * p)；窗口最长 2 s（低于约 24 rpm 转不满就丢弃，也防 u32 累加溢出） */
#define PHASELESS_ACC_FULL      ((uint32_t)(PHASELESS_PERIODS * 32768.0 * 60.0 * SLOWLOOP_FREQ / (SPEED_MAX * MOTOR_POLE_PAIRS) + 0.5))
#define PHASELESS_N_MAX         ((uint32_t)(2 * PWM_FREQ))    /* 缺相窗口最长的快环拍数（2 s） */
/* 超速：反电势线电压峰值到兜底满 BUS_REGEN_FULL_V 的转速，和机械允许转速取小 */
#define PROTECT_SPEED_UCG_RPM_OF(psi) (BUS_REGEN_FULL_V_OF(psi) / (1.7320508 * (psi)) * 60.0 / (2.0 * MATH_PI * MOTOR_POLE_PAIRS))
#define PROTECT_SPEED_OC_RPM_OF(psi)  (MOTOR_SPEED_LIMIT_RPM < PROTECT_SPEED_UCG_RPM_OF(psi) ? MOTOR_SPEED_LIMIT_RPM : PROTECT_SPEED_UCG_RPM_OF(psi))
#define PROTECT_SPEED_OC_Q15_OF(psi)  ((int32_t)(PROTECT_SPEED_OC_RPM_OF(psi) / SPEED_MAX * 32768.0))
#define PROTECT_SPEED_UCG_RPM   PROTECT_SPEED_UCG_RPM_OF(MOTOR_PSI_F_WB)
#define PROTECT_SPEED_OC_RPM    PROTECT_SPEED_OC_RPM_OF(MOTOR_PSI_F_WB)
#define PROTECT_SPEED_OC_Q15    PROTECT_SPEED_OC_Q15_OF(MOTOR_PSI_F_WB)
/* 兜底的回馈 iq 削顶（speed_ctrl.c）：母线 Q15（UDC_MAX 基准）和斜率 Q12（每 Q15 母线电压超出对应多少 Q15 iq 上限） */
#define BUS_REGEN_START_Q15_OF(psi) ((int16_t)(BUS_REGEN_START_V_OF(psi) / UDC_MAX * 32768.0))
#define BUS_REGEN_FULL_Q15_OF(psi)  ((int16_t)(BUS_REGEN_FULL_V_OF(psi) / UDC_MAX * 32768.0))
#define BUS_REGEN_K_Q12_OF(psi)     ((int32_t)((BUS_REGEN_IQ_MAX_A_OF(psi) / I_MAX * 32768.0) / \
                                     (BUS_REGEN_FULL_Q15_OF(psi) - BUS_REGEN_START_Q15_OF(psi)) * 4096.0 + 0.5))
#define BUS_REGEN_START_Q15     BUS_REGEN_START_Q15_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_FULL_Q15      BUS_REGEN_FULL_Q15_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_K_Q12         BUS_REGEN_K_Q12_OF(MOTOR_PSI_F_WB)
#define BUS_REGEN_BRAKE_Q15     ((int16_t)(BUS_REGEN_BRAKE_A / I_MAX * 32768.0 + 0.5))
/* 母线电压控制器（speed_ctrl.c busVdcLimit）。v = 母线 Q15，e2 = Vref^2 - v^2（Q30），s = |转速| Q15：
 *   允许的斜坡步长（Q31 / 慢环拍）= e2 * BUS_VDC_K_Q8 / s >> 8
 *   K = Kv * C/2 * UDC_MAX^2 / 2^30（J / Q30） * b / Kt（rpm/s per W*s/rad） * 32768*60/(2*pi*SPEED_MAX)（1/w 换成 1/s）
 *       / SPEED_MAX / SLOWLOOP_FREQ * 2^31（rpm/s 换成 Q31 步长） */
#define BUS_VDC_REF_Q15_OF(psi) ((int32_t)(BUS_VDC_REF_V_OF(psi) / UDC_MAX * 32768.0))
#define BUS_VDC_REF_SQ_OF(psi)  (BUS_VDC_REF_Q15_OF(psi) * BUS_VDC_REF_Q15_OF(psi))
#define BUS_VDC_K_Q8_OF(psi, acc) ((int32_t)(2.0 * MATH_PI * BUS_VDC_BW_HZ * 0.5 * BOARD_CBUS_UF * 1e-6 * UDC_MAX * UDC_MAX / 1073741824.0 \
                                 * (acc) / MOTOR_KT_NM_A_OF(psi) * 32768.0 * 60.0 / (2.0 * MATH_PI * SPEED_MAX) \
                                 / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 * 256.0 + 0.5))
#define BUS_VDC_REF_Q15         BUS_VDC_REF_Q15_OF(MOTOR_PSI_F_WB)
#define BUS_VDC_REF_SQ          BUS_VDC_REF_SQ_OF(MOTOR_PSI_F_WB)
#define BUS_VDC_K_Q8            BUS_VDC_K_Q8_OF(MOTOR_PSI_F_WB, MOTOR_ACC_RPM_S_PER_A)
#define BUS_VDC_SMIN_Q15        ((int32_t)(0.01 * 32768.0))     /* 只防除零：转速很低时允许减速度远大于额定，取多少都不影响结果 */
#define BUS_VDC_FILT_Q15        ((int32_t)(32768.0 * 2.0 * MATH_PI * BUS_VDC_BW_HZ / SLOWLOOP_FREQ + 0.5))   /* 制动 id 一阶平滑 */
#define BUS_BRAKE_LAM_K_Q12     ((int32_t)(4096.0 / BUS_BRAKE_FULL_LAMBDA + 0.5))

/* ---- 通用标幺 ------------------------------------------------------------ */
/* Q15 速度 -> 每 PWM 周期的电角度增量 */
#define SPIN_LOST_Q15           ((int16_t)(SPIN_LOST_RPM / SPEED_MAX * 32768.0))
#define PWM_5SEG_Q15            ((int16_t)(PWM_5SEG_RPM / SPEED_MAX * 32768.0))
#define SPEED_TO_THETA          Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ)

/* ---- 电流环（Pi_Run32：out[Q15 V] = Kp*e>>15 + sum(Ki*e)>>15，e 为 Q15 电流） */
#define CUR_GAIN_SCALE          (I_MAX / UDC_MAX * 32768.0)
/* 带 _OF 后缀的宏是"参数 -> 增益"的公式，运行时由 param.c 以辨识参数调用（double 运算，与编译期折叠结果逐位相同）；
 * 不带 _OF 的是代入 cfg_motor.h 标称值的结果，仅用于编译期检查（cfg_check_list.h）和标称值对照。 */
#define CURRENT_KP_OF(L)        ((int32_t)(2 * MATH_PI * CURRENT_BW_HZ * (L) * CUR_GAIN_SCALE))
#define CURRENT_KI_OF(R)        ((int32_t)(2 * MATH_PI * CURRENT_BW_HZ * (R) * CUR_GAIN_SCALE / PWM_FREQ))
#define CURRENT_KP_D            CURRENT_KP_OF(MOTOR_LD_H)
#define CURRENT_KI_D            CURRENT_KI_OF(MOTOR_RS_OHM)
#define CURRENT_KP_Q            CURRENT_KP_OF(MOTOR_LQ_H)
#define CURRENT_KI_Q            CURRENT_KI_OF(MOTOR_RS_OHM)

/* ---- 速度环（Pi_Run64，慢环；e 为 Q15 速度，out 为 Q15 电流） --------------- */
#define SPD_GAIN_SCALE_OF(acc)  (SPEED_MAX / I_MAX * 32768.0 / (acc))
#define SPEED_KP_OF(acc)        ((int32_t)(2 * SPEED_ZETA * 2 * MATH_PI * SPEED_WN_HZ * SPD_GAIN_SCALE_OF(acc)))
#define SPEED_KI_OF(acc)        ((int32_t)((2 * MATH_PI * SPEED_WN_HZ) * (2 * MATH_PI * SPEED_WN_HZ) * SPD_GAIN_SCALE_OF(acc) / SLOWLOOP_FREQ))
#define SPD_GAIN_SCALE          SPD_GAIN_SCALE_OF(MOTOR_ACC_RPM_S_PER_A)
#define SPEED_KP                SPEED_KP_OF(MOTOR_ACC_RPM_S_PER_A)
#define SPEED_KI                SPEED_KI_OF(MOTOR_ACC_RPM_S_PER_A)
/* 斜坡 jerk（Q31 / 慢环拍^2）：SPEED_RAMP_ROUND_S 内加速度从 0 升到 SPEED_RAMP_UP */
#define SPEED_JERK_Q31          ((int32_t)(SPEED_RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 / (SPEED_RAMP_ROUND_S * SLOWLOOP_FREQ) + 0.5))
/* 回退 I/F 后过零的开环加减速度（rpm/s）：OpenLoop_Prepare 按 OPENLOOP_REV_CURRENT_A / OPENLOOP_CURRENT_A 放大 OPENLOOP_ACC_RPM_S */
#define OPENLOOP_ACC_RPM_S_OF(acc)     (OPENLOOP_ACC_FRAC * (acc) * OPENLOOP_CURRENT_A)
#define OPENLOOP_REV_ACC_RPM_S_OF(acc) (OPENLOOP_ACC_RPM_S_OF(acc) * OPENLOOP_REV_CURRENT_A / OPENLOOP_CURRENT_A)
#define OPENLOOP_REV_ACC_RPM_S         OPENLOOP_REV_ACC_RPM_S_OF(MOTOR_ACC_RPM_S_PER_A)
/* 软停机：电流幅值每慢环拍减小多少（Q15），= S 形斜坡的最大转矩变化率（不低于 CURRENT_SLEW_MIN_A_S） */
#define CURRENT_SLEW_JERK_A_S_OF(acc)  (SPEED_RAMP_UP / (acc) / SPEED_RAMP_ROUND_S)
#define CURRENT_SLEW_A_S_OF(acc)       (CURRENT_SLEW_JERK_A_S_OF(acc) > CURRENT_SLEW_MIN_A_S ? CURRENT_SLEW_JERK_A_S_OF(acc) : CURRENT_SLEW_MIN_A_S)
#define CURRENT_SLEW_STEP_Q15_OF(acc)  ((int32_t)(CURRENT_SLEW_A_S_OF(acc) / SLOWLOOP_FREQ / I_MAX * 32768.0 + 0.5))
#define CURRENT_SLEW_STEP_Q15          CURRENT_SLEW_STEP_Q15_OF(MOTOR_ACC_RPM_S_PER_A)
#define SOFT_STOP_STEP_Q15             CURRENT_SLEW_STEP_Q15
/* 斜率前馈：iq_ff(Q15) = 每拍斜坡增量(Q31) * SPEED_FF_K_Q16 >> 16 */
#define SPEED_FF_K_Q16_OF(acc)  ((int32_t)(SPEED_MAX * SLOWLOOP_FREQ / ((acc) * I_MAX) * 32768.0 / 2147483648.0 * 65536.0 + 0.5))
#define SPEED_FF_K_Q16          SPEED_FF_K_Q16_OF(MOTOR_ACC_RPM_S_PER_A)
/* 上式的倒数：每拍斜坡增量(Q31) = iq(Q15) * SPEED_ACC_PER_IQ_Q8 >> 8（母线电压控制器把负载转矩换算为自然减速度） */
#define SPEED_ACC_PER_IQ_Q8_OF(acc)  ((int32_t)((acc) * I_MAX / (SPEED_MAX * SLOWLOOP_FREQ) * 2147483648.0 / 32768.0 * 256.0 + 0.5))
#define SPEED_ACC_PER_IQ_Q8     SPEED_ACC_PER_IQ_Q8_OF(MOTOR_ACC_RPM_S_PER_A)

#define IBUS_LIMIT_VALUE        Q15(IBUS_LIMIT_A / I_MAX)

/* ---- dq 解耦前馈 --------------------------------------------------------- */
/* 电角速度满量程（Q15 速度 32768 对应）rad/s */
#define W_EMAX_RAD_S            (2 * MATH_PI * SPEED_MAX / 60.0 * POLEPAIRS)
/* vq_ff = w*psi_f：Q15 速度 -> Q15 电压 */
#define DECOUPLE_K_EMF_Q15_OF(psi)   Q15(W_EMAX_RAD_S * (psi) / UDC_MAX)
/* w*L*i：(Q15 速度 * Q15 电流 >> 15) * K >> 12 -> Q15 电压；K 可能 > 1，用 Q12 */
#define DECOUPLE_K_L_Q12_OF(L)       ((int32_t)(W_EMAX_RAD_S * (L) * I_MAX / UDC_MAX * 4096.0))
#define DECOUPLE_K_LQ_Q12            DECOUPLE_K_L_Q12_OF(MOTOR_LQ_H)
#define DECOUPLE_K_LD_Q12            DECOUPLE_K_L_Q12_OF(MOTOR_LD_H)

/* ---- 磁链观测器 ---------------------------------------------------------- */
/* 定点基准（自动计算）。仅为定点缩放，不改变观测器动态：
 *   电压基准 FLUX_V_BASE = UDC_MAX（电流环 Q15 电压的满量程）；
 *   磁链基准 FLUX_PSI_N = FLUX_V_BASE / (2*pi*FLUX_WN_HZ)，即满量程电压在电频率 FLUX_WN_HZ 下对应的磁链。
 * 取值方法：观测器积分的是定子磁链，其上界为 psi_f + max(Ld,Lq)*I_MAX（ADC 满量程电流，实际控制电流不会达到），
 *   令其占量程的 FLUX_STATOR_FILL = 0.9 / FLUX_PARAM_MARGIN：
 *     FLUX_WN_HZ = FLUX_V_BASE * FLUX_STATOR_FILL / (2*pi*(psi_f + Lmax*I_MAX))     本电机 93.4 Hz，FLUX_PSI_N ≈ 0.81 Wb
 *   0.9 为量程上限（配置检查 FLUX_STATOR_FITS 与 param.c 运行时检查使用同一数值，留 10% 给积分暂态）。
 *   FLUX_PARAM_MARGIN（B 类）：辨识得到的 psi_f + L*I_MAX 相对配置值最多允许放大的倍数。配置值为估计值，
 *   而定点基准在编译期确定，运行时辨识无法改变它，因此需预留余量。取 1.5。
 *   反向约束：PLL 增益与 psi_f 的 Q15 值成反比，超出 int16 范围时封顶（带宽降低，param.c cfgWarn 第 2 位）。
 *   FILL 越小，psi_f 的 Q15 值越小、越容易封顶：本电机取 1.5 时 Kp = 29939，辨识出的 psi_f 比配置值小 9% 以内不封顶。
 *   积分步长 FLUX_KTS_Q15 = 2*pi*FLUX_WN_HZ/PWM_FREQ 须小于 1，故上限为 0.9*PWM_FREQ/(2*pi)（仅高压、小电感电机会触及，
 *   此时量程无法填满，分辨率不足由配置检查 FLUX_NOMINAL_RESOLUTION 报告）。 */
#define FLUX_V_BASE             (UDC_MAX)
#define FLUX_PARAM_MARGIN       1.5
#define FLUX_STATOR_FILL        (0.9 / FLUX_PARAM_MARGIN)
#define FLUX_STATOR_MAX_WB_OF(psi, ld, lq) ((psi) + ((ld) > (lq) ? (ld) : (lq)) * I_MAX)
#define FLUX_STATOR_MAX_WB      FLUX_STATOR_MAX_WB_OF(MOTOR_PSI_F_WB, MOTOR_LD_H, MOTOR_LQ_H)
#define FLUX_WN_FIT_HZ_OF(psi, ld, lq)     (FLUX_V_BASE * FLUX_STATOR_FILL / (2 * MATH_PI * FLUX_STATOR_MAX_WB_OF(psi, ld, lq)))
#define FLUX_WN_FIT_HZ          FLUX_WN_FIT_HZ_OF(MOTOR_PSI_F_WB, MOTOR_LD_H, MOTOR_LQ_H)
#define FLUX_WN_MAX_HZ          (0.9 * PWM_FREQ / (2 * MATH_PI))
#define FLUX_WN_HZ_OF(psi, ld, lq)         (FLUX_WN_FIT_HZ_OF(psi, ld, lq) < FLUX_WN_MAX_HZ ? FLUX_WN_FIT_HZ_OF(psi, ld, lq) : FLUX_WN_MAX_HZ)
#define FLUX_WN_HZ              FLUX_WN_HZ_OF(MOTOR_PSI_F_WB, MOTOR_LD_H, MOTOR_LQ_H)
#define FLUX_PSI_N_OF(psi, ld, lq)         (FLUX_V_BASE / (2 * MATH_PI * FLUX_WN_HZ_OF(psi, ld, lq)))
#define FLUX_PSI_N              FLUX_PSI_N_OF(MOTOR_PSI_F_WB, MOTOR_LD_H, MOTOR_LQ_H)
/* psi_f 在观测器 Q15 磁链单位里的值（0.141 Wb -> 6111） */
#define FLUX_NOMINAL_Q15_AT(psi, psiN)     Q15((psi) / (psiN))
#define FLUX_NOMINAL_Q15_OF(psi)           FLUX_NOMINAL_Q15_AT(psi, FLUX_PSI_N)
#define FLUX_NOMINAL_Q15        FLUX_NOMINAL_Q15_OF(MOTOR_PSI_F_WB)
#define FLUX_KRS_Q15_OF(R)      Q15((R) * I_MAX / FLUX_V_BASE)
/* 电流环电压单位（UDC_MAX 基准）下的 Rs：Q15 电流 -> Q15 电压（定位阻尼从 q 轴电压里扣电阻压降用） */
#define FOC_KRS_Q15_OF(R)       Q15((R) * I_MAX / UDC_MAX)
#define FLUX_KL_Q15_AT(L, psiN) Q15((L) * I_MAX / (psiN))
#define FLUX_KL_Q15_OF(L)       FLUX_KL_Q15_AT(L, FLUX_PSI_N)
/* IPD（cfg_internal.h）：门限 Q8；注入电压 V = 2 * I * L * PWM_FREQ（每拍电流变化 2I，三角波幅值 I），按 Lq 计算（param.c）；
 * 时长换算为快环拍 / 慢环拍；偏置电流 Q15 及每慢环拍的斜坡步长 */
#define IPD_MIN_RATIO_Q8        ((uint16_t)(IPD_MIN_RATIO * 256.0 + 0.5))
#define IPD_MIN_SAL_Q8          ((int32_t)(IPD_MIN_SAL * 256.0 + 0.5))
#define IPD_HF_V_OF(L)          (2.0 * IPD_HF_A * (L) * PWM_FREQ)
#define IPD_HF_V                IPD_HF_V_OF(MOTOR_LQ_H)
#define IPD_HF_TICKS            ((uint16_t)(IPD_HF_S * PWM_FREQ + 0.5))
#define IPD_SETTLE_TICKS        4           /* 每个注入方向开头不计的拍数 */
#define IPD_MEAS_MS             ((uint16_t)(IPD_HF_S * SLOWLOOP_FREQ + 0.5))
#define IPD_SETTLE_MS           ((uint16_t)(0.004 * SLOWLOOP_FREQ + 0.5))     /* 偏置到位后等电流环稳定（约 L/R） */
#define IPD_BIAS_Q15            ((int32_t)(IPD_BIAS_A / I_MAX * 32768.0 + 0.5))
#define IPD_BIAS_STEP_Q15       ((int32_t)(IPD_BIAS_A / I_MAX * 32768.0 / (IPD_RAMP_S * SLOWLOOP_FREQ) + 0.5))
/* 逆变器电压误差（cfg_board.h）：每相误差 = bus * KT * clamp(i * K_I / 256, ±32767) / 32768 / 32768
 *   KT  = INVERTER_DT_EFF_S * PWM_FREQ（Q15，母线的比例），K_I = 32768 / I0_Q15（Q8） */
#define FLUX_DT_KT_Q15_OF(td)   (INVERTER_DT_COMP ? Q15((td) * PWM_FREQ) : 0)
#define FLUX_DT_KI_Q8           ((int32_t)(INVERTER_DT_COMP ? 256.0 * I_MAX / INVERTER_DT_I0_A + 0.5 : 0))
#define FLUX_DT_I0_Q15          ((int32_t)(INVERTER_DT_I0_A / I_MAX * 32768.0 + 0.5))    /* 过渡带宽度（Q15），dtPhase 先按它限幅 */
/* 电感饱和（cfg_motor.h）：拐点（Q15 电流）、斜率（Q12，每 Q15 满量程电流）、下限（Q15） */
#define FLUX_LS_KNEE_Q15_OF(k)  Q15((k) / I_MAX)
#define FLUX_LS_SAT_K_Q12_OF(p) ((int32_t)((p) * I_MAX * 4096.0 + 0.5))
#define FLUX_LS_MAX_Q15_OF(a, k) (((a) > 0.0 && (a) - (k) < I_MAX) ? ((a) > (k) ? Q15(((a) - (k)) / I_MAX) : 0) : 32767)
#define FLUX_LS_MIN_Q15         Q15(MOTOR_LS_SAT_MIN)
#define FLUX_KTS_Q15_OF(psi, ld, lq) Q15(2.0f * 3.14159265f * FLUX_WN_HZ_OF(psi, ld, lq) / PWM_FREQ)
#define FLUX_KTS_Q15            FLUX_KTS_Q15_OF(MOTOR_PSI_F_WB, MOTOR_LD_H, MOTOR_LQ_H)
/* (Ld - Lq) 在"Q15 电流 -> Q15 磁链"单位下，有符号 Q15 */
#define FLUX_KDL_Q15_AT(ld, lq, psiN) Q15(((ld) - (lq)) * I_MAX / (psiN))
#define FLUX_KDL_Q15_OF(ld, lq) FLUX_KDL_Q15_AT(ld, lq, FLUX_PSI_N)

/* ---- 弱磁 ----------------------------------------------------------------- */
/* 弱磁积分增益：fwInteg（id 的 Q31）每慢环拍 += (WEAKEN_KI_Q20 * e_q15) >> 4 */
#define WEAKEN_KI_Q20               ((int32_t)(WEAKEN_KI_A_PER_S / I_MAX / SLOWLOOP_FREQ * 1048576.0))
#define WEAKEN_VLIM_Q15             Q15(1.0 - WEAKEN_VOLTAGE_MARGIN)    /* 弱磁开始介入的电压利用率（留 WEAKEN_VOLTAGE_MARGIN 的余量） */
#define WEAKEN_ID_MIN_Q15           Q15(WEAKEN_ID_MIN_A / I_MAX)    /* 弱磁 id 的下限（最负） */

/* 去直流高通的截止角频率 wc = fs / 2^SH（rad/s），换成 Q15 速度单位；补偿起点 */
/* 去直流窗口 2^SH 拍：最接近 FLUX_HPF_TAU_S * PWM_FREQ 的 2 的幂（对数意义上四舍五入） */
#define FLUX_HPF_TICKS_         (FLUX_HPF_TAU_S * PWM_FREQ)    /* 去直流时间常数换成快环拍数 */
#define FLUX_HPF_SH             ((FLUX_HPF_TICKS_) < 1448.0 ? 10 : (FLUX_HPF_TICKS_) < 2896.0 ? 11 : (FLUX_HPF_TICKS_) < 5793.0 ? 12 : \
                                 (FLUX_HPF_TICKS_) < 11585.0 ? 13 : 14)
/* 去直流截止角频率，用 Q15 转速表示（相位补偿 atan(wc/w) 里用） */
#define HPF_WC_Q15SPD           ((int32_t)((double)PWM_FREQ / (double)(1L << FLUX_HPF_SH) / W_EMAX_RAD_S * 32768.0 + 0.5))
/* NLFO（flux_obs.c）：corr = psi * fq >> SH，fq = ((ferr >> 12) * FLUX_NLFO_NORM_Q8) >> 8。
 * fq = ferr / |psi_f|^2（Q15）时径向误差每拍衰减 2 / 2^SH，即 tau = 2^(SH-1) / fs。SH 取 2 * tau * fs 最接近的 2 的幂，
 * 余下的比例 2^(SH-1) / (tau * fs) 乘进归一化，tau 可以连续取值 */
#define FLUX_NLFO_TICKS_        (2.0 * FLUX_NLFO_TAU_S * PWM_FREQ)    /* 2 * NLFO 时间常数换成快环拍数 */
#define FLUX_NLFO_NORM_Q8_OF(n) ((int32_t)(32768.0 * 4096.0 * 256.0 / ((double)(n) * (double)(n)) \
                                 * (double)(1L << FLUX_NLFO_SH) / FLUX_NLFO_TICKS_ + 0.5))
#define FLUX_NLFO_NORM_Q8       FLUX_NLFO_NORM_Q8_OF(FLUX_NOMINAL_Q15)
/* 归一化系数上限：快环把 ferr>>12 限在 ±ceil(2^23/norm)（超出部分 fq 本就饱和），乘积 <= 2^23 + norm 不溢出 int32 */
#define FLUX_NLFO_NORM_MAX      (1L << 22)
/* NLFO 修正的移位位数（取最接近的 2 的幂，余数乘进归一化） */
#define FLUX_NLFO_SH            ((FLUX_NLFO_TICKS_) < 181.0 ? 7 : (FLUX_NLFO_TICKS_) < 362.0 ? 8 : (FLUX_NLFO_TICKS_) < 724.0 ? 9 : \
                                 (FLUX_NLFO_TICKS_) < 1448.0 ? 10 : 11)
#define HPF_COMP_W0_Q15SPD      ((int32_t)(FLUX_HPF_COMP_MIN_RPM / SPEED_MAX * 32768.0))    /* 低于这个转速（Q15）不做去直流相位补偿 */

/* ---- 角度时序补偿 --------------------------------------------------------- */
#define INVPARK_ADV_Q8          ((int32_t)(CTRL_INVPARK_ADVANCE * 256.0 + 0.5))

/* ---- I/F 开环启动 ---------------------------------------------------------- */
#define ALIGN_ID_Q15         Q15(ALIGN_A / I_MAX)
#define ALIGN_STEP_Q15       ((int16_t)(ALIGN_RAMP_A_S / I_MAX / SLOWLOOP_FREQ * 32768.0 + 0.5))   /* 每慢环拍 */
/* 定位阻尼增益（A/V）：摆的刚度 K = 1.5*p^2*psi_f*I（N·m/rad，机械角），
 * J = 1.5*p*psi_f*60/(2*pi*b)；B = 2*zeta*sqrt(K*J)；iq = -B*w_m/(1.5*p*psi_f)，
 * 反电势 e_q = p*psi_f*w_m  ->  iq = -[B/(1.5*p^2*psi_f^2)] * e_q。换成 Q15 电流/Q15 电压，Q8 */
#define ALIGN_K_NM_OF(psi)          (1.5 * MOTOR_POLE_PAIRS * MOTOR_POLE_PAIRS * (psi) * ALIGN_A)
#define ALIGN_J_OF(psi, acc)        (1.5 * MOTOR_POLE_PAIRS * (psi) * 60.0 / (2 * MATH_PI * (acc)))
#define ALIGN_C1_OF(psi)            (2 * ALIGN_ZETA / (1.5 * MOTOR_POLE_PAIRS * MOTOR_POLE_PAIRS * (psi) * (psi)) \
                                     * UDC_MAX / I_MAX * 256.0)
#define ALIGN_T2_S2_OF(psi, acc)    (4.0 * MATH_PI * MATH_PI * ALIGN_J_OF(psi, acc) / ALIGN_K_NM_OF(psi))
#define ALIGN_T2_TICKS2_OF(psi, acc) ((uint64_t)(ALIGN_T2_S2_OF(psi, acc) * SLOWLOOP_FREQ * SLOWLOOP_FREQ + 0.5))
#define ALIGN_K_NM           ALIGN_K_NM_OF(MOTOR_PSI_F_WB)
#define ALIGN_J              ALIGN_J_OF(MOTOR_PSI_F_WB, MOTOR_ACC_RPM_S_PER_A)
/* sqrt(K*J) 要开方，在 param.c computeGain() 里算一次：gain_q8 = ALIGN_C1 * sqrt(K*J) */
#define ALIGN_C1             ALIGN_C1_OF(MOTOR_PSI_F_WB)
/* 定位摆周期的平方 T^2 = 4*pi^2*J/K（s^2），用于配置检查（慢环频率须远高于摆动频率） */
#define ALIGN_T2_S2          ALIGN_T2_S2_OF(MOTOR_PSI_F_WB, MOTOR_ACC_RPM_S_PER_A)
/* 定位摆周期的平方换算为慢环拍数的平方（param.c computeGain() 开方得到 T 的拍数，各段为 T 的倍数） */
#define ALIGN_T2_TICKS2      ALIGN_T2_TICKS2_OF(MOTOR_PSI_F_WB, MOTOR_ACC_RPM_S_PER_A)
/* 冷起动开环角超前转子的平衡负载角 δ_eq = asin(OPENLOOP_ACC_FRAC)（泰勒级数到 7 次，x <= 0.5 时误差 < 3e-4 rad），Q31 角度（2^32 = 一圈） */
#define OPENLOOP_LEAD_RAD    (OPENLOOP_ACC_FRAC + OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC / 6.0 \
                              + 3.0 / 40.0 * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC \
                              + 5.0 / 112.0 * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC \
                                * OPENLOOP_ACC_FRAC * OPENLOOP_ACC_FRAC)
#define OPENLOOP_LEAD_Q31    ((int32_t)(OPENLOOP_LEAD_RAD / (2.0 * MATH_PI) * 4294967296.0 + 0.5))
/* 摆动中起动：扫掠角（Q15 角度，65536 = 一圈）及其与定位电流的比值（Q16：定位角 = (ALIGN_ID_Q15 - id) * 本值 >> 16） */
#define ALIGN_SWING_SWEEP_Q15    ((int32_t)(ALIGN_SWING_SWEEP_DEG / 360.0 * 65536.0 + 0.5))
#define ALIGN_SWING_SWEEP_K_Q15  ((int32_t)((double)ALIGN_SWING_SWEEP_Q15 / ALIGN_ID_Q15 * 32768.0 + 0.5))   /* Q15：ALIGN_ID_Q15 * K <= 2^30，任意 I_MAX 下不溢出 */
/* 摆动中起动：扫掠结束后反电势滤波的建立时间（慢环拍），之后开始检测 */
#define ALIGN_SWING_SETTLE_TICKS ((uint16_t)(4.0 * ALIGN_EMF_FILT_TAU_S * SLOWLOOP_FREQ + 0.5))
/* 摆动中起动：反电势测量的延迟 = 滤波时间常数 + 电流 PI 积分项跟随扰动的时间常数（L/R + 1/电流环带宽）；
 * 转速 Q15 在该延迟内转过的电角度（Q15 角度），Q12 */
#define ALIGN_SWING_LAG_S_OF(L, R)      (ALIGN_EMF_FILT_TAU_S + (L) / (R) + 1.0 / (2.0 * MATH_PI * CURRENT_BW_HZ))
#define ALIGN_SWING_LAG_ANG_Q12_OF(L, R) ((int32_t)(SPEED_MAX / 60.0 * MOTOR_POLE_PAIRS * ALIGN_SWING_LAG_S_OF(L, R) * 2.0 * 4096.0 + 0.5))
#define ALIGN_SWING_LAG_S        ALIGN_SWING_LAG_S_OF(MOTOR_LQ_H, MOTOR_RS_OHM)
#define ALIGN_SWING_LAG_ANG_Q12  ALIGN_SWING_LAG_ANG_Q12_OF(MOTOR_LQ_H, MOTOR_RS_OHM)
/* 可接入的最小峰值转速 = ALIGN_SWING_MIN_DELTA * 平衡负载角 * 摆动角频率（电角速度）；此处为 Q15 转速 * 摆动周期（慢环拍），
 * param.c computeGain() 中除以摆动周期的拍数 */
#define ALIGN_SWING_MIN_SPD_T    (ALIGN_SWING_MIN_DELTA * OPENLOOP_LEAD_RAD * SLOWLOOP_FREQ * 60.0 / MOTOR_POLE_PAIRS / SPEED_MAX * 32768.0)
/* "静止"门限：反电势 p*psi_f*w_m（Q15 电压） */
#define ALIGN_STILL_EMF_Q15_OF(psi) ((int32_t)(ALIGN_STILL_RPM * 2 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS * (psi) / UDC_MAX * 32768.0 + 0.5))
#define ALIGN_STILL_EMF_Q15  ALIGN_STILL_EMF_Q15_OF(MOTOR_PSI_F_WB)
#define OPENLOOP_IQ_Q15               Q15(OPENLOOP_CURRENT_A / I_MAX)
#define OPENLOOP_REV_IQ_Q15           Q15(OPENLOOP_REV_CURRENT_A / I_MAX)
#define OPENLOOP_WARM_IMAX_Q15        Q15(OPENLOOP_WARM_MAX_A / I_MAX)
#define OPENLOOP_WARM_KI_Q15          ((int32_t)(32768.0 / MATH_SIN_DEG(OPENLOOP_WARM_DELTA_DEG) + 0.5))      /* 1/sin(δmax)，Q15 */
#define OPENLOOP_ACC_STEP_Q31_OF(acc) ((int32_t)(OPENLOOP_ACC_RPM_S_OF(acc) / SPEED_MAX / PWM_FREQ * 2147483648.0))      /* 每快环拍 */
#define OPENLOOP_ACC_STEP_Q31         OPENLOOP_ACC_STEP_Q31_OF(MOTOR_ACC_RPM_S_PER_A)
#define OPENLOOP_HANDOVER_Q31         ((int32_t)(OPENLOOP_HANDOVER_RPM / SPEED_MAX * 2147483648.0))
#define OPENLOOP_SPIN_MIN_Q15         Q15(OPENLOOP_SPIN_MIN_RPM / SPEED_MAX)
#define OPENLOOP_FALLBACK_Q15         Q15(OPENLOOP_FALLBACK_RPM / SPEED_MAX)
#define OPENLOOP_FALLBACK_Q31         ((int32_t)(OPENLOOP_FALLBACK_RPM / SPEED_MAX * 2147483648.0))
/* I/F 加速度对应的加速电流（模型：MOTOR_ACC_RPM_S_PER_A），切换时作为加速前馈的起点 */
#define OPENLOOP_ACC_FF_IQ_OF(acc)    Q15((OPENLOOP_ACC_RPM_S_OF(acc) / (acc)) / I_MAX)

/* ---- 飞车检测（flystart.c） ---- */
#define FLY_BLOCK_TICKS         (PWM_FREQ / 1000)           /* 每块 1 ms */
#define FLYCURRENT_TH_PU_OF(psi, R) (FLY_STILL_RPM * 2.0 * 3.14159265358979 / 60.0 * MOTOR_POLE_PAIRS * (psi) / (R) / I_MAX)
#define FLY_SETTLE_BLOCKS_OF(L, R) ((uint16_t)(FLY_SETTLE_TAUS * (L) / (R) * 1000.0 + 0.5))
#define FLY_SETTLE_BLOCKS       FLY_SETTLE_BLOCKS_OF(MOTOR_LD_H, MOTOR_RS_OHM)
#define FLY_STILL_BLOCKS        ((uint16_t)(FLYBRAKE_TIME * 1000.0 + 0.5))
#define FLY_COLD_STILL_BLOCKS   ((uint16_t)(FLY_COLD_STILL_TIME * 1000.0 + 0.5))   /* 冷起动：本次检测从未发现转动 */
/* 短接制动的转速衰减时间常数 τ = J / B（残余转速 FLY_STILL_END_RPM 见 cfg_app.h）：
 *   Kt = 1.5 p ψf（N·m/A），J = Kt / (MOTOR_ACC_RPM_S_PER_A · 2π/60)，B = 1.5 p² ψf² / Rs（N·m·s/rad，低速 ωL << Rs） */
#define FLY_KT_NM_A_OF(psi)     (1.5 * MOTOR_POLE_PAIRS * (psi))
#define FLY_J_KGM2_OF(psi, acc) (FLY_KT_NM_A_OF(psi) / ((acc) * 2.0 * MATH_PI / 60.0))
#define FLY_SHORT_B_OF(psi, R)  (1.5 * MOTOR_POLE_PAIRS * MOTOR_POLE_PAIRS * (psi) * (psi) / (R))
#define FLY_BRAKE_TAU_S_OF(psi, R, acc) (FLY_J_KGM2_OF(psi, acc) / FLY_SHORT_B_OF(psi, R))
#define FLY_MAYMOVE_EXPIRE_MS_OF(psi, R, acc) ((uint32_t)(FLY_MAYMOVE_EXPIRE_TAUS * FLY_BRAKE_TAU_S_OF(psi, R, acc) * 1000.0 + 0.5))
#define FLY_KT_NM_A             FLY_KT_NM_A_OF(MOTOR_PSI_F_WB)    /* 转矩常数 N·m/A */
#define FLY_J_KGM2              FLY_J_KGM2_OF(MOTOR_PSI_F_WB, MOTOR_ACC_RPM_S_PER_A)    /* 转动惯量 kg·m²（由加速度系数反推） */
#define FLY_SHORT_B             FLY_SHORT_B_OF(MOTOR_PSI_F_WB, MOTOR_RS_OHM)    /* 三相短接时的制动系数 N·m·s/rad（短路转矩 / 机械角速度） */
#define FLY_BRAKE_TAU_S         FLY_BRAKE_TAU_S_OF(MOTOR_PSI_F_WB, MOTOR_RS_OHM, MOTOR_ACC_RPM_S_PER_A)    /* 短接制动时转速按指数衰减的时间常数 s */
/* 停机超过该时间（ms）即认为转子已停：飞车检测据此决定是否跳过最坏情况制动等待 */
#define FLY_MAYMOVE_EXPIRE_MS   FLY_MAYMOVE_EXPIRE_MS_OF(MOTOR_PSI_F_WB, MOTOR_RS_OHM, MOTOR_ACC_RPM_S_PER_A)
#define FLY_TIMEOUT_BLOCKS      ((uint32_t)(FLY_BRAKE_TIMEOUT_S * 1000.0 + 0.5))    /* 短接制动超时（ms） */
#define FLY_CATCH_Q15           Q15(FLY_CATCH_RPM / SPEED_MAX)    /* 同向直接接进闭环的转速下限（Q15） */
#define TRACK_END_Q15           Q15(TRACK_END_RPM / SPEED_MAX)    /* 零电流跟踪的结束转速（Q15） */
#define FLY_REV_SPIN_MIN_Q15    Q15(FLY_REV_SPIN_MIN_RPM / SPEED_MAX)    /* 反向直接接进闭环的转速下限（Q15） */
#define FLY_STABLE_MIN_Q15      Q15(FLY_LOW_MIN_RPM / SPEED_MAX)    /* 从这个转速起就要测准（低速同向进 I/F） */
/* 每块转过的电角度（2^32 = 一圈）-> Q15 转速：spd = dphi * K >> 32，
 * K = 60 * 32768 / (Tblock * 极对数 * SPEED_MAX) */
#define FLY_DPHI_K              ((int32_t)(60.0 * 32768.0 * PWM_FREQ / FLY_BLOCK_TICKS / MOTOR_POLE_PAIRS / SPEED_MAX + 0.5))
/* Q15 转速 -> w*Lq/Rs（Q12）：阻抗角 = atan(w*Lq/Rs) */
#define FLY_WLR_Q12_OF(L, R)    ((int32_t)(SPEED_MAX * 2 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS * (L) / (R) * 4096.0 + 0.5))
#define FLY_WLR_Q12             FLY_WLR_Q12_OF(MOTOR_LQ_H, MOTOR_RS_OHM)
/* ---- 短脉冲测速 ---- */
/* 脉冲周期：SPEED_MAX 时一个周期最多转 120° 电角度（相邻脉冲的夹角不会混叠），且不超过 2 ms */
#define FLY_PULSE_PERIOD_RAW    (PWM_FREQ / (3.0 * SPEED_MAX / 60.0 * MOTOR_POLE_PAIRS))
#define FLY_PULSE_PERIOD_TICKS  ((uint16_t)(FLY_PULSE_PERIOD_RAW < PWM_FREQ * 0.002 ? FLY_PULSE_PERIOD_RAW : PWM_FREQ * 0.002))
#define FLY_PULSE_MAX_TICKS     (FLY_PULSE_PERIOD_TICKS - 2)    /* 单个测速脉冲最长的拍数（留 2 拍让电流回零） */
/* 电流到这么大就松开（|i|^2，Q15） */
/* 脉冲电流下限（Q15）= 噪声峰值（码）* 每码 Q15（16）* 2/sqrt(3) / tan(FLY_SEED_ANGLE_DEG)，按码数线性 */
#define FLY_PULSE_ITH_PER_CODE  ((int32_t)(16.0 * 1.1547 * MATH_SIN_DEG(90.0 - FLY_SEED_ANGLE_DEG) / MATH_SIN_DEG(FLY_SEED_ANGLE_DEG) + 0.5))
#define FLY_PULSE_ITH_MAX_Q15   16384       /* 0.5 * I_MAX：噪声达到此程度说明采样异常，门限不再增大 */
#define FLY_PULSE_TOL_MIN       ((int32_t)(2.0 * FLY_SEED_ANGLE_DEG / 360.0 * 4294967296.0))   /* 2^32 = 一圈 */
#define FLY_PULSE_BASE_TICKS    ((FLY_PULSE_N - 1) * FLY_PULSE_PERIOD_TICKS)    /* 第一个到最后一个脉冲之间的总拍数（累计夹角对应的时间） */
/* 累计夹角（2^28 = 一圈，最多约 600°）-> Q15 转速：spd = sum * K >> 28（本电机 K ≈ 2e7，乘积 < 2^55） */
#define FLY_PULSE_K             ((int64_t)(60.0 * 32768.0 * PWM_FREQ / MOTOR_POLE_PAIRS / SPEED_MAX / FLY_PULSE_BASE_TICKS + 0.5))
/* 推进接管角：adv(2^32) = sum(2^28) * n * ADV_K >> 16，ADV_K = 8 * 65536 / BASE */
#define FLY_PULSE_ADV_K         ((int32_t)(8.0 * 65536.0 / FLY_PULSE_BASE_TICKS + 0.5))
#define FLY_SYNC_S              (4.0 / (FLUXPLL_ZETA * 2.0 * MATH_PI * FLUXPLL_WN_HZ))    /* 接管后零电流同步时间 = PLL 包络时间常数的 4 倍（收敛到 2% 以内） */
#define FLY_SYNC_TICKS          ((uint16_t)(FLY_SYNC_S * SLOWLOOP_FREQ + 0.5))    /* 同上，慢环拍数 */
#define OPENLOOP_WARM_MAX_Q31         ((int32_t)(1.5 * OPENLOOP_FALLBACK_RPM / SPEED_MAX * 2147483648.0))    /* 回退进 I/F 时开环转速的上限（Q31，1.5 倍回退转速） */
#define OPENLOOP_AGREE_Q15            Q15(OPENLOOP_AGREE_RATIO)    /* I/F 切闭环：开环转速和 PLL 转速"一致"的相对误差门限 */
#define OPENLOOP_AGREE_TICKS          ((uint16_t)(OPENLOOP_AGREE_S * SLOWLOOP_FREQ))    /* 一致要持续的慢环拍数 */
#define OPENLOOP_TIMEOUT_TICKS        ((uint16_t)(OPENLOOP_TIMEOUT_S * SLOWLOOP_FREQ))    /* I/F 拖动超时（慢环拍） */
#define OPENLOOP_BLEND_STEP_Q15      ((int32_t)(32768.0 / (OPENLOOP_BLEND_S * SLOWLOOP_FREQ) + 0.5))           /* 每慢环拍 */

/* ---- FluxPLL ------------------------------------------------------------- */
/* err = |psi|*sin(dtheta)，鉴相增益 Kd = FLUX_NOMINAL_Q15（LSB/rad）；
 * 1 rad/s 电角速度 = PLL_KS（Q31 速度单位）。
 *   2*zeta*wn = Kd*Kp/Ks，wn^2 = Kd*Ki*fs/Ks */
#define PLL_KS                  (2147483648.0 / (2 * MATH_PI * SPEED_MAX * POLEPAIRS / 60.0))
#define FLUXPLL_KP_OF(n)        ((int32_t)(2 * FLUXPLL_ZETA * 2 * MATH_PI * FLUXPLL_WN_HZ * PLL_KS / (n)))
#define FLUXPLL_KI_OF(n)        ((int32_t)((2 * MATH_PI * FLUXPLL_WN_HZ) * (2 * MATH_PI * FLUXPLL_WN_HZ) * PLL_KS / (n) / PWM_FREQ))
#define FLUXPLL_KP_Q31_PER_ERR  FLUXPLL_KP_OF(FLUX_NOMINAL_Q15)
#define FLUXPLL_KI_Q31_PER_ERR  FLUXPLL_KI_OF(FLUX_NOMINAL_Q15)

#endif
