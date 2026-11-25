/* ============================================================================
 * @file    cfg_check_list.h
 * @brief   配置检查清单：只检查"定点 / 算术上必然出错"的条件
 *
 * 每项两行：上一行注释为说明，下一行为 CFG_CHECK(名称, 条件)。
 * 原则：只放违反后必然出现确定性算术错误的条件——定点溢出、截断为 0、除零、符号翻转、函数定义域，
 *       以及换成 Q15 后饱和而使保护永远不动作的项。门限之间的大小顺序、取值是否合理、物理上能否实现，
 *       都交给人工（违反时行为确定，可由调试观察）。可在运行时封顶的项由 param.c 封顶并置 cfgWarn。
 *   - 目标板（cfg_check.h）：gFluxDiag.cfgFailLine = 第一个不满足项的行号，电机不起动。
 *   - clang（CFG_STATIC_CHECK）：不满足的项直接产生编译错误。
 * 分两部分：第一部分只与板子 / 应用配置有关，始终检查；第二部分检查 cfg_motor.h 中 Rs / L / 饱和 / 磁链 / 加速度系数的
 * 配置值，只在 IDENT_TEST = 0 时检查：IDENT_TEST = 1 时这些值由辨识测得，配置值只是占位、不被使用，测得的值由
 * param.c checkParam 按同样的条件检查后才生效（checkParam 与第二部分逐项对应）。
 * 说明必须写在 CFG_CHECK 上一行的注释中，不能写成字符串：armcc 5 按本机多字节编码解析字符串中的
 * UTF-8 中文，可能吞掉结尾引号导致编译失败。
 * ==========================================================================*/

/* 0 < x < 1：能放进 Q15 且不饱和（Q15() 宏超量程只会悄悄饱和，不报错） */
#define FWC_POS_Q15(x)  ((x) > 0.0 && (x) < 0.99997)
/* 斜坡速率（Q31 / 慢环拍）；三者中的最大值决定制动距离计算的位宽 */
#define FWC_RAMP_Q31(r) ((r) / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0)
#define FWC_MAX(a, b)   ((a) > (b) ? (a) : (b))
#define FWC_RAMP_MAX    FWC_MAX(FWC_RAMP_Q31(SPEED_RAMP_REV), FWC_MAX(FWC_RAMP_Q31(SPEED_RAMP_UP), FWC_RAMP_Q31(SPEED_RAMP_DOWN)))

/* ===================== 一、板子与应用配置（始终检查） ===================== */
/* 极对数 / 额定电流 / SPEED_MAX 为正（作除数或决定符号）；Rs / L / 磁链 / 加速度系数为正：
 * IDENT_TEST = 1 时为占位值（任意正值），上电计算初始增益时作除数 */
CFG_CHECK(MOTOR_PARAMS_POSITIVE, MOTOR_POLE_PAIRS >= 1 && MOTOR_RS_OHM > 0 && MOTOR_LD_H > 0 && MOTOR_LQ_H > 0 &&
         MOTOR_PSI_F_WB > 0 && MOTOR_ACC_RPM_S_PER_A > 0 && MOTOR_I_RATED_A > 0 && SPEED_MAX > 0)
/* 每拍电角度增量（SPEED_MAX 时）必须 < 180°（否则采样混叠且 s16ThGain 符号翻转） */
CFG_CHECK(THETA_STEP_FITS, FWC_POS_Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ))
/* 起停门限：停机门限 > 0（否则 |指令| < 0 恒不成立，无法用指令停机）；起动门限 < SPEED_MAX（否则 Q15 饱和，永远不起动） */
CFG_CHECK(START_STOP_SPEED_FITS, RUN_TO_STOP_SPEED > 0 && STOP_TO_RUN_SPEED < SPEED_MAX)
/* 电感饱和下限在 (0, 1]（L <= L0，观测器定子磁链界才成立） */
CFG_CHECK(FLUX_LS_SAT_MIN_RANGE, MOTOR_LS_SAT_MIN > 0 && MOTOR_LS_SAT_MIN <= 1.0)
/* 逆变器电压误差：KT 在 Q15 内且非零；I0 > 0（作除数）且 Q8 斜率在 int32 内 */
CFG_CHECK(INVERTER_DT_FITS, !INVERTER_DT_COMP || (INVERTER_DT_EFF_S * PWM_FREQ * 32768.0 >= 1 && FWC_POS_Q15(INVERTER_DT_EFF_S * PWM_FREQ) &&
         INVERTER_DT_I0_A > 0.0 && 256.0 * I_MAX / INVERTER_DT_I0_A < 2147483647.0))
/* NLFO 时间常数为正（作除数） */
CFG_CHECK(NLFO_TAU_POSITIVE, FLUX_NLFO_TAU_S > 0.0)
/* 起动 / 停机方式只能取已定义的值 */
CFG_CHECK(MODES_VALID, (STARTUP_MODE == STARTUP_MODE_IPD || STARTUP_MODE == STARTUP_MODE_SWING) &&
         (STOP_MODE == STOP_MODE_RAMP || STOP_MODE == STOP_MODE_COAST))
/* 拖动电流与定位电流换成 Q15 后 >= 1（openloop.c 与摆动扫掠系数以它们作除数） */
CFG_CHECK(OPENLOOP_CURRENTS_NONZERO, OPENLOOP_IQ_Q15 >= 1 && ALIGN_ID_Q15 >= 1)
/* 摆动扫掠：定位电流 x 扫掠系数不溢出 int32；定位电流步长不被截断为 0 */
CFG_CHECK(ALIGN_SWEEP_FITS, (double)ALIGN_ID_Q15 * ALIGN_SWING_SWEEP_K_Q15 < 2147483647.0 && ALIGN_STEP_Q15 >= 1)
/* IPD：三角波拍数 >= 2 且在 int16 内，测量拍数与偏置步长不被截断为 0 */
CFG_CHECK(IPD_FITS, IPD_HF_TICKS >= 2 && IPD_HF_TICKS < 32768 && IPD_MEAS_MS >= 1 && IPD_BIAS_STEP_Q15 >= 1)
/* 平衡负载角 asin(OPENLOOP_ACC_FRAC) 的定义域 (0, 1)；回退 I/F 的 |iq| / sin(δmax) 要求 δmax 在 (0°, 90°) */
CFG_CHECK(OPENLOOP_ANGLES_RANGE, OPENLOOP_ACC_FRAC > 0 && OPENLOOP_ACC_FRAC < 1.0 &&
         OPENLOOP_WARM_DELTA_DEG > 0.0 && OPENLOOP_WARM_DELTA_DEG < 90.0)
/* 定位静止门限 ALIGN_STILL_EMF_PCT 换算为反电势至少 1 LSB；参数辨识等停稳的拍数在 uint16 内，半周期倍数为正；
 * 摆动阶跃角在 (0°, 90°)（q 轴反电势与转速同号）且整除 360°（序列走一圈回到起点） */
CFG_CHECK(OPENLOOP_STILL_RESOLUTION, ALIGN_STILL_EMF_Q15 >= 1 && ALIGN_QUIET_MIN_TICKS >= 1 && ALIGN_WAIT_MAX_S * SLOWLOOP_FREQ < 65535.0 &&
         ALIGN_QUIET_HALF_K >= 1 && IDENT_PEND_STEP_DEG > 0.0 && IDENT_PEND_STEP_DEG < 90.0 &&
         (360 % (int)IDENT_PEND_STEP_DEG) == 0 && (double)(int)IDENT_PEND_STEP_DEG == IDENT_PEND_STEP_DEG && IDENT_PEND_SPREAD > 1.0)
/* I/F 切换融合步长在 Q15 正数范围内 */
CFG_CHECK(OPENLOOP_BLEND_FITS, OPENLOOP_BLEND_STEP_Q15 >= 1 && OPENLOOP_BLEND_STEP_Q15 < 32767)
/* 慢环频率整除 PWM 频率；各一阶滤波移位窗口、失速检查拍数至少为 1；I/F 各段时间换成慢环拍数不超过 uint16 */
CFG_CHECK(SLOW_RATE_FITS, (PWM_FREQ % SLOWLOOP_FREQ) == 0 &&
         MEAS_VBUS_FILT_SH >= 1 && MEAS_IBUS_FILT_SH >= 1 && OPENLOOP_PLL_FILT_SH >= 1 &&
         ALIGN_EMF_FILT_SH >= 1 && SPIN_CHECK_TICKS >= 1 &&
         OPENLOOP_TIMEOUT_S * SLOWLOOP_FREQ < 65536 && OPENLOOP_AGREE_S * SLOWLOOP_FREQ < 65536)
/* 斜坡定点位宽：speed_ctrl.c 制动距离 c^2/(2j) 不溢出 32/64 位整数；圆角时间 > 0 */
CFG_CHECK(RAMP_FIXEDPOINT_FITS, SPEED_RAMP_ROUND_S > 0 && SPEED_JERK_Q31 >= 1 &&
         4294967296.0 / (2.0 * SPEED_JERK_Q31) < 4294967295.0 && FWC_RAMP_MAX < 16777216.0 &&
         FWC_RAMP_MAX * FWC_RAMP_MAX / 65536.0 * (4294967296.0 / (2.0 * SPEED_JERK_Q31)) < 9.2e18)
/* 转速反馈滤波拍数在移位表可表示范围内 */
CFG_CHECK(SPEED_FB_FILT_FITS, SPEED_FB_FILT_TICKS > 0.0 && SPEED_FB_FILT_TICKS < 128.0)
/* 短脉冲测速：脉冲周期至少 4 拍，脉冲个数 >= 3（至少两段间隔才能测速） */
CFG_CHECK(FLY_PULSE_PERIOD_FITS, FLY_PULSE_PERIOD_RAW >= 4.0 && FLY_PULSE_N >= 3)
/* 接管角推进 (pulseLen + 2*since) * ADV_K 放进 int32（32x32 乘法不溢出） */
CFG_CHECK(FLY_PULSE_ADV_FITS, (3.0 * FLY_PULSE_PERIOD_TICKS + 6.0) * FLY_PULSE_ADV_K < 2147483647.0)
/* 脉冲电流门限：噪声下限 >= 1 码；种子角 > 0（作 sin 的分母）；最短脉冲 >= 1 拍 */
CFG_CHECK(FLY_PULSE_CURRENT_FITS, BOARD_ADC_NOISE_CODES >= 1.0 && FLY_SEED_ANGLE_DEG > 0 && FLY_PULSE_MIN_TICKS >= 1)
/* 连续短接测量按 1 ms 分块，PWM_FREQ 须为 1 kHz 的整数倍 */
CFG_CHECK(FLY_PWM_MULTIPLE_OF_1K, (PWM_FREQ % 1000) == 0)
/* 等停稳时间用 ln((x-1)/(x+1))，x = FLY_STILL_RPM / FLY_STILL_END_RPM：定义域要求 0 < 残余转速 < 静止门限 */
CFG_CHECK(FLY_STILL_END_FITS, FLY_STILL_END_RPM > 0 && FLY_STILL_END_RPM < FLY_STILL_RPM)
/* 飞车接管同步拍数在 [1, 65535) 内 */
CFG_CHECK(FLY_SYNC_FITS, FLY_SYNC_TICKS >= 1 && FLY_SYNC_S * SLOWLOOP_FREQ < 65535.0)
/* 过压门限低于采样满量程 UDC_MAX（否则 Q15 饱和，过压保护永远不动作） */
CFG_CHECK(OVP_FITS, DCBUS_OVER < UDC_MAX)
/* 缺相：窗口电周期数 >= 1；窗口累加值与 2 s 内快环拍数乘积不溢出 32 位 */
CFG_CHECK(PHASELESS_FITS, PHASELESS_PERIODS >= 1 && PHASELESS_ACC_FULL > 0 &&
         PHASELESS_ACC_FULL < 2147483647.0 && PHASELESS_N_MAX * 32768.0 < 4294967295.0)
/* 母线电压控制器：电容与带宽为正，滤波系数在定点位宽内，全力制动门限在 (0, 1]（作除数） */
CFG_CHECK(BUS_VDC_BOARD_FITS, BOARD_CBUS_UF > 0 && BUS_VDC_BW_HZ > 0 && BUS_VDC_SMIN_Q15 > 0 &&
         BUS_VDC_FILT_Q15 >= 1 && BUS_VDC_FILT_Q15 < 32768 && BUS_BRAKE_FULL_LAMBDA > 0 && BUS_BRAKE_FULL_LAMBDA <= 1.0)
#if CTRL_WEAKEN_ENABLE
/* 弱磁积分增益非零，且 WEAKEN_ID_MIN_A < 0（符号） */
CFG_CHECK(WEAKEN_GAINS_FIT, WEAKEN_KI_Q20 >= 1 && WEAKEN_ID_MIN_A < 0)
#endif

/* ===================== 二、电机参数配置值（仅 IDENT_TEST = 0） ===================== */
/* 与 param.c checkParam 逐项对应：checkParam 对任意一组参数（含辨识测得值）检查同样的条件 */
#if !IDENT_TEST
/* [Rs] Rs*I_MAX 不超出观测器电压基准的 Q15 量程；电流环 Ki 不被截断为 0 */
CFG_CHECK(PARAM_RS_FITS, FWC_POS_Q15(MOTOR_RS_OHM * I_MAX / FLUX_V_BASE) && CURRENT_KI_D >= 1)
/* [L] 电流环 Kp 不被截断为 0；解耦电感系数在 16 位无符号内；飞车阻抗角系数非零、短接暂态拍数 >= 1；摆动滞后角在范围内 */
CFG_CHECK(PARAM_L_FITS, CURRENT_KP_D >= 1 && CURRENT_KP_Q >= 1 && DECOUPLE_K_LQ_Q12 < 65536 && DECOUPLE_K_LD_Q12 < 65536 &&
         FLY_WLR_Q12 >= 1 && FLY_SETTLE_BLOCKS >= 1 && ALIGN_SWING_LAG_ANG_Q12 < 1000000)
/* [L(i)] 饱和斜率非负且 Q12 乘法不溢出 */
CFG_CHECK(PARAM_LS_FITS, MOTOR_LS_SAT_PER_A >= 0 && MOTOR_LS_SAT_PER_A * I_MAX < 7.0)
/* [psi] 定子磁链在观测器量程内、磁链定点不过小、积分步长在 Q15 内、NLFO 归一化与 PLL 增益非零且不溢出；
 * 反电势前馈系数非零；飞车短路电流门限在 (0, 1)；回馈削顶 FULL > START（作除数） */
CFG_CHECK(PARAM_PSI_FITS, FLUX_STATOR_MAX_WB / FLUX_PSI_N < 0.9 && FLUX_NOMINAL_Q15 >= 16 &&
         FWC_POS_Q15(2.0 * MATH_PI * FLUX_WN_HZ / PWM_FREQ) &&
         FLUX_NLFO_NORM_Q8 >= 1 && FLUX_NLFO_NORM_Q8 < FLUX_NLFO_NORM_MAX &&
         FLUXPLL_KP_Q31_PER_ERR >= 1 && FLUXPLL_KI_Q31_PER_ERR >= 1 &&
         DECOUPLE_K_EMF_Q15_OF(MOTOR_PSI_F_WB) > 0 && FWC_POS_Q15(FLYCURRENT_TH_PU) && BUS_REGEN_FULL_Q15 > BUS_REGEN_START_Q15)
/* [acc] 速度环增益非零且不超 int32；I/F 加速度步长非零且反转放大后在 int32 内；电流变化步长在 Q15 内；
 * 斜率前馈在位宽内；定位摆周期拍数 >= 1 且其平方与扫掠拍数乘积不溢出；母线电压控制器增益在 int32 内且乘积不溢出 64 位 */
CFG_CHECK(PARAM_ACC_FITS, SPEED_KP >= 1 && SPEED_KI >= 1 &&
         (2 * SPEED_ZETA * 2 * MATH_PI * SPEED_WN_HZ * SPD_GAIN_SCALE) < 2147483647.0 &&
         OPENLOOP_ACC_STEP_Q31 >= 1 && (double)OPENLOOP_ACC_STEP_Q31 * OPENLOOP_REV_IQ_Q15 / OPENLOOP_IQ_Q15 < 2147483647.0 &&
         CURRENT_SLEW_STEP_Q15 >= 1 && CURRENT_SLEW_STEP_Q15 < 32767 &&
         SPEED_FF_K_Q16 >= 1 && SPEED_FF_K_Q16 < 2147483647 && SPEED_ACC_PER_IQ_Q8 >= 1 && SPEED_ACC_PER_IQ_Q8 < 32768 * 256 &&
         ALIGN_T2_TICKS2 >= 1.0 && ALIGN_SWING_MAX_T * ALIGN_SWING_MAX_T * ALIGN_T2_TICKS2 < 32767.0 * 32767.0 &&
         BUS_VDC_K_Q8 >= 1 && BUS_VDC_K_Q8 < 2147483647 &&
         (double)BUS_VDC_REF_SQ * BUS_VDC_K_Q8 / 65536.0 * (1073741824.0 / BUS_VDC_SMIN_Q15) < 1.8e19)
#endif
