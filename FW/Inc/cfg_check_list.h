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
 * 说明必须写在 CFG_CHECK 上一行的注释中，不能写成字符串：armcc 5 按本机多字节编码解析字符串中的
 * UTF-8 中文，可能吞掉结尾引号导致编译失败。
 * ==========================================================================*/

/* 0 < x < 1：能放进 Q15 且不饱和（Q15() 宏超量程只会悄悄饱和，不报错） */
#define FWC_POS_Q15(x)  ((x) > 0.0 && (x) < 0.99997)
/* 斜坡速率（Q31 / 慢环拍）；三者中的最大值决定制动距离计算的位宽 */
#define FWC_RAMP_Q31(r) ((r) / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0)
#define FWC_MAX(a, b)   ((a) > (b) ? (a) : (b))
#define FWC_RAMP_MAX    FWC_MAX(FWC_RAMP_Q31(SPEED_RAMP_REV), FWC_MAX(FWC_RAMP_Q31(SPEED_RAMP_UP), FWC_RAMP_Q31(SPEED_RAMP_DOWN)))

/* ---- 电机参数 ------------------------------------------------------------ */
/* 极对数 / Rs / Ld / Lq / 磁链 / 加速度系数 / 额定电流 / SPEED_MAX 为正（作除数或决定符号） */
CFG_CHECK(MOTOR_PARAMS_POSITIVE, MOTOR_POLE_PAIRS >= 1 && MOTOR_RS_OHM > 0 && MOTOR_LD_H > 0 && MOTOR_LQ_H > 0 &&
         MOTOR_PSI_F_WB > 0 && MOTOR_ACC_RPM_S_PER_A > 0 && MOTOR_I_RATED_A > 0 && SPEED_MAX > 0)

/* ---- 速度基准 ------------------------------------------------------------ */
/* SPEED_MAX 下反电势前馈系数须大于 0（>= UDC_MAX 时由 Q15 宏自动饱和并置 cfgWarn bit3） */
CFG_CHECK(EMF_AT_SPEED_MAX_FITS, DECOUPLE_K_EMF_Q15_OF(MOTOR_PSI_F_WB) > 0)
/* 每拍电角度增量（SPEED_MAX 时）必须 < 180°（否则采样混叠且 s16ThGain 符号翻转） */
CFG_CHECK(THETA_STEP_FITS, FWC_POS_Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ))
/* 起停门限：停机门限 > 0（否则 |指令| < 0 恒不成立，无法用指令停机）；起动门限 < SPEED_MAX（否则 Q15 饱和，永远不起动） */
CFG_CHECK(START_STOP_SPEED_FITS, RUN_TO_STOP_SPEED > 0 && STOP_TO_RUN_SPEED < SPEED_MAX)

/* ---- 电流环（超出 65535 时由 param.c 自动封顶并置 cfgWarn bit1） --------- */
/* 电流环 Kp、Ki 定点值不能被截断为 0 */
CFG_CHECK(CUR_GAINS_NONZERO, CURRENT_KP_D >= 1 && CURRENT_KP_Q >= 1 && CURRENT_KI_D >= 1 && CURRENT_KI_Q >= 1)

/* ---- 速度环（Pi_Run64） --------------------------------------------------- */
/* 速度环增益不能被截断为 0 或超出 int32 */
CFG_CHECK(SPD_GAINS_RANGE, SPEED_KP >= 1 && SPEED_KI >= 1 &&
         (2 * SPEED_ZETA * 2 * MATH_PI * SPEED_WN_HZ * SPD_GAIN_SCALE) < 2147483647.0)

/* ---- 磁链观测器 ---------------------------------------------------------- */
/* psi_f 在观测器磁链单位里不能被截断得过小（平方后供 NLFO 归一化使用） */
CFG_CHECK(FLUX_NOMINAL_RESOLUTION, FLUX_NOMINAL_Q15 >= 16)
/* 定子磁链 psi_f + Lmax*I_MAX 不超过观测器积分器限幅 FMAX（0.9） */
CFG_CHECK(FLUX_STATOR_FITS, FLUX_STATOR_MAX_WB / FLUX_PSI_N < 0.9)
/* Rs*I_MAX 不超出观测器电压基准 FLUX_V_BASE 的 Q15 量程 */
CFG_CHECK(FLUX_KRS_FITS, FWC_POS_Q15(MOTOR_RS_OHM * I_MAX / FLUX_V_BASE))
/* 电感饱和：斜率非负且 Q12 乘法不溢出；下限在 (0, 1]（L <= L0，上面的定子磁链界才成立） */
CFG_CHECK(FLUX_LS_SAT_FITS, MOTOR_LS_SAT_PER_A >= 0 && MOTOR_LS_SAT_PER_A * I_MAX < 7.0 &&
         MOTOR_LS_SAT_MIN > 0 && MOTOR_LS_SAT_MIN <= 1.0)
/* 逆变器电压误差：KT 在 Q15 内且非零；I0 > 0（作除数）且 Q8 斜率在 int32 内 */
CFG_CHECK(INVERTER_DT_FITS, !INVERTER_DT_COMP || (INVERTER_DT_EFF_S * PWM_FREQ * 32768.0 >= 1 && FWC_POS_Q15(INVERTER_DT_EFF_S * PWM_FREQ) &&
         INVERTER_DT_I0_A > 0.0 && 256.0 * I_MAX / INVERTER_DT_I0_A < 2147483647.0))
/* 观测器积分步长 KTs 须在 Q15 正数范围内 */
CFG_CHECK(FLUX_KTS_FITS, FWC_POS_Q15(2.0 * MATH_PI * FLUX_WN_HZ / PWM_FREQ))
/* NLFO：归一化定点非零且 ((ferr>>12)*norm) 不溢出 int32（|ferr>>12| <= 2^18，故 norm < 8192） */
CFG_CHECK(NLFO_FITS, FLUX_NLFO_NORM_Q8 >= 1 && FLUX_NLFO_NORM_Q8 < FLUX_NLFO_NORM_MAX && FLUX_NLFO_TAU_S > 0.0)

/* ---- FluxPLL（超出 32767 时由 param.c 自动封顶并置 cfgWarn bit2） -------- */
/* PLL 增益 Kp、Ki 定点值不能被截断为 0 */
CFG_CHECK(PLL_GAINS_NONZERO, FLUXPLL_KP_Q31_PER_ERR >= 1 && FLUXPLL_KI_Q31_PER_ERR >= 1)

/* ---- dq 解耦前馈：(Q15*Q15>>15) * K 不能溢出 int32 ------------------------- */
/* 解耦电感系数须在 16 位无符号范围内，保证与 Q15 相乘不溢出 int32 */
CFG_CHECK(DECOUPLE_K_FITS, DECOUPLE_K_LQ_Q12 < 65536 && DECOUPLE_K_LD_Q12 < 65536)

/* ---- 起动 ---------------------------------------------------------------- */
/* 起动 / 停机方式只能取已定义的值 */
CFG_CHECK(MODES_VALID, (STARTUP_MODE == STARTUP_MODE_IPD || STARTUP_MODE == STARTUP_MODE_SWING) &&
         (STOP_MODE == STOP_MODE_RAMP || STOP_MODE == STOP_MODE_COAST))
/* 拖动电流与定位电流换成 Q15 后 >= 1（openloop.c 与摆动扫掠系数以它们作除数） */
CFG_CHECK(OPENLOOP_CURRENTS_NONZERO, OPENLOOP_IQ_Q15 >= 1 && ALIGN_ID_Q15 >= 1)
/* 摆动中起动：拍数平方与扫掠角乘法不溢出 int32 */
CFG_CHECK(ALIGN_SWING_FITS, ALIGN_SWING_MAX_T * ALIGN_SWING_MAX_T * ALIGN_T2_TICKS2 < 32767.0 * 32767.0 &&
         (double)ALIGN_ID_Q15 * ALIGN_SWING_SWEEP_K_Q15 < 2147483647.0 && ALIGN_SWING_LAG_ANG_Q12 < 1000000)
/* IPD：三角波拍数 >= 2 且在 int16 内，测量拍数与偏置步长不被截断为 0 */
CFG_CHECK(IPD_FITS, IPD_HF_TICKS >= 2 && IPD_HF_TICKS < 32768 && IPD_MEAS_MS >= 1 && IPD_BIAS_STEP_Q15 >= 1)
/* I/F 加速度步长与定位电流步长不能被截断为 0 */
CFG_CHECK(OPENLOOP_ACC_RESOLUTION, OPENLOOP_ACC_STEP_Q31 >= 1 && ALIGN_STEP_Q15 >= 1)
/* 平衡负载角 asin(OPENLOOP_ACC_FRAC) 的定义域 (0, 1) */
CFG_CHECK(OPENLOOP_ACC_FRAC_RANGE, OPENLOOP_ACC_FRAC > 0 && OPENLOOP_ACC_FRAC < 1.0)
/* I/F 加速度步长 x 开环电流在 32 位有符号整数范围内（openloop.c 用 32 位乘除） */
CFG_CHECK(OPENLOOP_ACC_STEP_FITS, (double)OPENLOOP_ACC_STEP_Q31 * OPENLOOP_REV_IQ_Q15 / OPENLOOP_IQ_Q15 * OPENLOOP_WARM_IMAX_Q15 < 2147483647.0 &&
         (double)OPENLOOP_ACC_STEP_Q31 * OPENLOOP_WARM_IMAX_Q15 < 2147483647.0)
/* 回退 I/F：|iq| / sin(δmax) 要求 δmax 在 (0°, 90°)；电流变化步长在 Q15 正数范围内 */
CFG_CHECK(OPENLOOP_WARM_FITS, OPENLOOP_WARM_DELTA_DEG > 0.0 && OPENLOOP_WARM_DELTA_DEG < 90.0 &&
         CURRENT_SLEW_STEP_Q15 >= 1 && CURRENT_SLEW_STEP_Q15 < 32767)
/* 定位静止门限 ALIGN_STILL_RPM 换算为反电势至少 1 LSB */
CFG_CHECK(OPENLOOP_STILL_RESOLUTION, ALIGN_STILL_EMF_Q15 >= 1)
/* I/F 切换融合步长在 Q15 正数范围内 */
CFG_CHECK(OPENLOOP_BLEND_FITS, OPENLOOP_BLEND_STEP_Q15 >= 1 && OPENLOOP_BLEND_STEP_Q15 < 32767)

/* ---- 时基 ---------------------------------------------------------------- */
/* 慢环频率：整除 PWM 频率；单摆周期与各一阶滤波移位窗口、失速检查拍数至少为 1 */
CFG_CHECK(SLOW_RATE_FITS, (PWM_FREQ % SLOWLOOP_FREQ) == 0 && ALIGN_T2_TICKS2 >= 1.0 &&
         MEAS_VBUS_FILT_SH >= 1 && MEAS_IBUS_FILT_SH >= 1 && OPENLOOP_PLL_FILT_SH >= 1 &&
         ALIGN_EMF_FILT_SH >= 1 && SPIN_CHECK_TICKS >= 1)
/* 定位与 I/F 各段时间换成慢环拍数不超过 uint16（65535）；飞车稳定块数 >= 1 */
CFG_CHECK(OPENLOOP_TICKS_FIT, ALIGN_MAX_T * ALIGN_MAX_T * ALIGN_T2_S2 * SLOWLOOP_FREQ * SLOWLOOP_FREQ < 65535.0 * 65535.0 &&
         FLY_SETTLE_BLOCKS >= 1 && OPENLOOP_TIMEOUT_S * SLOWLOOP_FREQ < 65536 && OPENLOOP_AGREE_S * SLOWLOOP_FREQ < 65536)

/* ---- 速度斜坡 ------------------------------------------------------------ */
/* 斜坡定点位宽：speed_ctrl.c 制动距离 c^2/(2j) 与母线电压控制器不溢出 32/64 位整数；圆角时间 > 0 */
CFG_CHECK(RAMP_FIXEDPOINT_FITS, SPEED_RAMP_ROUND_S > 0 && SPEED_JERK_Q31 >= 1 &&
         4294967296.0 / (2.0 * SPEED_JERK_Q31) < 4294967295.0 && FWC_RAMP_MAX < 16777216.0 &&
         FWC_RAMP_MAX * FWC_RAMP_MAX / 65536.0 * (4294967296.0 / (2.0 * SPEED_JERK_Q31)) < 9.2e18 &&
         (double)BUS_VDC_REF_SQ * BUS_VDC_K_Q8 / 65536.0 * (1073741824.0 / BUS_VDC_SMIN_Q15) < 1.8e19 && BUS_VDC_SMIN_Q15 > 0)
/* 斜率前馈增益与倒数在正 int32 / Q8 位宽内 */
CFG_CHECK(SPEED_FF_K_FITS, SPEED_FF_K_Q16 >= 1 && SPEED_FF_K_Q16 < 2147483647 &&
         SPEED_ACC_PER_IQ_Q8 >= 1 && SPEED_ACC_PER_IQ_Q8 < 32768 * 256)
/* 转速反馈滤波拍数在移位表可表示范围内 */
CFG_CHECK(SPEED_FB_FILT_FITS, SPEED_FB_FILT_TICKS > 0.0 && SPEED_FB_FILT_TICKS < 128.0)
/* 软停机步长在 Q15 正数范围内 */
CFG_CHECK(SOFT_STOP_FITS, SOFT_STOP_STEP_Q15 >= 1 && SOFT_STOP_STEP_Q15 < 32767)

/* ---- 飞车检测 ------------------------------------------------------------ */
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
/* 阻抗角系数 FLY_WLR_Q12 非零（超出 65535 时由 param.c 自动封顶以防 32 位乘法溢出） */
CFG_CHECK(FLY_WLR_FITS, FLY_WLR_Q12 >= 1)
/* FLYCURRENT_TH_PU 必须在 (0, 1) 内 */
CFG_CHECK(FLY_CURRENT_TH_FITS, FWC_POS_Q15(FLYCURRENT_TH_PU))
/* 飞车接管同步拍数在 [1, 65535) 内 */
CFG_CHECK(FLY_SYNC_FITS, FLY_SYNC_TICKS >= 1 && FLY_SYNC_S * SLOWLOOP_FREQ < 65535.0)

/* ---- 保护与母线控制 ------------------------------------------------------ */
/* 过压门限低于采样满量程 UDC_MAX（否则 Q15 饱和，过压保护永远不动作） */
CFG_CHECK(OVP_FITS, DCBUS_OVER < UDC_MAX)
/* 缺相：窗口电周期数 >= 1；窗口累加值与 2 s 内快环拍数乘积不溢出 32 位 */
CFG_CHECK(PHASELESS_FITS, PHASELESS_PERIODS >= 1 && PHASELESS_ACC_FULL > 0 &&
         PHASELESS_ACC_FULL < 2147483647.0 && PHASELESS_N_MAX * 32768.0 < 4294967295.0)
/* 回馈削顶斜率以 (FULL - START) 作除数：定点后 FULL > START */
CFG_CHECK(BUS_REGEN_FITS, BUS_REGEN_FULL_Q15 > BUS_REGEN_START_Q15)
/* 母线电压控制器：电容与带宽为正，增益与滤波系数在定点位宽内，全力制动门限在 (0, 1]（作除数） */
CFG_CHECK(BUS_VDC_FITS, BOARD_CBUS_UF > 0 && BUS_VDC_BW_HZ > 0 &&
         BUS_VDC_K_Q8 >= 1 && BUS_VDC_K_Q8 < 2147483647 &&
         BUS_VDC_FILT_Q15 >= 1 && BUS_VDC_FILT_Q15 < 32768 && BUS_BRAKE_FULL_LAMBDA > 0 && BUS_BRAKE_FULL_LAMBDA <= 1.0)

/* ---- 弱磁 ---------------------------------------------------------------- */
#if CTRL_WEAKEN_ENABLE
/* 弱磁积分增益非零，且 WEAKEN_ID_MIN_A < 0（符号） */
CFG_CHECK(WEAKEN_GAINS_FIT, WEAKEN_KI_Q20 >= 1 && WEAKEN_ID_MIN_A < 0)
#endif
