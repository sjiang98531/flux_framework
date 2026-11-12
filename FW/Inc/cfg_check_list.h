/* ============================================================================
 * @file    cfg_check_list.h
 * @brief   配置检查清单（更换电机 / 驱动板后自动逐项检查）
 *
 * 每项两行：上一行注释为说明，下一行为 CFG_CHECK(名称, 条件)。条件为假表示这组参数推导出的
 * 定点常量会溢出、被截断为 0、精度不足，或门限之间相互矛盾。
 *   - 目标板（cfg_check.h）：gFluxDiag.cfgFailLine = 第一个不满足项的行号，电机不起动。
 *   - clang（CFG_STATIC_CHECK）：不满足的项直接产生编译错误。
 * 此处只放"计算结果必然错误"的硬条件，不放调参建议类的软条件。
 * 说明必须写在 CFG_CHECK 上一行的注释中，不能写成字符串：armcc 5 按本机多字节编码解析字符串中的
 * UTF-8 中文，可能吞掉结尾引号导致编译失败。
 * ==========================================================================*/

/* 0 < x < 1：能放进 Q15 且不饱和（Q15() 宏超量程只会悄悄饱和，不报错） */
#define FWC_POS_Q15(x)  ((x) > 0.0 && (x) < 0.99997)

/* ---- 电机参数本身 -------------------------------------------------------- */
/* cfg_motor.h 里极对数/Rs/Ld/Lq/磁链/加速度系数/SPEED_MAX 都必须为正 */
CFG_CHECK(MOTOR_PARAMS_POSITIVE, MOTOR_POLE_PAIRS >= 1 && MOTOR_RS_OHM > 0 && MOTOR_LD_H > 0 && MOTOR_LQ_H > 0 &&
         MOTOR_PSI_F_WB > 0 && MOTOR_ACC_RPM_S_PER_A > 0 && SPEED_MAX > 0)
/* 铭牌额定值：0 < MOTOR_I_RATED_A < I_MAX，0 < MOTOR_SPEED_RATED_RPM < SPEED_MAX，MOTOR_P_RATED_W > 0 */
CFG_CHECK(MOTOR_RATING_SANE, MOTOR_I_RATED_A > 0 && MOTOR_I_RATED_A < I_MAX && MOTOR_P_RATED_W > 0 &&
         MOTOR_SPEED_RATED_RPM > 0 && MOTOR_SPEED_RATED_RPM < SPEED_MAX)
#if !MOTOR_IPM
/* MOTOR_IPM=0（表贴）时 Ld 必须等于 Lq；凸极电机请置 MOTOR_IPM=1 */
CFG_CHECK(SPM_NEEDS_LD_EQ_LQ, MOTOR_LD_H == MOTOR_LQ_H)
#endif

/* ---- 速度基准 ------------------------------------------------------------ */
/* SPEED_MAX 下反电势须小于电压满量程，否则 vq 前馈 DECOUPLE_K_EMF_Q15 饱和 */
/* SPEED_MAX 下反电势 >= UDC_MAX：SPEED_MAX 取值过大（或母线采样满量程过小） */
CFG_CHECK(EMF_AT_SPEED_MAX_FITS, FWC_POS_Q15(W_EMAX_RAD_S * MOTOR_PSI_F_WB / UDC_MAX))
/* 每拍电角度增量（SPEED_MAX 时）必须 < 180° */
/* SPEED_MAX 时每个 PWM 周期转过 >= 180° 电角度：SPEED_MAX 太大或 PWM 太低 */
CFG_CHECK(THETA_STEP_FITS, FWC_POS_Q15(2 * SPEED_MAX * POLEPAIRS / 60.0 / PWM_FREQ))
/* 启停门限：0 < RUN_TO_STOP_SPEED < STOP_TO_RUN_SPEED < SPEED_MAX，且换算为 Q15 后回差至少 1 LSB（否则无回差） */
CFG_CHECK(START_STOP_SPEED_ORDER, RUN_TO_STOP_SPEED > 0 && STOP_TO_RUN_SPEED < SPEED_MAX &&
         Q15(STOP_TO_RUN_SPEED / SPEED_MAX) > Q15(RUN_TO_STOP_SPEED / SPEED_MAX))

/* ---- 电流环（Pi_Run32 要求 Kp、Ki <= 65535） ------------------------------ */
/* 电流环 Kp > 65535：CURRENT_BW_HZ 对这台电机太高（电感大），调低带宽 */
CFG_CHECK(CUR_KP_RANGE, CURRENT_KP_D <= 65535 && CURRENT_KP_Q <= 65535)
/* 电流环 Ki > 65535：CURRENT_BW_HZ 太高（电阻大） */
CFG_CHECK(CUR_KI_RANGE, CURRENT_KI_D <= 65535 && CURRENT_KI_Q <= 65535)
/* 电流环 Kp < 64（分辨率不够）：电感/I_MAX 太小，提高 CURRENT_BW_HZ */
CFG_CHECK(CUR_KP_RESOLUTION, CURRENT_KP_D >= 64 && CURRENT_KP_Q >= 64)
/* 电流环 Ki < 4（分辨率不够）：电阻/I_MAX 太小或 PWM 太高，提高 CURRENT_BW_HZ */
CFG_CHECK(CUR_KI_RESOLUTION, CURRENT_KI_D >= 4 && CURRENT_KI_Q >= 4)

/* ---- 速度环（Pi_Run64） --------------------------------------------------- */
/* 速度环增益截断成 0 或超出 int32：检查 MOTOR_ACC_RPM_S_PER_A、SPEED_WN_HZ */
CFG_CHECK(SPD_GAINS_RANGE, SPEED_KP >= 16 && SPEED_KI >= 1 &&
         (2 * SPEED_ZETA * 2 * MATH_PI * SPEED_WN_HZ * SPD_GAIN_SCALE) < 2147483647.0)
/* SPD_PI_LIMIT 必须小于电流满量程 I_MAX */
CFG_CHECK(SPD_LIMIT_FITS, SPD_PI_LIMIT > 0 && SPD_PI_LIMIT < I_MAX)

/* ---- 磁链观测器 ---------------------------------------------------------- */
/* 磁链定点基准 FLUX_PSI_N 自动按"最大定子磁链 psi_f + Lmax*I_MAX 占量程 0.9 / FLUX_PARAM_MARGIN = 0.6"取（cfg_derived.h）。 */
/* psi_f 在观测器磁链单位里 < 0.03（< 1000 LSB）：电流满量程 I_MAX 相对电机太大（L*I_MAX 远大于 psi_f，量程被电感磁链占掉），
 * 或 FLUX_WN_HZ 到了上限 0.9*PWM_FREQ/(2*pi)（UDC_MAX 相对电机磁链太高）。换采样电阻 / 运放倍数或提高 PWM_FREQ */
CFG_CHECK(FLUX_NOMINAL_RESOLUTION, MOTOR_PSI_F_WB / FLUX_PSI_N >= 0.03)
/* 定子磁链 psi_f + Lmax*I_MAX 超过观测器量程的 90%（自动取值下不会发生，用于防止基准公式被误改） */
CFG_CHECK(FLUX_STATOR_FITS, FLUX_STATOR_MAX_WB / FLUX_PSI_N < 0.9)
/* Rs*I_MAX >= UDC_MAX：电阻压降超过电压满量程，板子和电机不匹配 */
CFG_CHECK(FLUX_KRS_FITS, FWC_POS_Q15(MOTOR_RS_OHM * I_MAX / FLUX_V_BASE))
/* 电感饱和参数：拐点在电流量程内、斜率非负且不溢出、下限在 (0,1] */
CFG_CHECK(FLUX_LS_SAT_SANE, MOTOR_LS_SAT_KNEE_A >= 0 && MOTOR_LS_SAT_KNEE_A < I_MAX && MOTOR_LS_SAT_PER_A >= 0 &&
         MOTOR_LS_SAT_PER_A * I_MAX < 7.0 && MOTOR_LS_SAT_MIN > 0 && MOTOR_LS_SAT_MIN <= 1.0)
/* 逆变器电压误差参数：KT 要有分辨率（> 0 时至少 16 LSB）且 < 0.1 个周期；I0 在电流量程内，斜率不溢出 */
CFG_CHECK(INVERTER_DT_SANE, !INVERTER_DT_COMP || (INVERTER_DT_EFF_S * PWM_FREQ * 32768.0 >= 16 && INVERTER_DT_EFF_S * PWM_FREQ < 0.1 &&
         INVERTER_DT_I0_A > 0.01 && INVERTER_DT_I0_A < I_MAX && 256.0 * I_MAX / INVERTER_DT_I0_A < 60000.0))
/* 起动方式默认值只能取四种之一 */
CFG_CHECK(STARTUP_MODE_VALID, STARTUP_MODE == STARTUP_MODE_IPD || STARTUP_MODE == STARTUP_MODE_DRAG || STARTUP_MODE == STARTUP_MODE_ALIGN ||
         STARTUP_MODE == STARTUP_MODE_SWING)
/* 摆动中起动：扫掠角为 60° 的整数倍且不超过 180°；判定时间不少于半个摆动周期，最长等待的拍数不超过 uint16；
 * 定位电流不大于拖动电流（接入后电流只升不降）；定点量不溢出；
 * 最小峰值转速对应的反电势不少于 8 LSB（检测电平的间隔为其 1/4） */
CFG_CHECK(ALIGN_SWING_FITS, ALIGN_SWING_SWEEP_DEG >= 0 && ALIGN_SWING_SWEEP_DEG <= 180.0 &&
         ALIGN_SWING_SWEEP_DEG / 60.0 == (double)(int)(ALIGN_SWING_SWEEP_DEG / 60.0) &&
         ALIGN_SWING_QUIET_T >= 0.5 && ALIGN_SWING_MAX_T >= 1.5 && ALIGN_SWING_MAX_T * ALIGN_SWING_MAX_T * ALIGN_T2_TICKS2 < 32767.0 * 32767.0 &&
         ALIGN_SWING_MIN_DELTA > 0 && ALIGN_SWING_MIN_DELTA < 1.0 && ALIGN_ID_Q15 <= OPENLOOP_IQ_Q15 &&
         (double)ALIGN_ID_Q15 * ALIGN_SWING_SWEEP_K_Q16 < 2147483647.0 && ALIGN_SWING_LAG_ANG_Q12 < 1000000 &&
         (ALIGN_SWING_MIN_SPD_T * DECOUPLE_K_EMF_Q15_OF(MOTOR_PSI_F_WB) / 32768.0) * (ALIGN_SWING_MIN_SPD_T * DECOUPLE_K_EMF_Q15_OF(MOTOR_PSI_F_WB) / 32768.0) >= 8.0 * 8.0 * ALIGN_T2_TICKS2)
/* 定位后立即加速：拖动前至少等待 1 T（阻尼 0.75 时摆动衰减到 1/10 以下），总拍数不超过 uint16 */
CFG_CHECK(ALIGN_GO_FITS, ALIGN_GO_T >= 1.0 && ALIGN_A / ALIGN_RAMP_A_S * SLOWLOOP_FREQ < 32767.0 &&
         ALIGN_GO_T * ALIGN_GO_T * ALIGN_T2_TICKS2 < 32767.0 * 32767.0)
/* 停机方式默认值只能取两种之一 */
CFG_CHECK(STOP_MODE_VALID, STOP_MODE == STOP_MODE_RAMP || STOP_MODE == STOP_MODE_COAST)
/* IPD 注入电压加偏置电流环的输出不超出线性调制区（额定母线，偏置所需电压按 Rs 计）；拍数、斜坡步长有效；门限合理 */
CFG_CHECK(IPD_FITS, IPD_HF_V > 0 && IPD_HF_V + IPD_BIAS_A * MOTOR_RS_OHM * 2.0 < MAX_DUTY * UDC_RATE * 2.0 / 3.0 &&
         IPD_HF_TICKS >= 64 && IPD_HF_TICKS < 4096 && IPD_MEAS_MS >= 4 && IPD_BIAS_STEP_Q15 >= 1 && IPD_BIAS_A < I_MAX &&
         IPD_MIN_SAL > 0 && IPD_MIN_SAL < 1.0 && IPD_MIN_RATIO > 1.0 && IPD_MIN_RATIO < 10.0)
/* FLUX_WN_HZ 相对 PWM 频率太高 */
CFG_CHECK(FLUX_KTS_FITS, FWC_POS_Q15(2.0 * MATH_PI * FLUX_WN_HZ / PWM_FREQ))
/* FLUX_BUS_MIN_V 不低于额定母线 UDC_RATE：观测器会一直处于复位 */
CFG_CHECK(FLUX_BUS_MIN_BELOW_RATED, FLUX_BUS_MIN_V > 0 && FLUX_BUS_MIN_V < UDC_RATE)

/* ---- FluxPLL（增益字段是 int16） ------------------------------------------ */
/* PLL 增益放不进 int16：一般是 FLUX_NOMINAL_Q15 太小（先过 FLUX_NOMINAL_RESOLUTION）或 FLUXPLL_WN_HZ 太高 */
CFG_CHECK(PLL_GAINS_FIT, FLUXPLL_KP_Q31_PER_ERR <= 32767 && FLUXPLL_KI_Q31_PER_ERR <= 32767)
/* Ki 只有个位数时带宽量化误差达几十 %；SPEED_MAX 很高（数万 rpm）时可能不满足此项，需为 PLL 增加增益移位。 */
/* PLL 增益分辨率不足（Kp < 64 或 Ki < 16）：FLUXPLL_WN_HZ 太低，或 SPEED_MAX 很高 */
CFG_CHECK(PLL_GAINS_RESOLUTION, FLUXPLL_KP_Q31_PER_ERR >= 64 && FLUXPLL_KI_Q31_PER_ERR >= 16)

/* ---- dq 解耦前馈：(Q15*Q15>>15) * K 不能溢出 int32 ------------------------- */
/* SPEED_MAX 时 w*L*I_MAX 超过 UDC_MAX 的 16 倍：SPEED_MAX 或 I_MAX 不合理 */
CFG_CHECK(DECOUPLE_K_FITS, DECOUPLE_K_LQ_Q12 < 65536 && DECOUPLE_K_LD_Q12 < 65536)

/* ---- I/F 启动 ------------------------------------------------------------ */
/* OPENLOOP_CURRENT_A / ALIGN_A 必须在 (0, I_MAX) 内 */
CFG_CHECK(OPENLOOP_CURRENTS_FIT, OPENLOOP_CURRENT_A > 0 && OPENLOOP_CURRENT_A < I_MAX && ALIGN_A > 0 && ALIGN_A < I_MAX)
/* OPENLOOP_ACC_RPM_S 超过拖动电流能产生的最大加速度（MOTOR_ACC_RPM_S_PER_A * OPENLOOP_CURRENT_A），一定失步 */
CFG_CHECK(OPENLOOP_ACC_FEASIBLE, OPENLOOP_ACC_RPM_S < MOTOR_ACC_RPM_S_PER_A * OPENLOOP_CURRENT_A)
/* I/F 加速度或定位电流斜率在每拍截断成 0 */
CFG_CHECK(OPENLOOP_ACC_RESOLUTION, OPENLOOP_ACC_STEP_Q31 >= 1 && ALIGN_STEP_Q15 >= 1)
/* 要求 0 < OPENLOOP_SPIN_MIN_RPM < OPENLOOP_HANDOVER_RPM <= OPENLOOP_FALLBACK_RPM，且 1.5*OPENLOOP_FALLBACK_RPM < SPEED_MAX */
CFG_CHECK(OPENLOOP_SPEED_ORDER, OPENLOOP_SPIN_MIN_RPM > 0 && OPENLOOP_SPIN_MIN_RPM < OPENLOOP_HANDOVER_RPM &&
         OPENLOOP_HANDOVER_RPM <= OPENLOOP_FALLBACK_RPM && 1.5 * OPENLOOP_FALLBACK_RPM < SPEED_MAX)
/* 回退 I/F 的电流：δmax 在 20°～80°；OPENLOOP_CURRENT_A <= OPENLOOP_REV_CURRENT_A <= OPENLOOP_WARM_MAX_A < I_MAX，且不超过速度环限幅 */
CFG_CHECK(OPENLOOP_WARM_FITS, OPENLOOP_WARM_DELTA_DEG >= 20.0 && OPENLOOP_WARM_DELTA_DEG <= 80.0 && OPENLOOP_REV_CURRENT_A >= OPENLOOP_CURRENT_A &&
         OPENLOOP_WARM_MAX_A >= OPENLOOP_REV_CURRENT_A && OPENLOOP_WARM_MAX_A < I_MAX && OPENLOOP_WARM_MAX_A <= SPD_PI_LIMIT &&
         CURRENT_SLEW_STEP_Q15 >= 1 && CURRENT_SLEW_STEP_Q15 < 32767)
/* 缺相：窗口电周期数 >= 1、比值 > 1；窗口累加值、2 s 内的快环拍数不溢出（每拍 |i| <= 32768） */
CFG_CHECK(PHASELESS_FITS, PHASELESS_PERIODS >= 1 && PHASELESS_RATIO > 1 && PHASELESS_ACC_FULL > 0 &&
         PHASELESS_ACC_FULL < 2147483647.0 && PHASELESS_N_MAX * 32768.0 < 4294967295.0)
/* 超速门限在额定转速以上、Q15 量程以内 */
CFG_CHECK(SPEED_OC_FITS, PROTECT_SPEED_OC_RPM > MOTOR_SPEED_RATED_RPM && PROTECT_SPEED_OC_RPM < SPEED_MAX)
/* 定位静止门限 ALIGN_STILL_RPM 换成反电势不到 4 LSB，判不出静止 */
CFG_CHECK(OPENLOOP_STILL_RESOLUTION, ALIGN_STILL_EMF_Q15 >= 4)
/* NLFO：归一化定点有分辨率（>= 64 = 0.25）且 (ferr>>12)*norm 不溢出（|ferr>>12| <= 2^18），时间常数在 8~64 ms 的阶梯里 */
CFG_CHECK(NLFO_FITS, FLUX_NLFO_NORM_Q8 >= 64 && FLUX_NLFO_NORM_Q8 < 8192 && FLUX_NLFO_TAU_S >= 0.004 && FLUX_NLFO_TAU_S <= 0.064)
/* 冷起动平衡负载角：0 < OPENLOOP_ACC_FRAC <= 0.5（δ_eq <= 30°，asin 级数在适用范围内） */
CFG_CHECK(OPENLOOP_ACC_FRAC_RANGE, OPENLOOP_ACC_FRAC > 0 && OPENLOOP_ACC_FRAC <= 0.5)
/* I/F 加速度步长 x 开环电流在 32 位内（startup.c 用 32 位乘除，可能在快环里执行）：最大步长（反转加大）x 最大电流 < 2^31 */
CFG_CHECK(OPENLOOP_ACC_STEP_FITS, (double)OPENLOOP_ACC_STEP_Q31 * OPENLOOP_REV_IQ_Q15 / OPENLOOP_IQ_Q15 * OPENLOOP_WARM_IMAX_Q15 < 2147483647.0 &&
         (double)OPENLOOP_ACC_STEP_Q31 * OPENLOOP_WARM_IMAX_Q15 < 2147483647.0)
/* 慢环频率：整除 PWM 频率（快环每 N 拍挂起一次慢环）；至少为慢环中各环节特征频率的 20 倍：
 * 母线电压环带宽、转速反馈滤波、定位摆（f = 1/T，用 T^2 比较免开方）、S 形斜坡圆角 1/(2*pi*ROUND)；
 * 各一阶滤波窗口至少 2 拍、失速检查至少 1 拍（对应检查项 SLOW_RATE_FITS） */
/* 慢环频率与 PWM 频率及慢环各环节特征频率不匹配（见上方说明） */
CFG_CHECK(SLOW_RATE_FITS, (PWM_FREQ % SLOWLOOP_FREQ) == 0 && SLOWLOOP_FREQ >= 20.0 * BUS_VDC_BW_HZ &&
         SLOWLOOP_FREQ >= 20.0 * SPEED_FB_FILT_HZ && (double)SLOWLOOP_FREQ * SLOWLOOP_FREQ * ALIGN_T2_S2 >= 400.0 &&
         SLOWLOOP_FREQ >= 20.0 / (2.0 * MATH_PI * SPEED_RAMP_ROUND_S) && MEAS_VBUS_FILT_SH >= 1 && MEAS_IBUS_FILT_SH >= 1 &&
         OPENLOOP_PLL_FILT_SH >= 1 && ALIGN_EMF_FILT_SH >= 1 && SPIN_CHECK_TICKS >= 1)
/* 速度斜坡的定点位宽（speed_ctrl.c 制动距离 / 母线电压控制器，均不使用 64 位除法和开方）：
 * 制动距离 (c^2 >> 16) * 2^32/(2j) < 2^63，c 取最大斜率（反转 SPEED_RAMP_REV），c < 2^24（c^2 >> 16 放进 32 位，32x32 乘法）；2^32/(2j) 在 32 位内；
 * 母线电压控制器 (e2*K >> 16) * 2^30/SMIN < 2^64，SMIN > 0 */
CFG_CHECK(RAMP_FIXEDPOINT_FITS, SPEED_JERK_Q31 >= 1 && 4294967296.0 / (2.0 * SPEED_JERK_Q31) < 4294967295.0 &&
         SPEED_RAMP_REV / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 < 16777216.0 &&
         SPEED_RAMP_UP / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 <= SPEED_RAMP_REV / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 &&
         SPEED_RAMP_DOWN / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 <= SPEED_RAMP_REV / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0 &&
         (SPEED_RAMP_REV / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0) * (SPEED_RAMP_REV / SPEED_MAX / SLOWLOOP_FREQ * 2147483648.0)
           / 65536.0 * (4294967296.0 / (2.0 * SPEED_JERK_Q31)) < 9.2e18 &&
         (double)BUS_VDC_REF_SQ * BUS_VDC_K_Q8 / 65536.0 * (1073741824.0 / BUS_VDC_SMIN_Q15) < 1.8e19 && BUS_VDC_SMIN_Q15 > 0)
/* I/F 各段时间换成慢环拍数超过 65535（uint16 回绕） */
CFG_CHECK(OPENLOOP_TICKS_FIT, ALIGN_MAX_T * ALIGN_MAX_T * ALIGN_T2_S2 * SLOWLOOP_FREQ * SLOWLOOP_FREQ < 65535.0 * 65535.0 &&
         ALIGN_MIN_T > 0 && ALIGN_STILL_T >= 0.3 && ALIGN_MAX_T > ALIGN_MIN_T + ALIGN_STILL_T && FLY_SETTLE_BLOCKS >= 1 &&
         OPENLOOP_TIMEOUT_S * SLOWLOOP_FREQ < 65536 && OPENLOOP_AGREE_S * SLOWLOOP_FREQ < 65536)

/* 直接拖动：牵入频率 f0 低于 I/F 摆动频率（0 < DRAG_F0_RATIO < 1）、牵入段至少转一圈、拍数不超过 uint16；
 * 电压型驱动的电流积分带宽须远低于摆动频率（否则积分又将反电势抵消，失去阻尼）且不超过正常电流环带宽 */
CFG_CHECK(DRAG_FITS, DRAG_F0_RATIO > 0 && DRAG_F0_RATIO < 1 && DRAG_HOLD_T >= 1.0 / DRAG_F0_RATIO &&
         DRAG_T2_TICKS2 * DRAG_HOLD_T * DRAG_HOLD_T < 65535.0 * 65535.0 &&
         DRAG_CUR_BW_HZ > 0 && DRAG_CUR_BW_HZ * DRAG_CUR_BW_HZ * DRAG_T2_S2 < 0.25 * 0.25 && DRAG_CUR_BW_HZ < CURRENT_BW_HZ)

/* ---- 飞车检测 ------------------------------------------------------------ */
/* 短脉冲测速：脉冲周期至少 4 拍（SPEED_MAX 时 120° 电角度换成的拍数太少：PWM 太低或 SPEED_MAX 太高） */
CFG_CHECK(FLY_PULSE_PERIOD_FITS, FLY_PULSE_PERIOD_RAW >= 4.0 && FLY_PULSE_N >= 3)
/* 接管角推进 (pulseLen + 2*since) * ADV_K 放进 int32（32x32 乘法）：pulseLen < 周期，since <= 周期 + 3 */
CFG_CHECK(FLY_PULSE_ADV_FITS, (3.0 * FLY_PULSE_PERIOD_TICKS + 6.0) * FLY_PULSE_ADV_K < 2147483647.0)
/* 脉冲电流门限：噪声下限 >= 1 码且门限小于 I_MAX 的一半；种子角度精度 (0, 30°]；最短脉冲拍数在 [1, 最长) 内 */
CFG_CHECK(FLY_PULSE_CURRENT_FITS, BOARD_ADC_NOISE_CODES >= 1.0 && BOARD_ADC_NOISE_CODES * FLY_PULSE_ITH_PER_CODE < FLY_PULSE_ITH_MAX_Q15 &&
         FLY_SEED_ANGLE_DEG > 0 && FLY_SEED_ANGLE_DEG <= 30.0 && FLY_PULSE_MIN_TICKS >= 1 && FLY_PULSE_MIN_TICKS < FLY_PULSE_MAX_TICKS)
/* 连续短接测量按 1 ms 分块，PWM_FREQ 要是 1 kHz 的整数倍（例如 31.5 kHz 不行） */
CFG_CHECK(FLY_PWM_MULTIPLE_OF_1K, (PWM_FREQ % 1000) == 0)
/* 零电流跟踪结束转速：观测器在电动工况的接管下限（OPENLOOP_HANDOVER_RPM）~ 飞车直接接管门限之间 */
CFG_CHECK(TRACK_END_RANGE, TRACK_END_RPM >= OPENLOOP_HANDOVER_RPM && TRACK_END_RPM <= FLY_CATCH_RPM)
/* 要求 0 < 静止门限 FLY_STILL_RPM < FLY_LOW_MIN_RPM < FLY_CATCH < SPEED_MAX */
CFG_CHECK(FLY_SPEED_ORDER, FLY_CATCH_RPM > 0 && FLY_CATCH_RPM < SPEED_MAX &&
         FLY_STILL_RPM > 0 && FLY_LOW_MIN_RPM > FLY_STILL_RPM && FLY_LOW_MIN_RPM < FLY_CATCH_RPM)
/* 三相短接（飞车测速的退回路径、低速刹停）时的短路电流上限 ≈ ψf / L（高速极限），不能超过电流量程 I_MAX，
 * 否则短接就会过流（ψf 大、L 小的电机）。本电机 3.1 A < 7.96 A */
CFG_CHECK(SHORT_CIRCUIT_FITS, MOTOR_PSI_F_WB / MOTOR_LD_H < 0.9 * I_MAX)
/* 等停稳：残余转速要低于静止门限转速，且等待时间合理（< 刹停超时、< 60 s） */
CFG_CHECK(FLY_STILL_END_FITS, FLY_STILL_END_RPM > 0 && FLY_STILL_END_RPM < FLY_STILL_RPM &&
         FLY_BRAKE_TAU_S > 0 && FLY_BRAKE_TAU_S * 5.0 < FLY_BRAKE_TIMEOUT_S)
/* SPEED_MAX 时 w*Lq/Rs > 16：阻抗角定点放不下（电感大、电阻小的高速电机） */
CFG_CHECK(FLY_WLR_FITS, FLY_WLR_Q12 < 65536)
/* FLYCURRENT_TH_PU 必须在 (0, 1) 内 */
CFG_CHECK(FLY_CURRENT_TH_FITS, FWC_POS_Q15(FLYCURRENT_TH_PU))

/* ---- 保护 ---------------------------------------------------------------- */
/* 过压门限 DCBUS_OVER 要在额定母线 UDC_RATE 和采样满量程 UDC_MAX 之间 */
CFG_CHECK(OVP_IN_RANGE, DCBUS_OVER > UDC_RATE && DCBUS_OVER < UDC_MAX)
/* 母线门限排布（由过压门限往下推）：三档都在额定母线和过压门限之间、依次递增；定点后 START < FULL；
 * 制动 id 在 (0, I_MAX) 内 */
CFG_CHECK(BUS_REGEN_FITS, BUS_DV_DELAY_V > 0 && BUS_VDC_REF_V > UDC_RATE && BUS_REGEN_FULL_V < DCBUS_OVER &&
         BUS_REGEN_FULL_Q15 > BUS_REGEN_START_Q15 && BUS_REGEN_BRAKE_A > 0 && BUS_REGEN_BRAKE_A < I_MAX)
/* 调节点高于输入上限的空载母线：否则输入偏高时母线没有储能裕量，减速 / 换向指令执行不了。
 * 不满足：加母线电容、降 SPD_PI_LIMIT（减小 dV_d），或提高过压门限 */
CFG_CHECK(BUS_VDC_ABOVE_GRID, BUS_VDC_REF_V > BUS_IDLE_MAX_V)
/* 母线电压控制器：带宽不超过慢环频率的 1/50（母线滤波、离散化可忽略）；
 * 增益定点分辨率够（K >= 1，256 = 1.0）且不溢出；电容 > 0；受限程度门限在 (0, 1] */
CFG_CHECK(BUS_VDC_FITS, BOARD_CBUS_UF > 0 &&
         BUS_VDC_BW_HZ > 0 && BUS_VDC_BW_HZ * 50.0 <= SLOWLOOP_FREQ && BUS_VDC_K_Q8 >= 256 && BUS_VDC_K_Q8 < 2147483647 &&
         BUS_VDC_FILT_Q15 >= 1 && BUS_VDC_FILT_Q15 < 32768 && BUS_BRAKE_FULL_LAMBDA > 0 && BUS_BRAKE_FULL_LAMBDA <= 1.0)
/* 接管同步时间：1 拍 ~ 0.5 s（PLL 带宽太低时同步太久，滑行掉速太多） */
CFG_CHECK(FLY_SYNC_FITS, FLY_SYNC_TICKS >= 1 && FLY_SYNC_S <= 0.5)
/* 软停机步长 >= 1（分辨率）且最长停机过程（速度环限幅 SPD_PI_LIMIT 收到 0）不超过 2 s */
CFG_CHECK(SOFT_STOP_FITS, SOFT_STOP_STEP_Q15 >= 1 && SPD_PI_LIMIT / I_MAX * 32768.0 / SOFT_STOP_STEP_Q15 <= 2.0 * SLOWLOOP_FREQ)
/* 斜坡圆角：jerk 步长 >= 16（分辨率），圆角时间在 (0, 2] s */
CFG_CHECK(SPEED_ROUND_FITS, SPEED_RAMP_ROUND_S > 0 && SPEED_RAMP_ROUND_S <= 2.0 && SPEED_JERK_Q31 >= 16)
/* 斜率前馈增益分辨率：至少 64（约 1.5%） */
CFG_CHECK(SPEED_FF_K_FITS, SPEED_FF_K_Q16 >= 64 && SPEED_FF_K_Q16 < 1000000 &&
         SPEED_ACC_PER_IQ_Q8 >= 256 && SPEED_ACC_PER_IQ_Q8 < 32768 * 256)
/* 转速反馈滤波拐点：低于 PLL 带宽（不和 PLL 谐振闭环），且高于速度环带宽 5 倍（相位滞后可忽略） */
CFG_CHECK(SPEED_FB_FILT_FITS, SPEED_FB_FILT_REAL_HZ < FLUXPLL_WN_HZ && SPEED_FB_FILT_REAL_HZ > 5.0 * SPEED_WN_HZ &&
         SPEED_FB_FILT_TICKS < 128.0)
/* I/F 切换融合时间：步长 >= 1 且 < 32767（融合至少两拍，最长约 32 s） */
CFG_CHECK(OPENLOOP_BLEND_FITS, OPENLOOP_BLEND_STEP_Q15 >= 1 && OPENLOOP_BLEND_STEP_Q15 < 32767)
/* IBUS_LIMIT_A 必须在 (0, I_MAX) 内 */
CFG_CHECK(IBUS_LIMIT_FITS, IBUS_LIMIT_A > 0 && IBUS_LIMIT_A < I_MAX)

/* ---- 弱磁 ---------------------------------------------------------------- */
#if CTRL_WEAKEN_ENABLE
/* 弱磁积分增益截断成 0，或 WEAKEN_ID_MIN_A 不在 (-I_MAX, 0) 内 */
CFG_CHECK(WEAKEN_GAINS_FIT, WEAKEN_KI_Q20 >= 1 && WEAKEN_ID_MIN_A < 0 && WEAKEN_ID_MIN_A > -I_MAX)
#endif
