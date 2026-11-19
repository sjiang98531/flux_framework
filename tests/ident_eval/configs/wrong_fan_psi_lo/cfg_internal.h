/* ============================================================================
 * @file    cfg_internal.h
 * @brief   内部控制规格（物理单位），正常使用无需修改
 *
 * 配置分三层：电机 cfg_motor.h、应用 cfg_app.h、内部（本文件），另有驱动板 cfg_board.h（见 cfg_app.h 文件头）。
 * 本文件描述"期望的控制效果"：带宽（Hz）、阻尼比、时间常数（s）、额定值的倍数，全部由电机参数、驱动板参数
 * 和应用层配置自动换算；定点增益由 cfg_derived.h 计算。更换电机或应用时无需修改本文件。
 * 已移至 cfg_app.h 的宏不再在本文件重复说明，其应用参数与转速估算方法见 cfg_app.h。
 *
 * 参数分三类：
 *   A 类  物理输入：实测值或器件手册值（主要在 cfg_motor.h / cfg_board.h）；
 *   B 类  设计指标及其推导量：带宽、阻尼、时间常数，以及由 A/B 类推出的量；
 *   C 类  经验值：暂无物理推导，已在对应注释中标明（POWER_KP / POWER_KI、PRECHARGE_TIME）。
 * ==========================================================================*/
#ifndef CFG_INTERNAL_H
#define CFG_INTERNAL_H

/* ===================== 定标 ============================================== */
/* 速度标幺基准：Q15 速度 32767 对应 SPEED_MAX rpm，同时也是 PLL 速度上限（s32Speed 在 2^31 处饱和）。
 * 仅为定标量，须高于超速保护转速（配置检查 SPEED_OC_FITS）；取机械允许上限 / 0.9 自动留出 10% 余量。 */
#define SPEED_MAX               ((MOTOR_SPEED_LIMIT_RPM / 0.9) > 3200.0 ? (MOTOR_SPEED_LIMIT_RPM / 0.9) : 3200.0)
/* 极对数别名（Board/ 下的板级代码使用） */
#define POLEPAIRS               MOTOR_POLE_PAIRS
/* 电感饱和模型的下限（相对 MOTOR_LQ_H）：割线电感 L(i) 最低降到该倍数（拐点与斜率为电机参数，见 cfg_motor.h） */
#define MOTOR_LS_SAT_MIN        0.5

/* ===================== 电流环 ============================================ */
/* 零极点对消设计：Kp = 2*pi*BW*L，Ki = 2*pi*BW*Rs（d 轴用 Ld，q 轴用 Lq）。
 * 带宽选取：
 *   上限：数字延时约 1.5 个 PWM 周期（94 us），300 Hz 时相位裕量约 80°（800 Hz 仍有 60°）；
 *   下限：应高于最高电频率（2600 rpm 时 217 Hz），并比 PLL（15 Hz）快 10 倍以上。
 *   取 300 Hz（L 43.3 mH、Rs 7.35 Ω 时 Kp 44843、Ki 475）。
 * 快环 PI 采用 16 位增益格式，要求 Kp、Ki <= 65535，本电机约 438 Hz 封顶。电流 Q15 基准为硬件满量程
 * I_MAX（7.96 A）；如需更高带宽，可给增益增加移位（Kp*err >> 14、误差限 ±16383），无需改变电流基准。 */
#define CURRENT_BW_HZ           300.0
/* 角度时序补偿：Park 变换使用采样时刻 k 的估计角；反 Park 变换使用 k + CTRL_INVPARK_ADVANCE 拍的角度
 * （本拍计算的电压在下一个 PWM 周期生效，作用区间中点位于 k+1.5）。 */
#define CTRL_INVPARK_ADVANCE    1.5         /* PWM 周期 */
/* ===================== 速度环 ============================================ */
/* 被控对象 d(rpm)/dt = MOTOR_ACC_RPM_S_PER_A * iq，PI 闭环为二阶系统：
 *   Kp = 2*zeta*wn / b，Ki = wn^2 / b（b = MOTOR_ACC_RPM_S_PER_A）。
 * 给定经 S 形斜坡并带轨迹前馈，超调可忽略；取 zeta = 0.7、wn = 0.8 Hz（低于反馈滤波拐点 5 倍以上，
 * 且避免刚切入闭环低速段经 iq -> 观测器 -> PLL 耦合自激）。Kp/Ki 超过 65535，采用 64 位 PI。 */
#define SPEED_WN_HZ             0.8
#define SPEED_ZETA              0.7
/* 转速反馈滤波（gMeas.speed 一阶滑动平均，慢环执行）拐点：须明显低于 PLL 带宽 FLUXPLL_WN_HZ；
 * 实际窗口取 2^n 拍（cfg_derived.h SPEED_FB_FILT_SH）。 */
#define SPEED_FB_FILT_HZ        (0.35 * FLUXPLL_WN_HZ)      /* 本电机 5.25 Hz -> 32 拍（实际 4.97 Hz） */

/* 母线电流限（作用于速度环输出、只向下限制的 PI）。C 类经验值。 */
#define POWER_KP                (14000)
#define POWER_KI                (140)
/* 母线电流限（额定输入电流 P/UDC_RATE 约 0.32 A）。 */
#define IBUS_LIMIT_A            3.5         /* A */

/* ===================== 斜坡 ============================================== */
/* 反转过零前的减速斜率。同向调速见 cfg_app.h；反转先减速到 OPENLOOP_FALLBACK_RPM 再交给 I/F。 */
#define SPEED_RAMP_REV          (2.0 * SPEED_RAMP_DOWN)     /* rpm/s，本电机 400 */
/* 斜坡圆角：限制给定轨迹的加加速度（jerk），使加速度从 0 升至 SPEED_RAMP_UP 用时 SPEED_RAMP_ROUND_S（S 形曲线）。 */
#define SPEED_RAMP_ROUND_S      0.1         /* s */
/* 电流矢量最大变化率：等于 S 形斜坡最大转矩变化率，下限取 SPD_PI_LIMIT / 1.5（防大惯量电机变化过慢），
 * 用于软停机、回退 I/F 撤除制动 id 及 I/F 电流升至目标值。 */
#define CURRENT_SLEW_JERK_A_S   (SPEED_RAMP_UP / MOTOR_ACC_RPM_S_PER_A / SPEED_RAMP_ROUND_S)
#define CURRENT_SLEW_MIN_A_S    (SPD_PI_LIMIT / 1.5)
#define CURRENT_SLEW_A_S        (CURRENT_SLEW_JERK_A_S > CURRENT_SLEW_MIN_A_S ? CURRENT_SLEW_JERK_A_S : CURRENT_SLEW_MIN_A_S)

/* ===================== 起动：IPD / 摆动起动 + I/F 开环拖动 =============== */
/* 初始位置检测（IPD，ipd.h）：静止时高频方波注入测凸极 d 轴，再缓慢加 ±IPD_BIAS_A 直流偏置判 N 极。
 * 凸极比 < IPD_MIN_SAL 或极性对比度 < IPD_MIN_RATIO 时失败，本次自动退回摆动起动。 */
#define IPD_HF_A                (0.1 * MOTOR_I_RATED_A)     /* A，注入电流三角波幅值（本电机 0.075 A） */
#define IPD_HF_S                0.016       /* s，每个注入方向 / 每个偏置测量窗口的时长 */
#define IPD_BIAS_A              (0.8 * MOTOR_I_RATED_A)     /* A，判极性的直流偏置（本电机 0.6 A） */
#define IPD_RAMP_S              0.025       /* s，偏置电流从 0 升到 IPD_BIAS_A 的斜坡时间 */
#define IPD_MIN_SAL             0.05        /* 凸极比 |1/Ld - 1/Lq| / (1/Ld + 1/Lq) 的下限（本电机约 0.19） */
#define IPD_MIN_RATIO           1.1         /* 极性对比度（两侧 d 轴响应之比）的下限 */

/* 定位电流（摆动起动与 IDENT_TEST = 1 静止参数辨识共用）：单摆周期 T = 2*pi*sqrt(J/K) 由 param.c 动态计算。 */
#define ALIGN_A              (0.8 * MOTOR_I_RATED_A)     /* A，本电机 0.6 A */
#define ALIGN_RAMP_A_S       (3.8 * (MOTOR_I_RATED_A / 0.75))     /* A/s，定位电流上升斜率（按额定电流缩放，本电机 3.8 A/s） */
/* 以下三项及阻尼比 ALIGN_ZETA 仅在 IDENT_TEST = 1 静止参数辨识定位等停稳时使用（startup.c） */
#define ALIGN_MIN_T             1.6         /* T，最短定位时间（等待离开死点） */
#define ALIGN_STILL_T           0.4         /* T，低于 ALIGN_STILL_RPM 并持续该时间判为停稳 */
#define ALIGN_MAX_T             4.0         /* T，最长定位时间，到时直接开始测量 */
#define ALIGN_ZETA           0.75        /* 定位阻尼比（兼顾摆动衰减与 Rs/死区估计误差稳定裕量） */
/* 摆动中起动（STARTUP_MODE_SWING，详见 startup.c alignSwingSlow）：不加阻尼、不等停稳，
 * 在 d 轴电流升至 ALIGN_A 并扫掠 ALIGN_SWING_SWEEP_DEG 后，利用 |e_q| 过轴峰值及两侧 vd 差值解算位置与转速，
 * 瞬间接入 I/F 平衡点并按顺向 / 逆向过轴对称起动；无明显摆动或超时时按静止在定位轴上接入。 */
#define ALIGN_SWING_SWEEP_DEG   180.0       /* 电角度，电流上升过程中电流矢量扫过的角度（60° 的整数倍，起止均在相轴上，<= 180°） */
#define ALIGN_SWING_MIN_DELTA   0.5         /* 可接入的最小摆动：带入 I/F 后的负载角摆幅 / 平衡负载角 */
#define ALIGN_SWING_QUIET_T     0.5         /* T，判定"没有可测的摆动"的时间（任何摆动在半个周期内必出现一次速度峰值） */
#define ALIGN_SWING_MAX_T       3.0         /* T，等待顺向过轴的最长时间（大摆幅的周期为小摆幅的数倍） */
/* I/F 开环拖动：加速度取拖动电流最大能力的 1/3（平衡负载角 δ_eq = asin(OPENLOOP_ACC_FRAC) ≈ 19°），按惯量自动缩放。 */
#define OPENLOOP_ACC_FRAC             0.33        /* I/F 加速度 / 拖动电流可产生的最大加速度（余量 3 倍）= sin(平衡负载角) */
#define OPENLOOP_ACC_RPM_S            (OPENLOOP_ACC_FRAC * MOTOR_ACC_RPM_S_PER_A * OPENLOOP_CURRENT_A)   /* rpm/s（本电机 124 rpm/s） */
/* Spin -> I/F 回退（热接管）开环电流：保证负载角 <= OPENLOOP_WARM_DELTA_DEG；过零反转至少 OPENLOOP_REV_CURRENT_A。 */
#define OPENLOOP_WARM_DELTA_DEG       45.0
#define OPENLOOP_REV_CURRENT_A        (OPENLOOP_CURRENT_A / 0.75)       /* A，本电机 1.0 A */
#define OPENLOOP_WARM_MAX_A           (2.0 * MOTOR_I_RATED_A)     /* A，本电机 1.5 A */
/* Spin 失锁检测周期（C 类经验值）：指令同向而估计转速低于 SPIN_LOST_RPM 时每 SPIN_CHECK_S 检查一次 */
#define SPIN_CHECK_S            0.010       /* s */
/* 切换融合时间：切闭环后 id 从 I/F 分量渐变到 0、加速前馈渐入所用时间，取速度环时间常数 */
#define OPENLOOP_BLEND_S              (1.0 / (2.0 * 3.14159265359 * SPEED_WN_HZ))     /* 本电机 0.20 s */
/* I/F 切闭环一致判据与堵转超时（C 类经验值） */
#define OPENLOOP_AGREE_RATIO          0.15
#define OPENLOOP_AGREE_S              0.1         /* s */
#define OPENLOOP_TIMEOUT_S            2.0         /* s */

/* ===================== 起动公共参数 ====================================== */
#define PRECHARGE_TIME          20          /* ms，自举电容预充时间（C 类，应按自举电容 RC 及驱动芯片要求确定） */
#define STARTUP_SPEED_RAMP      60.0        /* rpm/s，OpenLoop（I/F）期间上报给斜坡状态的斜率（开环转速由 I/F 自行控制） */

/* ===================== 顺逆风（飞车）检测 ================================ */
/* 先行短脉冲测速；测不到或不一致时退回连续短接测量（flystart.c）：
 * 转速足够进 Spin（先零电流同步再闭环），较低进 I/F，更低则短接制动判静止后进摆动起动 / IPD。 */
#define FLYBRAKE_TIME           0.1         /* s，制动后判静止的基准时间（检测到过转动时参与 gFly.stillSpinMs 计算） */
#define FLY_COLD_STILL_TIME     0.03        /* s，冷起动 / 电流已低于残余转速门限时的低电流确认时间 */
/* 上次 Run 结束到本次起动间隔超过 FLY_MAYMOVE_EXPIRE_TAUS * tau（制动时间常数 J/B）即认为已停稳，跳过最坏情况等待 */
#define FLY_MAYMOVE_EXPIRE_TAUS  5.0
/* "在转"短路电流门限（标幺，相对 I_MAX）：低速近似 反电势(FLY_STILL_RPM) / Rs，仅与电机参数有关 */
#define FLYCURRENT_TH_PU        (FLY_STILL_RPM * 2.0 * 3.14159265358979 / 60.0 * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB \
                                 / MOTOR_RS_OHM / I_MAX)   /* 标幺（相对 I_MAX）：FLY_STILL_RPM 下的短路电流 */
#define CALIPWMPERCENT          (0.99)      /* 短接时下桥占空比 */
#define FLY_CATCH_RPM           OPENLOOP_FALLBACK_RPM   /* rpm，同向直接接入闭环的下限（观测器在此以上可靠） */
/* 短接后等待电流暂态衰减（约 5 个实际 L/R 时间常数）再开始测量 */
#define FLY_SETTLE_TAUS         10.0        /* x L/R（本电机 31 ms） */
#define FLY_STABLE_MS           8           /* 连续该次数（每次 1 ms）转速估计稳定才接管 */
/* 短脉冲测速：施加 FLY_PULSE_N 个下桥短接脉冲（最短 FLY_PULSE_MIN_TICKS 拍），利用电感积分散度 -∫e dt / L 测速与定角；
 * 最小触发电流按实测 ADC 噪声下角度误差 <= FLY_SEED_ANGLE_DEG 自适应计算（与采样噪声同步缩放）。 */
#define FLY_SEED_ANGLE_DEG      5.0
#define FLY_PULSE_MIN_TICKS     2
#define FLY_PULSE_N             6
#define FLY_BRAKE_TIMEOUT_S     10.0        /* s，短接制动超过该时间仍未停止（如持续受风）-> 故障 */

/* ===================== 回馈减速：母线电压控制 ============================ */
/* 无制动电阻时，通过控制斜坡减速率使允许回馈功率 P = Kv * e（e = C/2 * (Vref^2 - V^2)）与损耗平衡，
 * 负载转矩贡献的减速度由 PI 输出估计并叠加，母线高于 Vref 时仍按负载能力减速（详见 speed_ctrl.c）。
 * 门限排布（cfg_derived.h）：从过压保护 BOARD_DCBUS_OVP_V 向下每档间隔 dV_d（Td 内最大回馈压升）：
 *   FULL = OVP - dV_d（回馈 iq 兜底归零），START = FULL - dV_d（兜底比例带起点），REF = START - dV_d（Vref 调节点）。
 * 对大功率电机，单档压升自动封顶在 BUS_DV_MAX_V = 0.9*(OVP - BUS_IDLE_MAX_V)/3 以确保 REF 始终高于空载母线上限，
 * 并按 BUS_DV_MAX_V / dV_raw 同步按比例下调回馈方向最大电流 BUS_REGEN_IQ_MAX_A（加速方向仍保持 SPD_PI_LIMIT）。 */
#define BUS_VDC_BW_HZ           5.0         /* Hz，母线电压环带宽 */
/* 损耗制动：减速受母线限制时叠加负 id（产生铜损消耗动能），lam >= BUS_BRAKE_FULL_LAMBDA 给满 BUS_REGEN_BRAKE_A */
#define BUS_REGEN_BRAKE_A       (1.0 * MOTOR_I_RATED_A)     /* A，本电机 0.75 A */
#define BUS_BRAKE_FULL_LAMBDA   0.25        /* C 类经验值 */

/* ===================== 慢环一阶滤波时间常数 ============================== */
/* 以时间常数给出，移位位数按慢环频率自动推导（cfg_derived.h）。 */
#define MEAS_VBUS_FILT_TAU_S    0.004       /* s，母线电压（计入回馈测量延迟 Td，须受三档压升预算约束） */
#define MEAS_IBUS_FILT_TAU_S    0.008       /* s，母线电流（母线电流限） */
#define OPENLOOP_PLL_FILT_TAU_S       0.016       /* s，I/F 拖动时的 PLL 转速（用于与开环转速比较一致性） */
#define ALIGN_EMF_FILT_TAU_S 0.008       /* s，定位 / 摆动起动使用的 q 轴反电势 */

/* ===================== 保护 ============================================== */
#define DCBUS_OVER              (BOARD_DCBUS_OVP_V)         /* V，由硬件决定，见 cfg_board.h */
#define DCBUS_UNDER             (0.0)       /* V */
#define DCBUS_FILT_S            0.1         /* s，母线越限持续该时间才报故障 */
#define FAULTRELEASE_TIME       0.0         /* s */
/* 缺相检测（Spin、I/F 共用，protect.h）：窗口取 PHASELESS_PERIODS 个电周期，最大相均流 > PHASELESS_RATIO 倍最小相判缺相 */
#define PHASELESS_PERIODS       4
#define PHASELESS_RATIO         3

/* ===================== 磁链观测器 ======================================== */
/* 定点基准 FLUX_V_BASE / FLUX_WN_HZ / FLUX_PSI_N 由电机与驱动板参数自动计算（cfg_derived.h、param.c）。 */
#define FLUX_BUS_MIN_V          100.0       /* V，母线低于该值时观测器保持复位 */
#define FLUX_HPF_TAU_S          0.256       /* s，反电势去直流一阶滑动平均时间常数（window 见 cfg_derived.h） */
/* NLFO 径向修正时间常数：|psi| 偏离 |psi_f| 后按该时间常数拉回（cfg_derived.h 换算为移位 + 归一化） */
#define FLUX_NLFO_TAU_S         0.01917
#define FLUX_NLFO_FQ_LIMIT      32768       /* NLFO 修正量防溢出饱和限（Q15） */

/* ===================== FluxPLL =========================================== */
/* 二阶锁相环设计目标（运行时 param.c 按实际 fluxNomQ15 归一化）：比速度环快 ~20 倍、比电流环慢 ~20 倍 */
#define FLUXPLL_WN_HZ           15.0
#define FLUXPLL_ZETA            0.707

/* ===================== MTPA / 弱磁 ======================================= */
/* 默认均关闭（关闭时 id_ref 模块不编译）；用于 IPM（MOTOR_IPM=1）或高速扩速场合时打开，详见 id_ref.h */
#define CTRL_MTPA_ENABLE        0
#define CTRL_WEAKEN_ENABLE          0
#define WEAKEN_VOLTAGE_MARGIN       0.08        /* 电压裕量 */
#define WEAKEN_KI_A_PER_S           50.0        /* 100% VMAX 电压误差下的弱磁积分速率（A/s） */
#define WEAKEN_ID_MIN_A             (-2.0)      /* A，弱磁 id 下限（受退磁电流、效率约束） */

#endif
