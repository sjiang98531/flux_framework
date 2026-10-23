/* ============================================================================
 * cfg_ctrl.h —— 控制规格（物理单位）
 *
 * 写"想要的效果"（带宽 Hz、阻尼、rpm/s、A、秒），定点增益由 fw_config.h 算。
 * 从 0xF20E0003 起，各环按理论取值（不再沿用原厂）。每一项都注明了原厂/test33
 * 的值；把它们全部改回去（tools/sim/legacy_cfg.sh 做的就是这件事）就与 test33
 * 逐位相同。
 * ==========================================================================*/
#ifndef CFG_CTRL_H
#define CFG_CTRL_H

/* ===================== 外环选择（原厂结构） ============================== */
#define TORQUE_LOOP             (0)
#define SPEED_LOOP              (1)
#define POWER_LOOP              (2)
#define CLOSE_LOOP              SPEED_LOOP

/* 启停门限 */
#define STOP_TO_RUN_IQ          (0.3)       /* A    （TORQUE_LOOP） */
#define RUN_TO_STOP_IQ          (0.2)
#define STOP_TO_RUN_SPEED       15.0        /* rpm  （SPEED_LOOP） */
#define RUN_TO_STOP_SPEED       12.0
#define STOP_TO_RUN_POWER       3.0         /* W    （POWER_LOOP） */
#define RUN_TO_STOP_POWER       2.0

/* ===================== PI 实现 ============================================ */
/* 1 = 用自写的防溢出 PI（ctrl_pi.h）代替原厂闭源 CalcPI / Fast_Boot_Current_Controller。
 *     正常范围内逐位相同；增益按理论加大后，原厂 32 位乘法会溢出，必须用自写的。
 * 0 = 原厂库（只在下面的增益都很小时安全） */
#define CTRL_OWN_PI             1

/* ===================== 电流环 ============================================ */
/* 零极点对消设计：Kp = 2*pi*BW*L，Ki = 2*pi*BW*Rs（d 轴用 Ld，q 轴用 Lq）。
 * 理论取值：
 *   上限：数字延时约 1.5 拍 = 94 us，300 Hz 时相位裕量约 80°（800 Hz 仍有 60°）
 *   下限：要高于最高电频率（2600 rpm 时 217 Hz），并比 PLL（15 Hz）快 10 倍以上
 *   -> 300 Hz（Kp 47639、Ki 951；快环 PI 要求 Kp、Ki <= 65535，约 410 Hz 封顶）。
 * 原厂/test33：86.573 Hz（13747/274，Ki/Kp 正好等于 Rs/L——原厂也是零极点对消，
 * 只是带宽只比电机自身极点 R/L ≈ 51 Hz 快一点，高速和弱磁时不够）。 */
#define CURRENT_BW_HZ           300.0
#define CURRENT_OUTMAX          (3000)      /* 原厂初值（运行中被 VMAX 覆盖） */
/* dq 解耦前馈（只在 Spin 里）：vd_ff = -w*Lq*iq，vq_ff = w*(Ld*id + psi_f)。
 * 电流环只需要处理残差，高速/弱磁时 d、q 不再互相拖。进出 Spin 时把前馈量
 * 搬进/搬出积分器，无扰切换。0 = 原厂（无前馈）。 */
#define CTRL_DQ_DECOUPLE        1

/* ===================== 速度环 ============================================ */
/* 被控对象 d(rpm)/dt = MOTOR_ACC_RPM_S_PER_A * iq，PI 闭环成二阶：
 *   Kp = 2*zeta*wn / b，Ki = wn^2 / b（b = MOTOR_ACC_RPM_S_PER_A）。
 * 理论取值：zeta = 0.7（过冲约 5%）；带宽受 PLL（15 Hz）、转速滤波（8 ms）限制，
 * 上限约 3 Hz，风机负载平稳取 0.5 Hz（Kp 115582、Ki 259，需要 CTRL_OWN_PI = 1）。
 * 原厂/test33：0.3105 Hz / 0.078025（8000/100）—— 阻尼极小，台架上 3.2 s 周期
 * 振荡、+80~110 rpm 过冲就是它（smo_baseline 同样如此）。
 * 仿真（tools/sim/eval.py）：1172->600 rpm 下冲 71 -> 28 rpm，调节时间 11.8 -> 3.7 s；
 * 1000 rpm 突加 0.5 N·m：掉速 103 -> 35 rpm，恢复 7.4 -> 0.7 s；80 rpm 不再回退 Startup。 */
#define SPEED_WN_HZ             0.5
#define SPEED_ZETA              0.7
#define SPD_PI_LIMIT            (4.0)       /* A，速度环输出限幅 */

/* 加速度前馈（test30，非原厂）：斜坡离开 0 的方向上加 iq_ff = 斜坡率 / b，
 * 让 PI 只负责负载电流。减速不加（原厂行为，不额外制动）。 */
#define SPEED_ACC_FF_ENABLE     (1)

/* 功率环（POWER_LOOP 主环 / SPEED_LOOP 时作母线电流限）——原厂整定值，未物理化 */
#define POWER_KP                (14000)
#define POWER_KI                (140)
#define IBUS_LIMIT_A            (3.5)       /* SPEED_LOOP 下的母线电流限 */
#define POWER_LIMIT_SPEED_RPM   (3000.0)    /* POWER_LOOP 下的限速 */

/* ===================== 斜坡 ============================================== */
#define TORQUE_RAMP_UP          0.5         /* A/s */
#define TORQUE_RAMP_DOWN        0.5
#define SPEED_RAMP_UP           200.0       /* rpm/s */
#define SPEED_RAMP_DOWN         200.0
#define POWER_RAMP_UP           100.0       /* W/s */
#define POWER_RAMP_DOWN         100.0

/* ===================== 启动（原厂 Fast Boot 流程，参数全是原厂值） ======= */
#define FAST_BOOT_METHOD        1           /* 1 = 原厂快速启动（观测器从 Startup 第一拍就闭环） */
#define PRECHARGE_TIME          20          /* ms，自举电容预充 */
#define IQ_ALIGN                0.5         /* A（FAST_BOOT_METHOD=0 时用） */
#if (FAST_BOOT_METHOD == 1)
  #define ALIGN_TIME            0.001       /* s，原厂值；实际只是 Iq = IQ_STARTUP/4 打 1 ms */
  #define ALIGN_CURRENT_RAMP    5.0         /* A/s */
#else
  #define ALIGN_TIME            1.0
  #define ALIGN_CURRENT_RAMP    2.0
#endif
#define IQ_STARTUP              0.5         /* A */
#define STARTUP_SPEED_RAMP      60.0        /* rpm/s */
#define STARTUP_TIME            2.0         /* s，堵转/缺相判定的时间基准 */
#define MAXSTARTUP_SPEED        100.0       /* rpm */
#define STARTUP_TO_SPIN_SPEED   80.0        /* rpm，Fast_Boot_StartUp 判 Startup->Spin */

/* 启动期 PLL 速度下限（test29）：同时作用于 PLL 积分器和输出转速，且在推进
 * 角度之前施加。12.8 rpm = 原厂 Q31(0.004)。 */
#define STARTUP_PLL_FLOOR_RPM   12.8
/* Startup 期间上报给原厂代码的转速加速度限制（test31）：防 PLL 追相冲高
 * 导致 Fast_Boot 假切 Spin。0.5 A 只能产生约 250 rpm/s，1000 rpm/s 不会拖慢真启动。 */
#define STARTUP_REPORT_ACC_LIMIT 1000.0f    /* rpm/s */

/* ===================== 顺逆风（飞车）检测 ================================ */
#define FLY_CHECK_0BEMF         (0)
#define FLYBRAKE_TIME           0.1         /* s */
#define FLYCURRENT_TH_PU        0.02        /* 标幺（相对 I_MAX），0.02 * 7.964 A ≈ 0.16 A */
#define CALIPWMPERCENT          (0.99)
#define TORQUELOOP_TIME         0.3         /* s */

/* ===================== 保护 ============================================== */
#define DCBUS_OVER              (424.0)     /* V */
#define DCBUS_UNDER             (0.0)       /* V */
#define FREEWHEEL_TIME          0.0         /* s */
#define FAULTRELEASE_TIME       0.0         /* s */
#define BLOCK_TIME              2.0         /* s */
#define BLOCK_VOLTAGE_THRESHOLD (U_MAX * 0.05)          /* V */
#define BLOCK_SPEED_THRESHOLD   STARTUP_TO_SPIN_SPEED   /* rpm */
#define I_PHASSLESS_THRESHOLD   0.3         /* A */
#define SPEED_OC_RATIO          (0.90)      /* 超速保护：|filt| > 0.9 * SPEED_MAX */
#define PHASSLESS_TIME          3.5         /* s */

/* ===================== 磁链观测器 ======================================== */
/* 模型标幺基准（定点缩放用，不是可调参数） */
#define FLUX_V_BASE             (UDC_MAX)
#define FLUX_WN_HZ              100.0
#define FLUX_PSI_N              (FLUX_V_BASE / (2 * 3.14159265359 * FLUX_WN_HZ))   /* ≈ 0.756 Wb */

#define FLUX_BUS_MIN_V          100.0f      /* 母线低于它观测器保持复位 */
/* e 的去直流（高通）：滑动平均窗口 2^SH 拍。12 → tau ≈ 0.256 s */
#define FLUX_HPF_SH             12
/* NLFO 径向修正强度：tau ≈ 8 ms(SH=8) / 16 ms(9，默认) / 32 ms(10)。不要加大（变弱）。 */
#define FLUX_NLFO_SH            9
/* NLFO 修正量饱和（Q15）。只防溢出，不要收紧（test11 收到 8192 明显变差）。 */
#define FLUX_NLFO_FQ_LIMIT      32768
/* 去直流的相位补偿：e 先高通再积分，等效 1/(s+wc)，磁链角度超前 atan(wc/w)
 * （wc = 1/0.256 s：80 rpm 时 5.3°，30 rpm 时 14°）。1 = 按估计转速把输出的
 * psi_s 反转这个角度（慢环算 sin/cos，快环 4 次乘法）；转速低于 FLUX_HPF_COMP_MIN_RPM
 * 不补，到 2 倍该转速线性补满。0 = 不补（test33）。 */
#define CTRL_OBS_HPF_COMP       1
#define FLUX_HPF_COMP_MIN_RPM   20.0

/* ===================== FluxPLL =========================================== */
/* 二阶锁相环：wn、zeta 是"按 MOTOR_PSI_F_WB 计算"的真实环路参数。
 * 理论取值 15 Hz / 0.707（比速度环快 30 倍，比电流环慢 20 倍）-> Kp 27950、Ki 116。
 * 原厂/test33：14.346 / 0.676628（Kp 25583 / Ki 106，原设计 15 Hz/0.707 是按铭牌
 * 0.154 Wb 算的，实测 0.141 Wb 下真实值就是这个）。 */
#define FLUXPLL_WN_HZ           15.0
#define FLUXPLL_ZETA            0.707

/* ===================== MTPA / 弱磁 ======================================== */
/* 两个都默认关：关着时 mtpa_fw 模块不编译任何代码，行为与 test33 逐位相同。
 * MTPA：id = (-psi_f + sqrt(psi_f^2 + 4(Ld-Lq)^2 iq^2)) / (2(Ld-Lq))，只对 IPM 有意义
 *       （MOTOR_IPM = 0 时 id 恒为 0，开了也不起作用）。
 * 弱磁：电压矢量 |V| 超过 (1 - FW_VOLTAGE_MARGIN) * VMAX 时积分出负 id，把电压压回去；
 *       表贴和内嵌都能用。310 V 母线下本电机约 2088 rpm 到电压极限，再往上要弱磁。
 * 两者都只在 Spin（闭环运行）里生效；回退 Startup / 停机时 id 清零。 */
#define CTRL_MTPA_ENABLE        0
#define CTRL_FW_ENABLE          0
#define FW_VOLTAGE_MARGIN       0.08        /* 电压裕量（框架文档里定的 8%） */
/* 弱磁积分速度：电压误差为 100% VMAX 时 id 每秒变化 FW_KI_A_PER_S 安培。
 * 环路极点 ≈ FW_KI * omega*Ld / VMAX；本电机 2000 rpm 下 omega*Ld ≈ 48 V/A，
 * VMAX ≈ 174 V，50 A/s -> 约 2 Hz（比速度环快、比电流环慢很多）。 */
#define FW_KI_A_PER_S           50.0
#define FW_ID_MIN_A             (-2.0)      /* 弱磁 id 下限，A（受退磁电流、效率约束，按电机定） */
/* 电流圆：|i| <= SPD_PI_LIMIT。有 id 时 iq 上限自动缩成 sqrt(Imax^2 - id^2)。 */

/* ===================== 其他 ============================================== */
#define SPEEDUPDATE_FREQ        100         /* Hz */
#define FG_NUM                  5

#endif
