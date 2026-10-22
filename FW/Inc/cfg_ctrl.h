/* ============================================================================
 * cfg_ctrl.h —— 控制规格（物理单位）
 *
 * 写"想要的效果"（带宽 Hz、阻尼、rpm/s、A、秒），定点增益由 fw_config.h 算。
 * 默认值逐位复现 test33：凡是原厂整定的环，下面的 Hz/阻尼是从原厂增益
 * 反算出来的，注释里写了反算结果和它的物理含义。
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

/* ===================== 电流环 ============================================ */
/* 零极点对消设计：Kp = 2*pi*BW*L，Ki = 2*pi*BW*Rs（d 轴用 Ld，q 轴用 Lq）。
 * 原厂 13747/274 反算出 BW = 86.57 Hz，且 Ki/Kp 正好等于 Rs/L——原厂本来就是
 * 这么设计的。换电机只要 cfg_motor.h 的 Rs/Ld/Lq 对，带宽就不变。 */
#define CURRENT_BW_HZ           86.573
#define CURRENT_OUTMAX          (3000)      /* 原厂初值（运行中被 VMAX 覆盖） */

/* ===================== 速度环 ============================================ */
/* 被控对象 d(rpm)/dt = MOTOR_ACC_RPM_S_PER_A * iq，PI 闭环成二阶：
 *   Kp = 2*zeta*wn / b，Ki = wn^2 / b（b = MOTOR_ACC_RPM_S_PER_A）。
 * 原厂 8000/100 反算出 wn = 0.31 Hz、zeta = 0.078 —— 阻尼极小，台架上看到的
 * 3.2 s 周期振荡、+80~110 rpm 过冲就是它（smo_baseline 同样如此）。
 * 想改善就把 SPEED_ZETA 提到 0.5~0.7（增益会随之自动变化），先台架验证。 */
#define SPEED_WN_HZ             0.3105
#define SPEED_ZETA              0.078025
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

/* ===================== FluxPLL =========================================== */
/* 二阶锁相环：wn、zeta 是"按 MOTOR_PSI_F_WB 计算"的真实环路参数。
 * 原设计 15 Hz / 0.707（按 0.154 Wb 算的），在实测 0.141 Wb 下就是这里的
 * 14.35 Hz / 0.677；取这组数逐位复现 test33 的 Kp 25583 / Ki 106。 */
#define FLUXPLL_WN_HZ           14.346
#define FLUXPLL_ZETA            0.676628

/* ===================== 其他 ============================================== */
#define SPEEDUPDATE_FREQ        100         /* Hz */
#define FG_NUM                  5

#endif
