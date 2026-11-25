/* ============================================================================
 * @file    cfg_board.h
 * @brief   驱动板参数（HAL 层）
 *
 * 更换驱动板时只需修改：本文件 + Board/（SC_Init.c 外设初始化、Hardware_config.h 引脚与 ADC 通道）
 * + FW/Src/hal_sc32r602.c（电流采样符号 / 相序、PWM 寄存器写入）。
 * 驱动板物理参数（时钟、PWM 频率、死区、ADC 基准、采样电阻、运放增益）只在本文件定义一次，
 * Hardware_config.h 引用本文件。算法层（FW/ 其余文件）不直接访问寄存器。
 * ==========================================================================*/
#ifndef CFG_BOARD_H
#define CFG_BOARD_H

#include <stdint.h>

/* ---- 时钟与 PWM ---------------------------------------------------------- */
#define SYSCLK_HSI_72MHz        72000000.0                 /* Hz */
#define PWM_FREQ                16000                      /* Hz，快环频率 = PWM 频率 */
#define DEAD_TIME               (350)                     /* ns（Hardware_config.h 据此计算死区寄存器） */
/* 慢环频率（PendSV，快环每 PWM_FREQ/SLOWLOOP_FREQ 拍挂起一次）。必须整除 PWM_FREQ；
 * 下限：慢环越慢，回馈压升的检测延迟（BUS_DELAY_S）越长，母线门限间距越大，须保证调节点仍高于空载母线（人工保证，本电机约 500 Hz）。 */
#define SLOWLOOP_FREQ           1000                       /* Hz */
#define MAX_DUTY                (0.84)                     /* 电流环电压限幅 = MAX_DUTY * Vbus * 2/3 */

/* ---- 电流采样 ------------------------------------------------------------ */
#define ADC_REFV                5.0                        /* V */
#define R_SHUNT                 (0.010)                      /* ohm */
#define CURRENT_OPA_GAIN        (6.36)
/* 电流 Q15 满量程 = 5/2/3.139/0.1 = 7.964 A。gMeas.iabc、gFoc.idq / idqRef 均采用此基准。 */
#define I_MAX                   (ADC_REFV / 2 / CURRENT_OPA_GAIN / R_SHUNT)
#define IBUS_RECONSTRUCTED      1                          /* 1 = 母线电流使用 SVPWM 重构值 */
/* 硬件过流比较器（CMP3）DAC 门限电压及对应电流（母线采样与相电流共用 R_SHUNT 与 CURRENT_OPA_GAIN，
 * 零点为 ADC_REFV / 2，满量程摆幅 ADC_REFV / 2 对应 I_MAX，见 hal_sc32r602.c acquireSamples） */
#define IBUS_HOCP_Value         (4.3278)                   /* V（Hardware_config.h 据此计算 DAC 装载值 IBUS_HOCP_DACValue） */
#define BOARD_HOCP_A            ((IBUS_HOCP_Value - 0.5 * ADC_REFV) / (0.5 * ADC_REFV) * I_MAX)

/* 电流采样噪声峰值下限（ADC 码，1 码 = 量化误差）。实际值在上电零点校准时测得（gHal.adcNoisePk，
 * 取两者较大值），飞车测速脉冲的电流下限据此计算（flystart.c）。 */
#define BOARD_ADC_NOISE_CODES   1.0

/* ---- 母线电压 ------------------------------------------------------------ */
#define UDC_RATE                (24.0)                    /* V，额定母线电压 */

/* 逆变器电压误差（死区 + 管压降）：仅在观测器中补偿（指令电压 -> 实际电压），PWM 与电流环不做补偿。
 * 每相误差 = INVERTER_DT_EFF_S * PWM_FREQ * Vbus * clamp(i / INVERTER_DT_I0_A, -1, 1)：电流大于 I0 后为常值、
 * 与母线电压成正比（死区时间），小电流段线性过零（开关过程中电流方向不确定）。
 * INVERTER_DT_* 为驱动板实测值（可用参数辨识模式测量），INVERTER_DT_COMP 为补偿开关（0 = 不补偿）。
 * 本板：每相 4.07 V @ 310 V、16 kHz -> 0.82 us；I0 = 0.23 A。与电流符号无关的常值偏置由观测器去直流环节消除。 */
#define INVERTER_DT_EFF_S            0.35e-6         /* s，等效死区时间 */
#define INVERTER_DT_I0_A             0.50            /* A，误差线性过零的电流宽度 */
#define INVERTER_DT_COMP             1               /* 1 = 在观测器中补偿 */
#define UDC_MAX                 (55.0)                    /* V，母线采样满量程 = 电压 Q15 基准 */
/* 母线电容（整流后电解电容 + 薄膜电容）。回馈减速的母线电压控制器（BUS_VDC_*）据此将电压误差换算为能量：
 * E = C * V^2 / 2。 */
#define BOARD_CBUS_UF           (470.0)                    /* uF */
/* 母线过压保护门限：由母线电容、功率器件耐压（扣除裕量）决定，属于驱动板属性，与电机无关。 */
#define BOARD_DCBUS_OVP_V       (32.0)                    /* V */
/* 输入电压上限（产品规格）：空载母线最高电压 = BOARD_VBUS_IN_MAX_V。
 * 回馈减速的母线调节点必须高于该值：母线由电网经整流桥供电时不会低于电网峰值，调节点若在其下方，
 * 控制器会把"电网把母线抬高"当成"回馈把母线抬高"，减速 / 换向指令不被执行（由人工保证）。
 * 交流输入（板上整流）填 sqrt(2) * 输入电压上限有效值；直流输入填直流上限。
 * 本产品输入 176 ~ 264 Vac：264 * sqrt(2) = 373 V（台架 264 Vac 空载实测 372 V）。 */
#define BOARD_VAC_IN_MAX_V      (18.0)                    /* V rms，交流输入上限 */
#define BOARD_VBUS_IN_MAX_V     (BOARD_VAC_IN_MAX_V * 1.41421356)     /* V DC，本板 373 V */
#define BOARD_VBUS_TOL          (0.02)                     /* 母线采样相对误差（分压电阻 + ADC 增益） */

/* 相序 / 转向 RUN_DIR：见 cfg_app.h（应用层） */

/* ---- 上电调试 ------------------------------------------------------------ */
/* 1 = 不运行控制，三相输出固定 25% 占空比，用于检查驱动板 */
#define PWM_DEBUG               0

/* ---- 快环代码执行位置（compiler.h RAMFUNC）---------------------------------
 * 1 = Spin 快环每拍执行的函数放在 RAM 中运行（72 MHz 下 Flash 有 2 个等待周期、M0+ 无缓存，快环约快 27.5%），
 *     占用 RAM 约 4.7 KB；
 * 0 = 全部在 Flash 中运行：快环约慢 27.5%，节省约 4.7 KB RAM，链接脚本中的 ramfunc 段为空。两者控制行为相同。 */
#define FASTPATH_IN_RAM         0

/* ---- 参数辨识模式（identify.h）：1 = 给出一个转速指令即自动辨识：静止测量（不旋转，约 15 s）-> 自动停机后重新起动
 * -> 运行测量（磁链、加速度系数）-> 按指令转速继续运行；gIdent.state = 2 表示完成，结果在 gIdent / gParam。
 * 仅执行其中一步时修改 gIdentCfg.mode。正常运行必须为 0（编译期去除）。IPD 参数见 cfg_internal.h IPD_* ---- */
#define IDENT_TEST              1

#endif
