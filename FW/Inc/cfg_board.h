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
#define PWM_PERIOD              ((uint16_t)(SYSCLK_HSI_72MHz / (2 * PWM_FREQ)))  /* EPWM 半周期计数 = 2250 */
#define DEAD_TIME               (1200)                     /* ns（Hardware_config.h 据此计算死区寄存器） */
/* 慢环频率（PendSV，快环每 PWM_FREQ/SLOWLOOP_FREQ 拍挂起一次）。必须整除 PWM_FREQ；
 * 下限由配置检查 SLOW_RATE_FITS、BUS_VDC_ABOVE_GRID 确定（本电机约 500 Hz）。 */
#define SLOWLOOP_FREQ           1000                       /* Hz */
#define MAX_DUTY                (0.84)                     /* 电流环电压限幅 = MAX_DUTY * Vbus * 2/3 */

/* ---- 电流采样 ------------------------------------------------------------ */
#define ADC_REFV                5.0                        /* V */
#define R_SHUNT                 (0.1)                      /* ohm */
#define CURRENT_OPA_GAIN        (3.139)
/* 电流 Q15 满量程 = 5/2/3.139/0.1 = 7.964 A。gMeas.iabc、gFoc.idq / idqRef 均采用此基准。 */
#define I_MAX                   (ADC_REFV / 2 / CURRENT_OPA_GAIN / R_SHUNT)
#define IBUS_RECONSTRUCTED      1                          /* 1 = 母线电流使用 SVPWM 重构值 */

/* 电流采样噪声峰值下限（ADC 码，1 码 = 量化误差）。实际值在上电零点校准时测得（gHal.adcNoisePk，
 * 取两者较大值），飞车测速脉冲的电流下限据此计算（flystart.c）。 */
#define BOARD_ADC_NOISE_CODES   1.0

/* ---- 母线电压 ------------------------------------------------------------ */
#define UDC_RATE                (310.0)                    /* V，额定母线电压 */

/* 逆变器电压误差（死区 + 管压降）：仅在观测器中补偿（指令电压 -> 实际电压），PWM 与电流环不做补偿。
 * 每相误差 = INVERTER_DT_EFF_S * PWM_FREQ * Vbus * clamp(i / INVERTER_DT_I0_A, -1, 1)：电流大于 I0 后为常值、
 * 与母线电压成正比（死区时间），小电流段线性过零（开关过程中电流方向不确定）。
 * INVERTER_DT_* 为驱动板实测值（可用参数辨识模式测量），INVERTER_DT_COMP 为补偿开关（0 = 不补偿）。
 * 本板：每相 4.07 V @ 310 V、16 kHz -> 0.82 us；I0 = 0.23 A。与电流符号无关的常值偏置由观测器去直流环节消除。 */
#define INVERTER_DT_EFF_S            0.82e-6         /* s，等效死区时间 */
#define INVERTER_DT_I0_A             0.23            /* A，误差线性过零的电流宽度 */
#define INVERTER_DT_COMP             1               /* 1 = 在观测器中补偿 */
#define UDC_MAX                 (475.0)                    /* V，母线采样满量程 = 电压 Q15 基准 */
#define U_MAX                   (UDC_MAX / 1.732)          /* V */
/* 母线电容（整流后电解电容 + 薄膜电容）。回馈减速的母线电压控制器（BUS_VDC_*）据此将电压误差换算为能量：
 * E = C * V^2 / 2。 */
#define BOARD_CBUS_UF           (470.0)                    /* uF */
/* 母线过压保护门限：由母线电容、功率器件耐压（扣除裕量）决定，属于驱动板属性，与电机无关。 */
#define BOARD_DCBUS_OVP_V       (424.0)                    /* V */
/* 直流输入上限（产品规格）：空载母线最高电压 = BOARD_VBUS_IN_MAX_V。
 * 回馈减速的母线调节点必须高于该值，否则输入偏高时母线没有储能裕量，减速指令无法执行。
 * 规格书给出输入范围时填写上限；未给出时按额定 +10%。配置检查 BUS_VDC_ABOVE_GRID 会校验。
 * 交流输入（板上整流）的产品填 sqrt(2) * 电网电压上限有效值。 */
#define BOARD_VBUS_IN_MAX_V     (310.0 * 1.10)             /* V DC */
#define BOARD_VBUS_TOL          (0.02)                     /* 母线采样相对误差（分压电阻 + ADC 增益） */

/* ---- 相序 ---------------------------------------------------------------- */
/* 电流采样通道 / PWM 通道与电机相的对应关系（仅在 hal_sc32r602.c 中使用）。
 * RUN_CCW 将板上 U/V 两相对调，即改变电机旋转方向的硬件定义。
 * 正转（正的转速指令）应对应风机的工作方向（A 类，按叶轮 / 风道确定），本风机为 RUN_CW。 */
#define RUN_CW                  (0)
#define RUN_CCW                 (1)
#define RUN_DIR                 RUN_CW

/* ---- 上电调试 ------------------------------------------------------------ */
/* 1 = 不运行控制，三相输出固定 25% 占空比，用于检查驱动板 */
#define PWM_DEBUG               0

/* ---- 快环代码执行位置（compiler.h RAMFUNC）---------------------------------
 * 1 = Spin 快环每拍执行的函数放在 RAM 中运行（72 MHz 下 Flash 有 2 个等待周期、M0+ 无缓存，快环约快 27.5%），
 *     占用 RAM 约 4.7 KB；
 * 0 = 全部在 Flash 中运行：快环约慢 27.5%，节省约 4.7 KB RAM，链接脚本中的 ramfunc 段为空。两者控制行为相同。 */
#define FASTPATH_IN_RAM         0

/* ---- 诊断 ----------------------------------------------------------------- */
/* 观测器置信度统计窗口（B 类）：每个窗口计算一次置信度（含一次除法），不在每拍执行。64 ms = 慢环 1 kHz 下 64 帧。 */
#define DIAG_WINDOW_S           0.064

/* ---- 参数辨识模式（identify.h）：1 = 起动时只执行 定位 -> 死区阶梯 -> 电感 / IPD 测量，然后保持零电压，电机不旋转。
 * 正常运行必须为 0（编译期去除）。定位角可在调试器中通过 gIdentCfg 修改，脉冲参数见 gIpdCfg（cfg_ctrl.h IPD_*） ---- */
#define IDENT_TEST              0

#endif
