/* ============================================================================
 * cfg_board.h —— 驱动板参数（HAL 层）
 *
 * 换驱动板时只改：本文件 + Board/（SC_Init.c 外设初始化、Hardware_config.h
 * 引脚与 ADC 通道）+ FW/Src/hal_sc32r602.c（电流采样符号/相序、PWM 写寄存器）。
 * 算法层（FW/ 其余文件）不直接碰寄存器。
 *
 * 数值全部来自 test33 的 mc_customize.h，一个都没改。
 * ==========================================================================*/
#ifndef CFG_BOARD_H
#define CFG_BOARD_H

/* 器件头文件必须先于 mlib_types.h：它用枚举定义 TRUE/FALSE，mlib_types.h 又把它们定义成宏 */
#include "sc32_conf.h"
#include "basic.h"

/* ---- 时钟与 PWM ---------------------------------------------------------- */
#define SYSCLK_HSI_72MHz        72000000.0                 /* Hz */
#define PWM_FREQ                16000                      /* Hz，快环频率 = PWM 频率 */
#define PWM_PERIOD              ((uint16_t)(SYSCLK_HSI_72MHz / (2 * PWM_FREQ)))  /* EPWM 半周期计数 = 2250 */
#define DEAD_TIME               (1200)                     /* ns（Hardware_config.h 用来算死区寄存器） */
#define SLOWLOOP_FREQ           1000                       /* Hz，慢环名义频率（主循环轮询） */
#define MAX_DUTY                (0.84)                     /* 电流环电压限幅 = MAX_DUTY * Vbus * 2/3 */

/* ---- 电流采样 ------------------------------------------------------------ */
#define ADC_REFV                5.0                        /* V */
#define R_SHUNT                 (0.1)                      /* ohm */
#define CURRENT_OPA_GAIN        (3.139)
/* 电流 Q15 满量程。注意：= 5/2/3.139/0.1 = 7.964 A（原 mc_customize.h 注释里的
 * "3.93A" 是旧板子的值，早已过时）。gMeas.iabc、gFoc.idq / idqRef 都是这个口径。 */
#define I_MAX                   (ADC_REFV / 2 / CURRENT_OPA_GAIN / R_SHUNT)
#define IBUS_RECONSTRUCTED      1                          /* 1 = 母线电流用 SVPWM 重构值 */

/* 电流零点：上电采 256 个点求平均。test33 只采了 255 个却右移 8 位（除以 256），
 * 零点系统性偏小 1/256（每相约 -0.03 A）。1 = 修正；0 = 原厂行为。 */
#define HAL_OFFSET_256          1

/* ---- 母线电压 ------------------------------------------------------------ */
#define UDC_RATE                (310.0)                    /* V，额定母线（原厂母线纹波补偿用） */
#define UDC_MAX                 (475.0)                    /* V，母线采样满量程 = 电压 Q15 基准 */
#define U_MAX                   (UDC_MAX / 1.732)          /* V */

/* ---- 相序 ---------------------------------------------------------------- */
/* 电流采样通道 / PWM 通道与电机相的对应关系（只在 hal_sc32r602.c 里用）。
 * RUN_CCW 把板上 U/V 两相对调，即改变电机旋转方向的"硬件"定义。 */
#define RUN_CW                  (0)
#define RUN_CCW                 (1)
#define RUN_DIR                 RUN_CCW

/* ---- 上电调试 ------------------------------------------------------------ */
/* 1 = 不跑控制，三相输出固定 25% 占空比，用来查驱动板（原厂 PWM_DEBUG） */
#define PWM_DEBUG               0

/* ---- 快环 CPU 预算 ------------------------------------------------------- */
/* TIM3 = 36 MHz，2250 counts = 62.5 us = 一个 PWM 周期（diag 统计 overBudget 用） */
#define FLUX_ISR_BUDGET         2250

#endif
