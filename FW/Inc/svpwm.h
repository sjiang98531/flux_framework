/* ============================================================================
 * svpwm.h —— 空间矢量调制（自己实现，替代原厂闭源库的 PWM_GEN_calc / PWM_GEN_init）
 *
 * 输入 pwm_gen.Alpha/Beta：αβ 电压，Q15，32768 = Vbus/√3（已经过 FocCore_BusNormalize）。
 * 输出 pwm_gen.CompA/B/C：三相比较值（0..N_halfPeriod，"比较值越小、相电压越高"），
 *      以及本拍的母线电流重构值（单电阻/无母线采样时用）。
 *
 * 算法是常见的"扇区法"（与 TI 的 SVGEN 同一套 X/Y/Z 写法）：
 *   1. 由 Vβ、(√3Vα − Vβ)/2、(−√3Vα − Vβ)/2 的符号得到扇区号 n（1..6）
 *   2. 扇区内两个相邻矢量的作用时间 t1、t2 取 X/Y/Z 中的两个
 *   3. 七段式：零矢量时间平分到两端；五段式：只用一种零矢量（少开关一次）
 * 结果与原厂库逐位相同（tools/sim/libcheck.c 随机输入对比）。
 * ==========================================================================*/
#ifndef SVPWM_H
#define SVPWM_H
#include "mc_types.h"

#define SEVENMODE   1           /* 七段式 */
#define FIVEMODE    0           /* 五段式 */

/* 调制器状态。布局与原厂开源头文件 pwm_gen.h 的 PWM_GEN_T 相同（仿真里和原厂模型逐位对比要用），
 * 变量名沿用原厂的 pwm_gen（J-Scope 照旧看）。 */
typedef struct {
    int32_t Alpha;              /* 输入 αβ 电压，Q15，32768 = Vbus/√3 */
    int32_t Beta;
    int16_t CompA;              /* 输出三相比较值 */
    int16_t CompB;
    int16_t CompC;
    uint8_t Sector;             /* 扇区 1..6 */
    uint8_t Mode;               /* SEVENMODE / FIVEMODE */
    int16_t N_halfPeriod;       /* PWM 半周期计数 */
} Svpwm_TypeDef;

extern Svpwm_TypeDef pwm_gen;

void Svpwm_Init(Svpwm_TypeDef *u, int16_t halfPeriod);

/* 算比较值。iabc：本拍三相电流（Q15），用于重构母线电流，结果写到 *ibusRec。 */
void Svpwm_Calc(Svpwm_TypeDef *u, const MCFLIB_3_ABC_T_S16 *iabc, int16_t *ibusRec);

#endif
