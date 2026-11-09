/* ============================================================================
 * @file    svpwm.h
 * @brief   空间矢量脉宽调制（SVPWM）
 *
 * 输入 gSvpwm.Alpha/Beta：αβ 电压，Q15，32768 = Vbus/√3（已经过 FocCore_BusNormalize 归一化）。
 * 输出 gSvpwm.CompA/B/C：三相比较值（0..N_halfPeriod，比较值越小、相电压越高），
 *      以及本拍的母线电流重构值（无母线电流采样时使用）。
 *
 * 算法为扇区法（X/Y/Z 形式）：
 *   1. 由 Vβ、(√3Vα − Vβ)/2、(−√3Vα − Vβ)/2 的符号得到扇区号 n（1..6）
 *   2. 扇区内两个相邻基本矢量的作用时间 t1、t2 取 X/Y/Z 中的两个
 *   3. 七段式：零矢量时间平分到两端；五段式：只使用一种零矢量（减少一次开关）
 * ==========================================================================*/
#ifndef SVPWM_H
#define SVPWM_H
#include "qtypes.h"
#include "foc_math.h"

#define SEVENMODE   1           /* 七段式 */
#define FIVEMODE    0           /* 五段式 */

/* 调制器状态（全局实例 gSvpwm） */
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

extern Svpwm_TypeDef gSvpwm;

void Svpwm_Init(Svpwm_TypeDef *u, int16_t halfPeriod);    /* 清除状态、记录 PWM 半周期（上电调用一次） */

/* 计算三相比较值。iabc：本拍三相电流（Q15），用于重构母线电流，结果写入 *ibusRec。 */
void Svpwm_Calc(Svpwm_TypeDef *u, const Abc_TypeDef *iabc, int16_t *ibusRec);

#endif
