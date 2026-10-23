/* ============================================================================
 * meas.h —— 测量量（电流、母线、转速反馈）及其慢环滤波（替代原厂闭源库的 Var_Filt）
 *
 * Meas_SlowFilter 每个慢环拍（1 kHz）调用一次：
 *   vbusAvg = 滑动平均(vbus)        窗口 vbusFilt.u16Sh
 *   speed   = 滑动平均(speedRaw)    窗口 speedFilt.u16Sh（速度环用它）
 *   ibusAvg = 滑动平均(ibus)，ibus <= 0 时直接给 0（不滤）
 *   kvbus   = 母线电压归一化系数（FocCore_BusNormalize 用）
 * "滑动平均"是原厂开源头文件 mlib.h 里的 MLIB_FilterMA_Q15（一阶 IIR，时间常数 2^Sh 拍）。
 * 行为与原厂库逐位相同。
 * ==========================================================================*/
#ifndef MEAS_H
#define MEAS_H
#include "mc_types.h"

/* KVbus = MEAS_KVBUS_NUM / VbusAvg。归一化后 Vα 的 32768 对应 Vbus/√3：
 *   out = (in * KVbus >> 15) << 5 = in * 1.732 * 32768 / VbusAvg
 * 所以 MEAS_KVBUS_NUM = 1.732 * 2^25（原厂取 √3 ≈ 1.732，保留）。 */
#define MEAS_KVBUS_NUM  ((int32_t)(1.732 * 33554432.0))     /* = 58116276 */

/* 测量量。快环由 HAL 写采样值，Clarke 写 iab；慢环 Meas_SlowFilter 写滤波值。 */
typedef struct {
    MCFLIB_3_ABC_T_S16  iabc;           /* 三相电流 Q15（I_MAX 基准） */
    MCFLIB_3_ABC_T_S16  iabcOffset;     /* 电流零点（ADC 原始值 << 3） */
    MCFLIB_2_ALBE_T_S16 iab;            /* αβ 电流（Clarke） */
    int16_t vbus;                       /* 母线电压 Q15（UDC_MAX 基准），本拍采样 */
    int16_t vbusAvg;                    /* 母线电压滤波 */
    int16_t kvbus;                      /* 母线归一化系数 = MEAS_KVBUS_NUM / vbusAvg */
    int16_t ibus;                       /* 母线电流（IBUS_RECONSTRUCTED=1 时 = ibusRec） */
    int16_t ibusAvg;                    /* 母线电流滤波（<= 0 时为 0） */
    int16_t ibusRec;                    /* SVPWM 重构的母线电流 */
    int16_t speedRaw;                   /* 转速原始值 Q15（SPEED_MAX 基准）：闭环时 = PLL 转速，开环时 = 开环转速 */
    int16_t speed;                      /* 转速滤波 Q15：速度环、保护、状态切换都用它 */
    MLIB_FILTER_MA_T vbusFilt, ibusFilt, speedFilt;
} Meas_TypeDef;

extern Meas_TypeDef gMeas;

void Meas_SlowFilter(void);

#endif
