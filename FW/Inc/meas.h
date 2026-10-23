/* ============================================================================
 * meas.h —— 测量量的慢环滤波（自己实现，替代原厂闭源库的 Var_Filt）
 *
 * 每个慢环拍（1 kHz）调用一次：
 *   FOC.s16VbusAvg = 滑动平均(FOC.s16Vbus)                 窗口 USER.sVbus_Filt.u16Sh
 *   FOC.s16SpdFilt = 滑动平均(FOC.FluxFeedback.s16Speed)   窗口 USER.sSpeed_Filt.u16Sh（速度环用它）
 *   FOC.s16IbusAvg = 滑动平均(FOC.s16Ibus)，Ibus <= 0 时直接给 0（不滤）
 *   FOC.s16KVbus   = 母线电压归一化系数（FocCore_BusNormalize 用）
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

void Meas_SlowFilter(void);

#endif
