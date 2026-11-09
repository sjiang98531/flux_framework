/* ============================================================================
 * @file    meas.h
 * @brief   测量量（电流、母线、转速反馈）及其慢环滤波
 *
 * Meas_SlowFilter 每个慢环周期（1 kHz）调用一次：
 *   vbusAvg = 滤波(vbus)        窗口 vbusFilt.u16Sh
 *   speed   = 滤波(speedRaw)    窗口 speedFilt.u16Sh（速度环使用）
 *   ibusAvg = 滤波(ibus)，ibus <= 0 时直接置 0（不滤波）
 *   kvbus   = 母线电压归一化系数（FocCore_BusNormalize 使用）
 * 滤波器为 qmath.h 的 QMath_FilterMA（一阶 IIR，时间常数 2^Sh 拍）。
 * ==========================================================================*/
#ifndef MEAS_H
#define MEAS_H
#include "compiler.h"
#include "qtypes.h"
#include "foc_math.h"

/* KVbus = MEAS_KVBUS_NUM / VbusAvg。归一化后 Vα 的 32768 对应 Vbus/√3：
 *   out = (in * KVbus >> 15) << 5 = in * 1.732 * 32768 / VbusAvg
 * 故 MEAS_KVBUS_NUM = 1.732 * 2^25（√3 取 1.732）。 */
#define MEAS_KVBUS_NUM  ((int32_t)(1.732 * 33554432.0))     /* = 58116276 */

/* 测量量。快环中由 HAL 写入采样值、Clarke 写入 iab；慢环 Meas_SlowFilter 写入滤波值。 */
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
    int16_t speed;                      /* 转速滤波值 Q15：供速度环、保护、状态切换使用 */
    MLIB_FILTER_MA_T vbusFilt, ibusFilt, speedFilt;
} Meas_TypeDef;

extern Meas_TypeDef gMeas;

void Meas_Init(void);               /* 上电：设置三个滤波器窗口，累加器清零 */
void Meas_SlowFilter(void);

/* ---- 转速反馈：gMeas.speedRaw / speed / speedFilt 只能通过以下函数写入 ----
 * 转速来源由当前子状态决定：飞车检测 = 测速结果，I/F = 开环转速，Spin = PLL 转速；每拍由对应状态的代码通过
 * Meas_SpeedSource 提交，慢环 Meas_SlowFilter 滤波得到 gMeas.speed（速度环、保护、状态切换读取，其他地方只读）。 */
FORCE_INLINE void Meas_SpeedSource(int16_t raw)
{
    gMeas.speedRaw = raw;
}
void Meas_SpeedPreset(int16_t w);   /* 将滤波器预置为稳态 w，gMeas.speed = w（飞车接管、零电流跟踪转闭环时；不修改 speedRaw） */
void Meas_SpeedReset(void);         /* 来源、滤波值、累加器全部清零（停机复位、冷起动） */
void Meas_SpeedHoldZero(void);      /* 仅将滤波值置 0（定位 / IPD：转子静止，屏蔽来源的残留值） */

#endif
