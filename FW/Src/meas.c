/* ============================================================================
 * @file    meas.c
 * @brief   测量量慢环滤波，接口说明见 meas.h
 * ==========================================================================*/
#include "meas.h"
#include "cfg_derived.h"
#include "qmath.h"

Meas_TypeDef gMeas;

/* 上电：设置三个一阶低通的移位位数（由配置的时间常数换算），累加器清零，设置母线归一化系数初值 */
void Meas_Init(void)
{
    /* 母线电压、母线电流、转速三个滤波器：窗口见 cfg_internal.h MEAS_*_FILT / SPEED_FB_FILT_HZ */
    gMeas.speedFilt.u16Sh = SPEED_FB_FILT_SH;
    gMeas.speedFilt.s32Acc = 0;
    gMeas.vbusFilt.u16Sh = MEAS_VBUS_FILT_SH;
    gMeas.vbusFilt.s32Acc = 0;
    gMeas.ibusFilt.u16Sh = MEAS_IBUS_FILT_SH;
    gMeas.ibusFilt.s32Acc = 0;
    /* 母线归一化系数初值：按额定母线，与运行时公式相同（第一个慢环起按 vbusAvg 更新） */
    gMeas.kvbus = (int16_t)(MEAS_KVBUS_NUM / Q15(UDC_RATE / UDC_MAX));
}

/* 将转速滤波器直接预置为稳态 w，避免从 0 缓慢上升。用于飞车接管、零电流跟踪转闭环 */
void Meas_SpeedPreset(int16_t w)
{
    /* QMath_LowPass 输入恒为 w 时的稳态累加值 = w*2^sh - w */
    gMeas.speedFilt.s32Acc = ((int32_t)w << gMeas.speedFilt.u16Sh) - w;
    gMeas.speed = w;
}

/* 停机 / 重新起动：转速反馈和滤波器清零 */
void Meas_SpeedReset(void)
{
    gMeas.speedRaw = 0;
    gMeas.speed = 0;
    gMeas.speedFilt.s32Acc = 0;
}

/* 仅将滤波后的转速置 0（滤波器状态不变） */
void Meas_SpeedHoldZero(void)
{
    gMeas.speed = 0;
}

/* kvbus = MEAS_KVBUS_NUM / vbusAvg，结果封顶为 int16（母线很低时商 > 32767，直接强转会变为负数）。
 * 慢环每拍一次 32 位除法（约 350 counts）。 */
static int16_t kvbusOf(int16_t v)
{
    int32_t q = MEAS_KVBUS_NUM / v;
    return (int16_t)(q > 32767 ? 32767 : q);
}

/* 慢环每拍：母线电压、转速、母线电流分别经一阶低通，再计算母线归一化系数 kvbus。
 * 母线电流只滤波正值（负值为回馈，母线电流限只作用于电动方向） */
void Meas_SlowFilter(void)
{
    gMeas.vbusAvg = QMath_LowPass(gMeas.vbus, &gMeas.vbusFilt.s32Acc, gMeas.vbusFilt.u16Sh);
    gMeas.speed = QMath_LowPass(gMeas.speedRaw, &gMeas.speedFilt.s32Acc, gMeas.speedFilt.u16Sh);
    if (gMeas.ibus <= 0) gMeas.ibusAvg = 0;
    else gMeas.ibusAvg = QMath_LowPass(gMeas.ibus, &gMeas.ibusFilt.s32Acc, gMeas.ibusFilt.u16Sh);
    if (gMeas.vbusAvg > 0) gMeas.kvbus = kvbusOf(gMeas.vbusAvg);     /* 母线无电压（上电瞬间）时不更新 */
}
