/* ============================================================================
 * @file    foc_math.h
 * @brief   电机控制基础数学函数
 *
 *   FocMath_SinCos     查表正弦 / 余弦
 *   FocMath_SqrtQ15    查表开方
 *   FocMath_ClarkeSat / FocMath_Park / FocMath_InvPark  坐标变换
 *   Pi_Run32 / Pi_Run64  定点 PI 调节器
 *
 * 查找表定义在 qmath.c（按公式生成）。数据类型见 qtypes.h。
 * ==========================================================================*/
#ifndef FOC_MATH_H
#define FOC_MATH_H
#include <stdint.h>
#include "compiler.h"
#include "qtypes.h"
#include "qmath.h"

/* 四分之一周期正弦表 sin(i*90°/256)，Q15 */
extern const int16_t g_focSinTab[256];
/* 开方表 sqrt(i/512)，Q15，i = 0..512 */
extern const int16_t g_focSqrtTab[513];

/* 角度 Q15（-32768..32767 对应 -180°..180°，即 65536 = 一圈），返回 cos/sin（Q15）。
 * 不插值，分辨率 90°/256 = 0.35°。
 * 先把角度平移半圈变成 0..65535：高 2 位是象限，接下来 8 位是表索引。
 * 平移了 180°，所以各象限的符号与常规写法差一个负号。 */
FORCE_INLINE SinCos_TypeDef FocMath_SinCos(int16_t angle)
{
    SinCos_TypeDef r;
    uint16_t u = (uint16_t)((int32_t)angle + 0x8000);
    uint32_t idx = (u >> 6) & 0xFFu;
    int16_t s = g_focSinTab[idx];             /* sin(φ) */
    int16_t c = g_focSinTab[255u - idx];      /* ≈ cos(φ) */
    switch (u >> 14) {
    case 0:  r.s16Cos = (int16_t)-c; r.s16Sin = (int16_t)-s; break;    /* 实际角度 -180°..-90° */
    case 1:  r.s16Cos = s;           r.s16Sin = (int16_t)-c; break;    /* -90°..0° */
    case 2:  r.s16Cos = c;           r.s16Sin = s;           break;    /* 0°..90° */
    default: r.s16Cos = (int16_t)-s; r.s16Sin = c;           break;    /* 90°..180° */
    }
    return r;
}

/* sqrt(x)，x 为 Q30（0..2^30 对应 0..1），返回 Q15。查表不插值，输入按 2^21 分档
 * （精度约 0.1%~几 %，小输入时更粗）。用于电压圆限幅和诊断，不用于控制精度要求高的地方。
 * 负数按 0 处理，超过 2^30 按 1 处理（防止越界读表）。 */
FORCE_INLINE int16_t FocMath_SqrtQ15(int32_t x)
{
    if (x <= 0) return 0;
    if (x >= (1L << 30)) return g_focSqrtTab[512];
    return g_focSqrtTab[x >> 21];
}

/* Clarke：三相电流 -> 静止两相 αβ（α 沿 A 相）。双电阻采样，c = -a - b 已在 abc 中给出：
 * alpha = a，beta = (b - c) / sqrt3，beta 饱和到 Q15 */
FORCE_INLINE void FocMath_ClarkeSat(const Abc_TypeDef *in, AlphaBeta_TypeDef *out)
{
    int32_t b = (((int32_t)in->s16B - in->s16C) * Q15_INV_SQRT3) >> 15;
    out->s16Alpha = in->s16A;
    if (b > 32767) b = 32767;
    if (b < -32768) b = -32768;
    out->s16Beta = (int16_t)b;
}

/* Park：静止 αβ 坐标 -> 随转子旋转的 dq 坐标（d 轴沿转子磁链，q 轴超前 90°，iq 产生转矩）。
 * d = a cos + b sin，q = b cos - a sin，不饱和 */
FORCE_INLINE void FocMath_Park(const AlphaBeta_TypeDef *in, const SinCos_TypeDef *ang, Dq_TypeDef *out)
{
    int32_t a = in->s16Alpha, b = in->s16Beta, s = ang->s16Sin, c = ang->s16Cos;
    out->s16D = (int16_t)((c * a + s * b) >> 15);
    out->s16Q = (int16_t)((c * b - s * a) >> 15);
}

/* 反 Park：dq 电压指令 -> 静止 αβ 坐标（之后交给 SVPWM）。
 * alpha = d cos - q sin，beta = d sin + q cos，不饱和 */
FORCE_INLINE void FocMath_InvPark(const Dq_TypeDef *in, const SinCos_TypeDef *ang, AlphaBeta_TypeDef *out)
{
    int32_t d = in->s16D, q = in->s16Q, s = ang->s16Sin, c = ang->s16Cos;
    out->s16Beta  = (int16_t)((c * q + s * d) >> 15);
    out->s16Alpha = (int16_t)((c * d - s * q) >> 15);
}

/* ---- 定点 PI 调节器（抗积分饱和与防溢出） --------------------------------- */
FORCE_INLINE void Pi_Run32(PI_TypeDef *p)
{
    int32_t err = (int32_t)p->qInRef - (int32_t)p->qInMeas;
    int32_t hi = (int32_t)p->qOutMax << 15, lo = (int32_t)p->qOutMin << 15;
    int32_t sum, out;
    if (err > 32767) err = 32767;
    if (err < -32767) err = -32767;
    sum = QMath_AddSat32((int32_t)p->qdSum, (int32_t)p->qKi * err);
    if (sum >= hi) sum = hi;
    else if (sum <= lo) sum = lo;
    p->qdSum = sum;
    out = (((int32_t)p->qKp * err) >> 15) + (sum >> 15);
    if (out > p->qOutMax) out = p->qOutMax;
    else if (out < p->qOutMin) out = p->qOutMin;
    p->qOut = out;
}

FORCE_INLINE void Pi_Run64(PI_TypeDef *p)
{
    int64_t d = (int64_t)p->qInRef - (int64_t)p->qInMeas;
    int32_t err = (d > 2147483647) ? 2147483647 : (d < -2147483647) ? -2147483647 : (int32_t)d;
    int64_t hi = (int64_t)p->qOutMax << 15, lo = (int64_t)p->qOutMin << 15;
    int64_t sum = (int64_t)p->qdSum + (int64_t)(int32_t)p->qKi * err;
    int64_t out;
    if (sum >= hi) sum = hi;
    else if (sum <= lo) sum = lo;
    p->qdSum = (int32_t)sum;
    out = (((int64_t)(int32_t)p->qKp * err) >> 15) + (sum >> 15);
    if (out > p->qOutMax) out = p->qOutMax;
    else if (out < p->qOutMin) out = p->qOutMin;
    p->qOut = (int32_t)out;
}

#endif
