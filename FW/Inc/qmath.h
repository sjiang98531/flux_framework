/* ============================================================================
 * @file    qmath.h
 * @brief   定点运算工具（Q15 / Q31 / Q16.16）
 *
 * 约定：Q15 = int16，满量程 ±1.0 = ±32768；Q31 = int32；角度 Q16.16 = int32，
 * 高 16 位 0..65535 对应一个电周期。M0+ 无硬件除法，快环中禁止使用除法和开方；
 * QMath_Isqrt32 仅供慢环使用。
 * ==========================================================================*/
#ifndef QMATH_H
#define QMATH_H
#include <stdint.h>
#include "compiler.h"

/* a*b >> 15（不饱和） */
FORCE_INLINE int16_t QMath_MulQ15(int16_t a, int16_t b)
{
    return (int16_t)(((int32_t)a * b) >> 15);
}

/* 对称限幅到 [-n, n] */
FORCE_INLINE int32_t QMath_Clamp(int32_t x, int32_t n)
{
    return x > n ? n : (x < -n ? -n : x);
}

/* 饱和到 int16 */
FORCE_INLINE int16_t QMath_Sat16(int32_t x)
{
    return (int16_t)(x > 32767 ? 32767 : (x < -32768 ? -32768 : x));
}

/* |x|，不饱和（-32768 -> -32768） */
FORCE_INLINE int16_t QMath_Abs16(int16_t x)
{
    return (int16_t)(x < 0 ? -x : x);
}

/* 32 位饱和加法（结果限在 [INT32_MIN, INT32_MAX]），QMath_FilterMA 内部用。
 * 判溢出：两个非负数相加得负数 = 正溢出，两个负数相加得非负数 = 负溢出（用符号位运算，不用分支比较） */
FORCE_INLINE int32_t QMath_AddSat32Full(int32_t a, int32_t b)
{
    int32_t z = (int32_t)((uint32_t)a + (uint32_t)b);
    if ((~(a | b) & z) < 0) return INT32_MAX;
    if ((a & b & ~z) < 0) return INT32_MIN;
    return z;
}
/* 一阶低通（一阶 IIR，命名中的 MA 指"移动平均"形式）：
 *   acc += x（饱和），y = acc >> sh，acc -= y（饱和），返回饱和到 Q15 的 y。
 * 等价于 y += (x - y) / 2^sh，时间常数约 2^sh 拍；稳态 acc = x*2^sh - x。
 * 用于母线电压、转速、母线电流的慢环滤波（meas.c）。快环中也有调用，故强制内联。 */
FORCE_INLINE int16_t QMath_FilterMA(int16_t x, int32_t *acc, uint16_t sh)
{
    int32_t a = QMath_AddSat32Full(*acc, x), y = a >> sh, ny = (int32_t)(0u - (uint32_t)y);
    int32_t z = (int32_t)((uint32_t)a - (uint32_t)y);
    if ((~(a | ny) & z) < 0) z = INT32_MAX;
    else if ((a & ny & ~z) < 0) z = INT32_MIN;
    *acc = z;
    return (int16_t)(y > 32767 ? 32767 : (y < -32768 ? -32768 : y));
}

/* 32 位饱和加法，对称限在 ±0x7FFFFFFF（PI、PLL 用；QMath_AddSat32Full 限在 [INT32_MIN, INT32_MAX]） */
FORCE_INLINE int32_t QMath_AddSat32(int32_t a, int32_t b)
{
    if (b > 0 && a > (int32_t)0x7FFFFFFF - b) return  (int32_t)0x7FFFFFFF;
    if (b < 0 && a < -(int32_t)0x7FFFFFFF - b) return -(int32_t)0x7FFFFFFF;
    return a + b;
}

/* atan2(y, x)，CORDIC 16 次迭代，结果 2^32 = 一圈（误差约 2^-15 rad = 0.002°）。x = y = 0 返回 0。
 * 几百个周期，快环里只能偶尔用（飞车检测每 1 ms 最多一次）。 */
uint32_t QMath_Atan2(int32_t y, int32_t x);

/* 整数平方根 floor(sqrt(x))，逐位法约 16 次循环（每次定一位结果）。只在慢环用。 */
uint32_t QMath_Isqrt32(uint32_t x);
/* 同上，64 位输入（32 次循环）。仅在主循环 / 参数辨识模式中使用（慢环禁止 64 位开方） */
uint32_t QMath_Isqrt64(uint64_t x);

/* 单位化：(a, b) / sqrt(a^2 + b^2)，Q15（0 <= |a|, |b| <= 32767，不同时为 0）。不使用除法和开方：
 * 1/sqrt 用牛顿迭代 y <- y(3 - x y^2)/2，查表初值（相对误差 <= 6%）迭代 3 次，误差 < 1e-8；
 * 全是 32x32->64 乘法，定长，约 100 个时钟（M0+ 上一次 32 位除法约 350 个，QMath_Isqrt32 约 300 个）。
 * 结果和 a*32767/r（整数除法）相差不超过 1 LSB。 */
void QMath_NormQ15(int32_t a, int32_t b, int16_t *na, int16_t *nb);

#endif
