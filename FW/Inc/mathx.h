/* ============================================================================
 * mathx.h —— 定点小工具（Q15 / Q31 / Q16.16）
 *
 * 约定：Q15 = int16，满量程 ±1.0 = ±32768；Q31 = int32；角度 Q16.16 = int32，
 * 高 16 位 0..65535 对应一个电周期。M0+ 没有硬件除法，快环里不要用除法和
 * sqrt；这里的 Mx_Isqrt32 只给慢环用。
 * ==========================================================================*/
#ifndef MATHX_H
#define MATHX_H
#include <stdint.h>

#define MX_INLINE static __forceinline

/* a*b >> 15（不饱和） */
MX_INLINE int16_t Mx_MulQ15(int16_t a, int16_t b)
{
    return (int16_t)(((int32_t)a * b) >> 15);
}

/* 对称限幅到 [-n, n] */
MX_INLINE int32_t Mx_Clamp(int32_t x, int32_t n)
{
    return x > n ? n : (x < -n ? -n : x);
}

/* 饱和到 int16 */
MX_INLINE int16_t Mx_Sat16(int32_t x)
{
    return (int16_t)(x > 32767 ? 32767 : (x < -32768 ? -32768 : x));
}

/* 32 位饱和加法（结果限在 ±0x7FFFFFFF） */
MX_INLINE int32_t Mx_AddSat32(int32_t a, int32_t b)
{
    if (b > 0 && a > (int32_t)0x7FFFFFFF - b) return  (int32_t)0x7FFFFFFF;
    if (b < 0 && a < -(int32_t)0x7FFFFFFF - b) return -(int32_t)0x7FFFFFFF;
    return a + b;
}

/* 整数平方根 floor(sqrt(x))，逐位法约 16 次循环。只在慢环用。 */
uint32_t Mx_Isqrt32(uint32_t x);

#endif
