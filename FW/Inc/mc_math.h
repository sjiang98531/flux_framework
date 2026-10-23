/* ============================================================================
 * mc_math.h —— 电机控制用的基础数学（自己实现，替代原厂闭源库 mlib.o 里的函数）
 *
 *   Mc_SinCos   查表正弦/余弦（原 MLIB_SinCos）
 *   Mc_SqrtQ15  查表开方（原 Sqrt_Q15）
 *   Mc_RampQ31  斜坡（原 MLIB_Ramp_Q31）
 *
 * 表在 mc_tables.c，由 tools/gen_tables.py 按公式生成。结果与原厂库逐位相同
 * （tools/sim/libcheck.c 随机输入对比），所以替换后行为不变。
 * 类型（sAngle_Trig、MLIB_RAMP_T_Q31）沿用原厂开源头文件 mlib.h / mlib_types.h。
 * ==========================================================================*/
#ifndef MC_MATH_H
#define MC_MATH_H
#include <stdint.h>
#include "mlib.h"

/* 四分之一周期正弦表 sin(i*90°/256)，Q15 */
extern const int16_t g_mcSinTab[256];
/* 开方表 sqrt(i/512)，Q15，i = 0..512 */
extern const int16_t g_mcSqrtTab[513];

/* 角度 Q15（-32768..32767 对应 -180°..180°，即 65536 = 一圈），返回 cos/sin（Q15）。
 * 不插值，分辨率 90°/256 = 0.35°。
 * 先把角度平移半圈变成 0..65535：高 2 位是象限，接下来 8 位是表索引。
 * 平移了 180°，所以各象限的符号与常规写法差一个负号。 */
static __forceinline sAngle_Trig Mc_SinCos(int16_t angle)
{
    sAngle_Trig r;
    uint16_t u = (uint16_t)((int32_t)angle + 0x8000);
    uint32_t idx = (u >> 6) & 0xFFu;
    int16_t s = g_mcSinTab[idx];             /* sin(φ) */
    int16_t c = g_mcSinTab[255u - idx];      /* ≈ cos(φ) */
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
 * 负数按 0 处理，超过 2^30 按 1 处理（原厂库不检查，越界会读到表外）。 */
static __forceinline int16_t Mc_SqrtQ15(int32_t x)
{
    if (x <= 0) return 0;
    if (x >= (1L << 30)) return g_mcSqrtTab[512];
    return g_mcSqrtTab[x >> 21];
}

/* 斜坡：state 以 RampUp / RampDown 的步长走向 target，不越过 target。返回新的 state。 */
static __forceinline int32_t Mc_RampQ31(int32_t target, MLIB_RAMP_T_Q31 *p)
{
    int32_t st = p->s32State;
    if (st < target)      { st += p->s32RampUp;   if (st > target) st = target; }
    else if (st > target) { st -= p->s32RampDown; if (st < target) st = target; }
    p->s32State = st;
    return st;
}

#endif
