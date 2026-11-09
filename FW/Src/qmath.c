/* ============================================================================
 * @file    qmath.c
 * @brief   不适合内联的定点数学函数（开方、单位化、atan2），接口说明见 qmath.h
 * ==========================================================================*/
#include "qmath.h"

/* floor(sqrt(x))，逐位法（二进制手算开方）：从最高偶数位开始，每次尝试将结果的一位置 1，
 * 余数足够则保留。只用移位、加减和比较，无乘除法 */
uint32_t QMath_Isqrt32(uint32_t x)
{
    uint32_t r = 0, bit = 1UL << 30;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

/* 同 QMath_Isqrt32，64 位输入 */
uint32_t QMath_Isqrt64(uint64_t x)
{
    uint64_t r = 0, bit = (uint64_t)1 << 62;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return (uint32_t)r;
}

/* 1/sqrt 初值：x 归一化到 [2^28, 2^30) 后按 x >> 26（4..15）取 2^31 / sqrt(i + 0.5)，即 2^44 / sqrt(x) 的区间中点 */
static const uint32_t s_rsqrtSeed[16] = {
    0, 0, 0, 0, 1012333500, 915690104, 842312387, 784150157, 736580814, 696735698, 662727842, 633258380,
    607400100, 584471019, 563956835, 545461392
};

/* 将 (a, b) 单位化为 Q15（见 qmath.h）。步骤：
 *   1. x = a^2 + b^2，左移偶数位归一化到 [2^28, 2^30)，记录移位量 e；
 *   2. 查表得 1/sqrt(x) 的初值，牛顿迭代 3 次：y <- y * (3 - x*y^2) / 2（仅乘法）；
 *   3. 结果 = a * 32767 * y，再按 e/2 移回。 */
void QMath_NormQ15(int32_t a, int32_t b, int16_t *na, int16_t *nb)
{
    uint32_t x = (uint32_t)(a * a) + (uint32_t)(b * b), y, t, u;
    int32_t e = 0, sh, ra, rb;
    int i;
    if (x == 0) { *na = 32767; *nb = 0; return; }
    /* 归一化到 [2^28, 2^30)，移位量取偶数（开方后为整数位） */
    if (x < (1UL << 14)) { x <<= 16; e += 16; }
    if (x < (1UL << 22)) { x <<= 8;  e += 8; }
    if (x < (1UL << 26)) { x <<= 4;  e += 4; }
    if (x < (1UL << 28)) { x <<= 2;  e += 2; }
    if (x >= (1UL << 30)) { x >>= 2; e -= 2; }
    y = s_rsqrtSeed[x >> 26];               /* y ~ 2^44 / sqrt(x)，在 (2^29, 2^30] */
    for (i = 0; i < 3; i++) {
        t = (uint32_t)(((uint64_t)y * y) >> 30);
        u = (uint32_t)(((uint64_t)x * t) >> 28);                    /* = 2^30 * x y^2 / 2^88，收敛时 2^30 */
        y = (uint32_t)(((uint64_t)y * (uint32_t)(3UL * (1UL << 30) - u)) >> 31);
    }
    /* a / sqrt(a^2+b^2) * 32767 = a * 32767 * y * 2^(e/2) / 2^44 */
    sh = 44 - e / 2;
    ra = (int32_t)(((int64_t)(int32_t)(a * 32767) * (int32_t)y) >> sh);
    rb = (int32_t)(((int64_t)(int32_t)(b * 32767) * (int32_t)y) >> sh);
    *na = QMath_Sat16(ra);
    *nb = QMath_Sat16(rb);
}

/* atan(2^-i)，2^32 = 一圈 */
static const uint32_t s_atanTab[16] = {
    536870912, 316933406, 167458907, 85004756, 42667331, 21354465, 10679838, 5340245, 2670163, 1335087,
    667544, 333772, 166886, 83443, 41722, 20861
};

/* atan2(y, x)，CORDIC 向量模式：将向量 (x, y) 逐次旋转 ±atan(2^-i)，每次朝 x 轴方向旋转（使 y 趋于 0），
 * 累加的旋转角即为原向量角度。每次旋转只需移位与加减（旋转角 atan(2^-i) 的正切为 2^-i）。
 * 输出 2^32 = 一圈（0x40000000 = 90°），16 次迭代，精度约 0.002° */
uint32_t QMath_Atan2(int32_t y, int32_t x)
{
    uint32_t ang = 0;
    int i;
    if (x == 0 && y == 0) return 0;
    /* 归一化到 2^28 <= max(|x|,|y|) <= 2^29：上限为 CORDIC 增益 1.65 留出余量（并避免 -2^31 取负溢出），
     * 下限保证小输入也有足够的有效位 */
    while (x > (1L << 29) || x < -(1L << 29) || y > (1L << 29) || y < -(1L << 29)) { x >>= 1; y >>= 1; }
    while (x < (1L << 28) && x > -(1L << 28) && y < (1L << 28) && y > -(1L << 28)) { x <<= 1; y <<= 1; }
    if (x < 0) { x = -x; y = -y; ang = 0x80000000UL; }      /* 旋转 180°，落入右半平面 */
    for (i = 0; i < 16; i++) {
        int32_t xs = x >> i, ys = y >> i;
        if (y > 0) { x += ys; y -= xs; ang += s_atanTab[i]; }
        else       { x -= ys; y += xs; ang -= s_atanTab[i]; }
    }
    return ang;
}
