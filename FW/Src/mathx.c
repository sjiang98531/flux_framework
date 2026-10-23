#include "mathx.h"

uint32_t Mx_Isqrt32(uint32_t x)
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

uint32_t Mx_Isqrt64(uint64_t x)
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

/* atan(2^-i)，2^32 = 一圈 */
static const uint32_t s_atanTab[20] = {
    536870912, 316933406, 167458907, 85004756, 42667331, 21354465, 10679838, 5340245, 2670163, 1335087,
    667544, 333772, 166886, 83443, 41722, 20861, 10430, 5215, 2608, 1304
};

uint32_t Mx_Atan2(int32_t y, int32_t x)
{
    uint32_t ang = 0;
    int i;
    if (x == 0 && y == 0) return 0;
    /* 归一化到 2^28 <= max(|x|,|y|) <= 2^29：上限给 CORDIC 增益 1.65 留余量（也避开 -2^31 取负溢出），
     * 下限保证小输入也有足够的有效位 */
    while (x > (1L << 29) || x < -(1L << 29) || y > (1L << 29) || y < -(1L << 29)) { x >>= 1; y >>= 1; }
    while (x < (1L << 28) && x > -(1L << 28) && y < (1L << 28) && y > -(1L << 28)) { x <<= 1; y <<= 1; }
    if (x < 0) { x = -x; y = -y; ang = 0x80000000UL; }      /* 转 180°，落到右半平面 */
    for (i = 0; i < 20; i++) {
        int32_t xs = x >> i, ys = y >> i;
        if (y > 0) { x += ys; y -= xs; ang += s_atanTab[i]; }
        else       { x -= ys; y += xs; ang -= s_atanTab[i]; }
    }
    return ang;
}
