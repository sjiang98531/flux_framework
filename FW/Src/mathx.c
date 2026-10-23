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
