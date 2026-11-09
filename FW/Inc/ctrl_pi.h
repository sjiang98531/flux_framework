/* ============================================================================
 * @file    ctrl_pi.h
 * @brief   定点 PI 调节器（带抗积分饱和与防溢出）
 *
 * 算法：
 *   err  = InRef - InMeas
 *   sum += Ki*err，限制在 [OutMin<<15, OutMax<<15]
 *   out  = (Kp*err >> 15) + (sum >> 15)，限制在 [OutMin, OutMax]
 * 纯 32 位实现在增益较大时，大误差下 Kp*err 可能回绕成反号（等效正反馈），因此对误差限幅并使用饱和加法。
 *
 * Pi_Run32：快环使用。误差限在 ±32767、Kp/Ki <= 65535 时乘积不会溢出，积分采用饱和加法。
 * Pi_Run64：慢环使用（速度环 / 功率环），64 位乘积，Kp/Ki 可超过 65535。误差限在 int32 范围（输入为 Q15，实际远小于此），
 *           两个乘法均写成 32x32->64：Keil 对 64x64 乘法调用通用库 _ll_mul（M0+ 上约上千个时钟），
 *           对两个 32 位数扩展后相乘调用 __ARM_common_ll_mulss（几十个时钟）。
 * 状态存储使用 qtypes.h 中的 tPIParm（gFoc.piD/piQ、gSpd.pi/piPower）。
 * ==========================================================================*/
#ifndef CTRL_PI_H
#define CTRL_PI_H
#include "qtypes.h"
#include "compiler.h"
#include "qmath.h"

/**
 * @brief  快环 PI（电流环）
 *         输入 qInRef（指令）/ qInMeas（实测），输出 qOut，积分状态 qdSum（= 积分值 << 15）。
 *         误差限幅到 ±32767 -> 积分 sum += Ki*err（饱和加法）并限制在输出上下限 << 15（抗积分饱和）
 *         -> 输出 = Kp*err >> 15 + sum >> 15，再限幅。全部为 32 位乘加，每拍几十个时钟。
 * @param  p  PI 参数与状态
 */
FORCE_INLINE void Pi_Run32(tPIParm *p)
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

/**
 * @brief  慢环 PI（速度环）
 *         与 Pi_Run32 相同，但乘积与积分使用 64 位，增益可超过 65535。
 *         积分写回 int32 前已限制在输出范围 << 15 以内，不会溢出。
 * @param  p  PI 参数与状态
 */
FORCE_INLINE void Pi_Run64(tPIParm *p)
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
