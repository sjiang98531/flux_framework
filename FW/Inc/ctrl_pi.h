/* ============================================================================
 * ctrl_pi.h —— PI 调节器（替代原厂闭源 CalcPI，语义相同 + 防溢出）
 *
 * 原厂 CalcPI（反汇编 0x0800093C）：
 *   err  = InRef - InMeas
 *   sum += Ki*err，限在 [OutMin<<15, OutMax<<15]
 *   out  = (Kp*err >> 15) + (sum >> 15)，限在 [OutMin, OutMax]
 * 全部 32 位、不检查溢出：增益一大（按理论把电流环提到几百 Hz、速度环加大
 * 阻尼时就会），大误差下 Kp*err 回绕成反号，等于正反馈。
 *
 * Pi_Run32：快环用。误差限在 ±32767、Kp/Ki <= 65535 时乘积不会溢出，积分用
 *           饱和加法。正常工作范围内与 CalcPI 逐位相同。
 * Pi_Run64：慢环用（速度环/功率环），64 位乘法，Kp/Ki 可以超过 65535。
 * 存储仍用原厂开源头文件里的 tPIParm（FOC.CurID/CurIQ/Speed/Power）。
 * ==========================================================================*/
#ifndef CTRL_PI_H
#define CTRL_PI_H
#include "mlib.h"
#include "mathx.h"

static __forceinline void Pi_Run32(tPIParm *p)
{
    int32_t err = (int32_t)p->qInRef - (int32_t)p->qInMeas;
    int32_t hi = (int32_t)p->qOutMax << 15, lo = (int32_t)p->qOutMin << 15;
    int32_t sum, out;
    if (err > 32767) err = 32767;
    if (err < -32767) err = -32767;
    sum = Mx_AddSat32((int32_t)p->qdSum, (int32_t)p->qKi * err);
    if (sum >= hi) sum = hi;
    else if (sum <= lo) sum = lo;
    p->qdSum = sum;
    out = (((int32_t)p->qKp * err) >> 15) + (sum >> 15);
    if (out > p->qOutMax) out = p->qOutMax;
    else if (out < p->qOutMin) out = p->qOutMin;
    p->qOut = out;
}

static __forceinline void Pi_Run64(tPIParm *p)
{
    int64_t err = (int64_t)p->qInRef - (int64_t)p->qInMeas;
    int64_t hi = (int64_t)p->qOutMax << 15, lo = (int64_t)p->qOutMin << 15;
    int64_t sum = (int64_t)p->qdSum + (int64_t)p->qKi * err;
    int64_t out;
    if (sum >= hi) sum = hi;
    else if (sum <= lo) sum = lo;
    p->qdSum = (int32_t)sum;
    out = (((int64_t)p->qKp * err) >> 15) + (sum >> 15);
    if (out > p->qOutMax) out = p->qOutMax;
    else if (out < p->qOutMin) out = p->qOutMin;
    p->qOut = (int32_t)out;
}

#endif
