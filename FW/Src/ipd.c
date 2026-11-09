/* ============================================================================
 * @file    ipd.c
 * @brief   初始位置检测（饱和法），接口说明见 ipd.h
 * ==========================================================================*/
#include "ipd.h"
#include "cfg_derived.h"
#include "foc_core.h"
#include "meas.h"
#include "foc_math.h"
#include "qmath.h"
#include "hal.h"

volatile IpdCfg_TypeDef gIpdCfg = {
    Q15(IPD_PULSE_V / UDC_MAX),
    IPD_PULSE_TICKS,
    IPD_GAP_TICKS,
    IPD_PULSE_SHAPE
};
Ipd_TypeDef gIpd;

/* 方向之间等待时间的随机抖动：等间隔脉冲会形成单一音调，间隔在 [gap/2, 3gap/2) 内随机，
 * 使噪声频谱分散。线性同余随机数，仅用于定时，不影响测量 */
static uint32_t s_rng = 12345u;
static uint16_t s_gap;
/* 脉冲宽度与正弦相位步长（Q8，65536*256/p）。快环禁止除法（Ipd_Start 也会在快环中调用）：
 * 上电使用编译期值；gIpdCfg.pulseTicks 被修改后由主循环 Ipd_Background 重新计算，脉冲进行中不更新，宽度与步长同时更新 */
static uint16_t s_p = IPD_PULSE_TICKS;
static uint32_t s_phaseStepQ8 = (65536u << 8) / IPD_PULSE_TICKS;
/* 方向 k 的电角度 k*65536/IPD_N（Q16，65536 = 一圈），编译期计算：快环禁止除法，
 * M0+ 无除法指令，即使除以常数也会调用库函数 */
#define IPD_DIR_ANGLE(k) ((int16_t)((int32_t)(k) * 65536 / IPD_N))
static const int16_t s_dirAngle[IPD_N] = {
    IPD_DIR_ANGLE(0), IPD_DIR_ANGLE(1), IPD_DIR_ANGLE(2), IPD_DIR_ANGLE(3), IPD_DIR_ANGLE(4), IPD_DIR_ANGLE(5),
    IPD_DIR_ANGLE(6), IPD_DIR_ANGLE(7), IPD_DIR_ANGLE(8), IPD_DIR_ANGLE(9), IPD_DIR_ANGLE(10), IPD_DIR_ANGLE(11)
};
typedef char ipd_dir_table_matches_IPD_N[IPD_N == 12 ? 1 : -1];   /* 修改 IPD_N 时须同步修改上表 */

/* 下一次方向间的等待拍数：平均 gapTicks，在 [0.5, 1.5) 倍之间随机（线性同余随机数） */
static uint16_t nextGap(void)
{
    uint32_t g = gIpdCfg.gapTicks;
    s_rng = s_rng * 1664525u + 1013904223u;
    return (uint16_t)(g / 2 + (((s_rng >> 16) * g) >> 16));
}

/* 开始一轮 IPD（进入 Align 时调用）：从第 0 个方向开始 */
void Ipd_Start(void)
{
    gIpd.k = 0; gIpd.t = 0; gIpd.ok = 0;
    s_gap = nextGap();
    gIpd.runs++;
    gIpd.state = IPD_STATE_PULSING;
}

/* 主循环：gIpdCfg.pulseTicks 被修改后，重新计算正弦脉冲的相位步长（含除法，故不在快环中执行）；
 * 脉冲进行中不更新，宽度与步长同时更新 */
void Ipd_Background(void)
{
    uint16_t p = gIpdCfg.pulseTicks;
    uint32_t step;
    if (p == s_p || p == 0) return;
    step = (65536u << 8) / p;
    {
        IRQ_LOCK();
        if (gIpd.state != 1) { s_p = p; s_phaseStepQ8 = step; }
        IRQ_UNLOCK();
    }
}

/* 第 n 个脉冲的方向：0, 180, 30, 210, 60, 240 …。相反方向的两个脉冲相邻施加：
 * 不在 d 轴上的脉冲会产生转矩，反方向脉冲产生大小相等、符号相反的转矩，转子（机械时间常数约 0.5 s）
 * 所受冲量相互抵消，不会被扰动。结果仍按方向编号存储。 */
static uint16_t dirOf(uint16_t n)
{
    return (uint16_t)((n >> 1) + (n & 1) * (IPD_N / 2));
}

/* IPD 快环（旁路电流环）：在当前方向施加电压脉冲（方波 +V 半宽、-V 半宽，或一个正弦周期），
 * 脉冲期间记录电流在该方向投影的最大值 pk[k]（及垂直方向的 perp[k]，诊断用）；
 * 脉冲后输出零电压等待电流衰减（随机间隔），再切换到下一方向；12 个方向完成后 state = 2，交由慢环判定 */
void Ipd_Fast(void)
{
    int16_t v = 0;
    if (gIpd.state == 1) {
        uint16_t p = s_p, k = dirOf(gIpd.k);
        if (gIpd.t == 0) {
            gIpd.dir = FocMath_SinCos(s_dirAngle[k]);
            gIpd.pk[k] = 0; gIpd.perp[k] = 0;
        }
        if (gIpd.t < p + 2) {   /* 脉冲期间（采样滞后电压一拍）记录投影最大值 */
            int32_t ia = gMeas.iab.s16Alpha, ib = gMeas.iab.s16Beta;
            int16_t pr = (int16_t)((ia * gIpd.dir.s16Cos + ib * gIpd.dir.s16Sin) >> 15);
            if (pr > gIpd.pk[k]) {
                gIpd.pk[k] = pr;
                gIpd.perp[k] = (int16_t)((ib * gIpd.dir.s16Cos - ia * gIpd.dir.s16Sin) >> 15);
            }
        }
        /* 波形（gIpdCfg.shape），两种均为总宽 p 拍、电流起止为 0：
         *   0 方波：前半 +V、后半 -V，电流为三角波，峰值 V*(p/2)*Ts/L。同样峰值下时间最短、冲量最小，但边沿陡。
         *   1 正弦：V*sin(2*pi*t/p)，电流为钟形，峰值 V*p*Ts/(pi*L)。无尖角，但同样峰值需要更长时间。 */
        if (gIpd.t < p) {
            if (gIpdCfg.shape) v = (int16_t)(((int32_t)gIpdCfg.pulseV *
                                              FocMath_SinCos((int16_t)(((uint32_t)gIpd.t * s_phaseStepQ8) >> 8)).s16Sin) >> 15);
            else v = (int16_t)(gIpd.t < (p >> 1) ? gIpdCfg.pulseV : -gIpdCfg.pulseV);
        }
        if (++gIpd.t >= p + s_gap) {
            gIpd.t = 0;
            s_gap = nextGap();
            if (++gIpd.k >= IPD_N) { gIpd.k = 0; gIpd.state = IPD_STATE_DONE; }
        }
    }
    FocCore_ApplyVoltageAB((int16_t)(((int32_t)v * gIpd.dir.s16Cos) >> 15),
                           (int16_t)(((int32_t)v * gIpd.dir.s16Sin) >> 15));
}

/* IPD 判定（慢环，12 个方向完成后调用一次）：
 *   角度 = 各方向电流峰值的一次谐波方向（Σ pk·cos θk, Σ pk·sin θk 的 atan2），即 N 极方向；
 *   对比度 = 最大 / 最小峰值，>= IPD_MIN_RATIO 才视为可信。返回 1 = 可信 */
uint8_t Ipd_Evaluate(void)
{
    int32_t x = 0, y = 0, mx = 0, mn = 32767;
    uint16_t k;
    for (k = 0; k < IPD_N; k++) {
        sAngle_Trig a = FocMath_SinCos(s_dirAngle[k]);
        int32_t p = gIpd.pk[k];
        x += (p * a.s16Cos) >> 15;
        y += (p * a.s16Sin) >> 15;
        if (p > mx) mx = p;
        if (p < mn) mn = p;
    }
    if (mn < 1) mn = 1;
    gIpd.ratioQ8 = (uint16_t)QMath_Clamp((mx << 8) / mn, 65535);
    gIpd.theta = (int16_t)(QMath_Atan2(y, x) >> 16);
    gIpd.ok = (uint8_t)(mx > 0 && gIpd.ratioQ8 >= IPD_MIN_RATIO_Q8);
    if (!gIpd.ok) gIpd.fails++;
    gIpd.state = IPD_STATE_IDLE;
    return gIpd.ok;
}
