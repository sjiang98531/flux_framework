/* ============================================================================
 * @file    ipd.h
 * @brief   初始位置检测（IPD，静音：高频注入测凸极轴向 + 直流偏置判 N/S 极）
 *
 * 两步，全程无大电流脉冲，约 0.17 s，转子不动：
 *   1. 轴向：静止时在 α、β 方向依次注入每个 PWM 周期翻转一次的方波电压（PWM_FREQ / 2，电流为幅值 IPD_HF_A 的小三角波）。
 *      每拍的电流增量 = T * L^-1 * v，L^-1 = S*I + D*[cos2θ sin2θ; sin2θ -cos2θ]（S = (1/Ld + 1/Lq)/2，D = (1/Ld - 1/Lq)/2，
 *      θ = 转子 d 轴）。α 注入的 α、β 响应和 β 注入的 α、β 响应给出 2θ = atan2(αβ + βα, αα - ββ)：
 *      响应最大（电感最小）的方向就是 d 轴（Ld < Lq；本电机实测 Ld/Lq = 0.68）。凸极比 |D|/S < IPD_MIN_SAL 判失败。
 *   2. 极性：沿测得的轴向用电流环把 d 轴电流缓慢升到 +IPD_BIAS_A、再降到 -IPD_BIAS_A（斜坡，无冲击；沿 d 轴不产生转矩），
 *      两处各测一次同样注入下的 d 轴响应：电流磁场与永磁同向时铁芯更饱和、电感更小、响应更大，大的一侧就是 N 极。
 *      两侧之比 < IPD_MIN_RATIO 判失败。
 * 判失败时由调用方退回摆动起动。快环：Ipd_Fast（输出电压、累计响应）；慢环：Ipd_Slow（计算轴向、控制偏置电流、判定）。
 * ==========================================================================*/
#ifndef IPD_H
#define IPD_H
#include <stdint.h>
#include "qtypes.h"
#include "foc_math.h"

/* gIpd.state */
enum { IPD_STATE_IDLE = 0, IPD_STATE_HF = 1, IPD_STATE_AXIS = 2, IPD_STATE_BIAS = 3, IPD_STATE_DONE = 4 };

typedef struct {
    volatile uint8_t state;     /* 见上；DONE 后由慢环调用 Ipd_Evaluate */
    uint8_t  ok;                /* 上次判定：1 有效 */
    int16_t  theta;             /* 结果：N 极电角度（Q15 角度，65536 = 一圈） */
    uint16_t ratioQ8;           /* 极性对比度：两侧 d 轴响应大 / 小（Q8） */
    uint16_t runs, fails;       /* 次数统计 */
} Ipd_TypeDef;

extern Ipd_TypeDef gIpd;

void    Ipd_Start(void);        /* 开始一轮检测（调用方已开启 PWM 并输出零矢量） */
void    Ipd_Fast(void);         /* 快环每拍（gMeas.iab 已更新）：输出本拍电压；完成后输出零电压 */
void    Ipd_Slow(void);         /* 慢环每拍（检测进行中）：轴向计算、偏置电流斜坡、极性判定 */
uint8_t Ipd_Evaluate(void);     /* 慢环，state == DONE 后调用：返回 1 = 有效（结果在 gIpd.theta） */

#endif
