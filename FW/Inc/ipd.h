/* ============================================================================
 * @file    ipd.h
 * @brief   初始位置检测（IPD，饱和法）
 *
 * 静止时向 IPD_N 个方向（绝对电角度，按 0, 180, 30, 210 … 的顺序成对施加，使转矩冲量相互抵消）依次施加电压脉冲
 * （方波 +V/-V 或一个正弦周期，见 gIpdCfg.shape），之后以零矢量等待 gapTicks 拍（随机 0.5~1.5 倍）。
 * 永磁体使 N 极方向铁芯更饱和、电感更小，相同伏秒下电流最大。
 * 角度 = 各方向电流峰值的一次谐波：atan2(Σ pk·sin θk, Σ pk·cos θk)。
 * 峰值最大/最小之比 < IPD_MIN_RATIO 表示该电机饱和效应不明显（或转子在转动），判为失败，由调用方退回定位。
 *
 * 本电机 N 极方向峰值约为其余方向的 2.3 倍，一次谐波估计误差 ≤ 9°。
 * 快环（旁路电流环，直接输出电压）：Ipd_Fast；慢环判定：Ipd_Evaluate。
 * ==========================================================================*/
#ifndef IPD_H
#define IPD_H
#include <stdint.h>
#include "qtypes.h"
#include "foc_math.h"

#define IPD_N   12

typedef struct {
    int16_t  pulseV;        /* 脉冲电压幅值 Q15（UDC_MAX 基准），上电 = IPD_PULSE_V */
    uint16_t pulseTicks;    /* 脉冲总宽（PWM 拍）：方波 = 正、负各一半；正弦 = 一个周期 */
    uint16_t gapTicks;      /* 方向之间零矢量等待的平均值（PWM 拍），实际在 0.5~1.5 倍之间随机 */
    uint16_t shape;         /* 0 方波（+V 半宽、-V 半宽），1 正弦一个周期 */
} IpdCfg_TypeDef;

enum { IPD_STATE_IDLE = 0, IPD_STATE_PULSING = 1, IPD_STATE_DONE = 2 };    /* gIpd.state */

typedef struct {
    volatile uint8_t state;     /* 0 空闲，1 脉冲进行中，2 完成（等待慢环判定） */
    uint8_t  ok;                /* 上次判定：1 有效 */
    uint16_t k, t;              /* 快环：当前方向、方向内拍数 */
    sAngle_Trig dir;
    int16_t  pk[IPD_N], perp[IPD_N];    /* 各方向电流投影峰值 / 同一拍的垂直分量（Q15 电流） */
    int16_t  theta;             /* 结果：N 极电角度（Q15 角度，65536 = 一圈） */
    uint16_t ratioQ8;           /* 峰值最大/最小（Q8） */
    uint16_t runs, fails;       /* 次数统计 */
} Ipd_TypeDef;

extern volatile IpdCfg_TypeDef gIpdCfg;
extern Ipd_TypeDef gIpd;

void    Ipd_Start(void);        /* 开始一轮检测（调用方已开启 PWM 并输出零矢量） */
void    Ipd_Background(void);   /* 主循环：gIpdCfg.pulseTicks 修改后重新计算正弦相位步长（含除法，不在中断中执行） */
void    Ipd_Fast(void);         /* 快环每拍（gMeas.iab 已更新）：输出本拍电压；完成后持续输出零电压 */
uint8_t Ipd_Evaluate(void);     /* 慢环，state == 2 后调用：计算角度与对比度，返回 1 = 有效（结果在 gIpd.theta） */

#endif
