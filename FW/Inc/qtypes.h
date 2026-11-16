/* ============================================================================
 * @file    qtypes.h
 * @brief   定点数据类型与换算宏
 *
 * 内容：Q15() / Q31() 换算宏，以及 6 个基础结构体（坐标、sin/cos 对、斜坡、一阶滤波、PI、PLL）。
 * 坐标变换（Clarke / Park / 反 Park）见 foc_math.h，Q15 乘法见 qmath.h。
 * 说明：
 *   - 32 位量一律使用 int32_t（而非 long），保证目标板与 64 位主机上的溢出行为一致。
 *   - 本文件不定义 TRUE / FALSE，以免与芯片头文件中的 boolType 枚举冲突。
 * ==========================================================================*/
#ifndef QTYPES_H
#define QTYPES_H
#include <stdint.h>

/* 实数 -> 定点：截断取整，超出范围时饱和（cfg_derived.h 中的增益均由此宏换算） */
#define Q15(x) ((int16_t)((x) < 0.999969482421875 ? ((x) >= -1 ? (x)*0x8000 : 0x8000) : 0x7FFF))
#define Q31(x) ((int32_t)((x) < 1 ? ((x) >= -1 ? (x)*0x80000000 : 0x80000000) : 0x7FFFFFFF))

/* 三相 / 静止两相 / 旋转两相，Q15 */
typedef struct { int16_t s16A, s16B, s16C; } Abc_TypeDef;
typedef struct { int16_t s16Alpha, s16Beta; } AlphaBeta_TypeDef;
typedef struct { int16_t s16D, s16Q; } Dq_TypeDef;

/* 一个角度的 cos / sin，Q15（FocMath_SinCos 的返回值） */
typedef struct { int16_t s16Cos, s16Sin; } SinCos_TypeDef;

/* 速度斜坡（speed_ctrl.c）：升 / 降步长、当前值，Q31 */
typedef struct {
    int32_t s32RampUp;    /* 加速时每拍的增量 */
    int32_t s32RampDown;    /* 减速时每拍的减量（正数） */
    int32_t s32State;    /* 当前值 */
} Ramp_TypeDef;

/* 一阶滑动平均（QMath_LowPass）：累加器、窗口 2^u16Sh */
typedef struct {
    int32_t  s32Acc;    /* 累加器（稳态 = x * 2^sh - x） */
    uint16_t u16Sh;    /* 移位位数，时间常数约 2^sh 拍 */
} LowPass_TypeDef;

/* PI 调节器（foc_math.h Pi_Run32 / Pi_Run64）：积分和（Q15 << 15）、增益、限幅、输入、输出 */
typedef struct {
    int32_t qdSum;    /* 积分值 << 15（输出单位） */
    int32_t qKp, qKi;    /* 比例 / 积分增益（输出 += K * 误差 >> 15） */
    int32_t qOutMax, qOutMin;    /* 输出上下限（积分也限在这里面，抗饱和） */
    int32_t qInRef, qInMeas;    /* 给定、实测 */
    int32_t qOut;    /* 输出 */
} PI_TypeDef;

/* 磁链 PLL（flux_obs.c）：角度 / 转速 Q31，积分状态，增益 */
typedef struct {
    int32_t s32Theta;    /* 电角度 Q16.16（>> 16 得 Q15 角度，65536 = 一圈） */
    int32_t s32Speed;    /* 转速 Q31（0x7FFFFFFF ≈ SPEED_MAX） */
    int32_t s32I_1;    /* 积分器（转速的稳态部分） */
    int16_t s16IGain;    /* 积分增益：每 1 LSB 误差每拍的 Q31 转速增量 */
    int16_t s16PGain;    /* 比例增益：每 1 LSB 误差的 Q31 转速增量 */
    int16_t s16ThGain;    /* 转速 -> 每拍角度增量（SPEED_TO_THETA） */
} PLL_TypeDef;

#endif
