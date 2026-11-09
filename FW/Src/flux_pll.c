/* ============================================================================
 * @file    flux_pll.c
 * @brief   转子磁链锁相环，接口说明见 flux_pll.h
 *
 * 以磁链（而非反电势）鉴相：反电势幅值正比于转速，鉴相增益随转速变化；磁链幅值恒定，
 * 增益按实际磁链幅值一次计算即可在全速域通用。
 *
 * 二阶环路模型：err = Kd*sin(dth)，Kd = |psi|（Q15 LSB/rad）
 *   speed = Kp*err + Ki*sum(err)  ->  wn = sqrt(Kd*Ki*fs/Ks)，2*zeta*wn = Kd*Kp/Ks
 *   Ks = 2^31 / (2*pi*SPEED_MAX*p/60) = 1281685：1 rad/s 电角速度的 Q31 值
 * ==========================================================================*/
#include "flux_pll.h"
#include "qmath.h"
#include "compiler.h"
#include "param.h"

volatile FluxPLL_Diag_TypeDef gPllDiag;

/* 转速限幅：Q31 速度超过 2^31 会回绕，故限制在 0x7FFF8000（3199.9 rpm 仍可达，在回绕前停住）。 */
#define SPEED_LIMIT 0x7FFF8000L

/* 鉴相增益随 psi_f 变化：增益由 param.c 计算（封顶 32767） */
void FluxPLL_LoadGains(PLL_TypeDef *p)
{
    p->s16PGain = gGain.pllKp;
    p->s16IGain = gGain.pllKi;
}

/* 装载增益并清除全部状态（角度、转速、积分器、诊断）。上电及飞车接管（Est_OnCatch）时调用 */
void FluxPLL_Init(PLL_TypeDef *p)
{
    /* 增益字段为 int16：超出范围时封顶（否则会回绕为负增益，导致 PLL 正反馈发散）。
     * 封顶时 gFluxDiag.cfgWarn bit2 置位（diag.c）。 */
    FluxPLL_LoadGains(p);
    p->s16ThGain   = (int16_t)SPEED_TO_THETA;
    p->s32Theta    = 0;
    p->s32Speed    = 0;
    p->s32I_1      = 0;
    gPllDiag.errLast   = 0;
    gPllDiag.speedLast = 0;
    gPllDiag.thetaLast = 0;
    gPllDiag.satCount  = 0;
    gPllDiag.buildTag  = BUILD_TAG;
}

/* 鉴相器：磁链矢量与估计角方向的叉积 = |psi| * sin(真实角 - 估计角)。误差较小时 ≈ |psi| * 角度误差（rad），
 * 估计角落后时为正（PLL 加速追赶） */
RAMFUNC int16_t FluxPLL_PhaseError(int16_t psiAlpha, int16_t psiBeta, int16_t theta)
{
    sAngle_Trig a = FocMath_SinCos(theta);
    return (int16_t)(QMath_MulQ15(psiBeta, a.s16Cos) - QMath_MulQ15(psiAlpha, a.s16Sin));
}

/* 转速限幅在 ±SPEED_MAX（防止 Q31 回绕），触及限幅时计数 */
FORCE_INLINE int32_t speedClamp(int32_t v)
{
    if (v >  SPEED_LIMIT) { gPllDiag.satCount++; return  SPEED_LIMIT; }
    if (v < -SPEED_LIMIT) { gPllDiag.satCount++; return -SPEED_LIMIT; }
    return v;
}

/* 角度步进 = (speed * SPEED_TO_THETA) >> 15。
 * 写成 (speed>>15)*kTheta 以保持 32 位乘法，最多损失 kTheta 个 Q16.16 LSB，由积分吸收，不累积。 */
FORCE_INLINE int32_t angleStep(int32_t speed, int16_t kTheta)
{
    return (int32_t)(speed >> 15) * (int32_t)kTheta;
}

/* PI 一步并积分得到角度：积分器 += Ki*err，转速 = 积分器 + Kp*err，角度 += 转速 * SPEED_TO_THETA。返回新角度 */
FORCE_INLINE int16_t advance(PLL_TypeDef *p, int32_t kpTerm, int32_t kiTerm)
{
    p->s32I_1   = speedClamp(QMath_AddSat32(p->s32I_1, kiTerm));
    p->s32Speed = speedClamp(QMath_AddSat32(p->s32I_1, kpTerm));
    /* Q16.16 累加器在最高转速下约每 60 拍回绕一圈，经 uint32 相加保证回绕行为有定义 */
    p->s32Theta = (int32_t)((uint32_t)p->s32Theta + (uint32_t)angleStep(p->s32Speed, p->s16ThGain));
    gPllDiag.speedLast = (int16_t)(p->s32Speed >> 16);
    gPllDiag.thetaLast = (uint16_t)(p->s32Theta >> 16);
    return (int16_t)(p->s32Theta >> 16);
}

/* 按当前转速外推 advQ8/256 拍内转过的角度。用于反 Park：电压在下一个 PWM 周期才生效，届时转子已转过该角度 */
RAMFUNC int16_t FluxPLL_StepQ15(const PLL_TypeDef *p, int32_t advQ8)
{
    /* 一拍角度增量（Q16.16）* advQ8/256，取高 16 位。先右移 8 位防溢出 */
    int32_t step = angleStep(p->s32Speed, p->s16ThGain);
    return (int16_t)(((step >> 8) * advQ8) >> 16);
}

/* PLL 一拍（快环）：输入鉴相误差，输出新的估计角（Q15 角度），转速保存在 p->s32Speed */
RAMFUNC int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p)
{
    int32_t kpTerm = (int32_t)err * p->s16PGain;
    int32_t kiTerm = (int32_t)err * p->s16IGain;
    gPllDiag.errLast = err;
    return advance(p, kpTerm, kiTerm);
}
