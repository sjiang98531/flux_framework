/* 转子磁链 PLL，见 flux_pll.h。
 *
 * 为什么自己写：原厂 MLIB_PLL 是按 SMO 的反电势（幅值正比于转速）整定的，
 * 磁链幅值恒定，套原厂增益会让带宽高速塌、低速炸。自己掌握环路后，增益按
 * 实际磁链幅值一次算好，全速域通用。
 *
 * 二阶环路模型：err = Kd*sin(dth)，Kd = |psi|（Q15 LSB/rad）
 *   speed = Kp*err + Ki*sum(err)  ->  wn = sqrt(Kd*Ki*fs/Ks)，2*zeta*wn = Kd*Kp/Ks
 *   Ks = 2^31 / (2*pi*SPEED_MAX*p/60) = 1281685：1 rad/s 电角速度的 Q31 值
 */
#include "flux_pll.h"
#include "mathx.h"

volatile FluxPLL_Diag_TypeDef gFluxPLLDiag;

/* 原厂 MLIB_PLL 不限幅，Q31 速度过 2^31 直接回绕（原厂固件恰好在 3200 rpm 坏掉的原因）。
 * 限在 0x7FFF8000：3199.9 rpm 仍可达，回绕前一个 LSB 停住。 */
#define SPEED_LIMIT 0x7FFF8000L

void FluxPLL_Init(PLL_TypeDef *p)
{
    /* 字段是 int16：超了就封顶（不封顶会回绕成负增益，PLL 正反馈发散）。
     * 封顶时 gFluxDiag.cfgWarn bit2 置位（diag.c）。 */
    p->s16PGain    = FLUXPLL_KP_Q31_PER_ERR > 32767 ? 32767 : FLUXPLL_KP_Q31_PER_ERR;
    p->s16IGain    = FLUXPLL_KI_Q31_PER_ERR > 32767 ? 32767 : FLUXPLL_KI_Q31_PER_ERR;
    p->i16PGainSh  = 0;
    p->i16IGainSh  = 0;
    p->s16ThGain   = (int16_t)SPEED_TO_THETA;
    p->i16ThGainSh = 0;
    p->s32Theta    = 0;
    p->s32Speed    = 0;
    p->s32I_1      = 0;
    gFluxPLLDiag.errLast   = 0;
    gFluxPLLDiag.speedLast = 0;
    gFluxPLLDiag.thetaLast = 0;
    gFluxPLLDiag.satCount  = 0;
    gFluxPLLDiag.floorCount= 0;
    gFluxPLLDiag.buildTag  = FW_BUILD_TAG;
}

int16_t FluxPLL_PhaseError(int16_t psiAlpha, int16_t psiBeta, int16_t theta)
{
    sAngle_Trig a = MLIB_SinCos(theta);
    return (int16_t)(MLIB_Mul_Q15(psiBeta, a.s16Cos) - MLIB_Mul_Q15(psiAlpha, a.s16Sin));
}

static __forceinline int32_t speedClamp(int32_t v)
{
    if (v >  SPEED_LIMIT) { gFluxPLLDiag.satCount++; return  SPEED_LIMIT; }
    if (v < -SPEED_LIMIT) { gFluxPLLDiag.satCount++; return -SPEED_LIMIT; }
    return v;
}

/* 角度步进 = (speed * SPEED_TO_THETA) >> 15（与原厂 MLIB_PLL 一致，反汇编核实）。
 * (speed>>15)*kTheta 保持 32 位乘法，最多丢 kTheta 个 Q16.16 LSB，由积分吸收，不累积。 */
static __forceinline int32_t angleStep(int32_t speed, int16_t kTheta)
{
    return (int32_t)(speed >> 15) * (int32_t)kTheta;
}

static __forceinline int16_t advance(PLL_TypeDef *p, int32_t kpTerm, int32_t kiTerm, int32_t minSpeed)
{
    p->s32I_1   = speedClamp(Mx_AddSat32(p->s32I_1, kiTerm));
    p->s32Speed = speedClamp(Mx_AddSat32(p->s32I_1, kpTerm));
    /* test29：只在 Startup 生效（Spin 传 0，无操作）。
     * 磁链鉴相在零速时幅值是满的：err = -336 counts 就能把下限抵掉
     * （floorQ31/Kp = 8.59e6/25583 = 336），PLL 停住、转子不动。所以下限必须
     * 加在这里——算出 s32Speed 之后、推进 s32Theta 之前。放在 advance() 返回
     * 之后只是"显示"12.8 rpm，角度照样冻住。 */
    if (minSpeed > 0 && p->s32Speed < minSpeed) p->s32Speed = minSpeed;
    if (minSpeed < 0 && p->s32Speed > minSpeed) p->s32Speed = minSpeed;
    /* Q16.16 累加器全速时每 60 拍回绕一圈，经 uint32 相加保证回绕有定义 */
    p->s32Theta = (int32_t)((uint32_t)p->s32Theta + (uint32_t)angleStep(p->s32Speed, p->s16ThGain));
    gFluxPLLDiag.speedLast = (int16_t)(p->s32Speed >> 16);
    gFluxPLLDiag.thetaLast = (uint16_t)(p->s32Theta >> 16);
    return (int16_t)(p->s32Theta >> 16);
}

int16_t FluxPLL_StepQ15(const PLL_TypeDef *p, int32_t advQ8)
{
    /* 一拍角度增量（Q16.16）* advQ8/256，取高 16 位。先 >>8 防溢出 */
    int32_t step = angleStep(p->s32Speed, p->s16ThGain);
    return (int16_t)(((step >> 8) * advQ8) >> 16);
}

int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p)
{
    int32_t kpTerm = (int32_t)err * p->s16PGain;
    int32_t kiTerm = (int32_t)err * p->s16IGain;
    gFluxPLLDiag.errLast = err;
    return advance(p, kpTerm, kiTerm, 0);
}

int16_t FluxPLL_RunStartup(int16_t err, PLL_TypeDef *p, int16_t spdCmd, int32_t floorQ31)
{
    int32_t kpTerm = (int32_t)err * p->s16PGain;
    int32_t kiTerm = (int32_t)err * p->s16IGain;
    int32_t integ  = Mx_AddSat32(p->s32I_1, kiTerm);

    gFluxPLLDiag.errLast = err;
    /* 原厂 MLIB_PLL1 语义（反汇编）：下限加在积分器上，误差为 0 时仍有最低速度
     * 拖着转子走；负方向分支会把负的积分器直接抬到 +floor，换向时能干净地重启。 */
    if (spdCmd > 0) {
        if (integ < floorQ31) { integ = floorQ31; gFluxPLLDiag.floorCount++; }
    } else {
        if (integ > -floorQ31) { integ = -floorQ31; gFluxPLLDiag.floorCount++; }
    }
    p->s32I_1 = integ;
    return advance(p, kpTerm, 0, spdCmd > 0 ? floorQ31 : -floorQ31);
}
