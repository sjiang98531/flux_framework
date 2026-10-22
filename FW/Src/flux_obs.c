/* 转子磁链观测器，见 flux_obs.h。每个设计选择背后的实验在 flux_selfpll 的
 * 实验记录.md / README.md 里（test4~test33）。 */
#include "flux_obs.h"
#include "foc_core.h"
#include "diag.h"
#include "mathx.h"

FluxObs_TypeDef sFluxObs;
FluxInput_TypeDef gFluxInput;

#define FMAX ((int32_t)29491L * 65536L)
#define PSI_LIMIT 23170
/* |psi_f|^2，以磁链 LSB 的平方计（NLFO 用） */
#define FLUX_NOM2   ((int32_t)FLUX_NOMINAL_Q15 * (int32_t)FLUX_NOMINAL_Q15)

static __forceinline int16_t inputClamp(int32_t x)
{
    if (x > 29491 || x < -29491) gFluxDiag.inputSatCount++;
    return (int16_t)Mx_Clamp(x, 29491);
}
static __forceinline int32_t addFlux(int32_t x, int32_t dx)
{
    int32_t sum = x + dx;
    if (sum > FMAX || sum < -FMAX) gFluxDiag.inputSatCount++;
    return Mx_Clamp(sum, FMAX);
}

void FluxObs_Capture(const FOC_TypeDef *f, uint8_t valid)
{
    const PWM_GEN_T *p = &pwm_gen;
    gFluxInput.ia = f->sIabc.s16A; gFluxInput.ib = f->sIabc.s16B; gFluxInput.ic = f->sIabc.s16C;
    gFluxInput.bus = f->s16Vbus;
    gFluxInput.compareA = p->CompA; gFluxInput.compareB = p->CompB; gFluxInput.compareC = p->CompC;
    gFluxInput.period = p->N_halfPeriod; gFluxInput.pwmMode = p->Mode; gFluxInput.pwmValid = valid;
}

void FluxObs_Reset(void)
{
    sFluxObs.s32PsiAlF = 0; sFluxObs.s32PsiBeF = 0;
    sFluxObs.sEalAvgMA.s32Acc = 0; sFluxObs.sEbeAvgMA.s32Acc = 0;
    sFluxObs.s16PsiAl = 0; sFluxObs.s16PsiBe = 0;
    sFluxObs.s16PsiRAl = 0; sFluxObs.s16PsiRBe = 0; sFluxObs.s16PsiRMag = 0;
    sFluxObs.s16Eal = 0; sFluxObs.s16Ebe = 0;
    sFluxObs.u8DivCnt = 0; sFluxObs.haveCurrent = 0;
    gFluxDiag.voltageAlpha = 0; gFluxDiag.voltageBeta = 0;
}

void FluxObs_Init(void)
{
    sFluxObs.s16KRs = FLUX_KRS_Q15;
    sFluxObs.s16KLs = FLUX_KLQ_Q15;
    sFluxObs.s16KTs = FLUX_KTS_Q15;
    sFluxObs.sEalAvgMA.u16Sh = FLUX_HPF_SH; sFluxObs.sEbeAvgMA.u16Sh = FLUX_HPF_SH;
    FluxObs_Reset();
}

int16_t FluxObs_TargetMag(void)
{
    return FLUX_NOMINAL_Q15;
}

void FluxObs_BeginStartup(FOC_TypeDef *f, MLIB_FILTER_MA_T *speedFilter)
{
    /* 只在冷启动 Align -> Startup 调用。之前的飞车检测也用过同一个 sPLL，
     * 它残留的速度不能带进新观测器。调用方已对齐的角度、PLL 增益和所有 PI 保留。 */
    FluxObs_Reset();
    f->sPLL.s32Speed = 0; f->sPLL.s32I_1 = 0;
    f->FluxFeedback.s16Speed = 0;
    f->FluxFeedback.s16Ealpha_est = 0; f->FluxFeedback.s16Ebeta_est = 0;
    f->s16SpdFilt = 0; speedFilter->s32Acc = 0;
    /* test23：psi 初值种在 |psi_f|∠theta_hat，而不是 0。psi 从 0 起步时 NLFO
     * 约 30 ms 把幅值拉起来，但方向随 Rs/死区误差走，PLL 误差接近满量程，
     * theta_hat 甩半圈，造成 32~34 ms 后的 177~191 rpm 假转速和假切 Spin。
     * 调用方已设 sPLL.s32Theta = s16CmdTheta<<16，所以第一拍 PLL 误差正好为 0。 */
    {
        sAngle_Trig a = MLIB_SinCos(f->s16CmdTheta);
        sFluxObs.s32PsiAlF = (int32_t)Mx_MulQ15(FLUX_NOMINAL_Q15, a.s16Cos) << 16;
        sFluxObs.s32PsiBeF = (int32_t)Mx_MulQ15(FLUX_NOMINAL_Q15, a.s16Sin) << 16;
    }
}

static void FluxObs_Run(const FOC_TypeDef *f)
{
    MCFLIB_2_ALBE_T_S16 current;
    int16_t va, vb, ea, eb, ia, ib;
    int32_t x, y, corrA, corrB;
    if (!gFluxInput.pwmValid || gFluxInput.period != PWM_PERIOD || gFluxInput.bus < Q15(FLUX_BUS_MIN_V / UDC_MAX) ||
        gFluxInput.compareA < 0 || gFluxInput.compareA > PWM_PERIOD ||
        gFluxInput.compareB < 0 || gFluxInput.compareB > PWM_PERIOD ||
        gFluxInput.compareC < 0 || gFluxInput.compareC > PWM_PERIOD) {
        FluxObs_Reset();
        return;
    }
    current = f->sIAlBe;
    FocCore_VoltageFromCompares(gFluxInput.compareA, gFluxInput.compareB, gFluxInput.compareC,
                                gFluxInput.bus, &va, &vb);
    gFluxDiag.voltageAlpha = va; gFluxDiag.voltageBeta = vb;

    /* 电阻压降用区间平均电流（一段时间里的积分量）；Ls*i 用本拍瞬时值（状态量） */
    ia = current.s16Alpha; ib = current.s16Beta;
    if (sFluxObs.haveCurrent) {
        ia = ((int32_t)ia + sFluxObs.previousIa) / 2;
        ib = ((int32_t)ib + sFluxObs.previousIb) / 2;
    }
    sFluxObs.previousIa = current.s16Alpha; sFluxObs.previousIb = current.s16Beta; sFluxObs.haveCurrent = 1;

    ea = inputClamp((int32_t)va - Mx_MulQ15(sFluxObs.s16KRs, ia));
    eb = inputClamp((int32_t)vb - Mx_MulQ15(sFluxObs.s16KRs, ib));
    sFluxObs.s16Eal = ea; sFluxObs.s16Ebe = eb;
    ea = inputClamp((int32_t)ea - MLIB_FilterMA_Q15(ea, &sFluxObs.sEalAvgMA));
    eb = inputClamp((int32_t)eb - MLIB_FilterMA_Q15(eb, &sFluxObs.sEbeAvgMA));
    if (++sFluxObs.u8DivCnt >= 16) { sFluxObs.u8DivCnt = 0; }

    /* ---- NLFO 径向修正（借鉴 SguanFOC_Library Sguan_NLFO.c）--------------
     *     corr = Gain * psi_r * (|psi_f|^2 - |psi_r|^2)
     * 修正方向永远平行于 psi_r：电压误差只能改幅值，拉不动角度（老的 e/omega
     * 参考拉法有切向分量，把角度拉歪：test4/5/7/8）。
     * 定标（磁链 LSB，6111 = |psi_f|）：
     *   ferr = |psi_f|^2 - |psi_r|^2          |.| <= 1.07e9
     *   fq   = ferr/|psi_f|^2 (Q15) ≈ (ferr>>12)*3   限幅 ±32768
     *          （按 0.154 Wb 配的，0.141 下增益弱约 17%，tau ≈ 19 ms，能用，没改）
     *   corr = (psi_r*fq) >> FLUX_NLFO_SH      与 e*KTs 同一刻度
     * 用的是上一拍的 psi_s（前向欧拉：导数在旧状态上求）。 */
    x = (sFluxObs.s32PsiAlF + 32768) >> 16; y = (sFluxObs.s32PsiBeF + 32768) >> 16;
    {
        int32_t g0 = (int32_t)x - Mx_MulQ15(sFluxObs.s16KLs, current.s16Alpha);
        int32_t g1 = (int32_t)y - Mx_MulQ15(sFluxObs.s16KLs, current.s16Beta);
        int32_t ferr, fq;
        /* 先限幅再平方，否则 g0*g0 可达 3.1e9 溢出 */
        if (g0 > PSI_LIMIT) g0 = PSI_LIMIT;
        if (g0 < -PSI_LIMIT) g0 = -PSI_LIMIT;
        if (g1 > PSI_LIMIT) g1 = PSI_LIMIT;
        if (g1 < -PSI_LIMIT) g1 = -PSI_LIMIT;
        ferr = FLUX_NOM2 - ((int32_t)g0 * g0 + (int32_t)g1 * g1);
        fq = (ferr >> 12) * 3;
        if (fq > FLUX_NLFO_FQ_LIMIT) fq = FLUX_NLFO_FQ_LIMIT;
        if (fq < -FLUX_NLFO_FQ_LIMIT) fq = -FLUX_NLFO_FQ_LIMIT;
        corrA = ((int32_t)g0 * fq) >> FLUX_NLFO_SH;
        corrB = ((int32_t)g1 * fq) >> FLUX_NLFO_SH;
    }
    sFluxObs.s32PsiAlF = addFlux(sFluxObs.s32PsiAlF, 2 * ((int32_t)ea * sFluxObs.s16KTs + corrA));
    sFluxObs.s32PsiBeF = addFlux(sFluxObs.s32PsiBeF, 2 * ((int32_t)eb * sFluxObs.s16KTs + corrB));
    sFluxObs.s16PsiAl = (sFluxObs.s32PsiAlF + 32768) >> 16; sFluxObs.s16PsiBe = (sFluxObs.s32PsiBeF + 32768) >> 16;

    /* 输出：用积分后的 psi_s 算本拍的 psi_r */
    x = (int32_t)sFluxObs.s16PsiAl - Mx_MulQ15(sFluxObs.s16KLs, current.s16Alpha);
    y = (int32_t)sFluxObs.s16PsiBe - Mx_MulQ15(sFluxObs.s16KLs, current.s16Beta);
    if (x > PSI_LIMIT || x < -PSI_LIMIT || y > PSI_LIMIT || y < -PSI_LIMIT) gFluxDiag.fluxSatCount++;
    sFluxObs.s16PsiRAl = (int16_t)Mx_Clamp(x, PSI_LIMIT); sFluxObs.s16PsiRBe = (int16_t)Mx_Clamp(y, PSI_LIMIT);

    /* |psi_r| 只给 J-Scope / 置信度用，每 16 拍一次 sqrt */
    if (sFluxObs.u8DivCnt == 8) {
        sFluxObs.s16PsiRMag = Sqrt_Q15((int32_t)sFluxObs.s16PsiRAl * sFluxObs.s16PsiRAl +
                                       (int32_t)sFluxObs.s16PsiRBe * sFluxObs.s16PsiRBe);
    }
}

void FluxObs_UpdateFeedback(FOC_TypeDef *f)
{
    FluxObs_Run(f);
    /* 闭源库只读 FluxFeedback.s16Speed（Var_Filt，FOC+0x90）；两个 EMF 字段没有
     * 读者，保持 0（test27 反汇编核实）。 */
    f->FluxFeedback.s16Ealpha_est = 0;
    f->FluxFeedback.s16Ebeta_est = 0;
}
