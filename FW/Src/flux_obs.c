/* ============================================================================
 * @file    flux_obs.c
 * @brief   转子磁链观测器，接口说明见 flux_obs.h
 *
 * 原理（电压模型磁链观测器）：
 *   定子电压方程 v = Rs*i + d(psi_s)/dt  ->  psi_s = ∫(v - Rs*i) dt（定子磁链）
 *   转子磁链 psi_r = psi_s - L*i，其方向即转子 d 轴（N 极），角度由 PLL（flux_pll.c）锁定。
 * 纯积分存在三个问题，分别对应一项修正：
 *   1. 电压 / 电流中的直流偏置导致积分漂移 -> 积分前对反电势 e = v - Rs*i 去直流（一阶高通，FLUX_HPF_TAU_S）；
 *      高通在低速时引入相位超前 -> 输出时反向旋转对应角度补偿（由 FluxObs_SlowTick 计算 s_compCos / s_compSin）。
 *   2. Rs、电压误差使磁链幅值逐渐偏离 -> NLFO：每拍沿 psi_r 方向将 |psi_r| 拉向 psi_f（只修正幅值，不影响角度）。
 *   3. 电压取自调制器指令，逆变器死区 / 管压降带来误差 -> dtComp 按实测模型补偿（cfg_board.h INVERTER_DT_*）。
 *   此外，大电流时电感饱和，L*i 使用随电流下降的 L(i)（lsEff）。
 * 定点：电压 Q15（FLUX_V_BASE 基准），磁链 Q15（FLUX_PSI_N 基准，cfg_derived.h），积分状态再左移 16 位以保证精度。
 *
 * 每拍调用链：Clarke -> FluxObs_Run（在本拍调制之前，gSvpwm.Alpha/Beta 仍为上一拍指令）
 * ==========================================================================*/
#include "flux_obs.h"
#include "foc_core.h"
#include "meas.h"
#include "est.h"
#include "svpwm.h"
#include "qmath.h"
#include "compiler.h"
#include "param.h"

FluxObs_TypeDef gFluxObs;

/* FMAX：积分状态（磁链 << 16）限幅 = 0.9 满量程；PSI_LIMIT：转子磁链分量限幅 = 32768 / sqrt(2)（两分量平方和不溢出） */
#define FMAX ((int32_t)29491L * 65536L)
#define PSI_LIMIT 23170
/* psi_f（磁链 LSB）及 |psi_f|^2：运行时参数（param.c gGain） */
#define FLUX_NOM    gGain.fluxNomQ15
#define FLUX_NOM2   gGain.fluxNom2

/* 反电势输入限幅到 ±0.9 满量程 */
FORCE_INLINE int16_t inputClamp(int32_t x)
{
    return (int16_t)QMath_Clamp(x, 29491);
}
/* 磁链积分一步（饱和） */
FORCE_INLINE int32_t addFlux(int32_t x, int32_t dx)
{
    int32_t sum = x + dx;
    return QMath_Clamp(sum, FMAX);
}

/* 有效 KLs = KLs * L(|i|)/L0（MOTOR_LS_SAT_PER_A = 0 时恒等于 KLs）。|i| 用 max + 3/8 min 近似（误差 < 7%，对拐点位置精度足够），
 * 无除法、仅 32 位乘法。m 最大约 4.6e4，乘 K（< 7*4096）不溢出。 */
FORCE_INLINE int16_t lsEff(int16_t ia, int16_t ib)
{
    int32_t a = QMath_Abs16(ia), b = QMath_Abs16(ib), m, r;
    if (gGain.lsSatKQ12 <= 0) return gFluxObs.s16KLq;       /* PER_A = 0：不考虑饱和 */
    m = (a > b) ? a + ((3 * b) >> 3) : b + ((3 * a) >> 3);
    m -= gGain.lsKneeQ15;
    if (m <= 0) return gFluxObs.s16KLq;
    r = 32768 - ((m * gGain.lsSatKQ12) >> 12);
    if (r < FLUX_LS_MIN_Q15) r = FLUX_LS_MIN_Q15;
    return (int16_t)(((int32_t)gFluxObs.s16KLq * r) >> 15);
}

/* 逆变器电压误差（死区 + 管压降），见 cfg_board.h INVERTER_DT_*。指令电压 = 实际电压 + 误差，
 * 故 v_实际 = v_指令 - e。ia/ib 使用与 Rs 压降相同的区间平均电流（误差为整个周期内的平均量）。
 * 每相：x = clamp(i_x / I0, ±1)，e_x = Vdt * x，Vdt = bus * KT；再按 Clarke 合成 alpha/beta。
 * Vdt 只随母线平均值变化，由慢环计算（FluxObs_SlowTick）；0 = 不补偿（INVERTER_DT_COMP = 0 或死区为 0）。 */
static volatile int16_t s_vdt;
FORCE_INLINE int32_t dtPhase(int32_t i)
{
    return QMath_Clamp((i * FLUX_DT_KI_Q8) >> 8, 32767);
}
FORCE_INLINE void dtComp(int16_t ia, int16_t ib, int16_t *va, int16_t *vb)
{
    int32_t vdt = s_vdt, xa, xb, xc, ibs;               /* 每相误差幅值（Q15 电压） */
    if (vdt <= 0) return;
    ibs = ((int32_t)ib * 28378) >> 15;                   /* (sqrt3/2) * i_beta */
    xa = dtPhase(ia);
    xb = dtPhase((-(int32_t)ia >> 1) + ibs);     /* 有符号数除以 2 写成移位：/ 2 需额外两条指令修正负数取整方向 */
    xc = dtPhase((-(int32_t)ia >> 1) - ibs);
    /* e_alpha = (2xa - xb - xc)/3 * vdt，e_beta = (xb - xc)/sqrt3 * vdt */
    *va = (int16_t)(*va - ((((2 * xa - xb - xc) * 10923) >> 15) * vdt >> 15));
    *vb = (int16_t)(*vb - ((((xb - xc) * 18919) >> 15) * vdt >> 15));
}

/* 清除全部观测器状态（磁链、去直流滤波器、输出），非 Run 状态时每拍调用 */
void FluxObs_Reset(void)
{
    gFluxObs.s32PsiAlF = 0; gFluxObs.s32PsiBeF = 0;
    gFluxObs.sEalAvgMA.s32Acc = 0; gFluxObs.sEbeAvgMA.s32Acc = 0;
    gFluxObs.s16PsiAl = 0; gFluxObs.s16PsiBe = 0;
    gFluxObs.s16PsiRAl = 0; gFluxObs.s16PsiRBe = 0;
    gFluxObs.haveCurrent = 0;
}

#if MOTOR_IPM
static void setTarget(int32_t tgt);
#endif

/* 从 gGain 重新装载模型参数（上电或 param.c 更新参数时；在 Stop 状态下调用） */
void FluxObs_LoadModel(void)
{
    gFluxObs.s16KRs = gGain.fluxKrsQ15;
    gFluxObs.s16KLq = gGain.fluxKlQ15;
    gFluxObs.s16NlfoNormQ8 = gGain.nlfoNormQ8;
#if MOTOR_IPM
    setTarget(FLUX_NOM);
#endif
}

/* 上电：装载模型参数、积分步长 KTs、去直流窗口，清除状态 */
void FluxObs_Init(void)
{
    FluxObs_LoadModel();
    gFluxObs.s16KTs = FLUX_KTS_Q15;
    gFluxObs.sEalAvgMA.u16Sh = FLUX_HPF_SH; gFluxObs.sEbeAvgMA.u16Sh = FLUX_HPF_SH;
    FluxObs_Reset();
}

/* 去直流相位补偿：输出前将 psi_s 旋转 -alpha，alpha = sign(w)*atan(wc/|w|)。
 * cos/sin 在慢环中计算：(cos, sin) = (w, wc) 单位化（QMath_NormQ15，无开方和除法）。
 * w0~2w0 之间补偿角从 0 渐变到全值：wc 按 (w - w0)/w0 缩小（乘以编译期常数 wc/w0）。 */
static volatile int16_t s_compCos = 32767, s_compSin = 0;
#define HPF_WC_OVER_W0_Q16      ((int32_t)((double)HPF_WC_Q15SPD * 65536.0 / HPF_COMP_W0_Q15SPD + 0.5))

/* 慢环每拍（Ctrl_SlowTick）：
 *   1. 死区电压误差幅值 Vdt = busAvg * KT（母线平均值同样在慢环更新）；
 *   2. 按当前转速计算去直流相位补偿角。补偿对象为高通 wc/(s + wc) 引入的超前角 atan(wc/w)，
 *      低于 w0（HPF_COMP_W0_Q15SPD）不补偿，w0~2w0 之间渐入（避免在门限处角度跳变） */
void FluxObs_SlowTick(int16_t spdQ15, int16_t busAvg)
{
    int32_t w = spdQ15 < 0 ? -(int32_t)spdQ15 : spdQ15;
    int32_t wc = HPF_WC_Q15SPD, w0 = HPF_COMP_W0_Q15SPD;
    int16_t c, s;
    s_vdt = (gGain.dtKtQ15 > 0 && busAvg > 0) ? (int16_t)(((int32_t)busAvg * gGain.dtKtQ15) >> 15) : 0;
    if (w <= w0) { s_compCos = 32767; s_compSin = 0; return; }
    if (w < 2 * w0) wc = ((w - w0) * HPF_WC_OVER_W0_Q16) >> 16;
    QMath_NormQ15(w, wc, &c, &s);
    s_compCos = c;
    s_compSin = (int16_t)(spdQ15 < 0 ? -s : s);
}

#if MOTOR_IPM
/* 内嵌式：有效磁链幅值目标 = psi_f + (Ld-Lq)*id。快环读取这些量（各为一次 32 位读取，
 * 更新时最多有一拍新旧值混用，无害）。 */
static volatile int32_t s_target2, s_nlfoNorm;     /* |target|^2；256 / |target|^2 的定点（fq 计算见 FluxObs_Run） */

/* 内嵌式：设置 NLFO 幅值目标及对应归一化系数（慢环中执行，允许除法） */
static void setTarget(int32_t tgt)
{
    int32_t t2;
    if (tgt < FLUX_NOM / 2) tgt = FLUX_NOM / 2;   /* 限制 norm <= 4 倍标称值，防止溢出 */
    if (tgt > PSI_LIMIT) tgt = PSI_LIMIT;
    t2 = tgt * tgt;
    s_target2 = t2;
    s_nlfoNorm = (int32_t)(((int64_t)gGain.nlfoNormQ8 * FLUX_NOM2) / t2);       /* 慢环，允许除法 */
}

/* 内嵌式：id 指令变化时更新有效磁链目标 = psi_f + (Ld - Lq)*id */
void FluxObs_SetIdRef(int16_t idQ15)
{
    setTarget(FLUX_NOM + (((int32_t)gGain.fluxKdlQ15 * idQ15) >> 15));
}
#endif

/* 冷起动（IPD / 定位 -> I/F）时调用：磁链初始化为 |psi_f|∠theta（此时转子位于该角度），观测器从正确状态开始 */
void FluxObs_BeginOpenLoop(void)
{
    /* 仅在冷起动 Align/IPD -> OpenLoop 时调用（由 Est_OnEnterOpenLoopCold 调用，PLL 与转速清零也在该处） */
    FluxObs_Reset();
    /* psi 初值设为 |psi_f|∠theta_hat 而非 0：若从 0 起步，NLFO 需约 30 ms 建立幅值，期间方向受 Rs / 死区误差影响，
     * PLL 误差接近满量程，可能导致估计角偏转并产生虚假转速。
     * 调用方已设置 gEst.pll.s32Theta = gFoc.theta<<16，因此第一拍 PLL 误差恰好为 0。 */
    {
        SinCos_TypeDef a = FocMath_SinCos(gFoc.theta);
        gFluxObs.s32PsiAlF = (int32_t)QMath_MulQ15(FLUX_NOM, a.s16Cos) << 16;
        gFluxObs.s32PsiBeF = (int32_t)QMath_MulQ15(FLUX_NOM, a.s16Sin) << 16;
    }
}

/* 飞车接管时调用：按测得的转子角初始化磁链。存储的是定子磁链，故 = psi_f∠theta + L*i（短接电流仍存在） */
void FluxObs_SeedCatch(int16_t theta)
{
    /* psi_s = psi_r + Lq*i：短接电流较大（1500 rpm 约 2.8 A），不能只初始化 psi_r */
    SinCos_TypeDef a = FocMath_SinCos(theta);
    int16_t kls = lsEff(gMeas.iab.s16Alpha, gMeas.iab.s16Beta);
    FluxObs_Reset();
    gFluxObs.s32PsiAlF = ((int32_t)QMath_MulQ15(FLUX_NOM, a.s16Cos) + QMath_MulQ15(kls, gMeas.iab.s16Alpha)) << 16;
    gFluxObs.s32PsiBeF = ((int32_t)QMath_MulQ15(FLUX_NOM, a.s16Sin) + QMath_MulQ15(kls, gMeas.iab.s16Beta)) << 16;
}

/* 观测器一拍（快环）：
 *   1. 母线过低 -> 复位，不计算；
 *   2. 电压 v = 上一拍调制器 αβ 指令 * 本拍母线，减去死区误差；
 *   3. 反电势 e = v - Rs*i（i 取两拍平均：电阻压降为整个 PWM 周期的平均量），再去直流；
 *   4. NLFO 修正量（沿 psi_r 方向，按 |psi_f|^2 - |psi_r|^2 的比例）；
 *   5. 积分：psi_s += e*KTs + 修正；
 *   6. 输出 psi_r = 相位补偿(psi_s) - L(i)*i，供 PLL 使用。 */
RAMFUNC void FluxObs_Run(void)
{
    AlphaBeta_TypeDef current;
    int16_t va, vb, ea, eb, ia, ib;
    int32_t x, y, corrA, corrB;
    int16_t kls;
    int32_t lia, lib;               /* L(i)*i：NLFO 与输出各使用一次，同一拍取值相同，只计算一次 */
    int16_t bus = gMeas.vbus;
    if (bus < Q15(FLUX_BUS_MIN_V / UDC_MAX)) {
        FluxObs_Reset();
        return;
    }
    current = gMeas.iab;
    /* L*i 为状态量（磁链），与下方 g0/g1、输出一样使用本拍瞬时电流 */
    kls = lsEff(current.s16Alpha, current.s16Beta);
    lia = QMath_MulQ15(kls, current.s16Alpha);
    lib = QMath_MulQ15(kls, current.s16Beta);
    /* 上一拍电压 = 调制器指令 × 母线。gSvpwm.Alpha/Beta 的 32768 = Vbus/√3（相电压幅值），本拍尚未被覆盖；
     * 电流环将 |V| 限制在 vmax（SVPWM 线性区），故指令即为该拍实际输出的平均电压（死区另行补偿），
     * 七段式 / 五段式只差共模分量，αβ 相同。占空比在上一拍确定，乘以本拍采样的母线电压（母线波动时即为实际电压）。
     * 观测器只在 OpenLoop / Spin 中运行，这两个状态下 PWM 必定在输出；PWM 关闭时状态机在同拍或下一拍离开 Run，
     * 观测器随之复位（Ctrl_FastTick）。18919 = 1/√3（Q15）。 */
    va = (int16_t)(((((int32_t)gSvpwm.Alpha * bus) >> 15) * 18919) >> 15);
    vb = (int16_t)(((((int32_t)gSvpwm.Beta * bus) >> 15) * 18919) >> 15);

    /* 电阻压降使用区间平均电流（时间积分量）；Ls*i 使用本拍瞬时值（状态量） */
    ia = current.s16Alpha; ib = current.s16Beta;
    if (gFluxObs.haveCurrent) {
        ia = ((int32_t)ia + gFluxObs.previousIa) >> 1;  /* 向下取整，与 / 2 相差不足 1 LSB */
        ib = ((int32_t)ib + gFluxObs.previousIb) >> 1;
    }
    dtComp(ia, ib, &va, &vb);
    gFluxObs.previousIa = current.s16Alpha; gFluxObs.previousIb = current.s16Beta; gFluxObs.haveCurrent = 1;

    ea = inputClamp((int32_t)va - QMath_MulQ15(gFluxObs.s16KRs, ia));
    eb = inputClamp((int32_t)vb - QMath_MulQ15(gFluxObs.s16KRs, ib));
    ea = inputClamp((int32_t)ea - QMath_LowPass(ea, &gFluxObs.sEalAvgMA.s32Acc, gFluxObs.sEalAvgMA.u16Sh));
    eb = inputClamp((int32_t)eb - QMath_LowPass(eb, &gFluxObs.sEbeAvgMA.s32Acc, gFluxObs.sEbeAvgMA.u16Sh));

    /* ---- NLFO 径向修正 ------------------------------------------------------
     *     corr = Gain * psi_r * (|psi_f|^2 - |psi_r|^2)
     * 修正方向始终平行于 psi_r：电压误差只影响幅值，不会将角度拉偏
     * （若修正含切向分量，会引入角度误差）。
     * 定标（磁链 LSB，|psi_f| = FLUX_NOM）：
     *   ferr = |psi_f|^2 - |psi_r|^2          |.| <= 1.07e9
     *   fq   = ferr/|psi_f|^2 (Q15) = ((ferr>>12)*FLUX_NLFO_NORM_Q8)>>8   限幅 ±32768
     *   corr = (psi_r*fq) >> FLUX_NLFO_SH      与 e*KTs 同一刻度
     * 使用上一拍的 psi_s（前向欧拉：在旧状态上求导数）。 */
    x = (gFluxObs.s32PsiAlF + 32768) >> 16; y = (gFluxObs.s32PsiBeF + 32768) >> 16;
    {
        int32_t g0 = (int32_t)x - lia;
        int32_t g1 = (int32_t)y - lib;
        int32_t ferr, fq;
        /* 先限幅再平方，否则 g0*g0 可达 3.1e9 而溢出 */
        if (g0 > PSI_LIMIT) g0 = PSI_LIMIT;
        if (g0 < -PSI_LIMIT) g0 = -PSI_LIMIT;
        if (g1 > PSI_LIMIT) g1 = PSI_LIMIT;
        if (g1 < -PSI_LIMIT) g1 = -PSI_LIMIT;
#if MOTOR_IPM
        /* 目标与归一化系数随 id 变化：fq = ferr/target^2（Q15） */
        ferr = s_target2 - ((int32_t)g0 * g0 + (int32_t)g1 * g1);
        fq = ((ferr >> 12) * s_nlfoNorm) >> 8;
#else
        ferr = FLUX_NOM2 - ((int32_t)g0 * g0 + (int32_t)g1 * g1);
        fq = ((ferr >> 12) * gFluxObs.s16NlfoNormQ8) >> 8;
#endif
        if (fq > FLUX_NLFO_FQ_LIMIT) fq = FLUX_NLFO_FQ_LIMIT;
        if (fq < -FLUX_NLFO_FQ_LIMIT) fq = -FLUX_NLFO_FQ_LIMIT;
        corrA = ((int32_t)g0 * fq) >> FLUX_NLFO_SH;
        corrB = ((int32_t)g1 * fq) >> FLUX_NLFO_SH;
    }
    gFluxObs.s32PsiAlF = addFlux(gFluxObs.s32PsiAlF, 2 * ((int32_t)ea * gFluxObs.s16KTs + corrA));
    gFluxObs.s32PsiBeF = addFlux(gFluxObs.s32PsiBeF, 2 * ((int32_t)eb * gFluxObs.s16KTs + corrB));
    gFluxObs.s16PsiAl = (gFluxObs.s32PsiAlF + 32768) >> 16; gFluxObs.s16PsiBe = (gFluxObs.s32PsiBeF + 32768) >> 16;

    /* 输出：由积分后的 psi_s 计算本拍 psi_r */
    {   /* 先补偿去直流引入的相位超前（只修改输出，不修改积分状态） */
        int32_t c = s_compCos, s = s_compSin;
        int32_t ps = gFluxObs.s16PsiAl, pb = gFluxObs.s16PsiBe;
        x = ((ps * c + pb * s) >> 15) - lia;
        y = ((pb * c - ps * s) >> 15) - lib;
    }
    gFluxObs.s16PsiRAl = (int16_t)QMath_Clamp(x, PSI_LIMIT); gFluxObs.s16PsiRBe = (int16_t)QMath_Clamp(y, PSI_LIMIT);
}
