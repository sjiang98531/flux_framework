/* Host re-implementation of the closed-source SOC_MCI_2R_V102.lib, translated
 * instruction by instruction from the test33 AXF disassembly (llvm-objdump,
 * thumbv6m). Used ONLY by the host simulation as the REFERENCE model of the
 * vendor library:
 *   - sim_old (test33) links it, exactly like the old firmware linked the real lib;
 *   - since 0xF20E0006 the framework does not use the vendor library at all, so
 *     sim_new / sim_legacy do NOT link this file. run_equiv.sh (test33 + this model
 *     vs. our own code) and libcheck.c (function by function) prove that our
 *     replacements behave bit-for-bit like the vendor library. */
#include <stdint.h>
#include <math.h>
#include "mlib.h"
#include "mcflib.h"
#include "pwm_gen.h"
#include "motor_structure.h"
#include "vendor_tables.h"

extern USER_TypeDef USER;
extern FOC_TypeDef FOC;
extern FastBoot_TypeDef FastBoot_Para;
extern volatile unsigned int eM1_RunSubState;   /* enum, 4 bytes, Startup == 3 */

#ifndef NEWCODE        /* 新代码自己定义 pwm_gen（svpwm.c），只有 test33 构建用这里的 */
PWM_GEN_T pwm_gen;
#endif

static int16_t mulq15(int16_t a, int16_t b) { return (int16_t)(((int32_t)a * b) >> 15); }

/* 0x0800093C */
void CalcPI(tPIParm *p)
{
    int32_t err = (int32_t)p->qInRef - (int32_t)p->qInMeas;
    int32_t prop = (int32_t)((int32_t)p->qKp * err) >> 15;
    int32_t sum = (int32_t)p->qdSum + (int32_t)p->qKi * err;
    int32_t hi = (int32_t)p->qOutMax << 15, lo = (int32_t)p->qOutMin << 15;
    int32_t out;
    p->qdSum = sum;
    if (sum >= hi) p->qdSum = hi;
    else if (sum <= lo) p->qdSum = lo;
    out = prop + ((int32_t)p->qdSum >> 15);
    p->qOut = out;
    if (out > p->qOutMax) p->qOut = p->qOutMax;
    else if (out < p->qOutMin) p->qOut = p->qOutMin;
}

/* 0x08000A54 — ignores the s16Vbus argument; uses FOC.s16KVbus set by Var_Filt */
void DcBus_Ripple_suppress(MCFLIB_2_ALBE_T_S16 *in, Q15_t s16Vbus, MCFLIB_2_ALBE_T_S16 *out)
{
    (void)s16Vbus;
    out->s16Alpha = (int16_t)(mulq15(in->s16Alpha, FOC.s16KVbus) << 5);
    out->s16Beta  = (int16_t)(mulq15(in->s16Beta,  FOC.s16KVbus) << 5);
}

/* 0x08000D10 */
void Fast_Boot_Current_Controller(FOC_TypeDef *f)
{
    int32_t t;
    if (eM1_RunSubState == 3) {
        t = (int32_t)f->sIdq_cmd.s16Q * -2000;
        f->CurID.qdSum = t;
        if (t < -32767000) f->CurID.qdSum = -32767000;
        else if (t > 32767000) f->CurID.qdSum = 32767000;
    }
    f->CurID.qOutMax = f->s16VMAX;
    f->CurID.qOutMin = -f->CurID.qOutMax;
    f->CurID.qInRef = f->sIdq_cmd.s16D;
    f->CurID.qInMeas = f->sIdq.s16D;
    CalcPI(&f->CurID);
    f->sVdq.s16D = (int16_t)f->CurID.qOut;
    t = (int32_t)f->s16VMAX * f->s16VMAX - (int32_t)f->sVdq.s16D * f->sVdq.s16D;
    f->CurIQ.qOutMax = Sqrt_Q15(t);
    f->CurIQ.qOutMin = -f->CurIQ.qOutMax;
    f->CurIQ.qInRef = f->sIdq_cmd.s16Q;
    f->CurIQ.qInMeas = f->sIdq.s16Q;
    CalcPI(&f->CurIQ);
    f->sVdq.s16Q = (int16_t)f->CurIQ.qOut;
}

/* 0x08000DA4 */
uint8_t Fast_Boot_Spin(FOC_TypeDef *f)
{
    int32_t half = FastBoot_Para.DetcSpd >> 1;
    if (f->s16SpdFilt >= half && f->s16SpdCmd > 0) return 0;
    if (f->s16SpdFilt <= -half && f->s16SpdCmd < 0) return 0;
    USER.u16StartupTimeCnt = 0;
    return 1;
}

/* 0x08000DE8 */
uint8_t Fast_Boot_StartUp(FOC_TypeDef *f)
{
    if (USER.u16StartupTimeCnt <= FastBoot_Para.DetcTime) return 0;
    USER.u16StartupTimeCnt = 0;
    if (f->s16SpdFilt >= FastBoot_Para.DetcSpd && f->s16SpdCmd > 0) return 1;
    if (f->s16SpdFilt <= -FastBoot_Para.DetcSpd && f->s16SpdCmd < 0) return 1;
    return 0;
}

/* 0x080036BC */
void Var_Filt(void)
{
    FOC.s16VbusAvg = MLIB_FilterMA_Q15(FOC.s16Vbus, (MLIB_FILTER_MA_T *)&USER.sVbus_Filt);
    FOC.s16SpdFilt = MLIB_FilterMA_Q15(FOC.FluxFeedback.s16Speed, (MLIB_FILTER_MA_T *)&USER.sSpeed_Filt);
    if (FOC.s16Ibus <= 0) FOC.s16IbusAvg = 0;
    else FOC.s16IbusAvg = MLIB_FilterMA_Q15(FOC.s16Ibus, (MLIB_FILTER_MA_T *)&USER.sIbus_Filt);
    FOC.s16KVbus = (int16_t)(58116276 / (int32_t)FOC.s16VbusAvg);
}

/* 0x080028B8 */
sAngle_Trig MLIB_SinCos(Q15_t a)
{
    sAngle_Trig r;
    uint16_t u = (uint16_t)((int32_t)a + 0x8000);
    int q = (u >> 14) & 3;
    int idx = (u >> 6) & 0xff;
    int16_t ti = kSinTab[idx], tc = kSinTab[255 - idx];
    switch (q) {
    case 0: r.s16Cos = (int16_t)-tc; r.s16Sin = (int16_t)-ti; break;
    case 1: r.s16Cos = ti;           r.s16Sin = (int16_t)-tc; break;
    case 2: r.s16Cos = tc;           r.s16Sin = ti;           break;
    default:r.s16Cos = (int16_t)-ti; r.s16Sin = tc;           break;
    }
    return r;
}

/* 0x08003548 */
Q15_t Sqrt_Q15(int32_t v) { return kSqrtTab[v >> 21]; }

/* 0x0800288C */
Q31_t MLIB_Ramp_Q31(Q31_t target, MLIB_RAMP_T_Q31 *p)
{
    int32_t st = (int32_t)p->s32State, tg = (int32_t)target, r = tg;
    if (st < tg) { r = st + (int32_t)p->s32RampUp; if (r > tg) r = tg; }
    else if (st > tg) { r = st - (int32_t)p->s32RampDown; if (r < tg) r = tg; }
    p->s32State = r;
    return r;
}

static int32_t shBi(int32_t v, int16_t sh) { return sh >= 0 ? (int32_t)((uint32_t)v << sh) : (v >> -sh); }

/* 0x0800282C */
Q15_t MLIB_PLL(Q15_t err, PLL_TypeDef *p)
{
    int32_t kp = shBi((int32_t)p->s16PGain * err * 2, p->i16PGainSh);
    int32_t ki = shBi((int32_t)p->s16IGain * err * 2, p->i16IGainSh);
    int32_t sp, th;
    p->s32I_1 += ki;
    sp = kp + p->s32I_1;
    p->s32Speed = sp;
    th = (int32_t)((((sp & 0xffff) * (int32_t)p->s16ThGain) >> 15) + (((sp >> 16) * (int32_t)p->s16ThGain) << 1));
    th = shBi(th, p->i16ThGainSh);
    p->s32Theta += th;
    return (Q15_t)(p->s32Theta >> 16);
}
Q15_t MLIB_PLL1(Q15_t e, PLL_TypeDef *p, Q15_t c, Q31_t k) { (void)c; (void)k; return MLIB_PLL(e, p); }

/* 0x08002C2C — TI-style sector SVPWM, also reconstructs Ibus */
void PWM_GEN_calc(PWM_GEN_T *u)
{
    int32_t r3 = (u->Alpha * 28378) >> 15;
    int32_t X = u->Beta, half = X >> 1;
    int32_t vb = r3 - half, vc = -(r3 + half);
    int32_t Y = half + r3, Z = half - r3;
    uint8_t n = 0;
    int32_t t1 = 0, t2 = 0, P, a1, a2, ta, tb, tc, t0;
    if (X >= 0) n = 1;
    if (vb >= 0) n += 2;
    if (vc >= 0) n += 4;
    switch (n) {
    case 1: t1 = Z;  t2 = Y;  break;
    case 2: t1 = Y;  t2 = -X; break;
    case 3: t1 = -Z; t2 = X;  break;
    case 4: t1 = -X; t2 = Z;  break;
    case 5: t1 = X;  t2 = -Y; break;
    case 6: t1 = -Y; t2 = -Z; break;
    default: break;
    }
    P = u->N_halfPeriod;
    a1 = (P * t1) >> 15;
    a2 = (P * t2) >> 15;
    t0 = P - a1 - a2;
    ta = t0 >> 1;
    tb = ta + a1;
    tc = tb + a2;
    {
        int16_t IA = FOC.sIabc.s16A, IB = FOC.sIabc.s16B, IC = FOC.sIabc.s16C;
        int16_t *ib = (int16_t *)&FOC.s16Ibusreconstructed;
        if (u->Mode == 1) {
            switch (n) {
            case 1: u->CompA = tb; u->CompB = ta; u->CompC = tc; *ib = (int16_t)((IB * t1 - IC * t2) >> 15); break;
            case 2: u->CompA = ta; u->CompB = tc; u->CompC = tb; *ib = (int16_t)((IA * t1 - IB * t2) >> 15); break;
            case 3: u->CompA = ta; u->CompB = tb; u->CompC = tc; *ib = (int16_t)((IA * t1 - IC * t2) >> 15); break;
            case 4: u->CompA = tc; u->CompB = tb; u->CompC = ta; *ib = (int16_t)((IC * t1 - IA * t2) >> 15); break;
            case 5: u->CompA = tc; u->CompB = ta; u->CompC = tb; *ib = (int16_t)((IB * t1 - IA * t2) >> 15); break;
            case 6: u->CompA = tb; u->CompB = tc; u->CompC = ta; *ib = (int16_t)((IC * t1 - IB * t2) >> 15); break;
            default: u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1); *ib = 0; break;
            }
        } else if (u->Mode == 0) {
            switch (n) {
            case 1: u->CompA = P - a2; u->CompB = t0; u->CompC = P;      *ib = (int16_t)((IB * t1 - IC * t2) >> 15); break;
            case 2: u->CompA = t0;     u->CompB = P;  u->CompC = P - a2; *ib = (int16_t)((IA * t1 - IB * t2) >> 15); break;
            case 3: u->CompA = t0;     u->CompB = P - a2; u->CompC = P;  *ib = (int16_t)((IA * t1 - IC * t2) >> 15); break;
            case 4: u->CompA = P;      u->CompB = P - a2; u->CompC = t0; *ib = (int16_t)((IC * t1 - IA * t2) >> 15); break;
            case 5: u->CompA = P;      u->CompB = t0; u->CompC = P - a2; *ib = (int16_t)((IB * t1 - IA * t2) >> 15); break;
            case 6: u->CompA = P - a2; u->CompB = P;  u->CompC = t0;     *ib = (int16_t)((IC * t1 - IB * t2) >> 15); break;
            default: u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1); *ib = 0; break;
            }
        } else {
            u->CompA = u->CompB = u->CompC = (int16_t)(P >> 1);
        }
    }
    u->Sector = n;
}

/* 0x0800265C. The real code programs the bridge (lower switches on, compares =
 * CaliPWMDuty) and uses the MR coprocessor for sqrt; the sim does the same
 * through the register mock and libm. */
void MCF_FlyStart_2R(FOC_TypeDef *f)
{
    int16_t e;
    sAngle_Trig a;
    EPWM->EPWM_CHN |= 0x2a;
    EPWM->EPWM_CHN &= 0xffea;
    EPWM->EPWM_DT[0] = USER.u16CaliPWMDuty;
    EPWM->EPWM_DT[2] = USER.u16CaliPWMDuty;
    EPWM->EPWM_DT[4] = USER.u16CaliPWMDuty;
    MCFLIB_Clark_Sat_S16(&f->sIabc, &f->sIAlBe);
    f->sFly_Start.s32BemfAmp2 = (int32_t)f->sIAlBe.s16Alpha * f->sIAlBe.s16Alpha + (int32_t)f->sIAlBe.s16Beta * f->sIAlBe.s16Beta;
    f->sFly_Start.s16BemfAmp = (int16_t)sqrt((double)f->sFly_Start.s32BemfAmp2);
    a = MLIB_SinCos(f->sFly_Start.s16PosElEst_fly);
    f->sFly_Start.sAngle_fly.s16Cos = a.s16Cos;
    f->sFly_Start.sAngle_fly.s16Sin = a.s16Sin;
    e = (int16_t)(mulq15((int16_t)-f->sIAlBe.s16Alpha, f->sFly_Start.sAngle_fly.s16Cos) - mulq15(f->sIAlBe.s16Beta, f->sFly_Start.sAngle_fly.s16Sin));
    f->sFly_Start.s16ThetaErr_fly = e;
    f->sFly_Start.s16PosElEst_fly = MLIB_PLL(f->sFly_Start.s16ThetaErr_fly, &f->sPLL);
}
void MCF_FlyStart_0BEMF(FOC_TypeDef *f) { (void)f; }

/* 0x0800256C */
void MCF_ADV_Protect(FOC_TypeDef *f)
{
    ADV_PROTECT *p = &f->sProtect;
    uint32_t mn, mx;
    p->u16VoltageTemp = (uint16_t)(mulq15(f->sVdq.s16D, f->sVdq.s16D) + mulq15(f->sVdq.s16Q, f->sVdq.s16Q));
    if (MLIB_Abs_Q15(f->s16SpdFilt) < (int32_t)p->u16BlockSpeed && p->u16VoltageTemp < p->u16BlockVoltageAmp_square) {
        if (++p->u16BlockCounter > p->u16BlockTimeCmd) { p->u16BlockCounter = 0; p->bRotor_block = 1; }
    } else p->u16BlockCounter = 0;
    if (mulq15(f->s16SpeedRamp, f->s16SpdFilt) < 0 && p->u16VoltageTemp < p->u16BlockVoltageAmp_square) {
        if (++p->u16FspinCounter > p->u16BlockTimeCmd) { p->bErr_Spin = 1; p->u16FspinCounter = 0; }
    } else p->u16FspinCounter = 0;
    if (++p->u16PhaselessTimeCnt > p->u16PhaselessTimeCmd) {
        mn = p->u32Ia_Integral < p->u32Ib_Integral ? p->u32Ia_Integral : p->u32Ib_Integral;
        if (mn >= p->u32Ic_Integral) mn = p->u32Ic_Integral;
        p->u32PhaselessMinTemp = mn;
        mx = p->u32Ia_Integral > p->u32Ib_Integral ? p->u32Ia_Integral : p->u32Ib_Integral;
        if (mx <= p->u32Ic_Integral) mx = p->u32Ic_Integral;
        p->u32PhaselessMaxTemp = mx;
        if (mx > mn * 3) p->bPhaseless = 1;
        p->u16PhaselessTimeCnt = 0;
        p->u32PhaselessMaxTemp = 0; p->u32PhaselessMinTemp = 0;
        p->u32Ia_Integral = 0; p->u32Ib_Integral = 0; p->u32Ic_Integral = 0;
    }
    if (MLIB_Abs_Q15(f->s16SpdFilt) > (int32_t)p->u16Speed_OC_cmd) p->bSpeed_OC = 1;
}
