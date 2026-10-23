/* 逐函数对比：自己写的替代函数 vs 原厂库模型（vendor_lib.c，按反汇编翻译）。
 * 随机输入，每个函数几十万次，任何一次输出或状态不同就报错。
 *   tools/sim/build/libcheck            -> 最后一行 "LIBCHECK OK"
 * 覆盖：Mc_SinCos、Mc_SqrtQ15、Mc_RampQ31、Svpwm_Calc、FocCore_BusNormalize、
 *       Meas_SlowFilter、Protect_SpinSlow、FlyStart_CalibFast。 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mc_types.h"
#include "svpwm.h"
#include "meas.h"
#include "protect.h"
#include "flystart.h"
#include "foc_core.h"

EPWM_TypeDef sim_EPWM; TIM_TypeDef sim_TIM0, sim_TIM3; ADC_TypeDef sim_ADC; WDT_TypeDef sim_WDT; GPIO_TypeDef sim_GPIOA;
USER_TypeDef USER; FOC_TypeDef FOC; FastBoot_TypeDef FastBoot_Para; volatile unsigned int eM1_RunSubState;

/* 原厂库模型里的函数（vendor_lib.c），名字是原厂的 */
sAngle_Trig MLIB_SinCos(Q15_t a);
Q15_t Sqrt_Q15(int32_t v);
Q31_t MLIB_Ramp_Q31(Q31_t target, MLIB_RAMP_T_Q31 *p);
void PWM_GEN_calc(PWM_GEN_T *u);
void DcBus_Ripple_suppress(MCFLIB_2_ALBE_T_S16 *in, Q15_t s16Vbus, MCFLIB_2_ALBE_T_S16 *out);
void Var_Filt(void);
void MCF_ADV_Protect(FOC_TypeDef *f);
void MCF_FlyStart_2R(FOC_TypeDef *f);

/* flystart.c 调 HAL；这里给一个与 hal_sc32r602.c 相同的实现 */
void HAL_PwmShortLowSides(uint16_t cmp)
{
    EPWM->EPWM_CHN |= 0x2a;
    EPWM->EPWM_CHN &= 0xffea;
    EPWM->EPWM_DT[0] = cmp; EPWM->EPWM_DT[2] = cmp; EPWM->EPWM_DT[4] = cmp;
}

static uint32_t rs = 12345;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static int32_t rr(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }
static int fails;
#define CHECK(name, cond) do { if (!(cond)) { if (fails++ < 20) printf("MISMATCH %s at i=%ld\n", name, (long)i); } } while (0)

/* 随机填一个 FOC 结构体的"会被读到"的字段 */
static void randFoc(FOC_TypeDef *f)
{
    uint8_t *b = (uint8_t *)f; size_t k;
    for (k = 0; k < sizeof *f; k++) b[k] = (uint8_t)rnd();
}

int main(void)
{
    long i;
    /* ---- Mc_SinCos：全部 65536 个角度 ---- */
    for (i = -32768; i <= 32767; i++) {
        sAngle_Trig a = Mc_SinCos((int16_t)i), b = MLIB_SinCos((Q15_t)i);
        CHECK("SinCos", a.s16Cos == b.s16Cos && a.s16Sin == b.s16Sin);
    }
    /* ---- Mc_SqrtQ15：定义域 0..2^30 ---- */
    for (i = 0; i < 2000000; i++) {
        int32_t v = (i < 1000) ? (int32_t)(i << 20) : rr(0, 1 << 30);
        CHECK("Sqrt", Mc_SqrtQ15(v) == Sqrt_Q15(v));
    }
    /* ---- Mc_RampQ31 ---- */
    for (i = 0; i < 1000000; i++) {
        MLIB_RAMP_T_Q31 p1, p2; int32_t tg = (int32_t)rnd() >> 1, r1, r2;
        p1.s32RampUp = rr(0, 1 << 24); p1.s32RampDown = rr(0, 1 << 24); p1.s32State = (int32_t)rnd() >> 1; p2 = p1;
        r1 = Mc_RampQ31(tg, &p1); r2 = MLIB_Ramp_Q31(tg, &p2);
        CHECK("Ramp", r1 == r2 && memcmp(&p1, &p2, sizeof p1) == 0);
    }
    /* ---- Svpwm_Calc vs PWM_GEN_calc（模型读写全局 FOC） ---- */
    for (i = 0; i < 2000000; i++) {
        PWM_GEN_T u1, u2; int16_t ib1, ib2;
        memset(&u1, 0, sizeof u1);
        u1.Alpha = rr(-32768, 32767); u1.Beta = rr(-32768, 32767);
        if (i % 7 == 0) { u1.Alpha = 0; u1.Beta = 0; }
        u1.Mode = (uint8_t)(rnd() % 3); u1.N_halfPeriod = 2250; u1.CompA = 11; u1.CompB = 22; u1.CompC = 33; u2 = u1;
        FOC.sIabc.s16A = (int16_t)rr(-20000, 20000); FOC.sIabc.s16B = (int16_t)rr(-20000, 20000);
        FOC.sIabc.s16C = (int16_t)(-FOC.sIabc.s16A - FOC.sIabc.s16B);
        ib1 = ib2 = (int16_t)rnd(); FOC.s16Ibusreconstructed = ib2;
        Svpwm_Calc(&u1, &FOC.sIabc, &ib1);
        PWM_GEN_calc(&u2); ib2 = FOC.s16Ibusreconstructed;
        CHECK("Svpwm", memcmp(&u1, &u2, sizeof u1) == 0 && ib1 == ib2);
    }
    /* ---- FocCore_BusNormalize vs DcBus_Ripple_suppress（模型用 FOC.s16KVbus） ---- */
    for (i = 0; i < 1000000; i++) {
        MCFLIB_2_ALBE_T_S16 in, o1, o2; int16_t kv = (int16_t)rr(1500, 6000);
        in.s16Alpha = (int16_t)rr(-32768, 32767); in.s16Beta = (int16_t)rr(-32768, 32767);
        FOC.s16KVbus = kv;
        FocCore_BusNormalize(&in, kv, &o1); DcBus_Ripple_suppress(&in, 0, &o2);
        CHECK("BusNorm", o1.s16Alpha == o2.s16Alpha && o1.s16Beta == o2.s16Beta);
    }
    /* ---- Meas_SlowFilter vs Var_Filt（都读写全局 FOC/USER；母线 > 0） ---- */
    for (i = 0; i < 300000; i++) {
        FOC_TypeDef f0; USER_TypeDef u0; FOC_TypeDef f1; USER_TypeDef u1;
        randFoc(&FOC);
        FOC.s16Vbus = (int16_t)rr(1000, 32767); FOC.s16Ibus = (int16_t)rr(-3000, 30000);
        USER.sVbus_Filt.u16Sh = 3; USER.sSpeed_Filt.u16Sh = 3; USER.sIbus_Filt.u16Sh = 3;
        USER.sVbus_Filt.s32Acc = rr(8000, 262136); USER.sSpeed_Filt.s32Acc = rr(-262144, 262136); USER.sIbus_Filt.s32Acc = rr(0, 262136);
        f0 = FOC; u0 = USER;
        Meas_SlowFilter(); f1 = FOC; u1 = USER;
        FOC = f0; USER = u0;
        Var_Filt();
        CHECK("Meas", memcmp(&f1, &FOC, sizeof f1) == 0 && memcmp(&u1, &USER, sizeof u1) == 0);
    }
    /* ---- Protect_SpinSlow vs MCF_ADV_Protect ---- */
    for (i = 0; i < 300000; i++) {
        FOC_TypeDef a, b;
        randFoc(&a);
        a.sProtect.u16BlockTimeCmd = (uint16_t)rr(0, 5); a.sProtect.u16PhaselessTimeCmd = (uint16_t)rr(0, 5);
        a.sProtect.u16BlockCounter = (uint16_t)rr(0, 6); a.sProtect.u16FspinCounter = (uint16_t)rr(0, 6);
        a.sProtect.u16PhaselessTimeCnt = (uint16_t)rr(0, 6);
        a.sProtect.u32Ia_Integral = (uint32_t)rr(0, 1 << 28); a.sProtect.u32Ib_Integral = (uint32_t)rr(0, 1 << 28); a.sProtect.u32Ic_Integral = (uint32_t)rr(0, 1 << 28);
        a.sProtect.bRotor_block = 0; a.sProtect.bErr_Spin = 0; a.sProtect.bPhaseless = 0; a.sProtect.bSpeed_OC = 0;
        b = a;
        Protect_SpinSlow(&a); MCF_ADV_Protect(&b);
        /* 原厂在清零前把 min/max 写进 u32Phaseless*Temp 再清零；最终状态应完全相同 */
        CHECK("Protect", memcmp(&a, &b, sizeof a) == 0);
    }
    /* ---- FlyStart_CalibFast vs MCF_FlyStart_2R（含 EPWM 寄存器） ---- */
    for (i = 0; i < 300000; i++) {
        FOC_TypeDef a, b; EPWM_TypeDef e0, e1;
        randFoc(&a);
        a.sIabc.s16A = (int16_t)rr(-23000, 23000); a.sIabc.s16B = (int16_t)rr(-23000, 23000);
        a.sIabc.s16C = (int16_t)(-a.sIabc.s16A - a.sIabc.s16B);
        a.sPLL.i16PGainSh = (int16_t)rr(-3, 0); a.sPLL.i16IGainSh = (int16_t)rr(-3, 0); a.sPLL.i16ThGainSh = (int16_t)rr(-2, 0);
        b = a;
        memset(&sim_EPWM, 0, sizeof sim_EPWM); sim_EPWM.EPWM_CHN = rnd() & 0xffff; USER.u16CaliPWMDuty = (uint16_t)rr(0, 2250);
        e0 = sim_EPWM;
        FlyStart_CalibFast(&a); e1 = sim_EPWM;
        sim_EPWM = e0;
        MCF_FlyStart_2R(&b);
        CHECK("FlyStart", memcmp(&a, &b, sizeof a) == 0 && memcmp(&e1, &sim_EPWM, sizeof e1) == 0);
    }
    if (fails) { printf("LIBCHECK FAIL (%d mismatches)\n", fails); return 1; }
    printf("LIBCHECK OK\n");
    return 0;
}
