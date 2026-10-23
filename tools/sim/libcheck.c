/* 逐函数对比：自己写的替代函数 vs 原厂库模型（vendor_lib.c，按反汇编翻译）。
 * 随机输入，每个函数几十万次，任何一次输出或状态不同就报错。
 *   tools/sim/build/libcheck            -> 最后一行 "LIBCHECK OK"
 * 覆盖：Mc_SinCos、Mc_SqrtQ15、Mc_RampQ31、Svpwm_Calc、FocCore_BusNormalize、
 *       Meas_SlowFilter、Protect_SpinSlow、FlyStart_CalibFast。 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "motor_structure.h"        /* 原厂结构体：原厂库模型（vendor_lib.c）读写全局 FOC / USER */
#include "mc_types.h"
#include "svpwm.h"
#include "meas.h"
#include "protect.h"
#include "flystart.h"
#include "foc_core.h"
#include "est.h"
#include "speed_ctrl.h"
#include "hal.h"

EPWM_TypeDef sim_EPWM; TIM_TypeDef sim_TIM0, sim_TIM3; ADC_TypeDef sim_ADC; WDT_TypeDef sim_WDT; GPIO_TypeDef sim_GPIOA;
USER_TypeDef USER; FOC_TypeDef FOC; FastBoot_TypeDef FastBoot_Para; volatile unsigned int eM1_RunSubState;
/* 新代码的模块状态（被测的 meas.c / protect.c / flystart.c 读写；这里没链接的模块在此定义） */
FocCore_TypeDef gFoc; SpeedCtrl_TypeDef gSpd; Est_TypeDef gEst; Hal_TypeDef gHal;

/* 原厂库模型里的函数（vendor_lib.c），名字是原厂的。PWM_GEN_T 与 Svpwm_TypeDef 布局相同。 */
sAngle_Trig MLIB_SinCos(Q15_t a);
Q15_t Sqrt_Q15(int32_t v);
Q31_t MLIB_Ramp_Q31(Q31_t target, MLIB_RAMP_T_Q31 *p);
void PWM_GEN_calc(Svpwm_TypeDef *u);
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

/* 新模块字段 <-> 原厂结构体字段 的对应表。
 * 每个用例：随机填原厂结构体 -> 按表拷进新模块 -> 两边各跑一次 ->
 * 把新模块按表写回"跑之前的原厂结构体副本" -> 必须与原厂模型跑完的结构体逐字节相同。
 * 这样既比较了输出，也证明原厂模型没有写表外的字段。 */
#define FIELDS(X) \
    X(gMeas.iabc, f->sIabc) X(gMeas.iab, f->sIAlBe) X(gMeas.vbus, f->s16Vbus) X(gMeas.vbusAvg, f->s16VbusAvg) \
    X(gMeas.kvbus, f->s16KVbus) X(gMeas.ibus, f->s16Ibus) X(gMeas.ibusAvg, f->s16IbusAvg) \
    X(gMeas.ibusRec, f->s16Ibusreconstructed) X(gMeas.speedRaw, f->FluxFeedback.s16Speed) X(gMeas.speed, f->s16SpdFilt) \
    X(gMeas.vbusFilt, u->sVbus_Filt) X(gMeas.speedFilt, u->sSpeed_Filt) X(gMeas.ibusFilt, u->sIbus_Filt) \
    X(gFoc.vdq, f->sVdq) X(gSpd.ramp, f->s16SpeedRamp) X(gEst.pll, f->sPLL) X(gHal.caliDuty, u->u16CaliPWMDuty) \
    X(gProt.u16BlockTimeCmd, f->sProtect.u16BlockTimeCmd) X(gProt.u16BlockVoltageAmp_square, f->sProtect.u16BlockVoltageAmp_square) \
    X(gProt.u16BlockSpeed, f->sProtect.u16BlockSpeed) X(gProt.u16VoltageTemp, f->sProtect.u16VoltageTemp) \
    X(gProt.u16Speed_OC_cmd, f->sProtect.u16Speed_OC_cmd) X(gProt.u16BlockCounter, f->sProtect.u16BlockCounter) \
    X(gProt.u16FspinCounter, f->sProtect.u16FspinCounter) X(gProt.sIabc_Max, f->sProtect.sIabc_Max) \
    X(gProt.u32Ia_Integral, f->sProtect.u32Ia_Integral) X(gProt.u32Ib_Integral, f->sProtect.u32Ib_Integral) \
    X(gProt.u32Ic_Integral, f->sProtect.u32Ic_Integral) X(gProt.u32PhaselessMinTemp, f->sProtect.u32PhaselessMinTemp) \
    X(gProt.u32PhaselessMaxTemp, f->sProtect.u32PhaselessMaxTemp) X(gProt.u16PhaselessTimeCnt, f->sProtect.u16PhaselessTimeCnt) \
    X(gProt.u16PhaselessTimeCmd, f->sProtect.u16PhaselessTimeCmd) X(gProt.s16I_Phaseless_threshold, f->sProtect.s16I_Phaseless_threshold) \
    X(gProt.bPhaseless, f->sProtect.bPhaseless) X(gProt.bSpeed_OC, f->sProtect.bSpeed_OC) \
    X(gProt.bRotor_block, f->sProtect.bRotor_block) X(gProt.bErr_Spin, f->sProtect.bErr_Spin) \
    X(gFly.s16PosElEst_fly, f->sFly_Start.s16PosElEst_fly) X(gFly.s16ThetaErr_fly, f->sFly_Start.s16ThetaErr_fly) \
    X(gFly.sAngle_fly, f->sFly_Start.sAngle_fly) X(gFly.s32BemfAmp2, f->sFly_Start.s32BemfAmp2) \
    X(gFly.s16BemfAmp, f->sFly_Start.s16BemfAmp) X(gFly.s16BemfAmpTh, f->sFly_Start.s16BemfAmpTh) \
    X(gFly.u32FlyBrakeTimeCnt, f->sFly_Start.u32FlyBrakeTimeCnt) X(gFly.u32FlyBrakeTimeCmd, f->sFly_Start.u32FlyBrakeTimeCmd)
#define LOAD1(a, b)  { typedef char sz_##__LINE__[sizeof(a) == sizeof(b) ? 1 : -1]; memcpy((void *)&(a), (const void *)&(b), sizeof(a)); }
#define STORE1(a, b) memcpy((void *)&(b), (const void *)&(a), sizeof(a));
static void loadNew(const FOC_TypeDef *f, const USER_TypeDef *u) { FIELDS(LOAD1) }
static void storeNew(FOC_TypeDef *f, USER_TypeDef *u) { FIELDS(STORE1) }

static uint32_t rs = 12345;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }
static int32_t rr(int32_t lo, int32_t hi) { return lo + (int32_t)(rnd() % (uint32_t)(hi - lo + 1)); }
static int fails;
#define CHECK(name, cond) do { if (!(cond)) { if (fails++ < 20) printf("MISMATCH %s at i=%ld\n", name, (long)i); } } while (0)

static void randBytes(void *p, size_t n) { uint8_t *b = (uint8_t *)p; while (n--) *b++ = (uint8_t)rnd(); }

/* 两边各跑一次，比较整块原厂状态 */
static int sameAfter(void (*runNew)(void), void (*runVendor)(void))
{
    FOC_TypeDef f0 = FOC, fn; USER_TypeDef u0 = USER, un;
    loadNew(&f0, &u0);
    runNew();
    fn = f0; un = u0; storeNew(&fn, &un);
    runVendor();
    return memcmp(&fn, &FOC, sizeof fn) == 0 && memcmp(&un, &USER, sizeof un) == 0;
}
static void newMeas(void) { Meas_SlowFilter(); }
static void vendorMeas(void) { Var_Filt(); }
static void newProt(void) { Protect_SpinSlow(); }
static void vendorProt(void) { MCF_ADV_Protect(&FOC); }
static void newFly(void) { FlyStart_CalibFast(); }
static void vendorFly(void) { MCF_FlyStart_2R(&FOC); }

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
    /* ---- Svpwm_Calc vs PWM_GEN_calc（模型读 FOC.sIabc、写 FOC.s16Ibusreconstructed） ---- */
    for (i = 0; i < 2000000; i++) {
        Svpwm_TypeDef u1, u2; int16_t ib1, ib2; MCFLIB_3_ABC_T_S16 iabc;
        memset(&u1, 0, sizeof u1);
        u1.Alpha = rr(-32768, 32767); u1.Beta = rr(-32768, 32767);
        if (i % 7 == 0) { u1.Alpha = 0; u1.Beta = 0; }
        u1.Mode = (uint8_t)(rnd() % 3); u1.N_halfPeriod = 2250; u1.CompA = 11; u1.CompB = 22; u1.CompC = 33; u2 = u1;
        iabc.s16A = (int16_t)rr(-20000, 20000); iabc.s16B = (int16_t)rr(-20000, 20000);
        iabc.s16C = (int16_t)(-iabc.s16A - iabc.s16B);
        FOC.sIabc = iabc;
        ib1 = ib2 = (int16_t)rnd(); FOC.s16Ibusreconstructed = ib2;
        Svpwm_Calc(&u1, &iabc, &ib1);
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
    /* ---- Meas_SlowFilter vs Var_Filt（母线 > 0） ---- */
    for (i = 0; i < 300000; i++) {
        randBytes(&FOC, sizeof FOC); randBytes(&USER, sizeof USER);
        FOC.s16Vbus = (int16_t)rr(1000, 32767); FOC.s16Ibus = (int16_t)rr(-3000, 30000);
        USER.sVbus_Filt.u16Sh = 3; USER.sSpeed_Filt.u16Sh = 3; USER.sIbus_Filt.u16Sh = 3;
        USER.sVbus_Filt.s32Acc = rr(8000, 262136); USER.sSpeed_Filt.s32Acc = rr(-262144, 262136); USER.sIbus_Filt.s32Acc = rr(0, 262136);
        CHECK("Meas", sameAfter(newMeas, vendorMeas));
    }
    /* ---- Protect_SpinSlow vs MCF_ADV_Protect ---- */
    for (i = 0; i < 300000; i++) {
        ADV_PROTECT *a = &FOC.sProtect;
        randBytes(&FOC, sizeof FOC); randBytes(&USER, sizeof USER);
        a->u16BlockTimeCmd = (uint16_t)rr(0, 5); a->u16PhaselessTimeCmd = (uint16_t)rr(0, 5);
        a->u16BlockCounter = (uint16_t)rr(0, 6); a->u16FspinCounter = (uint16_t)rr(0, 6);
        a->u16PhaselessTimeCnt = (uint16_t)rr(0, 6);
        a->u32Ia_Integral = (uint32_t)rr(0, 1 << 28); a->u32Ib_Integral = (uint32_t)rr(0, 1 << 28); a->u32Ic_Integral = (uint32_t)rr(0, 1 << 28);
        a->bRotor_block = 0; a->bErr_Spin = 0; a->bPhaseless = 0; a->bSpeed_OC = 0;
        /* 原厂在清零前把 min/max 写进 u32Phaseless*Temp 再清零；最终状态应完全相同 */
        CHECK("Protect", sameAfter(newProt, vendorProt));
    }
    /* ---- FlyStart_CalibFast vs MCF_FlyStart_2R（含 EPWM 寄存器） ---- */
    for (i = 0; i < 300000; i++) {
        EPWM_TypeDef e0, e1; int ok;
        randBytes(&FOC, sizeof FOC); randBytes(&USER, sizeof USER);
        FOC.sIabc.s16A = (int16_t)rr(-23000, 23000); FOC.sIabc.s16B = (int16_t)rr(-23000, 23000);
        FOC.sIabc.s16C = (int16_t)(-FOC.sIabc.s16A - FOC.sIabc.s16B);
        FOC.sPLL.i16PGainSh = (int16_t)rr(-3, 0); FOC.sPLL.i16IGainSh = (int16_t)rr(-3, 0); FOC.sPLL.i16ThGainSh = (int16_t)rr(-2, 0);
        USER.u16CaliPWMDuty = (uint16_t)rr(0, 2250);
        memset(&sim_EPWM, 0, sizeof sim_EPWM); sim_EPWM.EPWM_CHN = rnd() & 0xffff;
        e0 = sim_EPWM;
        {   /* sameAfter 先跑新代码再跑模型；EPWM 寄存器要分别从同一初值开始 */
            FOC_TypeDef f0 = FOC, fn; USER_TypeDef u0 = USER, un;
            loadNew(&f0, &u0); newFly(); e1 = sim_EPWM;
            fn = f0; un = u0; storeNew(&fn, &un);
            sim_EPWM = e0; vendorFly();
            ok = memcmp(&fn, &FOC, sizeof fn) == 0 && memcmp(&un, &USER, sizeof un) == 0 && memcmp(&e1, &sim_EPWM, sizeof e1) == 0;
        }
        CHECK("FlyStart", ok);
    }
    if (fails) { printf("LIBCHECK FAIL (%d mismatches)\n", fails); return 1; }
    printf("LIBCHECK OK\n");
    return 0;
}
