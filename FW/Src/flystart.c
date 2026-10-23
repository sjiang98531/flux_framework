/* 飞车检测，见 flystart.h。 */
#include "flystart.h"
#include "meas.h"
#include "est.h"
#include "hal.h"
#include "mc_math.h"
#include "mathx.h"
#include "fw_config.h"

FlyStart_TypeDef gFly;

/* 带移位的乘：sh >= 0 左移，< 0 右移 */
static __forceinline int32_t shiftBi(int32_t v, int16_t sh)
{
    return sh >= 0 ? (int32_t)((uint32_t)v << sh) : (v >> -sh);
}

/* 跟踪短路电流角度的二阶 PLL（原厂 MLIB_PLL 的语义）：
 *   积分器 += Ki*err*2，转速 = 积分器 + Kp*err*2（各自再按 Sh 移位）
 *   角度 += 转速 * ThGain（32 位乘法拆成高低 16 位做，避免溢出）
 * 返回 Q15 角度。 */
static int16_t trackPll(int16_t err, PLL_TypeDef *p)
{
    int32_t kp = shiftBi((int32_t)p->s16PGain * err * 2, p->i16PGainSh);
    int32_t ki = shiftBi((int32_t)p->s16IGain * err * 2, p->i16IGainSh);
    int32_t sp, th;
    p->s32I_1 += ki;
    sp = kp + p->s32I_1;
    p->s32Speed = sp;
    th = (((sp & 0xFFFF) * (int32_t)p->s16ThGain) >> 15) + (((sp >> 16) * (int32_t)p->s16ThGain) << 1);
    th = shiftBi(th, p->i16ThGainSh);
    p->s32Theta += th;
    return (int16_t)(p->s32Theta >> 16);
}

void FlyStart_CalibFast(void)
{
    FlyStart_TypeDef *fs = &gFly;
    int16_t err;
    sAngle_Trig a;

    HAL_PwmShortLowSides(gHal.caliDuty);         /* 三相下桥短接 */

    MCFLIB_Clark_Sat_S16(&gMeas.iabc, &gMeas.iab);
    fs->s32BemfAmp2 = (int32_t)gMeas.iab.s16Alpha * gMeas.iab.s16Alpha + (int32_t)gMeas.iab.s16Beta * gMeas.iab.s16Beta;
    fs->s16BemfAmp = (int16_t)Mx_Isqrt32((uint32_t)fs->s32BemfAmp2);    /* 每拍一次逐位开方，约 16 次循环 */

    /* 鉴相：短路电流矢量与估计角的叉积 */
    a = Mc_SinCos(fs->s16PosElEst_fly);
    fs->sAngle_fly.s16Cos = a.s16Cos;
    fs->sAngle_fly.s16Sin = a.s16Sin;
    err = (int16_t)(MLIB_Mul_Q15((int16_t)-gMeas.iab.s16Alpha, a.s16Cos) - MLIB_Mul_Q15(gMeas.iab.s16Beta, a.s16Sin));
    fs->s16ThetaErr_fly = err;
    fs->s16PosElEst_fly = trackPll(err, &gEst.pll);
}

/* ===========================================================================
 * FLY_PHASOR：短路电流矢量测转速和角度（见 flystart.h）
 * =========================================================================*/
static int32_t s_sumA, s_sumB;          /* 本块电流累加 */
static int32_t s_prevA, s_prevB;        /* 上一块平均电流（/2，防乘法溢出） */
static int32_t s_C, s_D;                /* 叉积 / 点积的一阶低通 */
static int16_t s_prevSpd;
static uint16_t s_blocks;
static uint8_t s_tick, s_havePrev, s_haveCD, s_sawSpin, s_stillDone;

void FlyStart_Begin(void)
{
    s_sumA = 0; s_sumB = 0; s_tick = 0; s_blocks = 0;
    s_havePrev = 0; s_haveCD = 0; s_sawSpin = 0; s_prevSpd = 0; s_stillDone = 0;
    gFly.stage = 1; gFly.amp = 0; gFly.speed = 0; gFly.stable = 0; gFly.still = 0; gFly.brakeMs = 0;
}

/* 转子角 = 电流方向 + sgn(w) * (90° + atan(|w|Lq/Rs))，用本拍的瞬时电流 */
static int16_t catchAngle(int16_t spd)
{
    uint32_t ai = Mx_Atan2(gMeas.iab.s16Beta, gMeas.iab.s16Alpha);
    int32_t w = spd < 0 ? -(int32_t)spd : spd;
    uint32_t off = 0x40000000UL + Mx_Atan2(w * FLY_WLR_Q12, 32768L * 4096L);
    return (int16_t)((spd > 0 ? ai + off : ai - off) >> 16);
}

FlyVerdict_t FlyStart_DetectFast(int16_t cmdSign)
{
    int32_t a, b;

    HAL_PwmShortLowSides(gHal.caliDuty);               /* 三相下桥短接（也就是制动） */
    MCFLIB_Clark_Sat_S16(&gMeas.iabc, &gMeas.iab);
    /* 静止在上一块末尾判出，下一拍才切：静止时切到定位的那一拍与 FLY_VENDOR 相同
     * （进 Calib 后第 FLYBRAKE_TIME*PWM_FREQ + 1 拍），冷启动行为逐位不变 */
    if (s_stillDone) { gFly.stage = 4; return FLY_STILL; }
    s_sumA += gMeas.iab.s16Alpha;
    s_sumB += gMeas.iab.s16Beta;
    if (++s_tick < FLY_BLOCK_TICKS) return FLY_BUSY;

    /* ---- 每 1 ms 一次 ---- */
    s_tick = 0;
    a = s_sumA / (2 * FLY_BLOCK_TICKS);                 /* 平均电流的一半：|a|,|b| <= 16384 */
    b = s_sumB / (2 * FLY_BLOCK_TICKS);
    s_sumA = 0; s_sumB = 0;
    gFly.brakeMs++;
    gFly.amp = Mx_Sat16(2 * (int32_t)Mx_Isqrt32((uint32_t)(a * a + b * b)));
    if (gFly.amp <= gFly.s16BemfAmpTh) {
        /* 电流太小，转速测不准：当作没在转（本电机约 30 rpm 以下）。
         * 静止计时从进 Calib 就开始（静止时没有暂态要等） */
        s_havePrev = 0; s_haveCD = 0; s_prevSpd = 0;
        gFly.speed = 0; gFly.stable = 0;
        if (++gFly.still >= FLY_STILL_BLOCKS) {
            if (s_sawSpin) gFly.brakes++;
            s_stillDone = 1;
        }
        if (s_blocks < FLY_SETTLE_BLOCKS) s_blocks++;
    } else if (s_blocks < FLY_SETTLE_BLOCKS) {
        s_blocks++;                                     /* 在转：等短接暂态衰减后再测转速 */
        gFly.still = 0;
        s_sawSpin = 1;
    } else {
        gFly.stage = 2;
        s_sawSpin = 1;
        gFly.still = 0;
        if (s_havePrev) {
            /* 相邻两块的夹角 = 1 ms 转过的电角度。|c|,|d| <= 5.4e8，低通用先移位再相减防溢出 */
            int32_t c = s_prevA * b - s_prevB * a;
            int32_t d = s_prevA * a + s_prevB * b;
            int32_t w, dv;
            int16_t spd;
            if (!s_haveCD) { s_C = c; s_D = d; s_haveCD = 1; }
            else { s_C += (c >> 2) - (s_C >> 2); s_D += (d >> 2) - (s_D >> 2); }
            spd = Mx_Sat16((int32_t)(((int64_t)(int32_t)Mx_Atan2(s_C, s_D) * FLY_DPHI_K) >> 32));
            gFly.speed = spd;
            w = spd < 0 ? -(int32_t)spd : spd;
            dv = (int32_t)spd - s_prevSpd;
            if (dv < 0) dv = -dv;
            /* 稳定 = 够快、与上次同号、变化 < 1/8 */
            if (w >= FLY_STABLE_MIN_Q15 && (spd > 0) == (s_prevSpd > 0) && dv <= (w >> 3)) {
                if (gFly.stable < 0xFFFF) gFly.stable++;
            } else {
                gFly.stable = 0;
            }
            s_prevSpd = spd;
            if (gFly.stable >= FLY_STABLE_MS && (spd > 0) == (cmdSign > 0)) {
                gFly.speedCatch = spd;
                gFly.thetaCatch = catchAngle(spd);
                gFly.stage = 3;
                gFly.catches++;
                return (w >= FLY_CATCH_Q15) ? FLY_CATCH : FLY_CATCH_IF;   /* 后者只在 FLY_LOW_IF 出现 */
            }
            /* 反向、或低于接管转速：继续短接制动 */
        }
        s_havePrev = 1;
    }
    s_prevA = a; s_prevB = b;
    if (gFly.brakeMs > FLY_TIMEOUT_BLOCKS) { gFly.stage = 5; return FLY_TIMEOUT; }
    return FLY_BUSY;
}
