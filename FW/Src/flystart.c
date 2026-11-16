/* ============================================================================
 * @file    flystart.c
 * @brief   飞车检测，原理与接口说明见 flystart.h
 * ==========================================================================*/
#include "flystart.h"
#include "meas.h"
#include "flux_obs.h"
#include "hal.h"
#include "foc_math.h"
#include "qmath.h"
#include "cfg_derived.h"
#include "param.h"
#include "supervisor.h"

FlyStart_TypeDef gFly;

/* 直接接入 Spin 的转速下限。同向：FLY_CATCH。反向：进入 Spin 仅用于闭环减速，
 * 减速到 OPENLOOP_FALLBACK_RPM 即需回退 I/F；若接管点距回退点太近，Spin 中的 PLL 尚未收敛就要以其转速 / 角度初始化 I/F，
 * 可能导致失步。因此反向且低于 FLY_REV_SPIN_MIN_RPM 时，直接按飞车测得的转速 / 角度进入 I/F（测量值准确），由 I/F 减速过零。 */
static int32_t flySpinMin(uint8_t sameDir)
{
    if (!sameDir) return FLY_REV_SPIN_MIN_Q15;
    return FLY_CATCH_Q15;
}

/* ===========================================================================
 * 连续短接：由短路电流矢量测量转速与角度（见 flystart.h）
 * =========================================================================*/
static int32_t s_sumA, s_sumB;          /* 本块电流累加 */
static int32_t s_prevA, s_prevB;        /* 上一块平均电流（/2，防乘法溢出） */
static int32_t s_C, s_D;                /* 叉积 / 点积的一阶低通 */
static int16_t s_prevSpd;
static uint16_t s_blocks;
static uint8_t s_tick, s_havePrev, s_haveCD, s_sawSpin, s_stillDone;
/* 上次 Run 从 OpenLoop / Spin 退出且之后尚未判定过静止：转子可能仍以低于测速门限（约 30 rpm）的转速滑行，
 * 若仅按冷起动确认即判静止，IPD / I/F 将作用于仍在转动的转子而导致起动失败。该情况按"可能在转"处理，判静止有两条路：
 *   短路电流 <= 残余转速 FLY_STILL_END_RPM 对应的电流（gFly.s16EndAmpTh），持续 FLY_COLD_STILL_TIME：
 *     短路电流 = 反电势 / |Rs + jwL|，电流这么小说明转子此刻已低于允许的残余转速，不需要再制动（多数情况：停机后隔了一段时间才起动）；
 *   否则短接制动至停稳（gFly.stillSpinMs，按"门限转速衰减到残余转速"的最坏情况）。
 * 本次检测确实测到过转动（s_spinSeen）时只走第二条。上电冷起动不受影响 */
static uint8_t s_mayMove;
static uint32_t s_runEndMs;             /* 上次 Run 结束时的 gSup.ms，用于判断停机已多久 */
static uint8_t s_spinSeen;              /* 本次检测测到过转动（电流超过"在转"门限） */
static uint16_t s_quiet;                /* 连续低于残余转速电流的块数（ms） */

void FlyStart_NoteRunEnd(void) { s_mayMove = 1; s_runEndMs = gSup.ms; }
static uint8_t s_catchStep;             /* 接管分三拍完成，避免单拍计算量超过 PWM 周期 */
static uint32_t s_catchOff;             /* 阻抗角 + 90°（2^32 = 一圈） */

/* ===========================================================================
 * 短脉冲测速（见 cfg_internal.h"短脉冲测速"）
 *   P_ON：下桥短接，电流达到门限 s_pIth2 且至少 FLY_PULSE_MIN_TICKS 拍（第一个脉冲），或达到第一个脉冲的长度后松开
 *   P_GAP：上下桥全关（电流经二极管迅速降到 0），补足 FLY_PULSE_PERIOD_TICKS 后施加下一个脉冲
 *   每个脉冲末记录一次电流矢量；相邻两次的夹角 = 一个脉冲周期内转过的电角度
 *   P_CALC1 / 2 / 3：一致性与转速、电流角、接管角与判定，分三拍完成。全部使用 32 位运算及一次
 *   64 位乘法，无 64 位除法（保证单拍耗时不超过 PWM 周期）
 * =========================================================================*/
#define PULSE_FALLBACK  ((FlyVerdict_t)0xFF)
enum { P_ON = 1, P_GAP, P_CALC1, P_CALC2, P_CALC3, P_DONE };
static uint8_t  s_pState, s_pIdx, s_pBad, s_pNeedDphi;
static uint16_t s_pTick, s_pSince;
static int32_t  s_pA, s_pB, s_pPrevA, s_pPrevB;   /* 脉冲末电流矢量的一半（防止乘法溢出） */
static int32_t  s_pDphi[FLY_PULSE_N - 1];         /* 各次夹角（2^32 = 一圈） */
static int32_t  s_pSum;                           /* 夹角累计（2^28 = 一圈，最大约 600°，不溢出） */
static int16_t  s_pSpd;
static uint32_t s_pAng;
static int32_t  s_pIth2;                          /* 脉冲电流门限的平方（Q15^2），由 pulseBegin 按实测噪声计算 */

static void pulseBegin(void)
{
    s_pState = P_ON; s_pIdx = 0; s_pBad = 0; s_pNeedDphi = 0;
    s_pTick = 0; s_pSince = 0; s_pSum = 0;
    gFly.pulseLen = 0; gFly.pulseResult = FLY_PR_NONE;
    {   /* 门限 = 噪声峰值 * 每码门限（单次矢量角度误差 <= FLY_SEED_ANGLE_DEG） */
        int32_t n = gHal.adcNoisePk, ith;
        if (n < (int32_t)BOARD_ADC_NOISE_CODES) n = (int32_t)BOARD_ADC_NOISE_CODES;
        ith = n * FLY_PULSE_ITH_PER_CODE;
        if (ith > FLY_PULSE_ITH_MAX_Q15) ith = FLY_PULSE_ITH_MAX_Q15;
        s_pIth2 = ith * ith;
    }
}

/* 短脉冲无法测量（太慢 / 反向太低 / 多次测量不一致）：记录原因（gFly.pulseResult），退回连续短接测量 */
static FlyVerdict_t pulseFallback(uint16_t why)
{
    gFly.pulseResult = why;
    s_pState = P_DONE;
    return PULSE_FALLBACK;
}

/* 每拍调用（gMeas.iab 已为本拍电流）。返回 FLY_BUSY / FLY_CATCH / FLY_CATCH_OPENLOOP / PULSE_FALLBACK */
static FlyVerdict_t pulseFast(int16_t cmdSign)
{
    switch (s_pState) {
    case P_ON: {
        int32_t a = gMeas.iab.s16Alpha, b = gMeas.iab.s16Beta, m2 = a * a + b * b;
        HAL_PwmShortLowSides(gHal.caliDuty);
        gFly.stage = FLY_STAGE_PULSE;
        s_pTick++;
        if (gFly.pulseLen ? (s_pTick < gFly.pulseLen)
                          : ((m2 < s_pIth2 || s_pTick < FLY_PULSE_MIN_TICKS) && s_pTick < FLY_PULSE_MAX_TICKS))
            return FLY_BUSY;
        HAL_PwmLowOff();                                /* 松开：上下桥全关 */
        if (!gFly.pulseLen) {
            gFly.pulseLen = s_pTick;
            if (m2 < s_pIth2) return pulseFallback(FLY_PR_TOO_SLOW);     /* 最长脉冲也达不到精度要求的电流：转速太低（或静止） */
        }
        s_pPrevA = s_pA; s_pPrevB = s_pB;
        s_pA = a >> 1; s_pB = b >> 1;
        s_pNeedDphi = (s_pIdx > 0);
        s_pIdx++;
        s_pSince = 0;
        s_pState = P_GAP;
        return FLY_BUSY;
    }
    case P_GAP:
        s_pTick++; s_pSince++;
        if (s_pNeedDphi) {
            /* 夹角：atan2(叉积, 点积)，有符号 */
            int32_t d = (int32_t)QMath_Atan2(s_pPrevA * s_pB - s_pPrevB * s_pA, s_pPrevA * s_pA + s_pPrevB * s_pB);
            s_pNeedDphi = 0;
            s_pDphi[s_pIdx - 2] = d;
            s_pSum += d >> 4;
        }
        if (s_pIdx >= FLY_PULSE_N) { s_pState = P_CALC1; return FLY_BUSY; }
        if (s_pTick >= FLY_PULSE_PERIOD_TICKS) {
            s_pTick = 0;
            HAL_PwmShortLowSides(gHal.caliDuty);        /* 下一个脉冲 */
            s_pState = P_ON;
        }
        return FLY_BUSY;
    case P_CALC1: {
        int32_t w;
        uint8_t i;
        s_pSince++;
        /* 一致性：每个夹角与平均值之差不超过平均值的一半或 FLY_PULSE_TOL_MIN（取较大者）。
         * 下限 = 两次矢量各自角度噪声上限之和（2 * FLY_SEED_ANGLE_DEG，脉冲电流即按此确定） */
        /* 不显式求平均（快环不做除法）：将 |dphi - mean| <= tol 两边乘 (N-1)，mean = 16*sum/(N-1)：
         *   |(N-1)*dphi - 16*sum| <= max(8*|sum|, (N-1)*TOL_MIN)
         * 数学上精确，仅用 64 位乘法与比较，无除法 */
        {
            int64_t s16 = (int64_t)s_pSum << 4, as16 = s16 < 0 ? -s16 : s16;
            int64_t tolN = as16 >> 1;
            if (tolN < (int64_t)FLY_PULSE_TOL_MIN * (FLY_PULSE_N - 1)) tolN = (int64_t)FLY_PULSE_TOL_MIN * (FLY_PULSE_N - 1);
            for (i = 0; i < FLY_PULSE_N - 1; i++) {
                int64_t e = (int64_t)s_pDphi[i] * (FLY_PULSE_N - 1) - s16;
                if (e > tolN || e < -tolN) s_pBad = 1;
            }
        }
        if (s_pBad) return pulseFallback(FLY_PR_INCONSISTENT);
        s_pSpd = QMath_Sat16((int32_t)(((int64_t)s_pSum * FLY_PULSE_K) >> 28));
        w = s_pSpd < 0 ? -(int32_t)s_pSpd : s_pSpd;
        if (w < FLY_STABLE_MIN_Q15) return pulseFallback(FLY_PR_LOW);
        s_pState = P_CALC2;
        return FLY_BUSY;
    }
    case P_CALC2:
        s_pSince++;
        s_pAng = QMath_Atan2(s_pB, s_pA);
        s_pState = P_CALC3;
        return FLY_BUSY;
    case P_CALC3: {
        /* 电流方向 = 脉冲中点时刻的 -e 方向：转子角 = ∠i + sgn(w)*90°，再外推到本拍采样时刻
         * （脉冲中点到最后一次采样 pulseLen/2 拍 + 之后经过的 s_pSince 拍） */
        int32_t w = s_pSpd < 0 ? -(int32_t)s_pSpd : s_pSpd;
        uint32_t adv;
        s_pSince++;
        /* adv = 平均每拍转角 × (pulseLen/2 + s_pSince) = sum(2^28) * 16 * n / (2 * BASE)，n = pulseLen + 2*s_pSince */
        adv = (uint32_t)(int32_t)(((int64_t)s_pSum * (int32_t)((gFly.pulseLen + 2 * s_pSince) * FLY_PULSE_ADV_K)) >> 16);  /* 后两项之积 < 2^31（FLY_PULSE_ADV_FITS） */
        s_pAng += (s_pSpd > 0 ? 0x40000000UL : 0xC0000000UL) + adv;
        gFly.thetaCatch = (int16_t)(s_pAng >> 16);
        gFly.speedCatch = s_pSpd;
        gFly.speed = s_pSpd;
        gFly.stage = FLY_STAGE_CATCH;
        s_pState = P_DONE;
        if (w >= flySpinMin((s_pSpd > 0) == (cmdSign > 0))) { gFly.pulseResult = FLY_PR_SPIN; return FLY_CATCH; }
        gFly.pulseResult = FLY_PR_OPENLOOP;
        return FLY_CATCH_OPENLOOP;
    }
    default:
        return PULSE_FALLBACK;
    }
}

/* 从 gGain 重新装载飞车检测门限与等待停稳时间（上电及 param.c 更新参数时） */
void FlyStart_LoadGains(void)
{
    gFly.s16BemfAmpTh = gGain.flyBemfAmpTh;
    gFly.s16EndAmpTh = gGain.flyEndAmpTh;
    gFly.stillSpinMs = gGain.flyStillSpinMs;
}

/* 上电：判"在转"的电流门限；检测到过转动后"等待停稳"的时间 = FLYBRAKE_TIME + τ·ln(门限转速 / 残余转速) */
void FlyStart_Init(void)
{
    FlyStart_LoadGains();
}

/* 进入飞车检测（FlyCheck）时清除全部状态，首先进行短脉冲测速 */
void FlyStart_Begin(void)
{
    pulseBegin();
    s_sumA = 0; s_sumB = 0; s_tick = 0; s_blocks = 0;
    /* 停机已超过 flyMayMoveExpireMs：短接制动的转子早已低于残余转速，不必按最坏情况空等。
     * 间隔更短（真滑行中再起动）仍按"可能在转"保守处理。gSup.ms 在快环读不会被打断（PendSV 低于 ADC 中断）。 */
    if (s_mayMove && (uint32_t)(gSup.ms - s_runEndMs) >= gGain.flyMayMoveExpireMs) s_mayMove = 0;
    s_havePrev = 0; s_haveCD = 0; s_sawSpin = s_mayMove; s_prevSpd = 0; s_stillDone = 0; s_catchStep = 0;
    s_spinSeen = 0; s_quiet = 0;
    gFly.stage = FLY_STAGE_SETTLE; gFly.ampM2 = 0; gFly.speed = 0; gFly.stable = 0; gFly.still = 0; gFly.brakeMs = 0;
}

/* 转子角 = 电流方向 + sgn(w) * (90° + atan(|w|Lq/Rs))。分两拍计算：
 *   impedanceOffset：90° + 阻抗角，只与转速有关
 *   catchAngle：本拍瞬时电流方向 + 上述偏移 */
static uint32_t impedanceOffset(int16_t spd)
{
    int32_t w = spd < 0 ? -(int32_t)spd : spd;
    return 0x40000000UL + QMath_Atan2(w * gGain.flyWlrQ12, 32768L * 4096L);
}
static int16_t catchAngle(int16_t spd, uint32_t off)
{
    uint32_t ai = QMath_Atan2(gMeas.iab.s16Beta, gMeas.iab.s16Alpha);
    return (int16_t)((spd > 0 ? ai + off : ai - off) >> 16);
}

/* 飞车检测快环（每拍）：首先执行短脉冲测速，测得即直接给出结论；否则转入连续短接：
 *   下桥持续短接（同时起制动作用），每 1 ms 求一次平均电流矢量，相邻两块的夹角 = 1 ms 内转过的电角度 -> 转速；
 *   连续 FLY_STABLE_MS 次稳定即接管（随后两拍计算阻抗角与接管角）；电流始终很小 -> 静止；制动过久 -> 超时。
 * 返回 FLY_BUSY（测量中）/ FLY_STILL / FLY_CATCH / FLY_CATCH_OPENLOOP / FLY_TIMEOUT */
FlyVerdict_t FlyStart_DetectFast(int16_t cmdSign)
{
    int32_t a, b;

    if (s_pState != P_DONE) {
        FlyVerdict_t v;
        FocMath_ClarkeSat(&gMeas.iabc, &gMeas.iab);
        v = pulseFast(cmdSign);
        if (v != PULSE_FALLBACK) return v;
        /* 退回短接测量：从下一拍开始（本拍已全关或刚完成脉冲） */
        HAL_PwmShortLowSides(gHal.caliDuty);
        return FLY_BUSY;
    }
    HAL_PwmShortLowSides(gHal.caliDuty);               /* 三相下桥短接（同时起制动作用） */
    FocMath_ClarkeSat(&gMeas.iabc, &gMeas.iab);
    /* 静止在上一块末尾判出，下一拍才切换（冷起动约 FLY_COLD_STILL_TIME 后进入 Align） */
    if (s_stillDone) { gFly.stage = FLY_STAGE_STILL; return FLY_STILL; }
    /* 接管：判定之后第 1 拍计算阻抗角，第 2 拍用当拍电流计算转子角并切换 */
    if (s_catchStep == 1) { s_catchOff = impedanceOffset(gFly.speedCatch); s_catchStep = 2; return FLY_BUSY; }
    if (s_catchStep == 2) {
        int32_t w = gFly.speedCatch < 0 ? -(int32_t)gFly.speedCatch : gFly.speedCatch;
        s_catchStep = 0;
        gFly.thetaCatch = catchAngle(gFly.speedCatch, s_catchOff);
        gFly.stage = FLY_STAGE_CATCH;
        return (w >= flySpinMin((gFly.speedCatch > 0) == (cmdSign > 0))) ? FLY_CATCH : FLY_CATCH_OPENLOOP;
    }
    s_sumA += gMeas.iab.s16Alpha;
    s_sumB += gMeas.iab.s16Beta;
    if (++s_tick < FLY_BLOCK_TICKS) return FLY_BUSY;

    /* ---- 每 1 ms 执行一次 ---- */
    s_tick = 0;
    a = s_sumA / (2 * FLY_BLOCK_TICKS);                 /* 平均电流的一半：|a|,|b| <= 16384 */
    b = s_sumB / (2 * FLY_BLOCK_TICKS);
    s_sumA = 0; s_sumB = 0;
    gFly.brakeMs++;
    /* 快环不开方：判据 2*floor(sqrt(m)) <= th 等价于 m < (floor(th/2) + 1)^2（整数意义上精确） */
    gFly.ampM2 = (uint32_t)(a * a + b * b);
    if (gFly.s16BemfAmpTh >= 0 &&
        gFly.ampM2 < (uint32_t)(((gFly.s16BemfAmpTh >> 1) + 1) * ((gFly.s16BemfAmpTh >> 1) + 1))) {
        /* 电流过小，转速无法准确测量：视为未转动（本电机约 30 rpm 以下）。
         * 静止计时从进入 FlyCheck 即开始（静止时无暂态需等待） */
        s_havePrev = 0; s_haveCD = 0; s_prevSpd = 0;
        gFly.speed = 0; gFly.stable = 0;
        if (gFly.ampM2 < (uint32_t)(((gFly.s16EndAmpTh >> 1) + 1) * ((gFly.s16EndAmpTh >> 1) + 1))) s_quiet++;
        else s_quiet = 0;
        /* 冷起动无制动暂态，短确认即可；上次运行后可能在滑行：电流已低于残余转速对应值则同样短确认；测到过转动则等待停稳 */
        if (++gFly.still >= (s_sawSpin ? gFly.stillSpinMs : FLY_COLD_STILL_BLOCKS) ||
            (!s_spinSeen && s_quiet >= FLY_COLD_STILL_BLOCKS)) {
            s_stillDone = 1;
            s_mayMove = 0;
        }
        if (s_blocks < gGain.flySettleBlocks) s_blocks++;
    } else if (s_blocks < gGain.flySettleBlocks) {
        s_blocks++;                                     /* 正在转动：等待短接暂态衰减后再测转速 */
        gFly.still = 0;
        s_sawSpin = 1; s_spinSeen = 1; s_quiet = 0;
    } else {
        gFly.stage = FLY_STAGE_MEASURE;
        s_sawSpin = 1; s_spinSeen = 1; s_quiet = 0;
        gFly.still = 0;
        if (s_havePrev) {
            /* 相邻两块的夹角 = 1 ms 内转过的电角度。|c|,|d| <= 5.4e8，低通采用先移位再相减以防溢出 */
            int32_t c = s_prevA * b - s_prevB * a;
            int32_t d = s_prevA * a + s_prevB * b;
            int32_t w, dv;
            int16_t spd;
            if (!s_haveCD) { s_C = c; s_D = d; s_haveCD = 1; }
            else { s_C += (c >> 2) - (s_C >> 2); s_D += (d >> 2) - (s_D >> 2); }
            spd = QMath_Sat16((int32_t)(((int64_t)(int32_t)QMath_Atan2(s_C, s_D) * FLY_DPHI_K) >> 32));
            gFly.speed = spd;
            w = spd < 0 ? -(int32_t)spd : spd;
            dv = (int32_t)spd - s_prevSpd;
            if (dv < 0) dv = -dv;
            /* 稳定 = 转速足够、与上次同号、变化 < 1/8 */
            if (w >= FLY_STABLE_MIN_Q15 && (spd > 0) == (s_prevSpd > 0) && dv <= (w >> 3)) {
                if (gFly.stable < 0xFFFF) gFly.stable++;
            } else {
                gFly.stable = 0;
            }
            s_prevSpd = spd;
            if (gFly.stable >= FLY_STABLE_MS) {     /* 同向、反向均按转子自身方向接管（见 cfg_internal.h"转子与指令反向时"） */
                gFly.speedCatch = spd;
                s_catchStep = 1;                    /* 后续两拍完成接管（2 拍内转速变化可忽略） */
                return FLY_BUSY;
            }
            /* 反向或低于接管转速：继续短接制动 */
        }
        s_havePrev = 1;
    }
    s_prevA = a; s_prevB = b;
    if (gFly.brakeMs > FLY_TIMEOUT_BLOCKS) { gFly.stage = FLY_STAGE_TIMEOUT; return FLY_TIMEOUT; }
    return FLY_BUSY;
}
