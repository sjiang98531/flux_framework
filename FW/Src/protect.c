/* ============================================================================
 * @file    protect.c
 * @brief   运行中保护，接口说明见 protect.h
 * ==========================================================================*/
#include "protect.h"
#include "meas.h"
#include "hal.h"
#include "cfg_derived.h"
#include "param.h"
#include "qmath.h"
#include "compiler.h"

Protect_TypeDef gProt;

/* 母线过 / 欠压需持续的慢环拍数（滤除采样毛刺）。过压判断 vbusAvg；欠压判断 DCBUS_UV_PEAK_S 窗口内的母线峰值 */
#define BUS_FILT_TICKS  ((uint16_t)(DCBUS_FILT_S * SLOWLOOP_FREQ + 0.5))
#define BUS_PEAK_TICKS  ((uint16_t)(DCBUS_UV_PEAK_S * SLOWLOOP_FREQ + 0.5))

/* 上电：过压 / 欠压门限换算为 Q15，清除缺相统计 */
void Protect_Init(void)
{
    gProt.ovpCmd = Q15(DCBUS_OVER / UDC_MAX);
    gProt.uvpCmd = Q15(DCBUS_UNDER / UDC_MAX);
    Protect_PhaseReset();
}

/* 母线过压 / 欠压（慢环每拍，由 Ctrl_SlowTick 调用）：过压看母线平均值、欠压看母线峰值（即输入电压），越限持续 DCBUS_FILT_S 后立即关闭 PWM，
 * 结果写入 gProt.busFlags（PROTECT_BUS_OV / UV 位），由快环 faultDetect 转换为故障 */
void Protect_BusSlow(void)
{
    static uint16_t u16OvCntr, u16UvCntr, u16PkN;
    static int16_t s16Pk, s16PkWin = 32767;     /* 当前窗口峰值、上一窗口峰值（未满一个窗口前不判欠压） */
    uint8_t f = 0;
    if (gMeas.vbus > s16Pk) s16Pk = gMeas.vbus;
    if (++u16PkN >= BUS_PEAK_TICKS) { s16PkWin = s16Pk; s16Pk = 0; u16PkN = 0; }
    if (gMeas.vbusAvg > gProt.ovpCmd) {
        if (u16OvCntr <= BUS_FILT_TICKS) u16OvCntr++;
        else {
            f |= PROTECT_BUS_OV;
            HAL_PwmAllOff();
        }
    } else {
        u16OvCntr = 0;
    }
    if (s16PkWin < gProt.uvpCmd) {
        if (u16UvCntr <= BUS_FILT_TICKS) u16UvCntr++;
        else {
            f |= PROTECT_BUS_UV;
            HAL_PwmAllOff();
        }
    } else {
        u16UvCntr = 0;
    }
    gProt.busFlags = f;
}

/* 缺相统计清零（开始一个新窗口） */
void Protect_PhaseReset(void)
{
    gProt.sumAbsA = 0;
    gProt.sumAbsB = 0;
    gProt.sumAbsC = 0;
    gProt.sampleN = 0;
    gProt.angleAcc = 0;
}

/* 缺相统计（快环每拍）：三相 |i| 分别累加。某相断线时，该相累加值远小于另外两相 */
RAMFUNC void Protect_PhaseFast(void)
{
    if (gProt.sampleN >= PHASELESS_N_MAX) return;     /* 慢环将丢弃该窗口 */
    gProt.sumAbsA += (uint32_t)QMath_Abs16(gMeas.iabc.s16A);
    gProt.sumAbsB += (uint32_t)QMath_Abs16(gMeas.iabc.s16B);
    gProt.sumAbsC += (uint32_t)QMath_Abs16(gMeas.iabc.s16C);
    gProt.sampleN++;
}

/* 缺相判定（慢环每拍）：按转速累加电角度，满 PHASELESS_PERIODS 个电周期判定一次（整周期内的平均 |i| 才可比较）：
 *   电流足够大（最大相 >= PHASELESS_RATIO * 噪声，过小时比值无意义）且 最大 / 最小 > PHASELESS_RATIO -> 缺相。
 *   转速过低、2 s 内未转满则丢弃窗口。返回 1 = 缺相 */
uint8_t Protect_PhaseSlow(int16_t speedQ15)
{
    uint32_t a = gProt.sumAbsA, b = gProt.sumAbsB, c = gProt.sumAbsC;
    uint32_t mx, mn, n = gProt.sampleN, floor;
    uint8_t lost = 0;
    gProt.angleAcc += (uint32_t)(speedQ15 < 0 ? -(int32_t)speedQ15 : speedQ15);
    if (gProt.angleAcc < PHASELESS_ACC_FULL) {
        if (n >= PHASELESS_N_MAX) Protect_PhaseReset();     /* 转速过低，窗口作废 */
        return 0;
    }
    mx = a > b ? a : b;
    if (c > mx) mx = c;
    mn = a < b ? a : b;
    if (c < mn) mn = c;
    /* 电流门限：最大相平均 |i| >= PHASELESS_RATIO * 噪声峰值（码 -> Q15 每码 16） */
    floor = (uint32_t)PHASELESS_RATIO * 16u * (uint32_t)gHal.adcNoisePk;
    if (n > 0 && mx >= floor * n) {
        if (mx > (uint32_t)PHASELESS_RATIO * mn) lost = 1;
    }
    Protect_PhaseReset();
    return lost;
}

/* 超速：|转速| 超过 PROTECT_SPEED_OC_RPM（反电势达到回馈兜底母线门限的转速与机械允许转速中的较小值）时返回 1 */
uint8_t Protect_OverSpeed(int16_t speedQ15)
{
    return (uint8_t)((speedQ15 < 0 ? -(int32_t)speedQ15 : speedQ15) > gGain.protectSpeedOcQ15);
}
