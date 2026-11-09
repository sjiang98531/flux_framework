/* ============================================================================
 * @file    protect.h
 * @brief   运行中保护
 *
 * 缺相（Spin 与 I/F 共用）：
 *   快环 Protect_PhaseFast 累加三相电流绝对值；慢环 Protect_PhaseSlow 按转速累加电角度，
 *   满 PHASELESS_PERIODS 个电周期为一个窗口（转速越高窗口越短），窗口结束时比较：
 *   最大相平均 |i| > PHASELESS_RATIO 倍最小相 -> 缺相。三相平衡时比值 ≈ 1，断相时该相只剩采样噪声。
 *   最大相平均 |i| 低于 PHASELESS_RATIO 倍噪声峰值（gHal.adcNoisePk，上电自测）的窗口不做判断：
 *   电流过小时断相与噪声无法区分，宁可不判也不误判（轻载风机）。
 *   窗口 2 s 内未转满（低于约 24 rpm）则丢弃重新开始，同时防止累加溢出。
 * 超速：|转速| > PROTECT_SPEED_OC，门限由反电势与母线电压推导（cfg_ctrl.h）。
 * 堵转：由失锁停机 -> 重新检测 -> I/F 超时（MC_FAULT_ROTOR_BLOCKED）处理。
 * ==========================================================================*/
#ifndef PROTECT_H
#define PROTECT_H
#include "qtypes.h"
#include "foc_math.h"

typedef struct {
    uint32_t u32Ia_Integral;            /* 当前窗口三相电流绝对值累加（Q15，快环） */
    uint32_t u32Ib_Integral;
    uint32_t u32Ic_Integral;
    uint32_t u32PhaseN;                 /* 当前窗口的快环拍数 */
    uint32_t u32PhaseAcc;               /* 当前窗口转过的电角度（|转速 Q15| 的慢环累加） */
    uint16_t u16PhaseRatioLast;         /* 上一个已判断窗口：最大 / 最小 x 16（诊断用） */
    int16_t  ovpCmd;                    /* 母线过压门限 Q15 */
    int16_t  uvpCmd;                    /* 母线欠压门限 Q15 */
    volatile uint8_t busFlags;          /* 母线越限判定（PROTECT_BUS_OV / UV，慢环写、快环读） */
} Protect_TypeDef;

extern Protect_TypeDef gProt;

/* 母线过压 / 欠压（慢环每拍）：越限持续 DCBUS_FILT_S 后关闭 PWM，结果写入 gProt.busFlags（PROTECT_BUS_OV / UV 位） */
#define PROTECT_BUS_OV          0x01u
#define PROTECT_BUS_UV          0x02u
void Protect_Init(void);                        /* 上电：设置门限，缺相窗口清零 */
void Protect_BusSlow(void);       /* 慢环每拍调用，见上 */

void Protect_PhaseReset(void);    /* 缺相统计清零（开始新窗口） */
void Protect_PhaseFast(void);    /* 快环每拍：三相 |i| 各自累加 */
uint8_t Protect_PhaseSlow(int16_t speedQ15);    /* 1 = 缺相 */
uint8_t Protect_OverSpeed(int16_t speedQ15);    /* 1 = 超速 */

#endif
