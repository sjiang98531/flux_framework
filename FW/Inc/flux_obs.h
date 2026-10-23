/* ============================================================================
 * flux_obs.h —— 转子磁链观测器（电压模型 + 去直流 + NLFO 幅值约束）
 *
 *   e     = v - Rs*i_avg            v：上一拍 PWM 比较值 × 母线重建；i_avg：前后两拍平均
 *   e    -= MA(e)                   高通，去掉零点/电压误差带来的直流（tau ≈ 0.256 s）
 *   psi_s += 2*(e*KTs + corr)       corr = psi_r*(psi_f^2-|psi_r|^2)，纯径向
 *   psi_r = psi_s - Lq*i            -> FluxPLL 直接拿它鉴相
 *
 * 内嵌式（MOTOR_IPM = 1）：psi_s - Lq*i 是"有效磁链"（active flux），方向仍是
 * 转子 d 轴，幅值 = psi_f + (Ld-Lq)*id，所以 NLFO 的幅值目标随 id 变化
 * （FluxObs_SetIdRef，慢环调用）。表贴式 Ld = Lq，目标恒为 psi_f。
 *
 * 只用测量量和上一拍 PWM，不依赖任何 SMO 量。积分的是定子磁链 psi_s，
 * Lq*i 每拍现算，所以舍入误差和电感误差都不会被积分器累积。
 * ==========================================================================*/
#ifndef FLUX_OBS_H
#define FLUX_OBS_H
#include "fw_config.h"
#include "vendor_lib.h"

/* 快环入口抓的输入快照（本拍算新比较值之前） */
typedef struct {
    int16_t ia, ib, ic, bus;
    int16_t compareA, compareB, compareC;
    uint16_t period;
    uint8_t pwmValid, pwmMode;
} FluxInput_TypeDef;

typedef struct {
    int16_t s16KRs, s16KLs, s16KTs;          /* 模型常数 Q15（KLs 即 Lq） */
    int32_t s32PsiAlF, s32PsiBeF;            /* 定子磁链 Q16.16 */
    MLIB_FILTER_MA_T sEalAvgMA, sEbeAvgMA;   /* e 的去直流 */
    int16_t s16PsiAl, s16PsiBe;              /* 定子磁链 Q15 */
    int16_t s16PsiRAl, s16PsiRBe, s16PsiRMag;/* 转子磁链 Q15（RMag 每 16 拍算一次，诊断用） */
    int16_t s16Eal, s16Ebe;                  /* v - Rs*i（高通之前，J-Scope 用） */
    int16_t previousIa, previousIb;
    uint8_t u8DivCnt, haveCurrent;
} FluxObs_TypeDef;

extern FluxObs_TypeDef sFluxObs;
extern FluxInput_TypeDef gFluxInput;

void FluxObs_Init(void);
void FluxObs_Reset(void);
/* Align -> Startup：清观测器和 PLL 速度，把 psi 种在 |psi_f|∠s16CmdTheta 上 */
void FluxObs_BeginStartup(FOC_TypeDef *f, MLIB_FILTER_MA_T *speedFilter);
/* 快环入口：抓上一拍的 PWM 比较值、电流、母线 */
void FluxObs_Capture(const FOC_TypeDef *f, uint8_t valid);
/* 跑本拍模型。调用前必须已算好 f->sIAlBe（Clarke）。 */
void FluxObs_UpdateFeedback(FOC_TypeDef *f);
/* 当前 |psi_r| 的目标值（Q15 磁链），诊断/置信度用 */
int16_t FluxObs_TargetMag(void);
/* 按 PLL 估计转速更新去直流的相位补偿（慢环调用；CTRL_OBS_HPF_COMP = 0 时空函数） */
void FluxObs_SetSpeed(int16_t spdQ15);
#if MOTOR_IPM
/* 内嵌式：按 d 轴电流指令更新有效磁链幅值目标（慢环调用，含除法） */
void FluxObs_SetIdRef(int16_t idQ15);
#endif

#endif
