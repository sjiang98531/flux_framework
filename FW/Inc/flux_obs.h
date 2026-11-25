/* ============================================================================
 * @file    flux_obs.h
 * @brief   转子磁链观测器（电压模型 + 去直流 + NLFO 幅值约束）
 *
 *   e     = v - Rs*i_avg            v：上一拍调制器 αβ 指令 × 本拍母线；i_avg：前后两拍电流平均
 *   e    -= MA(e)                   高通，去除零点 / 电压误差引起的直流分量（tau ≈ 0.256 s）
 *   psi_s += 2*(e*KTs + corr)       corr = psi_r*(psi_f^2-|psi_r|^2)，纯径向修正
 *   psi_r = psi_s - Lq*i            -> 直接用于 FluxPLL 鉴相
 *
 * 凸极（Ld != Lq）：psi_s - Lq*i 为"有效磁链"（active flux），方向仍为转子 d 轴，
 * 幅值 = psi_f + (Ld-Lq)*id，因此 NLFO 幅值目标按实测 id 变化（FluxObs_SlowTick）。
 * Ld = Lq（表贴式）时目标恒为 psi_f。
 *
 * 仅使用测量量与上一拍 PWM 指令。积分对象为定子磁链 psi_s，Lq*i 每拍实时计算，
 * 因此舍入误差和电感误差不会在积分器中累积。
 * ==========================================================================*/
#ifndef FLUX_OBS_H
#define FLUX_OBS_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

typedef struct {
    int16_t s16KRs, s16KLq, s16KTs;          /* 模型常数 Q15（KLs 即 Lq） */
    int32_t s32PsiAlF, s32PsiBeF;            /* 定子磁链 Q16.16 */
    LowPass_TypeDef sEalAvgMA, sEbeAvgMA;   /* e 的去直流 */
    int16_t s16PsiAl, s16PsiBe;              /* 定子磁链 Q15 */
    int16_t s16PsiRAl, s16PsiRBe;            /* 转子磁链 Q15 */
    int16_t previousIa, previousIb;          /* 上一拍 αβ 电流（求两拍平均） */
    uint8_t haveCurrent;                     /* previousIa/Ib 有效 */
} FluxObs_TypeDef;

extern FluxObs_TypeDef gFluxObs;

void FluxObs_Init(void);            /* 上电 */
void FluxObs_LoadModel(void);       /* 从 gGain 重新装载 Rs / L / NLFO 归一化（param.c 更新参数时调用） */
void FluxObs_Reset(void);           /* 清除全部状态（非 Run 状态时每拍调用） */
/* Align -> OpenLoop：清除观测器、PLL 速度与转速反馈，将 psi 初始化为 |psi_f|∠gFoc.theta */
void FluxObs_BeginOpenLoop(void);
/* 飞车接管：清除观测器，将定子磁链初始化为 psi_f∠theta + Lq*i（theta、i 均为本拍采样时刻） */
void FluxObs_SeedCatch(int16_t theta);
/* 执行本拍模型。调用前须已计算 gMeas.iab（Clarke），且本拍尚未调制（gSvpwm.Alpha/Beta 仍为上一拍指令）。 */
void FluxObs_Run(void);
/* 慢环每拍：按转速更新去直流相位补偿，按母线平均值更新死区电压误差幅值，凸极时按实测 id 更新 NLFO 幅值目标 */
void FluxObs_SlowTick(int16_t spdQ15, int16_t busAvg);

/* ============================================================================
 * 转子磁链锁相环（FluxPLL）与无感角度估计调度（gEst）
 * ==========================================================================*/
void FluxPLL_Init(PLL_TypeDef *p);
void FluxPLL_LoadGains(PLL_TypeDef *p);    /* 仅重新装载 Kp / Ki（param.c 更新参数时调用） */
int16_t FluxPLL_PhaseError(int16_t psiAlpha, int16_t psiBeta, int16_t theta);
int16_t FluxPLL_StepQ15(const PLL_TypeDef *p, int32_t advQ8);
int16_t FluxPLL_Run(int16_t err, PLL_TypeDef *p);

typedef struct {
    PLL_TypeDef pll;            /* 磁链 PLL */
    int16_t     thetaPll;       /* PLL 输出角（已推进到下一拍） */
} Est_TypeDef;

extern Est_TypeDef gEst;

int16_t Est_TrackTick(void);                /* 观测器 + PLL 跟踪一拍（I/F 拖动期间，返回 k 时刻角） */
void    Est_SpinTick(void);                 /* Spin 闭环每拍：跟踪 + 反 Park 超前外推 + 写入转速反馈 */
void    Est_OnEnterOpenLoopCold(void);      /* 进入 I/F 冷起动（IPD / 摆动）：观测器与 PLL 对齐当前角 */
void    Est_OnCatch(int16_t theta, int16_t spd); /* 飞车直接接管进入 Spin */
void    Est_ResetRun(void);                 /* 停机 / 故障复位 */

#endif
