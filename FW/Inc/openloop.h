/* ============================================================================
 * @file    openloop.h
 * @brief   I/F 开环（Run 子状态 OpenLoop），实现见 openloop.c；起动流程总览见 startup.h
 * ==========================================================================*/
#ifndef OPENLOOP_H
#define OPENLOOP_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

void OpenLoop_Init(void);       /* 上电：飞车接管进入 I/F 的入口参数 */
void OpenLoop_LoadGains(void);  /* 从 gGain 重新装载飞车接管开环加速度步长（param.c 更新参数时调用） */
void OpenLoop_Fast(void);       /* I/F 快环：开环角积分、电流环按开环角运行，观测器只跟踪 */
void OpenLoop_Slow(void);       /* I/F 慢环：开环电流爬升、缺相检测、一致性判断 -> 切闭环 */
/* warm = 0：冷起动（IPD / SWING 之后，从当前 gFoc.theta 开始）；1：从 Spin 回退（从 PLL 角和转速开始） */
/* 进入 I/F 分两步："准备"在临界区外（只计算、不修改状态），"提交"在临界区内（只赋值），见 startup.c */
typedef struct {
    uint8_t  warm;    /* 1 = 从运行状态接管（回退 / 飞车），0 = 冷起动（IPD / SWING 后） */
    uint32_t thetaOffset;       /* 开环角 - 估计角（Q31 角度），提交时叠加到当时的 PLL 角上 */
    int32_t  olSpeed;           /* 开环起始转速 Q31 */
    int32_t  olCur, olCurTarget;    /* 开环电流起始值 / 目标值（Q15），起始值 < 目标值时按 CURRENT_SLEW_STEP_Q15_OF 上升 */
    int32_t  accScaleQ12, accStepFull, accStep;    /* 加速度缩放（开环电流 / 额定开环电流，Q12）、电流到目标后的加速度步长、起始加速度步长（Q31 / 快环拍） */
} OpenLoopEntry_TypeDef;
void OpenLoop_PrepareCold(OpenLoopEntry_TypeDef *e);    /* 准备（临界区外，只计算）：冷起动（IPD 之后） */
void OpenLoop_PrepareSwing(int16_t speedQ15, OpenLoopEntry_TypeDef *e);    /* 准备：摆动中起动（转子正以 speedQ15 经过 gFoc.theta） */
void OpenLoop_PrepareWarm(int16_t iqNow, uint8_t revCur, uint8_t revAcc, int16_t speedQ15, OpenLoopEntry_TypeDef *e);    /* 准备：从运行状态接管 */
void OpenLoop_Commit(const OpenLoopEntry_TypeDef *e);    /* 提交（临界区内，只赋值） */
uint8_t OpenLoop_Reversal(int32_t speedSign);    /* 开环目标方向与 speedSign 相反（I/F 需过零）时返回 1 */
/* 飞车接管进入 I/F（快环中调用，常数时间）：使用上电时预先计算的入口参数（iq = 0）+ 接管转速 */
void OpenLoop_CommitCatch(int16_t spd);

/* 当前开环转速（Q15 速度），用于相位补偿 */
int16_t OpenLoop_SpeedQ15(void);
/* 冷起动失速自适应档位（0 = 正常；冷起动 I/F 堵转一次加一档，最多 3 档：电流逐档加大、加速度逐档减半；上电清零）：诊断用 */
uint8_t OpenLoop_BoostLevel(void);
/* Spin 慢环调用：判断是否应交回 I/F（|转速| < 回退转速（gGain.spdFallback） 且指令反向或低于 闭环最低转速（gGain.spdSpinMin）） */
uint8_t OpenLoop_WantsFallback(void);

#endif
