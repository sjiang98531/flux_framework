/* ============================================================================
 * @file    startup.h
 * @brief   起动流程各子状态的处理函数
 *
 *   Charge 20 ms（自举预充）
 *   -> Calib（飞车检测：短脉冲 / 短接测转速、转向）
 *        ├─ 正在转动且转速足够 -> Spin（零电流同步后闭环）；低速 -> 直接进入 I/F
 *        └─ 静止 -> Align -> Startup（I/F）-> Spin
 *
 *   I/F（开环拖动）：
 *     Align 子状态首先执行 IPD（STARTUP_IPD，ipd.h）：以 12 个方向的电压脉冲确定 N 极角，直接从该角进入 Startup，
 *       电流从 1/4 起按 CURRENT_SLEW_A_S 升至 OPENLOOP_CURRENT_A（加速度同比例）；对比度不足时退回定位。
 *     Align：d 轴电流斜坡升至 ALIGN_A，在 0° 等待转子停稳；
 *       定位期间按 q 轴反电势施加阻尼电流（转子等效为无阻尼单摆）
 *     Startup：电流 OPENLOOP_CURRENT_A 沿开环角方向（与定位电流连续），开环角按
 *       OPENLOOP_ACC_RPM_S 加速到 OPENLOOP_HANDOVER_RPM 并保持；观测器 + PLL 同步跟踪（不参与控制）。
 *       PLL 转速（滤波）与开环转速相差 < OPENLOOP_AGREE_RATIO 并持续 OPENLOOP_AGREE_S
 *       -> 转矩不变切换：iq_new = I * sin(theta_ol - theta_est)，id = 0 -> Spin
 *       OPENLOOP_TIMEOUT_S 内仍不一致 -> 堵转故障
 *     Spin 回退（指令反向或低于闭环最低转速、减速到 OPENLOOP_FALLBACK_RPM）时，从当前估计角和转速继续开环拖动。
 *
 * 子状态之间的切换函数位于 supervisor.c（Sup_Switch*），本模块的处理函数在条件满足时调用它们。
 * ==========================================================================*/
#ifndef STARTUP_H
#define STARTUP_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"


/* 起动各子状态的计数与参数 */
typedef struct {
    uint16_t preChargeCmd;      /* 预充时长（快环拍） */
    uint16_t preChargeCnt;      /* 预充计时 */
    uint8_t  preChargePhase;    /* 预充时轮流导通 A/B/C 下桥：0/1/2 */
} Startup_TypeDef;

extern Startup_TypeDef gStartup;

void Startup_Init(void);    /* 上电：预充时间、飞车 / 定位的各段时间、飞车接管进入 I/F 的入口参数 */
/* 停机 / 故障复位：预充计数清零 */
void Startup_ResetRun(void);

/* ---- 子状态处理：Fast = 快环（ADC 中断）每拍，Slow = 慢环（PendSV）每拍（1 kHz） ---- */
void Startup_ChargeFast(void);    /* 预充：三相下桥轮流导通为自举电容充电，到时进入飞车检测 */
void Startup_ChargeSlow(void);    /* 预充：仅滤波 */
void Startup_CalibFast(void);    /* 飞车检测：执行检测，按结论切换状态 */
void Startup_CalibSlow(void);    /* 飞车检测：滤波、诊断量开方 */
void Startup_AlignFast(void);    /* IPD 输出脉冲 / 定位运行电流环 */
void Startup_AlignSlow(void);    /* IPD 判定 / 定位电流斜坡 + 阻尼 + 判停稳 */
void Startup_StartupFast(void);    /* I/F：开环角积分、电流环按开环角运行，观测器只跟踪 */
void Startup_StartupSlow(void);    /* I/F：开环电流爬升、缺相检测、一致性判断 -> 切闭环 */

/* 子状态切换时由 supervisor 调用 */
void Startup_OnEnterAlign(void);
/* warm = 0：定位后冷起动（从当前 gFoc.theta、零速开始）；1：从 Spin 回退（从 PLL 角和转速开始） */
/* 进入 I/F 分两步："准备"在临界区外（只计算、不修改状态），"提交"在临界区内（只赋值），见 startup.c */
typedef struct {
    uint8_t  warm;    /* 1 = 从运行状态接管（回退 / 飞车），0 = 冷起动（定位后） */
    uint32_t thetaOffset;       /* 开环角 - 估计角（Q31 角度），提交时叠加到当时的 PLL 角上 */
    int32_t  olSpeed;           /* 开环起始转速 Q31 */
    int32_t  olCur, olCurTarget;    /* 开环电流起始值 / 目标值（Q15），起始值 < 目标值时按 CURRENT_SLEW_A_S 上升 */
    int32_t  accScaleQ12, accStepFull, accStep;    /* 加速度缩放（开环电流 / 额定开环电流，Q12）、电流到目标后的加速度步长、起始加速度步长（Q31 / 快环拍） */
} OpenLoopEntry_TypeDef;
void Startup_PrepareOpenLoop(uint8_t warm, int16_t iqNow, uint8_t revCur, uint8_t revAcc, int16_t speedQ15, OpenLoopEntry_TypeDef *e);    /* 准备（临界区外，只计算） */
void Startup_CommitOpenLoop(const OpenLoopEntry_TypeDef *e);    /* 提交（临界区内，只赋值） */
uint8_t Startup_OpenLoopReversal(int32_t speedSign);    /* 开环目标方向与 speedSign 相反（I/F 需过零）时返回 1 */
/* 飞车接管进入 I/F（快环中调用，常数时间）：使用上电时预先计算的入口参数（iq = 0）+ 接管转速 */
void Startup_CommitCatchOpenLoop(int16_t spd);

/* 当前开环转速（Q15 速度），用于相位补偿 */
int16_t Startup_OpenLoopSpeedQ15(void);
/* Spin 慢环调用：判断是否应交回 I/F（|转速| < OPENLOOP_FALLBACK_RPM 且指令反向或低于 OPENLOOP_SPIN_MIN_RPM） */
uint8_t Startup_OpenLoopWantsFallback(void);
/* 回退 I/F 时的开环电流（Q15）：max(OPENLOOP_CURRENT_A, |iq|/sin(OPENLOOP_WARM_DELTA_DEG))，反转时至少 OPENLOOP_REV_CURRENT_A，上限 OPENLOOP_WARM_MAX_A */
int16_t Startup_OpenLoopWarmCurrent(int16_t iq, uint8_t reversal);

/* gOpenLoopDiag.stage 的取值（数值沿用历史台架记录） */
enum {
    OPENLOOP_STAGE_ALIGN    = 1,    /* 定位 */
    OPENLOOP_STAGE_RUN      = 3,    /* 开环加速 / 保持 */
    OPENLOOP_STAGE_HANDOVER = 5,    /* 已切闭环 */
    OPENLOOP_STAGE_TIMEOUT  = 6,    /* 超时故障 */
    OPENLOOP_STAGE_IPD      = 7     /* IPD */
};
typedef struct {
    uint16_t stage;         /* 7 IPD，1 定位，3 开环加速 / 保持，5 已切闭环，6 超时故障（2 保留未用） */
    int16_t  olSpeed;       /* 开环转速 Q15 */
    int16_t  pllFilt;       /* PLL 转速（滤波）Q15 */
    uint16_t agreeCnt;      /* 一致计数（慢环拍） */
    int16_t  deltaAtHandover;   /* 切换时 theta_ol - theta_est（Q15 角度，65536 = 360°） */
    int16_t  iqAtHandover;      /* 切换后 iq 指令 Q15 */
    uint16_t handovers, timeouts;    /* 上电以来切闭环次数、I/F 超时次数 */
    uint16_t alignTime;         /* 上次定位总用时（ms） */
} OpenLoopDiag_TypeDef;
extern volatile OpenLoopDiag_TypeDef gOpenLoopDiag;

#endif
