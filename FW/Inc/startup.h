/* ============================================================================
 * startup.h —— 启动流程各子状态的处理函数
 *
 *   Charge 20 ms（自举预充）
 *   -> Calib 100 ms（MCF_FlyStart_2R 短接下桥测电流，判断转子是否在转）
 *        ├─ 在转 -> Torqueloop 0.3 s（按测到的角度/转速直接力矩闭环）-> Spin
 *        └─ 静止 -> Align -> Startup -> Spin，按 STARTUP_MODE 分两种：
 *
 *   STARTUP_VENDOR（原厂 Fast Boot，test33）：
 *     Align 1 ms（Iq = IQ_STARTUP/4，角度 0）-> Startup（Iq 0.5 A，观测器+PLL 自由
 *     运行，PLL 12.8 rpm 下限推着走）-> Fast_Boot_StartUp 判 80 rpm -> Spin
 *
 *   STARTUP_IF（开环拖动）：
 *     Align：d 轴电流斜坡到 IF_ALIGN_A，在 0° 等转子停稳（IF_ALIGN_STAGES = 2 时先在 120°
 *       再慢慢转到 0°，避开死点）；
 *       定位期间按 q 轴反电势加阻尼电流（转子是个无阻尼单摆）
 *     Startup：电流 IF_CURRENT_A 沿开环角方向（与定位电流连续），开环角按
 *       IF_ACC_RPM_S 加速到 IF_HANDOVER_RPM 并保持；观测器+PLL 同时跟踪（不参与控制）。
 *       PLL 转速（滤波）与开环转速相差 < IF_AGREE_RATIO 持续 IF_AGREE_S
 *       -> 转矩不变切换：iq_new = I * sin(theta_ol - theta_est)，id = 0 -> Spin
 *       IF_TIMEOUT_S 内不一致 -> 堵转故障
 *     Spin 回退（转速掉到约 40 rpm 或反向）时，从当前估计角和转速接着开环拖。
 *
 * 子状态之间的切换函数在 supervisor.c（Sup_Switch*），这里的处理函数在
 * 条件满足时调用它们，调用顺序与 test33 完全一致。
 *
 * 待扩展（P4 阶段）：
 *   IPD 初始位置检测 + I/f 拖动 + 磁链接管 —— 零速带载启动
 *   HFI 高频注入（需要凸极，IPM 才有意义）
 * ==========================================================================*/
#ifndef STARTUP_H
#define STARTUP_H
#include "fw_config.h"
#include "mc_types.h"

#if (FAST_BOOT_METHOD != 1)
#error "只支持原厂快速启动（FAST_BOOT_METHOD = 1）。原厂的 0 分支在 test33 里本来就是半截注释掉的，已删除。"
#endif

/* 启动各子状态的计数和参数（原 USER / FastBoot_Para 里的启动部分，整数类型保持原样） */
typedef struct {
    uint16_t preChargeCmd;      /* 预充时长（快环拍） */
    uint16_t preChargeCnt;      /* 预充计时 */
    uint8_t  preChargePhase;    /* 预充轮流开 A/B/C 下桥：0/1/2 */
    uint16_t alignTimeCmd;      /* 原厂定位时长（慢环拍） */
    uint16_t alignTimeCnt;
    uint16_t alignCurAcc;       /* 原厂启动电流斜坡（每慢环拍） */
    int16_t  iqCmd;             /* 原厂启动 iq Q15 */
    uint16_t spdMax;            /* 原厂启动斜坡的转速上限 Q15 */
    uint16_t timeCmd;           /* 启动超时（慢环拍） */
    uint16_t timeCnt;           /* 原厂 Startup->Spin 判据的轮询计数 */
    uint16_t tqTimeCmd;         /* Torqueloop 时长（慢环拍） */
    uint16_t tqTimeCnt;
    int16_t  detSpd;            /* Startup->Spin 的转速门限 Q15（STARTUP_TO_SPIN_SPEED）；Spin 回退用它的一半 */
    uint16_t detTime;           /* 上面判据的轮询间隔（慢环拍） */
} Startup_TypeDef;

extern Startup_TypeDef gStartup;

void Startup_Init(void);

/* 进入 Calib 时清飞车检测计数 */
void Startup_ResetFlyCheck(void);
/* Startup->Spin 时清堵转计时 */
void Startup_ResetBlockTimer(void);

/* ---- 子状态处理：Fast = ADC 中断里每拍，Slow = 主循环每 1 ms ---- */
void Startup_ChargeFast(void);
void Startup_ChargeSlow(void);
void Startup_CalibFast(void);
void Startup_CalibSlow(void);
void Startup_AlignFast(void);
void Startup_AlignSlow(void);
void Startup_StartupFast(void);
void Startup_StartupSlow(void);
void Startup_TorqueloopFast(void);
void Startup_TorqueloopSlow(void);

/* 子状态切换时由 supervisor 调用（原厂模式下是空函数） */
void Startup_OnEnterAlign(void);
/* warm = 0：定位后冷启动（从当前 gFoc.theta、零速开始）；1：从 Spin 回退（从 PLL 角和转速开始） */
void Startup_OnEnterIF(uint8_t warm);

#if (STARTUP_MODE == STARTUP_IF)
/* 当前开环转速（Q15 速度），相位补偿用 */
int16_t Startup_IfSpeedQ15(void);
/* Spin 慢环调用：是否该交回 I/F（|转速| < IF_FALLBACK_RPM 且指令反向或低于 IF_SPIN_MIN_RPM） */
uint8_t Startup_IfWantsFallback(void);

typedef struct {
    uint16_t stage;         /* 1 定位（一段时只有这一步；两段时为 120°），2 两段时转到 0° 并等停稳，3 开环加速/保持，5 已切闭环，6 超时故障 */
    int16_t  olSpeed;       /* 开环转速 Q15 */
    int16_t  pllFilt;       /* PLL 转速（滤波）Q15 */
    uint16_t agreeCnt;      /* 一致计数（慢环拍） */
    int16_t  deltaAtHandover;   /* 切换时 theta_ol - theta_est（Q15 角度，65536 = 360°） */
    int16_t  iqAtHandover;      /* 切换后 iq 指令 Q15 */
    uint16_t handovers, timeouts;
    uint16_t alignTime;         /* 上次定位总用时（ms） */
} StartupIfDiag_TypeDef;
extern volatile StartupIfDiag_TypeDef gStartupIF;
#endif

#endif
