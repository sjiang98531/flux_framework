/* ============================================================================
 * @file    startup.h
 * @brief   起动流程各子状态的处理函数
 *
 *   Charge 20 ms（自举预充）
 *   -> FlyCheck（飞车检测：短脉冲 / 短接测转速、转向）
 *        ├─ 正在转动且转速足够 -> Spin（零电流同步后闭环）；低速 -> 直接进入 I/F
 *        └─ 静止 -> Align -> OpenLoop（I/F）-> Spin
 *
 *   I/F（开环拖动）：
 *     Align 子状态按起动方式 gStartup.mode（上电默认 STARTUP_MODE，cfg_app.h）二选一：
 *     IPD（STARTUP_MODE_IPD，ipd.h）：高频注入测凸极轴向、直流偏置判极性，确定 N 极角后直接从该角进入 OpenLoop，
 *       电流从 1/4 起按 CURRENT_SLEW_A_S 升至 OPENLOOP_CURRENT_A（加速度同比例）；对比度不足时本次自动退回摆动中起动（SWING）。
 *     摆动中起动（STARTUP_MODE_SWING，静音起动）：d 轴电流斜坡升至 ALIGN_A（同时电流矢量扫到定位轴），不加阻尼、不等停稳；
 *       转子顺转向摆过定位轴（q 轴反电势达到顺向峰值）的瞬间，开环角与开环转速接到转子上，负载角平滑建立后连续加速。
 *       （参数辨识 IDENT_TEST = 1 静止测量时仍先阻尼定位并等待停稳）
 *     OpenLoop：电流 OPENLOOP_CURRENT_A 沿开环角方向，开环角按
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
    volatile uint8_t mode;      /* 起动方式 STARTUP_MODE_IPD / STARTUP_MODE_SWING（调试器或上位机可改，下次起动生效；其他值按摆动中起动处理） */
} Startup_TypeDef;

extern Startup_TypeDef gStartup;

void Startup_Init(void);    /* 上电：预充时间、起动方式默认值、装载定位与摆动参数 */
void Startup_LoadGains(void);   /* 从 gGain 重新装载摆动参数与辨识定位参数（param.c 更新参数时调用） */
/* 停机 / 故障复位：预充计数清零 */
void Startup_ResetRun(void);

/* ---- 子状态处理：Fast = 快环（ADC 中断）每拍，Slow = 慢环（PendSV）每拍（1 kHz） ---- */
void Startup_ChargeFast(void);    /* 预充：三相下桥轮流导通为自举电容充电，到时进入飞车检测 */
void Startup_ChargeSlow(void);    /* 预充：仅滤波 */
void Startup_FlyCheckFast(void);    /* 飞车检测：执行检测，按结论切换状态 */
void Startup_FlyCheckSlow(void);    /* 飞车检测：测量滤波 */
void Startup_AlignFast(void);    /* IPD 输出电压 / 摆动起动或辨识定位运行电流环 */
void Startup_AlignSlow(void);    /* IPD 推进与判定 / 摆动起动 / 辨识定位（IDENT_TEST = 1）斜坡与阻尼 */

/* 子状态切换时由 supervisor 调用 */
void Startup_OnEnterAlign(void);

#endif
