/* ============================================================================
 * @file    diag.h
 * @brief   诊断量（只写不读，不参与控制；供调试器观察）
 * ==========================================================================*/
#ifndef DIAG_H
#define DIAG_H
#include <stdint.h>

typedef struct {
    uint32_t sampleCount;              /* 快环总拍数 */
    uint16_t mainState, runState;      /* gSup.mainState / runState 的拷贝 */
    int16_t  voltageAlpha, voltageBeta;/* 观测器用的重构电压（Q15，475 V 基准） */
    uint32_t inputSatCount, fluxSatCount;  /* 输入电压 / 磁链饱和计数 */
    uint32_t buildTag;                 /* = BUILD_TAG */
    /* ---- 配置检查 ---- */
    uint16_t cfgWarn;                 /* 配置检查：bit0 = MOTOR_IPM=0 但 Ld != Lq；bit1 = 电流环增益超 65535 被封顶；
                                        * bit2 = PLL 增益/角度增益超 int16 被封顶（PLL 带宽太高或 psi_f 太小）；
                                        * bit3 = SPEED_MAX 下反电势超过电压基准（DECOUPLE_K_EMF 饱和） */
    uint16_t cfgFailLine;              /* 配置检查（cfg_check.h）：第一个不满足项在 cfg_check_list.h 的行号，0 = 全部通过 */
    uint16_t cfgFailCount;             /* 不满足的项数；非 0 时报 MC_FAULT_CFG，电机不起动 */
    /* ---- 慢环健康与观测器置信度 ---- */
    uint16_t slowGap, slowGapMax;     /* 相邻两次慢环起点的间隔（快环拍数；名义 = PWM_FREQ/SLOWLOOP_FREQ = 16） */
    uint16_t slowDur, slowDurMax;      /* 一次慢环从开始到结束的墙钟时间（快环拍数，含被快环抢占的时间） */
    int16_t  confidence;               /* 观测器置信度 Q15：1 - | |psi_r| - 目标 | / 目标（每个统计窗口算一次） */
    uint16_t slowOverrun;              /* 慢环超时次数（饱和于 65535）：到期时上一次慢环尚未完成（PendSV 挂起位只有一个，本次丢弃） */
} FluxDiag_TypeDef;

extern volatile FluxDiag_TypeDef gFluxDiag;

void Diag_Init(void);    /* 上电：写入版本号、计数清零、记录配置检查结果 */
/* 慢环调用：更新置信度等慢变量 */
void Diag_SlowTick(void);
/* 慢环开始时调用：划分统计窗口（置信度每个窗口计算一次） */
void Diag_SlowBegin(void);
/* 清零所有最大值统计（可在调试器中调用；上电时已清零） */
void Diag_ResetMax(void);

#endif
