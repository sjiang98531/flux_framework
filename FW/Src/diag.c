/* ============================================================================
 * @file    diag.c
 * @brief   诊断：慢环健康计数、观测器置信度、配置检查结果、版本号
 *
 * 结果全部写入 gFluxDiag，供调试器观察，不参与控制（配置检查失败除外：faultDetect 据此报故障）。
 * ==========================================================================*/
#include "param.h"
#include "diag.h"
#include "cfg_derived.h"
#include "cfg_check.h"         /* 配置的编译期检查（cfg_check_list.h） */
#include "flux_obs.h"
#include "foc_core.h"
#include "supervisor.h"
#include "compiler.h"

volatile FluxDiag_TypeDef gFluxDiag;
/* 统计窗口（DIAG_WINDOW_S）：每个窗口结束后计算一次置信度（含一次除法，不在每拍执行） */
#define DIAG_WINDOW_PWM         ((uint32_t)(DIAG_WINDOW_S * PWM_FREQ + 0.5))
static uint32_t s_winSamp0;
static uint8_t  s_winHave, s_winPhase;

/* 清除所有最大值 / 最小值类诊断量（上电时调用；也可在调试器中调用以重新统计） */
void Diag_ResetMax(void)
{
    gFluxDiag.slowGapMax = 0; gFluxDiag.slowDurMax = 0;
    gFluxDiag.slowOverrun = 0;
}

/* 上电：写入版本号，计数清零；汇总配置警告位（Ld != Lq 但按表贴式配置、增益封顶等）；
 * 遍历编译期配置检查表，记录第一条未通过项的行号及未通过项数（不为 0 时电机不起动） */
void Diag_Init(void)
{
    unsigned i;

    gFluxDiag.buildTag = BUILD_TAG;
    gFluxDiag.sampleCount = 0;
    gFluxDiag.inputSatCount = 0; gFluxDiag.fluxSatCount = 0;
    gFluxDiag.confidence = 0;
    gFluxDiag.slowGap = gFluxDiag.slowGapMax = gFluxDiag.slowDur = gFluxDiag.slowDurMax = 0;
    gFluxDiag.cfgWarn = 0;
    Diag_ResetMax();
#if !MOTOR_IPM
    if (MOTOR_LD_H != MOTOR_LQ_H) gFluxDiag.cfgWarn |= 1u;
#endif
    /* bit1 电流环增益封顶，bit2 PLL 增益超出 int16，bit3 反电势前馈超量程：按当前参数计算（param.c computeGain） */
    gFluxDiag.cfgWarn |= gGain.cfgWarn;
    gFluxDiag.cfgFailLine = 0;
    gFluxDiag.cfgFailCount = 0;
    for (i = 0; i < sizeof s_fwCfgCheck / sizeof s_fwCfgCheck[0]; i++) {
        if (s_fwCfgCheck[i]) {
            if (gFluxDiag.cfgFailLine == 0) gFluxDiag.cfgFailLine = s_fwCfgCheck[i];
            gFluxDiag.cfgFailCount++;
        }
    }
}

/* 慢环开始时调用：统计窗口按快环拍数划分（与慢环频率、慢环超时无关），窗口结束时标记本拍计算置信度（Diag_SlowTick） */
void Diag_SlowBegin(void)
{
    uint32_t samp = gFluxDiag.sampleCount;
    if (!s_winHave || samp - s_winSamp0 >= DIAG_WINDOW_PWM) {
        if (s_winHave) s_winPhase = 4;
        s_winHave = 1; s_winSamp0 = samp;
    }
}

/* 观测器置信度（每个统计窗口一次）：1 - | |psi_r| - 目标 | / 目标，Q15。磁链幅值偏离标称越多越低 */
void Diag_SlowTick(void)
{
    int32_t mag = gFluxObs.s16PsiRMag;
    int32_t tgt = FluxObs_TargetMag();
    int32_t dev, c;
    if (s_winPhase != 4) return;        /* 每个统计窗口执行一次 */
    s_winPhase = 0;
    if (mag == 0 || tgt <= 0) { gFluxDiag.confidence = 0; return; }
    dev = mag - tgt;
    if (dev < 0) dev = -dev;
    c = 32767 - (int32_t)(((int32_t)dev * 32767) / tgt);   /* 慢环，允许除法 */
    gFluxDiag.confidence = (int16_t)(c < 0 ? 0 : c);
}
