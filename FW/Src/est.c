/* ============================================================================
 * @file    est.c
 * @brief   角度 / 转速估计调度
 *
 * 串联磁链观测器（flux_obs.c）与 PLL（flux_pll.c），结果写入电流环（gFoc.theta / thetaInv）
 * 和转速反馈（Meas_SpeedSource）。同时负责估计器的重新初始化：冷起动、停机复位、飞车接管。
 * ==========================================================================*/
#include "est.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "meas.h"
#include "foc_core.h"
#include "compiler.h"

Est_TypeDef gEst;

/* 观测器 + 运行期 PLL 跟踪一拍。返回 PLL 在本拍采样时刻 k 的角度（即鉴相所用角度）；
 * 返回时 PLL 已推进到 k+1（gEst.thetaPll）。k 时刻角取自 s32Theta（此时 thetaPll 已推进）。 */
RAMFUNC int16_t Est_TrackTick(void)
{
    int16_t thK = (int16_t)(gEst.pll.s32Theta >> 16), err;
    FluxObs_Run();
    err = FluxPLL_PhaseError(gFluxObs.s16PsiRAl, gFluxObs.s16PsiRBe, thK);
    gEst.thetaPll = FluxPLL_Run(err, &gEst.pll);
    return thK;
}

/* 闭环（Spin）每拍：跟踪一拍，Park 使用本拍采样时刻的角，反 Park 使用外推 INVPARK_ADV_Q8 拍后的角
 * （补偿电压延迟一拍生效），PLL 转速作为转速反馈 */
RAMFUNC void Est_SpinTick(void)
{
    /* Park 用 k 时刻角；反 Park 用 k + CTRL_INVPARK_ADVANCE 拍（电压在下一个 PWM 周期生效） */
    int16_t thK = Est_TrackTick();
    gFoc.theta = thK;
    gFoc.thetaInv = gFoc.theta + FluxPLL_StepQ15(&gEst.pll, INVPARK_ADV_Q8);
    Meas_SpeedSource((int16_t)(gEst.pll.s32Speed >> 16));
}

void Est_OnEnterOpenLoopCold(void)
{
    /* 冷起动（定位 / IPD -> I/F）：此前的飞车检测也使用同一个 gEst.pll，其残留速度不能带入新的估计。
     * PLL 角 = 起动角（第一拍 PLL 误差为 0），转速与积分器清零；观测器复位并将磁链初始化为 |psi_f|∠theta。
     * 调用方已对齐的角度、PLL 增益及所有 PI 保持不变。 */
    gEst.pll.s32Theta = gFoc.theta << 16;
    gEst.pll.s32Speed = 0; gEst.pll.s32I_1 = 0;
    Meas_SpeedReset();
    FluxObs_BeginOpenLoop();
}

/* 停机复位：PLL 转速与积分器清零（角度保留，下次起动时重新设置） */
void Est_ResetRun(void)
{
    gEst.pll.s32I_1 = 0;
    gEst.pll.s32Speed = 0;
}

/* 飞车接管：按测得的转子角 theta 和转速 spd 预置 PLL（角度、转速、积分器）、观测器磁链、转速反馈与滤波器，
 * 电流环角度也直接使用该值，使开启 PWM 的第一拍估计器即与转子同步 */
void Est_OnCatch(int16_t theta, int16_t spd)
{
    int32_t sp = (int32_t)spd << 16;
    FluxPLL_Init(&gEst.pll);                    /* 装载增益并清除状态 */
    gEst.pll.s32Speed = sp;
    gEst.pll.s32I_1 = sp;
    /* 下一拍 Est_SpinTick 以 s32Theta 作为"本拍采样时刻"的角，故先推进一拍（同 flux_pll.c 的 angleStep） */
    gEst.pll.s32Theta = (int32_t)((uint32_t)(uint16_t)theta << 16) + (sp >> 15) * (int32_t)SPEED_TO_THETA;
    gEst.thetaPll = (int16_t)(gEst.pll.s32Theta >> 16);
    gFoc.theta = gEst.thetaPll;
    gFoc.thetaInv = gEst.thetaPll;
    FluxObs_SeedCatch(theta);
    Meas_SpeedSource(spd);
    Meas_SpeedPreset(spd);
}
