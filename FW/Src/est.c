#include "est.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "meas.h"
#include "foc_core.h"
#include "speed_ctrl.h"

Est_TypeDef gEst;

static int32_t s32StartupSpdRep;    /* test31：Startup 期间上报的限速转速（Q31） */

/* 观测器 + 运行期 PLL 跟踪一拍。返回 PLL 对本拍（采样时刻 k）的角度（鉴相用的
 * 就是它）；返回时 PLL 已推进到 k+1（gEst.thetaPll）。用 s32Theta 而不是
 * thetaPll 取 k 时刻角：从原厂 Startup 切进来时 thetaPll 没有更新过。 */
int16_t Est_TrackTick(void)
{
    int16_t thK = (int16_t)(gEst.pll.s32Theta >> 16), err;
    FluxObs_UpdateFeedback();
    err = FluxPLL_PhaseError(sFluxObs.s16PsiRAl, sFluxObs.s16PsiRBe, thK);
    gEst.thetaPll = FluxPLL_Run(err, &gEst.pll);
    return thK;
}

void Est_SpinTick(void)
{
#if CTRL_ANGLE_COMP
    /* Park 用 k 时刻角；反 Park 用 k + CTRL_INVPARK_ADVANCE 拍（电压在下一个 PWM 周期生效） */
    int16_t thK = Est_TrackTick();
    gFoc.theta = thK + gEst.thetaOffset;
    gFoc.thetaInv = gFoc.theta + FluxPLL_StepQ15(&gEst.pll, INVPARK_ADV_Q8);
    gMeas.speedRaw = gEst.pll.s32Speed >> 16;
#else
    /* 原厂时序：Park 和反 Park 都用 PLL 推进到 k+1 的角度 */
    int16_t err;

    FluxObs_UpdateFeedback();
    err = FluxPLL_PhaseError(sFluxObs.s16PsiRAl, sFluxObs.s16PsiRBe, gEst.thetaPll);
    gEst.thetaPll = FluxPLL_Run(err, &gEst.pll);

    /* thetaOffset：飞车启动(Torqueloop)交接时的角度差，Spin 慢慢把它消到 0 */
    gFoc.theta = gEst.thetaPll + gEst.thetaOffset;
    /* 不加原厂 SMO 的相位补偿：PLL 直接读 psi_r，没有观测器滞后要补（test25/26） */

    gFoc.thetaInv = gFoc.theta + MLIB_Mul_Q15(gMeas.speed, SPEED_TO_THETA);
    gMeas.speedRaw = gEst.pll.s32Speed >> 16;
#endif
}

void Est_StartupTick(void)
{
    int16_t err;

    /* 原厂"强制方向"：上报转速不允许与指令反号 */
    if (gSpd.cmd > 0) {
        if (gMeas.speedRaw < 0) gMeas.speedRaw = 1;
    } else {
        if (gMeas.speedRaw > 0) gMeas.speedRaw = -1;
    }

    FluxObs_UpdateFeedback();

    /* test32/33：用 PLL 的当前角度鉴相（不是上一拍输出的拷贝），进入 Startup
     * 的第一拍误差才是 0；其余每拍两者逐位相同。 */
    err = FluxPLL_PhaseError(sFluxObs.s16PsiRAl, sFluxObs.s16PsiRBe, (int16_t)(gEst.pll.s32Theta >> 16));

    /* 12.8 rpm 下限 = 原厂 Q31(0.004)，保持原厂启动斜坡的行为 */
    gFoc.theta = FluxPLL_RunStartup(err, &gEst.pll, gSpd.cmd, STARTUP_PLL_FLOOR_Q31);
    gFoc.thetaInv = gFoc.theta;       /* 不做角度补偿 */
}

/* test31：Startup 期间上报给原厂代码的转速 = PLL 转速再加加速度限幅。
 * PLL 本身（角度、s32Speed）不受影响。原厂启动的切换判据（vendorStartupDone）只看 gMeas.speed
 * （Meas_SlowFilter 由这里的值滤出），80 rpm 持续 10 ms 就切 Spin。转子挣脱时 PLL
 * 追相会冲高 25~40 ms（test30：25 ms 内 165~279 rpm，而转子只有 54~110 rpm），
 * 足以骗过判据、切过去再塌回来（4/4 次）。0.5 A 只能让转子加速约 250 rpm/s，
 * 1000 rpm/s 的限幅不会拖慢真启动（核对过 test28_2/test29 的 17 次启动），
 * 却能把 30 ms 的冲高压到 +30 rpm。 */
void Est_StartupReportSpeed(void)
{
    int32_t s32Step = gEst.pll.s32Speed - s32StartupSpdRep;
    if (s32Step >  FLUX_STARTUP_ACC_STEP) s32Step =  FLUX_STARTUP_ACC_STEP;
    if (s32Step < -FLUX_STARTUP_ACC_STEP) s32Step = -FLUX_STARTUP_ACC_STEP;
    s32StartupSpdRep += s32Step;
    gMeas.speedRaw = (int16_t)(s32StartupSpdRep >> 16);
}

void Est_OnEnterStartupCold(void)
{
    gEst.pll.s32Theta = gFoc.theta << 16;
    FluxObs_BeginStartup();
    s32StartupSpdRep = 0;           /* BeginStartup 也把 PLL 速度清零了 */
}

void Est_OnCatch(int16_t theta, int16_t spd)
{
    int32_t sp = (int32_t)spd << 16;
    FluxPLL_Init(&gEst.pll);                    /* 增益（飞车检测前可能被 FLY_VENDOR 借用过）+ 清状态 */
    gEst.pll.s32Speed = sp;
    gEst.pll.s32I_1 = sp;
    /* 下一拍 Est_SpinTick 用 s32Theta 当"本拍采样时刻"的角：先推进一拍（同 flux_pll.c 的 angleStep） */
    gEst.pll.s32Theta = (int32_t)((uint32_t)(uint16_t)theta << 16) + (sp >> 15) * (int32_t)SPEED_TO_THETA;
    gEst.thetaPll = (int16_t)(gEst.pll.s32Theta >> 16);
    gEst.thetaOffset = 0;
    gFoc.theta = gEst.thetaPll;
    gFoc.thetaInv = gEst.thetaPll;
    FluxObs_SeedCatch(theta);
    gMeas.speedRaw = spd;
    gMeas.speed = spd;
    /* MLIB_FilterMA_Q15 输入恒为 x 时的稳态累加值 = x*2^sh - x */
    gMeas.speedFilt.s32Acc = ((int32_t)spd << gMeas.speedFilt.u16Sh) - spd;
    s32StartupSpdRep = sp;
}

void Est_OnEnterStartupFromSpin(void)
{
    s32StartupSpdRep = (int32_t)gMeas.speedRaw * 65536;
}
