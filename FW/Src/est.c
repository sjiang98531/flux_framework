#include "est.h"
#include "flux_obs.h"
#include "flux_pll.h"
#include "ctrl_globals.h"

static int32_t s32StartupSpdRep;    /* test31：Startup 期间上报的限速转速（Q31） */

void Est_SpinTick(FOC_TypeDef *f)
{
    int16_t err;

    FluxObs_UpdateFeedback(f);
    err = FluxPLL_PhaseError(sFluxObs.s16PsiRAl, sFluxObs.s16PsiRBe, f->s16ThetaPLL);
    f->s16ThetaPLL = FluxPLL_Run(err, &f->sPLL);

    /* s16ErrTheta：飞车启动(Torqueloop)交接时的角度差，Spin 慢慢把它消到 0 */
    f->s16CmdTheta = f->s16ThetaPLL + f->s16ErrTheta;
    /* 不加原厂 SMO 的相位补偿：PLL 直接读 psi_r，没有观测器滞后要补（test25/26） */
    f->s16ThetaComp = 0;
    f->s16CmdTheta = f->s16CmdTheta + f->s16ThetaComp;

    f->s16CmdTheta_Update = f->s16CmdTheta + MLIB_Mul_Q15(f->s16SpdFilt, SPEED_TO_THETA);
    f->FluxFeedback.s16Speed = f->sPLL.s32Speed >> 16;
}

void Est_StartupTick(FOC_TypeDef *f)
{
    int16_t err;

    /* 原厂"强制方向"：上报转速不允许与指令反号 */
    if (f->s16SpdCmd > 0) {
        if (f->FluxFeedback.s16Speed < 0) f->FluxFeedback.s16Speed = 1;
    } else {
        if (f->FluxFeedback.s16Speed > 0) f->FluxFeedback.s16Speed = -1;
    }

    FluxObs_UpdateFeedback(f);

    /* test32/33：用 PLL 的当前角度鉴相（不是上一拍输出的拷贝），进入 Startup
     * 的第一拍误差才是 0；其余每拍两者逐位相同。 */
    err = FluxPLL_PhaseError(sFluxObs.s16PsiRAl, sFluxObs.s16PsiRBe, (int16_t)(f->sPLL.s32Theta >> 16));

    /* 12.8 rpm 下限 = 原厂 Q31(0.004)，保持原厂启动斜坡的行为 */
    f->s16CmdTheta = FluxPLL_RunStartup(err, &f->sPLL, f->s16SpdCmd, STARTUP_PLL_FLOOR_Q31);
    f->s16CmdTheta_Update = f->s16CmdTheta;       /* 不做角度补偿 */
}

/* test31：Startup 期间上报给原厂代码的转速 = PLL 转速再加加速度限幅。
 * PLL 本身（角度、s32Speed）不受影响。Fast_Boot_StartUp() 只看 s16SpdFilt
 * （Var_Filt 由这里的值滤出），80 rpm 持续 10 ms 就切 Spin。转子挣脱时 PLL
 * 追相会冲高 25~40 ms（test30：25 ms 内 165~279 rpm，而转子只有 54~110 rpm），
 * 足以骗过判据、切过去再塌回来（4/4 次）。0.5 A 只能让转子加速约 250 rpm/s，
 * 1000 rpm/s 的限幅不会拖慢真启动（核对过 test28_2/test29 的 17 次启动），
 * 却能把 30 ms 的冲高压到 +30 rpm。 */
void Est_StartupReportSpeed(FOC_TypeDef *f)
{
    int32_t s32Step = f->sPLL.s32Speed - s32StartupSpdRep;
    if (s32Step >  FLUX_STARTUP_ACC_STEP) s32Step =  FLUX_STARTUP_ACC_STEP;
    if (s32Step < -FLUX_STARTUP_ACC_STEP) s32Step = -FLUX_STARTUP_ACC_STEP;
    s32StartupSpdRep += s32Step;
    f->FluxFeedback.s16Speed = (int16_t)(s32StartupSpdRep >> 16);
}

void Est_OnEnterStartupCold(FOC_TypeDef *f)
{
    f->sPLL.s32Theta = f->s16CmdTheta << 16;
    FluxObs_BeginStartup(f, &USER.sSpeed_Filt);
    s32StartupSpdRep = 0;           /* BeginStartup 也把 PLL 速度清零了 */
}

void Est_OnEnterStartupFromSpin(FOC_TypeDef *f)
{
    s32StartupSpdRep = (int32_t)f->FluxFeedback.s16Speed * 65536;
}
