#include "foc_core.h"
#include "hal.h"
#include "ctrl_globals.h"
#include "ctrl_pi.h"
#include "supervisor.h"

void FocCore_Init(void)
{
    /* 电压限幅：VMAX = MAX_DUTY * Vbus * 2/3（FOC_Current_Controller 里每拍算） */
    USER.s16DutyLimitCmd = Q15(MAX_DUTY);
    /* 母线纹波补偿系数初值（运行中 Var_Filt 用 58116276/VbusAvg 覆盖） */
    FOC.s16KVbus = Q15(0.866 * UDC_MAX / UDC_RATE / 4);

    /* d 轴电流环（用 Ld） */
    FOC.CurID.qKp = CURRENT_KP_D;
    FOC.CurID.qKi = CURRENT_KI_D;
    FOC.CurID.qOutMax = CURRENT_OUTMAX;
    FOC.CurID.qOutMin = -FOC.CurID.qOutMax;
    FOC.CurID.qdSum = 0;
    FOC.CurID.qInMeas = 0;
    FOC.CurID.qOut = 0;
    /* q 轴电流环（用 Lq） */
    FOC.CurIQ.qKp = CURRENT_KP_Q;
    FOC.CurIQ.qKi = CURRENT_KI_Q;
    FOC.CurIQ.qOutMax = CURRENT_OUTMAX;
    FOC.CurIQ.qOutMin = -FOC.CurIQ.qOutMax;
    FOC.CurIQ.qdSum = 0;
    FOC.CurIQ.qInMeas = 0;
    FOC.CurIQ.qOut = 0;

    FOC.s16ThetaComp = 0;
    /* 原厂 SMO 留下的存储，库只读 s16Speed（Var_Filt） */
    FOC.FluxFeedback.s16Ealpha_est = 0;
    FOC.FluxFeedback.s16Ebeta_est = 0;
    FOC.FluxFeedback.s16Speed = 0;
}

void FOC_Current_Controller(FOC_TypeDef *pFOC)
{
    int16_t s16Temp;

    pFOC->sAngle_cmd = MLIB_SinCos(pFOC->s16CmdTheta);
    MCFLIB_Park_S16(&pFOC->sIAlBe, &pFOC->sAngle_cmd, &pFOC->sIdq);

    s16Temp = MLIB_Mul_Q15(USER.s16DutyLimitCmd, pFOC->s16VbusAvg);
    pFOC->s16VMAX = MLIB_Mul_Q15(s16Temp, Q15(0.6667));     /* 幅值不变变换 */

#if CTRL_OWN_PI
    FocCore_DqPI(pFOC);
#else
    /* 原厂库：d/q 两个 CalcPI + Vq 圆限幅；Startup 子状态下预置 d 轴积分器 */
    Fast_Boot_Current_Controller(pFOC);
#endif

    pFOC->sAngle_cmd_update = pFOC->sAngle_cmd;
    MCFLIB_InvPark_S16(&pFOC->sVdq, &pFOC->sAngle_cmd_update, &pFOC->sVAlBe);
    DcBus_Ripple_suppress(&pFOC->sVAlBe, pFOC->s16VbusAvg, &pFOC->sVAlBe_cmd);
}

/* d/q 电流 PI（替代原厂 Fast_Boot_Current_Controller，逻辑按反汇编逐条对应）：
 *   Startup 子状态：d 轴积分器每拍预置成 -2000*iq_cmd（原厂的启动经验值，
 *                   相当于给一个 -omega*Lq*iq 量级的 Vd，保留）
 *   d 轴输出限 ±VMAX；q 轴限 ±sqrt(VMAX^2 - Vd^2)（电压圆，d 轴优先） */
void FocCore_DqPI(FOC_TypeDef *f)
{
    int32_t t;
    if (eM1_RunSubState == RunState_Startup) {
        t = (int32_t)f->sIdq_cmd.s16Q * -2000;
        if (t < -32767000) t = -32767000;
        else if (t > 32767000) t = 32767000;
        f->CurID.qdSum = t;
    }
    f->CurID.qOutMax = f->s16VMAX;
    f->CurID.qOutMin = -f->CurID.qOutMax;
    f->CurID.qInRef = f->sIdq_cmd.s16D;
    f->CurID.qInMeas = f->sIdq.s16D;
    Pi_Run32(&f->CurID);
    f->sVdq.s16D = (int16_t)f->CurID.qOut;

    t = (int32_t)f->s16VMAX * f->s16VMAX - (int32_t)f->sVdq.s16D * f->sVdq.s16D;
    f->CurIQ.qOutMax = Sqrt_Q15(t);
    f->CurIQ.qOutMin = -f->CurIQ.qOutMax;
    f->CurIQ.qInRef = f->sIdq_cmd.s16Q;
    f->CurIQ.qInMeas = f->sIdq.s16Q;
    Pi_Run32(&f->CurIQ);
    f->sVdq.s16Q = (int16_t)f->CurIQ.qOut;
}

void FocCore_Modulate(const FOC_TypeDef *f, uint8_t mode)
{
    pwm_gen.Mode = mode;
    pwm_gen.Alpha = f->sVAlBe_cmd.s16Alpha;
    pwm_gen.Beta = f->sVAlBe_cmd.s16Beta;
    PWM_GEN_calc(&pwm_gen);
    Update_PWM(&pwm_gen);
}

void FocCore_ModulateZero(void)
{
    pwm_gen.Mode = SEVENMODE;
    pwm_gen.Alpha = 0;
    pwm_gen.Beta = 0;
    PWM_GEN_calc(&pwm_gen);
    Update_PWM(&pwm_gen);
}
