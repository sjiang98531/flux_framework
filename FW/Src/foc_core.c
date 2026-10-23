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
    /* 快环 PI（Pi_Run32）要求 Kp、Ki <= 65535，超了封顶并在 gFluxDiag.cfgWarn 置 bit1 */
    FOC.CurID.qKp = CURRENT_KP_D > 65535 ? 65535 : CURRENT_KP_D;
    FOC.CurID.qKi = CURRENT_KI_D > 65535 ? 65535 : CURRENT_KI_D;
    FOC.CurID.qOutMax = CURRENT_OUTMAX;
    FOC.CurID.qOutMin = -FOC.CurID.qOutMax;
    FOC.CurID.qdSum = 0;
    FOC.CurID.qInMeas = 0;
    FOC.CurID.qOut = 0;
    /* q 轴电流环（用 Lq） */
    FOC.CurIQ.qKp = CURRENT_KP_Q > 65535 ? 65535 : CURRENT_KP_Q;
    FOC.CurIQ.qKi = CURRENT_KI_Q > 65535 ? 65535 : CURRENT_KI_Q;
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

#if CTRL_ANGLE_COMP
    /* 反 Park 角（s16CmdTheta_Update）由估计模块按时序补偿给出；相同时省一次查表 */
    if (pFOC->s16CmdTheta_Update != pFOC->s16CmdTheta)
        pFOC->sAngle_cmd_update = MLIB_SinCos(pFOC->s16CmdTheta_Update);
    else
        pFOC->sAngle_cmd_update = pFOC->sAngle_cmd;
#else
    pFOC->sAngle_cmd_update = pFOC->sAngle_cmd;
#endif
    MCFLIB_InvPark_S16(&pFOC->sVdq, &pFOC->sAngle_cmd_update, &pFOC->sVAlBe);
    DcBus_Ripple_suppress(&pFOC->sVAlBe, pFOC->s16VbusAvg, &pFOC->sVAlBe_cmd);
}

/* d/q 电流 PI（替代原厂 Fast_Boot_Current_Controller，逻辑按反汇编逐条对应）：
 *   Startup 子状态：d 轴积分器每拍预置成 -2000*iq_cmd（原厂的启动经验值，
 *                   相当于给一个 -omega*Lq*iq 量级的 Vd，保留）
 *   d 轴输出限 ±VMAX；q 轴限 ±sqrt(VMAX^2 - Vd^2)（电压圆，d 轴优先） */
#if CTRL_DQ_DECOUPLE
/* 开关请求由慢环发出（s_decReq：0 无，1 关，2 开），快环在 PI 之前执行：把前馈量
 * 搬进/搬出积分器和"前馈开关"在同一拍里完成，不会被中断打断成半截。 */
static volatile uint8_t s_decOn, s_decReq;
/* vd_ff = -w*Lq*iq，vq_ff = w*psi_f + w*Ld*id（指令电流，Q15 电压） */
static __forceinline void decoupleFF(const FOC_TypeDef *f, int32_t *ffd, int32_t *ffq)
{
    int32_t w = f->sPLL.s32Speed >> 16;
    int32_t wiq = (w * f->sIdq_cmd.s16Q) >> 15;
    int32_t wid = (w * f->sIdq_cmd.s16D) >> 15;
    *ffd = -((wiq * DEC_K_LQ_Q12) >> 12);
    *ffq = ((w * DEC_K_EMF_Q15) >> 15) + ((wid * DEC_K_LD_Q12) >> 12);
}

void FocCore_SetDecouple(FOC_TypeDef *f, uint8_t on)
{
    (void)f;
    s_decReq = on ? 2 : 1;
}

static __forceinline void applyDecoupleRequest(FOC_TypeDef *f)
{
    uint8_t on;
    int32_t ffd, ffq;
    if (!s_decReq) return;
    on = (uint8_t)(s_decReq - 1);
    s_decReq = 0;
    if (on == s_decOn) return;
    decoupleFF(f, &ffd, &ffq);
    if (on) { f->CurID.qdSum -= ffd << 15; f->CurIQ.qdSum -= ffq << 15; }
    else    { f->CurID.qdSum += ffd << 15; f->CurIQ.qdSum += ffq << 15; }
    s_decOn = on;
}

void FocCore_ResetDecouple(void) { s_decReq = 0; s_decOn = 0; }
#else
void FocCore_SetDecouple(FOC_TypeDef *f, uint8_t on) { (void)f; (void)on; }
void FocCore_ResetDecouple(void) { }
#endif

void FocCore_DqPI(FOC_TypeDef *f)
{
    int32_t t, ffd = 0, ffq = 0, vd, lim;
#if CTRL_DQ_DECOUPLE
    applyDecoupleRequest(f);
    if (s_decOn) decoupleFF(f, &ffd, &ffq);
#endif
#if (STARTUP_MODE == STARTUP_VENDOR)
    if (eM1_RunSubState == RunState_Startup) {
        t = (int32_t)f->sIdq_cmd.s16Q * -2000;
        if (t < -32767000) t = -32767000;
        else if (t > 32767000) t = 32767000;
        f->CurID.qdSum = t;
    }
#endif
    /* 前馈为 0 时与原厂逐位相同：PI 输出范围 = 电压限幅 - 前馈 */
    f->CurID.qOutMax = f->s16VMAX - ffd;
    f->CurID.qOutMin = -f->s16VMAX - ffd;
    f->CurID.qInRef = f->sIdq_cmd.s16D;
    f->CurID.qInMeas = f->sIdq.s16D;
    Pi_Run32(&f->CurID);
    vd = f->CurID.qOut + ffd;
    f->sVdq.s16D = (int16_t)vd;

    t = (int32_t)f->s16VMAX * f->s16VMAX - (int32_t)f->sVdq.s16D * f->sVdq.s16D;
    lim = Sqrt_Q15(t);
    f->CurIQ.qOutMax = lim - ffq;
    f->CurIQ.qOutMin = -lim - ffq;
    f->CurIQ.qInRef = f->sIdq_cmd.s16Q;
    f->CurIQ.qInMeas = f->sIdq.s16Q;
    Pi_Run32(&f->CurIQ);
    f->sVdq.s16Q = (int16_t)(f->CurIQ.qOut + ffq);
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
