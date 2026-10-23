#include "foc_core.h"
#include "hal.h"
#include "meas.h"
#include "est.h"
#include "ctrl_pi.h"
#include "supervisor.h"
#include "svpwm.h"

FocCore_TypeDef gFoc;

void FocCore_Init(void)
{
    /* 母线归一化系数初值（运行中 Meas_SlowFilter 按 VbusAvg 更新） */
    gMeas.kvbus = Q15(0.866 * UDC_MAX / UDC_RATE / 4);

    /* d 轴电流环（用 Ld） */
    /* 快环 PI（Pi_Run32）要求 Kp、Ki <= 65535，超了封顶并在 gFluxDiag.cfgWarn 置 bit1 */
    gFoc.piD.qKp = CURRENT_KP_D > 65535 ? 65535 : CURRENT_KP_D;
    gFoc.piD.qKi = CURRENT_KI_D > 65535 ? 65535 : CURRENT_KI_D;
    gFoc.piD.qOutMax = CURRENT_OUTMAX;
    gFoc.piD.qOutMin = -gFoc.piD.qOutMax;
    gFoc.piD.qdSum = 0;
    gFoc.piD.qInMeas = 0;
    gFoc.piD.qOut = 0;
    /* q 轴电流环（用 Lq） */
    gFoc.piQ.qKp = CURRENT_KP_Q > 65535 ? 65535 : CURRENT_KP_Q;
    gFoc.piQ.qKi = CURRENT_KI_Q > 65535 ? 65535 : CURRENT_KI_Q;
    gFoc.piQ.qOutMax = CURRENT_OUTMAX;
    gFoc.piQ.qOutMin = -gFoc.piQ.qOutMax;
    gFoc.piQ.qdSum = 0;
    gFoc.piQ.qInMeas = 0;
    gFoc.piQ.qOut = 0;

    /* 转速反馈清零（Meas_SlowFilter 滤成 gMeas.speed） */
    gMeas.speedRaw = 0;
}

void FocCore_CurrentLoop(void)
{
    int16_t s16Temp;

    gFoc.ang = Mc_SinCos(gFoc.theta);
    MCFLIB_Park_S16(&gMeas.iab, &gFoc.ang, &gFoc.idq);

    s16Temp = MLIB_Mul_Q15(FOC_DUTY_LIMIT_Q15, gMeas.vbusAvg);
    gFoc.vmax = MLIB_Mul_Q15(s16Temp, Q15(0.6667));     /* 幅值不变变换 */

    FocCore_DqPI();

#if CTRL_ANGLE_COMP
    /* 反 Park 角（gFoc.thetaInv）由估计模块按时序补偿给出；相同时省一次查表 */
    if (gFoc.thetaInv != gFoc.theta)
        gFoc.angInv = Mc_SinCos(gFoc.thetaInv);
    else
        gFoc.angInv = gFoc.ang;
#else
    gFoc.angInv = gFoc.ang;
#endif
    MCFLIB_InvPark_S16(&gFoc.vdq, &gFoc.angInv, &gFoc.vab);
    FocCore_BusNormalize(&gFoc.vab, gMeas.kvbus, &gFoc.vabMod);
}

/* d/q 电流 PI（原厂 Fast_Boot_Current_Controller 的逻辑，自写、防溢出）：
 *   Startup 子状态：d 轴积分器每拍预置成 -2000*iq_cmd（原厂的启动经验值，
 *                   相当于给一个 -omega*Lq*iq 量级的 Vd，保留）
 *   d 轴输出限 ±VMAX；q 轴限 ±sqrt(VMAX^2 - Vd^2)（电压圆，d 轴优先） */
#if CTRL_DQ_DECOUPLE
/* 开关请求由慢环发出（s_decReq：0 无，1 关，2 开），快环在 PI 之前执行：把前馈量
 * 搬进/搬出积分器和"前馈开关"在同一拍里完成，不会被中断打断成半截。 */
static volatile uint8_t s_decOn, s_decReq;
/* vd_ff = -w*Lq*iq，vq_ff = w*psi_f + w*Ld*id（指令电流，Q15 电压） */
static __forceinline void decoupleFF(int32_t *ffd, int32_t *ffq)
{
    int32_t w = gEst.pll.s32Speed >> 16;
    int32_t wiq = (w * gFoc.idqRef.s16Q) >> 15;
    int32_t wid = (w * gFoc.idqRef.s16D) >> 15;
    *ffd = -((wiq * DEC_K_LQ_Q12) >> 12);
    *ffq = ((w * DEC_K_EMF_Q15) >> 15) + ((wid * DEC_K_LD_Q12) >> 12);
}

void FocCore_SetDecouple(uint8_t on)
{
    s_decReq = on ? 2 : 1;
}

static __forceinline void applyDecoupleRequest(void)
{
    uint8_t on;
    int32_t ffd, ffq;
    if (!s_decReq) return;
    on = (uint8_t)(s_decReq - 1);
    s_decReq = 0;
    if (on == s_decOn) return;
    decoupleFF(&ffd, &ffq);
    if (on) { gFoc.piD.qdSum -= ffd << 15; gFoc.piQ.qdSum -= ffq << 15; }
    else    { gFoc.piD.qdSum += ffd << 15; gFoc.piQ.qdSum += ffq << 15; }
    s_decOn = on;
}

void FocCore_ResetDecouple(void) { s_decReq = 0; s_decOn = 0; }
#else
void FocCore_SetDecouple(uint8_t on) { (void)on; }
void FocCore_ResetDecouple(void) { }
#endif

void FocCore_DqPI(void)
{
    int32_t t, ffd = 0, ffq = 0, vd, lim;
#if CTRL_DQ_DECOUPLE
    applyDecoupleRequest();
    if (s_decOn) decoupleFF(&ffd, &ffq);
#endif
#if (STARTUP_MODE == STARTUP_VENDOR)
    if (gSup.runState == RunState_Startup) {
        t = (int32_t)gFoc.idqRef.s16Q * -2000;
        if (t < -32767000) t = -32767000;
        else if (t > 32767000) t = 32767000;
        gFoc.piD.qdSum = t;
    }
#endif
    /* 前馈为 0 时与原厂逐位相同：PI 输出范围 = 电压限幅 - 前馈 */
    gFoc.piD.qOutMax = gFoc.vmax - ffd;
    gFoc.piD.qOutMin = -gFoc.vmax - ffd;
    gFoc.piD.qInRef = gFoc.idqRef.s16D;
    gFoc.piD.qInMeas = gFoc.idq.s16D;
    Pi_Run32(&gFoc.piD);
    vd = gFoc.piD.qOut + ffd;
    gFoc.vdq.s16D = (int16_t)vd;

    t = (int32_t)gFoc.vmax * gFoc.vmax - (int32_t)gFoc.vdq.s16D * gFoc.vdq.s16D;
    lim = Mc_SqrtQ15(t);
    gFoc.piQ.qOutMax = lim - ffq;
    gFoc.piQ.qOutMin = -lim - ffq;
    gFoc.piQ.qInRef = gFoc.idqRef.s16Q;
    gFoc.piQ.qInMeas = gFoc.idq.s16Q;
    Pi_Run32(&gFoc.piQ);
    gFoc.vdq.s16Q = (int16_t)(gFoc.piQ.qOut + ffq);
}

void FocCore_Modulate(uint8_t mode)
{
    pwm_gen.Mode = mode;
    pwm_gen.Alpha = gFoc.vabMod.s16Alpha;
    pwm_gen.Beta = gFoc.vabMod.s16Beta;
    Svpwm_Calc(&pwm_gen, &gMeas.iabc, (int16_t *)&gMeas.ibusRec);
    HAL_PwmWriteCompares(&pwm_gen);
}

void FocCore_ModulateZero(void)
{
    pwm_gen.Mode = SEVENMODE;
    pwm_gen.Alpha = 0;
    pwm_gen.Beta = 0;
    Svpwm_Calc(&pwm_gen, &gMeas.iabc, (int16_t *)&gMeas.ibusRec);
    HAL_PwmWriteCompares(&pwm_gen);
}
