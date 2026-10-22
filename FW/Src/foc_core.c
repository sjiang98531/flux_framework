#include "foc_core.h"
#include "hal.h"
#include "ctrl_globals.h"

/* 比较值 -> 占空比（Q15）的系数：2^30 / PWM_PERIOD */
#define COUNT_SCALE ((int32_t)(1073741824.0f / PWM_PERIOD))

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

void FocCore_Clarke(FOC_TypeDef *f)
{
    MCFLIB_Clark_Sat_S16(&f->sIabc, &f->sIAlBe);
}

void FOC_Current_Controller(FOC_TypeDef *pFOC)
{
    int16_t s16Temp;

    pFOC->sAngle_cmd = MLIB_SinCos(pFOC->s16CmdTheta);
    MCFLIB_Park_S16(&pFOC->sIAlBe, &pFOC->sAngle_cmd, &pFOC->sIdq);

    s16Temp = MLIB_Mul_Q15(USER.s16DutyLimitCmd, pFOC->s16VbusAvg);
    pFOC->s16VMAX = MLIB_Mul_Q15(s16Temp, Q15(0.6667));     /* 幅值不变变换 */

    /* d/q 两个 PI + Vq 圆限幅（原厂库；Startup 子状态下库会预置 d 轴积分器） */
    Fast_Boot_Current_Controller(pFOC);

    pFOC->sAngle_cmd_update = pFOC->sAngle_cmd;
    MCFLIB_InvPark_S16(&pFOC->sVdq, &pFOC->sAngle_cmd_update, &pFOC->sVAlBe);
    DcBus_Ripple_suppress(&pFOC->sVAlBe, pFOC->s16VbusAvg, &pFOC->sVAlBe_cmd);
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

/* "比较值越小，相电压越高"。重建上一 PWM 周期的平均相电压：
 *   pole_x = (PERIOD - cmp_x)/PERIOD * Vbus
 *   v_alpha = (2a - b - c)/3，v_beta = (b - c)/sqrt(3) */
void FocCore_VoltageFromCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC, int16_t bus,
                                 int16_t *va, int16_t *vb)
{
    int32_t a, b, c, t;
    a = ((int32_t)(PWM_PERIOD - cmpA) * COUNT_SCALE) >> 15;
    b = ((int32_t)(PWM_PERIOD - cmpB) * COUNT_SCALE) >> 15;
    c = ((int32_t)(PWM_PERIOD - cmpC) * COUNT_SCALE) >> 15;
    a = (a * bus) >> 15; b = (b * bus) >> 15; c = (c * bus) >> 15;
    t = 2 * a - b - c;
    *va = (int16_t)((t * 21846 + 32768) >> 16);
    *vb = (int16_t)(((b - c) * 18919) >> 15);
}
