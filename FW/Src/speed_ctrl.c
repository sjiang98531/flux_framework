#include "speed_ctrl.h"
#include "meas.h"
#include "foc_core.h"
#include "hal.h"
#include "ctrl_pi.h"
#include <stdlib.h>

SpeedCtrl_TypeDef gSpd;

#define RUN_PI(p)   Pi_Run64(p)     /* 64 位，增益可以超过 65535 */

#define RAMP_Q31(rate, base)    Q31((rate) / (base) / SLOWLOOP_FREQ)

void SpeedCtrl_Init(void)
{
    gSpd.iqRampSt.s32RampUp   = RAMP_Q31(TORQUE_RAMP_UP, I_MAX);
    gSpd.iqRampSt.s32RampDown = RAMP_Q31(TORQUE_RAMP_DOWN, I_MAX);
    gSpd.speedRampSt.s32RampUp     = RAMP_Q31(SPEED_RAMP_UP, SPEED_MAX);
    gSpd.speedRampSt.s32RampDown   = RAMP_Q31(SPEED_RAMP_DOWN, SPEED_MAX);
    gSpd.pRampSt.s32RampUp     = RAMP_Q31(POWER_RAMP_UP, POWER_MAX);
    gSpd.pRampSt.s32RampDown   = RAMP_Q31(POWER_RAMP_DOWN, POWER_MAX);

    gSpd.iqLimit = Q15(SPD_PI_LIMIT / I_MAX);

    gSpd.pi.qKp = SPEED_KP;
    gSpd.pi.qKi = SPEED_KI;
    gSpd.pi.qdSum = 0;
    gSpd.pi.qInMeas = 0;
    gSpd.pi.qOut = 0;

    gSpd.piPower.qKp = POWER_KP;
    gSpd.piPower.qKi = POWER_KI;
    gSpd.piPower.qdSum = 0;
    gSpd.piPower.qInMeas = 0;
    gSpd.piPower.qOut = 0;

#if (CLOSE_LOOP == SPEED_LOOP)
    gSpd.pi.qOutMax = gSpd.iqLimit;
    gSpd.pi.qOutMin = -gSpd.iqLimit;
    gSpd.piPower.qOutMax = 0;                      /* 功率环只做母线电流限：只许往下拉 */
    gSpd.piPower.qOutMin = -gSpd.iqLimit;
#elif (CLOSE_LOOP == POWER_LOOP)
    gSpd.pi.qOutMax = 0;                      /* 速度环只做限速 */
    gSpd.pi.qOutMin = -gSpd.iqLimit;
    gSpd.piPower.qOutMax = gSpd.iqLimit;
    gSpd.piPower.qOutMin = -gSpd.iqLimit;
#endif
}

void SpeedCtrl_UpdatePowerFdk(void)
{
    gSpd.powerFdk = 2 * MLIB_Mul_Q15(Q15(0.866), MLIB_Mul_Q15(gMeas.vbusAvg, gMeas.ibusAvg));
}

void SpeedCtrl_SpinSlow(void)
{
    gSpd.ref = gSpd.cmd;

#if (CLOSE_LOOP == TORQUE_LOOP)
    gSpd.iqRampQ31 = Mc_RampQ31(gSpd.ref << 16, &gSpd.iqRampSt);
    gSpd.iqRamp = gSpd.iqRampQ31 >> 16;
    gFoc.idqRef.s16Q = gSpd.iqRamp;

#elif (CLOSE_LOOP == SPEED_LOOP)
    {
        int32_t s32RampPrev, s32IqCmd;
        int16_t s16AccFF;

        s32RampPrev = gSpd.rampQ31;
        gSpd.rampQ31 = Mc_RampQ31(gSpd.ref << 16, &gSpd.speedRampSt);
        gSpd.ramp = gSpd.rampQ31 >> 16;

        /* 加速度前馈：只在 |斜坡| 增大时加；减速保持原厂行为 */
        s16AccFF = 0;
#if (SPEED_ACC_FF_ENABLE)
        if ((gSpd.rampQ31 > s32RampPrev) && (gSpd.rampQ31 > 0))
            s16AccFF = SPEED_ACC_FF_IQ;
        else if ((gSpd.rampQ31 < s32RampPrev) && (gSpd.rampQ31 < 0))
            s16AccFF = -SPEED_ACC_FF_IQ;
#endif
        gSpd.pi.qInRef = gSpd.ramp;
        gSpd.pi.qInMeas = gMeas.speed;
        RUN_PI(&gSpd.pi);

        gSpd.piPower.qInRef = IBUS_LIMIT_VALUE;    /* 母线电流限 */
        gSpd.piPower.qInMeas = gMeas.ibusAvg;
        RUN_PI(&gSpd.piPower);

        s32IqCmd = (int32_t)gSpd.pi.qOut + gSpd.piPower.qOut + s16AccFF;
        if (s32IqCmd > gSpd.iqLimit)  s32IqCmd = gSpd.iqLimit;
        if (s32IqCmd < -gSpd.iqLimit) s32IqCmd = -gSpd.iqLimit;
        gFoc.idqRef.s16Q = (int16_t)s32IqCmd;
    }
#else
    gSpd.pRampQ31 = Mc_RampQ31(gSpd.ref << 16, &gSpd.pRampSt);
    gSpd.pRamp = gSpd.pRampQ31 >> 16;

    gSpd.pi.qInRef = SPEED_LIMIT_VALUE;       /* 限速 */
    gSpd.pi.qInMeas = gMeas.speed;
    RUN_PI(&gSpd.pi);

    gSpd.piPower.qInRef = gSpd.pRamp;
    gSpd.piPower.qInMeas = gSpd.powerFdk;
    RUN_PI(&gSpd.piPower);

    gFoc.idqRef.s16Q = gSpd.piPower.qOut + gSpd.pi.qOut;
#endif
}

/* test30：进入 Spin 后第一拍会加的前馈。原厂把速度 PI 积分器预置成当前整个
 * Iq 指令；再叠加前馈就会把加速电流算两遍（启动 Iq 本身已经在加速转子）。
 * Spin 的斜坡从 gMeas.speed 开始，所以指令离 0 比 filt 更远时就是在加速。 */
static int16_t accFFAtSpinEntry(void)
{
#if (SPEED_ACC_FF_ENABLE) && (CLOSE_LOOP == SPEED_LOOP)
    if ((gMeas.speed >= 0) && (gSpd.cmd > gMeas.speed)) return SPEED_ACC_FF_IQ;
    if ((gMeas.speed <= 0) && (gSpd.cmd < gMeas.speed)) return -SPEED_ACC_FF_IQ;
#endif
    return 0;
}

void SpeedCtrl_OnEnterSpin(void)
{
    gSpd.iqRampSt.s32RampUp   = RAMP_Q31(TORQUE_RAMP_UP, I_MAX);
    gSpd.iqRampSt.s32RampDown = RAMP_Q31(TORQUE_RAMP_DOWN, I_MAX);
    gSpd.speedRampSt.s32RampUp     = RAMP_Q31(SPEED_RAMP_UP, SPEED_MAX);
    gSpd.speedRampSt.s32RampDown   = RAMP_Q31(SPEED_RAMP_DOWN, SPEED_MAX);
    gSpd.pRampSt.s32RampUp     = RAMP_Q31(POWER_RAMP_UP, POWER_MAX);
    gSpd.pRampSt.s32RampDown   = RAMP_Q31(POWER_RAMP_DOWN, POWER_MAX);
    gSpd.speedRampSt.s32State      = gMeas.speed << 16;
    gSpd.pRampSt.s32State      = gSpd.powerFdk << 16;
    gSpd.iqRampSt.s32State    = gFoc.idqRef.s16Q << 16;
    gSpd.pi.qdSum               = ((int32_t)gFoc.idqRef.s16Q - accFFAtSpinEntry()) * 32768;
    gSpd.piPower.qdSum               = gFoc.idqRef.s16Q << 15;
}

void SpeedCtrl_UseStartupRamp(void)
{
    gSpd.speedRampSt.s32RampUp   = RAMP_Q31(STARTUP_SPEED_RAMP, SPEED_MAX);
    gSpd.speedRampSt.s32RampDown = RAMP_Q31(STARTUP_SPEED_RAMP, SPEED_MAX);
}
