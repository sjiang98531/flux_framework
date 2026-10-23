#include "speed_ctrl.h"
#include "ctrl_globals.h"
#include "ctrl_pi.h"
#include <stdlib.h>

int16_t s16PowerFdk;

#define RUN_PI(p)   Pi_Run64(p)     /* 64 位，增益可以超过 65535 */

#define RAMP_Q31(rate, base)    Q31((rate) / (base) / SLOWLOOP_FREQ)

void SpeedCtrl_Init(void)
{
    USER.sCurrentRamp.s32RampUp   = RAMP_Q31(TORQUE_RAMP_UP, I_MAX);
    USER.sCurrentRamp.s32RampDown = RAMP_Q31(TORQUE_RAMP_DOWN, I_MAX);
    USER.sSpeedRamp.s32RampUp     = RAMP_Q31(SPEED_RAMP_UP, SPEED_MAX);
    USER.sSpeedRamp.s32RampDown   = RAMP_Q31(SPEED_RAMP_DOWN, SPEED_MAX);
    USER.sPowerRamp.s32RampUp     = RAMP_Q31(POWER_RAMP_UP, POWER_MAX);
    USER.sPowerRamp.s32RampDown   = RAMP_Q31(POWER_RAMP_DOWN, POWER_MAX);

    USER.s16IqLimitMax = Q15(SPD_PI_LIMIT / I_MAX);

    FOC.Speed.qKp = SPEED_KP;
    FOC.Speed.qKi = SPEED_KI;
    FOC.Speed.qdSum = 0;
    FOC.Speed.qInMeas = 0;
    FOC.Speed.qOut = 0;

    FOC.Power.qKp = POWER_KP;
    FOC.Power.qKi = POWER_KI;
    FOC.Power.qdSum = 0;
    FOC.Power.qInMeas = 0;
    FOC.Power.qOut = 0;

#if (CLOSE_LOOP == SPEED_LOOP)
    FOC.Speed.qOutMax = USER.s16IqLimitMax;
    FOC.Speed.qOutMin = -USER.s16IqLimitMax;
    FOC.Power.qOutMax = 0;                      /* 功率环只做母线电流限：只许往下拉 */
    FOC.Power.qOutMin = -USER.s16IqLimitMax;
#elif (CLOSE_LOOP == POWER_LOOP)
    FOC.Speed.qOutMax = 0;                      /* 速度环只做限速 */
    FOC.Speed.qOutMin = -USER.s16IqLimitMax;
    FOC.Power.qOutMax = USER.s16IqLimitMax;
    FOC.Power.qOutMin = -USER.s16IqLimitMax;
#endif
}

void SpeedCtrl_UpdatePowerFdk(void)
{
    s16PowerFdk = 2 * MLIB_Mul_Q15(Q15(0.866), MLIB_Mul_Q15(FOC.s16VbusAvg, FOC.s16IbusAvg));
}

void SpeedCtrl_SpinSlow(void)
{
    FOC.s16SpdRef = FOC.s16SpdCmd;

#if (CLOSE_LOOP == TORQUE_LOOP)
    FOC.s32CurRamp = Mc_RampQ31(FOC.s16SpdRef << 16, &USER.sCurrentRamp);
    FOC.s16CurRamp = FOC.s32CurRamp >> 16;
    FOC.sIdq_cmd.s16Q = FOC.s16CurRamp;

#elif (CLOSE_LOOP == SPEED_LOOP)
    {
        int32_t s32RampPrev, s32IqCmd;
        int16_t s16AccFF;

        s32RampPrev = FOC.s32SpeedRamp;
        FOC.s32SpeedRamp = Mc_RampQ31(FOC.s16SpdRef << 16, &USER.sSpeedRamp);
        FOC.s16SpeedRamp = FOC.s32SpeedRamp >> 16;

        /* 加速度前馈：只在 |斜坡| 增大时加；减速保持原厂行为 */
        s16AccFF = 0;
#if (SPEED_ACC_FF_ENABLE)
        if ((FOC.s32SpeedRamp > s32RampPrev) && (FOC.s32SpeedRamp > 0))
            s16AccFF = SPEED_ACC_FF_IQ;
        else if ((FOC.s32SpeedRamp < s32RampPrev) && (FOC.s32SpeedRamp < 0))
            s16AccFF = -SPEED_ACC_FF_IQ;
#endif
        FOC.Speed.qInRef = FOC.s16SpeedRamp;
        FOC.Speed.qInMeas = FOC.s16SpdFilt;
        RUN_PI(&FOC.Speed);

        FOC.Power.qInRef = IBUS_LIMIT_VALUE;    /* 母线电流限 */
        FOC.Power.qInMeas = FOC.s16IbusAvg;
        RUN_PI(&FOC.Power);

        s32IqCmd = (int32_t)FOC.Speed.qOut + FOC.Power.qOut + s16AccFF;
        if (s32IqCmd > USER.s16IqLimitMax)  s32IqCmd = USER.s16IqLimitMax;
        if (s32IqCmd < -USER.s16IqLimitMax) s32IqCmd = -USER.s16IqLimitMax;
        FOC.sIdq_cmd.s16Q = (int16_t)s32IqCmd;
    }
#else
    FOC.s32PowerRamp = Mc_RampQ31(FOC.s16SpdRef << 16, &USER.sPowerRamp);
    FOC.s16PowerRamp = FOC.s32PowerRamp >> 16;

    FOC.Speed.qInRef = SPEED_LIMIT_VALUE;       /* 限速 */
    FOC.Speed.qInMeas = FOC.s16SpdFilt;
    RUN_PI(&FOC.Speed);

    FOC.Power.qInRef = FOC.s16PowerRamp;
    FOC.Power.qInMeas = s16PowerFdk;
    RUN_PI(&FOC.Power);

    FOC.sIdq_cmd.s16Q = FOC.Power.qOut + FOC.Speed.qOut;
#endif
}

/* test30：进入 Spin 后第一拍会加的前馈。原厂把速度 PI 积分器预置成当前整个
 * Iq 指令；再叠加前馈就会把加速电流算两遍（启动 Iq 本身已经在加速转子）。
 * Spin 的斜坡从 s16SpdFilt 开始，所以指令离 0 比 filt 更远时就是在加速。 */
static int16_t accFFAtSpinEntry(void)
{
#if (SPEED_ACC_FF_ENABLE) && (CLOSE_LOOP == SPEED_LOOP)
    if ((FOC.s16SpdFilt >= 0) && (FOC.s16SpdCmd > FOC.s16SpdFilt)) return SPEED_ACC_FF_IQ;
    if ((FOC.s16SpdFilt <= 0) && (FOC.s16SpdCmd < FOC.s16SpdFilt)) return -SPEED_ACC_FF_IQ;
#endif
    return 0;
}

void SpeedCtrl_OnEnterSpin(void)
{
    USER.sCurrentRamp.s32RampUp   = RAMP_Q31(TORQUE_RAMP_UP, I_MAX);
    USER.sCurrentRamp.s32RampDown = RAMP_Q31(TORQUE_RAMP_DOWN, I_MAX);
    USER.sSpeedRamp.s32RampUp     = RAMP_Q31(SPEED_RAMP_UP, SPEED_MAX);
    USER.sSpeedRamp.s32RampDown   = RAMP_Q31(SPEED_RAMP_DOWN, SPEED_MAX);
    USER.sPowerRamp.s32RampUp     = RAMP_Q31(POWER_RAMP_UP, POWER_MAX);
    USER.sPowerRamp.s32RampDown   = RAMP_Q31(POWER_RAMP_DOWN, POWER_MAX);
    USER.sSpeedRamp.s32State      = FOC.s16SpdFilt << 16;
    USER.sPowerRamp.s32State      = s16PowerFdk << 16;
    USER.sCurrentRamp.s32State    = FOC.sIdq_cmd.s16Q << 16;
    FOC.Speed.qdSum               = ((int32_t)FOC.sIdq_cmd.s16Q - accFFAtSpinEntry()) * 32768;
    FOC.Power.qdSum               = FOC.sIdq_cmd.s16Q << 15;
}

void SpeedCtrl_UseStartupRamp(void)
{
    USER.sSpeedRamp.s32RampUp   = RAMP_Q31(STARTUP_SPEED_RAMP, SPEED_MAX);
    USER.sSpeedRamp.s32RampDown = RAMP_Q31(STARTUP_SPEED_RAMP, SPEED_MAX);
}
