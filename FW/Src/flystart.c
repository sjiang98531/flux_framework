/* 飞车检测，见 flystart.h。 */
#include "flystart.h"
#include "meas.h"
#include "est.h"
#include "hal.h"
#include "mc_math.h"
#include "mathx.h"

FlyStart_TypeDef gFly;

/* 带移位的乘：sh >= 0 左移，< 0 右移 */
static __forceinline int32_t shiftBi(int32_t v, int16_t sh)
{
    return sh >= 0 ? (int32_t)((uint32_t)v << sh) : (v >> -sh);
}

/* 跟踪短路电流角度的二阶 PLL（原厂 MLIB_PLL 的语义）：
 *   积分器 += Ki*err*2，转速 = 积分器 + Kp*err*2（各自再按 Sh 移位）
 *   角度 += 转速 * ThGain（32 位乘法拆成高低 16 位做，避免溢出）
 * 返回 Q15 角度。 */
static int16_t trackPll(int16_t err, PLL_TypeDef *p)
{
    int32_t kp = shiftBi((int32_t)p->s16PGain * err * 2, p->i16PGainSh);
    int32_t ki = shiftBi((int32_t)p->s16IGain * err * 2, p->i16IGainSh);
    int32_t sp, th;
    p->s32I_1 += ki;
    sp = kp + p->s32I_1;
    p->s32Speed = sp;
    th = (((sp & 0xFFFF) * (int32_t)p->s16ThGain) >> 15) + (((sp >> 16) * (int32_t)p->s16ThGain) << 1);
    th = shiftBi(th, p->i16ThGainSh);
    p->s32Theta += th;
    return (int16_t)(p->s32Theta >> 16);
}

void FlyStart_CalibFast(void)
{
    FlyStart_TypeDef *fs = &gFly;
    int16_t err;
    sAngle_Trig a;

    HAL_PwmShortLowSides(gHal.caliDuty);         /* 三相下桥短接 */

    MCFLIB_Clark_Sat_S16(&gMeas.iabc, &gMeas.iab);
    fs->s32BemfAmp2 = (int32_t)gMeas.iab.s16Alpha * gMeas.iab.s16Alpha + (int32_t)gMeas.iab.s16Beta * gMeas.iab.s16Beta;
    fs->s16BemfAmp = (int16_t)Mx_Isqrt32((uint32_t)fs->s32BemfAmp2);    /* 每拍一次逐位开方，约 16 次循环 */

    /* 鉴相：短路电流矢量与估计角的叉积 */
    a = Mc_SinCos(fs->s16PosElEst_fly);
    fs->sAngle_fly.s16Cos = a.s16Cos;
    fs->sAngle_fly.s16Sin = a.s16Sin;
    err = (int16_t)(MLIB_Mul_Q15((int16_t)-gMeas.iab.s16Alpha, a.s16Cos) - MLIB_Mul_Q15(gMeas.iab.s16Beta, a.s16Sin));
    fs->s16ThetaErr_fly = err;
    fs->s16PosElEst_fly = trackPll(err, &gEst.pll);
}
