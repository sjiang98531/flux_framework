/* 运行中保护，见 protect.h。 */
#include "protect.h"
#include "meas.h"
#include "foc_core.h"
#include "speed_ctrl.h"

Protect_TypeDef gProt;

void Protect_SpinSlow(void)
{
    Protect_TypeDef *p = &gProt;
    uint32_t mn, mx;

    /* |Vdq|^2（Q15）：电压很小说明反电势很小，即转子基本没转起来 */
    p->u16VoltageTemp = (uint16_t)(MLIB_Mul_Q15(gFoc.vdq.s16D, gFoc.vdq.s16D) + MLIB_Mul_Q15(gFoc.vdq.s16Q, gFoc.vdq.s16Q));

    /* 堵转 */
    if (MLIB_Abs_Q15(gMeas.speed) < (int32_t)p->u16BlockSpeed && p->u16VoltageTemp < p->u16BlockVoltageAmp_square) {
        if (++p->u16BlockCounter > p->u16BlockTimeCmd) { p->u16BlockCounter = 0; p->bRotor_block = 1; }
    } else {
        p->u16BlockCounter = 0;
    }

    /* 反转失控：估计转速和指令方向相反，而且电压也小 */
    if (MLIB_Mul_Q15(gSpd.ramp, gMeas.speed) < 0 && p->u16VoltageTemp < p->u16BlockVoltageAmp_square) {
        if (++p->u16FspinCounter > p->u16BlockTimeCmd) { p->bErr_Spin = 1; p->u16FspinCounter = 0; }
    } else {
        p->u16FspinCounter = 0;
    }

    /* 缺相：一个窗口内三相电流绝对值积分，最大的超过最小的 3 倍 */
    if (++p->u16PhaselessTimeCnt > p->u16PhaselessTimeCmd) {
        mn = p->u32Ia_Integral < p->u32Ib_Integral ? p->u32Ia_Integral : p->u32Ib_Integral;
        if (mn >= p->u32Ic_Integral) mn = p->u32Ic_Integral;
        mx = p->u32Ia_Integral > p->u32Ib_Integral ? p->u32Ia_Integral : p->u32Ib_Integral;
        if (mx <= p->u32Ic_Integral) mx = p->u32Ic_Integral;
        if (mx > mn * 3) p->bPhaseless = 1;
        p->u16PhaselessTimeCnt = 0;
        p->u32PhaselessMaxTemp = 0;
        p->u32PhaselessMinTemp = 0;
        p->u32Ia_Integral = 0;
        p->u32Ib_Integral = 0;
        p->u32Ic_Integral = 0;
    }

    /* 超速 */
    if (MLIB_Abs_Q15(gMeas.speed) > (int32_t)p->u16Speed_OC_cmd) p->bSpeed_OC = 1;
}
