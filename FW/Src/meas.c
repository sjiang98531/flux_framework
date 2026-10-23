/* 测量量慢环滤波，见 meas.h。 */
#include "meas.h"

Meas_TypeDef gMeas;

void Meas_SlowFilter(void)
{
    gMeas.vbusAvg = MLIB_FilterMA_Q15(gMeas.vbus, &gMeas.vbusFilt);
    gMeas.speed = MLIB_FilterMA_Q15(gMeas.speedRaw, &gMeas.speedFilt);
    if (gMeas.ibus <= 0) gMeas.ibusAvg = 0;
    else gMeas.ibusAvg = MLIB_FilterMA_Q15(gMeas.ibus, &gMeas.ibusFilt);
    /* 慢环，允许除法。母线没电（上电瞬间）时不更新，避免除以 0 */
    if (gMeas.vbusAvg > 0) gMeas.kvbus = (int16_t)(MEAS_KVBUS_NUM / (int32_t)gMeas.vbusAvg);
}
