/* 测量量慢环滤波，见 meas.h。 */
#include "meas.h"
#include "ctrl_globals.h"

void Meas_SlowFilter(void)
{
    FOC.s16VbusAvg = MLIB_FilterMA_Q15(FOC.s16Vbus, &USER.sVbus_Filt);
    FOC.s16SpdFilt = MLIB_FilterMA_Q15(FOC.FluxFeedback.s16Speed, &USER.sSpeed_Filt);
    if (FOC.s16Ibus <= 0) FOC.s16IbusAvg = 0;
    else FOC.s16IbusAvg = MLIB_FilterMA_Q15(FOC.s16Ibus, &USER.sIbus_Filt);
    /* 慢环，允许除法。母线没电（上电瞬间）时不更新，避免除以 0 */
    if (FOC.s16VbusAvg > 0) FOC.s16KVbus = (int16_t)(MEAS_KVBUS_NUM / (int32_t)FOC.s16VbusAvg);
}
