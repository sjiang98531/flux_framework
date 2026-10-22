/* 与闭源库共享的全局对象。名字、类型、字段布局都是 ABI，见 vendor_lib.h。 */
#include "ctrl_globals.h"

USER_TypeDef     USER;
FOC_TypeDef      FOC;
FastBoot_TypeDef FastBoot_Para;
int16_t          M1FaultID;
int16_t          M1FaultID_Record;
