#ifndef __USER_SUB_H
#define __USER_SUB_H

#include <stdint.h>
#include "motor_structure.h"

extern USER_TypeDef	USER;
extern FOC_TypeDef	FOC;
extern FastBoot_TypeDef FastBoot_Para;
extern int16_t M1FaultID;
extern int16_t M1FaultID_Record;
extern uint16_t PeriodTime;

extern void variable_reset(void);
extern void Init_Parameter(void);


#endif
