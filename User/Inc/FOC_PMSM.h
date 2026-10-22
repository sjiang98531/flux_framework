#ifndef __FOC_PMSM_H
#define __FOC_PMSM_H

#include <stdint.h>
#include "motor_structure.h"

extern void FOC_Current_Controller(FOC_TypeDef *pFOC);
extern void PMSM_Obs(FOC_TypeDef *pFOC);


#endif
