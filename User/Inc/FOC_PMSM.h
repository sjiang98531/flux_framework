/* flux_framework: shim for the vendor header MCI/Inc/pwm_gen.h, which does
 * #include "FOC_PMSM.h" to get FOC_TypeDef. The old FOC_PMSM.c code now lives
 * in FW/Src/foc_core.c and FW/Src/est.c. Do not include pwm_gen.h from here
 * (it has no include guard); use FW/Inc/vendor_lib.h instead. */
#ifndef __FOC_PMSM_H
#define __FOC_PMSM_H
#include <stdint.h>
#include "motor_structure.h"
#endif
