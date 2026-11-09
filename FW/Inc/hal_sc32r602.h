/* ============================================================================
 * @file    hal_sc32r602.h
 * @brief   HAL 芯片移植头文件（SC32R602 + 本驱动板），仅由 hal.h 包含
 *
 * hal.h 中要求由各芯片自行提供的部分（位于快环热路径，必须为宏 / 内联，不能是函数调用）：
 *   桥臂开关 HAL_Pwm*On/Off、临界区 IRQ_LOCK / IRQ_UNLOCK。
 *   芯片头文件（includes.h -> sc32_conf.h、Hardware_config.h）也只经由本文件包含。
 * 移植到其他芯片：参照本文件编写 hal_<芯片>.h + hal_<芯片>.c，并修改 hal.h 中的 include。
 * FW 其余文件不得直接包含芯片头文件或访问寄存器。
 * ==========================================================================*/
#ifndef HAL_SC32R602_H
#define HAL_SC32R602_H
#include "compiler.h"
#include "includes.h"          /* sc32_conf.h（寄存器）、Hardware_config.h（桥臂开关宏、定时器装载值） */

/* ---- 桥臂开关（直接映射 Hardware_config.h 的宏，零开销） -------------------- */
#define HAL_PwmAllOff()         do { DISABLE_PWMOUT(); } while (0)   /* 上下桥全关 */
#define HAL_PwmAllOn()          do { ENABLE_PWMOUT(); } while (0)    /* 上下桥互补输出 */
#define HAL_PwmHighOff()        do { PWMABC_Dis; } while (0)
#define HAL_PwmHighOn()         do { PWMABC_Enb; } while (0)
#define HAL_PwmLowOff()         do { PWMABCN_Dis; } while (0)
#define HAL_PwmLowOn()          do { PWMABCN_Enb; } while (0)
#define HAL_PwmLowOnA()         do { PWMAN_Enb; } while (0)
#define HAL_PwmLowOnB()         do { PWMBN_Enb; } while (0)
#define HAL_PwmLowOnC()         do { PWMCN_Enb; } while (0)

/* ---- 临界区：Cortex-M PRIMASK 保存 / 恢复（在快环中调用或嵌套调用均安全）。更换内核时只需修改这两行 ---- */
#define IRQ_LOCK()              uint32_t irqPm_ = __get_PRIMASK(); __disable_irq()
#define IRQ_UNLOCK()            __set_PRIMASK(irqPm_)

#endif
