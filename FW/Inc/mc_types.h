/* ============================================================================
 * mc_types.h —— 公共类型与基础函数的统一入口
 *
 * 从 0xF20E0006 起框架不再链接原厂闭源库 SOC_MCI_2R_V102.lib，原来库里的函数
 * 全部自己实现：
 *   MLIB_SinCos / Sqrt_Q15 / MLIB_Ramp_Q31  -> mc_math.h  Mc_SinCos / Mc_SqrtQ15 / Mc_RampQ31
 *   PWM_GEN_calc / PWM_GEN_init / pwm_gen   -> svpwm.c    Svpwm_Calc / Svpwm_Init / pwm_gen
 *   DcBus_Ripple_suppress                   -> foc_core.h FocCore_BusNormalize
 *   Var_Filt                                -> meas.c     Meas_SlowFilter
 *   MCF_ADV_Protect                         -> protect.c  Protect_SpinSlow
 *   MCF_FlyStart_2R（含 MLIB_PLL）          -> flystart.c FlyStart_CalibFast
 *   Fast_Boot_Spin / Fast_Boot_StartUp      -> supervisor.c / startup.c 里的局部函数
 *   CalcPI / Fast_Boot_Current_Controller   -> ctrl_pi.h、foc_core.c（早已自写，原厂路径已删）
 *
 * 仍然使用原厂的"开源"部分（头文件，源码可见）：
 *   mlib_types.h / mlib.h   Q15 类型、内联的 MLIB_Mul_Q15、MLIB_FilterMA_Q15 等
 *   mcflib.h                内联的 Clarke / Park / 反 Park
 *   pwm_gen.h               PWM_GEN_T 结构体、SEVENMODE / FIVEMODE
 *   motor_structure.h       FOC_TypeDef / USER_TypeDef（原厂用户代码里的结构体）
 * 这些头文件里还留着闭源函数的声明（MLIB_SinCos、Var_Filt……），没有实现，
 * 谁再调用就会链接失败——这正是想要的。
 *
 * pwm_gen.h 没有 include guard，所以只能通过本文件包含。
 * 结构体布局暂时没动（J-Scope 地址不变）；不再受闭源库约束，以后可以重新整理。
 * ==========================================================================*/
#ifndef MC_TYPES_H
#define MC_TYPES_H
#include "motor_structure.h"
#include "mlib.h"
#include "mcflib.h"
#include "pwm_gen.h"
#include "mc_math.h"
#endif
