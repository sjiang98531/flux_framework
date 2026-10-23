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
 *   mlib_types.h / mlib.h   Q15 类型、内联的 MLIB_Mul_Q15、MLIB_FilterMA_Q15，
 *                           以及 tPIParm（PI）、PLL_TypeDef（PLL）两个结构体
 *   mcflib.h                内联的 Clarke / Park / 反 Park
 * 这些头文件里还留着闭源函数的声明（MLIB_SinCos……），没有实现，
 * 谁再调用就会链接失败——这正是想要的。
 *
 * 从 0xF20E0007 起不再使用原厂的 FOC / USER / FastBoot_Para 大结构体（motor_structure.h）
 * 和 pwm_gen.h：状态按模块分开，每个模块一个全局变量，定义在模块自己的 .c 里：
 *   gMeas   meas.h        电流、母线、转速反馈及滤波
 *   gFoc    foc_core.h    电流环（角度、dq 电流/电压、PI）
 *   gEst    est.h         磁链 PLL
 *   gSpd    speed_ctrl.h  外环指令、斜坡、速度/功率 PI
 *   gProt   protect.h     保护门限和标志
 *   gFly    flystart.h    飞车检测
 *   gStartup startup.h    预充、定位、启动、Torqueloop 的计数和参数
 *   gSup    supervisor.h  状态机、速度指令、故障
 *   gHal    hal.h         PWM 周期、慢环分频
 *   pwm_gen svpwm.h       调制器
 * 新旧变量名对照见 重构说明.md。
 * ==========================================================================*/
#ifndef MC_TYPES_H
#define MC_TYPES_H
#include <stdint.h>
/* 芯片头文件（经 includes.h -> sc32_conf.h）必须先于 mlib_types.h：后者在没定义时会
 * #define TRUE/FALSE，再包含芯片头文件就和它的 boolType 枚举冲突。 */
#include "includes.h"
#include "mlib.h"
#include "mcflib.h"
#include "mc_math.h"
#endif
