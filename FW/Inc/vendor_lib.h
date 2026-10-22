/* ============================================================================
 * vendor_lib.h —— 原厂闭源库 SOC_MCI_2R_V102.lib 的唯一入口
 *
 * pwm_gen.h 没有 include guard，同一个 .c 里包含两次就会重复定义
 * PWM_GEN_T，所以框架里所有文件都只包含本头文件，不直接包含 pwm_gen.h。
 *
 * 闭源库与用户代码之间的"ABI 约定"（反汇编 test33 AXF 核实，改了会出事）：
 *   1. 全局变量名与类型不能改：FOC、USER、FastBoot_Para、pwm_gen、
 *      eM1_RunSubState（Fast_Boot_Current_Controller 读它，== 3 表示 Startup）。
 *   2. FOC_TypeDef / USER_TypeDef / FastBoot_TypeDef 的字段布局不能改
 *      （User/Inc/motor_structure.h）。库按字节偏移访问。
 *   3. 库函数读写的字段：
 *      Var_Filt        FOC.s16Vbus/FluxFeedback.s16Speed/s16Ibus -> s16VbusAvg/s16SpdFilt/s16IbusAvg/s16KVbus
 *      Fast_Boot_*     FOC.s16SpdFilt、s16SpdCmd、USER.u16StartupTimeCnt、FastBoot_Para
 *      Fast_Boot_Current_Controller  sIdq、sIdq_cmd、s16VMAX、CurID/CurIQ -> sVdq
 *                      （Startup 子状态下把 CurID 积分器预置成 -2000*iq_cmd）
 *      MCF_ADV_Protect sVdq、s16SpdFilt、s16SpeedRamp、sProtect
 *      MCF_FlyStart_2R sIabc、sIAlBe、sFly_Start、sPLL（用 MLIB_PLL）、直接写 EPWM 寄存器
 *      PWM_GEN_calc    pwm_gen、FOC.sIabc -> FOC.s16Ibusreconstructed
 * 新增状态一律放在自己模块的结构体里，不要往上面这些结构体里加字段。
 * ==========================================================================*/
#ifndef VENDOR_LIB_H
#define VENDOR_LIB_H
#include "motor_structure.h"
#include "mlib.h"
#include "mcflib.h"
#include "pwm_gen.h"
#endif
