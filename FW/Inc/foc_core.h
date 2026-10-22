/* ============================================================================
 * foc_core.h —— FOC 核心：坐标变换、电流环、SVPWM、电压重建
 *
 * 电流环 PI、SVPWM 用原厂闭源库（Fast_Boot_Current_Controller / PWM_GEN_calc），
 * 这里负责调用顺序和参数。增益由 cfg_ctrl.h 的 CURRENT_BW_HZ 自动算。
 * ==========================================================================*/
#ifndef FOC_CORE_H
#define FOC_CORE_H
#include "fw_config.h"
#include "vendor_lib.h"

/* 电流环、母线补偿等参数初始化（写 FOC.CurID/CurIQ、USER.s16DutyLimitCmd 等） */
void FocCore_Init(void);

/* i_abc -> i_alpha_beta */
void FocCore_Clarke(FOC_TypeDef *f);

/* 电流环：Park -> PI（d/q）-> 反 Park -> 母线纹波补偿。角度用 f->s16CmdTheta。
 * 原厂函数名 FOC_Current_Controller，逻辑不变。 */
void FOC_Current_Controller(FOC_TypeDef *pFOC);

/* SVPWM：f->sVAlBe_cmd -> pwm_gen -> 比较寄存器。mode = SEVENMODE / FIVEMODE */
void FocCore_Modulate(const FOC_TypeDef *f, uint8_t mode);

/* 输出零电压矢量（50% 占空比），状态切换时用 */
void FocCore_ModulateZero(void);

/* 由上一拍的三相比较值和母线电压重建 alpha/beta 电压（Q15，FLUX_V_BASE 基准）。
 * 不含死区（死区补偿试过两次都更差，见 flux_selfpll 实验记录 test12/16）。 */
void FocCore_VoltageFromCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC, int16_t bus,
                                 int16_t *va, int16_t *vb);

#endif
