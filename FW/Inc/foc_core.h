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

/* i_abc -> i_alpha_beta（快环热路径，内联） */
static __forceinline void FocCore_Clarke(FOC_TypeDef *f)
{
    MCFLIB_Clark_Sat_S16(&f->sIabc, &f->sIAlBe);
}

/* 电流环：Park -> PI（d/q）-> 反 Park -> 母线纹波补偿。角度用 f->s16CmdTheta。
 * 原厂函数名 FOC_Current_Controller，逻辑不变。 */
void FOC_Current_Controller(FOC_TypeDef *pFOC);

/* d/q 电流 PI（自写，防溢出；CTRL_OWN_PI = 1 时 FOC_Current_Controller 调它） */
void FocCore_DqPI(FOC_TypeDef *f);

/* SVPWM：f->sVAlBe_cmd -> pwm_gen -> 比较寄存器。mode = SEVENMODE / FIVEMODE */
void FocCore_Modulate(const FOC_TypeDef *f, uint8_t mode);

/* 输出零电压矢量（50% 占空比），状态切换时用 */
void FocCore_ModulateZero(void);

/* 由上一拍的三相比较值和母线电压重建 alpha/beta 电压（Q15，FLUX_V_BASE 基准）。
 * 不含死区（死区补偿试过两次都更差，见 flux_selfpll 实验记录 test12/16）。
 * "比较值越小，相电压越高"：pole_x = (PERIOD - cmp_x)/PERIOD * Vbus，
 *   v_alpha = (2a - b - c)/3，v_beta = (b - c)/sqrt(3)。快环热路径，内联。 */
#define FOC_COUNT_SCALE ((int32_t)(1073741824.0f / PWM_PERIOD))     /* 2^30 / PWM_PERIOD */
static __forceinline void FocCore_VoltageFromCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC, int16_t bus,
                                                      int16_t *va, int16_t *vb)
{
    int32_t a, b, c, t;
    a = ((int32_t)(PWM_PERIOD - cmpA) * FOC_COUNT_SCALE) >> 15;
    b = ((int32_t)(PWM_PERIOD - cmpB) * FOC_COUNT_SCALE) >> 15;
    c = ((int32_t)(PWM_PERIOD - cmpC) * FOC_COUNT_SCALE) >> 15;
    a = (a * bus) >> 15; b = (b * bus) >> 15; c = (c * bus) >> 15;
    t = 2 * a - b - c;
    *va = (int16_t)((t * 21846 + 32768) >> 16);
    *vb = (int16_t)(((b - c) * 18919) >> 15);
}

#endif
