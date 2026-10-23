/* ============================================================================
 * protect.h —— 运行中保护（自己实现，替代原厂闭源库的 MCF_ADV_Protect）
 *
 * Spin 慢环每拍调用。结果写进 FOC.sProtect 的标志位，由 supervisor 处理：
 *   bRotor_block  堵转：|转速| < u16BlockSpeed 且 |Vdq|^2 < u16BlockVoltageAmp_square，
 *                 持续 u16BlockTimeCmd 拍
 *   bErr_Spin     反转失控：转速与斜坡指令反号 且 |Vdq|^2 小，持续 u16BlockTimeCmd 拍
 *   bPhaseless    缺相：每 u16PhaselessTimeCmd 拍比较一次三相电流绝对值积分
 *                 （快环累加 u32Ia/b/c_Integral），最大 > 3 倍最小 就判缺相
 *   bSpeed_OC     超速：|转速| > u16Speed_OC_cmd
 * 门限在 supervisor.c 的 initProtection() 里由 cfg_ctrl.h 的物理量算出。
 * 行为与原厂库逐位相同。
 * ==========================================================================*/
#ifndef PROTECT_H
#define PROTECT_H
#include "mc_types.h"

void Protect_SpinSlow(FOC_TypeDef *f);

#endif
