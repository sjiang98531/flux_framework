/* ============================================================================
 * flystart.h —— 飞车（转子已在转）检测（自己实现，替代原厂闭源库的 MCF_FlyStart_2R）
 *
 * 在 Calib 子状态的快环里每拍调用：
 *   1. 上桥全关、下桥全开（三相对地短接），转子反电势在绕组里产生短路电流
 *   2. 短路电流的 αβ 矢量 = 反电势 / 阻抗，方向随转子转：
 *        幅值 sFly_Start.s16BemfAmp = |i_αβ|（> s16BemfAmpTh 认为在转）
 *        角度用一个 PLL 跟踪（sFly_Start.s16PosElEst_fly），PLL 的转速就是转子转速
 *   3. supervisor 据此决定：静止 -> 定位；同向在转 -> Torqueloop 接管；
 *      逆向在转（I/F 模式）-> 继续短接制动
 * 已知限制（台架 2026-10-23）：几十 rpm 时 PLL 判的转向可能是错的。
 * 行为与原厂库逐位相同；跟踪用的 PLL 是原厂 MLIB_PLL 的语义（与 flux_pll 不同），
 * 只在这里用。
 * ==========================================================================*/
#ifndef FLYSTART_H
#define FLYSTART_H
#include "mc_types.h"

void FlyStart_CalibFast(FOC_TypeDef *f);

#endif
