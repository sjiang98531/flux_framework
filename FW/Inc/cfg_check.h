/* ============================================================================
 * @file    cfg_check.h
 * @brief   配置检查（仅由 diag.c 包含一次）
 *
 * 检查清单见 cfg_check_list.h。每项条件在编译期计算为常量（作为静态数组的初始化式，
 * 标准 C 要求编译期求值），上电时由 Diag_Init() 遍历：
 *   gFluxDiag.cfgFailLine  = 第一个不满足的检查项在 cfg_check_list.h 中的行号（0 = 全部通过）
 *   gFluxDiag.cfgFailCount = 不满足的项数
 * 任一项不满足时，supervisor 持续报 MC_FAULT_CFG（8），电机不会起动。
 *
 * 未直接采用编译报错的原因：检查条件均为浮点运算，armcc 5 不保证能在 #if 或数组长度中计算浮点。
 * 使用 clang 编译并定义 CFG_STATIC_CHECK 时，不满足的项会直接产生编译错误。
 * ==========================================================================*/
#ifndef CFG_CHECK_H
#define CFG_CHECK_H

#include "cfg_derived.h"
#include "identify.h"

/* 每项：满足 = 0，不满足 = 该项在 cfg_check_list.h 里的行号 */
#define CFG_CHECK(name, cond)  ((cond) ? 0 : __LINE__),
static const uint16_t s_fwCfgCheck[] = {
#include "cfg_check_list.h"
    0
};
#undef CFG_CHECK

#if defined(__clang__) && defined(CFG_STATIC_CHECK)
/* clang 可将浮点条件折叠为常量，不满足的项直接产生编译错误 */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-folding-constant"
#define CFG_CHECK(name, cond)  typedef char cfg_check_##name[(cond) ? 1 : -1];
#include "cfg_check_list.h"
#undef CFG_CHECK
#pragma clang diagnostic pop
#endif

#endif
