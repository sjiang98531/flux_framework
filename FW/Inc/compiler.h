/* ============================================================================
 * @file    compiler.h
 * @brief   编译器相关关键字的唯一定义处
 *
 * FW/ 中不直接使用 __forceinline、__attribute__ 等编译器扩展，更换编译器时只需修改本文件。
 *   FORCE_INLINE  强制内联的 static 函数。快环中的小函数必须内联：Keil -O1 对普通 static inline 不保证内联，
 *                 未内联时每个文件各生成一份副本，且 RAM 中的快环调用 Flash 函数还需长跳转。
 *   RAMFUNC       将函数放入 ramfunc 段，从 RAM 执行。72 MHz 下 Flash 有 2 个等待周期（Board/Src/SC_Init.c，
 *                 RCC_CFG0 WAIT = 2），M0+ 无缓存；SRAM 为 0 等待。链接脚本 Project/flux_framework.sct 将该段
 *                 放在 RW_IRAM1，上电时由 __main 与已初始化变量一起从 Flash 复制。只标注 Spin 快环每拍执行的函数
 *                 （约 4.6 KB）。主机编译（gcc）及静态检查（CFG_STATIC_CHECK）时定义为空。
 *                 FASTPATH_IN_RAM（cfg_board.h）= 0 时同样为空：全部代码留在 Flash
 *                 （快环约慢 27.5%，节省约 4.7 KB RAM）。
 * ==========================================================================*/
#ifndef COMPILER_H
#define COMPILER_H
#include "cfg_board.h"      /* FASTPATH_IN_RAM */

#if defined(__ARMCC_VERSION)                    /* Keil AC5 / AC6 */
#define FORCE_INLINE static __forceinline
#elif defined(__GNUC__)                         /* gcc / clang（主机编译） */
#define FORCE_INLINE static inline __attribute__((always_inline))
#else
#define FORCE_INLINE static inline
#endif

#if defined(__ARMCC_VERSION) && !defined(CFG_STATIC_CHECK) && FASTPATH_IN_RAM
#define RAMFUNC __attribute__((section("ramfunc")))
#else
#define RAMFUNC
#endif

#endif
