/* ============================================================================
 * @file    spin.h
 * @brief   闭环运行（Run 子状态 Spin），实现见 spin.c
 * ==========================================================================*/
#ifndef SPIN_H
#define SPIN_H
#include <stdint.h>

void    Spin_Fast(void);            /* 快环：估计 + 电流环 + 调制 */
void    Spin_Slow(void);            /* 慢环：速度环、MTPA、融合、失锁 / 回退判断、保护 */
void    Spin_Reset(void);           /* 停机 / 故障复位（Ctrl_ResetRunVariables） */
void    Spin_OnEnter(void);         /* I/F 切闭环时调用（Sup_SwitchStartupSpin，临界区内） */
void    Spin_OnEnterCatch(void);    /* 飞车直接接管进入 Spin 时调用（先零电流跟踪） */
uint8_t Spin_Tracking(void);        /* 1 = 零电流跟踪中 */
void    Spin_TrackBegin(void);      /* 进入零电流跟踪（闭环中停机、软停电流降到 0 后） */
void    Spin_TrackSlow(void);       /* 零电流跟踪的慢环 */

#endif
