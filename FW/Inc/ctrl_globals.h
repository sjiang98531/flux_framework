/* 与闭源库共享的全局对象（定义在 ctrl_globals.c）。名字和类型是 ABI，不能改。 */
#ifndef CTRL_GLOBALS_H
#define CTRL_GLOBALS_H
#include "motor_structure.h"

extern USER_TypeDef     USER;           /* 原厂"用户参数 + 杂项状态"结构体 */
extern FOC_TypeDef      FOC;            /* 原厂 FOC 主结构体 */
extern FastBoot_TypeDef FastBoot_Para;  /* Fast_Boot_StartUp/Spin 的判据 */
extern int16_t          M1FaultID;      /* 当前故障位 */
extern int16_t          M1FaultID_Record; /* 上电以来出现过的故障位 */
#endif
