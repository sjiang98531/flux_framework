/* 全局控制对象（定义在 ctrl_globals.c）。结构体沿用原厂用户代码 motor_structure.h。 */
#ifndef CTRL_GLOBALS_H
#define CTRL_GLOBALS_H
#include "motor_structure.h"

extern USER_TypeDef     USER;           /* 原厂"用户参数 + 杂项状态"结构体 */
extern FOC_TypeDef      FOC;            /* 原厂 FOC 主结构体 */
extern FastBoot_TypeDef FastBoot_Para;  /* 原厂启动切 Spin / Spin 回退的转速判据 */
extern int16_t          M1FaultID;      /* 当前故障位 */
extern int16_t          M1FaultID_Record; /* 上电以来出现过的故障位 */
#endif
