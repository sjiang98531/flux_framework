/*******************************************************************************
 * 版权所有 (C) 2023, COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD
 *
 * 文件名称: inlcudes.h
 * 内容摘要: 数据类型参数和系统相关头文件
 * 其它说明: 无
 * 当前版本: V1.0
 * 完成日期: 2024年1月10日
 *
 *******************************************************************************/

/*------------------------------prevent recursive inclusion -------------------*/
#ifndef __INCLUDES_H
#define __INCLUDES_H
#include "stdint.h"

//整定 __END/////////////////////////////////

/*------------------------头文件包含相关定义------------------------*/
#include "sc32_conf.h"
/* ADC DMA 缓冲区声明（定义在 FW/Src/hal_sc32r602.c） */
extern __align(4) u16 ADC_DMABuff[14];
#include "Hardware_config.h"
#include "SC_Init.h"



#endif /* __INCLUDES_H */
/*********** (C) COPYRIGHT SHENZHEN SINONE MICROELECTIONCS CO., LTD  *************/
/* ------------------------------END OF FILE------------------------------------ */


