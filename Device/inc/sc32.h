/**
  ******************************************************************************
  * @file    Socmcu0003.h
  * @author  SOC AE Team
  * @version V1.1
  * @date    24-07-2023
  * @brief
  *
  */

/** @addtogroup CMSIS_Device
* @{
*/

/** @addtogroup sc32
  * @{
  */

#ifndef SC32_H
#define SC32_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);

#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SC32_H */

/**
  * @}
  */

/**
* @}
*/

/************************ (C) COPYRIGHT SIN ONE CHIP *****END OF FILE****/
