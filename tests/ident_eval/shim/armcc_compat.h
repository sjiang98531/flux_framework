/* Force-included on the host: armcc keywords. */
#define __align(x) __attribute__((aligned(x)))
#define __forceinline inline __attribute__((always_inline))
/* 主机上没有中断：开关中断是空操作 */
#define __get_PRIMASK() (0u)
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq() ((void)0)
