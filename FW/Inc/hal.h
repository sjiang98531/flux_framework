/* ============================================================================
 * hal.h —— 硬件抽象层（SC32R602 + 本驱动板）
 *
 * 算法层只通过这里碰硬件：电流/母线采样、PWM 比较值、桥臂开关、定时器。
 * 实现在 hal_sc32r602.c；外设初始化在 Board/Src/SC_Init.c；引脚与通道
 * 宏在 Board/Inc/Hardware_config.h。换驱动板只改这三处 + cfg_board.h。
 * ==========================================================================*/
#ifndef HAL_H
#define HAL_H
#include "fw_config.h"
#include "includes.h"          /* sc32_conf.h、Hardware_config.h（桥臂开关宏） */
#include "mc_types.h"
#include "svpwm.h"

/* ---- 桥臂开关（直接映射 Hardware_config.h 的宏，零开销） -------------------- */
#define HAL_PwmAllOff()         do { DISABLE_PWMOUT(); } while (0)   /* 上下桥全关 */
#define HAL_PwmAllOn()          do { ENABLE_PWMOUT(); } while (0)    /* 上下桥互补输出 */
#define HAL_PwmHighOff()        do { PWMABC_Dis; } while (0)
#define HAL_PwmHighOn()         do { PWMABC_Enb; } while (0)
#define HAL_PwmLowOff()         do { PWMABCN_Dis; } while (0)
#define HAL_PwmLowOn()          do { PWMABCN_Enb; } while (0)
#define HAL_PwmLowOnA()         do { PWMAN_Enb; } while (0)
#define HAL_PwmLowOnB()         do { PWMBN_Enb; } while (0)
#define HAL_PwmLowOnC()         do { PWMCN_Enb; } while (0)

/* 三相比较值都设成半周期（50% 占空比） */
void HAL_PwmSetHalfDuty(void);
/* 三相上下桥是否全部使能（观测器只在真正输出 PWM 时信任重构电压）。快环热路径，内联。 */
static __forceinline uint8_t HAL_PwmAllEnabled(void)
{
    return (uint8_t)((EPWM->EPWM_CHN & 0x3fU) == 0x3fU);
}
/* 把 pwm_gen.CompA/B/C 写进比较寄存器（相序由 RUN_DIR 决定） */
void HAL_PwmWriteCompares(const Svpwm_TypeDef *u);
/* 飞车检测：上桥全关、下桥全开（三相对地短接），三相比较值 = cmp */
void HAL_PwmShortLowSides(uint16_t cmp);
/* FG 输出（TIM0 PWMA）占空比 */
void HAL_FgSetDuty(uint32_t duty);
/* 快环计时用的自由计数器（TIM3，36 MHz） */
uint32_t HAL_TimerNow(void);
/* HAL 的参数和慢环标志 */
typedef struct {
    uint16_t pwmPeriod;             /* EPWM 周期计数 = SYSCLK / 2 / PWM_FREQ */
    uint16_t slowLoopDiv;           /* 快环拍数 / 慢环拍 */
    volatile bool_t slowLoopFlag;   /* 快环置 1，main() 清 0 后跑一次慢环 */
    uint16_t caliDuty;              /* 飞车检测短接下桥时的比较值 */
    uint8_t  speedUpdateDiv;        /* 外环每几个慢环拍跑一次 */
} Hal_TypeDef;

extern Hal_TypeDef gHal;

/* 上电后 HAL 自己的参数（PWM 周期、慢环分频，写进 gHal） */
void HAL_InitParams(void);

#endif
