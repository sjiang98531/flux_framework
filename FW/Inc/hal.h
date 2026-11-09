/* ============================================================================
 * @file    hal.h
 * @brief   硬件抽象层接口（算法层只通过本接口访问硬件）
 *
 * 内容：电流 / 母线采样、PWM 比较值、桥臂开关、定时器、临界区、上电初始化。芯片相关部分分为两处：
 *   hal_sc32r602.h  快环热路径上必须以宏 / 内联实现的部分（桥臂开关、IRQ_LOCK），
 *                   芯片头文件只经由它包含；
 *   hal_sc32r602.c  其余接口的实现及中断入口；外设初始化在 Board/Src/SC_Init.c，
 *                   引脚与通道宏在 Board/Inc/Hardware_config.h。
 * 移植到其他芯片：编写 hal_<芯片>.h / .c，并修改下方一行 include；算法层（FW/ 其余文件）无需改动。
 * FW/ 中除上述文件外，不得包含芯片头文件或直接访问寄存器。
 * ==========================================================================*/
#ifndef HAL_H
#define HAL_H
#include "cfg_derived.h"
#include "compiler.h"
#include "qtypes.h"
#include "hal_sc32r602.h"      /* 芯片移植头文件：更换芯片时修改此行 */

/* 移植头文件必须提供（快环热路径，宏 / 内联）：
 *   HAL_PwmAllOff/On()、HAL_PwmHighOff/On()、HAL_PwmLowOff/On()、HAL_PwmLowOnA/B/C()   桥臂开关
 *   IRQ_LOCK() / IRQ_UNLOCK()   临界区。慢环中的状态切换需同时修改 runState、电流指令、积分器、开环角等，
 *                               而快环可随时打断慢环；若切换中途被打断，快环会以"新状态 + 旧指令"运行一到数拍。
 *                               因此所有由慢环调用的状态切换都整体置于临界区内；快环中断只被推迟几微秒，不会丢失。 */

/* 三相比较值均设为半周期（50% 占空比） */
void HAL_PwmSetHalfDuty(void);
/* 写三相比较值（下一个 PWM 周期生效；相序由 RUN_DIR 决定） */
void HAL_PwmWriteCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC);
/* 飞车检测：上桥全关、下桥全开（三相对地短接），三相比较值 = cmp */
void HAL_PwmShortLowSides(uint16_t cmp);
/* FG 输出（TIM0 PWMA）占空比 */
void HAL_FgSetDuty(uint32_t duty);
/* HAL 参数 */
typedef struct {
    uint16_t pwmPeriod;             /* EPWM 周期计数 = SYSCLK / 2 / PWM_FREQ */
    uint16_t slowLoopDiv;           /* 快环拍数 / 慢环拍 */
    uint16_t caliDuty;              /* 飞车检测短接下桥时的比较值 */
    uint16_t adcNoisePk;            /* 上电零点校准时测到的电流采样噪声峰值（ADC 码，两相取大，含 1 码量化） */
    volatile uint8_t ocLatch;       /* 硬件过流锁存：EPWM 刹车中断置 1，快环故障检测取走（HAL_TakeOcLatch） */
} Hal_TypeDef;

extern Hal_TypeDef gHal;

/* 硬件过流锁存：取走并清除（快环故障检测调用），1 = 期间发生过过流 */
FORCE_INLINE uint8_t HAL_TakeOcLatch(void)
{
    uint8_t f = gHal.ocLatch;
    if (f) gHal.ocLatch = 0;
    return f;
}
/* 清除锁存（Stop -> Run：停机期间的过流标志不带入新的运行） */
FORCE_INLINE void HAL_ClearOcLatch(void)
{
    gHal.ocLatch = 0;
}

/* 上电初始化，main() 按以下顺序调用：
 *   HAL_InitHardware()    外设（时钟、GPIO、ADC、运放、EPWM、定时器、DMA），此时中断尚未开启
 *   HAL_InitParams()      HAL 自身参数（PWM 周期、慢环分频，写入 gHal），由 Ctrl_Init 调用
 *   HAL_StartInterrupts() 配置优先级（硬件过流 > 快环 > 慢环）后开中断，快环开始运行 */
void HAL_InitHardware(void);    /* 外设初始化（Board/Src/SC_Init.c） */
void HAL_InitParams(void);    /* PWM 周期、慢环分频、短接占空比 */
void HAL_StartInterrupts(void);    /* 配置中断优先级并开中断 */
/* 驱动板自检（cfg_board.h PWM_DEBUG = 1）：三相固定 25% 占空比，不运行控制，不返回 */
void HAL_PwmDebugLoop(void);

#endif
