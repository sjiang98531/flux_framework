/* ============================================================================
 * @file    main.c
 * @brief   上电初始化 + 后台主循环
 *
 * 快环位于 ADC 中断（hal_sc32r602.c），周期 62.5 us，优先级 1；
 * 慢环位于 PendSV 中断，由快环每 16 拍挂起一次（1 kHz），优先级 3，可被快环抢占。
 * 主循环仅在两个中断均未运行时执行，处理非实时任务。
 * ==========================================================================*/
#include "hal.h"
#include "supervisor.h"
#include "param.h"

/* 转速换算辅助：设置 Target_Speed（rpm）后，Run_Speed_Target_rpm 给出对应的 Q15 指令值，
 * 可写入 gSup.speedCmd。 */
uint16_t Run_Speed_Target_rpm = 0;
float Target_Speed = 2.0f;
static float s_lastTarget = -1.0f;

/**
 * @brief  程序入口：上电初始化后进入主循环，不返回。
 *         控制全部在中断中执行：快环 ADC 中断（电流环、观测器）、慢环 PendSV（速度环、状态判断），
 *         主循环只处理非实时的后台任务。
 *         PWM_DEBUG = 1（cfg_board.h）时不运行控制，只输出固定占空比，用于检查驱动板。
 */
int main(void)
{
    HAL_InitHardware();         /* 外设初始化（hal.h） */
    Ctrl_Init();                /* 全部控制模块参数初始化 */

#if (PWM_DEBUG == 0)
    HAL_StartInterrupts();      /* 开中断，快环开始运行（慢环由快环挂起） */
#else
    HAL_PwmDebugLoop();         /* 驱动板自检：三相 25% 占空比，不运行控制（hal_sc32r602.c） */
#endif

    while (1) {
        Param_Background();     /* Stop 状态下应用辨识结果（param.h） */
        /* 仅在 Target_Speed 变化时换算（软浮点）。慢环在中断中运行，不受影响 */
        if (Target_Speed != s_lastTarget) {
            s_lastTarget = Target_Speed;
            Run_Speed_Target_rpm = Q15(Target_Speed / SPEED_MAX);
        }
    }
}
