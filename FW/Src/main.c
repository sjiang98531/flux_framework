/* ============================================================================
 * main.c —— 上电初始化 + 慢环轮询
 *
 * 快环在 ADC 中断里（hal_sc32r602.c），每 62.5 us 一次；
 * 慢环在这里轮询：快环每 16 拍置一次 gHal.slowLoopFlag（名义 1 kHz）。
 * 快环中断随时会打断慢环，两者不会"同时"执行。
 * ==========================================================================*/
#include "SC_Init.h"
#include "hal.h"
#include "supervisor.h"

/* 调试辅助：在 Watch 里改 Target_Speed，读下面三个量得到对应的 Q15 指令值，
 * 再写进 gSup.speedCmd（速度环用 Run_Speed_Target_rpm）。 */
uint16_t Run_Curr_Target_A = 0;
uint16_t Run_Speed_Target_rpm = 0;
uint16_t Run_Power_Target_W = 0;
float Target_Speed = 2.0f;

int main(void)
{
    Motor_Hardware_Init();      /* 外设初始化（Board/Src/SC_Init.c） */
    Ctrl_Init();                /* 全部控制模块参数初始化 */

#if (PWM_DEBUG == 0)
    Motor_NVIC_Config();        /* 开中断，快环开始跑 */
#else
    /* 驱动板自检：三相 25% 占空比，不跑控制 */
    PWMABCN_Dis;
    PWMABC_Dis;
    EPWM->EPWM_DT[0] = gHal.pwmPeriod >> 2;
    EPWM->EPWM_DT[2] = gHal.pwmPeriod >> 2;
    EPWM->EPWM_DT[4] = gHal.pwmPeriod >> 2;
    PWMAN_Enb;
    PWMBN_Enb;
    PWMCN_Enb;
    PWMA_Enb;
    PWMB_Enb;
    PWMC_Enb;
    while (1) {
        WDT->WDT_CON |= 0X01;   /* 喂狗 */
    }
#endif

    while (1) {
        if (gHal.slowLoopFlag == 1) {
            gHal.slowLoopFlag = 0;
            Ctrl_SlowTick();
        }
        Run_Curr_Target_A = Q15(Target_Speed / I_MAX);
        Run_Speed_Target_rpm = Q15(Target_Speed / SPEED_MAX);
        Run_Power_Target_W = Q15(Target_Speed / POWER_MAX);
    }
}
