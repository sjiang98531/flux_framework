/* ============================================================================
 * hal_sc32r602.c —— SC32R602 驱动板的 HAL 实现 + 中断入口
 *
 * 快环 = ADC 序列 0 结束中断（16 kHz）：
 *   上电先采 255 拍算电流零点（第 1000~1254 拍），之后每拍：
 *   读电流/母线 -> Ctrl_FastTick()（状态机快环）-> 置慢环标志 -> 计时
 * 慢环不在中断里，由 main() 轮询 USER.bSlowLoopFlag 执行（见 main.c）。
 * 逻辑与 test33 的 SC_it.c 逐行对应。
 * ==========================================================================*/
#include "hal.h"
#include "ctrl_globals.h"
#include "supervisor.h"
#include "diag.h"
#include "SC_it.h"
#include "svpwm.h"

__align(4) u16 ADC_DMABuff[14];          /* ADC -> DMA 缓冲（SC_Init.c 配置 DMA 目标地址） */

static uint32_t s_u32IaSum, s_u32IbSum, s_u32IbusSum;
static uint16_t s_u16Cnt;                /* 上电拍数（零点校准用） */
static uint16_t s_u16IbusOffset;
static uint8_t  s_u8SlowDiv;

/* ---------------------------------------------------------------------------
 * 采样：返回 1 表示零点已校准、本拍采样已写进 FOC，可以跑控制。
 * 零点 = 256 个采样之和 >> 8（HAL_OFFSET_256；test33 是 255 个，偏小 1/256）。
 * -------------------------------------------------------------------------*/
static __forceinline uint8_t acquireSamples(void)
{
    int16_t ia, ib;
    s_u16Cnt++;
#if HAL_OFFSET_256
    if (s_u16Cnt < 999) {               /* 第 999~1254 拍：256 个点 */
#else
    if (s_u16Cnt < 1000) {              /* test33：第 1000~1254 拍，只有 255 个点 */
#endif
        return 0;
    } else if (s_u16Cnt < 1255) {
        s_u32IaSum += ADC_DMABuff[0] << 3;
        s_u32IbSum += ADC_DMABuff[1] << 3;
        s_u32IbusSum += ADC_DMABuff[2] << 3;
        return 0;
    } else if (s_u16Cnt == 1255) {
        FOC.sIabc_offset.s16A = s_u32IaSum >> 8;
        FOC.sIabc_offset.s16B = s_u32IbSum >> 8;
        s_u16IbusOffset = s_u32IbusSum >> 8;
        s_u32IaSum = 0; s_u32IbSum = 0; s_u32IbusSum = 0;
        return 0;
    }
    s_u16Cnt = 1256;
    ia = (ADC_DMABuff[0] << 3);                    /* 12 位 ADC 左移 3 位 -> Q15 */
    ib = (ADC_DMABuff[1] << 3);
#if (RUN_DIR == RUN_CW)
    FOC.sIabc.s16A = (FOC.sIabc_offset.s16A - ia) << 1;   /* <<1：Vdd/2 对应 I_MAX */
    FOC.sIabc.s16B = (FOC.sIabc_offset.s16B - ib) << 1;
#elif (RUN_DIR == RUN_CCW)
    FOC.sIabc.s16B = (FOC.sIabc_offset.s16A - ia) << 1;
    FOC.sIabc.s16A = (FOC.sIabc_offset.s16B - ib) << 1;
#endif
    FOC.sIabc.s16C = -FOC.sIabc.s16B - FOC.sIabc.s16A;
#if (IBUS_RECONSTRUCTED == 1)
    FOC.s16Ibus = FOC.s16Ibusreconstructed;
#else
    FOC.s16Ibus = ((ADC_DMABuff[2] << 3) - s_u16IbusOffset) * 2;
#endif
    FOC.s16Vbus = (ADC_DMABuff[3] << 3);
    return 1;
}

/* 硬件过流（比较器逐周期保护）中断 */
void EPWM_IRQHandler(void)
{
    if ((EPWM->EPWM_STS & EPWM_STS_CBCIF) != 0) {
        DISABLE_PWMOUT();
        MC_FAULT_SET(M1FaultID, MC_FAULT_I_DCBUS_OVER);
        M1FaultID_Record |= M1FaultID;
        EPWM->EPWM_STS |= EPWM_STS_CBCIF;
    }
}

/* 快环 */
void ADC_IRQHandler(void)
{
    uint32_t t0 = TIM3->TIM_CNT;
    if (ADC_GetFlagStatus(ADC, ADC_Flag_EOSIF0) != RESET) {
        ADC_ClearFlag(ADC, ADC_Flag_EOSIF0);
        if (acquireSamples()) {
            Ctrl_FastTick();
            if (++s_u8SlowDiv >= USER.u16SlowLoopDiv) {
                s_u8SlowDiv = 0;
                USER.bSlowLoopFlag = 1;
            }
            Diag_IsrTiming(t0, TIM3->TIM_CNT);
        }
    }
}

void TIMER3_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_Flag_TI) != RESET) TIM_ClearFlag(TIM3, TIM_Flag_TI);
}

void TIMER0_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM0, TIM_Flag_TI) != RESET) TIM_ClearFlag(TIM0, TIM_Flag_TI);
}

/* ---------------------------------------------------------------------------
 * PWM
 * -------------------------------------------------------------------------*/
void HAL_PwmSetHalfDuty(void)
{
    EPWM->EPWM_DT[0] = USER.u16PWMPeriod >> 1;
    EPWM->EPWM_DT[2] = USER.u16PWMPeriod >> 1;
    EPWM->EPWM_DT[4] = USER.u16PWMPeriod >> 1;
}

void HAL_PwmWriteCompares(const PWM_GEN_T *u)
{
#if (RUN_DIR == RUN_CW)
    EPWM->EPWM_DT[4] = u->CompA;
    EPWM->EPWM_DT[2] = u->CompB;
#elif (RUN_DIR == RUN_CCW)
    EPWM->EPWM_DT[2] = u->CompA;
    EPWM->EPWM_DT[4] = u->CompB;
#endif
    EPWM->EPWM_DT[0] = u->CompC;
}

void HAL_PwmShortLowSides(uint16_t cmp)
{
    /* EPWM_CHN：bit0/2/4 = 三相上桥使能，bit1/3/5 = 三相下桥使能 */
    EPWM->EPWM_CHN |= 0x2a;         /* 下桥全开 */
    EPWM->EPWM_CHN &= 0xffea;       /* 上桥全关 */
    EPWM->EPWM_DT[0] = cmp;
    EPWM->EPWM_DT[2] = cmp;
    EPWM->EPWM_DT[4] = cmp;
}

void HAL_FgSetDuty(uint32_t duty)
{
    TIM_PWMSetDuty(TIM0, TIM_PWMChannel_PWMA, duty);
}

uint32_t HAL_TimerNow(void)
{
    return TIM3->TIM_CNT;
}

void HAL_InitParams(void)
{
    USER.u16PWMFreq = PWM_FREQ;
    USER.u16PWMPeriod = SYSCLK_HSI_72MHz / 2 / USER.u16PWMFreq;
    USER.u16SlowLoopDiv = PWM_FREQ / SLOWLOOP_FREQ;
    USER.u16CaliPWMDuty = (USER.u16PWMPeriod * CALIPWMPERCENT);
    USER.u8SpeedUpDatePeriod = SLOWLOOP_FREQ / SPEEDUPDATE_FREQ;
    USER.u32FGNum = (SYSCLK_HSI_72MHz * 60 * 256) / (SPEED_MAX * FG_NUM);
    Svpwm_Init(&pwm_gen, (int16_t)(SYSCLK_HSI_72MHz / PWM_FREQ / 2));
}
