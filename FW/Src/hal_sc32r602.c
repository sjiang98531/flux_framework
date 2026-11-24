/* ============================================================================
 * @file    hal_sc32r602.c
 * @brief   SC32R602 驱动板的 HAL 实现及中断入口
 *
 * 快环 = ADC 序列 0 转换结束中断（16 kHz）：
 *   上电后先采样 256 拍计算电流零点，之后每拍：
 *   读取电流 / 母线 -> Ctrl_FastTick()（状态机快环）-> 按分频挂起慢环
 * 慢环 = PendSV：快环每 PWM_FREQ / SLOWLOOP_FREQ 拍挂起一次，优先级最低（3），可被快环（ADC，1）、
 * 硬件过流（EPWM，0）随时抢占。只要快环 + 慢环的 CPU 占用低于 100%，慢环即严格每 16 个 PWM 周期执行一次，
 * 与 PWM 同相，不受 main() 中其他代码影响。
 * ==========================================================================*/
#include "hal.h"
#include "meas.h"
#include "protect.h"
#include "supervisor.h"
#include "qmath.h"
#include "SC_it.h"
#include "compiler.h"

__align(4) u16 ADC_DMABuff[14];          /* ADC -> DMA 缓冲（SC_Init.c 配置 DMA 目标地址） */
Hal_TypeDef gHal;

/* 上电零点校准：前 OFFSET_START 拍等运放、基准稳定（约 62 ms），之后 OFFSET_N 拍求零点与噪声（和 >> 8 = 平均） */
#define OFFSET_START    999u
#define OFFSET_N        256u
#define OFFSET_END      (OFFSET_START + OFFSET_N)

/* 零点校准累加器：A、B 相电流及母线电流的采样和（256 个） */
static uint32_t s_u32IaSum, s_u32IbSum, s_u32IbusSum;
static uint16_t s_u16Cnt;                /* 上电拍数（零点校准用） */
#if (IBUS_RECONSTRUCTED == 0)
static uint16_t s_u16IbusOffset;        /* 母线电流采样零点（仅在使用采样值时需要） */
#endif
static uint16_t s_adcMin[2], s_adcMax[2];   /* 零点窗口内两相电流采样的最小 / 最大值（用于测量噪声） */
/* 快环拍数计数，达到 gHal.slowLoopDiv（16）时挂起一次慢环 */
static uint8_t  s_u8SlowDiv;

/* ---------------------------------------------------------------------------
 * 采样：返回 1 表示零点已校准、本拍采样已写入 FOC，可以运行控制。
 * 零点 = 256 个采样之和 >> 8。
 * -------------------------------------------------------------------------*/
/* 采样换算（ADC 中断每拍调用，DMA 已将本拍采样搬入 ADC_DMABuff：[0] A 相、[1] B 相、[2] 母线电流、[3] 母线电压）。
 * 上电后先等待 999 拍（约 62 ms，等待运放、基准稳定），再采样 256 拍求零点与噪声，之后每拍：
 *   电流 = (零点 - 采样) << 1：采样电阻电压经运放偏置在 Vdd/2，减去零点得到带符号电流；
 *   12 位 ADC << 3 为 Q15，再 << 1 使 Vdd/2 对应 I_MAX（满量程）；
 *   RUN_DIR 决定 A / B 两相是否对调（交换一对相线 = 反转方向，无需改硬件）；C 相 = -(A + B)。 */
FORCE_INLINE uint8_t acquireSamples(void)
{
    int16_t ia, ib;
    s_u16Cnt++;
    if (s_u16Cnt < OFFSET_START) {               /* 第 999~1254 拍：共 256 个点 */
        return 0;
    } else if (s_u16Cnt < OFFSET_END) {
        uint8_t k;
        for (k = 0; k < 2; k++) {       /* 噪声：零电流时采样峰峰值 / 2 */
            uint16_t x = ADC_DMABuff[k];
            if (s_u16Cnt == OFFSET_START || x < s_adcMin[k]) s_adcMin[k] = x;
            if (s_u16Cnt == OFFSET_START || x > s_adcMax[k]) s_adcMax[k] = x;
        }
        s_u32IaSum += ADC_DMABuff[0] << 3;
        s_u32IbSum += ADC_DMABuff[1] << 3;
        s_u32IbusSum += ADC_DMABuff[2] << 3;
        return 0;
    } else if (s_u16Cnt == OFFSET_END) {
        gMeas.iabcOffset.s16A = s_u32IaSum >> 8;
        gMeas.iabcOffset.s16B = s_u32IbSum >> 8;
#if (IBUS_RECONSTRUCTED == 0)
        s_u16IbusOffset = s_u32IbusSum >> 8;
#endif
        {   /* 噪声峰值（码）= 峰峰值 / 2 向上取整 + 1 码量化，两相取较大值。飞车测速脉冲电流下限据此计算（flystart.c） */
            uint16_t na = (uint16_t)((s_adcMax[0] - s_adcMin[0] + 1) >> 1), nb = (uint16_t)((s_adcMax[1] - s_adcMin[1] + 1) >> 1);
            gHal.adcNoisePk = (uint16_t)((na > nb ? na : nb) + 1);
        }
        s_u32IaSum = 0;
        s_u32IbSum = 0;
        s_u32IbusSum = 0;
        return 0;
    }
    s_u16Cnt = OFFSET_END + 1;
    ia = (ADC_DMABuff[0] << 3);                    /* 12 位 ADC 左移 3 位 -> Q15 */
    ib = (ADC_DMABuff[1] << 3);
#if (RUN_DIR == RUN_CW)
    gMeas.iabc.s16A = (gMeas.iabcOffset.s16A - ia) << 1;   /* <<1：Vdd/2 对应 I_MAX */
    gMeas.iabc.s16B = (gMeas.iabcOffset.s16B - ib) << 1;
#elif (RUN_DIR == RUN_CCW)
    gMeas.iabc.s16B = (gMeas.iabcOffset.s16A - ia) << 1;
    gMeas.iabc.s16A = (gMeas.iabcOffset.s16B - ib) << 1;
#endif
    gMeas.iabc.s16C = QMath_Sat16(-(int32_t)gMeas.iabc.s16B - (int32_t)gMeas.iabc.s16A);
#if (IBUS_RECONSTRUCTED == 1)
    gMeas.ibus = gMeas.ibusRec;
#else
    gMeas.ibus = ((ADC_DMABuff[2] << 3) - s_u16IbusOffset) * 2;
#endif
    gMeas.vbus = (ADC_DMABuff[3] << 3);
    return 1;
}

/* 停机中电流零点跟踪（Stop 快环每拍调用）。
 * 上电校准只做一次；若当时功率部分尚未通电（例如先运行程序、后接母线），模拟前端的偏置与正常工作时不同，零点带误差，
 * 运行时表现为直流电流误差（开环拖动时 1 倍电频率的转矩波动、辨识结果偏差）。
 * 停机时 PWM 关闭、电流为零，正是测零点的条件：进入停机后先等 OFFSET_TRK_SETTLE 拍（约 4 ms）让绕组电流衰减到零，
 * 之后每 OFFSET_N 拍（16 ms）一个窗口重测零点，起动时用的总是最近一次在工作条件下测得的值，不增加起动时间。
 * 窗口内两相采样峰峰值都不超过上电噪声的约 2 倍才采用（确认电流确实为零：例如转子惯性转动、反电势经二极管导通时不更新） */
#define OFFSET_TRK_SETTLE   64u
static uint16_t s_trkN;
static uint32_t s_trkSum[3];
static uint16_t s_trkMin[2], s_trkMax[2];

void HAL_OffsetTrackRestart(void)
{
    s_trkN = 0;
}

void HAL_OffsetTrack(void)
{
    uint8_t k;
    if (s_u16Cnt <= OFFSET_END) return;                 /* 上电校准未完成 */
    if (++s_trkN <= OFFSET_TRK_SETTLE) return;
    if (s_trkN == OFFSET_TRK_SETTLE + 1u) {             /* 新窗口 */
        for (k = 0; k < 3; k++) s_trkSum[k] = 0;
        for (k = 0; k < 2; k++) s_trkMin[k] = s_trkMax[k] = ADC_DMABuff[k];
    }
    for (k = 0; k < 2; k++) {
        uint16_t x = ADC_DMABuff[k];
        if (x < s_trkMin[k]) s_trkMin[k] = x;
        if (x > s_trkMax[k]) s_trkMax[k] = x;
    }
    for (k = 0; k < 3; k++) s_trkSum[k] += (uint32_t)ADC_DMABuff[k] << 3;
    if (s_trkN < OFFSET_TRK_SETTLE + OFFSET_N) return;
    if ((uint16_t)(s_trkMax[0] - s_trkMin[0]) <= 4u * gHal.adcNoisePk &&
        (uint16_t)(s_trkMax[1] - s_trkMin[1]) <= 4u * gHal.adcNoisePk) {
        gMeas.iabcOffset.s16A = (int16_t)(s_trkSum[0] >> 8);
        gMeas.iabcOffset.s16B = (int16_t)(s_trkSum[1] >> 8);
#if (IBUS_RECONSTRUCTED == 0)
        s_u16IbusOffset = (uint16_t)(s_trkSum[2] >> 8);
#endif
    }
    s_trkN = OFFSET_TRK_SETTLE;                         /* 下一窗口 */
}

/* 硬件过流（比较器逐周期保护）中断：母线电流超过比较器门限（DAC 设定）时芯片已自动封锁 PWM，
 * 此处再次关闭输出并置锁存（下一拍快环 faultDetect 转换为故障），清除中断标志。优先级 0（最高） */
void EPWM_IRQHandler(void)
{
    if ((EPWM->EPWM_STS & EPWM_STS_CBCIF) != 0) {
        DISABLE_PWMOUT();
        gHal.ocLatch = 1;             /* 由快环 faultDetect 转换为 MC_FAULT_I_DCBUS_OVER（直接写故障位会被其清除） */
        EPWM->EPWM_STS |= EPWM_STS_CBCIF;
    }
}

/* 慢环中断（PendSV，最低优先级，见 HAL_StartInterrupts）：ADC 中断每 16 拍挂起一次 */
void PendSV_Handler(void)
{
    Ctrl_SlowTick();
}

/* 快环中断：ADC 序列转换结束（每个 PWM 周期一次，采样时刻由 EPWM 触发，位于 PWM 周期中点）。
 * 依次：读取采样 -> Ctrl_FastTick（状态机快环：电流环、观测器等）-> 每 16 拍挂起慢环。
 * 本函数及其调用的热路径可放在 RAM 中执行（RAMFUNC），以避免 Flash 等待周期 */
RAMFUNC void ADC_IRQHandler(void)
{
    /* 标志位直接读写寄存器（与 ADC_GetFlagStatus / ADC_ClearFlag 等价）：库函数位于 Flash，
     * 快环在 RAM 中执行时可避免每拍两次跳回 Flash */
    if ((ADC->ADC_STS & (uint16_t)ADC_Flag_EOSIF0) != 0) {
        ADC->ADC_STS = (uint16_t)ADC_Flag_EOSIF0;
        if (acquireSamples()) {
            Ctrl_FastTick();
            if (++s_u8SlowDiv >= gHal.slowLoopDiv) {
                s_u8SlowDiv = 0;
                SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
            }
        }
    }
}

/* TIM3：系统时基定时器（控制未使用），中断中只清除标志 */
void TIMER3_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_Flag_TI) != RESET) TIM_ClearFlag(TIM3, TIM_Flag_TI);
}

/* TIM0：FG 输出（转速脉冲信号，HAL_FgSetDuty），中断中只清除标志 */
void TIMER0_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM0, TIM_Flag_TI) != RESET) TIM_ClearFlag(TIM0, TIM_Flag_TI);
}

/* ---------------------------------------------------------------------------
 * PWM
 * -------------------------------------------------------------------------*/
/* 三相比较值均设为半周期（50% 占空比，三相电压相等、线电压为 0）。Stop 状态下每拍调用 */
void HAL_PwmSetHalfDuty(void)
{
    EPWM->EPWM_DT[0] = gHal.pwmPeriod >> 1;
    EPWM->EPWM_DT[2] = gHal.pwmPeriod >> 1;
    EPWM->EPWM_DT[4] = gHal.pwmPeriod >> 1;
}

/* 将 SVPWM 计算的三相比较值写入 EPWM（下一个 PWM 周期生效）。
 * 通道对应：EPWM_DT[4] / [2] / [0] = A / B / C 相；RUN_DIR = CCW 时 A、B 对调（与采样侧的对调配套） */
RAMFUNC void HAL_PwmWriteCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC)
{
#if (RUN_DIR == RUN_CW)
    EPWM->EPWM_DT[4] = cmpA;
    EPWM->EPWM_DT[2] = cmpB;
#elif (RUN_DIR == RUN_CCW)
    EPWM->EPWM_DT[2] = cmpA;
    EPWM->EPWM_DT[4] = cmpB;
#endif
    EPWM->EPWM_DT[0] = cmpC;
}

/* 三相下桥全开、上桥全关 = 三相绕组短接（用于飞车检测、制动）。cmp 为比较值（决定下桥导通占空比，gHal.caliDuty） */
void HAL_PwmShortLowSides(uint16_t cmp)
{
    /* EPWM_CHN：bit0/2/4 = 三相上桥使能，bit1/3/5 = 三相下桥使能 */
    EPWM->EPWM_CHN |= 0x2a;         /* 下桥全开 */
    EPWM->EPWM_CHN &= 0xffea;       /* 上桥全关 */
    EPWM->EPWM_DT[0] = cmp;
    EPWM->EPWM_DT[2] = cmp;
    EPWM->EPWM_DT[4] = cmp;
}

/* FG 输出（对外转速信号）的占空比；停机 / 故障时设为 0 */
void HAL_FgSetDuty(uint32_t duty)
{
    TIM_PWMSetDuty(TIM0, TIM_PWMChannel_PWMA, duty);
}

/* 驱动板自检（PWM_DEBUG = 1）：三相固定 25% 占空比互补输出，不运行控制、不开中断，可用示波器检查波形与死区。仅喂狗，不返回 */
void HAL_PwmDebugLoop(void)
{
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
}

/* 外设初始化：时钟、基准、ADC、比较器 / DAC（硬件过流）、运放、EPWM、TIM3 / TIM0、GPIO（见 Board/Src/SC_Init.c） */
void HAL_InitHardware(void)
{
    Motor_Hardware_Init();
}

/* 中断优先级（M0+ 只有 0~3 四级，0 最高）：硬件过流 EPWM 0、快环 ADC 1（Board/Src/SC_Init.c Motor_NVIC_Config，
 * 设置优先级并开中断），慢环 PendSV 3（由快环挂起，无需使能）。PendSV 须在快环开始运行前配置好 */
void HAL_StartInterrupts(void)
{
    NVIC_SetPriority(PendSV_IRQn, 3);
    Motor_NVIC_Config();
}

/* HAL 自身参数（由 Ctrl_Init 最先调用）：
 *   pwmPeriod   EPWM 周期计数（中心对齐，72 MHz / 2 / PWM_FREQ = 2250）
 *   slowLoopDiv 快环每多少拍挂起一次慢环（PWM_FREQ / SLOWLOOP_FREQ = 16）
 *   caliDuty    飞车短接时下桥的比较值（周期的 CALIPWMPERCENT） */
void HAL_InitParams(void)
{
    gHal.pwmPeriod = (uint16_t)(SYSCLK_HSI_72MHz / 2 / PWM_FREQ);
    gHal.slowLoopDiv = PWM_FREQ / SLOWLOOP_FREQ;
    gHal.caliDuty = (gHal.pwmPeriod * CALIPWMPERCENT);
}
