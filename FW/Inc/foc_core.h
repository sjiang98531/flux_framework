/* ============================================================================
 * foc_core.h —— FOC 核心：坐标变换、电流环、母线归一化、SVPWM 调用、电压重建
 *
 * 全部自写（Clarke/Park 用原厂开源头文件 mcflib.h 的内联函数）。
 * 电流环增益由 cfg_ctrl.h 的 CURRENT_BW_HZ 自动算。
 * ==========================================================================*/
#ifndef FOC_CORE_H
#define FOC_CORE_H
#include "fw_config.h"
#include "mc_types.h"
#include "meas.h"

/* 电流环的状态。角度由估计模块（闭环）或启动模块（定位、开环）每拍写入。 */
typedef struct {
    int16_t           theta;        /* Park 用的电角度（Q15 角度，65536 = 一圈） */
    int16_t           thetaInv;     /* 反 Park 用的电角度（含延时补偿） */
    sAngle_Trig       ang, angInv;  /* 上面两个角的 cos/sin */
    MCFLIB_2_DQ_T_S16 idq;          /* dq 电流实测 Q15 */
    MCFLIB_2_DQ_T_S16 idqRef;       /* dq 电流指令 Q15（速度环、MTPA/弱磁、启动写） */
    MCFLIB_2_DQ_T_S16 vdq;          /* dq 电压（电流 PI + 前馈输出），Q15（UDC_MAX 基准） */
    MCFLIB_2_ALBE_T_S16 vab;        /* αβ 电压 */
    MCFLIB_2_ALBE_T_S16 vabMod;     /* 母线归一化后送调制器的 αβ 电压（32768 = Vbus/√3） */
    int16_t           vmax;         /* 电压圆半径 = MAX_DUTY * VbusAvg * 2/3 */
    tPIParm           piD, piQ;     /* d / q 电流 PI */
} FocCore_TypeDef;

extern FocCore_TypeDef gFoc;

/* 电压限幅占空比（MAX_DUTY，Q15） */
#define FOC_DUTY_LIMIT_Q15      Q15(MAX_DUTY)

/* 电流环参数初始化 */
void FocCore_Init(void);

/* i_abc -> i_alpha_beta（快环热路径，内联） */
static __forceinline void FocCore_Clarke(void)
{
    MCFLIB_Clark_Sat_S16(&gMeas.iabc, &gMeas.iab);
}

/* 电流环：Park -> PI（d/q）-> 反 Park -> 母线归一化。角度用 gFoc.theta / thetaInv。 */
void FocCore_CurrentLoop(void);

/* d/q 电流 PI（自写，防溢出；FocCore_CurrentLoop 调它） */
void FocCore_DqPI(void);
/* dq 解耦前馈开关（进 Spin 开、离开 Spin 关）。会把前馈量搬进/搬出积分器，无扰切换。
 * CTRL_DQ_DECOUPLE = 0 时空函数。 */
void FocCore_SetDecouple(uint8_t on);
/* 停机/故障复位用：只关开关，不动积分器 */
void FocCore_ResetDecouple(void);

/* 母线归一化（替代原厂 DcBus_Ripple_suppress）：把以 UDC_MAX 为基准的 αβ 电压换成
 * 调制器的单位（32768 = Vbus/√3），kvbus = gMeas.kvbus（Meas_SlowFilter 算）：
 *   out = (in * kvbus >> 15) << 5     母线电压波动时自动补偿占空比 */
static __forceinline void FocCore_BusNormalize(const MCFLIB_2_ALBE_T_S16 *in, int16_t kvbus, MCFLIB_2_ALBE_T_S16 *out)
{
    out->s16Alpha = (int16_t)(MLIB_Mul_Q15(in->s16Alpha, kvbus) << 5);
    out->s16Beta  = (int16_t)(MLIB_Mul_Q15(in->s16Beta,  kvbus) << 5);
}

/* SVPWM：gFoc.vabMod -> pwm_gen -> 比较寄存器。mode = SEVENMODE / FIVEMODE */
void FocCore_Modulate(uint8_t mode);

/* 输出零电压矢量（50% 占空比），状态切换时用 */
void FocCore_ModulateZero(void);

/* 由上一拍的三相比较值和母线电压重建 alpha/beta 电压（Q15，FLUX_V_BASE 基准）。
 * 不含死区（死区补偿试过两次都更差，见 flux_selfpll 实验记录 test12/16）。
 * "比较值越小，相电压越高"：pole_x = (PERIOD - cmp_x)/PERIOD * Vbus，
 *   v_alpha = (2a - b - c)/3，v_beta = (b - c)/sqrt(3)。快环热路径，内联。 */
#define FOC_COUNT_SCALE ((int32_t)(1073741824.0f / PWM_PERIOD))     /* 2^30 / PWM_PERIOD */
static __forceinline void FocCore_VoltageFromCompares(int16_t cmpA, int16_t cmpB, int16_t cmpC, int16_t bus,
                                                      int16_t *va, int16_t *vb)
{
    int32_t a, b, c, t;
    a = ((int32_t)(PWM_PERIOD - cmpA) * FOC_COUNT_SCALE) >> 15;
    b = ((int32_t)(PWM_PERIOD - cmpB) * FOC_COUNT_SCALE) >> 15;
    c = ((int32_t)(PWM_PERIOD - cmpC) * FOC_COUNT_SCALE) >> 15;
    a = (a * bus) >> 15; b = (b * bus) >> 15; c = (c * bus) >> 15;
    t = 2 * a - b - c;
    *va = (int16_t)((t * 21846 + 32768) >> 16);
    *vb = (int16_t)(((b - c) * 18919) >> 15);
}

#endif
