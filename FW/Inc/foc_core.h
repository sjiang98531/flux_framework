/* ============================================================================
 * @file    foc_core.h
 * @brief   FOC 核心：坐标变换、电流环、母线归一化、SVPWM 调用
 *
 * Clarke / Park 变换见 foc_math.h。电流环增益由 cfg_ctrl.h 的 CURRENT_BW_HZ 自动计算。
 * ==========================================================================*/
#ifndef FOC_CORE_H
#define FOC_CORE_H
#include "cfg_derived.h"
#include "compiler.h"
#include "qtypes.h"
#include "foc_math.h"
#include "qmath.h"
#include "meas.h"

/* 电流环状态。角度由估计模块（闭环）或起动模块（定位、开环）每拍写入。 */
typedef struct {
    int16_t           theta;        /* Park 用的电角度（Q15 角度，65536 = 一圈） */
    int16_t           thetaInv;     /* 反 Park 用的电角度（含延时补偿） */
    sAngle_Trig       ang, angInv;  /* 上面两个角的 cos/sin */
    MCFLIB_2_DQ_T_S16 idq;          /* dq 电流实测 Q15 */
    MCFLIB_2_DQ_T_S16 idqRef;       /* dq 电流指令 Q15（由速度环、MTPA/弱磁、起动模块写入） */
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
void FocCore_LoadGains(void);       /* 从 gGain 重新装载电流环 Kp / Ki（param.c 更新参数时调用） */

/* i_abc -> i_alpha_beta（快环热路径，内联） */
FORCE_INLINE void FocCore_Clarke(void)
{
    FocMath_ClarkeSat(&gMeas.iabc, &gMeas.iab);
}

/* 电流环：Park -> PI（d/q）-> 反 Park -> 母线归一化。角度用 gFoc.theta / thetaInv。 */
void FocCore_CurrentLoop(void);

/* d/q 电流 PI（带防溢出；由 FocCore_CurrentLoop 调用） */
void FocCore_DqPI(void);
/* dq 解耦前馈开关（进入 Spin 时打开、离开时关闭）。切换时将前馈量移入 / 移出积分器，实现无扰切换。
 */
void FocCore_SetDecouple(uint8_t on);
/* 停机 / 故障复位用：只关闭开关，不修改积分器 */
void FocCore_ResetDecouple(void);
/* 停机 / 故障复位：电流 PI 积分器、电压、电流指令、角度、解耦开关清零 */
void FocCore_ResetRun(void);
/* 电流 PI 积分器预置（Q30 电压：qdSum >> 15 = 输出 Q15）。仅在状态切换时使用，其他地方不直接写 gFoc.piD/piQ */
void FocCore_PresetPI(int32_t dSum, int32_t qSum);
/* 飞车接管：电流指令为 0 时的稳态积分器值（d = 0，q = 该转速对应的反电势电压） */
void FocCore_PresetPIForSpeed(int16_t spdQ15);
/* 写电流指令的唯一入口：d、q 以一次 32 位写入完成。慢环随时可能被快环打断，
 * 若分两次写入，或先写中间值再修改，快环可能读到不完整的指令。
 * 慢环中的控制律只计算本地变量，计算完成后通过本函数一次性发布。 */
void FocCore_SetIdqRef(int16_t d, int16_t q);
/* 控制坐标系切换为落后 d 角度的新坐标系（新角 = 旧角 - d，a = sin/cos(d)）时，将电流 PI 积分器（电压）同步旋转，
 * 保持电压矢量在空间中不变。调用方需关中断。 */
void FocCore_RotatePI(sAngle_Trig a);

/* 母线归一化：将以 UDC_MAX 为基准的 αβ 电压换算为调制器单位（32768 = Vbus/√3），
 * kvbus = gMeas.kvbus（由 Meas_SlowFilter 计算）：
 *   out = (in * kvbus >> 15) << 5     母线电压波动时自动补偿占空比 */
FORCE_INLINE void FocCore_BusNormalize(const MCFLIB_2_ALBE_T_S16 *in, int16_t kvbus, MCFLIB_2_ALBE_T_S16 *out)
{
    out->s16Alpha = (int16_t)(QMath_MulQ15(in->s16Alpha, kvbus) << 5);
    out->s16Beta  = (int16_t)(QMath_MulQ15(in->s16Beta,  kvbus) << 5);
}

/* SVPWM：gFoc.vabMod -> gSvpwm -> 比较寄存器。mode = SEVENMODE / FIVEMODE */
void FocCore_Modulate(uint8_t mode);

/* 旁路电流环，直接输出 alpha/beta 电压（Q15，UDC_MAX 基准）：母线归一化 + 七段式调制（用于 IPD 脉冲） */
void FocCore_ApplyVoltageAB(int16_t va, int16_t vb);

/* 输出零电压矢量（50% 占空比），用于状态切换 */
void FocCore_ModulateZero(void);

#endif
