/* ============================================================================
 * @file    foc_core.c
 * @brief   电流环（d/q 两个 PI + 解耦前馈 + 电压圆限幅）、母线归一化、调制。接口说明见 foc_core.h
 *
 * 快环每拍执行顺序：FocCore_Clarke（foc_core.h）-> 估计模块 / 起动模块写入 gFoc.theta
 *   -> FocCore_CurrentLoop（Park、计算电压圆、DqPI、反 Park、母线归一化）-> FocCore_Modulate（SVPWM、写比较值）。
 * ==========================================================================*/
#include "foc_core.h"
#include <stddef.h>
#include "hal.h"
#include "meas.h"
#include "est.h"
#include "ctrl_pi.h"
#include "svpwm.h"
#include "qmath.h"
#include "compiler.h"
#include "param.h"

FocCore_TypeDef gFoc;

RAMFUNC static void dqPI(void);

/* 上电初始化：装载电流环增益，清零积分器（输出限幅每拍按电压圆重设） */
void FocCore_Init(void)
{
    Svpwm_Init(&gSvpwm, (int16_t)gHal.pwmPeriod);      /* 比较值满量程 = EPWM 周期计数（中心对齐，HAL_InitParams 已算好） */
    /* d 轴电流环（用 Ld）、q 轴（用 Lq）：增益由 param.c 按当前参数计算（封顶 65535，见 gGain.cfgWarn） */
    FocCore_LoadGains();
    gFoc.piD.qOutMax = 0;               /* 每拍在 FocCore_CurrentLoop 中按 VMAX 重设 */
    gFoc.piD.qOutMin = -gFoc.piD.qOutMax;
    gFoc.piD.qdSum = 0;
    gFoc.piD.qInMeas = 0;
    gFoc.piD.qOut = 0;
    gFoc.piQ.qOutMax = 0;
    gFoc.piQ.qOutMin = -gFoc.piQ.qOutMax;
    gFoc.piQ.qdSum = 0;
    gFoc.piQ.qInMeas = 0;
    gFoc.piQ.qOut = 0;
}

/* 从 gGain 装载两个电流 PI 的 Kp / Ki（上电及参数更新时） */
void FocCore_LoadGains(void)
{
    gFoc.piD.qKp = gGain.curKpD; gFoc.piD.qKi = gGain.curKiD;
    gFoc.piQ.qKp = gGain.curKpQ; gFoc.piQ.qKi = gGain.curKiQ;
}

/* 停机 / 重新起动前：清零积分器、电压、电流指令、角度，复位解耦开关 */
void FocCore_ResetRun(void)
{
    gFoc.piD.qdSum = 0;
    gFoc.piQ.qdSum = 0;
    gFoc.vdq.s16Q = 0;
    gFoc.vdq.s16D = 0;
    FocCore_SetIdqRef(0, 0);
    gFoc.theta = 0;
    FocCore_ResetDecouple();        /* 积分器已清零，只复位开关，不转移前馈量 */
}

/* 直接设置两个积分器（单位：Q15 电压 << 15）。用于状态切换，使 PI 输出从已知电压开始，而非从 0 开始 */
void FocCore_PresetPI(int32_t dSum, int32_t qSum)
{
    gFoc.piD.qdSum = dSum;
    gFoc.piQ.qdSum = qSum;
}

void FocCore_PresetPIForSpeed(int16_t spdQ15)
{
    /* 电流指令为 0、转子以 spd 旋转：d 轴为 0，q 轴 = 反电势电压（id = iq = 0 时的稳态） */
    FocCore_PresetPI(0, (((int32_t)spdQ15 * gGain.decKEmfQ15) >> 15) << 15);
}

/* 电流环一拍（快环）：
 *   1. Park：αβ 电流 -> dq 电流（使用 gFoc.theta）；
 *   2. 电压圆半径 vmax = MAX_DUTY * 母线 * 2/3（SVPWM 线性区可输出的最大相电压幅值，Q15 电压）；
 *   3. dqPI：d、q 两个 PI 加解耦前馈，得到 dq 电压；
 *   4. 反 Park（使用 gFoc.thetaInv，可超前于 theta，补偿 PWM 延迟一拍生效期间的转角）-> αβ 电压；
 *   5. 母线归一化：按实际母线电压换算为调制器单位（母线升高时占空比自动减小）。 */
RAMFUNC void FocCore_CurrentLoop(void)
{
    int16_t s16Temp;

    gFoc.ang = FocMath_SinCos(gFoc.theta);
    FocMath_Park(&gMeas.iab, &gFoc.ang, &gFoc.idq);

    s16Temp = QMath_MulQ15(FOC_DUTY_LIMIT_Q15, gMeas.vbusAvg);
    gFoc.vmax = QMath_MulQ15(s16Temp, Q15(0.6667));     /* 幅值不变变换 */

    dqPI();

    /* 反 Park 角（gFoc.thetaInv）由估计模块按时序补偿给出；与 theta 相同时省去一次查表 */
    if (gFoc.thetaInv != gFoc.theta)
        gFoc.angInv = FocMath_SinCos(gFoc.thetaInv);
    else
        gFoc.angInv = gFoc.ang;
    FocMath_InvPark(&gFoc.vdq, &gFoc.angInv, &gFoc.vab);
    FocCore_BusNormalize(&gFoc.vab, gMeas.kvbus, &gFoc.vabMod);
}

/* ---- dq 解耦前馈 ----
 * 开关请求由慢环发出（s_decReq：0 无请求，1 关闭，2 打开），快环在 PI 之前执行：
 * 前馈量移入 / 移出积分器与前馈开关切换在同一拍内完成，不会被中断分割。
 * s_decOn = 解耦前馈当前是否打开 */
enum { DEC_REQ_NONE = 0, DEC_REQ_OFF = 1, DEC_REQ_ON = 2 };
static volatile uint8_t s_decOn, s_decReq;
/* vd_ff = -w*Lq*iq，vq_ff = w*psi_f + w*Ld*id（指令电流，Q15 电压） */
/* 由 dq 电压方程，稳态时 vd = Rs*id - w*Lq*iq，vq = Rs*iq + w*(psi_f + Ld*id)。
 * 与转速成正比的耦合项 / 反电势项由前馈直接给出，PI 只处理 Rs*i 与动态过程，高速时电流环无需依靠积分追踪反电势。
 * w 使用 PLL 转速，电流使用指令值（实测值含噪声，前馈会放大噪声）。 */
FORCE_INLINE void decoupleFF(int32_t *ffd, int32_t *ffq)
{
    int32_t w = gEst.pll.s32Speed >> 16;
    int32_t wiq = (w * gFoc.idqRef.s16Q) >> 15;
    int32_t wid = (w * gFoc.idqRef.s16D) >> 15;
    *ffd = -((wiq * gGain.decKLqQ12) >> 12);
    *ffq = ((w * gGain.decKEmfQ15) >> 15) + ((wid * gGain.decKLdQ12) >> 12);
}

/* 慢环 / 切换函数请求打开（on = 1）或关闭解耦，由下一拍快环执行（applyDecoupleRequest） */
void FocCore_SetDecouple(uint8_t on)
{
    s_decReq = on ? DEC_REQ_ON : DEC_REQ_OFF;
}

/* 快环中执行开关请求。打开解耦后总电压 = PI 输出 + 前馈；为使切换时刻总电压不跳变，从积分器中减去前馈量
 * （关闭时加回）：总输出不变，仅改变这部分电压的提供者 */
FORCE_INLINE void applyDecoupleRequest(void)
{
    uint8_t on;
    int32_t ffd, ffq;
    if (s_decReq == DEC_REQ_NONE) return;
    on = (uint8_t)(s_decReq == DEC_REQ_ON);
    s_decReq = DEC_REQ_NONE;
    if (on == s_decOn) return;
    decoupleFF(&ffd, &ffq);
    if (on) { gFoc.piD.qdSum -= ffd << 15; gFoc.piQ.qdSum -= ffq << 15; }
    else    { gFoc.piD.qdSum += ffd << 15; gFoc.piQ.qdSum += ffq << 15; }
    s_decOn = on;
}

/* 复位解耦开关（不修改积分器：调用方已清零积分器） */
void FocCore_ResetDecouple(void) { s_decReq = DEC_REQ_NONE; s_decOn = 0; }

/* gFoc.idqRef 在 FocCore_TypeDef 中 4 字节对齐，可用一条 STR 指令写入 */
typedef char FocIdqRefAligned[(offsetof(FocCore_TypeDef, idqRef) % 4u == 0u) ? 1 : -1];
typedef union { MCFLIB_2_DQ_T_S16 dq; uint32_t u32; } FocDqWord;

/* 写入 dq 电流指令（慢环 -> 快环的唯一入口，其他地方不得直接写）。
 * d、q 拼成一个 32 位字一次写入：快环不会读到"新 d + 旧 q"的不完整指令 */
void FocCore_SetIdqRef(int16_t d, int16_t q)
{
    FocDqWord w;
    w.dq.s16D = d;
    w.dq.s16Q = q;
    ((volatile FocDqWord *)&gFoc.idqRef)->u32 = w.u32;
}

/* 将两个积分器（即 dq 电压）按角度 a 旋转。I/F 切闭环时，参考坐标系由开环角切换为观测器角，二者相差 a：
 * 电压矢量在空间中保持不变，仅更换坐标系表示，因此 dq 分量需反向旋转同一角度（startup.c openLoopHandover） */
void FocCore_RotatePI(sAngle_Trig a)
{
    int32_t vd = gFoc.piD.qdSum, vq = gFoc.piQ.qdSum;     /* 乘法写成 32x32->64（在临界区内调用，需尽量短） */
    gFoc.piD.qdSum = (int32_t)(((int64_t)vd * (int32_t)a.s16Cos - (int64_t)vq * (int32_t)a.s16Sin) >> 15);
    gFoc.piQ.qdSum = (int32_t)(((int64_t)vd * (int32_t)a.s16Sin + (int64_t)vq * (int32_t)a.s16Cos) >> 15);
}

/* d、q 两个电流 PI（每拍）：
 *   d 轴：PI 输出 + 前馈，限制在 ±vmax；
 *   q 轴：限制在 ±sqrt(vmax^2 - vd^2)（电压圆，d 轴优先：先保证磁场方向电压，剩余部分用于转矩）；
 *   PI 输出范围扣除前馈，使"PI + 前馈"的总和受限（积分不会因前馈而饱和） */
RAMFUNC static void dqPI(void)
{
    int32_t t, ffd = 0, ffq = 0, vd, lim;
    applyDecoupleRequest();
    if (s_decOn) decoupleFF(&ffd, &ffq);
    /* PI 输出范围 = 电压限幅 - 前馈 */
    gFoc.piD.qOutMax = gFoc.vmax - ffd;
    gFoc.piD.qOutMin = -gFoc.vmax - ffd;
    gFoc.piD.qInRef = gFoc.idqRef.s16D;
    gFoc.piD.qInMeas = gFoc.idq.s16D;
    Pi_Run32(&gFoc.piD);
    vd = gFoc.piD.qOut + ffd;
    gFoc.vdq.s16D = (int16_t)vd;

    t = (int32_t)gFoc.vmax * gFoc.vmax - (int32_t)gFoc.vdq.s16D * gFoc.vdq.s16D;
    lim = FocMath_SqrtQ15(t);
    gFoc.piQ.qOutMax = lim - ffq;
    gFoc.piQ.qOutMin = -lim - ffq;
    gFoc.piQ.qInRef = gFoc.idqRef.s16Q;
    gFoc.piQ.qInMeas = gFoc.idq.s16Q;
    Pi_Run32(&gFoc.piQ);
    gFoc.vdq.s16Q = (int16_t)(gFoc.piQ.qOut + ffq);
}

/* 调制：归一化后的 αβ 电压 -> SVPWM 三相比较值 -> 写寄存器（下一个 PWM 周期生效）。
 * mode = SEVENMODE（七段式）/ FIVEMODE（五段式，开关次数减少 1/3，spin.c 按转速选择）。
 * 同时由三相比较值与电流重构母线电流（ibusRec） */
RAMFUNC void FocCore_Modulate(uint8_t mode)
{
    gSvpwm.Mode = mode;
    gSvpwm.Alpha = gFoc.vabMod.s16Alpha;
    gSvpwm.Beta = gFoc.vabMod.s16Beta;
    Svpwm_Calc(&gSvpwm, &gMeas.iabc, (int16_t *)&gMeas.ibusRec);
    HAL_PwmWriteCompares(gSvpwm.CompA, gSvpwm.CompB, gSvpwm.CompC);
}

/* 旁路电流环，直接输出 αβ 电压（Q15，UDC_MAX 基准）。用于 IPD 脉冲、参数辨识注入 */
void FocCore_ApplyVoltageAB(int16_t va, int16_t vb)
{
    gFoc.vab.s16Alpha = va;
    gFoc.vab.s16Beta  = vb;
    FocCore_BusNormalize(&gFoc.vab, gMeas.kvbus, &gFoc.vabMod);
    FocCore_Modulate(SEVENMODE);
}

/* 输出零电压（三相占空比相同 = 零矢量）。用于状态切换时刻、开启 PWM 的第一拍 */
void FocCore_ModulateZero(void)
{
    gSvpwm.Mode = SEVENMODE;
    gSvpwm.Alpha = 0;
    gSvpwm.Beta = 0;
    Svpwm_Calc(&gSvpwm, &gMeas.iabc, (int16_t *)&gMeas.ibusRec);
    HAL_PwmWriteCompares(gSvpwm.CompA, gSvpwm.CompB, gSvpwm.CompC);
}
