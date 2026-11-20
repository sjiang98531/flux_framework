/* ============================================================================
 * @file    identify.c
 * @brief   参数辨识（IDENT_TEST = 1 时编译；正常运行时这些函数不被调用）。方法说明见 identify.h
 *
 * 自动方式（默认）：静止测量 -> 停机后重新起动 -> 运行测量，见 identify.h。
 * 静止测量（给指令后电机不旋转）：Ident_Reset 从 phase 9 开始，Align 子状态的快环 / 慢环交由本模块处理：
 *   phase 9  R/L 粗测：alpha 轴注入 200 Hz 正弦电压，不使用电流环；
 *            测得后立即更新电流环增益；若失败直接终止于 phase 4
 *   定位（正常进行，采用无源虚拟电阻阻尼）-> phase 1/2 死区阶梯：d 轴电流 11 档（子窗静止监测），拟合 vd = Rs*id + 死区电压
 *   phase 6  交流阻抗：2-DOF 控制（复阻抗电压前馈 + 保守带宽 PI），d/q 轴多频多幅值注入，同步提取 3 次谐波与正交轴静止监测
 *   phase 7  电感饱和 L(i) 拟合（含 3 次谐波精确描述函数，分摊到多个慢环周期）
 *   phase 3  IPD 对比度（ipdFit；置于交流阻抗之后，避免负偏置脉冲击飞低惯量转子）
 *   phase 4  完成：经门控校验后提交给 param.c（offerStill），回到 Stop 后生效
 * 运行测量（正常起动）：I/F 中测量开环磁链（Ident_OpenLoopFluxSlow），闭环稳速后零电流滑行测量
 *   psi_f 与加速度系数（Ident_RunSlow）。
 * 辨识模式的慢环允许使用浮点（仅在辨识时运行）。
 * ==========================================================================*/
#include "identify.h"
#include "cfg_derived.h"
#include "compiler.h"
#include "foc_core.h"
#include "meas.h"
#include "foc_math.h"
#include "qmath.h"
#include "flux_obs.h"
#include "spin.h"
#include "param.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

IdentResult_TypeDef gIdent;
volatile IdentCfg_TypeDef gIdentCfg = { 0 };
IdentTest_TypeDef gIdentTest;

/* 死区阶梯各档 d 轴电流（按额定电流缩放） */
static const int16_t s_dtLevel[IDENT_DT_N] = {
    IDENT_Q15_I(1.00), IDENT_Q15_I(0.80), IDENT_Q15_I(0.60), IDENT_Q15_I(0.50), IDENT_Q15_I(0.40),
    IDENT_Q15_I(0.30), IDENT_Q15_I(0.25), IDENT_Q15_I(0.20), IDENT_Q15_I(0.15), IDENT_Q15_I(0.10),
    IDENT_Q15_I(0.05)
};
#define DT_SETTLE_MS    300
#define DT_AVG_MS       200
#define DT_SUB_MS       20      /* 200 ms 平均窗拆为 10 段 20 ms 子窗，监测 vq 峰峰值（消除周期盲区） */
#define ZERO_WAIT_MS    50

/* 测量用定位角：gIdentCfg.alignDeg（度，可在线修改）换算为 Q15 角度 */
int16_t Ident_AlignTheta(void)
{
    int32_t d = gIdentCfg.alignDeg % 360;
    if (d < 0) d += 360;
    return (int16_t)(d * 65536 / 360);
}

static void rlStart(void);
static uint8_t s_autoStage;         /* 自动方式：0 = 静止测量未完成，1 = 静止测量已完成、执行运行测量 */
static uint8_t s_restartReq;        /* 自动方式：静止测量刚完成，请求停机再起动（一次性标志） */

uint8_t Ident_StaticThisStart(void)
{
    if (gIdentCfg.mode == IDENT_MODE_STATIC) return 1;
    if (gIdentCfg.mode != IDENT_MODE_AUTO) return 0;
    if (gIdent.state == IDENT_IDLE) s_autoStage = 0;
    return (uint8_t)(gIdent.state != IDENT_DONE && s_autoStage == 0);
}

uint8_t Ident_RunActive(void)
{
    if (gIdentCfg.mode == IDENT_MODE_RUN) return 1;
    return (uint8_t)(gIdentCfg.mode == IDENT_MODE_AUTO && s_autoStage == 1 && gIdent.state == IDENT_RUNNING);
}

uint8_t Ident_TakeRestart(void)
{
    uint8_t r = s_restartReq;
    s_restartReq = 0;
    return r;
}

enum { OLF_WAIT_STABLE = 0, OLF_AVG = 1, OLF_DONE = 2 };
static uint8_t s_olFluxSt;
static uint16_t s_olFluxN;

/* 死区阶梯子窗静止监测（每档 10 个 20 ms 子窗） */
static int32_t  s_dtSubVq, s_dtSubMin, s_dtSubMax;
static uint8_t  s_dtSubIdx, s_dtRetry;
static uint16_t s_dtDropMask;

void Ident_Reset(void)
{
    memset(&gIdent, 0, sizeof(gIdent));
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdent.state = IDENT_RUNNING;
    gIdent.acDlyTicks = (float)CTRL_INVPARK_ADVANCE;
    gIdent.snrMin = 1e9f;
    FocCore_LoadGains();
    rlStart();
    gIdentTest.phase = IDENT_PH_RL;
}

void Ident_Idle(void)
{
    gIdentTest.phase = IDENT_PH_NONE;
    s_olFluxSt = OLF_WAIT_STABLE;
    s_olFluxN = 0;
    FocCore_LoadGains();
}

void Ident_Begin(void)
{
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdentTest.sum[0] = gIdentTest.sum[1] = 0;
    s_dtRetry = 0;
    s_dtDropMask = 0;
    s_dtSubIdx = 0;
    s_dtSubVq = 0;
    s_dtSubMin = 32767;
    s_dtSubMax = -32768;
    gIdentTest.phase = IDENT_PH_DT_STEP;
}

uint8_t Ident_Active(void)
{
    return (uint8_t)(gIdentTest.phase != IDENT_PH_NONE);
}

/* 三相死区方向几何因子 g(θ) = (2/3) Σ |cos(θ - 120°k)| */
static float dtGeomFactor(int16_t thetaQ15)
{
    float th = (float)thetaQ15 * (float)(2.0 * MATH_PI / 65536.0);
    return (2.0f / 3.0f) * (fabsf(cosf(th)) + fabsf(cosf(th - 2.0943951f)) + fabsf(cosf(th + 2.0943951f)));
}

static void dtFit(void)
{
    const IdentTest_TypeDef *t = &gIdentTest;
    /* 静态门限回归三相退出死区过渡带的物理边界（2.0×I0 / 3.5×I0），
     * 不再将 ±2%FS 零偏余量硬叠加到静态下限，改由低端自适应剥离与斜率一致性拦截 */
    #define DT_FIT_FLOOR    (2.0f * (float)INVERTER_DT_I0_A)
    #define DT_XMAX_MIN     (3.5f * (float)INVERTER_DT_I0_A)
    float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0, g, vbus, vd0, sse = 0, xMin = 1e9f, xMax = 0.0f;
    uint16_t i, fitDrop = 0, totDrop = 0;
    float xArr[IDENT_DT_N], yArr[IDENT_DT_N];
    uint16_t nFit = 0;
    for (i = 0; i < IDENT_DT_N; i++) {
        float x = t->dtId[i] * (float)(I_MAX / 32768.0), y = t->dtVd[i] * (float)(UDC_MAX / 32768.0);
        float xRef = (float)s_dtLevel[i] * (float)(I_MAX / 32768.0);
        if (s_dtDropMask & (1u << i)) {
            totDrop++;
            if (xRef >= 0.45f * (float)IDENT_I_SCALE) fitDrop++;
            continue;
        }
        if (x < 0.45f * (float)IDENT_I_SCALE || x < DT_FIT_FLOOR) continue;   /* 死区过渡带内的档位不进拟合 */
        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;
        sx += x; sy += y; sxx += x * x; sxy += x * y; n += 1.0f;
        xArr[nFit] = x; yArr[nFit] = y; nFit++;
    }
    /* 可辨识性边界 + 点数下限 3（两点拟合残差恒为 0，分半斜率也无法做） */
    if (fitDrop >= 2u || totDrop >= 4u || n < 3.0f || (n * sxx - sx * sx) <= 0.0f ||
        xMax < DT_XMAX_MIN) {
        gIdent.warn |= IDENT_WARN_DT_UNSTABLE;
        gIdent.rsOhm = 0.0f;
        gIdent.deadVoltV = 0.0f;
        gIdent.deadTimeS = 0.0f;
        return;
    }
    gIdent.rsOhm = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    vd0 = (sy - gIdent.rsOhm * sx) / n;
    /* 低端自适应剥离：若最低电流段局部斜率明显高于前 nFit-1 点 OLS 斜率，说明最低点仍受过渡带污染 */
    while (nFit >= 4u) {
        float dxTail = xArr[nFit - 2u] - xArr[nFit - 1u];
        float sTail, sxH = 0, syH = 0, sxxH = 0, sxyH = 0, dH, rsHi;
        float nH = (float)(nFit - 1u);
        if (dxTail <= 1e-6f) break;
        sTail = (yArr[nFit - 2u] - yArr[nFit - 1u]) / dxTail;
        for (i = 0; i < nFit - 1u; i++) {
            float x = xArr[i], y = yArr[i];
            sxH += x; syH += y; sxxH += x * x; sxyH += x * y;
        }
        dH = nH * sxxH - sxH * sxH;
        if (dH <= 1e-12f) break;
        rsHi = (nH * sxyH - sxH * syH) / dH;
        if (rsHi > 0.0f && sTail > 1.35f * rsHi) {
            nFit--;
            n = (float)nFit;
            sx = sxH; sy = syH; sxx = sxxH; sxy = sxyH;
            xMin = xArr[nFit - 1u];
            xMax = xArr[0];
            gIdent.rsOhm = rsHi;
            vd0 = (sy - gIdent.rsOhm * sx) / n;
        } else {
            break;
        }
    }
    /* 斜率一致性检查：nFit >= 4 做分半斜率比，nFit == 3 做上下两段斜率比 */
    if (nFit >= 4u) {
        float sLo = 0, sHi = 0;
        uint16_t half = nFit / 2u;
        float sxL=0, syL=0, sxxL=0, sxyL=0, sxH=0, syH=0, sxxH=0, sxyH=0;
        for (i = 0; i < half; i++) { sxL+=xArr[i]; syL+=yArr[i]; sxxL+=xArr[i]*xArr[i]; sxyL+=xArr[i]*yArr[i]; }
        for (i = half; i < nFit; i++) { sxH+=xArr[i]; syH+=yArr[i]; sxxH+=xArr[i]*xArr[i]; sxyH+=xArr[i]*yArr[i]; }
        { float dL = (float)half*sxxL - sxL*sxL; sLo = (dL > 1e-12f) ? ((float)half*sxyL - sxL*syL) / dL : 0.0f; }
        { float dH = (float)(nFit-half)*sxxH - sxH*sxH; sHi = (dH > 1e-12f) ? ((float)(nFit-half)*sxyH - sxH*syH) / dH : 0.0f; }
        if (sLo <= 0.0f || sHi <= 0.0f || ((sLo > sHi) ? sLo / sHi : sHi / sLo) > 1.45f) {
            gIdent.warn |= IDENT_WARN_DT_UNSTABLE;
            gIdent.rsOhm = 0.0f;
            gIdent.deadVoltV = 0.0f;
            gIdent.deadTimeS = 0.0f;
            return;
        }
    } else if (nFit == 3u) {
        float dx01 = xArr[0] - xArr[1], dx12 = xArr[1] - xArr[2];
        float s01 = (dx01 > 1e-6f) ? (yArr[0] - yArr[1]) / dx01 : 0.0f;
        float s12 = (dx12 > 1e-6f) ? (yArr[1] - yArr[2]) / dx12 : 0.0f;
        if (s01 <= 0.0f || s12 <= 0.0f || ((s01 > s12) ? s01 / s12 : s12 / s01) > 1.45f) {
            gIdent.warn |= IDENT_WARN_DT_UNSTABLE;
            gIdent.rsOhm = 0.0f;
            gIdent.deadVoltV = 0.0f;
            gIdent.deadTimeS = 0.0f;
            return;
        }
    }
    /* 残差：口径与拟合窗统一，按纯电阻压降跨度归一化 */
    for (i = 0; i < nFit; i++) {
        float e = yArr[i] - (gIdent.rsOhm * xArr[i] + vd0);
        sse += e * e;
    }
    {
        float rmse = sqrtf(sse / n);
        float vDropSpan = fabsf(gIdent.rsOhm * (xMax - xMin));
        if (gIdent.rsOhm <= 0.0f || rmse > 0.12f * vDropSpan + 0.08f) {
            gIdent.warn |= IDENT_WARN_DT_UNSTABLE;
            gIdent.rsOhm = 0.0f;
            gIdent.deadVoltV = 0.0f;
            gIdent.deadTimeS = 0.0f;
            return;
        }
    }
    g = dtGeomFactor(Ident_AlignTheta());
    gIdent.deadVoltV = vd0 / g;
    if (gIdent.deadVoltV < 0.0f) gIdent.deadVoltV = 0.0f;
    vbus = gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
    gIdent.deadTimeS = (vbus > 1.0f) ? gIdent.deadVoltV / ((float)PWM_FREQ * vbus) : 0.0f;
}

static void ipdFit(void)
{
    gIdent.ipdRatio = (float)gIpd.ratioQ8 / 256.0f;
}

/* ============================================================================
 * 交流阻抗（phase 6）：二自由度控制（2-DOF）= 复阻抗电压前馈 + 保守带宽电流 PI
 *
 * 原理：
 *   1. 注入轴控制律：
 *        v_d(k) = [V_ff,s * sin(phi + dly) + V_ff,c * cos(phi + dly)] * env + PI_safe(I_ref*env*sin(phi) - i_d)
 *      传递函数 I(jw)/I_ref(jw) = (Z_hat + C) / (Z_true + C) -> 1∠0°：
 *        - 在 f2 = PWM_FREQ/10 下无幅值衰减、无指令放大撞限幅；
 *        - 反馈环保持保守带宽（<= PWM_FREQ/60），对全电感范围与 0.5~2.5 拍延迟均保持 >65° 相位裕度；
 *        - 重测按实测复阻抗更新前馈（允许 zRe < 0 以直接补偿未知延迟相位），一步收敛；
 *        - 大幅值饱和波峰处比例项自动削峰抑谐波，触及硬件峰值安全限时平滑折半退让或截断有效档数。
 *   2. 同步提取基波与 3 次谐波电流，在 dfStep() 中用分部积分精确描述函数拟合 L(i)。
 *   3. 零偏置段（seg 0..5）按整周期子窗监测正交轴直流/低频电压，正交滤除注入耦合、无盲区捕捉风车旋转。
 * ==========================================================================*/
#define AC_NF           3
#define AC_NSEG         12
#define AC_SETTLE_MIN   4u
#define AC_INT_MIN      8u
#define AC_RAMP         2u
#define AC_V_DELAY      CTRL_INVPARK_ADVANCE
#define AC_F_MIN_HZ     25.0f
#define AC_DT_SNR       2.0f
#define AC_SNR_MIN      0.60f

static uint16_t s_acPer[AC_NF + 1];
static float    s_acHz[AC_NF + 1];
static const uint8_t s_acSegQ[AC_NSEG] = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
static const uint8_t s_acSegF[AC_NSEG] = { 0, 1, 2, 0, 1, 2, 3, 0, 0, 0, 0, 3 };
#define AC_S_Q04    3
#define AC_S_Q04L   6
#define AC_S_Q10    7
#define AC_S_Q10L   11
static const int16_t s_acSegA[AC_NSEG] = {
    IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4),
    IDENT_Q15_I(0.4), IDENT_Q15_I(1.0), IDENT_Q15_I(1.5), IDENT_Q15_I(2.0), IDENT_Q15_I(2.5), IDENT_Q15_I(1.0) };
static const int16_t s_acSegB[AC_NSEG] = { 0, 0, 0, 0, 0, 0,
    0, IDENT_Q15_I(0.1), IDENT_Q15_I(0.225), IDENT_Q15_I(0.4), IDENT_Q15_I(0.625), IDENT_Q15_I(0.1) };

static volatile uint16_t s_acSeg;
static int16_t  s_acAmp, s_acBias;          /* s_acAmp: 交流段为目标电流幅值(Q15)，R/L 段为电压幅值(Q15) */
static int16_t  s_acVffS, s_acVffC;         /* 2-DOF 阻抗电压前馈：同相与正交（Q15 电压） */
static int16_t  s_acIpkLim, s_acIvecLim, s_acIpk; /* 瞬时单轴/合成矢量保护阈值与实测峰值（Q15 电流） */
static int32_t  s_acEnv, s_acEnvStep;
static uint32_t s_acPhi, s_acInc, s_acDly;
static uint32_t s_acT, s_acSettleT, s_acIntT;
static int16_t  s_acTheta;
static int32_t  s_acVs, s_acVc, s_acIs, s_acIc, s_acEs, s_acEc;
static int32_t  s_acI3s, s_acI3c;
/* 零偏置段正交轴整周期子窗静止监测（消除注入频率耦合，无盲区捕捉风车） */
static int32_t  s_acVqSum, s_acVqSubAcc, s_acVqSubMin, s_acVqSubMax;
static uint32_t s_acVqSubT, s_acVqSubCnt;
static float    s_acVqAxMin, s_acVqAxMax;
static int16_t  s_acVph;
#define AC_DT_KI_Q8 ((int32_t)(256.0 * I_MAX / INVERTER_DT_I0_A + 0.5))

static float    s_acZre[AC_NF], s_acZim[AC_NSEG];
static float    s_zCmdR[AC_NF], s_zCmdI[AC_NF], s_zDtR[AC_NF], s_zDtI[AC_NF];
static float    s_acDlyShift, s_acI00, s_acZreQ04, s_acI04L, s_acI10L;
static float    s_acH3s[5], s_acH3c[5];
static float    s_acTierL[4];
static float    s_acFfS, s_acFfC, s_acRefA; /* 当前段目标电流(A)与前馈电压(V)，用于重测更新 */
static uint8_t  s_acRetry, s_acOverI, s_acMotion;
static int16_t  s_acPlanA[4], s_acPlanB[4];
static uint8_t  s_acSatN, s_l0Only;         /* s_acSatN: 有效 q 轴饱和测量点数(1..5)；s_l0Only: 仅提交 L0 */
static float    s_acMaxPkA;
static volatile uint8_t s_acDone;

static float acVbusV(void)
{
    return (gMeas.vbusAvg > 0) ? (float)gMeas.vbusAvg * (float)(UDC_MAX / 32768.0) : (float)UDC_RATE;
}

static float acVlimV(void)
{
    return 0.92f * (float)MAX_DUTY * (2.0f / 3.0f) * acVbusV();
}

static int16_t acSegTargetA(uint16_t seg)
{
    if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) return s_acPlanA[seg - AC_S_Q10];
    return s_acSegA[seg];
}

static int16_t acSegTargetB(uint16_t seg)
{
    if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) return s_acPlanB[seg - AC_S_Q10];
    return s_acSegB[seg];
}

static void acSetFreq(uint16_t idx, float hz)
{
    uint32_t per = (hz > 0.0f) ? (uint32_t)((float)PWM_FREQ / hz + 0.5f) : 3750u;
    uint32_t maxPer = 30000u / AC_INT_MIN;
    if (per < 10u) per = 10u;
    if (per > maxPer) per = maxPer;
    s_acPer[idx] = (uint16_t)per;
    s_acHz[idx] = (float)PWM_FREQ / (float)per;
}

/* 保守反馈 PI 带宽：有 2-DOF 前馈承担主电压，反馈环仅需 <= PWM_FREQ/60（额外延迟时 /80），
 * 即使强饱和使增量电感降至 0.20*L0，瞬时穿越频率仍 <= PWM_FREQ/12，全工况稳定且有效削峰 */
static void acSetupPI(float lD, float lQ, float r)
{
    float bwMax = (s_acDlyShift < -0.25f) ? ((float)PWM_FREQ / 80.0f) : ((float)PWM_FREQ / 60.0f);
    float bw = (float)CURRENT_BW_HZ;
    if (bw > bwMax) bw = bwMax;
    if (lD > 0.0f && lQ > 0.0f && r > 0.0f) {
        int32_t kpD = (int32_t)(2.0f * (float)MATH_PI * bw * lD * (float)CUR_GAIN_SCALE + 0.5f);
        int32_t kpQ = (int32_t)(2.0f * (float)MATH_PI * bw * lQ * (float)CUR_GAIN_SCALE + 0.5f);
        int32_t ki  = (int32_t)(2.0f * (float)MATH_PI * bw * r  * (float)CUR_GAIN_SCALE / (float)PWM_FREQ + 0.5f);
        if (ki < 1) ki = 1;
        gFoc.piD.qKp = kpD > 65535 ? 65535 : kpD;
        gFoc.piQ.qKp = kpQ > 65535 ? 65535 : kpQ;
        gFoc.piD.qKi = ki  > 65535 ? 65535 : ki;
        gFoc.piQ.qKi = gFoc.piD.qKi;
    }
}

static void acInitFreqs(void)
{
    float r = gIdent.rsOhm > 0.0f ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
    float l = gIdent.rlL > 0.0f ? gIdent.rlL : (float)gParam.lqH;
    float iInj = (float)s_acSegA[0] * (float)(I_MAX / 32768.0);
    float iMax = (float)s_acSegA[AC_S_Q10 + 3] * (float)(I_MAX / 32768.0);
    float vDt = gIdent.deadVoltV > 0.0f ? gIdent.deadVoltV : 0.0f;
    float vLim = acVlimV();
    float f0 = (l > 0.0f) ? r / (2.0f * (float)MATH_PI * l) : AC_F_MIN_HZ;
    float fDt = (l > 0.0f && iInj > 0.0f) ? (AC_DT_SNR * vDt) / (2.0f * (float)MATH_PI * l * iInj) : 0.0f;
    float fMax0 = (float)PWM_FREQ / 20.0f;
    float fMaxHi = (float)PWM_FREQ / 10.0f;
    float f2, f1;
    uint16_t f;
    if (fDt > f0) f0 = fDt;
    if (l > 0.0f && iMax > 0.0f) {
        float vR = r * iMax + (4.0f / (float)MATH_PI) * 1.3333f * vDt;
        float vX = (vLim > vR) ? sqrtf(vLim * vLim - vR * vR) : 0.0f;
        float fV = vX / (2.0f * (float)MATH_PI * l * iMax);
        if (f0 > fV) f0 = fV;
    }
    if (f0 < AC_F_MIN_HZ) f0 = AC_F_MIN_HZ;
    if (f0 > fMax0) f0 = fMax0;
    acSetFreq(0, f0);
    f0 = s_acHz[0];
    f2 = (4.0f * f0 <= fMaxHi) ? 4.0f * f0 : fMaxHi;
    f1 = 2.0f * f0;
    if (f1 > 0.5f * (f0 + f2)) f1 = sqrtf(f0 * f2);
    acSetFreq(1, f1);
    acSetFreq(2, f2);
    acSetFreq(3, 0.5f * f0);
    for (f = 0; f <= AC_NF; f++) gIdent.acHz[f] = s_acHz[f];
    for (f = 0; f < 4u; f++) {
        s_acPlanA[f] = s_acSegA[AC_S_Q10 + f];
        s_acPlanB[f] = s_acSegB[AC_S_Q10 + f];
        s_acTierL[f] = l;
    }
    for (f = 0; f < 5u; f++) { s_acH3s[f] = 0.0f; s_acH3c[f] = 0.0f; }
    s_acSatN = 5u;
    s_l0Only = 0;
    s_acMotion = 0;
    s_acDlyShift = 0.0f;
    s_acI00 = 0.0f;
    s_acZreQ04 = 0.0f;
    s_acI04L = 0.0f;
    s_acI10L = 0.0f;
    s_acMaxPkA = 0.0f;
    gIdent.snrMin = 1e9f;
    acSetupPI(l, l, r);
}

static void acEvalDlyPoint(uint16_t f, float sh, float *zrOut, float *ziOut)
{
    float ph = sh * (2.0f * (float)MATH_PI * s_acHz[f] / (float)PWM_FREQ);
    float c = cosf(ph), s = sinf(ph);
    *zrOut = c * s_zCmdR[f] - s * s_zCmdI[f] - s_zDtR[f];
    *ziOut = s * s_zCmdR[f] + c * s_zCmdI[f] - s_zDtI[f];
}

static float acDlyCost(float sh, float rRef)
{
    float zr[AC_NF], zi[AC_NF], rAvg = 0.0f, cost = 0.0f;
    uint16_t f;
    for (f = 0; f < AC_NF; f++) {
        acEvalDlyPoint(f, sh, &zr[f], &zi[f]);
        if (zi[f] <= 0.0f || zr[f] <= -0.2f * rRef) return -1.0f;
        rAvg += zr[f];
    }
    rAvg /= (float)AC_NF;
    for (f = 0; f < AC_NF; f++) {
        float dv = zr[f] - rAvg;
        float dr = zr[f] - rRef;
        cost += dv * dv + 0.15f * dr * dr;
    }
    return cost;
}

static void acCalDelay(void)
{
    float rRef = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
    float baseCost = acDlyCost(0.0f, rRef), bestCost = 1e30f, bestSh = 0.0f;
    int16_t i;
    uint16_t f;
    for (i = -30; i <= 30; i++) {
        float sh = 0.05f * (float)i, c = acDlyCost(sh, rRef);
        if (c >= 0.0f && c < bestCost) { bestCost = c; bestSh = sh; }
    }
    if (bestCost < 1e29f) {
        float c0 = bestSh;
        for (i = -10; i <= 10; i++) {
            float sh = c0 + 0.005f * (float)i, c;
            if (sh < -1.5f || sh > 1.5f) continue;
            c = acDlyCost(sh, rRef);
            if (c >= 0.0f && c < bestCost) { bestCost = c; bestSh = sh; }
        }
        if (baseCost >= 0.0f && bestCost >= 0.5f * baseCost && fabsf(bestSh) < 0.15f) bestSh = 0.0f;
        s_acDlyShift = bestSh;
    } else {
        s_acDlyShift = 0.0f;
    }
    gIdent.acDlyShift = s_acDlyShift;
    gIdent.acDlyTicks = (float)AC_V_DELAY - s_acDlyShift;
    if (fabsf(s_acDlyShift) > 0.25f) {
        /* 仅作诊断告警：实板硬件采样/滤波延时叠加高频铁损涡流可产生非整数拍偏移
         * （实测 acDlyShift ≈ 0.69 拍，校准后三频点 acRohm 与 Rs 吻合在 2% 内），不置 AC_DRIFT 阻断 */
        gIdent.warn |= IDENT_WARN_DLY_MISMATCH;
    }
    for (f = 0; f < AC_NF; f++) {
        float zr, zi, w = 2.0f * (float)MATH_PI * s_acHz[f], z2, rUse;
        acEvalDlyPoint(f, s_acDlyShift, &zr, &zi);
        s_acZre[f] = zr;
        s_acZim[f] = zi;
        z2 = zr * zr + zi * zi;
        rUse = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : zr;
        gIdent.acLdMagH[f] = (z2 > rUse * rUse && w > 0.0f) ? sqrtf(z2 - rUse * rUse) / w : 0.0f;
    }
}

static void acPlanSatTiers(float lqEst, float rEst)
{
    float vLim = acVlimV();
    float vDt = (gIdent.deadVoltV > 0.0f) ? (4.0f / (float)MATH_PI) * 1.3333f * gIdent.deadVoltV : 0.0f;
    float w0 = 2.0f * (float)MATH_PI * s_acHz[0];
    float sc = (float)IDENT_I_SCALE;
    float bx = gIdentCfg.biasX ? (float)gIdentCfg.biasX : 1.0f;
    float z2 = rEst * rEst + (w0 * lqEst) * (w0 * lqEst);
    float iTopV = 2.5f * sc, iTopI = 2.5f * sc, iTop;

    if (z2 > 1e-12f) {
        float vl = 0.95f * vLim;
        float c = vDt * vDt - vl * vl;
        if (c < 0.0f) {
            float b = rEst * vDt;
            float disc = b * b - z2 * c;
            iTopV = (disc > 0.0f) ? (-b + sqrtf(disc)) / z2 : 0.0f;
        } else {
            iTopV = 0.0f;
        }
    }
    if (bx > 1.0f && sc > 0.0f) {
        float limA = (float)IDENT_VEC_MAX_A * 0.98f;
        float lo = 0.0f, hi = 2.5f * sc;
        uint16_t it;
        for (it = 0; it < 12u; it++) {
            float mid = 0.5f * (lo + hi);
            float r = mid / sc;
            float b1 = 0.1f * r * r * sc * bx;
            if (mid * mid + b1 * b1 <= limA * limA) lo = mid; else hi = mid;
        }
        iTopI = lo;
    }
    iTop = (iTopV < iTopI) ? iTopV : iTopI;
    if (iTop > 2.5f * sc) iTop = 2.5f * sc;

    if (iTop >= 2.40f * sc) return;
    if (gIdent.lqAtA[0] <= 0.0f || iTop < 1.8f * gIdent.lqAtA[0]) {
        s_l0Only = 1u;
        return;
    }
    {
        float iLow = 1.4f * gIdent.lqAtA[0];
        float q;
        uint16_t k;
        if (iLow < 0.7f * sc && iTop >= 1.2f * sc) iLow = 0.7f * sc;
        if (iTop < 1.5f * iLow) { s_l0Only = 1u; return; }
        q = powf(iTop / iLow, 1.0f / 3.0f);
        for (k = 0; k < 4u; k++) {
            float tk = (k == 3u) ? iTop : (iLow * powf(q, (float)k));
            float rk = (sc > 0.0f) ? (tk / sc) : 0.0f;
            float b1 = 0.1f * rk * rk * sc;
            int32_t qA  = (int32_t)(tk / (float)I_MAX * 32768.0f + 0.5f);
            int32_t qB1 = (int32_t)(b1 / (float)I_MAX * 32768.0f + 0.5f);
            if (qA > 32767) qA = 32767;
            if (qB1 > 32767) qB1 = 32767;
            s_acPlanA[k] = (int16_t)qA;
            s_acPlanB[k] = (int16_t)qB1;
        }
    }
}

/* 2-DOF 初始前馈电压矢量预测：(V_ff,s = R*I + V_dt1, V_ff,c = w*L*I) */
static void acSegInitFF(uint16_t seg, int16_t thetaQ15, float refA)
{
    uint16_t f = s_acSegF[seg];
    float w = 2.0f * (float)MATH_PI * s_acHz[f];
    float r = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
    float lEst = (gIdent.rlL > 0.0f) ? gIdent.rlL : (float)gParam.lqH;
    float eEst = 0.0f;

    if (gIdent.deadVoltV > 0.0f) {
        float k0 = (refA < (float)INVERTER_DT_I0_A) ? (refA / (float)INVERTER_DT_I0_A) : 1.0f;
        eEst = (4.0f / (float)MATH_PI) * dtGeomFactor(thetaQ15) * gIdent.deadVoltV * k0;
    }
    if (seg == AC_S_Q04L) {
        if (s_acZim[AC_S_Q04] > 0.0f) lEst = s_acZim[AC_S_Q04] / (2.0f * (float)MATH_PI * s_acHz[0]);
    } else if (seg == AC_S_Q10L) {
        lEst = s_acTierL[0];
    } else if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) {
        uint16_t k = (uint16_t)(seg - AC_S_Q10);
        if (k == 0u) {
            if (s_acZim[AC_S_Q04] > 0.0f) lEst = s_acZim[AC_S_Q04] / (2.0f * (float)MATH_PI * s_acHz[0]);
        } else if (k == 1u) {
            lEst = s_acTierL[0];
        } else {
            /* 线性外推饱和趋势，避免高档首次前馈偏高引起波峰尖冲过流 */
            float l1 = s_acTierL[k - 1u], l0 = s_acTierL[k - 2u];
            float dI = gIdent.lqAtA[k] - gIdent.lqAtA[k - 1u];
            lEst = l1;
            if (l1 < l0 && dI > 0.01f && refA > gIdent.lqAtA[k]) {
                lEst = l1 - (l0 - l1) * (refA - gIdent.lqAtA[k]) / dI;
                if (lEst < 0.5f * l1) lEst = 0.5f * l1;
            }
        }
    } else if (seg >= 1u && seg < AC_NF) {
        if (s_acZim[0] > 0.0f) lEst = s_acZim[0] / (2.0f * (float)MATH_PI * s_acHz[0]);
    } else if (seg == AC_S_Q04) {
        if (s_acZim[0] > 0.0f) lEst = s_acZim[0] / (2.0f * (float)MATH_PI * s_acHz[0]);
    } else if (seg > AC_S_Q04 && seg < AC_S_Q04L) {
        if (s_acZim[AC_S_Q04] > 0.0f) lEst = s_acZim[AC_S_Q04] / (2.0f * (float)MATH_PI * s_acHz[0]);
    }
    s_acFfS = r * refA + eEst;
    s_acFfC = w * lEst * refA;
}

static void acSegStart(uint16_t seg, uint8_t isRetry)
{
    uint16_t f = s_acSegF[seg];
    uint16_t per = s_acPer[f];
    uint32_t nSettle, nInt, subCyc;
    double dlyTicks = (double)AC_V_DELAY - (double)s_acDlyShift;
    int32_t bx = gIdentCfg.biasX ? (int32_t)gIdentCfg.biasX : 1;
    float ki = (float)(I_MAX / 32768.0);
    float refA;

    if (dlyTicks < 0.0) dlyTicks = 0.0;
    s_acTheta = (int16_t)(Ident_AlignTheta() + (s_acSegQ[seg] ? ANGLE_90_Q15 : 0));

    /* 目标基波电流（0.5*f0 段精确对齐对应 f0 档的实测幅值） */
    if (!isRetry) {
        if (seg == AC_S_Q04L) refA = gIdent.lqAtA[0];
        else if (seg == AC_S_Q10L) refA = gIdent.lqAtA[1];
        else refA = (float)acSegTargetA(seg) * ki;
        s_acRefA = refA;
        s_acRetry = 0;
        acSegInitFF(seg, s_acTheta, refA);
    } else {
        refA = s_acRefA;
    }

    /* 电流指令与偏置电流限幅（2-DOF 无需放大 I_cmd，始终等于真实目标电流） */
    {
        int32_t lim = (int32_t)((float)IDENT_VEC_MAX_A / (float)I_MAX * 32768.0f + 16.5f);
        int32_t amp = QMath_Clamp((int32_t)(refA / ki + 0.5f), lim);
        int64_t b = -(int64_t)acSegTargetB(seg) * bx;
        int32_t maxB = (int32_t)sqrtf((float)((double)lim * lim - (double)amp * amp));
        float iSafeMax = 0.94f * (float)((I_MAX < BOARD_HOCP_A) ? I_MAX : BOARD_HOCP_A);
        int32_t iInstMax = (int32_t)(iSafeMax / (float)I_MAX * 32768.0f);
        int32_t pkLim;
        if (b > maxB) b = maxB;
        if (b < -maxB) b = -maxB;
        s_acAmp = (int16_t)amp;
        s_acBias = (int16_t)b;

        if (iInstMax > 31800) iInstMax = 31800;
        s_acIvecLim = (int16_t)iInstMax;
        pkLim = (iInstMax > abs((int32_t)b))
              ? (int32_t)sqrtf((float)((double)iInstMax * iInstMax - (double)b * b))
              : 1000;
        s_acIpkLim = (int16_t)(pkLim > 32767 ? 32767 : pkLim);
    }

    /* 前馈电压矢量限幅（留 10% 余量给反馈 PI 与直流偏置轴） */
    {
        float vLim = 0.90f * acVlimV();
        float vMag = sqrtf(s_acFfS * s_acFfS + s_acFfC * s_acFfC);
        float scV = (vMag > vLim && vMag > 0.0f) ? (vLim / vMag) : 1.0f;
        float kq = (float)(32768.0 / UDC_MAX);
        s_acVffS = (int16_t)QMath_Clamp((int32_t)(s_acFfS * scV * kq + (s_acFfS >= 0.0f ? 0.5f : -0.5f)), 32767);
        s_acVffC = (int16_t)QMath_Clamp((int32_t)(s_acFfC * scV * kq + (s_acFfC >= 0.0f ? 0.5f : -0.5f)), 32767);
    }

    /* 高频段自动增加稳定与积分周期数（最低稳定 25 ms、最低积分 50 ms，偶数周期便于整周期子窗） */
    nSettle = (uint32_t)(0.025f * s_acHz[f] + 0.5f);
    if (nSettle < AC_SETTLE_MIN) nSettle = AC_SETTLE_MIN;
    nInt = (uint32_t)(0.050f * s_acHz[f] + 0.5f);
    if (nInt < AC_INT_MIN) nInt = AC_INT_MIN;
    if (nInt * (uint32_t)per > 30000u) nInt = 30000u / (uint32_t)per;
    if (nInt < AC_INT_MIN) nInt = AC_INT_MIN;
    if (nInt & 1u) nInt--;
    s_acSettleT = nSettle * (uint32_t)per;
    s_acIntT = nInt * (uint32_t)per;

    /* 零偏置段正交轴整周期子窗长度（约 15 ms，严格为交流周期的整数倍） */
    subCyc = (uint32_t)(0.015f * s_acHz[f]);
    if (subCyc < 1u) subCyc = 1u;
    if (subCyc > (nInt >> 1)) subCyc = nInt >> 1;
    s_acVqSubT = subCyc * (uint32_t)per;
    s_acVqSubCnt = 0;
    s_acVqSubAcc = 0;
    s_acVqSubMin = 32767;
    s_acVqSubMax = -32768;
    s_acVqSum = 0;

    gFoc.piD.qdSum = 0;
    if (seg == 0u || seg == AC_S_Q04) gFoc.piQ.qdSum = 0;

    s_acEnv = 0;
    s_acEnvStep = (int32_t)(32768L / ((int32_t)AC_RAMP * (int32_t)per)) + 1;
    s_acInc = (uint32_t)(4294967296.0 / (double)per + 0.5);
    s_acDly = (uint32_t)(dlyTicks * (double)s_acInc + 0.5);
    s_acPhi = 0;
    s_acT = 0;
    s_acIpk = 0;
    s_acOverI = 0;
    s_acVs = s_acVc = s_acIs = s_acIc = s_acEs = s_acEc = 0;
    s_acI3s = s_acI3c = 0;
    s_acVph = (int16_t)(gIdent.deadVoltV * (float)(32768.0 / UDC_MAX));
    COMPILER_BARRIER();    /* 编译器屏障：确保上面的非 volatile 写对快环可见后再清 s_acDone */
    s_acDone = 0;
    s_acSeg = seg;
}

static void acFast(void)
{
    uint16_t seg = s_acSeg;
    if (seg < AC_NSEG && !s_acDone) {
        uint32_t phiOld = s_acPhi;
        SinCos_TypeDef aOld = FocMath_SinCos((int16_t)(phiOld >> 16));
        SinCos_TypeDef b = FocMath_SinCos((int16_t)((phiOld + s_acDly) >> 16));
        int32_t vmax, vq, vd, vff, limD, idAbs, ref;

        /* 1. 瞬时死区电压沿注入轴投影（与本拍采样电流同时刻） */
        if (s_acT >= s_acSettleT) {
            int32_t ia = gMeas.iab.s16Alpha, ibs = ((int32_t)gMeas.iab.s16Beta * Q15_SQRT3_HALF) >> 15;
            int32_t xa, xb, xc, ea, eb, e;
            SinCos_TypeDef d = FocMath_SinCos(s_acTheta);
            xa = QMath_Clamp((ia * AC_DT_KI_Q8) >> 8, 32767);
            xb = QMath_Clamp(((-ia / 2 + ibs) * AC_DT_KI_Q8) >> 8, 32767);
            xc = QMath_Clamp(((-ia / 2 - ibs) * AC_DT_KI_Q8) >> 8, 32767);
            ea = ((2 * xa - xb - xc) * Q15_ONE_THIRD) >> 15;
            eb = ((xb - xc) * Q15_INV_SQRT3) >> 15;
            e = (((ea * d.s16Cos + eb * d.s16Sin) >> 15) * s_acVph) >> 15;
            s_acEs += (e * aOld.s16Sin) >> 15;
            s_acEc += (e * aOld.s16Cos) >> 15;
        }

        /* 2. Park 变换 */
        gFoc.theta = s_acTheta;
        gFoc.thetaInv = s_acTheta;
        gFoc.ang = FocMath_SinCos(s_acTheta);
        FocMath_Park(&gMeas.iab, &gFoc.ang, &gFoc.idq);

        /* 3. 瞬时峰值硬保护：比较合成电流 id²+iq² 与 iInstMax²（含偏置轴 PI 纹波和交叉饱和） */
        idAbs = gFoc.idq.s16D < 0 ? -(int32_t)gFoc.idq.s16D : (int32_t)gFoc.idq.s16D;
        {
            int32_t iqAbs = gFoc.idq.s16Q < 0 ? -(int32_t)gFoc.idq.s16Q : (int32_t)gFoc.idq.s16Q;
            int32_t iVecQ15 = (int32_t)s_acIvecLim;
            if (s_acT > 2u && (idAbs > s_acIpkLim ||
                ((int64_t)idAbs * idAbs + (int64_t)iqAbs * iqAbs) > (int64_t)iVecQ15 * iVecQ15)) {
                s_acOverI = 1u;
                s_acDone = 1u;
                FocCore_ApplyVoltageAB(0, 0);
                return;
            }
        }

        /* 4. 2-DOF 控制器：偏置轴 PI + 注入轴 [复阻抗前馈 + 保守 PI 削峰/归零] */
        {
            int16_t s16Temp = QMath_MulQ15(FOC_DUTY_LIMIT_Q15, gMeas.vbusAvg);
            vmax = QMath_MulQ15(s16Temp, Q15(0.6667));
        }
        gFoc.vmax = (int16_t)vmax;

        gFoc.piQ.qOutMax = vmax;
        gFoc.piQ.qOutMin = -vmax;
        gFoc.piQ.qInRef  = s_acBias;
        gFoc.piQ.qInMeas = gFoc.idq.s16Q;
        Pi_Run32(&gFoc.piQ);
        vq = gFoc.piQ.qOut;

        s_acPhi += s_acInc;
        if (s_acEnv < 32767) { s_acEnv += s_acEnvStep; if (s_acEnv > 32767) s_acEnv = 32767; }
        /* ref 与本拍采样电流 id 均对齐 phiOld（0 拍相位误差），vff 对齐电压生效相位 b = phiOld + dly */
        ref = ((((int32_t)s_acAmp * s_acEnv) >> 15) * aOld.s16Sin) >> 15;
        vff = (((int32_t)s_acVffS * b.s16Sin + (int32_t)s_acVffC * b.s16Cos) >> 15);
        vff = (vff * s_acEnv) >> 15;

        limD = FocMath_SqrtQ15(vmax * vmax - vq * vq);
        gFoc.piD.qOutMax = QMath_Clamp(limD - vff, 32767);
        gFoc.piD.qOutMin = QMath_Clamp(-limD - vff, 32767);
        gFoc.piD.qInRef  = (int16_t)ref;
        gFoc.piD.qInMeas = gFoc.idq.s16D;
        Pi_Run32(&gFoc.piD);
        vd = QMath_Clamp(gFoc.piD.qOut + vff, limD);

        gFoc.vdq.s16D = (int16_t)vd;
        gFoc.vdq.s16Q = (int16_t)vq;
        FocMath_InvPark(&gFoc.vdq, &gFoc.ang, &gFoc.vab);
        FocCore_ApplyVoltageAB(gFoc.vab.s16Alpha, gFoc.vab.s16Beta);

        /* 5. 稳态正交积分（基波电流/电压 + 3 次谐波电流 + 正交轴整周期子窗静止监测） */
        if (s_acT >= s_acSettleT) {
            SinCos_TypeDef a3 = FocMath_SinCos((int16_t)((phiOld * 3u) >> 16));
            s_acIs  += ((int32_t)gFoc.idq.s16D * aOld.s16Sin) >> 15;
            s_acIc  += ((int32_t)gFoc.idq.s16D * aOld.s16Cos) >> 15;
            s_acI3s += ((int32_t)gFoc.idq.s16D * a3.s16Sin) >> 15;
            s_acI3c += ((int32_t)gFoc.idq.s16D * a3.s16Cos) >> 15;
            s_acVs  += ((int32_t)gFoc.vdq.s16D * b.s16Sin) >> 15;
            s_acVc  += ((int32_t)gFoc.vdq.s16D * b.s16Cos) >> 15;
            if (idAbs > s_acIpk) s_acIpk = (int16_t)idAbs;
            if (seg < AC_S_Q04L || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
                if (seg < AC_S_Q04L) s_acVqSum += vq;
                s_acVqSubAcc += vq;
                if (++s_acVqSubCnt >= s_acVqSubT) {
                    int32_t avgQ = s_acVqSubAcc / (int32_t)s_acVqSubT;
                    if (avgQ < s_acVqSubMin) s_acVqSubMin = avgQ;
                    if (avgQ > s_acVqSubMax) s_acVqSubMax = avgQ;
                    s_acVqSubAcc = 0;
                    s_acVqSubCnt = 0;
                }
            }
        }
        if (++s_acT >= s_acSettleT + s_acIntT) s_acDone = 1;
        return;
    }
    FocCore_SetIdqRef(0, (seg < AC_NSEG) ? s_acBias : 0);
    gFoc.theta = s_acTheta;
    gFoc.thetaInv = s_acTheta;
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

/* 基于实测复阻抗的 2-DOF 前馈更新（返回 1 = 需重测，0 = 完成） */
static uint8_t acCheckAndUpdateFF(uint16_t seg, float ref, float accTol, float failTol, uint8_t maxRetry,
                                  float iAmp, float vAmp, float zRe, float zIm, float eAmp)
{
    float vLim = 0.88f * acVlimV();
    if (ref <= 0.0f) return 0;
    if (fabsf(iAmp - ref) <= accTol * ref) return 0;
    if (iAmp < ref && vAmp >= vLim) {
        if (seg < AC_S_Q04L) gIdent.warn |= IDENT_WARN_LOW_SNR;
        return 0;
    }
    if (s_acRetry >= maxRetry) {
        if (failTol > 0.0f && fabsf(iAmp - ref) > failTol * ref)
            gIdent.warn |= IDENT_WARN_AMP_FAIL;
        return 0;
    }
    if (iAmp > 0.05f * ref) {
        /* 保留 zRe 符号：在延迟校准前（seg 0..2）额外延迟会使 zRe < 0，直接带入即可在 1 步内补偿相位 */
        s_acFfS = zRe * ref + eAmp;
        s_acFfC = zIm * ref;
    } else {
        s_acFfS = (s_acFfS > 0.05f) ? (1.6f * s_acFfS) : 0.5f;
        s_acFfC *= 1.6f;
    }
    s_acRetry++;
    return 1;
}

static uint8_t acSegFinish(uint16_t seg)
{
    float vr = (float)(s_acVs - s_acEs), vi = (float)(s_acVc - s_acEc);
    float ir = (float)s_acIs, ii = (float)s_acIc, d = ir * ir + ii * ii;
    float kv = (float)(UDC_MAX / 32768.0), ki = (float)(I_MAX / 32768.0), kvi = kv / ki;
    float nInt = (float)s_acIntT;
    float iAmp = 0.0f, vAmp = 0.0f, eAmp = 0.0f, zRe = 0.0f, zIm = 0.0f;

    /* 瞬时过流硬保护触发处理 */
    if (s_acOverI) {
        if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) {
            uint16_t k = (uint16_t)(seg - AC_S_Q10);
            float prevI = gIdent.lqAtA[k];
            float nextRef = 0.5f * (prevI + s_acRefA);
            if (s_acRetry < 2u && nextRef >= 1.12f * prevI) {
                float sc = nextRef / s_acRefA;
                s_acRefA = nextRef;
                s_acFfS *= sc;
                s_acFfC *= 0.80f * sc;
                s_acRetry++;
                return 1;
            }
            /* 已达硬件安全峰值上限且无更多递增余量：截断至前 k 档，保留已有高质量饱和点 */
            s_acSatN = (uint8_t)(k + 1u);
            return 0;
        }
        if (s_acRetry < 2u) {
            s_acRefA *= 0.80f;
            s_acFfS *= 0.80f;
            s_acFfC *= 0.80f;
            s_acRetry++;
            return 1;
        }
        gIdent.warn |= IDENT_WARN_AC_FAIL;
        return 0;
    }

    if (d <= 0.0f) {
        if (acCheckAndUpdateFF(seg, s_acRefA, 0.15f, 0.25f, 2u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)) return 1;
        s_acZim[seg] = 0.0f;
        if (seg < AC_NF) s_zCmdR[seg] = s_zCmdI[seg] = s_zDtR[seg] = s_zDtI[seg] = s_acZre[seg] = 0.0f;
        gIdent.warn |= IDENT_WARN_AC_FAIL;
        return 0;
    }
    iAmp = 2.0f * sqrtf(d) / nInt * ki;
    vAmp = 2.0f * sqrtf((float)s_acVs * (float)s_acVs + (float)s_acVc * (float)s_acVc) / nInt * kv;
    eAmp = 2.0f * sqrtf((float)s_acEs * (float)s_acEs + (float)s_acEc * (float)s_acEc) / nInt * kv;
    zRe = (vr * ir + vi * ii) / d * kvi;
    zIm = (vi * ir - vr * ii) / d * kvi;

    if (seg < AC_S_Q04L) {
        if (acCheckAndUpdateFF(seg, s_acRefA, 0.15f, 0.25f, 2u, iAmp, vAmp, zRe, zIm, eAmp)) return 1;
    } else if (seg == AC_S_Q04L || seg == AC_S_Q10L) {
        float ref = (seg == AC_S_Q04L) ? gIdent.lqAtA[0] : gIdent.lqAtA[1];
        if (acCheckAndUpdateFF(seg, ref, 0.05f, 0.0f, 2u, iAmp, vAmp, zRe, zIm, eAmp)) return 1;
    } else if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) {
        if (acCheckAndUpdateFF(seg, s_acRefA, 0.15f, 0.0f, 2u, iAmp, vAmp, zRe, zIm, eAmp)) return 1;
    }

    /* 正交轴整周期静止监测：
     * - 零偏置段 (seg < AC_S_Q04L)：静止时正交轴无直流与低频电压，检查子窗峰峰值与段间均值漂移
     * - q 轴扫幅段 (AC_S_Q10..+3)：d 轴有恒定直流偏置，检查子窗峰峰值 vqPP（抵消直流压降与死区）以捕捉转子失稳跑飞 */
    if (seg < AC_S_Q04L || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
        float vqPP = (s_acVqSubMax >= s_acVqSubMin) ? (float)(s_acVqSubMax - s_acVqSubMin) * kv : 0.0f;
        float rUse = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
        float vThr = 0.20f * gIdent.deadVoltV + 0.02f * rUse * iAmp + 0.15f;
        if (seg < AC_S_Q04L) {
            float vqMean = (float)s_acVqSum / nInt * kv;
            if (seg == 0u || seg == AC_S_Q04) {
                s_acVqAxMin = vqMean;
                s_acVqAxMax = vqMean;
            } else {
                if (vqMean < s_acVqAxMin) s_acVqAxMin = vqMean;
                if (vqMean > s_acVqAxMax) s_acVqAxMax = vqMean;
            }
            if (vqPP > vThr || (s_acVqAxMax - s_acVqAxMin) > vThr)
                s_acMotion = 1u;
        } else {
            /* 真实 IPM 铁芯交叉饱和天然在正交轴感应 2·f0 电压分量，故不监测 2·f0，仅用子窗峰峰值 vqPP 监测失锁跑飞 */
            if (vqPP > 2.0f * vThr)
                s_acMotion = 1u;
        }
    }

    if (seg < AC_NF) {
        float cr = (float)s_acVs, ci = (float)s_acVc, er = (float)s_acEs, ei = (float)s_acEc;
        s_zCmdR[seg] = (cr * ir + ci * ii) / d * kvi;
        s_zCmdI[seg] = (ci * ir - cr * ii) / d * kvi;
        s_zDtR[seg]  = (er * ir + ei * ii) / d * kvi;
        s_zDtI[seg]  = (ei * ir - er * ii) / d * kvi;
        s_acZre[seg] = zRe;
    }
    s_acZim[seg] = zIm;
    if (seg == 0u) s_acI00 = iAmp;
    if (seg == AC_S_Q04) s_acZreQ04 = zRe;
    if (seg == AC_NF - 1u) acCalDelay();

    if (seg == AC_S_Q04 || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u)) {
        uint16_t n = (seg == AC_S_Q04) ? 0u : (uint16_t)(seg - AC_S_Q10 + 1u);
        if (n > 0u && iAmp < 1.08f * gIdent.lqAtA[n - 1u]) {
            /* 电压/硬件限幅导致该档未相对上一档明显递增：截断至前 n 档 */
            s_acSatN = (uint8_t)n;
            return 0;
        }
        {
            float iNorm = sqrtf(d), ca = ir / iNorm, sa = ii / iNorm;
            float c3 = 4.0f * ca * ca * ca - 3.0f * ca;
            float s3 = 3.0f * sa - 4.0f * sa * sa * sa;
            float i3r = (float)s_acI3s, i3i = (float)s_acI3c;
            float pkA = (float)s_acIpk * ki;
            gIdent.lqAtA[n] = iAmp;
            s_acH3s[n] = (i3r * c3 + i3i * s3) / iNorm;
            s_acH3c[n] = (i3i * c3 - i3r * s3) / iNorm;
            if (pkA > s_acMaxPkA) s_acMaxPkA = pkA;
            if (n > 0u) {
                float w0 = 2.0f * (float)MATH_PI * s_acHz[0];
                if (zIm > 0.0f && w0 > 0.0f) s_acTierL[n - 1u] = zIm / w0;
            }
        }
    }

    if (seg == AC_S_Q04L) {
        float w0 = 2.0f * (float)MATH_PI * s_acHz[0];
        float ldEst = (s_acZim[0] > 0.0f && w0 > 0.0f) ? (s_acZim[0] / w0) : gIdent.rlL;
        float lqEst = (s_acZim[AC_S_Q04] > 0.0f && w0 > 0.0f) ? (s_acZim[AC_S_Q04] / w0) : ldEst;
        float rEst = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : (float)gParam.rsOhm;
        s_acI04L = iAmp;
        /* q 段（seg 7..11）注入轴是电机 q 轴（piD），偏置轴是电机 d 轴（piQ） */
        acSetupPI(lqEst, ldEst, rEst);
        acPlanSatTiers(lqEst, rEst);
    }
    if (seg == AC_S_Q10L) s_acI10L = iAmp;
    return 0;
}

static float acKmot(void)
{
    float psi = gIdent.psiWb > 0 ? gIdent.psiWb : (float)gParam.psiWb;
    float acc = gIdent.accRpmSPerA > 0 ? gIdent.accRpmSPerA : (float)gParam.accRpmSPerA;
    return (float)MOTOR_POLE_PAIRS * psi * acc * (float)(2.0 * MATH_PI / 60.0);
}

static void acFit(void)
{
    uint16_t f;
    const float w12 = 2.0f * (float)MATH_PI * s_acHz[3], w25 = 2.0f * (float)MATH_PI * s_acHz[0];
    const float dk = (w12 > 0.0f && w25 > 0.0f) ? (1.0f / (w12 * w12) - 1.0f / (w25 * w25)) : 0.0f;
    float l04 = (w25 > 0.0f) ? s_acZim[AC_S_Q04] / w25 : 0.0f;
    float k04 = (dk > 1e-12f) ? (s_acZim[AC_S_Q04] / w25 - s_acZim[AC_S_Q04L] / w12) / dk : 0.0f;
    float k10 = (s_l0Only || s_acSatN < 2u || dk <= 1e-12f) ? 0.0f : (s_acZim[AC_S_Q10] / w25 - s_acZim[AC_S_Q10L] / w12) / dk;
    float snr04L = (gIdent.deadVoltV > 0.0f) ? fabsf(s_acZim[AC_S_Q04L]) * s_acI04L / gIdent.deadVoltV : 1e9f;

    gIdent.kSwingModel = acKmot();
    if (k04 < 0.0f) k04 = 0.0f;
    if (k10 < 0.0f) k10 = 0.0f;
    if (snr04L < 0.5f * AC_SNR_MIN ||
        (gIdent.lqAtA[0] > 0.0f && fabsf(s_acI04L - gIdent.lqAtA[0]) > 0.15f * gIdent.lqAtA[0])) {
        k04 = 0.0f;
        gIdent.warn |= IDENT_WARN_SWING_CLAMP;
    }
    if (!s_l0Only && s_acSatN >= 2u && gIdent.lqAtA[1] > 0.0f &&
        fabsf(s_acI10L - gIdent.lqAtA[1]) > 0.15f * gIdent.lqAtA[1]) {
        k10 = k04;
        gIdent.warn |= IDENT_WARN_SWING_CLAMP;
    }
    if (l04 > 0.0f) {
        float kMax = 0.35f * l04 * w25 * w25;
        if (k04 > kMax) { k04 = kMax; gIdent.warn |= IDENT_WARN_SWING_CLAMP | IDENT_WARN_AC_DRIFT; }
        if (k10 > kMax) { k10 = kMax; gIdent.warn |= IDENT_WARN_SWING_CLAMP | IDENT_WARN_AC_DRIFT; }
    }
    gIdent.kSwing[0] = k04;
    gIdent.kSwing[1] = k10;

    for (f = 0; f < AC_NF; f++) {
        float w = 2.0f * (float)MATH_PI * s_acHz[f];
        gIdent.acLdH[f] = s_acZim[f] / w;
        gIdent.acLqH[f] = s_acZim[AC_NF + f] / w + k04 / (w * w);
        gIdent.acRohm[f] = s_acZre[f];
    }
    gIdent.lqAtH[0] = s_acZim[AC_S_Q04] / w25 + k04 / (w25 * w25);
    if (!s_l0Only) {
        for (f = 1u; f < s_acSatN; f++) {
            float l1 = s_acZim[AC_S_Q10 + f - 1u] / w25 + k10 / (w25 * w25);
            gIdent.lqAtHb1[f - 1u] = l1;
            gIdent.lqAtHb2[f - 1u] = 0.0f;
            gIdent.lqAtH[f] = l1;
        }
        if (s_acSatN < 3u) {
            s_l0Only = 1u;
            gIdent.warn |= IDENT_WARN_AMP_ORDER;
        } else {
            for (f = 0; f + 1u < s_acSatN; f++) {
                if (!(gIdent.lqAtA[f] > 0.0f && gIdent.lqAtA[f + 1u] >= 1.08f * gIdent.lqAtA[f])) {
                    s_l0Only = 1u;
                    gIdent.warn |= IDENT_WARN_AMP_ORDER;
                    break;
                }
            }
            if (!(gIdent.lqAtA[s_acSatN - 1u] >= 1.40f * gIdent.lqAtA[1])) {
                s_l0Only = 1u;
                gIdent.warn |= IDENT_WARN_AMP_ORDER;
            }
        }
    } else {
        gIdent.warn |= IDENT_WARN_AMP_ORDER;
    }

    /* 扫幅不足 × 深饱和门控 & 电感反增物理护栏：
     * 1. 铁芯饱和时电感随电流只可能非增，若顶档电感反超档 0 超过 10% 必为偏轴/失锁，置 AC_DRIFT 阻断
     * 2. 当饱和拟合有效（s_acSatN >= 3）但顶档电感相对 L0 已明显下降且顶档电流未达 2.5×IDENT_I_SCALE，降级仅 L0 */
    if (!s_l0Only && s_acSatN >= 3u && gIdent.lqAtH[0] > 0.0f) {
        float lTop = gIdent.lqAtH[s_acSatN - 1u];
        float lRatio = (gIdent.lqAtH[0] > 1e-12f) ? lTop / gIdent.lqAtH[0] : 1.0f;
        float iTop = gIdent.lqAtA[s_acSatN - 1u];
        float iSwpMax = 2.5f * (float)IDENT_I_SCALE;
        if (lTop > 1.10f * gIdent.lqAtH[0])
            gIdent.warn |= IDENT_WARN_AC_DRIFT;
        if (lRatio < 0.70f && iTop < 0.85f * iSwpMax) {
            s_l0Only = 1u;
            gIdent.warn |= IDENT_WARN_AMP_ORDER;
        }
    }

    gIdent.ldH = gIdent.acLdH[0];
    gIdent.lqH = gIdent.acLqH[0];
    {
        float rUse = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : s_acZre[0];
        float xd = gIdent.acLdMagH[0] * w25;
        float z2q = s_acZreQ04 * s_acZreQ04 + s_acZim[AC_S_Q04] * s_acZim[AC_S_Q04];
        float xq = (z2q > rUse * rUse) ? sqrtf(z2q - rUse * rUse) : 0.0f;
        float snrD = (gIdent.deadVoltV > 0.0f) ? xd * s_acI00 / gIdent.deadVoltV : 1e9f;
        float snrQ = (gIdent.deadVoltV > 0.0f) ? xq * gIdent.lqAtA[0] / gIdent.deadVoltV : 1e9f;
        gIdent.snrMin = (snrD < snrQ) ? snrD : snrQ;
        if (gIdent.snrMin < AC_SNR_MIN) gIdent.warn |= IDENT_WARN_LOW_SNR;
    }
    if (s_acMotion ||
        gIdent.acLdH[0] <= 0.0f || gIdent.acLdH[1] <= 0.0f || gIdent.acLdH[2] <= 0.0f ||
        gIdent.acRohm[0] <= 0.0f || gIdent.acLqH[0] <= 0.0f ||
        gIdent.acLdH[2] < 0.65f * gIdent.acLdH[0] || gIdent.acLdH[2] > 1.35f * gIdent.acLdH[0] ||
        fabsf(gIdent.acLdMagH[0] - gIdent.acLdH[0]) > 0.25f * gIdent.acLdH[0]) {
        gIdent.warn |= IDENT_WARN_AC_DRIFT;
    }
}

/* ---- R/L 粗测（IDENT_PH_RL = 9）------------------------------------------------ */
#define RL_HZ       200.0
#define RL_PER      ((uint16_t)(PWM_FREQ / RL_HZ + 0.5))
#define RL_AVG      16
#define RL_I_Q15    (0.5 * MOTOR_I_RATED_A / I_MAX * 32768.0)
#define RL_V0_V     ((0.5 < 0.01 * UDC_RATE) ? 0.5 : (0.01 * UDC_RATE))
#define RL_V0_Q15   (Q15(RL_V0_V / UDC_MAX) < 1 ? 1 : Q15(RL_V0_V / UDC_MAX))
#define RL_VMAX_Q15 Q15(0.45 * UDC_RATE / 1.7320508 / UDC_MAX)

static void rlStart(void)
{
    s_acInc = (uint32_t)(4294967296.0 / (double)RL_PER + 0.5);
    s_acDly = (uint32_t)(AC_V_DELAY * (double)s_acInc + 0.5);
    s_acPhi = 0;
    s_acT = 0;
    s_acIs = s_acIc = 0;
    s_acAmp = RL_V0_Q15;
    s_acSeg = 0;
    s_acEs = 0;
    s_acVs = s_acVc = 0;
    s_acDone = 0;
    gIdent.rlR = -1.0f;
    gIdent.rlL = -1.0f;
}

static void rlFast(void)
{
    SinCos_TypeDef a = FocMath_SinCos((int16_t)(s_acPhi >> 16));
    SinCos_TypeDef b = FocMath_SinCos((int16_t)((s_acPhi - s_acDly) >> 16));
    if (!s_acDone) {
        int32_t i = gMeas.iab.s16Alpha;
        s_acIs += (i * b.s16Sin) >> 15;
        s_acIc += (i * b.s16Cos) >> 15;
        if (++s_acT >= RL_PER) s_acDone = 1;
    }
    s_acPhi += s_acInc;
    FocCore_ApplyVoltageAB((int16_t)(((int32_t)s_acAmp * a.s16Sin) >> 15), 0);
}

static uint8_t rlSlow(void)
{
    float is, ic, i1, v;
    if (!s_acDone) return 0;
    is = (float)s_acIs;
    ic = (float)s_acIc;
    s_acIs = s_acIc = 0;
    s_acT = 0;
    COMPILER_BARRIER();
    s_acDone = 0;
    i1 = 2.0f * sqrtf(is * is + ic * ic) / (float)RL_PER;
    v = (float)s_acAmp;
    if (s_acSeg == 0) {
        float k = (i1 > 1.0f) ? (float)RL_I_Q15 / i1 : 2.0f;
        int16_t nv;
        if (k > 0.85f && k < 1.15f) { s_acSeg = 1; s_acEs = 0; return 0; }
        if (k > 2.0f) k = 2.0f;
        if (k < 0.5f) k = 0.5f;
        if (v >= (float)RL_VMAX_Q15 && k > 1.0f) {
            if (i1 >= 0.15f * (float)RL_I_Q15) { s_acSeg = 1; s_acEs = 0; return 0; }
            gIdent.rlR = -1.0f;
            gIdent.warn |= IDENT_WARN_RL_TIMEOUT;
            return 1;
        }
        if (++s_acEs >= 200) {
            gIdent.warn |= IDENT_WARN_RL_TIMEOUT;
            if (i1 > 0.25f * (float)RL_I_Q15) { s_acSeg = 1; s_acEs = 0; return 0; }
            gIdent.rlR = -1.0f;
            return 1;
        }
        v *= 0.5f + 0.5f * k;
        if (v > (float)RL_VMAX_Q15) v = (float)RL_VMAX_Q15;
        nv = (int16_t)(v + 0.5f);
        if (nv < 1) nv = 1;
        if (nv == s_acAmp) {
            if (k > 1.0f && nv < (int16_t)RL_VMAX_Q15) nv++;
            else if (k < 1.0f && nv > 1) nv--;
        }
        s_acAmp = nv;
        return 0;
    }
    if (s_acSeg < 3) { s_acSeg++; return 0; }
    s_acVs += (int32_t)is;
    s_acVc += (int32_t)ic;
    if (++s_acEs < RL_AVG) return 0;
    {
        float ss = (float)s_acVs, cc = (float)s_acVc, n = (float)(RL_AVG * RL_PER), d = ss * ss + cc * cc;
        float r, x;
        if (d <= 0.0f) { gIdent.rlR = -1.0f; gIdent.warn |= IDENT_WARN_RL_TIMEOUT; return 1; }
        r = v * ss * n / (2.0f * d) * (float)(UDC_MAX / I_MAX);
        x = -v * cc * n / (2.0f * d) * (float)(UDC_MAX / I_MAX);
        gIdent.rlR = r;
        gIdent.rlL = x / (float)(2.0 * MATH_PI * RL_HZ);
        if (r > 0.0f && gIdent.rlL > 0.0f) {
            MotorParam_TypeDef p = gParam;
            p.rsOhm = r;
            p.lqH = gIdent.rlL;
            p.ldH = gIdent.rlL;
            (void)Param_ApplyNow(&p);
        }
    }
    return 1;
}

/* ---- q 轴饱和拟合（phase 7）：含 3 次谐波精确描述函数 ------------------------------ */
#define DF_NP   16
static uint16_t s_dfI;
static float s_dfBest, s_dfK, s_dfIk, s_dfL0;

static void dfStart(void)
{
    s_dfI = 0;
    s_dfBest = 1e30f;
    s_dfK = 0;
    s_dfIk = 0;
    s_dfL0 = gIdent.lqAtH[0];
}

static const float s_dfS1[DF_NP] = {
    0.0980171f, 0.2902847f, 0.4713967f, 0.6343933f, 0.7730105f, 0.8819213f, 0.9569403f, 0.9951847f,
    0.9951847f, 0.9569403f, 0.8819213f, 0.7730105f, 0.6343933f, 0.4713967f, 0.2902847f, 0.0980171f
};
static const float s_dfS3[DF_NP] = {
    0.2902847f, 0.7730105f, 0.9951847f, 0.8819213f, 0.4713967f, -0.0980171f, -0.6343933f, -0.9569403f,
   -0.9569403f, -0.6343933f, -0.0980171f, 0.4713967f, 0.8819213f, 0.9951847f, 0.7730105f, 0.2902847f
};
static const float s_dfC3[DF_NP] = {
    0.9569403f, 0.6343933f, 0.0980171f, -0.4713967f, -0.8819213f, -0.9951847f, -0.7730105f, -0.2902847f,
    0.2902847f, 0.7730105f, 0.9951847f, 0.8819213f, 0.4713967f, -0.0980171f, -0.6343933f, -0.9569403f
};

static uint8_t dfStep(void)
{
    float sc = (float)IDENT_I_SCALE;
    float ik = 0.05f * sc * (float)s_dfI, k, kStep = 0.005f / sc, kMax = 0.6001f / sc;
    uint16_t i, j, iStart;
    if (s_l0Only || s_acSatN < 3u) {
        gIdent.lsSatKneeA = 0.0f;
        gIdent.lsSatPerA = 0.0f;
        gIdent.lqH = gIdent.acLqH[0];
        return 1;
    }
    /* 若档 0（0.4×）存在严重死区 3 次谐波畸变（> 0.25）且有效档数 >= 4，跳过档 0 以防虚假悬崖带偏拟合 */
    iStart = (s_acSatN >= 4u && sqrtf(s_acH3s[0] * s_acH3s[0] + s_acH3c[0] * s_acH3c[0]) > 0.25f) ? 1u : 0u;
    for (k = 0; k <= kMax; k += kStep) {
        float slr = 0, srr = 0, sll = 0, l0, e;
        for (i = iStart; i < s_acSatN; i++) {
            float a = gIdent.lqAtA[i], h3s = s_acH3s[i], h3c = s_acH3c[i], r = 0;
            for (j = 0; j < DF_NP; j++) {
                float s1 = s_dfS1[j];
                float u = s1 + h3s * s_dfS3[j] + h3c * s_dfC3[j];
                float x = fabsf(a * u) - ik;
                float m = (x > 0.0f) ? (1.0f - k * x) : 1.0f;
                if (m < (float)MOTOR_LS_SAT_MIN) m = (float)MOTOR_LS_SAT_MIN;
                r += m * u * s1;
            }
            r *= 2.0f / (float)DF_NP;
            slr += gIdent.lqAtH[i] * r;
            srr += r * r;
            sll += gIdent.lqAtH[i] * gIdent.lqAtH[i];
        }
        if (srr <= 0.0f || slr <= 0.0f) continue;
        l0 = slr / srr;
        e = (sll - l0 * slr) / (l0 * l0);
        if (e < s_dfBest) {
            s_dfBest = e;
            s_dfK = k;
            s_dfIk = ik;
            s_dfL0 = l0;
        }
    }
    if (++s_dfI <= 48 && 0.05f * sc * (float)s_dfI <= gIdent.lqAtA[s_acSatN - 1u]) return 0;
    if (s_dfK * (gIdent.lqAtA[s_acSatN - 1u] - s_dfIk) < 0.03f) {
        s_dfK = 0;
        s_dfIk = 0;
        s_dfL0 = gIdent.lqAtH[iStart];
    }
    if (s_dfBest > (float)s_acSatN * 0.08f * 0.08f) gIdent.warn |= IDENT_WARN_AC_DRIFT;
    if (gIdent.lqAtH[0] > 0.0f && s_dfL0 > 1.25f * gIdent.lqAtH[0]) gIdent.warn |= IDENT_WARN_AC_DRIFT;
    gIdent.lsSatKneeA = s_dfIk;
    gIdent.lsSatPerA = s_dfK;
    gIdent.lqH = s_dfL0;
    return 1;
}

static void offerStill(uint8_t commitL)
{
    MotorParam_TypeDef p = gParam;
    uint16_t m = 0;
    if (gIdent.rsOhm > 0) { p.rsOhm = gIdent.rsOhm; m |= 1u; }
    if (commitL && gIdent.lqH > 0) {
        p.lqH = gIdent.lqH;
        p.ldH = MOTOR_IPM ? gIdent.ldH : gIdent.lqH;
        m |= 2u;
    }
    if (commitL && !s_l0Only && s_acSatN >= 3u && gIdent.lqH > 0) {
        float maxA = s_acMaxPkA;
        if (maxA < gIdent.lqAtA[s_acSatN - 1u]) maxA = gIdent.lqAtA[s_acSatN - 1u];
        p.lsKneeA = gIdent.lsSatKneeA;
        p.lsPerA = gIdent.lsSatPerA;
        /* 有界线性外推：运行期过载常超出扫幅上限（如额定 0.75 A 电机扫到 2.5 A 而过载 4.0 A），
         * 直接钳平会使观测器 L 偏高 ~20%（真实电机饱和越往后越快，误差只会更大）；
         * 允许按拟合斜率外推至 1.5 倍扫幅上限后再钳平，兼顾覆盖与无实测数据区的外推风险 */
        p.lsMaxA = maxA * 1.5f;
        m |= 4u;
    }
    if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) { p.dtEffS = gIdent.deadTimeS; m |= 32u; }
    Param_Offer(&p, m);
}

void Ident_Slow(void)
{
    IdentTest_TypeDef *t = &gIdentTest;
    switch (t->phase) {
    case IDENT_PH_RL:
        if (rlSlow()) {
            FocCore_PresetPI(0, 0);
            if (gIdent.rlR <= 0.0f || gIdent.rlL <= 0.0f) {
                FocCore_SetIdqRef(0, 0);
                gIdent.warn |= IDENT_WARN_RL_TIMEOUT | IDENT_WARN_AUTO_BLOCKED;
                gIdent.state = IDENT_DONE;
                t->cnt = 0;
                t->phase = IDENT_PH_DONE;
            } else {
                t->phase = IDENT_PH_NONE;
            }
        }
        break;
    case IDENT_PH_DT_STEP:
        FocCore_SetIdqRef(s_dtLevel[t->idx], 0);
        if (t->cnt == 0) {
            s_dtSubIdx = 0;
            s_dtSubVq = 0;
            s_dtSubMin = 32767;
            s_dtSubMax = -32768;
        }
        if (++t->cnt > DT_SETTLE_MS) {
            t->sum[0] += gFoc.idq.s16D;
            t->sum[1] += gFoc.vdq.s16D;
            s_dtSubVq += gFoc.vdq.s16Q;
            if (++s_dtSubIdx >= DT_SUB_MS) {
                int32_t avgQ = s_dtSubVq / DT_SUB_MS;
                if (avgQ < s_dtSubMin) s_dtSubMin = avgQ;
                if (avgQ > s_dtSubMax) s_dtSubMax = avgQ;
                s_dtSubVq = 0;
                s_dtSubIdx = 0;
            }
        }
        if (t->cnt >= DT_SETTLE_MS + DT_AVG_MS) {
            float vdAbs = fabsf((float)t->sum[1]) * (float)(UDC_MAX / 32768.0 / DT_AVG_MS);
            float vqPP  = (float)(s_dtSubMax - s_dtSubMin) * (float)(UDC_MAX / 32768.0);
            int16_t idAvg = (int16_t)(t->sum[0] / DT_AVG_MS);
            int16_t vdAvg = (int16_t)(t->sum[1] / DT_AVG_MS);
            float vqLim = 0.50f * vdAbs + 0.15f;
            uint8_t unstable = (uint8_t)(vdAbs > 0.05f && vqPP > vqLim);
            t->sum[0] = t->sum[1] = 0;
            t->cnt = 0;
            if (unstable && s_dtRetry < 2u) {
                s_dtRetry++;
                break;
            }
            if (unstable) {
                s_dtDropMask |= (uint16_t)(1u << t->idx);
            } else {
                t->dtId[t->idx] = idAvg;
                t->dtVd[t->idx] = vdAvg;
            }
            s_dtRetry = 0;
            if (++t->idx >= IDENT_DT_N) {
                t->idx = 0;
                t->phase = IDENT_PH_DT_ZERO;
                dtFit();
                {
                    MotorParam_TypeDef p = gParam;
                    if (gIdent.rsOhm > 0) p.rsOhm = gIdent.rsOhm;
                    if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) p.dtEffS = gIdent.deadTimeS;
                    (void)Param_ApplyNow(&p);
                }
            }
        }
        break;
    case IDENT_PH_DT_ZERO:
        FocCore_SetIdqRef(0, 0);
        if (++t->cnt >= ZERO_WAIT_MS) {
            FocCore_PresetPI(0, 0);
            acInitFreqs();
            acSegStart(0, 0);
            t->cnt = 0;
            t->phase = IDENT_PH_AC;
        }
        break;
    case IDENT_PH_AC:
        if (s_acDone) {
            if (acSegFinish(s_acSeg)) {
                acSegStart(s_acSeg, 1);
            } else {
                uint16_t nextSeg = (uint16_t)(s_acSeg + 1u);
                if (s_l0Only && s_acSeg == AC_S_Q04L) {
                    nextSeg = AC_NSEG;
                } else if (s_acSeg >= AC_S_Q10 && s_acSeg < AC_S_Q10 + 4u &&
                           (uint16_t)(s_acSeg - AC_S_Q10 + 2u) > (uint16_t)s_acSatN) {
                    if (s_acSatN < 2u) { s_l0Only = 1u; nextSeg = AC_NSEG; }
                    else nextSeg = AC_S_Q10L;
                }
                if (nextSeg < AC_NSEG) {
                    acSegStart(nextSeg, 0);
                } else {
                    s_acSeg = AC_NSEG;
                    acFit();
                    FocCore_LoadGains();
                    dfStart();
                    t->cnt = 0;
                    t->phase = IDENT_PH_SAT_FIT;
                }
            }
        }
        break;
    case IDENT_PH_SAT_FIT:
        if (dfStep()) {
            t->cnt = 0;
            Ipd_Start();
            t->phase = IDENT_PH_IPD;
        }
        break;
    case IDENT_PH_IPD:
        Ipd_Slow();
        if (gIpd.state == IPD_STATE_DONE) {
            uint8_t trustedPre;
            (void)Ipd_Evaluate();
            ipdFit();
            FocCore_LoadGains();
            trustedPre = (uint8_t)(gIdent.rsOhm > 0.0f && gIdent.lqH > 0.0f &&
                                   (!MOTOR_IPM || gIdent.ldH > 0.0f) &&
                                   gIdent.acLdH[0] > 0.0f && gIdent.acRohm[0] > 0.0f &&
                                   gIdent.rlR > 0.0f && gIdent.rlL > 0.0f &&
                                   !(gIdent.warn & (IDENT_WARN_AC_DRIFT | IDENT_WARN_LOW_SNR | IDENT_WARN_AC_FAIL |
                                                    IDENT_WARN_AMP_FAIL | IDENT_WARN_DT_UNSTABLE)));
            offerStill(trustedPre);
            {
                uint8_t ok = (uint8_t)(trustedPre && !(gParamStat.rejectMask & 39u) && gParamStat.pending);
                if (!ok) gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
                if (gIdentCfg.mode == IDENT_MODE_AUTO && ok) {
                    s_autoStage = 1;
                    s_restartReq = 1;
                } else {
                    gIdent.state = IDENT_DONE;
                }
            }
            t->cnt = 0;
            t->phase = IDENT_PH_DONE;
        }
        break;
    case IDENT_PH_DONE:
        break;
    default:
        break;
    }
}

/* ---- 运行辨识（零电流滑行 + 开环磁链）------------------------------------------- */
#define RUN_STABLE_MS   2000
#define RUN_PRE_MS      500
#define RUN_SKIP_MS     20
#define RUN_COAST_MS    400
#define RUN_MIN_RPM_RAW (TRACK_END_RPM / 0.45)
#define RUN_MIN_RPM     (RUN_MIN_RPM_RAW < 0.7 * MOTOR_SPEED_RATED_RPM ? RUN_MIN_RPM_RAW : 0.7 * MOTOR_SPEED_RATED_RPM)

enum { RN_WAIT_STABLE = 0, RN_PRE_AVG, RN_COAST, RN_DONE };
static uint8_t  s_rnSt;
static uint16_t s_rnT;
static int16_t  s_rnCmd;
static int32_t  s_rnIq, s_rnId, s_rnW;
static float    s_rnVd, s_rnVq, s_rnS[7], s_rnX2y, s_rnWs;

uint8_t Ident_HoldTrack(void)
{
    return (uint8_t)(Ident_RunActive() && s_rnSt == RN_COAST);
}

int16_t Ident_SpeedCmd(int16_t cmd)
{
    int16_t lo;
    if (gIdentCfg.mode != IDENT_MODE_AUTO || !Ident_RunActive() || cmd == 0) return cmd;
    lo = (int16_t)Q15(RUN_MIN_RPM * 1.1 / SPEED_MAX);
    if (cmd > 0 && cmd < lo) return lo;
    if (cmd < 0 && cmd > -lo) return (int16_t)-lo;
    return cmd;
}

void Ident_RunSlow(uint8_t spin, uint8_t tracking, int16_t cmd, int16_t ramp)
{
    int16_t w = (int16_t)(gEst.pll.s32Speed >> 16);
    if (!Ident_RunActive() || !spin) {
        s_rnSt = RN_WAIT_STABLE;
        s_rnT = 0;
        return;
    }
    if (cmd != s_rnCmd) {
        s_rnCmd = cmd;
        s_rnSt = RN_WAIT_STABLE;
        s_rnT = 0;
    }
    switch (s_rnSt) {
    case RN_WAIT_STABLE:
        if (!tracking && ramp == cmd && abs(w) >= Q15(RUN_MIN_RPM / SPEED_MAX)) {
            if (++s_rnT >= RUN_STABLE_MS) {
                s_rnT = 0;
                s_rnIq = s_rnId = s_rnW = 0;
                s_rnSt = RN_PRE_AVG;
            }
        } else {
            s_rnT = 0;
        }
        break;
    case RN_PRE_AVG:
        if (tracking) {
            s_rnSt = RN_WAIT_STABLE;
            s_rnT = 0;
            break;
        }
        s_rnIq += gFoc.idq.s16Q;
        s_rnId += gFoc.idq.s16D;
        s_rnW += w;
        if (++s_rnT >= RUN_PRE_MS) {
            uint16_t i;
            s_rnVd = s_rnVq = s_rnX2y = s_rnWs = 0;
            for (i = 0; i < 7; i++) s_rnS[i] = 0;
            s_rnT = 0;
            s_rnSt = RN_COAST;
            Spin_TrackBegin();
        }
        break;
    case RN_COAST:
        if (!tracking) {
            s_rnSt = RN_WAIT_STABLE;
            s_rnT = 0;
            break;
        }
        if (++s_rnT > RUN_SKIP_MS) {
            float x = (float)s_rnT * 0.001f, y = (float)w;
            s_rnVd += gFoc.vdq.s16D;
            s_rnVq += gFoc.vdq.s16Q;
            s_rnWs += y;
            s_rnS[0] += 1;
            s_rnS[1] += x;
            s_rnS[2] += x * x;
            s_rnS[3] += x * x * x;
            s_rnS[4] += x * x * x * x;
            s_rnS[5] += y;
            s_rnS[6] += x * y;
            s_rnX2y += x * x * y;
        }
        if (s_rnT >= RUN_SKIP_MS + RUN_COAST_MS) {
            float n = s_rnS[0], a1 = s_rnS[1], a2 = s_rnS[2], a3 = s_rnS[3], a4 = s_rnS[4];
            float y0 = s_rnS[5], y1 = s_rnS[6], y2 = s_rnX2y;
            float det = n * (a2 * a4 - a3 * a3) - a1 * (a1 * a4 - a3 * a2) + a2 * (a1 * a3 - a2 * a2);
            float db = n * (y1 * a4 - a3 * y2) - y0 * (a1 * a4 - a3 * a2) + a2 * (a1 * y2 - y1 * a2);
            float ks = (float)(SPEED_MAX / 32768.0);
            float wm = s_rnWs / n * ks, we = wm * (float)(2.0 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS);
            float iq = (float)s_rnIq / RUN_PRE_MS * (float)(I_MAX / 32768.0);
            float vm = sqrtf(s_rnVd * s_rnVd + s_rnVq * s_rnVq) / n * (float)(UDC_MAX / 32768.0);
            if (we < 0) we = -we;
            if (we > 1.0f) gIdent.psiWb = vm / we;
            if (det != 0 && (iq > 0.02f || iq < -0.02f)) gIdent.accRpmSPerA = -(db / det) * ks / iq;
            gIdent.runRpm = (float)s_rnW / RUN_PRE_MS * ks;
            gIdent.runIqA = iq;
            gIdent.runIdA = (float)s_rnId / RUN_PRE_MS * (float)(I_MAX / 32768.0);
            {
                MotorParam_TypeDef p = gParam;
                uint16_t m = 0;
                if (gIdent.psiWb > 0) { p.psiWb = gIdent.psiWb; m |= 8u; }
                if (gIdent.accRpmSPerA > 0) { p.accRpmSPerA = gIdent.accRpmSPerA; m |= 16u; }
                if (!Param_ApplyNow(&p)) Param_Offer(&p, m);
            }
            gIdent.state = IDENT_DONE;
            s_rnSt = RN_DONE;
        }
        break;
    default:
        break;
    }
}

#define OLFLUX_SETTLE_MS   300
#define OLFLUX_AVG_MS      500

uint8_t Ident_OpenLoopFluxSlow(int32_t ol, int32_t tgt)
{
    if (!Ident_RunActive() || s_olFluxSt == OLF_DONE) return 0;
    if (ol != tgt) {
        s_olFluxSt = OLF_WAIT_STABLE;
        s_olFluxN = 0;
        return 1;
    }
    if (s_olFluxSt == OLF_WAIT_STABLE) {
        if (++s_olFluxN < OLFLUX_SETTLE_MS) return 1;
        s_olFluxSt = OLF_AVG;
        s_olFluxN = 0;
        s_rnId = s_rnIq = 0;
        s_rnVd = s_rnVq = 0;
        return 1;
    }
    s_rnId += gFoc.idq.s16D;
    s_rnIq += gFoc.idq.s16Q;
    s_rnVd += gFoc.vdq.s16D;
    s_rnVq += gFoc.vdq.s16Q;
    if (++s_olFluxN < OLFLUX_AVG_MS) return 1;
    {
        const float ki = (float)(I_MAX / 32768.0) / OLFLUX_AVG_MS, kv = (float)(UDC_MAX / 32768.0) / OLFLUX_AVG_MS;
        float id = s_rnId * ki, iq = s_rnIq * ki, vd = s_rnVd * kv, vq = s_rnVq * kv;
        float w = (float)ol * (float)(SPEED_MAX / 2147483648.0 * 2.0 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS);
        float im = sqrtf(id * id + iq * iq), r = (float)gParam.rsOhm, l = Param_LqAt(im), vdt = 0, ed, eq;
        float vph = (float)gParam.dtEffS * (float)PWM_FREQ * gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
        uint16_t j;
        for (j = 0; j < 16; j++) {
            float s = sinf(((float)j + 0.5f) * (float)(MATH_PI / 32.0)), x = im * s / (float)INVERTER_DT_I0_A;
            vdt += (x > 1.0f ? 1.0f : x) * s;
        }
        vdt *= vph / 8.0f;
        if (im > 0.01f) { vd -= vdt * id / im; vq -= vdt * iq / im; }
        ed = vd - r * id + w * l * iq;
        eq = vq - r * iq - w * l * id;
        if (w < 0) w = -w;
        if (w > 1.0f) {
            MotorParam_TypeDef p = gParam;
            gIdent.psiOpenLoopWb = sqrtf(ed * ed + eq * eq) / w;
            p.psiWb = gIdent.psiOpenLoopWb;
            (void)Param_ApplyNow(&p);
        }
    }
    s_olFluxSt = OLF_DONE;
    return 0;
}

uint8_t Ident_Fast(void)
{
    if (gIdentTest.phase < IDENT_PH_IPD) return 0;
    if (gIdentTest.phase == IDENT_PH_AC || gIdentTest.phase == IDENT_PH_SAT_FIT) acFast();
    else if (gIdentTest.phase == IDENT_PH_RL) rlFast();
    else Ipd_Fast();
    return 1;
}