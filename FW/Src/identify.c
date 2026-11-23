/* ============================================================================
 * @file    identify.c
 * @brief   参数辨识（IDENT_TEST = 1 时编译；正常运行时这些函数不被调用）。方法与流程见 identify.h
 *
 * 静止测量（转子静止，操作者保证）：
 *   phase 9  R/L 粗测：α 轴注入正弦电压（不用电流环），幅值从小往上调到目标电流，得到粗略 R、L 先整定电流环
 *   定位（startup.c 两步定位）-> phase 1/2 死区阶梯：d 轴直流电流从最高档逐档下降，上半段直线拟合（斜率 Rs、截距死区）
 *   phase 6  Ld / Lq：d 轴、q 轴注入正弦电压（与 R/L 粗测同一方法），L = Im(Z)/ω；q 轴按 0.5 ~ 2 倍额定电流多档测量，拟合饱和 L(i)
 *   phase 5  收尾定位：交还 startup.c 辨识定位（定位电流 + 阻尼），等转子停稳
 *   phase 4  完成：结果交给 param.c，只有保护性条件（测不出正的 R / L）阻止自动重起动
 * 运行测量（正常起动）：I/F 恒速段测磁链（Ident_OpenLoopFluxSlow），测得后立即生效再切闭环；
 *   自动辨识中闭环稳速后小幅升速再降回，测加速度系数（Ident_RunSlow）。
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
#include "param.h"
#include "startup.h"
#include "speed_ctrl.h"
#include "supervisor.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

IdentResult_TypeDef gIdent;
volatile IdentCfg_TypeDef gIdentCfg = { 0 };
IdentTest_TypeDef gIdentTest;

/* ---- 公共 ---------------------------------------------------------------------- */
static uint8_t s_autoStage;         /* 自动方式：0 = 静止测量未完成，1 = 静止测量已完成、执行运行测量 */
static uint8_t s_holdOk;            /* 收尾定位结束后：1 = 自动重起动执行运行测量，0 = 结束 */
static uint8_t s_restartReq;        /* 自动方式：静止测量刚完成，请求停机再起动（一次性标志） */

enum { OLF_WAIT_STABLE = 0, OLF_AVG = 1, OLF_DONE = 2, OLF_NOSYNC = 3 };
static uint8_t s_olFluxSt;
static uint8_t s_jSt = 0xFF;        /* 惯量测量状态（0xFF = 不测），见 Ident_RunSlow */
static uint16_t s_olFluxN;

/* 测量用定位角：gIdentCfg.alignDeg（度）换算为 Q15 角度 */
int16_t Ident_AlignTheta(void)
{
    int32_t d = gIdentCfg.alignDeg % 360;
    if (d < 0) d += 360;
    return (int16_t)(d * 65536 / 360);
}

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

/* ---- 正弦电压注入（R/L 粗测与 Ld / Lq 共用）---------------------------------------
 * 沿 θ 轴注入 v = V sin(ωt)（αβ 直接输出，不用电流环），电流投影到 θ 轴后与延迟 AC_V_DELAY 拍的参考正弦做相关，
 * 得到基波 I∠φ，Z = V / I。幅值自适应：每个周期按目标电流 / 实测电流调整电压（从很小开始），落入 ±15% 后
 * 再等 3 个周期，平均 INJ_AVG 个周期。
 * 电感：R/L 粗测时取 L = Im(Z)/ω。测得 Rs 与死区之后改用幅值法 L = sqrt(|Z|^2 - R^2)/ω，R = Rs + 死区等效电阻
 * (4/π)·Vdt·g/I（死区电压为与电流同相的方波，基波 (4/π)Vdt，g 为三相在注入轴上的投影系数）：|Z| 与电压-电流延迟无关；
 * 若用 Im(Z)，延迟误差会把与电流同相的死区压降（小电流时相对很大）漏进虚部，使小电流档电感偏大 */
#define INJ_AVG         16
#define INJ_V0_V        ((0.5 < 0.01 * UDC_RATE) ? 0.5 : (0.01 * UDC_RATE))
#define INJ_V0_Q15      (Q15(INJ_V0_V / UDC_MAX) < 1 ? 1 : Q15(INJ_V0_V / UDC_MAX))
#define INJ_VMAX_Q15    Q15(0.45 * UDC_RATE / 1.7320508 / UDC_MAX)
#define INJ_RL_HZ       200.0
#define AC_V_DELAY      CTRL_INVPARK_ADVANCE

static uint32_t s_injPhi, s_injInc, s_injDly;
static uint16_t s_injPer, s_injT, s_injN;
static int16_t  s_injAmp;
static SinCos_TypeDef s_injAx;
static int32_t  s_injIs, s_injIc, s_injSs, s_injSc;
static uint8_t  s_injSt;            /* 0 = 调幅值，1..3 = 稳定，4 = 平均 */
static float    s_injHz, s_injTgt;     /* 注入频率；目标电流幅值（Q15） */
static int16_t  s_injTheta;
static volatile uint8_t s_injDone;

static void injStart(int16_t theta, float hz, float iA)
{
    s_injTgt = iA / (float)I_MAX * 32768.0f;
    s_injPer = (uint16_t)((float)PWM_FREQ / hz + 0.5f);
    s_injHz = (float)PWM_FREQ / (float)s_injPer;
    s_injInc = (uint32_t)(4294967296.0 / (double)s_injPer + 0.5);
    s_injDly = (uint32_t)(AC_V_DELAY * (double)s_injInc + 0.5);
    s_injAx = FocMath_SinCos(theta);
    s_injTheta = theta;
    s_injPhi = 0;
    s_injT = 0;
    s_injN = 0;
    s_injIs = s_injIc = s_injSs = s_injSc = 0;
    s_injAmp = INJ_V0_Q15;
    s_injSt = 0;
    s_injDone = 0;
}

static void injFast(void)
{
    SinCos_TypeDef a = FocMath_SinCos((int16_t)(s_injPhi >> 16));
    SinCos_TypeDef b = FocMath_SinCos((int16_t)((s_injPhi - s_injDly) >> 16));
    int32_t v = ((int32_t)s_injAmp * a.s16Sin) >> 15;
    if (!s_injDone) {
        int32_t i = ((int32_t)gMeas.iab.s16Alpha * s_injAx.s16Cos + (int32_t)gMeas.iab.s16Beta * s_injAx.s16Sin) >> 15;
        s_injIs += (i * b.s16Sin) >> 15;
        s_injIc += (i * b.s16Cos) >> 15;
        if (++s_injT >= s_injPer) s_injDone = 1;
    }
    s_injPhi += s_injInc;
    FocCore_ApplyVoltageAB((int16_t)((v * s_injAx.s16Cos) >> 15), (int16_t)((v * s_injAx.s16Sin) >> 15));
}

/* 每个注入周期结束时处理一次。返回 0 = 进行中，1 = 完成（*r, *l, *ia 有效：电阻、电感、实际电流基波幅值 A），
 * 2 = 失败（电压到上限仍达不到可用电流，或 200 周期未收敛） */
static uint8_t injSlow(float *r, float *l, float *ia)
{
    float is, ic, i1, v;
    if (!s_injDone) return 0;
    is = (float)s_injIs;
    ic = (float)s_injIc;
    s_injIs = s_injIc = 0;
    s_injT = 0;
    COMPILER_BARRIER();
    s_injDone = 0;
    v = (float)s_injAmp;
    if (s_injSt == 0) {
        float k;
        int16_t nv;
        i1 = 2.0f * sqrtf(is * is + ic * ic) / (float)s_injPer;
        k = (i1 > 1.0f) ? s_injTgt / i1 : 2.0f;
        if (k > 0.85f && k < 1.15f) { s_injSt = 1; return 0; }
        if (++s_injN >= 200 || (v >= (float)INJ_VMAX_Q15 && k > 1.0f)) {
            if (i1 >= 0.15f * s_injTgt) { s_injSt = 1; return 0; }   /* 电流偏小但可用 */
            return 2;
        }
        if (k > 2.0f) k = 2.0f;
        if (k < 0.5f) k = 0.5f;
        v *= 0.5f + 0.5f * k;
        if (v > (float)INJ_VMAX_Q15) v = (float)INJ_VMAX_Q15;
        nv = (int16_t)(v + 0.5f);
        if (nv < 1) nv = 1;
        if (nv == s_injAmp) {
            if (k > 1.0f && nv < (int16_t)INJ_VMAX_Q15) nv++;
            else if (k < 1.0f && nv > 1) nv--;
        }
        s_injAmp = nv;
        return 0;
    }
    if (s_injSt < 4) {
        if (++s_injSt == 4) { s_injSs = s_injSc = 0; s_injN = 0; }
        return 0;
    }
    s_injSs += (int32_t)is;
    s_injSc += (int32_t)ic;
    if (++s_injN < INJ_AVG) return 0;
    {
        float ss = (float)s_injSs, cc = (float)s_injSc, n = (float)(INJ_AVG * s_injPer), d = ss * ss + cc * cc;
        if (d <= 0.0f) return 2;
        *r = v * ss * n / (2.0f * d) * (float)(UDC_MAX / I_MAX);
        *l = -v * cc * n / (2.0f * d) * (float)(UDC_MAX / I_MAX) / (float)(2.0 * MATH_PI) / s_injHz;
        *ia = 2.0f * sqrtf(d) / n * (float)(I_MAX / 32768.0);
        if (gIdent.rsOhm > 0.0f && *ia > 0.0f) {
            float th = (float)s_injTheta * (float)(2.0 * MATH_PI / 65536.0);
            float g = (2.0f / 3.0f) * (fabsf(cosf(th)) + fabsf(cosf(th - 2.0943951f)) + fabsf(cosf(th + 2.0943951f)));
            float rt = gIdent.rsOhm + (float)(4.0 / MATH_PI) * gIdent.deadVoltV * g / *ia;
            float z = v * (float)(UDC_MAX / 32768.0) / *ia, x2 = z * z - rt * rt;
            *l = (x2 > 0.0f) ? sqrtf(x2) / (2.0f * (float)MATH_PI * s_injHz) : 0.0f;
        }
        return (*l > 0.0f) ? 1 : 2;
    }
}

/* ---- 死区阶梯（phase 1 / 2）-------------------------------------------------------
 * 定位轴上 d 轴直流电流逐档保持（稳定 DT_SETTLE_MS 后平均 DT_AVG_MS），记录 vd / id。
 * 静止时 vd = Rs*id + 死区误差电压；各相电流走出死区过渡带（宽度 INVERTER_DT_I0_A）后误差电压为常值 Vdt*g(θ)，
 * g = (2/3)Σ|cos(θ - 120°k)|。因此只用上半段（id >= 0.45 倍最高档且 >= 2*I0）的档位做直线拟合：斜率 = Rs，截距 / g = Vdt
 * （过渡带附近的形状因逆变器、电流零偏而异，不参与拟合）。最高档至少 4*I0、且须 >= 3.5*I0 才能分离，受 IDENT_VEC_MAX_A 限制 */
#define DT_TOP_RAW_A    ((IDENT_DT_TOP_A) > (4.0 * INVERTER_DT_I0_A) ? (IDENT_DT_TOP_A) : (4.0 * INVERTER_DT_I0_A))
#define DT_TOP_A        (DT_TOP_RAW_A < IDENT_VEC_MAX_A ? DT_TOP_RAW_A : IDENT_VEC_MAX_A)
static const float s_dtFrac[IDENT_DT_N] = { 1.00f, 0.80f, 0.60f, 0.50f, 0.40f, 0.30f, 0.25f, 0.20f, 0.15f, 0.10f, 0.05f };
#define DT_FIT_MIN_A    ((0.45 * DT_TOP_A) > (2.0 * INVERTER_DT_I0_A) ? (0.45 * DT_TOP_A) : (2.0 * INVERTER_DT_I0_A))
#define DT_SETTLE_MS    300
#define DT_AVG_MS       200
#define ZERO_WAIT_MS    50

static void dtFit(void)
{
    const IdentTest_TypeDef *t = &gIdentTest;
    float th = (float)Ident_AlignTheta() * (float)(2.0 * MATH_PI / 65536.0);
    float g = (2.0f / 3.0f) * (fabsf(cosf(th)) + fabsf(cosf(th - 2.0943951f)) + fabsf(cosf(th + 2.0943951f)));
    float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0, d, vbus, xMax = 0;
    uint16_t i;
    for (i = 0; i < IDENT_DT_N; i++) {
        float x = t->dtId[i] * (float)(I_MAX / 32768.0), y = t->dtVd[i] * (float)(UDC_MAX / 32768.0);
        if (x < 0.95f * (float)DT_FIT_MIN_A) continue;
        sx += x; sy += y; sxx += x * x; sxy += x * y; n += 1.0f;
        if (x > xMax) xMax = x;
    }
    d = n * sxx - sx * sx;
    gIdent.rsOhm = gIdent.deadVoltV = gIdent.deadTimeS = 0.0f;
    /* 原理上分不开（不足两档走出过渡带、最高档 < 3.5*I0）或 Rs 非正：不提交 Rs 与死区 */
    if (n < 2.0f || d <= 0.0f || xMax < 3.5f * (float)INVERTER_DT_I0_A) { gIdent.warn |= IDENT_WARN_DT_UNSTABLE; return; }
    gIdent.rsOhm = (n * sxy - sx * sy) / d;
    if (gIdent.rsOhm <= 0.0f) { gIdent.rsOhm = 0.0f; gIdent.warn |= IDENT_WARN_DT_UNSTABLE; return; }
    gIdent.deadVoltV = (sy - gIdent.rsOhm * sx) / n / g;
    if (gIdent.deadVoltV < 0.0f) gIdent.deadVoltV = 0.0f;
    vbus = gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
    gIdent.deadTimeS = (vbus > 1.0f) ? gIdent.deadVoltV / ((float)PWM_FREQ * vbus) : 0.0f;
}

/* ---- 静止测量流程 --------------------------------------------------------------- */

void Ident_Reset(void)
{
    memset(&gIdent, 0, sizeof(gIdent));
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdent.state = IDENT_RUNNING;
    gIdent.rlR = gIdent.rlL = -1.0f;
    FocCore_LoadGains();
    injStart(0, (float)INJ_RL_HZ, (float)IDENT_I_INJ_A);
    gIdentTest.phase = IDENT_PH_RL;
}

void Ident_Idle(void)
{
    gIdentTest.phase = IDENT_PH_NONE;
    s_olFluxSt = OLF_WAIT_STABLE;
    s_olFluxN = 0;
    FocCore_LoadGains();
}

/* 定位完成时由 startup.c 调用：首次定位后开始死区阶梯；收尾定位（IDENT_PH_HOLD）停稳后结束静止测量 */
void Ident_Begin(void)
{
    if (gIdentTest.phase == IDENT_PH_HOLD) {
        FocCore_SetIdqRef(0, 0);
        if (s_holdOk) {
            s_autoStage = 1;
            s_restartReq = 1;
        } else {
            gIdent.state = IDENT_DONE;
        }
        gIdentTest.cnt = 0;
        gIdentTest.phase = IDENT_PH_DONE;
        return;
    }
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdentTest.sum[0] = gIdentTest.sum[1] = 0;
    memset(gIdentTest.dtId, 0, sizeof(gIdentTest.dtId));
    memset(gIdentTest.dtVd, 0, sizeof(gIdentTest.dtVd));
    gIdentTest.phase = IDENT_PH_DT_STEP;
}

/* 静止测量的慢环由 Ident_Slow 接管；收尾定位期间交还 startup.c 的辨识定位 */
uint8_t Ident_Active(void)
{
    return (uint8_t)(gIdentTest.phase != IDENT_PH_NONE && gIdentTest.phase != IDENT_PH_HOLD);
}

/* 测不出正的 R / L（保护性条件）：结束静止测量，不提交、不自动重起动 */
static void staticAbort(uint16_t warn)
{
    FocCore_SetIdqRef(0, 0);
    FocCore_ApplyVoltageAB(0, 0);
    gIdent.warn |= warn | IDENT_WARN_AUTO_BLOCKED;
    gIdent.state = IDENT_DONE;
    gIdentTest.cnt = 0;
    gIdentTest.phase = IDENT_PH_DONE;
}

/* Ld / Lq 注入频率：感抗约为电阻（含死区等效电阻，取 R/L 粗测的实部）的 10 倍，阻抗角接近 90°，Im(Z) 对电压-电流延迟误差
 * 不敏感（误差按 cos 量级）；不低于 R/L 粗测频率、不高于 PWM_FREQ/16 */
static float ldqHz(void)
{
    float f = 10.0f * gIdent.rlR / (2.0f * (float)MATH_PI * gIdent.rlL);
    if (f < (float)INJ_RL_HZ) f = (float)INJ_RL_HZ;
    if (f > (float)PWM_FREQ / 16.0f) f = (float)PWM_FREQ / 16.0f;
    return f;
}

/* q 轴电感饱和：观测器模型为割线电感 L(i) = L0 * (1 - per * |i|)（拐点取 0）。正弦注入下测得的基波电感为该模型在一个周期上的
 * 平均（描述函数）：L1(I) = L0 * (1 - k * per * I)，k = 8 / (3π)（|sin|·sin 的基波系数），I 为电流基波幅值。
 * 对各档 (I, L1) 直线拟合：截距 = L0，per = -斜率 / (k * L0)。斜率不为负（不饱和）时 per = 0，L0 取最小档实测值。
 * 模型有效上限取最大实测幅值的 1.5 倍（运行中过载可能超出扫到的电流，在此范围内按拟合斜率外推） */
#define LS_N    4
static const float s_lsK[LS_N] = { 0.5f, 1.0f, 1.5f, 2.0f };
static float s_lsF0;                /* Ld 所用的基准频率 */

/* q 轴第 k 档的注入频率：尽量用基准频率（感抗远大于电阻，对电阻 / 死区误差不敏感），但所需电压 2πf·L·I 不超过注入电压上限的
 * 60%（L 取 R/L 粗测值，凸极 q 轴可能更大）；因电压降频时下限为感抗 3 倍电阻、且不低于 50 Hz（更低时 q 轴交变转矩使转子摆动），
 * 仍超压返回 0（该档及更高档不测）。第一档（0.5 倍额定）与 Ld 相同 */
static float lsHz(float iA)
{
    float vmax = 0.6f * (float)INJ_VMAX_Q15 * (float)(UDC_MAX / 32768.0), l = gIdent.rlL;
    float r = (gIdent.rsOhm > 0.0f) ? gIdent.rsOhm : gIdent.rlR, f = s_lsF0;
    float fv = vmax / (2.0f * (float)MATH_PI * l * iA), flo = 3.0f * r / (2.0f * (float)MATH_PI * l);
    if (flo < 50.0f) flo = 50.0f;
    if (iA > (float)IDENT_VEC_MAX_A) return 0.0f;
    if (f <= fv) return f;                  /* 基准频率可用（与 Ld 同频） */
    return (fv >= flo) ? fv : 0.0f;
}

static void lsFit(void)
{
    float sx = 0, sy = 0, sxx = 0, sxy = 0, n = (float)gIdent.lsN, d, s, l0;
    uint16_t i;
    gIdent.lqH = gIdent.lqAtH[0];
    gIdent.lsPerA = 0.0f;
    gIdent.lsMaxA = 0.0f;
    if (gIdent.lsN < 2u) return;
    for (i = 0; i < gIdent.lsN; i++) {
        float x = gIdent.lqAtA[i], y = gIdent.lqAtH[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
        if (x > gIdent.lsMaxA) gIdent.lsMaxA = x;
    }
    d = n * sxx - sx * sx;
    /* 各档电流跨度不足 1.5 倍时斜率没有信息量：不拟合（per = 0） */
    if (d <= 0.0f || gIdent.lsMaxA < 1.5f * gIdent.lqAtA[0]) { gIdent.lsMaxA = 0.0f; return; }
    s = (n * sxy - sx * sy) / d;
    l0 = (sy - s * sx) / n;
    if (s < 0.0f && l0 > 0.0f) {
        gIdent.lqH = l0;
        gIdent.lsPerA = -s / (l0 * (float)(8.0 / (3.0 * MATH_PI)));
    }
}

static void offerStill(void)
{
    MotorParam_TypeDef p = gParam;
    uint16_t m = 0;
    if (gIdent.rsOhm > 0) { p.rsOhm = gIdent.rsOhm; m |= 1u; }
    if (gIdent.ldH > 0 && gIdent.lqH > 0) {
        p.lqH = gIdent.lqH;
        p.ldH = MOTOR_IPM ? gIdent.ldH : gIdent.lqH;
        m |= 2u;
    }
    if (gIdent.lsMaxA > 0.0f) {     /* 拟合成立才提交，否则沿用配置的饱和曲线；测量范围以外按同一模型外推（下限 MOTOR_LS_SAT_MIN） */
        p.lsKneeA = 0.0;
        p.lsPerA = gIdent.lsPerA;
        p.lsMaxA = 0.0;
        m |= 4u;
    }
    if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) { p.dtEffS = gIdent.deadTimeS; m |= 32u; }
    Param_Offer(&p, m);
}

void Ident_Slow(void)
{
    IdentTest_TypeDef *t = &gIdentTest;
    switch (t->phase) {
    case IDENT_PH_RL: {
        float r = 0, l = 0, ia = 0;
        uint8_t s = injSlow(&r, &l, &ia);
        if (s == 2) { staticAbort(IDENT_WARN_RL_TIMEOUT); break; }
        if (s == 1) {
            MotorParam_TypeDef p = gParam;
            gIdent.rlR = r;
            gIdent.rlL = l;
            if (r <= 0.0f) { staticAbort(IDENT_WARN_RL_TIMEOUT); break; }
            p.rsOhm = r;                    /* 先整定电流环（R 含死区偏大） */
            p.lqH = p.ldH = l;
            (void)Param_ApplyNow(&p);
            FocCore_ApplyVoltageAB(0, 0);
            FocCore_PresetPI(0, 0);
            t->phase = IDENT_PH_NONE;       /* 交给 startup.c 定位，定位完成后 Ident_Begin */
        }
        break;
    }
    case IDENT_PH_DT_STEP:
        FocCore_SetIdqRef((int16_t)(s_dtFrac[t->idx] * (float)(DT_TOP_A / I_MAX * 32768.0)), 0);
        if (++t->cnt > DT_SETTLE_MS) {
            t->sum[0] += gFoc.idq.s16D;
            t->sum[1] += gFoc.vdq.s16D;
        }
        if (t->cnt >= DT_SETTLE_MS + DT_AVG_MS) {
            t->dtId[t->idx] = (int16_t)(t->sum[0] / DT_AVG_MS);
            t->dtVd[t->idx] = (int16_t)(t->sum[1] / DT_AVG_MS);
            t->sum[0] = t->sum[1] = 0;
            t->cnt = 0;
            /* 各档从大到小：不参与拟合的低电流档不必再测 */
            if (++t->idx >= IDENT_DT_N || s_dtFrac[t->idx] * (float)DT_TOP_A < 0.95f * (float)DT_FIT_MIN_A) {
                MotorParam_TypeDef p = gParam;
                t->idx = 0;
                t->phase = IDENT_PH_DT_ZERO;
                dtFit();
                if (gIdent.rsOhm > 0) p.rsOhm = gIdent.rsOhm;
                if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) p.dtEffS = gIdent.deadTimeS;
                (void)Param_ApplyNow(&p);
            }
        }
        break;
    case IDENT_PH_DT_ZERO:
        FocCore_SetIdqRef(0, 0);
        if (++t->cnt >= ZERO_WAIT_MS) {
            t->cnt = 0;
            t->idx = 0;
            injStart(Ident_AlignTheta(), ldqHz(), (float)IDENT_I_INJ_A);   /* 先 d 轴 */
            gIdent.injHz = s_injHz;
            t->phase = IDENT_PH_LDQ;
        }
        break;
    case IDENT_PH_LDQ: {
        /* idx 0：d 轴；idx 1..LS_N：q 轴，电流幅值依次为 s_lsK[] 倍额定电流（不超过 IDENT_VEC_MAX_A），同一频率 */
        float r = 0, l = 0, ia = 0;
        uint8_t s = injSlow(&r, &l, &ia);
        if (s == 2) {
            if (t->idx <= 1) { staticAbort(IDENT_WARN_L_FAIL); break; }
            s = 3;                          /* 大电流档达不到：用已测的档位 */
        }
        if (s == 1) {
            if (t->idx == 0) {
                gIdent.ldH = l;
            } else {
                gIdent.lqAtA[t->idx - 1] = ia;
                gIdent.lqAtH[t->idx - 1] = l;
                gIdent.lsN = (uint8_t)t->idx;
            }
            if (t->idx == 0) s_lsF0 = s_injHz;
            if (++t->idx <= LS_N) {
                float ia1 = s_lsK[t->idx - 1] * (float)MOTOR_I_RATED_A, f = lsHz(ia1);
                if (f > 0.0f) {
                    injStart((int16_t)(Ident_AlignTheta() + ANGLE_90_Q15), f, ia1);
                    break;
                }
            }
            s = 3;
        }
        if (s == 3) {
            FocCore_ApplyVoltageAB(0, 0);
            lsFit();
            offerStill();
            {
                uint8_t ok = (uint8_t)(!(gParamStat.rejectMask & 39u) && gParamStat.pending);
                if (!ok) gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
                s_holdOk = (uint8_t)(gIdentCfg.mode == IDENT_MODE_AUTO && ok);
            }
            /* 收尾定位：交还 startup.c 的辨识定位（定位轴上加定位电流与阻尼，等转子停稳），停稳后经 Ident_Begin 结束。
             * 自动重起动时转子须静止且在已知角度（q 轴大电流档可能推动转子） */
            t->cnt = 0;
            t->phase = IDENT_PH_HOLD;
            Startup_IdentHold();
        }
        break;
    }
    default:
        break;
    }
}

/* 注入段（R/L 粗测、Ld / Lq）由本模块输出电压；其余（死区阶梯、收尾定位）由定位快环运行电流环 */
uint8_t Ident_Fast(void)
{
    if (gIdentTest.phase != IDENT_PH_RL && gIdentTest.phase != IDENT_PH_LDQ) return 0;
    injFast();
    return 1;
}

/* ---- 运行测量：I/F 恒速段测磁链 ------------------------------------------------
 * e = v - R*i - Vdt - jωL*i（拖动坐标系，电流恒定故无 L*di/dt），psi = mean|e| / ω。
 * I/F 无阻尼，参数不准时负载角会摆动，e 在拖动坐标中来回转动：先对矢量平均再取模会按摆幅缩小，
 * 因此每拍先 8 ms 低通（滤掉 PWM / 采样噪声，远快于摆动）再取模、对模平均：转子同步时平均转速 = 拖动转速，与摆幅无关。
 * 凸极：电感压降取决于电流在转子 d/q 上的分量；反电势沿转子 q 轴，由 e 的方向得转子角，在转子坐标中重算（迭代 3 次） */
#define OLFLUX_SETTLE_MS   300
#define OLFLUX_AVG_MS      500
static float s_olfF[4], s_olfE, s_olfEd, s_olfEq;

static float olfEmf(float w, float *pd, float *pq)
{
    float im, r = (float)gParam.rsOhm, l, vdt = 0, vd = s_olfF[2], vq = s_olfF[3], ed, eq;
    float vph = (float)gParam.dtEffS * (float)PWM_FREQ * gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
    float id = s_olfF[0], iq = s_olfF[1];
    uint16_t j;
    im = sqrtf(id * id + iq * iq);
    l = Param_LqAt(im);
    for (j = 0; j < 16; j++) {
        float sn = sinf(((float)j + 0.5f) * (float)(MATH_PI / 32.0)), x = im * sn / (float)INVERTER_DT_I0_A;
        vdt += (x > 1.0f ? 1.0f : x) * sn;
    }
    vdt *= vph / 8.0f;
    if (im > 0.01f) { vd -= vdt * id / im; vq -= vdt * iq / im; }
    vd -= r * id;
    vq -= r * iq;
    ed = vd + w * l * iq;
    eq = vq - w * l * id;
    if ((float)gParam.ldH != l) {
        float ld = (float)gParam.ldH, m;
        for (j = 0; j < 3; j++) {
            float c, sn, a, b, idr, iqr, vdr, vqr;
            m = sqrtf(ed * ed + eq * eq);
            if (m < 1e-6f) break;
            c = eq / m;                 /* 转子 d 轴方向 (c, sn) = (eq, -ed) / |e| */
            sn = -ed / m;
            idr = c * id + sn * iq;
            iqr = -sn * id + c * iq;
            vdr = c * vd + sn * vq;
            vqr = -sn * vd + c * vq;
            a = vdr + w * l * iqr;      /* 转子坐标反电势 (vd' + ω Lq iq', vq' - ω Ld id')，转回拖动坐标 */
            b = vqr - w * ld * idr;
            ed = c * a - sn * b;
            eq = sn * a + c * b;
        }
    }
    *pd = ed;
    *pq = eq;
    return sqrtf(ed * ed + eq * eq);
}

uint8_t Ident_OpenLoopFluxSlow(int32_t ol, int32_t tgt)
{
    const float ki = (float)(I_MAX / 32768.0), kv = (float)(UDC_MAX / 32768.0);
    float x[4];
    uint16_t j;
    if (!Ident_RunActive() || s_olFluxSt == OLF_DONE) return 0;
    if (s_olFluxSt == OLF_NOSYNC && ol == tgt) return 0;     /* 本次未同步：交给切闭环判据（超时 -> 堵转重试），重新加速时再测 */
    if (ol != tgt) {
        s_olFluxSt = OLF_WAIT_STABLE;
        s_olFluxN = 0;
        return 1;
    }
    x[0] = gFoc.idq.s16D * ki;
    x[1] = gFoc.idq.s16Q * ki;
    x[2] = gFoc.vdq.s16D * kv;
    x[3] = gFoc.vdq.s16Q * kv;
    if (s_olFluxSt == OLF_WAIT_STABLE) {
        for (j = 0; j < 4; j++) s_olfF[j] = (s_olFluxN == 0) ? x[j] : s_olfF[j] + (x[j] - s_olfF[j]) * 0.125f;
        if (++s_olFluxN < OLFLUX_SETTLE_MS) return 1;
        s_olFluxSt = OLF_AVG;
        s_olFluxN = 0;
        s_olfE = s_olfEd = s_olfEq = 0;
        return 1;
    }
    {
        float w = (float)ol * (float)(SPEED_MAX / 2147483648.0 * 2.0 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS), ed, eq, mv;
        if (w < 0) w = -w;
        for (j = 0; j < 4; j++) s_olfF[j] += (x[j] - s_olfF[j]) * 0.125f;
        s_olfE += olfEmf(ol < 0 ? -w : w, &ed, &eq);
        s_olfEd += ed;
        s_olfEq += (ol < 0) ? -eq : eq;
        if (++s_olFluxN < OLFLUX_AVG_MS) return 1;
        /* 测量前提：转子跟着拖动转。同步时 e 在拖动坐标中不旋转、方向接近 q 轴；失步时 e 以滑差频率旋转（矢量平均趋于 0），
         * 转子停滞时只剩残差：R / 死区误差沿电流方向，电感误差 ω ΔL i 沿 q 轴、也与 ω 成正比（与反电势同形，只能按幅值区分）。
         * 判据：|矢量平均| > 0.3 mean|e|，方向在 q 轴 ±60° 内，且 mean|e| > 0.2 ω L(i) |i|（电感误差在 20% 以内时不会被当作反电势） */
        mv = sqrtf(s_olfEd * s_olfEd + s_olfEq * s_olfEq);
        {
            float im = sqrtf(s_olfF[0] * s_olfF[0] + s_olfF[1] * s_olfF[1]);
            if (s_olfE < 0.2f * w * Param_LqAt(im) * im * OLFLUX_AVG_MS) mv = 0;
        }
        if (!(mv > 0.3f * s_olfE && s_olfEq > 0.5f * mv)) {
            s_olFluxSt = OLF_NOSYNC;
            return 0;
        }
        if (w > 1.0f) {
            MotorParam_TypeDef p = gParam;
            gIdent.psiWb = s_olfE / OLFLUX_AVG_MS / w;
            /* 加速度系数 = 转矩常数 / 惯量 ∝ psi / J：惯量不辨识，按配置惯量随磁链换算 */
            if (p.psiWb > 0) p.accRpmSPerA *= gIdent.psiWb / p.psiWb;
            gIdent.accRpmSPerA = (float)p.accRpmSPerA;
            p.psiWb = gIdent.psiWb;
            if (!Param_ApplyNow(&p)) Param_Offer(&p, 8u | 16u);
        }
    }
    s_olFluxSt = OLF_DONE;
    if (gIdentCfg.mode != IDENT_MODE_AUTO) return 0;
    s_jSt = 0;                          /* 自动辨识：切闭环后测惯量（Ident_RunSlow） */
    return 0;
}

/* I/F 切闭环失败（堵转超时）：所测磁链可能来自未同步的拖动，下次拖动重新测量 */
void Ident_OpenLoopFluxRearm(void)
{
    if (Ident_RunActive()) s_olFluxSt = OLF_WAIT_STABLE;
}

/* 自动辨识（AUTO）运行段、I/F 磁链尚未测得：openloop.c 冷起动按保守加速度拖动（IDENT_MODE_RUN 属正常起动，不放慢） */
uint8_t Ident_OpenLoopFluxPending(void)
{
    return (uint8_t)(gIdentCfg.mode == IDENT_MODE_AUTO && Ident_RunActive() && s_olFluxSt != OLF_DONE);
}

/* ---- 运行测量：惯量（加速度系数）---------------------------------------------------
 * 闭环稳速后把转速指令升高 dw 再降回原值（测量期间斜坡加速度上限改为 a）。转矩平衡：
 *   iq(t) = T_load(ω) / Kt + (dω/dt) / acc
 * 负载只与转速有关（风机 ∝ ω²、摩擦、恒转矩），在 dw 的小范围内用二次式 b0 + b1 x + b2 x^2 表示，x = (ω - ω0) / dw。
 * 分 5 段对方程积分：稳速 ω0、升速（到稳定）、稳速 ω0+dw、降速（到稳定）、稳速 ω0。每段
 *   mean(iq) = b0 + b1 mean(x) + b2 mean(x^2) + c (ω_末 - ω_初) / (T a_ref)
 * 转速变化取实测值、段首尾都在稳态，结果与速度环跟随快慢无关；5 式 4 元最小二乘，acc = a_ref / c。
 * dw = 15% 指令转速；a 取使加速电流约为 0.3 倍额定电流（按当前加速度系数估计），并保证升速段不短于 0.4 s。
 * 前提是速度环未饱和：转速跟不上指令（3% 以内）不开始，升速段转速没到位则放弃，均保留按磁链换算的值 */
#define J_STABLE_MS     1000
#define J_SEG_MS        300                 /* 稳速段长度、升降速到位后的稳定时间 */
#define J_NP            4
enum { J_WAIT = 0, J_W0, J_UP, J_TOP, J_DN, J_BOT };
static int16_t  s_jCmd, s_jDw;      /* 基准指令、升速量（Q15） */
static int32_t  s_jRamp;            /* 测量用斜坡加速度上限（Q31 / 慢环拍） */
static uint32_t s_jT, s_jTs;
static float    s_jW0, s_jDwRpm, s_jAref;
static float    s_jA[J_NP][J_NP], s_jB[J_NP];
static float    s_sIq, s_sX, s_sX2, s_sW;   /* 当前段累加：iq、x、x^2；段首转速 */
static uint32_t s_sN;

int16_t Ident_SpeedCmd(int16_t cmd)
{
    return (s_jSt == J_UP || s_jSt == J_TOP) ? (int16_t)(cmd + s_jDw) : cmd;
}

int32_t Ident_RampQ31(void)
{
    return (s_jSt >= J_W0 && s_jSt <= J_BOT) ? s_jRamp : 0;
}

/* 4 元线性方程组（高斯消元，选主元） */
static uint8_t solve4(float a[J_NP][J_NP], float *bv, float *x)
{
    uint16_t i, j, k;
    for (i = 0; i < J_NP; i++) {
        uint16_t p = i;
        for (k = (uint16_t)(i + 1); k < J_NP; k++) if (fabsf(a[k][i]) > fabsf(a[p][i])) p = k;
        if (fabsf(a[p][i]) < 1e-12f) return 0;
        if (p != i) {
            for (j = 0; j < J_NP; j++) { float t = a[i][j]; a[i][j] = a[p][j]; a[p][j] = t; }
            { float t = bv[i]; bv[i] = bv[p]; bv[p] = t; }
        }
        for (k = (uint16_t)(i + 1); k < J_NP; k++) {
            float f = a[k][i] / a[i][i];
            for (j = i; j < J_NP; j++) a[k][j] -= f * a[i][j];
            bv[k] -= f * bv[i];
        }
    }
    for (i = J_NP; i-- > 0;) {
        float s = bv[i];
        for (j = (uint16_t)(i + 1); j < J_NP; j++) s -= a[i][j] * x[j];
        x[i] = s / a[i][i];
    }
    return 1;
}

static void jFinish(uint8_t ok)
{
    float x[J_NP];
    s_jSt = 0xFF;
    gIdent.state = IDENT_DONE;
    if (!ok || !solve4(s_jA, s_jB, x)) return;
    gIdent.jIqAcc = x[3];               /* a_ref 加速度所需的电流 A */
    /* 物理前提：加速需要正的电流，且信号足够（> 2% 额定电流）；否则保留按磁链换算的值 */
    if (x[3] > 0.02f * (float)MOTOR_I_RATED_A) {
        MotorParam_TypeDef p = gParam;
        gIdent.accRpmSPerA = s_jAref / x[3];
        p.accRpmSPerA = gIdent.accRpmSPerA;
        if (!Param_ApplyNow(&p)) Param_Offer(&p, 16u);
    }
}

/* 结束当前段：一行积分方程计入最小二乘 */
static void jRow(float w)
{
    float n = (float)s_sN, ph[J_NP], y = s_sIq / n;
    uint16_t i, j;
    ph[0] = 1.0f; ph[1] = s_sX / n; ph[2] = s_sX2 / n;
    ph[3] = (w - s_sW) * (float)SLOWLOOP_FREQ / (n * s_jAref);
    for (i = 0; i < J_NP; i++) { s_jB[i] += ph[i] * y; for (j = 0; j < J_NP; j++) s_jA[i][j] += ph[i] * ph[j]; }
    s_sN = 0; s_sIq = s_sX = s_sX2 = 0; s_sW = w;
}

void Ident_RunSlow(uint8_t spin)
{
    const float ks = (float)(SPEED_MAX / 32768.0), ka = (float)(SPEED_MAX * SLOWLOOP_FREQ / 2147483648.0);
    float sg, w;
    if (s_jSt == 0xFF) return;
    if (!spin || gIdentCfg.mode != IDENT_MODE_AUTO || gIdent.state != IDENT_RUNNING) { s_jSt = J_WAIT; s_jT = s_jTs = 0; return; }
    sg = (s_jCmd < 0) ? -1.0f : 1.0f;
    w = sg * (float)gMeas.speedRaw * ks;                                                /* 按指令转向取正方向；用未滤波转速（滤波滞后会使段首尾转速有误差） */
    if (s_jSt == J_WAIT) {
        /* 指令不变、斜坡已停、转速跟上指令（速度环未饱和）后再稳 J_STABLE_MS */
        if (gSpd.rampAcc != 0 || gSpd.cmd != s_jCmd) { s_jCmd = gSpd.cmd; s_jT = s_jTs = 0; return; }
        if (fabsf((float)(gMeas.speed - s_jCmd)) > 0.03f * fabsf((float)s_jCmd)) {
            s_jT = 0;
            if (++s_jTs > 10u * SLOWLOOP_FREQ) jFinish(0);                              /* 指令下一直跟不上（电流受限）：不测 */
            return;
        }
        if (++s_jT >= J_STABLE_MS) {
            float wc = fabsf((float)s_jCmd * ks), dw = 0.15f * wc, a = 0.3f * (float)MOTOR_I_RATED_A * (float)gParam.accRpmSPerA;
            uint16_t i, j;
            if (a * 0.4f > dw) a = dw / 0.4f;
            if (wc + dw > 0.9f * (float)SPEED_MAX) dw = -dw;                            /* 已接近上限：先降再升 */
            s_jDw = (int16_t)((s_jCmd < 0 ? -dw : dw) / ks);
            s_jDwRpm = fabsf(dw);
            s_jAref = a;
            gIdent.jAccRef = a;
            s_jRamp = (int32_t)(a / ka);
            if (s_jRamp < 1) s_jRamp = 1;
            s_jW0 = wc;
            for (i = 0; i < J_NP; i++) { s_jB[i] = 0; for (j = 0; j < J_NP; j++) s_jA[i][j] = 0; }
            s_sN = 0; s_sIq = s_sX = s_sX2 = 0; s_sW = w;
            s_jT = 0;
            s_jSt = J_W0;
        }
        return;
    }
    if (gSup.speedCmd != s_jCmd) { s_jSt = J_WAIT; s_jT = 0; return; }                 /* 指令被改：重来 */
    if (++s_jT > 20u * SLOWLOOP_FREQ) { jFinish(0); return; }                           /* 到不了目标（电流受限等）：放弃 */
    {
        float x = (w - s_jW0) / s_jDwRpm;
        s_sIq += sg * (float)gFoc.idq.s16Q * (float)(I_MAX / 32768.0);
        s_sX += x; s_sX2 += x * x;
        s_sN++;
    }
    if (s_jSt == J_W0 || s_jSt == J_TOP || s_jSt == J_BOT) {                            /* 稳速段 */
        if (s_jT < J_SEG_MS) return;
        if (s_jSt == J_TOP && s_sX < 0.8f * (float)s_sN) { jFinish(0); return; }        /* 转速没有升到位（电流受限）：放弃 */
        jRow(w);
        if (s_jSt == J_BOT) { jFinish(1); return; }
        s_jSt++; s_jT = 0; s_jTs = 0;
    } else {                                                                            /* 升 / 降速段：斜坡到位后再稳定 J_SEG_MS */
        int16_t tgt = (s_jSt == J_DN) ? s_jCmd : (int16_t)(s_jCmd + s_jDw);
        if (gSpd.rampAcc != 0 || abs(gSpd.ramp - tgt) > abs(s_jDw) / 16) { s_jTs = 0; return; }
        if (++s_jTs < J_SEG_MS) return;
        jRow(w);
        s_jSt++; s_jT = 0;
    }
}
