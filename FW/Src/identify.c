/* ============================================================================
 * @file    identify.c
 * @brief   参数辨识（IDENT_TEST = 1 时编译；正常运行时这些函数不被调用）。方法说明见 identify.h
 *
 * 自动方式（默认）：静止测量 -> 停机后重新起动 -> 运行测量，见 identify.h。
 * 静止测量（给指令后电机不旋转）：Ident_Reset 从 phase 9 开始，Align 子状态的快环 / 慢环交由本模块处理：
 *   phase 9  R/L 粗测：alpha 轴注入 200 Hz 电压，不使用电流环（rlStart / rlFast / rlSlow），测得后立即更新电流环增益；若失败直接终止于 phase 4
 *   定位（正常进行）-> phase 1/2 死区阶梯：d 轴电流 11 档，拟合 vd = Rs*id + 死区电压（dtFit）
 *   phase 3  IPD 对比度（ipdFit）
 *   phase 6  交流阻抗：d / q 轴多频率、多幅值注入（acSegStart / acFast / acSegFinish / acFit），0.5*f0 段按实测电流等幅重测
 *   phase 7  电感饱和 L(i) 拟合（dfStart / dfStep，分摊到多个慢环周期）
 *   phase 4  完成：经门控校验后提交给 param.c（offerStill，不可信时仅提交 Rs / 死区且阻止 AUTO 重起动），回到 Stop 后生效
 * 运行测量（正常起动）：I/F 中测量开环磁链（Ident_OpenLoopFluxSlow），闭环稳速后零电流滑行测量
 *   psi_f 与加速度系数（Ident_RunSlow）。
 * 辨识模式的慢环允许使用浮点（仅在辨识时运行）。
 * ==========================================================================*/
#include "identify.h"
#include "cfg_derived.h"
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

/* ===========================================================================
 * 参数辨识模式（IDENT_TEST），说明见 identify.h
 * =========================================================================*/
volatile IdentCfg_TypeDef gIdentCfg = { 0 };
IdentTest_TypeDef gIdentTest;

/* 死区阶梯各档 d 轴电流（按额定电流缩放）：从定位电流附近向上，再向下细分到小电流段（反映过零附近的特性） */
static const int16_t s_dtLevel[IDENT_DT_N] = {
    IDENT_Q15_I(1.00), IDENT_Q15_I(0.80), IDENT_Q15_I(0.60), IDENT_Q15_I(0.50), IDENT_Q15_I(0.40),
    IDENT_Q15_I(0.30), IDENT_Q15_I(0.25), IDENT_Q15_I(0.20), IDENT_Q15_I(0.15), IDENT_Q15_I(0.10),
    IDENT_Q15_I(0.05)
};
#define DT_SETTLE_MS    300
#define DT_AVG_MS       200
#define ZERO_WAIT_MS    50
#define DT_DRIFT_VQ_MAX 0.45f   /* 死区阶梯前后半窗 vq 漂移 / |vd| 上限（健康 100 组最大 0.300，留 1.5x 余量） */
#define DT_DRIFT_IQ_MAX 0.030f  /* 死区阶梯前后半窗 iq*rlR 漂移 / |vd| 上限（健康 100 组最大 0.0159） */
static uint8_t  s_dtRetry;
static uint16_t s_dtDropMask;


/* 测量用定位角：gIdentCfg.alignDeg（度，可在线修改）换算为 Q15 角度 */
int16_t Ident_AlignTheta(void)
{
    int32_t d = gIdentCfg.alignDeg % 360;
    if (d < 0) d += 360;
    return (int16_t)(d * 65536 / 360);
}

static void rlStart(void);
static uint8_t s_autoStage;         /* 自动方式：0 = 静止测量未完成，1 = 静止测量已完成、执行运行测量 */
static uint8_t s_restartReq;        /* 自动方式：静止测量完成，请求停机后重新起动（Ident_TakeRestart） */

/* 本次起动执行静止测量：仅静止方式；或自动方式且静止测量未完成（gIdent.state 被写 0 = 重新辨识） */
uint8_t Ident_StaticThisStart(void)
{
    if (gIdentCfg.mode == IDENT_MODE_STATIC) return 1;
    if (gIdentCfg.mode != IDENT_MODE_AUTO) return 0;
    if (gIdent.state == IDENT_IDLE) s_autoStage = 0;
    return (uint8_t)(gIdent.state != IDENT_DONE && s_autoStage == 0);
}

/* 运行测量进行中：仅运行方式（每个新指令一次）；或自动方式、静止测量已完成、运行测量尚未完成 */
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
/* 运行辨识开环磁链子状态（s_olFluxSt） */
enum {
    OLF_WAIT_STABLE = 0,            /* 等待开环稳速 OLFLUX_SETTLE_MS */
    OLF_AVG         = 1,            /* 累加平均 dq 电压与电流 OLFLUX_AVG_MS */
    OLF_DONE        = 2             /* 本次起动已测完开环磁链 */
};
static uint8_t s_olFluxSt;          /* 取值见 OLF_*（Ident_OpenLoopFluxSlow） */
static uint16_t s_olFluxN;

/* 开始一次静止测量（进入 Align 时）：从 R/L 粗测开始 */
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
    gIdentTest.phase = IDENT_PH_RL; /* 先测 R/L（电压模式，不使用电流环），测完再回 IDENT_PH_NONE 正常定位 */
}

/* 置为非测量状态（不执行静止测量的起动时清除上一次静止测量遗留的状态） */
void Ident_Idle(void)
{
    gIdentTest.phase = IDENT_PH_NONE;
    s_olFluxSt = OLF_WAIT_STABLE;
    s_olFluxN = 0;
    FocCore_LoadGains();
}

/* 定位完成后开始死区阶梯（IDENT_PH_DT_STEP） */
void Ident_Begin(void)
{
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdentTest.sum[0] = gIdentTest.sum[1] = 0;
    s_dtRetry = 0;
    s_dtDropMask = 0;
    gIdentTest.phase = IDENT_PH_DT_STEP;
}

/* 静止测量是否正在进行（Align 慢环据此将控制交给 Ident_Slow） */
uint8_t Ident_Active(void)
{
    return (uint8_t)(gIdentTest.phase != IDENT_PH_NONE);
}

/* 死区阶梯结束：对 id >= 0.45 A 的各档拟合 vd = Rs*id + Vd0（误差电压在 0.4 A 以上已饱和为常值）。
 * Vd0 为 d 轴方向的误差，换算为每相：Vd0 = Vph * g(θ)，g = 2/3 * Σ|cos(θ - 120k)|（θ = 定位角；0° 时 4/3，90° 时 2/√3）。
 * 慢环中执行，仅在辨识模式下运行一次，使用浮点。 */
static void dtFit(void)
{
    const IdentTest_TypeDef *t = &gIdentTest;
    float sx = 0, sy = 0, sxx = 0, sxy = 0, n = 0, th, g, vbus;
    uint16_t i;
    for (i = 0; i < IDENT_DT_N; i++) {
        float x = t->dtId[i] * (float)(I_MAX / 32768.0), y = t->dtVd[i] * (float)(UDC_MAX / 32768.0);
        if ((s_dtDropMask & (1u << i)) || x < 0.45f * (float)IDENT_I_SCALE) continue;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        n += 1;
    }
    if (n < 2 || (n * sxx - sx * sx) <= 0) {
        gIdent.warn |= IDENT_WARN_DT_UNSTABLE;
        gIdent.rsOhm = 0.0f;
        gIdent.deadVoltV = 0.0f;
        gIdent.deadTimeS = 0.0f;
        return;
    }
    gIdent.rsOhm = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    th = (float)Ident_AlignTheta() * (float)(2.0 * MATH_PI / 65536.0);
    g = (2.0f / 3.0f) * (fabsf(cosf(th)) + fabsf(cosf(th - 2.0943951f)) + fabsf(cosf(th + 2.0943951f)));
    gIdent.deadVoltV = (sy - gIdent.rsOhm * sx) / n / g;
    vbus = gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
    gIdent.deadTimeS = (vbus > 1.0f) ? gIdent.deadVoltV / ((float)PWM_FREQ * vbus) : 0;
}

/* IPD 结束：极性对比度（两侧 d 轴响应之比） */
static void ipdFit(void)
{
    gIdent.ipdRatio = (float)gIpd.ratioQ8 / 256.0f;
}

/* ---- 交流阻抗（phase 6）-----------------------------------------------------------------
 * 序列：方向 d（N 极）、q（N + 90°）× 自适应频率 f0 / f1 / f2（0.4 A 缩放档），再 q 轴 f0 增大幅值 1.0 / 1.5 / 2.0 / 2.5 A（饱和）。
 * f0 = max(25 Hz, R/(2πL), 2*Vdt/(2πL*I_inj))（不超 PWM_FREQ/20），f1/f2 最高至 PWM_FREQ/10，摆动配对 f3 = 0.5*f0。
 * 每段 AC_SETTLE 个周期稳定、AC_INT 个周期积分。
 * 快环：电流环闭环、角度固定，id 指令 = A sin(phi)，phi 用 Q32 相位累加；积分 Σv·sin、Σv·cos、Σi·sin、Σi·cos（整数）。
 * 本拍计算的电压延迟 CTRL_INVPARK_ADVANCE（1.5）个 PWM 周期才作用（下一周期生效、作用区间中点），与反 Park 角度补偿
 * 为同一物理量：电压按 phi(k + 1.5) 做相关。 */
#define AC_NF       3
#define AC_NSEG     16
#define AC_SETTLE   4
#define AC_INT      8
#define AC_V_DELAY  CTRL_INVPARK_ADVANCE
#define AC_F_MIN_HZ 25.0f
#define AC_DT_SNR   2.0f        /* 目标 ωL·I_inj >= AC_DT_SNR * V_dt */
#define AC_SNR_MIN  1.0f        /* 实测感抗压降相对死区电压最低门限（低于则置 IDENT_WARN_LOW_SNR） */
/* 频率编号 3 = 0.5*f0：仅用于 q 轴，与 f0 配对测量转子摆动（见 acFit） */
static uint16_t s_acPer[AC_NF + 1];     /* 拍 / 周期（acInitFreqs 计算） */
static float    s_acHz[AC_NF + 1];      /* 实际注入频率（Hz） */
static const uint8_t  s_acSegQ[AC_NSEG]  = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };      /* 0 = d 轴，1 = q 轴 */
static const uint8_t  s_acSegF[AC_NSEG]  = { 0, 1, 2, 0, 1, 2, 3, 0, 0, 0, 0, 3, 0, 0, 0, 0 };      /* 频率编号 */
/* 0.5*f0 0.4 A（段 6）紧跟 0.4 A 各段、位于大幅值段之前：大幅值段撤除偏置后转子会摆动一段时间，此时测小幅值不准确 */
#define AC_S_Q04    3       /* q f0 0.4 A */
#define AC_S_Q04L   6       /* q 0.5*f0 0.4 A */
#define AC_S_Q10    7       /* q f0 1.0 A（大幅值第一段，后面依次 1.5 / 2.0 / 2.5） */
#define AC_S_Q10L   11      /* q 0.5*f0 1.0 A */
#define AC_S_Q10B2  12      /* q f0 1.0 / 1.5 / 2.0 / 2.5 A，偏置 2 倍 */
static const int16_t  s_acSegA[AC_NSEG]  = {
    IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4), IDENT_Q15_I(0.4),
    IDENT_Q15_I(0.4), IDENT_Q15_I(1.0), IDENT_Q15_I(1.5), IDENT_Q15_I(2.0), IDENT_Q15_I(2.5), IDENT_Q15_I(1.0),
    IDENT_Q15_I(1.0), IDENT_Q15_I(1.5), IDENT_Q15_I(2.0), IDENT_Q15_I(2.5) };
/* 大幅值 q 段同时沿 N 极施加直流 0.1 A * (A/1A)^2 以固定转子：q 方向交流电流产生的磁阻转矩平均值 ∝ (Lq - Ld) * A^2 * sin 2δ，
 * 本电机 Ld < Lq，q 方向为不稳定平衡点，转子会逐渐转离（注入方向随之偏离 q 轴）。
 * N 极直流产生的永磁转矩刚度 ∝ psi * Ib，Ib > (Lq - Ld) A^2 / (2 psi) ≈ 0.05 A^2 即可抑制，取 2 倍。
 * 偏置刚度对应约 2 Hz 的摆动固有频率，远低于 25 Hz 注入频率，摆动修正不受影响。
 * 偏置本身会加深铁芯饱和、使 Lq 测量值偏小（交叉饱和），因此四档各测偏置 1 倍、2 倍两次，
 * 线性外推到 0 偏置：L0 = 2 L1 - L2。 */
static const int16_t  s_acSegB[AC_NSEG]  = { 0, 0, 0, 0, 0, 0,
    0, IDENT_Q15_I(0.1), IDENT_Q15_I(0.225), IDENT_Q15_I(0.4), IDENT_Q15_I(0.625), IDENT_Q15_I(0.1),
    IDENT_Q15_I(0.2), IDENT_Q15_I(0.45), IDENT_Q15_I(0.8), IDENT_Q15_I(1.25) };
static volatile uint16_t s_acSeg;    /* 段号 0..AC_NSEG-1，AC_NSEG 表示完成 */
static int16_t  s_acAmp, s_acBias;
static int32_t  s_acEnv, s_acEnvStep;  /* 幅值包络（Q15）：每段开始 AC_RAMP 个周期内从 0 线性升至 1 */
#define AC_RAMP     2
/* 包络的作用：q 段电流即转矩。若从 0 相位直接施加正弦，转矩 ∝ sin，其积分 = 转速 ∝ (1 - cos)，平均值不为 0，
 * 转子将以恒定转速漂移，注入方向随之偏离 q 轴。包络缓慢上升可使漂移速度减小 2π*AC_RAMP 倍 */
static uint32_t s_acPhi, s_acInc, s_acDly;
static uint16_t s_acT;
static int16_t  s_acTheta;
static int32_t  s_acVs, s_acVc, s_acIs, s_acIc, s_acEs, s_acEc;
static int16_t  s_acVph;            /* 每相死区电压（Q15，死区阶梯测得的值） */
#define AC_DT_KI_Q8 ((int32_t)(256.0 * I_MAX / INVERTER_DT_I0_A + 0.5))
static float    s_acZre[AC_NF], s_acZim[AC_NSEG];     /* 实部只用 d 轴三段（acRohm）；电流幅值直接写入 gIdent.lqAtA */
static float    s_zCmdR[AC_NF], s_zCmdI[AC_NF], s_zDtR[AC_NF], s_zDtI[AC_NF];  /* d 轴三频点指令电压/死区电压相对电流的复阻抗，用于延迟残差估算 */
static float    s_acDlyShift, s_acI00, s_acZreQ04, s_acI04L, s_acI10L; /* 延迟残差（PWM 拍）、基频 d/q 小信号电流与实部、0.5*f0 电流幅值 */
static float    s_acGain[AC_NSEG];      /* 各段指令幅值缩放（仿射/割线幅值标定及 0.5*f0 等幅对齐） */
static uint8_t  s_acRetry[AC_NSEG];     /* 各段重测次数 */
static uint8_t  s_tryHasLo, s_tryHasHi, s_tryHasCap, s_tryCapped, s_capMask;  /* s_capMask bit n: q轴第n档(0..4)指令被T1封顶 */
static int16_t  s_acPlanA[4], s_acPlanB[8]; /* 运行期重规划的 q 轴大幅值档目标与 1x/2x 偏置（seg 7..10, 12..15） */
static uint8_t  s_l0Only;                   /* 1 = 含 2x 偏置电流余量不足 1.52*t7，只提交 L0，不提交 lsKnee/lsPer */
static float    s_tryG1, s_tryGPrev, s_tryIPrev, s_tryXPrev;
static float    s_tryGLo, s_tryILo, s_tryGHi, s_tryIHi, s_tryHiCap;
static volatile uint8_t s_acDone;    /* 快环置 1：本段积分完成，等待慢环读取 */

static int16_t acSegTargetA(uint16_t seg)
{
    if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) return s_acPlanA[seg - AC_S_Q10];
    if (seg >= AC_S_Q10B2 && seg < AC_S_Q10B2 + 4u) return s_acPlanA[seg - AC_S_Q10B2];
    return s_acSegA[seg];
}

static int16_t acSegTargetB(uint16_t seg)
{
    if (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4u) return s_acPlanB[seg - AC_S_Q10];
    if (seg >= AC_S_Q10B2 && seg < AC_S_Q10B2 + 4u) return s_acPlanB[4u + (seg - AC_S_Q10B2)];
    return s_acSegB[seg];
}

static void acSetFreq(uint16_t idx, float hz)
{
    uint32_t per = (hz > 0.0f) ? (uint32_t)((float)PWM_FREQ / hz + 0.5f) : 65535u;
    uint32_t maxPer = 65535u / (uint32_t)(AC_SETTLE + AC_INT);   /* 保证 s_acT (uint16_t) 与 s_acIs/Vs (int32_t) 不溢出 */
    if (per < 10u) per = 10u;
    if (per > maxPer) per = maxPer;
    s_acPer[idx] = (uint16_t)per;
    s_acHz[idx] = (float)PWM_FREQ / (float)per;
}

/* 按给定 Ld/Lq/R 整定交流注入期间电流环 PI 带宽（不低于 CURRENT_BW_HZ，最高 bwMax）；完成后由 FocCore_LoadGains 恢复 */
static void acSetCurBw(float lD, float lQ, float r, float bwMax)
{
    float bw = 2.5f * s_acHz[0];
    if (bw < (float)CURRENT_BW_HZ) bw = (float)CURRENT_BW_HZ;
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

/* 基于 Rs/RL 与死区电压确定自适应注入频率 f0 / f1 / f2 / 0.5*f0，并提高小电感电机的测量期电流环带宽 */
static void acInitFreqs(void)
{
    float r = gIdent.rsOhm > 0.0f ? gIdent.rsOhm : (gIdent.rlR > 0.0f ? gIdent.rlR : (float)gParam.rsOhm);
    float l = gIdent.rlL > 0.0f ? gIdent.rlL : (float)gParam.lqH;
    float iInj = (float)s_acSegA[0] * (float)(I_MAX / 32768.0);
    float iMax = (float)s_acSegA[AC_S_Q10 + 3] * (float)(I_MAX / 32768.0);
    float vDt = gIdent.deadVoltV > 0.0f ? gIdent.deadVoltV : 0.0f;
    float vbus = (gMeas.vbusAvg > 0) ? (float)gMeas.vbusAvg * (float)(UDC_MAX / 32768.0) : (float)UDC_RATE;
    float f0 = (l > 0.0f) ? r / (2.0f * (float)MATH_PI * l) : AC_F_MIN_HZ;
    float fDt = (l > 0.0f && iInj > 0.0f) ? (AC_DT_SNR * vDt) / (2.0f * (float)MATH_PI * l * iInj) : 0.0f;
    float fMax0 = (float)PWM_FREQ / 20.0f;
    float fMaxHi = (float)PWM_FREQ / 10.0f;
    float f2, f1;
    uint16_t f;
    if (fDt > f0) f0 = fDt;
    if (l > 0.0f && iMax > 0.0f) {      /* 大幅值段电抗压降留出母线电压裕量（90% SVPWM 线电压半径） */
        float fV = (0.90f * (vbus > 1.0f ? vbus : (float)UDC_RATE) / 1.7320508f)
                 / (2.0f * (float)MATH_PI * l * iMax);
        if (f0 > fV) f0 = fV;
    }
    if (f0 < AC_F_MIN_HZ) f0 = AC_F_MIN_HZ;
    if (f0 > fMax0) f0 = fMax0;
    acSetFreq(0, f0);
    f0 = s_acHz[0];                     /* 反算整数周期对应的实际基频 */
    f2 = (4.0f * f0 <= fMaxHi) ? 4.0f * f0 : fMaxHi;
    f1 = 2.0f * f0;
    if (f1 > 0.5f * (f0 + f2)) f1 = sqrtf(f0 * f2);
    acSetFreq(1, f1);
    acSetFreq(2, f2);
    acSetFreq(3, 0.5f * f0);
    for (f = 0; f <= AC_NF; f++) gIdent.acHz[f] = s_acHz[f];
    for (f = 0; f < AC_NSEG; f++) { s_acGain[f] = 1.0f; s_acRetry[f] = 0; }
    for (f = 0; f < 4u; f++) {
        s_acPlanA[f] = s_acSegA[AC_S_Q10 + f];
        s_acPlanB[f] = s_acSegB[AC_S_Q10 + f];
        s_acPlanB[4u + f] = s_acSegB[AC_S_Q10B2 + f];
    }
    s_l0Only = 0;
    s_capMask = 0;
    s_acDlyShift = 0.0f;
    s_acI00 = 0.0f;
    s_acZreQ04 = 0.0f;
    s_acI04L = 0.0f;
    s_acI10L = 0.0f;
    gIdent.snrMin = 1e9f;
    acSetCurBw(l, l, r, (float)PWM_FREQ / 30.0f);
}

static void acEvalDlyPoint(uint16_t f, float sh, float *zrOut, float *ziOut)
{
    float ph = sh * (2.0f * (float)MATH_PI * s_acHz[f] / (float)PWM_FREQ);
    float c = cosf(ph), s = sinf(ph);
    *zrOut = c * s_zCmdR[f] - s * s_zCmdI[f] - s_zDtR[f];
    *ziOut = s * s_zCmdR[f] + c * s_zCmdI[f] - s_zDtI[f];
}

/* 计算候选延迟修正 sh（PWM 拍）下三频点实部一致性代价；不物理（感抗<=0 或实部严重为负）返回负数 */
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

/* 利用 d 轴三频点实部一致性（并微弱锚定直流 rsOhm）估算电压-电流总延迟相对于 AC_V_DELAY 的残差（PWM 拍）。
 * 回填修正后的 s_acZre/s_acZim[0..2] 及与延迟无关的幅值法电感 acLdMagH[0..2]。 */
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
    }
    if (bestCost >= 1e29f) bestSh = 0.0f;
    gIdent.acDlyShift = bestSh;
    gIdent.acDlyTicks = (float)AC_V_DELAY - bestSh;
    /* 仅在默认延迟导致非物理解（baseCost < 0）或三频点实部漂移超过 0.3*Rs 且拟合后离散度降为 <20% 时才修正延迟，避免噪声过拟合 */
    if (bestCost < 1e29f && (baseCost < 0.0f ||
        (fabsf(bestSh) > 0.30f && baseCost > 0.27f * rRef * rRef && bestCost < 0.20f * baseCost))) {
        s_acDlyShift = bestSh;
        gIdent.warn |= IDENT_WARN_DLY_MISMATCH;
    } else {
        s_acDlyShift = 0.0f;
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

static void acSegStart(uint16_t seg)    /* 慢环 */
{
    uint16_t f = s_acSegF[seg];
    double dlyTicks = (double)AC_V_DELAY - (double)s_acDlyShift;
    if (dlyTicks < 0.0) dlyTicks = 0.0;
    if (s_acRetry[seg] == 0u) {
        s_tryHasLo = s_tryHasHi = s_tryHasCap = s_tryCapped = 0;
        s_tryG1 = s_tryGPrev = s_tryIPrev = 0.0f;
        s_tryGLo = s_tryILo = s_tryGHi = s_tryIHi = s_tryHiCap = 0.0f;
    }
    s_acTheta = (int16_t)((gIpd.ok ? gIpd.theta : Ident_AlignTheta()) + (s_acSegQ[seg] ? ANGLE_90_Q15 : 0));
    {
        int32_t bx = gIdentCfg.biasX ? (int32_t)gIdentCfg.biasX : 1;
        int32_t lim = (int32_t)(IDENT_VEC_MAX_A / I_MAX * 32768.0 + 16.5);
        int32_t ampRaw = (s_acGain[seg] == 1.0f) ? (int32_t)acSegTargetA(seg)
                       : (int32_t)((float)acSegTargetA(seg) * s_acGain[seg] + 0.5f);
        int32_t amp = QMath_Clamp(ampRaw, lim);
        int64_t b = -(int64_t)acSegTargetB(seg) * bx;   /* 注入坐标系旋转了 +90°，N 极位于其 -q 方向 */
        int32_t maxB = (int32_t)sqrtf((float)((double)lim * lim - (double)amp * amp));
        s_tryCapped = (ampRaw > lim) ? 1u : 0u;
        if (b > maxB) b = maxB;
        if (b < -maxB) b = -maxB;
        s_acAmp = (int16_t)amp;
        s_acBias = (int16_t)b;
    }
    s_acEnv = 0;
    s_acEnvStep = (int32_t)(32768L / (AC_RAMP * (int32_t)s_acPer[f])) + 1;
    s_acInc = (uint32_t)(4294967296.0 / (double)s_acPer[f] + 0.5);
    s_acDly = (uint32_t)(dlyTicks * (double)s_acInc + 0.5);
    s_acPhi = 0;
    s_acT = 0;
    s_acVs = s_acVc = s_acIs = s_acIc = s_acEs = s_acEc = 0;
    s_acVph = (int16_t)(gIdent.deadVoltV * (float)(32768.0 / UDC_MAX));
    s_acDone = 0;
    s_acSeg = seg;
}

/* 交流阻抗快环：在本段注入角（d 或 q 轴方向）上，电流环跟踪正弦 d 轴电流指令（包络从 0 渐入），
 * 稳定 AC_SETTLE 个周期后，按注入相位对电流、电压、死区电压做正交积分（sin / cos 分量），积满 AC_INT 个周期后置完成 */
static void acFast(void)
{
    uint16_t seg = s_acSeg, per = s_acPer[s_acSegF[seg < AC_NSEG ? seg : 0]];
    int16_t ref = 0;
    if (seg < AC_NSEG && !s_acDone) {
        SinCos_TypeDef aOld = FocMath_SinCos((int16_t)(s_acPhi >> 16));
        SinCos_TypeDef b = FocMath_SinCos((int16_t)((s_acPhi + s_acDly) >> 16));
        SinCos_TypeDef a;
        if (s_acT >= AC_SETTLE * per) {
            /* 死区电压沿注入方向的分量（按本拍电流，模型同 flux_obs.c dtComp）：仅用于将实部还原为 Rs 以作校核 */
            int32_t ia = gMeas.iab.s16Alpha, ibs = ((int32_t)gMeas.iab.s16Beta * Q15_SQRT3_HALF) >> 15, xa, xb, xc, ea, eb, e;
            xa = QMath_Clamp((ia * AC_DT_KI_Q8) >> 8, 32767);
            xb = QMath_Clamp(((-ia / 2 + ibs) * AC_DT_KI_Q8) >> 8, 32767);
            xc = QMath_Clamp(((-ia / 2 - ibs) * AC_DT_KI_Q8) >> 8, 32767);
            ea = ((2 * xa - xb - xc) * Q15_ONE_THIRD) >> 15;
            eb = ((xb - xc) * Q15_INV_SQRT3) >> 15;
            {
                SinCos_TypeDef d = FocMath_SinCos(s_acTheta);
                e = (((ea * d.s16Cos + eb * d.s16Sin) >> 15) * s_acVph) >> 15;
            }
            s_acEs += (e * aOld.s16Sin) >> 15;
            s_acEc += (e * aOld.s16Cos) >> 15;
        }
        s_acPhi += s_acInc;
        a = FocMath_SinCos((int16_t)(s_acPhi >> 16));
        if (s_acEnv < 32767) { s_acEnv += s_acEnvStep; if (s_acEnv > 32767) s_acEnv = 32767; }
        ref = (int16_t)(((((int32_t)s_acAmp * s_acEnv) >> 15) * a.s16Sin) >> 15);
        FocCore_SetIdqRef(ref, s_acBias);
        gFoc.theta = s_acTheta;
        gFoc.thetaInv = s_acTheta;
        FocCore_CurrentLoop();
        FocCore_Modulate(SEVENMODE);
        if (s_acT >= AC_SETTLE * per) {
            s_acIs += ((int32_t)gFoc.idq.s16D * aOld.s16Sin) >> 15;
            s_acIc += ((int32_t)gFoc.idq.s16D * aOld.s16Cos) >> 15;
            s_acVs += ((int32_t)gFoc.vdq.s16D * b.s16Sin) >> 15;
            s_acVc += ((int32_t)gFoc.vdq.s16D * b.s16Cos) >> 15;
        }
        if (++s_acT >= (AC_SETTLE + AC_INT) * per) s_acDone = 1;
        return;
    }
    FocCore_SetIdqRef(0, 0);
    gFoc.theta = s_acTheta;
    gFoc.thetaInv = s_acTheta;
    FocCore_CurrentLoop();
    FocCore_Modulate(SEVENMODE);
}

/* 单段幅值标定（仿射首步 + 两点割线/二分 + 超调封顶）：返回 1 = 需重测本段；0 = 完成 */
static uint8_t acRetryGain(uint16_t seg, float ref, float accTol, float failTol, uint8_t maxRetry,
                           float iAmp, float vAmp, float vLim, uint8_t checkVLim)
{
    float g = s_acGain[seg], gNext;
    if (ref <= 0.0f || iAmp <= 0.0f) return 0;
    if (checkVLim && iAmp < (1.0f - accTol) * ref && vAmp > vLim) {
        gIdent.warn |= IDENT_WARN_LOW_SNR;
        return 0;
    }
    if (iAmp > 1.3f * ref && (!s_tryHasCap || g < s_tryHiCap)) {
        s_tryHasCap = 1;
        s_tryHiCap = g;
    }
    if (iAmp <= ref && (!s_tryHasLo || iAmp > s_tryILo || (iAmp == s_tryILo && g > s_tryGLo))) {
        s_tryHasLo = 1;
        s_tryGLo = g;
        s_tryILo = iAmp;
    }
    if (iAmp >= ref && (!s_tryHasHi || iAmp < s_tryIHi || (iAmp == s_tryIHi && g < s_tryGHi))) {
        s_tryHasHi = 1;
        s_tryGHi = g;
        s_tryIHi = iAmp;
    }
    if (fabsf(iAmp - ref) <= accTol * ref) return 0;
    if (s_acRetry[seg] >= maxRetry) {
        if (failTol > 0.0f && fabsf(iAmp - ref) > failTol * ref)
            gIdent.warn |= IDENT_WARN_AMP_FAIL;
        return 0;
    }
    if (s_acRetry[seg] == 0u) {
        gNext = g + (ref - iAmp) / ref;
        if (s_tryHasCap && gNext > s_tryHiCap) gNext = 0.7f * s_tryHiCap;
        if (gNext < 0.25f * g) gNext = 0.25f * g;
        s_tryG1 = gNext;
    } else {
        if (s_tryHasLo && s_tryHasHi && s_tryGHi > s_tryGLo + 1e-4f) {
            if (s_tryIHi > s_tryILo + 1e-4f * ref) {
                gNext = s_tryGLo + (ref - s_tryILo) * (s_tryGHi - s_tryGLo) / (s_tryIHi - s_tryILo);
            } else {
                gNext = 0.5f * (s_tryGLo + s_tryGHi);
            }
            if (!(gNext > s_tryGLo + 0.05f * (s_tryGHi - s_tryGLo) &&
                  gNext < s_tryGHi - 0.05f * (s_tryGHi - s_tryGLo))) {
                gNext = 0.5f * (s_tryGLo + s_tryGHi);
            }
        } else {
            float dg = g - s_tryGPrev, di = iAmp - s_tryIPrev;
            if (dg * di > 0.0f && fabsf(di) > 1e-3f * ref) {
                gNext = s_tryGPrev + (ref - s_tryIPrev) * dg / di;
            } else if (iAmp < ref) {
                gNext = g + (ref - iAmp) / ref;
            } else {
                gNext = 0.7f * g;
            }
        }
        if (gNext < 0.5f * s_tryG1) gNext = 0.5f * s_tryG1;
        if (gNext > 2.0f * s_tryG1) gNext = 2.0f * s_tryG1;
        if (s_tryHasCap && gNext >= s_tryHiCap) {
            gNext = s_tryHasLo ? 0.5f * (s_tryGLo + s_tryHiCap) : 0.7f * s_tryHiCap;
            if (gNext < 0.5f * s_tryG1) gNext = 0.5f * s_tryG1;
        }
    }
    if (s_tryCapped && gNext >= g) {
        if (failTol > 0.0f && fabsf(iAmp - ref) > failTol * ref)
            gIdent.warn |= IDENT_WARN_AMP_FAIL;
        return 0;
    }
    s_tryGPrev = g;
    s_tryIPrev = iAmp;
    s_tryXPrev = (float)s_acAmp * (float)(I_MAX / 32768.0);
    s_acGain[seg] = gNext;
    s_acRetry[seg]++;
    return 1;
}

/* 慢环：读取一段积分，Z = V/I（复数，以 sin 为实轴）。电压先扣除死区电压（与电流同时刻，无需延时对齐）：
 * 扣除后的实部应接近 Rs（用于校核死区模型）；死区与电流同相，是否扣除均不影响虚部（电感）。
 * 返回 1 = 需调整 s_acGain 并重测本段；0 = 完成 */
static uint8_t acSegFinish(uint16_t seg)
{
    float vr = (float)(s_acVs - s_acEs), vi = (float)(s_acVc - s_acEc), ir = (float)s_acIs, ii = (float)s_acIc, d = ir * ir + ii * ii;
    float kv = (float)(UDC_MAX / 32768.0), ki = (float)(I_MAX / 32768.0), kvi = kv / ki;
    float iAmp;
    if (d <= 0.0f) {
        s_acZim[seg] = 0.0f;
        if (seg < AC_NF) {
            s_zCmdR[seg] = s_zCmdI[seg] = s_zDtR[seg] = s_zDtI[seg] = s_acZre[seg] = 0.0f;
        }
        gIdent.warn |= IDENT_WARN_AC_FAIL;
        return 0;
    }
    iAmp = 2.0f * sqrtf(d) / (float)(AC_INT * s_acPer[s_acSegF[seg]]) * ki;
    if (seg < 6u || seg == AC_S_Q10) {
        float ref = (float)acSegTargetA(seg) * ki;
        float cr = (float)s_acVs, ci = (float)s_acVc;
        float vAmp = 2.0f * sqrtf(cr * cr + ci * ci) / (float)(AC_INT * s_acPer[s_acSegF[seg]]) * kv;
        float vdc = (gMeas.vbusAvg > 0) ? (float)gMeas.vbusAvg * kv : (float)UDC_RATE;
        float vLim = 0.85f * (0.90f * vdc / 1.7320508f);
        if (acRetryGain(seg, ref, 0.15f, 0.25f, 3u, iAmp, vAmp, vLim, 1u)) return 1;
        if (seg == AC_S_Q04 && iAmp > 1.3f * ref) gIdent.warn |= IDENT_WARN_AMP_FAIL;
        if (seg == AC_S_Q10) {  /* seg 6 重整定后用 seg 7 最后两点标定 I = α·I_cmd - β，含 2x 偏置预算重规划 seg 8-10/12-15 */
            float xCurr = (float)s_acAmp * ki;
            float alpha = 1.0f, beta = xCurr - iAmp;
            float limA = (float)IDENT_VEC_MAX_A * 0.98f, sc = (float)IDENT_I_SCALE;
            float bx = gIdentCfg.biasX ? (float)gIdentCfg.biasX : 1.0f;
            float tTop = 2.5f * sc;
            uint16_t k;
            if (s_acRetry[seg] > 0u) {
                float dx = xCurr - s_tryXPrev;
                float dy = iAmp - s_tryIPrev;
                if (dx * dy > 0.0f && fabsf(dx) > 1e-4f * ref) {
                    alpha = dy / dx;
                    if (alpha < 0.2f) alpha = 0.2f;
                    if (alpha > 2.0f) alpha = 2.0f;
                    beta = alpha * xCurr - iAmp;
                }
            }
            {   /* 二分求含 2x 偏置合成矢量不超 limA 的最大目标电流 tTop */
                float lo = iAmp, hi = 2.5f * sc;
                uint16_t it;
                if (hi < lo) hi = lo;
                for (it = 0; it < 12u; it++) {
                    float mid = (it == 0u) ? hi : 0.5f * (lo + hi);
                    float ic = (mid + beta) / alpha;
                    float r = (sc > 0.0f) ? (mid / sc) : 0.0f;
                    float b2 = 0.2f * r * r * sc * bx;
                    if (ic < 0.25f * mid) ic = 0.25f * mid;
                    if (ic * ic + b2 * b2 <= limA * limA) {
                        lo = mid;
                        if (it == 0u) break;
                    } else {
                        hi = mid;
                    }
                }
                tTop = lo;
            }
            {
                float ic0 = (iAmp + beta) / alpha;
                float r0 = (sc > 0.0f) ? (iAmp / sc) : 0.0f;
                float b20 = 0.2f * r0 * r0 * sc * bx;
                if (ic0 < 0.25f * iAmp) ic0 = 0.25f * iAmp;
                if (s_tryCapped || iAmp <= 0.0f || ic0 * ic0 + b20 * b20 > limA * limA || tTop < 1.52f * iAmp) {
                    s_l0Only = 1u;
                    for (k = AC_S_Q10 + 1u; k < AC_NSEG; k++) {
                        float t, icmd;
                        if (k == AC_S_Q10L) continue;
                        t = (float)acSegTargetA(k) * ki;
                        icmd = (t + beta) / alpha;
                        if (icmd < 0.25f * t) icmd = 0.25f * t;
                        s_acGain[k] = (t > 0.0f) ? (icmd / t) : 1.0f;
                    }
                } else {
                    float q = powf(tTop / iAmp, 1.0f / 3.0f), tk = iAmp;
                    int32_t qA0 = (int32_t)(iAmp / ki + 0.5f);
                    if (qA0 > 32767) qA0 = 32767;
                    s_acPlanA[0] = (int16_t)qA0;
                    s_acGain[AC_S_Q10B2] = ic0 / iAmp;
                    for (k = 1u; k < 4u; k++) {
                        float rk, icmd, b1, b2;
                        int32_t qA, qB1, qB2;
                        tk = (k == 3u) ? tTop : (tk * q);
                        rk = (sc > 0.0f) ? (tk / sc) : 0.0f;
                        icmd = (tk + beta) / alpha;
                        if (icmd < 0.25f * tk) icmd = 0.25f * tk;
                        b1 = 0.1f * rk * rk * sc;
                        b2 = 0.2f * rk * rk * sc;
                        qA = (int32_t)(tk / ki + 0.5f);
                        qB1 = (int32_t)(b1 / ki + 0.5f);
                        qB2 = (int32_t)(b2 / ki + 0.5f);
                        if (qA > 32767) qA = 32767;
                        if (qB1 > 32767) qB1 = 32767;
                        if (qB2 > 32767) qB2 = 32767;
                        s_acPlanA[k] = (int16_t)qA;
                        s_acPlanB[k] = (int16_t)qB1;
                        s_acPlanB[4u + k] = (int16_t)qB2;
                        s_acGain[AC_S_Q10 + k] = (tk > 0.0f) ? (icmd / tk) : 1.0f;
                        s_acGain[AC_S_Q10B2 + k] = s_acGain[AC_S_Q10 + k];
                    }
                }
            }
        }
    } else if (seg == AC_S_Q04L || seg == AC_S_Q10L) {
        float ref = (seg == AC_S_Q04L) ? gIdent.lqAtA[0] : gIdent.lqAtA[1];
        if (acRetryGain(seg, ref, 0.05f, 0.0f, 3u, iAmp, 0.0f, 0.0f, 0u)) return 1;
    }
    if (seg < AC_NF) {
        float cr = (float)s_acVs, ci = (float)s_acVc, er = (float)s_acEs, ei = (float)s_acEc;
        s_zCmdR[seg] = (cr * ir + ci * ii) / d * kvi;
        s_zCmdI[seg] = (ci * ir - cr * ii) / d * kvi;
        s_zDtR[seg]  = (er * ir + ei * ii) / d * kvi;
        s_zDtI[seg]  = (ei * ir - er * ii) / d * kvi;
        s_acZre[seg] = (vr * ir + vi * ii) / d * kvi;
    }
    s_acZim[seg] = (vi * ir - vr * ii) / d * kvi;
    if (seg == 0) s_acI00 = iAmp;
    if (seg == AC_S_Q04) s_acZreQ04 = (vr * ir + vi * ii) / d * kvi;
    if (seg == AC_NF - 1) acCalDelay();     /* d 轴三频点测完：拟合延迟偏差并更新 s_acZre/s_acZim[0..2] */
    if (seg == AC_S_Q04L) {     /* 小幅值段测完：用实测小信号 Ld / Lq 重算电流环增益后再测大幅值饱和段（额外延迟时按延迟下调带宽上限） */
        float w0 = 2.0f * (float)MATH_PI * s_acHz[0];
        float r = gIdent.rsOhm > 0.0f ? gIdent.rsOhm : (float)gParam.rsOhm;
        float bwMax = (s_acDlyShift < -0.25f) ? (float)PWM_FREQ / 45.0f : (float)PWM_FREQ / 30.0f;
        acSetCurBw(s_acZim[0] / w0, s_acZim[AC_S_Q04] / w0, r, bwMax);
        s_acI04L = iAmp;
    }
    if (seg == AC_S_Q10L)
        s_acI10L = iAmp;
    if (seg == AC_S_Q04 || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4)) {   /* q 轴 f0 五档的电流幅值（Σ i sin = A N / 2） */
        uint16_t n = (seg == AC_S_Q04) ? 0u : (uint16_t)(seg - AC_S_Q10 + 1u);
        gIdent.lqAtA[n] = iAmp;
        if (s_tryCapped) s_capMask |= (uint8_t)(1u << n);
        else s_capMask &= (uint8_t)~(1u << n);
    }
    return 0;
}

/* q 轴注入的转子摆动修正系数（理论值）Kmot = 1.5 p^2 psi^2 / J = p * psi * acc * 2π/60（acc = 加速度 / 转矩电流）。
 * 若运行辨识已测得（gIdent.psiWb、accRpmSPerA 非 0）则使用测得值，否则使用 cfg_motor.h。仅作对照：修正使用 0.5*f0 / f0 实测值 */
static float acKmot(void)
{
    float psi = gIdent.psiWb > 0 ? gIdent.psiWb : (float)gParam.psiWb;
    float acc = gIdent.accRpmSPerA > 0 ? gIdent.accRpmSPerA : (float)gParam.accRpmSPerA;
    return (float)MOTOR_POLE_PAIRS * psi * acc * (float)(2.0 * MATH_PI / 60.0);
}

/* 交流阻抗汇总：q 轴扣除转子摆动影响（0.5*f0 / f0 成对测出摆动系数），得到各频率的 Ld、Lq 及各幅值的 Lq，
 * 大幅值段由 1 倍 / 2 倍 N 极偏置外推到 0 偏置；最后取基频 f0 的值作为 Ld / Lq */
static void acFit(void)
{
    uint16_t f;
    /* q 轴注入产生交变转矩，转子随之微幅摆动，摆动反电势使 q 轴阻抗多出一项 Kmot / (jω)（容性，表观电感减小 Kmot/ω^2，
     * 见 acKmot）。25 Hz 常见约数%偏小，需加回；d 轴无转矩，无需修正。
     * 摆动与电流成正比，大幅值段同样修正 */
    /* 摆动系数实测：同一幅值下 0.5*f0 / f0 两段，Im(Z)/ω = L - K/ω^2，假设 L 在 f0 以下不随频率变化，解出 K。
     * 理论上 K = Kmot（转子自由摆动）；转子被静摩擦 / 齿槽卡住时 K -> 0，此时按理论值修正会过补偿；
     * 高基频（f0 高、1/w^2 小）下噪声可能被放大，故对非零理论值作 4*Kmot 上限保护。
     * 0.4 A 各段使用 0.4 A 测得的 K，大幅值段使用 1.0 A 测得的 K（幅值越大转子越自由） */
    const float w12 = 2.0f * (float)MATH_PI * s_acHz[3], w25 = 2.0f * (float)MATH_PI * s_acHz[0];
    const float dk = 1.0f / (w12 * w12) - 1.0f / (w25 * w25);
    float k04 = (s_acZim[AC_S_Q04] / w25 - s_acZim[AC_S_Q04L] / w12) / dk;
    float k10 = (s_acZim[AC_S_Q10] / w25 - s_acZim[AC_S_Q10L] / w12) / dk;
    {
        float km = acKmot();
        float l0 = (w25 > 0.0f) ? s_acZim[AC_S_Q04] / w25 : 0.0f;
        gIdent.kSwingModel = km;
        if (km > 0.0f && l0 > 0.0f && 10.0f * km / (w25 * w25 * l0) < 0.01f) {
            k04 = 0.0f;
            k10 = 0.0f;
        } else {
            if (k04 < 0) k04 = 0;
            if (k10 < 0) k10 = 0;
            if (gIdent.lqAtA[0] > 0.0f && fabsf(s_acI04L - gIdent.lqAtA[0]) > 0.15f * gIdent.lqAtA[0]) {
                k04 = 0.0f;
                gIdent.warn |= IDENT_WARN_SWING_CLAMP;
            }
            if (gIdent.lqAtA[1] > 0.0f && fabsf(s_acI10L - gIdent.lqAtA[1]) > 0.15f * gIdent.lqAtA[1]) {
                k10 = k04;
                gIdent.warn |= IDENT_WARN_SWING_CLAMP;
            }
            if (km > 0.0f) {
                if (k04 > 4.0f * km) { k04 = 4.0f * km; gIdent.warn |= IDENT_WARN_SWING_CLAMP; }
                if (k10 > 4.0f * km) { k10 = 4.0f * km; gIdent.warn |= IDENT_WARN_SWING_CLAMP; }
            }
        }
    }
    gIdent.kSwing[0] = k04;
    gIdent.kSwing[1] = k10;
    for (f = 0; f < AC_NF; f++) {
        float w = 2.0f * (float)MATH_PI * s_acHz[f];
        gIdent.acLdH[f] = s_acZim[f] / w;
        gIdent.acLqH[f] = s_acZim[AC_NF + f] / w + k04 / (w * w);
        gIdent.acRohm[f] = s_acZre[f];
    }
    for (f = 0; f < 5; f++) {   /* q 轴 f0 五档幅值：[0] = 0.4 A 档，[1..4] = 大幅值档 */
        uint16_t s = (uint16_t)(f == 0 ? AC_S_Q04 : AC_S_Q10 + f - 1);
        gIdent.lqAtH[f] = s_acZim[s] / w25 + (f == 0 ? k04 : k10) / (w25 * w25);
        if (f > 0) {    /* 偏置 1 倍 / 2 倍外推到 0 偏置 */
            float l2 = s_acZim[AC_S_Q10B2 + f - 1] / w25 + k10 / (w25 * w25);
            gIdent.lqAtHb1[f - 1] = gIdent.lqAtH[f];
            gIdent.lqAtHb2[f - 1] = l2;
            gIdent.lqAtH[f] = 2.0f * gIdent.lqAtH[f] - l2;
        }
    }
    if (!s_l0Only) {
        for (f = 0; f < 4u; f++) {
            float lo = gIdent.lqAtA[f], hi = gIdent.lqAtA[f + 1u];
            float need = (s_capMask & (1u << (f + 1u))) ? 0.97f : 1.15f;   /* 封顶档只要求不倒退 */
            if (!(lo > 0.0f && hi >= need * lo)) {
                gIdent.warn |= IDENT_WARN_AMP_ORDER;
                break;
            }
        }
        if (!(gIdent.lqAtA[4] >= 1.5f * gIdent.lqAtA[1])) {                /* 封顶后总跨度仍不足 */
            gIdent.warn |= IDENT_WARN_AMP_ORDER;
        }
    }
    /* 观测器需要的是转子坐标系中的低频电感（稳态运行时转子所见磁场恒定，转子内无涡流）。
     * 注入频率在满足信噪比与转折频率的前提下越低越好，故取基频 f0（s_acHz[0]）。 */
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
    if (gIdent.acLdH[0] <= 0.0f || gIdent.acLdH[1] <= 0.0f || gIdent.acLdH[2] <= 0.0f ||
        gIdent.acRohm[0] <= 0.0f || gIdent.acLqH[0] <= 0.0f ||
        gIdent.acLdH[2] < 0.65f * gIdent.acLdH[0] || gIdent.acLdH[2] > 1.35f * gIdent.acLdH[0] ||
        fabsf(gIdent.acLdMagH[0] - gIdent.acLdH[0]) > 0.25f * gIdent.acLdH[0]) {
        gIdent.warn |= IDENT_WARN_AC_DRIFT;
    }
}

/* ---- R/L（IDENT_PH_RL = 9，静止测量第 1 步）--------------------------------------------------------
 * 新电机没有可信的 L、R 初值，无法计算电流环增益。因此第一步不使用电流环：沿 alpha 轴直接施加 RL_HZ 正弦电压，
 * 幅值从 1 V 起按测得电流逐周期调整到 RL_I_A，稳定 2 个周期后积分 RL_AVG 个周期，Z = V / I（基波）。
 * 电流与"采样时刻起作用的电压相位"做相关（电压延迟 AC_V_DELAY 拍生效，同交流阻抗）。
 * 转子位置任意：测得的 L 介于 Ld、Lq 之间，且频率较高时比运行时偏小（涡流），R 中含死区电压（偏大）——
 * 仅用于计算电流环增益（误差几十 % 只影响带宽，不影响稳定），死区阶梯测得 Rs 后立即替换，交流阻抗测得 Lq 后最终替换。
 *
 * [8 KB SRAM 复用说明] R/L（IDENT_PH_RL）与交流阻抗（IDENT_PH_AC）在时间上严格互斥，故复用上方 s_ac* 静态变量：
 *   s_acAmp  = 当前正弦电压幅值（Q15）       s_acSeg   = R/L 子阶段（0 调幅，1..2 稳定，3 积分）
 *   s_acEs   = 已积分周期计数（0..RL_AVG）   s_acVs/Vc = 多周期基波电流正交累加和 */
#define RL_HZ       200.0
#define RL_PER      ((uint16_t)(PWM_FREQ / RL_HZ + 0.5))
#define RL_AVG      16
#define RL_I_Q15    (0.5 * MOTOR_I_RATED_A / I_MAX * 32768.0)
#define RL_V0_Q15   Q15(1.0 / UDC_MAX)
#define RL_VMAX_Q15 Q15(0.3 * UDC_RATE / 1.7320508 / UDC_MAX)

static void rlStart(void)       /* 慢环 */
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

/* R/L 快环：alpha 轴输出正弦电压（不使用电流环），对 alpha 电流按"电压生效时刻"的相位做正交积分，每个周期置一次完成 */
static void rlFast(void)
{
    SinCos_TypeDef a = FocMath_SinCos((int16_t)(s_acPhi >> 16));
    SinCos_TypeDef b = FocMath_SinCos((int16_t)((s_acPhi - s_acDly) >> 16));     /* 采样时刻正在起作用的电压的相位 */
    if (!s_acDone) {
        int32_t i = gMeas.iab.s16Alpha;
        s_acIs += (i * b.s16Sin) >> 15;
        s_acIc += (i * b.s16Cos) >> 15;
        if (++s_acT >= RL_PER) s_acDone = 1;
    }
    s_acPhi += s_acInc;
    FocCore_ApplyVoltageAB((int16_t)(((int32_t)s_acAmp * a.s16Sin) >> 15), 0);
}

/* 慢环：每个周期调整一次幅值；返回 1 = 完成（成功或放弃） */
static uint8_t rlSlow(void)
{
    float is, ic, i1, v;
    if (!s_acDone) return 0;
    is = (float)s_acIs;
    ic = (float)s_acIc;
    s_acIs = s_acIc = 0;
    s_acT = 0;
    s_acDone = 0;
    i1 = 2.0f * sqrtf(is * is + ic * ic) / (float)RL_PER;      /* 基波幅值（Q15 电流） */
    v = (float)s_acAmp;
    if (s_acSeg == 0) {             /* 调整幅值（加半步长阻尼与上限保护，防低电感死区拐点附近振荡不收敛） */
        float k = (i1 > 1.0f) ? (float)RL_I_Q15 / i1 : 2.0f;
        int16_t nv;
        if (k > 0.85f && k < 1.15f) { s_acSeg = 1; s_acEs = 0; return 0; }
        if (k > 2.0f) k = 2.0f;
        if (k < 0.5f) k = 0.5f;
        if (v >= (float)RL_VMAX_Q15 && k > 1.0f) { gIdent.rlR = -1; gIdent.warn |= IDENT_WARN_RL_TIMEOUT; return 1; }   /* 电压已达上限电流仍不足：放弃，保留原参数 */
        if (++s_acEs >= 200) {
            gIdent.warn |= IDENT_WARN_RL_TIMEOUT;
            if (i1 > 0.25f * (float)RL_I_Q15) { s_acSeg = 1; s_acEs = 0; return 0; }
            gIdent.rlR = -1;
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
    if (s_acSeg < 3) { s_acSeg++; return 0; }    /* 稳定 2 个周期 */
    s_acVs += (int32_t)is;
    s_acVc += (int32_t)ic;
    if (++s_acEs < RL_AVG) return 0;
    {
        float ss = (float)s_acVs, cc = (float)s_acVc, n = (float)(RL_AVG * RL_PER), d = ss * ss + cc * cc;
        float r, x;
        if (d <= 0) { gIdent.rlR = -1; gIdent.warn |= IDENT_WARN_RL_TIMEOUT; return 1; }
        r = v * ss * n / (2.0f * d) * (float)(UDC_MAX / I_MAX);
        x = -v * cc * n / (2.0f * d) * (float)(UDC_MAX / I_MAX);
        gIdent.rlR = r;
        gIdent.rlL = x / (float)(2.0 * MATH_PI * RL_HZ);
        if (r > 0 && gIdent.rlL > 0) {   /* 电流环、IPD 注入电压立即按此计算 */
            MotorParam_TypeDef p = gParam;
            p.rsOhm = r;
            p.lqH = gIdent.rlL;
            p.ldH = gIdent.rlL;
            (void)Param_ApplyNow(&p);
        }
    }
    return 1;
}

/* ---- q 轴饱和拟合（phase 7）--------------------------------------------------------------------------
 * 观测器模型：割线电感 L(i) = L0 * m(i)，m = max(1 - k * max(|i| - Ik, 0), MOTOR_LS_SAT_MIN)。
 * 正弦电流 A sinφ 下，磁链 L(|i|) i 的基波 / A（交流阻抗测到的"描述函数"电感）= L0 * r(A)，
 * r(A) = (4/π) ∫_0^{π/2} m(A sinφ) sin^2 φ dφ（16 点中点积分，m = 1 时正好 = 1）。
 * 对每个 (Ik, k)，L0 取最小二乘最优（Σ L r / Σ r^2），误差 = Σ (L - L0 r)^2 / L0^2。
 * 网格：Ik 0~2.4 A 步长 0.05、k 0~0.6/A 步长 0.005；每个慢环周期处理一个 Ik（49 拍完成，每拍约 1 万次乘加，避免长时间占用慢环）。 */
#define DF_NP   16
static uint16_t s_dfI;
static float s_dfBest, s_dfK, s_dfIk, s_dfL0;

/* 开始 L(i) 网格拟合（初值：L0 = 0.4 A 时的电感，不饱和） */
static void dfStart(void)
{
    s_dfI = 0;
    s_dfBest = 1e30f;
    s_dfK = 0;
    s_dfIk = 0;
    s_dfL0 = gIdent.lqAtH[0];
}

/* sin((j + 0.5) * π/2 / 16) 查表：避免每个慢环周期近 1 万次调用 sinf */
static const float s_dfSin[DF_NP] = {
    0.0490677f, 0.1467305f, 0.2429802f, 0.3368899f, 0.4275551f, 0.5141027f, 0.5956993f, 0.6715590f,
    0.7409511f, 0.8032075f, 0.8577286f, 0.9039893f, 0.9415441f, 0.9700313f, 0.9891765f, 0.9987955f
};

static uint8_t dfStep(void)     /* 1 = 完成 */
{
    float sc = (float)IDENT_I_SCALE;
    float ik = 0.05f * sc * (float)s_dfI, k, kStep = 0.005f / sc, kMax = 0.6001f / sc;
    uint16_t i, j;
    if (s_l0Only) {
        gIdent.lsSatKneeA = 0.0f;
        gIdent.lsSatPerA = 0.0f;
        gIdent.lqH = gIdent.acLqH[0];
        return 1;
    }
    for (k = 0; k <= kMax; k += kStep) {
        float slr = 0, srr = 0, sll = 0, l0, e;
        for (i = 0; i < 5; i++) {
            float a = gIdent.lqAtA[i], r = 0;
            for (j = 0; j < DF_NP; j++) {
                float s = s_dfSin[j], x = a * s - ik, m;
                m = (x > 0) ? 1.0f - k * x : 1.0f;
                if (m < (float)MOTOR_LS_SAT_MIN) m = (float)MOTOR_LS_SAT_MIN;
                r += m * s * s;
            }
            r *= 2.0f / (float)DF_NP;      /* (4/π) * (π/2)/NP */
            slr += gIdent.lqAtH[i] * r;
            srr += r * r;
            sll += gIdent.lqAtH[i] * gIdent.lqAtH[i];
        }
        if (srr <= 0) continue;
        l0 = slr / srr;
        e = (sll - l0 * slr) / (l0 * l0);
        if (e < s_dfBest) {
            s_dfBest = e;
            s_dfK = k;
            s_dfIk = ik;
            s_dfL0 = l0;
        }
    }
    if (++s_dfI <= 48) return 0;
    if (s_dfK * (gIdent.lqAtA[4] - s_dfIk) < 0.03f) {    /* 最大幅值处下降仍不足 3%：判为无明显饱和 */
        s_dfK = 0;
        s_dfIk = 0;
        s_dfL0 = gIdent.lqAtH[0];
    }
    gIdent.lsSatKneeA = s_dfIk;
    gIdent.lsSatPerA = s_dfK;
    gIdent.lqH = s_dfL0;
    return 1;
}

/* 静止测量结果提交给 param.c（自动生效）：Rs、Lq（SPM 时 Ld = Lq）、L(i)、死区。回到 Stop 后生效。
 * commitL = 0 时仅提交直流阶梯测得的 Rs 与死区（mask 1|32），保留原有 L / L(i)。
 * 每项先经 param.c 的合理性检查（物理上为正、定点范围可容纳，见 checkParam），不合理的保留原值并在 gParamStat.rejectMask 中标记 */
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
    if (commitL && !s_l0Only && gIdent.lqH > 0) {
        float maxA = 0.0f;
        uint16_t i;
        for (i = 0; i < 5u; i++) {
            if (gIdent.lqAtA[i] > maxA) maxA = gIdent.lqAtA[i];
        }
        p.lsKneeA = gIdent.lsSatKneeA;
        p.lsPerA = gIdent.lsSatPerA;
        p.lsMaxA = maxA;
        m |= 4u;
    }
    if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) { p.dtEffS = gIdent.deadTimeS; m |= 32u; }
    Param_Offer(&p, m);
}

/* 慢环（1 kHz），在 Align 子状态中替代定位逻辑。
 * 阶段转移顺序：IDENT_PH_RL(9) -> 定位(0) -> IDENT_PH_DT_STEP(1) -> IDENT_PH_DT_ZERO(2)
 *              -> IDENT_PH_IPD(3) -> IDENT_PH_AC(6) -> IDENT_PH_SAT_FIT(7) -> IDENT_PH_DONE(4) */
void Ident_Slow(void)
{
    IdentTest_TypeDef *t = &gIdentTest;
    switch (t->phase) {
    case IDENT_PH_RL:       /* R/L（快环输出电压）：完成后回到定位；若放弃（rlR/rlL <= 0）直接终止以免盲飞 */
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
    case IDENT_PH_DT_STEP:  /* 死区阶梯（复用 s_acVs/Vc/Is/Ic 统计前后半窗 vq/iq 漂移，判转子静止） */
        FocCore_SetIdqRef(s_dtLevel[t->idx], 0);
        if (t->cnt == 0) { s_acVs = s_acVc = s_acIs = s_acIc = 0; }
        if (++t->cnt > DT_SETTLE_MS) {
            t->sum[0] += gFoc.idq.s16D;
            t->sum[1] += gFoc.vdq.s16D;
            if (t->cnt <= DT_SETTLE_MS + DT_AVG_MS / 2) {
                s_acVs += gFoc.vdq.s16Q;
                s_acIs += gFoc.idq.s16Q;
            } else {
                s_acVc += gFoc.vdq.s16Q;
                s_acIc += gFoc.idq.s16Q;
            }
        }
        if (t->cnt >= DT_SETTLE_MS + DT_AVG_MS) {
            float vdAbs = fabsf((float)t->sum[1]) * (float)(UDC_MAX / 32768.0 / DT_AVG_MS);
            float dvq = fabsf((float)(s_acVc - s_acVs)) * (float)(UDC_MAX / 32768.0 / (DT_AVG_MS / 2));
            float diqR = fabsf((float)(s_acIc - s_acIs)) * (float)(I_MAX / 32768.0 / (DT_AVG_MS / 2)) * gIdent.rlR;
            uint8_t unstable = (uint8_t)(vdAbs > 0.05f && (dvq > DT_DRIFT_VQ_MAX * vdAbs || diqR > DT_DRIFT_IQ_MAX * vdAbs));
            int16_t idAvg = (int16_t)(t->sum[0] / DT_AVG_MS);
            int16_t vdAvg = (int16_t)(t->sum[1] / DT_AVG_MS);
            t->sum[0] = t->sum[1] = 0;
            s_acVs = s_acVc = s_acIs = s_acIc = 0;
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
                {   /* Rs、死区立即生效（电流环积分增益、观测器），供后续 IPD / 交流阻抗使用 */
                    MotorParam_TypeDef p = gParam;
                    if (gIdent.rsOhm > 0) p.rsOhm = gIdent.rsOhm;
                    if (INVERTER_DT_COMP && gIdent.deadTimeS > 0) p.dtEffS = gIdent.deadTimeS;
                    (void)Param_ApplyNow(&p);
                }
            }
        }
        break;
    case IDENT_PH_DT_ZERO:  /* 电流回零，等待 ZERO_WAIT_MS 后交由快环执行 IPD */
        FocCore_SetIdqRef(0, 0);
        if (++t->cnt >= ZERO_WAIT_MS) {
            t->cnt = 0;
            Ipd_Start();
            t->phase = IDENT_PH_IPD;
        }
        break;
    case IDENT_PH_IPD:      /* IPD 检测中（ipd.c），完成后复制结果 */
        Ipd_Slow();
        if (gIpd.state == IPD_STATE_DONE) {
            (void)Ipd_Evaluate();       /* 给出 gIpd.theta（交流阻抗的 d / q 方向）/ ratioQ8 */
            ipdFit();
            FocCore_PresetPI(0, 0);     /* IPD 结束，积分器从 0 开始 */
            acInitFreqs();
            acSegStart(0);
            t->cnt = 0;
            t->phase = IDENT_PH_AC;
        }
        break;
    case IDENT_PH_AC:       /* 快环执行交流阻抗测量中 */
        if (s_acDone) {
            if (acSegFinish(s_acSeg)) acSegStart(s_acSeg);
            else if (s_acSeg + 1 < AC_NSEG) acSegStart((uint16_t)(s_acSeg + 1));
            else {
                s_acSeg = AC_NSEG;
                acFit();
                FocCore_LoadGains();    /* 恢复标称带宽电流环增益，供后续流程使用 */
                dfStart();
                t->cnt = 0;
                t->phase = IDENT_PH_SAT_FIT;
            }
        }
        break;
    case IDENT_PH_SAT_FIT:  /* 饱和拟合（分摊到多个慢环周期），快环保持零电流 */
        if (dfStep()) {
            uint8_t trustedPre = (uint8_t)(gIdent.rsOhm > 0.0f && gIdent.lqH > 0.0f &&
                                           (!MOTOR_IPM || gIdent.ldH > 0.0f) &&
                                           gIdent.acLdH[0] > 0.0f && gIdent.acRohm[0] > 0.0f &&
                                           gIdent.rlR > 0.0f && gIdent.rlL > 0.0f &&
                                           !(gIdent.warn & (IDENT_WARN_AC_DRIFT | IDENT_WARN_LOW_SNR | IDENT_WARN_AC_FAIL |
                                                            IDENT_WARN_AMP_FAIL | IDENT_WARN_AMP_ORDER | IDENT_WARN_DT_UNSTABLE)));
            offerStill(trustedPre);
            if (!trustedPre) gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
            if (gIdentCfg.mode == IDENT_MODE_AUTO) {
                uint8_t ok = (uint8_t)(trustedPre && !(gParamStat.rejectMask & 39u) && gParamStat.pending);
                if (ok) {
                    s_autoStage = 1;            /* 停机（参数在 Stop 中装入）后重新起动，执行运行测量；state 保持 RUNNING */
                    s_restartReq = 1;
                } else {
                    gIdent.warn |= IDENT_WARN_AUTO_BLOCKED;
                    gIdent.state = IDENT_DONE;
                }
            } else {
                gIdent.state = IDENT_DONE;
            }
            t->cnt = 0;
            t->phase = IDENT_PH_DONE;
        }
        break;
    case IDENT_PH_DONE:     /* 完成（结果在 gIdent，已提交给 param.c） */
        break;
    default:
        break;
    }
}

/* ===========================================================================
 * 运行辨识：正常起动，闭环稳速后进行一次零电流滑行
 *   滑行前 RUN_PRE_MS：平均 iq（= 负载转矩 / Kt）与转速；
 *   滑行（复用零电流跟踪 Spin_TrackBegin，观测器 / PLL 照常跟踪转子，结束后按跟踪转速接回速度环）：
 *     psi_f = |v| / ω_e：零电流时电压即反电势，|v| 与坐标系角度无关（不受观测器角度误差影响），
 *     不依赖 Rs、L、死区（电流为 0，三者均不起作用）；
 *     dn/dt：转速对时间做二次拟合，取滑行起始时刻的斜率（该时刻负载转矩 = 滑行前的电磁转矩；风机类负载随转速变化，
 *     不能用窗口平均斜率）；accRpmSPerA = -(dn/dt) / iq（= Kt / J，即 cfg_motor.h MOTOR_ACC_RPM_S_PER_A 的定义）。
 *   iq 使用估计坐标系的值：角度误差 δ 只使其偏大 1/cos δ（10° 时仅 1.5%）。
 *   起始转速须 >= RUN_MIN_RPM：滑行 RUN_COAST_MS 期间转速须始终高于 TRACK_END_RPM（低于该值零电流跟踪会关闭 PWM）。
 *   同一转速指令只执行一次；修改指令并稳速后再执行一次（结果覆盖，runRpm / runIqA 记录工况）。
 * =========================================================================*/
#define RUN_STABLE_MS   2000
#define RUN_PRE_MS      500
#define RUN_SKIP_MS     20
#define RUN_COAST_MS    400     /* 滑行时长：越长加速度系数受 PLL 转速噪声影响越小 */
/* 运行辨识的最低起始转速：滑行 RUN_COAST_MS 的掉速须保持在 TRACK_END_RPM / 回退转速以上，本电机取 315 rpm */
#define RUN_MIN_BASE_RPM    (1.25 * (OPENLOOP_FALLBACK_RPM > TRACK_END_RPM ? OPENLOOP_FALLBACK_RPM : TRACK_END_RPM))
#define RUN_MIN_RPM         (RUN_MIN_BASE_RPM < 0.7 * MOTOR_SPEED_RATED_RPM ? RUN_MIN_BASE_RPM : 0.7 * MOTOR_SPEED_RATED_RPM)
/* 运行辨识闭环滑行子状态（s_rnSt） */
enum {
    RN_WAIT_STABLE = 0,             /* 等待闭环稳速 RUN_STABLE_MS */
    RN_PRE_AVG     = 1,             /* 滑行前平均 iq / id / 转速 RUN_PRE_MS */
    RN_COAST       = 2,             /* 零电流滑行跟踪中 RUN_COAST_MS */
    RN_DONE        = 3              /* 本次转速指令的滑行测量已完成 */
};
static uint8_t  s_rnSt;             /* 取值见 RN_* */
static uint16_t s_rnT;
static int16_t  s_rnCmd;
/* [8 KB SRAM 复用说明] 开环磁链（Ident_OpenLoopFluxSlow，在 OpenLoop 阶段）与闭环滑行（Ident_RunSlow，在 Spin 阶段）
 * 在时间上严格互斥，故共用下方 s_rnId / s_rnIq / s_rnVd / s_rnVq 累加器 */
static int32_t  s_rnIq, s_rnId, s_rnW;
static float    s_rnVd, s_rnVq, s_rnS[7];  /* 二次拟合：n, Σx, Σx², Σx³, Σx⁴, Σy, Σxy；Σx²y 单独 */
static float    s_rnX2y, s_rnWs;

/* 自动方式运行测量期间的转速指令：存在起动指令但低于 RUN_MIN_RPM 的 1.1 倍时，按 1.1 倍（同方向）运行，测量完成后回到原指令。
 * 留 10% 余量：稳速判断使用 PLL 转速（含噪声），指令恰为 RUN_MIN_RPM 时会在门限附近反复跳变而无法开始测量。
 * 停机指令（|cmd| 不高于起动门限）原样返回 */
int16_t Ident_SpeedCmd(int16_t cmd)
{
    const int16_t lo = Q15(1.1 * RUN_MIN_RPM / SPEED_MAX);
    if (gIdentCfg.mode != IDENT_MODE_AUTO || !Ident_RunActive()) return cmd;
    if (cmd > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && cmd < lo) return lo;
    if (cmd < -Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && cmd > -lo) return (int16_t)-lo;
    return cmd;
}

/* 运行测量滑行阶段：要求零电流跟踪不提前结束（由 spin.c 查询） */
uint8_t Ident_HoldTrack(void)
{
    return (uint8_t)(s_rnSt == RN_COAST);
}

/* 运行测量（Run 慢环每拍，Ident_RunActive 时）：闭环稳速 RUN_STABLE_MS -> 平均滑行前 iq、转速 RUN_PRE_MS
 * -> 进入零电流滑行，记录电压与转速曲线 RUN_COAST_MS -> 计算 psi_f = |v| / w_e、加速度系数 = -(dn/dt) / iq，提交给 param.c */
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
        if (!tracking) {    /* 跟踪因其他原因结束（停机 / 故障） */
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
            /* 正规方程 [n Sx Sx2; Sx Sx2 Sx3; Sx2 Sx3 Sx4][a b c] = [Sy Sxy Sx2y]，取 b（x = 0 = 滑行开始） */
            float n = s_rnS[0], a1 = s_rnS[1], a2 = s_rnS[2], a3 = s_rnS[3], a4 = s_rnS[4];
            float y0 = s_rnS[5], y1 = s_rnS[6], y2 = s_rnX2y;
            float det = n * (a2 * a4 - a3 * a3) - a1 * (a1 * a4 - a3 * a2) + a2 * (a1 * a3 - a2 * a2);
            float db = n * (y1 * a4 - a3 * y2) - y0 * (a1 * a4 - a3 * a2) + a2 * (a1 * y2 - y1 * a2);
            float ks = (float)(SPEED_MAX / 32768.0);                         /* Q15 -> rpm */
            float wm = s_rnWs / n * ks, we = wm * (float)(2.0 * MATH_PI / 60.0 * MOTOR_POLE_PAIRS);
            float iq = (float)s_rnIq / RUN_PRE_MS * (float)(I_MAX / 32768.0);
            float vm = sqrtf(s_rnVd * s_rnVd + s_rnVq * s_rnVq) / n * (float)(UDC_MAX / 32768.0);
            if (we < 0) we = -we;
            if (we > 1.0f) gIdent.psiWb = vm / we;
            if (det != 0 && (iq > 0.02f || iq < -0.02f)) gIdent.accRpmSPerA = -(db / det) * ks / iq;
            gIdent.runRpm = (float)s_rnW / RUN_PRE_MS * ks;
            gIdent.runIqA = iq;
            gIdent.runIdA = (float)s_rnId / RUN_PRE_MS * (float)(I_MAX / 32768.0);
            {   /* 立即生效，电机按新参数继续运行。整套参数不合理时改用 Param_Offer：逐项检查，仅替换合理的项（停机后生效），
                 * 被拒绝的项记录在 gParamStat.rejectMask */
                MotorParam_TypeDef p = gParam;
                uint16_t m = 0;
                if (gIdent.psiWb > 0) { p.psiWb = gIdent.psiWb; m |= 8u; }
                if (gIdent.accRpmSPerA > 0) { p.accRpmSPerA = gIdent.accRpmSPerA; m |= 16u; }
                if (!Param_ApplyNow(&p)) Param_Offer(&p, m);
            }
            gIdent.state = IDENT_DONE;
            s_rnSt = RN_DONE;               /* 释放跟踪：Spin_TrackSlow 在下一块结束时接回速度环 */
        }
        break;
    default:
        break;
    }
}

/* ===========================================================================
 * 运行辨识之一：I/F 开环中测量磁链
 * 开环拖到目标转速（指令限制在 OPENLOOP_HANDOVER_RPM ~ OPENLOOP_FALLBACK_RPM 之间）后暂不切闭环：稳定 300 ms、平均 500 ms，
 *   e = v - R i - Vdt(i) - jωL(|i|) i，psi_f = |e| / |ω|
 * 全部为 dq 平面中的模长 / 叉乘，与开环角是否对准转子无关（I/F 电流角滞后转子任意角度均可），ω 为开环电角速度（同步）。
 * 要求 R、L、死区准确：在静止测量之后进行（gParam 已为测得值）。死区按 cfg_board.h 的模型（clamp(i/I0)）取基波：
 *   Vdt = Vph * (1/8) Σ clamp(|i| sinθ_j / I0) sinθ_j（16 点，|i| >> I0 时 = 4/π Vph）。
 * 测得后立即生效（观测器幅值、PLL 增益），再按一致性判据切闭环。之后的零电流滑行还会再测一次 psi_f（不依赖 R、L）。
 * 变量与零电流滑行共用（两段不同时进行）。 */
#define OLFLUX_SETTLE_MS   300
#define OLFLUX_AVG_MS      500

/* 运行测量的开环磁链（I/F 慢环每拍）：开环匀速后等待 OLFLUX_SETTLE_MS，再平均 OLFLUX_AVG_MS 的 dq 电压、电流，
 * 扣除 Rs*i、w*L*i 和死区电压得到反电势，psi = |e| / w。返回 1 = 测量中（I/F 暂不切闭环），0 = 已完成 / 不测量 */
uint8_t Ident_OpenLoopFluxSlow(int32_t ol, int32_t tgt)
{
    if (!Ident_RunActive() || s_olFluxSt == OLF_DONE) return 0;
    if (ol != tgt) {    /* 仍在加速：暂不切闭环 */
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

/* 快环：IDENT_PH_IPD(3)、IDENT_PH_DONE(4) 由 IPD 模块输出电压（完成后持续输出零电压）；
 *       IDENT_PH_AC(6)、IDENT_PH_SAT_FIT(7) 由 acFast 输出（SAT_FIT 期间保持零电流）；
 *       IDENT_PH_RL(9) 由 rlFast 输出；其余阶段（0 定位 / 1 死区阶梯 / 2 回零）返回 0，照常运行电流环 */
uint8_t Ident_Fast(void)
{
    if (gIdentTest.phase < IDENT_PH_IPD) return 0;
    if (gIdentTest.phase == IDENT_PH_AC || gIdentTest.phase == IDENT_PH_SAT_FIT) acFast();
    else if (gIdentTest.phase == IDENT_PH_RL) rlFast();
    else Ipd_Fast();
    return 1;
}
