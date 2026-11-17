/* ============================================================================
 * @file    ipd.c
 * @brief   初始位置检测（静音：高频注入 + 直流偏置），原理见 ipd.h
 * ==========================================================================*/
#include "ipd.h"
#include "cfg_derived.h"
#include "foc_core.h"
#include "meas.h"
#include "foc_math.h"
#include "qmath.h"
#include "param.h"

Ipd_TypeDef gIpd;

static int16_t s_sign;              /* 本拍注入电压的符号（+1 / -1），每拍翻转 */
static int16_t s_ia, s_ib;          /* 上一拍的 αβ 电流 */
static uint16_t s_t;                /* 快环：当前注入方向内的拍数 */
static uint8_t s_dir;               /* 第 1 步：0 = α 注入，1 = β 注入 */
static int32_t s_acc[4];            /* 第 1 步：α 注入的 α、β 响应，β 注入的 α、β 响应（Σ 符号 * 电流增量，Q15 电流） */
static volatile uint8_t s_meas;     /* 第 2 步：1 = 快环累计 d 轴响应（慢环开关） */
static int32_t s_accD;              /* 第 2 步：本测量窗口的 d 轴响应 */
static int32_t s_resP, s_resN;      /* 第 2 步：+偏置、-偏置时的 d 轴响应 */
static uint8_t s_flip;              /* 第 1 步得出的整体符号（采样时序决定），第 2 步沿用 */
static int16_t s_axTheta;           /* 第 2 步：测得的轴向（Q15 角度） */
static SinCos_TypeDef s_ax;
static uint8_t s_bStep;             /* 第 2 步：0 升到 +偏置，1 测，2 降到 -偏置，3 测，4 回 0 */
static uint16_t s_bT;               /* 第 2 步：本段已走的慢环拍数 */
static int32_t s_bId;               /* 第 2 步：d 轴电流指令（Q15） */

/* 开始一轮 IPD（进入 Align 时调用）：从 α 方向注入开始 */
void Ipd_Start(void)
{
    s_acc[0] = s_acc[1] = s_acc[2] = s_acc[3] = 0;
    s_dir = 0;
    s_t = 0;
    s_sign = 1;
    s_meas = 0;
    s_ia = gMeas.iab.s16Alpha;
    s_ib = gMeas.iab.s16Beta;
    gIpd.ok = 0;
    gIpd.runs++;
    gIpd.state = IPD_STATE_HF;
}

/* IPD 快环。先按上一拍注入的符号累计本拍电流增量（采样滞后电压一拍或零拍只改变整体符号，由慢环按"同向响应为正"统一），
 * 再给出本拍电压：第 1 步只输出注入电压（αβ，不运行电流环）；第 2 步电流环按轴向维持偏置电流，注入电压叠加在 d 轴上 */
void Ipd_Fast(void)
{
    int32_t ia = gMeas.iab.s16Alpha, ib = gMeas.iab.s16Beta;
    int32_t da = ia - s_ia, db = ib - s_ib;
    int32_t v = gGain.ipdHfVQ15;
    s_ia = (int16_t)ia;
    s_ib = (int16_t)ib;
    if (gIpd.state == IPD_STATE_HF) {
        if (s_t >= IPD_SETTLE_TICKS) {      /* 每个方向开头几拍不计（含换方向时上一方向的最后一拍） */
            s_acc[s_dir * 2]     += s_sign * da;
            s_acc[s_dir * 2 + 1] += s_sign * db;
        }
        if (++s_t >= IPD_HF_TICKS + IPD_SETTLE_TICKS) {
            s_t = 0;
            if (++s_dir >= 2) {
                gIpd.state = IPD_STATE_AXIS;
                FocCore_ApplyVoltageAB(0, 0);
                return;
            }
        }
        /* 每个方向的第一拍用半幅值：电流三角波以 0 为中心，不带直流 */
        if (s_t == 0) { s_sign = 1; v >>= 1; } else s_sign = (int16_t)-s_sign;
        v *= s_sign;
        FocCore_ApplyVoltageAB((int16_t)(s_dir ? 0 : v), (int16_t)(s_dir ? v : 0));
    } else if (gIpd.state == IPD_STATE_BIAS) {
        if (s_meas) s_accD += s_sign * ((da * s_ax.s16Cos + db * s_ax.s16Sin) >> 15);
        gFoc.theta = s_axTheta;
        gFoc.thetaInv = s_axTheta;
        FocCore_CurrentLoop();
        s_sign = (int16_t)-s_sign;
        v *= s_sign;
        FocCore_ApplyVoltageAB((int16_t)(gFoc.vab.s16Alpha + ((v * s_ax.s16Cos) >> 15)),
                               (int16_t)(gFoc.vab.s16Beta + ((v * s_ax.s16Sin) >> 15)));
    } else {
        FocCore_ApplyVoltageAB(0, 0);
    }
}

/* 第 1 步结束：由四个响应算 d 轴方向（两个候选 θ、θ + 180°）和凸极比；不足时判失败 */
static void axisFit(void)
{
    int32_t aa = s_acc[0], ab = s_acc[1], ba = s_acc[2], bb = s_acc[3], x, y, s, ax, ay, mag;
    s_flip = (uint8_t)(aa + bb < 0);
    if (s_flip) {    /* 采样时序只影响整体符号：同向响应应为正 */
        aa = -aa;
        ab = -ab;
        ba = -ba;
        bb = -bb;
    }
    /* 2D*cos2θ、2D*sin2θ、2S（各值 < 2^20） */
    x = aa - bb;
    y = ab + ba;
    s = aa + bb;
    /* |2D| 用 max + 3/8 min 近似（误差 < 7%，只用于判门限），不开方 */
    ax = x < 0 ? -x : x;
    ay = y < 0 ? -y : y;
    mag = ax > ay ? ax + ((ay * 3) >> 3) : ay + ((ax * 3) >> 3);
    if (s <= 0 || (mag << 8) < s * IPD_MIN_SAL_Q8) {
        gIpd.ok = 0;
        gIpd.state = IPD_STATE_DONE;
        return;
    }
    s_axTheta = (int16_t)(QMath_Atan2(y, x) >> 17);   /* 2θ 的一半：Q31 角度 >> 16 = Q15 角度，再除以 2 */
    s_ax = FocMath_SinCos(s_axTheta);
    s_bStep = 0;
    s_bT = 0;
    s_bId = 0;
    s_resP = s_resN = 0;
    FocCore_PresetPI(0, 0);
    FocCore_SetIdqRef(0, 0);
    gIpd.state = IPD_STATE_BIAS;
}

/* 第 2 步（慢环每拍）：偏置电流斜坡和测量窗口；结束后判极性 */
static void biasStep(void)
{
    switch (s_bStep) {
    case 0:     /* 升到 +偏置 */
    case 2:     /* 降到 -偏置 */
    case 4:     /* 回到 0 */
    {
        int32_t tgt = (s_bStep == 0) ? IPD_BIAS_Q15 : (s_bStep == 2 ? -IPD_BIAS_Q15 : 0);
        if (s_bId < tgt) { s_bId += IPD_BIAS_STEP_Q15; if (s_bId > tgt) s_bId = tgt; }
        else if (s_bId > tgt) { s_bId -= IPD_BIAS_STEP_Q15; if (s_bId < tgt) s_bId = tgt; }
        FocCore_SetIdqRef((int16_t)s_bId, 0);
        if (s_bId == tgt && ++s_bT >= IPD_SETTLE_MS) {     /* 到达后再等电流环稳定 */
            s_bT = 0;
            if (s_bStep == 4) {
                int32_t mx = s_resP > s_resN ? s_resP : s_resN, mn = s_resP > s_resN ? s_resN : s_resP;
                gIpd.ratioQ8 = (uint16_t)(mn > 0 ? QMath_Clamp((mx << 8) / mn, 65535) : 0);
                gIpd.theta = (int16_t)(s_resP >= s_resN ? s_axTheta : s_axTheta + ANGLE_180_Q15);   /* 响应大（更饱和）的一侧是 N 极 */
                gIpd.ok = (uint8_t)(mn > 0 && gIpd.ratioQ8 >= IPD_MIN_RATIO_Q8);
                gIpd.state = IPD_STATE_DONE;
                return;
            }
            s_accD = 0;
            s_meas = 1;
            s_bStep++;
        }
        break;
    }
    case 1:     /* +偏置下测 */
    case 3:     /* -偏置下测 */
        if (++s_bT >= IPD_MEAS_MS) {
            s_meas = 0;
            if (s_flip) s_accD = -s_accD;
            if (s_bStep == 1) s_resP = s_accD; else s_resN = s_accD;
            s_bT = 0;
            s_bStep++;
        }
        break;
    default:
        break;
    }
}

/* IPD 慢环（检测进行中每拍调用） */
void Ipd_Slow(void)
{
    if (gIpd.state == IPD_STATE_AXIS) axisFit();
    else if (gIpd.state == IPD_STATE_BIAS) biasStep();
}

/* IPD 判定（慢环，state == DONE 后调用一次）：返回 1 = 可信（结果在 gIpd.theta）。电流已回到 0，快环输出零电压 */
uint8_t Ipd_Evaluate(void)
{
    if (!gIpd.ok) gIpd.fails++;
    FocCore_SetIdqRef(0, 0);
    gIpd.state = IPD_STATE_IDLE;
    return gIpd.ok;
}
