/* ============================================================================
 * @file    identify.c
 * @brief   参数辨识（IDENT_TEST = 1 时编译；正常运行时这些函数不被调用）。方法说明见 identify.h
 *
 * 自动方式（默认）：静止测量 -> 停机后重新起动 -> 运行测量，见 identify.h。
 * 静止测量（给指令后电机不旋转）：Ident_Reset 从 phase 9 开始，Align 子状态的快环 / 慢环交由本模块处理：
 *   phase 9  R/L 粗测：alpha 轴注入 200 Hz 电压，不使用电流环（rlStart / rlFast / rlSlow），测得后立即更新电流环增益
 *   定位（正常进行）-> phase 1/2 死区阶梯：d 轴电流 11 档，拟合 vd = Rs*id + 死区电压（dtFit）
 *   phase 3  IPD 对比度（ipdFit）
 *   phase 6  交流阻抗：d / q 轴多频率、多幅值注入（acSegStart / acFast / acSegFinish / acFit）
 *   phase 7  电感饱和 L(i) 拟合（dfStart / dfStep，分摊到多个慢环周期）
 *   phase 4  完成：结果提交给 param.c（offerStill），回到 Stop 后生效
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
#include <math.h>

IdentResult_TypeDef gIdent;

/* ===========================================================================
 * 参数辨识模式（IDENT_TEST），说明见 identify.h
 * =========================================================================*/
volatile IdentCfg_TypeDef gIdentCfg = { 0 };
IdentTest_TypeDef gIdentTest;

/* 辨识电流按 MOTOR_I_RATED_A 缩放（本电机 0.75 A 时 IDENT_I_SCALE = 1.0，与原值逐位相同）；
 * 最大测试电流（2.5 * SCALE 与大幅值正交偏置合成）不超过 SPD_PI_LIMIT，并保留余量防三相静止坐标系 int16 溢出 */
#define IDENT_I_SCALE_RAW   (MOTOR_I_RATED_A / 0.75)
#define IDENT_I_SCALE_MAX   ((SPD_PI_LIMIT < 0.72 * I_MAX ? SPD_PI_LIMIT : 0.72 * I_MAX) / 2.5)
#define IDENT_I_SCALE       (IDENT_I_SCALE_RAW < IDENT_I_SCALE_MAX ? IDENT_I_SCALE_RAW : IDENT_I_SCALE_MAX)
#define IDENT_Q15_I(a)      Q15((a) * IDENT_I_SCALE / I_MAX)

/* 死区阶梯各档 d 轴电流（按额定电流缩放）：从定位电流附近向上，再向下细分到小电流段（反映过零附近的特性） */
static const int16_t s_dtLevel[IDENT_DT_N] = {
    IDENT_Q15_I(1.00), IDENT_Q15_I(0.80), IDENT_Q15_I(0.60), IDENT_Q15_I(0.50), IDENT_Q15_I(0.40),
    IDENT_Q15_I(0.30), IDENT_Q15_I(0.25), IDENT_Q15_I(0.20), IDENT_Q15_I(0.15), IDENT_Q15_I(0.10),
    IDENT_Q15_I(0.05)
};
#define DT_SETTLE_MS    300
#define DT_AVG_MS       200
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
    gIdentTest.idx = 0;
    gIdentTest.cnt = 0;
    gIdent.state = IDENT_RUNNING;
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
        if (x < 0.45f * (float)IDENT_I_SCALE) continue;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        n += 1;
    }
    if (n < 2 || (n * sxx - sx * sx) <= 0) return;
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
 * 序列：方向 d（N 极）、q（N + 90°）× 频率 25 / 50 / 100 Hz（0.4 A），再 q 轴 25 Hz 增大幅值 1.0 / 1.5 / 2.0 / 2.5 A（饱和）。
 * 每段 AC_SETTLE 个周期稳定、AC_INT 个周期积分。
 * 快环：电流环闭环、角度固定，id 指令 = A sin(phi)，phi 用 Q32 相位累加；积分 Σv·sin、Σv·cos、Σi·sin、Σi·cos（整数）。
 * 本拍计算的电压延迟 CTRL_INVPARK_ADVANCE（1.5）个 PWM 周期才作用（下一周期生效、作用区间中点），与反 Park 角度补偿
 * 为同一物理量：电压按 phi(k + 1.5) 做相关。 */
#define AC_NF       3
#define AC_NSEG     16
#define AC_SETTLE   4
#define AC_INT      8
#define AC_V_DELAY  CTRL_INVPARK_ADVANCE
/* 频率编号 3 = 12.5 Hz：仅用于 q 轴，与 25 Hz 配对测量转子摆动（见 acFit） */
static const uint16_t s_acPer[AC_NF + 1] = {    /* 拍 / 周期 */
    (uint16_t)(PWM_FREQ / 25), (uint16_t)(PWM_FREQ / 50),
    (uint16_t)(PWM_FREQ / 100), (uint16_t)(PWM_FREQ / 12.5)
};
static const float    s_acHz[AC_NF + 1]  = { 25.0f, 50.0f, 100.0f, 12.5f };
static const uint8_t  s_acSegQ[AC_NSEG]  = { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };      /* 0 = d 轴，1 = q 轴 */
static const uint8_t  s_acSegF[AC_NSEG]  = { 0, 1, 2, 0, 1, 2, 3, 0, 0, 0, 0, 3, 0, 0, 0, 0 };      /* 频率编号 */
/* 12.5 Hz 0.4 A（段 6）紧跟 0.4 A 各段、位于大幅值段之前：大幅值段撤除偏置后转子会摆动一段时间，此时测小幅值不准确 */
#define AC_S_Q04    3       /* q 25 Hz 0.4 A */
#define AC_S_Q04L   6       /* q 12.5 Hz 0.4 A */
#define AC_S_Q10    7       /* q 25 Hz 1.0 A（大幅值第一段，后面依次 1.5 / 2.0 / 2.5） */
#define AC_S_Q10L   11      /* q 12.5 Hz 1.0 A */
#define AC_S_Q10B2  12      /* q 25 Hz 1.0 / 1.5 / 2.0 / 2.5 A，偏置 2 倍 */
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
static volatile uint8_t s_acDone;    /* 快环置 1：本段积分完成，等待慢环读取 */

static void acSegStart(uint16_t seg)    /* 慢环 */
{
    uint16_t f = s_acSegF[seg];
    s_acTheta = (int16_t)((gIpd.ok ? gIpd.theta : Ident_AlignTheta()) + (s_acSegQ[seg] ? ANGLE_90_Q15 : 0));
    s_acAmp = s_acSegA[seg];
    s_acEnv = 0;
    s_acEnvStep = (int32_t)(32768L / (AC_RAMP * (int32_t)s_acPer[f])) + 1;
    s_acBias = (int16_t)-(s_acSegB[seg] * (gIdentCfg.biasX ? gIdentCfg.biasX : 1));     /* 注入坐标系旋转了 +90°，N 极位于其 -q 方向 */
    s_acInc = (uint32_t)(4294967296.0 / (double)s_acPer[f] + 0.5);
    s_acDly = (uint32_t)(AC_V_DELAY * (double)s_acInc + 0.5);
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

/* 慢环：读取一段积分，Z = V/I（复数，以 sin 为实轴）。电压先扣除死区电压（与电流同时刻，无需延时对齐）：
 * 扣除后的实部应接近 Rs（用于校核死区模型）；死区与电流同相，是否扣除均不影响虚部（电感）。 */
static void acSegFinish(uint16_t seg)
{
    float vr = (float)(s_acVs - s_acEs), vi = (float)(s_acVc - s_acEc), ir = (float)s_acIs, ii = (float)s_acIc, d = ir * ir + ii * ii;
    float kv = (float)(UDC_MAX / 32768.0), ki = (float)(I_MAX / 32768.0);
    if (d <= 0) return;
    if (seg < AC_NF) s_acZre[seg] = (vr * ir + vi * ii) / d * kv / ki;
    s_acZim[seg] = (vi * ir - vr * ii) / d * kv / ki;
    if (seg == AC_S_Q04 || (seg >= AC_S_Q10 && seg < AC_S_Q10 + 4))     /* q 轴 25 Hz 五档的电流幅值（Σ i sin = A N / 2） */
        gIdent.lqAtA[seg == AC_S_Q04 ? 0 : seg - AC_S_Q10 + 1] = 2.0f * sqrtf(d) / (float)(AC_INT * s_acPer[s_acSegF[seg]]) * ki;
}

/* q 轴注入的转子摆动修正系数（理论值）Kmot = 1.5 p^2 psi^2 / J = p * psi * acc * 2π/60（acc = 加速度 / 转矩电流）。
 * 若运行辨识已测得（gIdent.psiWb、accRpmSPerA 非 0）则使用测得值，否则使用 cfg_motor.h。仅作对照：修正使用 12.5 / 25 Hz 实测值 */
static float acKmot(void)
{
    float psi = gIdent.psiWb > 0 ? gIdent.psiWb : (float)gParam.psiWb;
    float acc = gIdent.accRpmSPerA > 0 ? gIdent.accRpmSPerA : (float)gParam.accRpmSPerA;
    return (float)MOTOR_POLE_PAIRS * psi * acc * (float)(2.0 * MATH_PI / 60.0);
}

/* 交流阻抗汇总：q 轴扣除转子摆动影响（12.5 / 25 Hz 成对测出摆动系数），得到各频率的 Ld、Lq 及各幅值的 Lq，
 * 大幅值段由 1 倍 / 2 倍 N 极偏置外推到 0 偏置；最后取 25 Hz 的值作为 Ld / Lq */
static void acFit(void)
{
    uint16_t f;
    /* q 轴注入产生交变转矩，转子随之微幅摆动，摆动反电势使 q 轴阻抗多出一项 Kmot / (jω)（容性，表观电感减小 Kmot/ω^2，
     * 见 acKmot）。本电机 25 Hz 减小约 1.5 mH（3.6%）、50 Hz 0.4 mH、100 Hz 0.1 mH，需加回；d 轴无转矩，无需修正。
     * 摆动与电流成正比，大幅值段同样修正（2.5 A 时摆幅约 2° 电角度，仍属小角度） */
    /* 摆动系数实测：同一幅值下 12.5 / 25 Hz 两段，Im(Z)/ω = L - K/ω^2，假设 L 在 25 Hz 以下不随频率变化
     * （涡流影响：12.5 -> 25 Hz 约 0.4%），解出 K。
     * 理论上 K = Kmot（转子自由摆动）；转子被静摩擦 / 齿槽卡住时 K -> 0，此时按理论值修正会过补偿。
     * 0.4 A 各段使用 0.4 A 测得的 K，大幅值段使用 1.0 A 测得的 K（幅值越大转子越自由） */
    const float w12 = 2.0f * (float)MATH_PI * s_acHz[3], w25 = 2.0f * (float)MATH_PI * s_acHz[0];
    const float dk = 1.0f / (w12 * w12) - 1.0f / (w25 * w25);
    float k04 = (s_acZim[AC_S_Q04] / w25 - s_acZim[AC_S_Q04L] / w12) / dk;
    float k10 = (s_acZim[AC_S_Q10] / w25 - s_acZim[AC_S_Q10L] / w12) / dk;
    if (k04 < 0) k04 = 0;
    if (k10 < 0) k10 = 0;
    gIdent.kSwing[0] = k04;
    gIdent.kSwing[1] = k10;
    gIdent.kSwingModel = acKmot();
    for (f = 0; f < AC_NF; f++) {
        float w = 2.0f * (float)MATH_PI * s_acHz[f];
        gIdent.acLdH[f] = s_acZim[f] / w;
        gIdent.acLqH[f] = s_acZim[AC_NF + f] / w + k04 / (w * w);
        gIdent.acRohm[f] = s_acZre[f];
    }
    for (f = 0; f < 5; f++) {   /* q 轴 25 Hz 五档幅值：[0] = 0.4 A 段，[1..4] = 大幅值段 */
        uint16_t s = (uint16_t)(f == 0 ? AC_S_Q04 : AC_S_Q10 + f - 1);
        gIdent.lqAtH[f] = s_acZim[s] / w25 + (f == 0 ? k04 : k10) / (w25 * w25);
        if (f > 0) {    /* 偏置 1 倍 / 2 倍外推到 0 偏置 */
            float l2 = s_acZim[AC_S_Q10B2 + f - 1] / w25 + k10 / (w25 * w25);
            gIdent.lqAtHb1[f - 1] = gIdent.lqAtH[f];
            gIdent.lqAtHb2[f - 1] = l2;
            gIdent.lqAtH[f] = 2.0f * gIdent.lqAtH[f] - l2;
        }
    }
    /* 观测器需要的是转子坐标系中的低频电感（稳态运行时转子所见磁场恒定，转子内无涡流）。
     * 注入频率越低越接近该值，故取最低的 25 Hz（涡流效应在 50~100 Hz 之间开始明显）。 */
    gIdent.ldH = gIdent.acLdH[0];
    gIdent.lqH = gIdent.acLqH[0];
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
    if (s_acSeg == 0) {             /* 调整幅值 */
        float k = (i1 > 1.0f) ? (float)RL_I_Q15 / i1 : 2.0f;
        if (k > 0.85f && k < 1.15f) { s_acSeg = 1; return 0; }
        if (k > 2.0f) k = 2.0f;
        if (v >= (float)RL_VMAX_Q15 && k > 1.0f) { gIdent.rlR = -1; return 1; }   /* 电压已达上限电流仍不足：放弃，保留原参数 */
        v *= k;
        if (v > (float)RL_VMAX_Q15) v = (float)RL_VMAX_Q15;
        s_acAmp = (int16_t)v;
        return 0;
    }
    if (s_acSeg < 3) { s_acSeg++; return 0; }    /* 稳定 2 个周期 */
    s_acVs += (int32_t)is;
    s_acVc += (int32_t)ic;
    if (++s_acEs < RL_AVG) return 0;
    {
        float ss = (float)s_acVs, cc = (float)s_acVc, n = (float)(RL_AVG * RL_PER), d = ss * ss + cc * cc;
        float r, x;
        if (d <= 0) { gIdent.rlR = -1; return 1; }
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
 * 每项先经 param.c 的合理性检查（物理上为正、定点范围可容纳，见 checkParam），不合理的保留原值并在 gParamStat.rejectMask 中标记 */
static void offerStill(void)
{
    MotorParam_TypeDef p = gParam;
    uint16_t m = 0;
    if (gIdent.rsOhm > 0) { p.rsOhm = gIdent.rsOhm; m |= 1u; }
    if (gIdent.lqH > 0) {
        p.lqH = gIdent.lqH;
        p.ldH = MOTOR_IPM ? gIdent.ldH : gIdent.lqH;
        m |= 2u;
    }
    if (gIdent.lqH > 0) {
        p.lsKneeA = gIdent.lsSatKneeA;
        p.lsPerA = gIdent.lsSatPerA;
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
    case IDENT_PH_RL:       /* R/L（快环输出电压）：完成后回到定位（IDENT_PH_NONE = 定位正常进行，电流环已按 R/L 更新） */
        if (rlSlow()) {
            FocCore_PresetPI(0, 0);
            t->phase = IDENT_PH_NONE;
        }
        break;
    case IDENT_PH_DT_STEP:  /* 死区阶梯 */
        FocCore_SetIdqRef(s_dtLevel[t->idx], 0);
        if (++t->cnt > DT_SETTLE_MS) {
            t->sum[0] += gFoc.idq.s16D;
            t->sum[1] += gFoc.vdq.s16D;
        }
        if (t->cnt >= DT_SETTLE_MS + DT_AVG_MS) {
            t->dtId[t->idx] = (int16_t)(t->sum[0] / DT_AVG_MS);
            t->dtVd[t->idx] = (int16_t)(t->sum[1] / DT_AVG_MS);
            t->sum[0] = t->sum[1] = 0;
            t->cnt = 0;
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
            acSegStart(0);
            t->cnt = 0;
            t->phase = IDENT_PH_AC;
        }
        break;
    case IDENT_PH_AC:       /* 快环执行交流阻抗测量中 */
        if (s_acDone) {
            acSegFinish(s_acSeg);
            if (s_acSeg + 1 < AC_NSEG) acSegStart((uint16_t)(s_acSeg + 1));
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
            offerStill();
            if (gIdentCfg.mode == IDENT_MODE_AUTO) {
                s_autoStage = 1;            /* 停机（参数在 Stop 中装入）后重新起动，执行运行测量；state 保持 RUNNING */
                s_restartReq = 1;
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
        float im = sqrtf(id * id + iq * iq), r = (float)gParam.rsOhm, l = (float)gParam.lqH, vdt = 0, ed, eq;
        float vph = (float)gParam.dtEffS * (float)PWM_FREQ * gMeas.vbusAvg * (float)(UDC_MAX / 32768.0);
        uint16_t j;
        {   /* L(|i|)：与观测器相同的饱和模型 */
            float m = 1.0f - (float)gParam.lsPerA * (im - (float)gParam.lsKneeA);
            if (im > (float)gParam.lsKneeA) l *= (m < (float)MOTOR_LS_SAT_MIN ? (float)MOTOR_LS_SAT_MIN : m);
        }
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
