/* ============================================================================
 * @file    param.c
 * @brief   运行时电机参数：参数（物理量）-> 增益（定点数）、合理性检查、生效。整体说明见 param.h
 *
 * 函数一览：
 *   Param_Defaults     配置标称值 -> 参数
 *   checkParam         参数合理性检查（物理上为正、定点范围可容纳）
 *   computeGain        参数 -> 全部增益（double 运算，仅在主循环 / 上电 / 辨识模式中调用）
 *   install            将增益装入各模块（调用方已关中断）
 *   Param_Init         上电初始化
 *   Param_ApplyNow     辨识模式：立即生效
 *   Param_Offer        辨识结束：提交参数，待 Stop 后生效
 *   Param_Background   主循环：Stop 状态下生效
 * ==========================================================================*/
#include "param.h"
#include "cfg_derived.h"
#include "hal.h"
#include "foc_core.h"
#include "speed_ctrl.h"
#include "flux_obs.h"
#include "diag.h"
#include "supervisor.h"
#include "startup.h"
#include "openloop.h"
#include "flystart.h"
#include "qmath.h"

MotorParam_TypeDef gParam;
CtrlGain_TypeDef gGain;
ParamStat_TypeDef gParamStat;

/* 参数更新在主循环中执行（Param_Background）：用 double 计算新增益表（慢环中禁止软浮点），
 * 然后在 Stop 状态下关中断整体替换。辨识结果只在 Run 中提交（Param_Offer），Stop 中不会产生新结果 */
static MotorParam_TypeDef s_offer;          /* Param_Offer 提交、等待生效的参数 */
static volatile uint8_t s_offerReq;         /* 1 = s_offer 有效，主循环尚未处理 */

/* 将配置标称值（cfg_motor.h / cfg_board.h）填入 p */
static void Param_Defaults(MotorParam_TypeDef *p)
{
    p->rsOhm = MOTOR_RS_OHM;
    p->ldH = MOTOR_LD_H;
    p->lqH = MOTOR_LQ_H;
    p->lsKneeA = MOTOR_LS_SAT_KNEE_A;
    p->lsPerA = MOTOR_LS_SAT_PER_A;
    p->lsMaxA = 0.0;
    p->psiWb = MOTOR_PSI_F_WB;
    p->accRpmSPerA = MOTOR_ACC_RPM_S_PER_A;
    p->dtEffS = INVERTER_DT_EFF_S;
}

/* 合理性检查。不与配置标称值比较（新电机可能没有可靠的标称值），只检查"物理上为正"和"定点范围可容纳"
 * （各增益的 Q 格式、观测器磁链量程、NLFO 归一化范围，与 cfg_check_list.h 的编译期检查一致）。
 * 返回不合理项的位（位定义同 gParamStat.rejectMask），0 = 全部合理 */
static uint16_t checkParam(const MotorParam_TypeDef *p)
{
    uint16_t bad = 0;
    double psiN = (p->psiWb > 0 && p->ldH > 0 && p->lqH > 0) ? FLUX_PSI_N_OF(p->psiWb, p->ldH, p->lqH) : FLUX_PSI_N;
    /* bit0 Rs：为正，且满量程电流下的电阻压降在观测器电压基准 Q15 量程内 */
    if (!(p->rsOhm > 0 && p->rsOhm * I_MAX / FLUX_V_BASE < 0.99997)) bad |= 1u;
    /* bit1 L：为正，且满量程电流下的电感磁链 L*I 在观测器磁链量程内 */
    if (!(p->lqH > 0 && p->ldH > 0 && p->lqH * I_MAX / psiN < 0.9 && p->ldH * I_MAX / psiN < 0.9)) bad |= 2u;
    /* bit2 L(i)：拐点在电流量程内，斜率非负且 Q12 可容纳，上限电流为 0（不钳）或不低于拐点 */
    if (!(p->lsKneeA >= 0 && p->lsKneeA < I_MAX && p->lsPerA >= 0 && p->lsPerA * I_MAX < 7.0 && (p->lsMaxA == 0 || p->lsMaxA >= p->lsKneeA))) bad |= 4u;
    /* bit3 psi：最大定子磁链 psi + L*I 在观测器积分器限幅内，磁链定点非零且积分步长在 Q15 内 */
    if (!(p->psiWb > 0 && (p->psiWb + p->lqH * I_MAX) / psiN < 0.9 &&
          FLUX_NOMINAL_Q15_AT(p->psiWb, psiN) >= 16 &&
          2.0 * MATH_PI * FLUX_WN_HZ_OF(p->psiWb, p->ldH, p->lqH) / PWM_FREQ < 0.99997)) bad |= 8u;
    else {
        /* NLFO 归一化系数须非零；快环先把 ferr>>12 限在 fq 饱和点内再乘（flux_obs.c），norm < 2^22 即不溢出 int32 */
        int32_t n = FLUX_NLFO_NORM_Q8_OF(FLUX_NOMINAL_Q15_AT(p->psiWb, psiN));
        if (n < 1 || n >= FLUX_NLFO_NORM_MAX) bad |= 8u;
    }
    /* bit4 加速度系数：为正，且计算出的速度环 Kp、Ki 未被截断为 0 */
    if (!(p->accRpmSPerA > 0 && SPEED_KP_OF(p->accRpmSPerA) >= 1 && SPEED_KI_OF(p->accRpmSPerA) >= 1)) bad |= 16u;
    /* bit5 死区：非负，且小于 PWM 周期的 10%（超出则判为测量错误） */
    if (!(p->dtEffS >= 0 && p->dtEffS * PWM_FREQ < 0.1)) bad |= 32u;
    return bad;
}

/* 参数 -> 全部增益。公式与 cfg_derived.h 的编译期常量使用同一组宏（*_OF），参数等于配置标称值时结果逐位相同。
 * 被封顶的项在 cfgWarn 中记录一位（目标板上为 gFluxDiag.cfgWarn）。
 * 括号内为本电机配置标称值的计算结果，供对照 */
/* 观测器可靠性相关转速门限的换算比例 s = MOTOR_PSI_F_WB / psi，限制在 [0.25, 4]（见 param.h） */
static double spdThScale(const MotorParam_TypeDef *p)
{
    double s = (p->psiWb > 0) ? (double)MOTOR_PSI_F_WB / p->psiWb : 1.0;
    if (s < 0.25) s = 0.25;
    if (s > 4.0) s = 4.0;
    return s;
}


static void computeGain(const MotorParam_TypeDef *p, CtrlGain_TypeDef *g)
{
    int32_t k;
    double psiN = FLUX_PSI_N_OF(p->psiWb, p->ldH, p->lqH);
    g->cfgWarn = 0;
    /* 电流环：Kp = 2*pi*CURRENT_BW_HZ*L，Ki = 2*pi*CURRENT_BW_HZ*R/PWM_FREQ（Kp/Ki = L/R，PI 零点对消电机极点，
     * 闭环为一阶、带宽 CURRENT_BW_HZ）。d、q 轴各用各自的电感（44842 / 475） */
    g->curKpD = CURRENT_KP_OF(p->ldH);
    g->curKiD = CURRENT_KI_OF(p->rsOhm);
    g->curKpQ = CURRENT_KP_OF(p->lqH);
    g->curKiQ = CURRENT_KI_OF(p->rsOhm);
    /* Pi_Run32 的增益为 16 位无符号，超出时封顶（闭环带宽低于设计值），并置告警 bit1 */
    if (g->curKpD > 65535 || g->curKpQ > 65535 || g->curKiD > 65535 || g->curKiQ > 65535) g->cfgWarn |= 2u;
    if (g->curKpD > 65535) g->curKpD = 65535;
    if (g->curKiD > 65535) g->curKiD = 65535;
    if (g->curKpQ > 65535) g->curKpQ = 65535;
    if (g->curKiQ > 65535) g->curKiQ = 65535;
    /* 速度环：被控对象为"iq -> 加速度"的积分器（系数 accRpmSPerA），PI 按二阶系统 SPEED_WN_HZ / SPEED_ZETA 配置极点；
     * 前馈系数将斜坡轨迹加速度直接换算为 iq，速度环只修正残差 */
    g->spdKp = SPEED_KP_OF(p->accRpmSPerA);
    g->spdKi = SPEED_KI_OF(p->accRpmSPerA);
    g->spdFfKQ16 = SPEED_FF_K_Q16_OF(p->accRpmSPerA);
    g->spdAccPerIqQ8 = SPEED_ACC_PER_IQ_Q8_OF(p->accRpmSPerA);
    /* dq 解耦前馈：vq 中加 w*psi（反电势，16297），vd / vq 中加交叉耦合 w*Lq*iq、w*Ld*id（4982）。
     * 满量程转速下反电势超过电压满量程时置告警 bit3（解耦项会被限幅，高速时前馈不完整） */
    g->decKEmfQ15 = DECOUPLE_K_EMF_Q15_OF(p->psiWb);
    if (W_EMAX_RAD_S * p->psiWb / UDC_MAX >= 1.0) g->cfgWarn |= 8u;
    g->decKLqQ12 = DECOUPLE_K_L_Q12_OF(p->lqH);
    g->decKLdQ12 = DECOUPLE_K_L_Q12_OF(p->ldH);
    /* 观测器：按当前参数的磁链基准计算 psi_f 及其平方、积分步长 KTs、NLFO 归一化系数 */
    g->fluxNomQ15 = FLUX_NOMINAL_Q15_AT(p->psiWb, psiN);
    g->fluxNom2 = (int32_t)g->fluxNomQ15 * (int32_t)g->fluxNomQ15;
    g->nlfoNormQ8 = FLUX_NLFO_NORM_Q8_OF(g->fluxNomQ15);
    g->fluxKtsQ15 = FLUX_KTS_Q15_OF(p->psiWb, p->ldH, p->lqH);
    /* 观测器电压模型 psi_s = ∫(v - Rs*i)、psi_r = psi_s - L*i 的两个系数；
     * 电流环电压单位下的 Rs（定位判静止用）；IPM 的 Ld - Lq */
    g->fluxKrsQ15 = FLUX_KRS_Q15_OF(p->rsOhm);
    g->fluxKlQ15 = FLUX_KL_Q15_AT(p->lqH, psiN);
    g->focKrsQ15 = FOC_KRS_Q15_OF(p->rsOhm);
    g->fluxKdlQ15 = FLUX_KDL_Q15_AT(p->ldH, p->lqH, psiN);
    /* 观测器电感饱和 L(i)：拐点、相对拐点上限、斜率 */
    g->lsKneeQ15 = FLUX_LS_KNEE_Q15_OF(p->lsKneeA);
    g->lsMaxQ15 = FLUX_LS_MAX_Q15_OF(p->lsMaxA, p->lsKneeA);
    g->lsSatKQ12 = FLUX_LS_SAT_K_Q12_OF(p->lsPerA);
    /* 观测器死区补偿：等效死区 / PWM 周期（INVERTER_DT_COMP = 0 时为 0，不补偿） */
    g->dtKtQ15 = FLUX_DT_KT_Q15_OF(p->dtEffS);
    /* PLL：二阶锁相环按 FLUXPLL_WN_HZ / ZETA 配置极点。鉴相器输出 = |psi| * sin(角度误差)，与磁链幅值成正比，
     * 因此增益除以 fluxNomQ15 归一化（29939 / 124）。超出 int16 时封顶（带宽降低）并置告警 bit2 */
    k = FLUXPLL_KP_OF(g->fluxNomQ15);
    if (k > 32767) { k = 32767; g->cfgWarn |= 4u; }
    g->pllKp = (int16_t)k;
    k = FLUXPLL_KI_OF(g->fluxNomQ15);
    if (k > 32767) { k = 32767; g->cfgWarn |= 4u; }
    g->pllKi = (int16_t)k;
    /* 转速 -> 每拍角度增量系数超出 Q15 时同样置告警 bit2（与电机参数无关，只与 SPEED_MAX、极对数、PWM 频率有关） */
    if (SPEED_TO_THETA >= 32767) g->cfgWarn |= 4u;
    /* IPD 注入电压：电流三角波幅值 IPD_HF_A，按 Lq 计算 */
    g->ipdHfVQ15 = Q15(IPD_HF_V_OF(p->lqH) / UDC_MAX);
    /* 飞车：阻抗角系数 w*Lq/Rs（封顶 65535 防 32 位乘法溢出）、短接暂态等待时间（L/R 的倍数）、门限电流与等待停稳 / 过期时间 */
    g->flyWlrQ12 = FLY_WLR_Q12_OF(p->lqH, p->rsOhm);
    if (g->flyWlrQ12 > 65535) g->flyWlrQ12 = 65535;
    g->flySettleBlocks = FLY_SETTLE_BLOCKS_OF(p->ldH, p->rsOhm);
    {
        double thPu = FLYCURRENT_TH_PU_OF(p->psiWb, p->rsOhm);
        double x = FLY_STILL_RPM / FLY_STILL_END_RPM, y = (x - 1.0) / (x + 1.0), y2 = y * y, s = y, ln = 0.0, ms;
        int j;
        g->flyBemfAmpTh = Q15(thPu);
        g->flyEndAmpTh = Q15(thPu * FLY_STILL_END_RPM / FLY_STILL_RPM);
        for (j = 1; j < 60; j += 2) { ln += s / j; s *= y2; }
        ms = FLY_STILL_BLOCKS + FLY_BRAKE_TAU_S_OF(p->psiWb, p->rsOhm, p->accRpmSPerA) * 2.0 * ln * 1000.0;
        g->flyStillSpinMs = (uint16_t)(ms > 60000.0 ? 60000.0 : ms);
        g->flyMayMoveExpireMs = FLY_MAYMOVE_EXPIRE_MS_OF(p->psiWb, p->rsOhm, p->accRpmSPerA);
    }
    /* 起动与开环：定位摆周期、阻尼、摆动测量延迟、I/F 加速度、电流过渡斜率 */
    {
        uint32_t t = QMath_Isqrt64(ALIGN_T2_TICKS2_OF(p->psiWb, p->accRpmSPerA));
        g->alignPeriodTicks = (uint16_t)(t > 65535u ? 65535u : t);
        g->alignSwingMinSpd = (int16_t)(ALIGN_SWING_MIN_SPD_T / (double)(t ? t : 1) + 0.5);
        g->alignDampQ8 = (int32_t)(ALIGN_C1_OF(p->psiWb)
            * (double)QMath_Isqrt64((uint64_t)(ALIGN_K_NM_OF(p->psiWb) * ALIGN_J_OF(p->psiWb, p->accRpmSPerA) * 1099511627776.0))
            / 1048576.0);
        g->swingLagAngQ12 = ALIGN_SWING_LAG_ANG_Q12_OF(p->lqH, p->rsOhm);
    }
    g->openLoopAccStepQ31 = OPENLOOP_ACC_STEP_Q31_OF(p->accRpmSPerA);
    g->currentSlewStepQ15 = CURRENT_SLEW_STEP_Q15_OF(p->accRpmSPerA);
    g->rampOpenLoopQ31 = Q31(OPENLOOP_ACC_RPM_S_OF(p->accRpmSPerA) / SPEED_MAX / SLOWLOOP_FREQ);
    g->rampOpenLoopRevQ31 = Q31(OPENLOOP_REV_ACC_RPM_S_OF(p->accRpmSPerA) / SPEED_MAX / SLOWLOOP_FREQ);
    /* I/F 开环：按设计加速度 OPENLOOP_ACC_RPM_S_OF 拖动所需的 iq（加速度 / 加速度系数），切闭环时作为前馈（openloop.c） */
    g->openLoopAccFfIq = OPENLOOP_ACC_FF_IQ_OF(p->accRpmSPerA);
    /* 定位判静止：ALIGN_STILL_RPM 对应的反电势 */
    g->alignStillEmfQ15 = ALIGN_STILL_EMF_Q15_OF(p->psiWb);
    /* 母线回馈减速与超速保护 */
    g->busRegenStartQ15 = BUS_REGEN_START_Q15_OF(p->psiWb);
    g->busRegenFullQ15 = BUS_REGEN_FULL_Q15_OF(p->psiWb);
    g->busRegenIqMaxQ15 = BUS_REGEN_IQ_MAX_Q15_OF(p->psiWb);
    g->busRegenKQ12 = BUS_REGEN_K_Q12_OF(p->psiWb);
    g->busVdcRefSq = BUS_VDC_REF_SQ_OF(p->psiWb);
    g->busVdcKQ8 = BUS_VDC_K_Q8_OF(p->psiWb, p->accRpmSPerA);
    g->protectSpeedOcQ15 = PROTECT_SPEED_OC_Q15_OF(p->psiWb);
    {   /* 观测器可靠性相关的转速门限随磁链换算（见 param.h）：s = 配置磁链 / 当前磁链；s = 1 时各式与 cfg_derived.h 原宏逐位相同 */
        double s = spdThScale(p), top = 0.9 * SPEED_MAX;
        double hand, fb, sm, lost, te, fc, frev, warm;
        hand = OPENLOOP_HANDOVER_RPM * s;  if (hand > top) hand = top;
        fb   = OPENLOOP_FALLBACK_RPM * s;  if (fb > top) fb = top;
        sm   = OPENLOOP_SPIN_MIN_RPM * s;  if (sm > top) sm = top;
        lost = SPIN_LOST_RPM * s;          if (lost > top) lost = top;
        te   = TRACK_END_RPM * s;          if (te > top) te = top;
        fc   = FLY_CATCH_RPM * s;          if (fc > top) fc = top;
        frev = FLY_REV_SPIN_MIN_RPM * s;   if (frev > top) frev = top;
        warm = 1.5 * fb;                   if (warm > 0.95 * SPEED_MAX) warm = 0.95 * SPEED_MAX;
        g->spdHandoverQ31 = (int32_t)(hand / SPEED_MAX * 2147483648.0);
        g->spdFallbackQ31 = (int32_t)(fb / SPEED_MAX * 2147483648.0);
        g->spdWarmMaxQ31 = (int32_t)(warm / SPEED_MAX * 2147483648.0);
        g->spdFallbackQ15 = Q15(fb / SPEED_MAX);
        g->spdSpinMinQ15 = Q15(sm / SPEED_MAX);
        g->spdLostQ15 = (int16_t)(lost / SPEED_MAX * 32768.0);
        g->spdTrackEndQ15 = Q15(te / SPEED_MAX);
        g->spdFlyCatchQ15 = Q15(fc / SPEED_MAX);
        g->spdFlyRevMinQ15 = Q15(frev / SPEED_MAX);
    }
}

/* 将参数与增益装入各模块。调用方已关中断并确认电机处于 Stop（Param_ApplyNow 例外，见下）。
 * 只有结构体拷贝与整数赋值，无浮点运算（缩短关中断时间） */
static void install(const MotorParam_TypeDef *p, const CtrlGain_TypeDef *g)
{
    gParam = *p;
    gGain = *g;
    FocCore_LoadGains();                    /* 电流环 PI 增益（解耦系数在使用处直接读 gGain） */
    SpeedCtrl_LoadGains();                  /* 速度环 PI 增益（前馈系数直接读 gGain） */
    FluxObs_LoadModel();                    /* 观测器 Rs、L、KTs、NLFO 归一化（L(i)、死区系数直接读 gGain） */
    FluxPLL_LoadGains(&gEst.pll);           /* PLL 增益 */
    Startup_LoadGains();                    /* 定位摆周期、阻尼增益 */
    OpenLoop_LoadGains();                   /* 飞车接管开环步长 */
    FlyStart_LoadGains();                   /* 飞车电流门限与停稳等待时间 */
    gFluxDiag.cfgWarn = (uint16_t)((gFluxDiag.cfgWarn & ~14u) | gGain.cfgWarn);    /* 只更新 bit1~3，其他位为其他模块的告警 */
    gParamStat.applyCount++;
}

/* 上电：按配置标称值计算增益。此时各模块尚未 Init（其 Init 会读取 gGain），故只计算、不 install */
void Param_Init(void)
{
    MotorParam_TypeDef p;
    Param_Defaults(&p);
    computeGain(&p, &gGain);
    gParam = p;
    gParamStat.source = 0;
}

/* 辨识过程中立即生效（仅在辨识模式中使用）：R/L 测得后立即重算电流环，后续测量不依赖配置初值。
 * 在慢环中调用（辨识模式的慢环本身使用浮点），增益计算完成后关中断整体替换。
 * 与 Param_Background 不同，此时电机可能正由辨识程序驱动，因此由辨识程序保证在两段测量之间调用。
 * 参数不合理时返回 0，不更新 */
uint8_t Param_ApplyNow(const MotorParam_TypeDef *p)
{
    CtrlGain_TypeDef g;
    if (checkParam(p)) return 0;
    computeGain(p, &g);
    {
        IRQ_LOCK();
        install(p, &g);
        IRQ_UNLOCK();
    }
    gParamStat.source = 2;
    return 1;
}

/* 辨识结束时提交参数。validMask 标记本次测得的项：测得且合理的项才替换，其余保留当前值。
 * 组合后的整套参数再检查一次（单项合理但组合不合理，例如 psi + L*I 超出磁链量程，则整套放弃）。
 * 此处不立即生效：电机可能仍在旋转，交由主循环在 Stop 状态下更新（Param_Background） */
void Param_Offer(const MotorParam_TypeDef *p, uint16_t validMask)
{
    MotorParam_TypeDef n = gParam;
    uint16_t bad = checkParam(p);
    if ((validMask & 1u) && !(bad & 1u)) n.rsOhm = p->rsOhm;
    if ((validMask & 2u) && !(bad & 2u)) { n.ldH = p->ldH; n.lqH = p->lqH; }
    if ((validMask & 4u) && !(bad & 4u)) { n.lsKneeA = p->lsKneeA; n.lsPerA = p->lsPerA; n.lsMaxA = p->lsMaxA; }
    if ((validMask & 8u) && !(bad & 8u)) n.psiWb = p->psiWb;
    if ((validMask & 16u) && !(bad & 16u)) n.accRpmSPerA = p->accRpmSPerA;
    if ((validMask & 32u) && !(bad & 32u)) n.dtEffS = p->dtEffS;
    gParamStat.rejectMask = (uint16_t)(validMask & bad);    /* 已测得但不合理、被拒绝的项 */
    if (checkParam(&n)) return;         /* 组合后不合理（例如 psi + L*I 超出磁链量程）：整套放弃 */
    s_offer = n;
    s_offerReq = 1;
    gParamStat.pending = 1;
}

/* 主循环（main.c）每轮调用：存在待更新参数且电机处于 Stop（PWM 关闭、各模块未使用增益）时更新。
 * 分三步执行，使关中断期间只做拷贝：
 *   1. 关中断确认处于 Stop，拷贝待更新参数；
 *   2. 开中断状态下用 double 计算增益（在主循环中，耗时较长，但不影响快环 / 慢环）；
 *   3. 再次关中断并再次确认处于 Stop（第 2 步期间可能收到起动指令），装入各模块。
 * 不在 Stop 状态时不做任何操作，下一轮再试 */
void Param_Background(void)
{
    MotorParam_TypeDef p;
    CtrlGain_TypeDef g;
    uint8_t ok = 0;
    if (!s_offerReq) return;
    {
        IRQ_LOCK();
        if (gSup.mainState == MainState_Stop) { p = s_offer; ok = 1; }
        IRQ_UNLOCK();
    }
    if (!ok) return;
    computeGain(&p, &g);
    ok = 0;
    {
        IRQ_LOCK();
        if (gSup.mainState == MainState_Stop) { install(&p, &g); ok = 1; }
        IRQ_UNLOCK();
    }
    if (!ok) return;
    s_offerReq = 0;
    gParamStat.pending = 0;
    gParamStat.source = 2;
}

/* 按当前 gParam 的 L(|i|) 饱和模型计算电感（与 flux_obs.c:lsEff 一致，含 lsMaxA 钳位） */
float Param_LqAt(float iAbsA)
{
    float x = iAbsA;
    if (gParam.lsMaxA > 0.0 && x > (float)gParam.lsMaxA) x = (float)gParam.lsMaxA;
    if (x <= (float)gParam.lsKneeA || gParam.lsPerA <= 0.0) return (float)gParam.lqH;
    {
        float m = 1.0f - (float)gParam.lsPerA * (x - (float)gParam.lsKneeA);
        if (m < (float)MOTOR_LS_SAT_MIN) m = (float)MOTOR_LS_SAT_MIN;
        return (float)gParam.lqH * m;
    }
}
