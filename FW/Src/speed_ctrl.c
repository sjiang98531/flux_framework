/* ============================================================================
 * @file    speed_ctrl.c
 * @brief   速度环（慢环，1 kHz）：给定轨迹（S 形斜坡）+ 速度 PI + 轨迹加速度前馈 + 限幅，输出 iq 指令
 *
 * 结构（每个慢环周期 SpeedCtrl_SpinSlow）：
 *   指令 cmd -> [S 形斜坡：限加速度、限加加速度（jerk）] -> 给定 ramp
 *                    |                        \-> 斜坡本拍斜率 / 加速度系数 = 前馈 iq（加速所需电流）
 *   ramp - 实测转速 -> [速度 PI] -> + 前馈 + [母线电流限 PI，只向下限制] -> 限幅（电流上限、回馈兜底）-> iq 指令
 * 前馈直接给出按轨迹加速所需的转矩，速度 PI 只修正负载与模型误差，因此 PI 带宽可以很低（SPEED_WN_HZ）。
 * 减速时母线电压控制器（busVdcLimit）实时减小减速侧斜坡斜率，使回馈功率不超过损耗可吸收的功率，母线不过冲。
 * I/F 切闭环后的一段时间内，开环遗留的 d / q 电流分量按 blendW 渐变淡出（SpeedCtrl_Blend*）。
 * ==========================================================================*/
#include "speed_ctrl.h"
#include "meas.h"
#include "foc_core.h"
#include "hal.h"
#include "id_ref.h"
#include <stdlib.h>
#include "param.h"
#include "identify.h"

SpeedCtrl_TypeDef gSpd;

/* 斜坡速率（rpm/s）-> 每慢环拍转速增量（Q31，满量程 = base rpm） */
#define RAMP_Q31(rate, base)    Q31((rate) / (base) / SLOWLOOP_FREQ)
#define RAMP_UP_Q31             RAMP_Q31(SPEED_RAMP_UP, SPEED_MAX)          /* 运行：加速 */
#define RAMP_DOWN_Q31           RAMP_Q31(SPEED_RAMP_DOWN, SPEED_MAX)        /* 运行：同向减速 */
#define RAMP_REV_Q31            RAMP_Q31(SPEED_RAMP_REV, SPEED_MAX)         /* 运行：反转的减速段 */
#define RAMP_STARTUP_Q31        RAMP_Q31(STARTUP_SPEED_RAMP, SPEED_MAX)     /* I/F 期间（仅显示） */

/* 从 gGain 装载速度 PI 增益（上电及参数更新时） */
void SpeedCtrl_LoadGains(void)
{
    gSpd.pi.qKp = gGain.spdKp;
    gSpd.pi.qKi = gGain.spdKi;
}

/* 上电：斜坡速率、电流上限 SPD_PI_LIMIT、速度 PI 与母线电流限 PI 的增益 / 状态 / 限幅。
 * 母线电流限 PI（piIbus）输出范围为 [-iqLimit, 0]：母线电流超过 IBUS_LIMIT_A 时向下限制 iq，平时输出 0 */
void SpeedCtrl_Init(void)
{
    gSpd.speedRampSt.s32RampUp     = RAMP_UP_Q31;
    gSpd.speedRampSt.s32RampDown   = RAMP_DOWN_Q31;

    gSpd.iqLimit = Q15(SPD_PI_LIMIT / I_MAX);
    SpeedCtrl_BlendStop();

    SpeedCtrl_LoadGains();
    gSpd.pi.qdSum = 0;
    gSpd.pi.qInMeas = 0;
    gSpd.pi.qOut = 0;

    gSpd.piIbus.qKp = POWER_KP;
    gSpd.piIbus.qKi = POWER_KI;
    gSpd.piIbus.qdSum = 0;
    gSpd.piIbus.qInMeas = 0;
    gSpd.piIbus.qOut = 0;

    gSpd.pi.qOutMax = gSpd.iqLimit;
    gSpd.pi.qOutMin = -gSpd.iqLimit;
    gSpd.piIbus.qOutMax = 0;                      /* 功率环仅用于母线电流限：只向下限制 */
    gSpd.piIbus.qOutMin = -gSpd.iqLimit;
}

/* 回馈兜底限幅：返回"阻碍转动方向的 iq"允许的最大值（Q15，>= 0）。正常减速由下方的母线电压控制器
 * 限制，母线不会达到 START；仅在其无法限制时（负载突然倒拖等）起作用。 */
static int16_t regenIqAllowed(void)
{
    int32_t v = gMeas.vbusAvg;
    if (v <= gGain.busRegenStartQ15) return gGain.busRegenIqMaxQ15;
    if (v >= gGain.busRegenFullQ15) return 0;
    return (int16_t)(((gGain.busRegenFullQ15 - v) * gGain.busRegenKQ12) >> 12);
}

/* 母线电压控制器（原理见 cfg_internal.h"回馈减速"）。在斜坡运行之前调用：
 * 斜坡朝 0 方向运行（减速、回馈方向）时，将该侧步长限制为"负载自然减速度 + 允许回馈功率所能支持的减速度"：
 *   step = min(额定, a_负载 + K * max(Vref^2 - V^2, 0) / |n|)
 * a_负载 = 速度 PI 输出中与转动同向的部分（负载转矩；加减速转矩在轨迹前馈中）* 加速度系数：按该减速度减速时
 * iq 指令恰为 0，电机不向母线回馈。母线高于调节点时（储能裕量为 0）仍可按负载自然减速度减速，减速指令不会被搁置。
 * 同时计算受限程度 lam = 1 - step / 额定，供损耗制动 id 使用。只读写 gSpd，慢环调用。 */
static void busVdcLimit(void)
{
    int32_t st = gSpd.speedRampSt.s32State, tgt = (int32_t)gSpd.cmd << 16;
    int32_t lam = 0;
    if ((st > 0 && tgt < st) || (st < 0 && tgt > st)) {
        int32_t *pr = (st > 0) ? &gSpd.speedRampSt.s32RampDown : &gSpd.speedRampSt.s32RampUp;
        int32_t nom = (int32_t)*pr, v = gMeas.vbusAvg, e2 = gGain.busVdcRefSq - v * v, s = abs(gMeas.speed), step;
        int32_t iqLoad = (st > 0) ? gSpd.pi.qOut : -gSpd.pi.qOut;      /* 负载转矩对应的 iq（Q15，与转动同向为正） */
        int32_t stepLoad = (iqLoad > 0) ? (int32_t)(((int64_t)iqLoad * gGain.spdAccPerIqQ8) >> 8) : 0;
        if (s < BUS_VDC_SMIN_Q15) s = BUS_VDC_SMIN_Q15;
        if (e2 <= 0) {
            step = 0;                       /* 母线已达调节点：不允许回馈，只按负载自然减速度减速 */
        } else {
            /* 不使用 64 位除法。N = e2*K（< 2^44），先用乘法判断是否受限（N >= nom*s*256 即不受限，
             * 多数情况走此分支），受限时 1/s 用一次 32 位除法（s >= SMIN，r = 2^30/s < 2^22），
             * step = (N >> 16) * r >> 22（乘积 < 2^49）。截断误差 < 1 LSB */
            uint64_t n = (uint64_t)(uint32_t)e2 * (uint32_t)gGain.busVdcKQ8;
            if (n >= ((uint64_t)(uint32_t)nom * (uint32_t)s) << 8) {
                step = nom;
            } else {
                uint32_t r = (1UL << 30) / (uint32_t)s;
                step = (int32_t)(((uint64_t)(uint32_t)(n >> 16) * r) >> 22);     /* n >> 16 < 2^28 */
            }
        }
        step = (stepLoad >= nom - step) ? nom : step + stepLoad;       /* step、stepLoad 均 >= 0，step <= nom，不溢出 */
        if (step < nom) {
            /* lam = 1 - step / nom（Q15）：step、nom < 2^19，step << 12 < 2^31，nom >> 3 仍有 14 位以上精度 */
            uint32_t d = (uint32_t)nom >> 3;
            lam = (d == 0) ? 32767 : 32767 - (int32_t)(((uint32_t)step << 12) / d);
            if (lam < 0) lam = 0;
            *pr = step;
        }
    }
    gSpd.vdcLam = (int16_t)lam;
    {   /* 损耗制动 id：lam 超过 BUS_BRAKE_FULL_LAMBDA 时给满，其间线性；一阶平滑（时间常数同电压环） */
        int32_t k = (lam * BUS_BRAKE_LAM_K_Q12) >> 12, tgtId;
        if (k > 32767) k = 32767;
        tgtId = -((BUS_REGEN_BRAKE_Q15 * k) >> 15);
        gSpd.brakeIdQ15x = gSpd.brakeIdQ15x + (((tgtId << 15) - gSpd.brakeIdQ15x) >> 15) * BUS_VDC_FILT_Q15;
        gSpd.brakeId = (int16_t)(gSpd.brakeIdQ15x >> 15);
    }
}

/* 制动距离：加速度从 c（>= 0，Q31 / 拍）每拍减小 j 直至 0，期间斜坡还将经过的转速
 *   sum(c - j, c - 2j, ..., 0) = c^2/(2j) - c/2（离散求和，精确）
 * 1/(2j) 为编译期常数（BRAKE_INV_Q32 = 2^32/(2j)），仅用乘法：c < 2^24 -> c^2 >> 16 < 2^32，32x32->64 乘法。
 * S 形斜坡与回退前的减速衔接共用此函数，避免每拍 64 位开方 / 除法。 */
#define BRAKE_INV_Q32           ((uint32_t)(4294967296.0 / (2.0 * SPEED_JERK_Q31) + 0.5))
static int64_t brakeSq(int32_t c)               /* c^2 / (2j) */
{
    uint64_t c2 = (uint64_t)((uint32_t)c) * (uint32_t)c;
    return (int64_t)(((uint64_t)(uint32_t)(c2 >> 16) * BRAKE_INV_Q32) >> 16);     /* c < 2^24（RAMP_FIXEDPOINT_FITS） */
}
/* 本拍加速度取 c 后，"走完本拍 + 再按 jerk 收至 0"共需的转速：c + c^2/(2j) - c/2 */
static int64_t needDist(int32_t c)
{
    return (c <= 0) ? (int64_t)c : brakeSq(c) + (c >> 1);
}

/* 限 jerk 的速度斜坡（S 形）。状态：转速 v = speedRampSt.s32State、加速度 a = gSpd.rampAcc（Q31 / 拍）。
 * 每拍按"增加、保持、减小"的顺序选取第一个仍能及时制动的加速度（剩余距离 |e| >= needDist），即 bang-bang 式 jerk 规划：
 * 能加速则加到 a_max，到达制动距离时按 j 收回，恰好停在目标上（末拍由落点判断收口）。
 * 该方法每拍只需乘法，避免了 a* = min(a_max, sqrt(2 j |e|)) 所需的每拍 64 位开方。
 * a 超过本方向的 a_max（RampUp / RampDown，母线电压控制器会实时减小）时立即压到 a_max —— 这是物理约束，
 * 而非轨迹整形（a_max 本身随母线电压连续变化，不会跳变）。 */
static int32_t rampSCurve(int32_t tgt)
{
    Ramp_TypeDef *p = &gSpd.speedRampSt;
    int32_t v = (int32_t)p->s32State, a = gSpd.rampAcc, j = SPEED_JERK_Q31;
    int64_t e = (int64_t)tgt - v, nv;
    if (e == 0 && a == 0) return v;
    if (e != 0) {
        int32_t sgn = (e > 0) ? 1 : -1, as = a * sgn, cu;
        int32_t amax = (int32_t)((e > 0) ? p->s32RampUp : p->s32RampDown);
        int64_t ae = e < 0 ? -e : e;
        cu = as + j;
        if (cu > amax) cu = amax;
        if (needDist(cu) <= ae) as = cu;                            /* 增加（或已在 a_max 上保持） */
        else if (as <= amax && needDist(as) <= ae) { /* 保持 */ }
        else as -= j;                                               /* 减小 */
        a = as * sgn;
    }
    if (a > (int32_t)p->s32RampUp) a = (int32_t)p->s32RampUp;
    if (a < -(int32_t)p->s32RampDown) a = -(int32_t)p->s32RampDown;
    nv = (int64_t)v + a;
    if ((e > 0 && nv >= tgt) || (e < 0 && nv <= tgt) || e == 0) { nv = tgt; a = 0; }
    p->s32State = (int32_t)nv;
    gSpd.rampAcc = a;
    return (int32_t)nv;
}

/* 回退前的轨迹衔接：需交给 I/F 时（指令反向或低于闭环最低转速），回退时刻斜坡的减速度应等于 I/F 开环
 * 过零的减速度（OPENLOOP_REV_ACC_RPM_S_OF），以保证转矩连续。按 jerk 收回加速度需要 (a^2 - a_if^2) / (2j) 的转速，
 * 距回退点剩余该转速时，减速侧上限按 jerk 每拍减小，到达回退点时恰好为 a_if。 */
static void approachFallback(void)
{
    int32_t st = gSpd.speedRampSt.s32State, c = gSpd.cmd, ac = c < 0 ? -c : c;
    int32_t ast = st < 0 ? -st : st, aif = gGain.rampOpenLoopRevQ31, a = gSpd.rampAcc < 0 ? -gSpd.rampAcc : gSpd.rampAcc;
    int32_t *pr;
    int64_t zone;
    if (!((st > 0 && c < 0) || (st < 0 && c > 0) || ac < gGain.spdSpinMinQ15)) return;
    pr = (st > 0) ? &gSpd.speedRampSt.s32RampDown : &gSpd.speedRampSt.s32RampUp;
    if ((int32_t)*pr <= aif) return;
    zone = (int64_t)gGain.spdFallbackQ31 + ((a > aif) ? brakeSq(a) - brakeSq(aif) : 0);      /* (a^2 - a_if^2) / (2j) */
    if ((int64_t)ast <= zone + a) {
        int32_t lim = a - SPEED_JERK_Q31;
        if (lim < aif) lim = aif;
        if ((int32_t)*pr > lim) *pr = lim;
    }
}

/* 速度环一拍（Spin 慢环）：
 *   1. 非融合期：按方向选择斜坡速率（反转用 SPEED_RAMP_REV）、回退前衔接、母线电压限制减速；
 *   2. S 形斜坡运行一步，得到给定 ramp；斜坡本拍变化量 * 前馈系数 = 加速前馈 iq；
 *   3. 回馈兜底：母线超过 BUS_REGEN_START_V_OF 时限制"阻碍转动方向"的 iq；
 *   4. 速度 PI（给定 - 实测）+ 母线电流限 PI + 前馈 = iq 指令，限幅；id 指令 = 损耗制动 id（MTPA 关闭时）。
 * 结果写入 gSpd.idqCmd，由 spin.c 发布给电流环 */
void SpeedCtrl_SpinSlow(void)
{
    gSpd.idqCmd.s16D = 0;               /* id 默认 0（SPM）；回馈制动 id / MTPA 在下方覆盖 */

    {
        int32_t s32StPrev, s32IqCmd, s32IqMax, s32IqMin;
        int16_t s16AccFF;

        s32StPrev = gSpd.speedRampSt.s32State;
        if (gSpd.blendW >= 32767) {
            /* 反转（指令与斜坡反号）的减速段使用 SPEED_RAMP_REV，同向调速仍使用 SPEED_RAMP_UP/DOWN */
            int32_t st = gSpd.speedRampSt.s32State;
            gSpd.speedRampSt.s32RampUp   = (st < 0 && gSpd.cmd > 0) ? RAMP_REV_Q31 : RAMP_UP_Q31;
            gSpd.speedRampSt.s32RampDown = (st > 0 && gSpd.cmd < 0) ? RAMP_REV_Q31 : RAMP_DOWN_Q31;
#if IDENT_TEST
            {   /* 参数辨识惯量测量期间：升降速斜坡用测量加速度（identify.c Ident_RampQ31） */
                int32_t r = Ident_RampQ31();
                if (r > 0) { gSpd.speedRampSt.s32RampUp = r; gSpd.speedRampSt.s32RampDown = r; }
            }
#endif
            /* 以上每拍重设运行速率；下面两步按需要再压低减速一侧的步长（直接改 s32RampUp / s32RampDown） */
            approachFallback();     /* 回退 I/F 时刻的减速度 = I/F 过零的减速度 */
            busVdcLimit();          /* 减速侧步长按允许回馈功率限制（给定始终可实现） */
        }
        gSpd.rampQ31 = rampSCurve(gSpd.cmd << 16);
        gSpd.ramp = gSpd.rampQ31 >> 16;

        s16AccFF = 0;
        {   /* 轨迹前馈：iq_ff = 斜坡本拍实际斜率 / b（加速、减速及 I/F 融合期间的过渡斜率均适用） */
            int64_t ff = ((int64_t)(gSpd.speedRampSt.s32State - s32StPrev) * gGain.spdFfKQ16) >> 16;
            if (ff > gSpd.iqLimit) ff = gSpd.iqLimit;
            if (ff < -gSpd.iqLimit) ff = -gSpd.iqLimit;
            s16AccFF = (int16_t)ff;
        }
        s32IqMax = gSpd.iqLimit;
        s32IqMin = -gSpd.iqLimit;
        {   /* 兜底：限制回馈方向（iq 与转速反向）的上限，另一侧（加速）不受影响 */
            int32_t reg = regenIqAllowed();
            if (gMeas.speed >= 0) s32IqMin = -reg; else s32IqMax = reg;
        }
#if IDREF_ACTIVE
        /* 电流圆（id_ref.c）：iq 上限 = sqrt(Imax^2 - id^2)，id 取上一拍 IdRef_Slow 的结果（慢环 1 kHz，一拍滞后可忽略）。
         * PI 输出限幅只在此处设定，积分器随之限幅，加速到电流圆边界时不会积分饱和 */
        if (s32IqMax > gIdRef.iqLimit) s32IqMax = gIdRef.iqLimit;
        if (s32IqMin < -gIdRef.iqLimit) s32IqMin = -gIdRef.iqLimit;
#endif
        gSpd.pi.qOutMax = (int16_t)s32IqMax;          /* 速度 PI 积分器随之限幅，不累积 */
        gSpd.pi.qOutMin = (int16_t)s32IqMin;

        gSpd.pi.qInRef = gSpd.ramp;
        gSpd.pi.qInMeas = gMeas.speed;
        Pi_Run64(&gSpd.pi);

        gSpd.piIbus.qInRef = IBUS_LIMIT_VALUE;    /* 母线电流限 */
        gSpd.piIbus.qInMeas = gMeas.ibusAvg;
        Pi_Run64(&gSpd.piIbus);

        s32IqCmd = (int32_t)gSpd.pi.qOut + gSpd.piIbus.qOut + s16AccFF;
        if (s32IqCmd > s32IqMax) s32IqCmd = s32IqMax;
        if (s32IqCmd < s32IqMin) s32IqCmd = s32IqMin;
#if !IDREF_ACTIVE
        gSpd.idqCmd.s16D = gSpd.brakeId;            /* MTPA/弱磁打开时由 IdRef_Slow 叠加 */
#endif
        gSpd.idqCmd.s16Q = (int16_t)s32IqCmd;
    }
}

/* 进入 Spin（I/F 切换、零电流跟踪转闭环、软停取消）：斜坡从当前实测转速、加速度 0 开始；
 * 速度 PI 积分器预置为当前 iq 指令（转矩连续，进入时无跳变） */
void SpeedCtrl_OnEnterSpin(int16_t idNow, int16_t iqNow)
{
    gSpd.speedRampSt.s32RampUp     = RAMP_UP_Q31;
    gSpd.speedRampSt.s32RampDown   = RAMP_DOWN_Q31;
    gSpd.brakeId = 0;
    gSpd.brakeIdQ15x = 0;
    gSpd.vdcLam = 0;
    gSpd.idqCmd.s16D = idNow;           /* 从调用方给出的此刻电流指令开始 */
    gSpd.idqCmd.s16Q = iqNow;
    gSpd.speedRampSt.s32State      = gMeas.speed << 16;
    gSpd.rampAcc = 0;
    gSpd.pi.qdSum               = (int32_t)iqNow * 32768;
    gSpd.piIbus.qdSum               = 0;          /* 母线电流限 PI 只向下限制，未限流时积分为 0 */
}

/* 零电流跟踪期间每拍调用：速度环不运行，斜坡和积分器跟随估计转速（id = iq = 0），闭合速度环时没有跳变 */
void SpeedCtrl_FollowSpeed(void)
{
    SpeedCtrl_OnEnterSpin(0, 0);
}

/* I/F 切闭环时开始融合：记录切换时刻开环电流在观测器坐标系中的 d / q 分量（id0 / iq0），之后按 blendW 0 -> 1
 * 淡出；斜坡延续开环的转速与加速度，斜率在融合期间从开环加速度过渡到 SPEED_RAMP_UP / DOWN（SpeedCtrl_BlendSlow） */
void SpeedCtrl_BlendStart(int16_t id0, int16_t iq0, int32_t rampStartQ31, int16_t accScaleQ12, int16_t ff0)
{
    gSpd.blendId0 = id0;
    gSpd.blendIq0 = iq0;
    gSpd.blendW = 0;
    gSpd.blendAccQ12 = accScaleQ12;
    /* 斜坡延续开环：值 = 开环转速，斜率 = 开环加速度，融合期间斜率过渡到 SPEED_RAMP_UP / DOWN */
    gSpd.speedRampSt.s32State = rampStartQ31;
    gSpd.rampQ31 = rampStartQ31;
    gSpd.ramp = (int16_t)(rampStartQ31 >> 16);
    gSpd.speedRampSt.s32RampUp   = (int32_t)(((int64_t)gGain.rampOpenLoopQ31 * accScaleQ12) >> 12);
    gSpd.speedRampSt.s32RampDown = gSpd.speedRampSt.s32RampUp;
    /* 开环仍在加速（ff0 != 0）：斜坡加速度延续开环值，前馈连续 */
    gSpd.rampAcc = (ff0 > 0) ? gSpd.speedRampSt.s32RampUp : ((ff0 < 0) ? -gSpd.speedRampSt.s32RampUp : 0);
}

/* 停机复位：积分器、斜坡、融合状态清零 */
void SpeedCtrl_ResetRun(void)
{
    gSpd.pi.qdSum = 0;
    gSpd.piIbus.qdSum = 0;
    gSpd.speedRampSt.s32State = 0;
    gSpd.rampAcc = 0;
    SpeedCtrl_BlendStop();
    gSpd.ramp = 0;
    gSpd.rampQ31 = 0;
}

/* 将斜坡当前值直接设为 spdQ15（回退 I/F、接管时使给定从实际转速开始） */
void SpeedCtrl_PresetRamp(int16_t spdQ15)
{
    gSpd.speedRampSt.s32State = spdQ15 << 16;
}

/* 仅用于显示：将斜坡状态同步到 gSpd.ramp（零电流跟踪时速度环不运行，斜坡显示仍跟随实际转速） */
void SpeedCtrl_ShowRampState(void)
{
    gSpd.rampQ31 = gSpd.speedRampSt.s32State;
    gSpd.ramp = (int16_t)(gSpd.rampQ31 >> 16);
}

/* 结束 / 取消融合（blendW = 满量程，开环分量为 0） */
void SpeedCtrl_BlendStop(void)
{
    gSpd.blendW = 32767;
    gSpd.blendId0 = 0;
    gSpd.blendIq0 = 0;
    gSpd.blendAccQ12 = 4096;
}

/* 融合一拍（慢环）：blendW 按 OPENLOOP_BLEND_S 从 0 升至满量程；斜坡斜率同步过渡；
 * 在速度环输出指令上叠加 (1 - w) * 开环分量，切换时刻电流不跳变，之后平滑交给速度环 */
void SpeedCtrl_BlendSlow(void)
{
    int32_t w;
    if (gSpd.blendW >= 32767) return;
    w = (int32_t)gSpd.blendW + OPENLOOP_BLEND_STEP_Q15;
    if (w > 32767) w = 32767;
    gSpd.blendW = (int16_t)w;
    {   /* 斜坡斜率 I/F 加速度 -> SPEED_RAMP_UP / DOWN；w = 32767 时恰为 SpeedCtrl_OnEnterSpin 设置的值 */
        int32_t r0 = (int32_t)(((int64_t)gGain.rampOpenLoopQ31 * gSpd.blendAccQ12) >> 12);
        int32_t up = RAMP_UP_Q31, dn = RAMP_DOWN_Q31;
        gSpd.speedRampSt.s32RampUp   = (w >= 32767) ? up : r0 + (int32_t)(((int64_t)(up - r0) * w) >> 15);
        gSpd.speedRampSt.s32RampDown = (w >= 32767) ? dn : r0 + (int32_t)(((int64_t)(dn - r0) * w) >> 15);
    }
    /* 控制器输出照常全额生效，仅将切换时刻"误差坐标系"遗留的分量按 (1 - w) 淡出 */
    gSpd.idqCmd.s16D = (int16_t)(gSpd.idqCmd.s16D + (((int32_t)gSpd.blendId0 * (32767 - w)) >> 15));
    gSpd.idqCmd.s16Q = (int16_t)(gSpd.idqCmd.s16Q + (((int32_t)gSpd.blendIq0 * (32767 - w)) >> 15));
}

/* 进入 I/F 时：斜坡速率改为起动用的 STARTUP_SPEED_RAMP（仅用于显示，I/F 按自身开环加速度拖动） */
void SpeedCtrl_UseOpenLoopRamp(void)
{
    gSpd.rampAcc = 0;
    gSpd.speedRampSt.s32RampUp   = RAMP_STARTUP_Q31;
    gSpd.speedRampSt.s32RampDown = RAMP_STARTUP_Q31;
}
