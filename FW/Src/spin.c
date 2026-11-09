/* ============================================================================
 * @file    spin.c
 * @brief   闭环运行（Run 子状态 Spin）
 *
 *   Spin_Fast / Spin_Slow：快环（估计 + 电流环 + 调制）、慢环（速度环、MTPA、融合、失锁、回退、保护）
 *   零电流跟踪：飞车接管后的"先跟踪后控制"以及闭环中停机后的滑行跟踪
 * 进出 Spin 的状态切换位于 supervisor.c（Sup_Switch*），本文件只提供进入时的处理（Spin_OnEnter*）。
 * ==========================================================================*/
#include "spin.h"
#include "supervisor.h"
#include "hal.h"
#include "foc_core.h"
#include "est.h"
#include "startup.h"
#include "speed_ctrl.h"
#include "id_ref.h"
#include "meas.h"
#include "protect.h"
#include "flystart.h"
#include "identify.h"
#include "qmath.h"
#include <stdlib.h>
#include "compiler.h"

/* 零电流跟踪：Spin 中 id = iq = 0、PWM 保持输出、观测器与 PLL 持续跟踪转子，速度环不闭合。
 * 两个入口：飞车接管、闭环中停机（软停机电流降到 0 后）；两个出口：指令恢复 -> 闭合速度环，
 * 转速低于观测器可靠下限 -> 关闭 PWM 回到 Stop。见 Spin_TrackSlow。 */
static volatile uint8_t s_track;                /* 1 = 零电流跟踪中 */
static uint16_t s_trackN;                       /* 进入跟踪后的慢环拍数（饱和） */
static int32_t s_syncSum, s_syncN;              /* 当前块的 PLL 转速累加（块长 FLY_SYNC_TICKS/2，用于求平均） */

static int32_t s_fbLim;                 /* Spin -> I/F 回退前 |id| 的上限（Q15），按电流变化率降到 0；-1 = 未在回退，见 fallbackBlend */
static int8_t s_spinDir;              /* 进入 Spin 时的指令方向；之后指令反向（需要反转）时清 0 */
static uint16_t s_u16SpinCheckTime;     /* Spin 中 spinTooSlow 的轮询分频（SPIN_CHECK_TICKS） */

/* 符号：+1 / 0 / -1 */
static int8_t sign16(int16_t x)
{
    return (int8_t)((x > 0) - (x < 0));
}

/* 同向时估计转速降到 SPIN_LOST_RPM 以下：Spin 只在 >= 切换转速的区间运行，同向降到此值不属于正常减速
 * （正常减速在 OPENLOOP_FALLBACK_RPM 已交给 I/F），判为失锁或堵转 */
static uint8_t spinTooSlow(void)
{
    return (uint8_t)((int32_t)sign16(gSpd.cmd) * gMeas.speed < SPIN_LOST_Q15);
}

static void spinEntered(void);

/* 指令与估计转速同向 / 反向（任一为 0 时两者都不成立） */
static uint8_t cmdSameDir(void)
{
    return (uint8_t)(sign16(gSpd.cmd) * sign16(gMeas.speed) > 0);
}
static uint8_t cmdOppositeDir(void)
{
    return (uint8_t)(sign16(gSpd.cmd) * sign16(gMeas.speed) < 0);
}

/* 进入 Spin 的公共处理（I/F 切闭环、飞车接管、零电流跟踪转闭环）：记录指令方向，缺相统计与失速检查重新开始 */
static void spinBeginChecks(void)
{
    spinEntered();
    Protect_PhaseReset();
    s_u16SpinCheckTime = 0;
}

/* 停机复位：清除零电流跟踪标志 */
void Spin_Reset(void)
{
    s_track = 0;
}

/* 是否处于零电流跟踪中（supervisor.c 据此只执行 Spin_TrackSlow） */
uint8_t Spin_Tracking(void)
{
    return s_track;
}

/* 从 I/F 切入 Spin 时（Sup_SwitchStartupSpin）：记录方向，缺相统计与失速检查重新开始 */
void Spin_OnEnter(void)
{
    spinBeginChecks();
}

/* 飞车直接接入 Spin 时（Sup_SwitchCalibSpin，快环中）：进入零电流跟踪，速度环由零电流跟踪的慢环跟随转速 */
void Spin_OnEnterCatch(void)
{
    spinBeginChecks();
    Spin_TrackBegin();
}

/* ===========================================================================
 * 零电流跟踪
 * 电流控制为 0、电机不出力，PWM 保持输出，观测器与 PLL 持续跟踪转子。由此保持转子状态，
 * 滑行中重新给指令时可直接闭环，无需重新进行飞车测速。
 *   入口：飞车接管（脉冲给出初始角 / 转速）；闭环中停机（软停将电流降到 0 后）。
 *   出口：指令恢复（|cmd| > STOP_TO_RUN_SPEED）且已跟踪 FLY_SYNC_TICKS（PLL 收敛）-> 按跟踪转速闭合速度环，
 *         反向指令即为一次正常的闭环反转；
 *         无指令且转速低于 TRACK_END_RPM（观测器可靠下限）-> 关闭 PWM 回到 Stop，之后再起动走飞车检测 / 定位。
 * 转速按块平均（块长 FLY_SYNC_TICKS/2）：只在块结束时决定是否闭环，以刚结束一块的平均值预置转速滤波器与斜坡
 * （飞车入口的第一个决策点 = FLY_SYNC_TICKS，平均的是后半段）。
 * 顺风持续时持续跟踪；逆风使转子降速到结束转速以下时关闭 PWM，再起动按逆风飞车处理。
 * 零电流下只检查超速：堵转 / 缺相需有电流才有意义（零电流时三相积分全为噪声，缺相会误判）。 */
/* 开始零电流跟踪：电流指令置 0，计数清零（入口：飞车接管、闭环中软停完成） */
void Spin_TrackBegin(void)
{
    s_track = 1;
    s_trackN = 0;
    s_syncSum = 0; s_syncN = 0;
    FocCore_SetIdqRef(0, 0);
    SpeedCtrl_BlendStop();      /* 跟踪期间电流为 0，未完成的 I/F 切换融合作废，否则跟踪结束闭环时会重新叠加 */
}

/* 零电流跟踪一拍（慢环）：
 *   前 first 拍等待 PLL 收敛；之后每 half 拍为一块，累加 PLL 转速求平均；
 *   块结束时：存在指令（|指令| > STOP_TO_RUN_SPEED）-> 以该块平均转速预置转速滤波器、斜坡、速度环，结束跟踪并闭环；
 *            无指令且转速 < TRACK_END_RPM -> 关闭 PWM 回到 Stop；否则继续跟踪。
 *   零电流下只检查超速（缺相、堵转需有电流才有意义） */
void Spin_TrackSlow(void)
{
    const uint16_t half = FLY_SYNC_TICKS >> 1, first = FLY_SYNC_TICKS - (FLY_SYNC_TICKS >> 1);
    int16_t w = (int16_t)(gEst.pll.s32Speed >> 16);
    uint8_t want = (uint8_t)(abs(gSpd.cmd) > Q15(STOP_TO_RUN_SPEED / SPEED_MAX) && !(IDENT_TEST && Ident_HoldTrack()));
    uint16_t n = s_trackN;

    SpeedCtrl_FollowSpeed();            /* 速度环不运行：斜坡、积分器跟随估计转速，id = iq = 0 */
    FocCore_SetIdqRef(0, 0);
    Meas_SlowFilter();
    SpeedCtrl_ShowRampState();          /* 仅用于显示：跟踪中斜坡 = 估计转速（速度环不运行，否则会停留在停机前的指令） */
    if (!want) Sup_ClearRestarts();     /* 与 Stop 状态相同：指令归零时清除连续重启计数 */
    if (Protect_OverSpeed(gMeas.speed)) {
        Sup_RaiseFault(MC_FAULT_SPEED_OC);
        return;
    }
    if (n < 0xFFFF) s_trackN = n + 1;
    if (n < first) return;              /* PLL 仍在收敛 */
    if (((uint16_t)(n - first) % half) == 0) { s_syncSum = 0; s_syncN = 0; }    /* 新的一块开始 */
    s_syncSum += w; s_syncN++;
    if (s_syncN < half) return;
    w = (int16_t)(s_syncSum / s_syncN);
    if (want) {
        /* 转速滤波器按该块 PLL 转速平均值重新预置（稳态累加值 = x*2^sh - x），斜坡 / 积分器由此起步。
         * 使用平均值而非最后一拍：PLL 瞬时转速含噪声，按单拍预置会使速度环闭合瞬间产生电流冲击 */
        Sup_Log(SUP_EV_TRACK_END, 1);
        Meas_SpeedPreset(w);
        SpeedCtrl_OnEnterSpin(0, 0);
        spinBeginChecks();
        s_track = 0;
    } else if (abs(w) < TRACK_END_Q15) {
        Sup_Log(SUP_EV_TRACK_END, 0);
        HAL_PwmAllOff();
        Sup_SwitchRunStop();
    }
}

/* ===========================================================================
 * Spin（闭环运行）
 * =========================================================================*/
/* 闭环快环一拍：Clarke -> 估计（观测器 + PLL）-> 电流环 -> 缺相累加 -> 调制。
 * 零电流跟踪时同样走此路径（电流指令为 0） */
RAMFUNC void Spin_Fast(void)
{
    FocCore_Clarke();
    Est_SpinTick();
    FocCore_CurrentLoop();

    Protect_PhaseFast();     /* 缺相：三相 |i| 累加（Protect_PhaseSlow 按电周期比较） */

    /* 低速七段式，高速（> PWM_5SEG_RPM）五段式 */
    FocCore_Modulate((abs(gMeas.speed) < PWM_5SEG_Q15) ? SEVENMODE : FIVEMODE);
}

/* 进入 Spin 时记录指令方向 */
static void spinEntered(void)
{
    /* 仅当转子方向与指令一致时才记为"进入 Spin 时的方向"。飞车按转子自身方向接管而指令反向时，
     * 进入 Spin 即为一次正常反转（减速 -> 回退 I/F -> 过零），不能判为失锁 */
    s_fbLim = -1;
    s_spinDir = cmdSameDir() ? sign16(gSpd.cmd) : 0;
}

/* 失锁：指令方向自进入 Spin 以来未变，估计转速却变为 0 或反向。转子在几毫秒内不可能被自身正向转矩
 * 拖成反转，说明观测器 / PLL 已失锁。此时 PLL 的转速和角度均不可信，若用其初始化 I/F 会按错误方向拖动电机。
 * 因此关闭 PWM 回到 Stop：若指令仍存在则立即重新起动，由飞车检测重新测量真实转速后接管。
 * 被风吹至反转（指令不变、转子确实缓慢反转）也走此路径，同样交由飞车检测处理。连续失锁计入 faultRestarts。 */
static uint8_t spinLostLock(void)
{
    if (s_spinDir != 0 && (int32_t)gSpd.cmd * s_spinDir <= 0) s_spinDir = 0;     /* 指令反向：属于正常反转，按回退处理 */
    return (uint8_t)(s_spinDir != 0 && (int32_t)gMeas.speed * s_spinDir <= 0);
}

/* 失锁 / 失速：记录事件、计数重启次数、停机（若指令仍存在，Stop 慢环将立即重新起动，飞车检测重新测速） */
static void spinLostLockStop(void)
{
    Sup_Log(SUP_EV_LOST_LOCK, (uint16_t)gMeas.speed);
    Sup_CountRestart();
    Sup_SwitchRunStop();
}

/* 回退融合：Spin -> I/F 时 I/F 电流沿开环 d 轴、幅值为 I，而此时闭环 id 常为回馈制动的负 id，
 * 直接切换会使 d 轴电流阶跃。分两段消除阶跃：
 *  1. 判定需回退后先留在 Spin，将 |id| 上限按电流变化率 CURRENT_SLEW_A_S 降到 0（撤除回馈制动 id，电流只减不增，
 *     观测器不受影响；若在闭环中直接渐变到 I/F 的 d 分量，低速大 id 下电阻误差可能导致观测器失锁）；
 *  2. 进入 I/F 时开环电流从 |iq| / sin(OPENLOOP_WARM_DELTA_DEG) 起、按同一变化率升到目标（Startup_PrepareOpenLoop），
 *     切换时刻 d 轴仅增加 |iq|。
 * 两段均使用 CURRENT_SLEW_A_S（= 软停机 = S 形斜坡的最大转矩变化率）。
 * 条件中途不再成立（指令改回）时停止。 */

static uint8_t fallbackBlend(void)
{
    int32_t d = gSpd.idqCmd.s16D;
    if (s_fbLim < 0) {
        if (!Startup_OpenLoopWantsFallback()) return 0;
        s_fbLim = abs(d);
    } else if (cmdSameDir() && abs(gSpd.cmd) >= OPENLOOP_SPIN_MIN_Q15) {
        /* 开始融合后只依据指令：转速在回退门限附近含噪声，若按转速判断会反复清零、融合无法完成，
         * 导致长时间停留在门限以下的闭环中。仅当指令改回同向时才取消 */
        s_fbLim = -1;
        return 0;
    }
    s_fbLim -= CURRENT_SLEW_STEP_Q15;
    if (s_fbLim < 0) s_fbLim = 0;
    if (d > s_fbLim) d = s_fbLim;
    if (d < -s_fbLim) d = -s_fbLim;
    gSpd.idqCmd.s16D = (int16_t)d;
    return (uint8_t)(s_fbLim == 0);
}

/* 闭环慢环一拍：
 *   速度环 -> MTPA / 弱磁 -> I/F 融合 -> 回退前撤除 id -> 一次性发布电流指令；
 *   失锁 / 失速检查（停机重启）、回退判断（交给 I/F）；测量滤波；缺相、超速保护 */
void Spin_Slow(void)
{
    uint8_t fbReady;
    SpeedCtrl_SpinSlow();
    IdRef_Slow();          /* id 指令（MTPA + 弱磁）与 iq 限幅；功能关闭时为空函数 */
    SpeedCtrl_BlendSlow();  /* I/F 切换融合：切换遗留分量叠加其上并逐渐淡出 */
    fbReady = fallbackBlend();  /* 需回退 I/F 时，先将 |id| 按变化率降到 0 */
    /* 本拍电流指令一次性发布 */
    FocCore_SetIdqRef(gSpd.idqCmd.s16D, gSpd.idqCmd.s16Q);

    /* 低速区（含过零反向）由 I/F 开环负责，在观测器仍锁定的转速即交回；同向降到 SPIN_LOST_RPM 以下 = 失锁 */
    if (spinLostLock()) { spinLostLockStop(); return; }
    if (fbReady) {
        Sup_Log(SUP_EV_FALLBACK, SUP_FB_OPENLOOP);
        Sup_SwitchSpinStartup();
    } else if (!cmdOppositeDir()) {    /* 同向，或估计转速恰为 0（指令此处不为 0）：0 也按"转不动"判失锁 */
        if (++s_u16SpinCheckTime >= SPIN_CHECK_TICKS) {
            s_u16SpinCheckTime = 0;
            if (spinTooSlow()) {
                /* Spin 只在 >= 切换转速的区间运行，同向降到 SPIN_LOST_RPM 以下不属于正常减速（正常减速在
                 * OPENLOOP_FALLBACK_RPM 已交给 I/F），判为失锁或堵转：PLL 不可信，同样停机重新检测 */
                spinLostLockStop();
                return;
            }
        }
    } else {
        /* 兜底：指令反向且已低于闭环最低转速仍未回退（正常在 OPENLOOP_FALLBACK_RPM 即开始回退，仅当撤除制动 id 期间
         * 转速已很低时才会进入此分支），立即交给 I/F */
        if (abs(gMeas.speed) < OPENLOOP_SPIN_MIN_Q15) {
            Sup_Log(SUP_EV_FALLBACK, SUP_FB_REVERSE);
            Sup_SwitchSpinStartup();
        }
    }

    Meas_SlowFilter();

    /* 保护（protect.h）：缺相按电周期窗口判定；超速。堵转由失锁停机 -> 重新检测 -> I/F 超时处理 */
    if (Protect_PhaseSlow((int16_t)(gEst.pll.s32Speed >> 16))) {
        Sup_RaiseFault(MC_FAULT_PHASELESS);
        Sup_CountRestart();
    } else if (Protect_OverSpeed(gMeas.speed)) {
        Sup_RaiseFault(MC_FAULT_SPEED_OC);
        Sup_CountRestart();
    }
}

