/* ============================================================================
 * @file    flystart.h
 * @brief   飞车（转子已在旋转）检测
 *
 * 在 FlyCheck 子状态的快环中每拍调用，将三相下桥对地短接（上桥全关）：
 * 转子反电势在绕组中产生短路电流，稳态时（电压方程 0 = Rs*i + L*di/dt + jω*ψf∠θ）
 *     i = -jω*ψf∠θ / (Rs + jωL)
 *     |i| = |ω|ψf / sqrt(Rs² + ω²L²)              本电机 30 rpm 0.15 A，250 rpm 1.2 A，1500 rpm 2.8 A
 *     θ   = ∠i + sgn(ω) * (90° + atan(|ω|Lq/Rs))    电流方向 + 阻抗角 = 转子角（内嵌式同样成立，使用 Lq）
 *
 * 首先以短脉冲测速，见 cfg_internal.h。
 *   脉冲电流取满足初值精度的最小值（按上电测得的采样噪声计算，且至少 FLY_PULSE_MIN_TICKS 拍），接管时电流约为 0；
 *   测不到或结果不一致时，转入下述连续短接测量。
 *
 * 连续短接测量（FlyStart_DetectFast）：
 *   1. 短接后等待 FLY_SETTLE_TAUS 个 L/R 时间常数（电流暂态衰减）
 *   2. 每 1 ms 求一次电流矢量平均 I_n。相邻两块 Δφ = atan2(I_{n-1}×I_n, I_{n-1}·I_n)
 *      （叉积 / 点积经一阶低通后再求角度）= 1 ms 内转过的电角度 -> 转速，符号即转向
 *   3. |I| < 门限持续 FLYBRAKE_TIME -> FLY_STILL（静止，转入定位）
 *      连续 FLY_STABLE_MS 次稳定：|转速| 足够高（同向 FLY_CATCH_RPM、反向 FLY_REV_SPIN_MIN_RPM）-> FLY_CATCH，
 *      >= FLY_LOW_MIN_RPM -> FLY_CATCH_OPENLOOP，
 *      接管角 = 本拍 ∠i + 阻抗角（见上式）
 *      其余情况继续短接（即制动，短路转矩近似正比于转速）；超过 FLY_BRAKE_TIMEOUT_S 仍未停止 -> FLY_TIMEOUT
 *   每拍只做累加，每 1 ms 执行一次 CORDIC，无除法和开方。
 * ==========================================================================*/
#ifndef FLYSTART_H
#define FLYSTART_H
#include "qtypes.h"
#include "foc_math.h"

/* gFly.stage（诊断） */
enum {
    FLY_STAGE_IDLE = 0, FLY_STAGE_SETTLE = 1, FLY_STAGE_MEASURE = 2, FLY_STAGE_CATCH = 3,
    FLY_STAGE_STILL = 4, FLY_STAGE_TIMEOUT = 5, FLY_STAGE_PULSE = 6
};
/* gFly.pulseResult（诊断） */
enum {
    FLY_PR_NONE = 0, FLY_PR_SPIN = 1, FLY_PR_OPENLOOP = 2, FLY_PR_TOO_SLOW = 3, FLY_PR_LOW = 4, FLY_PR_INCONSISTENT = 5
};

/* 飞车检测状态 */
typedef struct {
    int16_t     s16BemfAmpTh;           /* 判"在转"的电流门限（FLYCURRENT_TH_PU） */
    /* ---- 连续短接测量 ---- */
    uint16_t    stage;          /* 0 未开始，1 等待暂态，2 测量 / 制动，3 接管，4 静止，5 超时，6 短脉冲测速 */
    int16_t     speed;          /* 测到的转速 Q15（SPEED_MAX 基准），带方向；电流低于门限时 0 */
    uint16_t    stable;         /* 转速估计连续稳定的次数（ms） */
    uint16_t    still;          /* 电流连续低于门限的时间（ms） */
    uint16_t    stillSpinMs;    /* 检测到过转动时判静止需等待的时间（ms），上电按电机参数计算一次（FLY_STILL_END_RPM） */
    uint32_t    brakeMs;        /* 本次检测已短接的时间（ms） */
    int16_t     thetaCatch;     /* 接管时的转子角（Q15 角度，本拍采样时刻） */
    int16_t     speedCatch;     /* 接管时的转速 Q15 */
    /* ---- 短脉冲测速 ---- */
    uint16_t    pulseLen;       /* 脉冲长度（拍），第一个脉冲按电流门限确定，后续相同 */
    uint32_t    ampM2;          /* 连续短接：每块平均电流一半的平方（快环写入并与门限平方比较） */
    uint16_t    pulseResult;    /* 0 未测，1 接管 Spin，2 接管 I/F，3 太慢，4 反向 / 太低，5 测量不一致 */
} FlyStart_TypeDef;

extern FlyStart_TypeDef gFly;

typedef enum {
    FLY_BUSY = 0,       /* 正在测量 / 制动 */
    FLY_STILL,          /* 静止 -> 定位 */
    FLY_CATCH,          /* 转速足够（同向 >= FLY_CATCH_RPM、反向 >= FLY_REV_SPIN_MIN_RPM）-> 直接进入 Spin */
    FLY_CATCH_OPENLOOP,       /* FLY_LOW_MIN_RPM ~ 上述门限 -> 直接进入 I/F 开环 */
    FLY_TIMEOUT         /* 制动超时 -> 故障 */
} FlyVerdict_t;
void FlyStart_Init(void);       /* 上电：电流门限、检测到过转动后等待停稳的时间 */
/* 进入 FlyCheck 时调用：清除状态 */
void FlyStart_Begin(void);
/* Run 从 OpenLoop / Spin 退出（停机、故障）时调用：转子可能仍在滑行，下次飞车检测按"检测到过转动"等待停稳 */
void FlyStart_NoteRunEnd(void);
/* 每拍调用（短接下桥、Clarke、累加；每 1 ms 判定一次）。cmdSign：指令方向（> 0 正转）。
 * 返回 FLY_CATCH / FLY_CATCH_OPENLOOP 时 gFly.thetaCatch / speedCatch 有效（对应本拍采样时刻）。 */
FlyVerdict_t FlyStart_DetectFast(int16_t cmdSign);


#endif
