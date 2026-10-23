/* ============================================================================
 * flystart.h —— 飞车（转子已在转）检测
 *
 * 两种方法都在 Calib 子状态的快环里每拍调用，都把三相下桥对地短接（上桥全关）：
 * 转子反电势在绕组里产生短路电流，稳态时（电压方程 0 = Rs*i + L*di/dt + jω*ψf∠θ）
 *     i = -jω*ψf∠θ / (Rs + jωL)
 *     |i| = |ω|ψf / sqrt(Rs² + ω²L²)              本电机 30 rpm 0.15 A，250 rpm 1.2 A，1500 rpm 2.8 A
 *     θ   = ∠i + sgn(ω) * (90° + atan(|ω|Lq/Rs))    电流方向 + 阻抗角 = 转子角（内嵌式也成立，用 Lq）
 *
 * FLY_VENDOR（FlyStart_CalibFast）：替代原厂闭源库的 MCF_FlyStart_2R，行为逐位相同。
 *   幅值 gFly.s16BemfAmp = |i|（> s16BemfAmpTh 认为在转）；角度用一个 PLL 跟踪
 *   （gFly.s16PosElEst_fly，原厂 MLIB_PLL 语义，借用 gEst.pll），PLL 的转速当转子转速。
 *   已知限制（台架 2026-10-23）：几十 rpm 时 PLL 判的转向可能是错的。
 *
 * FLY_PHASOR（FlyStart_DetectFast，0xF20E0008）：
 *   1. 短接后等 FLY_SETTLE_S（电流暂态衰减）
 *   2. 每 1 ms 求一次电流矢量平均 I_n。相邻两块 Δφ = atan2(I_{n-1}×I_n, I_{n-1}·I_n)
 *      （叉积/点积做一阶低通后再求角度）= 1 ms 里转过的电角度 -> 转速，符号就是转向
 *   3. |I| < 门限持续 FLYBRAKE_TIME -> FLY_STILL（静止，去定位）
 *      |转速| >= FLY_CATCH_RPM、与指令同向、连续 FLY_STABLE_MS 次稳定 -> FLY_CATCH，
 *      接管角 = 本拍 ∠i + 阻抗角（上式）
 *      其余情况继续短接（= 制动，短路转矩近似正比于转速）；FLY_BRAKE_TIMEOUT_S 还停不下 -> FLY_TIMEOUT
 *   每拍只做累加，1 ms 一次 CORDIC，没有除法和开方。
 * ==========================================================================*/
#ifndef FLYSTART_H
#define FLYSTART_H
#include "mc_types.h"

/* 飞车检测状态（字段沿用原厂 FLY_START） */
typedef struct {
    int16_t     s16PosElEst_fly;        /* 短路电流 PLL 的角度 */
    int16_t     s16ThetaErr_fly;
    sAngle_Trig sAngle_fly;
    int32_t     s32BemfAmp2;            /* |i|^2 */
    int16_t     s16BemfAmp;             /* |i| */
    int16_t     s16BemfAmpTh;           /* 判"在转"的电流门限 */
    uint32_t    u32FlyBrakeTimeCnt;     /* 未超门限的累计拍数 */
    uint32_t    u32FlyBrakeTimeCmd;     /* 超过它就判静止 */
    /* ---- FLY_PHASOR（J-Scope 看这几个） ---- */
    uint16_t    stage;          /* 0 未开始，1 等暂态，2 测量/制动，3 接管，4 静止，5 超时 */
    int16_t     amp;            /* 短路电流幅值 Q15（I_MAX 基准），1 ms 平均矢量的模 */
    int16_t     speed;          /* 测到的转速 Q15（SPEED_MAX 基准），带方向；电流低于门限时 0 */
    uint16_t    stable;         /* 转速估计连续稳定的次数（ms） */
    uint16_t    still;          /* 电流连续低于门限的时间（ms） */
    uint32_t    brakeMs;        /* 本次检测已短接的时间（ms） */
    int16_t     thetaCatch;     /* 接管时的转子角（Q15 角度，本拍采样时刻） */
    int16_t     speedCatch;     /* 接管时的转速 Q15 */
    uint16_t    catches;        /* 上电以来直接接管的次数 */
    uint16_t    brakes;         /* 上电以来"在转但不接管、先刹停"的次数 */
} FlyStart_TypeDef;

extern FlyStart_TypeDef gFly;

/* FLY_VENDOR：原厂算法（每拍） */
void FlyStart_CalibFast(void);

/* FLY_PHASOR */
typedef enum {
    FLY_BUSY = 0,       /* 还在测 / 在制动 */
    FLY_STILL,          /* 静止 -> 定位 */
    FLY_CATCH,          /* 同向、>= FLY_CATCH_RPM -> 直接进 Spin */
    FLY_CATCH_IF,       /* 同向、FLY_LOW_MIN_RPM ~ FLY_CATCH_RPM（只在 FLY_LOW_IF）-> 直接进 I/F 开环 */
    FLY_TIMEOUT         /* 制动超时 -> 故障 */
} FlyVerdict_t;
/* 进 Calib 时调用：清状态 */
void FlyStart_Begin(void);
/* 每拍调用（短接下桥、Clarke、累加；每 1 ms 判一次）。cmdSign：指令方向（> 0 正转）。
 * 返回 FLY_CATCH / FLY_CATCH_IF 时 gFly.thetaCatch / speedCatch 有效（对应本拍采样时刻）。 */
FlyVerdict_t FlyStart_DetectFast(int16_t cmdSign);

#endif
