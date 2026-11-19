/* ============================================================================
 * @file    identify.h
 * @brief   电机参数辨识
 *
 * 目的：更换电机时无需手工测量参数，上电执行一次辨识即可得到 cfg_motor.h / cfg_board.h 中的主要参数，
 * 再由 cfg_derived.h 的公式自动得到全部增益。辨识方法（静止或空载下进行）：
 *   R/L    ：200 Hz 电压注入（不使用电流环），得到粗略 R、L，用于先整定电流环
 *   Rs     ：定位位置施加多档直流电流，由电压-电流斜率得到 Rs，截距为死区等效电压
 *   Ld/Lq  ：d 轴 / q 轴方向交流阻抗（自适应基频 f0，由 R/(2πL) 与死区裕量确定，最低 25 Hz），L = Im(Z)/ω；q 轴多档幅值拟合饱和模型 L(i)
 *   IPD    ：初始位置检测（高频注入 + 直流偏置），给出 N 极位置与极性对比度
 *   psi_f  ：I/F 开环中 |v - Ri - Vdt - jωLi| / ω，或零电流滑行时 |v|/ω
 *   J（b） ：零电流滑行减速度 / 滑行前 iq（= MOTOR_ACC_RPM_S_PER_A）
 * 结果写入 gIdent，并交给 param.c（见 param.h），停机回到 Stop 时生效（仅本次上电期间有效，存于 RAM）；
 * 长期使用需将 gParam 抄回 cfg_motor.h / cfg_board.h。
 * ==========================================================================*/
#ifndef IDENTIFY_H
#define IDENTIFY_H
#include <stdint.h>
#include "ipd.h"

typedef enum {
    IDENT_IDLE = 0,
    IDENT_RUNNING,
    IDENT_DONE
} IdentState_TypeDef;

/* gIdent.warn 诊断告警位（0 = 全项正常；非 0 表示存在保护限幅、延迟偏差或未通过自动重起动校验） */
#define IDENT_WARN_RL_TIMEOUT   (1u << 0)   /* R/L 粗测未在 200 周期内收敛或因电流不足放弃 */
#define IDENT_WARN_SWING_CLAMP  (1u << 1)   /* 转子摆动系数触发物理上界 0.35*L0*w0^2 夹限或高低频电流幅值偏差回退 */
#define IDENT_WARN_DLY_MISMATCH (1u << 2)   /* 三频点实测电压-电流延迟与 CTRL_INVPARK_ADVANCE 偏差 > 0.25 拍 */
#define IDENT_WARN_AC_DRIFT     (1u << 3)   /* 交流阻抗三频点 L/R 漂移、|Z| 交叉校核、饱和拟合残差或正交轴静止监测异常 */
#define IDENT_WARN_AUTO_BLOCKED (1u << 4)   /* IDENT_MODE_AUTO 静止结果未通过合理性检查，已阻止自动重起动 */
#define IDENT_WARN_LOW_SNR      (1u << 5)   /* 交流注入感抗压降相对死区等效电压不足（ωL·I / Vdt < 0.60） */
#define IDENT_WARN_AC_FAIL      (1u << 6)   /* 交流阻抗分段电流积分为零或瞬时过流保护终止，测量无效 */
#define IDENT_WARN_AMP_FAIL     (1u << 7)   /* 交流注入 2-DOF 幅值标定重测后仍未进入目标 ±25% */
#define IDENT_WARN_AMP_ORDER    (1u << 8)   /* q 轴饱和扫幅点数不足 3 档、相邻比 < 1.08 或总跨度 < 1.40（退化为仅 L0） */
#define IDENT_WARN_DT_UNSTABLE  (1u << 9)   /* 死区阶梯期间转子未静止（高电流档丢档 >= 2 或总丢档 >= 4）或拟合残差过大，未提交 Rs 与死区 */

typedef struct {
    float rlR, rlL;     /* 第一步 R/L（200 Hz 电压注入，不使用电流环）：仅用于先整定电流环（R 含死区偏大，L 偏小）；-1 = 未测得 */
    float rsOhm;        /* 相电阻（死区阶梯高电流段斜率）-> cfg_motor.h MOTOR_RS_OHM */
    float ldH, lqH;     /* 基频 f0（见 acHz[0]）电感：d 轴（0.4 A 缩放档）/ q 轴（饱和拟合的 L0，见 lsSat*）（相对 IPD 测得的 N 极）-> MOTOR_LD_H / LQ_H。
                         * 观测器只用 Lq：psi_s - Lq*i = (psi_f + (Ld - Lq) id)∠θ（有效磁链），任意 id 下方向均为转子 d 轴。
                         * 观测器需要运行频率下的电感；高频脉冲法受涡流影响测得值偏小，故采用自适应低频交流阻抗法 */
    float acHz[4];      /* 自适应交流阻抗实际注入频率（Hz）：[0..2] = f0 / f1 / f2（最高不超 PWM_FREQ/10），[3] = 0.5*f0（摆动配对） */
    float acLdH[3], acLqH[3];   /* acHz[0..2] 的 d、q 轴电感（相位法 Im(Z)/ω，反映频率依赖性） */
    float acLdMagH[3];  /* acHz[0..2] d 轴电感幅值校核 sqrt(|Z|^2 - rsOhm^2)/ω（对纯延迟不敏感） */
    float acRohm[3];    /* acHz[0..2] d 轴阻抗实部（已扣除死区模型与拟合延迟偏差，应接近 rsOhm） */
    float acDlyShift;   /* d 轴三频点估算的延迟偏差（PWM 拍，正值 = 实际延迟小于 CTRL_INVPARK_ADVANCE） */
    float acDlyTicks;   /* 实测电压-电流总延迟（PWM 拍 = CTRL_INVPARK_ADVANCE - bestSh），供台架标定板级延迟 */
    float snrMin;       /* 小信号基频段实测感抗压降与死区电压最小比值 min(|Im(Z)|*I / Vdt) */
    float lqAtA[5], lqAtH[5];   /* q 轴 f0 交流 0.4 / 1.0 / 1.5 / 2.0 / 2.5 A（随额定电流缩放）：实际电流幅值（A）、
                                 * 含 3 次谐波描述函数修正的等效割线电感 L_sec（H）；[1..4] 已扣偏置交叉饱和（2*lqAtHb1 - lqAtHb2，限幅 20%） */
    float lqAtHb1[4], lqAtHb2[4];   /* 1.0 ~ 2.5 A 四档在 N 极偏置 1 倍 / 2 倍时的电感（反映交叉饱和） */
    float kSwing[2], kSwingModel;   /* 转子摆动系数（H·(rad/s)^2）：acHz[3] / f0 实测值（0.4 A、1.0 A）与理论值 p*psi*acc*2π/60。
                                     * 实测远小于理论说明转子被卡住未摆动；f0 修正量 = K / (2π·f0)^2 */
    float lsSatKneeA, lsSatPerA;    /* 由 lqAt* 拟合的观测器饱和模型（拐点 / 每 A 下降量）-> MOTOR_LS_SAT_KNEE_A / PER_A；
                                     * 拟合同时给出 L0 写入 lqH（-> MOTOR_LQ_H）。无明显饱和时 PER_A = 0、lqH = 0.4 A 档的值 */
    float psiOpenLoopWb;      /* 转子磁链（运行辨识第一步：I/F 开环中 |v - Ri - Vdt - jωLi| / ω），测得后立即生效再切闭环 */
    float psiWb;        /* 转子磁链（运行辨识：零电流滑行 |v|/ω）-> MOTOR_PSI_F_WB */
    float accRpmSPerA;  /* 转矩电流 -> 加速度（运行辨识：滑行减速度 / 滑行前 iq）-> MOTOR_ACC_RPM_S_PER_A */
    float runRpm, runIqA, runIdA;   /* 运行辨识工况：滑行前的转速、iq、id */
    float deadVoltV;    /* 死区等效电压（每相，测量时的母线电压下） */
    float deadTimeS;    /* 等效死区时间 = deadVoltV / (PWM_FREQ * Vbus) -> cfg_board.h INVERTER_DT_EFF_S */
    float ipdRatio;     /* IPD 极性对比度：两侧 d 轴响应之比（> IPD_MIN_RATIO 才能使用 IPD 起动） */
    IdentState_TypeDef state;
    uint16_t warn;      /* 诊断告警位，见 IDENT_WARN_* */
} IdentResult_TypeDef;

extern IdentResult_TypeDef gIdent;

/* ============================================================================
 * 参数辨识模式（IDENT_TEST，cfg_board.h；默认 0 = 整段在编译期去除）
 *
 * 置 1 后给出一个转速指令即自动完成辨识（gIdentCfg.mode = IDENT_MODE_AUTO，默认）：
 *   1. 静止测量：Charge -> FlyCheck -> Align（定位到 gIdentCfg.alignDeg）-> R/L -> 死区阶梯 -> IPD -> 交流阻抗 / 饱和，
 *      转子不转，约 15 s；结果交给 param.c 后自动停机（Run -> Stop），参数装入（gParamStat.pending = 0）后自动重新起动；
 *   2. 运行测量：正常起动（IPD / 摆动起动 -> I/F），I/F 到达目标转速后先测磁链并生效，再切闭环；闭环稳速 2 s 后零电流滑行 0.4 s，
 *      测得 psi_f 与加速度系数并立即生效，随后按指令转速继续运行；gIdent.state = IDENT_DONE。
 *   运行测量要求转速不低于 RUN_MIN_RPM（按电机转速自适应，本电机 315 rpm，保证滑行 0.4 s 后仍高于 TRACK_END_RPM）：指令更低时，
 *   测量期间按其 1.1 倍（本电机 347 rpm）运行，测量完成后回到指令转速。
 *   中途停机或故障：下次起动从未完成的步骤继续（静止测量未完成则重做）。重新辨识：停机后将 gIdent.state 写 0。
 *   结果仅保存在 RAM（gParam），掉电丢失；长期使用需按 gIdent 填回 cfg_motor.h / cfg_board.h。
 * 只执行其中一步：gIdentCfg.mode = IDENT_MODE_STATIC（仅静止测量，完成后保持零电压，给指令 0 停机，回到 Stop 时生效）/
 * IDENT_MODE_RUN（正常起动，每个新的转速指令稳速后执行一次运行测量）。下一次起动生效。
 *
 * 静止测量各步（phase 为 gIdentTest.phase 的取值）：
 *
 * 1. R/L（phase 9）：进入 Align 时首先执行，200 Hz 电压注入、不使用电流环，得到粗略 R、L 并立即更新电流环增益；
 *    若超时放弃且未测出有效 R/L 则直接终止于 phase 4 并置 RL_TIMEOUT | AUTO_BLOCKED，不进入后续大电流阶段；
 *    死区阶梯测完后 Rs / 死区也立即生效。因此静止测量不依赖配置中的 R、L 初值（R/L -> Rs 顺序）。
 * 2. 死区阶梯（phase 1）：转子停在定位角，d 轴电流逐档保持（每档先稳定 300 ms，再平均 200 ms），
 *    记录稳态 vd / id。静止时 vd = Rs*id + 电压误差（死区 + 管压降），高电流段拟合直线：
 *    斜率 = 实际 Rs，截距 = 死区等效电压；低电流段反映误差过零特性。
 *    观测器使用的正是该指令电压，因此测得的即为观测器所见的电压误差。
 * 3. IPD（phase 3）：高频注入测凸极轴向、直流偏置判极性（ipd.h），测得的 N 极应落在 alignDeg 上。
 * 4. 交流阻抗（phase 6）：电流环正常闭环，角度固定在 N（d 轴）/ N+90°（q 轴），电流指令为 0.4 A 正弦（随额定电流缩放），
 *    自适应频率 f0 / f1 / f2（f0 按 R/(2πL) 与死区信噪比确定，>= 25 Hz、<= PWM_FREQ/20，f2 <= PWM_FREQ/10），
 *    各 4 个周期稳定 + 8 个周期同步积分电压与电流基波，Z = V/I，L = Im(Z)/ω；并校核感抗信噪比 snrMin 与板级延迟 acDlyTicks。
 *    死区误差与电流同相，主要进入实部、减小对 L 的干扰；q 轴注入产生交变转矩，转子几乎不动。
 * 5. q 轴饱和（phase 6 后段 + 7）：q 轴 f0 再测 1.0 / 1.5 / 2.0 / 2.5 A（缩放档），并配 0.5*f0 两段等幅重测（偏差 >5% 自动重标指令）测转子摆动系数；
 *    五档基波电感按观测器 L(i) 模型拟合（正弦电流下的基波 = 模型在一个周期上的加权平均，见 identify.c dfStep），
 *    给出 MOTOR_LQ_H / LS_SAT_KNEE_A / PER_A。静止结果需通过可信度门控才向 Param_Offer 提交 L / L(i) 并允许 AUTO 自动重起动，否则仅提交 Rs / 死区并置 AUTO_BLOCKED。
 * 6. 运行辨识：正常起动；I/F 开环拖到目标转速时先测磁链（Ident_OpenLoopFluxSlow），
 *    生效后再切闭环；闭环稳速后零电流滑行 420 ms，给出 psi_f 与 MOTOR_ACC_RPM_S_PER_A。
 * 结果见 gIdent（调试器查看）；IPD 结果见 gIpd。
 * ==========================================================================*/
#define IDENT_DT_N      11

/* 辨识电流按 MOTOR_I_RATED_A 缩放（本电机 0.75 A 时 IDENT_I_SCALE = 1.0，与原值逐位相同）；
 * 最大测试电流（2.5 * SCALE）不超过 SPD_PI_LIMIT，且 seg 15 合成矢量峰值（2.5 A 档 + 2 倍 0.625 A 偏置）
 * 不超过 IDENT_VEC_MAX_A = min(0.80 * I_MAX, 0.9 * BOARD_HOCP_A / 1.15)（预留 15% 电流环超调与 10% 硬件比较器裕量） */
#define IDENT_VEC_PEAK_K    2.795085    /* sqrt(2.5^2 + 1.25^2)：seg 15 幅值与 2 倍偏置的合成 */
#define IDENT_HOCP_LIM_A    (0.90 * BOARD_HOCP_A / 1.15)
#define IDENT_VEC_MAX_A     ((0.80 * I_MAX) < IDENT_HOCP_LIM_A ? (0.80 * I_MAX) : IDENT_HOCP_LIM_A)
#define IDENT_I_SCALE_RAW   (MOTOR_I_RATED_A / 0.75)
#define IDENT_I_SCALE_MAX   ((SPD_PI_LIMIT / 2.5) < (IDENT_VEC_MAX_A / IDENT_VEC_PEAK_K) ? \
                             (SPD_PI_LIMIT / 2.5) : (IDENT_VEC_MAX_A / IDENT_VEC_PEAK_K))
#define IDENT_I_SCALE       (IDENT_I_SCALE_RAW < IDENT_I_SCALE_MAX ? IDENT_I_SCALE_RAW : IDENT_I_SCALE_MAX)
#define IDENT_Q15_I(a)      Q15((a) * IDENT_I_SCALE / I_MAX)

#define IDENT_MODE_AUTO     0       /* 静止测量 -> 自动重新起动 -> 运行测量 -> 按指令继续运行 */
#define IDENT_MODE_STATIC   1       /* 仅静止测量 */
#define IDENT_MODE_RUN      2       /* 仅运行测量（每个新指令一次） */

/* 静止测量阶段编号（gIdentTest.phase 取值，按执行顺序排列；保留原编号以兼容 J-Scope 观测配置）：
 *   执行顺序：RL(9) -> 定位(0) -> DT_STEP(1) -> DT_ZERO(2) -> AC(6) -> SAT_FIT(7) -> IPD(3) -> DONE(4) */
typedef enum {
    IDENT_PH_NONE     = 0,          /* 未开始，或 R/L 粗测完成后正在正常定位 */
    IDENT_PH_RL       = 9,          /* 第 1 步：200 Hz 电压注入粗测 R/L（定位前，不使用电流环） */
    IDENT_PH_DT_STEP  = 1,          /* 第 2 步：定位完成后，d 轴 11 档直流电流阶梯测 Rs 与死区（含子窗静止监测） */
    IDENT_PH_DT_ZERO  = 2,          /* 第 3 步：死区阶梯后电流回零等待 */
    IDENT_PH_AC       = 6,          /* 第 4 步：d/q 轴 2-DOF 多频多幅值交流阻抗测量（含 3 次谐波与正交轴静止监测） */
    IDENT_PH_SAT_FIT  = 7,          /* 第 5 步：q 轴电感饱和 L(i) 精确描述函数网格拟合（分摊到慢环） */
    IDENT_PH_IPD      = 3,          /* 第 6 步：IPD 高频注入 + 直流偏置测 N 极角度与极性对比度（置于 AC 之后避免击飞转子） */
    IDENT_PH_DONE     = 4           /* 第 7 步：静止测量全部完成，结果已提交 param.c */
} IdentPhase_TypeDef;

typedef struct {
    int16_t  alignDeg;      /* 定位角（电角度，度），下一次起动生效。IPD 参数见 cfg_internal.h IPD_* */
    uint8_t  mode;          /* IDENT_MODE_AUTO（默认）/ STATIC / RUN，见上；下一次起动生效 */
    uint8_t  biasX;         /* 大幅值 q 段 N 极直流偏置的倍数（0 = 1 倍）：改 2 / 3 倍再测，看偏置（交叉饱和）对 Lq 的影响 */
} IdentCfg_TypeDef;

typedef struct {
    volatile uint16_t phase;    /* 当前静止测量阶段，取值见 IdentPhase_TypeDef */
    uint16_t idx;               /* 死区档 */
    uint16_t cnt;               /* 慢环计数 */
    int32_t  sum[2];            /* 死区平均：id, vd */
    int16_t  dtId[IDENT_DT_N], dtVd[IDENT_DT_N];
} IdentTest_TypeDef;

extern volatile IdentCfg_TypeDef gIdentCfg;
extern IdentTest_TypeDef gIdentTest;

int16_t Ident_AlignTheta(void);     /* 定位角（Q15 角度，65536 = 一圈） */
uint8_t Ident_StaticThisStart(void);    /* 本次起动执行静止测量（进入 Align 时、定位结束时查询） */
uint8_t Ident_RunActive(void);          /* 运行测量进行中（I/F 测磁链、闭环滑行；受控减速停机让位） */
uint8_t Ident_TakeRestart(void);        /* 自动方式：静止测量刚完成，需停机后重新起动（读取后清除） */
int16_t Ident_SpeedCmd(int16_t cmd);    /* 自动方式运行测量期间的转速指令（不低于 1.1 倍 RUN_MIN_RPM） */
void    Ident_Reset(void);          /* 进 Align 时（静止测量） */
void    Ident_Idle(void);           /* 进 Align 时（运行辨识：不做静止测量，正常起动） */
void    Ident_Begin(void);          /* 定位完成时（代替切 I/F） */
uint8_t Ident_Active(void);         /* 测量已开始（慢环由 Ident_Slow 接管） */
void    Ident_Slow(void);
uint8_t Ident_Fast(void);           /* 1 = 本拍调制已由测量输出（快环跳过电流环） */
/* 运行辨识：supervisor 的 Run 慢环开头调用；滑行期间 Ident_HoldTrack() = 1，零电流跟踪不接回速度环 */
void    Ident_RunSlow(uint8_t spin, uint8_t tracking, int16_t cmd, int16_t ramp);
uint8_t Ident_HoldTrack(void);
/* 运行辨识：I/F 慢环在"切闭环"判断前调用；返回 1 = 正在开环里测磁链，先不切 */
uint8_t Ident_OpenLoopFluxSlow(int32_t ol, int32_t tgt);

#endif
