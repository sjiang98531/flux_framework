/* ============================================================================
 * @file    identify.h
 * @brief   电机参数辨识
 *
 * 目的：更换电机时无需手工测量参数，上电执行一次辨识即可得到 cfg_motor.h / cfg_board.h 中的主要参数，
 * 再由 cfg_derived.h 的公式自动得到全部增益。辨识方法（静止或空载下进行）：
 *   R/L    ：200 Hz 电压注入（不使用电流环），得到粗略 R、L，用于先整定电流环
 *   Rs     ：定位位置施加多档直流电流，由电压-电流斜率得到 Rs，截距为死区等效电压
 *   Ld/Lq  ：d 轴 / q 轴方向低频（25 Hz）交流阻抗，L = Im(Z)/ω；q 轴多档幅值拟合饱和模型 L(i)
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

typedef struct {
    float rlR, rlL;     /* 第一步 R/L（200 Hz 电压注入，不使用电流环）：仅用于先整定电流环（R 含死区偏大，L 偏小）；-1 = 未测得 */
    float rsOhm;        /* 相电阻（死区阶梯高电流段斜率）-> cfg_motor.h MOTOR_RS_OHM */
    float ldH, lqH;     /* 低频（25 Hz）电感：d 轴（0.4 A）/ q 轴（饱和拟合的 L0，见 lsSat*）（相对 IPD 测得的 N 极）-> MOTOR_LD_H / LQ_H。
                         * 观测器只用 Lq：psi_s - Lq*i = (psi_f + (Ld - Lq) id)∠θ（有效磁链），任意 id 下方向均为转子 d 轴。
                         * 观测器需要运行频率下的电感；高频脉冲法受涡流影响测得值偏小，故采用低频交流阻抗法 */
    float acLdH[3], acLqH[3];   /* 25 / 50 / 100 Hz 的 d、q 轴电感（反映频率依赖性） */
    float acRohm[3];    /* 25 / 50 / 100 Hz d 轴阻抗实部（已扣除死区模型，应接近 rsOhm，用于校核死区模型） */
    float lqAtA[5], lqAtH[5];   /* q 轴 25 Hz 交流 0.4 / 1.0 / 1.5 / 2.0 / 2.5 A：实际电流幅值（A）、基波电感（H，已含摆动修正；
                                 * [1..4] 为偏置外推到 0 的值 = 2 * lqAtHb1 - lqAtHb2） */
    float lqAtHb1[4], lqAtHb2[4];   /* 1.0 ~ 2.5 A 四档在 N 极偏置 1 倍 / 2 倍时的电感（反映交叉饱和） */
    float kSwing[2], kSwingModel;   /* 转子摆动系数（H·(rad/s)^2）：12.5 / 25 Hz 实测值（0.4 A、1.0 A）与理论值 p*psi*acc*2π/60。
                                     * 实测远小于理论说明转子被卡住未摆动；25 Hz 修正量 = K / (2π·25)^2 */
    float lsSatKneeA, lsSatPerA;    /* 由 lqAt* 拟合的观测器饱和模型（拐点 / 每 A 下降量）-> MOTOR_LS_SAT_KNEE_A / PER_A；
                                     * 拟合同时给出 L0 写入 lqH（-> MOTOR_LQ_H）。无明显饱和时 PER_A = 0、lqH = 0.4 A 时的值 */
    float psiOpenLoopWb;      /* 转子磁链（运行辨识第一步：I/F 开环中 |v - Ri - Vdt - jωLi| / ω），测得后立即生效再切闭环 */
    float psiWb;        /* 转子磁链（运行辨识：零电流滑行 |v|/ω）-> MOTOR_PSI_F_WB */
    float accRpmSPerA;  /* 转矩电流 -> 加速度（运行辨识：滑行减速度 / 滑行前 iq）-> MOTOR_ACC_RPM_S_PER_A */
    float runRpm, runIqA, runIdA;   /* 运行辨识工况：滑行前的转速、iq、id */
    float deadVoltV;    /* 死区等效电压（每相，测量时的母线电压下） */
    float deadTimeS;    /* 等效死区时间 = deadVoltV / (PWM_FREQ * Vbus) -> cfg_board.h INVERTER_DT_EFF_S */
    float ipdRatio;     /* IPD 极性对比度：两侧 d 轴响应之比（> IPD_MIN_RATIO 才能使用 IPD 起动） */
    IdentState_TypeDef state;
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
 *    死区阶梯测完后 Rs / 死区也立即生效。因此静止测量不依赖配置中的 R、L 初值（R/L -> Rs 顺序）。
 * 2. 死区阶梯（phase 1）：转子停在定位角，d 轴电流逐档保持（每档先稳定 300 ms，再平均 200 ms），
 *    记录稳态 vd / id。静止时 vd = Rs*id + 电压误差（死区 + 管压降），高电流段拟合直线：
 *    斜率 = 实际 Rs，截距 = 死区等效电压；低电流段反映误差过零特性。
 *    观测器使用的正是该指令电压，因此测得的即为观测器所见的电压误差。
 * 3. IPD（phase 3）：高频注入测凸极轴向、直流偏置判极性（ipd.h），测得的 N 极应落在 alignDeg 上。
 * 4. 交流阻抗（phase 6）：电流环正常闭环，角度固定在 N（d 轴）/ N+90°（q 轴），电流指令为 0.4 A 正弦，
 *    25 / 50 / 100 Hz，各 4 个周期稳定 + 8 个周期同步积分电压与电流基波，Z = V/I，L = Im(Z)/ω。
 *    在运行频率附近测量，不受涡流影响；死区误差与电流同相，只进入实部、不影响 L。
 *    q 轴注入产生的是数十 Hz 的交变转矩，转子几乎不动。
 * 5. q 轴饱和（phase 6 后段 + 7）：q 轴 25 Hz 再测 1.0 / 1.5 / 2.0 / 2.5 A，最后 12.5 Hz 0.4 / 1.0 A 两段测转子摆动系数；
 *    五档基波电感按观测器 L(i) 模型拟合（正弦电流下的基波 = 模型在一个周期上的加权平均，见 identify.c dfStep），
 *    给出 MOTOR_LQ_H / LS_SAT_KNEE_A / PER_A。
 * 6. 运行辨识：正常起动；I/F 开环拖到目标转速时先测磁链（Ident_OpenLoopFluxSlow），
 *    生效后再切闭环；闭环稳速后零电流滑行 420 ms，给出 psi_f 与 MOTOR_ACC_RPM_S_PER_A。
 * 结果见 gIdent（调试器查看）；IPD 结果见 gIpd。
 * ==========================================================================*/
#define IDENT_DT_N      11

#define IDENT_MODE_AUTO     0       /* 静止测量 -> 自动重新起动 -> 运行测量 -> 按指令继续运行 */
#define IDENT_MODE_STATIC   1       /* 仅静止测量 */
#define IDENT_MODE_RUN      2       /* 仅运行测量（每个新指令一次） */

typedef struct {
    int16_t  alignDeg;      /* 定位角（电角度，度），下一次起动生效。IPD 参数见 cfg_internal.h IPD_* */
    uint8_t  mode;          /* IDENT_MODE_AUTO（默认）/ STATIC / RUN，见上；下一次起动生效 */
    uint8_t  biasX;         /* 大幅值 q 段 N 极直流偏置的倍数（0 = 1 倍）：改 2 / 3 倍再测，看偏置（交叉饱和）对 Lq 的影响 */
} IdentCfg_TypeDef;

typedef struct {
    volatile uint16_t phase;    /* 0 未开始，1 死区阶梯，2 回零，3 IPD（快环），6 交流阻抗（快环），7 饱和拟合，9 R/L（定位前），4 完成 */
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
