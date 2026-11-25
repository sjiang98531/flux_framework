/* ============================================================================
 * @file    identify.h
 * @brief   电机参数辨识
 *
 * 目的：更换电机时不必手工测量参数，上电执行一次辨识得到 cfg_motor.h / cfg_board.h 中的主要参数，
 * 再由 param.c 自动算出全部增益。前提：辨识开始时转子静止（由操作者保证）。各项方法：
 *   R/L    ：α 轴正弦电压注入（不用电流环），幅值自适应，得到粗略 R、L，先整定电流环
 *   Rs、死区：定位轴上 d 轴多档直流电流，上半段（走出死区过渡带）的档位直线拟合：斜率 Rs，截距为死区电压
 *   Ld / Lq：d 轴、q 轴正弦电压注入（同 R/L 方法），Ld 与各档 Lq 共用一个频率（按 q 轴探测值与电压上限自适应）；
 *            q 轴按 0.5 / 1 / 1.5 / 2 倍额定电流测量，按描述函数直线拟合饱和模型 L(i) = L0 * (1 - per * |i|)
 *   J、psi_f（初值）：收尾定位时定位角阶跃 60°、关阻尼，转子在定位电流下自由摆动：由 q 轴反电势极值的间隔得单摆周期
 *            （加速度系数 = 120π / (p I T0^2)，与磁链无关），由极值的峰峰值得磁链（Ident_PendNote）
 *   psi_f  ：I/F 恒速段 mean|v - Ri - Vdt - jωLi| / ω（凸极按转子坐标），闭环稳速后再细测
 *   J      ：闭环中转速小幅升高再降回，iq = 负载(ω，二次式) + 斜坡加速度 / acc 最小二乘，负载与加速项分开（替换摆动测量值）
 * 因此 cfg_motor.h 只需铭牌数据（极对数、额定电流、额定转速、机械最高转速）：Rs / L / 饱和 / 磁链 / 加速度系数在辨识中
 * 测得前不被使用（填任意正值占位），测得后才生效。
 * 结果写入 gIdent 并交给 param.c，停机回到 Stop 时生效（仅存于 RAM）；长期使用需将 gParam 抄回配置文件。
 *
 * 使用（IDENT_TEST = 1，cfg_board.h）：给出转速指令即自动完成（gIdentCfg.mode = IDENT_MODE_AUTO，默认）：
 *   1. 静止测量：R/L -> 定位 -> 死区阶梯 -> Ld / Lq -> 收尾定位（摆动测量惯量与磁链，回到定位轴等停稳），约 30~90 s
 *      （等停稳按观测判断，转动惯量大、阻尼小的电机更久）；结果提交后自动停机、装入参数、重新起动；
 *   2. 运行测量：正常起动；I/F 到达目标转速时测磁链并立即生效，再切闭环；闭环稳速后转速升高约 15% 再降回，测加速度系数，
 *      随后按指令运行；gIdent.state = IDENT_DONE。
 *   只有保护性条件（测不出正的 R / L、参数被 param.c 拒绝）阻止提交与自动重起动（IDENT_WARN_AUTO_BLOCKED）。
 *   中途停机或故障：下次起动从未完成的步骤继续。重新辨识：停机后将 gIdent.state 写 0。
 * 只执行其中一步：IDENT_MODE_STATIC（仅静止测量，回到 Stop 时生效）/ IDENT_MODE_RUN（每次起动都在 I/F 中测磁链）。
 * ==========================================================================*/
#ifndef IDENTIFY_H
#define IDENTIFY_H
#include <stdint.h>

typedef enum {
    IDENT_IDLE = 0,
    IDENT_RUNNING,
    IDENT_DONE
} IdentState_TypeDef;

/* gIdent.warn：只有 AUTO_BLOCKED 阻止提交与自动重起动，其余为记录 */
#define IDENT_WARN_RL_TIMEOUT   (1u << 0)   /* R/L 粗测：电压到上限仍达不到可用电流，或未收敛（终止） */
#define IDENT_WARN_SWING_CLAMP  (1u << 1)   /* 辨识定位超时仍未停稳（startup.c，只记录） */
#define IDENT_WARN_L_FAIL       (1u << 3)   /* Ld / Lq 注入未得到正的电感（终止） */
#define IDENT_WARN_AUTO_BLOCKED (1u << 4)   /* 保护性条件或参数不合理，未提交 / 未自动重起动 */
#define IDENT_WARN_PSI_LOAD     (1u << 5)   /* 闭环细测磁链时负载电流超过额定（饱和使磁链偏小）：未采用，保留 I/F 粗测值 */
#define IDENT_WARN_PENDULUM     (1u << 6)   /* 摆动测量：各次周期相差超过 IDENT_PEND_SPREAD（齿槽 / 摩擦不可忽略），或没有有效摆动（只记录） */
#define IDENT_WARN_DT_UNSTABLE  (1u << 9)   /* 死区阶梯最高档未走出过渡带或 Rs 非正：Rs 与死区为 0、不提交 */

typedef struct {
    float rlR, rlL;     /* R/L 粗测（200 Hz，R 含死区偏大）：仅用于先整定电流环；-1 = 未测得 */
    float rsOhm;        /* 相电阻 -> MOTOR_RS_OHM */
    float ldH, lqH;     /* d 轴电感（0.5 倍额定电流幅值）/ q 轴零电流电感 L0（饱和拟合截距）-> MOTOR_LD_H / LQ_H */
    float lqAtA[4], lqAtH[4];   /* q 轴各档实际电流基波幅值（A）与基波电感（H），档数 lsN */
    float lsPerA, lsMaxA;   /* 饱和拟合：每 A 相对下降量（拐点取 0）-> MOTOR_LS_SAT_PER_A；最大测量电流（0 = 拟合不成立） */
    uint8_t lsN;
    float injHz;        /* Ld 与各档 Lq 共用的注入频率 */
    float pendT0;       /* 摆动测量：定位电流下的小摆幅单摆周期 s（0 = 未测得） */
    float pendPsiWb;    /* 摆动测量的磁链 Wb（反电势峰峰值，静止测量结束时生效，I/F 与闭环中再精测） */
    float psiWb;        /* 磁链（I/F 恒速段），测得后立即生效再切闭环 -> MOTOR_PSI_F_WB */
    float accRpmSPerA;  /* 加速度系数：摆动测量值（随测得的磁链修正凸极项），运行测量中由升降速实测替换 -> MOTOR_ACC_RPM_S_PER_A */
    float psiClWb;          /* 闭环细测的磁链 Wb（0 = 未测）；测得后替换 I/F 粗测值 psiWb */
    float jAccRef, jIqAcc;  /* 惯量测量：参考加速度 rpm/s、该加速度所需电流 A（最小二乘拟合）；acc = jAccRef / jIqAcc */
    float deadVoltV;    /* 死区等效电压（每相，测量时母线电压下） */
    float deadTimeS;    /* 等效死区时间 = deadVoltV / (PWM_FREQ * Vbus) -> INVERTER_DT_EFF_S */
    IdentState_TypeDef state;
    uint16_t warn;      /* 见 IDENT_WARN_* */
} IdentResult_TypeDef;

extern IdentResult_TypeDef gIdent;

#define IDENT_DT_N      11

/* 辨识电流：死区阶梯最高档 IDENT_DT_TOP_A（另至少 4 倍死区过渡带宽度，见 identify.c），注入幅值 IDENT_I_INJ_A；
 * 均不超过 IDENT_VEC_MAX_A = min(0.80 * I_MAX, 0.9 * BOARD_HOCP_A / 1.15)（预留电流环超调与硬件比较器裕量） */
#define IDENT_HOCP_LIM_A    (0.90 * BOARD_HOCP_A / 1.15)
#define IDENT_VEC_MAX_A     ((0.80 * I_MAX) < IDENT_HOCP_LIM_A ? (0.80 * I_MAX) : IDENT_HOCP_LIM_A)
#define IDENT_DT_TOP_A      (MOTOR_I_RATED_A / 0.75)
#define IDENT_I_INJ_A       (0.5 * MOTOR_I_RATED_A)

#define IDENT_MODE_AUTO     0       /* 静止测量 -> 自动重新起动 -> 运行测量 -> 按指令继续运行 */
#define IDENT_MODE_STATIC   1       /* 仅静止测量 */
#define IDENT_MODE_RUN      2       /* 正常起动，每次起动在 I/F 中测磁链 */

/* 静止测量阶段（gIdentTest.phase）。执行顺序：RL(9) -> 定位(0) -> DT_STEP(1) -> DT_ZERO(2) -> LDQ(6) -> HOLD(5，含摆动测量) -> DONE(4) */
typedef enum {
    IDENT_PH_NONE     = 0,          /* 未开始，或 R/L 粗测完成后正在定位（startup.c） */
    IDENT_PH_RL       = 9,          /* R/L 粗测（定位前） */
    IDENT_PH_DT_STEP  = 1,          /* 死区阶梯 */
    IDENT_PH_DT_ZERO  = 2,          /* 阶梯后电流回零 */
    IDENT_PH_LDQ      = 6,          /* d 轴、q 轴正弦注入测 Ld / Lq */
    IDENT_PH_HOLD     = 5,          /* 收尾定位：摆动测量惯量与磁链，回到定位轴等停稳（自动重起动时转子须静止且在已知角度） */
    IDENT_PH_DONE     = 4           /* 完成，结果已提交 param.c */
} IdentPhase_TypeDef;

typedef struct {
    int16_t  alignDeg;      /* 定位角（电角度，度），下一次起动生效 */
    uint8_t  mode;          /* IDENT_MODE_*，下一次起动生效 */
} IdentCfg_TypeDef;

typedef struct {
    volatile uint16_t phase;    /* 当前静止测量阶段 */
    uint16_t idx;               /* 死区档 / LDQ 轴（0 = d，1 = q） */
    uint16_t cnt;               /* 慢环计数 */
    int32_t  sum[2];            /* 死区平均：id, vd */
    int16_t  dtId[IDENT_DT_N], dtVd[IDENT_DT_N];
} IdentTest_TypeDef;

extern volatile IdentCfg_TypeDef gIdentCfg;
extern IdentTest_TypeDef gIdentTest;

int16_t Ident_AlignTheta(void);         /* 定位角（Q15 角度） */
uint8_t Ident_StaticThisStart(void);    /* 本次起动执行静止测量（进入 Align 时、定位结束时查询） */
uint8_t Ident_TakeRestart(void);        /* 自动方式：静止测量刚完成，需停机后重新起动（读取后清除） */
void    Ident_Reset(void);              /* 进 Align 时（静止测量） */
void    Ident_Idle(void);               /* 进 Align 时（不做静止测量，正常起动） */
void    Ident_Begin(void);              /* 定位完成时（代替切 I/F） */
/* 摆动测量（startup.c 收尾定位）：每步结束时记录 q 轴反电势极值（第一个 / 最后一个极值时刻 Q8 拍（从阶跃起算）、极值个数、
 * 前两个极值之差 Q15），全部完成并回到目标轴停稳后计算惯量与磁链并立即生效 */
void    Ident_PendNote(uint32_t firstQ8, uint32_t lastQ8, uint8_t n, int32_t ppQ15);
void    Ident_PendFinish(void);
uint8_t Ident_Active(void);             /* 静止测量已开始（慢环由 Ident_Slow 接管） */
void    Ident_Slow(void);
uint8_t Ident_Fast(void);               /* 1 = 本拍调制已由辨识输出（快环跳过电流环） */
/* I/F 慢环在切闭环判断前调用：返回 1 = 正在开环里测磁链，先不切 */
uint8_t Ident_OpenLoopFluxSlow(int32_t ol, int32_t tgt);
void    Ident_OpenLoopFluxRearm(void);      /* I/F 切闭环失败：下次拖动重新测磁链 */
uint8_t Ident_OpenLoopFluxPending(void);    /* 自动辨识运行段磁链尚未测得（openloop.c 冷起动用保守加速度） */
/* 惯量测量（自动辨识，闭环）：supervisor 的 Run 慢环每拍调用 Ident_RunSlow；测量期间 Ident_SpeedCmd 在指令上叠加小幅升速，
 * Ident_RampQ31 给出测量用斜坡加速度（0 = 不改，speed_ctrl.c） */
void    Ident_RunSlow(uint8_t spin);
int16_t Ident_SpeedCmd(int16_t cmd);
int32_t Ident_RampQ31(void);

#endif
