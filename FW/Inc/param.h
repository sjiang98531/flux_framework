/* ============================================================================
 * @file    param.h
 * @brief   运行时电机参数：辨识完成后自动生效（仅存于 RAM，掉电后恢复配置标称值）
 *
 * 设计目的：参数辨识（IDENT_TEST，identify.c）测得 Rs、L 等参数后，无需重新编译即可按新参数运行，
 * 因此将与电机参数相关的增益改为运行时计算：参数（物理量）-> 增益（定点数）。
 *
 * 全局量：
 *   gParam     当前使用的电机参数，物理单位（Ω、H、Wb……），double。
 *   gGain      由 gParam 计算出的各模块定点增益。各模块（电流环、速度环、观测器、PLL……）运行时读取此处，
 *              而不直接使用 cfg_derived.h 中的常量。
 *   gParamStat 参数更新状态。
 *
 * 公式唯一：gGain 的每一项均使用 cfg_derived.h 中带 _OF 后缀的宏计算（如 CURRENT_KP_OF(L)），
 * 编译期常量 CURRENT_KP_D 即 CURRENT_KP_OF(MOTOR_LD_H)。因此参数等于配置标称值时，运行时增益与
 * 编译期常量逐位相同。
 *
 * 仅可辨识的量走运行时路径：Rs、Ld/Lq、L(i) 饱和、psi_f、加速度系数（转动惯量）、等效死区。
 * 极对数、额定值、各转速门限（*_RPM）仍使用配置值，更换电机时按 cfg_motor.h / cfg_app.h 修改。
 *
 * 流程：
 *   上电          Param_Init：按配置标称值计算 gGain（须在各模块 Init 之前，各模块 Init 需读取 gGain）。
 *   辨识过程中    Param_ApplyNow：测得 R/L 后立即生效，后续测量使用新的电流环增益（仅在辨识模式中使用）。
 *   辨识结束      Param_Offer：提交新参数，逐项检查合理性，不合理的项保留原值，待回到 Stop 后更新。
 *   主循环        Param_Background：存在待更新参数且电机处于 Stop（PWM 关闭）时，计算增益并整体更新。
 *   本次上电期间即按新参数运行；如需长期使用，将 gParam 的值抄回 cfg_motor.h / cfg_board.h（不写 Flash）。
 * ==========================================================================*/
#ifndef PARAM_H
#define PARAM_H
#include <stdint.h>

/* 电机参数（物理单位）。每项对应配置中的一个标称宏，上电时从配置复制 */
typedef struct {
    double rsOhm;           /* 相电阻 Ω（MOTOR_RS_OHM） */
    double ldH, lqH;        /* d / q 轴电感 H（MOTOR_LD_H / LQ_H；表贴式 SPM 两者相同） */
    double lsKneeA, lsPerA; /* 电感饱和：拐点电流 A、拐点后每安培电感下降的比例（MOTOR_LS_SAT_KNEE_A / PER_A） */
    double psiWb;           /* 永磁磁链 Wb（MOTOR_PSI_F_WB），决定反电势 = psi * 电角速度 */
    double accRpmSPerA;     /* 每安培 q 轴电流产生的加速度 rpm/s（MOTOR_ACC_RPM_S_PER_A）= 转矩常数 / 转动惯量 */
    double dtEffS;          /* 逆变器等效死区时间 s（INVERTER_DT_EFF_S），观测器死区补偿用 */
} MotorParam_TypeDef;

/* 由 gParam 计算的增益（computeGain 写入，各模块读取）。量纲与公式见 cfg_derived.h 中同名的 *_OF 宏 */
typedef struct {
    /* ---- 电流环 PI（foc_core.c）：Kp = 2*pi*带宽*L，Ki = 2*pi*带宽*R / PWM 频率（零极点对消），已封顶 65535 */
    int32_t curKpD, curKiD, curKpQ, curKiQ;
    /* ---- 速度环 PI（speed_ctrl.c）：按 SPEED_WN_HZ / SPEED_ZETA 和加速度系数算；
     *      spdFfKQ16 = 轨迹加速度 -> 前馈 iq 的系数（Q16）；spdAccPerIqQ8 = 其倒数：iq -> 每拍斜坡增量（Q8） */
    int32_t spdKp, spdKi, spdFfKQ16, spdAccPerIqQ8;
    /* ---- dq 解耦前馈（foc_core.c）：反电势 w*psi（Q15）、交叉耦合 w*Lq*iq、w*Ld*id（Q12，系数可能 > 1） */
    int32_t decKEmfQ15, decKLqQ12, decKLdQ12;
    /* ---- 磁链观测器（flux_obs.c） */
    int16_t fluxNomQ15;     /* psi_f 在观测器磁链单位里的值（Q15），NLFO 以其为幅值目标、PLL 增益按其归一化 */
    int32_t fluxNom2;       /* fluxNomQ15 的平方，NLFO 比较 |psi|^2 用（快环不开方） */
    int16_t nlfoNormQ8;     /* NLFO 修正增益的归一化系数（Q8），把时间常数做成 FLUX_NLFO_TAU_S */
    /* fluxKrsQ15  Rs：Q15 电流 -> 观测器 Q15 电压（电阻压降 Rs*i）
     * fluxKlQ15   Lq：Q15 电流 -> 观测器 Q15 磁链（L*i，从定子磁链中减去得到转子磁链）
     * focKrsQ15   Rs：Q15 电流 -> 电流环 Q15 电压（定位时从 q 轴电压中扣除电阻压降，用于判静止）
     * fluxKdlQ15  Ld - Lq（内嵌式 IPM 的有效磁链；表贴式为 0） */
    int16_t fluxKrsQ15, fluxKlQ15, focKrsQ15, fluxKdlQ15;
    int16_t lsKneeQ15;      /* 电感饱和拐点（Q15 电流） */
    int32_t lsSatKQ12;      /* 拐点后电感下降斜率（Q12，每 Q15 满量程电流）；0 = 不饱和 */
    int16_t dtKtQ15;        /* 死区补偿：每相电压误差 = 母线 * dtKt（Q15 = 等效死区 / PWM 周期）；0 = 不补偿 */
    /* ---- PLL（flux_pll.c）：按 FLUXPLL_WN_HZ / ZETA 算，再除以 fluxNomQ15（鉴相器输出和磁链幅值成正比）；
     *      int16 放不下时封顶 32767 并置 cfgWarn bit2 */
    int16_t pllKp, pllKi;
    /* ---- 其他模块 */
    int16_t ipdPulseVQ15;       /* IPD 脉冲电压（Q15）：按目标峰值电流和 Ld 反算，ipd.c */
    int32_t flyWlrQ12;          /* 飞车：w*Lq/Rs 在满量程转速下的值（Q12），算短路电流的阻抗角 atan(w*L/R)，flystart.c */
    uint16_t flySettleBlocks;   /* 飞车：短接后等电流暂态衰减的时间（ms）= FLY_SETTLE_TAUS 个 L/R 时间常数 */
    int16_t openLoopAccFfIq;    /* I/F 开环：按 OPENLOOP_ACC_RPM_S 加速需要的 iq（Q15），startup.c 的切换前馈 */
    int32_t alignStillEmfQ15;   /* 定位：转速 ALIGN_STILL_RPM 对应的反电势（Q15），q 轴电压扣除 Rs*i 后低于该值判为静止 */
    uint16_t cfgWarn;           /* 增益超出定点范围被封顶：bit1 电流环，bit2 PLL，bit3 反电势前馈超量程 */
} CtrlGain_TypeDef;

/* 参数更新状态 */
typedef struct {
    uint8_t  source;        /* 0 = 配置标称值，2 = 本次辨识值 */
    uint8_t  pending;       /* 1 = 有辨识结果等待回到 Stop 后更新 */
    uint16_t rejectMask;    /* 最近一次 Param_Offer 被拒绝的项：bit0 Rs，1 L，2 L(i)，3 psi，4 acc，5 死区 */
    uint16_t applyCount;    /* 累计更新次数（含 Param_ApplyNow） */
} ParamStat_TypeDef;

extern MotorParam_TypeDef gParam;
extern CtrlGain_TypeDef gGain;
extern ParamStat_TypeDef gParamStat;

void Param_Init(void);                              /* 上电：按配置标称值计算 gGain。须在各模块 Init 之前调用（Ctrl_Init） */
void Param_Defaults(MotorParam_TypeDef *p);         /* 将配置标称值填入 p */
void Param_Offer(const MotorParam_TypeDef *p, uint16_t validMask);  /* 辨识结束时提交结果（validMask 位定义同 rejectMask，标记哪些项有新值），回到 Stop 后才更新 */
uint8_t Param_ApplyNow(const MotorParam_TypeDef *p);  /* 辨识模式：辨识过程中立即生效（慢环中调用），参数不合理时返回 0 */
void Param_Background(void);                        /* 主循环：Stop 状态（PWM 关闭）时更新 Param_Offer 提交的参数 */

#endif
