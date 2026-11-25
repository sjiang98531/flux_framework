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
    double lsMaxA;          /* 饱和模型有效上限电流 A（0 = 不钳位，辨识时取实际扫到的最大电流） */
    double psiWb;           /* 永磁磁链 Wb（MOTOR_PSI_F_WB），决定反电势 = psi * 电角速度 */
    double accRpmSPerA;     /* 每安培 q 轴电流产生的加速度 rpm/s（MOTOR_ACC_RPM_S_PER_A）= 转矩常数 / 转动惯量 */
    double dtEffS;          /* 逆变器等效死区时间 s（INVERTER_DT_EFF_S），观测器死区补偿用 */
} MotorParam_TypeDef;

/* 由 gParam 计算的增益（computeGain 写入，各模块读取）。量纲与公式见 cfg_derived.h 中同名的 *_OF 宏 */
typedef struct {
    /* ---- 电流环 PI（foc_core.c）：Kp = 2*pi*带宽*L，Ki = 2*pi*带宽*R / PWM 频率（零极点对消），已封顶 65535 */
    int32_t curKpD, curKiD, curKpQ, curKiQ;
    /* ---- 速度环与母线回馈控制（speed_ctrl.c） */
    int32_t spdKp, spdKi, spdFfKQ16, spdAccPerIqQ8;   /* 速度 PI、加速度前馈（Q16）及其倒数（Q8） */
    int32_t rampOpenLoopQ31, rampOpenLoopRevQ31;       /* I/F 开环 / 反转过零加速度对应的慢环斜坡步长（Q31） */
    int16_t busRegenStartQ15, busRegenFullQ15;         /* 母线回馈兜底起始 / 满限（Q15） */
    int16_t busRegenIqMaxQ15;                          /* 母线回馈方向允许的最大 iq（Q15） */
    int32_t busRegenKQ12;                              /* 母线回馈兜底削顶斜率（Q12） */
    int32_t busVdcRefSq;                               /* 母线电压控制器调节点平方（Q30） */
    int32_t busVdcKQ8;                                 /* 母线电压控制器增益（Q8） */
    /* ---- dq 解耦前馈（foc_core.c）：反电势 w*psi（Q15）、交叉耦合 w*Lq*iq、w*Ld*id（Q12，系数可能 > 1） */
    int32_t decKEmfQ15, decKLqQ12, decKLdQ12;
    /* ---- 磁链观测器与锁相环（flux_obs.c） */
    int16_t fluxNomQ15;     /* psi_f 在观测器磁链单位里的值（Q15），NLFO 以其为幅值目标、PLL 增益按其归一化 */
    int32_t fluxNom2;       /* fluxNomQ15 的平方，NLFO 比较 |psi|^2 用（快环不开方） */
    int32_t nlfoNormQ8;     /* NLFO 修正增益的归一化系数（Q8），把时间常数做成 FLUX_NLFO_TAU_S（低磁链、大电感电机可超过 16 位） */
    int16_t fluxKtsQ15;     /* 观测器积分步长 KTs（Q15，随动态磁链基准变化） */
    /* fluxKrsQ15  Rs：Q15 电流 -> 观测器 Q15 电压（电阻压降 Rs*i）
     * fluxKlQ15   Lq：Q15 电流 -> 观测器 Q15 磁链（L*i，从定子磁链中减去得到转子磁链）
     * focKrsQ15   Rs：Q15 电流 -> 电流环 Q15 电压（辨识定位时从 q 轴电压中扣除电阻压降）
     * fluxKdlQ15  Ld - Lq（内嵌式 IPM 的有效磁链与 MTPA；表贴式为 0） */
    int16_t fluxKrsQ15, fluxKlQ15, focKrsQ15, fluxKdlQ15;
    int16_t lsKneeQ15;      /* 电感饱和拐点（Q15 电流） */
    int16_t lsMaxQ15;       /* 相对拐点允许的最大超额电流 |i|-Ik（Q15 电流），超出后电感保持该处值；32767 = 不钳位 */
    int32_t lsSatKQ12;      /* 拐点后电感下降斜率（Q12，每 Q15 满量程电流）；0 = 不饱和 */
    int16_t dtKtQ15;        /* 死区补偿：每相电压误差 = 母线 * dtKt（Q15 = 等效死区 / PWM 周期）；0 = 不补偿 */
    int16_t pllKp, pllKi;   /* PLL（flux_obs.c）：按 FLUXPLL_WN_HZ / ZETA 算并除以 fluxNomQ15；封顶 32767 时置 cfgWarn bit2 */
    /* ---- 起动与 I/F 开环（ipd.c / startup.c / openloop.c / supervisor.c） */
    int16_t ipdHfVQ15;          /* IPD 注入电压（Q15）：按注入电流幅值和 Lq 计算，ipd.c */
    uint16_t alignPeriodTicks;  /* 定位单摆周期（慢环拍） */
    int16_t alignSwingMinSpd;   /* 摆动起动最小峰值转速（Q15） */
    int32_t alignDampQ8;        /* 定位阻尼增益（Q8） */
    int32_t alignStillEmfQ15;   /* 定位：转速 ALIGN_STILL_RPM 对应的反电势（Q15），低于该值判为静止 */
    int32_t swingLagAngQ12;     /* 摆动起动反电势测量延迟角度系数（Q12） */
    int32_t openLoopAccStepQ31; /* I/F 开环每快环拍转速步长（Q31） */
    int16_t openLoopAccFfIq;    /* I/F 开环：按 OPENLOOP_ACC_RPM_S_OF 加速需要的 iq（Q15），切换前馈 */
    int32_t currentSlewStepQ15; /* 软停机 / 电流过渡每慢环拍步长（Q15） */
    /* ---- 飞车检测（flystart.c） */
    int32_t flyWlrQ12;          /* 满量程转速下 w*Lq/Rs（Q12），算短路电流阻抗角 atan(w*L/R) */
    uint16_t flySettleBlocks;   /* 短接后等电流暂态衰减的时间（ms）= FLY_SETTLE_TAUS 个 L/R 时间常数 */
    int16_t flyBemfAmpTh;       /* 判"在转"短路电流门限（Q15） */
    int16_t flyEndAmpTh;        /* 残余转速对应短路电流门限（Q15） */
    uint16_t flyStillSpinMs;    /* 测到转动后等待停稳时间（ms） */
    uint32_t flyMayMoveExpireMs;/* 停机后跳过最坏情况等待的过期时间（ms） */
    /* ---- 观测器可靠性相关的转速门限（openloop.c / spin.c / speed_ctrl.c / supervisor.c / flystart.c / identify.c）
     * cfg_app.h 中以 rpm 给出，其本义是"反电势占额定母线的比例 k"（观测器是否可靠取决于反电势相对逆变器电压误差的大小），
     * 即 n = k * UDC_RATE * 60 / (2*pi * 极对数 * psi_f)：运行时按 MOTOR_PSI_F_WB / gParam.psiWb 换算，磁链测得与配置不同时随之调整。
     * 磁链等于配置值时与原编译期常数逐位相同。比例限制在 [0.25, 4]，换算后不超过 0.9 * SPEED_MAX。
     * 机械意义上的"静止"门限（FLY_STILL_*、FLY_LOW_MIN、ALIGN_STILL）与观测器去直流补偿起点（FLUX_HPF_COMP_MIN）不随之变化 */
    int32_t spdHandoverQ31;     /* OPENLOOP_HANDOVER_RPM：I/F 切入闭环 */
    int32_t spdFallbackQ31;     /* OPENLOOP_FALLBACK_RPM：闭环交还 I/F */
    int32_t spdWarmMaxQ31;      /* 1.5 * OPENLOOP_FALLBACK_RPM：回退进 I/F 时开环转速上限 */
    int16_t spdFallbackQ15;
    int16_t spdSpinMinQ15;      /* OPENLOOP_SPIN_MIN_RPM：闭环最低指令转速 */
    int16_t spdLostQ15;         /* SPIN_LOST_RPM：闭环失锁判据 */
    int16_t spdTrackEndQ15;     /* TRACK_END_RPM：零电流跟踪 / 受控减速结束 */
    int16_t spdFlyCatchQ15;     /* FLY_CATCH_RPM：飞车同向直接接入闭环下限 */
    int16_t spdFlyRevMinQ15;    /* FLY_REV_SPIN_MIN_RPM：飞车反向直接接入闭环下限 */
    /* ---- 保护与告警（protect.c / diag.c） */
    int32_t protectSpeedOcQ15;  /* 超速保护门限（Q15） */
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
void Param_Offer(const MotorParam_TypeDef *p, uint16_t validMask);  /* 辨识结束时提交结果（validMask 位定义同 rejectMask，标记哪些项有新值），回到 Stop 后才更新 */
uint8_t Param_ApplyNow(const MotorParam_TypeDef *p);  /* 辨识模式：辨识过程中立即生效（慢环中调用），参数不合理时返回 0 */
void Param_Background(void);                        /* 主循环：Stop 状态（PWM 关闭）时更新 Param_Offer 提交的参数 */
float Param_LqAt(float iAbsA);                      /* 按当前 gParam 的 L(|i|) 饱和模型（含 lsMaxA 钳位）计算电感 H */

#endif
