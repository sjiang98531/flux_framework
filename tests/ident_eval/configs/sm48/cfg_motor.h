/* ============================================================================
 * @file    cfg_motor.h
 * @brief   电机本体参数（物理单位）
 *
 * 配置三层中的"电机"层（另两层：cfg_app.h 应用、cfg_internal.h 内部）。更换电机时修改本文件，
 * 控制器中所有与电机相关的定点增益均由 cfg_derived.h 根据这些值自动计算。
 * 必填：极对数、铭牌额定电流 / 转速 / 功率、Rs、Ld / Lq、磁链、加速度系数（后五项可先填估计值，
 *       用参数辨识模式 cfg_board.h IDENT_TEST = 1 测量后回填，见 identify.h）。
 * 可选：电感饱和 MOTOR_LS_SAT_*（未知时 PER_A = 0 关闭）、机械允许转速 MOTOR_SPEED_LIMIT_RPM。
 * 当前值对应 5 对极高压风机电机（表贴式）。
 * ==========================================================================*/
#ifndef CFG_MOTOR_H
#define CFG_MOTOR_H

#define MOTOR_POLE_PAIRS        4.0             /* 极对数 */
/* 相电阻（星形接法，相电阻 = 线电阻 / 2）。 */
#define MOTOR_RS_OHM            0.12            /* 相电阻 ohm（线电阻 14.7 / 2） */
/* 电感（参数辨识模式测得，25 Hz 交流阻抗法，见 identify.h）。按实测分别填写：Ld != Lq 即凸极，观测器、电流环、
 * 解耦、MTPA 均直接按 Ld / Lq 计算，Ld = Lq 时各式自然退化为表贴式，不另设电机类型。
 *   Lq：多档电流幅值下的基波电感，外推到零偏置后按 L(i) 模型拟合：L0 = 43.3 mH、拐点 0、0.094/A。
 *   Ld：按仿真电机填写。 */
#define MOTOR_LD_H              120e-6          /* d 轴电感 H */
#define MOTOR_LQ_H              120e-6          /* q 轴电感 H */

/* 电感饱和（观测器使用，割线电感 L(i) = psi/i）。|i| <= 拐点时 L = MOTOR_LQ_H；超过后每 A 下降
 * MOTOR_LS_SAT_PER_A（相对值），最低为 MOTOR_LS_SAT_MIN 倍（cfg_internal.h）。PER_A = 0 时关闭（编译期去除，无开销）。 */
#define MOTOR_LS_SAT_KNEE_A     0.0             /* A，开始饱和的电流（峰值） */
#define MOTOR_LS_SAT_PER_A      0.0           /* 1/A，超过拐点后每 A 的相对下降量 */

/* 转子永磁磁链（铭牌 0.154 Wb，实测 0.141 Wb）。
 * 观测器幅值目标、OpenLoop 初值、PLL 鉴相增益均按此值计算。 */
#define MOTOR_PSI_F_WB          0.0045f

/* 转矩电流 -> 加速度（rpm/s per A），由转速斜坡响应辨识（风阻与摩擦单独拟合）。等价于转动惯量
 *   J = 1.5 * p * psi_f * 60 / (2*pi * MOTOR_ACC_RPM_S_PER_A) ≈ 0.020 kg·m²。
 * 速度环增益与加速度前馈均按此值计算。更换电机后可通过参数辨识或一次阶跃响应重新测定。 */
#define MOTOR_ACC_RPM_S_PER_A   4000.0
/* 铭牌：DC 310 V、100 W、0.75 A、1100 rpm。
 * 0.75 A 按相电流峰值理解（与 iq 同一口径）：1.5*5*0.154*0.75 = 0.866 N·m，乘 1100 rpm 约 99.8 W。
 * cfg_app.h / cfg_internal.h 中的电流类门限均以其倍数表示，更换电机时自动缩放。 */
extern double g_sim_irated_k;
#ifdef IN_CFG_CHECK_C
#define MOTOR_I_RATED_A 6.0
#else
#define MOTOR_I_RATED_A ((6.0) * g_sim_irated_k)
#endif            /* A，额定相电流（峰值） */
#define MOTOR_SPEED_RATED_RPM   6000.0          /* rpm，额定转速 */
#define MOTOR_P_RATED_W         200.0           /* W，额定功率 */
/* 机械允许最高转速（叶轮 / 轴承规格）。超速保护取它与"反电势达到母线兜底门限"转速中的较小值（cfg_internal.h）。 */
#define MOTOR_SPEED_LIMIT_RPM   12000.0          /* rpm */

#endif
