/* ============================================================================
 * cfg_motor.h —— 电机本体参数（物理单位）
 *
 * 换电机时先改这里，再按 重构说明.md 的"换电机流程"核对 cfg_ctrl.h。
 * 控制器里所有与电机有关的定点增益都在 fw_config.h 里由这些值自动算出。
 * 当前值 = test33 台架电机（5 对极高压风机，表贴式）。
 * ==========================================================================*/
#ifndef CFG_MOTOR_H
#define CFG_MOTOR_H

/* 0 = 表贴式（SPM，Ld = Lq）；1 = 内嵌式（IPM，Ld != Lq）。
 * 置 1 时观测器改用"有效磁链" psi_s - Lq*i，幅值目标 psi_f + (Ld-Lq)*id，
 * MTPA 才有意义。置 0 时 Ld/Lq 必须相等（init 时会检查，不等则置
 * gFluxDiag.cfgWarn 的 bit0）。 */
#define MOTOR_IPM               0

#define MOTOR_POLE_PAIRS        5.0             /* 极对数 */
#define MOTOR_RS_OHM            14.7            /* 相电阻 ohm */
#define MOTOR_LD_H              0.046           /* d 轴电感 H */
#define MOTOR_LQ_H              0.046           /* q 轴电感 H */

/* 转子磁链。铭牌 0.154 Wb，实测 0.141（test13：978 rpm 下 |e|/omega = 0.1409）。
 * 观测器的幅值目标、Startup 初值、PLL 鉴相增益都按它算。 */
#define MOTOR_PSI_F_WB          0.141f

/* 力矩电流 -> 加速度（rpm/s per A），由 test29 的 293->1172 rpm 斜坡辨识
 * （风阻与摩擦单独拟合，R^2 = 0.89）。它等价于转动惯量
 *   J = 1.5 * p * psi_f * 60 / (2*pi * MOTOR_ACC_RPM_S_PER_A) ≈ 0.020 kg·m²。
 * 速度环增益与加速度前馈都按它算。换电机后可以用 identify 模块（待实现）
 * 或一次阶跃响应重新辨识。 */
#define MOTOR_ACC_RPM_S_PER_A   501.0

/* 速度标幺基准：Q15 速度 32767 = SPEED_MAX rpm。也是 PLL 速度上限
 * （s32Speed 在 2^31 处饱和）。原厂名 SPEED_MAX，闭源库按这个编码。 */
#define SPEED_MAX               3200.0

/* 原厂老名字，Hardware/闭源库头文件之外仍有引用的保留别名 */
#define POLEPAIRS               MOTOR_POLE_PAIRS

#endif
