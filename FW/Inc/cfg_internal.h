/* ============================================================================
 * @file    cfg_internal.h
 * @brief   内部控制规格（物理单位），正常使用无需修改
 *
 * 配置分三层：电机 cfg_motor.h、应用 cfg_app.h、内部（本文件），另有驱动板 cfg_board.h（见 cfg_app.h 文件头）。
 * 本文件描述"期望的控制效果"：带宽（Hz）、阻尼比、时间常数（s）、额定值的倍数，全部由电机参数、驱动板参数
 * 和应用层配置自动换算；定点增益由 cfg_derived.h 计算。更换电机或应用时无需修改本文件。
 * 已移至 cfg_app.h 的宏在原位置保留一行"见 cfg_app.h"，其原理说明仍在本文件。
 *
 * 参数分三类：
 *   A 类  物理输入：实测值或器件手册值（主要在 cfg_motor.h / cfg_board.h）；
 *   B 类  设计指标及其推导量：带宽、阻尼、时间常数，以及由 A/B 类推出的量；
 *   C 类  经验值：暂无物理推导，已在对应注释中标明（POWER_KP / POWER_KI、PRECHARGE_TIME）。
 * ==========================================================================*/
#ifndef CFG_INTERNAL_H
#define CFG_INTERNAL_H

/* ===================== 定标 ============================================== */
/* 速度标幺基准：Q15 速度 32767 对应 SPEED_MAX rpm，同时也是 PLL 速度上限（s32Speed 在 2^31 处饱和）。
 * 仅为定标量，须高于超速保护转速（配置检查 SPEED_OC_FITS）。 */
#define SPEED_MAX               3200.0
/* 极对数别名（Board/ 下的板级代码使用） */
#define POLEPAIRS               MOTOR_POLE_PAIRS
/* 电感饱和模型的下限（相对 MOTOR_LQ_H）：割线电感 L(i) 最低降到该倍数（拐点与斜率为电机参数，见 cfg_motor.h） */
#define MOTOR_LS_SAT_MIN        0.5

/* ===================== 转速门限 ========================================== */
/* 所有转速门限均直接以 rpm（机械转速）给出。
 * 更换电机或母线电压时的估算方法：磁链观测器在低速是否可靠，取决于反电势 E = w_e*psi_f 相对逆变器
 * 电压误差（死区，约 母线电压 x 死区时间 x PWM 频率，与母线电压成正比）的大小。因此与观测器相关的门限
 * 宜按"反电势占额定母线 UDC_RATE 的比例 k"选取，再换算为转速：
 *     n [rpm] = k * UDC_RATE * 60 / (2*pi * 极对数 * psi_f)
 * 本电机（5 对极、0.141 Wb、310 V）每 1% 约 42 rpm。各门限对应的 k 值写在其注释中，供更换电机时参考。 */
/* 本节只给估算方法；各门限宏写在所属功能的一节里：ALIGN_STILL_RPM、OPENLOOP_*_RPM、SPIN_LOST_RPM、
 * FLY_*_RPM、TRACK_END_RPM、FLUX_HPF_COMP_MIN_RPM、PWM_5SEG_RPM，以及下面的启停门限。 */

/* ===================== 启停 ============================================== */
/* 外环为速度环。转速指令的启停门限（rpm，指令绝对值）：|指令| > STOP_TO_RUN_SPEED 时起动，
 * |指令| < RUN_TO_STOP_SPEED 时停机，二者之间为回差，防止指令在门限附近抖动时反复起停。
 * 这是对"转速指令"的判断，与观测器能力无关，故直接以 rpm 给出。
 * 停机均为软停机：先将电流按 CURRENT_SLEW_A_S（见"斜坡"）降到 0，再关闭 PWM
 * （本电机 4 A/s：闭环 0.3 A 约 75 ms，I/F 1 A 约 0.25 s）。 */
/* STOP_TO_RUN_SPEED：见 cfg_app.h */
/* RUN_TO_STOP_SPEED：见 cfg_app.h */

/* ===================== 电流环 ============================================ */
/* 零极点对消设计：Kp = 2*pi*BW*L，Ki = 2*pi*BW*Rs（d 轴用 Ld，q 轴用 Lq）。
 * 带宽选取：
 *   上限：数字延时约 1.5 个 PWM 周期（94 us），300 Hz 时相位裕量约 80°（800 Hz 仍有 60°）；
 *   下限：应高于最高电频率（2600 rpm 时 217 Hz），并比 PLL（15 Hz）快 10 倍以上。
 *   取 300 Hz（L 43.3 mH、Rs 7.35 Ω 时 Kp 44843、Ki 475）。
 * 快环 PI 采用 16 位增益格式，要求 Kp、Ki <= 65535，本电机约 438 Hz 封顶。电流 Q15 基准为硬件满量程
 * I_MAX（7.96 A）；如需更高带宽，可给增益增加移位（Kp*err >> 14、误差限 ±16383），无需改变电流基准。 */
#define CURRENT_BW_HZ           300.0
/* 角度时序补偿：Park 变换使用采样时刻 k 的估计角；反 Park 变换使用 k + CTRL_INVPARK_ADVANCE 拍的角度
 * （本拍计算的电压在下一个 PWM 周期生效，作用区间中点位于 k+1.5）。 */
#define CTRL_INVPARK_ADVANCE    1.5         /* PWM 周期 */
/* dq 解耦前馈（仅 Spin 状态）：vd_ff = -w*Lq*iq，vq_ff = w*(Ld*id + psi_f)。电流环只处理残差；
 * 进出 Spin 时将前馈量移入 / 移出积分器，实现无扰切换。 */

/* ===================== 速度环 ============================================ */
/* 被控对象 d(rpm)/dt = MOTOR_ACC_RPM_S_PER_A * iq，PI 闭环为二阶系统：
 *   Kp = 2*zeta*wn / b，Ki = wn^2 / b（b = MOTOR_ACC_RPM_S_PER_A）。
 * 闭环传函含零点（Ki/Kp = wn/(2*zeta)），阶跃给定时超调大于纯二阶 zeta=0.7 时的 5%（约 20%）；
 * 实际给定经过 S 形斜坡并带加速度前馈，超调可忽略。
 * 带宽选取：zeta = 0.7；wn 受两方面约束：
 *   1) 转速反馈滤波拐点应高于速度环带宽 5 倍（SPEED_FB_FILT_FITS，wn < 0.99 Hz）；
 *   2) 刚切入闭环的低速段，速度环经 iq -> 观测器角度误差 -> PLL 转速形成耦合，Rs 偏差较大时增益越高越易失锁。
 * 风机负载随转速增大，稳态跟踪误差约为 负载电流上升率 / Ki（Ki ∝ wn^2），故以 wn 调整带宽、
 * 保持 zeta 不变。取 0.8 Hz。由于 Kp、Ki 超过 65535，速度环使用 64 位 PI。 */
#define SPEED_WN_HZ             0.8
#define SPEED_ZETA              0.7
/* 转速反馈滤波（gMeas.speed 为 speedRaw 的一阶滑动平均，慢环执行）的拐点。速度环只需几 Hz 的反馈带宽；
 * 拐点必须明显低于 PLL 带宽 FLUXPLL_WN_HZ，否则速度环通过 iq -> 观测器误差 -> PLL 会在 PLL 频率附近
 * 形成闭环，在低速大 iq（刚切入闭环即加速）时可能自激。
 * 实际窗口取 2^n 拍（cfg_derived.h SPEED_FB_FILT_SH，向上取整，实际拐点不高于此值）。 */
#define SPEED_FB_FILT_HZ        (0.35 * FLUXPLL_WN_HZ)      /* 本电机 5.25 Hz -> 32 拍（实际 4.97 Hz） */
/* 速度环输出限幅（相电流峰值）：为加减速、换向留出余量（回馈电流同样受此限制）；
 * 上限受电机热极限与退磁电流约束，需要限制回馈泵升时可减小。 */
/* SPD_PI_LIMIT：见 cfg_app.h */

/* 轨迹前馈：iq_ff = J * d(w*)/dt / Kt = (每拍斜坡增量换算为 rpm/s) / b，加速、减速均施加。
 * 二自由度速度控制：给定轨迹（斜坡）的加速度由前馈直接给出转矩，PI 只修正负载与模型误差。
 * 减速段有前馈后，母线电压控制器（BUS_VDC_*）调整斜坡斜率时转矩立即跟随，母线电压环成为可设计的快速环。 */

/* 母线电流限（作用于速度环输出、只向下限制的 PI）。C 类经验值。 */
#define POWER_KP                (14000)
#define POWER_KI                (140)
/* 母线电流限（额定输入电流 P/UDC_RATE 约 0.32 A）。 */
#define IBUS_LIMIT_A            3.5         /* A */

/* ===================== 斜坡 ============================================== */
/* SPEED_RAMP_UP：见 cfg_app.h */
/* SPEED_RAMP_DOWN：见 cfg_app.h */
/* 反转（指令与当前转速反号）时过零前的减速斜率。同向调速仍使用上面两个斜率；反转需先将转速降到
 * OPENLOOP_FALLBACK_RPM 再交给 I/F，此段使用较大斜率。母线电压控制器（BUS_VDC_*）按回馈功率限制时，
 * 实际减速可能更慢。 */
#define SPEED_RAMP_REV          (2.0 * SPEED_RAMP_DOWN)     /* rpm/s，本电机 400 */
/* 斜坡圆角：限制给定轨迹的加加速度（jerk）。加速度从 0 升到 SPEED_RAMP_UP 用时 SPEED_RAMP_ROUND_S，
 * 接近目标时按同样的 jerk 提前将加速度收回 0（S 形曲线，即变频器的"斜坡圆弧时间"）。
 * 由于有斜率前馈（前馈电流 = 斜坡加速度 / b），梯形斜坡会使转矩阶跃；限制 jerk 后，
 * 转矩变化率 <= SPEED_RAMP_UP / b / SPEED_RAMP_ROUND_S（本电机 4 A/s）。 */
#define SPEED_RAMP_ROUND_S      0.1         /* s */
/* 电流矢量最大变化率：等于 S 形斜坡允许的最大转矩变化率 SPEED_RAMP_UP / b / SPEED_RAMP_ROUND_S（本电机 4 A/s）。
 * 所有非控制器产生的电流过渡均按此斜率执行：软停机、回退 I/F 前撤除制动 id、进入 I/F 后开环电流升至目标值。
 * 限制转矩（电流）变化率是低噪声运行的物理条件，全系统统一使用这一个值。 */
#define CURRENT_SLEW_A_S        (SPEED_RAMP_UP / MOTOR_ACC_RPM_S_PER_A / SPEED_RAMP_ROUND_S)

/* ===================== 起动：定位 + I/F 开环拖动 ========================== */
/* 流程：定位 -> I/F 开环拖动（电流矢量按给定加速度旋转，不使用观测器）-> 拖到 OPENLOOP_HANDOVER_RPM，
 * 观测器转速与开环转速一致后，按"转矩不变"原则融合切换到闭环 Spin。观测器在拖动期间持续运行，
 * 切换时已收敛。 */
/* 初始位置检测（IPD，见 ipd.h）：静止时依次施加 12 个方向的电压脉冲，根据磁路饱和造成的电流差异找出 N 极，
 * 直接从该角度开始 I/F 拖动，省去定位时间（约 1 s）及定位时转子被拉动的冲击。
 * 要求电机具有可测的饱和效应：峰值最大/最小 < IPD_MIN_RATIO 判为失败，本次自动退回定位。
 * 脉冲波形（IPD_PULSE_SHAPE，运行时可通过 gIpdCfg.shape 切换）：
 *   方波（+V 半宽、-V 半宽，电流为三角波）：同样峰值下时间最短、冲量最小，但边沿陡、噪声偏尖；
 *   正弦一个周期（电流为钟形，峰值 V*T/(pi*L)）：无尖角，但同样峰值需 1.6 倍时间，转矩冲量更大。
 * 相反方向成对相邻施加（0, 180, 30, 210 …），非 d 轴脉冲产生的转矩相互抵消，转子不被扰动。
 * 方向之间以零矢量等待，平均 IPD_GAP_S（随机 0.5~1.5 倍，避免形成固定音调）。
 * 参数以"期望电流峰值、脉宽"给出，所需电压由 d 轴电感计算（cfg_derived.h IPD_PULSE_V）：
 *   方波 V = I*L/(T/2)，正弦 V = I*pi*L/T（I 为不饱和方向的峰值）。更换电机自动换算；
 *   计算电压超过调制上限时配置检查报错（需加长 T）。
 *   本电机 0.6 A、1 ms 方波；N 极方向因饱和电流更大（约 2.3 倍）。 */
/* 起动方式：静止起动时确定转子初始角的方法。两种方式均编入固件，运行时选择：
 *   STARTUP_MODE_IPD    脉冲检测初始位置：约 0.13 s，转子不动；12 个脉冲有可闻噪声。检测失败（对比度不足）时本次自动退回定位。
 *   STARTUP_MODE_DRAG   直接拖动（静音起动）：不定位，旋转磁场以低频匀速将转子牵入同步后直接加速，见下文"直接拖动"。
 *                       代价：开始约 0.5 s 牵入时转子可能先反转（最多约半个极距）。
 * STARTUP_MODE 为上电默认值；运行中可修改 gStartup.mode（调试器或上位机通信），下一次起动生效（进入 Align 时读取一次）。
 * 飞车接管（转子在转时直接接管）与 Spin 回退 I/F 不受其影响。 */
/* STARTUP_MODE_IPD：见 cfg_app.h */
/* STARTUP_MODE_DRAG：见 cfg_app.h */
/* STARTUP_MODE：见 cfg_app.h */
#define IPD_PULSE_SHAPE         0           /* 0 方波，1 正弦 */
#define IPD_PEAK_A              (0.8 * MOTOR_I_RATED_A)     /* A，不饱和方向的电流峰值（本电机 0.6 A） */
#define IPD_PULSE_S             1.0e-3      /* s，单个方向的脉冲总宽（方波正负各一半） */
#define IPD_GAP_S               0.010       /* s，方向之间零矢量等待的平均值（实际随机 0.5~1.5 倍，约 L/R 的 1.5 倍） */
#define IPD_MIN_RATIO           1.3         /* 峰值最大/最小的下限（本电机约 2.3） */

/* 定位（IPD 检测失败时、参数辨识的静止测量时使用）：在定位角方向施加 d 轴电流，等待转子停稳。
 * 定位电流下转子相当于单摆：刚度 K = 1.5*p^2*psi_f*I（N·m/rad），周期 T = 2*pi*sqrt(J/K)
 * （本电机 0.6 A 时 0.50 s）。定位各段时间以 T 的倍数给出，T 在 Startup_Init 中由电机参数计算一次（开方），
 * 更换电机或修改定位电流后自动随之改变。 */
#define ALIGN_A              (0.8 * MOTOR_I_RATED_A)     /* A，本电机 0.6 A */
#define ALIGN_RAMP_A_S       3.0         /* A/s，定位电流上升斜率 */
/* 最短定位时间：死点附近（转矩约为 0）需先等转子离开死点再摆回，约 1.5 个周期，取约 1.6 T。 */
#define ALIGN_MIN_T             1.6         /* T，最短定位时间 */
/* 定位结束条件：满足最短时间后，转子转速（由反电势估计）低于 ALIGN_STILL_RPM 并持续 ALIGN_STILL_T，
 * 才开始拖动。摆到端点时速度也会短暂过零，持续窗口应长于端点附近"看似静止"的时间。
 * 最坏的端点位于离定位轴 90° 处（从死点附近摆下后的第一个回摆端点）：判据使用定位坐标系 q 轴的反电势
 * e_q = w*psi_f*cos(偏角)，此处转速过零、cos 也过零，e_q 按 t^3 增长：w = a*t，偏角变化 a*t^2/2，
 * a = (2*pi/T)^2（电角加速度），e_q 低于门限 w_th 的时间 = 2*(2*w_th/a^2)^(1/3)。
 * 本电机 T = 0.5 s、门限 8.4 rpm 时为 0.14 s（0.28 T），另加反电势滤波延迟。取 0.4 T（不小于 0.3 T）。 */
/* ALIGN_STILL_RPM：见 cfg_app.h */
#define ALIGN_STILL_T           0.4         /* T，低于 ALIGN_STILL_RPM 并持续该时间判为停稳 */
/* 最长定位时间（到时直接拖动）= 最短时间 + 阻尼衰减到 2% 的时间 4/(zeta*wn) ≈ 1.6 T + 2.55 T（zeta 0.25），取约 4 T。 */
#define ALIGN_MAX_T             4.0         /* T，最长定位时间，到时开始拖动 / 测量 */
/* 定位阻尼：电流闭环定位等效于无阻尼单摆（风机低速几乎无摩擦），转子持续摆动，
 * 若拖动开始时转子仍在运动则易失步。q 轴反电势正比于转子转速，按其反向施加少量 q 轴电流即构成阻尼。
 * ALIGN_ZETA 为摆的阻尼比，0 表示不加阻尼。不宜过大：反电势计算需减去 Rs*iq，Rs 配置误差会形成正反馈，
 * 环路增益约为 10*zeta*(Rs 相对误差)；取 0.25 时可容忍约 ±40% 的 Rs 误差。 */
#define ALIGN_ZETA           0.25
/* 拖动：电流 OPENLOOP_CURRENT_A 沿开环角方向，开环转速以 OPENLOOP_ACC_RPM_S 加速到 OPENLOOP_HANDOVER_RPM 后保持。
 * 加速度取拖动电流可产生的最大加速度（MOTOR_ACC_RPM_S_PER_A * OPENLOOP_CURRENT_A，本电机 0.75 A 约 376 rpm/s）
 * 的 1/3 作为余量，更换电机时按惯量自动缩放。
 * 切换转速应使反电势远大于电压误差：取反电势约 2.5% 额定母线（105 rpm 时约 7.7 V）。 */
/* OPENLOOP_CURRENT_A：见 cfg_app.h */
#define OPENLOOP_ACC_FRAC             0.33        /* I/F 加速度 / 拖动电流可产生的最大加速度（余量 3 倍）= sin(平衡负载角) */
#define OPENLOOP_ACC_RPM_S            (OPENLOOP_ACC_FRAC * MOTOR_ACC_RPM_S_PER_A * OPENLOOP_CURRENT_A)   /* rpm/s（本电机 124 rpm/s） */
/* 冷起动从平衡负载角开始：I/F 中转子相对开环电流矢量构成一个单摆，负载角 δ 的平衡点 δ_eq = asin(OPENLOOP_ACC_FRAC) ≈ 19°
 * （此时 I*Kt*sin δ 恰好产生 OPENLOOP_ACC_RPM_S 的加速度）；风机低速几乎没有摩擦，即无阻尼。若电流矢量放在转子角上（δ = 0）
 * 而加速度一次给满，δ 会在 0 与 2δ_eq 之间持续摆动，δ 回到 0 时转矩为 0，转速呈台阶状上升（加速、停顿、再加速）。
 * 冷起动时转子角已知（IPD 测得的 N 极角 / 定位角），因此开环角起始即超前转子 δ_eq（按转向取符号）：起动第一拍即处于平衡点，
 * 转子立即按 OPENLOOP_ACC_RPM_S 加速，负载角不摆动。IPD 后开环电流由 1/4 升至目标的过程中，加速度按电流比例给出，平衡角不变。 */
/* 直接拖动（STARTUP_MODE_DRAG）：不定位、不检测位置，给出指令后旋转磁场即以低频 f0 匀速转动，将转子牵入同步后再加速
 * （与 TI MCF8316 的 Slow First Cycle、Allegro 的 Align and Go 属同一类方法）。转子初始位置任意，被磁场牵入同步的过程是一个单摆；
 * 风机低速几乎没有摩擦，若采用电流型驱动（电流环 300 Hz），反电势被电流环完全抵消，没有阻尼，牵入时持续摆动并可能滑极。
 * 因此牵入段采用电压型驱动：电流环 Kp = 0、积分带宽 DRAG_CUR_BW_HZ（仅缓慢校正电流幅值），电压由前馈给出
 * = Rs*id + w0*(psi_f + Ld*id) + 逆变器电压误差（与观测器同一死区模型，按实测电流计算）。转子相对磁场摆动时，反电势产生的电流
 * 经定子电阻消耗，形成与转子位置无关的阻尼（相当于同步电机的阻尼绕组），B = 1.5*p^2*psi_f^2/Rs。死区补偿不可缺少：
 * 否则死区线性段（约 18 Ω）会抵消大部分阻尼。牵入段结束后切回电流环（前馈并入积分器，输出无跳变），开环角超前平衡负载角开始加速。
 * f0 = DRAG_F0_RATIO * I/F 摆动频率 f_n（本电机 f_n 2.24 Hz、T_n 0.45 s，f0 1.1 Hz = 13 rpm）：f0 过高转子跟不上，过低牵入慢。
 * 牵入段长 DRAG_HOLD_T 个 T_n：牵入的摆动依靠定子电阻阻尼（zeta 约 0.2）衰减，切回电流环时若仍在摆动，电流环下无阻尼，
 * 加速段将出现停顿；4 T_n 时摆动已衰减到 1/10 以下。从开始拖动到进入闭环约 2.6 s。 */
#define DRAG_F0_RATIO           0.5         /* 牵入频率 f0 / I/F 摆动频率 */
#define DRAG_HOLD_T             4.0         /* 匀速牵入段长度，T_n 的倍数（= 2 圈） */
#define DRAG_CUR_BW_HZ          0.5         /* 牵入段电压型驱动的电流积分带宽 Hz（远低于摆动频率，配置检查 < 1/(4 T_n)） */
/* Spin -> I/F 回退（热接管）时的开环电流：开环转矩 = I·Kt·sin(δ)，δ 超过 90° 即失步。
 *   I = max(OPENLOOP_CURRENT_A, |回退时的 iq| / sin(OPENLOOP_WARM_DELTA_DEG))：负载角 δ 不超过 45°，
 *       磁链 / 转矩系数偏小 25% 时仍在 70° 以内；
 *   开环需过零反转时 I 至少为 OPENLOOP_REV_CURRENT_A（本电机 1.0 A）：过零段需先制动再反向拖动，需求最大；
 *   上限 OPENLOOP_WARM_MAX_A。该电流只持续到切回 Spin（数秒），绕组温升可忽略。 */
#define OPENLOOP_WARM_DELTA_DEG       45.0
#define OPENLOOP_REV_CURRENT_A        (OPENLOOP_CURRENT_A / 0.75)       /* A，本电机 1.0 A */
#define OPENLOOP_WARM_MAX_A           (2.0 * MOTOR_I_RATED_A)     /* A，本电机 1.5 A */
/* OPENLOOP_HANDOVER_RPM：见 cfg_app.h */
/* 闭环运行的最低指令转速：|指令| 低于该值时电机以指令转速开环（I/F）运行。
 * 闭环可运行到比切换转速更低的转速，因此该值低于 OPENLOOP_HANDOVER_RPM。 */
/* OPENLOOP_SPIN_MIN_RPM：见 cfg_app.h */
/* Spin 失锁判据：指令同向、估计转速却降到该值以下（正常减速在 OPENLOOP_FALLBACK_RPM 已交给 I/F，同向降到此值
 * 只可能是观测器失锁或堵转）-> 停机并重新进行飞车检测。每 SPIN_CHECK_S 检查一次。C 类经验值。 */
/* SPIN_LOST_RPM：见 cfg_app.h */
#define SPIN_CHECK_S            0.010       /* s */
/* 闭环 -> I/F 的回退转速：指令反向或 |指令| < OPENLOOP_SPIN_MIN_RPM 时，Spin 减速到此转速即交给 I/F
 * （此时观测器仍锁定，交接角度准确），由开环带过零点，反向后再切回闭环。
 * 该值高于切换转速的原因：减速制动时反电势与 Rs*i 方向相反，端电压只有几伏，死区误差（约 3 V）占比大，
 * 观测器在 200 rpm 以下开始失准；加速时两者同向，低转速仍可靠。
 * 回退只取决于指令（反向 / 低于 OPENLOOP_SPIN_MIN_RPM），切换只在指令同向且 >= OPENLOOP_SPIN_MIN_RPM 时发生，
 * 二者互斥，不会来回跳变。 */
/* OPENLOOP_FALLBACK_RPM：见 cfg_app.h */
/* 切换融合时间：切换后 id 从 I/F 电流的 d 分量渐变到 0、加速前馈渐入所用的时间。
 * 观测器角度误差引起的转矩在此期间淡出，需让速度环来得及补偿：取速度环时间常数 1/(2*pi*SPEED_WN_HZ)。 */
#define OPENLOOP_BLEND_S              (1.0 / (2.0 * 3.14159265359 * SPEED_WN_HZ))     /* 本电机 0.20 s */
/* 切换判据：|观测器转速(滤波) - 开环转速| < OPENLOOP_AGREE_RATIO * 开环转速，持续 OPENLOOP_AGREE_S。C 类经验值。 */
#define OPENLOOP_AGREE_RATIO          0.15
#define OPENLOOP_AGREE_S              0.1         /* s */
/* 到达切换转速后 OPENLOOP_TIMEOUT_S 内仍未一致 -> 报堵转故障（随后自动重启）。C 类经验值。 */
#define OPENLOOP_TIMEOUT_S            2.0         /* s */

/* ===================== 起动公共参数 ====================================== */
#define PRECHARGE_TIME          20          /* ms，自举电容预充时间（C 类，应按自举电容 RC 及驱动芯片要求确定） */
#define STARTUP_SPEED_RAMP      60.0        /* rpm/s，OpenLoop（I/F）期间上报给斜坡状态的斜率（开环转速由 I/F 自行控制） */

/* ===================== 顺逆风（飞车）检测 ================================ */
/* 首先以短脉冲测速（见下）；测不到或结果不一致时退回连续短接测量（flystart.c）：
 * 每 1 ms 取一次短路电流矢量的平均，相邻两次的夹角即为 1 ms 内转过的电角度 -> 转速与转向；
 * 电流矢量方向加阻抗角即为转子角。转速足够高 -> 接入 Spin（同向 >= FLY_CATCH，反向 >= FLY_REV_SPIN_MIN_RPM）；
 * 较低但 >= FLY_LOW_MIN_RPM -> 直接进入 I/F；更低则继续短接制动，短路电流低于 FLYCURRENT_TH_PU 持续 FLYBRAKE_TIME
 * （检测到过转动时还需额外等待停稳）后按静止处理 -> 定位 -> I/F。 */

#define FLYBRAKE_TIME           0.1         /* s，短路电流低于门限持续该时间即判为静止 */
/* 静止判定转速：短路电流 = 反电势 / |Rs + jwL|，低速时约为 反电势 / Rs。以该转速下的短路电流作为
 * "在转"的电流门限 FLYCURRENT_TH_PU（本电机约 0.316 A），仅与电机参数有关，与采样电路无关。 */
/* FLY_STILL_RPM：见 cfg_app.h */
/* 制动后等待停稳：短路电流低于门限只说明转速低于 FLY_STILL_RPM，
 * 转子可能仍在转动。短接时低速制动转矩 ≈ B·ω，B = 1.5 p² ψf² / Rs，转速按 τ = J / B 指数衰减
 * （J 由 MOTOR_ACC_RPM_S_PER_A 反推，本电机 τ ≈ 0.4 s）。若检测到过转动，电流低于门限后再短接
 * FLYBRAKE_TIME + τ·ln(门限转速 / FLY_STILL_END_RPM) 才开始定位（本电机约 1.2 s）；
 * 始终静止（冷起动）时仍为 FLYBRAKE_TIME，起动时间不受影响。 */
/* FLY_STILL_END_RPM：见 cfg_app.h */
#define FLYCURRENT_TH_PU        (FLY_STILL_RPM * 2.0 * 3.14159265358979 / 60.0 * MOTOR_POLE_PAIRS * MOTOR_PSI_F_WB \
                                 / MOTOR_RS_OHM / I_MAX)   /* 标幺（相对 I_MAX）：FLY_STILL_RPM 下的短路电流 */
#define CALIPWMPERCENT          (0.99)      /* 短接时下桥占空比 */
#define FLY_CATCH_RPM           OPENLOOP_FALLBACK_RPM   /* rpm，同向且 >= 该值时直接接入闭环（取与 OPENLOOP_FALLBACK_RPM 相同，观测器在此以上可靠） */
/* 零电流跟踪结束转速（C 类经验值）：闭环中给出停机指令后不关闭 PWM，以零电流滑行跟踪转子，
 * 转速低于该值才关闭 PWM 回到 Stop。在跟踪期间重新给指令可直接闭环加速，无需飞车检测。
 * 跟踪期间零电流、无 Rs*i 误差，角度很准；限制因素是重新加速时带电流的那一刻，死区补偿误差越大、
 * 转速越低，角度误差越大。须不低于 OPENLOOP_HANDOVER_RPM（cfg_check_list.h 检查）。 */
/* TRACK_END_RPM：见 cfg_app.h */
/* 短接后等待电流暂态衰减再开始测量：暂态按 L/R 衰减（本电机 3.1 ms），实际衰减约为模型值的 2 倍，
 * 取 10 倍 L/R（约 5 个实际时间常数，暂态 < 1%）。 */
#define FLY_SETTLE_TAUS         10.0        /* x L/R（本电机 31 ms） */
#define FLY_STABLE_MS           8           /* 连续该次数（每次 1 ms）转速估计稳定才接管 */
/* 接管后"先跟踪后控制"：进入 Spin 后先保持 id = iq = 0（电流环照常工作，转子自由滑行），
 * 使观测器 / PLL / 转速滤波从测速初值收敛到真实转速，收敛后再闭合速度环；斜坡从收敛后的转速起步，积分器从 0 起步。
 * 等待时间按估计器自身动态计算（cfg_derived.h FLY_SYNC_TICKS）：PLL 误差按 e^(-zeta*wn*t) 衰减，
 * 取 4 个时间常数（98%，本电机 60 ms）；结束时以 PLL 转速重新预置转速滤波器。
 * 零电流时观测器无 Rs*i 误差，为其最准确的工况。代价是此期间转子自由滑行，接管后有少量掉速。 */
/* 短脉冲测速：进入 FlyCheck 后不长时间短接，而是每 FLY_PULSE_PERIOD_TICKS 施加一个下桥短接脉冲，
 * 电流达到门限（按实测采样噪声计算，见下）且至少 FLY_PULSE_MIN_TICKS 拍后松开（上下桥全关）。
 * 脉冲很短时电阻尚未起作用，电流 ≈ -∫e dt / L，方向与反电势相反：
 *   转子角 = ∠i + sgn(w)*90°，相邻脉冲的夹角 = 一个周期内转过的电角度 -> 转速与转向。
 * 共施加 FLY_PULSE_N 个脉冲（本电机约 8 ms），同向且转速足够即直接接管，接管时电流几乎为 0，
 * 避免长时间短接带来的制动冲击。
 * 测不到（转速太低）、反向、或多次测量不一致时，退回连续短接测量 / 制动。 */
/* 脉冲精度要求由后级决定：
 *  - 接入 Spin：只为开启 PWM 的瞬间提供初值（使电流环与反电势匹配），精确值由后续零电流同步获得，要求约 10°；
 *  - 低速直接接入 I/F（反向低于 FLY_REV_SPIN_MIN_RPM、同向低于 FLY_CATCH）：开环无观测器修正，
 *    初值角度即开环起始负载角，要求更严，约 5°。
 * 测量前无法确定后级，按严格者取 FLY_SEED_ANGLE_DEG = 5°。脉冲电流取"ADC 噪声下单次矢量角度误差 <= 5°"的最小值：
 *   I = 噪声峰值 * 每码电流 * 2/sqrt(3)（Clarke 放大）/ tan(5°)，噪声峰值在上电零点校准时测得（gHal.adcNoisePk）。
 *   ±2 码时约 0.10 A。最长脉冲也达不到该电流即判为转速太低，退回短接测量。
 * 门限随板级噪声自适应，更换驱动板或采样电阻时无需重调。
 * 脉冲不得短于 FLY_PULSE_MIN_TICKS 个 PWM 周期（短接占空比与采样时刻使 1 拍脉冲的中点不准）：
 * 高速时电流按 E * 2T / L 自然增大（本电机 1000 rpm 约 0.2 A），低速时按噪声下限截止。
 * 脉冲个数 FLY_PULSE_N 需保证一致性校验的余量。 */
#define FLY_SEED_ANGLE_DEG      5.0
#define FLY_PULSE_MIN_TICKS     2
#define FLY_PULSE_N             6
#define FLY_BRAKE_TIMEOUT_S     10.0        /* s，短接制动超过该时间仍未停止（如持续受风）-> 故障 */
/* 低于 FLY_CATCH_RPM 但 >= FLY_LOW_MIN_RPM：按测得的角度 / 转速直接进入 I/F 开环（与 Spin 回退相同的接法），省去制动和定位 */
/* FLY_LOW_MIN_RPM：见 cfg_app.h */
/* 转子与指令反向时：与同向相同，按转子自身方向接管（>= FLY_REV_SPIN_MIN_RPM 进 Spin，>= FLY_LOW_MIN_RPM 进 I/F），
 * 随后走正常换向流程：速度环减速 -> OPENLOOP_FALLBACK_RPM 回退 I/F -> I/F 过零 -> 反向切闭环。
 * 全程受控、电流有限幅，与运行中直接给反转指令为同一路径。 */
/* 反向接管时直接进入 Spin 的下限：需与回退点 OPENLOOP_FALLBACK_RPM 保持足够距离（约 2 倍），使 Spin 中的 PLL 有时间收敛；
 * 更低则直接进入 I/F（见 flystart.c）。 */
/* FLY_REV_SPIN_MIN_RPM：见 cfg_app.h */

/* ===================== 回馈减速：母线电压控制 ==================================
 * 物理背景：无制动电阻、整流桥不能回馈电网时，减速过程中转子动能只能进入母线电容，或在电机 / 逆变器中转为热量。
 *   C*V*dV/dt = P_回馈 - P_损耗，    P_回馈 ≈ J*w*a（a 为减速度）
 * 因此减速能力取决于"损耗能吸收多少功率"，而非斜坡设定：转速越高，同样减速度下回馈功率越大。
 * 方法（同工业变频器的 Vdc_max 控制）：母线电压控制器作用于斜坡（给定轨迹），而不是在速度环中削减 iq ——
 * 给定始终在物理上可实现，速度环不会饱和，无需抗饱和处理。
 *   电容储能裕量 e = C/2 * (Vref^2 - V^2)（J），允许回馈功率 P = Kv * e（Kv = 2*pi*BUS_VDC_BW_HZ，1/s），
 *   回馈对应的减速度 a_回馈 = P / (J*w) = b * P / (Kt*w)（按转速自动调度增益）。
 *   负载转矩本身也使转子减速（a_负载 = b * iq_负载，iq_负载取速度 PI 输出：加减速转矩在轨迹前馈中，PI 输出即负载转矩），
 *   这部分动能由负载消耗、不进入母线，因此斜坡减速率 = min(额定值, a_负载 + a_回馈)。
 *   母线高于调节点（e <= 0，如输入电压超出规格）时 a_回馈 = 0，仍按 a_负载 减速（iq 指令约为 0，不回馈），减速指令不会被搁置。
 * 仅用比例控制：电容本身即积分器（e 的导数就是净回馈功率），稳态时 P = 损耗、母线稳定在 Vref 略下方，无积分饱和问题。
 * 减速段有轨迹前馈，斜坡斜率变化时转矩立即跟随，该环的对象为 de/dt = -(P - 损耗)，
 * 闭环为一阶、时间常数 1/Kv。带宽受母线滤波（4 ms）与慢环频率（1 kHz）限制，取 5 Hz（32 ms）。
 * 所用参数均为物理量：母线电容 BOARD_CBUS_UF、转动惯量（MOTOR_ACC_RPM_S_PER_A）、Kt（极对数、磁链）。 */
/* 调节点 BUS_VDC_REF_V 由过压门限向下推导（cfg_derived.h），见下方"门限排布"。 */
#define BUS_VDC_BW_HZ           5.0         /* Hz，母线电压环带宽 */
/* 损耗制动（磁通制动）：减速受母线限制时叠加负 id（表贴电机 id 不产生转矩，只增加铜损 1.5*Rs*id^2，
 * 将动能转为热量，从而提高允许的减速度）。大小由母线电压控制器的受限程度 lam = 1 - a / a_额定 决定：
 * lam >= BUS_BRAKE_FULL_LAMBDA 时给满 BUS_REGEN_BRAKE_A，中间线性过渡；不受限（低速、母线有裕量）时为 0。
 * 再经一阶平滑（时间常数同母线电压环），无开关、无门限，电流连续。
 * 本电机 0.75 A 约 6 W（1.5 * 7.35 * 0.75^2），与额定电流下的铜损相当，只在减速期间持续数秒。 */
#define BUS_REGEN_BRAKE_A       (1.0 * MOTOR_I_RATED_A)     /* A，本电机 0.75 A */
#define BUS_BRAKE_FULL_LAMBDA   0.25        /* C 类经验值 */
/* 兜底保护：母线电压控制器无法限制时（如负载突然倒拖），在速度环中将回馈方向的 iq 上限从 SPD_PI_LIMIT 线性降到 0。
 *
 * 门限排布（全部由过压门限向下推导，无经验系数）：
 *   母线测量延迟 Td = 母线滤波时间常数 + 1 个慢环周期（本板 5 ms）。此期间最多进入的回馈能量为
 *   P_d * Td，P_d = Kt * SPD_PI_LIMIT * w_额定（额定转速下的最大回馈功率，本电机 487 W），对应母线电压升高
 *   dV_d = P_d * Td / (C * V_OVP)（本板 12.2 V）。从过压门限向下每隔 dV_d 设置一档：
 *     FULL  = OVP  - dV_d   兜底将回馈 iq 降到 0，测量滞后 Td 也不会触及 OVP（本板 411.8 V）
 *     START = FULL - dV_d   兜底比例带 = dV_d：延迟内的电压变化不超过一个比例带，兜底不会振荡（399.6 V）
 *     REF   = START - dV_d  母线电压控制器调节点，延迟内的超调不会触及兜底（387.3 V）
 *   下限约束：REF 须高于空载母线最高值 BOARD_VBUS_IN_MAX_V * (1 + BOARD_VBUS_TOL)（本板 373 * 1.02 = 381 V），
 *   否则输入电压偏高时没有储能裕量，减速 / 换向指令无法执行。
 *   本板过压门限 424 V 与空载母线上限 381 V 之间只有 43 V，三档各 dV_d 须不超过 14 V，
 *   因此母线滤波时间常数不能超过 4 ms（8 ms 时 dV_d = 22 V，REF = 358 V，低于 264 Vac 的空载母线）。
 *   无法排布（OVP 太低或 dV_d 太大）时配置检查报错：需增大电容、降低 SPD_PI_LIMIT、缩短母线滤波或提高过压门限。 */

/* ===================== 慢环一阶滤波时间常数 =====================================
 * 以时间常数给出，移位位数按慢环频率自动推导（cfg_derived.h）。 */
#define MEAS_VBUS_FILT_TAU_S    0.004       /* s，母线电压（回馈门限排布中的测量延迟 Td 包含此项，上限见"门限排布"） */
#define MEAS_IBUS_FILT_TAU_S    0.008       /* s，母线电流（母线电流限） */
#define OPENLOOP_PLL_FILT_TAU_S       0.016       /* s，I/F 拖动时的 PLL 转速（用于与开环转速比较一致性） */
#define ALIGN_EMF_FILT_TAU_S 0.008       /* s，定位阻尼使用的 q 轴反电势 */

/* ===================== 保护 ============================================== */
#define DCBUS_OVER              (BOARD_DCBUS_OVP_V)         /* V，由硬件决定，见 cfg_board.h */
#define DCBUS_UNDER             (0.0)       /* V */
#define DCBUS_FILT_S            0.1         /* s，母线越限持续该时间才报故障 */
#define FAULTRELEASE_TIME       0.0         /* s */
/* 缺相检测（Spin、I/F 共用，见 protect.h）：窗口为 PHASELESS_PERIODS 个电周期（整数个周期内三相 |i| 的平均值
 * 才相等，故转速越高窗口越短）。最大相平均 |i| > PHASELESS_RATIO 倍最小相时判为缺相；最大相低于
 * PHASELESS_RATIO 倍噪声峰值的窗口不判。4 个周期对应 1000 rpm 48 ms、60 rpm 0.8 s；
 * 比值 3 介于"平衡时 ≈ 1"与"断相时 = 电流 / 噪声"之间。 */
#define PHASELESS_PERIODS       4
#define PHASELESS_RATIO         3
/* 超速：取以下两者的较小值（cfg_derived.h PROTECT_SPEED_OC_RPM）：
 *   1) 反电势线电压峰值 sqrt(3)*psi_f*w_e 达到兜底满限 BUS_REGEN_FULL_V 的转速（超过后一旦关闭 PWM，
 *      反电势经续流二极管直接向母线充电，无法控制）；
 *   2) 机械允许转速 MOTOR_SPEED_LIMIT_RPM。 */

/* ===================== 磁链观测器 ======================================== */
/* 定点基准 FLUX_V_BASE / FLUX_WN_HZ / FLUX_PSI_N 由电机与驱动板参数自动计算（cfg_derived.h）。 */
#define FLUX_BUS_MIN_V          100.0       /* V，母线低于该值时观测器保持复位 */
/* 反电势去直流（高通）：一阶滑动平均，时间常数 FLUX_HPF_TAU_S，窗口取最近 2^SH 拍（cfg_derived.h FLUX_HPF_SH）。 */
#define FLUX_HPF_TAU_S          0.256
/* NLFO 径向修正时间常数：|psi| 偏离 |psi_f| 后按该时间常数拉回（cfg_derived.h 换算为移位 + 归一化，可连续取值）。
 * 折中：时间常数短 -> 初始误差 / 直流偏置收敛快；时间常数长 -> 低速时电压误差（死区）经径向修正转化成的角度误差小。
 * 低速磁链幅值偏大的根源是电压误差（死区），应通过死区补偿解决，而非加长该时间常数。 */
#define FLUX_NLFO_TAU_S         0.01917
/* NLFO 修正量饱和（Q15）。仅用于防溢出，不宜收紧。 */
#define FLUX_NLFO_FQ_LIMIT      32768
/* 去直流的相位补偿：反电势先高通再积分，等效 1/(s+wc)，磁链角超前 atan(wc/w)
 * （wc = 1/0.256 s：80 rpm 时 5.3°，30 rpm 时 14°）。按估计转速将输出 psi_s 反向旋转该角度
 * （慢环计算 sin/cos，快环 4 次乘法）；转速低于 FLUX_HPF_COMP_MIN_RPM 时不补偿，到 2 倍该转速线性补满。 */
/* FLUX_HPF_COMP_MIN_RPM：见 cfg_app.h */
/* 补偿所用转速：Spin 中使用 PLL 转速；I/F 拖动时使用开环转速（转子同步，比估计值可靠）；
 * 定位、飞车检测、停机时不补偿。 */

/* ===================== FluxPLL =========================================== */
/* 二阶锁相环：wn、zeta 为按 MOTOR_PSI_F_WB 计算的实际环路参数。
 * 取 15 Hz / 0.707（比速度环快约 20 倍，比电流环慢约 20 倍）-> Kp 27950、Ki 116。 */
#define FLUXPLL_WN_HZ           15.0
#define FLUXPLL_ZETA            0.707

/* ===================== MTPA / 弱磁 ======================================== */
/* 默认均关闭（本电机为表贴式，运行转速未达电压极限）：关闭时 id_ref 模块不编译任何代码。用于 IPM / 高速电机时打开。
 * MTPA：id = (-psi_f + sqrt(psi_f^2 + 4(Ld-Lq)^2 iq^2)) / (2(Ld-Lq))，仅对 IPM 有意义
 *       （MOTOR_IPM = 0 时 id 恒为 0，打开也不起作用）。
 * 弱磁：电压矢量 |V| 超过 (1 - WEAKEN_VOLTAGE_MARGIN) * VMAX 时积分产生负 id，将电压压回限值以内；
 *       表贴与内嵌式电机均适用。本电机在 310 V 母线下约 2088 rpm 达到电压极限。
 * 二者只在 Spin（闭环运行）中生效；回退 OpenLoop / 停机时 id 清零。 */
#define CTRL_MTPA_ENABLE        0
#define CTRL_WEAKEN_ENABLE          0
#define WEAKEN_VOLTAGE_MARGIN       0.08        /* 电压裕量 */
/* 弱磁积分速度：电压误差为 100% VMAX 时 id 每秒变化 WEAKEN_KI_A_PER_S 安培。
 * 环路极点 ≈ WEAKEN_KI_A_PER_S * omega*Ld / VMAX；本电机 2000 rpm 时 omega*Ld ≈ 48 V/A，
 * VMAX ≈ 174 V，50 A/s 对应约 2 Hz（快于速度环、远慢于电流环）。 */
#define WEAKEN_KI_A_PER_S           50.0
#define WEAKEN_ID_MIN_A             (-2.0)      /* A，弱磁 id 下限（受退磁电流、效率约束） */
/* 电流圆：|i| <= SPD_PI_LIMIT。存在 id 时 iq 上限自动缩为 sqrt(Imax^2 - id^2)。 */

/* ===================== PWM 调制方式 ======================================== */
/* 七段式 -> 五段式（不连续 PWM，开关损耗减少约 1/3，但低调制比时电流畸变较大）的切换转速。
 * 本质上由调制比（反电势 / 母线）决定，更换电机时宜保持调制比约 0.26（反电势约 15% 额定母线）。 */
/* PWM_5SEG_RPM：见 cfg_app.h */

#endif
