/* ============================================================================
 * @file    cfg_app.h
 * @brief   应用层配置（按产品要求修改）与现场调试项
 *
 * 配置分三层：
 *   cfg_motor.h     电机（更换电机必须修改）：铭牌值、Rs / L / 磁链 / 加速度系数（可先填估计值，辨识后回填）
 *   cfg_app.h       应用（按需修改）：转向、起动方式、停机方式、起停门限、加减速斜坡、电流上限；
 *                   以及转速门限、定位时间等可按现场现象直接调整的调试项（默认值一般可直接运行）
 *   cfg_internal.h  内部（无需修改）：带宽、阻尼、滤波、观测器、飞车检测、母线电压环等，全部由前两层与 cfg_board.h 自动换算
 * cfg_board.h 为驱动板硬件参数（采样、母线、死区），更换驱动板时修改。
 * 配置检查（cfg_check_list.h）未通过时电机不起动，gFluxDiag.cfgFailLine 给出未通过项的行号。
 * ==========================================================================*/
#ifndef CFG_APP_H
#define CFG_APP_H

/* ===================== 一、应用（按需修改） ============================== */
/* 转向：正的转速指令对应的旋转方向。RUN_CCW 在软件中将 U、V 两相对调（等效于对调一对相线，无需改动硬件）。
 * 按叶轮 / 风道的工作方向确定，本风机为 RUN_CW。 */
#define RUN_CW                  (0)
#define RUN_CCW                 (1)
#define RUN_DIR                 RUN_CW

/* 起动方式（上电默认值；运行中可修改 gStartup.mode，下一次起动生效；非法值按摆动起动处理）：
 *   STARTUP_MODE_IPD    检测初始位置：约 0.17 s，转子不动，无电流脉冲（高频注入 + 直流偏置）；检测失败时本次自动退回摆动起动
 *   STARTUP_MODE_SWING  摆动中起动（静音起动）：定位电流将转子拉向 0°，不加阻尼、不等转子停稳，转子顺转向摆过 0° 的瞬间
 *                       即从当时的角度和转速接入 I/F 连续加速，约 0.3 ~ 0.8 s 开始加速；转子可能先反转一段（最多约一个极距） */
#define STARTUP_MODE_IPD        0
#define STARTUP_MODE_SWING      3
#define STARTUP_MODE            STARTUP_MODE_SWING

/* 停机方式（上电默认值；运行中可修改 gSup.stopMode，立即生效）。停机 = |指令| < RUN_TO_STOP_SPEED：
 *   STOP_MODE_RAMP   受控减速：闭环运行中收到停机指令时，速度环按 SPEED_RAMP_DOWN 减速到 TRACK_END_RPM，
 *                    再将电流降到 0、关闭 PWM。减速度由斜坡决定，与负载大小、转向无关；
 *                    回馈受母线电压控制器限制（无制动电阻时，轻载下的减速度仍受损耗能吸收的功率限制）
 *   STOP_MODE_COAST  滑行：电流降到 0 后零电流跟踪，转子靠负载自行减速（减速度随负载、转向而变），
 *                    低于 TRACK_END_RPM 时关闭 PWM。全程不回馈
 * 两种方式下，减速 / 滑行途中重新给指令都直接续接闭环。 */
#define STOP_MODE_RAMP          0
#define STOP_MODE_COAST         1
#define STOP_MODE               STOP_MODE_RAMP

/* 起停门限（对转速指令判断）：|指令| > STOP_TO_RUN_SPEED 起动，< RUN_TO_STOP_SPEED 停机，二者之间为回差 */
#define STOP_TO_RUN_SPEED       15.0        /* rpm，起动门限 */
#define RUN_TO_STOP_SPEED       12.0        /* rpm，停机门限（须小于起动门限） */

/* 加减速斜坡（闭环运行时转速给定的变化率）。减速过快导致母线电压升高时，母线电压控制器自动放慢减速，不会过压 */
#define SPEED_RAMP_UP           200.0       /* rpm/s */
#define SPEED_RAMP_DOWN         200.0       /* rpm/s */

/* 电流上限，以额定电流（cfg_motor.h MOTOR_I_RATED_A，相电流峰值）的倍数表示，更换电机时自动缩放。
 * 须低于功率器件与电机退磁允许的电流；配置检查校验其在采样量程 I_MAX 以内。 */
#define SPD_PI_LIMIT            (2.0 * MOTOR_I_RATED_A)    /* A，闭环运行的相电流上限（速度环输出限幅），本电机 4.0 A */
#define OPENLOOP_CURRENT_A      (1.0 * MOTOR_I_RATED_A)     /* A，起动拖动（I/F）电流，本电机 0.75 A；带载无法起动时加大 */

/* ===================== 二、现场调试项 ==================================== */
/* 均直接以物理量（rpm、s）给出，默认值按本电机整定；更换电机后先用默认值试运行，再按现象调整。
 * 各项的原理与估算方法见 cfg_internal.h 对应小节。
 * 转速门限的估算：观测器是否可靠取决于反电势相对逆变器电压误差（死区，与母线电压成正比）的大小，
 * 按"反电势占额定母线的比例 k"换算：
 *     n [rpm] = k * UDC_RATE * 60 / (2*pi * 极对数 * psi_f)       本电机每 1% 约 42 rpm
 * 各门限之间的大小关系由配置检查保证（cfg_check_list.h）。 */

/* ---- 定位（仅参数辨识 IDENT_TEST = 1 静止测量时使用，SWING 起动使用同一定位电流 ALIGN_A）----
 * 定位电流下转子相当于单摆，周期 T 由电机参数与定位电流自动计算（本电机 0.5 s）：最短 1.6 T、停稳判据持续 0.4 T、最长 4 T。 */
#define ALIGN_STILL_EMF_PCT     0.1101      /* rpm，参数辨识静止测量的停稳判据（k 约 0.2%）。每次都等满上限时调高 */

/* ---- 开环拖动（I/F）与闭环的交接 ---- */
/* I/F 切换转速及相关门限：与固件 FW/Inc/cfg_app.h 默认值相同（各类电机统一，由额定转速与辨识参数自动换算） */
#define OPENLOOP_HANDOVER_RATED_K  0.10
#define OPENLOOP_HANDOVER_MAX_K    0.30
#define OPENLOOP_HANDOVER_VERR_K   0.5
#define OPENLOOP_HANDOVER_RS_ERR   0.10
#define OPENLOOP_SPIN_MIN_K        0.56
#define OPENLOOP_FALLBACK_K        2.40
#define SPIN_LOST_K                0.38
#define TRACK_END_K                1.24
#define FLY_REV_SPIN_MIN_K         4.80

/* ---- 顺逆风（飞车）起动 ---- */
#define FLY_STILL_RPM           800.0        /* rpm，低于该转速按静止处理（k 约 0.75%） */
#define FLY_STILL_END_RPM       80.0         /* rpm，制动后开始 IPD / 摆动起动时允许的残余转速。制动后起动出现抖动时调低 */
#define FLY_LOW_MIN_RPM         1000.0        /* rpm，同向转速不低于该值时直接接入 I/F，更低则先制动（须高于 FLY_STILL_RPM，k 约 1%） */

/* ---- 观测器 / 调制 ---- */
#define FLUX_HPF_COMP_MIN_RPM   800.0        /* rpm，观测器去直流相位补偿的起始转速（k 约 0.5%），2 倍该转速时补满 */
#define PWM_5SEG_RPM            10000.0       /* rpm，高于该转速使用五段式 PWM（开关损耗减少 1/3），调制比约 0.26；低速电流畸变大时调高 */

#endif
