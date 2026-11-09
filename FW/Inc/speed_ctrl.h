/* ============================================================================
 * @file    speed_ctrl.h
 * @brief   外环：速度斜坡 + 速度 PI + 轨迹前馈（+ 母线电流限）
 *
 *   iq_cmd = SpeedPI(ramp - filt) + PowerPI(Ibus 限) + iq_ff
 *   iq_ff  = 斜坡本拍实际斜率 / MOTOR_ACC_RPM_S_PER_A（加速、减速均施加）
 *   斜坡减速侧的步长由母线电压控制器限制（BUS_VDC_*：允许回馈功率 / (J*w)），给定始终可实现
 *
 * 速度 PI 增益由 cfg_ctrl.h 的 SPEED_WN_HZ / SPEED_ZETA 与电机的 MOTOR_ACC_RPM_S_PER_A 自动计算。
 * ==========================================================================*/
#ifndef SPEED_CTRL_H
#define SPEED_CTRL_H
#include "cfg_derived.h"
#include "qtypes.h"
#include "foc_math.h"

/* 外环状态 */
typedef struct {
    int16_t  cmd;               /* 本拍的转速指令（Q15），= gSup.speedCmd */
    int32_t  rampQ31;           /* 速度斜坡输出 Q31 */
    int16_t  ramp;              /* 速度斜坡输出 Q15 */
    int16_t  iqLimit;           /* iq 上限 Q15（SPD_PI_LIMIT） */
    int16_t  brakeId;           /* 损耗制动 id（Q15，<= 0），见 BUS_REGEN_BRAKE_A */
    int32_t  brakeIdQ15x;       /* brakeId 的一阶平滑状态（Q30） */
    int16_t  vdcLam;            /* 母线电压控制器的受限程度 lam = 1 - 减速步长 / 额定（Q15，0 = 不受限） */
    /* I/F -> Spin 融合：切换时刻将 I/F 电流原样换算到估计转子坐标系（id0, iq0）。
     * 速度 PI 预置为 iq0 的电动方向部分，其余部分（d 轴全部、q 轴反向部分）为观测器角度误差造成的分量，
     * 保存在 blendId0 / blendIq0，在 blendW 从 0 增至 32767（OPENLOOP_BLEND_S）期间按 (1 - w) 叠加并淡出；加速前馈按 w 渐入。
     * blendW = 32767 表示不融合（正常 Spin） */
    int16_t  blendW;
    int16_t  blendId0, blendIq0;    /* 切换时刻需淡出的 d / q 电流分量（Q15） */
    int16_t  blendAccQ12;       /* 切换时开环加速度 / OPENLOOP_ACC_RPM_S（Q12），斜坡斜率与加速前馈由此开始过渡 */
    Dq_TypeDef idqCmd;   /* Spin 慢环计算的电流指令（依次经速度环 -> MTPA/弱磁 -> 融合修改），
                                 * 每拍最后通过 FocCore_SetIdqRef 一次性发布到 gFoc.idqRef */
    Ramp_TypeDef speedRampSt;   /* 速度斜坡的步长与状态 */
    int32_t  rampAcc;           /* 速度斜坡当前加速度（Q31 / 慢环拍，带符号），其变化率受 SPEED_RAMP_ROUND_S 限制 */
    PI_TypeDef  pi;                /* 速度 PI */
    PI_TypeDef  piIbus;           /* 母线电流限 PI（只向下限制 iq） */
} SpeedCtrl_TypeDef;

extern SpeedCtrl_TypeDef gSpd;

void SpeedCtrl_Init(void);    /* 上电：斜坡速率、电流上限、两个 PI 的增益与限幅 */
void SpeedCtrl_LoadGains(void);     /* 从 gGain 重新装载速度 PI 增益（param.c 更新参数时调用） */
/* Spin 慢环：斜坡 + PI + 前馈，写入 gSpd.idqCmd（不直接写 gFoc.idqRef） */
void SpeedCtrl_SpinSlow(void);
/* 进入 Spin（从 OpenLoop 或飞车接管）：恢复运行斜坡率，按当前状态预置斜坡与 PI 积分器 */
void SpeedCtrl_OnEnterSpin(int16_t idNow, int16_t iqNow);    /* idNow / iqNow：此刻的电流指令，速度 PI 积分器预置为 iqNow */
/* 零电流跟踪中每拍：斜坡、积分器跟随估计转速（id = iq = 0） */
void SpeedCtrl_FollowSpeed(void);
/* 进入 OpenLoop：速度斜坡改用 STARTUP_SPEED_RAMP */
void SpeedCtrl_UseOpenLoopRamp(void);
/* I/F 切换时刻调用：开始融合。(id0, iq0) 为需淡出的分量（叠加在控制器输出上）；
 * 斜坡从 rampStartQ31（开环转速）以 I/F 加速度继续；accScaleQ12 = 开环加速度 / OPENLOOP_ACC_RPM_S */
void SpeedCtrl_BlendStart(int16_t id0, int16_t iq0, int32_t rampStartQ31, int16_t accScaleQ12, int16_t ff0);
/* 停止融合（离开 Spin / 停机 / 飞车直接进 Spin） */
void SpeedCtrl_BlendStop(void);
/* 停机 / 故障复位：PI 积分器、斜坡、融合清零 */
void SpeedCtrl_ResetRun(void);
/* 斜坡状态从该转速开始（进入 I/F 时） */
void SpeedCtrl_PresetRamp(int16_t spdQ15);
/* 仅用于显示：斜坡输出 = 斜坡状态（零电流跟踪期间速度环不运行） */
void SpeedCtrl_ShowRampState(void);
/* Spin 慢环在 SpeedCtrl_SpinSlow、IdRef_Slow 之后调用：将切换遗留分量叠加到 gSpd.idqCmd 上并淡出 */
void SpeedCtrl_BlendSlow(void);

#endif
