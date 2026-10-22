/* ============================================================================
 * identify.h —— 电机参数自动辨识（P3 阶段实现，现在只有接口）
 *
 * 目标：换电机时不用手量参数，上电跑一遍辨识就把 cfg_motor.h 里的量测出来，
 * 再由 fw_config.h 的公式自动得到全部增益。计划的方法（都在静止或空载下做）：
 *   Rs     ：Align 位置打两档直流电流，dV/dI（同时得到死区等效电压）
 *   Ld/Lq  ：d 轴/q 轴方向打电压脉冲，测 di/dt（IPM 才有区别；同一套也做 IPD）
 *   psi_f  ：空载稳速时 |v - Rs*i - jwL*i| / omega，或观测器 |psi_r| 的均值
 *   J（b） ：两段速度斜坡的 iq 差 / 加速度差（= MOTOR_ACC_RPM_S_PER_A）
 *   在线 Rs：温升后 Rs 漂移，用低频 d 轴注入慢慢修正（P6 以后）
 * 结果写进 IdentResult，由用户抄回 cfg_motor.h（暂不自动写 Flash）。
 * ==========================================================================*/
#ifndef IDENTIFY_H
#define IDENTIFY_H
#include <stdint.h>

typedef enum {
    IDENT_IDLE = 0,
    IDENT_RUNNING,
    IDENT_DONE,
    IDENT_NOT_IMPLEMENTED
} IdentState_TypeDef;

typedef struct {
    float rsOhm;        /* 相电阻 */
    float ldH, lqH;     /* d/q 电感 */
    float psiWb;        /* 转子磁链 */
    float accRpmSPerA;  /* 力矩电流 -> 加速度 */
    float deadVoltV;    /* 死区等效电压 */
    IdentState_TypeDef state;
} IdentResult_TypeDef;

extern IdentResult_TypeDef gIdent;

/* 请求一次辨识（电机必须在 Stop 状态）。现在返回 IDENT_NOT_IMPLEMENTED。 */
IdentState_TypeDef Ident_Request(void);

#endif
