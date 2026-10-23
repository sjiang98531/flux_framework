/* 仿真主程序与固件之间的接口（glue_old.c = test33，glue_new.c = 当前框架）。
 * sim_main.c 只通过这里碰固件，所以固件内部怎么组织结构体都不影响仿真主程序。 */
#ifndef SIM_GLUE_H
#define SIM_GLUE_H
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t theta;          /* Park 用的电角度（Q15 角度） */
    int16_t vd, vq;         /* 电流环输出电压 */
    int16_t id, iq;         /* 实测 dq 电流 */
    int16_t idRef, iqRef;   /* dq 电流指令 */
    int16_t spdFilt;        /* 速度环用的转速（Q15） */
    int16_t pllSpeed16;     /* PLL 转速 >> 16（Q15） */
    unsigned mainState, runState;
} SimProbe;

#define GLUE_STATE_BLOCKS 4

void glue_init(void);
void glue_isr(void);
void glue_main_poll(void);
void glue_cfg(int scenario);
void glue_set_speed_cmd(int16_t q15);
void glue_probe(SimProbe *p);
size_t glue_export(uint8_t *buf);              /* 语义状态（观测器、PLL、故障码），新旧代码含义相同 */
const void *glue_state_ptr(int i);             /* 全状态哈希用：固件主要状态块 */
size_t glue_state_size(int i);
int16_t glue_psi_mag(void);
#endif
