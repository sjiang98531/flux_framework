/* ============================================================================
 * mtpa_fw.h —— d 轴电流指令：MTPA（最大转矩电流比）+ 弱磁
 *
 *   id_cmd = id_mtpa(iq) + id_fw          只在 Spin 的慢环里算（1 kHz）
 *   iq 上限 = sqrt(Imax^2 - id^2)          同时改速度 PI 的输出限幅（防积分饱和）
 *
 * CTRL_MTPA_ENABLE / CTRL_FW_ENABLE 都为 0 时，下面的函数编译成空函数，
 * 对 gFoc / gSpd 没有任何写操作（默认配置 = test33 行为）。
 * 内嵌式电机（MOTOR_IPM = 1）同时通知观测器：有效磁链幅值目标随 id 变化。
 * ==========================================================================*/
#ifndef MTPA_FW_H
#define MTPA_FW_H
#include "fw_config.h"

#define MTPA_FW_ACTIVE  (CTRL_MTPA_ENABLE || CTRL_FW_ENABLE)

typedef struct {
    int16_t idMtpa;     /* Q15（I_MAX 基准） */
    int16_t idFw;       /* Q15 */
    int16_t idCmd;      /* Q15，= gFoc.idqRef.s16D */
    int16_t iqLimit;    /* Q15，当前 iq 上限 */
    int16_t vRatio;     /* |Vdq| / VMAX，Q15（弱磁环的反馈，J-Scope 看这个） */
    int32_t fwInteg;    /* 弱磁积分器，id 的 Q31 */
} MtpaFw_TypeDef;

extern MtpaFw_TypeDef gMtpaFw;

void MtpaFw_Init(void);
/* 离开 Spin（回退 Startup、停机、故障）时调用：id 清零、恢复速度 PI 限幅 */
void MtpaFw_Reset(void);
/* Spin 慢环，在速度环算出 iq 之后调用 */
void MtpaFw_Slow(void);

#endif
