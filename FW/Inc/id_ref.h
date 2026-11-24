/* ============================================================================
 * @file    id_ref.h
 * @brief   d 轴电流指令：MTPA（最大转矩电流比）+ 弱磁
 *
 *   id_cmd = id_mtpa(iq) + id_fw          只在 Spin 的慢环中计算（1 kHz）
 *   iq 上限 = sqrt(Imax^2 - id^2)          速度环在下一拍以此限制 PI 输出（含积分器，防积分饱和）
 *
 * CTRL_MTPA_ENABLE / CTRL_WEAKEN_ENABLE 均为 0 时，以下函数编译为空函数，
 * 对 gFoc / gSpd 无任何写操作。
 * ==========================================================================*/
#ifndef ID_REF_H
#define ID_REF_H
#include "cfg_derived.h"

#define IDREF_ACTIVE  (CTRL_MTPA_ENABLE || CTRL_WEAKEN_ENABLE)

typedef struct {
    int16_t idMtpa;     /* Q15（I_MAX 基准） */
    int16_t idFw;       /* Q15 */
    int16_t idCmd;      /* Q15，= gFoc.idqRef.s16D */
    int16_t iqLimit;    /* Q15，当前 iq 上限 */
    int16_t vRatio;     /* |Vdq| / VMAX，Q15（弱磁环反馈量） */
    int32_t fwInteg;    /* 弱磁积分器，id 的 Q31 */
} IdRef_TypeDef;

extern IdRef_TypeDef gIdRef;

void IdRef_Init(void);
/* 离开 Spin（回退 OpenLoop、停机、故障）时调用：id 清零、恢复 iq 上限 */
void IdRef_Reset(void);
/* Spin 慢环，在速度环计算出 iq 之后调用 */
void IdRef_Slow(void);

#endif
