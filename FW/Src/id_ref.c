/* ============================================================================
 * @file    id_ref.c
 * @brief   MTPA + 弱磁，接口说明见 id_ref.h。全部在慢环中计算，允许除法和开方。
 * ==========================================================================*/
#include "id_ref.h"
#include "foc_core.h"
#include "speed_ctrl.h"
#include "flux_obs.h"
#include "param.h"
#include "qmath.h"

IdRef_TypeDef gIdRef;

#if IDREF_ACTIVE

/* 上电：清除 MTPA / 弱磁状态，iq 上限 = 速度环电流上限 */
void IdRef_Init(void)
{
    gIdRef.idMtpa = 0;
    gIdRef.idFw = 0;
    gIdRef.idCmd = 0;
    gIdRef.iqLimit = gSpd.iqLimit;
    gIdRef.vRatio = 0;
    gIdRef.fwInteg = 0;
}

/* 停机 / 回退 I/F：id 归零，恢复 iq 上限（gIdRef.iqLimit） */
void IdRef_Reset(void)
{
    IdRef_Init();
    gSpd.idqCmd.s16D = 0;
}

/* MTPA：id = (sqrt(psi_f^2 + (2*(Ld-Lq)*iq)^2) - psi_f) / (2*(Ld-Lq))，慢环计算 */
static int16_t mtpaId(int16_t iq)
{
#if CTRL_MTPA_ENABLE
    int32_t psi = gGain.fluxNomQ15, kdl = gGain.fluxKdlQ15;
    int64_t p, q, id;
    uint32_t s;
    if (kdl == 0 || psi <= 0) return 0;
    p = (int64_t)psi << 15;                     /* psi_f，Q30 */
    q = (int64_t)iq * kdl * 2;                  /* 2*(Ld-Lq)*iq，Q30 */
    s = QMath_Isqrt64((uint64_t)(p * p + q * q));
    id = ((int64_t)s - p) / (2 * kdl);
    return QMath_Sat16((int32_t)QMath_Clamp((int32_t)id, 32767));
#else
    (void)iq;
    return 0;
#endif
}

/* 弱磁：|V| 超过 (1-margin)*VMAX 时积分产生负 id */
static int16_t fwId(void)
{
#if CTRL_WEAKEN_ENABLE
    int32_t vd = gFoc.vdq.s16D, vq = gFoc.vdq.s16Q, vmax = gFoc.vmax;
    int32_t vmag, ratio, err, lo = (int32_t)WEAKEN_ID_MIN_Q15 * 65536;
    if (vmax <= 0) return gIdRef.idFw;
    vmag = (int32_t)QMath_Isqrt32((uint32_t)(vd * vd + vq * vq));
    ratio = (vmag << 15) / vmax;                                 /* 慢环，允许除法 */
    if (ratio > 65535) ratio = 65535;
    gIdRef.vRatio = (int16_t)(ratio > 32767 ? 32767 : ratio);
    err = (int32_t)WEAKEN_VLIM_Q15 - ratio;                          /* >0：电压有裕量，id 向 0 回归 */
    gIdRef.fwInteg += (WEAKEN_KI_Q20 * err) >> 4;
    if (gIdRef.fwInteg > 0) gIdRef.fwInteg = 0;
    if (gIdRef.fwInteg < lo) gIdRef.fwInteg = lo;
    return (int16_t)(gIdRef.fwInteg >> 16);
#else
    return 0;
#endif
}

/* 慢环每拍（Spin，速度环之后）：id = MTPA id + 弱磁 id + 损耗制动 id，限制在电流上限内；
 * 电流圆剩余部分分配给 iq（sqrt(Imax^2 - id^2)），并交给速度环作为下一拍的 PI 输出限幅 */
void IdRef_Slow(void)
{
    int32_t imax = gSpd.iqLimit, id, iqLim, iq;

    gIdRef.idMtpa = mtpaId(gSpd.idqCmd.s16Q);
    gIdRef.idFw = fwId();
    id = (int32_t)gIdRef.idMtpa + gIdRef.idFw;
    id += gSpd.brakeId;                           /* 回馈减速时用于限制母线电压的损耗制动负 id（speed_ctrl.c） */
    if (id < -imax) id = -imax;
    if (id > imax) id = imax;

    /* 电流圆：iq 上限 sqrt(Imax^2 - id^2)。本拍 iq 在此限幅；gIdRef.iqLimit 由速度环在下一拍 PI 运行前
     * 作为输出限幅（含积分器，speed_ctrl.c），限幅只由速度环一处设定 */
    iqLim = (int32_t)QMath_Isqrt32((uint32_t)(imax * imax - id * id));
    iq = gSpd.idqCmd.s16Q;
    if (iq > iqLim) iq = iqLim;
    if (iq < -iqLim) iq = -iqLim;

    gIdRef.idCmd = (int16_t)id;
    gIdRef.iqLimit = (int16_t)iqLim;
    gSpd.idqCmd.s16D = (int16_t)id;
    gSpd.idqCmd.s16Q = (int16_t)iq;
}

#else   /* !IDREF_ACTIVE：不修改任何控制量 */

void IdRef_Init(void)  { }
void IdRef_Reset(void) { }
void IdRef_Slow(void)  { }

#endif
