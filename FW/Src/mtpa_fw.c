/* MTPA + 弱磁，见 mtpa_fw.h。全部在慢环里算，允许除法和开方。 */
#include "mtpa_fw.h"
#include "ctrl_globals.h"
#include "flux_obs.h"
#include "mathx.h"

MtpaFw_TypeDef gMtpaFw;

#if MTPA_FW_ACTIVE

void MtpaFw_Init(void)
{
    gMtpaFw.idMtpa = 0;
    gMtpaFw.idFw = 0;
    gMtpaFw.idCmd = 0;
    gMtpaFw.iqLimit = USER.s16IqLimitMax;
    gMtpaFw.vRatio = 0;
    gMtpaFw.fwInteg = 0;
}

void MtpaFw_Reset(void)
{
    MtpaFw_Init();
    FOC.sIdq_cmd.s16D = 0;
    FOC.Speed.qOutMax = USER.s16IqLimitMax;
    FOC.Speed.qOutMin = -USER.s16IqLimitMax;
#if MOTOR_IPM
    FluxObs_SetIdRef(0);
#endif
}

/* MTPA：标幺 id = (sqrt(1 + (2a*iq)^2) - 1) / (2a)，a = (Ld-Lq)*I_MAX/psi_f */
static int16_t mtpaId(int16_t iq)
{
#if MOTOR_IPM && CTRL_MTPA_ENABLE
    int64_t x = ((int64_t)iq * MTPA_K2A_Q16) >> 15;             /* 2a*iq_n，Q16 */
    uint32_t s = Mx_Isqrt64((uint64_t)((int64_t)1 << 32) + (uint64_t)(x * x));  /* Q16 */
    int64_t id = ((int64_t)((int32_t)s - 65536) * MTPA_KINV) >> 16;
    return Mx_Sat16((int32_t)Mx_Clamp((int32_t)id, 32767));
#else
    (void)iq;
    return 0;
#endif
}

/* 弱磁：|V| 超过 (1-margin)*VMAX 就积分出负 id */
static int16_t fwId(void)
{
#if CTRL_FW_ENABLE
    int32_t vd = FOC.sVdq.s16D, vq = FOC.sVdq.s16Q, vmax = FOC.s16VMAX;
    int32_t vmag, ratio, err, lo = (int32_t)FW_ID_MIN_Q15 * 65536;
    if (vmax <= 0) return gMtpaFw.idFw;
    vmag = (int32_t)Mx_Isqrt32((uint32_t)(vd * vd + vq * vq));
    ratio = (vmag << 15) / vmax;                                 /* 慢环，允许除法 */
    if (ratio > 65535) ratio = 65535;
    gMtpaFw.vRatio = (int16_t)(ratio > 32767 ? 32767 : ratio);
    err = (int32_t)FW_VLIM_Q15 - ratio;                          /* >0：有裕量，id 往 0 回 */
    gMtpaFw.fwInteg += (FW_KI_Q20 * err) >> 4;
    if (gMtpaFw.fwInteg > 0) gMtpaFw.fwInteg = 0;
    if (gMtpaFw.fwInteg < lo) gMtpaFw.fwInteg = lo;
    return (int16_t)(gMtpaFw.fwInteg >> 16);
#else
    return 0;
#endif
}

void MtpaFw_Slow(void)
{
    int32_t imax = USER.s16IqLimitMax, id, iqLim, iq;

    gMtpaFw.idMtpa = mtpaId(FOC.sIdq_cmd.s16Q);
    gMtpaFw.idFw = fwId();
    id = (int32_t)gMtpaFw.idMtpa + gMtpaFw.idFw;
    if (id < -imax) id = -imax;
    if (id > imax) id = imax;

    /* 电流圆：iq 上限 sqrt(Imax^2 - id^2)，同时作为速度 PI 的输出限幅 */
    iqLim = (int32_t)Mx_Isqrt32((uint32_t)(imax * imax - id * id));
    iq = FOC.sIdq_cmd.s16Q;
    if (iq > iqLim) iq = iqLim;
    if (iq < -iqLim) iq = -iqLim;

    gMtpaFw.idCmd = (int16_t)id;
    gMtpaFw.iqLimit = (int16_t)iqLim;
    FOC.sIdq_cmd.s16D = (int16_t)id;
    FOC.sIdq_cmd.s16Q = (int16_t)iq;
    FOC.Speed.qOutMax = iqLim;
    FOC.Speed.qOutMin = -iqLim;
#if MOTOR_IPM
    FluxObs_SetIdRef((int16_t)id);
#endif
}

#else   /* !MTPA_FW_ACTIVE：完全不碰控制量 */

void MtpaFw_Init(void)  { }
void MtpaFw_Reset(void) { }
void MtpaFw_Slow(void)  { }

#endif
