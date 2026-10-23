/* MTPA + 弱磁，见 mtpa_fw.h。全部在慢环里算，允许除法和开方。 */
#include "mtpa_fw.h"
#include "foc_core.h"
#include "speed_ctrl.h"
#include "flux_obs.h"
#include "mathx.h"

MtpaFw_TypeDef gMtpaFw;

#if MTPA_FW_ACTIVE

/* 电流圆给出的 iq 上限，写到"产生 iq 的那个环"的输出限幅上（防积分饱和）：
 *   SPEED_LOOP：速度 PI（功率环在这里只是母线电流限，只许往下拉，不动它的上限）
 *   POWER_LOOP：功率 PI；速度环在这里只是限速器（输出上限保持 0，只改下限）
 *   TORQUE_LOOP：没有 PI，iq 直接限幅即可 */
static void setIqLimit(int32_t lim)
{
#if (CLOSE_LOOP == SPEED_LOOP)
    gSpd.pi.qOutMax = lim;
    gSpd.pi.qOutMin = -lim;
#elif (CLOSE_LOOP == POWER_LOOP)
    gSpd.piPower.qOutMax = lim;
    gSpd.piPower.qOutMin = -lim;
    gSpd.pi.qOutMin = -lim;
#else
    (void)lim;
#endif
}

void MtpaFw_Init(void)
{
    gMtpaFw.idMtpa = 0;
    gMtpaFw.idFw = 0;
    gMtpaFw.idCmd = 0;
    gMtpaFw.iqLimit = gSpd.iqLimit;
    gMtpaFw.vRatio = 0;
    gMtpaFw.fwInteg = 0;
}

void MtpaFw_Reset(void)
{
    MtpaFw_Init();
    gFoc.idqRef.s16D = 0;
    setIqLimit(gSpd.iqLimit);
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
    int32_t vd = gFoc.vdq.s16D, vq = gFoc.vdq.s16Q, vmax = gFoc.vmax;
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
    int32_t imax = gSpd.iqLimit, id, iqLim, iq;

    gMtpaFw.idMtpa = mtpaId(gFoc.idqRef.s16Q);
    gMtpaFw.idFw = fwId();
    id = (int32_t)gMtpaFw.idMtpa + gMtpaFw.idFw;
    if (id < -imax) id = -imax;
    if (id > imax) id = imax;

    /* 电流圆：iq 上限 sqrt(Imax^2 - id^2)，同时作为速度 PI 的输出限幅 */
    iqLim = (int32_t)Mx_Isqrt32((uint32_t)(imax * imax - id * id));
    iq = gFoc.idqRef.s16Q;
    if (iq > iqLim) iq = iqLim;
    if (iq < -iqLim) iq = -iqLim;

    gMtpaFw.idCmd = (int16_t)id;
    gMtpaFw.iqLimit = (int16_t)iqLim;
    gFoc.idqRef.s16D = (int16_t)id;
    gFoc.idqRef.s16Q = (int16_t)iq;
    setIqLimit(iqLim);
#if MOTOR_IPM
    FluxObs_SetIdRef((int16_t)id);
#endif
}

#else   /* !MTPA_FW_ACTIVE：完全不碰控制量 */

void MtpaFw_Init(void)  { }
void MtpaFw_Reset(void) { }
void MtpaFw_Slow(void)  { }

#endif
