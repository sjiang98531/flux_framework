/* PMSM + inverter + ADC model for the frozen host simulation suite.
 * Extended with physical perturbation hooks:
 *   SIM_IGAIN      : current ADC gain multiplier (default 1.0)
 *   SIM_VBUS_GAIN  : Vbus ADC gain multiplier (default 1.0)
 *   SIM_OFF        : pre-calibration ADC offset (counts, removed by ZeroOffset)
 *   SIM_OFF_RUN    : post-calibration uncompensated ADC offset (counts, active when tNow > 0.22 s)
 *   SIM_COGGING    : cogging torque amplitude (Nm, sin(harm * theta_e))
 *   SIM_COG_HARM   : cogging electrical harmonic order (default 6)
 *   SIM_R_DRIFT    : relative Rs thermal drift (+0.15 = +15% ramp from t=1.5s to t=6.5s)
 *   SIM_TH0_DEG    : initial electrical angle in degrees (overrides SIM_TH0)
 */
#include <math.h>
#include <stdlib.h>
#include "plant.h"
#include "sc32_conf.h"
#include "cfg_board.h"
#include "cfg_motor.h"
#include "cfg_app.h"
#include "cfg_internal.h"

#if (RUN_DIR == RUN_CCW)
#define PL_DT_U 4
#define PL_DT_V 2
#define PL_ADC_U 0
#define PL_ADC_V 1
#else
#define PL_DT_U 2
#define PL_DT_V 4
#define PL_ADC_U 1
#define PL_ADC_V 0
#endif

extern uint16_t ADC_DMABuff[14];
#define PERIOD (SYSCLK_HSI_72MHz / 2.0 / (double)PWM_FREQ)
#define TWO_PI 6.283185307179586

static double dtSign(const Plant *p, double i)
{
    double x;
    if (p->dtI0 <= 0) return i > 0 ? 1.0 : -1.0;
    x = i / p->dtI0;
    return x > 1 ? 1.0 : (x < -1 ? -1.0 : x);
}

static double s_psiQsat = 0.0, s_iGainV = 1.0;   /* SIM_PSI_QSAT：磁钢磁链随 iq 交叉饱和 psi*(1-k*iq^2)；SIM_IGAIN_V：V 相电流采样增益（相对 U 相） */
/* 磁钢交链磁链（交叉饱和：q 轴电流大时铁心饱和，磁链变小） */
static double plant_psiEff(const Plant *p)
{
    double m = 1.0 - s_psiQsat * p->iq * p->iq;
    return p->psi * (m > 0.6 ? m : 0.6);
}

static double urand(Plant *p) { p->rng = p->rng * 1664525u + 1013904223u; return (p->rng >> 8) / 16777216.0; }

/* 仿真电机的真实参数：默认等于配置值；"只填铭牌"测试（run_eval.build_suite nameplate=True）中配置为占位值，
 * 真值由构建脚本以 SIMT_* 给出 */
#ifndef SIMT_RS
#define SIMT_RS  MOTOR_RS_OHM
#define SIMT_LD  MOTOR_LD_H
#define SIMT_LQ  MOTOR_LQ_H
#define SIMT_PSI MOTOR_PSI_F_WB
#define SIMT_ACC MOTOR_ACC_RPM_S_PER_A
#endif

double g_sim_iEd = 0.0, g_sim_iEq = 0.0;
static double s_vbusGain = 1.0, s_iGain = 1.0;
static double s_cogAmp = 0.0, s_cogHarm = 6.0, s_rDrift = 0.0;
static int s_offRun = 0;

void plant_init(Plant *p, int ipm)
{
    p->res = SIMT_RS; p->pp = MOTOR_POLE_PAIRS; p->psi = (double)SIMT_PSI;
    if (ipm) { p->lD = 0.040; p->lQ = 0.060; } else { p->lD = SIMT_LD; p->lQ = SIMT_LQ; }
    p->J = 1.5 * p->pp * p->psi / (SIMT_ACC * TWO_PI / 60.0);
    p->kFan = 1.5 * p->pp * p->psi * 0.292 / pow(1172.0 * TWO_PI / 60.0, 2.0);
    p->tFric = 0.01;
    p->vbus = p->vsrc = UDC_RATE; p->tdEff = INVERTER_DT_EFF_S;
    p->adcOff[0] = 2048 + 3; p->adcOff[1] = 2048 - 2;
    p->loadStep = 0.0;
    p->id = p->iq = p->wm = 0.0; g_sim_iEd = 0.0; g_sim_iEq = 0.0;
    p->thm = 1.5; p->iu = p->iv = p->iw = 0.0; p->te = 0.0; p->ipk = p->temin = p->temax = 0.0;
    p->rng = getenv("SIM_SEED") ? (uint32_t)atoi(getenv("SIM_SEED")) : 12345u;
    {
        const char *e;
        if ((e = getenv("SIM_TD"))) p->tdEff = atof(e);
        p->dtI0 = (e = getenv("SIM_DTI0")) ? atof(e) : INVERTER_DT_I0_A;
        p->dSatI = (e = getenv("SIM_DSAT")) ? atof(e) : 0.93;
        p->qSatK = (e = getenv("SIM_QSAT")) ? atof(e) : 0.0;
        p->qSatI0 = (e = getenv("SIM_QSAT_I0")) ? atof(e) : 1.0;
        if ((e = getenv("SIM_OFF"))) { p->adcOff[0] = 2048 + atoi(e); p->adcOff[1] = 2048 - atoi(e); }
        s_offRun = (e = getenv("SIM_OFF_RUN")) ? atoi(e) : 0;
        s_iGain = (e = getenv("SIM_IGAIN")) ? atof(e) : 1.0;
        s_iGainV = (e = getenv("SIM_IGAIN_V")) ? atof(e) : 1.0;
        s_psiQsat = (e = getenv("SIM_PSI_QSAT")) ? atof(e) : 0.0;
        s_vbusGain = (e = getenv("SIM_VBUS_GAIN")) ? atof(e) : 1.0;
        s_cogAmp = (e = getenv("SIM_COGGING")) ? atof(e) : 0.0;
        s_cogHarm = (e = getenv("SIM_COG_HARM")) ? atof(e) : 6.0;
        s_rDrift = (e = getenv("SIM_R_DRIFT")) ? atof(e) : 0.0;
        if ((e = getenv("SIM_TH0"))) p->thm = atof(e);
        if ((e = getenv("SIM_TH0_DEG"))) p->thm = (atof(e) * TWO_PI / 360.0) / (double)p->pp;
        if ((e = getenv("SIM_FRIC"))) p->tFric = atof(e);
        if ((e = getenv("SIM_W0"))) p->wm = atof(e) * TWO_PI / 60.0;
        if ((e = getenv("SIM_VBUS"))) p->vbus = p->vsrc = atof(e);
        if ((e = getenv("SIM_KR"))) p->res *= atof(e);
        if ((e = getenv("SIM_KL"))) { p->lD *= atof(e); p->lQ *= atof(e); }
        if ((e = getenv("SIM_KPSI"))) p->psi *= atof(e);
        if ((e = getenv("SIM_KJ"))) p->J *= atof(e);
        p->wind = (e = getenv("SIM_WIND")) ? atof(e) : 0.0;
        if ((e = getenv("SIM_KFAN"))) p->kFan *= atof(e);
        p->kFanRev = (e = getenv("SIM_KFAN_REV")) ? atof(e) : 1.0;
        p->noise = (e = getenv("SIM_NOISE")) ? atof(e) : 1.0;
        p->cbus = (e = getenv("SIM_CBUS")) ? atof(e) * 1e-6 : 0.0;
        p->rsrc = (e = getenv("SIM_RSRC")) ? atof(e) : 10.0;
        p->pstby = (e = getenv("SIM_PSTBY")) ? atof(e) : 3.0;
        p->openPh = (e = getenv("SIM_OPEN")) ? (e[0] == 'u' ? 0 : e[0] == 'v' ? 1 : 2) : -1;
        p->openAt = (e = getenv("SIM_OPEN_T")) ? atof(e) : 0.0;
        p->tNow = 0.0;
    }
}

static int phaseState(uint32_t chn, int bitHi)
{
    int hi = (chn >> bitHi) & 1, lo = (chn >> (bitHi + 1)) & 1;
    if (hi && lo) return 2;
    if (lo) return 1;
    if (hi) return 3;
    return 0;
}

static int s_dlyInit, s_dly;
static uint32_t s_chnPrev, s_chnPrev2;
static int32_t s_dtPrev[3], s_dtPrev2[3];

void plant_step(Plant *p, double dt)
{
    const int N = 8;
    double h = dt / N;
    uint32_t chn = EPWM->EPWM_CHN;
    int32_t dt4 = (int32_t)EPWM->EPWM_DT[PL_DT_U], dt2 = (int32_t)EPWM->EPWM_DT[PL_DT_V], dt0 = (int32_t)EPWM->EPWM_DT[0];
    if (!s_dlyInit) {
        const char *e = getenv("SIM_PWMDLY");
        s_dly = e ? atoi(e) : 0;
        s_dlyInit = 1;
        s_chnPrev = s_chnPrev2 = chn;
        s_dtPrev[0] = s_dtPrev2[0] = dt4; s_dtPrev[1] = s_dtPrev2[1] = dt2; s_dtPrev[2] = s_dtPrev2[2] = dt0;
    }
    if (s_dly == 1) {
        uint32_t c0 = chn; int32_t a = dt4, b = dt2, c = dt0;
        chn = s_chnPrev; dt4 = s_dtPrev[0]; dt2 = s_dtPrev[1]; dt0 = s_dtPrev[2];
        s_chnPrev = c0; s_dtPrev[0] = a; s_dtPrev[1] = b; s_dtPrev[2] = c;
    } else if (s_dly == 2) {
        uint32_t c0 = chn; int32_t a = dt4, b = dt2, c = dt0;
        chn = s_chnPrev2; dt4 = s_dtPrev2[0]; dt2 = s_dtPrev2[1]; dt0 = s_dtPrev2[2];
        s_chnPrev2 = s_chnPrev; s_dtPrev2[0] = s_dtPrev[0]; s_dtPrev2[1] = s_dtPrev[1]; s_dtPrev2[2] = s_dtPrev[2];
        s_chnPrev = c0; s_dtPrev[0] = a; s_dtPrev[1] = b; s_dtPrev[2] = c;
    }
    int su = phaseState(chn, PL_DT_U), sv = phaseState(chn, PL_DT_V), sw = phaseState(chn, 0);
    double du = (PERIOD - (double)dt4) / PERIOD;
    double dv = (PERIOD - (double)dt2) / PERIOD;
    double dw = (PERIOD - (double)dt0) / PERIOD;
    int driven = (su && sv && sw);
    int k;
    double rEff = p->res;
    if (s_rDrift != 0.0 && p->tNow > 1.5) {
        double frac = (p->tNow - 1.5) / 5.0;
        if (frac > 1.0) frac = 1.0;
        rEff = p->res * (1.0 + s_rDrift * frac);
    }
    if (du < 0) du = 0; if (du > 1) du = 1;
    if (dv < 0) dv = 0; if (dv > 1) dv = 1;
    if (dw < 0) dw = 0; if (dw > 1) dw = 1;
    if (su == 1) du = 0; if (sv == 1) dv = 0; if (sw == 1) dw = 0;
    if (su == 3) du = 1; if (sv == 3) dv = 1; if (sw == 3) dw = 1;
    for (k = 0; k < N; k++) {
        double the = p->thm * p->pp, we = p->wm * p->pp;
        double c = cos(the), s = sin(the);
        if (driven) {
            double dtc = p->tdEff / dt;
            double vu = du * p->vbus - dtSign(p, p->iu) * p->vbus * dtc * (su == 2);
            double vv = dv * p->vbus - dtSign(p, p->iv) * p->vbus * dtc * (sv == 2);
            double vw = dw * p->vbus - dtSign(p, p->iw) * p->vbus * dtc * (sw == 2);
            double vn = (vu + vv + vw) / 3.0;
            double va = vu - vn, vb = (vv - vw) / sqrt(3.0);
            double valpha = va, vbeta = vb;
            double vd = valpha * c + vbeta * s, vq = -valpha * s + vbeta * c;
            double pin;
            p->dbgVd = vd; p->dbgVq = vq;
            if (p->openPh >= 0 && p->tNow >= p->openAt) {
                static const double sh[3] = {0.0, 2.0943951023931953, -2.0943951023931953};
                double vp[3] = {vu, vv, vw}, ip[3] = {p->iu, p->iv, p->iw};
                int x = (p->openPh == 0) ? 1 : 0, y = (p->openPh == 2) ? 1 : 2;
                double ex = -we * p->psi * sin(the - sh[x]), ey = -we * p->psi * sin(the - sh[y]);
                double i = ip[x], ia, ib;
                i += (vp[x] - vp[y] - 2.0 * rEff * i - (ex - ey)) / (2.0 * p->lD) * h;
                ip[x] = i; ip[y] = -i; ip[p->openPh] = 0.0;
                ia = ip[0]; ib = (ip[1] - ip[2]) / sqrt(3.0);
                p->id = ia * c + ib * s; p->iq = -ia * s + ib * c;
                pin = (vp[x] - vp[y]) * i + p->pstby;
            } else {
                double psiF = plant_psiEff(p), ldInc = p->lD, psiD = p->lD * p->id + psiF, did, diq;
                if (p->dSatI > 0 && p->id > 0) {
                    ldInc = p->lD / (1.0 + p->id / p->dSatI);
                    psiD = psiF + p->lD * p->dSatI * log(1.0 + p->id / p->dSatI);
                }
                static int s_extInit = 0; static double s_smI0 = 0, s_crossNl = 0, s_eddyF = 0;
                if (!s_extInit) {
                    s_smI0 = getenv("SIM_SMOOTH_SAT_I0") ? atof(getenv("SIM_SMOOTH_SAT_I0")) : 0.0;
                    s_crossNl = getenv("SIM_CROSS_NL") ? atof(getenv("SIM_CROSS_NL")) : 0.0;
                    s_eddyF = getenv("SIM_EDDY_HZ") ? atof(getenv("SIM_EDDY_HZ")) : 0.0;
                    s_extInit = 1;
                }
                double lqInc = p->lQ;
                if (s_smI0 > 0.0) {
                    double u = p->iq / s_smI0, u2 = u * u;
                    lqInc = p->lQ * (0.5 + 0.5 * (1.0 - u2) / ((1.0 + u2) * (1.0 + u2)));
                } else if (p->qSatK > 0 && fabs(p->iq) > p->qSatI0) {
                    double a = fabs(p->iq), sc = 1.0 - p->qSatK * (a - p->qSatI0);
                    lqInc = (sc > 0.5) ? p->lQ * (sc - p->qSatK * a) : 0.5 * p->lQ;
                    if (lqInc < 0.05 * p->lQ) lqInc = 0.05 * p->lQ;
                }
                if (s_crossNl > 0.0) {
                    lqInc /= (1.0 + s_crossNl * p->id * p->id);
                }
                double vEdD = 0.0, vEdQ = 0.0, lDeff = ldInc, lQeff = lqInc;
                if (s_eddyF > 0.0) {
                    double re = 0.5 * rEff;
                    double le = 0.1 * p->lQ;
                    lDeff = 0.9 * ldInc;
                    lQeff = 0.9 * lqInc;
                    vEdD = re * (p->id - g_sim_iEd);
                    vEdQ = re * (p->iq - g_sim_iEq);
                    g_sim_iEd += (vEdD / le) * h;
                    g_sim_iEq += (vEdQ / le) * h;
                }
                did = (vd - rEff * p->id - vEdD + we * p->lQ * p->iq) / lDeff;
                diq = (vq - rEff * p->iq - vEdQ - we * psiD) / lQeff;
                p->id += did * h; p->iq += diq * h;
                pin = 1.5 * (vd * p->id + vq * p->iq) + p->pstby;
            }
            if (p->cbus > 0) {
                double isrc = (p->vsrc - p->vbus) / p->rsrc; if (isrc < 0) isrc = 0;
                p->vbus += (isrc - pin / p->vbus) / p->cbus * h;
            }
        } else {
            p->id = 0; p->iq = 0; g_sim_iEd = 0.0; g_sim_iEq = 0.0;
        }
        {
            double ia = p->id * c - p->iq * s, ib = p->id * s + p->iq * c;
            p->iu = ia; p->iv = -0.5 * ia + sqrt(3.0) / 2 * ib; p->iw = -p->iu - p->iv;
        }
        p->te = 1.5 * p->pp * (plant_psiEff(p) * p->iq + (p->lD - p->lQ) * p->id * p->iq);
        {
            double im = sqrt(p->id * p->id + p->iq * p->iq);
            if (im > p->ipk) p->ipk = im;
            if (p->te < p->temin) p->temin = p->te;
            if (p->te > p->temax) p->temax = p->te;
        }
        {
            double tCog = (s_cogAmp != 0.0) ? s_cogAmp * sin(s_cogHarm * the) : 0.0;
            double tl = p->kFan * (p->wm <= 0 ? 1.0 : p->kFanRev) * p->wm * fabs(p->wm)
                      + p->loadStep * (p->wm >= 0 ? 1 : -1) + tCog;
            tl -= p->wind;
            double net = p->te - tl;
            if (fabs(p->wm) < 1e-3 && fabs(net) < p->tFric) { p->wm = 0; }
            else {
                net -= p->tFric * (p->wm > 0 ? 1 : (p->wm < 0 ? -1 : (net > 0 ? 1 : -1)));
                p->wm += net / p->J * h;
            }
        }
        p->thm += p->wm * h;
        if (p->thm > TWO_PI) p->thm -= TWO_PI;
        if (p->thm < 0) p->thm += TWO_PI;
    }
    p->tNow += dt;
    {
        const double kAdc = (double)R_SHUNT * (double)CURRENT_OPA_GAIN / (double)ADC_REFV * 4096.0 * s_iGain;
        int offRunU = (p->tNow > 0.22) ? s_offRun : 0;
        int offRunV = (p->tNow > 0.22) ? -s_offRun : 0;
        int u = (int)lround(p->adcOff[0] + offRunU - p->iu * kAdc + (urand(p) - 0.5) * 2.0 * p->noise);
        int v = (int)lround(p->adcOff[1] + offRunV - p->iv * kAdc * s_iGainV + (urand(p) - 0.5) * 2.0 * p->noise);
        int vb = (int)lround(p->vbus * s_vbusGain / (double)UDC_MAX * 4096.0);
        if (u < 0) u = 0; if (u > 4095) u = 4095;
        if (v < 0) v = 0; if (v > 4095) v = 4095;
        if (vb < 0) vb = 0; if (vb > 4095) vb = 4095;
        ADC_DMABuff[PL_ADC_U] = (uint16_t)u;
        ADC_DMABuff[PL_ADC_V] = (uint16_t)v;
        ADC_DMABuff[2] = 2048;
        ADC_DMABuff[3] = (uint16_t)vb;
    }
}

double plant_rpm(const Plant *p) { return p->wm * 60.0 / TWO_PI; }
double plant_theta_e(const Plant *p) { double t = fmod(p->thm * p->pp, TWO_PI); return t < 0 ? t + TWO_PI : t; }
