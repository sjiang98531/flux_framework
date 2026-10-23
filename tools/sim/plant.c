/* PMSM + inverter + ADC model for the host simulation.
 * Board mapping (see Update_PWM / ADC_IRQHandler): phase U <- EPWM_DT[4],
 * enable bits 4/5, ADC ch0; V <- DT[2], bits 2/3, ADC ch1; W <- DT[0], bits 0/1.
 * Pole duty = (PERIOD - compare)/PERIOD ("smaller compare = higher voltage").
 * Current sense: ADC = 2048 - I*Rshunt*Gain/5V*4096 (firmware: I = offset - raw). */
#include <math.h>
#include <stdlib.h>
#include "plant.h"
#include "sc32_conf.h"

extern uint16_t ADC_DMABuff[14];
#define PERIOD 2250.0
#define TWO_PI 6.283185307179586

static double urand(Plant *p) { p->rng = p->rng * 1664525u + 1013904223u; return (p->rng >> 8) / 16777216.0; }

void plant_init(Plant *p, int ipm)
{
    p->res = 14.7; p->pp = 5.0; p->psi = 0.141;
    if (ipm) { p->lD = 0.040; p->lQ = 0.060; } else { p->lD = p->lQ = 0.046; }
    p->J = 1.5 * 5.0 * 0.141 / (501.0 * TWO_PI / 60.0);   /* 501 rpm/s per A (test29 ID) */
    p->kFan = 1.5 * 5.0 * 0.141 * 0.292 / pow(1172.0 * TWO_PI / 60.0, 2.0);
    p->tFric = 0.01;
    p->vbus = 310.0; p->tdEff = 0.2e-6;
    p->adcOff[0] = 2048 + 3; p->adcOff[1] = 2048 - 2;
    p->loadStep = 0.0;
    p->id = p->iq = p->wm = 0.0; p->thm = 1.5; p->iu = p->iv = p->iw = 0.0; p->te = 0.0;
    p->rng = 12345u;
    { const char *e;
      if ((e = getenv("SIM_TD"))) p->tdEff = atof(e);
      if ((e = getenv("SIM_OFF"))) { p->adcOff[0] = 2048 + atoi(e); p->adcOff[1] = 2048 - atoi(e); }
      if ((e = getenv("SIM_TH0"))) p->thm = atof(e);
      if ((e = getenv("SIM_FRIC"))) p->tFric = atof(e);
      if ((e = getenv("SIM_W0"))) p->wm = atof(e) * TWO_PI / 60.0;
      if ((e = getenv("SIM_VBUS"))) p->vbus = atof(e); }
}

static int phaseState(uint32_t chn, int bitHi)   /* 2 = PWM, 1 = low side, 0 = float */
{
    int hi = (chn >> bitHi) & 1, lo = (chn >> (bitHi + 1)) & 1;
    if (hi && lo) return 2;
    if (lo) return 1;
    if (hi) return 3;
    return 0;
}

void plant_step(Plant *p, double dt)
{
    const int N = 8;
    double h = dt / N;
    uint32_t chn = EPWM->EPWM_CHN;
    int su = phaseState(chn, 4), sv = phaseState(chn, 2), sw = phaseState(chn, 0);
    double du = (PERIOD - (double)(int32_t)EPWM->EPWM_DT[4]) / PERIOD;
    double dv = (PERIOD - (double)(int32_t)EPWM->EPWM_DT[2]) / PERIOD;
    double dw = (PERIOD - (double)(int32_t)EPWM->EPWM_DT[0]) / PERIOD;
    int driven = (su && sv && sw);
    int k;
    if (du < 0) du = 0; if (du > 1) du = 1;
    if (dv < 0) dv = 0; if (dv > 1) dv = 1;
    if (dw < 0) dw = 0; if (dw > 1) dw = 1;
    if (su == 1) du = 0; if (sv == 1) dv = 0; if (sw == 1) dw = 0;
    if (su == 3) du = 1; if (sv == 3) dv = 1; if (sw == 3) dw = 1;
    for (k = 0; k < N; k++) {
        double the = p->thm * p->pp, we = p->wm * p->pp;
        double c = cos(the), s = sin(the);
        if (driven) {
            double dtc = p->tdEff / (dt);
            double vu = du * p->vbus - (p->iu > 0 ? 1 : -1) * p->vbus * dtc * (su == 2);
            double vv = dv * p->vbus - (p->iv > 0 ? 1 : -1) * p->vbus * dtc * (sv == 2);
            double vw = dw * p->vbus - (p->iw > 0 ? 1 : -1) * p->vbus * dtc * (sw == 2);
            double vn = (vu + vv + vw) / 3.0;
            double va = vu - vn, vb = (vv - vw) / sqrt(3.0);
            double valpha = va, vbeta = vb;
            double vd = valpha * c + vbeta * s, vq = -valpha * s + vbeta * c;
            double did = (vd - p->res * p->id + we * p->lQ * p->iq) / p->lD;
            double diq = (vq - p->res * p->iq - we * (p->lD * p->id + p->psi)) / p->lQ;
            p->id += did * h; p->iq += diq * h;
        } else {
            /* open phases: no current path below the bus voltage */
            p->id = 0; p->iq = 0;
        }
        {
            double ia = p->id * c - p->iq * s, ib = p->id * s + p->iq * c;
            p->iu = ia; p->iv = -0.5 * ia + sqrt(3.0) / 2 * ib; p->iw = -p->iu - p->iv;
        }
        p->te = 1.5 * p->pp * (p->psi * p->iq + (p->lD - p->lQ) * p->id * p->iq);
        {
            double tl = p->kFan * p->wm * fabs(p->wm) + p->loadStep * (p->wm >= 0 ? 1 : -1);
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
    {
        const double kAdc = 0.1 * 3.139 / 5.0 * 4096.0;
        int u = (int)lround(p->adcOff[0] - p->iu * kAdc + (urand(p) - 0.5) * 2.0);
        int v = (int)lround(p->adcOff[1] - p->iv * kAdc + (urand(p) - 0.5) * 2.0);
        if (u < 0) u = 0; if (u > 4095) u = 4095;
        if (v < 0) v = 0; if (v > 4095) v = 4095;
        ADC_DMABuff[0] = (uint16_t)u;
        ADC_DMABuff[1] = (uint16_t)v;
        ADC_DMABuff[2] = 2048;
        ADC_DMABuff[3] = (uint16_t)lround(p->vbus / 475.0 * 4096.0);
    }
}

double plant_rpm(const Plant *p) { return p->wm * 60.0 / TWO_PI; }
double plant_theta_e(const Plant *p) { double t = fmod(p->thm * p->pp, TWO_PI); return t < 0 ? t + TWO_PI : t; }
