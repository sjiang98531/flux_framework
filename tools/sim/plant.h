#ifndef SIM_PLANT_H
#define SIM_PLANT_H
#include <stdint.h>

typedef struct {
    /* parameters */
    double res, lD, lQ, psi, J, pp;
    double kFan, tFric;          /* load: kFan*w^2 + Coulomb friction */
    double vbus, tdEff;          /* bus voltage, effective dead time [s] */
    int adcOff[2];               /* ADC zero offsets (counts), phases U,V */
    double loadStep;             /* extra load torque [Nm] */
    double wind;                 /* external driving torque [Nm] (SIM_WIND) */
    double noise;
    double dbgVd, dbgVq;         /* 调试：转子坐标系下的端电压 */                /* ADC 噪声幅值（LSB，均匀分布 ±noise） */
    /* state */
    double id, iq, wm, thm;      /* dq currents, mech speed rad/s, mech angle */
    double iu, iv, iw;
    double te;
    uint32_t rng;
} Plant;

void plant_init(Plant *p, int ipm);
/* Advance one PWM period using the compares/enables currently in the EPWM
 * registers, then fill ADC_DMABuff[0..3] with the end-of-period samples. */
void plant_step(Plant *p, double dt);
double plant_rpm(const Plant *p);
double plant_theta_e(const Plant *p);   /* rad, 0..2pi */
#endif
