#ifndef SIM_PLANT_H
#define SIM_PLANT_H
#include <stdint.h>

typedef struct {
    /* parameters */
    double res, lD, lQ, psi, J, pp;
    double kFan, kFanRev, tFric;          /* load: kFan*w^2 + Coulomb friction */
    double vbus, tdEff;          /* bus voltage, effective dead time [s] */
    double dtI0;                 /* dead-time error zero-crossing width [A], 0 = sign(i) */
    double dSatI;                /* d-axis saturation current [A] (id > 0 only), 0 = linear */
    double qSatK, qSatI0;        /* q-axis secant L(i) = Lq*max(1 - k*(|iq| - I0), 0.5), k = 0 off (SIM_QSAT / SIM_QSAT_I0) */
    double vsrc;                 /* 整流后空载母线（电网峰值），SIM_VBUS 设定 */
    int adcOff[2];               /* ADC zero offsets (counts), phases U,V */
    double loadStep;             /* extra load torque [Nm] */
    double wind;                 /* external driving torque [Nm] (SIM_WIND) */
    double noise;
    double cbus, rsrc, pstby;    /* 母线模型（SIM_CBUS = 母线电容 uF，0 = 母线固定）：电容 + 二极管整流桥（只能往母线送电）+ 待机损耗 W */
    double dbgVd, dbgVq;         /* 调试：转子坐标系下的端电压 */                /* ADC 噪声幅值（LSB，均匀分布 ±noise） */
    /* state */
    double id, iq, wm, thm;      /* dq currents, mech speed rad/s, mech angle */
    double iu, iv, iw;
    double te;
    double ipk, temin, temax;    /* 本 CSV 行（1 ms）内的相电流幅值峰值、转矩极值（每行输出后清零） */
    uint32_t rng;
    int openPh; double openAt, tNow;   /* 开相故障注入（SIM_OPEN=u|v|w，SIM_OPEN_T 秒起）：该相引线断开，另两相串联 */
} Plant;

void plant_init(Plant *p, int ipm);
/* Advance one PWM period using the compares/enables currently in the EPWM
 * registers, then fill ADC_DMABuff[0..3] with the end-of-period samples. */
void plant_step(Plant *p, double dt);
double plant_rpm(const Plant *p);
double plant_theta_e(const Plant *p);   /* rad, 0..2pi */
#endif
