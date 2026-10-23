/* Closed-loop host simulation: firmware fast/slow loops + vendor lib model + plant.
 * Usage: sim <scenario> <csv_out> <hash_out>
 * The same file is linked against the old (test33) or the new (flux_framework)
 * user code through glue_*.c. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sc32_conf.h"
#include "motor_structure.h"
#include "pwm_gen.h"
#include "plant.h"

EPWM_TypeDef sim_EPWM; TIM_TypeDef sim_TIM0, sim_TIM3; ADC_TypeDef sim_ADC; WDT_TypeDef sim_WDT; GPIO_TypeDef sim_GPIOA;
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *a, uint32_t f) { (void)a; (void)f; return SET; }
void ADC_ClearFlag(ADC_TypeDef *a, uint32_t f) { (void)a; (void)f; }
FlagStatus TIM_GetFlagStatus(TIM_TypeDef *t, uint32_t f) { (void)t; (void)f; return RESET; }
void TIM_ClearFlag(TIM_TypeDef *t, uint32_t f) { (void)t; (void)f; }
void TIM_PWMSetDuty(TIM_TypeDef *t, uint32_t ch, uint32_t d) { (void)t; (void)ch; (void)d; }

extern USER_TypeDef USER;
extern FOC_TypeDef FOC;
extern volatile unsigned int eM1_MainState, eM1_RunSubState;

void glue_init(void);
void glue_isr(void);
void glue_main_poll(void);
size_t glue_export(uint8_t *buf);   /* observer/PLL canonical state */
void glue_cfg(int scenario);        /* scenario-specific runtime config (new code only) */
int16_t glue_psi_mag(void);

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    while (n--) { h ^= *b++; h *= 1099511628211ULL; }
    return h;
}

static int16_t rpmQ15(double rpm) { return (int16_t)(rpm / 3200.0 * 32768.0); }

/* scenario: returns speed command (rpm, firmware sign) at time t */
static double profile(int sc, double t, Plant *pl)
{
    switch (sc) {
    case 0: /* baseline: start 293, step 1172, 600, reverse -400, stop, restart 1000 */
        if (t < 0.2) return 0;
        if (t < 4.0) return 293;
        if (t < 10.0) return 1172;
        if (t < 14.0) return 600;
        if (t < 20.0) return -400;
        if (t < 21.0) return 0;
        if (t < 26.0) return 1000;
        return 1000;
    case 1: /* load step at 1000 rpm */
        if (t > 8.0 && t < (getenv("SIM_LOAD_END") ? atof(getenv("SIM_LOAD_END")) : 12.0)) pl->loadStep = getenv("SIM_LOAD") ? atof(getenv("SIM_LOAD")) : 0.3; else pl->loadStep = 0.0;
        if (t < 0.2) return 0;
        return 1000;
    case 2: /* high speed: 1800 -> 2600 (field weakening region at 310 V) */
        if (t < 0.2) return 0;
        if (t < 8.0) return 1800;
        return 2600;
    case 4: /* start/stop cycling, short commands */
        { double tc = fmod(t, 2.5);
          if (t < 0.2) return 0;
          return tc < 1.6 ? 600 : 0; }
    case 5: /* hard reversal while fast */
        if (t < 0.2) return 0;
        if (t < 8.0) return 1000;
        if (t < 16.0) return -1000;
        return 300;
    default:
        return t < 0.2 ? 0 : 1000;
    }
}

int main(int argc, char **argv)
{
    int sc = argc > 1 ? atoi(argv[1]) : 0;
    const char *csvName = argc > 2 ? argv[2] : "sim.csv";
    const char *hashName = argc > 3 ? argv[3] : NULL;
    double tEnd = argc > 4 ? atof(argv[4]) : 28.0;
    int ipm = argc > 5 ? atoi(argv[5]) : 0;
    FILE *csv = fopen(csvName, "w");
    FILE *hf = hashName ? fopen(hashName, "wb") : NULL;
    Plant pl;
    long k, nTicks = (long)(tEnd * 16000.0);
    const double dt = 1.0 / 16000.0;
    uint8_t buf[512];

    plant_init(&pl, ipm);
    if (sc == 2) pl.vbus = 310.0;
    memset(&sim_EPWM, 0, sizeof sim_EPWM);
    glue_init();
    glue_cfg(sc);
    fprintf(csv, "t,main,run,cmd_rpm,filt_rpm,pll_rpm,true_rpm,iq_A,id_A,iqcmd_A,idcmd_A,theta_err_deg,psi_mag,vd,vq,fault\n");
    for (k = 0; k < nTicks; k++) {
        double t = k * dt;
        double cmd = profile(sc, t, &pl);
        USER.s16VspCmd = rpmQ15(cmd);
        sim_TIM3.TIM_CNT = (uint32_t)(k * 7u) & 0xffffu;
        plant_step(&pl, dt);
        glue_isr();
        glue_main_poll();
        if (hf) {
            uint64_t h = 1469598103934665603ULL;
            size_t n = glue_export(buf);
            h = fnv(h, &FOC, sizeof FOC);
            h = fnv(h, &USER, sizeof USER);
            h = fnv(h, &pwm_gen, sizeof pwm_gen);
            h = fnv(h, (const void *)&sim_EPWM, sizeof sim_EPWM);
            h = fnv(h, (const void *)&eM1_MainState, 4);
            h = fnv(h, (const void *)&eM1_RunSubState, 4);
            h = fnv(h, buf, n);
            fwrite(&h, 8, 1, hf);
        }
        if ((k & 15) == 0) {
            double thFw = (120.0 / 180.0 * M_PI) - plant_theta_e(&pl);
            double thHat = (double)(uint16_t)FOC.s16CmdTheta / 65536.0 * 2 * M_PI;
            double e = fmod(thHat - thFw + 9 * M_PI, 2 * M_PI) - M_PI;
            fprintf(csv, "%.4f,%u,%u,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.2f,%d,%d,%d,%d\n", t,
                    (unsigned)eM1_MainState, (unsigned)eM1_RunSubState, cmd,
                    FOC.s16SpdFilt * 3200.0 / 32768.0, (FOC.sPLL.s32Speed >> 16) * 3200.0 / 32768.0,
                    -plant_rpm(&pl),
                    FOC.sIdq.s16Q * 7.9643 / 32768.0, FOC.sIdq.s16D * 7.9643 / 32768.0,
                    FOC.sIdq_cmd.s16Q * 7.9643 / 32768.0, FOC.sIdq_cmd.s16D * 7.9643 / 32768.0,
                    e * 180.0 / M_PI, glue_psi_mag(), FOC.sVdq.s16D, FOC.sVdq.s16Q, 0);
        }
    }
    fclose(csv);
    if (hf) fclose(hf);
    return 0;
}
