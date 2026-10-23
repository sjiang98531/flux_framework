/* Prints the gains fw_config.h derives from the physical spec, next to the
 * literal constants test33 used. Exit code 1 if any differs. */
#include <stdio.h>
#include "fw_config.h"
static int bad;
static void chk(const char *n, long v, long ref) { printf("%-28s %8ld   test33 %8ld   %s\n", n, v, ref, v == ref ? "OK" : "DIFF"); if (v != ref) bad = 1; }
int main(void)
{
    chk("CURRENT_KP_D", CURRENT_KP_D, 13747);
    chk("CURRENT_KI_D", CURRENT_KI_D, 274);
    chk("CURRENT_KP_Q", CURRENT_KP_Q, 13747);
    chk("CURRENT_KI_Q", CURRENT_KI_Q, 274);
    chk("SPEED_KP", SPEED_KP, 8000);
    chk("SPEED_KI", SPEED_KI, 100);
    chk("FLUXPLL_KP_Q31_PER_ERR", FLUXPLL_KP_Q31_PER_ERR, 25583);
    chk("FLUXPLL_KI_Q31_PER_ERR", FLUXPLL_KI_Q31_PER_ERR, 106);
    chk("SPEED_ACC_FF_IQ", SPEED_ACC_FF_IQ, Q15((200.0 / 501.0) / (5.0/2/3.139/0.1)));
    chk("STARTUP_PLL_FLOOR_Q31", STARTUP_PLL_FLOOR_Q31, Q31(0.004));
    chk("FLUX_STARTUP_ACC_STEP", FLUX_STARTUP_ACC_STEP, 41943);
    chk("FLUX_NOMINAL_Q15", FLUX_NOMINAL_Q15, 6111);
    chk("SPEED_TO_THETA", SPEED_TO_THETA, 1092);
    printf("exact: KPd %.3f KId %.3f KPspd %.3f KIspd %.3f KPpll %.3f KIpll %.3f\n",
        2*FW_PI*CURRENT_BW_HZ*MOTOR_LD_H*CUR_GAIN_SCALE, 2*FW_PI*CURRENT_BW_HZ*MOTOR_RS_OHM*CUR_GAIN_SCALE/PWM_FREQ,
        2*SPEED_ZETA*2*FW_PI*SPEED_WN_HZ*SPD_GAIN_SCALE, (2*FW_PI*SPEED_WN_HZ)*(2*FW_PI*SPEED_WN_HZ)*SPD_GAIN_SCALE/SLOWLOOP_FREQ,
        2*FLUXPLL_ZETA*2*FW_PI*FLUXPLL_WN_HZ*PLL_KS/FLUX_NOMINAL_Q15, (2*FW_PI*FLUXPLL_WN_HZ)*(2*FW_PI*FLUXPLL_WN_HZ)*PLL_KS/FLUX_NOMINAL_Q15/PWM_FREQ);
    return bad;
}
