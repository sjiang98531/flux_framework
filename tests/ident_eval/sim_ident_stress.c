double g_sim_irated_k = 1.0;
static int g_sim_faultCount = 0, g_sim_inFault = 0;
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sc32_conf.h"
#include "plant.h"
#include "cfg_derived.h"
#include "supervisor.h"
#include "identify.h"
#include "param.h"
#include "meas.h"
#include "foc_core.h"
#include "flux_obs.h"
#include "diag.h"
#include "ipd.h"
#include "openloop.h"
uint8_t OpenLoop_BoostLevel(void) __attribute__((weak));   /* 旧版本固件无此函数时为 0 */

EPWM_TypeDef sim_EPWM;
TIM_TypeDef sim_TIM0, sim_TIM3;
ADC_TypeDef sim_ADC;
WDT_TypeDef sim_WDT;
GPIO_TypeDef sim_GPIOA;
SCB_TypeDef sim_SCB;
void ADC_IRQHandler(void);
void PendSV_Handler(void);

FlagStatus TIM_GetFlagStatus(TIM_TypeDef *t, uint32_t f) { (void)t; (void)f; return RESET; }
void TIM_ClearFlag(TIM_TypeDef *t, uint32_t f) { (void)t; (void)f; }
void TIM_PWMSetDuty(TIM_TypeDef *t, uint32_t ch, uint32_t d) { (void)t; (void)ch; (void)d; }
void Motor_Hardware_Init(void) {}
void Motor_NVIC_Config(void) {}

int main(int argc, char **argv) {
    Plant pl;
    double sim_secs = argc > 1 ? atof(argv[1]) : 35.0;
    double dt = 1.0 / (double)PWM_FREQ;
    long nTicks = (long)(sim_secs * PWM_FREQ);
    int mode = argc > 2 ? atoi(argv[2]) : IDENT_MODE_STATIC;
    double cmd_rpm = argc > 3 ? atof(argv[3]) : MOTOR_SPEED_RATED_RPM;
    int abort_mode = getenv("SIM_ABORT_PHASE") ? atoi(getenv("SIM_ABORT_PHASE")) : 0;
    int aborted = 0;

    if (getenv("SIM_IRATED_K")) g_sim_irated_k = atof(getenv("SIM_IRATED_K"));

    plant_init(&pl, 0);
    if (getenv("SIM_LD")) pl.lD = atof(getenv("SIM_LD"));
    if (getenv("SIM_LQ")) pl.lQ = atof(getenv("SIM_LQ"));
    if (getenv("SIM_RS")) pl.res = atof(getenv("SIM_RS"));
    if (getenv("SIM_PSI")) pl.psi = atof(getenv("SIM_PSI"));
    if (getenv("SIM_QSAT")) pl.qSatK = atof(getenv("SIM_QSAT"));
    if (getenv("SIM_QSAT_I0")) pl.qSatI0 = atof(getenv("SIM_QSAT_I0"));
    memset(&sim_EPWM, 0, sizeof(sim_EPWM));
    Ctrl_Init();
    gIdentCfg.mode = (uint8_t)mode;
    if (getenv("SIM_BIASX")) gIdentCfg.biasX = (uint8_t)atoi(getenv("SIM_BIASX"));
    if (getenv("SIM_ALIGNDEG")) gIdentCfg.alignDeg = (int16_t)atoi(getenv("SIM_ALIGNDEG"));

    int32_t midKpD = -1, midKpQ = -1;
    /* 起动敏感度实验：SIM_PK_<项>=倍数，把控制器参数 gParam 相对电机真值缩放（真值 = 配置标称），用于评估各参数误差容限 */
    int sens = 0;
    {
        MotorParam_TypeDef pk = gParam;
        const char *e;
        if ((e = getenv("SIM_PK_RS")))  { pk.rsOhm *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_L")))   { pk.ldH *= atof(e); pk.lqH *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_LD")))  { pk.ldH *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_PSI"))) { pk.psiWb *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_ACC"))) { pk.accRpmSPerA *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_DT")))  { pk.dtEffS *= atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_LSMAX"))) { pk.lsMaxA = atof(e); sens = 1; }
        if ((e = getenv("SIM_PK_LSPER"))) { pk.lsPerA = atof(e); if ((e = getenv("SIM_PK_LSKNEE"))) pk.lsKneeA = atof(e); sens = 1; }
        if (sens && !Param_ApplyNow(&pk)) { printf("SENS reject\n"); }
    }
    double tOL = -1, tSpin = -1; int spinExit = 0, lastRun = -1; double rpmErrMax2s = 0;
    long acStartTick = -1;
    for (long k = 0; k < nTicks; k++) {
        double t = k * dt;
        int done = (gIdent.state == IDENT_DONE) || (mode == IDENT_MODE_STATIC && gIdentTest.phase == IDENT_PH_DONE);
        int stopNow = (t < 0.2 || (mode == IDENT_MODE_STATIC && done) || (abort_mode == 2 && aborted));
        gSup.speedCmd = stopNow ? 0 : (int16_t)(cmd_rpm / SPEED_MAX * 32768.0);
        sim_TIM3.TIM_CNT = (uint32_t)(k * 7u) & 0xffffu;
        plant_step(&pl, dt);
        sim_ADC.ADC_STS = ADC_Flag_EOSIF0;
        ADC_IRQHandler();
        if (sim_SCB.ICSR & SCB_ICSR_PENDSVSET_Msk) {
            sim_SCB.ICSR = 0;
            PendSV_Handler();
        }
        Param_Background();
        if (getenv("SIM_DIAG_SU") && gSup.mainState == MainState_Run && (k % (long)(0.005 * PWM_FREQ)) == 0 && t < atof(getenv("SIM_DIAG_SU"))) {
            printf("SU t=%.3f run=%d th=%.1f rotor=%.1f rpm=%.1f id=%d iq=%d olspd=%d\n", t, (int)gSup.runState,
                   (double)(uint16_t)gFoc.theta * 360.0 / 65536.0, plant_theta_e(&pl) * 180.0 / 3.14159265358979323846,
                   plant_rpm(&pl), (int)gFoc.idqRef.s16D, (int)gFoc.idqRef.s16Q, (int)gMeas.speed);
        }
        if (sens && gSup.mainState == MainState_Run) {
            int rs = (int)gSup.runState;
            if (rs == RunState_OpenLoop && tOL < 0) tOL = t;
            if (rs == RunState_Spin && tSpin < 0) tSpin = t;
            if (lastRun == RunState_Spin && rs != RunState_Spin) spinExit++;
            lastRun = rs;
            if (t > sim_secs - 2.0) { double er = fabs(fabs(plant_rpm(&pl)) - cmd_rpm) / cmd_rpm; if (er > rpmErrMax2s) rpmErrMax2s = er; }
        }
        if (getenv("SIM_DIAG_TIER")) {
            static int lastPh = -1, lastRs = -1; static long lastK = 0;
            if ((int)gIdentTest.phase != lastPh || (int)gSup.runState != lastRs || ((getenv("SIM_DIAG_ALL") || gIdentTest.phase == IDENT_PH_AC) && k - lastK > (long)((getenv("SIM_DIAG_DT") ? atof(getenv("SIM_DIAG_DT")) : 0.25) * PWM_FREQ))) {
                lastPh = (int)gIdentTest.phase; lastRs = (int)gSup.runState; lastK = k;
                printf("TRACE t=%.3f ph=%d main=%d run=%d rpm=%.0f rotor=%.2f align=%.2f\n", t, lastPh, (int)gSup.mainState, (int)gSup.runState, plant_rpm(&pl),
                       plant_theta_e(&pl) * 180.0 / 3.14159265358979323846,
                       (double)(uint16_t)Ident_AlignTheta() * 360.0 / 65536.0);
            }
        }
        if (gIdentTest.phase == IDENT_PH_AC && acStartTick < 0) {
            acStartTick = k;
            midKpD = gFoc.piD.qKp;
            midKpQ = gFoc.piQ.qKp;
        }
        if (abort_mode && !aborted && acStartTick >= 0 && (k - acStartTick) > (long)(0.05 * PWM_FREQ)) {
            aborted = 1;
            if (abort_mode == 1) Sup_RaiseFault(MC_FAULT_I_DCBUS_OVER);
        }
        if (gSup.fault) {
            /* SIM_FAULT_CONTINUE=1：故障后继续仿真（验证故障释放后的自动重启），只在进入故障时打印一次 */
            if (!g_sim_inFault) {
                g_sim_inFault = 1;
                g_sim_faultCount++;
                if (!abort_mode) fprintf(stderr, "FAULT 0x%x at t=%.3f failLine=%u\n", (unsigned)gSup.fault, t, (unsigned)gFluxDiag.cfgFailLine);
            }
            if (!getenv("SIM_FAULT_CONTINUE")) break;
        } else {
            g_sim_inFault = 0;
        }
        if (abort_mode == 2 && aborted && gSup.mainState == MainState_Stop) break;
        if (mode == IDENT_MODE_STATIC && done && gSup.mainState == MainState_Stop && !gParamStat.pending) break;
        if (mode == IDENT_MODE_AUTO && gIdent.state == IDENT_DONE && !gParamStat.pending && gSup.runState == RunState_Spin) {
            static long doneTick = -1;
            if (doneTick < 0) doneTick = k;
            if (k - doneTick > (long)(0.5 * PWM_FREQ)) break;
        }
    }
    if (abort_mode) {
        printf("ABORT mode=%d fault=0x%x main=%d midKp=(%ld,%ld) endKp=(%ld,%ld) nomKp=(%ld,%ld) midKi=(%ld,%ld) endKi=(%ld,%ld) nomKi=(%ld,%ld)\n",
               abort_mode, (unsigned)gSup.fault, (int)gSup.mainState,
               (long)midKpD, (long)midKpQ, (long)gFoc.piD.qKp, (long)gFoc.piQ.qKp, (long)gGain.curKpD, (long)gGain.curKpQ,
               (long)gFoc.piD.qKp, (long)gFoc.piQ.qKp, (long)gFoc.piD.qKi, (long)gFoc.piQ.qKi, (long)gGain.curKiD, (long)gGain.curKiQ);
        return 0;
    }
    double scale_raw = (double)MOTOR_I_RATED_A / 0.75;
    double scale_max = IDENT_I_SCALE_MAX;
    double iscale = scale_raw < scale_max ? scale_raw : scale_max;
    double imax_swp = 2.5 * iscale;
    double spd_lim = (double)SPD_PI_LIMIT;
    double i_run = spd_lim < (double)I_MAX ? spd_lim : (double)I_MAX;
    double end_the_deg = plant_theta_e(&pl) * 180.0 / 3.14159265358979323846;
    double ipd_th_deg = (double)((uint16_t)gIpd.theta) * 360.0 / 65536.0;
    double id_max_swp = 0.0;
    for (int i = 0; i < 5; i++) {
        if (gIdent.lqAtA[i] > id_max_swp) id_max_swp = gIdent.lqAtA[i];
    }
    if (getenv("SIM_DIAG_TIER")) printf("PARAM rs=%.5g ld=%.5g lq=%.5g dt=%.4g psi=%.5g acc=%.5g knee=%.4g per=%.5g lsmax=%.4g src=%u\n",
        gParam.rsOhm, gParam.ldH, gParam.lqH, gParam.dtEffS * 1e6, gParam.psiWb, gParam.accRpmSPerA, gParam.lsKneeA, gParam.lsPerA, gParam.lsMaxA, (unsigned)gParamStat.source);
    if (sens) printf("SENS fault=0x%x main=%d run=%d tOL=%.3f tSpin=%.3f spinExit=%d rpm=%.1f errMax2s=%.3f nFault=%d boost=%u\n",
                    (unsigned)gSup.fault, (int)gSup.mainState, (int)gSup.runState, tOL, tSpin, spinExit, plant_rpm(&pl), rpmErrMax2s,
                    g_sim_faultCount, (unsigned)(OpenLoop_BoostLevel ? OpenLoop_BoostLevel() : 0));
    if (getenv("SIM_DIAG_TIER")) {
        printf("TIER");
        for (int i = 0; i < 5; i++) printf(" a%d=%.4f h%d=%.6g", i, gIdent.lqAtA[i], i, gIdent.lqAtH[i]);
        printf(" k04=%.6g k10=%.6g kM=%.6g\n", gIdent.kSwing[0], gIdent.kSwing[1], gIdent.kSwingModel);
    }
    printf("TRUE rs=%.6f ld=%.6g lq=%.6g ksat=%.6f i0sat=%.6f psi=%.6g imax=%.4f irun=%.4f imaxhw=%.4f dti0=%.4f iref0=%.4f\n",
           pl.res, pl.lD, pl.lQ, pl.qSatK, pl.qSatI0, pl.psi, imax_swp, i_run, (double)I_MAX, (double)INVERTER_DT_I0_A, 0.4 * iscale);
    printf("RES state=%d phase=%u fail=%u warn=0x%x snrMin=%.3f dlyTicks=%.3f rs=%.6f dt=%.4f rlR=%.4f rlL=%.6g ld=%.6g lq=%.6g knee=%.4f perA=%.5f dlySh=%.3f psi=%.6g acc=%.2f acHz=%.1f,%.1f,%.1f,%.1f acLd=%.6g,%.6g,%.6g acMag=%.6g,%.6g,%.6g acR=%.4f,%.4f,%.4f iq04=%.4f idMaxA=%.4f src=%u rej=%u pLd=%.6g pLq=%.6g pKnee=%.4f pPerA=%.5f pLsMax=%.4f ipdOk=%u ipdTh=%.1f endThe=%.1f ipk=%.3f\n",
           (int)gIdent.state, (unsigned)gIdentTest.phase, (unsigned)gFluxDiag.cfgFailLine, (unsigned)gIdent.warn,
           gIdent.snrMin, gIdent.acDlyTicks, gIdent.rsOhm, gIdent.deadTimeS*1e6, gIdent.rlR, gIdent.rlL,
           gIdent.ldH, gIdent.lqH, gIdent.lsSatKneeA, gIdent.lsSatPerA, gIdent.acDlyShift,
           gIdent.psiWb, gIdent.accRpmSPerA,
           gIdent.acHz[0], gIdent.acHz[1], gIdent.acHz[2], gIdent.acHz[3],
           gIdent.acLdH[0], gIdent.acLdH[1], gIdent.acLdH[2],
           gIdent.acLdMagH[0], gIdent.acLdMagH[1], gIdent.acLdMagH[2],
           gIdent.acRohm[0], gIdent.acRohm[1], gIdent.acRohm[2], gIdent.lqAtA[0], id_max_swp,
           (unsigned)gParamStat.source, (unsigned)gParamStat.rejectMask,
           gParam.ldH, gParam.lqH, gParam.lsKneeA, gParam.lsPerA, gParam.lsMaxA,
           (unsigned)gIpd.ok, ipd_th_deg, end_the_deg, pl.ipk);
    return 0;
}
