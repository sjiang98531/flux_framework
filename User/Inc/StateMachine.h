#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#include "stdlib.h"
#include "user_function.h"


/*! @brief Pointer to function */
typedef void (*pfcn_void_void)(void);

/*! @brief device fault typedef */
typedef uint16_t mcdef_fault_t;

/*! @brief States of machine enumeration */
typedef enum _m1_app_mainstate_t
{
    MainState_Fault = 0,
    MainState_Init  = 1,
    MainState_Stop  = 2,
    MainState_Run   = 3,    
} m1_app_mainstate_t; /* Main States */

/*! @brief States of machine enumeration */
typedef enum _m1_run_substate_t
{
    RunState_Charge = 0,
    RunState_Calib = 1,
    RunState_Align = 2,
    RunState_Startup = 3,
    RunState_Spin = 4,
    RunState_Torqueloop = 5,
} m1_run_substate_t; /* Run sub-states */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MC_FAULT_I_DCBUS_OVER 1		/* OverCurrent fault flag */
#define MC_FAULT_U_DCBUS_UNDER 2	/* Undervoltage fault flag */
#define MC_FAULT_U_DCBUS_OVER 3		/* Overvoltage fault flag */
#define MC_FAULT_ROTOR_BLOCKED 4	/* Blocked rotor fault flag */
#define MC_FAULT_PHASELESS 5		/* Phasee less fault flag */
#define MC_FAULT_SPEED_OC 6			/* Speed out of control fault flag */
//#define MC_FAULT_LOAD_OVER 4		/* Overload fault flag */
//#define MC_FAULT_SPEED_OVER 5		/* Over speed fault flag */

/* Sets the fault bit defined by faultid in the faults variable */
#define MC_FAULT_SET(faults, faultid) (faults |= ((mcdef_fault_t)1 << faultid))

/* Clears the fault bit defined by faultid in the faults variable */
#define MC_FAULT_CLEAR(faults, faultid) (faults &= ~((mcdef_fault_t)1 << faultid))

/* Check the fault bit defined by faultid in the faults variable, returns 1 or 0 */
#define MC_FAULT_CHECK(faults, faultid) ((faults & ((mcdef_fault_t)1 << faultid)) >> faultid)

/* Clears all fault bits in the faults variable */
#define MC_FAULT_CLEAR_ALL(faults) (faults = 0)


extern m1_app_mainstate_t eM1_MainState;                /* Main State */
extern m1_run_substate_t eM1_RunSubState;								/* Run Sub State */

/*! @brief Application sub-state function field - fast */
extern const pfcn_void_void s_STATE_FAST[4];

/*! @brief Application sub-state function field - slow */
extern const pfcn_void_void s_STATE_SLOW[4];

extern uint16_t hRemainder;
extern uint16_t u32FGPeriod;
extern uint16_t u16AbsSpdfilt;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void M1_Fault_Fast(void);
void M1_Init_Fast(void);
void M1_Stop_Fast(void);
void M1_Run_Fast(void);

void M1_Fault_Slow(void);
void M1_Init_Slow(void);
void M1_Stop_Slow(void);
void M1_Run_Slow(void);

static void M1_SwitchFaultStop(void);
static void M1_SwitchInitStop(void);
static void M1_SwitchStopFault(void);
static void M1_SwitchStopRun(void);
static void M1_SwitchRunFault(void);
static void M1_SwitchRunStop(void);

static void M1_RunCalibFast(void);
static void M1_RunChargeFast(void);
static void M1_RunAlignFast(void);
static void M1_RunStartupFast(void);
static void M1_RunSpinFast(void);
static void M1_RunFreewheelFast(void);
static void M1_RunTorqueloopFast(void);

static void M1_RunCalibSlow(void);
static void M1_RunChargeSlow(void);
static void M1_RunAlignSlow(void);
static void M1_RunStartupSlow(void);
static void M1_RunSpinSlow(void);
static void M1_RunFreewheelSlow(void);
static void M1_RunTorqueloopSlow(void);

static void M1_SwitchChargeCalib(void);
static void M1_SwitchRunCalibAlign(void);
static void M1_SwitchRunAlignStartup(void);
static void M1_SwitchRunAlignReady(void);
static void M1_SwitchRunStartupSpin(void);
static void M1_SwitchRunSpinStartup(void);
static void M1_SwitchRunStartupFreewheel(void);
static void M1_SwitchRunSpinFreewheel(void);
static void M1_SwitchRunFreewheelReady(void);
static void M1_SwitchRunCalibTorqueloop(void);
static void M1_SwitchRunTorqueloopSpin(void);

static void M1_FaultDetection(void);
static void Fault_LED_Disp(void);


#endif
