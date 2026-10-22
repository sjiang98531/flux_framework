
#ifndef __sc32f10xx_FOC_H
#define __sc32f10xx_FOC_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
  volatile unsigned       int INSTR;     /*!<                      */
  volatile unsigned       int CONFIG;    /*!< Address offset: 0x04 */
  volatile unsigned       int MADDR;     /*!< Address offset: 0x08 */
  volatile unsigned       int SADDR;     /*!< Address offset: 0x0C */
  volatile unsigned       int PWMPRD;    /*!< Address offset: 0x10 */
  volatile unsigned short int PI_SD_KP;  /*!< Address offset: 0x14 */
  volatile unsigned short int PI_SD_KI;  /*!< Address offset: 0x16 */
  volatile unsigned short int PI_SD_KC;  /*!< Address offset: 0x18 */
  volatile unsigned short int PI_SD_MAX; /*!< Address offset: 0x1A */
  volatile unsigned short int PI_SQ_KP;  /*!< Address offset: 0x1C */
  volatile unsigned short int PI_SQ_KI;  /*!< Address offset: 0x1E */
  volatile unsigned short int PI_SQ_KC;  /*!< Address offset: 0x20 */
  volatile unsigned short int PI_SQ_MAX; /*!< Address offset: 0x22 */
  volatile unsigned short int PI_P_KP;   /*!< Address offset: 0x24 */
  volatile unsigned short int PI_P_KI;   /*!< Address offset: 0x26 */
  volatile unsigned short int PI_P_KC;   /*!< Address offset: 0x28 */
  volatile unsigned short int PI_P_MAX;  /*!< Address offset: 0x2A */
} FOC_TypeDef;

#define FOC  ((FOC_TypeDef *) 0x40010400)

//=== auto gen =====
typedef struct
{
  volatile short int u;
  volatile short int v;
  volatile short int w;
  volatile short int Theta;
  volatile short int Alpha;
  volatile short int Beta;
  volatile short int arctan;
  volatile short int d;
  volatile short int q;
  volatile short int filler;
} CLARK_AND_PARK, *CLARK_AND_PARK_handle;

typedef struct
{
  volatile short int d;
  volatile short int q;
  volatile short int Theta;
  volatile short int Alpha;
  volatile short int Beta;
  volatile short int arctan;
  volatile short int Va;
  volatile short int Vb;
  volatile short int Vc;
  volatile short int sector;
  volatile short int duty_a;
  volatile short int duty_b;
  volatile short int duty_c;
  volatile short int duty_d;
} InvPARK_AND_SVPWM, *InvPARK_AND_SVPWM_handle;

typedef struct
{
  volatile short int InMeas;
  volatile short int InRef;
  volatile short int Ui;
  volatile short int Out;
} PI_S, *PI_S_handle;

typedef struct
{
  volatile short int InMeas;
  volatile short int InRef;
  volatile short int Sum;
  volatile short int Out;
} PI_P, *PI_P_handle;

typedef struct
{
  volatile short int Cos;
  volatile short int Sin;
  volatile short int ini_x;
  volatile short int ini_y;
  volatile short int Theta;
  volatile unsigned short int sqrt_out;
  volatile unsigned int sqrt_in;
  volatile int alu_a;
  volatile int alu_b;
  volatile int alu_c;
  volatile int alu_d;
} MATHS, *MATHS_handle;

typedef struct
{
  CLARK_AND_PARK I;
  InvPARK_AND_SVPWM V;
  PI_S PI_SD;
  PI_S PI_SQ;
  PI_P PI_P;
  MATHS Math;
} FOC_VAL, *FOC_VAL_handle;


#define foc_val_I_u 0x00
#define foc_val_I_v 0x02
#define foc_val_I_w 0x04
#define foc_val_I_Theta 0x06
#define foc_val_I_Alpha 0x08
#define foc_val_I_Beta 0x0A
#define foc_val_I_arctan 0x0C
#define foc_val_I_d 0x0E
#define foc_val_I_q 0x10
#define foc_val_I_filler 0x12
#define foc_val_V_d 0x14
#define foc_val_V_q 0x16
#define foc_val_V_Theta 0x18
#define foc_val_V_Alpha 0x1A
#define foc_val_V_Beta 0x1C
#define foc_val_V_arctan 0x1E
#define foc_val_V_Va 0x20
#define foc_val_V_Vb 0x22
#define foc_val_V_Vc 0x24
#define foc_val_V_sector 0x26
#define foc_val_V_duty_a 0x28
#define foc_val_V_duty_b 0x2A
#define foc_val_V_duty_c 0x2C
#define foc_val_V_duty_d 0x2E
#define foc_val_PI_SD_InMeas 0x30
#define foc_val_PI_SD_InRef 0x32
#define foc_val_PI_SD_Ui 0x34
#define foc_val_PI_SD_Out 0x36
#define foc_val_PI_SQ_InMeas 0x38
#define foc_val_PI_SQ_InRef 0x3A
#define foc_val_PI_SQ_Ui 0x3C
#define foc_val_PI_SQ_Out 0x3E
#define foc_val_PI_P_InMeas 0x40
#define foc_val_PI_P_InRef 0x42
#define foc_val_PI_P_Sum 0x44
#define foc_val_PI_P_Out 0x46
#define foc_val_Math_Cos 0x48
#define foc_val_Math_Sin 0x4A
#define foc_val_Math_ini_x 0x4C
#define foc_val_Math_ini_y 0x4E
#define foc_val_Math_Theta 0x50
#define foc_val_Math_sqrt_out 0x52
#define foc_val_Math_sqrt_in 0x54
#define foc_val_Math_alu_a 0x58
#define foc_val_Math_alu_b 0x5C
#define foc_val_Math_alu_c 0x60
#define foc_val_Math_alu_d 0x64

#define Declare_for_FOC      FOC_VAL MR_foc; \
                             unsigned int Instr_PARK[3] ; \
                             unsigned int Instr_InvPARK_to_SVPWM[4];

extern FOC_VAL MR_foc;
extern unsigned int Instr_PARK[3] ;
extern unsigned int Instr_InvPARK_to_SVPWM[4];


#define W16 1
#define W32 0
#define HH 24
#define HL 16
#define LH 8
#define LL 0
#define FILLER     0x00
#define SERIES_D_M 0xAD
#define SERIES_D_0 0x5D
#define SERIES_Q_M 0xA9
#define SERIES_Q_0 0x59
#define PARALLEL_M 0x0A
#define PARALLEL_0 0x05
#define FOC_INSTR_SIGNED_EXT 0xE1000000
#define FOC_INSTR_NEGATIVE   0xD2000000
#define FOC_INSTR_SHIFTER    0xC3000000
#define FOC_INSTR_ADD        0xB4000000
#define FOC_INSTR_SUB        0xA5000000
#define FOC_INSTR_MUL        0x96000000
#define FOC_INSTR_DIV        0x87000000
#define FOC_INSTR_PID        0x78000000
#define FOC_INSTR_INVCLARK   0x69000000
#define FOC_INSTR_SQRT       0x5A000000
#define FOC_INSTR_CORDIC_T   0x4B000000
#define FOC_INSTR_CORDIC_R   0x3C000000
#define FOC_INSTR_CLARK      0x2D000000
#define FOC_INSTR_SVPWM      0x1E000000

/*FOC->CONFIG = FOC->CONFIG | ( 0x04);*/   /* FIVE Mode */
/*FOC->CONFIG = FOC->CONFIG & (~0x04);*/   /* SEVEN Mode */
/*FOC->CONFIG = FOC->CONFIG | ( 0x08);*/   /* SPWM Mode */

#define MR_Initial()                 do\
                                     {\
	                                   FOC->SADDR  = (unsigned int)&MR_foc; \
                                     /*FOC->CONFIG = FOC->CONFIG | ( 0x04);*/ \
                                     FOC->CONFIG = FOC->CONFIG & (~0x04); \
                                     /*FOC->CONFIG = FOC->CONFIG | ( 0x08);*/ \
                                     MR_foc.PI_SD.Ui       = 0      ; \
                                     MR_foc.PI_SQ.Ui       = 0      ; \
                                     MR_foc.PI_P.Sum       = 0      ; \
                                     MR_foc.Math.Theta     = 0      ; \
                                     MR_foc.Math.ini_x     = 32767  ; \
                                     MR_foc.Math.ini_y     = 0      ; \
                                     Instr_PARK[0]             = ( ( FOC_INSTR_NEGATIVE ) + ( ( foc_val_I_Theta   | W16 ) << HL ) + ( ( FILLER                 ) << LH ) + ( foc_val_I_Theta | W16 ) ); \
                                     Instr_PARK[1]             = ( ( FOC_INSTR_CORDIC_R ) + ( ( foc_val_I_d             ) << HL ) + ( ( foc_val_I_Alpha        ) << LH ) + ( foc_val_I_Theta       ) ); \
                                     Instr_PARK[2]             = 0x00000000; \
                                     Instr_InvPARK_to_SVPWM[0] = ( ( FOC_INSTR_CORDIC_R ) + ( ( foc_val_V_Alpha   | W16 ) << HL ) + ( ( foc_val_V_d      | W16 ) << LH ) + ( foc_val_V_Theta | W16 ) ); \
                                     Instr_InvPARK_to_SVPWM[1] = ( ( FOC_INSTR_INVCLARK ) + ( ( foc_val_V_Va      | W16 ) << HL ) + ( ( foc_val_V_Beta   | W16 ) << LH ) + ( foc_val_V_Alpha | W16 ) ); \
                                     Instr_InvPARK_to_SVPWM[2] = ( ( FOC_INSTR_SVPWM    ) + ( ( foc_val_V_duty_a  | W16 ) << HL ) + ( ( foc_val_V_Va     | W16 ) << LH ) + ( foc_val_V_Vc    | W16 ) ); \
                                     Instr_InvPARK_to_SVPWM[3] = 0x00000000;\
                                     }\
                                     while(0)
 
#define MR_SinCos()            FOC->INSTR  = ( ( FOC_INSTR_CORDIC_R ) + ( ( foc_val_Math_Cos      | W16 ) << HL ) + ( ( foc_val_Math_ini_x   | W16 ) << LH ) + ( foc_val_Math_Theta   | W16 ) ) ; MR_NOP()
#define MR_InvPARK()           FOC->INSTR  = ( ( FOC_INSTR_CORDIC_R ) + ( ( foc_val_V_Alpha             ) << HL ) + ( ( foc_val_V_d                ) << LH ) + ( foc_val_V_Theta            ) ) ; MR_NOP()
//#define MR_PARK()              FOC->INSTR  = ( ( FOC_INSTR_CORDIC_R ) + ( ( foc_val_I_d                 ) << HL ) + ( ( foc_val_I_Alpha            ) << LH ) + ( foc_val_I_Theta            ) ) ; MR_NOP()
#define MR_CLARK()             FOC->INSTR  = ( ( FOC_INSTR_CLARK    ) + ( ( foc_val_I_Alpha       | W16 ) << HL ) + ( ( foc_val_I_u          | W16 ) << LH ) + ( foc_val_I_w          | W16 ) ) ; MR_NOP()
#define MR_InvClark()          FOC->INSTR  = ( ( FOC_INSTR_INVCLARK ) + ( ( foc_val_V_Va          | W16 ) << HL ) + ( ( foc_val_V_Beta       | W16 ) << LH ) + ( foc_val_V_Alpha      | W16 ) ) ; MR_NOP()
#define MR_SVPWM()             FOC->INSTR  = ( ( FOC_INSTR_SVPWM    ) + ( ( foc_val_V_duty_a      | W16 ) << HL ) + ( ( foc_val_V_Va         | W16 ) << LH ) + ( foc_val_V_Vc         | W16 ) ) ; MR_NOP()
#define MR_SPWM()              FOC->INSTR  = ( ( FOC_INSTR_SVPWM    ) + ( ( foc_val_V_duty_a      | W16 ) << HL ) + ( ( foc_val_V_Alpha      | W16 ) << LH ) + ( FILLER               | W16 ) ) ; MR_NOP()
#define MR_MUL()               FOC->INSTR  = ( ( FOC_INSTR_MUL      ) + ( ( foc_val_Math_alu_c          ) << HL ) + ( ( foc_val_Math_alu_b         ) << LH ) + ( foc_val_Math_alu_a         ) ) ; MR_NOP()
#define MR_DIV()               FOC->INSTR  = ( ( FOC_INSTR_DIV      ) + ( ( foc_val_Math_alu_c    | W32 ) << HL ) + ( ( foc_val_Math_alu_a   | W32 ) << LH ) + ( foc_val_Math_alu_b   | W32 ) ) ; MR_NOP() /* a/b = c...d */
#define MR_PID_SD_M()          FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SD_Ui            ) << HL ) + ( ( foc_val_PI_SD_InMeas       ) << LH ) + ( SERIES_D_M                 ) ) ; MR_NOP()
#define MR_PID_SD_0()          FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SD_Ui            ) << HL ) + ( ( foc_val_PI_SD_InMeas       ) << LH ) + ( SERIES_D_0                 ) ) ; MR_NOP()
#define MR_PID_SQ_M()          FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SQ_Ui            ) << HL ) + ( ( foc_val_PI_SQ_InMeas       ) << LH ) + ( SERIES_Q_M                 ) ) ; MR_NOP()
#define MR_PID_SQ_0()          FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SQ_Ui            ) << HL ) + ( ( foc_val_PI_SQ_InMeas       ) << LH ) + ( SERIES_Q_0                 ) ) ; MR_NOP()
#define MR_PID_P_M()           FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_P_Sum            ) << HL ) + ( ( foc_val_PI_P_InMeas        ) << LH ) + ( PARALLEL_M                 ) ) ; MR_NOP()
#define MR_PID_P_0()           FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_P_Sum            ) << HL ) + ( ( foc_val_PI_P_InMeas        ) << LH ) + ( PARALLEL_0                 ) ) ; MR_NOP()
#define MR_PID_SD_RESET()      FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SD_Ui            ) << HL ) + ( ( foc_val_PI_SD_InMeas        ) << LH ) + ( FILLER                     ) ) ; MR_NOP()
#define MR_PID_SQ_RESET()      FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_SQ_Ui            ) << HL ) + ( ( foc_val_PI_SQ_InMeas        ) << LH ) + ( FILLER                     ) ) ; MR_NOP()
#define MR_PID_P_RESET()       FOC->INSTR  = ( ( FOC_INSTR_PID      ) + ( ( foc_val_PI_P_Sum            ) << HL ) + ( ( foc_val_PI_P_InMeas        ) << LH ) + ( FILLER                     ) ) ; MR_NOP()
#define MR_ArcTan()            FOC->INSTR  = ( ( FOC_INSTR_CORDIC_T ) + ( ( foc_val_Math_Theta    | W16 ) << HL ) + ( ( foc_val_Math_Sin     | W16 ) << LH ) + ( foc_val_Math_Cos     | W16 ) ) ; MR_NOP()
#define MR_ArcTan_I()          FOC->INSTR  = ( ( FOC_INSTR_CORDIC_T ) + ( ( foc_val_I_arctan      | W16 ) << HL ) + ( ( foc_val_I_Beta       | W16 ) << LH ) + ( foc_val_I_Alpha      | W16 ) ) ; MR_NOP()
#define MR_ArcTan_V()          FOC->INSTR  = ( ( FOC_INSTR_CORDIC_T ) + ( ( foc_val_V_arctan      | W16 ) << HL ) + ( ( foc_val_V_Beta       | W16 ) << LH ) + ( foc_val_V_Alpha      | W16 ) ) ; MR_NOP()
#define MR_SQRT()              FOC->INSTR  = ( ( FOC_INSTR_SQRT     ) + ( ( foc_val_Math_sqrt_out | W16 ) << HL ) + ( ( FILLER                     ) << LH ) + ( foc_val_Math_sqrt_in       ) ) ; MR_NOP()
#define MR_NOP()               FOC->INSTR  = 0x00000000

#define MR_Single()            FOC->CONFIG = FOC->CONFIG & (~0x01);
#define MR_SeqMode()           FOC->CONFIG = FOC->CONFIG | ( 0x01);
//#define MR_PARK_to_SVPWM()     FOC->MADDR = Mem_Park_to_SVPWM ; MR_NOP()
//#define MR_PI_Series_D()       FOC->MADDR = Mem_PI_Series_D      ; MR_NOP()
//#define MR_PI_Series_Q()       FOC->MADDR = Mem_PI_Series_Q      ; MR_NOP()
#define MR_PARK()              FOC->MADDR = (unsigned int)&Instr_PARK[0]             ; MR_NOP()
#define MR_InvPARK_to_SVPWM()  FOC->MADDR = (unsigned int)&Instr_InvPARK_to_SVPWM[0] ; MR_NOP()


#ifdef __cplusplus
}
#endif

#endif


