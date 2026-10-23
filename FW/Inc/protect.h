/* ============================================================================
 * protect.h —— 运行中保护（自己实现，替代原厂闭源库的 MCF_ADV_Protect）
 *
 * Spin 慢环每拍调用。结果写进 gProt 的标志位，由 supervisor 处理：
 *   bRotor_block  堵转：|转速| < u16BlockSpeed 且 |Vdq|^2 < u16BlockVoltageAmp_square，
 *                 持续 u16BlockTimeCmd 拍
 *   bErr_Spin     反转失控：转速与斜坡指令反号 且 |Vdq|^2 小，持续 u16BlockTimeCmd 拍
 *   bPhaseless    缺相：每 u16PhaselessTimeCmd 拍比较一次三相电流绝对值积分
 *                 （快环累加 u32Ia/b/c_Integral），最大 > 3 倍最小 就判缺相
 *   bSpeed_OC     超速：|转速| > u16Speed_OC_cmd
 * 门限在 supervisor.c 的 initProtection() 里由 cfg_ctrl.h 的物理量算出。
 * 行为与原厂库逐位相同。
 * ==========================================================================*/
#ifndef PROTECT_H
#define PROTECT_H
#include "mc_types.h"

/* 保护的门限、计数和结果标志（字段沿用原厂 ADV_PROTECT，加上母线过/欠压门限） */
typedef struct {
    uint16_t u16BlockTimeCmd;           /* 堵转/反转失控判定时间（慢环拍） */
    uint16_t u16BlockVoltageAmp_square; /* 堵转判定的电压平方门限 */
    uint16_t u16BlockSpeed;             /* 堵转判定的转速门限 */
    uint16_t u16VoltageTemp;            /* 本拍 |Vdq|^2 */
    uint16_t u16Speed_OC_cmd;           /* 超速门限 */
    uint16_t u16BlockCounter;
    uint16_t u16FspinCounter;
    MCFLIB_3_ABC_T_S16 sIabc_Max;       /* 原厂启动期间三相电流峰值（缺相） */
    uint32_t u32Ia_Integral;            /* 三相电流绝对值积分（缺相，快环累加） */
    uint32_t u32Ib_Integral;
    uint32_t u32Ic_Integral;
    uint32_t u32PhaselessMinTemp;
    uint32_t u32PhaselessMaxTemp;
    uint16_t u16PhaselessTimeCnt;
    uint16_t u16PhaselessTimeCmd;
    int16_t  s16I_Phaseless_threshold;  /* 启动期缺相判定的电流门限 */
    bool_t   bPhaseless;
    bool_t   bSpeed_OC;
    bool_t   bRotor_block;
    bool_t   bErr_Spin;
    int16_t  ovpCmd;                    /* 母线过压门限 Q15 */
    int16_t  uvpCmd;                    /* 母线欠压门限 Q15 */
} Protect_TypeDef;

extern Protect_TypeDef gProt;

void Protect_SpinSlow(void);

#endif
