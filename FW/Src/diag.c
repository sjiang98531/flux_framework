/* ============================================================================
 * @file    diag.c
 * @brief   固件标识与配置检查结果
 *
 * 结果写入 gFluxDiag，供调试器 / 上位机读取；配置检查失败时 faultDetect 据此报故障，电机不起动。
 * ==========================================================================*/
#include "param.h"
#include "diag.h"
#include "cfg_derived.h"
#include "cfg_check.h"         /* 配置的编译期检查（cfg_check_list.h） */

volatile FluxDiag_TypeDef gFluxDiag;
/* 上电：写入版本号；汇总配置警告位（Ld != Lq 但按表贴式配置、增益封顶等）；
 * 遍历编译期配置检查表，记录第一条未通过项的行号及未通过项数（不为 0 时电机不起动） */
void Diag_Init(void)
{
    unsigned i;

    gFluxDiag.buildTag = BUILD_TAG;
    gFluxDiag.cfgWarn = 0;
    /* bit1 电流环增益封顶，bit2 PLL 增益超出 int16，bit3 反电势前馈超量程：按当前参数计算（param.c computeGain） */
    gFluxDiag.cfgWarn |= gGain.cfgWarn;
    gFluxDiag.cfgFailLine = 0;
    gFluxDiag.cfgFailCount = 0;
    for (i = 0; i < sizeof s_fwCfgCheck / sizeof s_fwCfgCheck[0]; i++) {
        if (s_fwCfgCheck[i]) {
            if (gFluxDiag.cfgFailLine == 0) gFluxDiag.cfgFailLine = s_fwCfgCheck[i];
            gFluxDiag.cfgFailCount++;
        }
    }
}
