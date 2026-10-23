#!/bin/bash
# 把一份工程副本的 cfg 改回 test33 等价配置（原厂增益、关掉 0xF20E0003 的全部新项）。
#   tools/sim/legacy_cfg.sh <工程副本目录>
set -e
D="$1"
sed -i 's/#define CTRL_DQ_DECOUPLE        1/#define CTRL_DQ_DECOUPLE        0/;
        s/#define CTRL_OBS_HPF_COMP       1/#define CTRL_OBS_HPF_COMP       0/;
        s/#define CURRENT_BW_HZ           300.0/#define CURRENT_BW_HZ           86.573/;
        s/#define SPEED_WN_HZ             0.5/#define SPEED_WN_HZ             0.3105/;
        s/#define SPEED_ZETA              0.7/#define SPEED_ZETA              0.078025/;
        s/#define FLUXPLL_WN_HZ           15.0/#define FLUXPLL_WN_HZ           14.346/;
        s/#define FLUXPLL_ZETA            0.707/#define FLUXPLL_ZETA            0.676628/' "$D/FW/Inc/cfg_ctrl.h"
sed -i 's/#define HAL_OFFSET_256          1/#define HAL_OFFSET_256          0/' "$D/FW/Inc/cfg_board.h"
for k in 'CURRENT_BW_HZ           86.573' 'SPEED_ZETA              0.078025' 'FLUXPLL_ZETA            0.676628' 'CTRL_DQ_DECOUPLE        0' 'CTRL_OBS_HPF_COMP       0'; do
  grep -q "define $k" "$D/FW/Inc/cfg_ctrl.h" || { echo "legacy_cfg: '$k' not applied" >&2; exit 1; }
done
grep -q 'define HAL_OFFSET_256          0' "$D/FW/Inc/cfg_board.h" || { echo "legacy_cfg: HAL_OFFSET_256 not applied" >&2; exit 1; }
