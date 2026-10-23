#!/bin/bash
# 改配置编一个仿真变体：tools/sim/variant.sh NAME "cfg_motor.h 的 sed" "cfg_ctrl.h 的 sed"
# 例：开弱磁  tools/sim/variant.sh fw "" 's/CTRL_FW_ENABLE          0/CTRL_FW_ENABLE          1/'
set -e
T="$(cd "$(dirname "$0")" && pwd)"; R="$(cd "$T/../.." && pwd)"; V="$T/build/var_$1"
rm -rf "$V" && mkdir -p "$V" && cp -r "$R"/{FW,MCI,User,Board} "$V"/
[ -n "$2" ] && sed -i "$2" "$V/FW/Inc/cfg_motor.h"
[ -n "$3" ] && sed -i "$3" "$V/FW/Inc/cfg_ctrl.h"
NEWDIR="$V" OUTNAME="sim_$1" "$T/build.sh" new
