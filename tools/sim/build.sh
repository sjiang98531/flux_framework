#!/bin/bash
# 构建主机仿真。在 Linux / WSL 下运行，需要 gcc 和 git。
#   tools/sim/build.sh            -> build/sim_old（test33，从 git tag base-test33 导出）
#                                    build/sim_new（当前工作区）
#   NEWDIR=<另一份工程> OUTNAME=sim_x tools/sim/build.sh new   只编新的
set -e
T="$(cd "$(dirname "$0")" && pwd)"; R="$(cd "$T/../.." && pwd)"; B="$T/build"; mkdir -p "$B"
CF="-O2 -g -fwrapv -Wall -Wno-unused-variable -Wno-unused-function -Wno-misleading-indentation -Wno-discarded-qualifiers -include $T/shim/armcc_compat.h -I$T/shim -I$T"
build_old() {
  O="$B/old"; rm -rf "$O"; mkdir -p "$O"
  git -C "$R" archive base-test33 | tar -x -C "$O"
  # armcc 允许"先非 static 声明、后 static 定义"，gcc 不允许：只在仿真副本里去掉 static
  sed -i 's/^static void M1_/void M1_/; s/^static void Fault_LED_Disp/void Fault_LED_Disp/' "$O/User/Src/StateMachine.c" "$O/User/Inc/StateMachine.h"
  gcc $CF -w -I$O/MCI/Inc -I$O/User/Inc -I$O/Main/Inc -I$O/User/HeadFiles "$T/sim_main.c" "$T/plant.c" "$T/vendor_lib.c" "$T/glue_old.c" \
    $O/User/Src/SC_it.c $O/User/Src/StateMachine.c $O/User/Src/FOC_PMSM.c $O/User/Src/FluxObs.c $O/User/Src/FluxPLL.c $O/Main/Src/user_function.c -lm -o "$B/sim_old"
  gcc -O2 -fwrapv -w -include $T/shim/armcc_compat.h -include stdlib.h -I$T/shim -I$T -I$O/MCI/Inc -I$O/User/Inc -I$O/Main/Inc -I$O/User/HeadFiles \
    "$T/fuzz.c" "$T/vendor_lib.c" $O/User/Src/FluxObs.c $O/User/Src/FluxPLL.c -lm -o "$B/fz_old"
}
build_new() {
  N="${NEWDIR:-$R}"; OUT="${OUTNAME:-sim_new}"; I="-I$N/FW/Inc -I$N/MCI/Inc -I$N/User/Inc -I$N/Board/Inc -I$N/User/HeadFiles"
  W="$B/obj_$OUT"; rm -rf "$W"; mkdir -p "$W"
  gcc $CF $I -c "$T/sim_main.c" -o "$W/sim_main.o"; gcc $CF $I -c "$T/plant.c" -o "$W/plant.o"; gcc $CF $I -c "$T/glue_new.c" -o "$W/glue.o"
  # 0xF20E0006 起框架不再用原厂库：新代码只链接自己的 FW/Src，不链接 vendor_lib.c（原厂库模型）
  for f in $N/FW/Src/*.c; do [ "$(basename $f)" = main.c ] && continue; gcc $CF $I -c "$f" -o "$W/fw_$(basename $f .c).o"; done
  gcc "$W"/*.o -lm -o "$B/$OUT"
  if [ "$OUT" = sim_new ]; then
    echo 'void HAL_PwmWriteCompares(const void *u){(void)u;}' > "$W/stub.c"
    gcc -O2 -fwrapv -w -DNEWCODE -include $T/shim/armcc_compat.h -include stdlib.h -I$T/shim -I$T $I "$T/fuzz.c" "$T/vendor_lib.c" "$W/stub.c" \
      $N/FW/Src/flux_obs.c $N/FW/Src/flux_pll.c $N/FW/Src/diag.c $N/FW/Src/mathx.c $N/FW/Src/foc_core.c $N/FW/Src/svpwm.c $N/FW/Src/mc_tables.c -lm -o "$B/fz_new"
    gcc -O2 -fwrapv -w -DNEWCODE -include $T/shim/armcc_compat.h -I$T/shim -I$T $I "$T/libcheck.c" "$T/vendor_lib.c" \
      $N/FW/Src/svpwm.c $N/FW/Src/mc_tables.c $N/FW/Src/meas.c $N/FW/Src/protect.c $N/FW/Src/flystart.c $N/FW/Src/mathx.c -lm -o "$B/libcheck"
    gcc -include $T/shim/armcc_compat.h -I$T/shim $I "$T/gain_check.c" -o "$B/gain_check"
  fi
}
build_legacy() {
  # 当前代码 + test33 等价配置（tools/sim/legacy_cfg.sh），用来和 test33 逐位对比
  L="$B/var_legacy"; rm -rf "$L"; mkdir -p "$L"; cp -r "$R"/{FW,MCI,User,Board} "$L"/
  "$T/legacy_cfg.sh" "$L"
  NEWDIR="$L" OUTNAME=sim_legacy build_new
  echo 'void HAL_PwmWriteCompares(const void *u){(void)u;}' > "$B/stub_legacy.c"
  I="-I$L/FW/Inc -I$L/MCI/Inc -I$L/User/Inc -I$L/Board/Inc -I$L/User/HeadFiles"
  gcc -O2 -fwrapv -w -DNEWCODE -include $T/shim/armcc_compat.h -include stdlib.h -I$T/shim -I$T $I "$T/fuzz.c" "$T/vendor_lib.c" "$B/stub_legacy.c" \
    $L/FW/Src/flux_obs.c $L/FW/Src/flux_pll.c $L/FW/Src/diag.c $L/FW/Src/mathx.c $L/FW/Src/foc_core.c $L/FW/Src/svpwm.c $L/FW/Src/mc_tables.c -lm -o "$B/fz_legacy"
  gcc -include $T/shim/armcc_compat.h -I$T/shim $I "$T/gain_check.c" -o "$B/gain_check_legacy"
}
case "${1:-all}" in old) build_old;; new) build_new;; legacy) build_legacy;; *) build_old; build_new; build_legacy;; esac
