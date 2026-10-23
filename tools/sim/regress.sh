#!/bin/bash
# 当前代码 vs 某个历史版本（git tag）逐拍对比"输出哈希"：每拍写进 PWM 寄存器的值 + 语义状态。
# 用来证明"只改代码组织、不改行为"的重构。最后一行 TOTAL/FAIL。
#   tools/sim/regress.sh [tag]        默认 tag = sim-outhash（第一个支持输出哈希的版本）
# 参考版本用它自己那一份 tools/sim 编译（所以 tag 必须 >= sim-outhash）。
set -e
T="$(cd "$(dirname "$0")" && pwd)"; R="$(cd "$T/../.." && pwd)"; B="$T/build"
TAG="${1:-sim-outhash}"
REF="$B/ref_$TAG"
if [ ! -x "$B/sim_ref_$TAG" ]; then
  rm -rf "$REF"; mkdir -p "$REF"; git -C "$R" archive "$TAG" | tar -x -C "$REF"
  (cd "$REF" && tools/sim/build.sh new >/dev/null 2>&1)
  cp "$REF/tools/sim/build/sim_new" "$B/sim_ref_$TAG"
fi
[ -x "$B/sim_new" ] || "$T/build.sh" new >/dev/null
cd "$B"; fails=0; total=0
run() { local tag=$1 sc=$2 te=$3; shift 3
  env SIM_HASH=out "$@" "./sim_ref_$TAG" $sc r_rg.csv r_rg.hash $te >/dev/null
  env SIM_HASH=out "$@" ./sim_new $sc n_rg.csv n_rg.hash $te >/dev/null
  r=$(python3 "$T/cmp.py" r_rg.hash n_rg.hash || true); total=$((total+1))
  case "$r" in IDENTICAL*) ;; *) fails=$((fails+1)); printf "%-40s %s\n" "$tag" "$r";; esac; }
for sc in 0 1 2 3 4 5 6 7 8 9; do
  for cond in "SIM_X=0" "SIM_PWMDLY=1 SIM_TD=0.6e-6" "SIM_TH0=4.7 SIM_KR=1.2" "SIM_TH0=2.3 SIM_KJ=2" "SIM_TD=1e-6 SIM_NOISE=4"; do
    run "sc$sc $cond" $sc 16 $cond
  done
done
for w in -800 -300 -100 100 300 800 1500; do run "flying W0=$w" 3 8 SIM_W0=$w SIM_TH0=1.1; done
run "locked rotor" 3 8 SIM_FRIC=5.0
run "low bus 150V" 0 16 SIM_VBUS=150
run "load 1.5Nm" 1 16 SIM_LOAD=1.5
echo "REGRESS vs $TAG: TOTAL $total FAIL $fails"
