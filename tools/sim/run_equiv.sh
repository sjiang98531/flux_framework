#!/bin/bash
# test33 与"当前代码 + test33 等价配置"（legacy_cfg.sh）逐位对比。先 tools/sim/build.sh。
# 证明重构与新增代码在关掉新项、用回原厂参数时不改变任何行为。最后一行 TOTAL/FAIL。
# 比较的是"输出哈希"（SIM_HASH=out）：每拍写进 PWM 寄存器的值 + 观测器/PLL/故障码等语义状态，
# 与固件内部结构体布局无关（0xF20E0007 起框架的结构体已经和 test33 完全不同）。
T="$(cd "$(dirname "$0")" && pwd)"; B="$T/build"; cd "$B"
fails=0; total=0
run() { local tag=$1; shift; env SIM_HASH=out "$@" ./sim_old $SC o_eq.csv o_eq.hash $TE >/dev/null; env SIM_HASH=out "$@" ./sim_legacy $SC n_eq.csv n_eq.hash $TE >/dev/null; r=$(python3 "$T/cmp.py" o_eq.hash n_eq.hash); total=$((total+1)); case "$r" in IDENTICAL*) ;; *) fails=$((fails+1));; esac; printf "%-34s %s\n" "$tag" "$r"; }
TE=20
for SC in 0 3 4 5; do for th in 0.1 2.5 4.0; do for td in 0 0.2e-6 0.4e-6; do run "sc$SC th$th td$td" SIM_TH0=$th SIM_TD=$td; done; done; done
SC=3; run "locked rotor" SIM_FRIC=5.0
SC=3; run "flying start +600" SIM_W0=600
SC=3; run "flying start -600" SIM_W0=-600
SC=0; run "low bus 150V" SIM_VBUS=150
SC=1; run "load step 0.3Nm" SIM_LOAD=0.3
SC=1; run "load step 1.5Nm" SIM_LOAD=1.5
SC=2; run "high speed 2600 (no FW)"
SC=0; TE=28; run "sc0 28s default"
./fz_old 2000000 fo.hash >/dev/null; ./fz_legacy 2000000 fn.hash >/dev/null; r=$(python3 "$T/cmp.py" fo.hash fn.hash); total=$((total+1)); case "$r" in IDENTICAL*) ;; *) fails=$((fails+1));; esac
printf "%-34s %s\n" "observer+PLL random fuzz" "$r"
echo "TOTAL $total FAIL $fails"
