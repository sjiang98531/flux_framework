#!/bin/bash
# 用 clang 按 Cortex-M0+（32 位，thumbv6m）对 Keil 工程里的每个 .c 做语法/类型检查（不生成代码）。
# 替代不了 Keil 编译，但能抓出 64 位主机编译看不到的问题。FW/ 下的文件开 -Wall -Wextra。
T="$(cd "$(dirname "$0")" && pwd)"; N="${1:-$(cd "$T/../../.." && pwd)}"
CI=$(mktemp -d); python3 "$T/mkci.py" "$N" "$CI"
INC="-I$CI -I$N/FW/Inc -I$N/MCI/Inc -I$N/User/Inc -I$N/Board/Inc -I$N/Main/Inc -I$N/User/HeadFiles -I$N/CMSIS -I$N/Device/inc -I$N/Device/src"
CF="--target=thumbv6m-none-eabi -mcpu=cortex-m0plus -ffreestanding -std=gnu99 -fsyntax-only -isystem $T/libc -include $T/libc/armcc_kw.h -D__ARMCC_VERSION=6100000 -DUSE_STDPERIPH_DRIVER -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -Wno-typedef-redefinition -Wno-nonportable-include-path"
n=0; bad=0
for f in $(python3 - "$N" <<'PY'
import sys,xml.etree.ElementTree as ET
n=sys.argv[1]
for f in ET.parse(n+'/Project/MotorControl_FOC.uvprojx').getroot().iter('File'):
    if f.findtext('FileType')=='1': print(n+'/Project/'+f.findtext('FilePath').replace('\\','/'))
PY
); do
  n=$((n+1)); case "$f" in */FW/*) W="";; *) W="-w";; esac
  out=$(clang $CF $W $INC "$f" 2>&1)
  [ -n "$out" ] && { bad=$((bad+1)); echo "== $f"; echo "$out" | grep -E 'error|warning' | head -20; }
done
rm -rf "$CI"; echo "checked $n files, $bad with diagnostics"
