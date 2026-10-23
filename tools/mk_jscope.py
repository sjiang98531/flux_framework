#!/usr/bin/env python3
"""由 Keil 编出的 .axf 生成 J-Scope 工程（.jscope），地址和偏移都从编译结果算，不手写。

    python3 tools/mk_jscope.py <axf> <out.jscope> [变量 ...]

  变量写成 C 表达式路径，如 gFoc.idq.s16D；不给就用下面 DEFAULT 的 10 个（J-Scope 一次最多 10 个）。
  地址 = axf 符号表里全局变量的地址 + 成员偏移；成员偏移、大小、类型用 clang 按 Cortex-M0+
  （thumbv6m，与 armcc 同为 AAPCS 布局）编译工程头文件得到。只支持 16 位整型成员。

需要 clang、llvm-nm、readelf（仿真容器里都有）。
"""
import os, re, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (路径, YResolution, RGB) —— 与 Data/bench_0xF20E0005/FluxFramework_v5.jscope 同一组量
DEFAULT = [
    ('gFluxDiag.runState',  1,   (255, 0, 0)),
    ('gStartupIF.stage',    1,   (255, 140, 0)),
    ('gStartupIF.olSpeed',  400, (0, 0, 255)),
    ('gStartupIF.pllFilt',  400, (0, 160, 255)),
    ('gFoc.idq.s16D',       600, (160, 0, 255)),
    ('gFoc.idq.s16Q',       400, (0, 200, 0)),
    ('gFoc.idqRef.s16Q',    400, (200, 0, 120)),
    ('gMeas.speed',         800, (120, 0, 200)),
    ('gSpd.ramp',           800, (255, 255, 0)),
    ('gFluxDiag.isrMax',    400, (0, 255, 255)),
]
HEADERS = ['meas.h', 'foc_core.h', 'est.h', 'speed_ctrl.h', 'protect.h', 'flystart.h', 'startup.h',
           'supervisor.h', 'hal.h', 'svpwm.h', 'diag.h', 'flux_obs.h', 'flux_pll.h', 'mtpa_fw.h']
TYPES = {1: ('short', 11, 32767), 2: ('unsigned short', 7, 65535)}
ELF_NAME = r'C:\Users\SOC\Desktop\电机ppt\flux_framework\Output\SOC_FOC_SDK_V1_0.axf'


def symbols(axf):
    out = subprocess.run(['readelf', '-sW', axf], capture_output=True, text=True, check=True).stdout
    t = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 8 and f[3] == 'OBJECT':
            t[f[7]] = int(f[1], 16)
    return t


def layout(paths):
    """每个路径：(相对变量起点的偏移, 大小, 类型码)。用数组大小把常量带出来，再用 llvm-nm -S 读。"""
    tmp = tempfile.mkdtemp()
    ci = os.path.join(tmp, 'ci'); os.mkdir(ci)
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools/sim/armchk/mkci.py'), ROOT, ci], check=True)
    src = ['#include <stddef.h>'] + ['#include "%s"' % h for h in HEADERS]
    for i, p in enumerate(paths):
        var = p.split('.')[0]
        src.append('char off_%d[(size_t)((const char *)&(%s) - (const char *)&%s) + 1];' % (i, p, var))
        src.append('char siz_%d[sizeof(%s)];' % (i, p))
        src.append('char typ_%d[_Generic((%s), short: 1, unsigned short: 2, default: 9)];' % (i, p))
    c = os.path.join(tmp, 'lay.c'); open(c, 'w').write('\n'.join(src) + '\n')
    o = os.path.join(tmp, 'lay.o')
    T = os.path.join(ROOT, 'tools/sim/armchk')
    inc = ['-I' + ci] + ['-I' + os.path.join(ROOT, d) for d in
                         ['FW/Inc', 'MCI/Inc', 'User/Inc', 'Board/Inc', 'Main/Inc', 'User/HeadFiles', 'CMSIS', 'Device/inc', 'Device/src']]
    subprocess.run(['clang', '--target=thumbv6m-none-eabi', '-mcpu=cortex-m0plus', '-ffreestanding', '-std=gnu11', '-w',
                    '-fcommon', '-isystem', T + '/libc', '-include', T + '/libc/armcc_kw.h', '-D__ARMCC_VERSION=6100000',
                    '-DUSE_STDPERIPH_DRIVER', '-c', c, '-o', o] + inc, check=True)
    out = subprocess.run(['llvm-nm', '-S', o], capture_output=True, text=True, check=True).stdout
    sz = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 4: sz[f[3]] = int(f[1], 16)
    return [(sz['off_%d' % i] - 1, sz['siz_%d' % i], sz['typ_%d' % i]) for i in range(len(paths))]


def main():
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    axf, out = sys.argv[1], sys.argv[2]
    sel = DEFAULT if len(sys.argv) == 3 else [(p, 400, (0, 0, 255)) for p in sys.argv[3:]]
    if len(sel) > 10: sys.exit('J-Scope 一次最多 10 个变量')
    syms = symbols(axf)
    lay = layout([p for p, _, _ in sel])
    xs = []
    pkt = 4                                         # 采样包前 4 字节是时间戳
    for (path, yres, (r, g, b)), (off, size, ty) in zip(sel, lay):
        var = path.split('.')[0]
        if var not in syms: sys.exit('axf 里找不到全局变量 %s' % var)
        if ty not in TYPES or size != 2: sys.exit('%s 不是 16 位整型' % path)
        tname, tid, vmax = TYPES[ty]
        addr = syms[var] + off
        xs.append('      <Symbol>\n        <Options Name="%s" NameWithParents="%s" Address="0x%08X" Type="%s" TypeID="%d" '
                  'Size="2" YResolution="%d" YOffset="0" YMinSampled="0" YMaxSampled="0" YAvgSampled="0" MaxValue="%d" '
                  'File="" GraphColorRed="%d" GraphColorGreen="%d" GraphColorBlue="%d" Offset="%d" ShowInGraph="1" '
                  'DrawStyle="0" IsAutoConfig="0" />\n      </Symbol>\n'
                  % (path.split('.')[-1], path, addr, tname, tid, yres, vmax, r, g, b, pkt))
        print('%-22s 0x%08X  %s' % (path, addr, tname))
        pkt += 2
    first = sel[0][0].split('.')[-1]
    doc = ('<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n<JScopeProject>\n  <FileVersion major="1" minor="1" />\n'
           '  <Project>\n    <Configuration>\n      <JLink>\n        <Options Connection="1" IsRTTSession="0" IsRTTWithTimeStamp="0" '
           'IsRTTPlus="0" SamplingPeriod="10" SerialNo="0" JTAG_IRPre="0" JTAG_DRPre="0" IP="" TargetDevice="Cortex-M0" Nickname="" '
           'InterfaceSpeed="4000" TargetInterface="SWD" />\n      </JLink>\n      <ElfFile Name="%s" />\n      <DataFile Name="" />\n'
           '      <Scope>\n        <Options TimeResolutionUs="100000" />\n      </Scope>\n      <Trigger>\n'
           '        <Options Level="0" Symbol="%s" On="0" Auto="0" Rise="0" Fall="0" />\n      </Trigger>\n    </Configuration>\n'
           '    <Windows>\n      <Graph Show="1" />\n      <Watch Show="1" />\n    </Windows>\n    <SelectedSymbols>\n%s'
           '    </SelectedSymbols>\n  </Project>\n</JScopeProject>\n') % (ELF_NAME, first, ''.join(xs))
    open(out, 'w', encoding='utf-8', newline='\r\n').write(doc)


if __name__ == '__main__':
    main()
