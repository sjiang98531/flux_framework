#!/usr/bin/env python3
"""Rewrite the <Groups> block and the C include path of Project/MotorControl_FOC.uvprojx
from the GROUPS table below, then re-parse the result as XML and check that every
listed file exists. Everything outside those two places is left byte-for-byte as is.

    python3 tools/update_uvprojx.py            # in the flux_framework root
"""
import os, re, sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.join(ROOT, 'Project', 'MotorControl_FOC.uvprojx')

INCLUDE_PATH = r'..\FW\Inc;..\Device\src;..\MCI\Inc;..\User\Inc;..\Board\Inc;..\User\HeadFiles;..\CMSIS;..\Device\inc'

# (group name, [paths relative to Project/])
GROUPS = [
    ('Config',     [r'..\FW\Inc\cfg_motor.h', r'..\FW\Inc\cfg_ctrl.h', r'..\FW\Inc\cfg_board.h', r'..\FW\Inc\fw_config.h']),
    ('App',        [r'..\FW\Src\main.c', r'..\FW\Src\supervisor.c', r'..\FW\Src\diag.c',
                    r'..\FW\Src\protect.c']),
    ('Control',    [r'..\FW\Src\startup.c', r'..\FW\Src\flystart.c', r'..\FW\Src\speed_ctrl.c', r'..\FW\Src\mtpa_fw.c',
                    r'..\FW\Src\identify.c']),
    ('Observer',   [r'..\FW\Src\est.c', r'..\FW\Src\flux_obs.c', r'..\FW\Src\flux_pll.c']),
    ('FOC_Core',   [r'..\FW\Src\foc_core.c', r'..\FW\Src\svpwm.c', r'..\FW\Src\meas.c', r'..\FW\Src\mathx.c',
                    r'..\FW\Src\mc_tables.c']),
    ('HAL_Board',  [r'..\FW\Src\hal_sc32r602.c', r'..\Board\Src\SC_Init.c', r'..\Board\Inc\Hardware_config.h']),
    ('Device',     None),   # None = keep the existing group unchanged
    ('Startup',    None),
    # 原来的 'MCI' 组（原厂闭源库 SOC_MCI_2R_V102.lib）在 0xF20E0006 删除
]

FILETYPE = {'.c': '1', '.s': '2', '.lib': '4', '.h': '5'}

def file_xml(path, ind):
    name = path.split('\\')[-1]
    ext = os.path.splitext(name)[1].lower()
    return (f'{ind}<File>\n'
            f'{ind}  <FileName>{name}</FileName>\n'
            f'{ind}  <FileType>{FILETYPE[ext]}</FileType>\n'
            f'{ind}  <FilePath>{path}</FilePath>\n'
            f'{ind}</File>\n')

def main():
    txt = open(PROJ, encoding='utf-8', newline='').read()
    nl = '\r\n' if '\r\n' in txt else '\n'
    t = txt.replace('\r\n', '\n')
    m = re.search(r'(?P<ind>[ \t]*)<Groups>\n(?P<body>.*?)(?P=ind)</Groups>', t, re.S)
    if not m:
        sys.exit('no <Groups> block')
    ind = m.group('ind')
    old_groups = {}
    for g in re.finditer(r'([ \t]*)<Group>\n(.*?)\1</Group>\n', m.group('body'), re.S):
        name = re.search(r'<GroupName>(.*?)</GroupName>', g.group(2)).group(1)
        old_groups[name] = g.group(0)
    gi, fi = ind + '  ', ind + '      '
    out = f'{ind}<Groups>\n'
    for name, files in GROUPS:
        if files is None:
            out += old_groups[name]
            continue
        files = [f for f in files if os.path.exists(os.path.join(ROOT, 'Project', f.replace('\\', '/')))]
        out += f'{gi}<Group>\n{gi}  <GroupName>{name}</GroupName>\n{gi}  <Files>\n'
        out += ''.join(file_xml(f, fi) for f in files)
        out += f'{gi}  </Files>\n{gi}</Group>\n'
    out += f'{ind}</Groups>'
    t = t[:m.start()] + out + t[m.end():]
    # C include path: the first non-empty <IncludePath> inside <Cads>
    t, n = re.subn(r'(<Cads>.*?<IncludePath>)[^<]*(</IncludePath>)',
                   lambda mm: mm.group(1) + INCLUDE_PATH + mm.group(2), t, count=1, flags=re.S)
    if n != 1:
        sys.exit('C IncludePath not found')
    # validate
    root = ET.fromstring(t.encode('utf-8'))
    missing = []
    for f in root.iter('File'):
        p = os.path.normpath(os.path.join(ROOT, 'Project', f.findtext('FilePath').replace('\\', '/')))
        if not os.path.exists(p):
            missing.append(p)
    if missing:
        sys.exit('missing files:\n  ' + '\n  '.join(missing))
    open(PROJ, 'w', encoding='utf-8', newline='').write(t.replace('\n', nl))
    groups = [(g.findtext('GroupName'), len(list(g.iter('File')))) for g in root.iter('Group')]
    print('OK', groups)
    print('IncludePath', root.find('.//Cads/VariousControls/IncludePath').text)

if __name__ == '__main__':
    main()
