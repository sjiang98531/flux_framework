#!/usr/bin/env python3
"""Regenerate the <Group> part of Project/MotorControl_FOC.uvoptx (Keil per-user
options: debugger, watch windows, tree state) so its file list matches the
uvprojx. Everything before the first <Group> (debug/flash settings, watch
windows) is kept byte-for-byte. The uvoptx is not under git (.gitignore).

    python3 tools/update_uvoptx.py [path/to/source.uvoptx]
"""
import os, re, sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJX = os.path.join(ROOT, 'Project', 'MotorControl_FOC.uvprojx')
OPTX = os.path.join(ROOT, 'Project', 'MotorControl_FOC.uvoptx')

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else OPTX
    txt = open(src, encoding='utf-8', newline='').read()
    nl = '\r\n' if '\r\n' in txt else '\n'
    t = txt.replace('\r\n', '\n')
    i = t.index('  <Group>')
    j = t.rindex('</ProjectOpt>')
    proj = ET.parse(PROJX).getroot()
    out, fileno = '', 0
    for gno, g in enumerate(proj.iter('Group'), 1):
        out += (f'  <Group>\n    <GroupName>{g.findtext("GroupName")}</GroupName>\n    <tvExp>1</tvExp>\n'
                f'    <tvExpOptDlg>0</tvExpOptDlg>\n    <cbSel>0</cbSel>\n    <RteFlg>0</RteFlg>\n')
        for f in g.iter('File'):
            fileno += 1
            out += (f'    <File>\n      <GroupNumber>{gno}</GroupNumber>\n      <FileNumber>{fileno}</FileNumber>\n'
                    f'      <FileType>{f.findtext("FileType")}</FileType>\n      <tvExp>0</tvExp>\n'
                    f'      <tvExpOptDlg>0</tvExpOptDlg>\n      <bDave2>0</bDave2>\n'
                    f'      <PathWithFileName>{f.findtext("FilePath")}</PathWithFileName>\n'
                    f'      <FilenameWithoutPath>{f.findtext("FileName")}</FilenameWithoutPath>\n'
                    f'      <RteFlg>0</RteFlg>\n      <bShared>0</bShared>\n    </File>\n')
        out += '  </Group>\n\n'
    t = t[:i] + out + t[j:]
    ET.fromstring(t.encode('utf-8'))       # must still be valid XML
    open(OPTX, 'w', encoding='utf-8', newline='').write(t.replace('\n', nl))
    print('OK', OPTX, fileno, 'files')

if __name__ == '__main__':
    main()
