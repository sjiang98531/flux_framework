#!/usr/bin/env python3
"""生成 FW/Src/mc_tables.c：正弦表和开方表，全部按公式算，不从任何库里拷数据。

    python3 tools/gen_tables.py           # 在 flux_framework 根目录

  g_mcSinTab[i]  = round(32767 * sin(i * 90° / 256))，i = 0..255（四分之一周期）
  g_mcSqrtTab[i] = floor(32767 * sqrt(i / 512))，      i = 0..512（输入 Q30 的 0..1）

这两条公式与原厂库里的表逐项相同（0xF20E0006 去库时核对过），所以替换后行为不变。
"""
import math, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'FW', 'Src', 'mc_tables.c')

sin_tab = [min(32767, round(32767 * math.sin(math.radians(i * 90.0 / 256)))) for i in range(256)]
sqrt_tab = [min(32767, math.floor(32767 * math.sqrt(i / 512.0))) for i in range(513)]

def rows(vals, per=16):
    return ',\n'.join('    ' + ', '.join(f'{v:5d}' for v in vals[k:k + per]) for k in range(0, len(vals), per))

txt = f'''/* 由 tools/gen_tables.py 生成，不要手改。
 *   g_mcSinTab[i]  = round(32767 * sin(i * 90deg / 256))，i = 0..255
 *   g_mcSqrtTab[i] = floor(32767 * sqrt(i / 512))，       i = 0..512 */
#include "mc_math.h"

const int16_t g_mcSinTab[256] = {{
{rows(sin_tab)}
}};

const int16_t g_mcSqrtTab[513] = {{
{rows(sqrt_tab)}
}};
'''
open(OUT, 'w', encoding='utf-8', newline='\r\n').write(txt)
print('wrote', OUT, len(sin_tab), len(sqrt_tab))
