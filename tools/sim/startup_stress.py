#!/usr/bin/env python3
"""冷启动压力扫描（0xF20E0004 审核用）：死区 / Rs / 噪声 / L / psi / J 失配 x 8 个初始角，全部带 PWM 晚一拍。
   python3 tools/sim/startup_stress.py tools/sim/build/sim_legacy tools/sim/build/sim_new
判定：8 s 内进闭环（<5 s）、无故障、最后 1 s 平均转速 > 900 rpm（指令 1000）。"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
BINS = sys.argv[1:]; sys.argv = sys.argv[:1]
import eval as E
from concurrent.futures import ProcessPoolExecutor
BASE = dict(os.environ)
def job(a):
    b, env, th = a
    os.environ.clear(); os.environ.update(BASE); os.environ.update(env)
    os.environ['SIM_PWMDLY'] = '1'; os.environ['SIM_TH0'] = str(th)
    d = E.run(b, 3, 8)
    spin = [t for t, r, m in zip(d['t'], d['run'], d['main']) if r == 4 and m == 3]
    fault = any(m == 0 for t, m in zip(d['t'], d['main']) if t > 0.3)
    last = E.win(d, 'true_rpm', 7, 8)
    ok = bool(spin and spin[0] < 5 and not fault and sum(last) / len(last) > 900)
    return b, tuple(sorted(env.items())), ok, (spin[0] - 0.2 if spin else None)
conds = [{'SIM_TD': str(td)} for td in [0, 0.2e-6, 0.4e-6, 0.6e-6, 0.8e-6, 1.0e-6, 1.2e-6]] + \
        [{'SIM_TD': '0.6e-6', 'SIM_KR': str(k)} for k in [0.8, 0.9, 1.1, 1.2, 1.3]] + \
        [{'SIM_TD': '0.6e-6', k: v} for k, v in [('SIM_NOISE', '4'), ('SIM_KL', '0.7'), ('SIM_KL', '1.4'),
                                                 ('SIM_KPSI', '0.9'), ('SIM_KJ', '2'), ('SIM_KJ', '0.5')]]
jobs = [(b, c, th) for c in conds for b in BINS for th in [0.1, 0.8, 1.5, 2.3, 3.1, 3.9, 4.7, 5.5]]
res = {}
with ProcessPoolExecutor() as ex:
    for b, c, ok, t in ex.map(job, jobs):
        r = res.setdefault(c, {}).setdefault(b, [0, []]); r[0] += ok
        if ok: r[1].append(t)
names = [os.path.basename(b) for b in BINS]
print('%-28s' % 'condition', ''.join('%-20s' % n for n in names))
tot = dict.fromkeys(BINS, 0)
for c, v in res.items():
    print('%-28s' % ' '.join(f'{k[4:]}={x}' for k, x in c),
          ''.join('%-20s' % (f"{v[b][0]}/8 t<={max(v[b][1]):.1f}s" if v[b][1] else f"{v[b][0]}/8") for b in BINS))
    for b in BINS: tot[b] += v[b][0]
print('TOTAL', '  '.join(f'{os.path.basename(b)} {n}/{8 * len(res)}' for b, n in tot.items()))
