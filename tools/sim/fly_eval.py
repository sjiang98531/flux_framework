#!/usr/bin/env python3
"""飞车启动评估：转子初速（固件方向的 rpm）× 初始角 × 指令方向，看能否进闭环、用时、有无故障。
   python3 tools/sim/fly_eval.py build/sim_new [build/sim_x ...]           全扫描（120 组）
   python3 tools/sim/fly_eval.py --stress build/sim_new                     参数失配/噪声/延时等 10 种条件
   每格：结果 用时 [Run 子状态序列] 最大 |iq|，结果 OK / FAULT / SLOW（10 s 内没稳在指令 ±100 rpm）。
   用时 = 稳在指令附近的时刻 - 0.2 s（含速度斜坡；冷启动到 1000 rpm 约 6.4 s）。
   接管角误差 thErr = 第一次进 Spin 那一行的 theta_err（仿真用真实转子角算）。"""
import subprocess, sys, os, csv, tempfile
W0 = [-1500, -800, -400, -250, -150, -60, -20, 0, 20, 60, 150, 250, 400, 800, 1500]
TH = [0.3, 1.9, 3.5, 5.1]
def one(b, w0, th, sc, extra):
    fd, fn = tempfile.mkstemp(suffix='.csv'); os.close(fd)
    # 仿真 csv 的 true_rpm = -plant_rpm，所以固件方向 w0 对应 SIM_W0 = -w0
    e = dict(os.environ, SIM_W0=str(-w0), SIM_TH0=str(th), **extra)
    subprocess.run([b, str(sc), fn, '', '10', '0'], env=e, check=True, stdout=subprocess.DEVNULL)
    r = list(csv.DictReader(open(fn))); os.remove(fn)
    fault = any(x['main'] == '0' for x in r if float(x['t']) > 0.25)
    cmd = 1000.0 if sc == 3 else -1000.0
    seq = []
    for x in r:
        if x['main'] == '3' and (not seq or seq[-1] != x['run']): seq.append(x['run'])
    tok = None
    for i, x in enumerate(r):
        if abs(float(x['true_rpm']) - cmd) < 100 and x['run'] == '4':
            if all(abs(float(y['true_rpm']) - cmd) < 100 for y in r[i:i + 100]): tok = float(x['t']) - 0.2; break
    maxi = max(abs(float(x['iq_A'])) for x in r)
    th = next((abs(float(x['theta_err_deg'])) for x in r if x['run'] == '4' and x['main'] == '3'), 0.0)
    return ('FAULT' if fault else ('OK' if tok is not None else 'SLOW')), tok, ''.join(seq)[:12], maxi, th
STRESS = [{}, {'SIM_KR': '0.7'}, {'SIM_KR': '1.3'}, {'SIM_KL': '0.7'}, {'SIM_KL': '1.3'}, {'SIM_KPSI': '0.8'},
          {'SIM_KPSI': '1.2'}, {'SIM_PWMDLY': '1', 'SIM_TD': '0.6e-6'}, {'SIM_TD': '1e-6', 'SIM_NOISE': '4'}, {'SIM_OFF': '20'}]
args = sys.argv[1:]
if args and args[0] == '--stress':
    for b in args[1:]:
        print('==', b, '(stress)')
        tot = okn = 0; thmax = 0.0
        for cond in STRESS:
            fails = []; ths = []
            for sc in (3, 9):
                for w0 in (-1500, -400, -150, -60, 45, 60, 100, 150, 300, 400, 800, 1500):
                    for th in (0.3, 3.5):
                        s_, t, q, m, e = one(b, w0, th, sc, cond)
                        tot += 1
                        if s_ == 'OK': okn += 1
                        else: fails.append(f"{'+' if sc == 3 else '-'}cmd w0{w0:+d} th{th} {s_}[{q}]")
                        if '14' in q[:3]: ths.append(e)
            thmax = max([thmax] + ths)
            print(f"{str(cond) or 'nominal':45s} fail {len(fails)}  catches {len(ths)}  max thErr {max(ths) if ths else 0:.1f}deg  " + '; '.join(fails[:4]))
        print(f'STRESS TOTAL {okn}/{tot} OK, max catch angle error {thmax:.1f} deg')
    sys.exit(0)
for b in args:
    extra = {}
    print('==', b)
    nok = n = 0; worst = 0
    for sc in (3, 9):
        for w0 in W0:
            res = [one(b, w0, th, sc, extra) for th in TH]
            n += len(res); ok = [t for s, t, _, _, _ in res if s == 'OK']; nok += len(ok)
            if ok: worst = max(worst, max(ok))
            print(f"cmd {'+' if sc == 3 else '-'}1000  w0 {w0:+6d}: " + '  '.join(f"{s}{'' if t is None else f' {t:4.2f}s'} [{q}] i{m:.1f}" for s, t, q, m, _ in res))
    print(f'TOTAL {nok}/{n} OK, worst {worst:.2f}s')
