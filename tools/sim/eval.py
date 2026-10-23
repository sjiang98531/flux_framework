#!/usr/bin/env python3
"""Performance evaluation of one or more sim binaries.
   python3 tools/sim/eval.py build/sim_new [build/sim_x ...]
Prints startup robustness, speed-step response, load step, low-speed angle error, high speed."""
import subprocess, sys, os, csv, statistics as st, tempfile, itertools
def run(binp, sc, T, env=None, ipm=0):
    fd, fn = tempfile.mkstemp(suffix='.csv'); os.close(fd)
    e = dict(os.environ); e.update(env or {})
    subprocess.run([binp, str(sc), fn, '', str(T), str(ipm)], env=e, check=True, stdout=subprocess.DEVNULL)
    rows = list(csv.DictReader(open(fn))); os.remove(fn)
    return {k: [float(r[k]) for r in rows] for k in rows[0]}
def win(d, k, a, b): return [v for t, v in zip(d['t'], d[k]) if a <= t < b]
def startup(binp, ipm=0):
    ok = 0; times = []; fb = 0; n = 0
    for th, td in itertools.product([0.1, 0.8, 1.5, 2.3, 3.1, 3.9, 4.7, 5.5], [0, 0.2e-6, 0.4e-6]):
        d = run(binp, 3, 6, {'SIM_TH0': str(th), 'SIM_TD': str(td)}, ipm); n += 1
        spin = [t for t, r, m in zip(d['t'], d['run'], d['main']) if r == 4 and m == 3]
        fault = any(m == 0 for t, m in zip(d['t'], d['main']) if t > 0.3)
        if spin and spin[0] < 5.0 and not fault:
            ok += 1; times.append(spin[0] - 0.2)
            after = [r for t, r in zip(d['t'], d['run']) if t > spin[0]]
            if 3 in after: fb += 1
    return n, ok, times, fb
def step(binp):
    d = run(binp, 6, 60)
    out = {}
    for name, (a, b, tgt, prev) in {'1172': (0.2, 20, 1172, 0), '600': (20, 40, 600, 1172), '1172b': (40, 60, 1172, 600)}.items():
        sp = win(d, 'true_rpm', a, b); tt = win(d, 't', a, b)
        if tgt > prev: ov = max(sp) - tgt
        else: ov = tgt - min(sp)
        settle = None
        for i in range(len(sp) - 1, -1, -1):
            if abs(sp[i] - tgt) > 0.02 * tgt: settle = tt[i] - a; break
        last = win(d, 'true_rpm', b - 5, b)
        out[name] = (ov, settle, st.pstdev(last), st.mean(win(d, 'iq_A', b - 5, b)))
    return out
def load(binp):
    d = run(binp, 8, 40)
    sp = win(d, 'true_rpm', 15, 30); tt = win(d, 't', 15, 30)
    dip = 1000 - min(sp)
    rec = None
    for i in range(len(sp) - 1, -1, -1):
        if abs(sp[i] - 1000) > 20: rec = tt[i] - 15; break
    return dip, rec
def lowspeed(binp):
    d = run(binp, 7, 30)
    res = {}
    for name, (a, b) in {'300': (5, 10), '150': (15, 20), '80': (25, 30)}.items():
        e = win(d, 'theta_err_deg', a, b); sp = win(d, 'true_rpm', a, b); run_ = win(d, 'run', a, b)
        res[name] = (st.mean(e), st.pstdev(e), st.mean(sp), min(run_))
    return res
def high(binp):
    d = run(binp, 2, 16)
    return st.mean(win(d, 'true_rpm', 14, 16)), st.mean(win(d, 'idcmd_A', 14, 16)), st.mean(win(d, 'theta_err_deg', 14, 16))
for b in sys.argv[1:]:
    print('=====', b)
    n, ok, times, fb = startup(b)
    print(f'startup: {ok}/{n} ok (<3 s, no fault), time mean {st.mean(times) if times else 0:.2f}s max {max(times) if times else 0:.2f}s, fell back after spin {fb}')
    for k, (ov, se, rip, iq) in step(b).items():
        print(f'step ->{k:6s}: overshoot {ov:6.1f} rpm, settle(2%) {se if se is not None else 0:5.2f}s, ripple(sd, last 5s) {rip:5.2f} rpm, iq {iq:.3f}A')
    dip, rec = load(b); print(f'load +0.5Nm @1000: dip {dip:.1f} rpm, back within ±20 rpm after {rec if rec is not None else 0:.2f}s')
    for k, (m, s, sp, mr) in lowspeed(b).items():
        print(f'low {k:>3s} rpm: theta err {m:6.2f}±{s:5.2f} deg, speed {sp:6.1f}, min runstate {mr:.0f}')
    hs, idc, te = high(b); print(f'high 2600 cmd: {hs:.0f} rpm, id_cmd {idc:.2f} A, theta err {te:.1f} deg')
