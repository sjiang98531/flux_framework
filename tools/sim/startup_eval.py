#!/usr/bin/env python3
"""Cold-start robustness over many initial rotor angles / dead-time errors / offsets.
   python3 tools/sim/startup_eval.py build/sim_a build/sim_b ..."""
import subprocess, sys, os, csv, tempfile, itertools, statistics as st
def one(b, th, td, ofs):
    fd, fn = tempfile.mkstemp(suffix='.csv'); os.close(fd)
    e = dict(os.environ, SIM_TH0=str(th), SIM_TD=str(td), SIM_OFF=str(ofs))
    subprocess.run([b, '3', fn, '', '6', '0'], env=e, check=True, stdout=subprocess.DEVNULL)
    r = list(csv.DictReader(open(fn))); os.remove(fn)
    spin = [float(x['t']) for x in r if x['run'] == '4' and x['main'] == '3']
    fault = any(x['main'] == '0' for x in r if float(x['t']) > 0.3)
    return (spin[0] - 0.2) if (spin and spin[0] < 3.0 and not fault) else None
for b in sys.argv[1:]:
    res = [one(b, th / 16 * 6.2832, td, ofs) for th, td, ofs in itertools.product(range(16), [0, 0.1e-6, 0.2e-6, 0.3e-6], [0, 3])]
    ok = [x for x in res if x is not None]
    print(f'{os.path.basename(b):22s} {len(ok)}/{len(res)} ok, mean {st.mean(ok):.2f}s, max {max(ok):.2f}s')
