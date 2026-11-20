#!/usr/bin/env python3
"""Startup robustness tests (target: reach I/F and closed loop; performance not required).

    python3 run_startup.py angles [--ref WORKTREE] [--work /tmp/dir]
        10 motor families x 36 initial rotor angles, controller parameters = plant truth (no identification,
        IDENT_MODE_RUN).  Isolates the startup sequence (FlyCheck -> Align/SWING -> I/F -> Spin) from identification.

    python3 run_startup.py identalign [--ref WORKTREE] [--work /tmp/dir]
        10 motor families x 36 initial angles, IDENT_MODE_STATIC: rotor angle at the start of the dead-time
        stairs relative to the align axis (sim convention: controller angle 0 = plant electrical 120 deg),
        blocked runs and unblocked runs with |Rs| or |Lq| error > 15%.

    python3 run_startup.py e2e --seed <seed> [--n 120] [--ref WORKTREE] [--work /tmp/dir]
        Random healthy motors from the blind distribution (run_blind.gen_healthy_jobs) x random initial angle,
        full IDENT_MODE_AUTO flow (static identification -> restart -> run measurement -> run).
        This is the end-to-end "identify then start" success rate.

Success = no supervisor fault, runState == Spin at the end, |rpm| > 20% of the command (30% for `angles`, 10 s).
Outcomes in e2e: ok / ident-blocked (IDENT_WARN_AUTO_BLOCKED stopped the auto restart) / faultXX / other.
"""
import argparse, collections, concurrent.futures, json, os, random, subprocess, sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import run_eval
import run_blind

CMD = {"fan": "1100", "fan48": "1100", "bigL": "1100", "mid16": "3000", "mid48": "3000",
       "sm120": "5000", "sm48": "5000", "vac16": "5000", "vac48": "5000", "ipm2": "1100"}
WORKERS = max(1, min(8, os.cpu_count() or 2))


def sens_line(lines):
    s = [l for l in lines if l.startswith("SENS ")]
    if not s:
        return None
    return dict(x.split("=") for x in s[-1].split()[1:] if "=" in x)


def angles(wd, secs="10"):
    def one(job):
        k, th = job
        env = dict(os.environ)
        env.update({"SIM_PWMDLY": "1", "SIM_NOISE": "2", "SIM_DSAT": "0", "SIM_PK_RS": "1", "SIM_TH0_DEG": str(th)})
        out = subprocess.run([os.path.join(wd, "exe_" + k), secs, "2", CMD[k]], env=env, capture_output=True, text=True).stdout.splitlines()
        kv = sens_line(out)
        ok = bool(kv) and kv["fault"] == "0x0" and kv["run"] == "4" and int(kv["spinExit"]) == 0 and abs(float(kv["rpm"])) > 0.3 * float(CMD[k])
        return k, th, ok, float(kv["tSpin"]) if kv else -1.0

    jobs = [(k, th) for k in CMD for th in range(0, 360, 10)]
    with concurrent.futures.ThreadPoolExecutor(WORKERS) as ex:
        res = list(ex.map(one, jobs))
    total = 0
    for k in CMD:
        r = [x for x in res if x[0] == k]
        bad = [x[1] for x in r if not x[2]]
        ts = sorted(x[3] for x in r if x[2])
        total += len(r) - len(bad)
        print(f"  {k:6s} {len(r) - len(bad):2d}/36  tSpin median {ts[len(ts) // 2] if ts else -1:.2f} s  fail angles {bad}")
    print(f"  total {total}/{len(res)}")
    return res


AXIS_DEG = 120.0     # plant electrical angle of the controller's 0 deg axis (measured: 120.6..123.2 deg incl. friction)


def identalign(wd):
    def one(job):
        k, th = job
        env = dict(os.environ)
        env.update({"SIM_PWMDLY": "1", "SIM_NOISE": "2", "SIM_DSAT": "0", "SIM_TH0_DEG": str(th),
                    "SIM_DIAG_TIER": "1", "SIM_DIAG_DT": "100"})
        out = subprocess.run([os.path.join(wd, "exe_" + k), "35", "1", "1100"], env=env, capture_output=True, text=True).stdout.splitlines()
        dt = [l for l in out if l.startswith("TRACE") and " ph=1 " in l]
        t = [l for l in out if l.startswith("TRUE")]
        r = [l for l in out if l.startswith("RES")]
        if not (dt and t and r):
            return k, th, None, None, True, 0.0, 0.0
        rotor = float(dt[0].split("rotor=")[1].split()[0])
        t_dt = float(dt[0].split()[1][2:])
        T = run_eval.parse_kv(t[0]); R = run_eval.parse_kv(r[0])
        blk = bool(int(R["warn"], 16) & 0x10)
        rs = (float(R["rs"]) / float(T["rs"]) - 1) * 100 if float(R["rs"]) > 0 else -100.0
        lq = (float(R["lq"]) / float(T["lq"]) - 1) * 100 if float(R["lq"]) > 0 else -100.0
        return k, th, ((rotor - AXIS_DEG + 180) % 360) - 180, t_dt, blk, rs, lq

    jobs = [(k, th) for k in CMD for th in range(0, 360, 10)]
    with concurrent.futures.ThreadPoolExecutor(WORKERS) as ex:
        res = list(ex.map(one, jobs))
    for k in CMD:
        r = [x for x in res if x[0] == k and x[2] is not None]
        off = [(x[1], round(x[2])) for x in r if abs(x[2]) >= 15]
        blk = [x[1] for x in r if x[4]]
        bad = [x[1] for x in r if not x[4] and (abs(x[5]) > 15 or abs(x[6]) > 15)]
        ts = sorted(x[3] for x in r)
        print(f"  {k:6s} align time median {ts[len(ts) // 2] - 0.25 if ts else -1:.2f} s | off axis >=15 deg {off} | blocked {blk} | unblocked |Rs|/|Lq| > 15% {bad}")
    return res


def e2e(wd, seed, n, secs="30"):
    jobs = run_blind.gen_healthy_jobs(seed, n)
    rng = random.Random(seed + 7)
    pre = [(j, rng.randrange(360)) for j in jobs]

    def one(x):
        (tag, k, md, rpm, envx, meta), th = x
        env = dict(os.environ)
        env.update(envx)
        env.update({"SIM_TH0_DEG": str(th), "SIM_PK_RS": "1"})
        out = subprocess.run([os.path.join(wd, "exe_" + k), secs, "0", CMD[k]], env=env, capture_output=True, text=True).stdout.splitlines()
        kv = sens_line(out)
        r = [l for l in out if l.startswith("RES ")]
        res = dict(x.split("=") for x in r[0].split()[1:] if "=" in x) if r else {}
        warn = int(res.get("warn", "0"), 16) if res else 0
        if not kv:
            why = "noout"
        elif kv["fault"] == "0x0" and kv["run"] == "4" and abs(float(kv["rpm"])) > 0.2 * float(CMD[k]):
            why = "ok"
        elif warn & 0x10:
            why = "ident-blocked"
        elif kv["fault"] != "0x0":
            why = "fault" + kv["fault"]
        else:
            why = "other"
        return tag, k, th, why

    with concurrent.futures.ThreadPoolExecutor(WORKERS) as ex:
        res = list(ex.map(one, pre))
    c = collections.Counter(r[3] for r in res)
    print(f"  ok {c['ok']}/{len(res)}  outcomes {dict(c)}")
    fam = collections.defaultdict(list)
    for r in res:
        fam[r[1]].append(r[3] == "ok")
    print("  " + "  ".join(f"{k}:{sum(v)}/{len(v)}" for k, v in sorted(fam.items())))
    print("  failures:", [(r[0], r[1], r[2], r[3]) for r in res if r[3] != "ok"])
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=["angles", "identalign", "e2e"])
    ap.add_argument("--ref", default="WORKTREE")
    ap.add_argument("--work", default="/tmp/ff_startup")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--n", type=int, default=120)
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--json", default=None)
    ap.add_argument("--secs", default=None, help="simulated seconds per run (default 10 for angles, 30 for e2e)")
    ap.add_argument("--retry", action="store_true", help="keep simulating after a fault (SIM_FAULT_CONTINUE=1) so automatic restarts are exercised")
    a = ap.parse_args()
    if a.retry:
        os.environ["SIM_FAULT_CONTINUE"] = "1"
    if not a.no_build:
        run_eval.build_suite(a.ref, a.work)
    if a.test == "angles":
        print(f"=== initial-angle sweep ({a.ref}) ===")
        res = angles(a.work, a.secs or "10")
    elif a.test == "identalign":
        print(f"=== identification align, initial-angle sweep ({a.ref}) ===")
        res = identalign(a.work)
    else:
        if a.seed is None:
            ap.error("e2e needs --seed")
        print(f"=== end-to-end AUTO identification + start, seed={a.seed} n={a.n} ({a.ref}) ===")
        res = e2e(a.work, a.seed, a.n, a.secs or "30")
    if a.json:
        json.dump(res, open(a.json, "w"))


if __name__ == "__main__":
    main()
