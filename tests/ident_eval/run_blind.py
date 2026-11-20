#!/usr/bin/env python3
"""Parameterised blind-test runner - Protocol v2.1 (PROTOCOL_V2.md section 7).

Usage:
    python3 run_blind.py --seed <fresh seed> [--ref WORKTREE|<git ref>] [--work /tmp/dir] [--json out.json]

Generate the seed with `python3 -c "import secrets; print(secrets.randbelow(10**9))"` and use it once.

Acceptance lines (unchanged from v2.1, written before running):
- N=300 healthy motors with combined perturbations (igain/vbus U[0.95,1.05], off_run +-0.5%FS,
  deadtime U[0.7,1.3], Rs drift U[0,0.15]).
- SwpL residual after removing known sensor scaling: p95 <= 8%.
- High-risk silent failure (SwpL > 25% and not blocked): == 0.
- Healthy false block FP <= 3%.
- Stress subset (off_run +-2%FS, N=30): silent failure == 0 (blocks and safe aborts allowed).
- Bad-motor recall >= 90% on both the frozen (20261188) and independent (704910903) sets,
  ambiguous wind < 0.13 Nm excluded.

Metric corrections vs run_blind_670303305.py (v2.1 definitions, implementation only):
1. A run that ends with a supervisor fault before identification completes (state != DONE, phase == 0,
   nothing committed) is a SAFE ABORT, counted with blocks - as run_blind_661924030.py already did.
   The 670303305 runner counted these as silent failures.
2. Rs is reported against the cold (pre-drift) true Rs, as the perturbation suite defines
   (`SIM_R_DRIFT=+0.15 -> |Rs| <= 4%`).  The residual only removes the known V/I sensor scaling z;
   the previous residual also removed (1 + drift), which double-counted the drift.
"""
import argparse, collections, json, os, random, sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import run_eval

N_HEALTHY = 300


def family(rng, u, pwm48):
    L = 15e-6 * ((120e-3 / 15e-6) ** u)
    if L < 60e-6:
        return L, ("vac48" if pwm48 else "vac16"), 0.073 * ((L / 15.8e-6) ** 0.5), (0.35e-6 if pwm48 else 0.82e-6), 0.50, 5.0
    if L < 800e-6:
        return L, ("sm48" if pwm48 else "sm120"), 0.12 * ((L / 150e-6) ** 0.5), (0.40e-6 if pwm48 else 1.20e-6), 0.40, 2.0
    if L < 12e-3:
        return L, ("mid48" if pwm48 else "mid16"), 1.20 * ((L / 3e-3) ** 0.6), (0.40e-6 if pwm48 else 0.82e-6), 0.30, 1.0
    key = "fan48" if pwm48 else ("bigL" if L > 65e-3 else "fan")
    return L, key, 7.35 * ((L / 43.3e-3) ** 0.5), (0.40e-6 if pwm48 else 0.82e-6), 0.23, 0.5


def gen_healthy_jobs(seed, count):
    """Identical distribution and RNG call order to run_blind_670303305.gen_healthy_jobs_blind."""
    rng = random.Random(seed)
    jobs = []
    for idx in range(count):
        u = rng.random()
        pwm48 = rng.choice([False, True])
        L, key, r_nom, td_nom, dti0, i0_sat = family(rng, u, pwm48)
        r_mult = rng.uniform(0.5, 1.5); td_mult = rng.uniform(0.5, 1.5); ir_mult = rng.uniform(0.5, 1.5)
        sal = rng.uniform(1.0, 1.45); ksat = rng.uniform(0.0, 0.03) if L < 1e-3 else rng.uniform(0.0, 0.09)
        igain = rng.uniform(0.95, 1.05)
        vbusg = rng.uniform(0.95, 1.05)
        tdm = rng.uniform(0.70, 1.30)
        rdrift = rng.uniform(0.0, 0.15)
        offrun = rng.choice([-10, -5, 0, 5, 10])
        envx = {
            "SIM_PWMDLY": "1", "SIM_NOISE": str(rng.choice([2, 4])), "SIM_SEED": str(seed + idx * 97),
            "SIM_RS": f"{r_nom * r_mult:.6g}", "SIM_LD": f"{L:.6g}", "SIM_LQ": f"{L * sal:.6g}",
            "SIM_TD": f"{td_nom * td_mult * tdm:.6g}", "SIM_DTI0": f"{dti0 * rng.uniform(0.7, 1.3):.4f}",
            "SIM_QSAT": f"{ksat:.5f}", "SIM_QSAT_I0": f"{i0_sat:.3f}", "SIM_DSAT": "0",
            "SIM_IRATED_K": f"{ir_mult:.4f}",
            "SIM_IGAIN": f"{igain:.4f}", "SIM_VBUS_GAIN": f"{vbusg:.4f}",
            "SIM_R_DRIFT": f"{rdrift:.4f}", "SIM_OFF_RUN": str(offrun),
        }
        jobs.append((f"h{idx:03d}", key, "1", "1100", envx, {"igain": igain, "vbusg": vbusg, "rdrift": rdrift, "tdm": tdm}))
    return jobs


def gen_stress_jobs(seed, count=30):
    """Identical to the stress subset of run_blind_670303305.py (seed + 1)."""
    rng2 = random.Random(seed + 1)
    jobs = []
    for idx in range(count):
        u = rng2.random()
        pwm48 = rng2.choice([False, True])
        L, key, r_nom, td_nom, dti0, i0_sat = family(rng2, u, pwm48)
        sal = rng2.uniform(1.0, 1.45); ksat = rng2.uniform(0.0, 0.03) if L < 1e-3 else rng2.uniform(0.0, 0.09)
        envx = {"SIM_PWMDLY": "1", "SIM_NOISE": str(rng2.choice([2, 4])), "SIM_SEED": str(seed + 1000 + idx * 97),
                "SIM_RS": f"{r_nom * rng2.uniform(0.5, 1.5):.6g}", "SIM_LD": f"{L:.6g}", "SIM_LQ": f"{L * sal:.6g}",
                "SIM_TD": f"{td_nom * rng2.uniform(0.5, 1.5):.6g}", "SIM_DTI0": f"{dti0 * rng2.uniform(0.7, 1.3):.4f}",
                "SIM_QSAT": f"{ksat:.5f}", "SIM_QSAT_I0": f"{i0_sat:.3f}", "SIM_DSAT": "0",
                "SIM_IRATED_K": f"{rng2.uniform(0.5, 1.5):.4f}", "SIM_OFF_RUN": rng2.choice(["41", "-41"])}
        jobs.append((f"st{idx:02d}", key, "1", "1100", envx))
    return jobs


def run(wd, key, md, rpm, envx):
    r = run_eval.run_case(wd, key, md, rpm, envx)
    res = r.get("res")
    r["safe_abort"] = bool(res) and int(res["state"]) != 2 and int(res["phase"]) == 0 and int(res.get("src", "0")) != 2
    if r["safe_abort"]:
        r["blk"] = True
    return r


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * (len(v) - 1)))] if v else 0.0


def bad_gate(wd, seed):
    tp = fn = amb = 0
    fns = []
    for j in run_eval.gen_bad_motor_jobs(seed, 30):
        if j[0].startswith("bad_wnd") and float(j[4].get("SIM_WIND", "0")) < 0.13:
            amb += 1
            continue
        r = run(wd, j[1], j[2], j[3], j[4])
        if r["bad"]:
            if r["blk"]:
                tp += 1
            else:
                fn += 1
                fns.append((j[0], round(r["swp_l"], 2), round(r["rs_e"], 2), r["res"]["warn"]))
    return tp, fn, amb, fns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--ref", default="WORKTREE")
    ap.add_argument("--work", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--no-build", action="store_true")
    a = ap.parse_args()
    wd = a.work or f"/tmp/ff_blind_{a.seed}"
    if not a.no_build:
        run_eval.build_suite(a.ref, wd)

    rows = []
    for tag, key, md, rpm, envx, meta in gen_healthy_jobs(a.seed, N_HEALTHY):
        r = run(wd, key, md, rpm, envx)
        z = meta["vbusg"] / meta["igain"]
        r["swp_resid"] = max(0.0, r["swp_l"] - abs(z - 1.0) * 100.0)
        r["rs_resid_e"] = r["rs_e"] - (z - 1.0) * 100.0
        rows.append((tag, key, r))

    unb = [r for _, _, r in rows if not r["blk"]]
    high_risk = [(t, k, r) for t, k, r in rows if not r["blk"] and r["swp_l"] > 25.0]
    bad_unb = [(t, k, r) for t, k, r in rows if not r["blk"] and r["bad"]]
    fp = [(t, k, r) for t, k, r in rows if r["blk"] and not r["bad"]]
    n_abort = sum(1 for _, _, r in rows if r["safe_abort"])

    print(f"=== BLIND HEALTHY N={N_HEALTHY} seed={a.seed} ref={a.ref} ===")
    print(f"  unblocked {len(unb)} | blocked {N_HEALTHY - len(unb)} (safe abort {n_abort})")
    print(f"  SwpL raw      p50={pct([r['swp_l'] for r in unb], .5):.2f}% p95={pct([r['swp_l'] for r in unb], .95):.2f}% max={max(r['swp_l'] for r in unb):.2f}%")
    print(f"  SwpL residual p50={pct([r['swp_resid'] for r in unb], .5):.2f}% p95={pct([r['swp_resid'] for r in unb], .95):.2f}% max={max(r['swp_resid'] for r in unb):.2f}%   [ACCEPT p95<=8%]")
    print(f"  |Rs| raw      p50={pct([abs(r['rs_e']) for r in unb], .5):.2f}% p95={pct([abs(r['rs_e']) for r in unb], .95):.2f}% max={max(abs(r['rs_e']) for r in unb):.2f}%")
    print(f"  |Rs| resid(z) p50={pct([abs(r['rs_resid_e']) for r in unb], .5):.2f}% p95={pct([abs(r['rs_resid_e']) for r in unb], .95):.2f}%")
    print(f"  High-risk silent (SwpL>25% unblocked): {len(high_risk)}   [ACCEPT ==0]")
    for t, k, r in high_risk:
        print(f"    {t} ({k}) swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
    print(f"  Bad unblocked (SwpL>15 or |Rs|>15 or state): {len(bad_unb)}   [report]")
    for t, k, r in bad_unb:
        print(f"    {t} ({k}) swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
    print(f"  FP (blocked healthy): {len(fp)} = {len(fp) / N_HEALTHY * 100:.1f}%   [ACCEPT <=3%]")
    for t, k, r in fp:
        print(f"    {t} ({k}) warn={r['res']['warn']} swp={r['swp_l']:.1f}%")

    fam = collections.defaultdict(list)
    for t, k, r in rows:
        fam[k].append(r)
    print("  per family: unblocked/total, SwpL residual p50/p95/max")
    for k in sorted(fam):
        u = [r["swp_resid"] for r in fam[k] if not r["blk"]]
        if u:
            print(f"    {k:7s} {len(u):3d}/{len(fam[k]):3d}  {pct(u, .5):5.2f} {pct(u, .95):5.2f} {max(u):5.2f}")
        else:
            print(f"    {k:7s}   0/{len(fam[k]):3d}")

    print("\n=== STRESS SUBSET off_run=+-2%FS (N=30) ===")
    n_blk = n_ab = 0
    silent = []
    for tag, key, md, rpm, envx in gen_stress_jobs(a.seed):
        r = run(wd, key, md, rpm, envx)
        if r["safe_abort"]:
            n_ab += 1
        elif r["blk"]:
            n_blk += 1
        elif r["swp_l"] > 25.0 or abs(r["rs_e"]) > 15.0:
            silent.append((tag, key, round(r["swp_l"], 1), round(r["rs_e"], 1), r["res"]["warn"]))
    print(f"  blocked {n_blk} | safe abort {n_ab} | silent fail {len(silent)}   [ACCEPT silent==0]")
    for s in silent:
        print("   ", s)

    summary = {"seed": a.seed, "ref": a.ref, "n": N_HEALTHY, "unblocked": len(unb),
               "swp_res_p95": pct([r["swp_resid"] for r in unb], .95), "high_risk": len(high_risk),
               "bad_unblocked": len(bad_unb), "fp_rate": len(fp) / N_HEALTHY, "stress_silent": len(silent)}
    for name, s in [("frozen", 20261188), ("independent", 704910903)]:
        tp, fn, amb, fns = bad_gate(wd, s)
        rec = tp / (tp + fn) if tp + fn else 1.0
        print(f"\n=== BAD-MOTOR GATE {name} (seed {s}) === recall {tp}/{tp + fn} = {rec * 100:.0f}% (ambiguous excluded {amb})   [ACCEPT >=90%]")
        for f in fns:
            print("   FN:", f)
        summary[f"recall_{name}"] = rec

    if a.json:
        json.dump(summary, open(a.json, "w"), indent=2)


if __name__ == "__main__":
    main()
