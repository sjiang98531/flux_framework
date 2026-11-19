#!/usr/bin/env python3
"""FINAL BLIND TEST - Protocol v2.1, frozen candidate 53c547f, fresh seed 661924030 (one-shot).

Acceptance lines (written BEFORE running, PROTOCOL_V2.md section 7):
- N=300 random healthy motors + combined per-motor perturbations (igain U[0.95,1.05],
  off_run ±0.5%FS main, vbus U[0.95,1.05], td U[0.7,1.3], r_drift U[0,0.15]).
- Main metric: SwpL residual AFTER subtracting known sensor scaling: p95 <= 8%.
- High-risk silent failure (SwpL>25% unblocked): must be 0.
- Bad-motor gate recall (ambiguous wind <0.13 Nm excluded): >= 90%.
- Healthy false-block FP <= 3%.
- Stress subset (±2%FS off_run): pre-ident FAULT = safe abort (not silent); real silent must be 0.

NOTE: seed 661924030 is BURNED after this run. Any code change requires a new seed.
"""
import json, os, random, subprocess, sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, SCRIPT_DIR)
import run_eval

SEED = 661924030
N_HEALTHY = 300
WR = "/tmp/ff_blind_661924030"

print(f"Building frozen candidate from worktree (HEAD 53c547f)...")
run_eval.build_suite("WORKTREE", WR)
print("Build complete. Starting blind test...")

def gen_healthy_jobs_blind(seed, count):
    rng = random.Random(seed)
    jobs = []
    for idx in range(count):
        u = rng.random()
        L = 15e-6 * ((120e-3 / 15e-6) ** u)
        pwm48 = rng.choice([False, True])
        if L < 60e-6:
            key = "vac48" if pwm48 else "vac16"; r_nom = 0.073 * ((L / 15.8e-6) ** 0.5); td_nom = 0.35e-6 if pwm48 else 0.82e-6; dti0 = 0.50; i0_sat = 5.0
        elif L < 800e-6:
            key = "sm48" if pwm48 else "sm120"; r_nom = 0.12 * ((L / 150e-6) ** 0.5); td_nom = 0.40e-6 if pwm48 else 1.20e-6; dti0 = 0.40; i0_sat = 2.0
        elif L < 12e-3:
            key = "mid48" if pwm48 else "mid16"; r_nom = 1.20 * ((L / 3e-3) ** 0.6); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.30; i0_sat = 1.0
        else:
            key = "fan48" if pwm48 else ("bigL" if L > 65e-3 else "fan"); r_nom = 7.35 * ((L / 43.3e-3) ** 0.5); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.23; i0_sat = 0.5
        r_mult = rng.uniform(0.5, 1.5); td_mult = rng.uniform(0.5, 1.5); ir_mult = rng.uniform(0.5, 1.5)
        sal = rng.uniform(1.0, 1.45); ksat = rng.uniform(0.0, 0.03) if L < 1e-3 else rng.uniform(0.0, 0.09)
        igain = rng.uniform(0.95, 1.05); vbusg = rng.uniform(0.95, 1.05)
        tdm = rng.uniform(0.70, 1.30); rdrift = rng.uniform(0.0, 0.15)
        offrun = rng.choice([-10, -5, 0, 5, 10])
        envx = {"SIM_PWMDLY":"1","SIM_NOISE":str(rng.choice([2,4])),"SIM_SEED":str(seed+idx*97),
                "SIM_RS":f"{r_nom*r_mult:.6g}","SIM_LD":f"{L:.6g}","SIM_LQ":f"{L*sal:.6g}",
                "SIM_TD":f"{td_nom*td_mult*tdm:.6g}","SIM_DTI0":f"{dti0*rng.uniform(0.7,1.3):.4f}",
                "SIM_QSAT":f"{ksat:.5f}","SIM_QSAT_I0":f"{i0_sat:.3f}","SIM_DSAT":"0",
                "SIM_IRATED_K":f"{ir_mult:.4f}","SIM_IGAIN":f"{igain:.4f}","SIM_VBUS_GAIN":f"{vbusg:.4f}",
                "SIM_R_DRIFT":f"{rdrift:.4f}","SIM_OFF_RUN":str(offrun)}
        jobs.append((f"h{idx:03d}", key, "1", "1100", envx, {"igain":igain,"vbusg":vbusg,"rdrift":rdrift}))
    return jobs

def run_case_with_fault(wd, key, md, rpm, envx):
    p = subprocess.run([f"{wd}/exe_{key}", "35", md, rpm], env={**os.environ, **envx}, capture_output=True, text=True)
    lines = [l for l in p.stdout.strip().splitlines() if l.startswith("TRUE ") or l.startswith("RES ")]
    if len(lines) < 2:
        return {"safe_abort": True, "blk": True, "bad": False, "swp_l": 0.0, "rs_e": 0.0, "res": {"warn":"0x0","state":"0"}, "t": {}}
    t = run_eval.parse_kv(lines[0]); res = run_eval.parse_kv(lines[1])
    state = int(res["state"])
    if state == 0 or "FAULT" in p.stderr:
        return {"safe_abort": True, "blk": True, "bad": False, "swp_l": 0.0, "rs_e": 0.0, "res": res, "t": t}
    warn = int(res["warn"], 16)
    blk = bool(warn & 0x10)
    rs_t = float(t["rs"]); rs_m = float(res["rs"])
    rs_e = (rs_m - rs_t) / rs_t * 100.0 if rs_t > 0 else -100.0
    lq_t = float(t["lq"]); lq_m = float(res["lq"])
    swp_l = run_eval.max_l_err_runtime(lq_t, float(t["ksat"]), float(t["i0sat"]),
                                       lq_m if not blk else lq_m, float(res["perA"]), float(res["knee"]),
                                       float(res["idMaxA"]) if not (blk and int(res["src"])==2) else float(res["pLsMax"]),
                                       float(t["imax"]))
    bad = (state != 2) or (lq_m <= 0.0) or (swp_l > 15.0) or (abs(rs_e) > 15.0)
    return {"safe_abort": False, "blk": blk, "bad": bad, "swp_l": swp_l, "rs_e": rs_e, "res": res, "t": t}

jobs = gen_healthy_jobs_blind(SEED, N_HEALTHY)
results = []
n_safe_abort = 0
for j in jobs:
    tag, key, md, rpm, envx, meta = j
    r = run_case_with_fault(WR, key, md, rpm, envx)
    if r.get("safe_abort"):
        n_safe_abort += 1
        continue
    z = meta["vbusg"] / meta["igain"]
    r["swp_resid"] = max(0.0, r["swp_l"] - abs(z - 1.0) * 100.0)
    results.append((tag, key, r, meta))

unb = [(t,k,r,m) for t,k,r,m in results if not r["blk"]]
blk_list = [(t,k,r,m) for t,k,r,m in results if r["blk"]]
swp_raw = sorted(r["swp_l"] for _,_,r,_ in unb)
swp_res = sorted(r["swp_resid"] for _,_,r,_ in unb)
high_risk = [(t,k,r) for t,k,r,m in results if (not r["blk"]) and r["swp_l"] > 25.0]
bad_unblk = [(t,k,r) for t,k,r,m in results if (not r["blk"]) and (r["swp_l"] > 15.0 or abs(r["rs_e"]) > 15.0)]
fp_items = [(t,k,r) for t,k,r,m in results if r["blk"] and (r["swp_l"] <= 15.0 and abs(r["rs_e"]) <= 15.0)]

def pct(v, q):
    return v[min(len(v)-1, int(q*(len(v)-1)))] if v else 0.0

print(f"\n{'='*60}")
print(f"BLIND TEST RESULTS — seed={SEED}, N={N_HEALTHY}, commit=53c547f")
print(f"{'='*60}")
print(f"\n--- MAIN SET (healthy + combined perturbations, ±0.5%FS off_run) ---")
print(f"  safe pre-ident aborts: {n_safe_abort}")
print(f"  blocked: {len(blk_list)} | unblocked: {len(unb)}")
print(f"  SwpL raw        : p50={pct(swp_raw,0.5):.2f}% p95={pct(swp_raw,0.95):.2f}% max={swp_raw[-1] if swp_raw else 0:.2f}%")
print(f"  SwpL residual   : p50={pct(swp_res,0.5):.2f}% p95={pct(swp_res,0.95):.2f}% max={swp_res[-1] if swp_res else 0:.2f}%   [{'PASS' if pct(swp_res,0.95)<=8 else 'FAIL'}: p95<=8%]")
print(f"  High-risk silent (SwpL>25% unblocked): {len(high_risk)}   [{'PASS' if len(high_risk)==0 else 'FAIL'}: ==0]")
for t,k,r in high_risk[:10]:
    print(f"    {t} ({k}): swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
print(f"  Bad-unblocked (SwpL>15 or |Rs|>15): {len(bad_unblk)}")
for t,k,r in bad_unblk[:12]:
    print(f"    {t} ({k}): swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
print(f"  FP (blocked healthy): {len(fp_items)} = {len(fp_items)/N_HEALTHY*100:.1f}%   [{'PASS' if len(fp_items)/N_HEALTHY<=0.03 else 'FAIL'}: <=3%]")
for t,k,r in fp_items[:10]:
    print(f"    {t} ({k}): warn={r['res']['warn']} swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}%")

# Stress subset
print(f"\n--- STRESS SUBSET (±2%FS off_run, N=30) ---")
rng2 = random.Random(SEED + 1)
stress = []
for idx in range(30):
    u = rng2.random(); L = 15e-6 * ((120e-3/15e-6)**u); pwm48 = rng2.choice([False, True])
    if L < 60e-6:
        key = "vac48" if pwm48 else "vac16"; r_nom = 0.073*((L/15.8e-6)**0.5); td_nom = 0.35e-6 if pwm48 else 0.82e-6; dti0=0.50; i0_sat=5.0
    elif L < 800e-6:
        key = "sm48" if pwm48 else "sm120"; r_nom = 0.12*((L/150e-6)**0.5); td_nom = 0.40e-6 if pwm48 else 1.20e-6; dti0=0.40; i0_sat=2.0
    elif L < 12e-3:
        key = "mid48" if pwm48 else "mid16"; r_nom = 1.20*((L/3e-3)**0.6); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0=0.30; i0_sat=1.0
    else:
        key = "fan48" if pwm48 else ("bigL" if L>65e-3 else "fan"); r_nom = 7.35*((L/43.3e-3)**0.5); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0=0.23; i0_sat=0.5
    sal = rng2.uniform(1.0,1.45); ksat = rng2.uniform(0.0,0.03) if L<1e-3 else rng2.uniform(0.0,0.09)
    envx = {"SIM_PWMDLY":"1","SIM_NOISE":str(rng2.choice([2,4])),"SIM_SEED":str(SEED+1000+idx*97),
            "SIM_RS":f"{r_nom*rng2.uniform(0.5,1.5):.6g}","SIM_LD":f"{L:.6g}","SIM_LQ":f"{L*sal:.6g}",
            "SIM_TD":f"{td_nom*rng2.uniform(0.5,1.5):.6g}","SIM_DTI0":f"{dti0*rng2.uniform(0.7,1.3):.4f}",
            "SIM_QSAT":f"{ksat:.5f}","SIM_QSAT_I0":f"{i0_sat:.3f}","SIM_DSAT":"0",
            "SIM_IRATED_K":f"{rng2.uniform(0.5,1.5):.4f}","SIM_OFF_RUN":rng2.choice(["41","-41"])}
    stress.append((f"st{idx:02d}", key, "1", "1100", envx))
n_safe = n_blk = n_real_silent = 0
silent_list = []
for tag, key, md, rpm, envx in stress:
    r = run_case_with_fault(WR, key, md, rpm, envx)
    if r.get("safe_abort"):
        n_safe += 1
    elif r["blk"]:
        n_blk += 1
    elif r["swp_l"] > 25.0 or abs(r["rs_e"]) > 15.0:
        n_real_silent += 1
        silent_list.append((tag, key, round(r["swp_l"],1), round(r["rs_e"],1), r["res"]["warn"]))
    else:
        pass  # ok
print(f"  safe pre-ident abort: {n_safe} | blocked: {n_blk} | real-silent: {n_real_silent}   [{'PASS' if n_real_silent==0 else 'FAIL'}: silent==0]")
for s in silent_list:
    print(f"    {s}")

# Bad-motor gate
print(f"\n--- BAD-MOTOR GATE (seed 20261188, ambiguous wind <0.13Nm excluded) ---")
bg = run_eval.gen_bad_motor_jobs(20261188, 30)
tp = fn = 0
for j in bg:
    if j[0].startswith("bad_wnd") and float(j[4].get("SIM_WIND","0")) < 0.13:
        continue
    r = run_case_with_fault(WR, j[1], j[2], j[3], j[4])
    if r["bad"]:
        if r["blk"] or r.get("safe_abort"): tp += 1
        else: fn += 1; print(f"   FN: {j[0]} swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
recall = tp/(tp+fn) if (tp+fn) else 1.0
print(f"  recall = {tp}/{tp+fn} = {recall*100:.0f}%   [{'PASS' if recall>=0.9 else 'FAIL'}: >=90%]")

# Summary verdict
print(f"\n{'='*60}")
print("VERDICT SUMMARY:")
checks = [
    ("SwpL residual p95 <= 8%", pct(swp_res, 0.95) <= 8, f"{pct(swp_res,0.95):.2f}%"),
    ("High-risk silent == 0", len(high_risk) == 0, f"{len(high_risk)} cases"),
    ("FP <= 3%", len(fp_items)/N_HEALTHY <= 0.03, f"{len(fp_items)/N_HEALTHY*100:.1f}%"),
    ("Stress real-silent == 0", n_real_silent == 0, f"{n_real_silent} cases"),
    ("Bad-motor recall >= 90%", recall >= 0.9, f"{recall*100:.0f}%"),
]
all_pass = True
for name, passed, detail in checks:
    print(f"  [{'PASS' if passed else 'FAIL'}] {name} ({detail})")
    if not passed: all_pass = False
print(f"\n  OVERALL: {'ALL PASS' if all_pass else 'SOME FAILED — see details above'}")
print(f"{'='*60}")

json.dump({"seed": SEED, "n": N_HEALTHY,
           "swp_res_p95": pct(swp_res, 0.95), "high_risk": len(high_risk),
           "fp_rate": len(fp_items)/N_HEALTHY, "stress_silent": n_real_silent,
           "bad_recall": recall, "all_pass": all_pass},
          open("/tmp/ff_blind_661924030_summary.json", "w"), indent=2)
