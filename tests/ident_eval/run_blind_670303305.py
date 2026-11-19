#!/usr/bin/env python3
"""BLIND TEST RUNNER - Protocol v2.1, frozen candidate, seed 670303305 (one-shot).

Acceptance lines (written BEFORE running, PROTOCOL_V2.md section 7):
- N=300 random healthy motors + combined per-motor perturbations (igain U[0.95,1.05],
  off_run ±0.5%FS main (±2%FS separate stress subset), vbus U[0.95,1.05], td U[0.7,1.3], r_drift U[0,0.15]).
- Main metric: SwpL residual AFTER subtracting known sensor scaling effect: p95 <= 8%.
- High-risk silent failure (SwpL>25% unblocked): must be 0.
- Bad-motor gate recall (ambiguous wind <0.13 Nm excluded): >= 90%.
- Healthy false-block FP <= 3% (stress-subset blocks don't count).
"""
import json, math, os, random, sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
sys.path.insert(0, SCRIPT_DIR)
import run_eval

SEED = 670303305
N_HEALTHY = 300
WR = "/tmp/ff_blind_670303305"

run_eval.build_suite("WORKTREE", WR)

def gen_healthy_jobs_blind(seed, count):
    """Same motor distribution as frozen gen_random_jobs + per-motor combined sensor perturbations."""
    rng = random.Random(seed)
    jobs = []
    for idx in range(count):
        u = rng.random()
        L = 15e-6 * ((120e-3 / 15e-6) ** u)
        pwm48 = rng.choice([False, True])
        if L < 60e-6:
            key = "vac48" if pwm48 else "vac16"
            r_nom = 0.073 * ((L / 15.8e-6) ** 0.5); td_nom = 0.35e-6 if pwm48 else 0.82e-6; dti0 = 0.50; i0_sat = 5.0
        elif L < 800e-6:
            key = "sm48" if pwm48 else "sm120"
            r_nom = 0.12 * ((L / 150e-6) ** 0.5); td_nom = 0.40e-6 if pwm48 else 1.20e-6; dti0 = 0.40; i0_sat = 2.0
        elif L < 12e-3:
            key = "mid48" if pwm48 else "mid16"
            r_nom = 1.20 * ((L / 3e-3) ** 0.6); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.30; i0_sat = 1.0
        else:
            key = "fan48" if pwm48 else ("bigL" if L > 65e-3 else "fan")
            r_nom = 7.35 * ((L / 43.3e-3) ** 0.5); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.23; i0_sat = 0.5
        r_mult = rng.uniform(0.5, 1.5); td_mult = rng.uniform(0.5, 1.5); ir_mult = rng.uniform(0.5, 1.5)
        sal = rng.uniform(1.0, 1.45); ksat = rng.uniform(0.0, 0.03) if L < 1e-3 else rng.uniform(0.0, 0.09)
        igain = rng.uniform(0.95, 1.05)
        vbusg = rng.uniform(0.95, 1.05)
        tdm = rng.uniform(0.70, 1.30)
        rdrift = rng.uniform(0.0, 0.15)
        offrun = rng.choice([-10, -5, 0, 5, 10])  # ±0.5%FS = ±10 LSB main set
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

def sensor_scale_effect(meta):
    """Known Z=V/I scaling: L and R read high by vbusg/igain. Rs additionally by (1+rdrift) since plant drifts up.
    L(i) shape unaffected except through rs? No: L error = vbusg/igain - 1. Rs error = vbusg/igain*(1+rdrift) - 1 (approx)."""
    z = meta["vbusg"] / meta["igain"]
    return z

jobs = gen_healthy_jobs_blind(SEED, N_HEALTHY)
results = []
for j in jobs:
    tag, key, md, rpm, envx, meta = j
    r = run_eval.run_case(WR, key, md, rpm, envx)
    z = sensor_scale_effect(meta)
    # Residual SwpL after removing known scaling: model L scaled by z -> error from scaling alone ~ (z-1) on L.
    # Lq_err% = z-1 approx; SwpL max-err is bounded below by |z-1|. Subtract that component (worst-case linear).
    r["swp_resid"] = max(0.0, r["swp_l"] - abs(z - 1.0) * 100.0)
    r["lq_resid_e"] = r["lq_e"] - (z - 1.0) * 100.0
    r["rs_resid_e"] = r["rs_e"] - ((z * (1.0 + meta["rdrift"])) - 1.0) * 100.0
    results.append((tag, key, r))

unb = [r for _, _, r in results if not r["blk"]]
blk = [r for _, _, r in results if r["blk"]]
swp_raw = sorted(r["swp_l"] for r in unb)
swp_res = sorted(r["swp_resid"] for r in unb)
rs_res = sorted(abs(r["rs_resid_e"]) for r in unb)
high_risk = [(t, k, r) for t, k, r in results if (not r["blk"]) and r["swp_l"] > 25.0]
bad_unblk = [(t, k, r) for t, k, r in results if (not r["blk"]) and r["bad"]]
fp_items = [(t, k, r) for t, k, r in results if r["blk"] and (not r["bad"])]

def pct(v, q):
    return v[min(len(v) - 1, int(q * (len(v) - 1)))] if v else 0.0

print(f"=== BLIND HEALTHY N={N_HEALTHY} seed={SEED} (combined perturbations) ===")
print(f"  blocked: {len(blk)} | unblocked: {len(unb)}")
print(f"  SwpL raw        : p50={pct(swp_raw,0.5):.2f}% p95={pct(swp_raw,0.95):.2f}% max={swp_raw[-1]:.2f}%")
print(f"  SwpL residual   : p50={pct(swp_res,0.5):.2f}% p95={pct(swp_res,0.95):.2f}% max={swp_res[-1]:.2f}%   [ACCEPT: p95<=8%]")
print(f"  |Rs| residual   : p50={pct(rs_res,0.5):.2f}% p95={pct(rs_res,0.95):.2f}% max={rs_res[-1]:.2f}%")
print(f"  High-risk silent (SwpL>25% unblocked): {len(high_risk)}   [ACCEPT: ==0]")
for t, k, r in high_risk[:10]:
    print(f"    {t} ({k}): swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
print(f"  Bad-unblocked (SwpL>15 or |Rs|>15 or state): {len(bad_unblk)}   [report only]")
for t, k, r in bad_unblk[:12]:
    print(f"    {t} ({k}): swp={r['swp_l']:.1f}% rs={r['rs_e']:+.1f}% warn={r['res']['warn']}")
print(f"  FP (blocked healthy): {len(fp_items)} = {len(fp_items)/N_HEALTHY*100:.1f}%   [ACCEPT: <=3%]")
for t, k, r in fp_items[:10]:
    print(f"    {t} ({k}): warn={r['res']['warn']} swp={r['swp_l']:.1f}%")

# Stress subset: off_run ±41 LSB (±2%FS), 30 samples
print("\n=== STRESS SUBSET off_run=±2%FS (N=30, only no-silent-failure required) ===")
rng2 = random.Random(SEED + 1)
stress_jobs = []
for idx in range(30):
    u = rng2.random()
    L = 15e-6 * ((120e-3 / 15e-6) ** u)
    pwm48 = rng2.choice([False, True])
    if L < 60e-6:
        key = "vac48" if pwm48 else "vac16"; r_nom = 0.073 * ((L / 15.8e-6) ** 0.5); td_nom = 0.35e-6 if pwm48 else 0.82e-6; dti0 = 0.50; i0_sat = 5.0
    elif L < 800e-6:
        key = "sm48" if pwm48 else "sm120"; r_nom = 0.12 * ((L / 150e-6) ** 0.5); td_nom = 0.40e-6 if pwm48 else 1.20e-6; dti0 = 0.40; i0_sat = 2.0
    elif L < 12e-3:
        key = "mid48" if pwm48 else "mid16"; r_nom = 1.20 * ((L / 3e-3) ** 0.6); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.30; i0_sat = 1.0
    else:
        key = "fan48" if pwm48 else ("bigL" if L > 65e-3 else "fan"); r_nom = 7.35 * ((L / 43.3e-3) ** 0.5); td_nom = 0.40e-6 if pwm48 else 0.82e-6; dti0 = 0.23; i0_sat = 0.5
    sal = rng2.uniform(1.0, 1.45); ksat = rng2.uniform(0.0, 0.03) if L < 1e-3 else rng2.uniform(0.0, 0.09)
    envx = {"SIM_PWMDLY":"1","SIM_NOISE":str(rng2.choice([2,4])),"SIM_SEED":str(SEED+1000+idx*97),
            "SIM_RS":f"{r_nom*rng2.uniform(0.5,1.5):.6g}","SIM_LD":f"{L:.6g}","SIM_LQ":f"{L*sal:.6g}",
            "SIM_TD":f"{td_nom*rng2.uniform(0.5,1.5):.6g}","SIM_DTI0":f"{dti0*rng2.uniform(0.7,1.3):.4f}",
            "SIM_QSAT":f"{ksat:.5f}","SIM_QSAT_I0":f"{i0_sat:.3f}","SIM_DSAT":"0",
            "SIM_IRATED_K":f"{rng2.uniform(0.5,1.5):.4f}","SIM_OFF_RUN":rng2.choice(["41","-41"])}
    stress_jobs.append((f"st{idx:02d}", key, "1", "1100", envx))
n_blk = n_silent = 0
silent_list = []
for tag, key, md, rpm, envx in stress_jobs:
    r = run_eval.run_case(WR, key, md, rpm, envx)
    if r["blk"]:
        n_blk += 1
    elif r["swp_l"] > 25.0 or abs(r["rs_e"]) > 15.0:
        n_silent += 1
        silent_list.append((tag, key, round(r["swp_l"], 1), round(r["rs_e"], 1), r["res"]["warn"]))
print(f"  blocked: {n_blk}/30 | silent-fail: {n_silent}   [ACCEPT: silent-fail == 0; blocks OK]")
for s in silent_list:
    print("   ", s)

# Bad-motor gate (frozen seed 20261188, ambiguous wind excluded per v2.1)
print("\n=== BAD-MOTOR GATE (seed 20261188, ambiguous wind <0.13Nm excluded) ===")
bg = run_eval.gen_bad_motor_jobs(20261188, 30)
tp = fn = 0
for j in bg:
    if j[0].startswith("bad_wnd") and float(j[4].get("SIM_WIND", "0")) < 0.13:
        continue
    r = run_eval.run_case(WR, j[1], j[2], j[3], j[4])
    if r["bad"]:
        if r["blk"]: tp += 1
        else: fn += 1; print("   FN:", j[0], round(r['swp_l'],2), round(r['rs_e'],2), r['res']['warn'])
recall = tp / (tp + fn) if (tp + fn) else 1.0
print(f"  recall = {tp}/{tp+fn} = {recall*100:.0f}%   [ACCEPT: >=90%]")

json.dump({"seed": SEED, "n": N_HEALTHY,
           "swp_res_p95": pct(swp_res, 0.95), "swp_res_max": swp_res[-1] if swp_res else 0,
           "high_risk": len(high_risk), "fp_rate": len(fp_items) / N_HEALTHY,
           "stress_silent": n_silent, "bad_recall": recall},
          open("/tmp/ff_blind_670303305_summary.json", "w"), indent=2)
