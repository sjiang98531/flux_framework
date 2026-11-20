#!/usr/bin/env python3
"""Frozen evaluation runner for parameter identification (PROTOCOL.md v1.0).

Supports evaluating any git ref (--ref <hash>) or custom FW source override
(--override-identify <path> --override-startup <path>) under the EXACT same
runtime Param_LqAt (with lsMaxA clamp) and fixed [0, I_run] metric.
"""
import argparse, concurrent.futures, json, math, os, random, re, shutil, subprocess, sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
SEEDS = json.load(open(os.path.join(SCRIPT_DIR, "seeds.json")))

ALL_CFGS = [
    "vac48", "vac16", "sm48", "sm120", "mid48", "mid16", "fan48", "fan", "bigL",
    "ipm2", "wrong_fan", "wrong_fan_psi_hi", "wrong_fan_psi_lo", "wrong_sm", "wrong_sm_psi_hi",
]

# --- 1. Runtime-exact Param_LqAt model (matches FW/Src/flux_obs.c:lsEff & param.c:Param_LqAt) ---
def l_true_sec(l0, ksat, i0sat, i):
    if ksat <= 0.0 or i <= i0sat:
        return l0
    return l0 * max(0.5, 1.0 - ksat * (i - i0sat))

def l_run_model(l0, per_a, knee_a, ls_max_a, i):
    if l0 <= 0.0:
        return 0.0
    x = min(i, ls_max_a) if ls_max_a > 0.0 else i
    if x <= knee_a or per_a <= 0.0:
        return l0
    m = max(0.5, 1.0 - per_a * (x - knee_a))
    return l0 * m

def max_l_err_runtime(lq_t, ksat, i0sat, lq_m, per_m, knee_m, ls_max_m, i_upper):
    if lq_m <= 0.0 or i_upper <= 0.0:
        return 100.0
    worst = 0.0
    for k in range(1, 201):
        i = i_upper * k / 200.0
        lt = l_true_sec(lq_t, ksat, i0sat, i)
        lm = l_run_model(lq_m, per_m, knee_m, ls_max_m, i)
        e = abs(lm - lt) / lt * 100.0
        if e > worst:
            worst = e
    return worst

def l_inc_ratio_plant(ksat, i0sat, i):
    if ksat <= 0.0 or i <= i0sat:
        return 1.0
    sc = 1.0 - ksat * (i - i0sat)
    r = (sc - ksat * i) if sc > 0.5 else 0.5
    return max(0.05, r)

def parse_kv(line):
    d = {}
    for p in line.split():
        if "=" in p:
            k, v = p.split("=", 1)
            d[k] = v
    return d

def pct(vals, q):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(len(s) - 1, int(q * (len(s) - 1)))]

# --- 2. Build helpers ---
def patch_runtime_tables(src_txt):
    """Allow runtime g_sim_irated_k in static const tables s_dtLevel / s_acSegA / s_acSegB."""
    m_dt = re.search(r"static const int16_t\s+s_dtLevel\[IDENT_DT_N\]\s*=\s*\{([^;]+)\};", src_txt, re.S)
    m_a = re.search(r"static const int16_t\s+s_acSegA\[AC_NSEG\]\s*=\s*\{([^;]+)\};", src_txt, re.S)
    m_b = re.search(r"static const int16_t\s+s_acSegB\[AC_NSEG\]\s*=\s*\{([^;]+)\};", src_txt, re.S)
    if not (m_dt and m_a and m_b):
        return src_txt
    init_fn = (
        "static int16_t s_acSegA[AC_NSEG], s_acSegB[AC_NSEG];\n"
        "static void simInitTables(void) {\n"
        f"    int16_t dt[IDENT_DT_N] = {{{m_dt.group(1)}}};\n"
        f"    int16_t sa[AC_NSEG] = {{{m_a.group(1)}}};\n"
        f"    int16_t sb[AC_NSEG] = {{{m_b.group(1)}}};\n"
        "    memcpy(s_dtLevel, dt, sizeof(dt));\n"
        "    memcpy(s_acSegA, sa, sizeof(sa));\n"
        "    memcpy(s_acSegB, sb, sizeof(sb));\n"
        "}\n"
    )
    src_txt = src_txt.replace(m_dt.group(0), "static int16_t s_dtLevel[IDENT_DT_N];\nstatic void simInitTables(void);")
    src_txt = src_txt.replace(m_a.group(0), init_fn)
    src_txt = src_txt.replace(m_b.group(0), "")
    # Inject simInitTables() at start of Ident_Reset
    m_rst = re.search(r"void Ident_Reset\(void\)\s*\{", src_txt)
    assert m_rst, "Ident_Reset not found"
    src_txt = src_txt[:m_rst.end()] + "\n    simInitTables();" + src_txt[m_rst.end():]
    return src_txt

def export_git_ref(ref, dst_dir):
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir)
    os.makedirs(dst_dir)
    p = subprocess.Popen(["git", "-C", REPO_ROOT, "archive", ref, "FW", "Board", "User"], stdout=subprocess.PIPE)
    subprocess.run(["tar", "-x", "-C", dst_dir], stdin=p.stdout, check=True)
    p.wait()
    if p.returncode != 0:
        raise RuntimeError(f"git archive failed for {ref}")

def build_suite(ref, work_dir, cfgs=None, override_identify=None, override_startup=None, override_identify_h=None):
    if cfgs is None:
        cfgs = ALL_CFGS
    if os.path.exists(work_dir):
        shutil.rmtree(work_dir)
    os.makedirs(work_dir)
    base_src = os.path.join(work_dir, "_base")
    if ref == "WORKTREE":
        for sub in ["FW", "Board", "User"]:
            shutil.copytree(os.path.join(REPO_ROOT, sub), os.path.join(base_src, sub))
    else:
        export_git_ref(ref, base_src)
    if override_identify:
        shutil.copy(override_identify, os.path.join(base_src, "FW", "Src", "identify.c"))
    if override_startup:
        shutil.copy(override_startup, os.path.join(base_src, "FW", "Src", "startup.c"))
    if override_identify_h:
        shutil.copy(override_identify_h, os.path.join(base_src, "FW", "Inc", "identify.h"))

    shim_dir = os.path.join(SCRIPT_DIR, "shim")
    plant_c = os.path.join(SCRIPT_DIR, "plant.c")
    sim_c = os.path.join(SCRIPT_DIR, "sim_ident_stress.c")

    def build_one(k):
        dst = os.path.join(work_dir, f"cfg_{k}")
        shutil.copytree(base_src, dst)
        cfg_ov = os.path.join(SCRIPT_DIR, "configs", k)
        for h in os.listdir(cfg_ov):
            shutil.copy(os.path.join(cfg_ov, h), os.path.join(dst, "FW", "Inc", h))
        id_path = os.path.join(dst, "FW", "Src", "identify.c")
        id_txt = open(id_path, encoding="utf-8", errors="ignore").read().replace("\r\n", "\n")
        if "g_sim_irated_k" in open(os.path.join(dst, "FW", "Inc", "cfg_motor.h")).read():
            id_txt = patch_runtime_tables(id_txt)
            ol_path = os.path.join(dst, "FW", "Src", "openloop.c")
            ol_txt = open(ol_path, encoding="utf-8", errors="ignore").read().replace("\r\n", "\n")
            ol_txt = ol_txt.replace("static volatile int32_t s_accStep = OPENLOOP_ACC_STEP_Q31;",
                                    "static volatile int32_t s_accStep = 0;")
            open(ol_path, "w", encoding="utf-8").write(ol_txt)
        open(id_path, "w", encoding="utf-8").write(id_txt)

        inc = (f"-I'{shim_dir}' -I'{SCRIPT_DIR}' -I'{dst}/FW/Inc' "
               f"-I'{dst}/User/Inc' -I'{dst}/Board/Inc' -I'{dst}/User/HeadFiles'")
        cf = (f"-O2 -g -fwrapv -Wall -Wno-unused-variable -Wno-unused-function "
              f"-Wno-misleading-indentation -Wno-discarded-qualifiers "
              f"-include '{shim_dir}/armcc_compat.h' -include stdio.h {inc} " + os.environ.get("IDENT_EVAL_CFLAGS", ""))
        objs = []
        srcs = [sim_c, plant_c] + [
            os.path.join(dst, "FW", "Src", f)
            for f in sorted(os.listdir(os.path.join(dst, "FW", "Src")))
            if f.endswith(".c") and f != "main.c"
        ]
        for s in srcs:
            o = os.path.join(work_dir, f"{k}_" + os.path.basename(s)[:-2] + ".o")
            extra = "-DIN_CFG_CHECK_C=1" if os.path.basename(s) == "diag.c" else ""
            subprocess.run(f"gcc {cf} {extra} -c '{s}' -o '{o}'", shell=True, check=True)
            objs.append(o)
        subprocess.run(["gcc"] + objs + ["-lm", "-o", os.path.join(work_dir, f"exe_{k}")], check=True)

    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 4)) as ex:
        list(ex.map(build_one, cfgs))

# --- 3. Execution & evaluation ---
def run_case(work_dir, key, md, rpm, envx, sim_s="35"):
    env = dict(os.environ)
    env.update(envx)
    r = subprocess.run([os.path.join(work_dir, f"exe_{key}"), sim_s, md, rpm],
                       env=env, capture_output=True, text=True)
    lines = [l for l in r.stdout.strip().splitlines() if l.startswith("TRUE ") or l.startswith("RES ")]
    if len(lines) < 2:
        return {"err": f"NO_OUT rc={r.returncode}", "bad": True, "blk": False, "run_l": 100.0, "swp_l": 100.0, "rs_e": -100.0}
    t = parse_kv(lines[0])
    res = parse_kv(lines[1])
    warn = int(res["warn"], 16)
    blk = bool(warn & 0x10)
    state = int(res["state"])
    src = int(res["src"])

    rs_t = float(t["rs"]); ld_t = float(t["ld"]); lq_t = float(t["lq"])
    ksat = float(t["ksat"]); i0sat = float(t["i0sat"])
    i_run = float(t["irun"]); i_swp = float(t["imax"])

    rs_m = float(res["rs"]); ld_m = float(res["ld"]); lq_m = float(res["lq"])
    rs_e = (rs_m - rs_t) / rs_t * 100.0 if rs_m > 0 else -100.0
    ld_e = (ld_m - ld_t) / ld_t * 100.0 if ld_m > 0 else -100.0
    lq_e = (lq_m - lq_t) / lq_t * 100.0 if lq_m > 0 else -100.0

    # Use committed runtime gParam when unblocked & applied; else raw gIdent for TP/FP audit
    if (not blk) and src == 2:
        m_lq = float(res["pLq"])
        m_per = float(res["pPerA"])
        m_knee = float(res["pKnee"])
        m_lsmax = float(res["pLsMax"])
    else:
        m_lq = lq_m
        m_per = float(res["perA"])
        m_knee = float(res["knee"])
        m_lsmax = float(res["idMaxA"])

    run_l = max_l_err_runtime(lq_t, ksat, i0sat, m_lq, m_per, m_knee, m_lsmax, i_run)
    swp_l = max_l_err_runtime(lq_t, ksat, i0sat, m_lq, m_per, m_knee, m_lsmax, i_swp)
    # Protocol v2 bad definition: SwpL (in-sweep accuracy) + Rs + state; RunL (overload extrapolation) reported separately
    bad = (state != 2) or (lq_m <= 0.0) or (swp_l > 15.0) or (abs(rs_e) > 15.0)
    linc_run = l_inc_ratio_plant(ksat, i0sat, i_run)
    return {
        "t": t, "res": res, "warn": warn, "blk": blk, "bad": bad,
        "rs_e": rs_e, "ld_e": ld_e, "lq_e": lq_e,
        "run_l": run_l, "swp_l": swp_l, "linc_run": linc_run,
    }

def gen_random_jobs(seed, count):
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
        envx = {
            "SIM_PWMDLY": "1", "SIM_NOISE": str(rng.choice([2, 4])), "SIM_SEED": str(seed + idx * 97),
            "SIM_RS": f"{r_nom * r_mult:.6g}", "SIM_LD": f"{L:.6g}", "SIM_LQ": f"{L * sal:.6g}",
            "SIM_TD": f"{td_nom * td_mult:.6g}", "SIM_DTI0": f"{dti0 * rng.uniform(0.7, 1.3):.4f}",
            "SIM_QSAT": f"{ksat:.5f}", "SIM_QSAT_I0": f"{i0_sat:.3f}", "SIM_DSAT": "0",
            "SIM_IRATED_K": f"{ir_mult:.4f}",
        }
        jobs.append((f"s{seed}_{idx:03d}", key, "1", "1100", envx))
    return jobs

def gen_bad_motor_jobs(seed, count):
    """Frozen 'should-block' stress batch ensuring TP + FN >= 10."""
    rng = random.Random(seed)
    jobs = []
    for idx in range(count):
        mode = idx % 5
        if mode == 0:
            # Ultra-low L / dead-time swamped
            L = rng.uniform(1.5e-6, 5.0e-6)
            envx = {"SIM_PWMDLY":"1","SIM_NOISE":"4","SIM_SEED":str(seed+idx*31),
                    "SIM_RS":f"{rng.uniform(0.08,0.20):.4f}","SIM_LD":f"{L:.6g}","SIM_LQ":f"{L*1.1:.6g}",
                    "SIM_TD":f"{rng.uniform(1.5e-6,2.8e-6):.6g}","SIM_DTI0":"0.5","SIM_DSAT":"0"}
            jobs.append((f"bad_snr_{idx:02d}", "vac16", "1", "5000", envx))
        elif mode == 1:
            # Extreme deep saturation (l_inc collapses)
            ksat = rng.uniform(0.32, 0.55)
            envx = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_SEED":str(seed+idx*31),
                    "SIM_QSAT":f"{ksat:.4f}","SIM_QSAT_I0":"0.0","SIM_DSAT":"0"}
            jobs.append((f"bad_sat_{idx:02d}", "fan", "1", "1100", envx))
        elif mode == 2:
            # Wind torque disturbance during static ident
            w = rng.uniform(0.025, 0.20)
            envx = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_SEED":str(seed+idx*31),
                    "SIM_WIND":f"{w:.4f}","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}
            jobs.append((f"bad_wnd_{idx:02d}", "fan", "1", "1100", envx))
        elif mode == 3:
            # Severe low Vbus voltage collapse
            vb = rng.uniform(25.0, 50.0)
            envx = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_SEED":str(seed+idx*31),
                    "SIM_VBUS":f"{vb:.1f}","SIM_QSAT":"0.12","SIM_QSAT_I0":"0.0"}
            jobs.append((f"bad_vbu_{idx:02d}", "fan", "1", "1100", envx))
        else:
            # Grossly undersized I_rated on saturating motor (0.20x..0.35x -> can't see overload saturation)
            ir_k = rng.uniform(0.20, 0.35)
            envx = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_SEED":str(seed+idx*31),
                    "SIM_IRATED_K":f"{ir_k:.3f}","SIM_QSAT":"0.18","SIM_QSAT_I0":"0.6","SIM_DSAT":"0"}
            jobs.append((f"bad_ira_{idx:02d}", "fan", "1", "1100", envx))
    return jobs

def run_batch(work_dir, jobs):
    def _one(j):
        tag, key, md, rpm, envx = j
        return tag, key, run_case(work_dir, key, md, rpm, envx)
    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, os.cpu_count() or 4)) as ex:
        return list(ex.map(_one, jobs))

def summarize_batch(results):
    tp = fp = tn = fn = 0
    unb_run, unb_swp = [], []
    fn_items, fp_items = [], []
    for tag, key, r in results:
        bad, blk = r["bad"], r["blk"]
        if bad and blk: tp += 1
        elif (not bad) and blk: fp += 1; fp_items.append((tag, key, r["run_l"], r["rs_e"], hex(r.get("warn", 0))))
        elif (not bad) and (not blk): tn += 1
        else: fn += 1; fn_items.append((tag, key, r["run_l"], r["swp_l"], r["rs_e"], hex(r.get("warn", 0))))
        if not blk:
            unb_run.append(r["run_l"])
            unb_swp.append(r["swp_l"])
    n = len(results)
    return {
        "n": n, "tp": tp, "fp": fp, "tn": tn, "fn": fn,
        "fp_rate": fp / n * 100.0 if n else 0.0,
        "unb_run_p50": pct(unb_run, 0.50), "unb_run_p95": pct(unb_run, 0.95), "unb_run_max": max(unb_run) if unb_run else 0.0,
        "unb_swp_p95": pct(unb_swp, 0.95), "unb_swp_max": max(unb_swp) if unb_swp else 0.0,
        "fn_items": fn_items, "fp_items": fp_items,
    }

def run_full_eval(work_dir, label, sections=None):
    if sections is None:
        sections = {"named", "old6", "offgn", "hard1119", "perturb", "align_grid", "bad_gate"}
    out = {"label": label}

    if "named" in sections:
        named_jobs = [
            ("fan_auto",          "fan", "0", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("fan_dly0",          "fan", "1", "1100", {"SIM_PWMDLY":"0","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("fan_dly2",          "fan", "1", "1100", {"SIM_PWMDLY":"2","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("sm120_sat",         "sm120", "1", "5000", {"SIM_PWMDLY":"1","SIM_NOISE":"4","SIM_TD":"1.5e-6","SIM_LQ":"180e-6","SIM_QSAT":"0.025","SIM_QSAT_I0":"2.0","SIM_DSAT":"0"}),
            ("sm120_dly2",        "sm120", "1", "5000", {"SIM_PWMDLY":"2","SIM_NOISE":"4","SIM_TD":"1.5e-6","SIM_LQ":"180e-6","SIM_QSAT":"0.025","SIM_QSAT_I0":"2.0","SIM_DSAT":"0"}),
            ("vac48_sat",         "vac48", "1", "5000", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_TD":"0.5e-6","SIM_DTI0":"0.5","SIM_QSAT":"0.015","SIM_QSAT_I0":"5.0","SIM_DSAT":"0"}),
            ("vac16_sat",         "vac16", "1", "5000", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_TD":"0.82e-6","SIM_DTI0":"0.5","SIM_QSAT":"0.015","SIM_QSAT_I0":"5.0","SIM_DSAT":"0"}),
            ("mid48_auto",        "mid48", "0", "3000", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LQ":"0.0040","SIM_QSAT":"0.05","SIM_QSAT_I0":"1.0","SIM_DSAT":"0"}),
            ("bigL_sat",          "bigL", "1", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("ipm2_sat",          "ipm2", "1", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_QSAT":"0.060","SIM_QSAT_I0":"0.5"}),
            ("wrong_fan_psi_hi",  "wrong_fan_psi_hi", "0", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("wrong_fan_psi_lo",  "wrong_fan_psi_lo", "0", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("wrong_sm_sat",      "wrong_sm", "1", "5000", {"SIM_PWMDLY":"1","SIM_NOISE":"4","SIM_TD":"1.5e-6","SIM_RS":"0.12","SIM_LD":"120e-6","SIM_LQ":"180e-6","SIM_QSAT":"0.025","SIM_QSAT_I0":"2.0","SIM_DSAT":"0"}),
            ("wind_fan_002",      "fan", "1", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_WIND":"0.02","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("wind_fan_015",      "fan", "1", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_WIND":"0.15","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}),
            ("low_l_phase_vac16", "vac16", "1", "5000", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_RS":"0.18","SIM_LD":"15e-6","SIM_LQ":"15e-6","SIM_TD":"0.5e-6","SIM_DTI0":"0.5","SIM_DSAT":"0"}),
        ]
        res = run_batch(work_dir, named_jobs)
        out["named"] = {tag: {"warn": hex(r["warn"]), "blk": r["blk"], "rs_e": round(r["rs_e"], 2),
                              "ld_e": round(r["ld_e"], 2), "lq_e": round(r["lq_e"], 2),
                              "run_l": round(r["run_l"], 2), "swp_l": round(r["swp_l"], 2)}
                        for tag, _, r in res}

    if "old6" in sections:
        all18 = gen_random_jobs(SEEDS["old_fn_seed_20261118"]["seed"], SEEDS["old_fn_seed_20261118"]["count"])
        idxs = SEEDS["old_fn_seed_20261118"]["indices"]
        jobs = [all18[i] for i in idxs]
        res = run_batch(work_dir, jobs)
        out["old6"] = {tag: {"key": k, "warn": hex(r["warn"]), "blk": r["blk"], "bad": r["bad"],
                             "rs_e": round(r["rs_e"], 2), "run_l": round(r["run_l"], 2), "swp_l": round(r["swp_l"], 2)}
                       for tag, k, r in res}

    if "offgn" in sections:
        rng = random.Random(SEEDS["vac48_offgn_30x"]["seed"])
        jobs = []
        for i in range(SEEDS["vac48_offgn_30x"]["count"]):
            envx = {
                "SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_TD":"0.5e-6","SIM_DTI0":"0.5",
                "SIM_QSAT":"0.015","SIM_QSAT_I0":"5.0","SIM_DSAT":"0",
                "SIM_SEED": str(1000 + i * 37),
                "SIM_OFF": str(rng.randint(-25, 25)),
                "SIM_IGAIN": f"{rng.uniform(0.88, 1.12):.3f}",
            }
            jobs.append((f"offgn_{i:02d}", "vac48", "1", "5000", envx))
        out["offgn"] = summarize_batch(run_batch(work_dir, jobs))

    if "hard1119" in sections:
        jobs = gen_random_jobs(SEEDS["hard_seed_20261119"]["seed"], SEEDS["hard_seed_20261119"]["count"])
        out["hard1119"] = summarize_batch(run_batch(work_dir, jobs))

    if "perturb" in sections:
        base_fan = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}
        base_sm  = {"SIM_PWMDLY":"1","SIM_NOISE":"4","SIM_TD":"1.5e-6","SIM_LQ":"180e-6","SIM_QSAT":"0.025","SIM_QSAT_I0":"2.0","SIM_DSAT":"0"}
        p_jobs = [
            ("igain_-5_fan",  "fan",   "1", "1100", {**base_fan, "SIM_IGAIN":"0.95"}),
            ("igain_+5_fan",  "fan",   "1", "1100", {**base_fan, "SIM_IGAIN":"1.05"}),
            ("igain_-5_sm",   "sm120", "1", "5000", {**base_sm,  "SIM_IGAIN":"0.95"}),
            ("igain_+5_sm",   "sm120", "1", "5000", {**base_sm,  "SIM_IGAIN":"1.05"}),
            ("vbus_-5_fan",   "fan",   "1", "1100", {**base_fan, "SIM_VBUS_GAIN":"0.95"}),
            ("vbus_+5_fan",   "fan",   "1", "1100", {**base_fan, "SIM_VBUS_GAIN":"1.05"}),
            ("offrun_+2_fan", "fan",   "1", "1100", {**base_fan, "SIM_OFF_RUN":"41"}),
            ("offrun_-2_fan", "fan",   "1", "1100", {**base_fan, "SIM_OFF_RUN":"-41"}),
            ("offrun_+2_sm",  "sm120", "1", "5000", {**base_sm,  "SIM_OFF_RUN":"41"}),
            ("td_-30_fan",    "fan",   "1", "1100", {**base_fan, "SIM_TD":"0.574e-6"}),
            ("td_+30_fan",    "fan",   "1", "1100", {**base_fan, "SIM_TD":"1.066e-6"}),
            ("td_-30_sm",     "sm120", "1", "5000", {**base_sm,  "SIM_TD":"1.05e-6"}),
            ("td_+30_sm",     "sm120", "1", "5000", {**base_sm,  "SIM_TD":"1.95e-6"}),
            ("rdrift_+15_fan","fan",   "1", "1100", {**base_fan, "SIM_R_DRIFT":"0.15"}),
            ("rdrift_+15_sm", "sm120", "1", "5000", {**base_sm,  "SIM_R_DRIFT":"0.15"}),
            ("cog_fric_fan",  "fan",   "1", "1100", {**base_fan, "SIM_COGGING":"0.02","SIM_FRIC":"0.03"}),
            ("cog_fric_ipm",  "ipm2",  "1", "1100", {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_QSAT":"0.060","SIM_QSAT_I0":"0.5","SIM_COGGING":"0.02","SIM_FRIC":"0.03"}),
        ]
        res = run_batch(work_dir, p_jobs)
        p_out = {}
        for tag, _, r in res:
            if tag.startswith(("igain_", "vbus_")):
                ok = (not r["blk"]) and abs(r["rs_e"]) <= 11.0 and abs(r["lq_e"]) <= 11.0 and r["run_l"] <= 15.0
            elif tag.startswith("rdrift_"):
                ok = (not r["blk"]) and r["run_l"] <= 12.0
            else:
                ok = (not r["blk"]) and abs(r["rs_e"]) <= 4.0 and r["run_l"] <= 12.0
            p_out[tag] = {"ok": ok, "blk": r["blk"], "warn": hex(r["warn"]),
                          "rs_e": round(r["rs_e"], 2), "lq_e": round(r["lq_e"], 2), "run_l": round(r["run_l"], 2)}
        out["perturb"] = p_out

    if "align_grid" in sections:
        base_fan = {"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_LD":"0.0293","SIM_QSAT":"0.094","SIM_QSAT_I0":"0.0"}
        g_jobs = []
        for kj in [0.1, 0.3, 1.0, 3.0, 10.0]:
            for kpsi in [0.5, 1.0, 2.0]:
                g_jobs.append((f"J{kj}_P{kpsi}", "fan", "1", "1100", {**base_fan, "SIM_KJ":str(kj), "SIM_KPSI":str(kpsi)}))
        res = run_batch(work_dir, g_jobs)
        g_out = {}
        for tag, _, r in res:
            ok = (not r["blk"]) and (not (r["warn"] & 0x200)) and abs(r["rs_e"]) <= 4.0 and r["run_l"] <= 12.0
            g_out[tag] = {"ok": ok, "blk": r["blk"], "warn": hex(r["warn"]),
                          "rs_e": round(r["rs_e"], 2), "run_l": round(r["run_l"], 2)}
        out["align_grid"] = g_out

    if "bad_gate" in sections:
        bad_jobs = gen_bad_motor_jobs(SEEDS["bad_motor_gate_seed"]["seed"], SEEDS["bad_motor_gate_seed"]["count"])
        # v2.1: wind torque in [0.08, 0.13) Nm is ground-truth ambiguous on the fan plant (rotor may not move;
        # measured results often accurate). Excluded from the confusion matrix (counted separately as ambiguous).
        results = []
        n_amb = 0
        for j in bad_jobs:
            if j[0].startswith("bad_wnd"):
                w = float(j[4].get("SIM_WIND", "0"))
                if w < 0.13:
                    n_amb += 1
                    continue
            results.append(j)
        s = summarize_batch(run_batch(work_dir, results))
        s["n_ambiguous_wind"] = n_amb
        out["bad_gate"] = s

    return out

def print_summary(out):
    print(f"\n================ {out['label']} ================")
    if "named" in out:
        print("--- Named (16) ---")
        for k, v in out["named"].items():
            print(f"  {k:<18} warn={v['warn']:<6} blk={int(v['blk'])} Rs={v['rs_e']:+6.2f}% Ld={v['ld_e']:+6.2f}% Lq={v['lq_e']:+6.2f}% RunL={v['run_l']:5.2f}% SwpL={v['swp_l']:5.2f}%")
    if "old6" in out:
        print("--- Old 6 FNs (20261118) ---")
        for k, v in out["old6"].items():
            print(f"  {k:<12} ({v['key']:<5}) warn={v['warn']:<6} blk={int(v['blk'])} bad={int(v['bad'])} Rs={v['rs_e']:+6.2f}% RunL={v['run_l']:5.2f}% SwpL={v['swp_l']:5.2f}%")
    for sec, title in [("offgn", "vac48_offgn_30x"), ("hard1119", "Hard Seed 20261119 (N=120)"), ("bad_gate", "Bad-Motor Gate Set (N=30)")]:
        if sec in out:
            s = out[sec]
            print(f"--- {title} ---")
            print(f"  TP={s['tp']} FP={s['fp']} TN={s['tn']} FN={s['fn']} (FP={s['fp_rate']:.1f}%) | unb RunL p50={s['unb_run_p50']:.2f}% p95={s['unb_run_p95']:.2f}% max={s['unb_run_max']:.2f}% | SwpL p95={s['unb_swp_p95']:.2f}% max={s['unb_swp_max']:.2f}%")
            if s["fn_items"]:
                print("  FN:", s["fn_items"][:6])
            if s["fp_items"]:
                print("  FP:", s["fp_items"][:6])
    if "perturb" in out:
        p = out["perturb"]
        n_ok = sum(1 for v in p.values() if v["ok"])
        print(f"--- Perturbation Suite ({n_ok}/{len(p)} PASS) ---")
        for k, v in p.items():
            mark = "PASS" if v["ok"] else "FAIL"
            print(f"  [{mark}] {k:<15} warn={v['warn']:<6} blk={int(v['blk'])} Rs={v['rs_e']:+6.2f}% Lq={v['lq_e']:+6.2f}% RunL={v['run_l']:5.2f}%")
    if "align_grid" in out:
        g = out["align_grid"]
        n_ok = sum(1 for v in g.values() if v["ok"])
        max_rs = max(abs(v["rs_e"]) for v in g.values())
        max_l = max(v["run_l"] for v in g.values())
        print(f"--- Align Damping Grid J(0.1..10) x Psi(0.5..2) ({n_ok}/{len(g)} PASS, max|Rs|={max_rs:.2f}%, maxRunL={max_l:.2f}%) ---")
        for k, v in g.items():
            if not v["ok"]:
                print(f"  [FAIL] {k:<10} warn={v['warn']:<6} blk={int(v['blk'])} Rs={v['rs_e']:+6.2f}% RunL={v['run_l']:5.2f}%")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--label", default=None)
    ap.add_argument("--work", required=True)
    ap.add_argument("--sections", default=None)
    ap.add_argument("--override-identify", default=None)
    ap.add_argument("--override-startup", default=None)
    ap.add_argument("--json-out", default=None)
    args = ap.parse_args()
    secs = set(args.sections.split(",")) if args.sections else None
    build_suite(args.ref, args.work, override_identify=args.override_identify, override_startup=args.override_startup)
    res = run_full_eval(args.work, args.label or args.ref, sections=secs)
    print_summary(res)
    if args.json_out:
        open(args.json_out, "w").write(json.dumps(res, indent=2))
