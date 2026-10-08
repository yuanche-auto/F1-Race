"""Reproduce the L3 results (stochastic disturbances, stochastic MPC with calibrated tightening).

Needs the L1 plans (scripts/run_l1.py) and the 12 % plan of L2 (scripts/run_l2.py,
or it is solved here).

    python scripts/run_l3.py              # calibration + Monte Carlo, ~2 h on 7 processes
    python scripts/run_l3.py calibrate    # only the calibration laps (results/L3/calibration.json)
    python scripts/run_l3.py iterate      # second calibration round on the SMPCs' own laps
    python scripts/run_l3.py report       # tables / figures from results/L3/runs.json
"""
from __future__ import annotations

import json
import os
import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.experiments.l3 import L3Config, calibrate, config_dict, paired_test, run_jobs, summarize  # noqa: E402

OUT = ROOT / "results" / "L3"
UNIFORM = (0.95, 0.93, 0.88)
UNIFORM_TRACK = ((0.95, 0.5), (0.89, 0.8))           # (eta, extra track margin [m])
ABLATION_EPS = 0.01
ROBUST_EPS = (0.05, 0.01)       # calibration robustness checks (truncated, iterated)
TRUNCATE_S = 2.0                # predictions ignored before a failed calibration lap ends
PAIRS = [("smpc_0.05", "nmpc_0.95"), ("smpc_0.05", "nmpc_0.89_m1.1"), ("smpc_0.01", "nmpc_0.89_m1.1"),
         ("smpc_0.01", "nmpc_0.88"), ("smpc_0.01", "smpc_flat"), ("smpc_0.01", "smpc_mean"),
         ("smpc_0.01", "smpc_fric_only"), ("smpc_0.01", "smpc_track_only"), ("nmpc_0.95", "tracker"),
         ("smpc_0.05_trunc", "smpc_0.05"), ("smpc_0.01_trunc", "smpc_0.01"),
         ("smpc_0.05_iter", "smpc_0.05"), ("smpc_0.01_iter", "smpc_0.01"),
         ("smpc_0.01_otherenv", "smpc_0.01")]


def plans_for(keys) -> dict:
    """Plans keyed by (eta, track margin): the L1 plans and the 12 % plan of L2 (margin
    0.3 m); missing ones are solved from the ideal L1 plan and cached in
    results/L3/plans.pkl."""
    from f1race.dynamics import DynVehicle
    from f1race.planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
    from f1race.track import Track

    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    plans = {(eta, 0.3): o for eta, o in cache["plans"].items()}
    l2 = ROOT / "results" / "L2" / "plan_088.pkl"
    if l2.exists():
        plans[(0.88, 0.3)] = pickle.load(open(l2, "rb"))
    path = OUT / "plans.pkl"
    extra = pickle.load(open(path, "rb")) if path.exists() else {}
    extra = {(k if isinstance(k, tuple) else (k, 0.3)): v for k, v in extra.items()}
    plans.update(extra)
    for key in keys:
        if key in plans:
            continue
        eta, margin = key
        o = solve_min_time_dynamic(Track.load("Spielberg"), DynVehicle(),
                                   initial_guess_from_dynamic(plans[(1.0, 0.3)]), eta=eta, margin=margin)
        if not o.success:
            raise RuntimeError(f"plan for eta = {eta}, margin = {margin} failed: {o.status}")
        plans[key] = extra[key] = o
        OUT.mkdir(parents=True, exist_ok=True)
        pickle.dump(extra, open(path, "wb"))
    return plans


def plan_eta_for(eta_k) -> float:
    """Reference plan of an SMPC: the margin of the end of the horizon, rounded down
    to 0.01."""
    return float(np.floor(min(eta_k) * 100.0) / 100.0)


def run_calibration(cfg: L3Config) -> dict:
    plans = plans_for([(0.95, 0.3)])
    spec = {"kind": "nmpc", "plan_eta": 0.95, "eta": 0.95}
    c = config_dict(cfg)
    res = run_jobs([("nmpc_0.95", spec, s, plans, c, True) for s in cfg.calib_seeds], cfg.workers)
    samples = [r.pop("samples") for r in res]
    save_samples(samples, "ce")
    cal = {"runs": res, "smpc": [calibrate(samples, cfg, e) for e in cfg.eps],
           "smpc_trunc": [calibrate(samples, cfg, e, TRUNCATE_S) for e in ROBUST_EPS]}
    stats = {}
    k = np.concatenate([s["k"] for s in samples])
    up = np.concatenate([s["u_pred"] for s in samples])
    ur = np.concatenate([s["u_real"] for s in samples])
    f = np.concatenate([s["f"] for s in samples])
    en = np.concatenate([s["e_n"] for s in samples])
    near = up >= cfg.u_threshold
    for i in (0, 1, 5, 10, 20, 29):
        m = (k == i) & near
        d = ur[m] - up[m]
        stats[i] = {"usage_err_mean": float(d.mean()), "usage_err_std": float(d.std()),
                    "with_friction_std": float((d + 1 - f[m]).std()),
                    "lateral_err_p95": float(np.quantile(en[k == i], 0.95))}
    cal["stage_stats"] = stats
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "calibration.json").write_text(json.dumps(cal, indent=1))
    return cal


def save_samples(samples: list[dict], tag: str) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    arrays = {f"{i}_{k}": np.asarray(v) for i, smp in enumerate(samples) for k, v in smp.items()}
    np.savez_compressed(OUT / f"calibration_samples_{tag}.npz", **arrays)


def run_iteration(cfg: L3Config) -> dict:
    """Second calibration round: each SMPC (eps in ROBUST_EPS) drives the calibration laps
    with the back-offs of the first round, and its own prediction errors give new ones."""
    cal_path = OUT / "calibration.json"
    cal = json.loads(cal_path.read_text())
    specs = controllers(cfg, cal)
    names = [f"smpc_{e}" for e in ROBUST_EPS]
    plans = plans_for(sorted({(specs[n]["plan_eta"], 0.3) for n in names}))
    c = config_dict(cfg)
    res = run_jobs([(n, specs[n], s, plans, c, True) for n in names for s in cfg.calib_seeds],
                   int(os.environ.get("L3_WORKERS", cfg.workers)))
    cal["smpc_iter"], cal["iteration_runs"] = [], []
    for e, n in zip(ROBUST_EPS, names):
        rs = [r for r in res if r["controller"] == n]
        samples = [r.pop("samples") for r in rs]
        save_samples(samples, f"iter_{e}")
        cal["smpc_iter"].append(calibrate(samples, cfg, e))
        cal["iteration_runs"] += rs
    cal_path.write_text(json.dumps(cal, indent=1))
    return cal


def failure_analysis(cfg: L3Config) -> None:
    """Re-run the failed MPC laps (deterministic) and record whether an axle exceeded its
    friction ellipse in the last 2 s before the car left the track."""
    results = json.loads((OUT / "runs.json").read_text())
    cal = json.loads((OUT / "calibration.json").read_text())
    specs = controllers(cfg, cal)
    failed = [(r["controller"], r["seed"]) for r in results
              if not r["completed"] and r["controller"] != "tracker" and r["controller"] in specs]
    plans = plans_for(sorted({(specs[n]["plan_eta"], specs[n].get("plan_margin", 0.3)) for n, _ in failed}))
    c = config_dict(cfg)
    spec_nr = lambda n: {k: v for k, v in specs[n].items() if k != "measure_events"}   # noqa: E731
    res = run_jobs([(n, spec_nr(n), s, plans, c, False) for n, s in failed],
                   int(os.environ.get("L3_WORKERS", cfg.workers)))
    (OUT / "failures.json").write_text(json.dumps(res, indent=1))
    print(failure_table(res))


def failure_table(res) -> str:
    lines = ["| controller | failed laps | friction ellipse exceeded in the last 2 s |", "|---|---|---|"]
    for c in [c for c in LABELS if any(r["controller"] == c for r in res)]:
        rs = [r for r in res if r["controller"] == c]
        n = sum(r["max_usage_last_2s"] > 1.0 for r in rs)
        lines.append(f"| {LABELS[c]} | {len(rs)} | {n} |")
    return "\n".join(lines)


def controllers(cfg: L3Config, cal: dict) -> dict:
    specs = {"tracker": {"kind": "tracker", "plan_eta": 0.95}}
    for eta in UNIFORM:
        specs[f"nmpc_{eta:.2f}"] = {"kind": "nmpc", "plan_eta": eta, "eta": eta}
    for eta, extra in UNIFORM_TRACK:                 # uniform friction and track reserves
        m = round(0.3 + extra, 2)
        specs[f"nmpc_{eta:.2f}_m{m}"] = {"kind": "nmpc", "plan_eta": eta, "plan_margin": m,
                                         "eta": eta, "mpc_margin": m}
    for key, suffix in (("smpc", ""), ("smpc_trunc", "_trunc"), ("smpc_iter", "_iter")):
        for c in cal.get(key, []):
            specs[f"smpc_{c['eps']}{suffix}"] = {"kind": "nmpc", "plan_eta": plan_eta_for(c["eta"]),
                                                 "eta": c["eta"], "backoff": c["backoff"],
                                                 "measure_events": True}
    # ablation of the eps = 0.01 controller: which part of the tightening matters?
    c = next(c for c in cal["smpc"] if c["eps"] == ABLATION_EPS)
    pe = plan_eta_for(c["eta"])
    specs["smpc_fric_only"] = {"kind": "nmpc", "plan_eta": pe, "eta": c["eta"]}
    specs["smpc_track_only"] = {"kind": "nmpc", "plan_eta": 0.95, "eta": 0.95, "backoff": c["backoff"]}
    specs["smpc_flat"] = {"kind": "nmpc", "plan_eta": pe, "eta": c["eta"][0],
                          "backoff": [0.0] + [c["backoff"][1]] * cfg.N}
    # calibration from another environment (Python 3.11, numpy 2.4.6, scipy 1.17.1; computed
    # by the reviewer from the same procedure; docs/calibration/L3_calibration_other_env.json)
    other = ROOT / "docs" / "calibration" / "L3_calibration_other_env.json"
    if other.exists():
        co = next(c for c in json.loads(other.read_text())["full"] if c["eps"] == 0.01)
        specs["smpc_0.01_otherenv"] = {"kind": "nmpc", "plan_eta": plan_eta_for(co["eta"]),
                                       "eta": co["eta"], "backoff": co["backoff"], "measure_events": True}
    # the same total reserve without the shape: horizon means of eta_k and b_k, same plan
    specs["smpc_mean"] = {"kind": "nmpc", "plan_eta": pe, "eta": float(np.mean(c["eta"])),
                          "backoff": [0.0] + [float(np.mean(c["backoff"][1:]))] * cfg.N}
    return specs


LABELS = {"tracker": "plan 5 % + tracker", "nmpc_0.95": "NMPC, uniform 5 %",
          "nmpc_0.93": "NMPC, uniform 7 %", "nmpc_0.88": "NMPC, uniform 12 %",
          "nmpc_0.95_m0.8": "NMPC, uniform 5 % + 0.5 m", "nmpc_0.89_m1.1": "NMPC, uniform 11 % + 0.8 m",
          "smpc_0.2": "SMPC ε = 0.2", "smpc_0.05": "SMPC ε = 0.05", "smpc_0.01": "SMPC ε = 0.01",
          "smpc_fric_only": "ε = 0.01, friction tightening only",
          "smpc_track_only": "ε = 0.01, track back-off only (5 % plan)",
          "smpc_flat": "ε = 0.01 plan, stage-0/1 tightening at every stage",
          "smpc_mean": "ε = 0.01 plan, horizon-mean tightening at every stage",
          "smpc_0.05_trunc": "SMPC ε = 0.05, calibration without the last 2 s before failures",
          "smpc_0.01_trunc": "SMPC ε = 0.01, calibration without the last 2 s before failures",
          "smpc_0.05_iter": "SMPC ε = 0.05, re-calibrated on its own laps",
          "smpc_0.01_iter": "SMPC ε = 0.01, re-calibrated on its own laps",
          "smpc_0.01_otherenv": "SMPC ε = 0.01, calibrated in the other environment"}


def tables(results, cal) -> str:
    summ = summarize(results)
    lines = ["| controller | laps completed | 95 % interval | mean lap (completed) | lap std | time over the plant's ellipse | failures | mean solve |",
             "|---|---|---|---|---|---|---|---|"]
    for c in [c for c in LABELS if c in summ]:
        s = summ[c]
        ci = s["completed_ci95"]
        lines.append(f"| {LABELS[c]} | {s['completed']}/{s['runs']} | {100 * ci[0]:.0f}–{100 * ci[1]:.0f} % | "
                     + (f"{s['lap_mean']:.2f} s | {s['lap_std']:.2f} s" if s["lap_mean"] else "– | –")
                     + f" | {100 * s['usage_over_1_fraction_mean']:.2f} % | {'; '.join(s['failures']) or '-'} | "
                     + (f"{s['solve_ms_mean']:.0f} ms |" if s["solve_ms_mean"] else "– |"))
    lines += ["", "| comparison (same seeds) | only the first completes | only the second | exact McNemar p |",
              "|---|---|---|---|"]
    for a, b in PAIRS:
        if a in summ and b in summ:
            t = paired_test(results, a, b)
            lines.append(f"| {LABELS[a]} vs {LABELS[b]} | {t['only_a']} | {t['only_b']} | {t['p']:.3g} |")
    ev = [c for c in LABELS if any(r["controller"] == c and "events" in r for r in results)]
    if ev:
        lines += ["", "| controller | near-limit predictions whose realised error exceeded the stage's friction reserve | predicted axle positions whose realised error exceeded the stage's track back-off |",
                  "|---|---|---|"]
        for c in ev:
            rs = [r["events"] for r in results if r["controller"] == c and "events" in r]
            fn, fv = sum(e["friction_n"] for e in rs), sum(e["friction_violations"] for e in rs)
            tn, tv = sum(e["track_n"] for e in rs), sum(e["track_violations"] for e in rs)
            if fn == 0 and tn == 0:                 # run before the event statistics existed
                continue
            lines.append(f"| {LABELS[c]} | {100 * fv / max(fn, 1):.2f} % of {fn} | {100 * tv / max(tn, 1):.2f} % of {tn} |")
    lines += ["", "| ε | η at stage 0 / 10 / 20 / 29 | track back-off [m] at stage 1 / 10 / 20 / 30 | reference plan |",
              "|---|---|---|---|"]
    for key, tag in (("smpc", ""), ("smpc_trunc", " (without the last 2 s before failures)"),
                     ("smpc_iter", " (re-calibrated on the SMPC's own laps)")):
        for c in cal.get(key, []):
            e, b = c["eta"], c["backoff"]
            lines.append(f"| {c['eps']}{tag} | {e[0]:.3f} / {e[10]:.3f} / {e[20]:.3f} / {e[29]:.3f} | "
                         f"{b[1]:.2f} / {b[10]:.2f} / {b[20]:.2f} / {b[30]:.2f} | η = {plan_eta_for(e):.2f} |")
    return "\n".join(lines)


def figures(results, cal, figdir: Path) -> None:
    figdir.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.6))
    for c in cal["smpc"]:
        axs[0].plot(c["eta"], label=f"ε = {c['eps']}")
        axs[1].plot(c["backoff"], label=f"ε = {c['eps']}")
    for eta in UNIFORM:
        axs[0].axhline(eta, ls=":", color="grey")
    axs[0].set_xlabel("prediction stage (50 ms)"); axs[0].set_ylabel("friction factor η_k")
    axs[1].set_xlabel("prediction stage (50 ms)"); axs[1].set_ylabel("track back-off [m]")
    for ax in axs[:2]:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    summ = summarize(results)
    for c in [c for c in LABELS if c in summ]:
        s = summ[c]
        rate = s["completed"] / s["runs"]
        lap = s["lap_mean"] if s["lap_mean"] else np.nan
        axs[2].scatter(lap, 100 * rate, s=40)
        axs[2].annotate(LABELS[c], (lap, 100 * rate), fontsize=8, xytext=(4, 4), textcoords="offset points")
    axs[2].set_xlabel("mean lap time of completed laps [s]"); axs[2].set_ylabel("laps completed [%]")
    axs[2].grid(alpha=0.3)
    fig.savefig(figdir / "smpc.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def report(cfg: L3Config) -> None:
    results = json.loads((OUT / "runs.json").read_text())
    cal = json.loads((OUT / "calibration.json").read_text())
    (OUT / "summary.json").write_text(json.dumps({"config": config_dict(cfg), "summary": summarize(results)},
                                                 indent=2))
    t = tables(results, cal)
    (OUT / "tables.md").write_text(t, encoding="utf-8")
    print(t)
    figures(results, cal, ROOT / "docs" / "figures" / "L3")


def main() -> None:
    cfg = L3Config()
    args = sys.argv[1:]
    if args == ["report"]:
        report(cfg)
        return
    if args == ["iterate"]:
        run_iteration(cfg)
        return
    if args == ["failures"]:
        failure_analysis(cfg)
        return
    cal_path = OUT / "calibration.json"
    if args == ["calibrate"] or not cal_path.exists():
        run_calibration(cfg)
        if args == ["calibrate"]:
            return
    cal = json.loads(cal_path.read_text())
    specs = controllers(cfg, cal)
    names = [a for a in args if a in specs] or list(specs)
    plans = plans_for(sorted({(s["plan_eta"], s.get("plan_margin", 0.3)) for s in specs.values()}))
    c = config_dict(cfg)
    old = json.loads((OUT / "runs.json").read_text()) if (OUT / "runs.json").exists() else []
    done = {(r["controller"], r["seed"]) for r in old}           # runs are deterministic: resume
    jobs = [(n, specs[n], s, plans, c, False) for n in names for s in cfg.test_seeds
            if (n, s) not in done]
    new = run_jobs(jobs, int(os.environ.get("L3_WORKERS", cfg.workers)))
    results = old + new
    (OUT / "runs.json").write_text(json.dumps(results, indent=1))
    report(cfg)


if __name__ == "__main__":
    main()
