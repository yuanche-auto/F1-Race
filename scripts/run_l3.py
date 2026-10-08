"""Reproduce the L3 results (stochastic disturbances, chance-constrained MPC).

Needs the L1 plans (scripts/run_l1.py) and the 12 % plan of L2 (scripts/run_l2.py,
or it is solved here).

    python scripts/run_l3.py              # calibration + Monte Carlo, ~2 h on 7 processes
    python scripts/run_l3.py calibrate    # only the calibration laps (results/L3/calibration.json)
    python scripts/run_l3.py report       # tables / figures from results/L3/runs.json
"""
from __future__ import annotations

import json
import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.experiments.l3 import L3Config, calibrate, config_dict, run_jobs, summarize  # noqa: E402

OUT = ROOT / "results" / "L3"
UNIFORM = (0.95, 0.93, 0.88)
UNIFORM_TRACK = ((0.95, 0.5), (0.89, 0.8))           # (eta, extra track margin [m])


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
    cal = {"runs": res, "smpc": [calibrate(samples, cfg, e) for e in cfg.eps]}
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


def controllers(cfg: L3Config, cal: dict) -> dict:
    specs = {"tracker": {"kind": "tracker", "plan_eta": 0.95}}
    for eta in UNIFORM:
        specs[f"nmpc_{eta:.2f}"] = {"kind": "nmpc", "plan_eta": eta, "eta": eta}
    for eta, extra in UNIFORM_TRACK:                 # uniform friction and track reserves
        m = round(0.3 + extra, 2)
        specs[f"nmpc_{eta:.2f}_m{m}"] = {"kind": "nmpc", "plan_eta": eta, "plan_margin": m,
                                         "eta": eta, "mpc_margin": m}
    for c in cal["smpc"]:
        specs[f"smpc_{c['eps']}"] = {"kind": "nmpc", "plan_eta": plan_eta_for(c["eta"]),
                                     "eta": c["eta"], "backoff": c["backoff"]}
    return specs


LABELS = {"tracker": "plan 5 % + tracker", "nmpc_0.95": "NMPC, uniform 5 %",
          "nmpc_0.93": "NMPC, uniform 7 %", "nmpc_0.88": "NMPC, uniform 12 %",
          "nmpc_0.95_m0.8": "NMPC, uniform 5 % + 0.5 m", "nmpc_0.89_m1.1": "NMPC, uniform 11 % + 0.8 m",
          "smpc_0.2": "SMPC ε = 0.2", "smpc_0.05": "SMPC ε = 0.05", "smpc_0.01": "SMPC ε = 0.01"}


def tables(results, cal) -> str:
    summ = summarize(results)
    lines = ["| controller | laps completed | mean lap (completed) | lap std | time over the plant's ellipse | failures | mean solve |",
             "|---|---|---|---|---|---|---|"]
    for c in [c for c in LABELS if c in summ]:
        s = summ[c]
        lines.append(f"| {LABELS[c]} | {s['completed']}/{s['runs']} | "
                     + (f"{s['lap_mean']:.2f} s | {s['lap_std']:.2f} s" if s["lap_mean"] else "– | –")
                     + f" | {100 * s['usage_over_1_fraction_mean']:.2f} % | {'; '.join(s['failures']) or '-'} | "
                     + (f"{s['solve_ms_mean']:.0f} ms |" if s["solve_ms_mean"] else "– |"))
    lines += ["", "| ε | η at stage 0 / 10 / 20 / 29 | track back-off [m] at stage 1 / 10 / 20 / 30 | reference plan |",
              "|---|---|---|---|"]
    for c in cal["smpc"]:
        e, b = c["eta"], c["backoff"]
        lines.append(f"| {c['eps']} | {e[0]:.3f} / {e[10]:.3f} / {e[20]:.3f} / {e[29]:.3f} | "
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
    jobs = [(n, specs[n], s, plans, c, False) for n in names for s in cfg.test_seeds]
    new = run_jobs(jobs, cfg.workers)
    old = json.loads((OUT / "runs.json").read_text()) if (OUT / "runs.json").exists() else []
    results = [r for r in old if r["controller"] not in names] + new
    (OUT / "runs.json").write_text(json.dumps(results, indent=1))
    report(cfg)


if __name__ == "__main__":
    main()
