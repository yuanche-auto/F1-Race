"""Reproduce the L2 results (closed loop with model mismatch, Monte Carlo).

Needs the L1 plans: run scripts/run_l1.py first (it caches results/L1/l1_plans.pkl).

    python scripts/run_l2.py                      # ~45 min on 8 worker processes
    python scripts/run_l2.py nmpc_track_robust    # only some controllers (merged into runs.json)
    python scripts/run_l2.py report               # tables / figure from results/L2/runs.json
"""
from __future__ import annotations

import json
import pickle
import sys
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.experiments.l2 import L2Config, run, summarize  # noqa: E402

LABELS = {"plan_track": "plan 5 % + tracker", "plan_track_safe": "plan 7 % + tracker",
          "nmpc_track": "NMPC on plan 5 %", "nmpc_track_safe": "NMPC on plan 7 %",
          "nmpc_track_robust": "NMPC on plan 12 %"}


def figures(results, figdir: Path) -> None:
    figdir.mkdir(parents=True, exist_ok=True)
    ctrls = [c for c in LABELS if any(r["controller"] == c for r in results)]
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.8))
    for k, c in enumerate(ctrls):
        rs = [r for r in results if r["controller"] == c]
        mu = np.array([r["plant"].get("mu", 1.0) for r in rs])
        lap = np.array([r["lap_time"] if r["completed"] else np.nan for r in rs])
        ok = np.isfinite(lap)
        axs[0].scatter(mu[ok], lap[ok], label=LABELS[c], s=30, marker="o")
        axs[0].scatter(mu[~ok], np.full((~ok).sum(), 79.5 + 0.3 * k), marker="x", s=40,
                       color=axs[0].collections[-1].get_facecolor()[0] if ok.any() else None)
        axs[1].bar(k, ok.mean() * 100, label=LABELS[c])
    axs[0].set_xlabel("friction factor of the plant (1 = nominal)")
    axs[0].set_ylabel("lap time [s]   (x = did not finish)")
    axs[0].grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[1].set_xticks(range(len(ctrls)))
    axs[1].set_xticklabels([LABELS[c] for c in ctrls], rotation=15, fontsize=8)
    axs[1].set_ylabel("laps completed [%]")
    axs[1].grid(alpha=0.3, axis="y")
    fig.savefig(figdir / "monte_carlo.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def tables(results) -> str:
    """Markdown tables for docs/L2.md: summary per controller and outcome per plant."""
    import json as _json

    ctrls = [c for c in LABELS if any(r["controller"] == c for r in results)]
    lines = ["| controller | laps completed | mean lap (completed) | nominal car | failures | mean solve |",
             "|---|---|---|---|---|---|"]
    for c in ctrls:
        rs = [r for r in results if r["controller"] == c]
        done = [r["lap_time"] for r in rs if r["completed"]]
        nom = next(r for r in rs if r["plant"].get("nominal"))
        fails = sorted({r["stop_reason"] for r in rs if not r["completed"]})
        sm = [r["solve_ms_mean"] for r in rs if c.startswith("nmpc") and r.get("solve_ms_mean")]
        nom_cell = f"{nom['lap_time']:.2f} s" if nom["completed"] else f"{nom['stop_reason']} @ {nom['distance']:.0f} m"
        lines.append(f"| {LABELS[c]} | {len(done)}/{len(rs)} | "
                     f"{(sum(done) / len(done)) if done else float('nan'):.2f} s | "
                     f"{nom_cell} | {', '.join(fails) or '-'} | "
                     f"{(f'{sum(sm) / len(sm):.0f} ms') if sm else '–'} |")
    plants = {}
    for r in results:
        plants.setdefault(_json.dumps(r["plant"], sort_keys=True), {})[r["controller"]] = r
    order = sorted(plants, key=lambda k: (0 if "nominal" in k else 1, _json.loads(k).get("mu", 1.0)))
    hdr = "| friction | mass | C_lA | C_dA | I_z | B | front aero | " + " | ".join(LABELS[c] for c in ctrls) + " |"
    lines += ["", hdr, "|" + "---|" * (7 + len(ctrls))]
    for k in order:
        p_ = _json.loads(k)
        if p_.get("nominal"):
            cells = ["nominal"] + [""] * 6
        else:
            cells = [f"{p_['mu']:.3f}", f"{p_['m']:.3f}", f"{p_['ClA']:.3f}", f"{p_['CdA']:.3f}",
                     f"{p_['Iz']:.3f}", f"{p_['B']:.3f}", f"{p_['aero_front']:+.3f}"]
        for c in ctrls:
            r = plants[k].get(c)
            cells.append("–" if r is None else (f"{r['lap_time']:.2f}" if r["completed"]
                                                 else f"{r['stop_reason']} @ {r['distance']:.0f} m"))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def robust_plan(cache):
    """12 % plan (eta = 0.88), solved once from the ideal L1 plan and cached."""
    path = ROOT / "results" / "L2" / "plan_088.pkl"
    if path.exists():
        return pickle.load(open(path, "rb"))
    from f1race.dynamics import DynVehicle
    from f1race.planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
    from f1race.track import Track

    o = solve_min_time_dynamic(Track.load("Spielberg"), DynVehicle(),
                               initial_guess_from_dynamic(cache["plans"][1.0]), eta=0.88)
    if not o.success:
        raise RuntimeError(f"eta = 0.88 plan failed: {o.status}")
    path.parent.mkdir(parents=True, exist_ok=True)
    pickle.dump(o, open(path, "wb"))
    return o


def report(out: Path, cfg) -> None:
    results = json.loads((out / "runs.json").read_text())
    for r in results:                      # trackers have no solver (older runs logged junk here)
        if not r["controller"].startswith("nmpc"):
            r["solve_ms_mean"] = r["solve_ms_max"] = None
    (out / "runs.json").write_text(json.dumps(results, indent=1))
    summ = summarize(results)
    (out / "summary.json").write_text(json.dumps({"config": asdict(cfg), "summary": summ}, indent=2))
    (out / "tables.md").write_text(tables(results), encoding="utf-8")
    print(tables(results))
    figures(results, ROOT / "docs" / "figures" / "L2")


def main() -> None:
    if sys.argv[1:] == ["report"]:
        report(ROOT / "results" / "L2", L2Config())
        return
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    plans = {"plan": cache["plans"][0.95], "safe": cache["plans"][0.93], "robust": robust_plan(cache)}
    cfg = L2Config()
    out = ROOT / "results" / "L2"
    ctrls = sys.argv[1:] or ("plan_track", "plan_track_safe", "nmpc_track", "nmpc_track_safe",
                             "nmpc_track_robust")
    new = run(plans, cfg, controllers=ctrls, outdir=None)
    old = json.loads((out / "runs.json").read_text()) if (out / "runs.json").exists() else []
    results = [r for r in old if r["controller"] not in ctrls] + new
    out.mkdir(parents=True, exist_ok=True)
    (out / "runs.json").write_text(json.dumps(results, indent=1))
    report(out, cfg)


if __name__ == "__main__":
    main()
