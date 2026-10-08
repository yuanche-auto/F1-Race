"""Reproduce the L4 results (identify the car from an exploration lap, re-plan, drive).

Needs the L1 plans (scripts/run_l1.py) and the 12 % plan of L2 (scripts/run_l2.py).

    python scripts/run_l4.py           # ~1 h on 7 processes; writes results/L4/
    python scripts/run_l4.py report    # tables / figure from results/L4/runs.json
    python scripts/run_l4.py noise     # identification from noisy data (results/L4/noisy.json, ~40 min)
"""
from __future__ import annotations

import json
import pickle
import sys
from multiprocessing import get_context
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.estimation.identify import PARAMS  # noqa: E402
from f1race.experiments.l2 import sample_plants  # noqa: E402
from f1race.experiments.l4 import (L4Config, config_dict, paired, run_plant,  # noqa: E402
                                   run_plant_noisy, summarize)

OUT = ROOT / "results" / "L4"
KEYS = ("nominal", "explore", "identified", "oracle")
LABELS = {"nominal": "NMPC, nominal model, 5 %", "explore": "exploration: NMPC, nominal model, 12 %",
          "identified": "NMPC, identified model, 5 %", "oracle": "NMPC, true model, 5 %"}


def cell(r) -> str:
    return f"{r['lap_time']:.2f}" if r["completed"] else f"{r['stop_reason']} @ {r['distance']:.0f} m"


def tables(results) -> str:
    summ = summarize(results)
    lines = ["| controller | laps completed | 95 % interval | mean lap (completed) |", "|---|---|---|---|"]
    for k in KEYS:
        s = summ[k]
        ci = s["completed_ci95"]
        lines.append(f"| {LABELS[k]} | {s['completed']}/{s['runs']} | {100 * ci[0]:.0f}–{100 * ci[1]:.0f} % | "
                     + (f"{s['lap_mean']:.2f} s |" if s["lap_mean"] else "– |"))
    lines += ["", "| comparison (same cars) | only the first completes | only the second | exact McNemar p |",
              "|---|---|---|---|"]
    for a, b in (("identified", "nominal"), ("identified", "explore"), ("oracle", "identified")):
        t = paired(results, a, b)
        lines.append(f"| {LABELS[a]} vs {LABELS[b]} | {t['only_a']} | {t['only_b']} | {t['p']:.3g} |")
    if "identified_minus_oracle_lap" in summ:
        d = summ["identified_minus_oracle_lap"]
        lines += ["", f"Lap time identified minus true model, on the {d['cars']} cars both complete: "
                      f"mean {d['mean']:+.3f} s, largest {d['max_abs']:.3f} s."]
    order = sorted(results, key=lambda r: (0 if r["plant"].get("nominal") else 1, r["plant"].get("mu", 1.0)))
    lines += ["", "| friction | mass | C_lA | B | front aero | nominal model | exploration | identified | true model | identification error (max over parameters) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in order:
        p = r["plant"]
        head = (["nominal", "", "", "", ""] if p.get("nominal") else
                [f"{p['mu']:.3f}", f"{p['m']:.3f}", f"{p['ClA']:.3f}", f"{p['B']:.3f}", f"{p['aero_front']:+.3f}"])
        err = np.abs(np.subtract(r["identification"]["theta"], r["true_theta"]))
        worst = PARAMS[int(np.argmax(err))]
        lines.append("| " + " | ".join(head + [cell(r[k]) for k in KEYS]
                                        + [f"{err.max():.4f} ({worst})"]) + " |")
    return "\n".join(lines)


def figures(results, figdir: Path) -> None:
    figdir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4.8))
    for k, mk in zip(KEYS, "osD^"):
        mu = np.array([r["plant"].get("mu", 1.0) for r in results])
        lap = np.array([r[k]["lap_time"] if r[k]["completed"] else np.nan for r in results])
        ok = np.isfinite(lap)
        ax.scatter(mu[ok], lap[ok], marker=mk, s=36, label=f"{LABELS[k]} ({ok.sum()}/{len(ok)})")
    ax.set_xlabel("friction factor of the car (1 = nominal)")
    ax.set_ylabel("lap time [s] (completed laps)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.savefig(figdir / "identify_replan.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def report(cfg: L4Config) -> None:
    results = json.loads((OUT / "runs.json").read_text())
    (OUT / "summary.json").write_text(json.dumps({"config": config_dict(cfg), "summary": summarize(results)},
                                                 indent=2))
    t = tables(results)
    (OUT / "tables.md").write_text(t, encoding="utf-8")
    print(t)
    figures(results, ROOT / "docs" / "figures" / "L4")


def noise_study(cfg: L4Config, plans) -> None:
    c = config_dict(cfg)
    jobs = [(info, v, plans, c, 1000 + k)
            for k, (info, v) in enumerate(sample_plants(DynVehicle(), cfg.n_plants, cfg.seed))]
    res = []
    with get_context("spawn").Pool(int(__import__("os").environ.get("L4_WORKERS", cfg.workers))) as pool:
        for r in pool.imap_unordered(run_plant_noisy, jobs):
            res.append(r)
            print(r["plant"], cell(r["identified"]), np.round(r["identification"]["theta"], 3), flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "noisy.json").write_text(json.dumps(res, indent=1, default=float))
    print(noise_table(res))


def noise_table(res) -> str:
    err = np.array([np.subtract(r["identification"]["theta"], r["true_theta"]) for r in res])
    done = [r for r in res if r["identified"]["completed"]]
    lines = [f"Identified from noisy data: {len(done)}/{len(res)} laps completed.", "",
             "| parameter | " + " | ".join(PARAMS) + " |", "|---|" + "---|" * len(PARAMS),
             "| mean error | " + " | ".join(f"{e:+.3f}" for e in err.mean(axis=0)) + " |",
             "| largest error | " + " | ".join(f"{e:.3f}" for e in np.abs(err).max(axis=0)) + " |"]
    return "\n".join(lines)


def main() -> None:
    cfg = L4Config()
    if sys.argv[1:] == ["report"]:
        report(cfg)
        return
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    plans = {1.0: cache["plans"][1.0], 0.95: cache["plans"][0.95],
             0.88: pickle.load(open(ROOT / "results" / "L2" / "plan_088.pkl", "rb"))}
    if sys.argv[1:] == ["noise"]:
        noise_study(cfg, plans)
        return
    c = config_dict(cfg)
    jobs = [(info, v, plans, c) for info, v in sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)]
    results = []
    with get_context("spawn").Pool(cfg.workers) as pool:
        for res in pool.imap_unordered(run_plant, jobs):
            results.append(res)
            print(res["plant"], {k: cell(res[k]) for k in KEYS}, flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "runs.json").write_text(json.dumps(results, indent=1, default=float))
    report(cfg)


if __name__ == "__main__":
    main()
