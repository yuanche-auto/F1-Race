"""L2 sensitivity study: one plant parameter at a time at the ends of its range.

    python scripts/l2_studies.py        # ~15 min on 8 workers; writes results/L2/oat.json
"""
from __future__ import annotations

import json
import pickle
import sys
from dataclasses import asdict, replace
from multiprocessing import get_context
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l2 import PERTURBATION, L2Config, run_one  # noqa: E402


def one_at_a_time(nom: DynVehicle):
    out = []
    for k, (lo, hi) in PERTURBATION.items():
        for val in (lo, hi):
            if k == "m" and val == 1.0:
                continue
            if k == "mu":
                v = replace(nom, mu_x=nom.mu_x * val, mu_y=nom.mu_y * val)
            elif k == "aero_front":
                v = replace(nom, aero_front=nom.aero_front + val)
            else:
                v = replace(nom, **{k: getattr(nom, k) * val})
            out.append(({k: val}, v))
    return out


def main() -> None:
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    plans = {"plan": cache["plans"][0.95], "safe": cache["plans"][0.93]}
    cfg = asdict(L2Config())
    plants = one_at_a_time(DynVehicle())
    jobs = [(c, info, v, plans, cfg) for c in ("plan_track", "nmpc_track") for info, v in plants]
    results = []
    with get_context("spawn").Pool(8) as pool:
        for r in pool.imap_unordered(run_one, jobs):
            results.append(r)
            print(r["controller"], r["plant"], r["stop_reason"], f"{r['lap_time']:.3f}", flush=True)
    (ROOT / "results" / "L2").mkdir(parents=True, exist_ok=True)
    (ROOT / "results" / "L2" / "oat.json").write_text(json.dumps(results, indent=1))
    print("\n| parameter | value | plan 5 % + tracker | NMPC on plan 5 % |")
    for info, _ in plants:
        cells = []
        for c in ("plan_track", "nmpc_track"):
            r = next(r for r in results if r["controller"] == c and r["plant"] == info)
            cells.append(f"{r['lap_time']:.2f} s" if r["completed"] else f"{r['stop_reason']} at {r['distance']:.0f} m")
        (k, v), = info.items()
        print(f"| {k} | {v} | {cells[0]} | {cells[1]} |")


if __name__ == "__main__":
    main()
