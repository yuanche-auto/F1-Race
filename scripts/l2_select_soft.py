"""Choose how the NMPC softens its friction / slip constraints, on a tuning set that is
separate from the evaluation cars (nominal car + 4 cars drawn with seed 1; the
Monte Carlo uses seed 0).

Variants: hard limits (the original formulation), soft at stage 0 only, soft at
stages 0-1, soft everywhere (with 500 instead of 200 IPOPT iterations, since with
200 it hit the iteration limit on the nominal car). Criterion, in this order:
laps completed, solves on the previous plan (fallbacks), mean lap time.

    python scripts/l2_select_soft.py      # ~20 min on 8 processes; results/L2/soft_selection.json
"""
from __future__ import annotations

import json
import pickle
import sys
import time
from multiprocessing import get_context
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

VARIANTS = {"hard": (False, 200), "soft stage 0": (1, 200), "soft stages 0-1": (2, 200),
            "soft everywhere": (True, 500)}


def _job(args):
    sys.path.insert(0, str(ROOT / "src"))
    import numpy as np

    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, sample_plants, x0_from_plan
    from f1race.sim.dynamic import DynamicSim
    from f1race.track import Track

    name, idx = args
    soft, maxit = VARIANTS[name]
    track, nom = Track.load("Spielberg"), DynVehicle()
    o = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    info, plant = sample_plants(nom, 4, seed=1)[idx]
    path, _ = dyn_reference(track, nom, o)
    mpc = FrenetMPC(track, nom, ref_from_plan(o), N=30, dt=0.05, eta=0.95, soft_limits=soft, max_iter=maxit)
    ctl = MPCController(mpc, 0.01)
    t0 = time.time()
    r = DynamicSim(track, plant, 0.01).run(ctl, x0_from_plan(o, path), t_max=120.0)
    st = mpc.stats
    return {"variant": name, "car": info, "completed": r.completed, "lap_time": r.lap_time,
            "stop_reason": r.stop_reason, "distance": float(r.progress[-1]),
            "fallbacks": int(sum(x["fallback"] for x in st)), "solves": len(st),
            "max_limit_slack": float(max(x["limit_slack"] for x in st)),
            "iterations_mean": float(np.mean([x["iter"] for x in st])), "wall_s": time.time() - t0}


def main() -> None:
    jobs = [(v, i) for v in VARIANTS for i in range(5)]
    res = []
    with get_context("spawn").Pool(8) as pool:
        for r in pool.imap_unordered(_job, jobs):
            res.append(r)
            print(r["variant"], r["car"], r["stop_reason"], round(r["distance"]), r["lap_time"],
                  "fallbacks", r["fallbacks"], flush=True)
    out = ROOT / "results" / "L2"
    out.mkdir(parents=True, exist_ok=True)
    (out / "soft_selection.json").write_text(json.dumps(res, indent=1))
    print("\n| variant | laps completed | solves on the previous plan | mean lap (completed) |")
    print("|---|---|---|---|")
    for v in VARIANTS:
        rs = [r for r in res if r["variant"] == v]
        done = [r["lap_time"] for r in rs if r["completed"]]
        print(f"| {v} | {len(done)}/{len(rs)} | {sum(r['fallbacks'] for r in rs)} | "
              f"{(sum(done) / len(done)) if done else float('nan'):.2f} s |")


if __name__ == "__main__":
    main()
