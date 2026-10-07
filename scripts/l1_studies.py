"""L1 numerical studies (run after scripts/run_l1.py, which caches the plans):

1. grid study of the dynamic OCP (eta = 1, warm-started from the best ideal plan)
2. where the L0 -> L1 lap-time gap comes from (model variants)
3. robustness of the feed-forward/feedback tracker: plan margin x tracker gains

    python scripts/l1_studies.py                 # ~40 min; writes results/L1/studies.json
    python scripts/l1_studies.py gap tracker     # only some parts (merged into studies.json)
"""
from __future__ import annotations

import json
import pickle
import sys
import time
from dataclasses import replace
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

GAINS = [
    {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0},
    {"k_p": 0.05, "x_la": 12.0, "k_r": 0.05, "kv": 8.0},
    {"k_p": 0.05, "x_la": 15.0, "k_r": 0.05, "kv": 8.0},
    {"k_p": 0.08, "x_la": 12.0, "k_r": 0.05, "kv": 8.0},
    {"k_p": 0.08, "x_la": 8.0, "k_r": 0.10, "kv": 8.0},
    {"k_p": 0.05, "x_la": 12.0, "k_r": 0.05, "kv": 4.0},
    {"k_p": 0.03, "x_la": 12.0, "k_r": 0.05, "kv": 8.0},
    {"k_p": 0.10, "x_la": 10.0, "k_r": 0.05, "kv": 8.0},
]


def _track_job(args):
    sys.path.insert(0, str(ROOT / "src"))
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l1 import run_dyn_closed_loop
    from f1race.track import Track

    eta, gains, o = args
    r = run_dyn_closed_loop(Track.load("Spielberg"), DynVehicle(), o, 0.01, gains)
    return {"eta": eta, "gains": gains, "completed": r.completed, "lap_time": r.lap_time,
            "stop_reason": r.stop_reason, "distance": float(r.progress[-1]), "max_usage": r.max_usage,
            "min_edge_margin": r.min_margin, "max_abs_e": r.info["max_abs_e"],
            "max_rear_slip_deg": r.info["max_rear_slip_deg"]}


def main() -> None:
    from f1race.dynamics import DynVehicle
    from f1race.planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
    from f1race.track import Track

    parts = set(sys.argv[1:]) or {"grid", "gap", "tracker"}
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    plans = cache["plans"]
    track, veh = Track.load("Spielberg"), DynVehicle()
    ideal = plans[1.0]
    path = ROOT / "results" / "L1" / "studies.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    out.setdefault("ocp_grid", {}); out.setdefault("gap", {}); out.setdefault("tracker", [])

    for ds in ((5.0, 2.5, 1.25) if "grid" in parts else ()):
        t0 = time.time()
        o = solve_min_time_dynamic(track, veh, initial_guess_from_dynamic(ideal), ds=ds, eta=1.0, max_iter=2000)
        out["ocp_grid"][ds] = {"lap_time": o.lap_time, "success": o.success, "iterations": o.iterations,
                               "seconds": time.time() - t0}
        print("grid", ds, out["ocp_grid"][ds], flush=True)

    # (stiffer tyres are not a valid variant here: the lateral dynamics become too
    # stiff for 2.5 m intervals with two RK4 sub-steps and the optimiser exploits the
    # integration error)
    variants = {
        "L1 model": veh,
        "no longitudinal load transfer (h_cg = 0)": replace(veh, h_cg=0.0),
        "brake balance 50 % front": replace(veh, brake_front=0.50),
        "brake balance 70 % front": replace(veh, brake_front=0.70),
        "half the yaw inertia": replace(veh, Iz=0.5 * veh.Iz),
    }
    if "gap" in parts:
        out["gap"] = {}
    for name, v in (variants.items() if "gap" in parts else ()):
        o = solve_min_time_dynamic(track, v, initial_guess_from_dynamic(ideal), eta=1.0, max_iter=2000)
        out["gap"][name] = {"lap_time": o.lap_time, "success": o.success}
        print("gap", name, out["gap"][name], flush=True)
    out["gap"]["L0 kinematic (reference)"] = {"lap_time": cache["o0"].lap_time, "success": True}

    jobs = [(eta, g, plans[eta]) for eta in sorted(k for k in plans if k < 1.0) for g in GAINS]
    if "tracker" in parts:
        out["tracker"] = []
    else:
        jobs = []
    with get_context("spawn").Pool(12) as pool:
        for res in pool.imap_unordered(_track_job, jobs):
            out["tracker"].append(res)
            print("tracker", res, flush=True)

    path.write_text(json.dumps(out, indent=2, default=str))
    print("\n| OCP grid ds [m] | lap time [s] | iterations |")
    for ds, r in out["ocp_grid"].items():  # noqa: B007
        print(f"| {ds} | {r['lap_time']:.3f} | {r['iterations']} |")
    print("\n| model | ideal lap [s] |")
    for k, r in out["gap"].items():
        print(f"| {k} | {r['lap_time']:.3f} |")
    print("\n| plan margin | k_p | x_la | k_r | k_v | result | lap [s] | min edge margin [m] | max |e| [m] |")
    for r in sorted(out["tracker"], key=lambda r: (-r["eta"], str(r["gains"]))):
        g = r["gains"]
        res = "lap" if r["completed"] else f"{r['stop_reason']} at {r['distance']:.0f} m"
        print(f"| {r['eta']} | {g['k_p']} | {g['x_la']} | {g['k_r']} | {g['kv']} | {res} | "
              f"{r['lap_time']:.3f} | {r['min_edge_margin']:.3f} | {r['max_abs_e']:.3f} |")


if __name__ == "__main__":
    main()
