"""Baselines for L5 that do not learn (review):

* ``tracker_k``: the L1 tracker with a constant reference-speed factor k (0.85, 0.90)
  and no steering offset, i.e. the residual policy's actions frozen;
* ``nmpc_slow``: the L2 NMPC with a uniform reserve large enough to lap the nominal car
  about as slowly as the RL policy (friction factor ETA_SLOW, track margin MARGIN_SLOW),
  on its own plan with the same reserves.

Both on the L2 cars (51) and the L3 disturbance seeds (30), the test sets of L5.

    python scripts/l5_baselines.py tune       # nominal-car lap of the slow NMPC candidates
    python scripts/l5_baselines.py            # all baselines -> results/L5/baselines.json
"""
from __future__ import annotations

import json
import os
import pickle
import sys
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = ROOT / "results" / "L5"
CANDIDATES = [(0.82, 1.3), (0.80, 1.3), (0.78, 1.3)]      # (eta, track margin [m]) for "tune"
ETA_SLOW, MARGIN_SLOW = 0.82, 1.3                          # nominal lap closest to the RL policy (see docs)
FACTORS = (0.85, 0.90)


def slow_plan(eta: float, margin: float):
    from f1race.dynamics import DynVehicle
    from f1race.planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
    from f1race.track import Track

    path = OUT / f"plan_{eta:.2f}_{margin:.1f}.pkl"
    if path.exists():
        return pickle.load(open(path, "rb"))
    ideal = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][1.0]
    o = solve_min_time_dynamic(Track.load("Spielberg"), DynVehicle(), initial_guess_from_dynamic(ideal),
                               eta=eta, margin=margin)
    if not o.success:
        raise RuntimeError(f"plan eta={eta} margin={margin}: {o.status}")
    OUT.mkdir(parents=True, exist_ok=True)
    pickle.dump(o, open(path, "wb"))
    return o


def _job(args):
    sys.path.insert(0, str(ROOT / "src"))
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants

    kind, param, which, idx, plan = args
    nom = DynVehicle()
    if kind == "tracker":
        from f1race.rl.env import RacingEnv, evaluate_lap

        o = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
        env = RacingEnv(o, randomize_plant=False)
        a1 = (param - 1.0) / env.speed_down                     # constant action giving factor param
        policy = lambda obs: np.array([0.0, a1])                # noqa: E731
        if which == "L2":
            info, plant = sample_plants(nom, 50, 0)[idx]
            r = evaluate_lap(policy, env, plant=plant)
            r["plant"] = info
        else:
            r = evaluate_lap(policy, env, disturbances=True, dist_seed=idx)
            r["seed"] = idx
        r["controller"] = f"tracker_{param:.2f}"
    else:
        from f1race.control.nmpc import FrenetMPC, MPCController
        from f1race.experiments.l1 import dyn_reference
        from f1race.experiments.l2 import ref_from_plan, x0_from_plan
        from f1race.experiments.l3 import L3Config, config_dict, disturbances
        from f1race.sim.dynamic import DynamicSim
        from f1race.sim.stochastic import StochasticSim
        from f1race.track import Track

        eta, margin = param
        track = Track.load("Spielberg")
        path, _ = dyn_reference(track, nom, plan)
        mpc = FrenetMPC(track, nom, ref_from_plan(plan), mode="track", N=30, dt=0.05, eta=eta, margin=margin)
        ctl = MPCController(mpc, 0.01)
        if which == "L2":
            info, plant = sample_plants(nom, 50, 0)[idx]
            res = DynamicSim(track, plant, 0.01).run(ctl, x0_from_plan(plan, path), t_max=120.0)
        else:
            dist = disturbances(track, idx, config_dict(L3Config()))
            res = StochasticSim(track, nom, dist, 0.01).run(ctl, x0_from_plan(plan, path), t_max=120.0)
        r = {"completed": res.completed, "lap_time": res.lap_time, "stop_reason": res.stop_reason,
             "distance": float(res.progress[-1]), "fallbacks": int(sum(x["fallback"] for x in mpc.stats))}
        if which == "L2":
            r["plant"] = info
        else:
            r["seed"] = idx
        r["controller"] = f"nmpc_{eta:.2f}_m{margin:.1f}"
    r["set"] = which
    return r


def main() -> None:
    workers = int(os.environ.get("L5_WORKERS", 8))
    if sys.argv[1:] == ["tune"]:
        jobs = [("nmpc", c, "L2", 0, slow_plan(*c)) for c in CANDIDATES]
    else:
        plan = slow_plan(ETA_SLOW, MARGIN_SLOW)
        jobs = [("tracker", k, w, i, None) for k in FACTORS for w, n in (("L2", 51), ("L3", 30)) for i in range(n)]
        jobs += [("nmpc", (ETA_SLOW, MARGIN_SLOW), w, i, plan) for w, n in (("L2", 51), ("L3", 30))
                 for i in range(n)]
    res = []
    with get_context("spawn").Pool(workers) as pool:
        for r in pool.imap_unordered(_job, jobs):
            res.append(r)
            print(r["controller"], r["set"], r.get("seed", ""), r["stop_reason"], round(r["distance"]),
                  r["lap_time"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    name = "baselines_tune.json" if sys.argv[1:] == ["tune"] else "baselines.json"
    (OUT / name).write_text(json.dumps(res, indent=1, default=float))
    for c in sorted({r["controller"] for r in res}):
        for w in ("L2", "L3"):
            rs = [r for r in res if r["controller"] == c and r["set"] == w]
            if rs:
                done = [r["lap_time"] for r in rs if r["completed"]]
                print(f"{c:18s} {w} {len(done)}/{len(rs)} mean {np.mean(done) if done else float('nan'):.2f}")


if __name__ == "__main__":
    main()
