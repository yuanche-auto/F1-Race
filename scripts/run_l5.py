"""Evaluate the L5 residual RL policy against the bare tracker on the L2 cars and the L3
disturbance seeds (the same test sets as the model-based controllers of L2 and L3).

    python scripts/run_l5.py [model name]      # default results/L5/sac_residual.zip
"""
from __future__ import annotations

import json
import pickle
import sys
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = ROOT / "results" / "L5"


def _job(args):
    sys.path.insert(0, str(ROOT / "src"))
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.rl.env import RacingEnv, evaluate_lap

    name, which, idx, use_policy = args
    plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    policy, history = None, 1
    if use_policy:
        from stable_baselines3 import SAC

        model = SAC.load(OUT / name, device="cpu")
        dim = model.observation_space.shape[0]
        history = 1 if dim == 19 else (dim - 2) // 19
        policy = lambda obs: model.predict(obs, deterministic=True)[0]      # noqa: E731
    env = RacingEnv(plan, randomize_plant=False, history=history)
    if which == "L2":
        info, plant = sample_plants(DynVehicle(), 50, 0)[idx]
        r = evaluate_lap(policy, env, plant=plant)
        r["plant"] = info
    else:
        r = evaluate_lap(policy, env, disturbances=True, dist_seed=idx)
        r["seed"] = idx
    r.update({"set": which, "controller": "residual RL" if use_policy else "tracker"})
    return r


def main() -> None:
    name = sys.argv[1] if len(sys.argv) > 1 else "sac_residual"
    jobs = [(name, "L2", i, p) for i in range(51) for p in (False, True)]       # the L2 cars
    jobs += [(name, "L3", s, p) for s in range(30) for p in (False, True)]      # the L3 test seeds
    res = []
    with get_context("spawn").Pool(int(__import__("os").environ.get("L5_WORKERS", 8))) as pool:
        for r in pool.imap_unordered(_job, jobs):
            res.append(r)
            print(r["set"], r["controller"], r["stop_reason"], round(r["distance"]), r["lap_time"], flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}_eval.json").write_text(json.dumps(res, indent=1))
    for st in ("L2", "L3"):
        for c in ("tracker", "residual RL"):
            rs = [r for r in res if r["set"] == st and r["controller"] == c]
            done = [r["lap_time"] for r in rs if r["completed"]]
            print(f"{st} {c:12s} {len(done)}/{len(rs)} laps, mean "
                  f"{np.mean(done) if done else float('nan'):.2f} s")


if __name__ == "__main__":
    main()
