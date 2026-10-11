"""L6: the selected L5 policy (and, as a consistency check, the bare tracker of the RL
environment) on the L6 cars with a steering lag. Needs the RL extras (`pip install -e .[rl]`).

    python scripts/l6_rl.py [tau ...]        # default 0.03 0.06 0.12 -> results/L6/rl.jsonl
    L6_CARS=0 python scripts/l6_rl.py 0.005 0.015   # only some cars (the lag sweep 1a: the nominal car)
"""
from __future__ import annotations

import json
import os
import pickle
import sys
from multiprocessing import get_context
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = ROOT / "results" / "L6"
POLICY = "B_s1_500k"
N_CARS = 25


def _job(args):
    sys.path.insert(0, str(ROOT / "src"))
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.rl.env import RacingEnv, evaluate_lap

    car, tau, use_policy = args
    plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    policy, history = None, 1
    if use_policy:
        from stable_baselines3 import SAC

        model = SAC.load(ROOT / "models" / "L5" / POLICY, device="cpu")
        history = (model.observation_space.shape[0] - 2) // 19
        policy = lambda obs: model.predict(obs, deterministic=True)[0]      # noqa: E731
    env = RacingEnv(plan, randomize_plant=False, history=history, steer_mode="angle", observe="command")
    info, plant = sample_plants(DynVehicle(), N_CARS - 1, 0)[car]
    r = evaluate_lap(policy, env, plant=plant, steer_lag=tau if tau > 0 else None)
    r.update({"round": 1, "car": car, "plant": info, "tau": tau, "observe": "command",
              "controller": "rl" if use_policy else "rl_env_tracker"})
    return r


def main() -> None:
    taus = [float(a) for a in sys.argv[1:]] or [0.03, 0.06, 0.12]
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "rl.jsonl"
    done = set()
    if path.exists():
        for line in open(path, encoding="utf-8"):
            r = json.loads(line)
            done.add((r["car"], r["tau"], r["controller"]))
    cars = [int(c) for c in os.environ["L6_CARS"].split(",")] if os.environ.get("L6_CARS") else range(N_CARS)
    jobs = [(c, t, p) for t in taus for c in cars for p in (True, False)
            if (c, t, "rl" if p else "rl_env_tracker") not in done]
    with get_context("spawn").Pool(int(os.environ.get("L6_WORKERS", 8))) as pool, open(path, "a", encoding="utf-8") as f:
        for r in pool.imap_unordered(_job, jobs):
            f.write(json.dumps(r) + "\n")
            f.flush()
            print(r["tau"], r["car"], r["controller"], r["stop_reason"], round(r["distance"]), r["lap_time"], flush=True)


if __name__ == "__main__":
    main()
