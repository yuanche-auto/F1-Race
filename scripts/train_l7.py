"""L7 Part C: train one run of group P or F (docs/L7_protocol.md section 5, amendment A7).

    .venv-rl/Scripts/python scripts/train_l7.py --group F --seed 1          # -> results/L7/partC/F_s1/

Settings of L5 group B (SAC, 256x256, history of 8 observations + last action, 40 s episodes,
steering offset +-0.02 rad, speed factor 0.85-1.05, 600 000 decisions in 8 parallel
environments, a checkpoint every 50 000) on the L7 training distribution
(``rl.curriculum.L7TrainEnv``: four environments with a steering lag drawn per episode, four
with the L3 disturbances). Group F: 30 % of the episodes start 50-150 m before a position of
the failure pool – the last 200 failure positions: the failures of the curriculum laps C (40
laps, without a filter; off track or spin, not a time limit) run at every checkpoint, and the
crash points of training episodes; every record of training.json holds the pool's
composition (pool_train, pool_C).
At every checkpoint the validation laps V (40) are run for the selection (all runs).

Seeds: SAC seed = the run's seed (1-5); environment seeds 70 000 + 1 000 seed + rank (disjoint
from L5's 10 000 + ...). Training is not bit-reproducible (parallel environments); the
evaluation of a checkpoint is.
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import pickle
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
N_ENVS = 8
EPISODE_S = 40.0
HISTORY = 8
P_FAIL_START = 0.3
POOL_SIZE = 200
NL = chr(10)


def make_env(rank: int, seed: int, pool_file: str | None, p_fail: float):
    def _init():
        sys.path.insert(0, str(ROOT / "src"))
        from stable_baselines3.common.monitor import Monitor

        from f1race.rl.curriculum import L7TrainEnv

        plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
        env = L7TrainEnv(plan, pool_file=pool_file, p_fail_start=p_fail, randomize_plant=True,
                         disturbances=rank % 2 == 1, history=HISTORY, episode_s=EPISODE_S,
                         steer_mode="angle", seed=70_000 + 1000 * seed + rank)
        return Monitor(env)
    return _init


def eval_laps(model, set_name: str) -> list[dict]:
    """Full laps of the checkpoint (deterministic, without a filter) on a set."""
    from f1race.experiments.l7 import cases, make_env as eval_env, run_lap

    env = eval_env("angle")
    pol = lambda obs: model.predict(obs, deterministic=True)[0]              # noqa: E731
    out = []
    for c in cases(set_name):
        r = run_lap(pol, env, c)
        out.append({"case": c["name"], "completed": r["completed"], "lap_time": r["lap_time"],
                    "stop_reason": r["stop_reason"], "distance": r["distance"],
                    "s_end": float(env.s_prev)})          # centre-line position where the lap ended
    return out


def main() -> None:
    from stable_baselines3 import SAC
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from f1race.track import Track

    ap = argparse.ArgumentParser()
    ap.add_argument("--group", choices=["P", "F"], required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--steps", type=int, default=600_000)
    ap.add_argument("--every", type=int, default=50_000)
    ap.add_argument("--val-set", default="V", help="validation set (V; X for a code check)")
    ap.add_argument("--cur-set", default="C", help="curriculum set (C; X for a code check)")
    ap.add_argument("--out", default=None, help="output directory (default results/L7/partC/<group>_s<seed>)")
    a = ap.parse_args()
    name = f"{a.group}_s{a.seed}"
    out = Path(a.out) if a.out else ROOT / "results" / "L7" / "partC" / name
    out.mkdir(parents=True, exist_ok=True)
    L = Track.load("Spielberg").length
    pool = collections.deque(maxlen=POOL_SIZE)
    pool_file = out / "pool.json"
    is_f = a.group == "F"
    if is_f:
        pool_file.write_text("[]")

    class CrashPoints(BaseCallback):
        """Group F: crash points of training episodes join the failure pool."""

        def _on_step(self) -> bool:
            if is_f:
                for info, done in zip(self.locals["infos"], self.locals["dones"]):
                    if done and info.get("reason") in ("off track", "spin"):
                        pool.append(("train", float(info["s"]) % L))
            return True

    venv = SubprocVecEnv([make_env(i, a.seed, str(pool_file) if is_f else None, P_FAIL_START if is_f else 0.0)
                          for i in range(N_ENVS)], start_method="spawn")
    model = SAC("MlpPolicy", venv, learning_rate=3e-4, buffer_size=300_000, batch_size=256,
                gamma=0.99, tau=0.005, train_freq=1, gradient_steps=1, learning_starts=5_000,
                policy_kwargs={"net_arch": [256, 256]}, seed=a.seed, verbose=0, device="cpu")
    cb = CrashPoints()
    t0 = time.time()
    log = []
    done = 0
    while done < a.steps:
        model.learn(total_timesteps=min(10_000, a.steps - done), reset_num_timesteps=False, callback=cb)
        done = model.num_timesteps
        if is_f:
            pool_file.write_text(json.dumps([p for _, p in pool]))
        eps = list(model.ep_info_buffer)
        rec = {"steps": done, "minutes": (time.time() - t0) / 60.0,
               "ep_reward_mean": float(sum(e["r"] for e in eps) / max(len(eps), 1)),
               "ep_len_mean": float(sum(e["l"] for e in eps) / max(len(eps), 1)), "pool": len(pool),
               "pool_train": sum(src == "train" for src, _ in pool), "pool_C": sum(src == "C" for src, _ in pool)}
        if done % a.every == 0:
            k = done // 1000
            model.save(out / f"{name}_{k}k")
            t1 = time.time()
            val = eval_laps(model, a.val_set)
            with open(out / "validation.jsonl", "a", encoding="utf-8") as fo:
                for r in val:
                    fo.write(json.dumps({"checkpoint": f"{name}_{k}k", **r}) + NL)
            rec["val_completed"] = sum(r["completed"] for r in val)
            if is_f:
                cur = eval_laps(model, a.cur_set)
                with open(out / "curriculum.jsonl", "a", encoding="utf-8") as fo:
                    for r in cur:
                        fo.write(json.dumps({"checkpoint": f"{name}_{k}k", **r}) + NL)
                for r in cur:                         # off track / spin only, as the crash points (pc-m1)
                    if r["stop_reason"] in ("off track", "spin"):
                        pool.append(("C", r["s_end"] % L))
                pool_file.write_text(json.dumps([p for _, p in pool]))
                rec["cur_failures"] = sum(r["stop_reason"] in ("off track", "spin") for r in cur)
                rec["pool_train"] = sum(src == "train" for src, _ in pool)
                rec["pool_C"] = sum(src == "C" for src, _ in pool)
            rec["eval_minutes"] = (time.time() - t1) / 60.0
        log.append(rec)
        print(json.dumps(rec), flush=True)
        model.save(out / name)
        (out / "training.json").write_text(json.dumps(log, indent=1))
        if is_f:
            (out / "pool_sources.json").write_text(json.dumps(list(pool)))
    venv.close()


if __name__ == "__main__":
    main()
