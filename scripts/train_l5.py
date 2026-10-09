"""Train the residual reinforcement-learning policy of L5 (SAC on top of the L1 tracker).

Needs the optional RL dependencies (pip install -e .[rl]) and the L1 plans
(scripts/run_l1.py).

    python scripts/train_l5.py                 # 300k steps, 8 parallel environments
    python scripts/train_l5.py 100000 runname  # shorter run / other name
    L5_HISTORY=8 python scripts/train_l5.py 600000 sac_history8   # policy with an observation history
    (further settings: L5_SEED, L5_EPISODE_S, L5_STEER_MODE, L5_CHECKPOINT_EVERY; see l5_multiseed.py)
"""
from __future__ import annotations

import json
import os
import pickle
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = Path(os.environ.get("L5_OUT", ROOT / "results" / "L5"))
N_ENVS = int(os.environ.get("L5_ENVS", "8"))
HISTORY = int(os.environ.get("L5_HISTORY", "1"))
SEED = int(os.environ.get("L5_SEED", "0"))
EPISODE_S = float(os.environ.get("L5_EPISODE_S", "20"))
STEER_MODE = os.environ.get("L5_STEER_MODE", "angle")
CHECKPOINT_EVERY = int(os.environ.get("L5_CHECKPOINT_EVERY", "0"))   # 0: only the latest model


def make_env(rank: int, disturbances: bool):
    def _init():
        sys.path.insert(0, str(ROOT / "src"))
        from stable_baselines3.common.monitor import Monitor

        from f1race.rl.env import RacingEnv

        plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
        env = RacingEnv(plan, randomize_plant=True, disturbances=disturbances, history=HISTORY,
                        episode_s=EPISODE_S, steer_mode=STEER_MODE, seed=10_000 + 1000 * SEED + rank)
        return Monitor(env)
    return _init


def main() -> None:
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import SubprocVecEnv

    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 300_000
    name = sys.argv[2] if len(sys.argv) > 2 else "sac_residual"
    OUT.mkdir(parents=True, exist_ok=True)
    # half of the environments with the L3 disturbances, all with L2-style plant parameters
    venv = SubprocVecEnv([make_env(i, disturbances=i % 2 == 1) for i in range(N_ENVS)],
                         start_method="spawn")
    model = SAC("MlpPolicy", venv, learning_rate=3e-4, buffer_size=300_000, batch_size=256,
                gamma=0.99, tau=0.005, train_freq=1, gradient_steps=1, learning_starts=5_000,
                policy_kwargs={"net_arch": [256, 256]}, seed=SEED, verbose=0, device="cpu")
    t0 = time.time()
    log = []
    chunk = 10_000 if CHECKPOINT_EVERY else 20_000
    done = 0
    while done < steps:
        model.learn(total_timesteps=min(chunk, steps - done), reset_num_timesteps=False)
        done = model.num_timesteps
        eps = list(model.ep_info_buffer)
        rec = {"steps": done, "minutes": (time.time() - t0) / 60.0,
               "ep_reward_mean": float(sum(e["r"] for e in eps) / max(len(eps), 1)),
               "ep_len_mean": float(sum(e["l"] for e in eps) / max(len(eps), 1))}
        log.append(rec)
        print(json.dumps(rec), flush=True)
        model.save(OUT / name)
        if CHECKPOINT_EVERY and done % CHECKPOINT_EVERY == 0:
            model.save(OUT / f"{name}_{done // 1000}k")
        (OUT / f"{name}_training.json").write_text(json.dumps(log, indent=1))
    venv.close()


if __name__ == "__main__":
    main()
