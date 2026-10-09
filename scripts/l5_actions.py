"""Where the L5 policies lose time: speed factor and steering offset along the track on
the nominal car (docs/figures/L5/actions.png).

    python scripts/l5_actions.py
"""
from __future__ import annotations

import pickle
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def lap_actions(name: str | None):
    from stable_baselines3 import SAC

    from f1race.rl.env import RacingEnv

    plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    history, model = 1, None
    if name:
        path = ROOT / "models" / "L5" / name
        if not path.with_suffix(".zip").exists():
            path = ROOT / "results" / "L5" / "multiseed" / name
        model = SAC.load(path, device="cpu")
        dim = model.observation_space.shape[0]
        history = 1 if dim == 19 else (dim - 2) // 19
    env = RacingEnv(plan, randomize_plant=False, history=history, episode_s=120.0)
    obs, _ = env.reset(options={"plant": env.nominal, "start_index": 0})
    s, k, d, v = [], [], [], []
    done = False
    while not done and env.progress < env.track.length:
        a = np.zeros(2) if model is None else model.predict(obs, deterministic=True)[0]
        obs, _, term, trunc, _ = env.step(a)
        done = term or trunc
        s.append(env.progress); k.append(env.ctl.v_scale); d.append(env.ctl.d_offset); v.append(env.x[3])
    return np.array(s), np.array(k), np.array(d), np.array(v), env.t


def main() -> None:
    out = ROOT / "docs" / "figures" / "L5"
    out.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for name, label in ((None, "tracker (no policy)"), ("B_s1_500k", "RL, selected checkpoint (B, seed 1, 500k)"),
                        ("A_s3_600k", "RL, median run of group A (seed 3, final)")):
        s, k, d, v, t = lap_actions(name)
        axs[0].plot(s, v * 3.6, lw=0.8, label=f"{label}: {t:.2f} s")
        if name:
            axs[1].plot(s, k, lw=0.8, label=label)
            axs[2].plot(s, np.degrees(d), lw=0.8, label=label)
            print(f"{label}: lap {t:.2f} s, mean speed factor {k.mean():.3f}, "
                  f"share of time below 0.95: {np.mean(k < 0.95):.2f}")
    axs[0].set_ylabel("speed [km/h]"); axs[1].set_ylabel("speed factor"); axs[2].set_ylabel("steering offset [deg]")
    axs[2].set_xlabel("distance [m]")
    for ax in axs:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.savefig(out / "actions.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
