"""L8 runs (docs/L8_protocol.md): model-based controllers, the RL loop, and the fine-tuning of
Part C. Run with .venv-rl from a pinned worktree; test sets (A, A0, B) are run once.

    python scripts/run_l8.py model --set A --kinds nmpc,nmpc_reserve,smpc,identified,lag_identified
    python scripts/run_l8.py rl --set A [--policies ...]             # RL policies and the bare tracker
    python scripts/run_l8.py finetune --target Zandvoort --seed 1     # Part C (after Parts A and B)
    python scripts/run_l8.py ft-eval --target Zandvoort               # fine-tuned checkpoints on their laps
    python scripts/run_l8.py jobs                                     # the job list (counts)

Sets: ``A`` (test tracks), ``A0`` (Spielberg reference; only ``nmpc_reserve`` and
``identified`` are new, the rest is L7's), ``B`` (wet Spielberg); ``VA`` / ``VB`` (validation),
``XA`` / ``XB`` (code checks). Records: results/L8/eval/<set>_model.jsonl, <set>_rl.jsonl (key
``job`` = kind or policy | case); fine-tuning: results/L8/partC/<target>_s<seed>/.

Policies: the nine final L5 checkpoints, ``B_s1_500k``, ``tracker`` (the bare tracker, no
policy), the ten final L7 checkpoints and the two validation-selected ones (protocol section 3).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_l7_partB import pool_run  # noqa: E402

OUT = ROOT / "results" / "L8" / "eval"
L5_FINALS = [f"{g}_s{s}" for g in "ABC" for s in (1, 2, 3)]
L7_FINALS = [f"{g}_s{s}_600k" for g in "PF" for s in range(1, 6)]
L7_SELECTED = ["P_s5_500k", "F_s5_450k"]
POLICIES = L5_FINALS + ["B_s1_500k", "tracker"] + L7_FINALS + L7_SELECTED
FT_TARGETS = ("Zandvoort", "SaoPaulo", "Norisring", "Nuerburgring", "wet")
FT_SEEDS = (1, 2, 3)
FT_STEPS = 100_000
FT_WET = (0.70, 0.90)
FT_INDEX = {t: i for i, t in enumerate(FT_TARGETS + ("Monza", "Sakhir"))}   # validation tracks: code checks


def policy_path(name: str) -> Path:
    if name == "B_s1_500k":
        return ROOT / "models" / "L5" / "B_s1_500k.zip"
    if name in L5_FINALS:
        return ROOT / "results" / "L5" / "multiseed" / f"{name}.zip"
    if name.startswith("ft_"):                                    # ft_<target>_s<k>
        return ROOT / "results" / "L8" / "partC" / name[3:] / f"{name}.zip"
    return ROOT / "results" / "L7" / "partC" / name.rsplit("_", 1)[0] / f"{name}.zip"


def steer_mode(name: str) -> str:
    return "lateral" if name.startswith("C_") else "angle"


def set_cases(name: str) -> list[dict]:
    from f1race.experiments.l8 import cases

    return cases(name)


def job_model(args):
    kind, set_name, case_name = args
    from f1race.experiments.l8 import drive

    case = next(c for c in set_cases(set_name) if c["name"] == case_name)
    t0 = time.time()
    out = drive(case, kind)
    out.update({"job": f"{kind}|{case_name}", "kind": kind, "case": case_name, "set": set_name, "track": case["track"],
                "car": case["car"], "wet": case["wet"], "patch_seed": case["patch_seed"],
                "dist_seed": case["dist_seed"], "true_mu": float(case["plant"].mu_x / 1.5), "wall_s": time.time() - t0})
    return out


def job_rl(args):
    policy_name, set_name, case_name = args
    from f1race.experiments.l8 import make_env, plans_for, rl_lap, t_max_for

    case = next(c for c in set_cases(set_name) if c["name"] == case_name)
    if policy_name == "tracker":
        pol = None
    else:
        from stable_baselines3 import SAC

        model = SAC.load(policy_path(policy_name), device="cpu")
        pol = lambda obs: model.predict(obs, deterministic=True)[0]          # noqa: E731
    t0 = time.time()
    env = make_env(case["track"], steer_mode(policy_name))
    r = rl_lap(pol, env, case, t_max_for(plans_for(case["track"])))
    r.update({"job": f"{policy_name}|{case_name}", "policy": policy_name, "case": case_name, "set": set_name,
              "track": case["track"], "car": case["car"], "wet": case["wet"], "patch_seed": case["patch_seed"],
              "dist_seed": case["dist_seed"], "wall_s": time.time() - t0})
    return r


# ---------------------------------------------------------------- Part C: fine-tuning
def ft_env(rank: int, target: str, seed: int):
    def _init():
        sys.path.insert(0, str(ROOT / "src"))
        from dataclasses import replace

        from stable_baselines3.common.monitor import Monitor

        from f1race.experiments.l8 import plans_for
        from f1race.rl.env import RacingEnv, sample_plant

        track = "Spielberg" if target == "wet" else target

        class FTEnv(RacingEnv):
            """L5 group-B training environment on the target; for ``wet`` the sampled car's
            friction is scaled by a factor drawn per episode from FT_WET (protocol section 6)."""

            def reset(self, *, seed=None, options=None):
                options = dict(options or {})
                if target == "wet" and "plant" not in options:
                    p = sample_plant(self.nominal, self.rng)
                    f = float(self.rng.uniform(*FT_WET))
                    options["plant"] = replace(p, mu_x=p.mu_x * f, mu_y=p.mu_y * f)
                return super().reset(seed=seed, options=options)

        env = FTEnv(plans_for(track)[0.95], track=track, randomize_plant=True, disturbances=rank % 2 == 1,
                    history=8, episode_s=40.0, steer_mode="angle", seed=80_000 + 1000 * seed + 100 * FT_INDEX[target] + rank)
        return Monitor(env)
    return _init


def finetune(target: str, seed: int, steps: int, out: Path) -> None:
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import SubprocVecEnv

    name = f"ft_{target}_s{seed}"
    d = out / f"{target}_s{seed}"
    d.mkdir(parents=True, exist_ok=True)
    venv = SubprocVecEnv([ft_env(i, target, seed) for i in range(8)], start_method="spawn")
    model = SAC.load(policy_path("B_s1_500k"), env=venv, device="cpu")
    model.set_random_seed(seed)
    t0 = time.time()
    log = []
    done = 0
    while done < steps:
        model.learn(total_timesteps=min(10_000, steps - done), reset_num_timesteps=done == 0)
        done = model.num_timesteps
        eps = list(model.ep_info_buffer)
        log.append({"steps": done, "minutes": (time.time() - t0) / 60.0,
                    "ep_reward_mean": float(sum(e["r"] for e in eps) / max(len(eps), 1)),
                    "ep_len_mean": float(sum(e["l"] for e in eps) / max(len(eps), 1))})
        print(json.dumps(log[-1]), flush=True)
        (d / "training.json").write_text(json.dumps(log, indent=1))
    model.save(d / name)
    venv.close()


def ft_cases(target: str) -> tuple[str, list[str]]:
    """The evaluation laps of a fine-tuned checkpoint: its track's laps of Part A, or Part B."""
    if target == "wet":
        return "B", [c["name"] for c in set_cases("B")]
    return "A", [c["name"] for c in set_cases("A") if c["track"] == target]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["model", "rl", "finetune", "ft-eval", "jobs"])
    ap.add_argument("--set", default="XA")
    ap.add_argument("--kinds", default="nmpc,nmpc_reserve,smpc,identified,lag_identified")
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--target", default="Zandvoort")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--steps", type=int, default=FT_STEPS)
    ap.add_argument("--out", default=None)
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    if a.what == "jobs":
        from f1race.experiments.l8 import plans_for, t_max_for

        for s in ("A", "A0", "B"):
            cs = set_cases(s)
            kinds = ["nmpc_reserve", "identified"] if s == "A0" else a.kinds.split(",")
            print(f"{s}: {len(cs)} laps; model-based {len(kinds)} x {len(cs)} = {len(kinds) * len(cs)}"
                  + ("" if s == "A0" else f"; RL {len(POLICIES)} x {len(cs)} = {len(POLICIES) * len(cs)}"))
        for t in sorted({c["track"] for c in set_cases("A")} | {"Spielberg"}):
            try:
                print(f"  t_max {t}: {t_max_for(plans_for(t)):.1f} s")
            except FileNotFoundError:
                print(f"  t_max {t}: plans missing")
        print(f"Part C: {len(FT_TARGETS)} targets x {len(FT_SEEDS)} seeds x {FT_STEPS} decisions; evaluation "
              + ", ".join(f"{t} {len(ft_cases(t)[1])} laps" for t in FT_TARGETS))
        return
    out = Path(a.out) if a.out else OUT
    if a.what == "model":
        names = [c["name"] for c in set_cases(a.set)]
        pool_run(job_model, [(k, a.set, n) for k in a.kinds.split(",") for n in names], out / f"{a.set}_model.jsonl",
                 a.workers)
    elif a.what == "rl":
        names = [c["name"] for c in set_cases(a.set)]
        pool_run(job_rl, [(p, a.set, n) for p in a.policies.split(",") for n in names], out / f"{a.set}_rl.jsonl",
                 a.workers)
    elif a.what == "finetune":
        finetune(a.target, a.seed, a.steps, Path(a.out) if a.out else ROOT / "results" / "L8" / "partC")
    else:
        set_name, names = ft_cases(a.target)
        pols = [f"ft_{a.target}_s{k}" for k in FT_SEEDS]
        pool_run(job_rl, [(p, set_name, n) for p in pols for n in names], out / f"ft_{a.target}_rl.jsonl", a.workers)


if __name__ == "__main__":
    main()
