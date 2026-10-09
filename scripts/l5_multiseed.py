"""L5 redo with several training seeds (protocol: docs/L5_protocol.md).

Groups (history policy): A = 20 s episodes, steering offset +-0.02 rad (original);
B = 40 s episodes, same offset; C = 40 s episodes, offset also limited to a lateral-
acceleration change of 4 m/s^2. Seeds 1-3 each, 600 000 decisions, a checkpoint every
50 000. Every checkpoint is evaluated on the validation set (10 cars drawn with seed
1, 10 disturbance seeds 2000-2009; full laps); per group the checkpoint with the most
completed validation laps is selected (ties: lower mean lap time, then the later
checkpoint). Then the selected checkpoint of each group and the final checkpoint of
every seed are evaluated on the test sets (51 L2 cars, L3 seeds 0-29).

    python scripts/l5_multiseed.py            # everything; resumable (finished steps are kept)
    python scripts/l5_multiseed.py report     # tables from the stored results

Results in results/L5/multiseed/ (checkpoints, validation.json, test.json, tables.md).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = ROOT / "results" / "L5" / "multiseed"
GROUPS = {"A": {"episode_s": 20.0, "steer_mode": "angle"},
          "B": {"episode_s": 40.0, "steer_mode": "angle"},
          "C": {"episode_s": 40.0, "steer_mode": "lateral"}}
SEEDS = (1, 2, 3)
STEPS = 600_000
EVERY = 50_000
CONCURRENT = int(os.environ.get("L5_CONCURRENT", "3"))
WORKERS = int(os.environ.get("L5_WORKERS", "12"))
PY = sys.executable


def run_name(g: str, s: int) -> str:
    return f"{g}_s{s}"


def checkpoints(g: str, s: int) -> list[tuple[int, Path]]:
    return [(k, OUT / f"{run_name(g, s)}_{k // 1000}k.zip") for k in range(EVERY, STEPS + 1, EVERY)]


def trained(g: str, s: int) -> bool:
    return all(p.exists() for _, p in checkpoints(g, s))


# ---------------------------------------------------------------- training
def train_all() -> None:
    queue = [(g, s) for s in SEEDS for g in GROUPS if not trained(g, s)]
    running: list[tuple[subprocess.Popen, str]] = []
    OUT.mkdir(parents=True, exist_ok=True)
    while queue or running:
        while queue and len(running) < CONCURRENT:
            g, s = queue.pop(0)
            env = dict(os.environ, L5_HISTORY="8", L5_SEED=str(s), L5_EPISODE_S=str(GROUPS[g]["episode_s"]),
                       L5_STEER_MODE=GROUPS[g]["steer_mode"], L5_CHECKPOINT_EVERY=str(EVERY),
                       L5_OUT=str(OUT))
            log = open(OUT / f"{run_name(g, s)}.log", "w")
            p = subprocess.Popen([PY, "-u", str(ROOT / "scripts" / "train_l5.py"), str(STEPS), run_name(g, s)],
                                 env=env, stdout=log, stderr=subprocess.STDOUT, cwd=str(ROOT))
            running.append((p, run_name(g, s)))
            print(time.strftime("%H:%M"), "started", run_name(g, s), flush=True)
        time.sleep(30)
        for p, n in list(running):
            if p.poll() is not None:
                running.remove((p, n))
                print(time.strftime("%H:%M"), "finished", n, "exit", p.returncode, flush=True)


# ---------------------------------------------------------------- evaluation
def episode(args) -> dict:
    """One full lap of a checkpoint on one validation or test case."""
    sys.path.insert(0, str(ROOT / "src"))
    import pickle

    from stable_baselines3 import SAC

    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.rl.env import RacingEnv, evaluate_lap

    path, group, case = args
    plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    model = SAC.load(path, device="cpu")
    env = RacingEnv(plan, randomize_plant=False, history=8, steer_mode=GROUPS[group]["steer_mode"])
    policy = lambda obs: model.predict(obs, deterministic=True)[0]      # noqa: E731
    kind, idx = case
    if kind == "val_car":
        info, plant = sample_plants(DynVehicle(), 10, seed=1)[1 + idx]
        r = evaluate_lap(policy, env, plant=plant)
    elif kind == "val_dist":
        r = evaluate_lap(policy, env, disturbances=True, dist_seed=2000 + idx)
    elif kind == "test_car":
        info, plant = sample_plants(DynVehicle(), 50, seed=0)[idx]
        r = evaluate_lap(policy, env, plant=plant)
    else:
        r = evaluate_lap(policy, env, disturbances=True, dist_seed=idx)
    r.update({"checkpoint": Path(path).stem, "group": group, "case": [kind, idx]})
    return r


VAL_CASES = [("val_car", i) for i in range(10)] + [("val_dist", i) for i in range(10)]
TEST_CASES = [("test_car", i) for i in range(51)] + [("test_dist", i) for i in range(30)]


def evaluate(jobs, path: Path) -> list[dict]:
    old = json.loads(path.read_text()) if path.exists() else []
    done = {(r["checkpoint"], tuple(r["case"])) for r in old}
    todo = [j for j in jobs if (Path(j[0]).stem, tuple(j[2])) not in done]
    res = list(old)
    if todo:
        with get_context("spawn").Pool(WORKERS) as pool:
            for k, r in enumerate(pool.imap_unordered(episode, todo)):
                res.append(r)
                if k % 50 == 0:
                    path.write_text(json.dumps(res, indent=1, default=float))
    path.write_text(json.dumps(res, indent=1, default=float))
    return res


def score(rs: list[dict]) -> tuple[int, float]:
    done = [r["lap_time"] for r in rs if r["completed"]]
    return len(done), (float(np.mean(done)) if done else float("inf"))


def select(val: list[dict]) -> dict:
    """Per group: most completed validation laps, then lower mean lap time, then later."""
    out = {}
    for g in GROUPS:
        best = None
        for s in SEEDS:
            for k, p in checkpoints(g, s):
                rs = [r for r in val if r["checkpoint"] == p.stem]
                n, lap = score(rs)
                key = (-n, lap, -k)
                if best is None or key < best[0]:
                    best = (key, p.stem, n, lap, len(rs))
        out[g] = {"checkpoint": best[1], "val_completed": best[2], "val_laps": best[4], "val_lap_mean": best[3]}
    return out


# ---------------------------------------------------------------- report
def mcnemar(a: dict, b: dict) -> tuple[int, int, float]:
    from scipy.stats import binomtest

    only_a = sum(a[k] and not b[k] for k in a)
    only_b = sum(b[k] and not a[k] for k in a)
    n = only_a + only_b
    return only_a, only_b, (binomtest(min(only_a, only_b), n, 0.5).pvalue if n else 1.0)


def report() -> str:
    val = json.loads((OUT / "validation.json").read_text())
    test = json.loads((OUT / "test.json").read_text())
    sel = json.loads((OUT / "selection.json").read_text())

    def test_score(stem, kind):
        rs = [r for r in test if r["checkpoint"] == stem and r["case"][0] == kind]
        n, lap = score(rs)
        return f"{n}/{len(rs)}" + (f", {lap:.2f} s" if n else "")

    lines = ["**(b) final checkpoint of every seed (no selection)**", "",
             "| group | seed | validation | L2 test (51 cars) | L3 test (30 laps) |", "|---|---|---|---|---|"]
    finals = {}
    for g in GROUPS:
        for s in SEEDS:
            stem = f"{run_name(g, s)}_{STEPS // 1000}k"
            n, lap = score([r for r in val if r["checkpoint"] == stem])
            l2 = [r for r in test if r["checkpoint"] == stem and r["case"][0] == "test_car"]
            l3 = [r for r in test if r["checkpoint"] == stem and r["case"][0] == "test_dist"]
            laps = [r["lap_time"] for r in l2 + l3 if r["completed"]]
            # median-seed ranking (protocol): total completed test laps, then mean lap time
            finals[(g, s)] = (sum(r["completed"] for r in l2 + l3), -(float(np.mean(laps)) if laps else 1e9), -s)
            lines.append(f"| {g} | {s} | {n}/20 | {test_score(stem, 'test_car')} | {test_score(stem, 'test_dist')} |")
    lines += ["", "**(c) median seed per group** (completed test laps of the final checkpoint, L2 + L3; "
              "ties: lower mean lap time, then lower seed)", ""]
    for g in GROUPS:
        order = sorted(SEEDS, key=lambda s: finals[(g, s)])
        med = order[1]
        stem = f"{run_name(g, med)}_{STEPS // 1000}k"
        lines.append(f"* {g}: seed {med}: L2 {test_score(stem, 'test_car')}, L3 {test_score(stem, 'test_dist')}")
    lines += ["", "**(a) checkpoint selected on the validation set**", "",
              "| group | checkpoint | validation | L2 test | L3 test |", "|---|---|---|---|---|"]
    for g, x in sel.items():
        lines.append(f"| {g} | {x['checkpoint']} | {x['val_completed']}/{x['val_laps']}"
                     + (f", {x['val_lap_mean']:.2f} s" if x["val_completed"] else "")
                     + f" | {test_score(x['checkpoint'], 'test_car')} | {test_score(x['checkpoint'], 'test_dist')} |")
    lines += ["", "**Validation laps completed per checkpoint** (columns: 50k … 600k)", "",
              "| run | " + " | ".join(f"{k // 1000}k" for k in range(EVERY, STEPS + 1, EVERY)) + " |",
              "|---|" + "---|" * (STEPS // EVERY)]
    for g in GROUPS:
        for s in SEEDS:
            cells = [str(score([r for r in val if r["checkpoint"] == p.stem])[0]) for _, p in checkpoints(g, s)]
            lines.append(f"| {run_name(g, s)} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main() -> None:
    if sys.argv[1:] == ["report"]:
        t = report()
        (OUT / "tables.md").write_text(t, encoding="utf-8")
        print(t)
        return
    train_all()
    vjobs = [(str(p), g, c) for g in GROUPS for s in SEEDS for _, p in checkpoints(g, s) for c in VAL_CASES]
    val = evaluate(vjobs, OUT / "validation.json")
    sel = select(val)
    (OUT / "selection.json").write_text(json.dumps(sel, indent=1, default=float))
    print(json.dumps(sel, indent=1, default=float), flush=True)
    stems = {(x["checkpoint"], g) for g, x in sel.items()}
    stems |= {(f"{run_name(g, s)}_{STEPS // 1000}k", g) for g in GROUPS for s in SEEDS}
    tjobs = [(str(OUT / f"{st}.zip"), g, c) for st, g in sorted(stems) for c in TEST_CASES]
    evaluate(tjobs, OUT / "test.json")
    t = report()
    (OUT / "tables.md").write_text(t, encoding="utf-8")
    print(t, flush=True)


if __name__ == "__main__":
    main()
