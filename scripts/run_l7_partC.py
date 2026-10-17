"""L7 Part C: evaluation of the trained checkpoints on the test sets T1 + T2 (105 laps) –
docs/L7_protocol.md sections 5–8, amendments A7, A8. Run with .venv-rl from a pinned worktree;
T1 / T2 are run once.

    python scripts/run_l7_partC.py select --models <dir>                       # validation-selected checkpoint per group
    python scripts/run_l7_partC.py jobs   --models <dir>                       # the job list (counts)
    python scripts/run_l7_partC.py rl --condition none|main --models <dir>     # RL loop on T1 + T2

``--models``: the training output (``results/L7/partC`` of the training worktree), with one
directory per run (``P_s1`` ... ``F_s5``) holding the checkpoints ``<run>_<k>k.zip`` and
``validation.jsonl``.

Policies: the ten final checkpoints (``P_s1_600k`` ... ``F_s5_600k``; S1, S3, section 6) and the
validation-selected checkpoint of each group (secondary; only evaluated separately when it is
not a final checkpoint). Selection as in L5, per group among its 5 x 12 checkpoints: most
validation laps V completed (of 40), ties by the lower mean lap time of the completed V laps,
then the later checkpoint, then the lower seed. ``select`` writes ``selection.json`` from V
only, before any test lap is run.

Conditions: ``none``; ``main`` = the NMPC filter (N = 30) with the main estimate (b), as in Part B
(same code path: ``f1race.experiments.l7``), including the snapshots for the false
interventions (A8). The bare tracker and the model-based baselines are not re-run: they do not
depend on the training, and Part B's runs of them on the same laps are used (the evaluation is
deterministic; Part B step 2, consistency checks). Records: <out>/rl_<condition>.jsonl, per-lap
decision logs <out>/dec/<condition>/<policy>__<case>.npz.
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
from run_l7_partB import pool_run, test_cases            # noqa: E402

OUT = ROOT / "results" / "L7" / "partC_eval"
GROUPS = ("P", "F")
SEEDS = (1, 2, 3, 4, 5)
FINAL_K = 600
RUNS = [f"{g}_s{s}" for g in GROUPS for s in SEEDS]
FINALS = [f"{r}_{FINAL_K}k" for r in RUNS]
CONDITIONS = ("none", "main")
MODELS_ENV = "F1_L7_PARTC_MODELS"
SET_ENV = "F1_L7_PARTC_SET"


def eval_cases() -> list[dict]:
    """T1 + T2, or the set named in F1_L7_PARTC_SET (X for a code check)."""
    if os.environ.get(SET_ENV):
        from f1race.experiments.l7 import cases

        return cases(os.environ[SET_ENV])
    return test_cases()


def run_of(checkpoint: str) -> str:
    return checkpoint.rsplit("_", 1)[0]


def checkpoint_path(models: Path, checkpoint: str) -> Path:
    return models / run_of(checkpoint) / f"{checkpoint}.zip"


def select(models: Path) -> dict:
    """The validation-selected checkpoint of each group (L5 rule, see the module docstring)."""
    out = {}
    for g in GROUPS:
        cand = []
        for s in SEEDS:
            f = models / f"{g}_s{s}" / "validation.jsonl"
            rs = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
            by = {}
            for r in rs:
                by.setdefault(r["checkpoint"], []).append(r)
            for ck, v in by.items():
                done = [r["lap_time"] for r in v if r["completed"]]
                k = int(ck.rsplit("_", 1)[1][:-1])
                cand.append({"checkpoint": ck, "seed": s, "steps_k": k, "val_laps": len(v), "val_completed": len(done),
                             "val_lap_mean": float(np.mean(done)) if done else float("inf")})
        cand.sort(key=lambda c: (-c["val_completed"], c["val_lap_mean"], -c["steps_k"], c["seed"]))
        out[g] = {**cand[0], "candidates": len(cand)}
    return out


def policies(models: Path, out: Path) -> list[str]:
    sel = json.loads((out / "selection.json").read_text())
    extra = [v["checkpoint"] for v in sel.values() if v["checkpoint"] not in FINALS]
    return FINALS + extra


def job_rl(args):
    policy_name, condition, case_name = args
    from stable_baselines3 import SAC

    from f1race.dynamics import DynVehicle
    from f1race.experiments.l7 import NMPCFilter, make_env, plan, rollout_unfiltered, run_lap
    from f1race.track import Track

    models = Path(os.environ[MODELS_ENV])
    out = Path(os.environ["F1_L7_PARTC_OUT"])
    case = next(c for c in eval_cases() if c["name"] == case_name)
    track = Track.load("Spielberg")
    model = SAC.load(checkpoint_path(models, policy_name), device="cpu")
    pol = lambda obs: model.predict(obs, deterministic=True)[0]              # noqa: E731
    env = make_env("angle")
    filt = None
    if condition == "main":
        est = json.loads((ROOT / "results" / "L7" / "K" / "estimates.json").read_text())["b"]
        filt = NMPCFilter(track, DynVehicle(), plan(0.95), eta=est["eta"], backoff=est["backoff"], record=False)
    t0 = time.time()
    L = track.length
    r = run_lap(pol, env, case, filt, snap_at=tuple(j * L / 8 for j in range(8)) if filt is not None else ())
    rec = {"job": f"{policy_name}|{condition}|{case_name}", "policy": policy_name, "run": run_of(policy_name),
           "group": policy_name[0], "condition": condition, "case": case_name, "set": case["set"], "car": case["car"],
           "tau": case["tau"], "dist_seed": case["dist_seed"], "completed": r["completed"], "lap_time": r["lap_time"],
           "stop_reason": r["stop_reason"], "distance": r["distance"], "wall_s": time.time() - t0}
    d = r["decisions"]
    if len(d["t"]):
        rec.update({"decisions": int(len(d["t"])), "interventions": int(d["changed"].sum()),
                    "filter_failures": int(d["fail"].sum()),
                    "slack_decisions": int(((d["slack"] > 1e-3) | (d["limit_slack"] > 1e-3)).sum()),
                    "time_ms": [float(np.mean(d["time"]) * 1e3), float(np.percentile(d["time"], 95) * 1e3),
                                float(np.percentile(d["time"], 99) * 1e3), float(np.max(d["time"]) * 1e3)]})
        dd = out / "dec" / condition
        dd.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(dd / f"{policy_name}__{case_name}.npz", **{k: np.asarray(v) for k, v in d.items()})
        env2 = make_env("angle")
        rec["false_interventions"] = []
        for sn in r["snapshots"]:
            ro = rollout_unfiltered(pol, env2, case, sn["snap"])
            rec["false_interventions"].append({"at": sn["at"], "progress": sn["progress"], **ro})
    return rec


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["select", "jobs", "rl"])
    ap.add_argument("--models", default=str(ROOT / "results" / "L7" / "partC"))
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--condition", default="none", choices=CONDITIONS)
    ap.add_argument("--policies", default=None, help="comma-separated checkpoints (default: finals + selected)")
    ap.add_argument("--set", default=None, help="evaluate on this set instead of T1 + T2 (X for a code check)")
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    models, out = Path(a.models).resolve(), Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if a.what == "select":
        sel = select(models)
        (out / "selection.json").write_text(json.dumps(sel, indent=1))
        print(json.dumps(sel, indent=1))
        return
    pols = a.policies.split(",") if a.policies else policies(models, out)
    missing = [p for p in pols if not checkpoint_path(models, p).exists()]
    if missing:
        raise SystemExit(f"missing checkpoints: {missing}")
    if a.set:
        os.environ[SET_ENV] = a.set                                          # code check only
    names = [c["name"] for c in eval_cases()]
    if a.what == "jobs":
        print(f"policies ({len(pols)}): {', '.join(pols)}")
        print(f"cases: {len(names)} (T1 {sum(n.startswith('T1') for n in names)}, T2 {sum(n.startswith('T2') for n in names)})")
        for c in CONDITIONS:
            print(f"rl --condition {c}: {len(pols)} x {len(names)} = {len(pols) * len(names)} laps")
        return
    os.environ[MODELS_ENV] = str(models)
    os.environ["F1_L7_PARTC_OUT"] = str(out)
    pool_run(job_rl, [(p, a.condition, n) for p in pols for n in names], out / f"rl_{a.condition}.jsonl", a.workers)


if __name__ == "__main__":
    main()
