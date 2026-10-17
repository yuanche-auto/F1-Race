"""L7 Part B: evaluation on the test sets T1 (75 laps) and T2 (30 laps) – docs/L7_protocol.md
section 4.3 and 6, amendments A3–A7. Run with .venv-rl from a pinned worktree; T1 / T2 are
run once.

    python scripts/run_l7_partB.py rl --condition none|main|cbf [--policies ...]   # RL loop (policies + bare tracker)
    python scripts/run_l7_partB.py baseline --kind nmpc|smpc|lag_identified          # model-based baselines, no filter
    python scripts/run_l7_partB.py jobs                                              # the job list (counts)

Policies: the nine final L5 checkpoints (``results/L5/multiseed/<g>_s<k>.zip``), the selected
``B_s1_500k`` and ``tracker`` (the bare tracker, no policy). Conditions: ``none``; ``main`` =
the NMPC filter (N = 30) with the main estimate (b) (``results/L7/K/estimates.json``);
``cbf`` = the CBF-QP with alpha = 8 (the alpha rule, A6) and margin / friction factor b_10 /
eta_10 of (b). With a filter, the episode state is kept at the first intervention at or after
each j L / 8 (j = 0 ... 7) and the unfiltered policy is rolled out from it for 3 s (false
interventions, section 6 and A8: at most eight per lap; the report draws 200 per policy
uniformly with seed 0 and gives the rate per eighth of the lap). Records: results/L7/partB/rl_<condition>.jsonl, per-lap decision logs
results/L7/partB/dec/<condition>/<policy>__<case>.npz (for the attribution of failures,
pb-m2); baselines.jsonl.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from multiprocessing import get_context
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "L7" / "partB"
FINALS = [f"{g}_s{s}" for g in "ABC" for s in (1, 2, 3)]
POLICIES = FINALS + ["B_s1_500k", "tracker"]
CONDITIONS = ("none", "main", "cbf")
KINDS = ("nmpc", "smpc", "lag_identified")
CBF_ALPHA = 8.0
NL = chr(10)


def policy_path(name: str) -> Path:
    return ROOT / "models" / "L5" / f"{name}.zip" if name == "B_s1_500k" else ROOT / "results" / "L5" / "multiseed" / f"{name}.zip"


def steer_mode(name: str) -> str:
    return "lateral" if name.startswith("C_") else "angle"


def test_cases():
    from f1race.experiments.l7 import cases

    return cases("T1") + cases("T2")


def job_rl(args):
    policy_name, condition, case_name = args
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l7 import CBFFilter, NMPCFilter, make_env, plan, rollout_unfiltered, run_lap
    from f1race.track import Track

    case = next(c for c in test_cases() if c["name"] == case_name)
    track = Track.load("Spielberg")
    if policy_name == "tracker":
        pol = None
    else:
        from stable_baselines3 import SAC

        model = SAC.load(policy_path(policy_name), device="cpu")
        pol = lambda obs: model.predict(obs, deterministic=True)[0]          # noqa: E731
    env = make_env(steer_mode(policy_name))
    filt = None
    est = json.loads((ROOT / "results" / "L7" / "K" / "estimates.json").read_text())["b"]
    if condition == "main":
        filt = NMPCFilter(track, DynVehicle(), plan(0.95), eta=est["eta"], backoff=est["backoff"], record=False)
    elif condition == "cbf":
        filt = CBFFilter(track, DynVehicle(), alpha=CBF_ALPHA, margin=est["backoff"][10], eta=est["eta"][10])
    t0 = time.time()
    L = track.length
    r = run_lap(pol, env, case, filt, snap_at=tuple(j * L / 8 for j in range(8)) if filt is not None else ())
    rec = {"job": f"{policy_name}|{condition}|{case_name}", "policy": policy_name, "condition": condition,
           "case": case_name, "set": case["set"], "car": case["car"], "tau": case["tau"],
           "dist_seed": case["dist_seed"], "completed": r["completed"], "lap_time": r["lap_time"],
           "stop_reason": r["stop_reason"], "distance": r["distance"], "wall_s": time.time() - t0}
    d = r["decisions"]
    if len(d["t"]):
        rec.update({"decisions": int(len(d["t"])), "interventions": int(d["changed"].sum()),
                    "filter_failures": int(d["fail"].sum()),
                    "slack_decisions": int(((d["slack"] > 1e-3) | (d["limit_slack"] > 1e-3)).sum()),
                    "time_ms": [float(np.mean(d["time"]) * 1e3), float(np.percentile(d["time"], 95) * 1e3),
                                float(np.percentile(d["time"], 99) * 1e3), float(np.max(d["time"]) * 1e3)]})
        dd = OUT / "dec" / condition
        dd.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(dd / f"{policy_name}__{case_name}.npz", **{k: np.asarray(v) for k, v in d.items()})
        env2 = make_env(steer_mode(policy_name))
        rec["false_interventions"] = []
        for sn in r["snapshots"]:
            ro = rollout_unfiltered(pol, env2, case, sn["snap"])
            rec["false_interventions"].append({"at": sn["at"], "progress": sn["progress"], **ro})
    return rec


def job_baseline(args):
    kind, case_name = args
    from f1race.experiments.l7 import drive_baseline
    from f1race.track import Track

    case = next(c for c in test_cases() if c["name"] == case_name)
    t0 = time.time()
    out = drive_baseline(case, kind, Track.load("Spielberg"))
    out.update({"job": f"{kind}|{case_name}", "kind": kind, "case": case_name, "set": case["set"],
                "car": case["car"], "tau": case["tau"], "dist_seed": case["dist_seed"], "wall_s": time.time() - t0})
    return out


def safe(args):
    fn, a = args
    try:
        return fn(a)
    except Exception:                                       # noqa: BLE001
        return {"job": "|".join(map(str, a)), "error": traceback.format_exc()}


def pool_run(fn, jobs, path: Path, workers: int):
    path.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if path.exists():
        done = {json.loads(l)["job"] for l in path.read_text().splitlines() if l.strip() and '"error"' not in l}
    todo = [j for j in jobs if "|".join(map(str, j)) not in done]
    print(f"{len(todo)} jobs -> {path}", flush=True)
    if not todo:
        return
    with get_context("spawn").Pool(min(workers, len(todo)), maxtasksperchild=1) as pool, \
            open(path, "a", encoding="utf-8") as fo:
        for rec in pool.imap_unordered(safe, [(fn, j) for j in todo]):
            fo.write(json.dumps(rec, default=float) + NL)
            fo.flush()
            print(rec.get("job"), rec.get("stop_reason"), "error" if "error" in rec else "", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["rl", "baseline", "jobs"])
    ap.add_argument("--condition", default="none", choices=CONDITIONS)
    ap.add_argument("--kind", default="nmpc", choices=KINDS)
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    names = [c["name"] for c in test_cases()]
    if a.what == "jobs":
        print(f"cases: {len(names)} (T1 {sum(n.startswith('T1') for n in names)}, T2 {sum(n.startswith('T2') for n in names)})")
        print(f"RL loop: {len(POLICIES)} policies x {len(CONDITIONS)} conditions x {len(names)} = "
              f"{len(POLICIES) * len(CONDITIONS) * len(names)} laps")
        print(f"baselines: {len(KINDS)} x {len(names)} = {len(KINDS) * len(names)} laps (no filter)")
        return
    if a.what == "rl":
        pols = a.policies.split(",")
        jobs = [(p, a.condition, n) for p in pols for n in names]
        pool_run(job_rl, jobs, OUT / f"rl_{a.condition}.jsonl", a.workers)
    else:
        pool_run(job_baseline, [(a.kind, n) for n in names], OUT / "baselines.jsonl", a.workers)


if __name__ == "__main__":
    main()
