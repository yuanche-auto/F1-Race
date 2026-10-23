"""Control expansion phase F runs (docs/CX_F_protocol.md, revision 0 with its clarifications m1–m3, n1, n2).
`.venv-rl` (torch, SB3, gymnasium – as L7); one fresh process per job (spawn pool, maxtasksperchild=1),
single-threaded BLAS. Benchmark stages run from a pinned worktree; records in results/CX/F/
(key ``job`` = stage|condition|proposer|case|setting), per-lap decision records in results/CX/F/<stage>_dec/.

    python scripts/run_cx_f.py jobs                                # the job lists (counts)
    python scripts/run_cx_f.py tune [--workers 20]                 # FV: shield m in {0.25, 0.5, 1.0}, none, backup
    python scripts/report_cx_f.py tune                             # the rule of section 4 -> results/CX/F/selected.json
    python scripts/run_cx_f.py test --conditions none,main_repro,retry [--workers 20]   # amendment F1: what runs
    python scripts/run_cx_f.py test --conditions main_full   # only if the reproduction check of main fails (section 3)

**Amendment F1 (option 1):** the shield part of phase F was closed without the benchmark – the `tune` stage and the
`shield` / `backup` conditions are development only and were never run on FV, T1 or T2.

Conditions (section 3): ``none`` (no filter), ``main_repro`` (L7's main filter on the 22 laps of the reproduction
check, with the command log), ``retry`` (L7's main filter with retried solves), ``shield`` (section 3.1), ``backup``
(the backup action at every decision; proposer ``backup``). With a filter (main, retry, shield) the episode state is
kept at the first intervention at or after each j L / 8 and the unfiltered proposer is rolled out from it for 3 s
(false interventions, L7's A8 – review n2).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
import traceback
from multiprocessing import get_context
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "CX" / "F"
L7_EST = ROOT / "results" / "L7" / "K" / "estimates.json"


def _key(j: tuple) -> str:
    return "|".join(j[:5])


def tag(setting: dict) -> str:
    """Directory name of a setting's decision records: e.g. ``m0.5``; ``default`` without one."""
    return "_".join(f"{k}{v}" for k, v in sorted(setting.items())) or "default"


def repro_subset() -> list[tuple[str, str]]:
    """Section 3: two of the 105 T1 + T2 cases per proposer (order of PROPOSERS), rng 0."""
    from f1race.experiments import cx_f

    names = [c["name"] for c in cx_f.cases("T1") + cx_f.cases("T2")]
    rng = np.random.default_rng(0)
    return [(p, names[i]) for p in cx_f.PROPOSERS for i in rng.choice(len(names), 2, replace=False)]


def _filter(condition: str, track):
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l7 import NMPCFilter, plan

    est = json.loads(L7_EST.read_text())["b"]
    return NMPCFilter(track, DynVehicle(), plan(0.95), eta=est["eta"], backoff=est["backoff"], record=False,
                      retry=condition == "retry")


def _retry_stats(mpc) -> dict:
    """Per solve of the lap: first attempt failed; recovered by retry 1 / 2; still failed (fallback)."""
    first_fail = rec1 = rec2 = fell = 0
    for s in mpc.stats:
        att = s.get("attempts", [])
        if len(att) > 1:
            first_fail += 1
            if s["fallback"]:
                fell += 1
            elif len(att) == 2:
                rec1 += 1
            else:
                rec2 += 1
    return {"first_attempt_failed": first_fail, "recovered_retry1": rec1, "recovered_retry2": rec2,
            "fallbacks": fell}


def job(args):
    stage, condition, proposer, case_name, setting_json = args
    from f1race.experiments import cx_f, l7
    from f1race.track import Track

    t0 = time.time()
    setting = json.loads(setting_json)
    case = cx_f.case_by_name(case_name)
    track = Track.load("Spielberg")
    L = track.length
    if proposer == "backup":
        pol, mode = (lambda obs: np.array(cx_f.A_B)), "angle"
    else:
        pol, mode = cx_f.load_policy(proposer), cx_f.steer_mode(proposer)
    env = l7.make_env(mode)
    snap_at = tuple(j * L / 8 for j in range(8))
    filt = None
    if condition in ("main_repro", "main_full", "retry"):
        filt = _filter(condition, track)
        r = l7.run_lap(pol, env, case, filt, snap_at=snap_at, log_commands=True)
    elif condition == "shield":
        sh = cx_f.Shield(env, cx_f.ShieldConfig(margin=float(setting["m"]), backup_eta=setting.get("eta_b")))
        r = cx_f.run_lap_shield(pol, env, case, sh, snap_at=snap_at)
    elif condition == "backup":
        r = l7.run_lap(pol, env, case, None, log_commands=True)
    else:                                                   # none
        r = l7.run_lap(pol, env, case, None)
    rec = {"job": _key(args), "stage": stage, "condition": condition, "policy": proposer, "case": case_name,
           "setting": setting, "set": case["set"], "car": case["car"], "tau": case["tau"],
           "dist_seed": case["dist_seed"], "completed": r["completed"], "lap_time": r["lap_time"],
           "stop_reason": r["stop_reason"], "distance": r["distance"]}
    d = r["decisions"]
    if len(d["t"]):
        a = cx_f.lap_audit(d, r["completed"], L)
        rec.update({k: a[k] for k in ("decisions", "interventions", "on_switches", "longest_run",
                                      "interventions_before_failure", "time_over_50ms")})
        tm = np.asarray(d["time"], float)
        rec["time_ms"] = [float(tm.mean() * 1e3), float(np.percentile(tm, 95) * 1e3),
                          float(np.percentile(tm, 99) * 1e3), float(tm.max() * 1e3)]
        if "fail" in d and len(d["fail"]):
            rec["filter_failures"] = int(np.asarray(d["fail"], bool).sum())
            rec["slack_decisions"] = int(((np.asarray(d["slack"]) > 1e-3) | (np.asarray(d["limit_slack"]) > 1e-3)).sum())
        if "no_option" in d:
            rec["no_option"] = int(np.asarray(d["no_option"], bool).sum())
            rec["reasons"] = {k: int(np.sum(np.asarray(d["reason"]) == k)) for k in ("margin", "spin", "not settled")}
    if len(d["t"]) or len(d.get("u_appl", [])):
        dd = OUT / f"{stage}_dec" / condition / tag(setting)
        dd.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(dd / f"{proposer}__{case_name}.npz",
                            **{k: (np.asarray(v).astype(str) if k == "reason" else np.asarray(v)) for k, v in d.items()})
    if condition == "retry":
        rec["retry"] = _retry_stats(filt.mpc)
    if filt is not None or condition == "shield":
        env2 = l7.make_env(mode)
        rec["false_interventions"] = []
        for sn in r["snapshots"]:
            ro = l7.rollout_unfiltered(pol, env2, case, sn["snap"])
            rec["false_interventions"].append({"at": sn["at"], "progress": sn["progress"], **ro})
    rec["wall_s"] = time.time() - t0
    return rec


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage, condition, proposers, names, setting: dict) -> list[tuple]:
    sj = json.dumps(setting, sort_keys=True)
    return [(stage, condition, p, n, sj) for p in proposers for n in names]


def tune_jobs() -> list[tuple]:
    from f1race.experiments import cx_f

    names = [c["name"] for c in cx_f.cases("FV")]
    j = []
    for m in cx_f.M_GRID:
        j += _jobs("tune", "shield", cx_f.FINALS, names, {"m": m})
    j += _jobs("tune", "none", cx_f.FINALS, names, {})
    j += _jobs("tune", "backup", ["backup"], names, {})
    return j


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/F/selected.json - run report_cx_f.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest():
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def test_jobs(conditions: list[str]) -> list[tuple]:
    from f1race.experiments import cx_f

    names = [c["name"] for c in cx_f.cases("T1") + cx_f.cases("T2")]
    j = []
    for c in conditions:
        if c == "none":
            j += _jobs("test", "none", cx_f.PROPOSERS, names, {})
        elif c == "retry":
            j += _jobs("test", "retry", cx_f.PROPOSERS, names, {})
        elif c == "backup":
            j += _jobs("test", "backup", ["backup"], names, {})
        elif c == "main_repro":
            j += [("test", "main_repro", p, n, "{}") for p, n in repro_subset()]
        elif c == "main_full":
            j += _jobs("test", "main_full", cx_f.PROPOSERS, names, {})
        elif c == "shield":
            j += _jobs("test", "shield", cx_f.PROPOSERS, names, {"m": _selected()["m"]})
        else:
            raise SystemExit(f"unknown condition {c}")
    return j


def _run(jobs: list[tuple], path: Path, workers: int) -> None:
    """One fresh process per job; resumable (a job with a record that is not an error is skipped)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if path.exists():
        for l in path.read_text().splitlines():
            if l.strip():
                r = json.loads(l)
                if "error" not in r:
                    done.add(r["job"])
    todo = [j for j in jobs if _key(j) not in done]
    print(f"{len(todo)} jobs -> {path}", flush=True)
    if not todo:
        return
    with get_context("spawn").Pool(min(workers, len(todo)), maxtasksperchild=1) as pool, \
            open(path, "a", encoding="utf-8") as fo:
        for rec in pool.imap_unordered(safe, todo):
            fo.write(json.dumps(rec, default=float) + "\n")
            fo.flush()
            print(rec["job"], rec.get("stop_reason", ""), "error" if "error" in rec else "", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["jobs", "tune", "test"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--conditions", default="none,main_repro,retry")
    a = ap.parse_args()
    if a.what == "jobs":
        tj = tune_jobs()
        print(f"tune: {len(tj)} laps (shield {sum(j[1] == 'shield' for j in tj)}, none {sum(j[1] == 'none' for j in tj)}, "
              f"backup {sum(j[1] == 'backup' for j in tj)})")
        tt = test_jobs(["none", "main_repro", "retry"])
        print("test (amendment F1): " + ", ".join(f"{c} {sum(j[1] == c for j in tt)}" for c in ("none", "main_repro", "retry"))
              + " laps; main_full 1155 only if the reproduction check fails")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    else:
        _run(test_jobs(a.conditions.split(",")), OUT / "test.jsonl", a.workers)


if __name__ == "__main__":
    main()
