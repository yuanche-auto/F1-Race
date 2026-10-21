"""Control expansion phase A runs (docs/CX_A_protocol.md, revision 1). Windows `.venv`; one fresh process
per job (spawn pool, maxtasksperchild=1, as L7 / L8), single-threaded BLAS. Benchmark stages are run from
a pinned worktree (roadmap section 1); records in results/CX/A/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_a.py jobs                                  # the job lists (counts)
    python scripts/run_cx_a.py tune  [--workers 20]                  # CXV: 24 mpcc, 24 mpcc_centre, 6 nmpc settings
    python scripts/report_cx_a.py tune                               # the rule of section 4 -> results/CX/A/selected.json
    python scripts/run_cx_a.py test  [--workers 20]                  # CXT + NOM with the selected settings, + tracker
    python scripts/run_cx_a.py smoke [--kinds ...]                   # code check on the nominal car only (results/CX/A/smoke/)

``test`` refuses to start without selected.json, and records the selected settings and the SHA-256 of the
tuning records it was chosen from.
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
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from f1race.experiments.cx_a import KINDS, cases, grid, setting_key  # noqa: E402

OUT = ROOT / "results" / "CX" / "A"
SMOKE_SETTINGS = {"tracker": {}, "nmpc": {"c": 1.0, "margin": 0.3},
                  "mpcc": {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}, "mpcc_centre": {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}}


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def job(args):
    """One lap: (stage, kind, setting key, case name[, setting json])."""
    from f1race.experiments.cx_a import lap

    stage, kind, key, case_name, setting_json = args
    setting = json.loads(setting_json)
    case = next(c for set_ in ("CXV", "CXT", "NOM") if case_name.startswith(set_)
                for c in cases(set_) if c["name"] == case_name)
    t0 = time.time()
    out = lap(case, kind, setting)
    out.update({"job": "|".join([stage, kind, key, case_name]), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def tune_jobs() -> list[tuple]:
    names = [c["name"] for c in cases("CXV")]
    return [("tune", k, setting_key(s), n, json.dumps(s)) for k in ("nmpc", "mpcc", "mpcc_centre")
            for s in grid(k) for n in names]


def test_jobs(selected: dict) -> list[tuple]:
    names = [c["name"] for c in cases("CXT")] + [c["name"] for c in cases("NOM")]
    out = []
    for k in KINDS:
        s = {} if k == "tracker" else selected[k]["setting"]
        out += [("test", k, setting_key(s), n, json.dumps(s)) for n in names]
    return out


def _run(jobs: list[tuple], path: Path, workers: int) -> None:
    """One fresh process per job (spawn, maxtasksperchild=1); resumable: jobs with a record that is not
    an error are skipped (an error record stays in the file)."""
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
    ap.add_argument("what", choices=["jobs", "tune", "test", "smoke"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--kinds", default=",".join(KINDS))
    a = ap.parse_args()
    if a.what == "jobs":
        tj = tune_jobs()
        print(f"tune: {len(tj)} laps (CXV 10 cars x (6 nmpc + 24 mpcc + 24 mpcc_centre settings))")
        print(f"test: {len(KINDS)} controllers x (25 CXT + 1 NOM) = {len(KINDS) * 26} laps")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what == "test":
        sel_path = OUT / "selected.json"
        if not sel_path.exists():
            raise SystemExit("no results/CX/A/selected.json - run report_cx_a.py tune (and have it reviewed) first")
        sel = json.loads(sel_path.read_text())
        tune_sha = hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest()
        if sel.get("tune_sha256") != tune_sha:
            raise SystemExit("selected.json was not made from the present tune.jsonl")
        _run(test_jobs(sel["kinds"]), OUT / "test.jsonl", a.workers)
    else:
        kinds = a.kinds.split(",")
        jobs = [("smoke", k, setting_key(SMOKE_SETTINGS[k]), "NOM_car0", json.dumps(SMOKE_SETTINGS[k])) for k in kinds]
        _run(jobs, OUT / "smoke" / "smoke.jsonl", a.workers)


if __name__ == "__main__":
    main()
