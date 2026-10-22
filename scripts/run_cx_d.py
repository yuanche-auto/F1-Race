"""Control expansion phase D runs (docs/CX_D_protocol.md, revision 0 with its clarifications). Windows `.venv`; one
fresh process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. Benchmark stages run from a pinned
worktree (roadmap section 1); records in results/CX/D/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_d.py jobs                        # the job lists (counts)
    python scripts/run_cx_d.py tune [--workers 20]         # CDV: adaptive (lam_mu) and adaptive_bank (W), 120 laps
    python scripts/report_cx_d.py tune                     # the rule of section 4 -> results/CX/D/selected.json
    python scripts/run_cx_d.py test [--workers 20] [--id-workers 6]
                                                           # CDT + NOM and CDD: the 7 controllers (707 laps)
    python scripts/run_cx_d.py secondary [--workers 20]    # CDT_NOISE, CDT_LAG30, CDT_WET85, CDP, CXD (840 laps)
    python scripts/run_cx_d.py smoke [--kinds ...]         # code check on the nominal car only (results/CX/D/smoke/)

``test`` and ``secondary`` refuse to start without selected.json made from the present tune.jsonl (SHA-256).
``identified`` jobs run in their own pool with at most ``--id-workers`` processes (its re-plan needs ~7.4 GB; the
same number of re-plan slots, L6's ``replan_slot``), CDT before CDD (CDD reuses each car's cached identification).
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
from f1race.experiments import cx_d  # noqa: E402

OUT = ROOT / "results" / "CX" / "D"


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def _case(name: str) -> dict:
    return next(c for c in cx_d.cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def job(args):
    """One lap: (stage, kind, setting key, case name, setting json)."""
    stage, kind, key, case_name, setting_json = args
    t0 = time.time()
    out = cx_d.lap(_case(case_name), kind, json.loads(setting_json))
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str]) -> list[tuple]:
    return [(stage, kind, cx_d.setting_key(setting), n, json.dumps(setting)) for n in names]


def _names(*sets: str) -> list[str]:
    return [c["name"] for s_ in sets for c in cx_d.cases(s_)]


def tune_jobs() -> list[tuple]:
    return [j for k, g in cx_d.GRIDS.items() for s_ in g for j in _jobs("tune", k, s_, _names("CDV"))]


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/D/selected.json - run report_cx_d.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest():
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def settings(sel: dict) -> dict:
    lam = sel["kinds"]["adaptive"]["setting"]
    return {"nmpc": {}, "tube": {}, "identified": {}, "adaptive_oracle": {}, "adaptive": lam,
            "adaptive_bank": sel["kinds"]["adaptive_bank"]["setting"], "nmpc_param": dict(lam)}


def test_jobs(st: dict) -> list[tuple]:
    return [j for k in cx_d.KINDS for j in _jobs("test", k, st[k], _names("CDT", "NOM", "CDD"))]


def secondary_jobs(st: dict) -> list[tuple]:
    return [j for k in cx_d.SECONDARY_KINDS for j in _jobs("secondary", k, st[k], _names(*cx_d.SECONDARY))]


def _run(jobs: list[tuple], path: Path, workers: int) -> None:
    """One fresh process per job (spawn, maxtasksperchild=1); resumable: jobs with a record that is not an error are
    skipped (an error record stays in the file)."""
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


def _run_test(jobs: list[tuple], path: Path, workers: int, id_workers: int) -> None:
    """The non-identified jobs, then identified on CDT + NOM, then on CDD (which reuses the cached identification)."""
    os.environ["L6_REPLAN_SLOTS"] = str(id_workers)
    _run([j for j in jobs if j[1] != "identified"], path, workers)
    ident = [j for j in jobs if j[1] == "identified"]
    _run([j for j in ident if not j[3].startswith("CDD_")], path, id_workers)
    _run([j for j in ident if j[3].startswith("CDD_")], path, workers)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["jobs", "tune", "test", "secondary", "smoke"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--id-workers", type=int, default=6)
    ap.add_argument("--kinds", default=",".join(cx_d.KINDS))
    a = ap.parse_args()
    if a.what == "jobs":
        st = {k: {} for k in cx_d.KINDS} | {"adaptive": {"lam_mu": 0.995}, "adaptive_bank": {"W": 0.2},
                                           "nmpc_param": {"lam_mu": 0.995}}
        print(f"tune: {len(tune_jobs())} laps (CDV 20 cases x 3 settings x adaptive, adaptive_bank)")
        print(f"test: {len(test_jobs(st))} laps (CDT 50 + NOM + CDD 50 = 101 cases x {len(cx_d.KINDS)} controllers)")
        print(f"secondary: {len(secondary_jobs(st))} laps ({len(cx_d.SECONDARY_KINDS)} controllers x "
              f"{len(_names(*cx_d.SECONDARY))} cases: {', '.join(cx_d.SECONDARY)})")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what == "test":
        _run_test(test_jobs(settings(_selected())), OUT / "test.jsonl", a.workers, a.id_workers)
    elif a.what == "secondary":
        if not (OUT / "report" / "CX_D.md").exists():
            raise SystemExit("the secondary conditions run after the primary report (protocol section 2)")
        _run(secondary_jobs(settings(_selected())), OUT / "secondary.jsonl", a.workers)
    else:
        st = {k: {} for k in cx_d.KINDS} | {"adaptive": {"lam_mu": 0.995}, "adaptive_bank": {"W": 0.2},
                                           "nmpc_param": {"lam_mu": 0.995}}
        os.environ["L6_REPLAN_SLOTS"] = str(a.id_workers)
        jobs = [j for k in a.kinds.split(",") for j in _jobs("smoke", k, st[k], ["NOM_car0"])]
        _run(jobs, OUT / "smoke" / "smoke.jsonl", a.workers)


if __name__ == "__main__":
    main()
