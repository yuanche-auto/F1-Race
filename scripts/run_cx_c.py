"""Control expansion phase C runs (docs/CX_C_protocol.md, revision 0 with its clarifications). Windows `.venv`; one
fresh process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. Benchmark stages run from a pinned
worktree (roadmap section 1); records in results/CX/C/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_c.py jobs                       # the job lists (counts)
    python scripts/run_cx_c.py tune [--workers 20]        # CXV: tube and tube_track, c_R in {0.1, 1, 10}
    python scripts/report_cx_c.py tune                    # the rule of section 4 -> results/CX/C/selected.json (to the reviewer)
    python scripts/run_cx_c.py test [--workers 20]        # CCT + NOM: tracker, nmpc, nmpc_reserve, tube, tube_track
    python scripts/run_cx_c.py secondary [--workers 20]   # CCT_LAG30, CXD, CXL8, after report/CX_C.md
    python scripts/run_cx_c.py smoke [--kinds ...]        # code check on the nominal car only (results/CX/C/smoke/)

``test`` and ``secondary`` refuse to start without selected.json made from the present tune.jsonl (SHA-256).
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
from f1race.experiments import cx_c  # noqa: E402

OUT = ROOT / "results" / "CX" / "C"


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def _case(name: str) -> dict:
    return next(c for c in cx_c.cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def job(args):
    """One lap: (stage, kind, setting key, case name, setting json)."""
    stage, kind, key, case_name, setting_json = args
    t0 = time.time()
    out = cx_c.lap(_case(case_name), kind, json.loads(setting_json))
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str]) -> list[tuple]:
    return [(stage, kind, cx_c.setting_key(setting), n, json.dumps(setting)) for n in names]


def tune_jobs() -> list[tuple]:
    names = [c["name"] for c in cx_c.cases("CXV")]
    return [j for k in cx_c.TUBE_KINDS for s_ in cx_c.TUBE_GRID for j in _jobs("tune", k, s_, names)]


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/C/selected.json - run report_cx_c.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest():
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def settings(sel: dict) -> dict:
    return {**{k: cx_c.fixed_setting(k) for k in ("tracker", "nmpc", "nmpc_reserve")},
            **{k: sel["kinds"][k]["setting"] for k in cx_c.TUBE_KINDS}}


def test_jobs(st: dict) -> list[tuple]:
    names = [c["name"] for c in cx_c.cases("CCT")] + [c["name"] for c in cx_c.cases("NOM")]
    return [j for k in cx_c.KINDS for j in _jobs("test", k, st[k], names)]


def secondary_jobs(st: dict) -> list[tuple]:
    names = [c["name"] for s_ in cx_c.SECONDARY for c in cx_c.cases(s_)]
    return [j for k in cx_c.KINDS for j in _jobs("secondary", k, st[k], names)]


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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["jobs", "tune", "test", "secondary", "smoke"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--kinds", default=",".join(cx_c.KINDS))
    a = ap.parse_args()
    if a.what == "jobs":
        n_cct = len(cx_c.cases("CCT")) + 1
        n2 = sum(len(cx_c.cases(s_)) for s_ in cx_c.SECONDARY)
        print(f"tune: {len(tune_jobs())} laps (CXV 10 cars x 3 c_R x tube, tube_track)")
        print(f"test: {n_cct} cases x {len(cx_c.KINDS)} controllers = {n_cct * len(cx_c.KINDS)} laps")
        print(f"secondary: {len(cx_c.KINDS)} controllers x {n2} laps (CCT_LAG30 50, CXD 30, CXL8 4) = {len(cx_c.KINDS) * n2}")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what == "test":
        _run(test_jobs(settings(_selected())), OUT / "test.jsonl", a.workers)
    elif a.what == "secondary":
        if not (OUT / "report" / "CX_C.md").exists():
            raise SystemExit("the secondary conditions run after the primary report (protocol section 2)")
        _run(secondary_jobs(settings(_selected())), OUT / "secondary.jsonl", a.workers)
    else:
        st = {**{k: cx_c.fixed_setting(k) for k in ("tracker", "nmpc", "nmpc_reserve")},
              **{k: {"c_R": 1.0} for k in cx_c.TUBE_KINDS}}
        jobs = [j for k in a.kinds.split(",") for j in _jobs("smoke", k, st[k], ["NOM_car0"])]
        _run(jobs, OUT / "smoke" / "smoke.jsonl", a.workers)


if __name__ == "__main__":
    main()
