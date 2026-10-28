"""Control expansion phase H runs (docs/CX_H_protocol.md, revision 0 with its clarifications m1–m3, n1, n2). Windows
`.venv`; one fresh process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. A job of `atv` / `atv_nominal`
/ `lmpc` is one car's whole sequence (two initialisation laps and ten learning laps); `nmpc` / `identified` one lap.
Benchmark stages run from a pinned worktree; records in results/CX/H/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_h.py jobs                       # the job lists (counts)
    python scripts/run_cx_h.py dev [--workers 20]         # section 9.3: atv, atv_nominal (K 20), nmpc, identified on
                                                          # NOM + DEV -> results/CX/H/dev/laps.jsonl
    python scripts/run_cx_h.py tune [--workers 20]        # HV: atv with K in {10, 20, 40} (30 sequences)
    python scripts/report_cx_h.py tune                    # the rule of section 4 -> results/CX/H/selected.json
    python scripts/run_cx_h.py test [--workers 20] [--id-workers 6]      # HT + NOM: nmpc, identified, atv, atv_nominal
    python scripts/run_cx_h.py secondary [--workers 20]   # after the primary report: lmpc on HT; HT_LAG30, HT_WET85

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
from f1race.experiments import cx_h  # noqa: E402

OUT = ROOT / "results" / "CX" / "H"


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def job(args):
    stage, kind, key, case_name, setting_json = args
    t0 = time.time()
    out = cx_h.run(cx_h.case_by_name(case_name), kind, json.loads(setting_json))
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str]) -> list[tuple]:
    return [(stage, kind, cx_h.setting_key(setting), n, json.dumps(setting)) for n in names]


def _names(*sets: str) -> list[str]:
    return [c["name"] for s_ in sets for c in cx_h.cases(s_)]


def dev_jobs() -> list[tuple]:
    names = _names("NOM", "DEV")
    j = _jobs("dev", "atv", cx_h.DEV_SETTING, names) + _jobs("dev", "atv_nominal", cx_h.DEV_SETTING, names)
    return j + _jobs("dev", "nmpc", {}, names) + _jobs("dev", "identified", {}, names)


def tune_jobs() -> list[tuple]:
    return [j for s_ in cx_h.K_GRID for j in _jobs("tune", "atv", s_, _names("HV"))]


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/H/selected.json - run report_cx_h.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest():
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def settings(sel: dict) -> dict:
    k = sel["kinds"]["atv"]["setting"]
    return {"nmpc": {}, "identified": {}, "atv": k, "atv_nominal": dict(k), "lmpc": dict(k)}


def test_jobs(st: dict) -> list[tuple]:
    return [j for k in cx_h.PRIMARY_KINDS for j in _jobs("test", k, st[k], _names("HT", "NOM"))]


def secondary_jobs(st: dict) -> list[tuple]:
    j = _jobs("secondary", "lmpc", st["lmpc"], _names("HT"))
    return j + [jj for k in cx_h.PRIMARY_KINDS for jj in _jobs("secondary", k, st[k], _names(*cx_h.SECONDARY))]


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
    ap.add_argument("what", choices=["jobs", "dev", "tune", "test", "secondary"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--id-workers", type=int, default=6)
    a = ap.parse_args()
    if a.what == "jobs":
        st = settings({"kinds": {"atv": {"setting": {"K": 20}}}})
        print(f"dev: {len(dev_jobs())} jobs; tune: {len(tune_jobs())} sequences; test: {len(test_jobs(st))} jobs; "
              f"secondary: {len(secondary_jobs(st))} jobs")
        return
    if a.what == "dev":
        os.environ["L6_REPLAN_SLOTS"] = str(a.id_workers)
        jobs = dev_jobs()
        _run([j for j in jobs if j[1] != "identified"], OUT / "dev" / "laps.jsonl", a.workers)
        _run([j for j in jobs if j[1] == "identified"], OUT / "dev" / "laps.jsonl", a.id_workers)
    elif a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    else:
        st = settings(_selected())
        if a.what == "secondary" and not (OUT / "report" / "CX_H.md").exists():
            raise SystemExit("the secondary arms run after the primary report (protocol section 4)")
        jobs = test_jobs(st) if a.what == "test" else secondary_jobs(st)
        os.environ["L6_REPLAN_SLOTS"] = str(a.id_workers)
        path = OUT / f"{a.what}.jsonl"
        _run([j for j in jobs if j[1] != "identified"], path, a.workers)
        _run([j for j in jobs if j[1] == "identified"], path, a.id_workers)


if __name__ == "__main__":
    main()
