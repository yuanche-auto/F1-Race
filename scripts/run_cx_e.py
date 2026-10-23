"""**Phase E was closed without the benchmark (amendment E1); the tune / test / secondary stages of this runner have never
been run end to end – validate them before use.**
Exception: the `dev` stage (and `_jobs` / `_run`, which `scripts/cx_e_dev.py diag` uses) was run – it produced
results/CX/E/dev/laps.jsonl and diag.jsonl of protocol section 9.

Control expansion phase E runs (docs/CX_E_protocol.md, revision 0 with its clarifications). Windows `.venv`; one fresh
process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. A job of `lmpc` / `lmpc_nominal` is one car's whole
sequence (two initialisation laps and ten learning laps); `nmpc` / `identified` one lap. Benchmark stages run from a
pinned worktree; records in results/CX/E/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_e.py jobs                       # the job lists (counts)
    python scripts/run_cx_e.py tune [--workers 20]        # CEV: lmpc with K in {10, 20, 40} (30 sequences, 360 laps)
    python scripts/report_cx_e.py tune                    # the rule of section 4 -> results/CX/E/selected.json
    python scripts/run_cx_e.py test [--workers 20] [--id-workers 6]   # CET + NOM: nmpc, identified, lmpc, lmpc_nominal
    python scripts/run_cx_e.py secondary [--workers 20]   # CET_LAG30, CET_WET85
    python scripts/run_cx_e.py dev [--workers 20] [--k 20,10,5]   # section 9: NOM and DEV sequences (results/CX/E/dev/)

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
from f1race.experiments import cx_e  # noqa: E402

OUT = ROOT / "results" / "CX" / "E"


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def _case(name: str) -> dict:
    return next(c for c in cx_e.cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def job(args):
    stage, kind, key, case_name, setting_json = args
    t0 = time.time()
    out = cx_e.run(_case(case_name), kind, json.loads(setting_json))
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str]) -> list[tuple]:
    return [(stage, kind, cx_e.setting_key(setting), n, json.dumps(setting)) for n in names]


def _names(*sets: str) -> list[str]:
    return [c["name"] for s_ in sets for c in cx_e.cases(s_)]


def tune_jobs() -> list[tuple]:
    return [j for s_ in cx_e.K_GRID for j in _jobs("tune", "lmpc", s_, _names("CEV"))]


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/E/selected.json - run report_cx_e.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest():
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def settings(sel: dict) -> dict:
    k = sel["kinds"]["lmpc"]["setting"]
    return {"nmpc": {}, "identified": {}, "lmpc": k, "lmpc_nominal": dict(k)}


def test_jobs(st: dict) -> list[tuple]:
    return [j for k in cx_e.KINDS for j in _jobs("test", k, st[k], _names("CET", "NOM"))]


def secondary_jobs(st: dict) -> list[tuple]:
    return [j for k in cx_e.KINDS for j in _jobs("secondary", k, st[k], _names(*cx_e.SECONDARY))]


def dev_jobs(ks: list[int]) -> list[tuple]:
    names = _names("NOM", "DEV")
    j = []
    for k in ks:
        j += _jobs("dev", "lmpc", {"K": k}, names)
    j += _jobs("dev", "lmpc_nominal", {"K": ks[0]}, names)
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
    ap.add_argument("what", choices=["jobs", "tune", "test", "secondary", "dev"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--id-workers", type=int, default=6)
    ap.add_argument("--k", default="20,10,5", help="dev: the K values (the first also for lmpc_nominal)")
    a = ap.parse_args()
    if a.what == "jobs":
        st = settings({"kinds": {"lmpc": {"setting": {"K": 20}}}})
        print(f"tune: {len(tune_jobs())} sequences (CEV 10 cars x 3 K) = {12 * len(tune_jobs())} laps")
        tj = test_jobs(st)
        print(f"test: {len(tj)} jobs ({sum(j[1] in cx_e.LEARN_KINDS for j in tj)} sequences of 12 laps, "
              f"{sum(j[1] in cx_e.FIXED_KINDS for j in tj)} single laps)")
        print(f"secondary: {len(secondary_jobs(st))} jobs")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what in ("test", "secondary"):
        st = settings(_selected())
        if a.what == "secondary" and not (OUT / "report" / "CX_E.md").exists():
            raise SystemExit("the secondary conditions run after the primary report (protocol section 2)")
        jobs = test_jobs(st) if a.what == "test" else secondary_jobs(st)
        os.environ["L6_REPLAN_SLOTS"] = str(a.id_workers)
        path = OUT / f"{a.what}.jsonl"
        _run([j for j in jobs if j[1] != "identified"], path, a.workers)
        _run([j for j in jobs if j[1] == "identified"], path, a.id_workers)
    else:
        _run(dev_jobs([int(k) for k in a.k.split(",")]), OUT / "dev" / "laps.jsonl", a.workers)


if __name__ == "__main__":
    main()
