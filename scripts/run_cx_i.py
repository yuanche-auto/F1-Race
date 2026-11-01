"""Control expansion phase I runs (docs/CX_I_protocol.md, revision 0 with its clarifications m1, m2, n1, n2; candidate I-1).
Windows `.venv`; one fresh process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. A job of `pb` /
`pb_nominal` / `lmpc` is one car's whole sequence (two initialisation laps and ten learning laps); `nmpc` / `identified`
one lap. Records in results/CX/I/ (key ``job`` = stage|kind|setting|case).

No benchmark stage exists: the benchmark (IV tuning, IT test, secondary) would have been registered by a revision after
the gate passed (clarification m1); the gate failed, and the user closed the LMPC line (section 11).

Revision 1 (section 10): the reproduction check on NOM – one job per arm (R1, R2, R3: the whole sequence of two
initialisation and 40 consecutive learning laps), each lap appended to ``progress_<arm>.jsonl`` as it is done.

    python scripts/run_cx_i.py jobs                       # the job list (counts)
    python scripts/run_cx_i.py dev [--workers 20]         # section 9.3: stage A (pb on NOM, eps 0.02 / 0.04 / 0.08),
                                                          # stage B (pb, pb_nominal at eps 0.04 on NOM + DEV), nmpc and
                                                          # identified on NOM + DEV -> results/CX/I/dev/laps.jsonl
    python scripts/run_cx_i.py repro [--arms R1 R2 R3]    # section 10: the arms -> results/CX/I/repro/laps.jsonl
    python scripts/run_cx_i.py repro --arms R1 --n-learn 2 --out repro_smoke   # the scratch smoke (section 10.7)
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
from multiprocessing import get_context
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from f1race.experiments import cx_i  # noqa: E402

OUT = ROOT / "results" / "CX" / "I"


def _key(j: tuple) -> str:
    return "|".join(j[:4])


def job(args):
    stage, kind, key, case_name, setting_json = args
    t0 = time.time()
    out = cx_i.run(cx_i.case_by_name(case_name), kind, json.loads(setting_json))
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str]) -> list[tuple]:
    return [(stage, kind, cx_i.setting_key(setting), n, json.dumps(setting)) for n in names]


def _names(*sets: str) -> list[str]:
    return [c["name"] for s_ in sets for c in cx_i.cases(s_)]


def dev_jobs() -> list[tuple]:
    nom, both = _names("NOM"), _names("NOM", "DEV")
    a = [j for s_ in cx_i.EPS_GRID if s_ != cx_i.DEV_SETTING for j in _jobs("dev", "pb", s_, nom)]
    b = _jobs("dev", "pb", cx_i.DEV_SETTING, both) + _jobs("dev", "pb_nominal", cx_i.DEV_SETTING, both)
    return a + b + _jobs("dev", "nmpc", {}, both) + _jobs("dev", "identified", {}, both)


def repro_job(args):
    """One arm of the reproduction check; each lap's record appended to its progress log as it is done."""
    arm, n_learn, progress = args
    t0 = time.time()

    def on_lap(m):
        with open(progress, "a", encoding="utf-8") as fo:
            fo.write(json.dumps({"arm": arm, "wall_s": time.time() - t0, **m}, default=float) + "\n")

    try:
        out = cx_i.repro(arm, n_learn, on_lap)
        out.update({"job": f"repro|{arm}", "stage": "repro", "wall_job_s": time.time() - t0})
        return out
    except Exception:                                       # noqa: BLE001
        return {"job": f"repro|{arm}", "error": traceback.format_exc()}


def _meta(path: Path) -> None:
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    path.write_text(json.dumps({"commit": commit, "dirty": dirty, "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


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
    ap.add_argument("what", choices=["jobs", "dev", "repro"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--id-workers", type=int, default=6)
    ap.add_argument("--arms", nargs="+", default=list(cx_i.REPRO))
    ap.add_argument("--n-learn", type=int, default=None)
    ap.add_argument("--out", default="repro")
    a = ap.parse_args()
    if a.what == "repro":
        out = OUT / a.out
        out.mkdir(parents=True, exist_ok=True)
        _meta(out / "meta_laps.json")
        done = set()
        if (out / "laps.jsonl").exists():
            done = {json.loads(l)["job"] for l in (out / "laps.jsonl").read_text().splitlines()
                    if l.strip() and "error" not in json.loads(l)}
        todo = [(arm, a.n_learn, str(out / f"progress_{arm}.jsonl")) for arm in a.arms if f"repro|{arm}" not in done]
        print(f"{len(todo)} arms -> {out}", flush=True)
        if not todo:
            return
        with get_context("spawn").Pool(len(todo), maxtasksperchild=1) as pool, \
                open(out / "laps.jsonl", "a", encoding="utf-8") as fo:
            for rec in pool.imap_unordered(repro_job, todo):
                fo.write(json.dumps(rec, default=float) + "\n")
                fo.flush()
                print(rec["job"], rec.get("stop_reason", ""), "error" if "error" in rec else "", flush=True)
        return
    if a.what == "jobs":
        print(f"dev: {len(dev_jobs())} jobs")
        return
    os.environ["L6_REPLAN_SLOTS"] = str(a.id_workers)
    jobs = dev_jobs()
    (OUT / "dev").mkdir(parents=True, exist_ok=True)
    _meta(OUT / "dev" / "meta_laps.json")
    _run([j for j in jobs if j[1] != "identified"], OUT / "dev" / "laps.jsonl", a.workers)
    _run([j for j in jobs if j[1] == "identified"], OUT / "dev" / "laps.jsonl", a.id_workers)


if __name__ == "__main__":
    main()
