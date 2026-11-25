"""Control expansion phase J runs (docs/CX_J_protocol.md, revision 0 with its clarifications). Windows `.venv`; one fresh
process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. A job is one lap. Records in results/CX/J/
(key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_j.py jobs                     # the job lists (counts)
    python scripts/run_cx_j.py refs [--workers 20]      # 9.2 / 9.4: nmpc, tube, adaptive on DEV and NOM in the four conditions,
                                                        # each lap with its recorded arrays -> dev/refs.jsonl, dev/replay_laps/
    python scripts/run_cx_j.py gate [--workers 20]      # 9.4: ra (2, 0.1), ra_fixed, ra_fixed_res, ra_shrink -> dev/gate.jsonl
    python scripts/run_cx_j.py tune [--workers 20]      # section 4: ra's 9 settings on JV (after packet 2) -> tune.jsonl
    python scripts/run_cx_j.py test [--workers 20]      # section 8: after packet 2's APPROVE and the selection -> test.jsonl

``tune`` and ``test`` need the file ``results/CX/J/APPROVED_<stage>`` (written by hand when the review has approved it), and
``test`` also ``selected.json``.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from f1race.experiments import cx_j  # noqa: E402

OUT = cx_j.OUT
CONDS = cx_j.CONDITIONS


def _job(args: tuple) -> dict:
    stage, kind, setting_json, case_name, record = args
    from f1race.experiments import cx_j

    t0 = time.time()
    setting = json.loads(setting_json)
    out: dict = {"stage": stage, "job": _key(args), "commit": os.environ.get("CXJ_COMMIT", "?")}
    try:
        case = cx_j.case_by_name(case_name)
        out.update(cx_j.lap(case, kind, setting, record=record))
    except Exception as e:                                             # noqa: BLE001 - an error record, not a lost run
        import traceback

        out.update({"kind": kind, "setting": setting, "setting_key": cx_j.setting_key(setting), "case": case_name,
                    "completed": False, "lap_time": float("nan"), "error": f"{type(e).__name__}: {e}",
                    "traceback": traceback.format_exc()[-1500:]})
    out["wall_job_s"] = time.time() - t0
    return out


def _key(args: tuple) -> str:
    stage, kind, setting_json, case_name, _ = args
    return f"{stage}|{kind}|{cx_j.setting_key(json.loads(setting_json))}|{case_name}"


def _all(set_: str, conds=CONDS) -> list[str]:
    return [c["name"] for cond in conds for c in cx_j.cases(f"{set_}_{cond}")]


def _j(stage, kind, setting, names, rec=None) -> list[tuple]:
    return [(stage, kind, json.dumps(setting), n, None if rec is None else str(rec / f"{kind}_{n}.npz")) for n in names]


def refs_jobs() -> list[tuple]:
    rec = OUT / "dev" / "replay_laps"
    names = _all("DEV") + _all("NOM")
    return [j for k in cx_j.PARENTS for j in _j("refs", k, {}, names, rec)]


def gate_jobs() -> list[tuple]:
    names = _all("DEV") + _all("NOM")
    return [j for k in cx_j.RA_KINDS for j in _j("gate", k, cx_j.DEV_SETTING, names)]


def tune_jobs() -> list[tuple]:
    names = _all("JV")
    return [j for g in cx_j.RHO_GRID for j in _j("tune", "ra", g, names)]


def test_jobs(setting: dict) -> list[tuple]:
    names = [c["name"] for s in ("JT", "JD", "JN", "JL") for c in cx_j.cases(s)]
    two = [c["name"] for s in ("JT", "JD") for c in cx_j.cases(s)]
    nom = _all("NOM")
    jobs = [j for k in cx_j.TEST_KINDS for j in _j("test", k, setting if k in cx_j.RA_KINDS else {}, names)]
    jobs += _j("test", "adaptive_oracle", {}, two)
    jobs += [j for k in cx_j.TEST_KINDS for j in _j("test", k, setting if k in cx_j.RA_KINDS else {}, nom)]
    return jobs


def _launch(path: Path, n: int) -> None:
    """One line per launch (review m4: never overwritten): the commit, the dirty flag, the time and the number of jobs."""
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    path.parent.mkdir(parents=True, exist_ok=True)
    os.environ["CXJ_COMMIT"] = commit + ("+dirty" if dirty else "")
    with path.open("a") as fh:
        fh.write(json.dumps({"commit": commit, "dirty": dirty, "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "jobs": n}) + "\n")


def _run(jobs: list[tuple], path: Path, workers: int) -> None:
    from multiprocessing import get_context

    path.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if path.exists():
        done = {json.loads(l)["job"] for l in path.read_text().splitlines() if l.strip()}
    todo = [j for j in jobs if _key(j) not in done]
    _launch(path.parent / f"launches_{path.stem}.jsonl", len(todo))
    print(f"{len(todo)} jobs -> {path}", flush=True)
    if not todo:
        return
    with get_context("spawn").Pool(min(workers, len(todo)), maxtasksperchild=1) as pool, path.open("a") as fh:
        for i, r in enumerate(pool.imap_unordered(_job, todo), start=1):
            fh.write(json.dumps(r, default=float) + "\n")
            fh.flush()
            print(i, r["job"], r.get("stop_reason"), r.get("lap_time"), r.get("error", ""), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["jobs", "refs", "gate", "tune", "test"])
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    if a.what == "jobs":
        print(f"refs: {len(refs_jobs())}, gate: {len(gate_jobs())}, tune: {len(tune_jobs())}, "
              f"test: {len(test_jobs(cx_j.DEV_SETTING))}")
        return
    if a.what == "refs":
        return _run(refs_jobs(), OUT / "dev" / "refs.jsonl", a.workers)
    if a.what == "gate":
        return _run(gate_jobs(), OUT / "dev" / "gate.jsonl", a.workers)
    if not (OUT / f"APPROVED_{a.what}").exists():
        raise SystemExit(f"{a.what}: the review's approval is not recorded ({OUT / ('APPROVED_' + a.what)})")
    if a.what == "tune":
        return _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    sel = json.loads((OUT / "selected.json").read_text())["ra"]["setting"]
    return _run(test_jobs(sel), OUT / "test.jsonl", a.workers)


if __name__ == "__main__":
    main()
