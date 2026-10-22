"""Control expansion phase B runs (docs/CX_B_protocol.md, revision 0 with its clarifications and amendments).
Windows `.venv`; one fresh process per job (spawn pool, maxtasksperchild=1), single-threaded BLAS. Benchmark
stages run from a pinned worktree (roadmap section 1); records in results/CX/B/ (key ``job`` =
stage|kind|setting|case|r<replicate>).

    python scripts/run_cx_b.py jobs                         # the job lists (counts)
    python scripts/run_cx_b.py tune    [--workers 20]       # CXV: the 9 mppi settings at K_RT
    python scripts/report_cx_b.py tune                      # the rule of section 4 -> results/CX/B/selected.json
    python scripts/run_cx_b.py tune_lp [--workers 20]       # CXV: mppi_lp's 3 cut-offs on mppi's choice
    python scripts/report_cx_b.py tune_lp                   # -> results/CX/B/selected_lp.json (both to the reviewer)
    python scripts/run_cx_b.py test    [--workers 20]       # CBT + NOM: tracker, nmpc, mpcc, mppi (replicates 0-2),
                                                            # mppi_lp, mppi_big
    python scripts/run_cx_b.py secondary [--workers 20]     # CBT_LAG30, CXD, CXL8, after report/CX_B.md
    python scripts/run_cx_b.py smoke [--kinds ...]          # code check on the nominal car only (results/CX/B/smoke/)

``tune`` refuses to start while K_RT is not fixed; ``tune_lp`` and ``test`` refuse to start without the reviewed
selections and check that they were made from the present tuning records (SHA-256).
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
from f1race.experiments import cx_b  # noqa: E402

OUT = ROOT / "results" / "CX" / "B"
REPLICATES = (0, 1, 2)                     # mppi on CBT (section 5); only replicate 0 enters H-B1 / H-B2


def _key(j: tuple) -> str:
    return "|".join([*j[:4], f"r{j[4]}"])


def _case(name: str) -> dict:
    return next(c for c in cx_b.cases(name.rsplit("_", 1)[0]) if c["name"] == name)


def job(args):
    """One lap: (stage, kind, setting key, case name, replicate, setting json)."""
    stage, kind, key, case_name, rep, setting_json = args
    t0 = time.time()
    out = cx_b.lap(_case(case_name), kind, json.loads(setting_json), rep)
    out.update({"job": _key(args), "stage": stage, "wall_job_s": time.time() - t0})
    return out


def safe(j):
    try:
        return job(j)
    except Exception:                                       # noqa: BLE001
        return {"job": _key(j), "error": traceback.format_exc()}


def _jobs(stage: str, kind: str, setting: dict, names: list[str], reps=(0,)) -> list[tuple]:
    return [(stage, kind, cx_b.setting_key(setting), n, r, json.dumps(setting)) for n in names for r in reps]


def tune_jobs() -> list[tuple]:
    if cx_b.K_RT is None:
        raise SystemExit("K_RT is not fixed (protocol section 3.4: the timing of section 9.5, reviewed in packet 2)")
    names = [c["name"] for c in cx_b.cases("CXV")]
    return [j for g in cx_b.MPPI_GRID for j in _jobs("tune", "mppi", {**g, "K": cx_b.K_RT}, names)]


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _selected() -> dict:
    p = OUT / "selected.json"
    if not p.exists():
        raise SystemExit("no results/CX/B/selected.json - run report_cx_b.py tune (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_sha256") != _sha(OUT / "tune.jsonl"):
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return sel


def _selected_lp() -> dict:
    p = OUT / "selected_lp.json"
    if not p.exists():
        raise SystemExit("no results/CX/B/selected_lp.json - run report_cx_b.py tune_lp (and have it reviewed) first")
    sel = json.loads(p.read_text())
    if sel.get("tune_lp_sha256") != _sha(OUT / "tune_lp.jsonl"):
        raise SystemExit("selected_lp.json was not made from the present tune_lp.jsonl")
    if sel.get("selected_json_sha256") != _sha(OUT / "selected.json"):
        raise SystemExit("selected_lp.json was not made on the present selected.json (mppi_lp's base)")
    return sel


def settings(sel: dict, sel_lp: dict) -> dict:
    m = sel["kinds"]["mppi"]["setting"]
    return {**{k: cx_b.baseline_setting(k) for k in cx_b.BASELINES}, "mppi": m,
            "mppi_lp": sel_lp["kinds"]["mppi_lp"]["setting"], "mppi_big": cx_b.big_setting(m)}


def test_jobs(st: dict) -> list[tuple]:
    names = [c["name"] for c in cx_b.cases("CBT")] + [c["name"] for c in cx_b.cases("NOM")]
    return [j for k in cx_b.KINDS for j in _jobs("test", k, st[k], names, REPLICATES if k == "mppi" else (0,))]


def secondary_jobs(st: dict) -> list[tuple]:
    names = [c["name"] for s_ in cx_b.SECONDARY for c in cx_b.cases(s_)]
    return [j for k in cx_b.KINDS for j in _jobs("secondary", k, st[k], names)]


def _run(jobs: list[tuple], path: Path, workers: int) -> None:
    """One fresh process per job (spawn, maxtasksperchild=1); resumable: jobs with a record that is not an
    error are skipped (an error record stays in the file)."""
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
    ap.add_argument("what", choices=["jobs", "tune", "tune_lp", "test", "secondary", "smoke"])
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--kinds", default=",".join(cx_b.KINDS))
    a = ap.parse_args()
    if a.what == "jobs":
        n_cbt = len(cx_b.cases("CBT")) + 1
        n2 = sum(len(cx_b.cases(s_)) for s_ in cx_b.SECONDARY)
        print(f"tune: {len(cx_b.MPPI_GRID) * 10} laps (CXV 10 cars x 9 mppi settings); tune_lp: 30 laps")
        print(f"test: {n_cbt} cases x (3 baselines + 3 mppi replicates + mppi_lp + mppi_big) = {n_cbt * 8} laps")
        print(f"secondary: {len(cx_b.KINDS)} controllers x {n2} laps (CBT_LAG30 50, CXD 30, CXL8 4) = {len(cx_b.KINDS) * n2}")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what == "tune_lp":
        base = _selected()["kinds"]["mppi"]["setting"]
        names = [c["name"] for c in cx_b.cases("CXV")]
        _run([j for s_ in cx_b.lp_grid(base) for j in _jobs("tune", "mppi_lp", s_, names)], OUT / "tune_lp.jsonl",
             a.workers)
    elif a.what == "test":
        _run(test_jobs(settings(_selected(), _selected_lp())), OUT / "test.jsonl", a.workers)
    elif a.what == "secondary":
        if not (OUT / "report" / "CX_B.md").exists():
            raise SystemExit("the secondary conditions run after the primary report (protocol section 2)")
        _run(secondary_jobs(settings(_selected(), _selected_lp())), OUT / "secondary.jsonl", a.workers)
    else:
        small = {"lam": 1.0, "c_sigma": 1.0, "K": 64}
        st = {**{k: cx_b.baseline_setting(k) for k in cx_b.BASELINES}, "mppi": small,
              "mppi_lp": {**small, "f_c": 2.0}, "mppi_big": cx_b.big_setting(small)}
        jobs = [j for k in a.kinds.split(",") for j in _jobs("smoke", k, st[k], ["NOM_car0"])]
        _run(jobs, OUT / "smoke" / "smoke.jsonl", a.workers)


if __name__ == "__main__":
    main()
