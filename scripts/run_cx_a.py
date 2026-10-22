"""Control expansion phase A runs (docs/CX_A_protocol.md, revision 1). Windows `.venv`; one fresh process
per job (spawn pool, maxtasksperchild=1, as L7 / L8), single-threaded BLAS. Benchmark stages are run from
a pinned worktree (roadmap section 1); records in results/CX/A/ (key ``job`` = stage|kind|setting|case).

    python scripts/run_cx_a.py jobs                                  # the job lists (counts)
    python scripts/run_cx_a.py tune  [--workers 20]                  # CXV: 24 mpcc, 24 mpcc_centre, 6 nmpc settings
    python scripts/report_cx_a.py tune                               # the rule of section 4 -> results/CX/A/selected.json
    python scripts/run_cx_a.py tune_a2 [--workers 20]                # amendment A2: mpcc and mpcc_centre with the soft bound on CXV
    python scripts/report_cx_a.py tune --a2                          # nmpc from tune.jsonl, the MPCCs from tune_a2.jsonl
    python scripts/run_cx_a.py test  [--workers 20]                  # CXT + NOM with the selected settings, + tracker
    python scripts/run_cx_a.py tune_ca [--workers 20]                # amendment A1: mpcc_ca's 6 settings on CXV (after selected.json)
    python scripts/report_cx_a.py tune_ca                            # -> results/CX/A/selected_ca.json (to the reviewer)
    python scripts/run_cx_a.py test --with-ca                        # CXT + NOM for mpcc_ca too (needs selected_ca.json)
    python scripts/run_cx_a.py secondary [--workers 20]              # CXT_LAG30, CXD, CXL8 (protocol section 2), after the primary report
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
from f1race.experiments.cx_a import KINDS, MPCC_GRID_A2, SECONDARY, ca_grid, cases, grid, setting_key  # noqa: E402

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
    case = next(c for c in cases(case_name.rsplit("_", 1)[0]) if c["name"] == case_name)
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


def test_jobs(selected: dict, kinds=KINDS) -> list[tuple]:
    names = [c["name"] for c in cases("CXT")] + [c["name"] for c in cases("NOM")]
    out = []
    for k in kinds:
        s = {} if k == "tracker" else selected[k]["setting"]
        out += [("test", k, setting_key(s), n, json.dumps(s)) for n in names]
    return out


def secondary_jobs(selected: dict) -> list[tuple]:
    names = [c["name"] for set_ in SECONDARY for c in cases(set_)]
    out = []
    for k in KINDS:
        s = {} if k == "tracker" else selected[k]["setting"]
        out += [("secondary", k, setting_key(s), n, json.dumps(s)) for n in names]
    return out


def _selected(name: str = "selected.json", tune: str = "tune.jsonl") -> dict:
    sel_path = OUT / name
    if not sel_path.exists():
        raise SystemExit(f"no results/CX/A/{name} - run report_cx_a.py (and have it reviewed) first")
    sel = json.loads(sel_path.read_text())
    if sel.get("tune_sha256") != hashlib.sha256((OUT / tune).read_bytes()).hexdigest():
        raise SystemExit(f"{name} was not made from the present {tune}")
    if "tune_a2_sha256" in sel and sel["tune_a2_sha256"] != hashlib.sha256((OUT / "tune_a2.jsonl").read_bytes()).hexdigest():
        raise SystemExit(f"{name} was not made from the present tune_a2.jsonl")
    return sel


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
    ap.add_argument("what", choices=["jobs", "tune", "tune_a2", "tune_ca", "test", "secondary", "smoke"])
    ap.add_argument("--with-ca", action="store_true", help="test: mpcc_ca too (amendment A1)")
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--kinds", default=",".join(KINDS))
    a = ap.parse_args()
    if a.what == "jobs":
        tj = tune_jobs()
        print(f"tune: {len(tj)} laps (CXV 10 cars x (6 nmpc + 24 mpcc + 24 mpcc_centre settings))")
        print(f"test: {len(KINDS)} controllers x (25 CXT + 1 NOM) = {len(KINDS) * 26} laps")
        n2 = sum(len(cases(s_)) for s_ in SECONDARY)
        print(f"secondary: {len(KINDS)} controllers x {n2} laps (CXT_LAG30 25, CXD 30, CXL8 4) = {len(KINDS) * n2}")
        return
    if a.what == "tune":
        _run(tune_jobs(), OUT / "tune.jsonl", a.workers)
    elif a.what == "tune_a2":
        names = [c["name"] for c in cases("CXV")]
        _run([("tune", k, setting_key(s_), n, json.dumps(s_)) for k in ("mpcc", "mpcc_centre")
              for s_ in MPCC_GRID_A2 for n in names], OUT / "tune_a2.jsonl", a.workers)
    elif a.what == "tune_ca":
        base = _selected()["kinds"]["mpcc"]["setting"]
        names = [c["name"] for c in cases("CXV")]
        _run([("tune", "mpcc_ca", setting_key(s_), n, json.dumps(s_)) for s_ in ca_grid(base) for n in names],
             OUT / "tune_ca.jsonl", a.workers)
    elif a.what == "test":
        sel = _selected()["kinds"]
        kinds = KINDS
        if a.with_ca:
            sel = {**sel, **_selected("selected_ca.json", "tune_ca.jsonl")["kinds"]}
            kinds = KINDS + ("mpcc_ca",)
        _run(test_jobs(sel, kinds), OUT / "test.jsonl", a.workers)
    elif a.what == "secondary":
        if not (OUT / "report" / "CX_A.md").exists():
            raise SystemExit("the secondary conditions run after the primary report (protocol section 2)")
        _run(secondary_jobs(_selected()["kinds"]), OUT / "secondary.jsonl", a.workers)
    else:
        kinds = a.kinds.split(",")
        jobs = [("smoke", k, setting_key(SMOKE_SETTINGS[k]), "NOM_car0", json.dumps(SMOKE_SETTINGS[k])) for k in kinds]
        _run(jobs, OUT / "smoke" / "smoke.jsonl", a.workers)


if __name__ == "__main__":
    main()
