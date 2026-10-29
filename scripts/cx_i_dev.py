"""Control expansion phase I – development checks of candidate I-1 (docs/CX_I_protocol.md section 9; NOM and DEV only).
Windows `.venv`; records in results/CX/I/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_i_dev.py nlp-repro     # 9.1: phase E's NLP unchanged (the bound off) – lmpc K 20 on NOM against
                                             # phase E's record
    python -m pytest tests/test_cx_i.py      # 9.2: the bound (unit tests)
    python scripts/run_cx_i.py dev           # 9.3: stages A and B, nmpc and identified on NOM + DEV
    python scripts/report_cx_i.py dev        # 9.4: the tables and the gate

Revision 1 (section 10.7), records in results/CX/I/repro/:
    python scripts/cx_i_dev.py repro-r0      # r0: with the new options off, phase E's lmpc K 20 and lmpc_nominal K 10 and
                                             # phase H's atv_nominal K 20 on NOM reproduce their records
    python scripts/cx_i_dev.py repro-r2      # r2: v_c's rule and the two initialisation laps at v_c (from the smoke's
                                             # progress log, results/CX/I/repro_smoke/progress_R1.jsonl)
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
DEV = ROOT / "results" / "CX" / "I" / "dev"
E_DEV = ROOT / "results" / "CX" / "E" / "dev" / "laps.jsonl"
H_DEV = ROOT / "results" / "CX" / "H" / "dev" / "laps.jsonl"
REPRO = ROOT / "results" / "CX" / "I" / "repro"


def meta(what: str, d: Path = DEV) -> None:
    d.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (d / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def nlp_repro() -> None:
    from f1race.experiments import cx_e

    meta("nlp_repro")
    ref = next(json.loads(l) for l in E_DEV.read_text().splitlines()
               if l.strip() and json.loads(l).get("job") == "dev|lmpc|K=20|NOM_car0")
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], "lmpc", {"K": 20})
    pick = lambda rec: [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in rec["laps"][:4]]  # noqa: E731
    out = {"wall_s": time.time() - t0,
           "phase_e": {"lap_times": ref["lap_times"], "first_failed": ref["first_failed_learning_lap"], "laps": pick(ref)},
           "now": {"lap_times": r["lap_times"], "first_failed": r["first_failed_learning_lap"], "laps": pick(r)}}
    out["equal"] = out["phase_e"]["lap_times"] == out["now"]["lap_times"] and out["phase_e"]["laps"] == out["now"]["laps"]
    (DEV / "nlp_repro.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


_R0 = [("E", "lmpc", {"K": 20}, "nlp"), ("E", "lmpc_nominal", {"K": 10}, "nlp"), ("H", "lmpc_nominal", {"K": 20}, "atv")]


def _laps(rec: dict) -> list:
    return [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in rec["laps"]]


def _r0_job(i: int) -> dict:
    from f1race.experiments import cx_e

    phase, kind, setting, solver = _R0[i]
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], kind, setting, solver=solver)
    return {"i": i, "lap_times": r["lap_times"], "laps": _laps(r), "wall_s": time.time() - t0}


def repro_r0() -> None:
    """r0: the default path unchanged (section 10.7) – three NOM records reproduced lap for lap."""
    from multiprocessing import get_context

    meta("r0", REPRO)
    ref = []
    for phase, kind, setting, solver in _R0:
        path, k = (E_DEV, kind) if phase == "E" else (H_DEV, "atv_nominal")
        key = f"K={setting['K']}"
        ref.append(next(json.loads(l) for l in path.read_text().splitlines() if l.strip()
                        and json.loads(l).get("kind") == k and json.loads(l).get("setting_key") == key
                        and json.loads(l).get("case") == "NOM_car0" and "error" not in json.loads(l)))
    with get_context("spawn").Pool(len(_R0), maxtasksperchild=1) as pool:
        now = sorted(pool.map(_r0_job, range(len(_R0))), key=lambda r: r["i"])
    out = []
    for (phase, kind, setting, solver), rf, nw in zip(_R0, ref, now):
        eq = rf["lap_times"] == nw["lap_times"] and _laps(rf) == [tuple(x) for x in nw["laps"]]
        out.append({"phase": phase, "kind": kind, "setting": setting, "solver": solver, "equal": eq,
                    "record": {"lap_times": rf["lap_times"], "laps": _laps(rf)}, "now": nw})
    res = {"all_equal": all(o["equal"] for o in out), "checks": out}
    (REPRO / "r0.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({"all_equal": res["all_equal"], "equal": [o["equal"] for o in out]}))


def repro_r2() -> None:
    """r2: v_c's rule and the two initialisation laps at v_c (section 10.7), from the smoke's progress log."""
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_i
    from f1race.track import Track

    import numpy as np

    meta("r2", REPRO)
    tr = Track.load("Spielberg")
    k = np.abs(tr.kappa)
    laps = [json.loads(l) for l in (ROOT / "results" / "CX" / "I" / "repro_smoke" / "progress_R1.jsonl").read_text()
            .splitlines() if l.strip()]
    init = [{key: l.get(key) for key in ("lap", "completed", "lap_time", "stop_reason", "distance", "run_out_completed",
                                         "tyre_usage_max_front_rear", "min_edge_margin", "solves", "fallbacks",
                                         "solve_ms_mean_p95_max", "start_s", "wall_lap_s")}
            for l in laps if l["kind"] == "init"]
    out = {"v_c_rule": cx_i.v_c_rule(tr, DynVehicle()), "V_C": cx_i.V_C, "max_abs_kappa": float(k.max()),
           "s_at_max": float(tr.s[k.argmax()]), "L": float(tr.length), "init_laps": init}
    (REPRO / "r2.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["nlp-repro", "repro-r0", "repro-r2"])
    a = ap.parse_args()
    {"nlp-repro": nlp_repro, "repro-r0": repro_r0, "repro-r2": repro_r2}[a.what]()


if __name__ == "__main__":
    main()
