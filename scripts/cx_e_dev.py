"""Development checks of CX-E (docs/CX_E_protocol.md section 9) on NOM and DEV only; records in results/CX/E/dev/ with the
commit and the dirty flag.

    python scripts/cx_e_dev.py nlp          # 1: one solve against a stored lap's own continuation; near the line (m3)
    python scripts/cx_e_dev.py regression   # 2: the corrected step's one-step residual, in and out of sample (a DEV car)
    python scripts/run_cx_e.py dev ...      # 3, 4: initialisation and learning laps (results/CX/E/dev/laps.jsonl)
    python scripts/cx_e_dev.py summary      # 3-5: the lap-time curves, completions, slacks, solve times; the gate
    python scripts/cx_e_dev.py diag         # after the gate failed: the terminal condition with delta, P = 1, both
                                            # (development options; results/CX/E/dev/diag.jsonl)
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
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

from f1race.experiments import cx_e  # noqa: E402

DEV = ROOT / "results" / "CX" / "E" / "dev"


def _meta() -> dict:
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    return {"commit": commit, "dirty": dirty, "date": time.strftime("%Y-%m-%d %H:%M")}


def _write(name: str, obj) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / name).write_text(json.dumps({**_meta(), **obj}, indent=1, default=float))
    print(json.dumps(obj, indent=1, default=float)[:4000])


def _stored_nmpc_lap(case: dict, eta_p: float):
    """One lap of `nmpc`'s NMPC on a library plan, recorded (50 ms) as the store records it."""
    from f1race.control.lmpc import Recorder
    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_a, cx_d
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, x0_from_plan
    from f1race.experiments.l8 import plans_for
    from f1race.track import Track

    track, plans = Track.load("Spielberg"), plans_for("Spielberg")
    etas, lib = cx_d.plan_library()
    o95 = plans[0.95]
    path, _ = dyn_reference(track, DynVehicle(), o95)
    x0 = x0_from_plan(o95, path)
    w = {k: v * 2.0 for k, v in cx_a.NMPC_TRACK_W.items()}
    plan = o95 if eta_p == 0.95 else lib[int(np.flatnonzero(np.isclose(etas, eta_p))[0])]
    mpc = FrenetMPC(track, DynVehicle(), ref_from_plan(plan), mode="track", N=cx_e.N, dt=cx_e.MPC_DT, eta=0.95,
                    margin=0.6, weights=w)
    rec = Recorder(MPCController(mpc, cx_e.DT), track, cx_e.MPC_DT)
    res = cx_e._run(case, track, rec, x0)
    m = cx_e._lap_metrics(res, track, rec)
    return track, np.array(rec.rec_t), np.array(rec.rec_x), np.array(rec.rec_u), m


def nlp_check() -> None:
    """Item 1: the safe set made of one stored lap's states around its own state N steps after the solve's state, the
    correction zero: the terminal condition holds (slack ~0), lambda on the simplex; and near the line (s_N past L)."""
    from f1race.control.lmpc import LapStore, LearningMPC
    from f1race.dynamics import DynVehicle

    case = cx_e.cases("NOM")[0]
    track, t, X, U, m = _stored_nmpc_lap(case, 0.95)
    K = 10
    lmpc = LearningMPC(track, DynVehicle(), n_pts=2 * K, N=cx_e.N, dt=cx_e.MPC_DT)
    store = LapStore(lmpc._step)
    store.add(t, X, U, m["lap_time"])
    rows = []
    L = track.length
    k_line = int(np.flatnonzero(X[:, 0] > L - 60.0)[0])
    for k in (100, 400, 800, 1200, k_line):
        lmpc.reset()
        D, c, _ = store.select(X[k + cx_e.N, 0], K, 1)          # one stored lap (repeated twice in the P = 2 slots)
        u, info = lmpc.solve(X[k], U[k - 1] if k else np.zeros(3), D, c)
        Xp, Up, lam = lmpc.prediction()
        rows.append({"k": k, "s": float(X[k, 0]), "s_N": float(Xp[-1, 0]), "status": info["status"],
                     "term_slack": info["term_slack"], "limit_slack": info["limit_slack"],
                     "lambda_sum": float(lam.sum()), "lambda_min": float(lam.min()),
                     "selected_s_min_max": [float(D[0].min()), float(D[0].max())],
                     "lambda_weight_past_line": float(lam[D[0] > L].sum())})
    _write("nlp.json", {"stored_lap_time": m["lap_time"], "rows": rows,
                        "line_case": {"k": k_line, "note": "s_N past L: the selected points lie past the line"}})


def regression_check() -> None:
    """Item 2: on DEV car 1, the two initialisation laps stored; the third lap (the 0.95 plan) out of sample: the one-step
    residual of (v_x, v_y, r) with the nominal step and with the corrected step."""
    from f1race.control.lmpc import FS, LapStore, LearningMPC, regressor
    from f1race.dynamics import DynVehicle

    case = cx_e.cases("DEV")[0]
    laps = [_stored_nmpc_lap(case, e) for e in (0.73, 0.79, 0.95)]
    track = laps[0][0]
    lmpc = LearningMPC(track, DynVehicle(), n_pts=20, N=cx_e.N, dt=cx_e.MPC_DT)
    store = LapStore(lmpc._step)
    for _, t, X, U, m in laps[:2]:
        store.add(t, X, U, m["lap_time"] if m["completed"] else t[-1])
    out = {}
    for name, (_, t, X, U, m) in (("in_sample_lap_0.79", laps[1]), ("out_of_sample_lap_0.95", laps[2])):
        nxt = np.array([np.asarray(lmpc._step(X[k], U[k] / np.array([1.0, FS, FS]))).ravel() for k in range(len(X) - 1)])
        e_nom = X[1:, 3:6] - nxt[:, 3:6]
        Z = regressor(X[:-1], U[:-1])
        G, fb = store.regress(Z)
        corr = np.einsum("nij,nj->ni", G, Z)
        e_cor = e_nom - corr
        out[name] = {"samples": len(Z), "fallback_share": float(np.mean(fb)),
                     "rms_nominal_vx_vy_r": np.sqrt(np.mean(e_nom ** 2, axis=0)).tolist(),
                     "rms_corrected_vx_vy_r": np.sqrt(np.mean(e_cor ** 2, axis=0)).tolist(),
                     "lap_completed": m["completed"]}
    _write("regression.json", {"car": case["name"], "plant": case["info"], **out})


def summary() -> None:
    """Items 3-5 from laps.jsonl: per sequence the lap-time curve, completions, terminal slack, solve time; the gate."""
    from report_cx_a import load

    recs = [r for r in load(DEV / "laps.jsonl") if "error" not in r]
    errors = [r["job"] for r in load(DEV / "laps.jsonl") if "error" in r]
    rows = []
    for r in sorted(recs, key=lambda r: (r["kind"], r["setting_key"], r["case"])):
        learn = [l_ for l_ in r["laps"] if l_["kind"] == "learn"]
        rows.append({"kind": r["kind"], "setting": r["setting_key"], "case": r["case"],
                     "init": [round(l_["lap_time"], 2) if l_["completed"] else l_["stop_reason"] for l_ in r["laps"][:2]],
                     "learn": [round(l_["lap_time"], 2) if l_["completed"] else
                               (f"{l_['stop_reason']} {l_['failure_s']:.0f} m" if l_.get("failure_s") is not None
                                else l_["stop_reason"]) for l_ in learn],
                     "tenth_completed": r["completed"], "first_failed_learning_lap": r["first_failed_learning_lap"],
                     "term_slack_share": [l_.get("term_slack_share_above_1e-3") for l_ in learn],
                     "solve_ms_mean": [round(l_["solve_ms_mean_p95_max"][0]) for l_ in learn if l_.get("solve_ms_mean_p95_max")],
                     "regression_fallback_share": [l_.get("regression_fallback_share") for l_ in learn]})
    gate = {}
    for key in sorted({(r["kind"], r["setting_key"]) for r in recs}):
        sub = [r for r in recs if (r["kind"], r["setting_key"]) == key]
        nom = next((r for r in sub if r["case"] == "NOM_car0"), None)
        dev = [r for r in sub if r["case"].startswith("DEV_")]
        nom_ok = bool(nom and all(l_["completed"] for l_ in nom["laps"] if l_["kind"] == "learn"))
        faster = bool(nom and nom_ok and nom["laps"][-1]["lap_time"] < nom["laps"][2]["lap_time"])
        dev_started = [r for r in dev if r["started"]]
        gate["|".join(key)] = {"NOM_all_10_completed": nom_ok, "NOM_tenth_faster_than_first": faster,
                               "DEV_started": len(dev_started), "DEV_tenth_completed": sum(r["completed"] for r in dev_started),
                               "passed": bool(nom_ok and faster and sum(r["completed"] for r in dev_started) >= 3)}
    _write("summary.json", {"errors": errors, "rows": rows, "gate_by_setting": gate,
                            "laps_meta": json.loads((DEV / "laps_meta.json").read_text()) if (DEV / "laps_meta.json").exists() else None})


def diag(workers: int) -> None:
    """The gate failed (no sequence beyond learning lap 2): two hypotheses on the mechanism – the steering angle left free
    in the terminal condition (clarification m3) and the safe set mixing two laps (P = 2) – tested on NOM and DEV cars 1
    and 4, K = 5, four learning laps."""
    from run_cx_e import _jobs, _run

    variants = [{"K": 5, "P": 2, "term7": 1, "n_learn": 4}, {"K": 5, "P": 1, "n_learn": 4},
                {"K": 5, "P": 1, "term7": 1, "n_learn": 4}]
    names = ["NOM_car0", "DEV_car1", "DEV_car4"]
    jobs = [j for v in variants for j in _jobs("diag", "lmpc_nominal", v, names)]
    jobs += _jobs("diag", "lmpc", variants[-1], names)
    (DEV / "diag_meta.json").write_text(json.dumps({**_meta(), "variants": variants}, indent=1))
    _run(jobs, DEV / "diag.jsonl", workers)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["nlp", "regression", "summary", "diag"])
    ap.add_argument("--workers", type=int, default=12)
    a = ap.parse_args()
    if a.what == "diag":
        diag(a.workers)
        return
    {"nlp": nlp_check, "regression": regression_check, "summary": summary}[a.what]()


if __name__ == "__main__":
    main()
