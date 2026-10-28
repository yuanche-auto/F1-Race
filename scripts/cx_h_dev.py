"""Control expansion phase H – development checks (docs/CX_H_protocol.md section 9; NOM and DEV only). Windows `.venv`;
records in results/CX/H/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_h_dev.py qp            # 9.1: the full-size QP – the constraint Jacobian against central differences;
                                             # a feasible candidate (a model rollout whose end state is a stored point)
                                             # keeps every slack at zero
    python scripts/cx_h_dev.py nlp-repro     # 9.2: phase E's NLP unchanged – lmpc K 20 on NOM against phase E's record
    python scripts/run_cx_h.py dev           # 9.3: atv, atv_nominal (K 20), nmpc, identified on NOM + DEV
    python scripts/report_cx_h.py dev        # 9.4: the tables and the gate
    python scripts/cx_h_dev.py diag [--workers 6]   # after the gate: a diagnostic variant (clarification m1 – a diagnostic
                                             # only, it cannot pass the gate): atv with qpOASES and a box on the QP's
                                             # input step, on NOM + DEV -> dev/diag.jsonl
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
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
DEV = ROOT / "results" / "CX" / "H" / "dev"
E_DEV = ROOT / "results" / "CX" / "E" / "dev" / "laps.jsonl"


def meta(what: str) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (DEV / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def qp() -> None:
    from f1race.control.lmpc import ATVLearningMPC
    from f1race.control.nmpc import FS
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_a, cx_d, cx_e
    from f1race.track import Track

    meta("qp")
    tr = Track.load("Spielberg")
    t0 = time.time()
    m = ATVLearningMPC(tr, DynVehicle(), n_pts=40, N=cx_e.N, dt=cx_e.MPC_DT, eta=cx_e.ETA,
                       margin=cx_d.NMPC_SETTING["margin"], w_T=cx_e.W_T)
    build_s = time.time() - t0
    rng = np.random.default_rng(0)
    nz, npar = m._nlp["x"].shape[0], m._nlp["p"].shape[0]
    z = rng.standard_normal(nz) * 0.1
    z[m.Xi[:, 0]] = 500.0 + 2.0 * np.arange(m.N + 1)
    z[m.Xi[:, 3]] = 40.0
    p = rng.standard_normal(npar) * 0.1
    J = np.asarray(m._jg(z, p).full())
    errs = []
    for j in rng.choice(nz, 60, replace=False):
        e = np.zeros(nz)
        e[j] = 1e-6
        fd = (np.asarray(m._gf(z + e, p)).ravel() - np.asarray(m._gf(z - e, p)).ravel()) / 2e-6
        errs.append(float(np.max(np.abs(J[:, j] - fd) / (1.0 + np.abs(fd)))))
    # a feasible candidate: a rollout of the nominal step along the centre line at moderate speed
    out = {"build_s": build_s, "jacobian_max_rel_err": max(errs), "feasible": []}
    for i, v in ((200, 30.0), (1500, 45.0), (3000, 25.0)):
        x = np.array([tr.s[i], 0.0, 0.0, v, 0.0, 0.0, 0.0])
        U = np.tile([0.0, 1500.0, 0.0], (m.N, 1))
        X = [x]
        for k in range(m.N):
            X.append(np.asarray(m._step(X[-1], U[k] / np.array([1.0, FS, FS]))).ravel())
        X = np.array(X)
        D = np.column_stack([X[-1, :6] + 0.3 * j for j in range(m.n_pts)])
        c = 10.0 + np.arange(m.n_pts, dtype=float)
        u, info = m.solve_qp(X[0], U[0], D, c, None, m.candidate_vector(X, U))
        out["feasible"].append({"s": float(tr.s[i]), "v": v, **{k: info[k] for k in ("status", "fallback", "slack",
                                                                                     "limit_slack", "term_slack", "time")}})
    (DEV / "qp.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def nlp_repro() -> None:
    from f1race.experiments import cx_e

    meta("nlp_repro")
    ref = next(json.loads(l) for l in E_DEV.read_text().splitlines()
               if l.strip() and json.loads(l).get("job") == "dev|lmpc|K=20|NOM_car0")
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], "lmpc", {"K": 20})
    out = {"wall_s": time.time() - t0, "phase_e": {"lap_times": ref["lap_times"], "first_failed": ref["first_failed_learning_lap"],
                                                  "laps": [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason"))
                                                           for l in ref["laps"][:4]]},
           "now": {"lap_times": r["lap_times"], "first_failed": r["first_failed_learning_lap"],
                   "laps": [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in r["laps"][:4]]}}
    out["equal"] = out["phase_e"]["lap_times"] == out["now"]["lap_times"] and out["phase_e"]["laps"] == out["now"]["laps"]
    (DEV / "nlp_repro.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


DIAG_SETTING = {"K": 20, "qp_solver": "qpoases", "trust": [0.2, 1000.0]}


def diag(workers: int) -> None:
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_cx_h

    meta("diag")
    names = run_cx_h._names("NOM", "DEV")
    run_cx_h._run(run_cx_h._jobs("diag", "atv", DIAG_SETTING, names), DEV / "diag.jsonl", workers)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["qp", "nlp-repro", "diag"])
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    if a.what == "diag":
        diag(a.workers)
    else:
        {"qp": qp, "nlp-repro": nlp_repro}[a.what]()


if __name__ == "__main__":
    main()
