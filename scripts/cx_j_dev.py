"""Control expansion phase J – development checks (docs/CX_J_protocol.md section 9; NOM and DEV only). Windows `.venv`;
records in results/CX/J/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_j_dev.py repro     # 9.1: with the new options off, phase C's tube and phase D's adaptive reproduce
                                         # their NOM records (worktrees F1-Race-wt-CXC-run / CXD-run)
    python scripts/cx_j_dev.py drifts    # 9.5: the drifts drawn for JD (and JV), before any JV lap
    python scripts/run_cx_j.py refs      # the parents' laps (9.2, 9.4)
    python scripts/cx_j_dev.py replay    # 9.2: the RLS on the recorded laps - sigma, errors, q, the capped shares per grid setting
    python scripts/run_cx_j.py gate      # 9.4
    python scripts/cx_j_dev.py gate      # 9.4: the gate's table
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
import numpy as np  # noqa: E402

from f1race.experiments import cx_c, cx_d, cx_j  # noqa: E402

DEV = cx_j.OUT / "dev"
WT = Path(r"C:\Users\yuanche\F1-Race-worktrees")
COMP = ("s", "n", "xi", "vx", "vy", "r", "delta")


def meta(what: str) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (DEV / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def _records(path: Path, kind: str, case: str) -> dict:
    for l in path.read_text().splitlines():
        if l.strip():
            d = json.loads(l)
            if d.get("kind") == kind and d.get("case") == case and "error" not in d:
                return d
    raise KeyError((path, kind, case))


def repro() -> None:
    meta("repro")
    out = []
    for kind, path, case_name, setting in (
            ("tube", WT / "F1-Race-wt-CXC-run" / "results" / "CX" / "C" / "test.jsonl", "NOM_car0", {}),
            ("adaptive", WT / "F1-Race-wt-CXD-run" / "results" / "CX" / "D" / "test.jsonl", "NOM_car0", {})):
        ref = _records(path, kind, case_name)
        now = cx_j.lap(next(c for c in cx_j.cases("NOM_C")), kind, setting)
        keys = ("completed", "lap_time", "stop_reason", "distance", "min_edge_margin", "solves", "fallbacks",
                "tyre_usage_max_front_rear", "steer_rate_rms")
        eq = {k: ref.get(k) == now.get(k) for k in keys}
        out.append({"kind": kind, "equal": all(eq.values()), "fields": eq, "record": {k: ref.get(k) for k in keys},
                    "now": {k: now.get(k) for k in keys}})
        print(kind, "equal", all(eq.values()), ref["lap_time"], now["lap_time"], flush=True)
    (DEV / "repro.json").write_text(json.dumps({"all_equal": all(o["equal"] for o in out), "checks": out}, indent=1,
                                               default=float))


def drifts() -> None:
    meta("drifts")
    res = {}
    for name in ("JD", "JV_D"):
        cs = cx_j.cases(name)
        d = np.array([[c["drift"]["s_on"], c["drift"]["ell"], c["drift"]["f_end"]] for c in cs])
        mu = np.array([cx_d.true_theta(c)[0] for c in cs])
        end = d[:, 0] + d[:, 1]
        q = lambda a: [float(v) for v in (np.min(a), *np.percentile(a, (25, 50, 75)), np.max(a))]  # noqa: E731
        res[name] = {"n": len(cs), "s_on": q(d[:, 0]), "ell": q(d[:, 1]), "f_end": q(d[:, 2]),
                     "mu_f_end": q(mu * d[:, 2]), "cars_mu_f_end_below_0.70": int(np.sum(mu * d[:, 2] < 0.70)),
                     "share_end_before_2700m": float(np.mean(end < 2700.0)),
                     "share_end_before_3830m": float(np.mean(end < 3830.0)),
                     "share_onset_before_2700m": float(np.mean(d[:, 0] < 2700.0))}
    (DEV / "drifts.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


def _wrap(a, L):
    return (a + 0.5 * L) % L - 0.5 * L


def _replay_job(path: str) -> dict:
    from f1race.control.adaptive_mpc import applied_input
    from f1race.control.nmpc import MPCController
    from f1race.control.radaptive import CAP, KAPPA, MIN_RESIDUALS, QUANTILE, WINDOW
    from f1race.dynamics import DynVehicle
    from f1race.estimation.identify import HI, LO
    from f1race.estimation.rls import RecursiveLS
    from f1race.track import Track

    p = Path(path)
    kind, case_name = p.stem.split("_", 1)
    z = np.load(p)
    case = cx_j.case_by_name(case_name)
    tr = Track.load(case["track"])
    veh = DynVehicle()
    t, x, u, pred1 = z["t"], z["x"], z["u"], z["pred1"]
    est = RecursiveLS(veh, 0.01, lam_mu=cx_j.LAM_MU)
    fr = MPCController.__new__(MPCController)
    fr.track, fr.s_unwrapped = tr, None
    xf = np.array([MPCController.frenet(fr, xi) for xi in x])
    field = cx_j.make_sim(case, tr)
    f_fn = cx_d.friction_field(field)
    truth = cx_d.true_theta(case)
    th_log, sig_log, s_log, i_log = [], [], [], []
    for k in range(len(t)):
        if k:
            est.update(x[k - 1, 3:7], applied_input(veh, x[k - 1], u[k - 1]), x[k, 3:6])
        if k % 5 == 0:
            th_log.append(est.theta.copy())
            sig_log.append(np.sqrt(np.maximum(np.diag(est.P), 0.0)))
            s_log.append(xf[k, 0])
            i_log.append(k)
    th, sig, s = np.array(th_log), np.array(sig_log), np.array(s_log)
    f = np.array([float(f_fn(q)) for q in s]) if f_fn is not None else np.ones(len(s))
    true = np.tile(truth, (len(s), 1))
    true[:, 0] = truth[0] * f
    err = np.abs(th - true)
    h = np.minimum(CAP, KAPPA * sig)
    # residuals of the recorded first-stage predictions against the measured Frenet state 50 ms later
    ok = pred1[:, 1] == 0.0
    res = []
    L = tr.length
    for j in range(len(pred1) - 1):
        if not ok[j]:
            continue
        k = int(round(pred1[j + 1, 0] / 0.01))
        if k >= len(xf) or abs(pred1[j + 1, 0] - pred1[j, 0] - 0.05) > 1e-6:
            continue
        d = xf[k] - pred1[j, 2:9]
        d[0] = _wrap(d[0], L)
        res.append((pred1[j + 1, 0], np.abs(d)))
    R = np.array([r[1] for r in res])
    Rt = np.array([r[0] for r in res])
    q_log = []
    for j in range(len(R)):
        w = R[max(0, j + 1 - WINDOW):j + 1]
        q_log.append(np.quantile(w, QUANTILE, axis=0) if len(w) >= MIN_RESIDUALS else np.zeros(7))
    q = np.array(q_log) if q_log else np.zeros((1, 7))
    # capped shares of the grid's settings on the whole predictions (every tenth solve), with the estimate of the replay
    ctl, mpc, _, _ = cx_j.build("ra", cx_j.DEV_SETTING, tr, case, cx_j.make_sim(case, tr))
    tube = ctl.tube
    shares: dict = {}
    for i, tp in enumerate(z["pred_t"]):
        k5 = int(np.searchsorted(i_log, int(round(tp / 0.01))))
        k5 = min(k5, len(th) - 1)
        qi = int(min(np.searchsorted(Rt, tp, side="right") - 1, len(q) - 1)) if len(Rt) else 0
        qq = q[max(qi, 0)]
        lo, hi = np.maximum(th[k5] - h[k5], LO), np.minimum(th[k5] + h[k5], HI)
        lo_c, hi_c = np.maximum(th[k5] - CAP, LO), np.minimum(th[k5] + CAP, HI)
        X, U = z["pred_X"][i].astype(float), z["pred_U"][i].astype(float)
        for g in cx_j.RHO_GRID:
            tb = tube.compute(X, U, th[k5], lo, hi, g["rho_s"] * qq, g["rho0"] * qq)
            r = shares.setdefault(cx_j.setting_key(g), [])
            r.append((tb["capped_eta_share"], tb["capped_track_share"], float(np.mean(tb["eta"][1:])),
                      float(tb["backoff"][10]), float(tb["backoff"][-1])))
        tb = tube.compute(X, U, th[k5], lo_c, hi_c)
        shares.setdefault("cap_box_no_residual", []).append((tb["capped_eta_share"], tb["capped_track_share"],
                                                             float(np.mean(tb["eta"][1:])), float(tb["backoff"][10]),
                                                             float(tb["backoff"][-1])))
    return {"kind": kind, "case": case_name, "n_solves": int(len(pred1)), "n_res": int(len(R)),
            "sigma_p50": np.median(sig, axis=0).tolist(), "two_sigma_below_cap_share": np.mean(KAPPA * sig < CAP, axis=0).tolist(),
            "all_at_cap_share": float(np.mean(np.all(h == CAP, axis=1))),
            "err_p50": np.median(err, axis=0).tolist(), "err_within_2sigma_share": np.mean(err <= 2 * sig, axis=0).tolist(),
            "err_within_h_share": np.mean(err <= h, axis=0).tolist(),
            "q_p50": np.median(q, axis=0).tolist(), "q_p95": np.percentile(q, 95, axis=0).tolist(),
            "residual_p90": np.quantile(R, 0.9, axis=0).tolist() if len(R) else None,
            "shares": {k: np.mean(np.array(v), axis=0).tolist() for k, v in shares.items()}}


def replay(workers: int) -> None:
    from multiprocessing import get_context

    meta("replay")
    files = sorted(str(p) for p in (DEV / "replay_laps").glob("*.npz"))
    with get_context("spawn").Pool(min(workers, len(files)), maxtasksperchild=1) as pool:
        rec = list(pool.imap_unordered(_replay_job, files))
    (DEV / "replay.json").write_text(json.dumps(rec, indent=1, default=float))
    print(len(rec), "laps replayed")


def gate() -> None:
    meta("gate_table")                                   # the run's own record is written by run_cx_j.py (review m4)
    refs = [json.loads(l) for l in (DEV / "refs.jsonl").read_text().splitlines() if l.strip()]
    ra = [json.loads(l) for l in (DEV / "gate.jsonl").read_text().splitlines() if l.strip()]
    allr = refs + ra
    err = [r["job"] for r in allr if "error" in r]
    done = lambda k, c: {r["case"]: bool(r["completed"]) for r in allr if r["kind"] == k and r["case"].startswith(c)}  # noqa: E731
    kinds = list(cx_j.PARENTS) + list(cx_j.RA_KINDS)
    table = {}
    for k in kinds:
        table[k] = {}
        for c in cx_j.CONDITIONS:
            dev, nom = done(k, f"DEV_{c}_"), done(k, f"NOM_{c}_")
            table[k][c] = {"dev": [sum(dev.values()), len(dev)], "nom": [sum(nom.values()), len(nom)]}
    num = lambda k, cs: sum(table[k][c]["dev"][0] for c in cs)  # noqa: E731
    g1 = table["ra"]["C"]["nom"] == [1, 1]
    a2, r2 = num("adaptive", "CD"), num("ra", "CD")
    t3, r3 = num("tube", "NL"), num("ra", "NL")
    info2, info3 = a2 >= 3, t3 >= 3
    g2 = info2 and r2 >= a2 - 1
    g3 = info3 and r3 >= int(np.ceil(t3 / 2))
    out = {"errors": err, "table": table, "gate": {"1_nom_constant": g1, "2_ra": r2, "2_adaptive": a2, "2_information": info2,
                                                   "2_passed": bool(g2), "3_ra": r3, "3_tube": t3, "3_information": info3,
                                                   "3_passed": bool(g3), "passed": bool(g1 and g2 and g3)}}
    (DEV / "gate.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out["gate"], indent=1))
    for k in kinds:
        print(k, {c: (table[k][c]["dev"], table[k][c]["nom"]) for c in cx_j.CONDITIONS})


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["repro", "drifts", "replay", "gate"])
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    {"repro": repro, "drifts": drifts, "gate": gate}.get(a.what, lambda: replay(a.workers))()


if __name__ == "__main__":
    main()
