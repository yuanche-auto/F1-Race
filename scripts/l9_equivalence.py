"""L9 equivalence check of the acados NMPC (docs/L9_protocol.md section 4; validation set V only).

Four configurations of the L2 NMPC – ``nmpc`` (5 % plan, eta 0.95), ``nmpc_reserve`` (12 % plan,
eta 0.88), ``smpc`` (L3 back-offs and per-stage eta) on V's cars without lag, and the lag-aware
NMPC (the nominal model with tau = 30 ms) on V's cars with 30 ms lag. The IPOPT NMPC drives each
lap (offline mode); every solve's problem is logged (measured state, previous input, the shifted
previous solution IPOPT starts from, IPOPT's first command). 50 problems per configuration are
drawn with seed 0 (200 in all); acados SQP, started from the same initial guess, solves each to
convergence. Criterion (pre-registered): the first commands agree within 2 % of each input's range
(steering rate 2 x delta_rate_max, forces 20 kN) in at least 95 % of the 200 problems.

    python scripts/l9_equivalence.py run --workers 20       # in WSL (source ~/f1race-env.sh)
    python scripts/l9_equivalence.py report
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
from multiprocessing import get_context
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

OUT = ROOT / "results" / "L9" / "equivalence"
CONFIGS = ("nmpc", "nmpc_reserve", "smpc", "lag")
PER_CONFIG = 50
TOL = 0.02


def config(name: str):
    from f1race.experiments.l6 import smpc_spec
    from f1race.experiments.l9 import plans

    P = plans()
    if name == "nmpc":
        return P[0.95], 0.95, None, None
    if name == "nmpc_reserve":
        return P[0.88], 0.88, None, None
    if name == "smpc":
        sp = smpc_spec(json.loads((ROOT / "results" / "L3" / "calibration.json").read_text()))
        return P[sp["plan_eta"]], np.asarray(sp["eta"], float), np.asarray(sp["backoff"], float), None
    return P[0.95], 0.95, None, 0.03


def job_log(args):
    """One IPOPT lap of a configuration on a V case; returns the logged problems."""
    cfg, case_name = args
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.experiments.l9 import RealTimeLagMPCController, RealTimeMPCController, cases, make_mpc, make_sim
    from f1race.track import Track

    case = next(c for c in cases("V") if c["name"] == case_name)
    track, nom = Track.load("Spielberg"), DynVehicle()
    o, eta, backoff, lag = config(cfg)
    mpc = make_mpc("ipopt", track, nom, o, eta, backoff, lag)
    logs = []
    orig = mpc.solve

    def logged(xf, u_prev, **kw):
        z0 = None
        if mpc.z is not None:
            z0 = mpc.z.copy()
            z0[mpc.Xi[:-1]] = mpc.z[mpc.Xi[1:]]
            z0[mpc.Ui[:-1]] = mpc.z[mpc.Ui[1:]]
            z0[mpc.Xi[0]] = xf
        u, info = orig(xf, u_prev, **kw)
        if z0 is not None and not info["fallback"]:
            logs.append({"x0": np.array(xf), "u_prev": np.array(u_prev), "X": z0[mpc.Xi].copy(),
                         "U": z0[mpc.Ui].copy(), "u_ipopt": np.array(u), "status": info["status"]})
        return u, info

    mpc.solve = logged
    path, _ = dyn_reference(track, nom, o)
    ctl = RealTimeLagMPCController(mpc, 0.01, "offline") if lag else RealTimeMPCController(mpc, 0.01, "offline")
    r = make_sim(case, track).run(ctl, x0_from_plan(o, path), t_max=121.0)
    return {"config": cfg, "case": case_name, "completed": bool(r.completed), "distance": float(r.progress[-1]),
            "problems": logs}


def job_check(args):
    """acados SQP from the same initial guess on a batch of problems of one configuration."""
    cfg, problems, wid = args
    from f1race.control.acados_mpc import AcadosFrenetMPC
    from f1race.control.nmpc import FS
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import ref_from_plan
    from f1race.track import Track

    track, nom = Track.load("Spielberg"), DynVehicle()
    o, eta, backoff, lag = config(cfg)
    mpc = AcadosFrenetMPC(track, nom, ref_from_plan(o), N=30, dt=0.05, eta=eta, backoff=backoff, steer_lag=lag,
                          nlp="SQP", name=f"eq_{cfg}_{wid}_{os.getpid()}")
    rng = np.array([2 * nom.delta_rate_max, 2e4, 2e4])
    out = []
    for p in problems:
        up = p["u_prev"] / np.array([1.0, FS, FS])
        Ua = [up] + [p["U"][i] for i in range(len(p["U"]))]
        for i in range(mpc.N + 1):                   # IPOPT's initial guess, augmented by the previous input
            mpc.solver.set(i, "x", np.concatenate([p["X"][i], Ua[i]]))
        for i in range(mpc.N):
            mpc.solver.set(i, "u", p["U"][i])
        mpc.initialised = True
        mpc._last_plan = None
        u, info = mpc.solve(p["x0"], p["u_prev"])
        dev = np.abs(u - p["u_ipopt"]) / rng
        out.append({"config": cfg, "dev": dev.tolist(), "max_dev": float(dev.max()), "status": info["status"],
                    "iter": info["iter"], "ipopt_status": p["status"]})
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["run", "report"])
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if a.what == "run":
        from f1race.experiments.l9 import cases

        V = cases("V")
        jobs = [(c, v["name"]) for c in CONFIGS for v in V
                if v["dist_seed"] is None and abs(v["tau"] - (0.03 if c == "lag" else 0.0)) < 1e-9]
        with get_context("spawn").Pool(a.workers, maxtasksperchild=1) as pool:
            logs = pool.map(job_log, jobs)
        pickle.dump(logs, open(OUT / "logs.pkl", "wb"))
        rng = np.random.default_rng(0)
        batches = []
        for c in CONFIGS:
            probs = [p for lg in logs if lg["config"] == c for p in lg["problems"]]
            idx = sorted(rng.choice(len(probs), size=min(PER_CONFIG, len(probs)), replace=False))
            sel = [probs[i] for i in idx]
            batches += [(c, sel[k::5], k) for k in range(5)]
        with get_context("spawn").Pool(a.workers, maxtasksperchild=1) as pool:
            res = [r for b in pool.map(job_check, batches) for r in b]
        (OUT / "check.json").write_text(json.dumps({"laps": [{k: v for k, v in lg.items() if k != "problems"} | {
            "problems": len(lg["problems"])} for lg in logs], "results": res}, indent=1))
    d = json.loads((OUT / "check.json").read_text())
    res = d["results"]
    md = ["# L9 equivalence check (validation set V)", "",
          f"IPOPT laps logged: {len(d['laps'])}; problems checked: {len(res)} (criterion: within {TOL:.0%} in >= 95 %)", "",
          "| configuration | problems | within 2 % | max scaled deviation: median / P95 / max | acados SQP status |",
          "|---|---|---|---|---|"]
    for c in CONFIGS + ("all",):
        rs = res if c == "all" else [r for r in res if r["config"] == c]
        m = np.array([r["max_dev"] for r in rs])
        st = {}
        for r in rs:
            st[r["status"]] = st.get(r["status"], 0) + 1
        md.append(f"| {c} | {len(rs)} | {np.mean(m <= TOL):.3f} | {np.median(m):.4f} / {np.percentile(m, 95):.4f} / "
                  f"{m.max():.4f} | {st} |")
    ok = np.mean([r["max_dev"] <= TOL for r in res]) >= 0.95
    md += ["", f"**Criterion {'met' if ok else 'NOT met'}.**"]
    (OUT / "equivalence.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
