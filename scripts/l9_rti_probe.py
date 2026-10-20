"""Feasibility probe for L9 (not a protocol run; docs/L9_protocol.md section 12): time of one
SQP-RTI step of the L2 NMPC NLP with casadi's sqpmethod and its built-in QP solvers, against the
IPOPT solve, on Spielberg's nominal car along the 5 % plan (closed loop with IPOPT, RTI timed on
the same states).

    python scripts/l9_rti_probe.py [qrqp osqp hpipm]
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import casadi as ca
import numpy as np

from f1race.control.nmpc import FrenetMPC, MPCController
from f1race.dynamics import DynVehicle
from f1race.experiments.l1 import dyn_reference
from f1race.experiments.l2 import ref_from_plan, x0_from_plan
from f1race.experiments.l8 import plans_for
from f1race.sim.dynamic import DynamicSim
from f1race.track import Track

track, nom = Track.load("Spielberg"), DynVehicle()
o = plans_for("Spielberg")[0.95]
path, _ = dyn_reference(track, nom, o)
mpc = FrenetMPC(track, nom, ref_from_plan(o), mode="track", N=30, dt=0.05, eta=0.95)
ctl = MPCController(mpc, 0.01)
nlp = mpc._nlp
print("variables", nlp["x"].numel(), "constraints", nlp["g"].numel())

solvers = {}
for qp in sys.argv[1:] or ["qrqp", "osqp", "hpipm"]:
    opts = {"max_iter": 1, "qpsol": qp, "print_time": False, "print_header": False, "print_iteration": False,
            "print_status": False, "error_on_fail": False, "convexify_strategy": "regularize", "convexify_margin": 1e-6}
    if qp == "qrqp":
        opts["qpsol_options"] = {"print_iter": False, "print_header": False, "error_on_fail": False}
    elif qp == "osqp":
        opts["qpsol_options"] = {"osqp": {"verbose": False, "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 4000},
                                 "error_on_fail": False}
    elif qp == "hpipm":
        opts["qpsol_options"] = {"error_on_fail": False}
    try:
        t0 = time.time()
        solvers[qp] = ca.nlpsol(f"rti_{qp}", "sqpmethod", nlp, opts)
        print(qp, "built in %.1f s" % (time.time() - t0))
    except Exception as e:                               # noqa: BLE001
        print(qp, "build failed:", str(e)[:200])

sim = DynamicSim(track, nom, 0.01)
x0 = x0_from_plan(o, path)
times = {k: [] for k in solvers}
du = {k: [] for k in solvers}
state = {"n": 0}
orig_solve = mpc.solve


def probe_solve(xf, u_prev, **kw):
    z_prev, lg, lx = mpc.z, mpc.lam_g, mpc.lam_x
    u, info = orig_solve(xf, u_prev, **kw)
    state["n"] += 1
    if z_prev is not None and state["n"] % 10 == 0:
        p = np.concatenate([xf, [u_prev[0], u_prev[1] / 1e4, u_prev[2] / 1e4]])
        z0 = z_prev.copy()
        z0[mpc.Xi[:-1]] = z_prev[mpc.Xi[1:]]
        z0[mpc.Ui[:-1]] = z_prev[mpc.Ui[1:]]
        z0[mpc.Xi[0]] = xf
        for k, s in solvers.items():
            t0 = time.perf_counter()
            sol = s(x0=z0, p=p, lbx=mpc.lbz, ubx=mpc.ubz, lbg=mpc.lbg, ubg=mpc.ubg, lam_g0=lg, lam_x0=lx)
            times[k].append(time.perf_counter() - t0)
            ur = np.asarray(sol["x"]).ravel()[mpc.Ui[0]] * np.array([1.0, 1e4, 1e4])
            du[k].append(np.abs(ur - u) / np.array([0.5, 2e4, 2e4]))
    return u, info


mpc.solve = probe_solve
r = sim.run(ctl, x0, t_max=25.0)
ip = [x["time"] for x in mpc.stats]
print("IPOPT per solve: mean %.1f ms, P95 %.1f, max %.1f (n=%d)" % (1e3 * np.mean(ip), 1e3 * np.percentile(ip, 95),
                                                                       1e3 * np.max(ip), len(ip)))
for k in solvers:
    t = np.array(times[k]) * 1e3
    d = np.array(du[k])
    print(f"RTI {k}: mean {t.mean():.1f} ms, P95 {np.percentile(t, 95):.1f}, max {t.max():.1f} (n={len(t)}); "
          f"first-input deviation from IPOPT (scaled) median {np.median(d.max(axis=1)):.3f}, P95 {np.percentile(d.max(axis=1), 95):.3f}")
