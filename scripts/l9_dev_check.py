"""L9 development check of the acados NMPC (not a protocol run; Spielberg, nominal car):

1. the IPOPT NMPC (FrenetMPC) drives 20 s; every 10th solve, the acados SQP (to convergence) solves
   the same problem (same state, previous input; warm-started from its own previous solution) and
   the first commands are compared;
2. one closed-loop lap with the acados SQP-RTI NMPC in the offline mode: outcome, lap time, the
   feedback and preparation times.

    python scripts/l9_dev_check.py          # in WSL: source ~/f1race-env.sh first
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from f1race.control.acados_mpc import AcadosFrenetMPC  # noqa: E402
from f1race.control.nmpc import FrenetMPC, MPCController  # noqa: E402
from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import ref_from_plan, x0_from_plan  # noqa: E402
from f1race.experiments.l8 import plans_for  # noqa: E402
from f1race.sim.dynamic import DynamicSim  # noqa: E402
from f1race.track import Track  # noqa: E402

RANGE = np.array([2 * DynVehicle().delta_rate_max, 2e4, 2e4])          # protocol section 4: rate range, 20 kN

track, nom = Track.load("Spielberg"), DynVehicle()
o = plans_for("Spielberg")[0.95]
path, _ = dyn_reference(track, nom, o)
ref = ref_from_plan(o)
x0 = x0_from_plan(o, path)

t0 = time.time()
sqp = AcadosFrenetMPC(track, nom, ref, eta=0.95, nlp="SQP", name="dev_sqp")
print(f"acados SQP built in {time.time() - t0:.0f} s", flush=True)
ip = FrenetMPC(track, nom, ref, mode="track", N=30, dt=0.05, eta=0.95)
orig = ip.solve
dev, sqp_iter, sqp_status = [], [], []
k = {"n": 0}


def probe(xf, u_prev, **kw):
    u, info = orig(xf, u_prev, **kw)
    k["n"] += 1
    ua, ia = sqp.solve(xf, u_prev)
    if k["n"] % 10 == 0:
        dev.append(np.abs(ua - u) / RANGE)
        sqp_iter.append(ia["iter"])
        sqp_status.append(ia["status"])
    return u, info


ip.solve = probe
r = DynamicSim(track, nom, 0.01).run(MPCController(ip, 0.01), x0, t_max=20.0)
d = np.array(dev)
print(f"equivalence probe: {len(d)} states; max scaled deviation of the first command: median {np.median(d.max(1)):.4f}, "
      f"P95 {np.percentile(d.max(1), 95):.4f}, max {d.max():.4f}; within 2 %: {np.mean(d.max(1) <= 0.02):.3f}; "
      f"SQP iterations median {np.median(sqp_iter):.0f}, status {sorted(set(sqp_status))}", flush=True)

t0 = time.time()
rti = AcadosFrenetMPC(track, nom, ref, eta=0.95, nlp="SQP_RTI", name="dev_rti")
print(f"acados RTI built in {time.time() - t0:.0f} s", flush=True)
r = DynamicSim(track, nom, 0.01).run(MPCController(rti, 0.01), x0, t_max=121.0)
fb = np.array([s["time_feedback"] for s in rti.stats]) * 1e3
pr = np.array([s["time_preparation"] for s in rti.stats]) * 1e3
print(f"RTI lap: {r.stop_reason}, lap {r.lap_time:.2f} s (IPOPT nominal lap for reference: run separately), "
      f"{len(fb)} calls, fallbacks {sum(s['fallback'] for s in rti.stats)}; feedback ms mean {fb.mean():.2f} "
      f"P99 {np.percentile(fb, 99):.2f} max {fb.max():.2f}; preparation ms mean {pr.mean():.2f} P99 "
      f"{np.percentile(pr, 99):.2f} max {pr.max():.2f}", flush=True)
