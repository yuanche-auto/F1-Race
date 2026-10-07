"""L0 numerical studies (review follow-up): grid convergence of the OCP, resolution of the
QSS profile, and sensitivity of the closed loop to the simulator / control step.

    python scripts/l0_studies.py        # ~4 min; writes results/L0/studies.json, prints tables
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.control.path_tracking import PathTracker  # noqa: E402
from f1race.experiments.l0 import plan_from_points  # noqa: E402
from f1race.path import Path as RPath  # noqa: E402
from f1race.planning.mincurv import min_curvature_line  # noqa: E402
from f1race.planning.ocp_kinematic import initial_guess_from_path, solve_min_time  # noqa: E402
from f1race.planning.qss import speed_profile  # noqa: E402
from f1race.sim.kinematic import KinematicSim  # noqa: E402
from f1race.track import Track  # noqa: E402
from f1race.vehicle import Vehicle  # noqa: E402


def closed_loop(track, veh, path, v, a, k, dt):
    ctl = PathTracker(path, v, a, veh, dt, kappa_ref=k)
    sim = KinematicSim(track, veh, dt, stop_on_violation=False)
    k0 = path.kappa[0] if k is None else k[0]
    x0 = np.array([path.x[0], path.y[0], path.psi[0], v[0], np.arctan(veh.L * k0)])
    r = sim.run(ctl, x0, t_max=150)
    e = np.array([l[2] for l in ctl.log])
    return {"lap_time": r.lap_time, "max_usage": r.max_usage, "n_over_1": int((r.usage > 1).sum()),
            "min_edge_margin": r.min_margin, "max_abs_e": float(np.abs(e).max())}


def main() -> None:
    track, veh = Track.load("Spielberg"), Vehicle()
    alpha, mc = min_curvature_line(track, veh, margin=0.3)
    idx = np.arange(0, len(track.s), 3)
    qk = speed_profile(mc, veh, 1.0)
    init = initial_guess_from_path(track, veh, idx, alpha, mc, qk.v, qk.a)
    out = {"ocp_grid": {}, "qss_resolution": {}, "dt_sensitivity": {}}

    for ds in (8.0, 4.0, 2.0, 1.0):
        t0 = time.time()
        o = solve_min_time(track, veh, init, ds=ds, eta=1.0)
        out["ocp_grid"][ds] = {"lap_time": o.lap_time, "success": o.success, "iterations": o.iterations,
                               "seconds": time.time() - t0}
    xy = np.column_stack([mc.x, mc.y])
    out["qss_resolution"]["3 (QP points)"] = speed_profile(mc, veh, 1.0).lap_time
    for res in (1.0, 0.5, 0.25):
        out["qss_resolution"][res] = speed_profile(RPath.from_points(xy, resample=res), veh, 1.0).lap_time

    eta = 0.97
    md = RPath.from_points(xy, resample=0.5)
    qp = speed_profile(md, veh, eta)
    op = solve_min_time(track, veh, init, ds=2.0, eta=eta)
    plans = {"mincurv_qss": (md, qp.v, qp.a, None),
             "ocp": plan_from_points(op.xy(track), op.v, kappa=np.tan(op.delta) / veh.L)}
    for name, (path, v, a, k) in plans.items():
        for dt in (0.005, 0.01, 0.02, 0.04):
            out["dt_sensitivity"][f"{name} dt={dt}"] = closed_loop(track, veh, path, v, a, k, dt)

    (ROOT / "results" / "L0").mkdir(parents=True, exist_ok=True)
    (ROOT / "results" / "L0" / "studies.json").write_text(json.dumps(out, indent=2, default=str))
    print("| OCP grid ds [m] | lap time [s] | iterations |")
    for ds, r in out["ocp_grid"].items():
        print(f"| {ds} | {r['lap_time']:.3f} | {r['iterations']} |")
    print("\n| QSS path resolution [m] | lap time [s] |")
    for k, v in out["qss_resolution"].items():
        print(f"| {k} | {v:.3f} |")
    print("\n| plan, dt | closed-loop lap [s] | max usage | steps > 1 | min edge margin [m] | max lateral error [m] |")
    for k, r in out["dt_sensitivity"].items():
        print(f"| {k} | {r['lap_time']:.3f} | {r['max_usage']:.4f} | {r['n_over_1']} | "
              f"{r['min_edge_margin']:.3f} | {r['max_abs_e']:.3f} |")


if __name__ == "__main__":
    main()
