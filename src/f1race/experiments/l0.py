"""L0 experiment: deterministic kinematic model.

1. minimum-curvature line + QSS speed profile (planning margin eta_plan)
2. minimum-time OCP (ideal eta = 1 and with the planning margin)
3. closed-loop verification of both plans in the time-domain simulator
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path as FsPath

import numpy as np

from ..control.path_tracking import PathTracker
from ..path import Path
from ..planning.mincurv import min_curvature_line
from ..planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from ..planning.qss import speed_profile
from ..sim.kinematic import KinematicSim, SimResult
from ..track import Track
from ..vehicle import Vehicle


@dataclass
class L0Config:
    track: str = "Spielberg"
    margin: float = 0.3          # extra distance to the edges [m]
    eta_plan: float = 0.97       # friction fraction used by the plans that are executed
    ocp_ds: float = 4.0          # OCP grid [m]
    dt: float = 0.02             # simulator step [s]
    mincurv_step: int = 3        # every k-th centreline sample for the QP


@dataclass
class L0Result:
    config: dict
    centreline_qss: float
    mincurv_qss_ideal: float
    mincurv_qss_plan: float
    ocp_ideal: float
    ocp_plan: float
    closed_loop: dict = field(default_factory=dict)
    timing: dict = field(default_factory=dict)


def plan_from_points(xy: np.ndarray, v: np.ndarray, smooth: float = 0.0, ds: float = 0.5,
                     kappa: np.ndarray | None = None):
    """Dense reference for tracking: the spline through the points re-sampled every ``ds``
    metres (a coarse polyline would add chord errors of ds^2 kappa / 8 to the measured
    lateral error), with the speed interpolated in arclength and the matching
    acceleration a = d(v^2)/(2 ds). ``kappa`` (on the points) is the plan's own steering
    curvature, interpolated likewise and used as feed-forward."""
    knots = Path.from_points(xy, smooth=smooth)
    dense = Path.from_points(xy, smooth=smooth, resample=ds)
    vd = np.interp(dense.s, knots.s, v, period=knots.length)
    vn = np.roll(vd, -1)
    a = (vn**2 - vd**2) / (2.0 * dense.ds)
    kd = None if kappa is None else np.interp(dense.s, knots.s, kappa, period=knots.length)
    return dense, vd, a, kd


def run_closed_loop(track: Track, veh: Vehicle, path: Path, v: np.ndarray, a: np.ndarray,
                    kappa_ref: np.ndarray | None, dt: float) -> SimResult:
    ctl = PathTracker(path, v, a, veh, dt, kappa_ref=kappa_ref)
    sim = KinematicSim(track, veh, dt)
    x0 = np.array([path.x[0], path.y[0], path.psi[0], v[0], np.arctan(veh.L * path.kappa[0])])
    res = sim.run(ctl, x0, t_max=3.0 * track.length / max(v.mean(), 1.0))
    log = np.array([l[1:] for l in ctl.log])
    res.info["max_abs_e"] = float(np.abs(log[:, 1]).max())
    res.info["max_dv"] = float(np.max(np.abs(res.state[: len(log), 3] - log[:, 3])))
    return res


def run(cfg: L0Config = L0Config(), outdir: str | FsPath | None = None, verbose: bool = True):
    t_all = time.time()
    track = Track.load(cfg.track)
    veh = Vehicle()
    timing = {}

    cl = Path.from_points(np.column_stack([track.x[::cfg.mincurv_step], track.y[::cfg.mincurv_step]]))
    centre = speed_profile(cl, veh).lap_time

    t0 = time.time()
    alpha, mc = min_curvature_line(track, veh, margin=cfg.margin, step=cfg.mincurv_step)
    timing["mincurv_s"] = time.time() - t0
    q_ideal = speed_profile(mc, veh, 1.0)
    q_plan = speed_profile(mc, veh, cfg.eta_plan)
    idx = np.arange(0, len(track.s), cfg.mincurv_step)
    init = initial_guess_from_path(track, veh, idx, alpha, mc, q_ideal.v, q_ideal.a)

    t0 = time.time()
    o_ideal = solve_min_time(track, veh, init, ds=cfg.ocp_ds, margin=cfg.margin, eta=1.0)
    o_plan = solve_min_time(track, veh, init, ds=cfg.ocp_ds, margin=cfg.margin, eta=cfg.eta_plan)
    timing["ocp_s"] = time.time() - t0

    closed = {}
    sims = {}
    for name, (path, v, a, kref) in {
        "mincurv_qss": plan_from_points(np.column_stack([mc.x, mc.y]), q_plan.v),
        "ocp": plan_from_points(o_plan.xy(track), o_plan.v, kappa=np.tan(o_plan.delta) / veh.L),
    }.items():
        t0 = time.time()
        r = run_closed_loop(track, veh, path, v, a, kref, cfg.dt)
        timing[f"sim_{name}_s"] = time.time() - t0
        sims[name] = (path, v, r)
        closed[name] = {"lap_time": r.lap_time, "completed": r.completed, "stop_reason": r.stop_reason,
                        "max_usage": r.max_usage, "min_edge_margin": r.min_margin, **r.info}

    res = L0Result(asdict(cfg), centre, q_ideal.lap_time, q_plan.lap_time,
                   o_ideal.lap_time, o_plan.lap_time, closed, timing)
    res.timing["total_s"] = time.time() - t_all
    for o in (o_ideal, o_plan):
        if not o.success:
            raise RuntimeError(f"OCP did not converge: {o.status}")
    if verbose:
        print(json.dumps(asdict(res), indent=2))
    if outdir is not None:
        out = FsPath(outdir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(asdict(res), indent=2))
        np.savez(out / "l0_data.npz", alpha=alpha, mc_x=mc.x, mc_y=mc.y, mc_v=q_plan.v,
                 ocp_s=o_ideal.s, ocp_n=o_ideal.n, ocp_v=o_ideal.v, ocp_delta=o_ideal.delta,
                 ocp_fx=o_ideal.fx, ocp_plan_n=o_plan.n, ocp_plan_v=o_plan.v)
    return res, dict(track=track, veh=veh, mc=mc, q_plan=q_plan, q_ideal=q_ideal,
                     o_ideal=o_ideal, o_plan=o_plan, sims=sims)
