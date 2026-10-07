"""L1 experiment: dynamic single-track model with tyre slip and load transfer.

1. L0 kinematic OCP (reference, ideal)
2. L1 dynamic OCP: ideal (eta = 1) and with the planning margin
3. closed loop of the L1 plan on the dynamic simulator (feed-forward + feedback)
4. the L0 plan driven on the dynamic car by the L0 controller (what the kinematic
   model misses)
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path as FsPath

import numpy as np

from ..control.dyn_tracking import DynTracker
from ..control.path_tracking import PathTracker
from ..dynamics import DynVehicle
from ..path import Path
from ..planning.mincurv import min_curvature_line
from ..planning.ocp_dynamic import initial_guess_from_kinematic, solve_min_time_dynamic
from ..planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from ..planning.qss import speed_profile
from ..sim.dynamic import DynamicSim, DynSimResult
from ..track import Track
from .l0 import plan_from_points


@dataclass
class L1Config:
    track: str = "Spielberg"
    margin: float = 0.3
    eta_plan: float = 0.97
    ds_l0: float = 4.0
    ds_l1: float = 2.5
    nsub: int = 2
    dt: float = 0.01


@dataclass
class L1Result:
    config: dict
    l0_ocp_ideal: float
    l1_ocp_ideal: float
    l1_ocp_plan: float
    closed_loop: dict = field(default_factory=dict)
    timing: dict = field(default_factory=dict)


def dyn_reference(track: Track, veh: DynVehicle, o, ds: float = 0.5):
    """Dense CG path through the OCP nodes and feed-forward arrays on it."""
    xy = o.xy(track)
    knots = Path.from_points(xy)
    dense = Path.from_points(xy, resample=ds)
    per = knots.length
    itp = lambda a: np.interp(dense.s, knots.s, a, period=per)
    ff = {"vx": itp(o.vx), "delta": itp(o.delta), "r": itp(o.r), "f": itp(o.f_drive + o.f_brake)}
    return dense, ff


def run_dyn_closed_loop(track, veh, o, dt) -> DynSimResult:
    path, ff = dyn_reference(track, veh, o)
    ctl = DynTracker(path, ff, veh, dt)
    sim = DynamicSim(track, veh, dt)
    i = 0
    x0 = np.array([path.x[0], path.y[0], path.psi[0] - np.arctan2(o.vy[i], o.vx[i]),
                   o.vx[i], o.vy[i], o.r[i], o.delta[i]])
    res = sim.run(ctl, x0, t_max=3.0 * track.length / max(o.vx.mean(), 1.0))
    log = np.array([l[1:] for l in ctl.log])
    res.info["max_abs_e"] = float(np.abs(log[:, 1]).max())
    res.info["max_dv"] = float(np.max(np.abs(res.state[: len(log), 3] - log[:, 3])))
    return res


class _KinOnDyn:
    """Drive the dynamic car with the kinematic (L0) controller: the rear-axle state
    of the dynamic car is handed to the L0 tracker."""

    def __init__(self, tracker: PathTracker, veh: DynVehicle):
        self.tr, self.veh = tracker, veh

    def __call__(self, t, x):
        px, py, psi, vx, vy, r, d = x
        xr = np.array([px - self.veh.lr * np.cos(psi), py - self.veh.lr * np.sin(psi), psi, vx, d])
        drate, fx = self.tr(t, xr)
        return drate, max(fx, 0.0), min(fx, 0.0)


def run_l0_plan_on_dyn(track, veh, o0, dt) -> DynSimResult:
    path, v, a, k = plan_from_points(o0.xy(track), o0.v, kappa=np.tan(o0.delta) / veh.L)
    ctl = _KinOnDyn(PathTracker(path, v, a, veh, dt, kappa_ref=k), veh)
    sim = DynamicSim(track, veh, dt)
    psi0 = path.psi[0]
    r0 = v[0] * np.tan(o0.delta[0]) / veh.L
    x0 = np.array([path.x[0] + veh.lr * np.cos(psi0), path.y[0] + veh.lr * np.sin(psi0), psi0,
                   v[0], veh.lr * r0, r0, o0.delta[0]])
    return sim.run(ctl, x0, t_max=3.0 * track.length / max(v.mean(), 1.0))


def summarize(r: DynSimResult) -> dict:
    return {"lap_time": r.lap_time, "completed": r.completed, "stop_reason": r.stop_reason,
            "t_end": float(r.t[-1]), "distance": float(r.progress[-1]),
            "max_usage": r.max_usage, "slide_fraction": r.slide_fraction,
            "min_edge_margin": r.min_margin, **r.info}


def run(cfg: L1Config = L1Config(), outdir=None, verbose: bool = True):
    t_all = time.time()
    track = Track.load(cfg.track)
    veh = DynVehicle()
    timing = {}

    alpha, mc = min_curvature_line(track, veh, margin=cfg.margin)
    q = speed_profile(mc, veh, 1.0)
    idx = np.arange(0, len(track.s), 3)
    init0 = initial_guess_from_path(track, veh, idx, alpha, mc, q.v, q.a)
    o0 = solve_min_time(track, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=1.0)
    o0p = solve_min_time(track, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=cfg.eta_plan)

    t0 = time.time()
    init1 = initial_guess_from_kinematic(track, veh, o0)
    o1 = solve_min_time_dynamic(track, veh, init1, ds=cfg.ds_l1, nsub=cfg.nsub, margin=cfg.margin, eta=1.0)
    o1p = solve_min_time_dynamic(track, veh, init1, ds=cfg.ds_l1, nsub=cfg.nsub, margin=cfg.margin,
                                 eta=cfg.eta_plan)
    timing["l1_ocp_s"] = time.time() - t0
    for o in (o0, o0p, o1, o1p):
        if not o.success:
            raise RuntimeError(f"OCP did not converge: {o.status}")

    t0 = time.time()
    cl_l1 = run_dyn_closed_loop(track, veh, o1p, cfg.dt)
    cl_l0 = run_l0_plan_on_dyn(track, veh, o0p, cfg.dt)
    timing["sim_s"] = time.time() - t0

    res = L1Result(asdict(cfg), o0.lap_time, o1.lap_time, o1p.lap_time,
                   {"l1_plan_on_dynamic_car": summarize(cl_l1),
                    "l0_plan_on_dynamic_car": summarize(cl_l0)}, timing)
    res.timing["total_s"] = time.time() - t_all
    if verbose:
        print(json.dumps(asdict(res), indent=2))
    if outdir is not None:
        out = FsPath(outdir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "summary.json").write_text(json.dumps(asdict(res), indent=2))
        np.savez(out / "l1_data.npz", **{f"o1_{k}": getattr(o1, k) for k in
                                          ["s", "n", "xi", "vx", "vy", "r", "delta", "f_drive", "f_brake"]},
                 o0_s=o0.s, o0_n=o0.n, o0_v=o0.v)
    return res, dict(track=track, veh=veh, o0=o0, o0p=o0p, o1=o1, o1p=o1p, cl_l1=cl_l1, cl_l0=cl_l0)
