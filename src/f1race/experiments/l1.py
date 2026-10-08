"""L1 experiment: dynamic single-track model with tyre slip and load transfer.

1. L0 kinematic OCP (reference, ideal)
2. L1 dynamic OCP with continuation (the problem has several local optima; warm
   starting between margins finds better ones): ideal (eta = 1) and plans with
   margins eta in {0.97, 0.95, 0.93}
3. closed loop of the L1 plan (eta_plan) on the dynamic simulator
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
from ..planning.ocp_dynamic import (initial_guess_from_dynamic, initial_guess_from_kinematic,
                                   initial_guess_quasi_steady, solve_min_time_dynamic)
from ..planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from ..planning.qss import speed_profile
from ..sim.dynamic import DynamicSim, DynSimResult
from ..track import Track
from .l0 import plan_from_points


@dataclass
class L1Config:
    track: str = "Spielberg"
    margin: float = 0.3
    eta_plan: float = 0.95
    eta_l0_plan: float = 0.97
    first_eta: float = 0.97
    etas: tuple = (0.97, 0.95, 0.93)
    ds_l0: float = 2.0
    ds_l1: float = 2.5
    nsub: int = 2
    dt: float = 0.01
    tracker: dict = field(default_factory=lambda: {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0})


@dataclass
class L1Result:
    config: dict
    l0_ocp_ideal: float
    l1_ocp_ideal: float
    l1_ocp_plan: float
    closed_loop: dict = field(default_factory=dict)
    plans: dict = field(default_factory=dict)
    solve_log: list = field(default_factory=list)
    timing: dict = field(default_factory=dict)


def code_fingerprint() -> str:
    """SHA-256 of the modules that determine the L1 plans (stored with the cached plans so
    that tests can tell whether the cache was produced by the current code)."""
    import hashlib

    root = FsPath(__file__).resolve().parents[1]
    files = ["track.py", "vehicle.py", "path.py", "dynamics.py", "planning/mincurv.py",
             "planning/qss.py", "planning/ocp_kinematic.py", "planning/ocp_dynamic.py",
             "experiments/l1.py"]
    h = hashlib.sha256()
    for f in files:
        h.update((root / f).read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()


def dyn_reference(track: Track, veh: DynVehicle, o, ds: float = 0.5):
    """Dense CG path through the OCP nodes and feed-forward arrays on it."""
    xy = o.xy(track)
    knots = Path.from_points(xy)
    dense = Path.from_points(xy, resample=ds)
    per = knots.length
    itp = lambda a: np.interp(dense.s, knots.s, a, period=per)
    ff = {"vx": itp(o.vx), "delta": itp(o.delta), "r": itp(o.r), "f": itp(o.f_drive + o.f_brake)}
    return dense, ff


def run_dyn_closed_loop(track, veh, o, dt, tracker: dict | None = None) -> DynSimResult:
    path, ff = dyn_reference(track, veh, o)
    ctl = DynTracker(path, ff, veh, dt, **(tracker or {}))
    sim = DynamicSim(track, veh, dt)
    i = 0
    x0 = np.array([path.x[0], path.y[0], path.psi[0] - np.arctan2(o.vy[i], o.vx[i]),
                   o.vx[i], o.vy[i], o.r[i], o.delta[i]])
    res = sim.run(ctl, x0, t_max=3.0 * track.length / max(o.vx.mean(), 1.0))
    log = np.array([l[1:] for l in ctl.log])
    res.info["max_abs_e"] = float(np.abs(log[:, 1]).max())
    res.info["max_dv"] = float(np.max(np.abs(res.state[: len(log), 3] - log[:, 3])))
    res.info["max_rear_slip_deg"] = float(np.degrees(np.abs(res.slip[:, 1]).max()))
    return res


def first_dynamic_plan(track, veh, o0, eta: float, log: list, **kw):
    """First dynamic solve, from the kinematic L0 solution, with fall-backs.

    Whether IPOPT converges from a guess built from the L0 solution is decided by
    rounding: the same guess perturbed by 1e-10 (relative) converged in 6 of 7 trials
    from the kinematic guess and in 4 of 7 from the quasi-steady one, and the two
    numpy/scipy builds tried disagree on the plain kinematic guess. Every converged
    start led to the same final plans (the continuation in ``dynamic_plans`` removes
    the differences between local optima). Four different guesses are therefore tried
    in turn, each capped at 600 iterations (successful solves take 100-330); in the
    seven perturbed trials this sequence always converged, once only at the third
    guess."""
    attempts = [("kinematic", lambda: initial_guess_from_kinematic(track, veh, o0)),
                ("quasi-steady at 97 % speed", lambda: initial_guess_quasi_steady(track, veh, o0, 0.97)),
                ("quasi-steady at 93 % speed", lambda: initial_guess_quasi_steady(track, veh, o0, 0.93)),
                ("kinematic at 93 % speed", lambda: initial_guess_from_kinematic(track, veh, o0, 0.93))]
    for name, make in attempts:
        o = solve_min_time_dynamic(track, veh, make(), eta=eta, max_iter=600, **kw)
        log.append({"step": f"first eta={eta}", "init": name, "status": o.status,
                    "iterations": o.iterations, "lap_time": o.lap_time})
        if o.success:
            return o
    raise RuntimeError(f"dynamic OCP: no initial guess converged for eta = {eta}: {log}")


def dynamic_plans(track, veh, o0, cfg, log: list | None = None) -> dict:
    """Continuation over the friction margin. The dynamic OCP has several local optima
    and the kinematic solution is a poor initial guess for eta = 1 (74.6 s instead of
    73.9 s on Spielberg): solve eta = first_eta (with fall-backs), the ideal from that,
    every margin from the ideal, and finally the ideal again from the executed plan;
    keep the best solution found for each eta. Every solve is logged."""
    kw = dict(ds=cfg.ds_l1, nsub=cfg.nsub, margin=cfg.margin)
    log = [] if log is None else log
    best = {}

    def keep(eta, init, step):
        o = solve_min_time_dynamic(track, veh, init, eta=eta, **kw)
        log.append({"step": step, "status": o.status, "iterations": o.iterations, "lap_time": o.lap_time})
        if o.success and (eta not in best or o.lap_time < best[eta].lap_time):
            best[eta] = o

    best[cfg.first_eta] = first_dynamic_plan(track, veh, o0, cfg.first_eta, log, **kw)
    keep(1.0, initial_guess_from_dynamic(best[cfg.first_eta]), f"ideal from eta={cfg.first_eta}")
    if 1.0 not in best:
        raise RuntimeError(f"dynamic OCP: ideal problem failed: {log}")
    for eta in cfg.etas:
        keep(eta, initial_guess_from_dynamic(best[1.0]), f"eta={eta} from ideal")
    for eta in cfg.etas:                                 # second chance from the neighbour
        if eta not in best:
            src = min(best, key=lambda e: abs(e - eta))
            keep(eta, initial_guess_from_dynamic(best[src]), f"eta={eta} from eta={src}")
    keep(1.0, initial_guess_from_dynamic(best[cfg.eta_plan]), f"ideal from eta={cfg.eta_plan}")
    missing = [e for e in (1.0, *cfg.etas) if e not in best]
    if missing:
        raise RuntimeError(f"dynamic OCP failed for eta = {missing}: {log}")
    return best


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
    # the plan L0 executes (its own margin, 0.97) is the one driven on the dynamic car
    o0p = solve_min_time(track, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=cfg.eta_l0_plan)

    t0 = time.time()
    solve_log: list = []
    plans = dynamic_plans(track, veh, o0, cfg, solve_log)
    timing["l1_ocp_s"] = time.time() - t0
    o1, o1p = plans[1.0], plans[cfg.eta_plan]
    for o in (o0, o0p):
        if not o.success:
            raise RuntimeError(f"OCP did not converge: {o.status}")

    t0 = time.time()
    cl_l1 = run_dyn_closed_loop(track, veh, o1p, cfg.dt, cfg.tracker)
    cl_l0 = run_l0_plan_on_dyn(track, veh, o0p, cfg.dt)
    timing["sim_s"] = time.time() - t0

    res = L1Result(asdict(cfg), o0.lap_time, o1.lap_time, o1p.lap_time,
                   {"l1_plan_on_dynamic_car": summarize(cl_l1),
                    "l0_plan_on_dynamic_car": summarize(cl_l0)},
                   plans={str(k): v.lap_time for k, v in sorted(plans.items())},
                   solve_log=solve_log, timing=timing)
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
    if outdir is not None:
        import pickle
        with open(FsPath(outdir) / "l1_plans.pkl", "wb") as fh:
            pickle.dump({"o0": o0, "o0p": o0p, "plans": plans, "code": code_fingerprint(),
                         "config": asdict(cfg)}, fh)
    return res, dict(track=track, veh=veh, o0=o0, o0p=o0p, o1=o1, o1p=o1p, plans=plans,
                     cl_l1=cl_l1, cl_l0=cl_l0)
