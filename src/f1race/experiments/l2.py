"""L2 experiment: closed loop with model mismatch.

Every controller uses the nominal vehicle model; the simulated car (the
"plant") has perturbed parameters. Monte Carlo over plant samples drawn with a
fixed seed. Controllers:

* ``plan_track``      offline L1 plan (eta_plan) + feed-forward/feedback tracker
* ``plan_track_safe`` the same with a larger planning margin (constraint tightening)
* ``nmpc_track``      NMPC following the L1 plan (re-plans every 50 ms, eta 0.95)
* ``nmpc_track_safe`` the same on the 7 % plan with eta 0.93 in the MPC (tightening)
* ``nmpc_track_robust`` the same on a 12 % plan with eta 0.88 (strong tightening)
* ``rh_mintime``      receding-horizon minimum-time MPC over the next 200 m of track
                      (no reference line). Experimental: not part of the default
                      Monte Carlo, see docs/L2.md for why.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field, replace
from multiprocessing import get_context
from pathlib import Path as FsPath

import numpy as np

from ..control.dyn_tracking import DynTracker
from ..control.nmpc import FrenetMPC, MPCController
from ..control.rh_mpc import SpatialMinTimeMPC, SpatialMPCController
from ..dynamics import DynVehicle
from ..sim.dynamic import DynamicSim
from ..track import Track
from .l1 import dyn_reference

PERTURBATION = {            # multiplicative ranges (uniform), "aero_front" additive
    "mu": (0.92, 1.03),
    "m": (1.00, 1.05),
    "ClA": (0.90, 1.10),
    "CdA": (0.90, 1.10),
    "Iz": (0.85, 1.15),
    "B": (0.85, 1.15),
    "aero_front": (-0.03, 0.03),
}


def sample_plants(nominal: DynVehicle, n: int, seed: int = 0) -> list[tuple[dict, DynVehicle]]:
    rng = np.random.default_rng(seed)
    out = [({"nominal": True}, nominal)]
    for _ in range(n):
        f = {k: float(rng.uniform(*r)) for k, r in PERTURBATION.items()}
        v = replace(nominal, mu_x=nominal.mu_x * f["mu"], mu_y=nominal.mu_y * f["mu"],
                    m=nominal.m * f["m"], ClA=nominal.ClA * f["ClA"], CdA=nominal.CdA * f["CdA"],
                    Iz=nominal.Iz * f["Iz"], B=nominal.B * f["B"],
                    aero_front=nominal.aero_front + f["aero_front"])
        out.append((f, v))
    return out


def x0_from_plan(o, path) -> np.ndarray:
    return np.array([path.x[0], path.y[0], path.psi[0] - np.arctan2(o.vy[0], o.vx[0]),
                     o.vx[0], o.vy[0], o.r[0], o.delta[0]])


def ref_from_plan(o) -> dict:
    return {"s": o.s, "n": o.n, "xi": o.xi, "vx": o.vx}


def run_one(args) -> dict:
    """Worker: one (controller, plant) lap. Builds everything locally (CasADi objects
    are not picklable)."""
    ctrl, plant_info, plant, plans, cfg = args
    track = Track.load(cfg["track"])
    model = DynVehicle()
    dt = cfg["dt"]
    o_plan, o_safe = plans["plan"], plans["safe"]
    t0 = time.time()
    path, ff = dyn_reference(track, model, o_plan)
    x0 = x0_from_plan(o_plan, path)
    if ctrl == "plan_track":
        controller = DynTracker(path, ff, model, dt, **cfg["tracker"])
    elif ctrl == "plan_track_safe":
        path_s, ff_s = dyn_reference(track, model, o_safe)
        controller = DynTracker(path_s, ff_s, model, dt, **cfg["tracker"])
        x0 = x0_from_plan(o_safe, path_s)
    elif ctrl == "nmpc_track":
        mpc = FrenetMPC(track, model, ref_from_plan(o_plan), mode="track", N=cfg["N"],
                        dt=cfg["mpc_dt"], eta=cfg["mpc_eta"])
        controller = MPCController(mpc, dt)
    elif ctrl == "nmpc_track_robust":
        o_rob = plans["robust"]
        path_r, _ = dyn_reference(track, model, o_rob)
        x0 = x0_from_plan(o_rob, path_r)
        mpc = FrenetMPC(track, model, ref_from_plan(o_rob), mode="track", N=cfg["N"],
                        dt=cfg["mpc_dt"], eta=cfg["mpc_eta_robust"])
        controller = MPCController(mpc, dt)
    elif ctrl == "nmpc_track_safe":
        path_s, _ = dyn_reference(track, model, o_safe)
        x0 = x0_from_plan(o_safe, path_s)
        mpc = FrenetMPC(track, model, ref_from_plan(o_safe), mode="track", N=cfg["N"],
                        dt=cfg["mpc_dt"], eta=cfg["mpc_eta_safe"])
        controller = MPCController(mpc, dt)
    elif ctrl == "rh_mintime":
        mpc = SpatialMinTimeMPC(track, model, {"s": o_plan.s, "vx": o_plan.vx}, eta=cfg["mpc_eta"])
        controller = SpatialMPCController(mpc, dt, period=cfg["mpc_dt"])
    else:
        raise ValueError(ctrl)
    sim = DynamicSim(track, plant, dt)
    r = sim.run(controller, x0, t_max=cfg["t_max"])
    is_mpc = isinstance(controller, (MPCController, SpatialMPCController))
    solve_times = [l[3] for l in controller.log] if is_mpc else []
    return {"controller": ctrl, "plant": plant_info, "lap_time": r.lap_time, "completed": r.completed,
            "stop_reason": r.stop_reason, "distance": float(r.progress[-1]),
            "max_usage": r.max_usage, "slide_fraction": r.slide_fraction,
            "min_edge_margin": r.min_margin,
            "max_rear_slip_deg": float(np.degrees(np.abs(r.slip[:, 1]).max())),
            "solve_ms_mean": float(np.mean(solve_times) * 1e3) if solve_times else None,
            "solve_ms_max": float(np.max(solve_times) * 1e3) if solve_times else None,
            "wall_s": time.time() - t0}


@dataclass
class L2Config:
    track: str = "Spielberg"
    n_plants: int = 12
    seed: int = 0
    dt: float = 0.01
    N: int = 30
    mpc_dt: float = 0.05
    mpc_eta: float = 0.95
    mpc_eta_safe: float = 0.93
    mpc_eta_robust: float = 0.88
    t_max: float = 120.0
    workers: int = 8
    tracker: dict = field(default_factory=lambda: {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0})


def run(plans: dict, cfg: L2Config = L2Config(), controllers=("plan_track", "plan_track_safe",
                                                              "nmpc_track", "nmpc_track_safe"),
        outdir=None, verbose: bool = True) -> list[dict]:
    """``plans``: dict with DynOCPResult entries 'plan' (eta = mpc_eta, e.g. 0.95) and
    'safe' (larger margin, e.g. 0.93), both on the nominal model."""
    plants = sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)
    jobs = [(c, info, v, plans, asdict(cfg)) for c in controllers for info, v in plants]
    with get_context("spawn").Pool(cfg.workers) as pool:
        results = []
        for res in pool.imap_unordered(run_one, jobs):
            results.append(res)
            if verbose:
                print(f"{res['controller']:16s} {res['stop_reason']:14s} lap {res['lap_time']:.3f} "
                      f"usage {res['max_usage']:.3f} margin {res['min_edge_margin']:.3f} "
                      f"plant {res['plant']}", flush=True)
    if outdir is not None:
        out = FsPath(outdir)
        out.mkdir(parents=True, exist_ok=True)
        (out / "runs.json").write_text(json.dumps(results, indent=1))
    return results


def summarize(results: list[dict]) -> dict:
    out = {}
    for c in sorted({r["controller"] for r in results}):
        rs = [r for r in results if r["controller"] == c]
        done = [r for r in rs if r["completed"]]
        laps = np.array([r["lap_time"] for r in done])
        nominal = [r for r in rs if r["plant"].get("nominal")]
        out[c] = {"runs": len(rs), "completed": len(done),
                  "lap_mean": float(laps.mean()) if len(laps) else None,
                  "lap_max": float(laps.max()) if len(laps) else None,
                  "lap_nominal_plant": nominal[0]["lap_time"] if nominal else None,
                  "failures": sorted({r["stop_reason"] for r in rs if not r["completed"]}),
                  "max_usage": float(max(r["max_usage"] for r in rs)),
                  "min_edge_margin": float(min(r["min_edge_margin"] for r in rs)),
                  "solve_ms_mean": (float(np.mean([r["solve_ms_mean"] for r in rs]))
                                    if rs[0]["solve_ms_mean"] is not None else None)}
    return out
