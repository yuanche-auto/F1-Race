"""L4 experiment: learn the car, then drive it.

The first 24 perturbed cars of L2 (same seed) and the nominal car. L2 showed that a
controller built on the nominal model fails on about half of them, whatever
friction reserve it keeps. Here every car first drives an exploration lap with a
cautious controller (the L2 NMPC with a 12 % reserve), the parameters are
identified from the logged data (``estimation/identify.py``; a partial lap is
enough), the reference is re-planned with the identified model, and the NMPC
then drives with the identified model.

Per car:

* ``nominal``    L2 NMPC on the nominal 5 % plan with the nominal model (baseline)
* ``explore``    L2 NMPC on the nominal 12 % plan (the data-collection lap)
* ``identified`` NMPC with the identified model on a 5 % plan re-planned with it
* ``oracle``     the same with the true parameters (what perfect knowledge buys)
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, replace

import numpy as np

from ..control.nmpc import FrenetMPC, MPCController
from ..dynamics import DynVehicle
from ..estimation.identify import PARAMS, data_from_run, identify, theta_of, vehicle_from
from ..planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
from ..sim.dynamic import DynamicSim
from ..track import Track
from .l1 import dyn_reference
from .l2 import ref_from_plan, x0_from_plan


@dataclass
class L4Config:
    track: str = "Spielberg"
    n_plants: int = 24                 # the first 24 of the L2 cars (same seed) + nominal
    seed: int = 0                      # same plants as L2
    eta: float = 0.95                  # margin of the plan and the MPC after identification
    explore_eta: float = 0.88
    dt: float = 0.01
    N: int = 30
    mpc_dt: float = 0.05
    t_max: float = 120.0
    workers: int = 7


def nmpc_lap(track: Track, plant: DynVehicle, model: DynVehicle, plan, eta: float, cfg: dict):
    path, _ = dyn_reference(track, model, plan)
    mpc = FrenetMPC(track, model, ref_from_plan(plan), mode="track", N=cfg["N"], dt=cfg["mpc_dt"], eta=eta)
    ctl = MPCController(mpc, cfg["dt"])
    t0 = time.time()
    r = DynamicSim(track, plant, cfg["dt"]).run(ctl, x0_from_plan(plan, path), t_max=cfg["t_max"])
    return r, {"lap_time": r.lap_time, "completed": r.completed, "stop_reason": r.stop_reason,
               "distance": float(r.progress[-1]), "max_usage": r.max_usage,
               "min_edge_margin": r.min_margin, "wall_s": time.time() - t0,
               "solve_ms_mean": float(np.mean([l[3] for l in ctl.log]) * 1e3)}


def replan(track: Track, veh: DynVehicle, warm, eta: float):
    """Plan for a given model, warm-started from a nominal plan (one solve, then one
    retry from the other warm start if it fails)."""
    log = []
    for name, w in warm:
        o = solve_min_time_dynamic(track, veh, initial_guess_from_dynamic(w), eta=eta)
        log.append({"warm": name, "status": o.status, "iterations": o.iterations, "lap_time": o.lap_time})
        if o.success:
            return o, log
    return None, log


def run_plant(args) -> dict:
    """Worker: all laps of one car."""
    info, plant, plans, cfg = args
    track = Track.load(cfg["track"])
    nom = DynVehicle()
    out = {"plant": info, "true_theta": theta_of(info).tolist()}
    warm = [("nominal plan", plans[cfg["eta"]]), ("ideal plan", plans[1.0])]

    _, out["nominal"] = nmpc_lap(track, plant, nom, plans[cfg["eta"]], cfg["eta"], cfg)
    r_exp, out["explore"] = nmpc_lap(track, plant, nom, plans[cfg["explore_eta"]], cfg["explore_eta"], cfg)

    ident = identify(data_from_run(r_exp, cfg["dt"]), nom)
    out["identification"] = {"theta": ident["theta"].tolist(), "std": ident["std"].tolist(),
                             "rms": ident["rms"], "samples": ident["samples"],
                             "max_usage_in_data": ident["max_usage_in_data"]}
    for key, veh in (("identified", ident["vehicle"]), ("oracle", plant)):
        t0 = time.time()
        o, log = replan(track, veh, warm, cfg["eta"])
        if o is None:
            out[key] = {"completed": False, "stop_reason": "plan failed", "plan_log": log,
                        "lap_time": float("nan"), "distance": 0.0}
            continue
        _, res = nmpc_lap(track, plant, veh, o, cfg["eta"], cfg)
        res.update({"plan_lap_time": o.lap_time, "plan_log": log, "plan_s": time.time() - t0})
        out[key] = res
    return out


def run_plant_noisy(args) -> dict:
    """Worker: the same pipeline with measurement noise on the logged exploration data
    (the L3 noise levels; the controllers themselves see the true state, so that only
    the identification is affected). The exploration lap is re-simulated (it is
    deterministic), noise is added to the logged states, the parameters are identified
    by the same least squares, the plan is re-solved and the NMPC drives with it."""
    from ..sim.stochastic import NOISE_SIGMA

    info, plant, plans, cfg, seed = args
    track = Track.load(cfg["track"])
    nom = DynVehicle()
    r_exp, _ = nmpc_lap(track, plant, nom, plans[cfg["explore_eta"]], cfg["explore_eta"], cfg)
    rng = np.random.default_rng(seed)
    noisy = replace(r_exp, state=r_exp.state + NOISE_SIGMA * rng.standard_normal(r_exp.state.shape))
    ident = identify(data_from_run(noisy, cfg["dt"]), nom)
    out = {"plant": info, "true_theta": theta_of(info).tolist(),
           "identification": {"theta": ident["theta"].tolist(), "std": ident["std"].tolist(),
                              "rms": ident["rms"], "samples": ident["samples"]}}
    warm = [("nominal plan", plans[cfg["eta"]]), ("ideal plan", plans[1.0])]
    o, log = replan(track, ident["vehicle"], warm, cfg["eta"])
    if o is None:
        out["identified"] = {"completed": False, "stop_reason": "plan failed", "plan_log": log,
                             "lap_time": float("nan"), "distance": 0.0}
    else:
        _, res = nmpc_lap(track, plant, ident["vehicle"], o, cfg["eta"], cfg)
        res.update({"plan_lap_time": o.lap_time, "plan_log": log})
        out["identified"] = res
    return out


def summarize(results: list[dict]) -> dict:
    from scipy.stats import binomtest

    out = {}
    for key in ("nominal", "explore", "identified", "oracle"):
        rs = [r[key] for r in results]
        done = [r for r in rs if r["completed"]]
        ci = binomtest(len(done), len(rs)).proportion_ci(method="exact")
        out[key] = {"runs": len(rs), "completed": len(done), "completed_ci95": [ci.low, ci.high],
                    "lap_mean": float(np.mean([r["lap_time"] for r in done])) if done else None}
    err = np.array([np.subtract(r["identification"]["theta"], r["true_theta"]) for r in results])
    out["identification_max_abs_error"] = dict(zip(PARAMS, np.abs(err).max(axis=0).round(4).tolist()))
    both = [r for r in results if r["identified"]["completed"] and r["oracle"]["completed"]]
    if both:
        d = np.array([r["identified"]["lap_time"] - r["oracle"]["lap_time"] for r in both])
        out["identified_minus_oracle_lap"] = {"cars": len(both), "mean": float(d.mean()),
                                              "max_abs": float(np.abs(d).max())}
    return out


def paired(results: list[dict], a: str, b: str) -> dict:
    """Exact McNemar test on the same cars."""
    from scipy.stats import binomtest

    only_a = sum(r[a]["completed"] and not r[b]["completed"] for r in results)
    only_b = sum(r[b]["completed"] and not r[a]["completed"] for r in results)
    n = only_a + only_b
    return {"a": a, "b": b, "only_a": only_a, "only_b": only_b,
            "p": float(binomtest(min(only_a, only_b), n, 0.5).pvalue) if n else 1.0}


def config_dict(cfg: L4Config) -> dict:
    return asdict(cfg)


__all__ = ["L4Config", "run_plant", "summarize", "nmpc_lap", "replan", "vehicle_from"]
