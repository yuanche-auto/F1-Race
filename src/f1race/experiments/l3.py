"""L3 experiment: stochastic disturbances.

The car is the nominal one (no parameter mismatch, unlike L2), but on every lap
it meets a different realisation of three disturbances (``sim/stochastic.py``):
friction varying along the track, lateral gusts and noise on the measured state.
The controllers know the statistics, not the realisation. Question: how large a
friction / track-limit reserve does the MPC need, and does a reserve shaped by
the uncertainty (stochastic MPC with chance constraints) beat a uniform one?

Controllers (all on the nominal model):

* ``tracker``    L1 plan (5 %) + feed-forward/feedback tracker
* ``nmpc_<eta>`` L2 NMPC on the L1 plan with the same margin: a uniform reserve
                 (certainty equivalence for eta = 0.95)
* ``smpc_<eps>`` NMPC with per-stage tightening calibrated so that each predicted
                 constraint (friction ellipse of the plant, track limits) holds with
                 probability 1 - eps: the back-offs are empirical quantiles of the
                 prediction errors of the certainty-equivalent NMPC on separate
                 calibration laps (other random seeds).
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from multiprocessing import get_context

import numpy as np

from ..control.dyn_tracking import DynTracker
from ..control.nmpc import FrenetMPC, MPCController
from ..dynamics import DynVehicle
from ..sim.stochastic import StochasticSim, nominal_usage, sample_disturbances
from ..track import Track
from .l1 import dyn_reference
from .l2 import ref_from_plan, x0_from_plan


@dataclass
class L3Config:
    track: str = "Spielberg"
    sigma_mu: float = 0.03            # friction: standard deviation of the factor along the track
    corr_len: float = 100.0           # ... and its correlation length [m]
    gust_sigma: float = 300.0         # lateral gust force: standard deviation [N]
    gust_tau: float = 1.0             # ... and correlation time [s]
    noise: bool = True                # measurement noise (sim/stochastic.NOISE_SIGMA)
    calib_seeds: tuple = (1000, 1001, 1002, 1003, 1004, 1005)
    test_seeds: tuple = tuple(range(30))
    eps: tuple = (0.2, 0.05, 0.01)
    u_threshold: float = 0.85         # calibrate friction errors where the prediction is near the limit
    dt: float = 0.01
    N: int = 30
    mpc_dt: float = 0.05
    t_max: float = 120.0
    workers: int = 7
    tracker: dict = field(default_factory=lambda: {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0})


def disturbances(track: Track, seed: int, cfg: dict):
    return sample_disturbances(track, seed, cfg["sigma_mu"], cfg["corr_len"], cfg["gust_sigma"],
                               cfg["gust_tau"], cfg["noise"], cfg["dt"], cfg["t_max"])


def _calibration_samples(track: Track, veh: DynVehicle, ctl: MPCController, r, dist, cfg: dict) -> dict:
    """Prediction errors of every MPC solve of a run: for stage k of the prediction made
    at time t, compare with the simulated car at t + k * mpc_dt."""
    step = int(round(cfg["mpc_dt"] / cfg["dt"]))
    K = len(r.t)
    u_real = nominal_usage(veh, r.state, r.inputs)                       # (K, 2)
    s_real, _ = track.project(r.state[:, 0], r.state[:, 1])
    f_real = dist.friction(s_real) if dist.friction is not None else np.ones(K)
    c, s = np.cos(r.state[:, 2]), np.sin(r.state[:, 2])
    n_ax = []
    for arm in (veh.lf, -veh.lr):
        _, n = track.project(r.state[:, 0] + arm * c, r.state[:, 1] + arm * s)
        n_ax.append(n)
    n_ax = np.column_stack(n_ax)                                         # (K, 2) front, rear
    arms = np.array([veh.lf, -veh.lr])
    N = cfg["N"]
    cols = {"k": [], "u_pred": [], "u_real": [], "f": [], "e_n": []}
    for t, X, U in ctl.predictions:
        js = int(round(t / cfg["dt"])) + np.arange(N + 1) * step
        m = js < K
        na_pred = X[:, 1:2] + arms[None, :] * np.sin(X[:, 2:3])            # (N + 1, 2)
        e_n = np.abs(n_ax[js[m]] - na_pred[m]).max(axis=1)
        up = nominal_usage(veh, X[:N], U)                                  # (N, 2)
        ax = np.argmax(up, axis=1)                                         # axle closer to its limit
        mk = m[:N]
        cols["k"].append(np.arange(N + 1)[m])
        cols["e_n"].append(e_n)
        cols["f"].append(f_real[js[m]])
        cols["u_pred"].append(np.concatenate([up[mk, ax[mk]], np.full(m.sum() - mk.sum(), np.nan)]))
        ur = u_real[js[:N][mk], ax[mk]]
        cols["u_real"].append(np.concatenate([ur, np.full(m.sum() - mk.sum(), np.nan)]))
    return {k: np.concatenate(v) if v else np.zeros(0) for k, v in cols.items()}


def run_one(args) -> dict:
    """Worker: one (controller, seed) lap; CasADi objects are built locally."""
    name, spec, seed, plans, cfg, record = args
    track = Track.load(cfg["track"])
    model = DynVehicle()
    dt = cfg["dt"]
    o = plans[(spec["plan_eta"], spec.get("plan_margin", 0.3))]
    path, ff = dyn_reference(track, model, o)
    x0 = x0_from_plan(o, path)
    t0 = time.time()
    if spec["kind"] == "tracker":
        controller = DynTracker(path, ff, model, dt, **cfg["tracker"])
    else:
        mpc = FrenetMPC(track, model, ref_from_plan(o), mode="track", N=cfg["N"], dt=cfg["mpc_dt"],
                        eta=spec["eta"], margin=spec.get("mpc_margin", 0.3), backoff=spec.get("backoff"))
        controller = MPCController(mpc, dt, record=record)
    dist = disturbances(track, seed, cfg)
    r = StochasticSim(track, model, dist, dt).run(controller, x0, t_max=cfg["t_max"])
    u_plant = r.usage.max(axis=1)
    out = {"controller": name, "seed": seed, "lap_time": r.lap_time, "completed": r.completed,
           "stop_reason": r.stop_reason, "distance": float(r.progress[-1]),
           "max_usage": r.max_usage, "slide_fraction": r.slide_fraction,
           "usage_over_1_fraction": float(np.mean(u_plant > 1.0)),
           "min_edge_margin": r.min_margin,
           "max_rear_slip_deg": float(np.degrees(np.abs(r.slip[:, 1]).max())),
           "wall_s": time.time() - t0}
    if spec["kind"] != "tracker":
        st = [l[3] for l in controller.log]
        out["solve_ms_mean"] = float(np.mean(st) * 1e3)
        out["fallbacks"] = int(sum(1 for x in controller.mpc.stats if x["fallback"]))
    if record:
        out["samples"] = {k: v.tolist() for k, v in
                          _calibration_samples(track, model, controller, r, dist, cfg).items()}
    return out


def calibrate(samples: list[dict], cfg: L3Config, eps: float) -> dict:
    """Per-stage back-offs from pooled calibration samples.

    Friction: the plant's ellipse holds if the nominal usage of the car stays below the
    local friction factor f, i.e. u_pred + (u_real - u_pred) + (1 - f) <= 1. With
    d = u_real - u_pred + 1 - f, the stage-k friction factor of the SMPC is
    eta_k = 1 - q_k, q_k the (1 - eps) quantile of d over the samples whose prediction
    is near the limit (u_pred >= u_threshold).
    Track limits: back-off b_k = (1 - eps) quantile of the axle-point lateral error.
    Both are made non-decreasing along the horizon."""
    k = np.concatenate([np.asarray(s["k"]) for s in samples])
    up = np.concatenate([np.asarray(s["u_pred"]) for s in samples])
    ur = np.concatenate([np.asarray(s["u_real"]) for s in samples])
    f = np.concatenate([np.asarray(s["f"]) for s in samples])
    en = np.concatenate([np.asarray(s["e_n"]) for s in samples])
    N = cfg.N
    q_u, b_n, n_u = np.zeros(N), np.zeros(N + 1), np.zeros(N, int)
    for i in range(N):
        m = (k == i) & (up >= cfg.u_threshold)
        n_u[i] = m.sum()
        q_u[i] = np.quantile(ur[m] - up[m] + 1.0 - f[m], 1.0 - eps) if m.any() else np.nan
    for i in range(1, N + 1):
        b_n[i] = np.quantile(en[k == i], 1.0 - eps)
    q_u = np.maximum.accumulate(np.maximum(np.nan_to_num(q_u, nan=0.0), 0.0))
    b_n[1:] = np.maximum.accumulate(b_n[1:])
    return {"eps": eps, "eta": (1.0 - q_u).tolist(), "backoff": b_n.tolist(), "samples_per_stage": n_u.tolist()}


def run_jobs(jobs, workers: int, verbose: bool = True) -> list[dict]:
    out = []
    with get_context("spawn").Pool(workers) as pool:
        for res in pool.imap_unordered(run_one, jobs):
            out.append(res)
            if verbose:
                print(f"{res['controller']:12s} seed {res['seed']:5d} {res['stop_reason']:14s} "
                      f"lap {res['lap_time']:.3f} usage {res['max_usage']:.3f} "
                      f"over1 {res['usage_over_1_fraction']:.4f} margin {res['min_edge_margin']:.3f}",
                      flush=True)
    return out


def summarize(results: list[dict]) -> dict:
    out = {}
    for c in sorted({r["controller"] for r in results}):
        rs = [r for r in results if r["controller"] == c]
        done = [r for r in rs if r["completed"]]
        laps = np.array([r["lap_time"] for r in done])
        from scipy.stats import binomtest

        ci = binomtest(len(done), len(rs)).proportion_ci(method="exact")
        out[c] = {"runs": len(rs), "completed": len(done), "completed_ci95": [ci.low, ci.high],
                  "lap_mean": float(laps.mean()) if len(laps) else None,
                  "lap_std": float(laps.std()) if len(laps) else None,
                  "usage_over_1_fraction_mean": float(np.mean([r["usage_over_1_fraction"] for r in rs])),
                  "failures": sorted({f"{r['stop_reason']} @ {r['distance']:.0f} m" for r in rs
                                      if not r["completed"]}),
                  "solve_ms_mean": (float(np.mean([r["solve_ms_mean"] for r in rs]))
                                    if "solve_ms_mean" in rs[0] else None)}
    return out


def paired_test(results: list[dict], a: str, b: str) -> dict:
    """Exact McNemar test over the seeds both controllers were run on."""
    from scipy.stats import binomtest

    A = {r["seed"]: r["completed"] for r in results if r["controller"] == a}
    B = {r["seed"]: r["completed"] for r in results if r["controller"] == b}
    common = sorted(set(A) & set(B))
    only_a = sum(A[k] and not B[k] for k in common)
    only_b = sum(B[k] and not A[k] for k in common)
    n = only_a + only_b
    return {"a": a, "b": b, "seeds": len(common), "only_a": only_a, "only_b": only_b,
            "p": float(binomtest(min(only_a, only_b), n, 0.5).pvalue) if n else 1.0}


def config_dict(cfg: L3Config) -> dict:
    return json.loads(json.dumps(asdict(cfg)))
