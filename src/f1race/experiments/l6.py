"""L6 experiment: model mismatch with a structural error (steering actuator lag).

The cars of L4 (nominal car + the first 24 L2 cars, seed 0) get a first-order
steering lag (``sim/lagged.py``); the controllers see their own commanded angle in
the steering slot (``observe="command"``) unless stated otherwise. Controllers
(round 1, ``docs/L6_protocol.md``):

* ``tracker``        L1 tracker on the 5 % plan
* ``nmpc``           L2 NMPC, nominal model, 5 % plan, eta 0.95
* ``explore``        the same with a uniform 12 % reserve (L2 ``nmpc_track_robust``);
                     also the exploration lap of the identification
* ``smpc``           L3 stochastic MPC, eps = 0.01 (L3 calibration, unchanged)
* ``identified``     L4: parameters identified from the exploration lap as the
                     controller logged it, re-planned, NMPC with the identified model
* ``oracle_param``   true parameters (re-planned), no lag in the model
* ``lag_nominal``    nominal parameters, the correct lag in the model
* ``oracle_struct``  true parameters and the correct lag (reference: the correct structure;
                     not a strict upper bound, its plan has no lag)
* ``lag_identified`` the correct structure with tau identified together with the seven
                     parameter factors from the exploration lap (review MAJOR-1)
* ``lag_half``, ``lag_double``  nominal parameters, lag model with 0.5 tau / 2 tau
                     (sensitivity to a wrong time constant; nominal car)

The L5 policy is evaluated separately (``scripts/l6_rl.py``, needs the RL extras).
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, replace

import numpy as np

from ..control.dyn_tracking import DynTracker
from ..control.nmpc import FrenetMPC, LagAwareMPCController, MPCController
from ..dynamics import DynVehicle
from ..estimation.identify import data_from_run, identify, theta_of
from ..sim.dynamic import DynamicSim
from ..sim.lagged import LaggedSim
from ..track import Track
from .l1 import dyn_reference
from .l2 import ref_from_plan, x0_from_plan
from .l4 import replan

CONTROLLERS = ("tracker", "nmpc", "explore", "smpc", "identified", "oracle_param", "lag_nominal",
               "oracle_struct", "lag_identified")
NEEDS_EXPLORE = ("identified", "lag_identified")
HORIZON_STAGES = (5, 10, 20, 30)


@dataclass
class L6Config:
    track: str = "Spielberg"
    n_plants: int = 24                 # the L4 cars: nominal + the first 24 L2 cars (seed 0)
    seed: int = 0
    eta: float = 0.95
    explore_eta: float = 0.88
    dt: float = 0.01
    N: int = 30
    mpc_dt: float = 0.05
    t_max: float = 120.0
    workers: int = 8
    observe: str = "command"
    tracker: tuple = (("k_p", 0.06), ("x_la", 12.0), ("k_r", 0.05), ("kv", 8.0))


def config_dict(cfg: L6Config) -> dict:
    d = asdict(cfg)
    d["tracker"] = dict(cfg.tracker)
    return d


def make_sim(track: Track, plant: DynVehicle, tau: float, cfg: dict, observe: str | None = None):
    if tau == 0.0:
        return DynamicSim(track, plant, cfg["dt"])
    return LaggedSim(track, plant, tau, cfg["dt"], observe=observe or cfg["observe"])


def observed_run(r, sim):
    """The run as the controller logged it (7-dim states of ``sim.observe``)."""
    if not isinstance(sim, LaggedSim):
        return r
    return replace(r, state=np.array([sim.observe(x) for x in r.state]))


def prediction_errors(track: Track, ctl, r, stages=HORIZON_STAGES) -> dict:
    """Lateral-position error of the MPC predictions against the realised path, per
    horizon stage: mean |error| and 95th percentile over the solves of the lap."""
    t, X = r.t, r.state
    _, n_real = track.project(X[:, 0], X[:, 1])
    dt = ctl.mpc.dt
    out = {}
    for k in stages:
        e = []
        for (t0, Xp, _) in ctl.predictions:
            j = int(round((t0 + k * dt - t[0]) / (t[1] - t[0])))
            if j < len(t):
                e.append(abs(Xp[k, 1] - n_real[j]))
        out[str(k)] = ([float(np.mean(e)), float(np.percentile(e, 95))] if e else None)
    return out


def drive(track: Track, plant: DynVehicle, model: DynVehicle, plan, eta: float, tau: float, cfg: dict,
          lag_model: float | None = None, backoff=None, observe: str | None = None, tracker: bool = False):
    """One lap of the plant (with steering lag tau) under an NMPC (or the tracker) built
    on ``model``; ``lag_model``: steering lag in the controller's model."""
    path, ff = dyn_reference(track, model, plan)
    x0 = x0_from_plan(plan, path)
    sim = make_sim(track, plant, tau, cfg, observe)
    t0 = time.time()
    if tracker:
        ctl = DynTracker(path, ff, model, cfg["dt"], **cfg["tracker"])
    else:
        mpc = FrenetMPC(track, model, ref_from_plan(plan), mode="track", N=cfg["N"], dt=cfg["mpc_dt"],
                        eta=eta, backoff=backoff, steer_lag=lag_model)
        if lag_model:
            ctl = LagAwareMPCController(mpc, cfg["dt"], record=True,
                                        observe=sim.observe_mode if isinstance(sim, LaggedSim) else "command")
        else:
            ctl = MPCController(mpc, cfg["dt"], record=True)
    r = sim.run(ctl, x0, t_max=cfg["t_max"])
    out = {"lap_time": r.lap_time, "completed": r.completed, "stop_reason": r.stop_reason,
           "distance": float(r.progress[-1]), "max_usage": r.max_usage, "min_edge_margin": r.min_margin,
           "slide_fraction": r.slide_fraction,
           "max_rear_slip_deg": float(np.degrees(np.abs(r.slip[:, 1]).max())),
           "wall_s": time.time() - t0}
    if isinstance(sim, LaggedSim):
        out["max_steer_lag_rad"] = float(np.abs(r.state[:, 6] - r.state[:, 7]).max())
    if not tracker:
        st = ctl.mpc.stats
        out.update({"solve_ms_mean": float(np.mean([s["time"] for s in st]) * 1e3),
                    "fallbacks": int(sum(s["fallback"] for s in st)), "solves": len(st),
                    "prediction_error_n": prediction_errors(track, ctl, r)})
    return r, sim, out


def smpc_spec(cal: dict, eps: float = 0.01) -> dict:
    c = next(c for c in cal["smpc"] if c["eps"] == eps)
    return {"eta": c["eta"], "backoff": c["backoff"],
            "plan_eta": float(np.floor(min(c["eta"]) * 100.0) / 100.0)}


def run_job(args) -> dict:
    """Worker: one (car, tau, controller) lap; ``identified`` re-runs nothing but uses the
    exploration lap passed in ``extra``. CasADi objects are built locally."""
    job, cfg = args
    track = Track.load(cfg["track"])
    nom = DynVehicle()
    plant, tau, name, plans = job["plant"], job["tau"], job["controller"], job["plans"]
    rec = {"round": job.get("round", 1), "car": job["car"], "plant": job["info"], "tau": tau,
           "observe": job.get("observe", cfg["observe"]), "controller": name}
    obs = rec["observe"]
    eta, lag = cfg["eta"], (tau if tau > 0 else None)
    t0 = time.time()
    if name == "tracker":
        _, _, out = drive(track, plant, nom, plans[eta], eta, tau, cfg, observe=obs, tracker=True)
    elif name == "nmpc":
        _, _, out = drive(track, plant, nom, plans[eta], eta, tau, cfg, observe=obs)
    elif name == "explore":
        r, sim, out = drive(track, plant, nom, plans[cfg["explore_eta"]], cfg["explore_eta"], tau, cfg,
                            observe=obs)
        ro = observed_run(r, sim)
        if job.get("save_dir"):                     # the logged lap, for the residual model (round 2)
            from pathlib import Path

            d = Path(job["save_dir"])
            d.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(d / f"explore_{job['car']}_{tau}_{obs}.npz", t=ro.t, state=ro.state,
                                inputs=ro.inputs, usage=ro.usage, plant_state=r.state)
        ident = identify(data_from_run(ro, cfg["dt"]), nom)
        out["identification"] = {"theta": ident["theta"].tolist(), "std": ident["std"].tolist(),
                                 "rms": ident["rms"], "samples": ident["samples"],
                                 "max_usage_in_data": ident["max_usage_in_data"]}
    elif name == "smpc":
        sp = job["smpc"]
        _, _, out = drive(track, plant, nom, plans[sp["plan_eta"]], sp["eta"], tau, cfg,
                          backoff=sp["backoff"], observe=obs)
    elif name == "lag_nominal":
        _, _, out = drive(track, plant, nom, plans[eta], eta, tau, cfg, lag_model=lag, observe=obs)
    elif name in ("lag_half", "lag_double"):
        f = 0.5 if name == "lag_half" else 2.0
        _, _, out = drive(track, plant, nom, plans[eta], eta, tau, cfg, lag_model=f * tau, observe=obs)
        out["model_tau"] = f * tau
    elif name == "lag_identified":
        from types import SimpleNamespace

        from ..estimation.identify import identify_with_lag

        z = np.load(job["explore_file"])
        ident = identify_with_lag(SimpleNamespace(state=z["state"], inputs=z["inputs"], usage=z["usage"]),
                                  cfg["dt"], nom)
        veh, tau_hat = ident["vehicle"], ident["tau"]
        info = {"theta": ident["theta"].tolist(), "tau": tau_hat, "rms": ident["rms"],
                "rms_without_lag": ident["rms_without_lag"], "samples": ident["samples"],
                "on_bounds": ident["on_bounds"]}
        o, log = replan(track, veh, [("nominal plan", plans[eta]), ("ideal plan", plans[1.0])], eta)
        if o is None:
            out = {"completed": False, "stop_reason": "plan failed", "lap_time": float("nan"),
                   "distance": 0.0, "plan_log": log}
        else:
            _, _, out = drive(track, plant, veh, o, eta, tau, cfg, observe=obs,
                              lag_model=max(tau_hat, 1e-4))
            out.update({"plan_lap_time": o.lap_time, "plan_log": log})
        out["identification_lag"] = info
    elif name in ("oracle_param", "oracle_struct"):
        o = job["oracle_plan"]
        if o is None:
            out = {"completed": False, "stop_reason": "plan failed", "lap_time": float("nan"), "distance": 0.0}
        else:
            _, _, out = drive(track, plant, plant, o, eta, tau, cfg, observe=obs,
                              lag_model=lag if name == "oracle_struct" else None)
    elif name == "identified":
        from ..estimation.identify import vehicle_from
        veh = vehicle_from(job["theta"], nom)
        o, log = replan(track, veh, [("nominal plan", plans[eta]), ("ideal plan", plans[1.0])], eta)
        if o is None:
            out = {"completed": False, "stop_reason": "plan failed", "lap_time": float("nan"),
                   "distance": 0.0, "plan_log": log}
        else:
            _, _, out = drive(track, plant, veh, o, eta, tau, cfg, observe=obs)
            out.update({"plan_lap_time": o.lap_time, "plan_log": log})
    else:
        raise ValueError(name)
    rec.update(out)
    rec["job_s"] = time.time() - t0
    return rec


def oracle_plan_job(args):
    """Worker: the plan of the true car (no lag: the plans are minimum-time references)."""
    car, plant, plans, cfg = args
    track = Track.load(cfg["track"])
    o, log = replan(track, plant, [("nominal plan", plans[cfg["eta"]]), ("ideal plan", plans[1.0])], cfg["eta"])
    return car, o, log


def key_of(rec: dict) -> str:
    return f"{rec['car']}|{rec['tau']}|{rec['observe']}|{rec['controller']}"


def load_jsonl(path) -> list[dict]:
    try:
        return [json.loads(l) for l in open(path, encoding="utf-8") if l.strip()]
    except FileNotFoundError:
        return []


def summarize(results: list[dict]) -> dict:
    from scipy.stats import binomtest

    out = {}
    groups = sorted({(r["tau"], r["observe"], r["controller"]) for r in results})
    for tau, obs, c in groups:
        rs = [r for r in results if (r["tau"], r["observe"], r["controller"]) == (tau, obs, c)]
        done = [r for r in rs if r["completed"]]
        ci = binomtest(len(done), len(rs)).proportion_ci(method="exact")
        nominal = [r for r in rs if r["plant"].get("nominal")]
        out[f"{tau}|{obs}|{c}"] = {
            "tau": tau, "observe": obs, "controller": c, "runs": len(rs), "completed": len(done),
            "completed_ci95": [ci.low, ci.high],
            "lap_mean": float(np.mean([r["lap_time"] for r in done])) if done else None,
            "lap_nominal_car": nominal[0]["lap_time"] if nominal else None,
            "stop_reasons": {s: sum(r["stop_reason"] == s for r in rs) for s in sorted({r["stop_reason"] for r in rs})}}
    return out


def paired(results: list[dict], tau: float, a: str, b: str, observe: str = "command") -> dict:
    """Exact McNemar test on the same cars (one tau and observation condition)."""
    from scipy.stats import binomtest

    A = {r["car"]: r["completed"] for r in results if r["tau"] == tau and r["observe"] == observe
         and r["controller"] == a}
    B = {r["car"]: r["completed"] for r in results if r["tau"] == tau and r["observe"] == observe
         and r["controller"] == b}
    common = sorted(set(A) & set(B))
    only_a = sum(A[k] and not B[k] for k in common)
    only_b = sum(B[k] and not A[k] for k in common)
    n = only_a + only_b
    return {"tau": tau, "a": a, "b": b, "cars": len(common), "only_a": only_a, "only_b": only_b,
            "p": float(binomtest(min(only_a, only_b), n, 0.5).pvalue) if n else 1.0}


__all__ = ["L6Config", "CONTROLLERS", "run_job", "oracle_plan_job", "summarize", "paired", "theta_of"]
