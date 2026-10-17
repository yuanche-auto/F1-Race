"""L8: generalisation – new tracks and new conditions (docs/L8_protocol.md).

* ``VALIDATION``, ``TEST``, ``RESERVE``: the track draw of ``scripts/l8_tracks.py``.
* ``compute_plans(name)``: the plans every plan-based controller gets on a track, made by the
  L1 pipeline unchanged (minimum-curvature line, kinematic OCP, dynamic OCP with the
  continuation over the friction margin: eta 1.0, 0.97, 0.95, 0.93) plus the 12 % plan
  (eta 0.88, from the ideal plan, as ``scripts/run_l2.py``); cached in
  ``results/L8/plans/<name>.pkl`` with the code fingerprint of L1. ``plans_for(name)``: those
  plans, or Spielberg's L1 / L2 caches.
* ``cases(set)``: the laps of a set (protocol section 2) – ``A`` (test tracks: the 25 T1 cars and
  the L3 seeds 0-9), ``A0`` (Spielberg reference), ``B`` (wet Spielberg: the 25 cars at 0.85 and
  0.75, patch seeds 0-29), ``VA`` / ``VB`` (validation: Monza and Sakhir; wet Spielberg with the V
  cars), ``XA`` / ``XB`` (code checks: two laps each on a validation track / under wet).
* ``PatchField``: four wet patches (x 0.75, 100 m with 25 m ramps, centres >= 300 m apart).
* ``drive(case, kind)``: the model-based controllers of section 3 (``nmpc``, ``nmpc_reserve``,
  ``smpc``, ``identified``, ``lag_identified``) on one lap, with the failure handling of l8-M1.
* ``make_env(track, steer_mode)``: the RL loop of L7 (``l7.run_lap``) on a track; ``L8Env``
  carries the patches.
"""
from __future__ import annotations

import json
import pickle
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from ..dynamics import DynVehicle
from ..planning.mincurv import min_curvature_line
from ..planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
from ..planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from ..planning.qss import speed_profile
from ..track import Track
from .l1 import L1Config, code_fingerprint, dynamic_plans

ROOT = Path(__file__).resolve().parents[3]
VALIDATION = ("Monza", "Sakhir")
# the draw's test tracks were Zandvoort, SaoPaulo, Norisring, Nuerburgring; Norisring's plans failed (all four
# initial guesses of L1 at the 600-iteration cap, 2026-10-17) and it is replaced by the first reserve track
# (protocol section 2)
TEST = ("Zandvoort", "SaoPaulo", "Austin", "Nuerburgring")
FAILED = {"Norisring": "plans: no initial guess of the first dynamic solve converged (eta 0.97)"}
RESERVE = ("Silverstone", "Shanghai", "Montreal", "Catalunya", "MoscowRaceway", "Hockenheim",
           "YasMarina", "Sepang", "Spa", "Oschersleben", "BrandsHatch", "Sochi", "MexicoCity", "Melbourne",
           "Budapest", "Suzuka")
WET_LEVELS = (0.85, 0.75)
PATCH_FACTOR, PATCH_LEN, PATCH_RAMP, N_PATCHES, PATCH_GAP = 0.75, 100.0, 25.0, 4, 300.0
DT, MPC_DT, N = 0.01, 0.05, 30
EXPLORE_SEED_OFFSET = 100_000          # as L7 (A3.7): a separate draw of the disturbances
NOISE_SEED_OFFSET = 200_000
T_MAX_FACTOR = 1.6                     # lap time limit: 1.6 x the 5 % plan (Spielberg: 121 s; L7 used 120 s)
KINDS = ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified")


# ---------------------------------------------------------------- plans
def plans_path(name: str) -> Path:
    return ROOT / "results" / "L8" / "plans" / f"{name}.pkl"


def compute_plans(name: str, cfg: L1Config | None = None) -> dict:
    """The L1 plans and the 12 % plan of a track (see the module docstring); raises if a plan
    fails, with the solve log."""
    cfg = cfg or L1Config(track=name)
    t0 = time.time()
    track, veh = Track.load(name), DynVehicle()
    alpha, mc = min_curvature_line(track, veh, margin=cfg.margin)
    q = speed_profile(mc, veh, 1.0)
    idx = np.arange(0, len(track.s), 3)
    init0 = initial_guess_from_path(track, veh, idx, alpha, mc, q.v, q.a)
    o0 = solve_min_time(track, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=1.0)
    if not o0.success:
        raise RuntimeError(f"{name}: kinematic OCP failed: {o0.status}")
    log: list = []
    plans = dynamic_plans(track, veh, o0, cfg, log)
    o88 = solve_min_time_dynamic(track, veh, initial_guess_from_dynamic(plans[1.0]), eta=0.88)
    log.append({"step": "eta=0.88 from ideal (as run_l2)", "status": o88.status, "iterations": o88.iterations,
                "lap_time": o88.lap_time})
    if not o88.success:
        raise RuntimeError(f"{name}: eta = 0.88 plan failed: {log}")
    plans[0.88] = o88
    out = {"track": name, "o0": o0, "plans": plans, "log": log, "code": code_fingerprint(),
           "minutes": (time.time() - t0) / 60.0}
    p = plans_path(name)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "wb") as fh:
        pickle.dump(out, fh)
    return out


def load_plans(name: str) -> dict:
    with open(plans_path(name), "rb") as fh:
        d = pickle.load(fh)
    if d["code"] != code_fingerprint():
        raise RuntimeError(f"{name}: plans made with other planning code; recompute")
    return d


def plans_for(name: str) -> dict:
    """eta -> plan of a track: Spielberg from the L1 / L2 caches (as L7), the others from L8's."""
    if name == "Spielberg":
        with open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb") as fh:
            plans = dict(pickle.load(fh)["plans"])
        with open(ROOT / "results" / "L2" / "plan_088.pkl", "rb") as fh:
            plans[0.88] = pickle.load(fh)
        return plans
    return load_plans(name)["plans"]


def t_max_for(plans: dict) -> float:
    return T_MAX_FACTOR * float(plans[0.95].lap_time)


# ---------------------------------------------------------------- conditions
class PatchField:
    """Friction factor along the lap: 1 on the dry track, PATCH_FACTOR on four 100 m patches with
    25 m linear ramps; centres uniform over the lap with the patch seed, redrawn (same generator)
    until every two are at least PATCH_GAP apart (protocol section 2)."""

    def __init__(self, length: float, seed: int):
        rng = np.random.default_rng(seed)
        while True:
            c = np.sort(rng.uniform(0.0, length, N_PATCHES))
            if np.diff(np.r_[c, c[0] + length]).min() >= PATCH_GAP:
                break
        self.length, self.centres = float(length), c
        self._grid = np.linspace(0.0, length, int(length * 2) + 1)
        self._f = self._eval(self._grid)

    def _eval(self, s: np.ndarray) -> np.ndarray:
        L = self.length
        f = np.ones_like(s, dtype=float)
        for c in self.centres:
            d = np.abs((s - c + 0.5 * L) % L - 0.5 * L)
            w = np.clip((0.5 * PATCH_LEN + PATCH_RAMP - d) / PATCH_RAMP, 0.0, 1.0)
            f = np.minimum(f, 1.0 - (1.0 - PATCH_FACTOR) * w)
        return f

    def __call__(self, s):
        return np.interp(np.mod(s, self.length), self._grid, self._f)


def wet(veh: DynVehicle, level: float) -> DynVehicle:
    return replace(veh, mu_x=veh.mu_x * level, mu_y=veh.mu_y * level)


def _car_cases(set_: str, track: str, seed: int, n: int, with_nominal: bool, prefix: str,
               level: float | None = None) -> list[dict]:
    from .l2 import sample_plants

    cars = sample_plants(DynVehicle(), n, seed)
    idx = range(len(cars)) if with_nominal else range(1, len(cars))
    out = []
    for i in idx:
        info, veh = cars[i]
        out.append({"set": set_, "track": track, "car": i, "info": {**info, "wet": level},
                    "plant": wet(veh, level) if level is not None else veh, "wet": level, "patch_seed": None,
                    "dist_seed": None, "tau": 0.0, "name": f"{prefix}_car{i}"})
    return out


def _nominal_case(set_: str, track: str, name: str, dist_seed=None, patch_seed=None) -> dict:
    return {"set": set_, "track": track, "car": None, "info": {"nominal": True}, "plant": DynVehicle(),
            "wet": None, "patch_seed": patch_seed, "dist_seed": dist_seed, "tau": 0.0, "name": name}


def cases(name: str) -> list[dict]:
    """The laps of a set (protocol section 2)."""
    out: list[dict] = []
    if name in ("A", "VA", "XA"):
        tracks = TEST if name == "A" else VALIDATION
        for t in tracks:
            if name == "A":
                out += _car_cases(name, t, 0, 24, True, f"A_{t}")
                out += [_nominal_case(name, t, f"A_{t}_dist{d}", dist_seed=d) for d in range(10)]
            elif name == "VA":
                out += _car_cases(name, t, 1, 10, False, f"VA_{t}")
                out += [_nominal_case(name, t, f"VA_{t}_dist{d}", dist_seed=d) for d in range(2000, 2005)]
            else:
                out += _car_cases(name, t, 1, 10, False, f"XA_{t}")[:1]
                out += [_nominal_case(name, t, f"XA_{t}_dist2000", dist_seed=2000)]
    elif name == "A0":
        out += _car_cases(name, "Spielberg", 0, 24, True, "A0")
        out += [_nominal_case(name, "Spielberg", f"A0_dist{d}", dist_seed=d) for d in range(10)]
    elif name in ("B", "VB", "XB"):
        seed, n, nom = (0, 24, True) if name == "B" else (1, 10, False)
        for lv in WET_LEVELS:
            cc = _car_cases(name, "Spielberg", seed, n, nom, f"{name}_wet{int(round(lv * 100))}", level=lv)
            out += cc[:1] if name == "XB" else cc
        ps = range(30) if name == "B" else range(1000, 1010) if name == "VB" else range(1000, 1001)
        out += [_nominal_case(name, "Spielberg", f"{name}_patch{k}", patch_seed=k) for k in ps]
    else:
        raise KeyError(name)
    return out


def make_sim(case: dict, track: Track, t_max: float, seed_offset: int = 0):
    """The case's simulator: L3 disturbances (with measurement noise), the wet patches, or
    neither (a wet level is in the plant)."""
    from ..sim.dynamic import DynamicSim
    from ..sim.stochastic import Disturbances, StochasticSim, sample_disturbances

    if case["dist_seed"] is not None:
        return StochasticSim(track, case["plant"], sample_disturbances(track, case["dist_seed"] + seed_offset,
                                                                       t_max=max(150.0, t_max + 10.0)), DT)
    if case["patch_seed"] is not None:
        return StochasticSim(track, case["plant"], Disturbances(PatchField(track.length, case["patch_seed"]),
                                                                None, None, 0), DT)
    return DynamicSim(track, case["plant"], DT)


# ---------------------------------------------------------------- model-based controllers
def explore_lap(case: dict, track: Track, plans: dict, t_max: float):
    """The L6 / L7 exploration lap (nominal NMPC on the 12 % plan, eta 0.88) under the case's
    condition (a separate draw of the L3 disturbances; the same wet level or patches), as the
    controller logged it (with the L3 measurement noise on a disturbance case)."""
    from types import SimpleNamespace

    from ..control.nmpc import FrenetMPC, MPCController
    from ..sim.stochastic import NOISE_SIGMA
    from .l1 import dyn_reference
    from .l2 import ref_from_plan, x0_from_plan

    nom = DynVehicle()
    o = plans[0.88]
    path, _ = dyn_reference(track, nom, o)
    mpc = FrenetMPC(track, nom, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=0.88)
    ctl = MPCController(mpc, DT)
    sim = make_sim(case, track, t_max, EXPLORE_SEED_OFFSET)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=t_max)
    state = r.state.copy()
    noisy = case["dist_seed"] is not None
    if noisy:
        rng = np.random.default_rng(case["dist_seed"] + NOISE_SEED_OFFSET)
        state = state + NOISE_SIGMA * rng.standard_normal(state.shape)
    return SimpleNamespace(t=r.t, state=state, inputs=r.inputs, usage=r.usage, noisy=noisy,
                           completed=bool(r.completed), stop_reason=r.stop_reason, distance=float(r.progress[-1]))


def identify_run(ex, lag: bool) -> dict:
    """Parameters (``identified``) or tau and parameters (``lag_identified``) from an
    exploration lap: least squares without noise, the EKF under the L3 noise (as L4 / L6 / L7)."""
    from ..estimation.ekf import ekf_identify, ekf_identify_lag
    from ..estimation.identify import LO, HI, data_from_run, identify, identify_with_lag
    from ..sim.stochastic import NOISE_SIGMA

    nom = DynVehicle()
    if lag:
        r = (ekf_identify_lag(ex.state, ex.inputs, ex.usage, DT, nom, NOISE_SIGMA) if ex.noisy
             else identify_with_lag(ex, DT, nom))
    else:
        r = (ekf_identify(ex.state, ex.inputs, ex.usage, DT, nom, NOISE_SIGMA) if ex.noisy
             else identify(data_from_run(ex, DT), nom))
    theta = np.asarray(r["theta"], float)
    return {"vehicle": r["vehicle"], "theta": theta.tolist(), "tau": float(r.get("tau", 0.0)),
            "on_bounds": [bool(np.isclose(v, lo) or np.isclose(v, hi)) for v, lo, hi in zip(theta, LO, HI)],
            "samples": int(r.get("samples", 0))}


def drive(case: dict, kind: str) -> dict:
    """One lap of a model-based controller of section 3 (L6 failure handling). For the
    identification controllers (l8-M1): all logged exploration data are used; an error of the
    identification gives "identification failed", a failed ``replan_robust`` "plan failed" –
    both not completed, no retry."""
    from ..control.nmpc import FrenetMPC, LagAwareMPCController, MPCController
    from ..planning.replan import replan_robust
    from .l1 import dyn_reference
    from .l2 import ref_from_plan, x0_from_plan
    from .l6 import replan_slot, smpc_spec

    track = Track.load(case["track"])
    plans = plans_for(case["track"])
    t_max = t_max_for(plans)
    nom = DynVehicle()
    out: dict = {"t_max": t_max}
    lag = None
    if kind == "nmpc":
        o, model, eta, backoff = plans[0.95], nom, 0.95, None
    elif kind == "nmpc_reserve":
        o, model, eta, backoff = plans[0.88], nom, 0.88, None
    elif kind == "smpc":
        sp = smpc_spec(json.loads((ROOT / "results" / "L3" / "calibration.json").read_text()))
        o, model, eta, backoff = plans[sp["plan_eta"]], nom, sp["eta"], sp["backoff"]
    elif kind in ("identified", "lag_identified"):
        ex = explore_lap(case, track, plans, t_max)
        out["explore"] = {"completed": ex.completed, "distance": ex.distance, "stop_reason": ex.stop_reason}
        try:
            ident = identify_run(ex, lag=kind == "lag_identified")
        except Exception as e:                                       # noqa: BLE001 (l8-M1)
            out.update({"completed": False, "stop_reason": "identification failed", "lap_time": float("nan"),
                        "distance": 0.0, "identification_error": f"{type(e).__name__}: {e}"})
            return out
        out["identification"] = {k: v for k, v in ident.items() if k != "vehicle"}
        with replan_slot():
            o, log = replan_robust(track, ident["vehicle"], [("nominal plan", plans[0.95]), ("ideal plan", plans[1.0])],
                                   0.95, nominal=nom)
        out["plan_log"] = log
        if o is None:
            out.update({"completed": False, "stop_reason": "plan failed", "lap_time": float("nan"), "distance": 0.0})
            return out
        out["plan_lap_time"] = float(o.lap_time)
        model, eta, backoff = ident["vehicle"], 0.95, None
        if kind == "lag_identified":
            lag = max(ident["tau"], 1e-4)
    else:
        raise ValueError(kind)
    path, _ = dyn_reference(track, model, o)
    mpc = FrenetMPC(track, model, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=eta, backoff=backoff,
                    steer_lag=lag)
    ctl = LagAwareMPCController(mpc, DT, observe="command") if lag is not None else MPCController(mpc, DT)
    sim = make_sim(case, track, t_max)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=t_max)
    st = mpc.stats
    tm = np.array([x["time"] for x in st]) * 1e3 if st else np.array([np.nan])
    out.update({"completed": bool(r.completed), "lap_time": float(r.lap_time), "stop_reason": r.stop_reason,
                "distance": float(r.progress[-1]), "max_usage": float(np.max(r.usage)),
                "fallbacks": int(sum(x["fallback"] for x in st)), "solves": len(st),
                "solve_ms": [float(np.mean(tm)), float(np.percentile(tm, 95)), float(np.percentile(tm, 99)),
                             float(np.max(tm))]})
    return out


# ---------------------------------------------------------------- RL loop
def _l8env_class():
    from ..rl.env import RacingEnv

    class L8Env(RacingEnv):
        """``RacingEnv`` on any track; ``patch_seed`` (set before ``l7.run_lap``) puts the wet
        patches of section 2 under the car (no gusts, no noise)."""

        patch_seed = None

        def reset(self, *, seed=None, options=None):
            obs, info = super().reset(seed=seed, options=options)
            if self.patch_seed is not None:
                from ..sim.stochastic import Disturbances, StochasticSim

                plant = (options or {}).get("plant", self.nominal)
                self.sim = StochasticSim(self.track, plant,
                                         Disturbances(PatchField(self.track.length, self.patch_seed), None, None, 0),
                                         self.dt)
            return obs, info

    return L8Env


def make_env(track: str, steer_mode: str = "angle"):
    """As ``l7.make_env`` (history 8, commanded angle observed), on ``track`` with its 5 % plan."""
    return _l8env_class()(plans_for(track)[0.95], track=track, randomize_plant=False, history=8,
                          steer_mode=steer_mode, observe="command")


def rl_lap(policy, env, case: dict, t_max: float) -> dict:
    from .l7 import run_lap

    env.patch_seed = case["patch_seed"]
    r = run_lap(policy, env, case, None, t_max=t_max)
    return {"completed": r["completed"], "lap_time": r["lap_time"], "stop_reason": r["stop_reason"],
            "distance": r["distance"]}


__all__ = ["VALIDATION", "TEST", "RESERVE", "WET_LEVELS", "KINDS", "plans_path", "compute_plans", "load_plans",
           "plans_for", "t_max_for", "PatchField", "wet", "cases", "make_sim", "explore_lap", "identify_run", "drive",
           "make_env", "rl_lap"]
