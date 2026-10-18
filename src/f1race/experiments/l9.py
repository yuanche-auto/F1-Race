"""L9: real-time control under an equal compute budget (docs/L9_protocol.md).

* ``cases(set)``: L7's sets – ``T1new`` / ``T2new`` (test), ``V`` (validation), ``X`` (smoke).
* ``make_mpc(impl, ...)``: the L2 NMPC as ``FrenetMPC`` (``impl="ipopt"``) or ``AcadosFrenetMPC``
  (``impl="rti"``, SQP-RTI; ``impl="sqp"``, SQP to convergence – equivalence check only).
* ``RealTimeMPCController`` / ``RealTimeLagMPCController``: the controllers of L2–L8 under the rule
  of section 3. ``mode="offline"``: as in L2–L8 (the command takes effect at the control instant).
  ``mode="realtime"``: each call is timed; its command takes effect at t_k + latency rounded up to
  the simulator step, the previous command is held until then; a computation longer than the
  period still completes, and the next one starts at the first control instant after the
  controller is free. Latency = the whole call for IPOPT; for RTI the feedback phase, and the
  controller is busy for feedback + preparation. ``delay``: sensor delay (the measurement of
  t - delay is used).
* ``drive(case, kind, impl, mode, ...)``: one lap of a model-based controller of section 4, with
  its exploration lap (``nmpc_reserve``, same implementation and mode unless ``explore_mode`` is
  given – the secondary condition of l9-m2) and the l8-M1 failure handling.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ..control.nmpc import FrenetMPC, LagAwareMPCController, MPCController
from ..dynamics import DynVehicle
from ..track import Track

ROOT = Path(__file__).resolve().parents[3]
DT, PERIOD, N = 0.01, 0.05, 30
T_MAX = 121.0                                   # 1.6 x the 5 % plan on Spielberg (as L8)
KINDS = ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified")
IMPLS = ("ipopt", "rti")
MODES = ("offline", "realtime")
EXPLORE_SEED_OFFSET, NOISE_SEED_OFFSET = 100_000, 200_000      # as L7 / L8


def cases(name: str) -> list[dict]:
    from .l7 import cases as l7_cases

    return l7_cases(name)


def plans() -> dict:
    from .l8 import plans_for

    return plans_for("Spielberg")


def make_mpc(impl: str, track: Track, model: DynVehicle, plan, eta, backoff=None, lag=None, name: str = "l9"):
    from .l2 import ref_from_plan

    ref = ref_from_plan(plan)
    if impl == "ipopt":
        return FrenetMPC(track, model, ref, mode="track", N=N, dt=PERIOD, eta=eta, backoff=backoff, steer_lag=lag)
    from ..control.acados_mpc import AcadosFrenetMPC

    return AcadosFrenetMPC(track, model, ref, N=N, dt=PERIOD, eta=eta, backoff=backoff, steer_lag=lag,
                           nlp="SQP_RTI" if impl == "rti" else "SQP", name=name)


class _RealTime:
    """The real-time rule around ``mpc.solve`` (mixin of the two controllers below)."""

    def _rt_init(self, mode: str, delay: float):
        if mode not in MODES:
            raise ValueError(mode)
        self.rt_mode, self.delay = mode, float(delay)
        self.pending = None                      # (t_apply, command)
        self.busy_until = 0.0
        self.calls: list[dict] = []
        self._buf: deque = deque(maxlen=int(round(self.delay / self.sim_dt)) + 1)

    def _measure(self, x):
        """Sensor delay: the state of t - delay (the oldest in the buffer until it is full)."""
        self._buf.append(np.array(x, float))
        return self._buf[0]

    def _schedule(self, t: float, solve) -> None:
        """Apply a pending command whose time has come; start a computation at a control instant
        when the controller is free."""
        if self.pending is not None and t + 1e-9 >= self.pending[0]:
            self.u = self.pending[1]
            self.pending = None
        if t + 1e-9 < self.t_next or (self.rt_mode == "realtime" and t + 1e-9 < self.busy_until):
            return
        t0 = time.perf_counter()
        u, info = solve()
        wall = time.perf_counter() - t0
        if "time_feedback" in info:              # RTI: latency = feedback, busy = feedback + preparation
            lat, busy = info["time_feedback"], info["time_feedback"] + info["time_preparation"]
        else:
            lat, busy = wall, wall
        rec = {"t": t, "latency": lat, "busy": busy, "wall": wall, "status": info["status"],
               "fallback": bool(info.get("fallback", False))}
        if self.rt_mode == "offline":
            self.u = u
            self.t_next = t + PERIOD
        else:
            t_apply = t + math.ceil(lat / self.sim_dt - 1e-9) * self.sim_dt
            self.pending = (t_apply, u)
            self.busy_until = t + busy
            k = math.floor((t + busy) / PERIOD - 1e-9) + 1     # the first control instant after it
            self.t_next = max(t + PERIOD, k * PERIOD)
            rec.update({"miss": busy > PERIOD, "t_apply": t_apply})
        self.calls.append(rec)


class RealTimeMPCController(_RealTime, MPCController):
    def __init__(self, mpc, sim_dt: float = DT, mode: str = "offline", delay: float = 0.0, tc_eta=0.99):
        MPCController.__init__(self, mpc, sim_dt, tc_eta)
        self._rt_init(mode, delay)

    def __call__(self, t: float, x: np.ndarray):
        from ..control.dyn_tracking import traction_limit

        xm = self._measure(x)
        xf = self.frenet(xm)
        self._schedule(t, lambda: self.mpc.solve(xf, self.u))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:              # traction control on the true state, every step
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


class RealTimeLagMPCController(_RealTime, LagAwareMPCController):
    """``LagAwareMPCController`` (observe="command") under the real-time rule."""

    def __init__(self, mpc, sim_dt: float = DT, mode: str = "offline", delay: float = 0.0, tc_eta=0.99):
        LagAwareMPCController.__init__(self, mpc, sim_dt, tc_eta, observe="command")
        self._rt_init(mode, delay)

    def __call__(self, t: float, x: np.ndarray):
        from ..control.dyn_tracking import traction_limit

        xm = self._measure(x)
        d_act, d_cmd = self.observe(t, float(xm[6])), float(xm[6])
        xa = np.array(xm, float)
        xa[6] = d_act
        xf = np.concatenate([self.frenet(xa), [d_cmd]])
        self._schedule(t, lambda: self.mpc.solve(xf, self.u))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = d_act + (d_cmd - d_act) * (1.0 - np.exp(-self.sim_dt / self.tau))
            fd, fb = traction_limit(veh, xa, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


def make_sim(case: dict, track: Track, seed_offset: int = 0):
    from .l7 import make_sim_for

    return make_sim_for(case, track, seed_offset)


def explore_lap(case: dict, track: Track, impl: str, mode: str, name: str):
    """The exploration lap (``nmpc_reserve``: nominal model, 12 % plan, eta 0.88) in the given
    implementation and mode; logged as the controller saw it (commanded angle in the steering slot
    with a plant lag; the L3 measurement noise on a disturbance case) – as L7 / L8."""
    from ..sim.lagged import LaggedSim
    from ..sim.stochastic import NOISE_SIGMA
    from .l1 import dyn_reference
    from .l2 import x0_from_plan

    nom = DynVehicle()
    o = plans()[0.88]
    path, _ = dyn_reference(track, nom, o)
    mpc = make_mpc(impl, track, nom, o, 0.88, name=name + "_explore")
    ctl = RealTimeMPCController(mpc, DT, mode)
    sim = make_sim(case, track, EXPLORE_SEED_OFFSET)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=T_MAX)
    state = np.array([sim.observe(x) for x in r.state]) if isinstance(sim, LaggedSim) else r.state.copy()
    noisy = case["dist_seed"] is not None
    if noisy:
        rng = np.random.default_rng(case["dist_seed"] + NOISE_SEED_OFFSET)
        state = state + NOISE_SIGMA * rng.standard_normal(state.shape)
    return SimpleNamespace(t=r.t, state=state, inputs=r.inputs, usage=r.usage, noisy=noisy,
                           completed=bool(r.completed), stop_reason=r.stop_reason, distance=float(r.progress[-1]),
                           calls=ctl.calls)


def timing_summary(calls: list[dict]) -> dict:
    if not calls:
        return {}
    lat = np.array([c["latency"] for c in calls]) * 1e3
    busy = np.array([c["busy"] for c in calls]) * 1e3
    out = {"calls": len(calls), "latency_ms": [float(lat.mean()), float(np.percentile(lat, 95)),
                                               float(np.percentile(lat, 99)), float(lat.max())],
           "busy_ms": [float(busy.mean()), float(np.percentile(busy, 95)), float(np.percentile(busy, 99)),
                       float(busy.max())],
           "fallbacks": int(sum(c["fallback"] for c in calls))}
    if "miss" in calls[0]:
        out["miss_rate"] = float(np.mean([c["miss"] for c in calls]))
    return out


def explore_summary(ex) -> dict:
    return {"completed": ex.completed, "distance": ex.distance, "stop_reason": ex.stop_reason,
            **timing_summary(ex.calls)}


def identify_and_plan(ex, kind: str) -> dict:
    """Identification from a logged exploration lap and the re-plan (l8-M1: an error of the
    identification -> "identification failed", no plan -> "plan failed"; no retry). Untimed – a pit
    stop between the two laps."""
    from ..planning.replan import replan_robust
    from .l6 import replan_slot
    from .l8 import identify_run

    P = plans()
    out: dict = {"explore": explore_summary(ex)}
    try:
        ident = identify_run(ex, lag=kind == "lag_identified")
    except Exception as e:                       # noqa: BLE001 (l8-M1)
        out.update({"failed": "identification failed", "identification_error": f"{type(e).__name__}: {e}"})
        return out
    out["identification"] = {k: v for k, v in ident.items() if k != "vehicle"}
    with replan_slot():
        o, log = replan_robust(Track.load("Spielberg"), ident["vehicle"],
                               [("nominal plan", P[0.95]), ("ideal plan", P[1.0])], 0.95, nominal=DynVehicle())
    out["plan_log"] = log
    if o is None:
        out["failed"] = "plan failed"
        return out
    out.update({"plan": o, "plan_lap_time": float(o.lap_time), "vehicle": ident["vehicle"],
                "tau": float(ident["tau"]) if kind == "lag_identified" else None})
    return out


def drive(case: dict, kind: str, impl: str, mode: str, delay: float = 0.0, explore_mode: str | None = None,
          name: str = "l9", planned: dict | None = None) -> dict:
    """One lap of a model-based controller (protocol section 4) in an implementation and mode. For
    ``identified`` / ``lag_identified``: ``planned`` is the result of ``identify_and_plan`` (the
    staged runs of scripts/run_l9.py); without it the exploration lap, identification and re-plan
    run here first."""
    from .l1 import dyn_reference
    from .l2 import x0_from_plan
    from .l6 import smpc_spec

    track = Track.load("Spielberg")
    P = plans()
    nom = DynVehicle()
    out: dict = {"t_max": T_MAX}
    lag = None
    if kind == "nmpc":
        o, model, eta, backoff = P[0.95], nom, 0.95, None
    elif kind == "nmpc_reserve":
        o, model, eta, backoff = P[0.88], nom, 0.88, None
    elif kind == "smpc":
        sp = smpc_spec(json.loads((ROOT / "results" / "L3" / "calibration.json").read_text()))
        o, model, eta, backoff = P[sp["plan_eta"]], nom, np.asarray(sp["eta"], float), np.asarray(sp["backoff"], float)
    elif kind in ("identified", "lag_identified"):
        if planned is None:
            ex = explore_lap(case, track, impl, explore_mode or mode, name)
            planned = identify_and_plan(ex, kind)
        out.update({k: v for k, v in planned.items() if k not in ("plan", "vehicle", "tau", "failed")})
        out["explore_mode"] = explore_mode or mode
        if "failed" in planned:
            out.update({"completed": False, "stop_reason": planned["failed"], "lap_time": float("nan"),
                        "distance": 0.0})
            return out
        o, model, eta, backoff = planned["plan"], planned["vehicle"], 0.95, None
        if kind == "lag_identified":
            lag = max(planned["tau"], 1e-4)
    else:
        raise ValueError(kind)
    path, _ = dyn_reference(track, model, o)
    mpc = make_mpc(impl, track, model, o, eta, backoff, lag, name=name)
    ctl = (RealTimeLagMPCController(mpc, DT, mode, delay) if lag is not None
           else RealTimeMPCController(mpc, DT, mode, delay))
    sim = make_sim(case, track)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=T_MAX)
    out.update({"completed": bool(r.completed), "lap_time": float(r.lap_time), "stop_reason": r.stop_reason,
                "distance": float(r.progress[-1]), **timing_summary(ctl.calls)})
    return out


__all__ = ["cases", "plans", "make_mpc", "RealTimeMPCController", "RealTimeLagMPCController", "explore_lap",
           "timing_summary", "explore_summary", "identify_and_plan", "drive", "KINDS", "IMPLS", "MODES"]
