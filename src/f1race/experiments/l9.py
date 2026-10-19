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


class ShiftFrenetMPC(FrenetMPC):
    """``FrenetMPC`` (unchanged) whose warm start can be shifted by ``shift`` 50 ms steps (L9 A1.3):
    the stored solution is pre-shifted by shift - 1 steps before ``FrenetMPC.solve`` shifts it by
    one (shift = 0: pre-shifted back by one). ``shift=1`` is FrenetMPC exactly."""

    def solve(self, x0, u_prev, shift: int = 1, **kw):
        m = int(shift) - 1
        if self.z is not None and m != 0:
            X, U = self.z[self.Xi].copy(), self.z[self.Ui].copy()
            if m > 0:
                m = min(m, len(U))
                X = np.vstack([X[m:], np.repeat(X[-1:], m, 0)])
                U = np.vstack([U[m:], np.repeat(U[-1:], m, 0)])
            else:
                X, U = np.vstack([X[:1], X[:-1]]), np.vstack([U[:1], U[:-1]])
            self.z = self.z.copy()
            self.z[self.Xi], self.z[self.Ui] = X, U
        return super().solve(x0, u_prev, **kw)

    def presolve(self, x0, u_prev) -> None:
        """A1.4: the plan before the lap starts (one IPOPT solve from the initial guess, untimed)."""
        FrenetMPC.solve(self, x0, u_prev)


def make_mpc(impl: str, track: Track, model: DynVehicle, plan, eta, backoff=None, lag=None, name: str = "l9"):
    from .l2 import ref_from_plan

    ref = ref_from_plan(plan)
    if impl == "ipopt":
        return ShiftFrenetMPC(track, model, ref, mode="track", N=N, dt=PERIOD, eta=eta, backoff=backoff, steer_lag=lag)
    from ..control.acados_mpc import AcadosFrenetMPC

    return AcadosFrenetMPC(track, model, ref, N=N, dt=PERIOD, eta=eta, backoff=backoff, steer_lag=lag,
                           nlp="SQP_RTI" if impl == "rti" else "SQP", name=name)


class _RealTime:
    """The real-time rule around ``mpc.solve`` (mixin of the two controllers below).

    ``hold="plan"`` (amendment A1, the main rule): a solve started at t_k and ready at t_k + c
    (rounded up to the simulator step) is executed from then on time-aligned – at time t its
    segment j = floor((t - t_k) / 50 ms); the next solution replaces it as soon as it is ready; past
    its last segment (j >= N) the steering rate is 0 and the forces of the last segment are held
    (``exhausted`` counts those steps). The warm start of a solve is the previous solution shifted
    by the 50 ms steps elapsed since the previous solve started (A1.3), and in the real-time mode a
    plan is computed before the lap starts, untimed (A1.4). ``hold="last"`` (the rule of revision 1,
    a secondary condition): the first command of a solution is applied when ready and held."""

    def _rt_init(self, mode: str, delay: float, hold: str = "plan"):
        if mode not in MODES or hold not in ("plan", "last"):
            raise ValueError((mode, hold))
        self.rt_mode, self.delay, self.hold = mode, float(delay), hold
        self.pending = None                      # (t_ready, t_start, first command, plan inputs)
        self.active = None                       # (t_start, plan inputs (N, 3))
        self.busy_until = 0.0
        self.last_start = None
        self.exhausted = 0
        self.calls: list[dict] = []
        self._buf: deque = deque(maxlen=int(round(self.delay / self.sim_dt)) + 1)

    def _measure(self, x):
        """Sensor delay: the state of t - delay (the oldest in the buffer until it is full)."""
        self._buf.append(np.array(x, float))
        return self._buf[0]

    def _plan_command(self, t: float) -> np.ndarray:
        t0, U = self.active
        j = int(math.floor((t - t0) / PERIOD + 1e-9))
        if j < len(U):
            return np.array(U[j], float)
        self.exhausted += 1
        return np.array([0.0, U[-1][1], U[-1][2]], float)

    def _schedule(self, t: float, xf: np.ndarray) -> None:
        """Pre-start plan (real-time, once); switch to a solution whose time has come; start a
        computation at a control instant when the controller is free; set the current command."""
        rt = self.rt_mode == "realtime"
        if rt and self.last_start is None and self.hold == "plan":
            self.mpc.presolve(xf, self.u)                  # A1.4, untimed
            self.active, self.last_start = (t, self.mpc.prediction()[1]), t
        if self.pending is not None and t + 1e-9 >= self.pending[0]:
            if self.hold == "plan":
                self.active = (self.pending[1], self.pending[3])
            else:
                self.u = self.pending[2]
            self.pending = None
        if not (t + 1e-9 < self.t_next or (rt and t + 1e-9 < self.busy_until)):
            shift = 1 if (not rt or self.last_start is None) else int(round((t - self.last_start) / PERIOD))
            t0 = time.perf_counter()
            u, info = self.mpc.solve(xf, self.u, shift=shift)
            wall = time.perf_counter() - t0
            U = self.mpc.prediction()[1]
            if "time_feedback" in info:          # RTI: latency = feedback, busy = feedback + preparation
                lat, busy = info["time_feedback"], info["time_feedback"] + info["time_preparation"]
            else:
                lat, busy = wall, wall
            rec = {"t": t, "latency": lat, "busy": busy, "wall": wall, "status": info["status"], "shift": shift,
                   "fallback": bool(info.get("fallback", False))}
            if not rt:
                self.u = u
                self.active = (t, U)
                self.t_next = t + PERIOD
            else:
                t_ready = t + math.ceil(lat / self.sim_dt - 1e-9) * self.sim_dt
                self.pending = (t_ready, t, np.array(u, float), U)
                self.busy_until = t + busy
                k = math.floor((t + busy) / PERIOD - 1e-9) + 1     # the first control instant after it
                self.t_next = max(t + PERIOD, k * PERIOD)
                rec.update({"miss": busy > PERIOD, "t_apply": t_ready})
            self.last_start = t
            self.calls.append(rec)
        if self.hold == "plan" and self.active is not None:
            self.u = self._plan_command(t)


class RealTimeMPCController(_RealTime, MPCController):
    def __init__(self, mpc, sim_dt: float = DT, mode: str = "offline", delay: float = 0.0, tc_eta=0.99,
                 hold: str = "plan"):
        MPCController.__init__(self, mpc, sim_dt, tc_eta)
        self._rt_init(mode, delay, hold)

    def __call__(self, t: float, x: np.ndarray):
        from ..control.dyn_tracking import traction_limit

        xm = self._measure(x)
        xf = self.frenet(xm)
        self._schedule(t, xf)
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:              # traction control on the true state, every step
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


class RealTimeLagMPCController(_RealTime, LagAwareMPCController):
    """``LagAwareMPCController`` (observe="command") under the real-time rule."""

    def __init__(self, mpc, sim_dt: float = DT, mode: str = "offline", delay: float = 0.0, tc_eta=0.99,
                 hold: str = "plan"):
        LagAwareMPCController.__init__(self, mpc, sim_dt, tc_eta, observe="command")
        self._rt_init(mode, delay, hold)

    def __call__(self, t: float, x: np.ndarray):
        from ..control.dyn_tracking import traction_limit

        xm = self._measure(x)
        d_act, d_cmd = self.observe(t, float(xm[6])), float(xm[6])
        xa = np.array(xm, float)
        xa[6] = d_act
        xf = np.concatenate([self.frenet(xa), [d_cmd]])
        self._schedule(t, xf)
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = d_act + (d_cmd - d_act) * (1.0 - np.exp(-self.sim_dt / self.tau))
            fd, fb = traction_limit(veh, xa, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb


def make_sim(case: dict, track: Track, seed_offset: int = 0):
    from .l7 import make_sim_for

    return make_sim_for(case, track, seed_offset)


def explore_lap(case: dict, track: Track, impl: str, mode: str, name: str, hold: str = "plan"):
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
    ctl = RealTimeMPCController(mpc, DT, mode, hold=hold)
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
           "fallbacks": int(sum(c["fallback"] for c in calls)),
           "shift_max": int(max(c.get("shift", 1) for c in calls))}
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
          name: str = "l9", planned: dict | None = None, hold: str = "plan") -> dict:
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
            ex = explore_lap(case, track, impl, explore_mode or mode, name, hold)
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
    ctl = (RealTimeLagMPCController(mpc, DT, mode, delay, hold=hold) if lag is not None
           else RealTimeMPCController(mpc, DT, mode, delay, hold=hold))
    sim = make_sim(case, track)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=T_MAX)
    out.update({"completed": bool(r.completed), "lap_time": float(r.lap_time), "stop_reason": r.stop_reason,
                "distance": float(r.progress[-1]), "hold": hold, "plan_exhausted_steps": int(ctl.exhausted),
                **timing_summary(ctl.calls)})
    return out


# ---------------------------------------------------------------- RL policies and the bare tracker
def _l9env_class():
    from ..rl.env import RacingEnv

    class L9Env(RacingEnv):
        """``RacingEnv`` with the real-time rule for the policy's action (section 3; A1: no plan, the
        previous action held): ``step_rt(action, k)`` keeps the previous action for the first k
        simulator steps of the decision (k = the policy's latency in 10 ms steps; k >= 5 carries the
        rest over to the next decision) – ``k = 0`` is ``RacingEnv.step``. ``delay``: the tracker and
        the policy see the state of t - delay."""

        delay = 0.0

        def reset(self, *, seed=None, options=None):
            self._carry = None                     # (remaining steps, action) of a late action
            self._prev_a = np.zeros(2)
            self._dbuf = deque(maxlen=int(round(self.delay / self.dt)) + 1)
            return super().reset(seed=seed, options=options)

        def _measure(self, x):
            self._dbuf.append(np.array(x, float))
            return super()._measure(self._dbuf[0])

        def _set_action(self, a):
            bound = self.steer_max
            if self.steer_mode == "lateral":
                v = max(float(self._meas[3]), 1.0)
                bound = min(bound, self.a_corr * self.nominal.L / (v * v))
            self.ctl.d_offset = bound * a[0]
            self.ctl.v_scale = 1.0 + (self.speed_up if a[1] > 0 else self.speed_down) * a[1]

        def step_rt(self, action, k: int):
            a = np.clip(np.asarray(action, float), -1.0, 1.0)
            pending = (int(k), a)
            tr = self.track
            ds_total, crashed, reason = 0.0, False, ""
            self._set_action(self._prev_a)
            for i in range(self.ctrl_every):
                if pending is not None and i >= pending[0]:
                    self._set_action(pending[1])
                    self._prev_a = pending[1]
                    pending = None
                u = self.sim.apply_limits(self.x, self.ctl(self.t, self._measure(self._view(self.x))))
                self.x = self.sim.step(self.x, u)
                self.t += self.dt
                s, mr, mf = self.margins(self.x)
                ds = (s - self.s_prev + 0.5 * tr.length) % tr.length - 0.5 * tr.length
                self.s_prev = s
                ds_total += ds
                beta = np.arctan2(self.x[4], max(self.x[3], 1e-3))
                if min(mr, mf) < 0.0:
                    crashed, reason = True, "off track"
                    break
                if abs(beta) > 0.5 or self.x[3] < 1.0:
                    crashed, reason = True, "spin"
                    break
            self._carry = None if pending is None else (pending[0] - self.ctrl_every, pending[1])
            self.progress += ds_total
            if crashed:
                obs = np.zeros(self.observation_space.shape, np.float32)
            else:
                self._hist = self._hist[1:] + [self._obs(self._meas)]
                self._last_a = a.astype(np.float32)
                obs = self._stacked()
            truncated = (not crashed) and self.t >= self.episode_s
            return obs, crashed, truncated, {"progress": self.progress, "t": self.t, "reason": reason}

    return L9Env


def make_env(steer_mode: str = "angle", delay: float = 0.0):
    env = _l9env_class()(plans()[0.95], randomize_plant=False, history=8, steer_mode=steer_mode, observe="command")
    env.delay = float(delay)
    return env


def rl_lap(policy, env, case: dict, mode: str, t_max: float = T_MAX) -> dict:
    """One flying lap of tracker + ``policy`` (None: the bare tracker) under the rule of section 3:
    offline – the action takes effect at the decision; real-time – the policy call is timed and its
    action takes effect after the latency rounded up to 10 ms, the previous action held until then;
    a computation longer than the period delays the next decision (the policy is busy)."""
    env.episode_s = t_max
    opts = {"plant": case["plant"], "start_index": 0, "disturbances": case["dist_seed"] is not None,
            "dist_seed": case["dist_seed"] or 0}
    if case["tau"] > 0:
        opts["steer_lag"] = case["tau"]
    obs, _ = env.reset(options=opts)
    L = env.track.length
    calls, busy_until, done, info, carry_a = [], 0.0, False, {}, None
    while not done:
        t = env.t
        if policy is None:
            a, k = np.zeros(2), 0
        elif mode == "realtime" and t + 1e-9 < busy_until:
            a, k = env._prev_a, 0                    # still computing: nothing new this decision
        else:
            t0 = time.perf_counter()
            a = policy(obs)
            c = time.perf_counter() - t0
            k = 0 if mode == "offline" else math.ceil(c / env.dt - 1e-9)
            busy_until = t + c
            calls.append({"t": t, "latency": c, "busy": c, "miss": c > PERIOD, "fallback": False, "status": 0})
        if env._carry is not None:                    # a late action from the previous decision
            k_c, a_c = env._carry
            obs, term, trunc, info = env.step_rt(a_c, max(k_c, 0))
        else:
            obs, term, trunc, info = env.step_rt(a, k)
        done = term or trunc or env.progress >= L
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)
    out = {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
           "stop_reason": "lap completed" if completed else (info.get("reason") or "time limit"),
           "distance": float(env.progress)}
    if calls:
        out.update(timing_summary(calls) if mode == "realtime" else {"calls": len(calls)})
    return out


__all__ = ["cases", "plans", "make_mpc", "make_env", "rl_lap", "RealTimeMPCController", "RealTimeLagMPCController", "explore_lap",
           "timing_summary", "explore_summary", "identify_and_plan", "drive", "KINDS", "IMPLS", "MODES"]
