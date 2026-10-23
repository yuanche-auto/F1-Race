"""L7 Part B: the safety layer – sets, the filtered closed loop of the RL policy, exploration
laps and the three risk estimates (docs/L7_protocol.md section 4, amendment A3).

* ``cases(name)``: the laps of a set (K, A, V, C, T1, T2), each a car and a condition
  (steering lag tau, or the L3 disturbances with a seed).
* ``run_lap``: one flying lap of tracker + policy (``RacingEnv``) with an optional filter in
  the loop (A3.2) and a log of the true car every 10 ms and of the filter's predictions.
* ``NMPCFilter`` / ``CBFFilter``: the two filters behind one interface.
* ``explore_lap``: the exploration lap of a case (A3.7) as the controller logged it.
* risk estimates: ``prediction_samples`` (the L3 prediction errors, generalised as in A3.4),
  ``estimate_a`` (identified structure, A3.6), ``residual_for`` / ``sigma_k`` / ``conformal_q``
  (A3.5); (b) is ``experiments.l3.calibrate`` on the samples.
"""
from __future__ import annotations

import math
import pickle
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ..control.nmpc import FS, FrenetMPC, MPCController
from ..dynamics import DynVehicle
from ..sim.stochastic import NOISE_SIGMA, nominal_usage
from ..track import Track
from .l2 import ref_from_plan, sample_plants

ROOT = Path(__file__).resolve().parents[3]
DT, MPC_DT, N = 0.01, 0.05, 30
TAUS = (0.0, 0.03, 0.06)
EXPLORE_SEED_OFFSET = 100_000          # A3.7: a separate draw of the disturbances
NOISE_SEED_OFFSET = 200_000            # measurement noise added to a logged disturbance exploration lap
TRUNC_S = 2.0                          # predictions reaching into the last 2 s of a failed lap dropped
CHANGE_RATE, CHANGE_FORCE = 0.01, 100.0


# ---------------------------------------------------------------- sets
SETS = {   # name: (car seed, number of cars, with the nominal car, disturbance seeds)
    "T1": (0, 24, True, ()), "T2": (None, 0, False, tuple(range(30))),
    "V": (1, 10, False, tuple(range(2000, 2010))), "K": (7, 10, False, tuple(range(3000, 3010))),
    "A": (8, 5, False, tuple(range(4000, 4005))), "C": (9, 10, False, tuple(range(5000, 5010))),
    "T1new": (10, 24, True, ()), "T2new": (None, 0, False, tuple(range(6000, 6030))),
    "X": (99, 2, False, (9999,)),      # smoke tests of the code only, not a protocol set
}


def cases(name: str) -> list[dict]:
    """The laps of a set (protocol section 2; A3.1: K, A, V, C without the nominal car)."""
    seed, n, with_nominal, dist = SETS[name]
    nom = DynVehicle()
    out = []
    if seed is not None:
        cars = sample_plants(nom, n, seed)
        idx = range(len(cars)) if with_nominal else range(1, len(cars))
        for tau in TAUS:
            for i in idx:
                out.append({"set": name, "car": i, "info": cars[i][0], "plant": cars[i][1], "tau": tau,
                            "dist_seed": None, "name": f"{name}_car{i}_tau{int(round(tau * 1000))}"})
    for d in dist:
        out.append({"set": name, "car": None, "info": {"nominal": True}, "plant": nom, "tau": 0.0,
                    "dist_seed": d, "name": f"{name}_dist{d}"})
    return out


def plan(eta: float = 0.95):
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    if eta in cache["plans"]:
        return cache["plans"][eta]
    return pickle.load(open(ROOT / "results" / "L2" / "plan_088.pkl", "rb"))       # 0.88 (L2)


def changed(u_out, u) -> bool:
    d = np.abs(np.asarray(u_out, float) - np.asarray(u, float))
    return bool(d[0] > CHANGE_RATE or d[1] > CHANGE_FORCE or d[2] > CHANGE_FORCE)


# ---------------------------------------------------------------- filters
class NMPCFilter:
    """``FrenetMPC(mode="filter")`` around the proposal (A3.2, A3.3). ``tau_model``: lag in
    the model (estimate (a)); ``eta`` / ``backoff``: static tightening ((b)); ``band``: a
    function (filter, xf) -> (eta_k, b_k) evaluated at every decision ((c)). ``horizon``: the
    number of 50 ms stages (30; 10 for candidate (2) of the training filter, A6). ``retry``
    (control expansion phase F, docs/CX_F_protocol.md): failed solves retried as
    ``FrenetMPC(retry=True)``; default off – L7's filter (L6's failure handling)."""

    def __init__(self, track: Track, model: DynVehicle, ref_plan, tau_model: float | None = None,
                 eta=0.95, backoff=None, band=None, record: bool = True, horizon: int = N,
                 retry: bool = False):
        self.mpc = FrenetMPC(track, model, ref_from_plan(ref_plan), mode="filter", N=horizon, dt=MPC_DT,
                             eta=eta, backoff=backoff, steer_lag=tau_model,
                             param_tightening=band is not None, retry=retry)
        self.conv = MPCController(self.mpc, DT, tc_eta=None)     # its frenet() only
        self.tau, self.band, self.record = tau_model, band, record
        self.u_prev = np.zeros(3)
        self.d_hat = self.c_prev = self.t_prev = None
        self.predictions: list[tuple] = []                       # (t, X, U, b_k)
        self.states: list[tuple] = []                            # (t, y, angle estimate, Frenet state)
        self.stats: list[dict] = []

    def observe(self, t: float, y: np.ndarray) -> None:
        """Every 10 ms: the open-loop observer of the road-wheel angle (as LagAwareMPCController)."""
        if self.tau is None:
            return
        dc = float(y[6])
        if self.d_hat is None:
            self.d_hat = dc
        else:
            h = t - self.t_prev
            if h > 0.0:
                rho = (dc - self.c_prev) / h
                self.d_hat = dc - rho * self.tau + (self.d_hat - self.c_prev + rho * self.tau) * math.exp(-h / self.tau)
        self.c_prev, self.t_prev = dc, t

    def frenet(self, y: np.ndarray) -> np.ndarray:
        if self.tau is None:
            return self.conv.frenet(y)
        ya = np.array(y, float)
        ya[6] = self.d_hat if self.d_hat is not None else ya[6]
        return np.concatenate([self.conv.frenet(ya), [float(y[6])]])

    def filter(self, t: float, y: np.ndarray, u_des: np.ndarray):
        xf = self.frenet(y)
        self.states.append((t, np.array(y, float), self.d_hat, np.array(xf, float)))
        kw = {}
        b = self.mpc.backoff
        if self.band is not None:
            eta_k, b = self.band(self, xf)
            kw = {"eta": eta_k, "backoff": b}
        u, info = self.mpc.solve(xf, self.u_prev, u_des=u_des, **kw)
        if self.record:
            X, U = self.mpc.prediction()
            self.predictions.append((t, X.astype(np.float32), U.astype(np.float32), np.asarray(b, np.float32)))
        info = {"fail": bool(info["fallback"]), "slack": info["slack"], "limit_slack": info["limit_slack"],
                "time": info["time"]}
        self.stats.append(info)
        return np.asarray(u, float), info

    def applied(self, u: np.ndarray) -> None:
        self.u_prev = np.asarray(u, float)


class CBFFilter:
    """``CBFQPFilter`` behind the same interface (A3.9)."""

    def __init__(self, track: Track, veh: DynVehicle, alpha: float, margin: float, eta: float):
        from ..control.safety_filter import CBFQPFilter

        # the proposal aims 10 ms ahead (the tracker), a filtered command is held for the
        # decision's 50 ms (A3.2): its rate reaches the filtered angle at the end of the hold
        self.f = CBFQPFilter(track, veh, eta=eta, margin=margin, alpha=alpha, ctrl_dt=DT, hold_dt=MPC_DT)
        self.track = track
        self.conv = MPCController(SimpleNamespace(track=track), DT, tc_eta=None)
        self.predictions, self.stats = [], []

    def observe(self, t, y) -> None:
        return

    def filter(self, t, y, u_des):
        s, n, xi_h = self.conv.frenet(y)[:3]
        xi = xi_h + math.atan2(float(y[4]), max(float(y[3]), 1.0))
        u, info = self.f.filter((s % self.track.length, n, xi, xi_h), y, u_des)
        info = {"fail": not info["ok"], "slack": info["sigma_edge"], "limit_slack": info["sigma_friction"],
                "time": info["time"]}
        self.stats.append(info)
        return u, info

    def applied(self, u) -> None:
        return


# ---------------------------------------------------------------- the closed loop
def make_env(steer_mode: str = "angle"):
    from ..rl.env import RacingEnv

    return RacingEnv(plan(0.95), randomize_plant=False, history=8, steer_mode=steer_mode, observe="command")


def env_snapshot(env) -> dict:
    """The episode state of a ``RacingEnv`` (to restart a rollout from it, section 6)."""
    import copy

    ctl = env.ctl
    snap = {"x": np.array(env.x, float), "t": env.t, "progress": env.progress, "s_prev": env.s_prev,
            "hist": [h.copy() for h in env._hist], "last_a": env._last_a.copy(),
            "meas": np.array(env._meas, float),
            "ctl": {"idx": ctl.idx, "d_offset": ctl.d_offset, "v_scale": ctl.v_scale, "last": ctl.last},
            "hint": list(env.margins.hint),
            "noise_rng": copy.deepcopy(env.noise_rng) if env.noise is not None else None}
    sim = env.sim
    if hasattr(sim, "f_now"):                        # StochasticSim: time, friction, gust
        snap["sim"] = {"t": sim.t, "f_now": sim.f_now, "gust_now": sim.gust_now, "veh": sim.veh}
    return snap


def env_restore(env, case: dict, snap: dict) -> np.ndarray:
    """Reset ``env`` to ``case`` and overwrite its episode state with ``snap``; returns the
    observation."""
    opts = {"plant": case["plant"], "start_index": 0, "disturbances": case["dist_seed"] is not None,
            "dist_seed": case["dist_seed"] or 0}
    if case["tau"] > 0:
        opts["steer_lag"] = case["tau"]
    env.reset(options=opts)
    env.x, env.t, env.progress, env.s_prev = snap["x"].copy(), snap["t"], snap["progress"], snap["s_prev"]
    env._hist = [h.copy() for h in snap["hist"]]
    env._last_a = snap["last_a"].copy()
    env._meas = snap["meas"].copy()
    for k, v in snap["ctl"].items():
        setattr(env.ctl, k, v)
    env.margins.hint = list(snap["hint"])
    if snap["noise_rng"] is not None:
        import copy

        env.noise_rng = copy.deepcopy(snap["noise_rng"])
    if "sim" in snap:
        for k, v in snap["sim"].items():
            setattr(env.sim, k, v)
    return env._stacked()


def rollout_unfiltered(policy, env, case: dict, snap: dict, decisions: int = 60) -> dict:
    """Section 6, false interventions: from the state at an intervention, the policy without
    the filter for ``decisions`` decisions (3 s); a crash means the intervention was needed."""
    obs = env_restore(env, case, snap)
    episode_s = env.episode_s
    env.episode_s = snap["t"] + decisions * env.dt * env.ctrl_every + 1.0
    reason, n = "", 0
    for n in range(1, decisions + 1):
        a = np.zeros(2) if policy is None else policy(obs)
        obs, _, term, trunc, info = env.step(a)
        if term:
            reason = info.get("reason", "crash")
            break
        if trunc or env.progress >= env.track.length:
            break
    env.episode_s = episode_s
    return {"crashed": bool(reason), "reason": reason, "decisions": n}


def run_lap(policy, env, case: dict, filt=None, t_max: float = 120.0, snap_at=(),
            log_commands: bool = False) -> dict:
    """One flying lap of tracker + ``policy`` (obs -> action; None: the bare tracker) on
    ``case``, with ``filt`` in the loop (A3.2). Returns the outcome, the log of the true car
    (10 ms: t, state[:7], applied input, true usage) and the filter's record. ``snap_at``:
    lap distances [m]; at the first intervention at or after each, the episode state before
    that decision is kept (``snapshots``) for the false-intervention rollouts (section 6).
    ``log_commands`` (control expansion phase F; default off): the first 10 ms command of
    every decision as proposed (after the plant's limits) and as applied, in
    ``decisions["u_prop"]`` / ``decisions["u_appl"]`` – also without a filter."""
    episode_s = env.episode_s
    env.episode_s = t_max
    opts = {"plant": case["plant"], "start_index": 0, "disturbances": case["dist_seed"] is not None,
            "dist_seed": case["dist_seed"] or 0}
    if case["tau"] > 0:
        opts["steer_lag"] = case["tau"]
    obs, _ = env.reset(options=opts)
    sim = env.sim
    orig = sim.apply_limits
    st = {"first": False, "held": None}
    T, X, U, US = [], [], [], []
    dec = {"t": [], "changed": [], "fail": [], "slack": [], "limit_slack": [], "time": [], "s": []}
    if log_commands:
        dec.update({"u_prop": [], "u_appl": []})

    def apply_limits(x, u):
        out = orig(x, u)
        y = env._meas                                   # what the tracker saw at this step
        if log_commands and filt is None and st["first"]:
            st["first"] = False
            dec["u_prop"].append(np.array(out, float))
            dec["u_appl"].append(np.array(out, float))
        if filt is not None:
            filt.observe(env.t, y)
            if st["first"]:
                st["first"] = False
                u_f, info = filt.filter(env.t, np.asarray(y, float), out)
                ch = changed(u_f, out)
                st["held"] = np.asarray(u_f, float) if ch else None
                dec["t"].append(env.t)
                dec["changed"].append(ch)
                dec["s"].append(env.ctl.last[0] if env.ctl.last is not None else float("nan"))
                for k in ("fail", "slack", "limit_slack", "time"):
                    dec[k].append(info[k])
                if log_commands:
                    dec["u_prop"].append(np.array(out, float))
                    dec["u_appl"].append(np.array(orig(x, st["held"]) if ch else out, float))
            if st["held"] is not None:
                out = orig(x, st["held"])
            filt.applied(out)
        T.append(env.t)
        X.append(np.array(x[:7], float))
        U.append(np.array(out, float))
        US.append(sim._forces(x, out)[1])
        return out

    sim.apply_limits = apply_limits
    L = env.track.length
    done, info = False, {}
    pending = sorted(snap_at)
    snapshots = []
    while not done:
        a = np.zeros(2) if policy is None else policy(obs)
        snap = env_snapshot(env) if pending and env.progress >= pending[0] else None
        n_dec = len(dec["t"])
        st["first"], st["held"] = True, None
        obs, _, term, trunc, info = env.step(a)
        if snap is not None and len(dec["changed"]) > n_dec and dec["changed"][-1]:
            snapshots.append({"at": pending.pop(0), "progress": snap["progress"], "snap": snap})
        done = term or trunc or env.progress >= L
    env.episode_s = episode_s
    sim.apply_limits = orig
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)
    log = SimpleNamespace(t=np.array(T), state=np.array(X), inputs=np.array(U), usage=np.array(US),
                          completed=bool(completed))
    return {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
            "stop_reason": "lap completed" if completed else (info.get("reason") or "time limit"),
            "distance": float(env.progress), "log": log,
            "decisions": {k: np.array(v) for k, v in dec.items()}, "snapshots": snapshots}


# ---------------------------------------------------------------- exploration laps
def explore_lap(case: dict, track: Track) -> SimpleNamespace:
    """A3.7: the L6 ``explore`` lap (nominal NMPC, 12 % reserve plan, eta 0.88) with the
    case's lag, or a separate draw of its disturbances; the run as the controller logged
    it (7-dim, commanded angle in the steering slot; with the L3 measurement noise on a
    disturbance condition)."""
    from ..sim.dynamic import DynamicSim
    from ..sim.lagged import LaggedSim
    from ..sim.stochastic import StochasticSim, sample_disturbances
    from .l1 import dyn_reference
    from .l2 import x0_from_plan

    nom = DynVehicle()
    o = plan(0.88)
    path, _ = dyn_reference(track, nom, o)
    mpc = FrenetMPC(track, nom, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=0.88)
    ctl = MPCController(mpc, DT)
    x0 = x0_from_plan(o, path)
    noisy = case["dist_seed"] is not None
    if noisy:
        dist = sample_disturbances(track, case["dist_seed"] + EXPLORE_SEED_OFFSET, t_max=150.0)
        sim = StochasticSim(track, case["plant"], dist, DT)
    elif case["tau"] > 0:
        sim = LaggedSim(track, case["plant"], case["tau"], DT, observe="command")
    else:
        sim = DynamicSim(track, case["plant"], DT)
    r = sim.run(ctl, x0, t_max=120.0)
    state = np.array([sim.observe(x) for x in r.state]) if isinstance(sim, LaggedSim) else r.state.copy()
    if noisy:                                         # the logged lap is the measured one
        rng = np.random.default_rng(case["dist_seed"] + NOISE_SEED_OFFSET)
        state = state + NOISE_SIGMA * rng.standard_normal(state.shape)
    return SimpleNamespace(t=r.t, state=state, inputs=r.inputs, usage=r.usage, noisy=noisy,
                           completed=bool(r.completed), stop_reason=r.stop_reason,
                           distance=float(r.progress[-1]))


def make_sim_for(case: dict, track: Track, seed_offset: int = 0):
    """The case's simulator: L3 disturbances (with measurement noise), the steering lag
    (commanded angle observed), or neither."""
    from ..sim.dynamic import DynamicSim
    from ..sim.lagged import LaggedSim
    from ..sim.stochastic import StochasticSim, sample_disturbances

    if case["dist_seed"] is not None:
        return StochasticSim(track, case["plant"], sample_disturbances(track, case["dist_seed"] + seed_offset,
                                                                       t_max=150.0), DT)
    if case["tau"] > 0:
        return LaggedSim(track, case["plant"], case["tau"], DT, observe="command")
    return DynamicSim(track, case["plant"], DT)


def drive_baseline(case: dict, kind: str, track: Track) -> dict:
    """The model-based baselines of section 4.3 on one case, without a filter: ``nmpc`` (L2,
    nominal model, 5 % plan, eta 0.95), ``smpc`` (L3 SMPC eps 0.01, calibration unchanged, as
    in L6), ``lag_identified`` (the best of L6: tau and the parameters identified from the
    case's exploration lap – least squares without noise, the EKF with the lag under the L3
    noise –, the plan by ``replan_robust``, the lag-aware NMPC). L6 failure handling."""
    import json

    from ..control.nmpc import LagAwareMPCController
    from ..planning.replan import replan_robust
    from .l1 import dyn_reference
    from .l2 import x0_from_plan
    from .l6 import replan_slot, smpc_spec

    nom = DynVehicle()
    out = {}
    if kind == "nmpc":
        o, model, ctl_kind, eta, backoff, lag = plan(0.95), nom, "mpc", 0.95, None, None
    elif kind == "smpc":
        sp = smpc_spec(json.loads((ROOT / "results" / "L3" / "calibration.json").read_text()))
        o, model, ctl_kind, eta, backoff, lag = plan(sp["plan_eta"]), nom, "mpc", sp["eta"], sp["backoff"], None
    elif kind == "lag_identified":
        ex = explore_lap(case, track)
        ident = identify_case(ex)
        out["identification"] = {"tau": ident["tau"], "theta": ident["theta"], "explore_completed": ex.completed,
                                 "explore_distance": ex.distance}
        with replan_slot():
            o, log = replan_robust(track, ident["vehicle"], [("nominal plan", plan(0.95)), ("ideal plan", plan(1.0))],
                                   0.95, nominal=nom)
        out["plan_log"] = log
        if o is None:
            out.update({"completed": False, "stop_reason": "plan failed", "lap_time": float("nan"), "distance": 0.0})
            return out
        out["plan_lap_time"] = o.lap_time
        model, ctl_kind, eta, backoff, lag = ident["vehicle"], "lag", 0.95, None, max(ident["tau"], 1e-4)
    else:
        raise ValueError(kind)
    path, _ = dyn_reference(track, model, o)
    mpc = FrenetMPC(track, model, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=eta, backoff=backoff,
                    steer_lag=lag)
    ctl = LagAwareMPCController(mpc, DT, observe="command") if ctl_kind == "lag" else MPCController(mpc, DT)
    sim = make_sim_for(case, track)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=120.0)
    st = mpc.stats
    out.update({"completed": bool(r.completed), "lap_time": float(r.lap_time), "stop_reason": r.stop_reason,
                "distance": float(r.progress[-1]), "fallbacks": int(sum(x["fallback"] for x in st)),
                "solves": len(st), "solve_ms_mean": float(np.mean([x["time"] for x in st]) * 1e3) if st else None})
    return out


def cut(run, t0: float, t1: float) -> SimpleNamespace:
    m = (run.t >= t0) & (run.t < t1)
    return SimpleNamespace(t=run.t[m], state=run.state[m], inputs=run.inputs[m], usage=run.usage[m])


# ---------------------------------------------------------------- prediction errors (A3.4)
def axle_n(track: Track, veh: DynVehicle, state: np.ndarray) -> np.ndarray:
    """Lateral positions of the front and rear axle points of logged 7-dim states (K, 2)."""
    c, s = np.cos(state[:, 2]), np.sin(state[:, 2])
    out = []
    for arm in (veh.L - veh.lr, -veh.lr):
        _, n = track.project(state[:, 0] + arm * c, state[:, 1] + arm * s)
        out.append(n)
    return np.column_stack(out)


def prediction_samples(track: Track, model: DynVehicle, predictions, log, completed: bool) -> dict:
    """The L3 calibration samples of every recorded prediction (``l3._calibration_samples``
    with u_real the true car's usage and f = 1, A3.4): stage k, predicted usage of the
    axle closer to its limit, the true usage of that axle at the stage's time, the larger
    lateral error of the two axle points, the band b_k the filter used, the time."""
    step = int(round(MPC_DT / DT))
    K = len(log.t)
    n_ax = axle_n(track, model, log.state)
    arms = np.array([model.L - model.lr, -model.lr])
    t_end = float(log.t[-1])
    cols = {"k": [], "u_pred": [], "u_real": [], "f": [], "e_n": [], "t": [], "b": [], "p": []}
    for p_i, (t, Xp, Up, b) in enumerate(predictions):
        Xp, Up = np.asarray(Xp, float), np.asarray(Up, float)
        js = int(round(t / DT)) + np.arange(N + 1) * step
        m = js < K
        na_pred = Xp[:, 1:2] + arms[None, :] * np.sin(Xp[:, 2:3])
        e_n = np.abs(n_ax[js[m]] - na_pred[m]).max(axis=1)
        up = nominal_usage(model, Xp[:N, :7], Up)
        ax = np.argmax(up, axis=1)
        mk = m[:N]
        nm = int(m.sum())
        cols["k"].append(np.arange(N + 1)[m])
        cols["t"].append(np.full(nm, t))
        cols["p"].append(np.full(nm, p_i))
        cols["e_n"].append(e_n)
        cols["f"].append(np.ones(nm))
        cols["b"].append(np.broadcast_to(np.asarray(b, float), (N + 1,))[m])
        cols["u_pred"].append(np.concatenate([up[mk, ax[mk]], np.full(nm - int(mk.sum()), np.nan)]))
        ur = log.usage[js[:N][mk], ax[mk]]
        cols["u_real"].append(np.concatenate([ur, np.full(nm - int(mk.sum()), np.nan)]))
    out = {k: np.concatenate(v) if v else np.zeros(0) for k, v in cols.items()}
    out.update({"t_end": t_end, "completed": bool(completed)})
    return out


def rollout_errors_sim(track: Track, model: DynVehicle, tau: float | None, states, log) -> dict:
    """Superseded (A4-M1), kept only for the comparison reported in A4: the rollout with the
    simulator's 10 ms integration instead of the filter's discrete model. Open-loop
    predictions of the filter's car under the inputs actually applied: from
    the filter's state at each decision (measured y; with a lag model the road-wheel angle
    estimate), the model is stepped with the logged 10 ms inputs; per stage k (50 ms) the
    larger axle-point lateral error against the true car, and the model's usage of the axle
    closer to its limit against the true usage of that axle. Returns arrays over
    (decision, k) pairs like ``prediction_samples``."""
    from ..sim.dynamic import DynamicSim
    from ..sim.lagged import LaggedSim

    step = int(round(MPC_DT / DT))
    Kn = len(log.t)
    lag = tau is not None and tau > 0
    sim = LaggedSim(track, model, tau, DT, observe="command") if lag else DynamicSim(track, model, DT)
    n_ax = axle_n(track, model, log.state)
    arms = np.array([model.L - model.lr, -model.lr])
    cols = {"k": [], "e_n": [], "u_pred": [], "u_real": [], "t": [], "p": []}
    for p_i, (t, y, d_hat, *_) in enumerate(states):
        j0 = int(round(t / DT))
        if j0 >= Kn:
            continue
        x = np.concatenate([y[:6], [d_hat if d_hat is not None else y[6], y[6]]]) if lag else np.array(y[:7], float)
        for k in range(1, N + 1):
            jk = j0 + k * step
            if jk >= Kn:
                break
            up = []
            for j in range(j0 + (k - 1) * step, jk):
                u = log.inputs[j]
                up.append(sim._forces(x, u)[1])
                x = sim.step(x, u)
            c, s_ = math.cos(x[2]), math.sin(x[2])
            _, n = track.project(x[0] + arms * c, x[1] + arms * s_)
            upm = np.max(np.array(up), axis=0)
            ax = int(np.argmax(upm))
            cols["k"].append(k)
            cols["e_n"].append(float(np.abs(n_ax[jk] - n).max()))
            cols["u_pred"].append(float(upm[ax]))
            cols["u_real"].append(float(np.max(log.usage[jk - step:jk, ax])))
            cols["t"].append(t)
            cols["p"].append(p_i)
    out = {k: np.array(v) for k, v in cols.items()}
    out["f"] = np.ones(len(out["k"]))
    out.update({"t_end": float(log.t[-1]), "completed": bool(log.completed)})
    return out


def block_inputs(inputs: np.ndarray, j0: int, n_blocks: int) -> np.ndarray:
    """The applied 10 ms inputs averaged over 50 ms blocks from index j0 (the mean steering
    rate keeps the angle increment, the mean force the impulse); forces scaled as in the
    NLP (1 / FS). Fewer rows where the log ends."""
    step = int(round(MPC_DT / DT))
    out = []
    for k in range(n_blocks):
        a, b = j0 + k * step, j0 + (k + 1) * step
        if b > len(inputs):
            break
        u = inputs[a:b].mean(axis=0)
        out.append([u[0], u[1] / FS, u[2] / FS])
    return np.array(out)


def model_rollout(mpc: FrenetMPC, xf0: np.ndarray, U: np.ndarray) -> np.ndarray:
    """The filter's own discrete model (``FrenetMPC._step``: Frenet coordinates, one RK4 step
    of 50 ms, the input held) from ``xf0`` under the scaled block inputs ``U``."""
    X = [np.asarray(xf0, float)]
    for u in U:
        X.append(np.asarray(mpc._step(X[-1], u)).ravel())
    return np.array(X)


def rollout_errors(track: Track, mpc: FrenetMPC, states, log) -> dict:
    """A4 / A4-M1: the error of the filter's own model rolled out open loop from the filter's
    Frenet state at each decision (with a lag model, the road-wheel angle estimate) under the
    applied inputs averaged over each 50 ms block. Per stage k (k * 50 ms): the larger
    axle-point lateral error, predicted n + arm sin(xi) (as the filter's track constraint and
    L3's calibration; the chord stays a separate margin of the constraint) against the true
    car's axle points; the model's ellipse usage (the filter's expression) at state k with
    block k (the 50 ms after X_k, as the NLP's stage-k ellipse constraint, imposed at the
    node) of the axle closer to its limit, against the true usage of that axle at t_k (with
    the input applied then), as L3's u_pred / u_real. Rows over (decision p, stage k)."""
    step = int(round(MPC_DT / DT))
    veh = mpc.veh
    Kn = len(log.t)
    n_ax = axle_n(track, veh, log.state)
    arms = np.array([veh.L - veh.lr, -veh.lr])
    cols = {"k": [], "e_n": [], "u_pred": [], "u_real": [], "t": [], "p": []}
    for p_i, (t, _, _, xf) in enumerate(states):
        j0 = int(round(t / DT))
        U = block_inputs(log.inputs, j0, N)
        if len(U) == 0:
            continue
        X = model_rollout(mpc, xf, U)                        # (len(U) + 1, nx)
        up = nominal_usage(veh, X[:len(U), :7], U * np.array([1.0, FS, FS]))
        for k in range(1, len(X)):
            jk = j0 + k * step
            if jk >= Kn:
                break
            na = X[k, 1] + arms * np.sin(X[k, 2])
            cols["k"].append(k)
            cols["e_n"].append(float(np.abs(n_ax[jk] - na).max()))
            if k < len(U):
                ax = int(np.argmax(up[k]))
                cols["u_pred"].append(float(up[k, ax]))
                cols["u_real"].append(float(log.usage[jk, ax]))
            else:
                cols["u_pred"].append(np.nan)
                cols["u_real"].append(np.nan)
            cols["t"].append(t)
            cols["p"].append(p_i)
    out = {k: np.array(v) for k, v in cols.items()}
    out["f"] = np.ones(len(out["k"]))
    out.update({"t_end": float(log.t[-1]), "completed": bool(log.completed)})
    return out


def horizon_ok(smp: dict) -> np.ndarray:
    """A3.8: predictions whose horizon reaches into the last 2 s of a failed lap dropped."""
    if smp["completed"]:
        return np.ones(len(smp["k"]), bool)
    return np.asarray(smp["t"]) + N * MPC_DT <= smp["t_end"] - TRUNC_S


# ---------------------------------------------------------------- (a) identified structure (A3.6)
def identify_case(run, dt: float = DT) -> dict:
    from ..estimation.ekf import ekf_identify_lag
    from ..estimation.identify import identify_with_lag

    nom = DynVehicle()
    if getattr(run, "noisy", False):
        r = ekf_identify_lag(run.state, run.inputs, run.usage, dt, nom, NOISE_SIGMA)
    else:
        r = identify_with_lag(run, dt, nom)
    return {"vehicle": r["vehicle"], "tau": float(r["tau"]), "theta": np.asarray(r["theta"]).tolist()}


def open_loop_errors(track: Track, veh: DynVehicle, tau: float, run, stride: int = 5) -> np.ndarray:
    """(n_starts, N + 1) axle-point lateral errors of open-loop predictions of the filter's
    discrete model of ``veh`` with steering lag ``tau`` (A4-M1: ``FrenetMPC._step``, 50 ms
    blocks of the logged inputs), started at every ``stride``-th logged state (Frenet; the
    road-wheel angle from the logged commands through the lag); NaN where the log ends."""
    from ..estimation.identify import lag_filter

    tau = max(tau, 1e-4)
    mpc = FrenetMPC(track, veh, ref_from_plan(plan(0.95)), mode="filter", N=N, dt=MPC_DT, steer_lag=tau)
    conv = MPCController(mpc, DT, tc_eta=None)
    step = int(round(MPC_DT / DT))
    Kn = len(run.t)
    dact = lag_filter(run.state[:, 6], DT, tau, a0=run.state[0, 6])
    n_ax = axle_n(track, veh, run.state)
    arms = np.array([veh.L - veh.lr, -veh.lr])
    starts = np.arange(0, Kn, stride)
    out = np.full((len(starts), N + 1), np.nan)
    for r_i, i in enumerate(starts):
        ya = np.array(run.state[i], float)
        ya[6] = dact[i]
        xf = np.concatenate([conv.frenet(ya), [run.state[i, 6]]])
        U = block_inputs(run.inputs, i, N)
        X = model_rollout(mpc, xf, U)
        out[r_i, 0] = 0.0
        for k in range(1, len(X)):
            jk = i + k * step
            if jk >= Kn:
                break
            na = X[k, 1] + arms * np.sin(X[k, 2])
            out[r_i, k] = float(np.abs(n_ax[jk] - na).max())
    return out


def open_loop_errors_sim(track: Track, veh: DynVehicle, tau: float, run, stride: int = 5) -> np.ndarray:
    """Superseded (A4-M1), kept only for the comparison reported in A4: the replay with the
    simulator's 10 ms integration."""
    from ..estimation.identify import lag_filter
    from ..sim.lagged import LaggedSim

    sim = LaggedSim(track, veh, max(tau, 1e-4), DT, observe="command")
    step = int(round(MPC_DT / DT))
    Kn = len(run.t)
    dact = lag_filter(run.state[:, 6], DT, max(tau, 1e-4), a0=run.state[0, 6])
    n_ax = axle_n(track, veh, run.state)
    arms = np.array([veh.L - veh.lr, -veh.lr])
    starts = np.arange(0, Kn, stride)
    out = np.full((len(starts), N + 1), np.nan)
    for r_i, i in enumerate(starts):
        x = np.concatenate([run.state[i, :6], [dact[i], run.state[i, 6]]])
        out[r_i, 0] = 0.0
        for k in range(1, N + 1):
            for j in range(i + (k - 1) * step, i + k * step):
                if j >= Kn:
                    break
                x = sim.step(x, run.inputs[j])
            jk = i + k * step
            if jk >= Kn:
                break
            c, s_ = math.cos(x[2]), math.sin(x[2])
            _, n = track.project(x[0] + arms * c, x[1] + arms * s_)
            out[r_i, k] = float(np.abs(n_ax[jk] - n).max())
    return out


def estimate_a(track: Track, run) -> dict:
    """(a): identify on the first two thirds of the exploration lap, the 95 % quantile per k
    of the open-loop axle errors on the last third, non-decreasing in k."""
    t_cut = run.t[0] + 2.0 / 3.0 * (run.t[-1] - run.t[0])
    first = cut(run, run.t[0], t_cut)
    first.noisy = getattr(run, "noisy", False)
    ident = identify_case(first)
    last = cut(run, t_cut, run.t[-1] + 1.0)
    E = open_loop_errors(track, ident["vehicle"], ident["tau"], last)
    b = np.zeros(N + 1)
    for k in range(1, N + 1):
        v = E[:, k][np.isfinite(E[:, k])]
        b[k] = float(np.quantile(v, 0.95)) if len(v) else np.nan
    b[1:] = np.maximum.accumulate(np.nan_to_num(b[1:], nan=0.0))
    return {"vehicle": ident["vehicle"], "tau": ident["tau"], "theta": ident["theta"], "backoff": b.tolist(),
            "starts": int(E.shape[0])}


# ---------------------------------------------------------------- (c) conformal residual (A3.5)
def residual_for(run, model: DynVehicle):
    """The L6 residual model (unchanged) on the whole exploration lap, targets relative to
    ``model`` (the filter's model)."""
    from ..estimation.identify import Data, data_from_run
    from ..estimation.residual import ResidualModel, inputs_of, targets_of

    d = data_from_run(run, DT)
    d5 = Data(d.state[::5], d.inputs[::5], d.accel[::5])
    return ResidualModel().fit(inputs_of(d5), targets_of(d5, model))


def sigma_k(res_model, X: np.ndarray, U: np.ndarray) -> np.ndarray:
    """sigma(k), k = 0..N, of the lateral position along a prediction (X: N + 1 states, U: N
    inputs, forces in N): sum_{j<k} (k - j - 1/2) h^2 sigma_vy,j (CautiousMPCController, kappa 1)."""
    Z = np.column_stack([X[:N, 3:7], U])
    sig = res_model.predict(Z)[1][:, 1]
    h = MPC_DT
    out = np.zeros(N + 1)
    for k in range(1, N + 1):
        j = np.arange(k)
        out[k] = float(np.sum((k - j - 0.5) * h * h * sig[:k]))
    return out


def conformal_q(ratios: np.ndarray, level: float = 0.95) -> float:
    """The ceil(level (n + 1))-th smallest of the n ratios |error| / sigma (split conformal);
    if ceil(level (n + 1)) > n the quantile is infinite."""
    r = np.sort(np.asarray(ratios, float))
    n = len(r)
    i = int(math.ceil(level * (n + 1)))
    if n == 0 or i > n:
        return float("inf")
    return float(r[i - 1])


def conformal_qk(k: np.ndarray, ratios: np.ndarray, level: float = 0.95) -> np.ndarray:
    """q-hat per stage (A3.5 after the review of 14e1fda): q_k from the ratios of stage k only;
    q_0 = 0."""
    q = np.zeros(N + 1)
    for kk in range(1, N + 1):
        q[kk] = conformal_q(ratios[k == kk], level)
    return q


def band_c(res_model, q, eta0: float = 0.95):
    """(c) in use: eta_k = eta0, b_k = q_k sigma(k) (q: N + 1 values) along the previous
    prediction shifted by one step (the constant initial guess for the first solve), as
    CautiousMPCController."""
    def f(filt: NMPCFilter, xf: np.ndarray):
        mpc = filt.mpc
        if mpc.z is None:
            X = np.tile(xf, (N + 1, 1))
            U = np.tile(filt.u_prev, (N, 1))
        else:
            Xp, Up = mpc.z[mpc.Xi], mpc.z[mpc.Ui] * np.array([1.0, FS, FS])
            X = np.vstack([Xp[1:], Xp[-1:]])
            U = np.vstack([Up[1:], Up[-1:]])
        return np.full(N, eta0), np.asarray(q, float) * sigma_k(res_model, X[:, :7], U)
    return f


__all__ = ["cases", "SETS", "run_lap", "NMPCFilter", "CBFFilter", "explore_lap", "prediction_samples",
           "horizon_ok", "rollout_errors", "env_snapshot", "env_restore", "rollout_unfiltered",
           "make_sim_for", "drive_baseline", "rollout_errors_sim", "block_inputs", "model_rollout", "estimate_a", "residual_for", "sigma_k", "conformal_q", "conformal_qk", "band_c", "changed"]
