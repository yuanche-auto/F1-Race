"""Control expansion, phase A (docs/CX_A_protocol.md, revision 1): the sets, the tuning grids, the
controllers and one lap with every metric of protocol section 5.

* Sets (section 2): ``CXV`` – ``sample_plants(nominal, 10, seed 301)[1:]`` (tuning), ``CXT`` –
  ``sample_plants(nominal, 25, seed 302)[1:]`` (test), ``NOM`` – the nominal car (development only; not
  part of a test). Spielberg, start at the state of the 5 % plan at s = 0, one lap, 121 s.
* Controllers (section 3): ``tracker`` (5 % plan + L1 dynamic tracker, L2's gains), ``nmpc``
  (``FrenetMPC`` tracking the 5 % plan; setting: scale ``c`` of the tracking weights and ``margin``),
  ``mpcc`` / ``mpcc_centre`` (``DynamicMPCC`` with the racing line / centre line as reference path and the
  speed bound at every stage; setting: ``q_c``, ``q_v``, ``margin``). Shared: nominal model, eta 0.95,
  N = 30 x 50 ms, IPOPT, traction control 0.99, no retries.
* ``mpcc_ca`` (amendment A1, secondary): ``mpcc`` with the CiMPCC curvature-weighted speed cost
  (``DynamicMPCC(curvature_speed=...)``); its base (q_c, q_v, margin) is ``mpcc``'s CXV choice, and only
  ``alpha`` and ``w_ca`` are tuned on CXV (``ca_grid``), after the main tuning.
* Amendment A2: the MPCCs' speed bound softened (``soft_bound`` = 1 in a setting; ``MPCC_GRID_A2``), re-tuned on
  CXV after the first tuning showed infeasible solves on cars that brake worse than the model.
* Secondary conditions (section 2, after the primary report): ``CXT_LAG30`` – the CXT cars with the L6
  steering lag tau = 30 ms (commanded angle observed, nominal-model controllers as L6's rows without a lag
  model); ``CXD`` – the nominal car under the L3 disturbances (seeds 7400–7429, with L3's measurement noise);
  ``CXL8`` – the nominal car on the four L8 test tracks with their L8 plans (time limit 1.6 x the plan, as
  L8). The simulator is L7's ``make_sim_for`` (as L7–L9).
* ``lap(case, kind, setting)``: the record of section 5. The MPCC's speed-bound activity: at stage 0 a
  *measured* quantity (the state of stage 0 is the measurement; the bound acts on stages 1 ... N) –
  vx >= v_ref(theta_0 - e_l,0) - 0.01 m/s – and over the predicted stages 1 ... N of every solve.
"""
from __future__ import annotations

import math
import time

import numpy as np

from ..dynamics import DynVehicle
from .l2 import PERTURBATION, ref_from_plan, sample_plants, x0_from_plan  # noqa: F401  (PERTURBATION: tests)

SETS = {"CXV": (301, 10), "CXT": (302, 25)}
SECONDARY = ("CXT_LAG30", "CXD", "CXL8")
LAG_TAU, DIST_SEEDS = 0.03, tuple(range(7400, 7430))
ETA, DT, T_MAX, MPC_DT, N = 0.95, 0.01, 121.0, 0.05, 30
KINDS = ("tracker", "nmpc", "mpcc", "mpcc_centre")
TRACKER_GAINS = {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0}          # L2Config.tracker
NMPC_TRACK_W = {"n": 1.0, "xi": 10.0, "v": 0.2}                                # FrenetMPC defaults (L2)
MPCC_GRID = [{"q_c": qc, "q_v": qv, "margin": m}
             for qc in (0.05, 0.5, 2.0, 8.0) for qv in (0.5, 1.0, 2.0) for m in (0.3, 0.6)]
NMPC_GRID = [{"c": c, "margin": m} for c in (0.5, 1.0, 2.0) for m in (0.3, 0.6)]
BOUND_TOL = 0.01                                                               # m/s, protocol section 5
CA_EXTRA = [{"alpha": a, "w_ca": w} for a in (1.0, 3.0) for w in (0.02, 0.1, 0.5)]   # amendment A1
MPCC_GRID_A2 = [{**g, "soft_bound": 1} for g in MPCC_GRID]       # amendment A2: the soft speed bound


def ca_grid(base: dict) -> list[dict]:
    """mpcc_ca's CXV grid: mpcc's chosen (q_c, q_v, margin) with each (alpha, w_ca) of CA_EXTRA."""
    keep = [k for k in ("q_c", "q_v", "margin", "soft_bound") if k in base]
    return [{**{k: base[k] for k in keep}, **e} for e in CA_EXTRA]


def grid(kind: str) -> list[dict]:
    return {"nmpc": NMPC_GRID, "mpcc": MPCC_GRID, "mpcc_centre": MPCC_GRID, "tracker": [{}]}[kind]


def setting_key(setting: dict) -> str:
    """``k=v`` joined by commas (numbers as ``:g``; strings and lists – phase H's development options – as text)."""
    def fmt(v) -> str:
        if isinstance(v, (list, tuple)):
            return "/".join(fmt(x) for x in v)
        return f"{v:g}" if isinstance(v, (int, float)) and not isinstance(v, bool) else str(v)
    return ",".join(f"{k}={fmt(setting[k])}" for k in sorted(setting)) or "default"


def cases(name: str) -> list[dict]:
    """The laps of a set; every case has ``track`` (Spielberg unless CXL8), ``tau`` (0 unless CXT_LAG30) and
    ``dist_seed`` (None unless CXD)."""
    base = {"track": "Spielberg", "tau": 0.0, "dist_seed": None}
    nom = {"info": {"nominal": True}, "plant": DynVehicle()}
    if name == "NOM":
        return [{**base, **nom, "set": "NOM", "car": 0, "name": "NOM_car0"}]
    if name == "CXD":
        return [{**base, **nom, "set": name, "car": 0, "dist_seed": d, "name": f"CXD_dist{d}"} for d in DIST_SEEDS]
    if name == "CXL8":
        from .l8 import TEST

        return [{**base, **nom, "set": name, "car": 0, "track": t, "name": f"CXL8_{t}"} for t in TEST]
    lag = name == "CXT_LAG30"
    seed, n = SETS["CXT" if lag else name]
    cars = sample_plants(DynVehicle(), n, seed)
    return [{**base, "set": name, "car": i, "info": cars[i][0], "plant": cars[i][1], "tau": LAG_TAU if lag else 0.0,
             "name": f"{name}_car{i}"} for i in range(1, n + 1)]


def plan(track: str = "Spielberg"):
    from .l8 import plans_for

    return plans_for(track)[ETA]


def build(kind: str, setting: dict, track, o):
    """(controller, mpc or None, x0) for one lap; the model is always the nominal car."""
    from ..control.dyn_tracking import DynTracker
    from ..control.mpcc import DynamicMPCC
    from ..control.nmpc import FrenetMPC, MPCController
    from .l1 import dyn_reference

    model = DynVehicle()
    path, ff = dyn_reference(track, model, o)
    x0 = x0_from_plan(o, path)
    if kind == "tracker":
        return DynTracker(path, ff, model, DT, **TRACKER_GAINS), None, x0
    if kind == "nmpc":
        w = {k: v * setting["c"] for k, v in NMPC_TRACK_W.items()}
        mpc = FrenetMPC(track, model, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=ETA,
                        margin=setting["margin"], weights=w)
        return MPCController(mpc, DT), mpc, x0
    if kind in ("mpcc", "mpcc_centre", "mpcc_ca"):
        cs = {"alpha": setting["alpha"], "w_ca": setting["w_ca"]} if kind == "mpcc_ca" else None
        mpc = DynamicMPCC(track, model, ref_from_plan(o), N=N, dt=MPC_DT, eta=ETA, margin=setting["margin"],
                          weights={"q_c": setting["q_c"], "q_v": setting["q_v"]},
                          reference="centre" if kind == "mpcc_centre" else "raceline", speed_bound="stages",
                          curvature_speed=cs, speed_bound_soft=bool(setting.get("soft_bound", 0)))
        return BoundRecorder(mpc, DT, o), mpc, x0
    raise ValueError(kind)


def _vref_fn(o, length: float):
    s, v = np.asarray(o.s, float) % length, np.asarray(o.vx, float)
    return lambda q: np.interp(np.mod(q, length), s, v, period=length)


class BoundRecorder:
    """``MPCCController`` that also records, at every solve, the speed bound's activity (protocol
    section 5): at stage 0 from the measurement, over stages 1 ... N from the prediction. A fallback
    solve's "prediction" is the shifted initial guess, not a solution: it is left out of the stage
    statistic and counted (review n1 of 6bcc6ef); the measured stage-0 entry is kept for every solve."""

    def __init__(self, mpc, sim_dt: float, o):
        from ..control.mpcc import MPCCController

        self.ctl = MPCCController(mpc, sim_dt)
        self.mpc = mpc
        self.vref = _vref_fn(o, mpc.track.length)
        self._el = mpc.errors_fn.map(mpc.N + 1)
        self.active0: list[bool] = []
        self.active_stages: list[float] = []
        self.fallback_solves = 0

    @property
    def log(self):
        return self.ctl.log

    def __call__(self, t: float, x: np.ndarray):
        n0 = len(self.mpc.stats)
        u = self.ctl(t, x)
        if len(self.mpc.stats) > n0:
            X, _ = self.mpc.prediction()
            el = np.asarray(self._el(X[:, 0], X[:, 1], X[:, 7])[1]).ravel()
            th0 = self.ctl.theta
            el0 = float(self.mpc.errors_fn(x[0], x[1], th0)[1])
            self.active0.append(bool(x[3] >= self.vref(th0 - el0) - BOUND_TOL))
            if self.mpc.stats[-1]["fallback"]:
                self.fallback_solves += 1
            else:
                act = X[1:, 3] >= self.vref(X[1:, 7] - el[1:]) - BOUND_TOL
                self.active_stages.append(float(np.mean(act)))
        return u


def _pct(a, qs=(5, 50, 95)) -> list[float]:
    a = np.asarray(a, float)
    return [float(np.percentile(a, q)) for q in qs] if a.size else [math.nan] * len(qs)


def lap(case: dict, kind: str, setting: dict) -> dict:
    """One lap of ``kind`` with ``setting`` on ``case``; the metrics of protocol section 5."""
    from ..track import Track
    from .l7 import make_sim_for

    t0 = time.time()
    tname = case.get("track", "Spielberg")
    track, o = Track.load(tname), plan(tname)
    t_max = T_MAX if tname == "Spielberg" else 1.6 * float(o.lap_time)       # L8's rule elsewhere
    ctl, mpc, x0 = build(kind, setting, track, o)
    sim = make_sim_for({"plant": case["plant"], "tau": case.get("tau", 0.0),
                        "dist_seed": case.get("dist_seed")}, track)          # DynamicSim, LaggedSim or StochasticSim
    res = sim.run(ctl, x0, t_max=t_max)
    X, U = res.state, res.inputs
    L = track.length
    s_c, n_c = track.project(X[:, 0], X[:, 1])
    n_plan = np.interp(s_c, np.asarray(o.s) % L, np.asarray(o.n), period=L)
    v_plan = _vref_fn(o, L)(s_c)
    f_tot = U[:, 1] + U[:, 2]
    out = {
        "kind": kind, "setting": setting, "setting_key": setting_key(setting), "case": case["name"],
        "set": case["set"], "car": case["car"], "plant": case["info"], "track": tname,
        "tau": case.get("tau", 0.0), "dist_seed": case.get("dist_seed"), "t_max": t_max,
        "completed": bool(res.completed), "lap_time": float(res.lap_time), "stop_reason": res.stop_reason,
        "distance": float(res.progress[-1]),
        "failure_s": None if res.completed else float(s_c[-1]),
        "centre_offset_abs_mean_max": [float(np.mean(np.abs(n_c))), float(np.max(np.abs(n_c)))],
        "plan_line_offset_abs_mean_max": [float(np.mean(np.abs(n_c - n_plan))), float(np.max(np.abs(n_c - n_plan)))],
        "speed_over_plan_p5_p50_p95": _pct(X[:, 3] / np.maximum(v_plan, 1.0)),
        "min_edge_margin": float(np.min(res.margin)),
        "tyre_usage_max_front_rear": [float(np.max(res.usage[:, 0])), float(np.max(res.usage[:, 1]))],
        "tyre_usage_mean_front_rear": [float(np.mean(res.usage[:, 0])), float(np.mean(res.usage[:, 1]))],
        "steer_rate_rms": float(np.sqrt(np.mean(U[:, 0] ** 2))),
        # applied net longitudinal force (F_brake <= 0), differences over 10 ms (what the actuator sees, incl.
        # traction control) and over 50 ms (every controller sampled alike - an MPC changes its input every
        # 50 ms, the tracker every 10 ms, so the 10 ms value is not comparable across them; review m4)
        "force_increment_rms_N": float(np.sqrt(np.mean(np.diff(f_tot) ** 2))) if len(f_tot) > 1 else 0.0,
        "force_increment_rms_50ms_N": float(np.sqrt(np.mean(np.diff(f_tot[::5]) ** 2))) if len(f_tot) > 5 else 0.0,
        "adaptation_data": "none",
    }
    if mpc is not None:
        st = mpc.stats
        ts = np.array([s_["time"] for s_ in st]) * 1e3
        ok = ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        out.update({
            "solves": len(st),
            "solve_ms_mean_p95_p99_max": [float(ts.mean()), *(float(np.percentile(ts, q)) for q in (95, 99)),
                                          float(ts.max())],
            "solves_over_50ms": float(np.mean(ts > 50.0)),
            "not_converged": int(sum(s_["status"] not in ok for s_ in st)),
            "fallbacks": int(sum(bool(s_["fallback"]) for s_ in st)),
            "track_slack_max_mean": [float(max(s_["slack"] for s_ in st)), float(np.mean([s_["slack"] for s_ in st]))],
            "limit_slack_max": float(max(s_["limit_slack"] for s_ in st)),
        })
    if isinstance(ctl, BoundRecorder):
        out["bound_active_stage0_measured"] = float(np.mean(ctl.active0)) if ctl.active0 else math.nan
        out["bound_active_stages_1_N"] = float(np.mean(ctl.active_stages)) if ctl.active_stages else math.nan
        out["bound_stage_stats_fallbacks_excluded"] = ctl.fallback_solves
    out["wall_s"] = time.time() - t0
    return out


def select(records: list[dict], kind: str) -> dict:
    """The tuning rule of protocol section 4 on the CXV records of one controller: most completed laps;
    ties by the mean lap time on the laps every tied setting completes (skipped if they share none);
    then the setting closest to the defaults (mpcc: smaller q_c, then q_v nearest 1, then margin 0.3;
    nmpc: c nearest 1, then the smaller c, then margin 0.3; mpcc_ca (amendment A1): the smaller w_ca, then
    the smaller alpha). Returns {"setting", "table"}."""
    by: dict[str, dict] = {}
    for r in records:
        if r.get("kind") != kind or "error" in r:
            continue
        d = by.setdefault(r["setting_key"], {"setting": r["setting"], "done": {}})
        d["done"][r["case"]] = r["lap_time"] if r["completed"] else None
    if not by:
        raise ValueError(f"no records for {kind}")
    n_cases = {len(d["done"]) for d in by.values()}
    if len(n_cases) != 1:
        raise ValueError(f"{kind}: settings with different numbers of laps {sorted(n_cases)}")
    car_sets = {frozenset(d["done"]) for d in by.values()}
    if len(car_sets) != 1:                              # review n2: the same cars for every setting
        raise ValueError(f"{kind}: settings driven on different cars")
    table = {k: {"setting": d["setting"], "completed": sum(v is not None for v in d["done"].values()),
                 "laps": len(d["done"])} for k, d in by.items()}
    best = max(t["completed"] for t in table.values())
    tied = [k for k, t in table.items() if t["completed"] == best]
    if len(tied) > 1:
        common = set.intersection(*({c for c, v in by[k]["done"].items() if v is not None} for k in tied))
        if common:
            mean = {k: float(np.mean([by[k]["done"][c] for c in common])) for k in tied}
            for k in tied:
                table[k]["mean_lap_common"] = mean[k]
            m = min(mean.values())
            tied = [k for k in tied if mean[k] == m]

    def closeness(k):
        st = by[k]["setting"]
        if kind == "nmpc":
            return (abs(math.log(st["c"])), st["c"], st["margin"])
        if kind == "mpcc_ca":                         # closest to the cost being off: small w_ca, then alpha
            return (st["w_ca"], st["alpha"])
        return (st["q_c"], abs(math.log(st["q_v"])), st["q_v"], st["margin"])

    chosen = min(tied, key=closeness)
    return {"setting": by[chosen]["setting"], "setting_key": chosen, "table": table}
