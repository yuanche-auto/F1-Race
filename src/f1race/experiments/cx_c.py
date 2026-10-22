"""Control expansion, phase C (docs/CX_C_protocol.md, revision 0 with its clarifications): the sets, the tuning grid,
the controllers and one lap with every metric of protocol section 5.

* Sets (section 2): ``CXV`` – phase A's tuning set (``cx_a.cases("CXV")``); ``CCT`` – ``sample_plants(nominal, 50,
  seed 322)[1:]`` (test); ``CXDEV`` – ``sample_plants(nominal, 5, seed 399)[1:]`` (development checks only); ``NOM``;
  secondary ``CCT_LAG30`` (the CCT cars with the L6 lag of 30 ms), ``CXD`` and ``CXL8`` (``cx_a.cases``).
* Controllers (section 3): ``tracker`` and ``nmpc`` through ``cx_b.lap`` (phase A's ``cx_a.lap`` plus the overlap) – so that
  ``nmpc``'s records on CXD / CXL8 are produced by the code that produced phase B's (the reproducibility check);
  ``nmpc_reserve`` (``nmpc``'s objective on the 12 % plan, eta 0.88), ``tube`` and ``tube_track`` (``control/tube.py``)
  through ``mpc_lap``. All start at the state of the 5 % plan at s = 0 and are measured against the 5 % plan.
* Settings: ``tube`` / ``tube_track`` – ``c_R`` (``TUBE_GRID``).
"""
from __future__ import annotations

import math
import time

import numpy as np

from ..dynamics import DynVehicle
from . import cx_a, cx_b
from .l2 import PERTURBATION, ref_from_plan, sample_plants, x0_from_plan

SETS = {"CCT": (322, 50), "CXDEV": (399, 5)}
SECONDARY = ("CCT_LAG30", "CXD", "CXL8")
ETA, DT, T_MAX, MPC_DT, N = cx_a.ETA, cx_a.DT, cx_a.T_MAX, cx_a.MPC_DT, cx_a.N
NMPC_SETTING = cx_b.NMPC_SETTING                         # c 2, margin 0.6 (phase A, CXV choice)
ETA_RESERVE = 0.88                                       # nmpc_reserve: the 12 % plan and eta 0.88 in the MPC
ETA_MIN = 0.70
TUBE_GRID = [{"c_R": c} for c in (0.1, 1.0, 10.0)]
BASELINES = ("tracker", "nmpc")
TUBE_KINDS = ("tube", "tube_track")
KINDS = ("tracker", "nmpc", "nmpc_reserve", "tube", "tube_track")      # the secondary conditions run these five
# amendment C1 (review of packet 3): nmpc_param - the tube's code path with no tightening (null), c_R 1, untuned; on CCT
# and NOM only; run as the kind "tube" with this setting and relabelled by the runner (``PATH_KIND``)
NMPC_PARAM_SETTING = {"c_R": 1.0, "null": 1}
PATH_KIND = {"nmpc_param": "tube"}
TEST_KINDS = ("tracker", "nmpc", "nmpc_param", "nmpc_reserve", "tube", "tube_track")
COVER_STAGES = 10                                        # coverage over stages 1 ... 10 (section 5)


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


def fixed_setting(kind: str) -> dict:
    return {"tracker": {}, "nmpc": NMPC_SETTING, "nmpc_reserve": {**NMPC_SETTING, "eta": ETA_RESERVE},
            "nmpc_param": NMPC_PARAM_SETTING}[kind]


def cases(name: str) -> list[dict]:
    """The laps of a set; every case has ``track``, ``tau`` and ``dist_seed`` (as phases A / B)."""
    if name in ("NOM", "CXD", "CXL8", "CXV"):
        return cx_a.cases(name)
    lag = name == "CCT_LAG30"
    seed, n = SETS["CCT" if lag else name]
    cars = sample_plants(DynVehicle(), n, seed)
    return [{"track": "Spielberg", "tau": cx_a.LAG_TAU if lag else 0.0, "dist_seed": None, "set": name, "car": i,
             "info": cars[i][0], "plant": cars[i][1], "name": f"{name}_car{i}"} for i in range(1, n + 1)]


def build(kind: str, setting: dict, track, plans: dict):
    """(controller, mpc, tube or None, x0) for nmpc_reserve and the tube kinds; the model is the nominal car."""
    from ..control.nmpc import FrenetMPC, MPCController
    from ..control.tube import Tube, TubeMPCController
    from .l1 import dyn_reference

    model = DynVehicle()
    o = plans[0.95]
    path, _ = dyn_reference(track, model, o)
    x0 = x0_from_plan(o, path)
    w = {k: v * NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    margin = NMPC_SETTING["margin"]
    if kind == "nmpc_reserve":
        mpc = FrenetMPC(track, model, ref_from_plan(plans[ETA_RESERVE]), mode="track", N=N, dt=MPC_DT, eta=ETA_RESERVE,
                        margin=margin, weights=w)
        return MPCController(mpc, DT), mpc, None, x0
    if kind in TUBE_KINDS:
        mpc = FrenetMPC(track, model, ref_from_plan(o), mode="track", N=N, dt=MPC_DT, eta=ETA, margin=margin, weights=w,
                        param_tightening=True)
        tube = Tube(mpc, PERTURBATION, margin=margin, eta=ETA, c_R=float(setting["c_R"]), eta_min=ETA_MIN,
                    track_only=kind == "tube_track", null=bool(setting.get("null", 0)))
        return TubeMPCController(mpc, tube, DT, keep=COVER_STAGES), mpc, tube, x0
    raise ValueError(kind)


def _pct(a, qs=(5, 50, 95)) -> list[float]:
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return [float(np.percentile(a, q)) for q in qs] if a.size else [math.nan] * len(qs)


def coverage(track, res, tube_log: list[dict], arms) -> dict:
    """Section 5: for every solve and stage j = 1 ... COVER_STAGES, the realised lateral error of the axle points between
    the measured state at t + j h and the solve's prediction, against the stage's back-off b_j (the realised error is the
    re-solving NMPC's, the bound assumes K acting - a diagnostic of scale; clarification m1)."""
    X = res.state
    s_c, n_c = track.project(X[:, 0], X[:, 1])
    xi_c = np.angle(np.exp(1j * (X[:, 2] - track.heading_at(s_c))))
    t = res.t
    errs, bounds = [], []
    for rec in tube_log:
        for j in range(1, COVER_STAGES + 1):
            idx = int(round((rec["t"] + j * MPC_DT) / DT))
            if idx >= len(t):
                break
            xp = rec["X_pred"][j]
            e = max(abs((n_c[idx] + a * math.sin(xi_c[idx])) - (xp[1] + a * math.sin(xp[2]))) for a in arms)
            errs.append(e)
            bounds.append(float(rec["b"][j]))
    errs, bounds = np.array(errs), np.array(bounds)
    if not errs.size:
        return {"coverage_exceed_share": math.nan, "coverage_err_over_bound_p50_p95": [math.nan, math.nan], "coverage_n": 0}
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = errs / np.maximum(bounds, 1e-9)
    return {"coverage_exceed_share": float(np.mean(errs > bounds)), "coverage_err_over_bound_p50_p95":
            [float(np.percentile(ratio, 50)), float(np.percentile(ratio, 95))], "coverage_n": int(errs.size),
            "realised_lateral_error_p50_p95_max_m": [float(np.percentile(errs, 50)), float(np.percentile(errs, 95)),
                                                     float(errs.max())]}


def mpc_lap(case: dict, kind: str, setting: dict) -> dict:
    """One lap of nmpc_reserve / tube / tube_track with the metrics of section 5 (trajectory metrics as ``cx_a.lap``,
    against the 5 % plan)."""
    from ..track import Track
    from .l7 import make_sim_for
    from .l8 import plans_for

    t0 = time.time()
    tname = case.get("track", "Spielberg")
    track, plans = Track.load(tname), plans_for(tname)
    o = plans[0.95]
    t_max = T_MAX if tname == "Spielberg" else 1.6 * float(o.lap_time)
    ctl, mpc, tube, x0 = build(kind, setting, track, plans)
    sim = make_sim_for({"plant": case["plant"], "tau": case.get("tau", 0.0), "dist_seed": case.get("dist_seed")}, track)
    res = sim.run(ctl, x0, t_max=t_max)
    X, U = res.state, res.inputs
    L = track.length
    s_c, n_c = track.project(X[:, 0], X[:, 1])
    n_plan = np.interp(s_c, np.asarray(o.s) % L, np.asarray(o.n), period=L)
    v_plan = cx_a._vref_fn(o, L)(s_c)
    f_tot = U[:, 1] + U[:, 2]
    st = mpc.stats
    ts = np.array([s_["time"] for s_ in st]) * 1e3
    ok = ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    out = {
        "kind": kind, "setting": setting, "setting_key": setting_key(setting), "replicate": None,
        "case": case["name"], "set": case["set"], "car": case["car"], "plant": case["info"], "track": tname,
        "tau": case.get("tau", 0.0), "dist_seed": case.get("dist_seed"), "t_max": t_max,
        "completed": bool(res.completed), "lap_time": float(res.lap_time), "stop_reason": res.stop_reason,
        "distance": float(res.progress[-1]), "failure_s": None if res.completed else float(s_c[-1]),
        "centre_offset_abs_mean_max": [float(np.mean(np.abs(n_c))), float(np.max(np.abs(n_c)))],
        "plan_line_offset_abs_mean_max": [float(np.mean(np.abs(n_c - n_plan))), float(np.max(np.abs(n_c - n_plan)))],
        "speed_over_plan_p5_p50_p95": cx_a._pct(X[:, 3] / np.maximum(v_plan, 1.0)),
        "min_edge_margin": float(np.min(res.margin)),
        "tyre_usage_max_front_rear": [float(np.max(res.usage[:, 0])), float(np.max(res.usage[:, 1]))],
        "tyre_usage_mean_front_rear": [float(np.mean(res.usage[:, 0])), float(np.mean(res.usage[:, 1]))],
        "steer_rate_rms": float(np.sqrt(np.mean(U[:, 0] ** 2))),
        "force_increment_rms_N": float(np.sqrt(np.mean(np.diff(f_tot) ** 2))) if len(f_tot) > 1 else 0.0,
        "force_increment_rms_50ms_N": float(np.sqrt(np.mean(np.diff(f_tot[::5]) ** 2))) if len(f_tot) > 5 else 0.0,
        "adaptation_data": "none",
        "solves": len(st),
        "solve_ms_mean_p95_p99_max": [float(ts.mean()), *(float(np.percentile(ts, q)) for q in (95, 99)), float(ts.max())],
        "solves_over_50ms": float(np.mean(ts > 50.0)),
        "not_converged": int(sum(s_["status"] not in ok for s_ in st)),
        "fallbacks": int(sum(bool(s_["fallback"]) for s_ in st)),
        "track_slack_max_mean": [float(max(s_["slack"] for s_ in st)), float(np.mean([s_["slack"] for s_ in st]))],
        "limit_slack_max": float(max(s_["limit_slack"] for s_ in st)),
        **cx_b.overlap_metrics(U),
    }
    if tube is not None:
        lg = ctl.tube_log
        tt = np.array([r["tube_time"] for r in lg]) * 1e3
        b_all = np.concatenate([r["b_all"][1:] for r in lg])
        eta_all = np.concatenate([r["eta"] for r in lg])
        tot = tt + ts[: len(tt)]
        out.update({
            "tube_ms_mean_p95_p99_max": [float(tt.mean()), *(float(np.percentile(tt, q)) for q in (95, 99)), float(tt.max())],
            "step_ms_tube_plus_solve_mean_p95": [float(tot.mean()), float(np.percentile(tot, 95))],
            "backoff_m_p5_p50_p95": _pct(b_all), "backoff_stageN_m_p5_p50_p95": _pct([r["b_N"] for r in lg]),
            "eta_k_p5_p50_p95": _pct(eta_all), "eta_stageN_p5_p50_p95": _pct([r["eta"][-1] for r in lg]),
            "capped_track_share_mean": float(np.mean([r["capped_track"] for r in lg])),
            "capped_eta_share_mean": float(np.mean([r["capped_eta"] for r in lg])),
            "b_max": tube.b_max,
            **coverage(track, res, lg, (mpc.veh.lf, -mpc.veh.lr)),
        })
    out["wall_s"] = time.time() - t0
    return out


def lap(case: dict, kind: str, setting: dict, replicate: int = 0) -> dict:
    """One lap: tracker and nmpc through ``cx_b.lap`` (phase A / B's code path), the others through ``mpc_lap``."""
    if kind in BASELINES:
        return cx_b.lap(case, kind, setting)
    return mpc_lap(case, kind, setting)


def select(records: list[dict], kind: str) -> dict:
    """Protocol section 4: the most completed CXV laps; ties by the mean lap time on the cars every tied setting completes
    (skipped if none); then the smaller |log c_R|, then the smaller c_R. Different lap counts or cars stop it."""
    by: dict[str, dict] = {}
    for r in records:
        if r.get("kind") != kind or "error" in r:
            continue
        d = by.setdefault(r["setting_key"], {"setting": r["setting"], "done": {}})
        d["done"][r["case"]] = r["lap_time"] if r["completed"] else None
    if not by:
        raise ValueError(f"no records for {kind}")
    if len({len(d["done"]) for d in by.values()}) != 1:
        raise ValueError(f"{kind}: settings with different numbers of laps")
    if len({frozenset(d["done"]) for d in by.values()}) != 1:
        raise ValueError(f"{kind}: settings driven on different cars")
    table = {k: {"setting": d["setting"], "completed": sum(v is not None for v in d["done"].values()),
                 "laps": len(d["done"])} for k, d in by.items()}
    best = max(t_["completed"] for t_ in table.values())
    tied = [k for k, t_ in table.items() if t_["completed"] == best]
    if len(tied) > 1:
        common = set.intersection(*({c for c, v in by[k]["done"].items() if v is not None} for k in tied))
        if common:
            mean = {k: float(np.mean([by[k]["done"][c] for c in common])) for k in tied}
            for k in tied:
                table[k]["mean_lap_common"] = mean[k]
            tied = [k for k in tied if mean[k] == min(mean.values())]
    chosen = min(tied, key=lambda k: (abs(math.log(by[k]["setting"]["c_R"])), by[k]["setting"]["c_R"]))
    return {"setting": by[chosen]["setting"], "setting_key": chosen, "table": table}
