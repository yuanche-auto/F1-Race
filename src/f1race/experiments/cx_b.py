"""Control expansion, phase B (docs/CX_B_protocol.md, revision 0 with its clarifications): the sets, the tuning
grids, the MPPI controllers and one lap with every metric of protocol section 5.

* Sets (section 2): ``CXV`` – phase A's tuning set (``sample_plants(nominal, 10, seed 301)[1:]``, the same cases
  as ``cx_a.cases("CXV")``); ``CBT`` – ``sample_plants(nominal, 50, seed 312)[1:]`` (test); ``NOM`` – the nominal
  car (development; reported separately); secondary ``CBT_LAG30`` (the CBT cars with the L6 lag of 30 ms),
  ``CXD`` and ``CXL8`` (phase A's cases, ``cx_a.cases``).
* Controllers (section 3): the baselines ``tracker``, ``nmpc`` (phase A's CXV choice c 2, margin 0.6) and ``mpcc``
  (phase A's choice after amendment A2) run through ``cx_a.lap`` unchanged, so that their records are produced by
  phase A's code (the reproducibility check of section 2 on CXD / CXL8); ``mppi``, ``mppi_lp`` and ``mppi_big``
  (``control/mppi.py``) solve ``nmpc``'s optimal control problem (its weights scaled by c = 2, margin 0.6, eta 0.95,
  N = 30 x 50 ms) behind ``nmpc.MPCController``.
* Settings: ``mppi`` – ``lam`` and ``c_sigma`` (the noise scale on SIGMA_BASE), ``K``; ``mppi_lp`` – the same and
  ``f_c`` (Butterworth order 2); ``mppi_big`` – ``mppi``'s with 4 K.
* Random numbers of an MPPI lap: ``default_rng((SET_CODE[set], car, replicate))`` (section 3.2).
"""
from __future__ import annotations

import math
import time

import numpy as np

from ..dynamics import DynVehicle
from . import cx_a
from .l2 import ref_from_plan, sample_plants, x0_from_plan

SETS = {"CXV": (301, 10), "CBT": (312, 50)}
SECONDARY = ("CBT_LAG30", "CXD", "CXL8")
SET_CODE = {"NOM": 0, "CXV": 1, "CBT": 2, "CBT_LAG30": 3, "CXD": 4, "CXL8": 5}
ETA, DT, T_MAX, MPC_DT, N = cx_a.ETA, cx_a.DT, cx_a.T_MAX, cx_a.MPC_DT, cx_a.N
BASELINES = ("tracker", "nmpc", "mpcc")
MPPI_KINDS = ("mppi", "mppi_lp", "mppi_big")
KINDS = BASELINES + MPPI_KINDS
NMPC_SETTING = {"c": 2.0, "margin": 0.6}                                     # phase A, CXV choice
MPCC_SETTING = {"q_c": 8.0, "q_v": 2.0, "margin": 0.3, "soft_bound": 1}      # phase A, CXV choice after A2
SIGMA_BASE = (0.03, 2000.0)                                                  # rad/s, N (section 4)
MPPI_GRID = [{"lam": lam, "c_sigma": c} for lam in (0.1, 1.0, 10.0) for c in (0.5, 1.0, 2.0)]
LP_FC, LP_ORDER = (1.0, 2.0, 4.0), 2
K_CANDIDATES, K_BUDGET_MS = (256, 512, 1024, 2048, 4096), 40.0              # section 3.4
K_RT: int | None = None                         # fixed by the timing of section 9.5 (packet 2), then a constant
GRID_CENTRE = {"lam": 1.0, "c_sigma": 1.0}


def baseline_setting(kind: str) -> dict:
    return {"tracker": {}, "nmpc": NMPC_SETTING, "mpcc": MPCC_SETTING}[kind]


def lp_grid(base: dict) -> list[dict]:
    """mppi_lp's CXV grid on mppi's chosen (lam, c_sigma, K)."""
    return [{**base, "f_c": fc} for fc in LP_FC]


def big_setting(base: dict) -> dict:
    return {**base, "K": 4 * int(base["K"])}


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


def cases(name: str) -> list[dict]:
    """The laps of a set; every case has ``track``, ``tau`` and ``dist_seed`` (as phase A's)."""
    if name in ("NOM", "CXD", "CXL8", "CXV"):
        return cx_a.cases(name)
    lag = name == "CBT_LAG30"
    seed, n = SETS["CBT"]
    cars = sample_plants(DynVehicle(), n, seed)
    return [{"track": "Spielberg", "tau": cx_a.LAG_TAU if lag else 0.0, "dist_seed": None, "set": name, "car": i,
             "info": cars[i][0], "plant": cars[i][1], "name": f"{name}_car{i}"} for i in range(1, n + 1)]


def rng_seed(case: dict, replicate: int) -> tuple[int, int, int]:
    """(set code, car index, replicate); car index = the disturbance seed on CXD, the track index on CXL8."""
    if case["set"] == "CXD":
        car = int(case["dist_seed"])
    elif case["set"] == "CXL8":
        from .l8 import TEST

        car = list(TEST).index(case["track"])
    else:
        car = int(case["car"])
    return (SET_CODE[case["set"]], car, int(replicate))


def build_mppi(kind: str, setting: dict, track, o, seed, diagnostics: bool = True):
    """(controller, mppi, x0); the model is the nominal car, the problem is nmpc's (NMPC_SETTING)."""
    from ..control.mppi import MPPI
    from ..control.nmpc import MPCController
    from .l1 import dyn_reference

    if kind not in MPPI_KINDS:
        raise ValueError(kind)
    model = DynVehicle()
    path, _ = dyn_reference(track, model, o)
    w = {k: v * NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    lowpass = (float(setting["f_c"]), LP_ORDER) if kind == "mppi_lp" else None
    mp = MPPI(track, model, ref_from_plan(o), N=N, dt=MPC_DT, eta=ETA, margin=NMPC_SETTING["margin"], weights=w,
              K=int(setting["K"]), lam=float(setting["lam"]),
              sigma=tuple(float(setting["c_sigma"]) * s for s in SIGMA_BASE), seed=seed, lowpass=lowpass,
              diagnostics=diagnostics)
    return MPCController(mp, DT), mp, x0_from_plan(o, path)


def _pct(a, qs=(5, 50, 95)) -> list[float]:
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return [float(np.percentile(a, q)) for q in qs] if a.size else [math.nan] * len(qs)


def mppi_lap(case: dict, kind: str, setting: dict, replicate: int = 0) -> dict:
    """One MPPI lap with the metrics of protocol section 5 (the trajectory metrics computed as ``cx_a.lap``)."""
    from ..track import Track
    from .l7 import make_sim_for

    t0 = time.time()
    tname = case.get("track", "Spielberg")
    track, o = Track.load(tname), cx_a.plan(tname)
    t_max = T_MAX if tname == "Spielberg" else 1.6 * float(o.lap_time)
    seed = rng_seed(case, replicate)
    ctl, mp, x0 = build_mppi(kind, setting, track, o, seed)
    sim = make_sim_for({"plant": case["plant"], "tau": case.get("tau", 0.0),
                        "dist_seed": case.get("dist_seed")}, track)
    res = sim.run(ctl, x0, t_max=t_max)
    X, U = res.state, res.inputs
    L = track.length
    s_c, n_c = track.project(X[:, 0], X[:, 1])
    n_plan = np.interp(s_c, np.asarray(o.s) % L, np.asarray(o.n), period=L)
    v_plan = cx_a._vref_fn(o, L)(s_c)
    f_tot = U[:, 1] + U[:, 2]
    st = mp.stats
    steps = st[1:]                                     # the first call (20 iterations) is reported apart
    ts = np.array([s_["time"] for s_ in steps]) * 1e3
    ratio = [s_["cost_after"] / s_["cost_before"] for s_ in steps
             if np.isfinite(s_.get("cost_before", np.nan)) and s_.get("cost_before", 0) > 0]
    return {
        "kind": kind, "setting": setting, "setting_key": setting_key(setting), "replicate": int(replicate),
        "rng_seed": list(seed), "case": case["name"], "set": case["set"], "car": case["car"], "plant": case["info"],
        "track": tname, "tau": case.get("tau", 0.0), "dist_seed": case.get("dist_seed"), "t_max": t_max,
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
        "first_call_ms_iters": [float(st[0]["time"] * 1e3), int(st[0]["iters"])] if st else None,
        "first_call_no_valid_iterations": int(st[0].get("no_valid_iterations", 0)) if st else None,
        "step_ms_mean_p50_p95_p99_max": ([float(ts.mean()), *(float(np.percentile(ts, q)) for q in (50, 95, 99)),
                                      float(ts.max())] if ts.size else None),
        "steps_over_50ms": float(np.mean(ts > 50.0)) if ts.size else math.nan,
        "no_valid_steps": int(sum(s_["no_valid"] for s_ in st)),
        "no_penalty_free_sample_share": float(np.mean([s_["no_free"] for s_ in st])) if st else math.nan,
        "best_sample_penalised_share": float(np.mean([s_["best_penalised"] for s_ in st])) if st else math.nan,
        "ess_p5_p50_p95": _pct([s_["ess"] for s_ in st]),
        "penalised_sample_share_mean": float(np.nanmean([s_["penalised_share"] for s_ in st])) if st else math.nan,
        "invalid_samples_total_max": [int(sum(s_["invalid"] for s_ in st)), int(max((s_["invalid"] for s_ in st), default=0))],
        "cost_after_over_before_p5_p50_p95": _pct(ratio),
        "cost_after_worse_share": float(np.mean(np.asarray(ratio) > 1.0)) if ratio else math.nan,
        "track_slack_max_mean": [float(np.nanmax([s_["slack"] for s_ in st])), float(np.nanmean([s_["slack"] for s_ in st]))],
        "limit_slack_max": float(np.nanmax([s_["limit_slack"] for s_ in st])),
        "wall_s": time.time() - t0,
    }


def lap(case: dict, kind: str, setting: dict, replicate: int = 0) -> dict:
    """One lap: the baselines through phase A's ``cx_a.lap`` (unchanged), the MPPIs through ``mppi_lap``."""
    if kind in BASELINES:
        rec = cx_a.lap(case, kind, setting)
        rec["replicate"] = None
        return rec
    return mppi_lap(case, kind, setting, replicate)
