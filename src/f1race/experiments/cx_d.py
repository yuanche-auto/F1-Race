"""Control expansion, phase D (docs/CX_D_protocol.md, revision 0 with its clarifications): the sets, the friction
drift, the plan library, the controllers and one lap with every metric of protocol section 5.

* Sets (section 2): ``CDV`` = ``CDV_C`` (phase A's CXV cars, constant) + ``CDV_D`` (the same cars with a drift, seed
  334) – tuning; ``CDT`` – ``sample_plants(nominal, 50, seed 332)[1:]`` (test, constant); ``CDD`` – the CDT cars with
  a drift each (seed 333; test); ``DEV_C`` / ``DEV_D`` – phase C's CXDEV cars (seed 399), constant / with drifts (seed
  335; development checks only); ``NOM``; secondary ``CDT_NOISE`` (the L3 measurement noise only, noise seed
  332 000 + car), ``CDT_LAG30``, ``CDT_WET85`` (mu x 0.85), ``CDP`` (nominal car, L8's wet patches 0-29), ``CXD``
  (phases A-C's).
* Friction drift: f(s) = 1 before s_on, linear to f_end over l, f_end after; s_on ~ U(600, 2600) m, l ~ U(100, 1500)
  m, f_end ~ U(0.75, 0.90), drawn (s_on, l, f_end) per car in car order; on both axles' mu_x, mu_y at the car's
  position (``StochasticSim`` with a friction field, no gusts, no noise).
* Plan library: L1's plans at eta_p 1.00 / 0.97 / 0.95 / 0.93, L2's 0.88 and the new ones of
  ``scripts/cx_d_plans.py``, ascending in eta_p.
* Controllers (section 3): ``nmpc`` (phase A's build, c 2, margin 0.6), ``tube`` (phase C's build, c_R 1), the
  adaptive NMPCs on ``FrenetMPC(param_model=True)`` with the library – ``adaptive`` (RLS, setting ``lam_mu``),
  ``adaptive_bank`` (model bank, setting ``W``), ``adaptive_oracle``, ``nmpc_param`` (the RLS runs at ``lam_mu``,
  the nominal car and the 5 % plan are handed to the NLP) – and ``identified`` (L8's pipeline with this phase's
  objective; its exploration lap, identification and plan are computed once per car and cached). All start at the
  state of the 5 % plan at s = 0 and are measured against the 5 % plan; one lap, 121 s.
"""
from __future__ import annotations

import math
import os
import pickle
import time
from pathlib import Path

import numpy as np

from ..dynamics import DynVehicle
from . import cx_a, cx_b, cx_c
from .l2 import ref_from_plan, sample_plants, x0_from_plan

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results" / "CX" / "D"
PLANS_PATH = OUT / "plans" / "plans_cxd.pkl"
ID_CACHE = OUT / "identified_cache"
ETA, DT, T_MAX, MPC_DT, N = cx_a.ETA, cx_a.DT, cx_a.T_MAX, cx_a.MPC_DT, cx_a.N
NMPC_SETTING = cx_b.NMPC_SETTING                           # c 2, margin 0.6
TUBE_SETTING = {"c_R": 1.0}                                # phase C's selection
CDT_SEED, N_CARS = 332, 50
DRIFT_SEEDS = {"CDD": 333, "CDV_D": 334, "DEV_D": 335}
NOISE_SEED_BASE = 332_000
WET85 = 0.85
S_ON, ELL, F_END = (600.0, 2600.0), (100.0, 1500.0), (0.75, 0.90)
ADAPTIVE_GRID = [{"lam_mu": v} for v in (0.99, 0.995, 0.999)]
BANK_GRID = [{"W": v} for v in (0.1, 0.2, 0.5)]
GRIDS = {"adaptive": ADAPTIVE_GRID, "adaptive_bank": BANK_GRID}
ADAPTIVE_KINDS = ("adaptive", "adaptive_bank", "adaptive_oracle", "nmpc_param")
KINDS = ("nmpc", "nmpc_param", "adaptive", "adaptive_bank", "adaptive_oracle", "tube", "identified")
SECONDARY = ("CDT_NOISE", "CDT_LAG30", "CDT_WET85", "CDP", "CXD")
SECONDARY_KINDS = ("nmpc", "tube", "adaptive", "adaptive_oracle")
SETTLE_TOL = 0.02                                          # |mu_hat - mu f| for the settling delay (section 5)


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


# ---------------------------------------------------------------- sets and conditions
class DriftField:
    """Friction factor along the lap: 1 before s_on, linear to f_end over ell, f_end afterwards."""

    def __init__(self, length: float, s_on: float, ell: float, f_end: float):
        self.length, self.s_on, self.ell, self.f_end = float(length), float(s_on), float(ell), float(f_end)

    def __call__(self, s):
        q = np.mod(np.asarray(s, float), self.length)
        w = np.clip((q - self.s_on) / self.ell, 0.0, 1.0)
        out = 1.0 - (1.0 - self.f_end) * w
        return float(out) if np.ndim(out) == 0 else out


def draw_drifts(n: int, seed: int) -> list[dict]:
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        s_on, ell, f_end = rng.uniform(*S_ON), rng.uniform(*ELL), rng.uniform(*F_END)
        out.append({"s_on": float(s_on), "ell": float(ell), "f_end": float(f_end)})
    return out


def _cars(seed: int, n: int) -> list[tuple[dict, DynVehicle]]:
    return sample_plants(DynVehicle(), n, seed)[1:]


def _case(set_: str, name: str, car: int, info: dict, plant: DynVehicle, car_key: str, **kw) -> dict:
    base = {"track": "Spielberg", "tau": 0.0, "dist_seed": None, "drift": None, "patch_seed": None, "noise_seed": None,
            "wet": None}
    return {**base, "set": set_, "name": name, "car": car, "info": info, "plant": plant, "car_key": car_key, **kw}


def cases(name: str) -> list[dict]:
    """The laps of a set (protocol section 2)."""
    from .l8 import wet

    if name == "CDV":
        return cases("CDV_C") + cases("CDV_D")
    if name == "CXD":
        return [{**c, "drift": None, "patch_seed": None, "noise_seed": None, "wet": None, "car_key": "nominal"}
                for c in cx_a.cases("CXD")]
    if name == "NOM":
        return [_case("NOM", "NOM_car0", 0, {"nominal": True}, DynVehicle(), "nominal")]
    if name == "CDP":
        return [_case("CDP", f"CDP_patch{k}", 0, {"nominal": True}, DynVehicle(), "nominal", patch_seed=k)
                for k in range(30)]
    src = {"CDV_C": ("CXV", 301, 10), "CDV_D": ("CXV", 301, 10), "DEV_C": ("CXDEV", 399, 5),
           "DEV_D": ("CXDEV", 399, 5)}.get(name, ("CDT", CDT_SEED, N_CARS))
    if name not in ("CDV_C", "CDV_D", "DEV_C", "DEV_D", "CDT", "CDD", "CDT_NOISE", "CDT_LAG30", "CDT_WET85"):
        raise KeyError(name)
    _, seed, n = src
    cars = _cars(seed, n)
    drifts = draw_drifts(n, DRIFT_SEEDS[name]) if name in DRIFT_SEEDS else [None] * n
    out = []
    for i, ((info, plant), dr) in enumerate(zip(cars, drifts), start=1):
        kw: dict = {"drift": dr}
        if name == "CDT_NOISE":
            kw["noise_seed"] = NOISE_SEED_BASE + i
        elif name == "CDT_LAG30":
            kw["tau"] = cx_a.LAG_TAU
        elif name == "CDT_WET85":
            kw["wet"], plant = WET85, wet(plant, WET85)
        out.append(_case(name, f"{name}_car{i}", i, info, plant, f"seed{seed}_car{i}", **kw))
    return out


def make_sim(case: dict, track):
    """The case's simulator (sections 2, 3): phases A-C's for CXD and the lag, a friction field for the drift and the
    patches, the measurement noise alone for CDT_NOISE, else the plain simulator (a wet level is in the plant)."""
    from ..sim.dynamic import DynamicSim
    from ..sim.stochastic import NOISE_SIGMA, Disturbances, StochasticSim
    from .l7 import make_sim_for
    from .l8 import PatchField

    if case["dist_seed"] is not None or case["tau"] > 0:
        return make_sim_for({"plant": case["plant"], "tau": case["tau"], "dist_seed": case["dist_seed"]}, track)
    if case["drift"] is not None:
        d = case["drift"]
        return StochasticSim(track, case["plant"], Disturbances(DriftField(track.length, d["s_on"], d["ell"], d["f_end"]),
                                                                None, None, 0), DT)
    if case["patch_seed"] is not None:
        return StochasticSim(track, case["plant"], Disturbances(PatchField(track.length, case["patch_seed"]), None, None,
                                                                0), DT)
    if case["noise_seed"] is not None:
        return StochasticSim(track, case["plant"], Disturbances(None, None, NOISE_SIGMA.copy(), case["noise_seed"]), DT)
    return DynamicSim(track, case["plant"], DT)


def true_theta(case: dict) -> np.ndarray:
    """The car's factors with a wet level in the friction factor (the field f(s) comes on top)."""
    from ..estimation.identify import theta_of

    th = theta_of({k: v for k, v in case["info"].items() if k != "nominal"})
    if case.get("wet"):
        th[0] *= case["wet"]
    return th


def friction_field(sim):
    dist = getattr(sim, "dist", None)
    return None if dist is None else dist.friction


# ---------------------------------------------------------------- plan library
def plan_library() -> tuple[np.ndarray, list]:
    """(eta_p ascending, plans): L1 / L2's and the new ones (``scripts/cx_d_plans.py``)."""
    from .l1 import code_fingerprint
    from .l8 import plans_for

    have = plans_for("Spielberg")
    with open(PLANS_PATH, "rb") as fh:
        d = pickle.load(fh)
    if d["code"] != code_fingerprint():
        raise RuntimeError("plans_cxd.pkl was made with other planning code; recompute")
    lib = {**{e: have[e] for e in (1.0, 0.97, 0.95, 0.93, 0.88)}, **d["plans"]}
    etas = sorted(lib)
    return np.array(etas, float), [lib[e] for e in etas]


# ---------------------------------------------------------------- controllers
def build(kind: str, setting: dict, track, case: dict, sim):
    """(controller, mpc, x0, extra) for every kind but ``identified`` (``identified_parts``)."""
    from ..control.adaptive_mpc import AdaptiveMPCController
    from ..control.nmpc import FrenetMPC
    from ..estimation.bank import ModelBank
    from ..estimation.rls import RecursiveLS
    from .l1 import dyn_reference
    from .l8 import plans_for

    plans = plans_for(case["track"])
    if kind == "nmpc":
        ctl, mpc, x0 = cx_a.build("nmpc", NMPC_SETTING, track, plans[ETA])
        return ctl, mpc, x0, {}
    if kind == "tube":
        ctl, mpc, tube, x0 = cx_c.build("tube", TUBE_SETTING, track, plans)
        return ctl, mpc, x0, {"tube": tube}
    if kind not in ADAPTIVE_KINDS:
        raise ValueError(kind)
    model = DynVehicle()
    etas, lib = plan_library()
    o = plans[ETA]
    path, _ = dyn_reference(track, model, o)
    x0 = x0_from_plan(o, path)
    w = {k: v * NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    mpc = FrenetMPC(track, model, [ref_from_plan(p) for p in lib], mode="track", N=N, dt=MPC_DT, eta=ETA,
                    margin=NMPC_SETTING["margin"], weights=w, param_model=True,
                    ref_grid=int(np.flatnonzero(np.isclose(etas, ETA))[0]))
    if kind == "adaptive_oracle":
        th0, field = true_theta(case), friction_field(sim)

        def oracle(s: float) -> np.ndarray:
            th = th0.copy()
            if field is not None:
                th[0] *= float(field(s))
            return th
        ctl = AdaptiveMPCController(mpc, DT, etas, source="oracle", oracle=oracle)
    elif kind == "adaptive_bank":
        ctl = AdaptiveMPCController(mpc, DT, etas, estimator=ModelBank(model, DT, window=float(setting["W"])))
    else:
        est = RecursiveLS(model, DT, lam_mu=float(setting["lam_mu"]))
        ctl = AdaptiveMPCController(mpc, DT, etas, estimator=est, source="null" if kind == "nmpc_param" else "estimate")
    return ctl, mpc, x0, {"etas": etas}


def identified_parts(case: dict, track) -> dict:
    """L8's ``identified`` steps up to the plan (section 3.7): the exploration lap under the car's initial condition
    (no drift), L4's least squares, ``replan_robust`` at 0.95 – cached per car (``car_key``; deterministic)."""
    from ..planning.replan import replan_robust
    from .l6 import replan_slot
    from .l8 import explore_lap, identify_run, plans_for

    p = ID_CACHE / f"{case['car_key']}.pkl"
    if p.exists():
        with open(p, "rb") as fh:
            return pickle.load(fh) | {"cached": True}
    plans = plans_for("Spielberg")
    ex_case = {"plant": case["plant"], "dist_seed": None, "patch_seed": None}
    ex = explore_lap(ex_case, track, plans, T_MAX)
    out: dict = {"explore": {"completed": ex.completed, "distance": ex.distance, "stop_reason": ex.stop_reason}}
    try:
        ident = identify_run(ex, lag=False)
    except Exception as e:                                          # noqa: BLE001 (L8's l8-M1)
        out.update({"failed": "identification failed", "identification_error": f"{type(e).__name__}: {e}"})
    else:
        out["identification"] = {k: v for k, v in ident.items() if k != "vehicle"}
        out["vehicle"] = ident["vehicle"]
        with replan_slot():
            o, log = replan_robust(track, ident["vehicle"], [("nominal plan", plans[0.95]), ("ideal plan", plans[1.0])],
                                   0.95, nominal=DynVehicle())
        out["plan_log"] = log
        if o is None:
            out["failed"] = "plan failed"
        else:
            out["plan"] = o
    ID_CACHE.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(f".{os.getpid()}.tmp")
    with open(tmp, "wb") as fh:
        pickle.dump(out, fh)
    os.replace(tmp, p)
    return out | {"cached": False}


# ---------------------------------------------------------------- one lap
def _pct(a, qs=(50, 95)) -> list[float]:
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return [float(np.percentile(a, q)) for q in qs] if a.size else [math.nan] * len(qs)


def adaptation_metrics(case: dict, ctl, sim, etas, track) -> dict:
    """Section 5 for the adaptive controllers: estimation error, friction tracking, plan too fast."""
    tr = np.array(ctl.trace, float)
    if not len(tr):
        return {}
    t, s, j, mu_plan, th = tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 3], tr[:, 4:11]
    field = friction_field(sim)
    th_true = true_theta(case)
    f = np.array([float(field(q)) for q in s]) if field is not None else np.ones(len(s))
    mu_true = th_true[0] * f
    err = th[:, 0] - mu_true
    ae = np.abs(err)
    eta_p = np.interp(j, np.arange(len(etas)), etas)
    out = {"mu_abs_err_p50_p95": _pct(ae), "mu_err_end": float(err[-1]),
           "factor_err_end": (th[-1] - np.r_[th_true[0] * f[-1], th_true[1:]]).tolist(),
           "plan_eta_p5_p50_p95": _pct(eta_p, (5, 50, 95)),
           "too_fast_share": float(np.mean(eta_p > ETA * mu_true + 0.01)),
           "mu_plan_err_p50_p95": _pct(np.abs(mu_plan - mu_true))}
    bad = np.flatnonzero(ae >= SETTLE_TOL)
    out["settle_from_start_s"] = 0.0 if not bad.size else (None if bad[-1] == len(t) - 1 else float(t[bad[-1] + 1]))
    d = case.get("drift")
    if d is not None:
        q = np.mod(s, track.length)
        on, end = q >= d["s_on"], q >= d["s_on"] + d["ell"]
        out["mu_abs_err_before_during_after_p50_p95"] = [_pct(ae[~on]), _pct(ae[on & ~end]), _pct(ae[end])]
        if on.any():
            i_on = int(np.flatnonzero(on)[0])
            after = ae[i_on:]
            b = np.flatnonzero(after >= SETTLE_TOL)
            out["drift_onset_t"] = float(t[i_on])
            out["drift_settle_delay_s"] = (0.0 if not b.size else
                                           (None if b[-1] == len(after) - 1 else float(t[i_on + b[-1] + 1] - t[i_on])))
            out["mu_abs_err_after_onset_max"] = float(after.max())
        else:
            out["drift_onset_t"] = out["drift_settle_delay_s"] = None
    if case.get("noise_seed") is not None:
        half = len(err) // 2
        out["mu_bias_last_half"] = float(np.mean(err[half:]))
    if ctl.est is not None:
        out["estimator"] = ctl.est.diagnostics()
    return out


def lap(case: dict, kind: str, setting: dict) -> dict:
    """One lap of ``kind`` with ``setting`` on ``case``; the metrics of section 5 (as phases A-C, against the 5 % plan,
    plus the adaptation metrics)."""
    from ..control.nmpc import FrenetMPC, MPCController
    from ..track import Track
    from .l1 import dyn_reference
    from .l8 import plans_for

    t0 = time.time()
    track = Track.load(case["track"])
    plans = plans_for(case["track"])
    o = plans[ETA]
    sim = make_sim(case, track)
    extra_rec: dict = {}
    if kind == "identified":
        parts = identified_parts(case, track)
        extra_rec["identified"] = {k: v for k, v in parts.items() if k not in ("vehicle", "plan")}
        if "failed" in parts:
            return {"kind": kind, "setting": setting, "setting_key": setting_key(setting), "case": case["name"],
                    "set": case["set"], "car": case["car"], "plant": case["info"], "completed": False,
                    "lap_time": float("nan"), "stop_reason": parts["failed"], "distance": 0.0, "failure_s": None,
                    **extra_rec, "wall_s": time.time() - t0}
        model, op = parts["vehicle"], parts["plan"]
        path, _ = dyn_reference(track, model, op)
        x0 = x0_from_plan(op, path)
        w = {k: v * NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
        mpc = FrenetMPC(track, model, ref_from_plan(op), mode="track", N=N, dt=MPC_DT, eta=ETA,
                        margin=NMPC_SETTING["margin"], weights=w)
        ctl, extra = MPCController(mpc, DT), {}
        extra_rec["plan_lap_time"] = float(op.lap_time)
    else:
        ctl, mpc, x0, extra = build(kind, setting, track, case, sim)
    res = sim.run(ctl, x0, t_max=T_MAX)
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
        "kind": kind, "setting": setting, "setting_key": setting_key(setting), "case": case["name"], "set": case["set"],
        "car": case["car"], "plant": case["info"], "track": case["track"], "tau": case["tau"],
        "dist_seed": case["dist_seed"], "drift": case["drift"], "patch_seed": case["patch_seed"],
        "noise_seed": case["noise_seed"], "wet": case["wet"], "t_max": T_MAX,
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
        "solves": len(st),
        "solve_ms_mean_p95_p99_max": [float(ts.mean()), *(float(np.percentile(ts, q)) for q in (95, 99)), float(ts.max())],
        "solves_over_50ms": float(np.mean(ts > 50.0)),
        "not_converged": int(sum(s_["status"] not in ok for s_ in st)),
        "fallbacks": int(sum(bool(s_["fallback"]) for s_ in st)),
        "track_slack_max_mean": [float(max(s_["slack"] for s_ in st)), float(np.mean([s_["slack"] for s_ in st]))],
        "limit_slack_max": float(max(s_["limit_slack"] for s_ in st)),
        **cx_b.overlap_metrics(U),
        **extra_rec,
    }
    if kind in ADAPTIVE_KINDS:
        out.update(adaptation_metrics(case, ctl, sim, extra["etas"], track))
    if case["drift"] is not None and not res.completed:
        out["failure_s_minus_onset"] = float(np.mod(s_c[-1], L) - case["drift"]["s_on"])
    out["wall_s"] = time.time() - t0
    return out


# ---------------------------------------------------------------- tuning rule
def select(records: list[dict], kind: str) -> dict:
    """Section 4: the most completed CDV laps; ties by the mean lap time on the cases every tied setting completes
    (skipped if none); then the middle setting of the grid, then the smaller value (the lowest and the highest setting
    are equally far from the middle - a tie the protocol leaves open; stated in section 9). Different lap counts or
    cases stop it."""
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
        raise ValueError(f"{kind}: settings driven on different cases")
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
    grid = [setting_key(g) for g in GRIDS[kind]]
    chosen = min(tied, key=lambda k: (abs(grid.index(k) - (len(grid) - 1) / 2), grid.index(k)))
    return {"setting": by[chosen]["setting"], "setting_key": chosen, "table": table}


def drift_summary(name: str = "CDD") -> dict:
    """Protocol section 9 item 7 (review m2): the drifts drawn for a set, before any lap."""
    cs = cases(name)
    d = np.array([[c["drift"]["s_on"], c["drift"]["ell"], c["drift"]["f_end"]] for c in cs])
    mu = np.array([true_theta(c)[0] for c in cs])
    end = d[:, 0] + d[:, 1]
    q = lambda a: [float(np.min(a)), float(np.percentile(a, 25)), float(np.median(a)), float(np.percentile(a, 75)),
                   float(np.max(a))]
    return {"set": name, "n": len(cs), "s_on_min_q1_med_q3_max": q(d[:, 0]), "ell_min_q1_med_q3_max": q(d[:, 1]),
            "f_end_min_q1_med_q3_max": q(d[:, 2]), "mu_f_end_min_q1_med_q3_max": q(mu * d[:, 2]),
            "cars_mu_f_end_below_0.70": int(np.sum(mu * d[:, 2] < 0.70)),
            "share_drift_ends_before_2700m": float(np.mean(end < 2700.0)),
            "share_drift_ends_before_3830m": float(np.mean(end < 3830.0)),
            "share_onset_before_2700m": float(np.mean(d[:, 0] < 2700.0))}
