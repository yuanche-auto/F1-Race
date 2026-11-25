"""Control expansion phase J – robust adaptive MPC (docs/CX_J_protocol.md, revision 0 with its clarifications): the sets,
the corrected friction drift, the controllers and one lap with every metric of protocol section 5 (phase D's lap with the
tube's additions).

* Sets (section 2): the cars of ``JT`` (``sample_plants(nominal, 50, seed 391)[1:]``) in four conditions ``JT`` (constant),
  ``JD`` (a friction drift each, seed 393), ``JN`` (the L3 measurement noise, noise seed 391 000 + car) and ``JL`` (the 30 ms
  steering lag); ``JV_C`` / ``JV_D`` / ``JV_N`` / ``JV_L`` – phase A's CXV cars (seed 301; drift seed 334, noise seed
  301 000 + car; tuning); ``DEV_*`` – phase C's CXDEV cars (seed 399; drift seed 335, noise 399 000 + car); ``NOM_*`` – the
  nominal car (drift s_on 1 600 m, ell 800 m, f_end 0.82; noise seed 399 900).
* The drift field ``StartFixedDrift``: phase D's with the lap's start (s = L - eps) counted as s = 0.
* Controllers (section 3): ``nmpc``, ``tube``, ``adaptive`` and ``adaptive_oracle`` through phase D's builder; ``ra`` /
  ``ra_fixed`` / ``ra_fixed_res`` / ``ra_shrink`` through ``control/radaptive.py`` (the RLS with lambda_mu 0.995, phase D's
  tuning; the settings ``rho0`` / ``rho_s``).
* ``lap(..., record=True)`` (development checks 9.2 and 9.4): the first-stage prediction of every solve, the whole
  prediction of every tenth solve and what the controller saw at every simulator step.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from ..dynamics import DynVehicle
from . import cx_a, cx_c, cx_d
from .l2 import ref_from_plan, x0_from_plan

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results" / "CX" / "J"
ETA, DT, MPC_DT, N = cx_d.ETA, cx_d.DT, cx_d.MPC_DT, cx_d.N
LAM_MU = 0.995                                          # phase D's choice (kept)
TUBE_SETTING = {"c_R": 1.0}                             # phase C's choice (kept)
JT_SEED, N_CARS, JD_DRIFT_SEED, JT_NOISE = 391, 50, 393, 391_000
JV_SEED, JV_N, JV_DRIFT_SEED, JV_NOISE = 301, 10, 334, 301_000
DEV_SEED, DEV_N, DEV_DRIFT_SEED, DEV_NOISE = 399, 5, 335, 399_000
NOM_DRIFT = {"s_on": 1600.0, "ell": 800.0, "f_end": 0.82}
NOM_NOISE = 399_900
CONDITIONS = ("C", "D", "N", "L")                       # constant, drift, noise, lag
RHO_GRID = [{"rho0": a, "rho_s": b} for a in (0.0, 2.0, 4.0) for b in (0.0, 0.1, 0.2)]
DEV_SETTING = {"rho0": 2.0, "rho_s": 0.1}               # the middle of the grid (the gate's setting)
NO_RESIDUAL = {"rho0": 0.0, "rho_s": 0.0}
RA_KINDS = ("ra", "ra_fixed", "ra_fixed_res", "ra_shrink")
PARENTS = ("nmpc", "tube", "adaptive")
DEV_KINDS = PARENTS + RA_KINDS
TEST_KINDS = PARENTS + RA_KINDS
ADAPTIVE_KINDS = cx_d.ADAPTIVE_KINDS + RA_KINDS


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


# ---------------------------------------------------------------- sets and conditions
class StartFixedDrift(cx_d.DriftField):
    """Phase D's drift field with positions within 10^-6 m below L counted as s = 0 (the lap's start state projects to
    L - 1.4e-11; CX-D packet 4)."""

    def __call__(self, s):
        q = np.mod(np.asarray(s, float), self.length)
        return super().__call__(np.where(q > self.length - 1e-6, 0.0, q))


def make_sim(case: dict, track):
    """The case's simulator: the corrected drift field, else phase D's."""
    if case["drift"] is not None:
        from ..sim.stochastic import Disturbances, StochasticSim

        d = case["drift"]
        return StochasticSim(track, case["plant"], Disturbances(StartFixedDrift(track.length, d["s_on"], d["ell"],
                                                                                d["f_end"]), None, None, 0), DT)
    return cx_d.make_sim(case, track)


def _cars(seed: int, n: int):
    return cx_d._cars(seed, n)


def cases(name: str) -> list[dict]:
    """The laps of one set and condition, e.g. ``JT``, ``JD``, ``JV_N``, ``DEV_L``, ``NOM_D`` (section 2)."""
    base, cond = (name[0:2], {"JT": "C", "JD": "D", "JN": "N", "JL": "L"}.get(name)) if name in ("JT", "JD", "JN", "JL") \
        else (name.rsplit("_", 1)[0], name.rsplit("_", 1)[-1])
    if cond not in CONDITIONS:
        raise KeyError(name)
    if name in ("JT", "JD", "JN", "JL"):
        seed, n, drift_seed, noise, key = JT_SEED, N_CARS, JD_DRIFT_SEED, JT_NOISE, f"seed{JT_SEED}"
    elif base == "JV":
        seed, n, drift_seed, noise, key = JV_SEED, JV_N, JV_DRIFT_SEED, JV_NOISE, f"seed{JV_SEED}"
    elif base == "DEV":
        seed, n, drift_seed, noise, key = DEV_SEED, DEV_N, DEV_DRIFT_SEED, DEV_NOISE, f"seed{DEV_SEED}"
    elif base == "NOM":
        kw = {"drift": NOM_DRIFT} if cond == "D" else {}
        if cond == "N":
            kw["noise_seed"] = NOM_NOISE
        if cond == "L":
            kw["tau"] = cx_a.LAG_TAU
        return [cx_d._case(name, f"{name}_car0", 0, {"nominal": True}, DynVehicle(), "nominal", **kw)]
    else:
        raise KeyError(name)
    cars = _cars(seed, n)
    drifts = cx_d.draw_drifts(n, drift_seed) if cond == "D" else [None] * n
    out = []
    for i, ((info, plant), dr) in enumerate(zip(cars, drifts), start=1):
        kw: dict = {"drift": dr}
        if cond == "N":
            kw["noise_seed"] = noise + i
        elif cond == "L":
            kw["tau"] = cx_a.LAG_TAU
        out.append(cx_d._case(name, f"{name}_car{i}", i, info, plant, f"{key}_car{i}", **kw))
    return out


def case_by_name(name: str) -> dict:
    set_, _ = name.rsplit("_car", 1)
    return next(c for c in cases(set_) if c["name"] == name)


# ---------------------------------------------------------------- controllers
def build(kind: str, setting: dict, track, case: dict, sim):
    """(controller, mpc, x0, extra): phase D's kinds through ``cx_d.build``; the ``ra`` kinds below."""
    if kind == "tube":
        ctl, mpc, tube, x0 = cx_c.build("tube", TUBE_SETTING, track, _plans(case["track"]))
        return ctl, mpc, x0, {"tube": tube}
    if kind in ("nmpc", "adaptive_oracle"):
        return cx_d.build(kind, {}, track, case, sim)
    if kind == "adaptive":
        return cx_d.build("adaptive", {"lam_mu": LAM_MU}, track, case, sim)
    if kind not in RA_KINDS:
        raise ValueError(kind)
    from ..control.nmpc import FrenetMPC
    from ..control.radaptive import AdaptiveTube, RobustAdaptiveMPCController
    from ..estimation.rls import RecursiveLS
    from .l1 import dyn_reference

    model = DynVehicle()
    etas, lib = cx_d.plan_library()
    o = _plans(case["track"])[ETA]
    path, _ = dyn_reference(track, model, o)
    x0 = x0_from_plan(o, path)
    w = {k: v * cx_d.NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    mpc = FrenetMPC(track, model, [ref_from_plan(p) for p in lib], mode="track", N=N, dt=MPC_DT, eta=ETA,
                    margin=cx_d.NMPC_SETTING["margin"], weights=w, param_model=True, param_tightening=True,
                    ref_grid=int(np.flatnonzero(np.isclose(etas, ETA))[0]))
    tube = AdaptiveTube(mpc, margin=cx_d.NMPC_SETTING["margin"], eta=ETA, c_R=TUBE_SETTING["c_R"], eta_min=cx_c.ETA_MIN)
    est = RecursiveLS(model, DT, lam_mu=LAM_MU)
    s = {"ra": {}, "ra_fixed_res": {}, "ra_fixed": NO_RESIDUAL, "ra_shrink": NO_RESIDUAL}[kind] or setting
    ctl = RobustAdaptiveMPCController(mpc, tube, DT, etas, est, rho0=s["rho0"], rho_s=s["rho_s"],
                                      shrink=kind in ("ra", "ra_shrink"))
    return ctl, mpc, x0, {"etas": etas, "tube": tube}


def _plans(track_name: str):
    from .l8 import plans_for

    return plans_for(track_name)


# ---------------------------------------------------------------- one lap
class _Seen:
    """Wraps a controller: what it saw (time, state) and commanded at every call, and the first-stage prediction of every
    solve (from its MPC, after the call); ``record`` in ``lap``."""

    def __init__(self, ctl, mpc):
        self.ctl, self.mpc = ctl, mpc
        self.x, self.u, self.t = [], [], []
        self.pred1, self.pred_full, self.solve_s = [], [], []
        self.n_stats = 0

    def __getattr__(self, name):
        return getattr(self.ctl, name)

    def __call__(self, t, x):
        out = self.ctl(t, x)
        self.t.append(t)
        self.x.append(np.array(x, float))
        self.u.append(np.array(out, float))
        if len(self.mpc.stats) > self.n_stats:
            self.n_stats = len(self.mpc.stats)
            info = self.mpc.stats[-1]
            X, U = self.mpc.prediction()
            self.pred1.append(np.concatenate([[t, float(info["fallback"])], X[1]]))
            if (self.n_stats - 1) % 10 == 0:
                self.pred_full.append((t, X.astype(np.float32), U.astype(np.float32)))
        return out


def _post_ra(out: dict, ctl, mpc, sim, res, track, extra) -> None:
    """Section 5 for the ``ra`` kinds: the box, the residual terms, the tightening, the coverage and the true errors."""
    log = getattr(ctl, "tube_log", None)
    if not log or "h" not in log[0]:
        return
    ctl_ = getattr(ctl, "ctl", ctl)
    pct = cx_c._pct
    H = np.array([r["h"] for r in log])
    Q = np.array([r["q"] for r in log])
    sig = np.array([r["sigma"] for r in log])
    eta = np.concatenate([r["eta"][1:] for r in log])
    b = np.concatenate([r["b_all"][1:] for r in log])
    th = np.array([r["theta"] for r in log])
    case = out["_case"]
    truth = cx_d.true_theta(case)
    field = cx_d.friction_field(sim)
    s = np.array([r["s"] for r in log])
    f = np.array([float(field(q)) for q in s]) if field is not None else np.ones(len(s))
    true_series = np.tile(truth, (len(s), 1))
    true_series[:, 0] = truth[0] * f
    err = np.abs(th - true_series)
    tt = np.array([r["tube_time"] for r in log]) * 1e3
    ts = np.array([s_["time"] for s_ in mpc.stats]) * 1e3
    out.update({
        "h_p5_p50_p95_per_param": [pct(H[:, i]) for i in range(7)],
        "at_cap_share_per_param": [float(np.mean(np.isclose(H[:, i], ctl_.cap[i]))) for i in range(7)],
        "q_p50_p95_per_component": [[float(np.percentile(Q[:, i], v)) for v in (50, 95)] for i in range(7)],
        "sigma_p50_per_param": [float(np.median(sig[:, i])) for i in range(7)],
        "err_within_h_share_per_param": [float(np.mean(err[:, i] <= H[:, i])) for i in range(7)],
        "err_end_vs_sigma_end": [[float(err[-1, i]), float(sig[-1, i])] for i in range(7)],
        "eta_k_p5_p50_p95": pct(eta), "backoff_m_p5_p50_p95": pct(b),
        "capped_track_share_mean": float(np.mean([r["capped_track"] for r in log])),
        "capped_eta_share_mean": float(np.mean([r["capped_eta"] for r in log])),
        "tube_ms_mean_p95_max": [float(tt.mean()), float(np.percentile(tt, 95)), float(tt.max())],
        "step_ms_tube_plus_solve_mean": float(np.mean(tt[: len(ts)] + ts[: len(tt)])) if len(tt) else math.nan,
        "residuals_n": len(ctl_.residuals),
        **cx_c.coverage(track, res, [{"t": r["t"], "b": r["b_all"], "X_pred": r["X_pred"]} for r in log],
                        (mpc.veh.lf, -mpc.veh.lr)),
    })


def lap(case: dict, kind: str, setting: dict, record: str | Path | None = None) -> dict:
    """One lap of ``kind`` on ``case`` with ``setting`` (the residual coefficients for the ``ra`` kinds, ignored by the
    others); ``record``: path of the recorded arrays (``.npz``; development checks 9.2 / 9.4)."""
    seen: dict = {}

    def wrap(ctl, mpc):
        if record is None:
            return ctl
        seen["s"] = _Seen(ctl, mpc)
        return seen["s"]

    def post(out, ctl, mpc, sim, res, track, extra):
        out["_case"] = case
        if kind in RA_KINDS:
            _post_ra(out, getattr(ctl, "ctl", ctl), mpc, sim, res, track, extra)
        out.pop("_case")
        if record is not None:
            s_ = seen["s"]
            Path(record).parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(record, t=np.array(s_.t), x=np.array(s_.x), u=np.array(s_.u), pred1=np.array(s_.pred1),
                                pred_t=np.array([p[0] for p in s_.pred_full]), pred_X=np.array([p[1] for p in s_.pred_full]),
                                pred_U=np.array([p[2] for p in s_.pred_full]))
    out = cx_d.lap(case, kind, setting, build_fn=build, sim_fn=make_sim, adaptive_kinds=ADAPTIVE_KINDS, wrap=wrap,
                   post=post)
    out["condition"] = next((c for c, n in (("D", "drift"), ("N", "noise_seed"), ("L", "tau")) if case.get(n)), "C")
    return out


# ---------------------------------------------------------------- tuning rule
def select(records: list[dict], full: bool = False) -> dict:
    """Section 4 for ``ra`` (``full``: require the nine grid settings on exactly the forty JV laps): the most completed JV laps of all four conditions; ties by the mean lap time on the cases every
    tied setting completes (skipped if none); then the setting nearest the middle of the grid (rho0 2, rho_s 0.1); then the
    smaller rho0, the smaller rho_s. Different lap counts or cases stop it."""
    by: dict[str, dict] = {}
    for r in records:
        if r.get("kind") != "ra" or "error" in r:
            continue
        d = by.setdefault(r["setting_key"], {"setting": r["setting"], "done": {}})
        if r["case"] in d["done"]:
            raise ValueError(f"ra {r['setting_key']}: {r['case']} twice")
        d["done"][r["case"]] = r["lap_time"] if r["completed"] else None
    if not by:
        raise ValueError("no records for ra")
    if len({frozenset(d["done"]) for d in by.values()}) != 1:
        raise ValueError("ra: settings with different laps or cases")
    if full:                                             # the registered tuning: 9 settings x 40 JV laps (review m3)
        names = {c["name"] for cond in CONDITIONS for c in cases(f"JV_{cond}")}
        if set(by) != {setting_key(g) for g in RHO_GRID} or set(next(iter(by.values()))["done"]) != names:
            raise ValueError("ra: not the nine grid settings on the forty JV laps")
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

    def dist(k):
        s = by[k]["setting"]
        return (abs([0.0, 2.0, 4.0].index(s["rho0"]) - 1) + abs([0.0, 0.1, 0.2].index(s["rho_s"]) - 1), s["rho0"], s["rho_s"])
    chosen = min(tied, key=dist)
    return {"setting": by[chosen]["setting"], "setting_key": chosen, "table": table}


__all__ = ["cases", "case_by_name", "lap", "select", "build", "make_sim", "StartFixedDrift", "RHO_GRID", "DEV_SETTING",
           "RA_KINDS", "TEST_KINDS", "DEV_KINDS", "CONDITIONS", "setting_key"]
