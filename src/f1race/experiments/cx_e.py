"""Control expansion, phase E (docs/CX_E_protocol.md, revision 0 with its clarifications): the sets, the initialisation and
learning laps of a car, and the fixed comparisons.

* Sets (section 2): ``CEV`` – phase A's CXV cars (tuning); ``CET`` – ``sample_plants(nominal, 50, seed 342)[1:]`` (test);
  ``DEV`` – phase C's CXDEV cars (development only); ``NOM``; secondary ``CET_LAG30`` (the 30 ms steering lag),
  ``CET_WET85`` (mu x 0.85). Case records as phase D's (``cx_d._case``), so that ``cx_d.make_sim`` and ``cx_d.lap`` serve.
* ``learn(case, kind, setting)``: ``lmpc`` / ``lmpc_nominal`` on one car – two initialisation laps (``nmpc``'s NMPC on the
  plans of phase D's library at eta_p 0.73 and 0.79), then ten learning laps, every lap from the 5 % plan's start state,
  driven on for 250 m past the line (unscored); a crash (off track, spin, the time limit) ends the car's sequence (the
  remaining laps not completed); the record lists every lap.
* ``fixed_lap(case, kind)``: ``nmpc`` and ``identified`` through phase D's ``cx_d.lap`` (``identified``'s cache in
  results/CX/E/identified_cache).
"""
from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from ..dynamics import DynVehicle
from . import cx_a, cx_c, cx_d
from .l2 import ref_from_plan, x0_from_plan

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results" / "CX" / "E"
ETA, DT, MPC_DT, N = cx_d.ETA, cx_d.DT, cx_d.MPC_DT, cx_d.N
T_LAP = 121.0                                  # the scored lap's limit (phases A-D)
RUNOUT = 250.0                                 # m past the line, unscored (section 2)
INIT_ETAS = (0.73, 0.79)
N_LEARN, P_LAPS = 10, 2
W_T = 100.0
K_GRID = [{"K": k} for k in (10, 20, 40)]
NMPC_SETTING = cx_d.NMPC_SETTING
CET_SEED = 342
LEARN_KINDS = ("lmpc", "lmpc_nominal")
FIXED_KINDS = ("nmpc", "identified")
KINDS = ("nmpc", "identified", "lmpc", "lmpc_nominal")
SECONDARY = ("CET_LAG30", "CET_WET85")


def setting_key(setting: dict) -> str:
    return cx_a.setting_key(setting)


def cases(name: str) -> list[dict]:
    from .l2 import sample_plants
    from .l8 import wet

    if name == "NOM":
        return [cx_d._case("NOM", "NOM_car0", 0, {"nominal": True}, DynVehicle(), "nominal")]
    src = {"CEV": (301, 10), "DEV": (399, 5), "CET": (CET_SEED, 50), "CET_LAG30": (CET_SEED, 50),
           "CET_WET85": (CET_SEED, 50)}
    if name not in src:
        raise KeyError(name)
    seed, n = src[name]
    out = []
    for i, (info, plant) in enumerate(sample_plants(DynVehicle(), n, seed)[1:], start=1):
        kw: dict = {}
        if name == "CET_LAG30":
            kw["tau"] = cx_a.LAG_TAU
        elif name == "CET_WET85":
            kw["wet"], plant = 0.85, wet(plant, 0.85)
        out.append(cx_d._case(name, f"{name}_car{i}", i, info, plant, f"seed{seed}_car{i}", **kw))
    return out


def _lap_metrics(res, track, ctl=None) -> dict:
    """The scored lap (time at s = L) and the run-out of one simulated lap."""
    L = track.length
    prog = np.asarray(res.progress, float)
    above = np.flatnonzero(prog >= L)                  # progress at each recorded step; the run-out is 250 m long
    crossed = bool(above.size)
    lap_time = math.nan
    if crossed:
        k = int(above[0])
        t0, t1, p0, p1 = res.t[k - 1], res.t[k], prog[k - 1], prog[k]
        lap_time = float(t0 + (t1 - t0) * (L - p0) / max(p1 - p0, 1e-12))
    completed = crossed and lap_time <= T_LAP
    run_out_ok = bool(res.completed)                   # the sim reached L + RUNOUT
    s_c, n_c = track.project(res.state[:, 0], res.state[:, 1])
    out = {"completed": bool(completed), "lap_time": float(lap_time) if completed else math.nan,
           "stop_reason": "lap completed" if completed else (res.stop_reason if not crossed else "time limit"),
           "distance": float(min(prog[-1], L)), "failure_s": None if completed else float(np.mod(s_c[-1], L)),
           "run_out_completed": run_out_ok, "run_out_stop": None if run_out_ok else res.stop_reason,
           "min_edge_margin": float(np.min(res.margin)),
           "tyre_usage_max_front_rear": [float(np.max(res.usage[:, 0])), float(np.max(res.usage[:, 1]))]}
    if ctl is not None and getattr(ctl, "mpc", None) is not None and getattr(ctl.mpc, "stats", None):
        st = ctl.mpc.stats
        ts = np.array([s_["time"] for s_ in st]) * 1e3
        out.update({"solves": len(st), "solve_ms_mean_p95_max": [float(ts.mean()), float(np.percentile(ts, 95)),
                                                                   float(ts.max())],
                    "fallbacks": int(sum(bool(s_["fallback"]) for s_ in st))})
        if "term_slack" in st[0]:
            ts_ = np.array([s_["term_slack"] for s_ in st])
            out.update({"term_slack_share_above_1e-3": float(np.mean(ts_ > 1e-3)), "term_slack_max": float(ts_.max())})
        if getattr(ctl, "diag", None):
            out["regression_fallback_share"] = float(np.mean([d["fallback_share"] for d in ctl.diag]))
        if getattr(ctl, "eps", None) is not None and ctl.diag:   # phase I's speed bound (CX_I section 5)
            d = ctl.diag
            out.update({"speed_slack_share_above_1e-3": float(np.mean([d_["speed_slack"] > 1e-3 for d_ in d])),
                        "speed_near_share": float(np.mean([d_["speed_near_share"] for d_ in d])),
                        "speed_slack_max": float(max(d_["speed_slack"] for d_ in d)),
                        "last_solve_s_vx_vref_vbar1": [float(d[-1][k]) for k in ("s", "vx", "v_ref", "vbar1")]})
        if getattr(ctl, "lin_err", None):              # phase H's ATV controller: the linearisation error per solve
            le = np.array(ctl.lin_err)
            out["lin_err_ds_dn_dvx_p50"] = [float(v) for v in np.median(le, axis=0)]
            out["lin_err_ds_dn_dvx_p95"] = [float(v) for v in np.percentile(le, 95, axis=0)]
    return out


def _run(case: dict, track, ctl, x0) -> object:
    sim = cx_d.make_sim(case, track)
    return sim.run(ctl, x0, t_max=T_LAP + 15.0, laps=1.0 + RUNOUT / track.length)


def learn(case: dict, kind: str, setting: dict, solver: str = "nlp") -> dict:
    """Section 3: the initialisation laps and the ten learning laps of one car. ``solver`` (control expansion phase H,
    docs/CX_H_protocol.md): "nlp" – this phase's IPOPT problem (default); "atv" – the same problem as one QP per solve
    along E.2's candidate (``ATVLearningMPC``, ``ATVLMPCController``). Setting key ``eps`` (control expansion phase I,
    docs/CX_I_protocol.md, candidate I-1; NLP only): the speed bound per lap with that step; absent – this phase's
    problem."""
    from ..control.lmpc import (TERM, TERM7, ATVLearningMPC, ATVLMPCController, LapStore, LearningMPC, LMPCController,
                                Recorder)
    from ..control.nmpc import FrenetMPC, MPCController
    from ..track import Track
    from .l1 import dyn_reference
    from .l8 import plans_for

    if kind not in LEARN_KINDS:
        raise ValueError(kind)
    t0 = time.time()
    track = Track.load(case["track"])
    plans = plans_for(case["track"])
    etas, lib = cx_d.plan_library()
    model = DynVehicle()
    o95 = plans[ETA]
    path, _ = dyn_reference(track, model, o95)
    x0 = x0_from_plan(o95, path)
    w = {k: v * NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    K = int(setting["K"])
    P = int(setting.get("P", P_LAPS))                 # development options (protocol section 9): P, delta in the terminal
    term = TERM7 if setting.get("term7") else TERM    # condition, fewer learning laps; the registered settings use none
    n_learn = int(setting.get("n_learn", N_LEARN))
    if solver not in ("nlp", "atv"):
        raise ValueError(solver)
    cls, ctl_cls = (ATVLearningMPC, ATVLMPCController) if solver == "atv" else (LearningMPC, LMPCController)
    kw = {}
    eps = setting.get("eps")
    if eps is not None:                              # phase I (I-1): the speed bound per lap, on the NLP only
        if solver != "nlp":
            raise ValueError("the speed bound is defined on the NLP only")
        kw["speed_bound"], eps = True, float(eps)
    if solver == "atv":                              # phase H's development diagnostics (CX-H clarification m1; the
        if setting.get("qp_solver"):                 # registered settings use neither)
            kw["qp_solver"], kw["qp_opts"] = setting["qp_solver"], {"printLevel": "none", "sparse": True}
        if setting.get("trust"):
            kw["trust"] = tuple(setting["trust"])
    lmpc = cls(track, model, n_pts=P * K, N=N, dt=MPC_DT, eta=ETA, margin=NMPC_SETTING["margin"], w_T=W_T, term=term,
               **kw)
    store = LapStore(lmpc._step)
    laps: list[dict] = []
    for e in INIT_ETAS:
        plan = lib[int(np.flatnonzero(np.isclose(etas, e))[0])]
        mpc = FrenetMPC(track, model, ref_from_plan(plan), mode="track", N=N, dt=MPC_DT, eta=ETA,
                        margin=NMPC_SETTING["margin"], weights=w)
        rec = Recorder(MPCController(mpc, DT), track, MPC_DT)
        res = _run(case, track, rec, x0)
        m = _lap_metrics(res, track, rec)
        m.update(lap=f"init_{e:.2f}", kind="init")
        laps.append(m)
        if m["completed"] and m["run_out_completed"]:
            store.add(np.array(rec.rec_t), np.array(rec.rec_x), np.array(rec.rec_u), m["lap_time"])
    started = len(store) > 0
    alive = started
    for j in range(1, n_learn + 1):
        if not alive:
            laps.append({"lap": f"learn_{j}", "kind": "learn", "completed": False, "lap_time": math.nan,
                         "stop_reason": "not started (initialisation failed)" if not started else "sequence ended",
                         "distance": 0.0, "failure_s": None})
            continue
        lmpc.reset()
        ctl = ctl_cls(lmpc, store, K, P, regression=kind == "lmpc", sim_dt=DT, eps=eps)
        res = _run(case, track, ctl, x0)
        m = _lap_metrics(res, track, ctl)
        m.update(lap=f"learn_{j}", kind="learn", stored_laps=len(store),
                 past_line_points_max=int(max((d["past_line_pts"] for d in ctl.diag), default=0)))
        laps.append(m)
        if m["completed"] and m["run_out_completed"]:
            store.add(np.array(ctl.rec_t), np.array(ctl.rec_x), np.array(ctl.rec_u), m["lap_time"])
        else:
            alive = False                               # a crash (in the lap or its run-out) ends the sequence
    learn_laps = [l_ for l_ in laps if l_["kind"] == "learn"]
    first_fail = next((i + 1 for i, l_ in enumerate(learn_laps) if not l_["completed"]), None)
    tenth = learn_laps[-1]
    return {"kind": kind, "setting": setting, "setting_key": setting_key(setting), "case": case["name"], "set": case["set"],
            "car": case["car"], "plant": case["info"], "tau": case["tau"], "wet": case["wet"],
            "initialisation_completed": [bool(l_["completed"]) for l_ in laps if l_["kind"] == "init"],
            "started": started, "first_failed_learning_lap": first_fail,
            "completed": bool(tenth["completed"]), "lap_time": float(tenth["lap_time"]) if tenth["completed"] else math.nan,
            "stop_reason": tenth["stop_reason"], "distance": tenth.get("distance", 0.0), "failure_s": tenth.get("failure_s"),
            "first_learning_lap_time": float(learn_laps[0]["lap_time"]) if learn_laps[0]["completed"] else math.nan,
            "lap_times": [float(l_["lap_time"]) if l_["completed"] else None for l_ in laps],
            "laps": laps, "wall_s": time.time() - t0}


def fixed_lap(case: dict, kind: str) -> dict:
    """``nmpc`` and ``identified`` (one scored lap, phase D's code path; ``identified``'s cache under results/CX/E)."""
    if kind not in FIXED_KINDS:
        raise ValueError(kind)
    cx_d.ID_CACHE = OUT / "identified_cache"           # this process only (one job per process)
    rec = cx_d.lap(case, kind, {})
    rec["setting"], rec["setting_key"] = {}, "default"
    return rec


def run(case: dict, kind: str, setting: dict) -> dict:
    return learn(case, kind, setting) if kind in LEARN_KINDS else fixed_lap(case, kind)


def select(records: list[dict], kind: str = "lmpc", prefer=None) -> dict:
    """Section 4: the most completed tenth learning laps on CEV; ties by the mean tenth lap time on the cars every tied
    setting completes (skipped if none); then K = 20, then the smaller K. ``prefer`` (a setting -> sort key; phase I: eps
    0.04, then the smaller eps) replaces the last two steps."""
    by: dict[str, dict] = {}
    for r in records:
        if r.get("kind") != kind or "error" in r:
            continue
        d = by.setdefault(r["setting_key"], {"setting": r["setting"], "done": {}})
        d["done"][r["case"]] = r["lap_time"] if r["completed"] else None
    if not by:
        raise ValueError(f"no records for {kind}")
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
    prefer = prefer or (lambda s: (s["K"] != 20, s["K"]))
    chosen = min(tied, key=lambda k: prefer(by[k]["setting"]))
    return {"setting": by[chosen]["setting"], "setting_key": chosen, "table": table}
