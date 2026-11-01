"""Control expansion phase I – development checks of candidate I-1 (docs/CX_I_protocol.md section 9; NOM and DEV only).
Windows `.venv`; records in results/CX/I/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_i_dev.py nlp-repro     # 9.1: phase E's NLP unchanged (the bound off) – lmpc K 20 on NOM against
                                             # phase E's record
    python -m pytest tests/test_cx_i.py      # 9.2: the bound (unit tests)
    python scripts/run_cx_i.py dev           # 9.3: stages A and B, nmpc and identified on NOM + DEV
    python scripts/report_cx_i.py dev        # 9.4: the tables and the gate

Revision 1 (section 10.7), records in results/CX/I/repro/:
    python scripts/cx_i_dev.py repro-r0      # r0: with the new options off, phase E's lmpc K 20 and lmpc_nominal K 10 and
                                             # phase H's atv_nominal K 20 on NOM reproduce their records
    python scripts/cx_i_dev.py repro-r2      # r2: v_c's rule and the two initialisation laps at v_c (from the smoke's
                                             # progress log, results/CX/I/repro_smoke/progress_R1.jsonl)
    python scripts/cx_i_dev.py repro-r2-usage  # r2: where the first v_c lap's largest tyre usage arises (descriptive)
    python scripts/cx_i_dev.py repro-r2-diag   # R2's reading (c2) examined: K 20 from phase E's start, R2 with K 10
    python scripts/cx_i_dev.py repro-snap      # R1 again through learning lap 3, the store pickled after every lap
    python scripts/cx_i_dev.py repro-replay --which R3_learn1|R1_learn4   # a failed lap again from its snapshot
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
DEV = ROOT / "results" / "CX" / "I" / "dev"
E_DEV = ROOT / "results" / "CX" / "E" / "dev" / "laps.jsonl"
H_DEV = ROOT / "results" / "CX" / "H" / "dev" / "laps.jsonl"
REPRO = ROOT / "results" / "CX" / "I" / "repro"


def meta(what: str, d: Path = DEV) -> None:
    d.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (d / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def nlp_repro() -> None:
    from f1race.experiments import cx_e

    meta("nlp_repro")
    ref = next(json.loads(l) for l in E_DEV.read_text().splitlines()
               if l.strip() and json.loads(l).get("job") == "dev|lmpc|K=20|NOM_car0")
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], "lmpc", {"K": 20})
    pick = lambda rec: [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in rec["laps"][:4]]  # noqa: E731
    out = {"wall_s": time.time() - t0,
           "phase_e": {"lap_times": ref["lap_times"], "first_failed": ref["first_failed_learning_lap"], "laps": pick(ref)},
           "now": {"lap_times": r["lap_times"], "first_failed": r["first_failed_learning_lap"], "laps": pick(r)}}
    out["equal"] = out["phase_e"]["lap_times"] == out["now"]["lap_times"] and out["phase_e"]["laps"] == out["now"]["laps"]
    (DEV / "nlp_repro.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


_R0 = [("E", "lmpc", {"K": 20}, "nlp"), ("E", "lmpc_nominal", {"K": 10}, "nlp"), ("H", "lmpc_nominal", {"K": 20}, "atv")]


def _laps(rec: dict) -> list:
    return [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in rec["laps"]]


def _r0_job(i: int) -> dict:
    from f1race.experiments import cx_e

    phase, kind, setting, solver = _R0[i]
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], kind, setting, solver=solver)
    return {"i": i, "lap_times": r["lap_times"], "laps": _laps(r), "wall_s": time.time() - t0}


def repro_r0() -> None:
    """r0: the default path unchanged (section 10.7) – three NOM records reproduced lap for lap."""
    from multiprocessing import get_context

    meta("r0", REPRO)
    ref = []
    for phase, kind, setting, solver in _R0:
        path, k = (E_DEV, kind) if phase == "E" else (H_DEV, "atv_nominal")
        key = f"K={setting['K']}"
        ref.append(next(json.loads(l) for l in path.read_text().splitlines() if l.strip()
                        and json.loads(l).get("kind") == k and json.loads(l).get("setting_key") == key
                        and json.loads(l).get("case") == "NOM_car0" and "error" not in json.loads(l)))
    with get_context("spawn").Pool(len(_R0), maxtasksperchild=1) as pool:
        now = sorted(pool.map(_r0_job, range(len(_R0))), key=lambda r: r["i"])
    out = []
    for (phase, kind, setting, solver), rf, nw in zip(_R0, ref, now):
        eq = rf["lap_times"] == nw["lap_times"] and _laps(rf) == [tuple(x) for x in nw["laps"]]
        out.append({"phase": phase, "kind": kind, "setting": setting, "solver": solver, "equal": eq,
                    "record": {"lap_times": rf["lap_times"], "laps": _laps(rf)}, "now": nw})
    res = {"all_equal": all(o["equal"] for o in out), "checks": out}
    (REPRO / "r0.json").write_text(json.dumps(res, indent=1, default=float))
    print(json.dumps({"all_equal": res["all_equal"], "equal": [o["equal"] for o in out]}))


def repro_r2() -> None:
    """r2: v_c's rule and the two initialisation laps at v_c (section 10.7), from the smoke's progress log."""
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_i
    from f1race.track import Track

    import numpy as np

    meta("r2", REPRO)
    tr = Track.load("Spielberg")
    k = np.abs(tr.kappa)
    laps = [json.loads(l) for l in (ROOT / "results" / "CX" / "I" / "repro_smoke" / "progress_R1.jsonl").read_text()
            .splitlines() if l.strip()]
    init = [{key: l.get(key) for key in ("lap", "completed", "lap_time", "stop_reason", "distance", "run_out_completed",
                                         "tyre_usage_max_front_rear", "min_edge_margin", "solves", "fallbacks",
                                         "solve_ms_mean_p95_max", "start_s", "wall_lap_s")}
            for l in laps if l["kind"] == "init"]
    out = {"v_c_rule": cx_i.v_c_rule(tr, DynVehicle()), "V_C": cx_i.V_C, "max_abs_kappa": float(k.max()),
           "s_at_max": float(tr.s[k.argmax()]), "L": float(tr.length), "init_laps": init}
    (REPRO / "r2.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def repro_r2_usage() -> None:
    """r2's tyre usage (descriptive): the first v_c lap driven again (deterministic; R1's ``init_vc1`` without its
    run-out) with the usage of each 10 ms step split into its longitudinal and lateral parts – where and why the
    largest usage arises, against section 10.2's estimate of about 0.5."""
    import numpy as np

    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.dynamics import DynVehicle, tyre_forces
    from f1race.experiments import cx_a, cx_d, cx_e, cx_i
    from f1race.track import Track

    meta("r2_usage", REPRO)
    case = cx_e.cases("NOM")[0]
    tr, veh = Track.load(case["track"]), DynVehicle()
    v_c = cx_i.V_C
    ref = {"s": tr.s, "n": np.zeros_like(tr.s), "xi": np.zeros_like(tr.s), "vx": np.full(tr.s.shape, v_c)}
    w = {k: v * cx_e.NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    mpc = FrenetMPC(tr, veh, ref, mode="track", N=cx_e.N, dt=cx_e.MPC_DT, eta=cx_e.ETA,
                    margin=cx_e.NMPC_SETTING["margin"], weights=w)
    x0 = np.array([tr.x[0], tr.y[0], float(tr.heading_at(0.0)), v_c, 0.0, 0.0, 0.0])
    t0 = time.time()
    res = cx_d.make_sim(case, tr).run(MPCController(mpc, cx_e.DT), x0, t_max=1.25 * tr.length / v_c + 50.0)
    X, U = res.state, res.inputs
    F = tyre_forces(veh, X[:, 3], X[:, 4], X[:, 5], X[:, 6], U[:, 1], U[:, 2])
    lon = np.stack([F["fxf"] / (veh.mu_x * F["fzf"]), F["fxr"] / (veh.mu_x * F["fzr"])], axis=1)
    lat = np.stack([F["fyf"] / (veh.mu_y * F["fzf"]), F["fyr"] / (veh.mu_y * F["fzr"])], axis=1)
    s = np.mod(res.progress, tr.length)
    kap = np.interp(s, tr.s, tr.kappa)
    peaks, um = [], res.usage.max(axis=1)
    for i in np.argsort(-um):                          # the largest usages, at least 20 m apart
        if len(peaks) == 10:
            break
        if all(abs(s[i] - p["s"]) > 20.0 for p in peaks):
            a = int(np.argmax(res.usage[i]))
            peaks.append({"s": float(s[i]), "t": float(res.t[i]), "axle": "front" if a == 0 else "rear",
                          "usage": float(res.usage[i, a]), "lon": float(lon[i, a]), "lat": float(lat[i, a]),
                          "vx": float(X[i, 3]), "kappa": float(kap[i]), "f_drive": float(U[i, 1]),
                          "f_brake": float(U[i, 2]), "delta": float(X[i, 6]), "r": float(X[i, 5])})
    out = {"lap_time": float(res.lap_time), "completed": bool(res.completed), "stop_reason": res.stop_reason,
           "smoke_init_vc1": 538.5054517776696, "wall_s": time.time() - t0,
           "usage_max_front_rear": [float(res.usage[:, 0].max()), float(res.usage[:, 1].max())],
           "share_steps_usage_above": {str(q): float(np.mean(um > q)) for q in (0.5, 0.7, 0.8, 0.9)},
           "lat_part_max_front_rear": [float(np.abs(lat[:, 0]).max()), float(np.abs(lat[:, 1]).max())],
           "lon_part_max_front_rear": [float(np.abs(lon[:, 0]).max()), float(np.abs(lon[:, 1]).max())],
           "vx_min_max": [float(X[:, 3].min()), float(X[:, 3].max())], "peaks": peaks}
    np.savez_compressed(REPRO / "r2_usage.npz", t=res.t, s=s, usage=res.usage, lon=lon, lat=lat, vx=X[:, 3], kappa=kap)
    (REPRO / "r2_usage.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


_R2D = [("E_start_K20", {"K": 20, "n_learn": 2}), ("R2_K10", {"K": 10, "consecutive": True, "n_learn": 2})]
_LAP_KEYS = ("lap", "completed", "lap_time", "stop_reason", "failure_s", "solves", "fallbacks",
             "term_slack_share_above_1e-3", "term_slack_max", "tyre_usage_max_front_rear", "min_edge_margin",
             "fallbacks_last20", "failure_class", "failure_vx", "failure_s_lap", "failure_v_stored")


def _r2d_job(i: int) -> dict:
    from f1race.experiments import cx_e

    name, setting = _R2D[i]
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], "lmpc_nominal", setting)
    return {"i": i, "name": name, "setting": setting, "lap_times": r["lap_times"], "wall_s": time.time() - t0,
            "laps": [{k: l.get(k) for k in _LAP_KEYS} for l in r["laps"]]}


def repro_r2_diag() -> None:
    """Reading (c2) of R2 examined (section 10.5, clarification m1; descriptive): R2's first learning lap failed. Two
    neighbours with two learning laps each: lmpc_nominal K 20 under phase E's conditions (every lap from the plan's
    start; is the consecutive start the cause?) and R2 with K 10 (phase E's selected K for lmpc_nominal)."""
    from multiprocessing import get_context

    meta("r2_diag", REPRO)
    with get_context("spawn").Pool(len(_R2D), maxtasksperchild=1) as pool:
        out = sorted(pool.map(_r2d_job, range(len(_R2D))), key=lambda r: r["i"])
    (REPRO / "r2_diag.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps([{"name": o["name"], "lap_times": o["lap_times"]} for o in out], default=float))


def repro_snap() -> None:
    """Snapshots for the diagnostics of R3's (c2) and R1's failure (descriptive): R1 driven again through its third
    learning lap (deterministic; its lap times checked against R1's record), with the store and the next lap's start
    state and carried command pickled after every stored lap – ``snap/R1_store<n>.pkl`` (n = 2: the two v_c laps, R3's
    store at its first learning lap; n = 5: R1's store at its fourth learning lap)."""
    import pickle

    import numpy as np

    from f1race.control import lmpc as L
    from f1race.experiments import cx_e, cx_i

    meta("snap", REPRO)
    d = REPRO / "snap"
    d.mkdir(parents=True, exist_ok=True)
    held: dict = {}
    handover, add = cx_e._handover, L.LapStore.add

    def handover_held(res, track, rec_t, rec_u):
        out = handover(res, track, rec_t, rec_u)
        held["x"], held["u"] = out[1].copy(), np.asarray(out[2], float).copy()
        return out

    def add_snap(self, t, X, U, t_finish):
        add(self, t, X, U, t_finish)
        with open(d / f"R1_store{len(self.laps)}.pkl", "wb") as f:
            pickle.dump({"laps": self.laps, "x_start": held["x"], "u_start": held["u"]}, f)

    cx_e._handover, L.LapStore.add = handover_held, add_snap
    t0 = time.time()
    r = cx_i.repro("R1", n_learn=3)
    ref = [json.loads(l)["lap_time"] for l in (REPRO / "progress_R1.jsonl").read_text().splitlines() if l.strip()][:5]
    out = {"lap_times": r["lap_times"], "R1_record": ref, "equal": r["lap_times"] == ref, "wall_s": time.time() - t0,
           "snapshots": sorted(p.name for p in d.glob("R1_store*.pkl"))}
    (d / "snap.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, default=float))


_REPLAY = {"R3_learn1": ("R3", 2, "learn_1"), "R1_learn4": ("R1", 5, "learn_4")}


def repro_replay(which: str) -> None:
    """A failed lap of the reproduction check driven again from its snapshot (``repro-snap``; descriptive): checked
    against the arm's record (stop, arclength, solves, fallbacks), with every solve's status, iterations, slacks and the
    prediction's first and last stage (``replay_<which>.json``). R3's first learning lap: also its first QP solved again
    as probes – as it was, with the time-to-go shifted by its minimum (the same QP's solution, as the lambdas sum to
    one; clarification n1's scale), and with OSQP's iteration limit x 10."""
    import pickle
    from collections import Counter

    import numpy as np

    from f1race.control.lmpc import TERM, ATVLearningMPC, ATVLMPCController, LapStore, LearningMPC, LMPCController
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_e, cx_i
    from f1race.track import Track

    arm, n, lap = _REPLAY[which]
    meta(f"replay_{which}", REPRO)
    snap = pickle.loads((REPRO / "snap" / f"R1_store{n}.pkl").read_bytes())
    case = cx_e.cases("NOM")[0]
    tr, model = Track.load(case["track"]), DynVehicle()
    atv = cx_i.REPRO[arm][1] == "atv"
    K, P = cx_i.K, cx_e.P_LAPS

    def build(**kw):
        return (ATVLearningMPC if atv else LearningMPC)(tr, model, n_pts=P * K, N=cx_e.N, dt=cx_e.MPC_DT, eta=cx_e.ETA,
                                                          margin=cx_e.NMPC_SETTING["margin"], w_T=cx_e.W_T, term=TERM,
                                                          **kw)

    lm = build()
    store = LapStore(lm._step)
    store.laps = snap["laps"]
    ctl = (ATVLMPCController if atv else LMPCController)(lm, store, K, P, regression=False, sim_dt=cx_e.DT)
    ctl.u = snap["u_start"].copy()
    pred, first = [], {}
    solve_name = "solve_qp" if atv else "solve"
    inner = getattr(lm, solve_name)

    def solve_logged(*args, **kw):
        if not first:
            first["args"] = [np.array(a_, float) if a_ is not None else None for a_ in args]
        out = inner(*args, **kw)
        X = lm.z[lm.Xi]
        pred.append({"n1": float(X[1, 1]), "vx1": float(X[1, 3]), "nN": float(X[-1, 1]), "vxN": float(X[-1, 3])})
        return out

    setattr(lm, solve_name, solve_logged)
    t_limit = cx_e.learn_time_limit([l_["T"] for l_ in snap["laps"][:2]])
    t0 = time.time()
    res = cx_e._run(case, tr, ctl, snap["x_start"], t_max=t_limit + 50.0)
    m = cx_e._lap_metrics(res, tr, ctl, t_limit)
    rec = next(json.loads(l_) for l_ in (REPRO / f"progress_{arm}.jsonl").read_text().splitlines()
               if l_.strip() and json.loads(l_)["lap"] == lap)
    keys = ("stop_reason", "failure_s", "solves", "fallbacks")
    rows = [{"t": d["t"], "s": d["s"], "c_min": d["c_min"], **{k: st[k] for k in ("status", "iter", "time", "fallback",
                                                                                   "slack", "limit_slack", "term_slack")},
             **p_} for d, st, p_ in zip(ctl.diag, lm.stats, pred)]
    if atv:
        for r_, le in zip(rows, ctl.lin_err):
            r_["lin_err_ds_dn_dvx"] = list(le)
    out = {"which": which, "wall_s": time.time() - t0, "t_limit": t_limit,
           "replay": {k: m.get(k) for k in keys}, "record": {k: rec.get(k) for k in keys},
           "status_counts": dict(Counter(r_["status"] for r_ in rows)), "rows": rows}
    out["equal"] = all(out["replay"][k] == out["record"][k] for k in keys)
    if atv:                                            # the first QP again, as probes
        x0, u_prev, D, c, G, zbar = first["args"]
        probes = {}
        for name, kw, c_ in (("as_run", {}, c), ("c_shifted", {}, c - c.min()),
                             ("max_iter_x10", {"osqp": {"max_iter": 200000}}, c)):
            q = build(**kw)
            u, info = q.solve_qp(x0, u_prev, D, c_, G, zbar)
            probes[name] = {"status": info["status"], "iter": info["iter"], "fallback": info["fallback"],
                            "time": info["time"], "u": [float(v) for v in u], "c_min_max": [float(c_.min()),
                                                                                         float(c_.max())]}
        out["first_qp_probes"] = probes
    (REPRO / f"replay_{which}.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}, indent=1, default=float))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["nlp-repro", "repro-r0", "repro-r2", "repro-r2-usage", "repro-r2-diag",
                                     "repro-snap", "repro-replay"])
    ap.add_argument("--which", choices=sorted(_REPLAY), default="R3_learn1")
    a = ap.parse_args()
    if a.what == "repro-replay":
        return repro_replay(a.which)
    {"nlp-repro": nlp_repro, "repro-r0": repro_r0, "repro-r2": repro_r2, "repro-r2-usage": repro_r2_usage,
     "repro-r2-diag": repro_r2_diag, "repro-snap": repro_snap}[a.what]()


if __name__ == "__main__":
    main()
