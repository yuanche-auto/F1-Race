"""Development checks of CX-D (docs/CX_D_protocol.md section 9) on NOM and the development cars DEV only; records in
results/CX/D/dev/ with the commit and the dirty flag.

    python scripts/cx_d_dev.py unchanged [--base 55c5925]   # the existing FrenetMPC NLPs against nmpc.py of a commit
    python scripts/cx_d_dev.py step       # 1: the step model against the simulator; the Jacobian against differences
    python scripts/cx_d_dev.py nlp        # 2: the parameterised NLP against FrenetMPC (nominal, vertex car, library plan)
    python scripts/cx_d_dev.py plans      # 3: the plan library; the interpolant at integer j
    python scripts/cx_d_dev.py laps [--workers 20]   # 6: development laps (the gate), records with every metric
    python scripts/cx_d_dev.py replay [--workers 12] # 4: the estimators fed offline with recorded laps
    python scripts/cx_d_dev.py drifts     # 7: the drifts drawn for CDD and CDV_D (review m2)
    python scripts/cx_d_dev.py gate       # the gate of item 6 from laps.jsonl (with nmpc on the DEV drifts, review n2)
    python scripts/cx_d_dev.py cdd-failures --records <test.jsonl>   # descriptive, after the primary report
    python scripts/cx_d_dev.py cdd-rerun --records <test.jsonl>      # the drift field's start (packet 4)

Timing (item 5) is in the lap records (estimator us per update, NMPC solve ms).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from multiprocessing import get_context
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

from f1race.experiments import cx_d  # noqa: E402

DEV = ROOT / "results" / "CX" / "D" / "dev"


def _meta() -> dict:
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    return {"commit": commit, "dirty": dirty, "date": time.strftime("%Y-%m-%d %H:%M")}


def _write(name: str, obj) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / name).write_text(json.dumps({**_meta(), **obj}, indent=1, default=float))
    print(json.dumps(obj, indent=1, default=float)[:3000])


def unchanged(base: str) -> None:
    """The NLP (objective, constraint function, bounds) of the existing FrenetMPC options, built by this commit's nmpc.py
    and by ``base``'s (``git show``), evaluated at random points – identical to the bit or not."""
    import importlib.util
    import tempfile

    import casadi as ca

    import f1race.control.nmpc as new
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import ref_from_plan
    from f1race.experiments.l8 import plans_for
    from f1race.track import Track

    src = subprocess.run(["git", "show", f"{base}:src/f1race/control/nmpc.py"], cwd=ROOT, capture_output=True,
                         text=True, check=True).stdout
    with tempfile.TemporaryDirectory() as td:
        f = Path(td) / "nmpc_base.py"
        f.write_text(src, encoding="utf-8")
        spec = importlib.util.spec_from_file_location("f1race.control.nmpc_base", f)
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
    tr, pl, veh = Track.load("Spielberg"), plans_for("Spielberg"), DynVehicle()
    ref = ref_from_plan(pl[0.95])
    w = {"n": 2.0, "xi": 20.0, "v": 0.4}
    rng = np.random.default_rng(0)
    rows = []
    for c in (dict(), dict(param_tightening=True), dict(steer_lag=0.03), dict(mode="filter"), dict(mode="mpcc"),
              dict(soft_limits=5)):
        a = old.FrenetMPC(tr, veh, ref, N=30, dt=0.05, eta=0.95, margin=0.6, weights=w, **c)
        b = new.FrenetMPC(tr, veh, ref, N=30, dt=0.05, eta=0.95, margin=0.6, weights=w, **c)
        fa = ca.Function("fa", [a._nlp["x"], a._nlp["p"]], [a._nlp["f"], a._nlp["g"]])
        fb = ca.Function("fb", [b._nlp["x"], b._nlp["p"]], [b._nlp["f"], b._nlp["g"]])
        same = True
        for _ in range(5):
            z = rng.normal(size=a._nlp["x"].shape[0])
            z[a.Xi[:, 3]] = 50 + np.abs(z[a.Xi[:, 3]])
            p = rng.normal(size=a._nlp["p"].shape[0])
            p[3] = 50.0
            A, B = fa(z, p), fb(z, p)
            same &= bool(np.array_equal(np.asarray(A[0]), np.asarray(B[0])) and np.array_equal(np.asarray(A[1]), np.asarray(B[1])))
        bounds = bool(np.array_equal(a.lbg, b.lbg) and np.array_equal(a.ubg, b.ubg) and np.array_equal(a.lbz, b.lbz)
                      and np.array_equal(a.ubz, b.ubz))
        rows.append({"options": c or "default", "f_g_identical_5_random_points": same, "bounds_identical": bounds})
    _write("unchanged.json", {"base": base, "rows": rows})


def step_check() -> None:
    from f1race.dynamics import DynVehicle
    from f1race.estimation.identify import theta_of
    from f1race.estimation.rls import _vehicle, step_function, step_np
    from f1race.experiments.l2 import sample_plants
    from f1race.sim.dynamic import DynamicSim
    from f1race.sim.stochastic import Disturbances, StochasticSim
    from f1race.track import Track

    tr, nom = Track.load("Spielberg"), DynVehicle()
    fn = step_function(nom, cx_d.DT)
    rng = np.random.default_rng(1)
    out = {"cars": []}
    for info, car in sample_plants(nom, 5, 399)[1:3] + [({}, nom)]:
        th = theta_of(info)
        for f in (1.0, 0.8):
            sim = (DynamicSim(tr, car, cx_d.DT) if f == 1.0 else
                   StochasticSim(tr, car, Disturbances(lambda s, f=f: f, None, None, 0), cx_d.DT))
            th_f = th.copy()
            th_f[0] *= f
            worst = worst_np = 0.0
            nsat = 0
            for _ in range(500):
                x = np.array([0, 0, 0, rng.uniform(20, 88), rng.uniform(-2.5, 2.5), rng.uniform(-0.7, 0.7),
                              rng.uniform(-0.25, 0.25)])
                u = sim.apply_limits(x, [rng.uniform(-1, 1), rng.uniform(0, 2.5e4), rng.uniform(-5e4, 0)])
                if f != 1.0:
                    sim._update(x)
                nsat += max(sim._forces(x, u)[1]) > 1.0
                xn = sim.step(x, u)
                worst = max(worst, float(np.max(np.abs(np.asarray(fn(x[3:7], u, th_f)[0]).ravel() - xn[3:6]))))
                worst_np = max(worst_np, float(np.max(np.abs(step_np(_vehicle(nom, th_f), x[3:7], u, cx_d.DT) - xn[3:6]))))
            out["cars"].append({"car": info or "nominal", "friction_factor": f, "samples": 500, "saturated": nsat,
                                "casadi_max_abs": worst, "numpy_max_abs": worst_np})
    jac = []
    for x, u in ((np.array([60.0, 0.5, 0.3, 0.05]), np.array([0.1, 5e3, 0.0])),
                 (np.array([35.0, -0.8, -0.5, -0.12]), np.array([-0.3, 0.0, -1e4])),
                 (np.array([80.0, 0.1, 0.05, 0.01]), np.array([0.0, 6e3, 0.0]))):
        th = theta_of(sample_plants(nom, 5, 399)[1][0])
        J = np.asarray(fn(x, u, th)[1])
        Jfd = np.zeros_like(J)
        for i in range(7):
            e = np.zeros(7)
            e[i] = 1e-6
            Jfd[:, i] = (np.asarray(fn(x, u, th + e)[0]).ravel() - np.asarray(fn(x, u, th - e)[0]).ravel()) / 2e-6
        jac.append(float(np.max(np.abs(J - Jfd)) / np.max(np.abs(J))))
    out["jacobian_rel_err"] = jac
    _write("step.json", out)


def nlp_check() -> None:
    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.dynamics import DynVehicle
    from f1race.estimation.identify import vehicle_from
    from f1race.experiments import cx_a
    from f1race.experiments.l2 import ref_from_plan
    from f1race.experiments.l8 import plans_for
    from f1race.sim.dynamic import DynamicSim
    from f1race.track import Track

    tr, pl, nom = Track.load("Spielberg"), plans_for("Spielberg"), DynVehicle()
    etas, lib = cx_d.plan_library()
    g = int(np.flatnonzero(np.isclose(etas, 0.95))[0])
    w = {k: v * 2.0 for k, v in cx_a.NMPC_TRACK_W.items()}
    kw = dict(mode="track", N=cx_d.N, dt=cx_d.MPC_DT, eta=0.95, margin=0.6, weights=w)
    t0 = time.time()
    pm = FrenetMPC(tr, nom, [ref_from_plan(p) for p in lib], param_model=True, ref_grid=g, **kw)
    build_s = time.time() - t0
    ctl, _, x0 = cx_a.build("nmpc", {"c": 2.0, "margin": 0.6}, tr, pl[0.95])
    res = DynamicSim(tr, nom, cx_d.DT).run(ctl, x0, t_max=20.0)
    fr = MPCController(pm, cx_d.DT)
    xs = [fr.frenet(res.state[k]) for k in (100, 600, 1100, 1600)]

    def cold(m, x, **a):
        m.z = m.lam_g = m.lam_x = None
        u, info = m.solve(x, np.zeros(3), **a)
        return u, m.z[m.Xi].copy(), info["status"], info["iter"]

    th_v = np.array([0.92, 1.05, 0.9, 1.1, 0.85, 1.15, 0.03])
    j88 = int(np.flatnonzero(np.isclose(etas, 0.88))[0])
    j_low = 0
    rows = []
    for x in xs:
        for name, ref_m, args in (("nominal, 5 % plan", FrenetMPC(tr, nom, ref_from_plan(pl[0.95]), **kw), {}),
                                  ("vertex car", FrenetMPC(tr, vehicle_from(th_v, nom), ref_from_plan(pl[0.95]), **kw),
                                   {"theta": th_v}),
                                  ("plan 0.88", FrenetMPC(tr, nom, ref_from_plan(pl[0.88]), **kw), {"plan": float(j88)}),
                                  (f"plan {etas[j_low]:.2f}", FrenetMPC(tr, nom, ref_from_plan(lib[j_low]), **kw),
                                   {"plan": float(j_low)})):
            a, b = cold(ref_m, x), cold(pm, x, **args)
            rows.append({"s": float(x[0]), "case": name, "du_max_abs": float(np.max(np.abs(a[0] - b[0]))),
                         "du_rel": float(np.max(np.abs(a[0] - b[0]) / np.maximum(np.abs(a[0]), 1.0))),
                         "dX_max_abs": float(np.max(np.abs(a[1] - b[1]))), "status_iter": [a[2:], b[2:]]})
    _write("nlp.json", {"build_s": build_s, "library_etas": etas.tolist(), "rows": rows})


def plans_check() -> None:
    import pickle

    from f1race.experiments.l2 import ref_from_plan

    etas, lib = cx_d.plan_library()
    with open(cx_d.PLANS_PATH, "rb") as fh:
        d = pickle.load(fh)
    from f1race.control.nmpc import _periodic_library_interpolant
    from f1race.track import Track

    L = Track.load("Spielberg").length
    g = int(np.flatnonzero(np.isclose(etas, 0.95))[0])
    sg = np.asarray(lib[g].s, float)
    F = np.column_stack([np.interp(sg, np.asarray(p.s, float), np.asarray(p.vx, float), period=L) for p in lib])
    f = _periodic_library_interpolant("v", sg, F, L)
    err = []
    for j, p in enumerate(lib):
        r = ref_from_plan(p)
        q = np.asarray(r["s"], float)[::7]
        v = np.array([float(f(np.array([qq, j]))) for qq in q])
        err.append(float(np.max(np.abs(v - np.interp(q, np.asarray(r["s"]), np.asarray(r["vx"]), period=L)))))
    _write("plans.json", {"etas": etas.tolist(), "lap_time_s": [float(p.lap_time) for p in lib],
                          "same_s_grid_as_0.95": [bool(len(p.s) == len(sg) and np.allclose(p.s, sg)) for p in lib],
                          "interpolant_vs_plan_vx_max_abs": err,
                          "new_plans_log": {str(k): {kk: vv for kk, vv in v.items() if kk != "solves"} | {
                              "solves": len(v.get("solves", []))} for k, v in d["log"].items()},
                          "plans_file_commit": d.get("commit"), "plans_file_dirty": d.get("dirty")})


def _dev_jobs() -> list[tuple]:
    from run_cx_d import _jobs, _names

    j = []
    for k, g in cx_d.GRIDS.items():
        for s_ in g:
            j += _jobs("dev", k, s_, ["NOM_car0"])
    j += _jobs("dev", "adaptive_oracle", {}, ["NOM_car0"]) + _jobs("dev", "nmpc_param", {"lam_mu": 0.995}, ["NOM_car0"])
    j += _jobs("dev", "nmpc", {}, ["NOM_car0"])
    dev = _names("DEV_C", "DEV_D")
    j += _jobs("dev", "nmpc", {}, dev) + _jobs("dev", "adaptive", {"lam_mu": 0.995}, dev)
    j += _jobs("dev", "adaptive_oracle", {}, dev)
    return j


def laps(workers: int) -> None:
    from run_cx_d import _run

    _run(_dev_jobs(), DEV / "laps.jsonl", workers)
    (DEV / "laps_meta.json").write_text(json.dumps(_meta(), indent=1))


def gate() -> None:
    from report_cx_a import load

    recs = {(r["kind"], r["setting_key"], r["case"]): r for r in load(DEV / "laps.jsonl") if "error" not in r}
    done = lambda k, s, c: bool(recs[(k, s, c)]["completed"]) if (k, s, c) in recs else None
    dd = [c["name"] for c in cx_d.cases("DEV_D")]
    oracle_dd = [done("adaptive_oracle", "default", c) for c in dd]
    ad_nom = {g_["lam_mu"]: done("adaptive", cx_d.setting_key(g_), "NOM_car0") for g_ in cx_d.ADAPTIVE_GRID}
    res = {"adaptive_oracle_NOM": done("adaptive_oracle", "default", "NOM_car0"),
           "adaptive_oracle_DEV_D": f"{sum(bool(x) for x in oracle_dd)} of {len(dd)}",
           "adaptive_NOM_by_lam_mu": ad_nom,
           "nmpc_DEV_D (background, review n2)": f"{sum(bool(done('nmpc', 'default', c)) for c in dd)} of {len(dd)}",
           "adaptive_DEV_D (lam_mu 0.995)": f"{sum(bool(done('adaptive', 'lam_mu=0.995', c)) for c in dd)} of {len(dd)}"}
    res["gate_passed"] = bool(res["adaptive_oracle_NOM"] and sum(bool(x) for x in oracle_dd) >= 4 and any(ad_nom.values()))
    _write("gate.json", res)


def _replay_one(args) -> dict:
    """Item 4: one recorded lap (state, applied inputs) fed to an estimator offline."""
    rec_path, case_name, kind, setting = args
    import pickle

    from f1race.dynamics import DynVehicle
    from f1race.estimation.bank import ModelBank
    from f1race.estimation.rls import RecursiveLS
    from f1race.track import Track

    from run_cx_d import _case

    with open(rec_path, "rb") as fh:
        lap = pickle.load(fh)
    case = _case(case_name)
    tr = Track.load("Spielberg")
    th_true = cx_d.true_theta(case)
    X, U = lap["state"], lap["inputs"]
    field = None
    if case["drift"] is not None:
        d = case["drift"]
        field = cx_d.DriftField(tr.length, d["s_on"], d["ell"], d["f_end"])
    s, _ = tr.project(X[:, 0], X[:, 1])
    est = (RecursiveLS(DynVehicle(), cx_d.DT, lam_mu=setting["lam_mu"]) if kind == "rls"
           else ModelBank(DynVehicle(), cx_d.DT, window=setting["W"]))
    th_hist, mu_plan = [], []
    for k in range(len(X) - 1):
        est.update(X[k, 3:7], U[k], X[k + 1, 3:6])
        th_hist.append(est.theta.copy())
        mu_plan.append(est.mu)
    th_hist = np.array(th_hist)
    f = np.array([float(field(q)) for q in s[1:]]) if field is not None else np.ones(len(th_hist))
    truth = np.tile(th_true, (len(th_hist), 1))
    truth[:, 0] *= f
    err = th_hist - truth
    tol = np.where(np.arange(7) == 6, 0.02, 0.02 * np.abs(truth[0]))          # 2 % (aero_front: 0.02 absolute)
    prog = lap["progress"][1:]
    conv = []
    for i in range(7):
        bad = np.flatnonzero(np.abs(err[:, i]) >= tol[i])
        conv.append(0.0 if not bad.size else (None if bad[-1] == len(err) - 1 else float(prog[bad[-1] + 1])))
    out = {"lap": case_name, "recorded_with": lap["kind"], "estimator": kind, "setting": setting,
           "lap_distance": float(prog[-1]), "converged_within_2pct_from_m": dict(zip(
               ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front"), conv)),
           "err_end": err[-1].tolist(), "mu_abs_err_p50_p95": [float(np.percentile(np.abs(err[:, 0]), q)) for q in (50, 95)]}
    if field is not None:
        q = cx_d.lap_position(np.unwrap(s, period=tr.length), tr.length)[1:]    # not mod L (the start is at L - 1e-11)
        on = q >= case["drift"]["s_on"]
        if on.any():
            i0 = int(np.flatnonzero(on)[0])
            a = np.abs(err[i0:, 0])
            b = np.flatnonzero(a >= 0.02)
            out["drift_settle_delay_s"] = (0.0 if not b.size else (None if b[-1] == len(a) - 1 else (b[-1] + 1) * cx_d.DT))
            out["mu_abs_err_after_onset_p50_p95_max"] = [float(np.percentile(a, 50)), float(np.percentile(a, 95)),
                                                         float(a.max())]
    out.update(est.diagnostics())
    return out


def _record_lap(args) -> str:
    """A lap of ``kind`` on ``case`` with its state and applied inputs kept (for the replays)."""
    import pickle

    from f1race.track import Track

    from run_cx_d import _case

    case_name, kind, setting = args
    case = _case(case_name)
    tr = Track.load("Spielberg")
    sim = cx_d.make_sim(case, tr)
    ctl, mpc, x0, _ = cx_d.build(kind, setting, tr, case, sim)
    res = sim.run(ctl, x0, t_max=cx_d.T_MAX)
    p = DEV / "replay_laps" / f"{kind}_{case_name}.pkl"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "wb") as fh:
        pickle.dump({"kind": kind, "case": case_name, "state": res.state, "inputs": res.inputs, "progress": res.progress,
                     "completed": bool(res.completed)}, fh)
    return str(p)


def replay(workers: int) -> None:
    rec = [(c, "nmpc", {}) for c in ["NOM_car0"] + [c["name"] for c in cx_d.cases("DEV_C")]]
    rec += [(c["name"], "adaptive_oracle", {}) for c in cx_d.cases("DEV_D")]
    ctx = get_context("spawn")
    with ctx.Pool(min(workers, len(rec)), maxtasksperchild=1) as pool:
        paths = pool.map(_record_lap, rec)
    jobs = [(p, c, "rls", s_) for p, (c, _, _) in zip(paths, rec) for s_ in cx_d.ADAPTIVE_GRID]
    jobs += [(p, c, "bank", s_) for p, (c, _, _) in zip(paths, rec) for s_ in cx_d.BANK_GRID]
    with ctx.Pool(min(workers, len(jobs)), maxtasksperchild=1) as pool:
        out = pool.map(_replay_one, jobs)
    _write("replay.json", {"replays": out})


class _StartFixDrift(cx_d.DriftField):
    """The drift field with the start state's projection (s = L - 1e-11) counted as the start of the lap, not its end."""

    def __call__(self, s):
        q = np.mod(np.asarray(s, float), self.length)
        q = np.where(q > self.length - 1e-3, 0.0, q)
        w = np.clip((q - self.s_on) / self.ell, 0.0, 1.0)
        out = 1.0 - (1.0 - self.f_end) * w
        return float(out) if np.ndim(out) == 0 else out


def _rerun_one(args) -> dict:
    """One CDD lap of the benchmark again, with the drift field as run (``original``) or with the start fixed
    (``startfix``)."""
    variant, kind, setting, case_name = args
    from run_cx_d import _case

    if variant == "startfix":
        cx_d.DriftField = _StartFixDrift
    t0 = time.time()
    try:
        rec = cx_d.lap(_case(case_name), kind, setting)
    except Exception:                                   # noqa: BLE001
        import traceback
        return {"variant": variant, "kind": kind, "case": case_name, "error": traceback.format_exc()}
    rec.update(variant=variant, wall_job_s=time.time() - t0)
    return rec


def cdd_rerun(workers: int, records: Path) -> None:
    """Packet 4: the start of every drift lap projects to s = L - 1e-11, so the drift field gave f_end for the first
    simulator step and the onset diagnostics counted from the lap start. (a) ``adaptive`` on CDD again with the field as
    run: the laps must reproduce test.jsonl, the diagnostics are now from the onset; (b) nmpc, adaptive,
    adaptive_oracle and tube on CDD with the start fixed: do the outcomes change?"""
    from report_cx_a import load

    names = [c["name"] for c in cx_d.cases("CDD")]
    jobs = [("original", "adaptive", {"lam_mu": 0.995}, n) for n in names]
    jobs += [("startfix", k, {"lam_mu": 0.995} if k == "adaptive" else {}, n)
             for k in ("nmpc", "adaptive", "adaptive_oracle", "tube") for n in names]
    path = DEV / "cdd_rerun.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if path.exists():
        done = {(r["variant"], r["kind"], r["case"]) for r in map(json.loads, path.read_text().splitlines())
                if "error" not in r}
    todo = [j for j in jobs if (j[0], j[1], j[3]) not in done]
    print(len(todo), "laps", flush=True)
    if todo:
        with get_context("spawn").Pool(min(workers, len(todo)), maxtasksperchild=1) as pool, \
                open(path, "a", encoding="utf-8") as fo:
            for rec in pool.imap_unordered(_rerun_one, todo):
                fo.write(json.dumps(rec, default=float) + "\n")
                fo.flush()
    recs = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    test = {(r["kind"], r["case"]): r for r in load(records)}
    keys = ("completed", "lap_time", "stop_reason", "distance", "failure_s", "min_edge_margin", "tyre_usage_max_front_rear",
            "plan_line_offset_abs_mean_max", "mu_abs_err_p50_p95", "too_fast_share")
    out = {"records": str(records), "errors": sum("error" in r for r in recs), "original_same_as_test": 0,
           "original_diff": [], "startfix": {}}
    for r in recs:
        if "error" in r:
            continue
        o = test[(r["kind"], r["case"])]
        if r["variant"] == "original":
            bad = [k for k in keys if json.dumps(r.get(k)) != json.dumps(o.get(k))]
            if bad:
                out["original_diff"].append(f"{r['case']}: {bad}")
            else:
                out["original_same_as_test"] += 1
        else:
            s_ = out["startfix"].setdefault(r["kind"], {"laps": 0, "completed_startfix": 0, "completed_test": 0,
                                                         "outcome_changed": [], "lap_time_diff_max_abs": 0.0})
            s_["laps"] += 1
            s_["completed_startfix"] += int(r["completed"])
            s_["completed_test"] += int(o["completed"])
            if r["completed"] != o["completed"]:
                s_["outcome_changed"].append(r["case"])
            elif r["completed"]:
                s_["lap_time_diff_max_abs"] = max(s_["lap_time_diff_max_abs"], abs(r["lap_time"] - o["lap_time"]))
    orig = [r for r in recs if r.get("variant") == "original" and "error" not in r]
    v = [r.get("drift_settle_delay_s") for r in orig]
    ok = [x for x in v if x is not None]
    med = lambda a: float(np.median(a)) if a else None
    out["adaptive_onset_diagnostics"] = {
        "settle_delay_s_median_min_max": [med(ok), min(ok), max(ok)] if ok else None, "never": len(v) - len(ok),
        "completed_settle_median": med([r["drift_settle_delay_s"] for r in orig if r["completed"]
                                        and r["drift_settle_delay_s"] is not None]),
        "failed_settle_median": med([r["drift_settle_delay_s"] for r in orig if not r["completed"]
                                     and r["drift_settle_delay_s"] is not None]),
        "abs_err_p95_before_during_after_median": [float(np.nanmedian([r["mu_abs_err_before_during_after_p50_p95"][i][1]
                                                                       for r in orig])) for i in range(3)],
        "per_lap": [{"case": r["case"], "completed": r["completed"], "settle_s": r["drift_settle_delay_s"],
                     "p95_before_during_after": [x[1] for x in r["mu_abs_err_before_during_after_p50_p95"]],
                     "after_onset_max": r.get("mu_abs_err_after_onset_max")} for r in sorted(orig, key=lambda r: r["car"])]}
    _write("cdd_rerun.json", out)


def cdd_failures(path: Path) -> None:
    """Descriptive, after the primary report (review of packet 3): where on CDD each controller's laps end relative to
    the drift – before the onset, during the decline, after its end – and whether the drift is still under way at the
    corner at ~3.83 km (s_on + l > 3 830 m); for the adaptive controllers the settle delay of completed and failed laps."""
    from report_cx_a import load

    recs = [r for r in load(path) if "error" not in r and r["set"] == "CDD"]
    out = {"source": str(path), "kinds": {}}
    for k in cx_d.KINDS:
        rs = [r for r in recs if r["kind"] == k]
        rows = {"laps": len(rs), "completed": sum(r["completed"] for r in rs)}
        late = [r for r in rs if r["drift"]["s_on"] + r["drift"]["ell"] > 3830.0]
        rows["drift_under_way_at_3830m"] = {"laps": len(late), "completed": sum(r["completed"] for r in late)}
        rest = [r for r in rs if r["drift"]["s_on"] + r["drift"]["ell"] <= 3830.0]
        rows["drift_ended_before_3830m"] = {"laps": len(rest), "completed": sum(r["completed"] for r in rest)}
        where = {"before onset": 0, "during the decline": 0, "after its end": 0, "no position": 0}
        for r in rs:
            if r["completed"]:
                continue
            s, d = r.get("failure_s"), r["drift"]
            if s is None:
                where["no position"] += 1
            elif s < d["s_on"]:
                where["before onset"] += 1
            elif s < d["s_on"] + d["ell"]:
                where["during the decline"] += 1
            else:
                where["after its end"] += 1
        rows["failures"] = where
        rows["failure_s"] = sorted(round(r["failure_s"]) for r in rs if not r["completed"] and r.get("failure_s") is not None)
        if k in cx_d.ADAPTIVE_KINDS:
            for lab, sub in (("completed", [r for r in rs if r["completed"]]), ("failed", [r for r in rs if not r["completed"]])):
                v = [r.get("drift_settle_delay_s") for r in sub if "drift_settle_delay_s" in r]
                ok = [x for x in v if x is not None]
                rows[f"settle_delay_{lab}"] = {"n": len(v), "never": len(v) - len(ok),
                                               "median_s": float(np.median(ok)) if ok else None}
        out["kinds"][k] = rows
    _write("cdd_failures.json", out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["unchanged", "step", "nlp", "plans", "laps", "replay", "drifts", "gate",
                                     "cdd-failures", "cdd-rerun"])
    ap.add_argument("--records", default=None, help="cdd-failures: the test.jsonl to read")
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--base", default="55c5925")
    a = ap.parse_args()
    if a.what == "unchanged":
        unchanged(a.base)
    elif a.what == "step":
        step_check()
    elif a.what == "nlp":
        nlp_check()
    elif a.what == "plans":
        plans_check()
    elif a.what == "laps":
        laps(a.workers)
    elif a.what == "replay":
        replay(a.workers)
    elif a.what == "gate":
        gate()
    elif a.what == "cdd-failures":
        cdd_failures(Path(a.records))
    elif a.what == "cdd-rerun":
        cdd_rerun(a.workers, Path(a.records))
    else:
        _write("drifts.json", {"CDD": cx_d.drift_summary("CDD"), "CDV_D": cx_d.drift_summary("CDV_D"),
                               "CDD_draws": [c["drift"] for c in cx_d.cases("CDD")]})


if __name__ == "__main__":
    main()
