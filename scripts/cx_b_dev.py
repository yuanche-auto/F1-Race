"""Control expansion phase B – the development checks of docs/CX_B_protocol.md section 9, on the nominal car
only (records in results/CX/B/dev/, each with the commit and a dirty flag; not results of the protocol).

    python scripts/cx_b_dev.py checks          # 9.1 model, 9.2 cost / slack elimination, the drive-and-brake
                                               #     question on nmpc's nominal lap  -> dev/checks.json, dev/nmpc_nom.npz
    python scripts/cx_b_dev.py timing          # 9.5 one NOM lap per K, one after the other, each in a fresh
                                               #     process with nothing else running -> dev/timing.json (the K rule)
    python scripts/cx_b_dev.py gap --K K       # 9.4 optimality gap at 20 states of nmpc's nominal lap -> dev/gap.json
    python scripts/cx_b_dev.py laps --K K      # 9.6 the 9 grid settings and mppi_lp at the centre on NOM -> dev/laps.jsonl
    python scripts/cx_b_dev.py nomdiag --K K   # after 9.6 (no mppi setting completed NOM): white vs low-pass noise and
                                               #     more iterations per step, replicates 0-2 -> dev/nom_diagnostic.jsonl
    python scripts/cx_b_dev.py nomdiag-big --K K   # the same with 4 K samples -> dev/nom_diagnostic_big.jsonl
    python scripts/cx_b_dev.py timing-lp       # review c1 of packet 2: the K rule on mppi_lp, K 1024 / 2048 -> dev/timing_lp.json
    python scripts/cx_b_dev.py trace --K K     # review c2: the traces quoted in 9.7 -> dev/traces.npz, dev/traces.json
    python scripts/cx_b_dev.py gate-lp         # review g1 of packet 3: the 9.6 gate at K_RT -> dev/gate_k1024.jsonl
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
DEV = ROOT / "results" / "CX" / "B" / "dev"


def _git() -> dict:
    g = lambda *a: subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    return {"commit": g("rev-parse", "HEAD"), "dirty": bool(g("status", "--porcelain", "--untracked-files=no"))}


def _setup():
    from f1race.experiments import cx_a, cx_b
    from f1race.track import Track

    track, o = Track.load("Spielberg"), cx_a.plan()
    return cx_a, cx_b, track, o


def checks() -> None:
    import casadi as ca
    import numpy as np

    from f1race.control.mppi import MPPI
    from f1race.control.nmpc import FS
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import ref_from_plan
    from f1race.sim.dynamic import DynamicSim

    cx_a, cx_b, track, o = _setup()
    out = {"git": _git(), "date": time.strftime("%Y-%m-%d %H:%M")}
    ctl, mpc, x0 = cx_a.build("nmpc", cx_b.NMPC_SETTING, track, o)
    w = {k: v * cx_b.NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    mp = MPPI(track, DynVehicle(), ref_from_plan(o), N=cx_b.N, dt=cx_b.MPC_DT, eta=cx_b.ETA,
              margin=cx_b.NMPC_SETTING["margin"], weights=w, K=8)
    # 9.1 model
    rng = np.random.default_rng(91)
    M = 2000
    X = np.column_stack([rng.uniform(0, track.length, M), rng.uniform(-4, 4, M), rng.uniform(-0.15, 0.15, M),
                         rng.uniform(15, 90, M), rng.uniform(-2, 2, M), rng.uniform(-0.6, 0.6, M),
                         rng.uniform(-0.3, 0.3, M)])
    Uu = np.column_stack([rng.uniform(-1, 1, M), rng.uniform(0, 18e3, M), rng.uniform(-35e3, 0, M)])
    xn, _ = mp.step(X, Uu)
    ref = np.array([np.asarray(mpc._step(X[k], Uu[k] / np.array([1.0, FS, FS]))).ravel() for k in range(M)])
    out["model_max_rel_diff"] = float(np.max(np.abs(xn - ref) / np.maximum(np.abs(ref), 1.0)))
    out["model_max_abs_diff"] = float(np.max(np.abs(xn - ref)))
    # nmpc's nominal lap, every solve's z and parameters kept
    caps = []
    inner = mpc.solve

    def solve(x0_, u_prev, **kw):
        u, info = inner(x0_, u_prev, **kw)
        caps.append((np.asarray(x0_, float).copy(), np.asarray(u_prev, float).copy(), mpc.z.copy(), info["status"]))
        return u, info

    mpc.solve = solve
    res = DynamicSim(track, DynVehicle(), cx_b.DT).run(ctl, x0, t_max=cx_b.T_MAX)
    out["nmpc_nom_lap"] = {"completed": bool(res.completed), "lap_time": float(res.lap_time), "solves": len(caps),
                           "stop_reason": res.stop_reason}
    Uall = np.array([c[2][mpc.Ui] * np.array([1.0, FS, FS]) for c in caps])          # (solves, N, 3)
    both = np.minimum(Uall[..., 1], -Uall[..., 2])
    out["nmpc_drive_and_brake_at_once"] = {
        "share_of_predicted_stages_both_above_1N": float(np.mean(both > 1.0)),
        "share_of_applied_stage0_both_above_1N": float(np.mean(both[:, 0] > 1.0)),
        "max_min_drive_minus_brake_N": float(both.max()), "max_at_stage0_N": float(both[:, 0].max()),
        "p99_at_stage0_N": float(np.percentile(both[:, 0], 99))}
    np.savez_compressed(DEV / "nmpc_nom.npz", x0=np.array([c[0] for c in caps]), u_prev=np.array([c[1] for c in caps]),
                        U=Uall, status=np.array([c[3] for c in caps]))
    # 9.2 cost = the NLP's objective with the slacks eliminated, at 20 solves of the lap
    J = ca.Function("J", [mpc._nlp["x"], mpc._nlp["p"]], [mpc._nlp["f"]])
    G = ca.Function("G", [mpc._nlp["x"], mpc._nlp["p"]], [mpc._nlp["g"]])
    rows = []
    for i in np.linspace(1, len(caps) - 1, 20).astype(int):
        x0_, up, z, status = caps[i]
        p = np.concatenate([x0_, up / np.array([1.0, FS, FS])])
        Uz = z[mpc.Ui] * np.array([1.0, FS, FS])
        V = np.stack([Uz[:, 0], Uz[:, 1] + Uz[:, 2]], axis=1)[None]
        r = mp.rollout(x0_, V, up, detail=True)
        zz = z.copy()
        zz[mpc.Xi], zz[mpc.Ui], zz[mpc.Si], zz[mpc.Ti] = (r["X"][0], r["V3"][0] / np.array([1.0, FS, FS]),
                                                          r["sigma"][0], r["tau"][0])
        g = np.asarray(G(zz, p)).ravel()
        Uz3 = z[mpc.Ui] * np.array([1.0, FS, FS])
        V3 = np.stack([Uz3[:, 0], np.maximum(Uz3[:, 1] + Uz3[:, 2], 0), np.minimum(Uz3[:, 1] + Uz3[:, 2], 0)], axis=1)
        dyn = max(float(np.max(np.abs(np.asarray(mpc._step(z[mpc.Xi[k]], z[mpc.Ui[k]])).ravel() - z[mpc.Xi[k + 1]])))
                  for k in range(mpc.N))
        rows.append({"solve": int(i), "s": float(x0_[0]), "ipopt_status": status,
                     "rollout_rel_diff": abs(float(J(zz, p)) - float(r["cost"][0])) / max(abs(float(J(zz, p))), 1e-300),
                     "rollout_g_violation": float(max(np.max(mpc.lbg - g), np.max(g - mpc.ubg))),
                     "mppi_cost_of_ipopt_inputs": float(r["cost"][0]), "nlp_objective_at_ipopt_z": float(J(z, p)),
                     "ipopt_min_slack": float(min(z[mpc.Si].min(), z[mpc.Ti].min())),
                     "ipopt_dynamics_defect_max": dyn,
                     "inputs_changed_by_split": float(np.max(np.abs(V3 - Uz3)))})
    out["cost_identity"] = rows
    out["cost_identity_max_rollout_rel_diff"] = max(r_["rollout_rel_diff"] for r_ in rows)
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / "checks.json").write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "cost_identity"}, indent=1))
    for r_ in rows:
        print(r_)


def timing_one(K: int, kind: str = "mppi") -> dict:
    cx_a, cx_b, track, o = _setup()
    case = cx_b.cases("NOM")[0]
    setting = {**cx_b.GRID_CENTRE, "K": K, **({"f_c": cx_b.LP_FC_STAGE1} if kind == "mppi_lp" else {})}
    rec = cx_b.mppi_lap(case, kind, setting, replicate=0)
    rec.update(_git())
    return rec


def timing_lp() -> None:
    """Review c1 of packet 2 (amendment B2): the K rule of section 3.4 on the primary, mppi_lp at the grid centre
    (2 Hz), K = 1 024 and 2 048, one complete nominal lap each, one after the other in fresh processes."""
    from f1race.experiments import cx_b

    recs = []
    for K in (1024, 2048):
        p = subprocess.run([sys.executable, __file__, "timing-one", "--K", str(K), "--kind", "mppi_lp"], cwd=ROOT,
                           capture_output=True, text=True)
        if p.returncode != 0:
            raise RuntimeError(p.stderr[-2000:])
        rec = json.loads(p.stdout.strip().splitlines()[-1])
        recs.append(rec)
        print(K, rec["completed"], rec["lap_time"], rec["stop_reason"], round(rec["distance"]), rec["solves"], "steps;",
              "step ms mean/P50/P95/P99/max", [round(x, 1) for x in rec["step_ms_mean_p50_p95_p99_max"]],
              "share > 50 ms", round(rec["steps_over_50ms"], 3), flush=True)
    med = {r["setting"]["K"]: r["step_ms_mean_p50_p95_p99_max"][1] for r in recs}
    chosen = 2048 if med[2048] <= cx_b.K_BUDGET_MS else 1024
    (DEV / "timing_lp.json").write_text(json.dumps({"rule": "review c1: 2048 if its median step <= 40 ms, else 1024",
                                                    "chosen_K": chosen, "records": recs, **_git()}, indent=1))
    print("K_RT:", chosen)


def trace(K: int) -> None:
    """Review c2 of packet 2: the two laps whose traces section 9.7 quotes, recorded per simulator step and per
    MPPI step (mppi replicate 0 and mppi_lp at 2 Hz replicate 1; the grid centre, K) -> dev/traces.npz, and the
    quoted quantities -> dev/traces.json."""
    import numpy as np

    from f1race.dynamics import DynVehicle
    from f1race.sim.dynamic import DynamicSim

    cx_a, cx_b, track, o = _setup()
    L = track.length
    out, summary = {}, {}
    for name, kind, setting, rep in (("mppi_r0", "mppi", {**cx_b.GRID_CENTRE, "K": K}, 0),
                                     ("mppi_lp_r1", "mppi_lp", {**cx_b.GRID_CENTRE, "K": K, "f_c": 2.0}, 1)):
        ctl, mp, x0 = cx_b.build_mppi(kind, setting, track, o, cx_b.rng_seed(cx_b.cases("NOM")[0], rep))
        res = DynamicSim(track, DynVehicle(), cx_b.DT).run(ctl, x0, t_max=cx_b.T_MAX)
        X, U = res.state, res.inputs
        s, n = track.project(X[:, 0], X[:, 1])
        vref = cx_a._vref_fn(o, L)(s)
        st = mp.stats
        ess = np.array([q["ess"] for q in st])
        nofree = np.array([q["no_free"] for q in st])
        step_s = s[::5][: len(st)]
        prog = res.progress
        out.update({f"{name}_{k}": v for k, v in dict(
            t=res.t, s=s, n=n, vx=X[:, 3], vy=X[:, 4], r=X[:, 5], delta=X[:, 6], vref=vref, F_drive=U[:, 1],
            F_brake=U[:, 2], usage=res.usage, margin=res.margin, progress=prog, ess=ess, no_free=nofree,
            step_s=step_s).items()})
        straight = prog < 300.0
        # the last run of steps with no penalty-free sample that reaches the end of the lap
        k = len(nofree)
        while k > 0 and nofree[k - 1]:
            k -= 1
        win = (prog > 3700.0) & (prog < 3850.0)
        summary[name] = {"kind": kind, "setting": setting, "replicate": rep, "completed": bool(res.completed),
                         "distance": float(prog[-1]), "stop_reason": res.stop_reason,
                         "main_straight_speed_below_plan_max_mps": float(np.max((vref - X[:, 3])[straight])),
                         "main_straight_overlap_median_N": float(np.median(np.minimum(U[straight, 1], -U[straight, 2]))),
                         "steps_without_penalty_free_sample_until_the_end_from_s": (float(step_s[k]) if k < len(nofree)
                                                                                   else None),
                         "ess_from_there_max": float(ess[k:].max()) if k < len(nofree) else None,
                         "window_3700_3850m_rear_usage_max": float(res.usage[win, 1].max()) if win.any() else None,
                         "window_3700_3850m_vy_max": float(X[win, 4].max()) if win.any() else None}
        print(name, summary[name], flush=True)
    np.savez_compressed(DEV / "traces.npz", **out)
    (DEV / "traces.json").write_text(json.dumps({"K": K, "laps": summary, **_git()}, indent=1))


def timing() -> None:
    from f1race.experiments import cx_b

    recs = []
    for K in cx_b.K_CANDIDATES:                         # one after the other, each in a fresh process
        p = subprocess.run([sys.executable, __file__, "timing-one", "--K", str(K)], cwd=ROOT, capture_output=True,
                           text=True)
        if p.returncode != 0:
            raise RuntimeError(p.stderr[-2000:])
        rec = json.loads(p.stdout.strip().splitlines()[-1])
        recs.append(rec)
        print(K, rec["completed"], rec["lap_time"], rec["stop_reason"], round(rec["distance"]), "step ms",
              [round(x, 1) for x in rec["step_ms_mean_p50_p95_p99_max"]], flush=True)
    ok = [r["setting"]["K"] for r in recs if r["step_ms_mean_p50_p95_p99_max"][1] <= cx_b.K_BUDGET_MS]
    chosen = max(ok) if ok else None
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / "timing.json").write_text(json.dumps({"rule": "largest K with median step <= 40 ms (section 3.4)",
                                                 "chosen_K": chosen, "records": recs, **_git()}, indent=1))
    print("chosen K:", chosen, "(none: an amendment, clarification m1b)" if chosen is None else "")


def gap(K: int) -> None:
    import numpy as np

    from f1race.control.mppi import MPPI
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import ref_from_plan

    cx_a, cx_b, track, o = _setup()
    d = np.load(DEV / "nmpc_nom.npz")
    w = {k: v * cx_b.NMPC_SETTING["c"] for k, v in cx_a.NMPC_TRACK_W.items()}
    sig = tuple(cx_b.GRID_CENTRE["c_sigma"] * s for s in cx_b.SIGMA_BASE)
    rows = []
    for i in np.linspace(1, len(d["x0"]) - 1, 20).astype(int):
        x0, up, Ui, Uprev = d["x0"][i], d["u_prev"][i], d["U"][i], d["U"][i - 1]
        mp = MPPI(track, DynVehicle(), ref_from_plan(o), N=cx_b.N, dt=cx_b.MPC_DT, eta=cx_b.ETA,
                  margin=cx_b.NMPC_SETTING["margin"], weights=w, K=K, lam=cx_b.GRID_CENTRE["lam"], sigma=sig,
                  seed=(9, int(i), 0), diagnostics=False, inputs=cx_b.INPUTS)
        to_in = (lambda U3: U3) if mp.nu == 3 else (lambda U3: np.stack([U3[:, 0], U3[:, 1] + U3[:, 2]], axis=1))
        cost = lambda Uin: float(mp.rollout(x0, Uin[None], up)["cost"][0])
        # J*: the cost of IPOPT's inputs on the rollout (amendment B1.4); with three inputs they are not changed
        row = {"solve": int(i), "s": float(x0[0]), "ipopt_cost": cost(to_in(Ui)),
               "ipopt_inputs_changed_by_clamp": float(np.max(np.abs(mp.rollout(x0, to_in(Ui)[None], up)["V3"][0] - Ui)))}
        for name, U0 in (("warm", np.vstack([to_in(Uprev)[1:], np.zeros((1, mp.nu))])), ("zero", np.zeros((cx_b.N, mp.nu)))):
            mp.U = U0.copy()
            row[f"{name}_cost_0"] = cost(mp.U)
            done = 0
            for n_it in (1, 5, 50):
                while done < n_it:
                    mp.iterate(x0, up)
                    done += 1
                row[f"{name}_cost_{n_it}"] = cost(mp.U)
        rows.append(row)
        print(row, flush=True)
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / "gap.json").write_text(json.dumps({"K": K, "setting": cx_b.GRID_CENTRE, "inputs": cx_b.INPUTS, "rows": rows,
                                              **_git()}, indent=1))


def _lap_job(args):
    kind, setting = args
    from f1race.experiments import cx_b

    rec = cx_b.mppi_lap(cx_b.cases("NOM")[0], kind, setting, replicate=0)
    rec.update(_git())
    return rec


def laps(K: int) -> None:
    from f1race.experiments import cx_b

    jobs = [("mppi", {**g, "K": K}) for g in cx_b.MPPI_GRID] + [("mppi_lp", {**cx_b.GRID_CENTRE, "K": K, "f_c": 2.0})]
    DEV.mkdir(parents=True, exist_ok=True)
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(DEV / "laps.jsonl", "w") as fo:
        for r in pool.imap_unordered(_lap_job, jobs):
            fo.write(json.dumps(r) + "\n")
            fo.flush()
            print(r["kind"], r["setting_key"], r["completed"], r["lap_time"], r["stop_reason"], round(r["distance"]),
                  "step ms", [round(x, 1) for x in r["step_ms_mean_p50_p95_p99_max"]], "no-free",
                  round(r["no_penalty_free_sample_share"], 3), flush=True)


def _diag_job(args):
    kind, setting, rep = args
    from f1race.experiments import cx_b

    rec = cx_b.mppi_lap(cx_b.cases("NOM")[0], kind, setting, replicate=rep)
    rec.update(_git())
    return rec


def gate_lp() -> None:
    """Review g1 of packet 3: section 9.6's gate at the K in force - the 9 first-step settings of mppi_lp
    (lp_stage1(K_RT), 2 Hz) on the nominal car, replicate 0; development laps, no selection -> dev/gate_k1024.jsonl."""
    from f1race.experiments import cx_b

    jobs = [("mppi_lp", s_, 0) for s_ in cx_b.lp_stage1(cx_b.K_RT)]
    path = DEV / f"gate_k{cx_b.K_RT}.jsonl"
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(path, "w") as fo:
        for r in pool.imap_unordered(_diag_job, jobs):
            fo.write(json.dumps(r) + "\n")
            fo.flush()
            print(r["kind"], r["setting_key"], r["completed"], r["lap_time"], r["stop_reason"], round(r["distance"]),
                  "no-free", round(r["no_penalty_free_sample_share"], 3), "ess", [round(x, 1) for x in r["ess_p5_p50_p95"]],
                  flush=True)
    done = sum(json.loads(l)["completed"] for l in open(path))
    print(f"gate at K {cx_b.K_RT}: {done} of {len(jobs)} settings complete the nominal lap")


def nomdiag_big(K: int) -> None:
    """The nominal car: the grid centre with 4 K samples (offline, mppi_big's budget), white and low-pass (2 Hz)
    noise, replicates 0-2 - whether the failures of 9.6 / nomdiag are a matter of the sample budget."""
    from f1race.experiments import cx_b

    c = {**cx_b.GRID_CENTRE, "K": 4 * K}
    jobs = [("mppi_big", c, r) for r in (0, 1, 2)] + [("mppi_lp", {**c, "f_c": 2.0}, r) for r in (0, 1, 2)]
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(DEV / "nom_diagnostic_big.jsonl", "w") as fo:
        for r in pool.imap_unordered(_diag_job, jobs):
            fo.write(json.dumps(r) + "\n")
            fo.flush()
            print(r["kind"], r["setting_key"], "r", r["replicate"], r["completed"], r["lap_time"], r["stop_reason"],
                  round(r["distance"]), "no-free", round(r["no_penalty_free_sample_share"], 3), flush=True)


def nomdiag(K: int) -> None:
    """The nominal car only: the grid centre with white noise (mppi) and with low-pass noise (mppi_lp, f_c 1 / 2 / 4
    Hz), and with white noise but 4 iterations per step at K / 4 (about the same time per step); replicates 0-2
    (replicate 0 of mppi and of mppi_lp at 2 Hz repeats the 9.6 laps)."""
    from f1race.experiments import cx_b

    c = {**cx_b.GRID_CENTRE, "K": K}
    jobs = ([("mppi", c, r) for r in (0, 1, 2)] + [("mppi_lp", {**c, "f_c": 2.0}, r) for r in (0, 1, 2)]
            + [("mppi_lp", {**c, "f_c": f}, 0) for f in (1.0, 4.0)]
            + [("mppi", {**c, "K": K // 4, "iters": 4}, r) for r in (0, 1, 2)])
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(DEV / "nom_diagnostic.jsonl", "w") as fo:
        for r in pool.imap_unordered(_diag_job, jobs):
            fo.write(json.dumps(r) + "\n")
            fo.flush()
            print(r["kind"], r["setting_key"], "r", r["replicate"], r["completed"], r["lap_time"], r["stop_reason"],
                  round(r["distance"]), "step ms", [round(x, 1) for x in r["step_ms_mean_p50_p95_p99_max"]], "no-free",
                  round(r["no_penalty_free_sample_share"], 3), "overlap", [round(x) for x in r["overlap_N_p50_p90_max"]],
                  flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("checks", "timing", "timing-one", "timing-lp", "gap", "laps", "nomdiag",
                                     "nomdiag-big", "trace", "gate-lp"))
    ap.add_argument("--K", type=int)
    ap.add_argument("--kind", default="mppi")
    a = ap.parse_args()
    DEV.mkdir(parents=True, exist_ok=True)
    if a.what == "checks":
        checks()
    elif a.what == "timing":
        timing()
    elif a.what == "timing-one":
        print(json.dumps(timing_one(a.K, a.kind)))
    elif a.what == "timing-lp":
        timing_lp()
    elif a.what == "trace":
        trace(a.K)
    elif a.what == "gate-lp":
        gate_lp()
    elif a.what == "gap":
        gap(a.K)
    elif a.what == "nomdiag":
        nomdiag(a.K)
    elif a.what == "nomdiag-big":
        nomdiag_big(a.K)
    else:
        laps(a.K)
