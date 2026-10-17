"""Solve-time benchmark of the candidate L7 safety filters (docs/L7_filter_benchmark.md).

    .venv-rl/Scripts/python scripts/l7_filter_bench.py collect          # RL laps -> results/L7_bench/states_*.npz
    python scripts/l7_filter_bench.py bench --filter qp:qpoases --lap 0  # one filter on one lap -> json
    python scripts/l7_filter_bench.py bench --filter nmpc --lap 1 --tag load
    python scripts/l7_filter_bench.py report

Laps: 0 = nominal car, no lag; 1 = L2 car 5, tau 30 ms; 2 = L2 car 12, tau 60 ms.
Filters: ``qp:<solver>`` (CBFQPFilter) and ``nmpc`` / ``nmpc_lag`` (FrenetMPC(mode="filter"),
without / with the steering-lag model; ``nmpc_lag`` only on laps with a lag).
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "L7_bench"
LAPS = [(0, 0.0), (5, 0.03), (12, 0.06)]          # (car of sample_plants(nominal, 24, 0), tau)
POLICY = "B_s1_500k"


def plan95():
    return pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]


def collect() -> None:
    """Drive the policy around each lap and log, at every decision, the state the
    controller sees, the true state and the first command of tracker + policy."""
    from stable_baselines3 import SAC

    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.rl.env import RacingEnv

    model = SAC.load(ROOT / "models" / "L5" / POLICY, device="cpu")
    history = (model.observation_space.shape[0] - 2) // 19
    OUT.mkdir(parents=True, exist_ok=True)
    for k, (car, tau) in enumerate(LAPS):
        env = RacingEnv(plan95(), randomize_plant=False, history=history, steer_mode="angle", observe="command")
        _, plant = sample_plants(DynVehicle(), 24, 0)[car]
        env.episode_s = 120.0
        opts = {"plant": plant, "start_index": 0}
        if tau:
            opts["steer_lag"] = tau
        obs, _ = env.reset(options=opts)
        rec = {"x_seen": [], "x_true": [], "u": [], "t": [], "action": []}
        first = {"on": False}
        orig = env.sim.apply_limits

        def apply_limits(x, u):                      # log the first command after each decision
            if first["on"]:
                rec["x_seen"].append(np.array(env._view(x), float))
                rec["x_true"].append(np.array(x, float))
                rec["u"].append(np.array(u, float))
                rec["t"].append(env.t)
                first["on"] = False
            return orig(x, u)
        env.sim.apply_limits = apply_limits
        done, L = False, env.track.length
        while not done:
            a = model.predict(obs, deterministic=True)[0]
            rec["action"].append(np.array(a, float))
            first["on"] = True
            obs, _, term, trunc, info = env.step(a)
            done = term or trunc or env.progress >= L
        out = {key: np.array(v) for key, v in rec.items()}
        np.savez_compressed(OUT / f"states_{k}.npz", car=car, tau=tau, completed=env.progress >= L,
                            reason=info.get("reason", ""), **out)
        print(f"lap {k}: car {car} tau {tau} decisions {len(rec['u'])} completed {env.progress >= L} "
              f"{info.get('reason', '')}", flush=True)


def changed(u_out, u) -> bool:
    """The filter changed the command: steering rate by more than 0.01 rad/s or a force by
    more than 100 N (above the solvers' tolerances)."""
    d = np.abs(np.asarray(u_out, float) - np.asarray(u, float))
    return bool(d[0] > 0.01 or d[1] > 100.0 or d[2] > 100.0)


def bench(filt: str, lap: int, tag: str, limit: int | None) -> dict:
    from types import SimpleNamespace

    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import ref_from_plan
    from f1race.track import Track

    z = np.load(OUT / f"states_{lap}.npz")
    X, U = z["x_seen"], z["u"].copy()
    tau = float(z["tau"])
    nom = DynVehicle()
    # the tracker's command is limited by the plant's actuators before it acts (steering rate
    # and angle, engine power: ``DynamicSim.apply_limits``; all cars here have the nominal
    # limits): the filters get the command as it would act
    from f1race.sim.dynamic import DynamicSim

    lim = DynamicSim.apply_limits
    U = np.array([lim(SimpleNamespace(veh=nom), x, u) for x, u in zip(X, U)])
    if limit:
        X, U = X[:limit], U[:limit]
    track = Track.load("Spielberg")
    plan = plan95()
    times, fails, moved = [], 0, 0
    t_build = time.perf_counter()
    if filt.startswith("qp:"):
        from f1race.control.safety_filter import CBFQPFilter

        f = CBFQPFilter(track, nom, eta=0.95, solver=filt[3:])
        ctl = MPCController(SimpleNamespace(track=track), 0.01)        # only for its projection
        t_build = time.perf_counter() - t_build
        for x, u in zip(X, U):
            t0 = time.perf_counter()
            s, n, xi_h = ctl.frenet(x)[:3]
            xi = xi_h + np.arctan2(x[4], max(x[3], 1.0))             # course angle
            _, info = f.filter((s % track.length, n, xi, xi_h), x, u)
            times.append(time.perf_counter() - t0)
            fails += not info["ok"]
            moved += changed(info["u"], u)
    else:
        lag = tau if filt == "nmpc_lag" else None
        if filt == "nmpc_lag" and not tau:
            raise ValueError("nmpc_lag needs a lap with a lag")
        if filt == "nmpc10_b":                       # A6 candidate (2): N = 10 with (b), k <= 10
            est = json.loads((ROOT / "results" / "L7" / "K" / "estimates.json").read_text())
            mpc = FrenetMPC(track, nom, ref_from_plan(plan), mode="filter", N=10, dt=0.05,
                            eta=est["b"]["eta"][:10], backoff=est["b"]["backoff"][:11])
        else:
            mpc = FrenetMPC(track, nom, ref_from_plan(plan), mode="filter", N=30, dt=0.05, eta=0.95,
                            steer_lag=lag)           # L6 failure handling (docs/prep_robustness.md)
        ctl = MPCController(mpc, 0.01)
        t_build = time.perf_counter() - t_build
        u_prev = np.zeros(3)
        d_hat = None
        for x, u in zip(X, U):
            t0 = time.perf_counter()
            xf = ctl.frenet(x)
            if lag:                                   # commanded angle in the slot: open-loop observer
                d_hat = x[6] if d_hat is None else x[6] + (d_hat - x[6]) * np.exp(-0.05 / lag)
                xf = np.concatenate([xf[:6], [d_hat, x[6]]])
            u_out, info = mpc.solve(xf, u_prev, u_des=u)
            times.append(time.perf_counter() - t0)
            fails += info["fallback"]
            moved += changed(u_out, u)
            u_prev = u_out
    t = np.array(times) * 1e3
    res = {"filter": filt, "lap": lap, "tag": tag, "calls": len(t), "build_s": t_build,
           "mean_ms": float(t.mean()), "median_ms": float(np.median(t)), "p95_ms": float(np.percentile(t, 95)),
           "p99_ms": float(np.percentile(t, 99)), "max_ms": float(t.max()), "fail_frac": fails / len(t),
           "intervention_frac": moved / len(t), "pid": os.getpid()}
    OUT.mkdir(parents=True, exist_ok=True)
    name = f"bench_{filt.replace(':', '-')}_lap{lap}_{tag}_{os.getpid()}.json"
    (OUT / name).write_text(json.dumps(res, indent=1))
    print(res, flush=True)
    return res


def report() -> None:
    rows = [json.loads(p.read_text()) for p in sorted(OUT.glob("bench_*.json"))]
    groups: dict = {}
    for r in rows:
        groups.setdefault((r["filter"], r["tag"]), []).append(r)
    print("| filter | load | calls | mean ms | median | P95 | P99 | max | failed | changed |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for (f, tag), rs in sorted(groups.items()):
        n = sum(r["calls"] for r in rs)
        w = lambda k: sum(r[k] * r["calls"] for r in rs) / n          # noqa: E731
        print(f"| {f} | {tag} ({len(rs)} runs) | {n} | {w('mean_ms'):.2f} | {np.median([r['median_ms'] for r in rs]):.2f} "
              f"| {max(r['p95_ms'] for r in rs):.2f} | {max(r['p99_ms'] for r in rs):.2f} | "
              f"{max(r['max_ms'] for r in rs):.1f} | {100 * w('fail_frac'):.2f} % | {100 * w('intervention_frac'):.1f} % |")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["collect", "bench", "report"])
    ap.add_argument("--filter", default="qp:qpoases")
    ap.add_argument("--lap", type=int, default=0)
    ap.add_argument("--tag", default="alone")
    ap.add_argument("--limit", type=int, default=None)
    a = ap.parse_args()
    if a.what == "collect":
        collect()
    elif a.what == "bench":
        bench(a.filter, a.lap, a.tag, a.limit)
    else:
        report()


if __name__ == "__main__":
    main()
