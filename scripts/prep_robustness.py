"""Numerical failures of L6, reproduced before any fix (ROADMAP section 7, point 2).

The failing cases are rebuilt from the records in ``results/L6_final`` (identified
parameters and tau as logged; the true cars from ``sample_plants``), so no exploration
lap has to be re-run.

    python scripts/prep_robustness.py replan             # the three re-plans that failed from both warm starts
    python scripts/prep_robustness.py burst --case 20,0.03,oracle_struct
    python scripts/prep_robustness.py list               # the cases
    python scripts/prep_robustness.py burst --case 20,0.03,oracle_struct --variant retry --trial 3

Output: ``results/prep/<kind>_<case>[_<variant>_t<trial>].json`` (per-solve statistics of
the NMPC, the re-plan log).

The failures turned out to depend on rounding (three of them did not recur on the new
machine), so a fix is judged by failure rates over repeats of the same case: trial k > 0
perturbs the input by a relative 1e-9 (initial state of the lap) or 1e-10 (warm-start
plans of a re-plan, as the L1 test of the first solve), with seed k; trial 0 is unperturbed.
Variants: lap ``old`` (L6 settings), ``retry`` (``FrenetMPC(retry=True)``), ``brake``
(``fallback="brake"`` only), ``retry_brake`` (both); re-plan ``old`` (``l4.replan``), ``robust`` (``replan_robust``).
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import time
from dataclasses import replace

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.control.nmpc import FrenetMPC, LagAwareMPCController, MPCController  # noqa: E402
from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.estimation.identify import vehicle_from  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import ref_from_plan, sample_plants, x0_from_plan  # noqa: E402
from f1race.experiments.l6 import L6Config, config_dict, load_jsonl, make_sim, replan, replan_slot  # noqa: E402
from f1race.planning.replan import replan_robust  # noqa: E402
from f1race.track import Track  # noqa: E402

FINAL = ROOT / "results" / "L6_final"
OUT = ROOT / "results" / "prep"

# (round file, car, tau, controller): the re-plans that failed from both warm starts
REPLAN_CASES = [("round1", 20, 0.06, "lag_identified"), ("round3", 20, 0.0, "noisy_ekf"),
                ("round3", 21, 0.0, "noisy_ekf")]
# laps that left the track after a burst of failed NMPC solves (docs/L6.md, sections 2, 3)
BURST_CASES = [("round1", 20, 0.06, "lag_identified"),        # plan failed in L6 (re-plan, robust)
               ("round3", 20, 0.0, "noisy_ekf"), ("round3", 21, 0.0, "noisy_ekf"),   # plan failed in L6
               ("round1", 0, 0.12, "lag_identified"), ("round1", 20, 0.03, "oracle_struct"),
               ("round1", 20, 0.06, "oracle_struct"), ("round1", 4, 0.06, "oracle_struct"),
               ("round1", 18, 0.06, "lag_identified"), ("round1", 21, 0.06, "lag_identified")]


def base_plans() -> dict:
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    return {1.0: cache["plans"][1.0], 0.95: cache["plans"][0.95]}


def record(f: str, car: int, tau: float, ctl: str) -> dict:
    for r in load_jsonl(FINAL / f"{f}.jsonl"):
        if r["car"] == car and r["tau"] == tau and r["controller"] == ctl and r["observe"] == "command":
            return r
    raise KeyError((f, car, tau, ctl))


def model_of(rec: dict) -> tuple[DynVehicle, float | None]:
    """The controller's model as in the run: identified parameters (and tau), or the true car."""
    nom = DynVehicle()
    if "identification_lag" in rec:
        return vehicle_from(rec["identification_lag"]["theta"], nom), rec["identification_lag"]["tau"]
    if "identification" in rec and rec["controller"].startswith("noisy"):
        return vehicle_from(rec["identification"]["theta"], nom), rec["identification"].get("tau")
    raise ValueError(rec["controller"])


def perturbed_plan(o, trial: int, rel: float = 1e-10):
    if trial == 0:
        return o
    rng = np.random.default_rng(trial)
    kw = {k: getattr(o, k) * (1.0 + rel * rng.standard_normal(len(o.s)))
          for k in ("n", "xi", "vx", "vy", "r", "delta", "f_drive", "f_brake")}
    return replace(o, **kw)


def run_replan(case, variant: str = "old", trial: int = 0) -> dict:
    f, car, tau, name = case
    rec = record(*case)
    veh, _ = model_of(rec)
    plans = base_plans()
    eta = L6Config().eta
    warm = [("nominal plan", perturbed_plan(plans[eta], trial)), ("ideal plan", perturbed_plan(plans[1.0], trial))]
    t0 = time.time()
    track = Track.load("Spielberg")
    if variant == "old":
        o, log = replan(track, veh, warm, eta)
    else:
        with replan_slot():
            o, log = replan_robust(track, veh, warm, eta, nominal=DynVehicle())
    return {"case": list(case), "variant": variant, "trial": trial, "original_log": rec.get("plan_log"),
            "log": log, "success": o is not None, "plan_lap_time": None if o is None else o.lap_time,
            "wall_s": time.time() - t0}


def plan_for(case, track, model, cfg):
    """The re-planned reference of an identified-model case, computed once and cached
    (``replan_robust``: the same plan as ``l4.replan`` whenever that one succeeds)."""
    path = OUT / f"plan_{case[0]}_{case[1]}_{case[2]}_{case[3]}.pkl"
    if path.exists():
        return pickle.load(open(path, "rb"))
    plans = base_plans()
    with replan_slot():
        o, log = replan_robust(track, model, [("nominal plan", plans[cfg["eta"]]), ("ideal plan", plans[1.0])],
                               cfg["eta"], nominal=DynVehicle())
    pickle.dump((o, log), open(path, "wb"))
    return o, log


def run_burst(case, variant: str = "old", trial: int = 0) -> dict:
    f, car, tau, name = case
    rec = record(*case)
    cfg = config_dict(L6Config())
    track = Track.load(cfg["track"])
    info, plant = sample_plants(DynVehicle(), cfg["n_plants"], cfg["seed"])[car]
    if name == "oracle_struct":
        model, lag = plant, tau
        plan = pickle.load(open(ROOT / "results" / "L6" / "oracle_plans.pkl", "rb"))[car]["plan"]
    else:
        model, lag = model_of(rec)
        plan, _ = plan_for(case, track, model, cfg)
        lag = max(lag, 1e-4) if tau > 0 else None       # round 3: no lag, no lag model
    path, _ = dyn_reference(track, model, plan)
    x0 = x0_from_plan(plan, path)
    if trial:
        x0 = x0 * (1.0 + 1e-9 * np.random.default_rng(trial).standard_normal(len(x0)))
    sim = make_sim(track, plant, tau, cfg, "command")
    opts = {"old": {}, "retry": {"retry": True}, "brake": {"fallback": "brake"},
            "retry_brake": {"retry": True, "fallback": "brake"}}[variant]
    mpc = FrenetMPC(track, model, ref_from_plan(plan), mode="track", N=cfg["N"], dt=cfg["mpc_dt"],
                    eta=cfg["eta"], steer_lag=lag, **opts)
    ctl = (LagAwareMPCController(mpc, cfg["dt"], record=False, observe="command") if lag
           else MPCController(mpc, cfg["dt"]))
    t0 = time.time()
    r = sim.run(ctl, x0, t_max=cfg["t_max"])
    solves = [{"t": l[0], "s": l[1], "n": l[2], **st} for l, st in zip(ctl.log, mpc.stats)]
    return {"case": list(case), "variant": variant, "trial": trial, "original": {k: rec.get(k) for k in ("stop_reason", "distance", "fallbacks",
                                                                     "solves", "lap_time")},
            "now": {"stop_reason": r.stop_reason, "distance": float(r.progress[-1]),
                    "fallbacks": int(sum(s["fallback"] for s in mpc.stats)), "solves": len(mpc.stats),
                    "retried": int(sum(len(s.get("attempts", [])) > 1 for s in mpc.stats)),
                    "lap_time": r.lap_time,
                    "solve_ms": [float(np.mean([s["time"] for s in mpc.stats]) * 1e3),
                                 float(np.max([s["time"] for s in mpc.stats]) * 1e3)]},
            "solves": solves, "wall_s": time.time() - t0}


def report() -> None:
    """Tables of all trials in results/prep (markdown)."""
    rows = [json.loads(p.read_text()) for p in sorted(OUT.glob("*.json"))]
    print("| lap case | variant | trials | completed | off / spin at (m) | fallbacks per lap | retried solves per lap | lap time (s) |")
    print("|---|---|---|---|---|---|---|---|")
    laps: dict = {}
    for r in rows:
        if "now" in r:
            laps.setdefault((tuple(r["case"]), r.get("variant", "old")), []).append(r)
    for (case, v), rs in sorted(laps.items(), key=lambda kv: (kv[0][0][1:], kv[0][1])):
        rs.sort(key=lambda r: r.get("trial", 0))
        done = [r for r in rs if r["now"]["stop_reason"] == "lap completed"]
        pos = [f"{r['now']['distance']:.0f}" for r in rs if r["now"]["stop_reason"] != "lap completed"]
        fb = ", ".join(str(r["now"]["fallbacks"]) for r in rs)
        rt = ", ".join(str(r["now"].get("retried", "-")) for r in rs)
        lt = ", ".join(f"{r['now']['lap_time']:.2f}" for r in done) or "-"
        print(f"| car {case[1]}, {1e3 * case[2]:.0f} ms, {case[3]} | {v} | {len(rs)} | {len(done)} | "
              f"{', '.join(pos) or '-'} | {fb} | {rt} | {lt} |")
    print()
    print("| re-plan case | variant | trials | succeeded | solves per trial | succeeded at | plan lap (s) |")
    print("|---|---|---|---|---|---|---|")
    plans: dict = {}
    for r in rows:
        if "log" in r and "now" not in r:
            plans.setdefault((tuple(r["case"]), r.get("variant", "old")), []).append(r)
    for (case, v), rs in sorted(plans.items()):
        ok = [r for r in rs if r["success"]]
        steps = sorted({r["log"][-1].get("step", "direct") + " / " + r["log"][-1]["warm"] for r in ok})
        lt = ", ".join(f"{r['plan_lap_time']:.2f}" for r in ok if r.get("plan_lap_time")) or "-"
        print(f"| car {case[1]}, {1e3 * case[2]:.0f} ms, {case[3]} | {v} | {len(rs)} | {len(ok)} | "
              f"{', '.join(str(len(r['log'])) for r in rs)} | {'; '.join(steps) or '-'} | {lt} |")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["replan", "burst", "list", "report"])
    ap.add_argument("--case", help="car,tau,controller (default: all cases of that kind)")
    ap.add_argument("--variant", default="old")
    ap.add_argument("--trial", type=int, default=0)
    a = ap.parse_args()
    if a.what == "list":
        print("replan:", REPLAN_CASES, "\nburst:", BURST_CASES)
        return
    if a.what == "report":
        report()
        return
    cases = REPLAN_CASES if a.what == "replan" else BURST_CASES
    if a.case:
        c, t, n = a.case.split(",")
        cases = [x for x in cases if x[1:] == (int(c), float(t), n)]
    OUT.mkdir(parents=True, exist_ok=True)
    for case in cases:
        fn = run_replan if a.what == "replan" else run_burst
        res = fn(case, a.variant, a.trial)
        tag = f"{a.what}_{case[0]}_{case[1]}_{case[2]}_{case[3]}"
        if a.variant != "old" or a.trial:
            tag += f"_{a.variant}_t{a.trial}"
        (OUT / f"{tag}.json").write_text(json.dumps(res, indent=1, default=float))
        print(tag, {k: v for k, v in res.items() if k not in ("solves",)}, flush=True)


if __name__ == "__main__":
    main()
