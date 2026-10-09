"""Reproduce L6 round 1 (steering actuator lag; docs/L6_protocol.md).

Needs the L1 plans, the 12 % plan of L2 and the L3 calibration (results/L3/calibration.json).

    python scripts/run_l6.py plans                       # true-car plans (results/L6/oracle_plans.pkl)
    python scripts/run_l6.py round1 --taus 0.06          # laps -> results/L6/round1.jsonl (resumable)
    python scripts/run_l6.py round1 --taus 0.03,0.12 --controllers nmpc,explore,identified,oracle_param,oracle_struct
    python scripts/run_l6.py round1 --taus 0.06 --observe angle --controllers tracker,nmpc
    python scripts/run_l6.py report                      # tables from round1.jsonl and rl.jsonl
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import traceback
from collections import deque
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from multiprocessing import get_context
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l2 import sample_plants  # noqa: E402
from f1race.experiments.l6 import (CONTROLLERS, NEEDS_EXPLORE, L6Config, config_dict, key_of,  # noqa: E402
                                   load_jsonl, oracle_plan_job, paired, run_job, smpc_spec, summarize)

OUT = ROOT / "results" / "L6"


def base_plans() -> dict:
    cache = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))
    return {1.0: cache["plans"][1.0], 0.95: cache["plans"][0.95],
            0.88: pickle.load(open(ROOT / "results" / "L2" / "plan_088.pkl", "rb"))}


def oracle_plans(cfg: L6Config, plans: dict) -> dict:
    path = OUT / "oracle_plans.pkl"
    have = pickle.load(open(path, "rb")) if path.exists() else {}
    cars = sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)
    todo = [(k, v, plans, config_dict(cfg)) for k, (_, v) in enumerate(cars) if k not in have]
    if todo:
        with get_context("spawn").Pool(min(cfg.workers, len(todo))) as pool:
            for k, o, log in pool.imap_unordered(oracle_plan_job, todo):
                have[k] = {"plan": o, "log": log}
                print("oracle plan", k, log, flush=True)
                OUT.mkdir(parents=True, exist_ok=True)
                pickle.dump(have, open(path, "wb"))
    return have


def round1(cfg: L6Config, taus, controllers, cars, observe: str) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "config.json").write_text(json.dumps(config_dict(cfg), indent=1))
    plans = base_plans()
    sp = smpc_spec(json.loads((ROOT / "results" / "L3" / "calibration.json").read_text()))
    assert sp["plan_eta"] in plans                               # the 12 % plan (0.88)
    orc = oracle_plans(cfg, plans) if {"oracle_param", "oracle_struct"} & set(controllers) else {}
    plants = sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)
    path = OUT / "round1.jsonl"
    existing = load_jsonl(path)
    done = {key_of(r) for r in existing}
    cfgd = config_dict(cfg)

    def job(car, tau, name, **extra):
        info, plant = plants[car]
        j = {"car": car, "info": info, "plant": plant, "tau": tau, "observe": observe, "controller": name,
             "plans": plans, "smpc": sp, "round": 1, "save_dir": str(OUT / "explore")}
        if name in ("oracle_param", "oracle_struct"):
            j["oracle_plan"] = orc[car]["plan"]
        j.update(extra)
        return j

    def k(car, tau, name):
        return f"{car}|{tau}|{observe}|{name}"

    dependent = [c for c in NEEDS_EXPLORE if c in controllers]
    if dependent and "explore" not in controllers:
        controllers = ["explore"] + list(controllers)
    order = [c for c in ("explore",) + CONTROLLERS + ("lag_half", "lag_double")
             if c in controllers and c not in NEEDS_EXPLORE]
    order = list(dict.fromkeys(order))
    # tau-major order (the first tau of the list is finished first); at most `workers` jobs are
    # in flight, and the laps that need an exploration lap go to the front of the queue
    jobs = [job(c, t, n) for t in taus for n in order for c in cars if k(c, t, n) not in done]

    def after_explore(rec):
        """Jobs that use an exploration lap (its identified parameters or its saved data)."""
        out = []
        f = OUT / "explore" / f"explore_{rec['car']}_{rec['tau']}_{rec['observe']}.npz"
        for n in dependent:
            if k(rec["car"], rec["tau"], n) in done:
                continue
            if n == "identified":
                out.append(job(rec["car"], rec["tau"], n, theta=rec["identification"]["theta"]))
            elif f.exists():
                out.append(job(rec["car"], rec["tau"], n, explore_file=str(f)))
            else:
                print("no saved exploration lap for", n, rec["car"], rec["tau"], flush=True)
        return out

    for r in existing:                                       # exploration laps already run
        if (r["controller"] == "explore" and r["tau"] in taus and r["car"] in cars
                and r["observe"] == observe):
            jobs = after_explore(r) + jobs
    print(f"{len(jobs)} jobs (+ identification laps after the exploration laps)", flush=True)
    pending = deque(jobs)
    with ProcessPoolExecutor(cfg.workers, mp_context=get_context("spawn")) as ex, \
            open(path, "a", encoding="utf-8") as f:
        futs = {}

        def fill():
            while pending and len(futs) < cfg.workers:
                nj = pending.popleft()
                futs[ex.submit(run_job, (nj, cfgd))] = nj

        fill()
        while futs:
            fin, _ = wait(futs, return_when=FIRST_COMPLETED)
            for fu in fin:
                j = futs.pop(fu)
                try:
                    rec = fu.result()
                except Exception:                            # noqa: BLE001
                    msg = traceback.format_exc()
                    print("FAILED", j["car"], j["tau"], j["controller"], msg, flush=True)
                    with open(OUT / "errors.log", "a", encoding="utf-8") as fe:
                        fe.write(f"{j['car']} {j['tau']} {j['controller']}\n{msg}\n")
                    fill()
                    continue
                f.write(json.dumps(rec) + "\n")
                f.flush()
                print(f"tau {rec['tau']:.3f} car {rec['car']:2d} {rec['controller']:13s} {rec['stop_reason']:14s} "
                      f"lap {rec['lap_time']:.3f} dist {rec['distance']:.0f} ({rec['job_s']:.0f} s)", flush=True)
                if rec["controller"] == "explore":
                    pending.extendleft(reversed(after_explore(rec)))
            fill()


PAIRS = [("oracle_struct", "oracle_param"), ("lag_nominal", "nmpc"), ("lag_identified", "identified"),
         ("oracle_struct", "lag_identified"), ("identified", "nmpc"),
         ("identified", "oracle_param"), ("oracle_param", "nmpc"), ("smpc", "nmpc"), ("rl", "nmpc")]
LABELS = {"tracker": "L1 tracker, 5 % plan", "nmpc": "NMPC, nominal model, 5 %",
          "lag_identified": "NMPC, lag + parameters identified", "lag_half": "NMPC, nominal + lag model 0.5 τ",
          "lag_double": "NMPC, nominal + lag model 2 τ",
          "explore": "NMPC, nominal model, uniform 12 %", "smpc": "SMPC ε = 0.01 (L3)",
          "identified": "NMPC, identified model (L4)", "oracle_param": "NMPC, true parameters, no lag",
          "lag_nominal": "NMPC, nominal parameters + lag", "oracle_struct": "NMPC, true parameters + lag (correct structure)",
          "rl": "residual RL (L5, B_s1_500k)", "rl_env_tracker": "tracker in the RL environment (check)"}


def report() -> None:
    res = load_jsonl(OUT / "round1.jsonl") + load_jsonl(OUT / "rl.jsonl")
    summ = summarize(res)
    (OUT / "summary_round1.json").write_text(json.dumps(summ, indent=1))
    lines = []
    # 1a: the nominal car (car 0) at every tau
    nom = [r for r in res if r["car"] == 0 and r["observe"] == "command"]
    taus = sorted({r["tau"] for r in nom})
    lines += ["### 1a – nominal car (lap time, or where it stopped)", "",
              "| controller | " + " | ".join(f"{1000 * t:.0f} ms" for t in taus) + " |",
              "|---|" + "---|" * len(taus)]
    for c in LABELS:
        cells = []
        for t in taus:
            r = next((r for r in nom if r["tau"] == t and r["controller"] == c), None)
            cells.append("–" if r is None else (f"{r['lap_time']:.2f}" if r["completed"]
                                                else f"{r['stop_reason']} @ {r['distance']:.0f} m"))
        if any(x != "–" for x in cells):
            lines.append(f"| {LABELS[c]} | " + " | ".join(cells) + " |")
    lines.append("")
    # 1b: taus with all 25 cars
    for obs in sorted({r["observe"] for r in res}):
        full = sorted({r["tau"] for r in res if r["observe"] == obs
                       and summ[f"{r['tau']}|{obs}|{r['controller']}"]["runs"] >= 25})
        if not full:
            continue
        lines += [f"### 1b – 25 cars, observation: {obs} (laps completed, mean lap)", "",
                  "| controller | " + " | ".join(f"τ = {1000 * t:.0f} ms" for t in full) + " |",
                  "|---|" + "---|" * len(full)]
        for c in LABELS:
            cells = []
            for t in full:
                s_ = summ.get(f"{t}|{obs}|{c}")
                if s_ is None or s_["runs"] < 25:
                    cells.append("–")
                    continue
                ci = s_["completed_ci95"]
                lap = f", {s_['lap_mean']:.2f} s" if s_["lap_mean"] else ""
                cells.append(f"{s_['completed']}/{s_['runs']} ({100 * ci[0]:.0f}–{100 * ci[1]:.0f} %){lap}")
            if any(x != "–" for x in cells):
                lines.append(f"| {LABELS[c]} | " + " | ".join(cells) + " |")
        lines.append("")
        if obs == "command":
            lines += ["| τ | comparison (same cars) | only the first | only the second | exact McNemar p |",
                      "|---|---|---|---|---|"]
            for t in full:
                for a, b in PAIRS:
                    pr = paired(res, t, a, b)
                    if pr["cars"] >= 25:
                        lines.append(f"| {1000 * t:.0f} ms | {a} vs {b} | {pr['only_a']} | {pr['only_b']} | {pr['p']:.3g} |")
            lines.append("")
    (OUT / "tables_round1.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["plans", "round1", "report"])
    ap.add_argument("--taus", default="0.03,0.06,0.12")
    ap.add_argument("--controllers", default=",".join(CONTROLLERS))
    ap.add_argument("--cars", default="all")
    ap.add_argument("--observe", default="command")
    ap.add_argument("--workers", type=int, default=int(os.environ.get("L6_WORKERS", 8)))
    a = ap.parse_args()
    cfg = L6Config(workers=a.workers)
    if a.what == "plans":
        oracle_plans(cfg, base_plans())
    elif a.what == "round1":
        cars = list(range(cfg.n_plants + 1)) if a.cars == "all" else [int(c) for c in a.cars.split(",")]
        round1(cfg, [float(t) for t in a.taus.split(",")], a.controllers.split(","), cars, a.observe)
    else:
        report()


if __name__ == "__main__":
    main()
