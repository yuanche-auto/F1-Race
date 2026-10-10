"""Reproduce L6 round 1 (steering actuator lag; docs/L6_protocol.md).

Needs the L1 plans, the 12 % plan of L2 and the L3 calibration (results/L3/calibration.json).

    python scripts/run_l6.py plans                       # true-car plans (results/L6/oracle_plans.pkl)
    python scripts/run_l6.py round1 --taus 0.06          # laps -> results/L6/round1.jsonl (resumable)
    python scripts/run_l6.py round1 --taus 0.03,0.12 --controllers nmpc,explore,identified,oracle_param,oracle_struct
    python scripts/run_l6.py round1 --taus 0.06 --observe angle --controllers tracker,nmpc
    python scripts/run_l6.py report                      # tables from round1.jsonl and rl.jsonl
    python scripts/run_l6.py round2 --taus 0.06,0.03,0.01  # residual models + laps -> results/L6/round2.jsonl
    python scripts/run_l6.py round3                      # noise only: LS raw / smoothed / EKF -> round3.jsonl
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

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l2 import sample_plants  # noqa: E402
from f1race.experiments.l6 import (CONTROLLERS, NEEDS_EXPLORE, L6Config, config_dict, key_of,  # noqa: E402
                                   load_jsonl, oracle_plan_job, paired, run_job, run_round2_job, run_round3_job, safe_job,
                                   smpc_spec,
                                   summarize)

OUT = ROOT / "results" / "L6"
NL = chr(10)


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


def _errors(recs) -> bool:
    """Log failed jobs (error records of ``safe_job``); they are re-run on the next call."""
    bad = [r for r in recs if "error" in r]
    for r in bad:
        print("FAILED", r["car"], r["tau"], r["error"].splitlines()[-1], flush=True)
        with open(OUT / "errors.log", "a", encoding="utf-8") as fe:
            fe.write(f"{r['car']} {r['tau']}" + NL + r["error"] + NL)
    return bool(bad)


def round2(cfg: L6Config, taus, cars, observe: str) -> None:
    """Residual models from the saved exploration laps of round 1, and their laps."""
    plans = base_plans()
    plants = sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)
    path = OUT / "round2.jsonl"
    done = {(r["car"], r["tau"], r["observe"]) for r in load_jsonl(path)}
    jobs = []
    for t in taus:
        for c in cars:
            f = OUT / "explore" / f"explore_{c}_{t}_{observe}.npz"
            if (c, t, observe) in done:
                continue
            if not f.exists():
                print("missing exploration lap", f, flush=True)
                continue
            info, plant = plants[c]
            jobs.append({"car": c, "info": info, "plant": plant, "tau": t, "observe": observe,
                         "plans": plans, "explore_file": str(f)})
    print(f"{len(jobs)} round-2 jobs", flush=True)
    cfgd = config_dict(cfg)
    with get_context("spawn").Pool(cfg.workers) as pool, open(path, "a", encoding="utf-8") as fo:
        for recs in pool.imap_unordered(safe_job, [(run_round2_job, (j, cfgd)) for j in jobs]):
            if _errors(recs):
                continue
            for rec in recs:
                fo.write(json.dumps(rec) + "\n")
                cl = rec.get("closed_loop", {})
                print(f"tau {rec['tau']:.3f} car {rec['car']:2d} {rec['controller']:17s} {rec['stop_reason']:14s} "
                      f"lap {rec['lap_time']:.3f} dist {rec['distance']:.0f} closed-loop coverage "
                      f"{cl.get('coverage')}", flush=True)
            fo.flush()


def round3(cfg: L6Config, cars) -> None:
    """Noise only: three identifications from the noisy exploration lap of each car."""
    plans = base_plans()
    plants = sample_plants(DynVehicle(), cfg.n_plants, cfg.seed)
    path = OUT / "round3.jsonl"
    done = {r["car"] for r in load_jsonl(path)}
    jobs = [({"car": c, "info": plants[c][0], "plant": plants[c][1], "plans": plans}, config_dict(cfg))
            for c in cars if c not in done]
    print(f"{len(jobs)} round-3 jobs", flush=True)
    with get_context("spawn").Pool(cfg.workers) as pool, open(path, "a", encoding="utf-8") as fo:
        for recs in pool.imap_unordered(safe_job, [(run_round3_job, j) for j in jobs]):
            if _errors(recs):
                continue
            for rec in recs:
                fo.write(json.dumps(rec) + NL)
                print(f"car {rec['car']:2d} {rec['controller']:16s} {rec['stop_reason']:14s} lap {rec['lap_time']:.3f} "
                      f"max parameter error {rec['identification']['max_abs_error']:.3f}", flush=True)
            fo.flush()


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
    more = report_round2() + report_round3()
    if more:
        (OUT / "tables_rounds23.md").write_text("\n".join(more) + "\n", encoding="utf-8")
    print("\n".join(lines + more))


def _pool(recs, key):
    """Pooled (inside, n) per output of the coverage reports ``key`` of the records."""
    ins, n, ins_near, n_near = np.zeros(3), 0, np.zeros(3), 0
    for r in recs:
        c = r.get(key) or {}
        if not c.get("n"):
            continue
        ins += np.asarray(c["inside"]); n += c["n"]
        ins_near += np.asarray(c.get("inside_near", [0, 0, 0])); n_near += c.get("n_near", 0)
    return ins, n, ins_near, n_near


def _fmt_cov(ins, n) -> str:
    return " / ".join(f"{100 * x / n:.1f}" for x in ins) if n else "–"


def report_round2() -> list[str]:
    r1 = load_jsonl(OUT / "round1.jsonl")
    r2 = load_jsonl(OUT / "round2.jsonl")
    if not r2:
        return []
    res = r1 + r2
    lines = ["### Round 2 – residual learning (25 cars, laps completed, mean lap)", ""]
    taus = sorted({r["tau"] for r in r2})
    names = ["identified", "residual", "residual_cautious", "lag_identified", "oracle_struct"]
    lines += ["| controller | " + " | ".join(f"τ = {1000 * t:.0f} ms" for t in taus) + " |",
              "|---|" + "---|" * len(taus)]
    summ = summarize(res)
    for c in names:
        cells = []
        for t in taus:
            s_ = summ.get(f"{t}|command|{c}")
            cells.append("–" if s_ is None else
                         f"{s_['completed']}/{s_['runs']}" + (f", {s_['lap_mean']:.2f} s" if s_["lap_mean"] else ""))
        lines.append(f"| {c} | " + " | ".join(cells) + " |")
    lines += ["", "Coverage of the 95 % intervals in % (v_x' / v_y' / r'), pooled over the cars:", "",
              "| τ | held-out (exploration lap) | closed loop, `residual` | near-limit, `residual` | "
              "closed loop, `residual_cautious` | k-step ±b_k (k = 10 / 20), `residual_cautious` | go? |",
              "|---|---|---|---|---|---|---|"]
    verdicts = []
    for t in taus:
        rr = [r for r in r2 if r["tau"] == t and r["controller"] == "residual"]
        rc = [r for r in r2 if r["tau"] == t and r["controller"] == "residual_cautious"]
        h_i, h_n, _, _ = _pool(rr, "heldout")
        c_i, c_n, cn_i, cn_n = _pool(rr, "closed_loop")
        q_i, q_n, _, _ = _pool(rc, "closed_loop")
        ks = []
        for k in ("10", "20"):
            n = sum((r.get("kstep") or {}).get(k, {}).get("n", 0) for r in rc)
            i = sum((r.get("kstep") or {}).get(k, {}).get("inside", 0) for r in rc)
            ks.append(f"{100 * i / n:.1f}" if n else "–")
        ok = (c_n > 0 and cn_n > 0 and all(0.90 <= x / c_n <= 0.98 for x in c_i)
              and all(0.90 <= x / cn_n <= 0.98 for x in cn_i))
        verdicts.append(ok)
        lines.append(f"| {1000 * t:.0f} ms | {_fmt_cov(h_i, h_n)} (n = {h_n}) | {_fmt_cov(c_i, c_n)} (n = {c_n}) | "
                     f"{_fmt_cov(cn_i, cn_n)} (n = {cn_n}) | {_fmt_cov(q_i, q_n)} | {' / '.join(ks)} | "
                     f"{'go' if ok else 'no-go'} |")
    lines += ["", f"Go / no-go for L7 (protocol section 5, A4): **{'go' if verdicts and all(verdicts) else 'no-go'}**.", ""]
    lines += ["| τ | comparison (same cars) | only the first | only the second | exact McNemar p |",
              "|---|---|---|---|---|"]
    for t in taus:
        for a, b in (("residual", "identified"), ("residual_cautious", "residual"),
                     ("oracle_struct", "residual"), ("lag_identified", "residual")):
            pr = paired(res, t, a, b)
            if pr["cars"]:
                lines.append(f"| {1000 * t:.0f} ms | {a} vs {b} | {pr['only_a']} | {pr['only_b']} | {pr['p']:.3g} |")
    return lines + [""]


def report_round3() -> list[str]:
    from f1race.estimation.identify import PARAMS

    r3 = load_jsonl(OUT / "round3.jsonl")
    if not r3:
        return []
    summ = summarize(r3)
    lines = ["### Round 3 – noise only (τ = 0, 25 cars)", "",
             "| identification | laps completed | mean lap | largest parameter error (median over cars) | "
             "cars with a factor on its bound | mean error per factor (" + ", ".join(PARAMS) + ") |",
             "|---|---|---|---|---|---|"]
    for c in ("noisy_ls", "noisy_ls_smooth", "noisy_ekf"):
        rs = [r for r in r3 if r["controller"] == c]
        if not rs:
            continue
        s_ = summ[f"0.0|command|{c}"]
        err = np.array([r["identification"]["error"] for r in rs])
        mx = np.median([r["identification"]["max_abs_error"] for r in rs])
        nb = sum(r["identification"]["on_bounds"] > 0 for r in rs)
        lap = f"{s_['lap_mean']:.2f} s" if s_["lap_mean"] else "–"
        lines.append(f"| {c} | {s_['completed']}/{s_['runs']} | {lap} | {mx:.3f} | {nb} | "
                     + ", ".join(f"{e:+.3f}" for e in err.mean(axis=0)) + " |")
    lines += ["", "| comparison (same cars) | only the first | only the second | exact McNemar p |", "|---|---|---|---|"]
    for a, b in (("noisy_ls_smooth", "noisy_ls"), ("noisy_ekf", "noisy_ls")):
        pr = paired(r3, 0.0, a, b)
        lines.append(f"| {a} vs {b} | {pr['only_a']} | {pr['only_b']} | {pr['p']:.3g} |")
    return lines + [""]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["plans", "round1", "round2", "round3", "report"])
    ap.add_argument("--taus", default="0.03,0.06,0.12")
    ap.add_argument("--controllers", default=",".join(CONTROLLERS))
    ap.add_argument("--cars", default="all")
    ap.add_argument("--observe", default="command")
    ap.add_argument("--workers", type=int, default=int(os.environ.get("L6_WORKERS", 8)))
    ap.add_argument("--results", default=None, help="results directory (default results/L6)")
    a = ap.parse_args()
    if a.results:
        global OUT
        OUT = Path(a.results)
    cfg = L6Config(workers=a.workers)
    if a.what == "plans":
        oracle_plans(cfg, base_plans())
    elif a.what == "round3":
        round3(cfg, list(range(cfg.n_plants + 1)) if a.cars == "all" else [int(c) for c in a.cars.split(",")])
    elif a.what in ("round1", "round2"):
        cars = list(range(cfg.n_plants + 1)) if a.cars == "all" else [int(c) for c in a.cars.split(",")]
        taus = [float(t) for t in a.taus.split(",")]
        if a.what == "round1":
            round1(cfg, taus, a.controllers.split(","), cars, a.observe)
        else:
            round2(cfg, taus, cars, a.observe)
    else:
        report()


if __name__ == "__main__":
    main()
