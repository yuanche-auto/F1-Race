"""Control expansion phase F – development checks (docs/CX_F_protocol.md section 9; NOM and FD only).
`.venv-rl`; records in results/CX/F/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_f_dev.py monitor          # 9.1: plan states accepted, pushed states rejected; check time
    python scripts/cx_f_dev.py laps [--workers 20]   # 9.2–9.4: backup, shield (m 0.5; B_s1_500k, tracker), retry on NOM
    python scripts/cx_f_dev.py gate             # 9.5 and 9.6 (review m3) from laps.jsonl
    python scripts/cx_f_dev.py backup-plans [--workers 20]   # diagnostic after gate (i) failed: the bare tracker
                                                # at speed factors on the 0.95 plan, and on slower plans, NOM + FD
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
DEV = ROOT / "results" / "CX" / "F" / "dev"
M_DEV = 0.5


def meta(what: str) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (DEV / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def monitor() -> None:
    """9.1 on NOM: at 60 points of the 0.95 plan (its own states), the shield's check of the zero action;
    the same states pushed 3 m towards the left edge with a heading error of 0.15 rad towards it."""
    from f1race.experiments import cx_f, l7

    meta("monitor")
    env = l7.make_env()
    sh = cx_f.Shield(env, cx_f.ShieldConfig(margin=M_DEV))
    n = len(env.path.s)
    out = {"plan": [], "pushed": []}
    for i in np.linspace(0, n, 60, endpoint=False).astype(int):
        env.reset(options={"plant": cx_f.cases("NOM")[0]["plant"], "start_index": int(i)})
        y = np.array(env._meas, float)
        t0 = time.perf_counter()
        ok, info = sh.check(copy.copy(env.ctl), y, env.steer_max)
        dt = time.perf_counter() - t0
        out["plan"].append({"i": int(i), "s": float(env.path.s[i]), "ok": ok, "reason": info["reason"], "time_s": dt})
        yb = y.copy()
        c, s = np.cos(y[2]), np.sin(y[2])
        yb[0] -= 3.0 * s
        yb[1] += 3.0 * c
        yb[2] += 0.15
        ok2, info2 = sh.check(copy.copy(env.ctl), yb, env.steer_max)
        out["pushed"].append({"i": int(i), "ok": ok2, "reason": info2["reason"]})
    tm = np.array([r["time_s"] for r in out["plan"]])
    out["summary"] = {"plan_accepted": sum(r["ok"] for r in out["plan"]), "pushed_rejected":
                      sum(not r["ok"] for r in out["pushed"]), "n": 60, "check_ms_mean": float(tm.mean() * 1e3),
                      "check_ms_max": float(tm.max() * 1e3),
                      "plan_rejected_at": [(r["s"], r["reason"]) for r in out["plan"] if not r["ok"]]}
    (DEV / "monitor.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out["summary"], indent=1, default=float))


def lap_jobs() -> list[tuple]:
    from f1race.experiments import cx_f

    names = [c["name"] for c in cx_f.cases("NOM") + cx_f.cases("FD")]
    j = [("dev", "backup", "backup", n, "{}") for n in names]
    j += [("dev", "shield", p, n, json.dumps({"m": M_DEV})) for p in ("B_s1_500k", "tracker") for n in names]
    j += [("dev", "none", p, n, "{}") for p in ("B_s1_500k", "tracker") for n in names]
    j += [("dev", "retry", "B_s1_500k", "NOM", "{}")]
    return j


def laps(workers: int) -> None:
    import run_cx_f

    meta("laps")                                    # decision records: results/CX/F/dev_dec/ (stage "dev")
    run_cx_f._run(lap_jobs(), DEV / "laps.jsonl", workers)


def gate() -> None:
    recs = [json.loads(l) for l in (DEV / "laps.jsonl").read_text().splitlines() if l.strip()]
    recs = [r for r in recs if "error" not in r]
    by = {(r["condition"], r["policy"], r["case"]): r for r in recs}
    rows = []
    for (c, p, n), r in sorted(by.items()):
        rows.append({"condition": c, "policy": p, "case": n, "completed": r["completed"], "lap_time": r["lap_time"],
                     "stop": r["stop_reason"], "distance": round(r["distance"], 1),
                     "interventions_per_decision": (r["interventions"] / r["decisions"]) if r.get("decisions") else None,
                     "no_option": r.get("no_option"), "time_ms": r.get("time_ms"), "retry": r.get("retry")})
    nom_b = by.get(("backup", "backup", "NOM"))
    nom_s = by.get(("shield", "B_s1_500k", "NOM"))
    fd_b = [r for (c, p, n), r in by.items() if c == "backup" and n.startswith("FD")]
    g = {"i_backup_completes_NOM": bool(nom_b and nom_b["completed"]),
         "ii_shield_B_s1_500k_NOM": bool(nom_s and nom_s["completed"] and nom_s["interventions"] < 0.5 * nom_s["decisions"]),
         "ii_interventions_share": (nom_s["interventions"] / nom_s["decisions"]) if nom_s else None,
         "m3_backup_FD_completed": f"{sum(r['completed'] for r in fd_b)} / {len(fd_b)}",
         "m3_more_conservative_a_b_to_discuss": sum(r["completed"] for r in fd_b) < 6}
    mon = DEV / "monitor.json"
    if mon.exists():
        ms = json.loads(mon.read_text())["summary"]["check_ms_mean"]
        g["iii_check_ms_mean_one_process"] = ms
        g["iii_check_below_50ms"] = ms < 50.0
    g["passed"] = bool(g["i_backup_completes_NOM"] and g["ii_shield_B_s1_500k_NOM"] and g.get("iii_check_below_50ms"))
    (DEV / "gate.json").write_text(json.dumps({"gate": g, "laps": rows}, indent=1, default=float))
    print(json.dumps(g, indent=1, default=float))
    for r in rows:
        print(r["condition"], r["policy"], r["case"], r["completed"], r["stop"], r["distance"],
              None if r["interventions_per_decision"] is None else round(r["interventions_per_decision"], 3),
              r["no_option"])


SPEED_FACTORS = (1.0, 0.97, 0.95, 0.92, 0.9, 0.87, 0.85)
BACKUP_ETAS = (0.95, 0.93, 0.91, 0.88, 0.85, 0.82, 0.79, 0.76, 0.73)


def _plan(eta: float):
    from f1race.experiments.cx_d import plan_library

    etas, plans = plan_library()
    return plans[int(np.argmin(np.abs(etas - eta)))]


def backup_job(args):
    """One lap of the bare tracker: ("factor", k) on the 0.95 plan with the speed factor k held,
    or ("plan", eta) with zero action on the plan eta (its own feed-forward)."""
    kind, val, case_name = args
    from f1race.experiments import cx_f, l7
    from f1race.rl.env import RacingEnv

    t0 = time.time()
    case = cx_f.case_by_name(case_name)
    if kind == "factor":
        env = l7.make_env()
        a1 = (val - 1.0) / 0.15
        pol = lambda obs: np.array([0.0, a1])           # noqa: E731
    else:
        env = RacingEnv(_plan(val), randomize_plant=False, history=8, steer_mode="angle", observe="command")
        pol = None
    r = l7.run_lap(pol, env, case, None)
    return {"job": f"{kind}|{val}|{case_name}", "kind": kind, "value": val, "case": case_name, "tau": case["tau"],
            "completed": r["completed"], "lap_time": r["lap_time"], "stop_reason": r["stop_reason"],
            "distance": r["distance"], "wall_s": time.time() - t0}


def backup_plans(workers: int) -> None:
    from multiprocessing import get_context

    from f1race.experiments import cx_f

    meta("backup_plans")
    names = [c["name"] for c in cx_f.cases("NOM") + cx_f.cases("FD")]
    jobs = [("factor", k, "NOM") for k in SPEED_FACTORS] + [("plan", e, n) for e in BACKUP_ETAS for n in names]
    with get_context("spawn").Pool(min(workers, len(jobs)), maxtasksperchild=1) as pool:
        recs = list(pool.imap_unordered(backup_job, jobs))
    path = DEV / "backup_plans.jsonl"
    path.write_text("".join(json.dumps(r, default=float) + chr(10) for r in recs))
    for kind, vals in (("factor", SPEED_FACTORS), ("plan", BACKUP_ETAS)):
        for v in vals:
            rs = [r for r in recs if r["kind"] == kind and r["value"] == v]
            nom = [r for r in rs if r["case"] == "NOM"]
            fd = [r for r in rs if r["case"] != "NOM"]
            laps = [r["lap_time"] for r in rs if r["completed"]]
            print(f"{kind} {v}: NOM {'yes' if nom and nom[0]['completed'] else 'no'} "
                  f"({nom[0]['stop_reason']} {nom[0]['distance']:.0f} m)" if nom else "",
                  f"FD {sum(r['completed'] for r in fd)} / {len(fd)}",
                  "by tau " + " ".join(f"{tau}:{sum(r['completed'] for r in fd if r['tau'] == tau and 'dist' not in r['case'])}"
                                       for tau in (0.0, 0.03, 0.06))
                  + f" dist:{sum(r['completed'] for r in fd if 'dist' in r['case'])}" if fd else "",
                  f"mean lap {np.mean(laps):.2f}" if laps else "")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["monitor", "laps", "gate", "backup-plans"])
    ap.add_argument("--workers", type=int, default=20)
    a = ap.parse_args()
    if a.what == "monitor":
        monitor()
    elif a.what == "laps":
        laps(a.workers)
    elif a.what == "backup-plans":
        backup_plans(a.workers)
    else:
        gate()


if __name__ == "__main__":
    main()
