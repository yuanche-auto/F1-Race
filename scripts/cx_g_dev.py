"""Control expansion phase G – development checks (docs/CX_G_protocol.md section 9; NOM and GD only), in WSL
(source ~/f1race-env.sh). Records in results/CX/G/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_g_dev.py regress       # the default path: offline RTI `nmpc` laps against L9's offline records
    python scripts/run_cx_g.py run --tag dev --sets NOM,GD                    # 9.3: explore, plan, the four primary arms
    python scripts/cx_g_dev.py equivalence   # 9.1 / gate (iii): offline, d = 0 and L_hat = 0: rti_dc == rti
    python scripts/cx_g_dev.py predictor     # 9.2: NOM, the exact (nominal) model, d = 20 ms: the prediction error
    python scripts/cx_g_dev.py gate          # 9.4 from dev/eval.jsonl, equivalence.json, predictor.json
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
DEV = ROOT / "results" / "CX" / "G" / "dev"
L9_OFFLINE = Path("/mnt/c/Users/yuanche/F1-Race-worktrees/F1-Race-wt-L9-run/results/L9/offline/model.jsonl")
REGRESS_CASES = ("T1new_car0_tau0", "T2new_dist6000")


def meta(what: str) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (DEV / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def regress() -> None:
    """Offline RTI `nmpc` laps (deterministic) against L9's offline records: the outcome, the distance and the lap
    time must be equal (the default path of experiments/l9 unchanged)."""
    from f1race.experiments import l9

    meta("regress")
    ref = {}
    for l in L9_OFFLINE.read_text().splitlines():
        r = json.loads(l)
        if r.get("kind") == "nmpc" and r.get("impl") == "rti" and r.get("case") in REGRESS_CASES and "error" not in r:
            ref[r["case"]] = r
    out = []
    for c in REGRESS_CASES:
        case = next(x for s in ("T1new", "T2new") for x in l9.cases(s) if x["name"] == c)
        r = l9.drive(case, "nmpc", "rti", "offline", name=f"cxg_regress_{os.getpid()}")
        o = ref[c]
        out.append({"case": c, "completed": [o["completed"], r["completed"]], "distance": [o["distance"], r["distance"]],
                    "lap_time": [o["lap_time"], r["lap_time"]],
                    "equal": o["completed"] == r["completed"] and abs(o["distance"] - r["distance"]) < 1e-9
                    and (not r["completed"] or abs(o["lap_time"] - r["lap_time"]) < 1e-9)})
    (DEV / "regress.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def _offline_pair(case: dict, kind: str, planned) -> dict:
    from f1race.experiments import l9

    ka, kb = {}, {}
    l9.drive(case, kind, "rti", "offline", name=f"cxg_eq_a_{os.getpid()}", planned=planned, keep=ka)
    l9.drive(case, kind, "rti", "offline", name=f"cxg_eq_b_{os.getpid()}", planned=planned,
             comp={"d": 0.0, "lhat": 0.01}, keep=kb)
    a, b = ka["run"], kb["run"]
    n = min(len(a.state), len(b.state))
    return {"case": case["name"], "kind": kind, "completed": [bool(a.completed), bool(b.completed)],
            "steps": [len(a.state), len(b.state)],
            "max_state_diff": float(np.max(np.abs(a.state[:n] - b.state[:n]))) if n else 0.0,
            "bit_identical": len(a.state) == len(b.state) and bool(np.array_equal(a.state, b.state))}


def equivalence() -> None:
    """9.1 / gate (iii): offline mode, d = 0, comp with L_hat (forced to 0 offline): the lap of `rti_dc` equals that of
    `rti` – `nmpc` on NOM, and `lag_identified` on NOM and the first GD case with the dev plans."""
    from f1race.experiments import cx_g

    meta("equivalence")
    out = [_offline_pair(cx_g.case_by_name("NOM"), "nmpc", None)]
    for c in ("NOM", "GD_car1_tau30"):
        planned = pickle.load(open(DEV / "plans" / f"{c}.pkl", "rb"))
        out.append(_offline_pair(cx_g.case_by_name(c), "lag_identified", planned))
    (DEV / "equivalence.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def predictor() -> None:
    """9.2: NOM, `nmpc` (the nominal model = the plant: the exact model, no lag, no noise), real time, d = 20 ms,
    compensated: the prediction error at t_k + L_hat (the L_hat step uses the plan's command before traction control,
    the plant gets it after – the only expected difference), and the same without compensation for scale."""
    from f1race.experiments import cx_g, l9

    meta("predictor")
    out = {}
    for arm, comp in (("comp", {"d": 0.02, "lhat": cx_g.LHAT}), ("none", None)):
        keep: dict = {}
        r = l9.drive(cx_g.case_by_name("NOM"), "nmpc", "rti", "realtime", delay=0.02, name=f"cxg_pred_{os.getpid()}",
                     comp=comp, log_assumed=True, keep=keep)
        out[arm] = {"completed": r["completed"], "distance": r["distance"],
                    "prediction_error": cx_g.prediction_errors(keep["calls"], keep["run"].state)}
    (DEV / "predictor.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def gate() -> None:
    recs = [json.loads(l) for l in (DEV / "eval.jsonl").read_text().splitlines() if l.strip()]
    recs = [r for r in recs if "error" not in r]
    by = {(r["arm"], r["case"]): r for r in recs}
    nom = by.get(("rti_dc_d20", "NOM"))
    eq = json.loads((DEV / "equivalence.json").read_text()) if (DEV / "equivalence.json").exists() else []
    lat = [r["latency_ms"][0] for r in recs if r["arm"] == "rti_dc_d20" and "latency_ms" in r]
    g = {"i_rti_dc_d20_completes_NOM": bool(nom and nom["completed"]),
         "ii_rti_dc_d20_mean_latency_ms": float(np.mean(lat)) if lat else None,
         "ii_below_10ms": bool(lat and np.mean(lat) < 10.0),
         "iii_offline_equivalence": bool(eq) and all(e["bit_identical"] or e["max_state_diff"] < 1e-9 for e in eq)}
    g["passed"] = g["i_rti_dc_d20_completes_NOM"] and g["ii_below_10ms"] and g["iii_offline_equivalence"]
    rows = {}
    for arm in ("rti_d0", "rti_dc_d0", "rti_d20", "rti_dc_d20"):
        rs = [r for (a, _), r in by.items() if a == arm]
        rows[arm] = {"laps": len(rs), "completed": sum(r["completed"] for r in rs),
                     "per_case": {r["case"]: (r["completed"], round(r["distance"], 1)) for r in rs}}
    (DEV / "gate.json").write_text(json.dumps({"gate": g, "arms": rows}, indent=1, default=float))
    print(json.dumps({"gate": g, "arms": rows}, indent=1, default=float))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["regress", "equivalence", "predictor", "gate"])
    a = ap.parse_args()
    {"regress": regress, "equivalence": equivalence, "predictor": predictor, "gate": gate}[a.what]()


if __name__ == "__main__":
    main()
