"""Control expansion phase A – code check of the secondary-condition paths and of mpcc_ca on cases outside every
protocol set (not a result): the nominal car with the 30 ms lag, the nominal car under the L3 disturbances of seed
9999 (L7's smoke seed, not 7400-7429), the nominal car on Monza (an L8 validation track, not a test track), and
mpcc_ca on the nominal car. One process per lap.

    python scripts/cx_a_secondary_smoke.py      # -> results/CX/A/dev/secondary_smoke.jsonl
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from multiprocessing import get_context
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

MPCC = {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}
JOBS = [("DEV_lag30", "Spielberg", 0.03, None, "nmpc", {"c": 1.0, "margin": 0.3}),
        ("DEV_lag30", "Spielberg", 0.03, None, "mpcc", MPCC),
        ("DEV_dist9999", "Spielberg", 0.0, 9999, "mpcc", MPCC),
        ("DEV_Monza", "Monza", 0.0, None, "mpcc", MPCC),
        ("DEV_nominal", "Spielberg", 0.0, None, "mpcc_ca", {**MPCC, "alpha": 3.0, "w_ca": 0.1})]


def one(j):
    from f1race.dynamics import DynVehicle
    from f1race.experiments.cx_a import lap

    name, track, tau, dist, kind, setting = j
    case = {"set": "DEV", "car": 0, "info": {"nominal": True}, "plant": DynVehicle(), "name": name,
            "track": track, "tau": tau, "dist_seed": dist}
    r = lap(case, kind, setting)
    git = lambda *a: subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    r["commit"], r["dirty"] = git("rev-parse", "HEAD"), bool(git("status", "--porcelain", "--untracked-files=no"))
    return r


if __name__ == "__main__":
    out = ROOT / "results" / "CX" / "A" / "dev" / "secondary_smoke.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with get_context("spawn").Pool(len(JOBS), maxtasksperchild=1) as pool, open(out, "w", encoding="utf-8") as fo:
        for r in pool.imap_unordered(one, JOBS):
            fo.write(json.dumps(r, default=float) + "\n")
            print(r["case"], r["kind"], r["stop_reason"], round(r["lap_time"], 3) if r["completed"] else round(r["distance"]),
                  r["t_max"], flush=True)
