"""Control expansion phase A – the diagnostic behind amendment A2 (not a result): CXV (tuning) cars with `mpcc`'s first
CXV choice (q_c 0.5, q_v 1, margin 0.6) and the hard or the soft speed bound; per lap the outcome, the fallbacks, the
solver statuses, the largest speed slack, and at the fallback solves the measured speed above the plan's bound.

    python scripts/cx_a_bound_diagnostic.py            # cars 2, 5, 7, hard and soft -> results/CX/A/dev/bound_diagnostic.jsonl
"""
from __future__ import annotations

import collections
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

FIRST_CHOICE = {"q_c": 0.5, "q_v": 1.0, "margin": 0.6}


def one(job):
    import numpy as np

    from f1race.experiments import cx_a
    from f1race.sim.dynamic import DynamicSim
    from f1race.track import Track

    car, soft = job
    case = next(c for c in cx_a.cases("CXV") if c["name"] == car)
    track, o = Track.load("Spielberg"), cx_a.plan()
    ctl, mpc, x0 = cx_a.build("mpcc", {**FIRST_CHOICE, "soft_bound": int(soft)}, track, o)
    over = []
    inner = ctl.__call__

    class C:
        def __call__(self, t, x):
            n0 = len(mpc.stats)
            u = inner(t, x)
            if len(mpc.stats) > n0 and mpc.stats[-1]["fallback"]:
                th = ctl.ctl.theta
                el0 = float(mpc.errors_fn(x[0], x[1], th)[1])
                over.append(float(x[3] - ctl.vref(th - el0)))
            return u

    res = DynamicSim(track, case["plant"], cx_a.DT).run(C(), x0, t_max=cx_a.T_MAX)
    git = lambda *a: subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    return {"car": car, "soft": soft, "completed": bool(res.completed), "stop_reason": res.stop_reason,
            "distance": float(res.progress[-1]), "lap_time": float(res.lap_time),
            "fallbacks": int(sum(s["fallback"] for s in mpc.stats)),
            "statuses": dict(collections.Counter(s["status"] for s in mpc.stats)),
            "max_speed_slack": float(max(s.get("speed_slack", 0.0) for s in mpc.stats)),
            "fallback_speed_over_bound_min_median_max": [float(np.min(over)), float(np.median(over)), float(np.max(over))]
            if over else None,
            "commit": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain", "--untracked-files=no"))}


if __name__ == "__main__":
    jobs = [(c, s) for c in ("CXV_car2", "CXV_car5", "CXV_car7") for s in (False, True)]
    out = ROOT / "results" / "CX" / "A" / "dev" / "bound_diagnostic.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(out, "w", encoding="utf-8") as fo:
        for r in pool.imap_unordered(one, jobs):
            fo.write(json.dumps(r) + "\n")
            print(r["car"], "soft" if r["soft"] else "hard", r["stop_reason"], round(r["distance"]), "fallbacks",
                  r["fallbacks"], r["statuses"], r["fallback_speed_over_bound_min_median_max"], flush=True)
