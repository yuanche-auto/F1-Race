"""L8: closed-loop check of the plans of a validation track with the nominal car (protocol
section 12): the L1 tracker on the 5 % plan and the nominal NMPC (L2 settings, L6 failure
handling) – one lap each. Validation tracks only.

    python scripts/l8_validation_check.py Monza tracker      # or nmpc
"""
from __future__ import annotations

import json
import os
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.control.dyn_tracking import DynTracker  # noqa: E402
from f1race.control.nmpc import FrenetMPC, MPCController  # noqa: E402
from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import ref_from_plan, x0_from_plan  # noqa: E402
from f1race.experiments.l8 import VALIDATION, load_plans  # noqa: E402
from f1race.sim.dynamic import DynamicSim  # noqa: E402
from f1race.track import Track  # noqa: E402

if __name__ == "__main__":
    name, kind = sys.argv[1], sys.argv[2]
    if name not in VALIDATION:
        raise SystemExit("validation tracks only")
    track, nom = Track.load(name), DynVehicle()
    o = load_plans(name)["plans"][0.95]
    path, ff = dyn_reference(track, nom, o)
    if kind == "tracker":
        ctl = DynTracker(path, ff, nom, 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    else:
        mpc = FrenetMPC(track, nom, ref_from_plan(o), mode="track", N=30, dt=0.05, eta=0.95)
        ctl = MPCController(mpc, 0.01)
    t0 = time.time()
    r = DynamicSim(track, nom, 0.01).run(ctl, x0_from_plan(o, path), t_max=200.0)
    out = {"track": name, "controller": kind, "completed": bool(r.completed), "lap_time": float(r.lap_time),
           "plan_lap_time": float(o.lap_time), "stop_reason": r.stop_reason, "distance": float(r.progress[-1]),
           "max_usage": float(np.max(r.usage)), "wall_s": time.time() - t0}
    if kind == "nmpc":
        out["fallbacks"] = int(sum(x["fallback"] for x in mpc.stats))
        out["solves"] = len(mpc.stats)
    print(json.dumps(out), flush=True)
