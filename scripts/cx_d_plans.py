"""The plan library of control expansion phase D (docs/CX_D_protocol.md section 3.3): L1's dynamic minimum-time plans
of the nominal car at the friction margins eta_p in {1.00, 0.97, 0.95, 0.93} (L1 cache) and 0.88 (L2's 12 % plan) are
reused; the new margins 0.91, 0.85, 0.82, 0.79, 0.76, 0.73, 0.70, 0.67 are solved here with ``replan_robust`` for the
nominal car, each warm-started from the nearest plan above it (0.91 from 0.93, 0.85 from 0.88, then each from the
previous new one). A plan that fails is left out and logged (protocol: dropped from the grid, reported before any CDV
lap). Output: results/CX/D/plans/plans_cxd.pkl (git-ignored) with the planning code's fingerprint.

    python scripts/cx_d_plans.py
"""
from __future__ import annotations

import json
import os
import pickle
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments import cx_d  # noqa: E402
from f1race.experiments.l1 import code_fingerprint  # noqa: E402
from f1race.experiments.l8 import plans_for  # noqa: E402
from f1race.planning.replan import replan_robust  # noqa: E402
from f1race.track import Track  # noqa: E402

CHAIN = ((0.91, 0.93), (0.85, 0.88), (0.82, 0.85), (0.79, 0.82), (0.76, 0.79), (0.73, 0.76), (0.70, 0.73), (0.67, 0.70))


def main() -> None:
    track, nom = Track.load("Spielberg"), DynVehicle()
    have = plans_for("Spielberg")
    new, logs = {}, {}
    for eta, src in CHAIN:
        warm = new.get(src, have.get(src))
        t0 = time.time()
        if warm is None:
            logs[eta] = {"skipped": f"no warm start (eta {src} failed)"}
            print(eta, "skipped", flush=True)
            continue
        o, log = replan_robust(track, nom, [(f"eta {src}", warm)], eta, nominal=nom)
        logs[eta] = {"warm": src, "solves": log, "minutes": (time.time() - t0) / 60.0, "ok": o is not None}
        print(eta, "ok" if o is not None else "FAILED", f"{(time.time() - t0) / 60:.1f} min",
              f"lap {o.lap_time:.3f} s" if o is not None else "", flush=True)
        if o is not None:
            new[eta] = o
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts"], cwd=ROOT, capture_output=True,
                                text=True).stdout.strip())
    out = {"plans": new, "log": logs, "code": code_fingerprint(), "commit": commit, "dirty": dirty}
    p = cx_d.PLANS_PATH
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "wb") as fh:
        pickle.dump(out, fh)
    (p.parent / "plans_cxd_log.json").write_text(json.dumps(
        {str(k): v for k, v in logs.items()} | {"commit": commit, "dirty": dirty}, indent=1, default=str))
    print("written", p, "plans:", sorted(new), flush=True)


if __name__ == "__main__":
    main()
