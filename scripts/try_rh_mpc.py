"""Exploratory runs of the receding-horizon minimum-time MPC (docs/L2.md, section 4).

    python scripts/try_rh_mpc.py N ds nsub eta both_ends t_max [terminal_eta]
    e.g. python scripts/try_rh_mpc.py 50 4.0 3 0.95 1 30 0.95

Needs results/L1/l1_plans.pkl (scripts/run_l1.py). Prints how the run ended,
solver statistics and solve times.
"""
from __future__ import annotations

import pickle
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.control.rh_mpc import SpatialMinTimeMPC, SpatialMPCController  # noqa: E402
from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import x0_from_plan  # noqa: E402
from f1race.sim.dynamic import DynamicSim  # noqa: E402
from f1race.track import Track  # noqa: E402


def main(argv) -> None:
    N, ds, nsub, eta = int(argv[0]), float(argv[1]), int(argv[2]), float(argv[3])
    both, t_max = argv[4] == "1", float(argv[5])
    term_eta = float(argv[6]) if len(argv) > 6 else eta
    plans = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"]
    track, veh = Track.load("Spielberg"), DynVehicle()
    term, start = plans[term_eta], plans[0.95]
    mpc = SpatialMinTimeMPC(track, veh, {"s": term.s, "vx": term.vx}, N=N, ds=ds, nsub=nsub, eta=eta,
                            both_ends=both, max_iter=300)
    ctl = SpatialMPCController(mpc, 0.01, period=0.05)
    path, _ = dyn_reference(track, veh, start)
    t0 = time.time()
    r = DynamicSim(track, veh, 0.01).run(ctl, x0_from_plan(start, path), t_max=t_max)
    T = np.array([l[3] for l in ctl.log])
    st = [l[4] for l in ctl.log]
    print(f"N={N} ds={ds} nsub={nsub} eta={eta} both_ends={both} terminal eta={term_eta}: "
          f"{r.stop_reason} at t={r.t[-1]:.1f} s, {r.progress[-1]:.0f} m; usage {r.max_usage:.3f}; "
          f"edge margin {r.min_margin:.3f} m; solve mean {T.mean() * 1e3:.0f} ms, "
          f"p95 {np.percentile(T, 95) * 1e3:.0f} ms; wall {time.time() - t0:.0f} s")
    print({k: st.count(k) for k in set(st)})


if __name__ == "__main__":
    main(sys.argv[1:])
