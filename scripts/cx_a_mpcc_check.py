"""Control expansion, phase A – development check of the dynamic MPCC (not a benchmark): one closed-loop
lap of Spielberg with the nominal car from the start of the 5 % plan, for the MPCC and, on the same lap,
the existing NMPC (``FrenetMPC``, tracking mode) for reference. Code check before the phase-A protocol;
nothing here is a pre-registered result.

    python scripts/cx_a_mpcc_check.py mpcc      # -> results/CX/A/dev/dev_check_mpcc_centre_terminal.json
    python scripts/cx_a_mpcc_check.py nmpc      # -> results/CX/A/dev/dev_check_nmpc.json
    python scripts/cx_a_mpcc_check.py mpcc --reference raceline --speed-bound stages
    (options: --reference centre|raceline, --speed-bound none|terminal|stages)
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from f1race.control.mpcc import DynamicMPCC, MPCCController, path_errors  # noqa: E402
from f1race.control.nmpc import FrenetMPC, MPCController  # noqa: E402
from f1race.dynamics import DynVehicle  # noqa: E402
from f1race.experiments.l1 import dyn_reference  # noqa: E402
from f1race.experiments.l2 import ref_from_plan, x0_from_plan  # noqa: E402
from f1race.experiments.l8 import plans_for  # noqa: E402
from f1race.sim.dynamic import DynamicSim  # noqa: E402
from f1race.track import Track  # noqa: E402

DT, ETA, T_MAX = 0.01, 0.95, 121.0
OUT = ROOT / "results" / "CX" / "A" / "dev"


def git_state() -> dict:
    def run(*a):
        return subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    return {"commit": run("rev-parse", "HEAD"), "dirty": bool(run("status", "--porcelain", "--untracked-files=no"))}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("ctrl", choices=["mpcc", "nmpc"])
    ap.add_argument("--reference", default="centre", choices=["centre", "raceline"])
    ap.add_argument("--speed-bound", default="terminal", choices=["none", "terminal", "stages"])
    ap.add_argument("--qc", type=float, default=None, help="contouring weight (default of DynamicMPCC)")
    ap.add_argument("--margin", type=float, default=0.3, help="track margin [m] (the NMPC's 0.3)")
    a = ap.parse_args()
    tr, veh = Track.load("Spielberg"), DynVehicle()
    o = plans_for("Spielberg")[ETA]
    path, _ = dyn_reference(tr, veh, o)
    x0 = x0_from_plan(o, path)
    t0 = time.time()
    if a.ctrl == "mpcc":
        mpc = DynamicMPCC(tr, veh, ref_from_plan(o), eta=ETA, reference=a.reference,
                          speed_bound=a.speed_bound, margin=a.margin,
                          weights=None if a.qc is None else {"q_c": a.qc})
        ctl = MPCCController(mpc, DT)
    else:
        mpc = FrenetMPC(tr, veh, ref_from_plan(o), eta=ETA)
        ctl = MPCController(mpc, DT)
    build = time.time() - t0
    res = DynamicSim(tr, veh, DT).run(ctl, x0, t_max=T_MAX)
    st = mpc.stats
    ts = np.array([s["time"] for s in st]) * 1e3
    X = res.state
    s_c, n_c = tr.project(X[:, 0], X[:, 1])
    ec, _ = path_errors(tr, X[:, 0], X[:, 1], s_c)
    out = {
        "ctrl": a.ctrl + (f"_{a.reference}_{a.speed_bound}" + (f"_qc{a.qc:g}" if a.qc is not None else "")
                          + (f"_m{a.margin:g}" if a.margin != 0.3 else "") if a.ctrl == "mpcc" else ""),
        "completed": bool(res.completed),
        "lap_time": float(res.lap_time), "stop_reason": res.stop_reason, "distance": float(res.progress[-1]),
        "solves": len(st), "build_s": build, "wall_s": time.time() - t0,
        "solve_ms_mean_p95_p99_max": [float(ts.mean()), float(np.percentile(ts, 95)),
                                      float(np.percentile(ts, 99)), float(ts.max())],
        "fallbacks": int(sum(bool(s.get("fallback")) for s in st)),
        "not_converged": int(sum(s["status"] not in ("Solve_Succeeded", "Solved_To_Acceptable_Level") for s in st)),
        "max_track_slack": float(max(s["slack"] for s in st)),
        "min_edge_margin_m": float(np.min(res.margin)),
        "centre_line_offset_abs_mean_max": [float(np.mean(np.abs(ec))), float(np.max(np.abs(ec)))],
        "tyre_usage_max_front_rear": [float(np.max(res.usage[:, 0])), float(np.max(res.usage[:, 1]))],
        "steer_rate_rms": float(np.sqrt(np.mean(res.inputs[:, 0] ** 2))),
        "eta": ETA, "N": mpc.N, "dt_mpc": mpc.dt, **git_state(),
    }
    if a.ctrl == "mpcc":
        out["weights"] = mpc.w
        out["reference"], out["speed_bound"], out["margin"] = mpc.reference, mpc.speed_bound, a.margin
        out["failure_xy"] = [float(X[-1, 0]), float(X[-1, 1])]
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f"dev_check_{out['ctrl']}.json"
    f.write_text(json.dumps(out, indent=1))
    print(json.dumps(out))


if __name__ == "__main__":
    main()
