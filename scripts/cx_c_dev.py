"""Control expansion phase C – the development checks of docs/CX_C_protocol.md section 9 (9.1 and 9.2 are unit tests in
tests/test_tube.py), on the nominal car and the development cars CXDEV only (records in results/CX/C/dev/, each with
the commit and a dirty flag; not results of the protocol).

    python scripts/cx_c_dev.py wcheck      # 9.3 the vertex disturbance bound against 1 000 random L2 cars -> dev/wcheck.json
    python scripts/cx_c_dev.py laps        # 9.6 (+ 9.5 timing) NOM with the 6 settings -> dev/laps.jsonl, the gate
    python scripts/cx_c_dev.py cxdev       # 9.4 nmpc and tube (c_R 1) on the 5 development cars -> dev/cxdev.jsonl
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from multiprocessing import get_context
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
DEV = ROOT / "results" / "CX" / "C" / "dev"


def _git() -> dict:
    g = lambda *a: subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    return {"commit": g("rev-parse", "HEAD"), "dirty": bool(g("status", "--porcelain", "--untracked-files=no"))}


def wcheck() -> None:
    """Section 9.3: at 100 states of nmpc's nominal lap (phase B's record results/CX/B/dev/nmpc_nom.npz: the measured
    state and the applied first input of every solve), the one-step mismatch of 1 000 random cars drawn uniformly from
    the L2 box against the vertex maximum, per state component."""
    from dataclasses import replace

    import numpy as np

    from f1race.control.nmpc import FrenetMPC
    from f1race.control.tube import Tube, frenet_step
    from f1race.dynamics import DynVehicle
    from f1race.experiments import cx_a, cx_c
    from f1race.experiments.l2 import PERTURBATION, ref_from_plan
    from f1race.track import Track

    track, o = Track.load("Spielberg"), cx_a.plan()
    mpc = FrenetMPC(track, DynVehicle(), ref_from_plan(o), N=30, dt=0.05, eta=0.95, margin=0.6, param_tightening=True)
    tube = Tube(mpc, PERTURBATION, margin=0.6)
    d = np.load(ROOT / "results" / "CX" / "B" / "dev" / "nmpc_nom.npz")
    idx = np.linspace(1, len(d["x0"]) - 1, 100).astype(int)
    X, U = d["x0"][idx], d["U"][idx, 0]
    rng = np.random.default_rng(93)
    f = {k: rng.uniform(*PERTURBATION[k], size=(1000, 1)) for k in PERTURBATION}
    nom = DynVehicle()
    cars = replace(nom, mu_x=nom.mu_x * f["mu"], mu_y=nom.mu_y * f["mu"], m=nom.m * f["m"], ClA=nom.ClA * f["ClA"],
                   CdA=nom.CdA * f["CdA"], Iz=nom.Iz * f["Iz"], B=nom.B * f["B"], aero_front=nom.aero_front + f["aero_front"])
    with np.errstate(all="ignore"):
        base = frenet_step(nom, tube.kap, X, U, 0.05)
        rnd = np.abs(frenet_step(cars, tube.kap, X[None], U[None], 0.05) - base[None])      # (1000, 100, 7)
        ver = np.abs(frenet_step(tube.vertex_veh, tube.kap, X[None], U[None], 0.05) - base[None]).max(axis=0)
    names = ["s", "n", "xi", "vx", "vy", "r", "delta"]
    exceed = rnd > ver[None] * (1 + 1e-9) + 1e-12
    out = {"states": len(idx), "cars": 1000, **_git(),
           "share_exceeding_vertex_max_per_component": dict(zip(names, exceed.mean(axis=(0, 1)).tolist())),
           "largest_ratio_random_over_vertex_per_component": dict(zip(names, np.nanmax(
               np.where(ver[None] > 0, rnd / np.maximum(ver[None], 1e-300), 0.0), axis=(0, 1)).tolist())),
           "vertex_max_median_per_component": dict(zip(names, np.median(ver, axis=0).tolist()))}
    DEV.mkdir(parents=True, exist_ok=True)
    (DEV / "wcheck.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


def _lap_job(args):
    kind, setting, case_name = args
    from f1race.experiments import cx_c

    case = next(c for s_ in ("NOM", "CXDEV") for c in cx_c.cases(s_) if c["name"] == case_name)
    rec = cx_c.lap(case, kind, setting)
    rec.update(_git())
    return rec


def _pool(jobs, path: Path) -> list[dict]:
    DEV.mkdir(parents=True, exist_ok=True)
    out = []
    with get_context("spawn").Pool(len(jobs), maxtasksperchild=1) as pool, open(path, "w") as fo:
        for r in pool.imap_unordered(_lap_job, jobs):
            fo.write(json.dumps(r, default=float) + "\n")
            fo.flush()
            out.append(r)
            print(r["kind"], r["setting_key"], r["case"], r["completed"], round(r["lap_time"], 3) if r["completed"] else "",
                  r["stop_reason"], round(r["distance"]), "capped", r.get("capped_track_share_mean"),
                  r.get("capped_eta_share_mean"), "cover", r.get("coverage_exceed_share"), flush=True)
    return out


def laps() -> None:
    """Section 9.6: NOM with the 6 settings (and 9.5: the tube times of these laps). Gate: tube completes NOM with at least
    one setting whose capped shares (track and eta) are both at most one half."""
    from f1race.experiments import cx_c

    jobs = [(k, s_, "NOM_car0") for k in cx_c.TUBE_KINDS for s_ in cx_c.TUBE_GRID]
    recs = _pool(jobs, DEV / "laps.jsonl")
    ok = [r["setting_key"] for r in recs if r["kind"] == "tube" and r["completed"]
          and r["capped_track_share_mean"] <= 0.5 and r["capped_eta_share_mean"] <= 0.5]
    print("gate (9.6):", "passed by " + ", ".join(ok) if ok else "FAILED - an amendment before any CXV lap")


def cxdev() -> None:
    """Section 9.4: nmpc and tube (c_R 1) on the five development cars; the tube's coverage and back-offs per lap."""
    from f1race.experiments import cx_c

    names = [c["name"] for c in cx_c.cases("CXDEV")]
    jobs = [("nmpc", cx_c.fixed_setting("nmpc"), n) for n in names] + [("tube", {"c_R": 1.0}, n) for n in names]
    _pool(jobs, DEV / "cxdev.jsonl")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("wcheck", "laps", "cxdev"))
    a = ap.parse_args()
    {"wcheck": wcheck, "laps": laps, "cxdev": cxdev}[a.what]()
