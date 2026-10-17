"""L8: generalisation – new tracks and new conditions (docs/L8_protocol.md).

* ``VALIDATION``, ``TEST``, ``RESERVE``: the track draw of ``scripts/l8_tracks.py``.
* ``compute_plans(name)``: the plans every plan-based controller gets on a track, made by the
  L1 pipeline unchanged (minimum-curvature line, kinematic OCP, dynamic OCP with the
  continuation over the friction margin: eta 1.0, 0.97, 0.95, 0.93) plus the 12 % plan
  (eta 0.88, from the ideal plan, as ``scripts/run_l2.py``); cached in
  ``results/L8/plans/<name>.pkl`` with the code fingerprint of L1.
"""
from __future__ import annotations

import pickle
import time
from pathlib import Path

import numpy as np

from ..dynamics import DynVehicle
from ..planning.mincurv import min_curvature_line
from ..planning.ocp_dynamic import initial_guess_from_dynamic, solve_min_time_dynamic
from ..planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from ..planning.qss import speed_profile
from ..track import Track
from .l1 import L1Config, code_fingerprint, dynamic_plans

ROOT = Path(__file__).resolve().parents[3]
VALIDATION = ("Monza", "Sakhir")
TEST = ("Zandvoort", "SaoPaulo", "Norisring", "Nuerburgring")
RESERVE = ("Austin", "Silverstone", "Shanghai", "Montreal", "Catalunya", "MoscowRaceway", "Hockenheim",
           "YasMarina", "Sepang", "Spa", "Oschersleben", "BrandsHatch", "Sochi", "MexicoCity", "Melbourne",
           "Budapest", "Suzuka")


def plans_path(name: str) -> Path:
    return ROOT / "results" / "L8" / "plans" / f"{name}.pkl"


def compute_plans(name: str, cfg: L1Config | None = None) -> dict:
    """The L1 plans and the 12 % plan of a track (see the module docstring); raises if a plan
    fails, with the solve log."""
    cfg = cfg or L1Config(track=name)
    t0 = time.time()
    track, veh = Track.load(name), DynVehicle()
    alpha, mc = min_curvature_line(track, veh, margin=cfg.margin)
    q = speed_profile(mc, veh, 1.0)
    idx = np.arange(0, len(track.s), 3)
    init0 = initial_guess_from_path(track, veh, idx, alpha, mc, q.v, q.a)
    o0 = solve_min_time(track, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=1.0)
    if not o0.success:
        raise RuntimeError(f"{name}: kinematic OCP failed: {o0.status}")
    log: list = []
    plans = dynamic_plans(track, veh, o0, cfg, log)
    o88 = solve_min_time_dynamic(track, veh, initial_guess_from_dynamic(plans[1.0]), eta=0.88)
    log.append({"step": "eta=0.88 from ideal (as run_l2)", "status": o88.status, "iterations": o88.iterations,
                "lap_time": o88.lap_time})
    if not o88.success:
        raise RuntimeError(f"{name}: eta = 0.88 plan failed: {log}")
    plans[0.88] = o88
    out = {"track": name, "o0": o0, "plans": plans, "log": log, "code": code_fingerprint(),
           "minutes": (time.time() - t0) / 60.0}
    p = plans_path(name)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "wb") as fh:
        pickle.dump(out, fh)
    return out


def load_plans(name: str) -> dict:
    with open(plans_path(name), "rb") as fh:
        d = pickle.load(fh)
    if d["code"] != code_fingerprint():
        raise RuntimeError(f"{name}: plans made with other planning code; recompute")
    return d


__all__ = ["VALIDATION", "TEST", "RESERVE", "plans_path", "compute_plans", "load_plans"]
