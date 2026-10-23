"""Control expansion phase G: delay-compensated RTI under L9's real-time rule (docs/CX_G_protocol.md,
revision 0 with its clarifications m1–m3, n1, n2).

* Sets (section 2): ``GT1`` – the nominal car and ``sample_plants(nominal, 24, seed 371)`` × τ ∈ {0, 30,
  60 ms}; ``GT2`` – the nominal car with the L3 disturbances, seeds 7700–7729 (test, 105 laps); ``GD`` –
  ``sample_plants(nominal, 3, seed 379)[1:]`` × τ and seeds 7790–7792 (development); ``NOM``.
* Arms (sections 3, 4): ``ARMS`` – name -> (kind, sensor delay d, compensation or None); the
  compensation is ``experiments.l9``'s ``comp`` option ({"d": the assumed delay, "lhat": 10 ms}).
* ``drive_arm``: one evaluated lap of an arm (``l9.drive`` in the real-time mode with the RTI, the shared
  plan of the case) and its phase-G summaries: the prediction error of the state each solve assumed
  against the true state at the time it assumed it for, the application delays (review m1), the
  share of late solves and the prediction's cost.
"""
from __future__ import annotations

import math

import numpy as np

from ..dynamics import DynVehicle
from .l2 import sample_plants
from . import l9

TAUS = (0.0, 0.03, 0.06)
LHAT = 0.01                                     # the fixed command-latency estimate (section 3.1)
SETS = {   # name: (car seed, number of cars, with the nominal car, disturbance seeds)
    "GT1": (371, 24, True, ()), "GT2": (None, 0, False, tuple(range(7700, 7730))),
    "GD": (379, 3, False, tuple(range(7790, 7793))),
}


def _comp(d: float) -> dict:
    return {"d": d, "lhat": LHAT}


ARMS = {
    # primary (section 3): lag_identified, both arms at d = 0 and 20 ms
    "rti_d0": ("lag_identified", 0.0, None), "rti_dc_d0": ("lag_identified", 0.0, _comp(0.0)),
    "rti_d20": ("lag_identified", 0.02, None), "rti_dc_d20": ("lag_identified", 0.02, _comp(0.02)),
    # S1: a larger and a mis-specified delay
    "rti_d40": ("lag_identified", 0.04, None), "rti_dc_d40": ("lag_identified", 0.04, _comp(0.04)),
    "rti_dc20_d40": ("lag_identified", 0.04, _comp(0.02)),
    # S2: the nominal-model NMPC
    "nmpc_rti_d20": ("nmpc", 0.02, None), "nmpc_rti_dc_d20": ("nmpc", 0.02, _comp(0.02)),
}
PRIMARY = ("rti_d0", "rti_dc_d0", "rti_d20", "rti_dc_d20")
S1 = ("rti_d40", "rti_dc_d40", "rti_dc20_d40")
S2 = ("nmpc_rti_d20", "nmpc_rti_dc_d20")


def cases(name: str) -> list[dict]:
    """The laps of a set, as ``l7.cases`` builds them (same fields)."""
    nom = DynVehicle()
    if name == "NOM":
        return [{"set": "NOM", "car": 0, "info": {"nominal": True}, "plant": nom, "tau": 0.0, "dist_seed": None,
                 "name": "NOM"}]
    seed, n, with_nominal, dist = SETS[name]
    out = []
    if seed is not None:
        cars = sample_plants(nom, n, seed)
        idx = range(len(cars)) if with_nominal else range(1, len(cars))
        for tau in TAUS:
            for i in idx:
                out.append({"set": name, "car": i, "info": cars[i][0], "plant": cars[i][1], "tau": tau,
                            "dist_seed": None, "name": f"{name}_car{i}_tau{int(round(tau * 1000))}"})
    for d in dist:
        out.append({"set": name, "car": None, "info": {"nominal": True}, "plant": nom, "tau": 0.0, "dist_seed": d,
                    "name": f"{name}_dist{d}"})
    return out


def case_by_name(name: str) -> dict:
    return next(c for c in cases(name.split("_")[0]) if c["name"] == name)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def prediction_errors(calls: list[dict], states: np.ndarray, dt: float = l9.DT) -> dict:
    """The state each solve assumed against the true state at the time it assumed it for (``t_plan``):
    position [m], heading [rad], v_x, v_y [m/s], yaw rate [rad/s], steering angle [rad] (the controller's
    actual-angle estimate for a lag-aware one, else the angle it holds) – median and P95 of |error|."""
    rows = []
    for c in calls:
        if "x_assumed" not in c:
            continue
        i = int(round(c["t_plan"] / dt))
        if i >= len(states):
            continue
        xa, xt = np.asarray(c["x_assumed"], float), np.asarray(states[i], float)
        rows.append([math.hypot(xa[0] - xt[0], xa[1] - xt[1]), abs(_wrap(xa[2] - xt[2])), abs(xa[3] - xt[3]),
                     abs(xa[4] - xt[4]), abs(xa[5] - xt[5]), abs(xa[6] - xt[6])])
    if not rows:
        return {}
    a = np.array(rows)
    names = ("pos", "heading", "vx", "vy", "r", "delta")
    return {"n": len(rows), **{f"{k}_p50": float(np.median(a[:, j])) for j, k in enumerate(names)},
            **{f"{k}_p95": float(np.percentile(a[:, j], 95)) for j, k in enumerate(names)}}


def application_delays(calls: list[dict]) -> dict:
    """Review m1: the time from a solve's start to the time its command takes effect [ms]."""
    d = np.array([(c["t_apply"] - c["t"]) * 1e3 for c in calls if "t_apply" in c])
    if not d.size:
        return {}
    vals, counts = np.unique(np.round(d).astype(int), return_counts=True)
    return {"mean_ms": float(d.mean()), "max_ms": float(d.max()),
            "histogram_ms": {str(int(v)): int(n) for v, n in zip(vals, counts)}}


def drive_arm(case: dict, arm: str, planned: dict | None, name: str = "cxg") -> dict:
    """One evaluated lap of an arm in the real-time mode (RTI, the A1 rule) with the case's shared plan."""
    kind, d, comp = ARMS[arm]
    keep: dict = {}
    out = l9.drive(case, kind, "rti", "realtime", delay=d, name=name, planned=planned, comp=comp, log_assumed=True,
                   keep=keep)
    calls = keep.get("calls", [])
    out["arm"] = arm
    if "run" in keep:
        out["prediction_error"] = prediction_errors(calls, keep["run"].state)
    out["application_delay"] = application_delays(calls)
    if calls and "late" in calls[0]:
        out["late_share"] = float(np.mean([c["late"] for c in calls]))
    if calls and "time_pred" in calls[0]:
        tp = np.array([c["time_pred"] for c in calls]) * 1e3
        out["time_pred_ms"] = [float(tp.mean()), float(np.percentile(tp, 99)), float(tp.max())]
    return out


__all__ = ["cases", "case_by_name", "ARMS", "PRIMARY", "S1", "S2", "LHAT", "drive_arm", "prediction_errors",
           "application_delays"]
