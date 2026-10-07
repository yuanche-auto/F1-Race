"""Quasi-steady-state (QSS) minimum-time speed profile along a fixed closed path.

For the kinematic model, a path with curvature kappa(s) fixes the steering
delta = atan(L kappa), the lateral force m v^2 kappa and the steering rate
(d delta/ds) v. With w = v^2 the remaining freedom is the tangential acceleration
u = dv/dt = (1/2) dw/ds, bounded by the friction ellipse (with downforce), the
power limit and the resistances (Vehicle.accel_limits). The time-optimal profile
is obtained with the classic forward (max acceleration) / backward (max
braking) passes (each segment uses a constant acceleration that is admissible at
both of its ends), started at the slowest point of the loop and repeated until
the profile is periodic (one sweep normally suffices; on long constant-radius
sections drag makes the sustainable speed lower than the lateral limit and the
sweeps converge to that equilibrium).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..path import Path
from ..vehicle import Vehicle


@dataclass
class SpeedProfile:
    v: np.ndarray        # speed at the path points [m/s]
    a: np.ndarray        # tangential acceleration on segment i -> i+1 [m/s^2]
    t: np.ndarray        # time at the path points [s], t[0] = 0
    v_max: np.ndarray    # pointwise speed limit [m/s]
    lap_time: float


def pointwise_v2_max(path: Path, vehicle: Vehicle) -> np.ndarray:
    """Largest admissible v^2 at each point (lateral friction and steering rate)."""
    w_lat = vehicle.v2_lateral_max(path.kappa)
    L = vehicle.L
    ddelta_ds = L * path.dkappa / (1.0 + (L * path.kappa) ** 2)
    with np.errstate(divide="ignore"):
        w_rate = np.where(np.abs(ddelta_ds) > 1e-12,
                          (vehicle.delta_rate_max / np.abs(ddelta_ds)) ** 2, np.inf)
    w_top = vehicle.top_speed() ** 2
    return np.minimum(np.minimum(w_lat, w_rate), w_top)


def _fwd(veh: Vehicle, wi: float, ki: float, kj: float, ds: float) -> float:
    """Largest w at the end of a segment reachable from wi with constant acceleration
    admissible at BOTH ends (the limit is evaluated at the start and at the predicted
    end; the smaller one is used)."""
    _, a1 = veh.accel_limits(wi, ki)
    wp = max(wi + 2.0 * a1 * ds, 0.0)
    _, a2 = veh.accel_limits(wp, kj)
    return max(wi + 2.0 * min(a1, a2) * ds, 0.0)


def _bwd(veh: Vehicle, wj: float, kj: float, ki: float, ds: float) -> float:
    """Largest w at the start of a segment from which wj can be reached with a
    constant deceleration admissible at both ends."""
    a1, _ = veh.accel_limits(wj, kj)
    wp = max(wj - 2.0 * a1 * ds, 0.0)
    a2, _ = veh.accel_limits(wp, ki)
    return max(wj - 2.0 * max(a1, a2) * ds, 0.0)


def speed_profile(path: Path, vehicle: Vehicle, eta: float = 1.0,
                  max_sweeps: int = 50) -> SpeedProfile:
    """Time-optimal speed on a closed path; friction scaled by ``eta`` (planning margin)."""
    veh = vehicle.scaled(eta) if eta != 1.0 else vehicle
    kap = path.kappa
    ds = path.ds
    N = len(kap)
    wmax = pointwise_v2_max(path, veh)
    i0 = int(np.argmin(wmax))

    w = wmax.copy()
    for _ in range(max_sweeps):                         # forward passes until periodic
        w_old = w.copy()
        for k in range(1, N + 1):
            i, j = (i0 + k - 1) % N, (i0 + k) % N
            w[j] = min(w[j], _fwd(veh, w[i], kap[i], kap[j], ds[i]))
        if np.max(np.abs(w - w_old)) < 1e-9:
            break
    for _ in range(max_sweeps):                         # backward passes until periodic
        w_old = w.copy()
        for k in range(1, N + 1):
            j, i = (i0 - k + 1) % N, (i0 - k) % N
            w[i] = min(w[i], _bwd(veh, w[j], kap[j], kap[i], ds[i]))
        if np.max(np.abs(w - w_old)) < 1e-9:
            break

    v = np.sqrt(w)
    vn = np.roll(v, -1)
    a = (vn**2 - v**2) / (2.0 * ds)
    dt = 2.0 * ds / (v + vn)
    t = np.concatenate([[0.0], np.cumsum(dt)[:-1]])
    return SpeedProfile(v, a, t, np.sqrt(wmax), float(dt.sum()))
