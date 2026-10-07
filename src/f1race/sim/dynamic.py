"""Time-domain simulator of the dynamic single-track model (dynamics.py).

State  [x, y, psi, vx, vy, r, delta]  (CG position, heading, body velocities, yaw rate, steering)
Input  [delta_rate_cmd, f_drive_cmd, f_brake_cmd], held over one RK4 step.

Unlike the kinematic model, tyre saturation is physical here: forces outside
an axle's friction ellipse are scaled back onto it (the tyre slides) and the
event is recorded. A run stops when the car body leaves the track, when the
car spins (|side-slip| > 0.5 rad or vx < 1 m/s), or when the lap is complete.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..dynamics import DynVehicle, body_accel, saturate, tyre_forces
from ..track import Track

Controller = Callable[[float, np.ndarray], tuple[float, float, float]]


@dataclass
class DynSimResult:
    t: np.ndarray
    state: np.ndarray        # (K, 7)
    inputs: np.ndarray       # (K, 3) applied
    progress: np.ndarray
    usage: np.ndarray        # (K, 2) front / rear ellipse usage before saturation
    slip: np.ndarray         # (K, 2) slip angles front / rear
    margin: np.ndarray       # (K, 2) edge margin rear / front axle body
    lap_time: float
    completed: bool
    stop_reason: str
    info: dict = field(default_factory=dict)

    @property
    def max_usage(self) -> float:
        return float(self.usage.max())

    @property
    def min_margin(self) -> float:
        return float(self.margin.min())

    @property
    def slide_fraction(self) -> float:
        return float(np.mean(self.usage.max(axis=1) > 1.0))


class DynamicSim:
    def __init__(self, track: Track, vehicle: DynVehicle, dt: float = 0.01,
                 stop_on_slide: bool = False):
        self.track, self.veh, self.dt = track, vehicle, dt
        self.stop_on_slide = stop_on_slide

    def _forces(self, x, u):
        _, _, _, vx, vy, r, d = x
        F = tyre_forces(self.veh, vx, vy, r, d, u[1], u[2])
        Fs, uf, ur = saturate(self.veh, F)
        return Fs, (uf, ur), (float(F["af"]), float(F["ar"]))

    def _f(self, x, u):
        _, _, psi, vx, vy, r, d = x
        F, _, _ = self._forces(x, u)
        dvx, dvy, dr = body_accel(self.veh, vx, vy, r, d, F)
        c, s = np.cos(psi), np.sin(psi)
        return np.array([vx * c - vy * s, vx * s + vy * c, r, dvx, dvy, dr, u[0]])

    def apply_limits(self, x, u_cmd):
        veh = self.veh
        drate = float(np.clip(u_cmd[0], -veh.delta_rate_max, veh.delta_rate_max))
        d = x[6]
        if (d >= veh.delta_max and drate > 0) or (d <= -veh.delta_max and drate < 0):
            drate = 0.0
        fd = float(np.clip(u_cmd[1], 0.0, veh.P_max / max(x[3], 1.0)))
        fb = float(min(u_cmd[2], 0.0))
        return np.array([drate, fd, fb])

    def step(self, x, u):
        h = self.dt
        k1 = self._f(x, u)
        k2 = self._f(x + 0.5 * h * k1, u)
        k3 = self._f(x + 0.5 * h * k2, u)
        k4 = self._f(x + h * k3, u)
        xn = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        xn[6] = np.clip(xn[6], -self.veh.delta_max, self.veh.delta_max)
        return xn

    def edge_margin(self, x):
        tr, veh = self.track, self.veh
        c, s = np.cos(x[2]), np.sin(x[2])
        px = np.array([x[0] - veh.lr * c, x[0] + veh.lf * c])
        py = np.array([x[1] - veh.lr * s, x[1] + veh.lf * s])
        sa, n = tr.project(px, py)
        wl, wr = tr.interp(tr.w_left, sa), tr.interp(tr.w_right, sa)
        m = np.minimum(wl - n, wr + n) - veh.half_width
        sc, _ = tr.project(np.array([x[0]]), np.array([x[1]]))
        return float(sc[0]), float(m[0]), float(m[1])

    def run(self, controller: Controller, x0, t_max: float = 300.0, laps: float = 1.0) -> DynSimResult:
        tr = self.track
        x = np.asarray(x0, float).copy()
        s_prev, mr, mf = self.edge_margin(x)
        prog, t = 0.0, 0.0
        T, X, U, P, US, SL, M = [], [], [], [], [], [], []
        lap_time, completed, reason = float("nan"), False, "time limit"
        target = laps * tr.length
        while t < t_max:
            u = self.apply_limits(x, controller(t, x))
            _, us, sl = self._forces(x, u)
            T.append(t); X.append(x.copy()); U.append(u); P.append(prog)
            US.append(us); SL.append(sl); M.append((mr, mf))
            beta = np.arctan2(x[4], max(x[3], 1e-3))
            if min(mr, mf) < 0.0:
                reason = "off track"; break
            if abs(beta) > 0.5 or x[3] < 1.0:
                reason = "spin"; break
            if self.stop_on_slide and max(us) > 1.0:
                reason = "tyre saturation"; break
            xn = self.step(x, u)
            sn, mr, mf = self.edge_margin(xn)
            ds = (sn - s_prev + 0.5 * tr.length) % tr.length - 0.5 * tr.length
            if prog + ds >= target:
                lap_time = t + self.dt * (target - prog) / max(ds, 1e-9)
                completed, reason = True, "lap completed"
                break
            x, prog, s_prev, t = xn, prog + ds, sn, t + self.dt
        return DynSimResult(np.array(T), np.array(X), np.array(U), np.array(P), np.array(US),
                            np.array(SL), np.array(M), lap_time, completed, reason)
