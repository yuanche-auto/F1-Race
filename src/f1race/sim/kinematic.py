"""Time-domain simulator of the kinematic single-track model (see vehicle.py).

State  [x, y, psi, v, delta] (rear-axle position, heading, speed, steering angle)
Input  [delta_rate_cmd, fx_cmd] held constant over one step (RK4).

Actuator limits are applied physically (steering rate/angle saturation, power
limit). The friction ellipse and the track limits cannot be "violated" by a
kinematic model, so they are monitored: every step records the ellipse usage
and the distance of the car body to the track edges, and the run reports
violations instead of silently continuing.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..track import Track
from ..vehicle import Vehicle

Controller = Callable[[float, np.ndarray], tuple[float, float]]


@dataclass
class SimResult:
    t: np.ndarray
    state: np.ndarray        # (K, 5)
    inputs: np.ndarray       # (K, 2) applied delta_rate, fx
    progress: np.ndarray     # unwrapped centreline arclength of the rear axle
    usage: np.ndarray        # friction-ellipse usage
    margin: np.ndarray       # (K, 2) edge distance of rear / front axle body [m]
    lap_time: float          # nan if the lap was not completed
    completed: bool
    stop_reason: str
    info: dict = field(default_factory=dict)

    @property
    def max_usage(self) -> float:
        return float(self.usage.max())

    @property
    def min_margin(self) -> float:
        return float(self.margin.min())


class KinematicSim:
    def __init__(self, track: Track, vehicle: Vehicle, dt: float = 0.02,
                 usage_tol: float = 1.0 + 1e-6, stop_on_violation: bool = True):
        self.track, self.veh, self.dt = track, vehicle, dt
        self.usage_tol = usage_tol
        self.stop_on_violation = stop_on_violation

    # --------------------------------------------------------------- physics
    def _f(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        veh = self.veh
        _, _, psi, v, delta = x
        drate, fx = u
        ax = (fx - veh.resist(v * v)) / veh.m
        return np.array([v * np.cos(psi), v * np.sin(psi), v * np.tan(delta) / veh.L, ax, drate])

    def apply_limits(self, x: np.ndarray, u_cmd) -> np.ndarray:
        veh = self.veh
        drate = float(np.clip(u_cmd[0], -veh.delta_rate_max, veh.delta_rate_max))
        delta = x[4]
        if (delta >= veh.delta_max and drate > 0) or (delta <= -veh.delta_max and drate < 0):
            drate = 0.0
        fx = min(float(u_cmd[1]), veh.P_max / max(x[3], 1.0))
        return np.array([drate, fx])

    def step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        h = self.dt
        k1 = self._f(x, u)
        k2 = self._f(x + 0.5 * h * k1, u)
        k3 = self._f(x + 0.5 * h * k2, u)
        k4 = self._f(x + h * k3, u)
        xn = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        xn[4] = np.clip(xn[4], -self.veh.delta_max, self.veh.delta_max)
        return xn

    def edge_margin(self, x: np.ndarray) -> tuple[float, float, float]:
        """(s of rear axle, margin rear, margin front): distance of the car body edge
        (half width around each axle point) to the nearest track edge."""
        tr, veh = self.track, self.veh
        px = np.array([x[0], x[0] + veh.L * np.cos(x[2])])
        py = np.array([x[1], x[1] + veh.L * np.sin(x[2])])
        s, n = tr.project(px, py)
        wl, wr = tr.interp(tr.w_left, s), tr.interp(tr.w_right, s)
        m = np.minimum(wl - n, wr + n) - veh.half_width
        return float(s[0]), float(m[0]), float(m[1])

    # ------------------------------------------------------------------- run
    def run(self, controller: Controller, x0: np.ndarray, t_max: float = 300.0,
            laps: float = 1.0) -> SimResult:
        tr, veh = self.track, self.veh
        x = np.asarray(x0, float).copy()
        s0, mr, mf = self.edge_margin(x)
        prog = 0.0
        s_prev = s0
        T, X, U, P, US, M = [], [], [], [], [], []
        t = 0.0
        lap_time, completed, reason = float("nan"), False, "time limit"
        target = laps * tr.length
        while t < t_max:
            u = self.apply_limits(x, controller(t, x))
            us = float(veh.usage(x[3], x[4], u[1]))
            T.append(t); X.append(x.copy()); U.append(u); P.append(prog); US.append(us); M.append((mr, mf))
            if self.stop_on_violation and (min(mr, mf) < 0.0 or us > self.usage_tol):
                reason = "off track" if min(mr, mf) < 0.0 else "friction limit exceeded"
                break
            xn = self.step(x, u)
            sn, mr, mf = self.edge_margin(xn)
            ds = (sn - s_prev + 0.5 * tr.length) % tr.length - 0.5 * tr.length
            prog_n = prog + ds
            if prog_n >= target:                       # interpolate the crossing time
                lap_time = t + self.dt * (target - prog) / max(prog_n - prog, 1e-9)
                completed, reason = True, "lap completed"
                T.append(t + self.dt); X.append(xn.copy()); U.append(u); P.append(prog_n)
                US.append(float(veh.usage(xn[3], xn[4], u[1]))); M.append((mr, mf))
                break
            x, prog, s_prev, t = xn, prog_n, sn, t + self.dt
        return SimResult(np.array(T), np.array(X), np.array(U), np.array(P), np.array(US),
                         np.array(M), lap_time, completed, reason)
