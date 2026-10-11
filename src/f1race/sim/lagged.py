"""Plant with a structural model error (L6): a first-order steering actuator lag.

State  [x, y, psi, vx, vy, r, delta, delta_cmd]

The controllers command a steering rate as before; it moves the commanded angle
delta_cmd (rate and angle limits as in ``DynamicSim``). The road wheels follow
with a first-order lag,

    delta' = (delta_cmd - delta) / tau,

and the tyre forces depend on the actual angle delta. The controllers' model has
no such lag (delta' = delta_rate), so no choice of parameters reproduces this car.

What the controllers see is set by ``observe``:

* ``"command"`` (main condition of L6): the steering slot of the 7-dim state they
  receive holds their own commanded angle – there is no sensor on the road-wheel
  angle, the lag is hidden inside the car's response (like a servo without
  feedback, or tyre forces that build up behind a measured steering angle);
* ``"angle"``: the steering slot holds the measured road-wheel angle (ablation).
"""
from __future__ import annotations

import numpy as np

from ..dynamics import DynVehicle, body_accel, saturate, tyre_forces
from ..track import Track
from .dynamic import DynamicSim


class LaggedSim(DynamicSim):
    def __init__(self, track: Track, vehicle: DynVehicle, tau: float, dt: float = 0.01,
                 observe: str = "command", stop_on_slide: bool = False):
        if tau <= 0.0:
            raise ValueError("tau must be positive (use DynamicSim without a lag)")
        if observe not in ("command", "angle"):
            raise ValueError(observe)
        super().__init__(track, vehicle, dt, stop_on_slide)
        self.tau, self.observe_mode = float(tau), observe

    @staticmethod
    def lift(x7) -> np.ndarray:
        """7-dim state (steady steering: actual = commanded angle) -> 8-dim plant state."""
        x7 = np.asarray(x7, float)
        return np.concatenate([x7[:7], [x7[6]]])

    def observe(self, x) -> np.ndarray:
        """The 7-dim state the controllers receive."""
        y = np.array(x[:7], float)
        if self.observe_mode == "command":
            y[6] = x[7]
        return y

    def _forces(self, x, u):
        _, _, _, vx, vy, r, d = x[:7]
        F = tyre_forces(self.veh, vx, vy, r, d, u[1], u[2])
        Fs, uf, ur = saturate(self.veh, F)
        return Fs, (uf, ur), (float(F["af"]), float(F["ar"]))

    def _f(self, x, u):
        _, _, psi, vx, vy, r, d, dc = x
        F, _, _ = self._forces(x, u)
        dvx, dvy, dr = body_accel(self.veh, vx, vy, r, d, F)
        c, s = np.cos(psi), np.sin(psi)
        return np.array([vx * c - vy * s, vx * s + vy * c, r, dvx, dvy, dr, (dc - d) / self.tau, u[0]])

    def apply_limits(self, x, u_cmd):
        """As ``DynamicSim.apply_limits``, with the angle limit on the commanded angle."""
        veh = self.veh
        drate = float(np.clip(u_cmd[0], -veh.delta_rate_max, veh.delta_rate_max))
        dc = x[7]
        if (dc >= veh.delta_max and drate > 0) or (dc <= -veh.delta_max and drate < 0):
            drate = 0.0
        fd = float(np.clip(u_cmd[1], 0.0, veh.P_max / max(x[3], 1.0)))
        fb = float(min(u_cmd[2], 0.0))
        return np.array([drate, fd, fb])

    def _f6(self, z, d, u):
        return self._f(np.concatenate([z, [d, d]]), u)[:6]

    def step(self, x, u):
        """The lag is integrated exactly over the step (the commanded angle is a ramp),
        the other states by RK4 with the exact road-wheel angle at the RK4 nodes (as in
        the lag model of ``FrenetMPC``; stable for any tau)."""
        h, tau = self.dt, self.tau
        a0, c0, rho = x[6], x[7], u[0]

        def d_at(t):
            return c0 + rho * (t - tau) + (a0 - c0 + rho * tau) * np.exp(-t / tau)

        z = x[:6]
        k1 = self._f6(z, a0, u)
        k2 = self._f6(z + 0.5 * h * k1, d_at(0.5 * h), u)
        k3 = self._f6(z + 0.5 * h * k2, d_at(0.5 * h), u)
        k4 = self._f6(z + h * k3, d_at(h), u)
        xn = np.empty(8)
        xn[:6] = z + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        xn[6] = d_at(h)
        xn[7] = np.clip(c0 + rho * h, -self.veh.delta_max, self.veh.delta_max)
        return xn

    def run(self, controller, x0, t_max: float = 300.0, laps: float = 1.0):
        """``x0``: 7-dim (lifted with actual = commanded angle) or 8-dim. The controller
        is called with ``observe(x)``; the result holds the full 8-dim plant state."""
        x0 = np.asarray(x0, float)
        if len(x0) == 7:
            x0 = self.lift(x0)
        return super().run(lambda t, x: controller(t, self.observe(x)), x0, t_max, laps)
