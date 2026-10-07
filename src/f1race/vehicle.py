"""Vehicle parameters and the force limits shared by planners, controllers and simulators.

L0 uses a kinematic single-track ("bicycle") model referenced at the rear axle:

    x' = v cos(psi),  y' = v sin(psi),  psi' = v tan(delta) / L,  v' = a_x,  delta' = delta_rate

with a friction ellipse on the total tyre force, aerodynamic downforce and drag,
rolling resistance and an engine power limit:

    Fz   = m g + q_lift v^2                     (normal load incl. downforce)
    F_y  = m v^2 tan(delta) / L                 (lateral force, rear-axle path)
    m a_x = F_x - q_drag v^2 - c_rr Fz          (F_x: longitudinal tyre force)
    (F_x / (mu_x Fz))^2 + (F_y / (mu_y Fz))^2 <= 1
    F_x v <= P_max,  |delta| <= delta_max,  |delta_rate| <= delta_rate_max

The default parameters describe a generic open-wheel racing car; they are
round illustrative numbers, not data of any specific car.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

G = 9.81


@dataclass(frozen=True)
class Vehicle:
    m: float = 750.0             # mass [kg]
    L: float = 3.6               # wheelbase [m]
    lr: float = 1.9              # rear axle -> CG [m] (used from L1 on)
    half_width: float = 1.0      # half of the car width [m]
    rho: float = 1.2             # air density [kg/m^3]
    CdA: float = 1.1             # drag area [m^2]
    ClA: float = 3.5             # downforce area [m^2]
    c_rr: float = 0.012          # rolling resistance coefficient
    mu_x: float = 1.5            # longitudinal friction coefficient
    mu_y: float = 1.6            # lateral friction coefficient
    P_max: float = 550e3         # engine power [W]
    delta_max: float = 0.4       # steering angle limit [rad]
    delta_rate_max: float = 1.0  # steering rate limit [rad/s]

    # ---------------------------------------------------------------- derived
    @property
    def q_lift(self) -> float:
        return 0.5 * self.rho * self.ClA

    @property
    def q_drag(self) -> float:
        return 0.5 * self.rho * self.CdA

    def scaled(self, eta: float) -> "Vehicle":
        """Copy with friction coefficients scaled by eta (planning margin)."""
        return replace(self, mu_x=self.mu_x * eta, mu_y=self.mu_y * eta)

    def fz(self, v2):
        return self.m * G + self.q_lift * v2

    def resist(self, v2):
        """Aerodynamic drag + rolling resistance [N] at speed^2 = v2."""
        return self.q_drag * v2 + self.c_rr * self.fz(v2)

    def top_speed(self) -> float:
        """Speed where full power balances drag and rolling resistance on a straight."""
        lo, hi = 1.0, 200.0
        for _ in range(100):
            v = 0.5 * (lo + hi)
            if self.P_max / v > self.resist(v * v):
                lo = v
            else:
                hi = v
        return 0.5 * (lo + hi)

    # ------------------------------------------------------- friction ellipse
    def fx_available(self, v2, kappa):
        """Longitudinal tyre force still available while cornering on curvature kappa
        at speed^2 = v2 (0 where the lateral demand alone reaches the ellipse)."""
        fz = self.fz(v2)
        lat = self.m * np.abs(kappa) * v2 / (self.mu_y * fz)
        return self.mu_x * fz * np.sqrt(np.maximum(1.0 - lat**2, 0.0))

    def v2_lateral_max(self, kappa):
        """Largest v^2 with lateral demand inside the ellipse (inf if downforce wins)."""
        k = np.abs(np.asarray(kappa, float))
        den = self.m * k - self.mu_y * self.q_lift
        with np.errstate(divide="ignore"):
            return np.where(den > 0, self.mu_y * self.m * G / np.where(den > 0, den, 1.0), np.inf)

    def accel_limits(self, v2, kappa):
        """(a_min, a_max) along the path at speed^2 = v2 on curvature kappa."""
        fxa = self.fx_available(v2, kappa)
        v = np.sqrt(np.maximum(v2, 1e-6))
        f_drive = np.minimum(fxa, self.P_max / v)
        res = self.resist(v2)
        return (-fxa - res) / self.m, (f_drive - res) / self.m

    def usage(self, v, delta, fx):
        """Friction-ellipse usage (1 = at the limit) of a kinematic state."""
        v2 = np.asarray(v, float) ** 2
        fz = self.fz(v2)
        fy = self.m * v2 * np.tan(delta) / self.L
        return np.hypot(fx / (self.mu_x * fz), fy / (self.mu_y * fz))
