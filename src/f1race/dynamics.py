"""Dynamic single-track model used from L1 on (reference point: centre of gravity).

States   vx, vy (body-frame velocity of the CG), r (yaw rate), delta (steering)
Inputs   delta_rate, F_drive >= 0 (rear axle), F_brake <= 0 (split front/rear)

    m (vx' - r vy) = F_xr + F_xf cos d - F_yf sin d - q_drag vx^2 - c_rr (F_zf + F_zr)
    m (vy' + r vx) = F_yr + F_yf cos d + F_xf sin d
    I_z r'         = l_f (F_yf cos d + F_xf sin d) - l_r F_yr

Tyres: slip angles a_f = d - atan((vy + l_f r)/vx), a_r = -atan((vy - l_r r)/vx);
lateral force F_yi = mu_y F_zi sin(C atan(B a_i)) (simplified magic formula,
peak mu_y F_zi at a = tan(pi/(2C))/B); combined slip limited by a friction
ellipse per axle (constraint in the optimiser, saturation in the simulator).
Normal loads: static split + downforce split + longitudinal load transfer
m a_x h / L with a_x estimated from the longitudinal forces.

All functions take ``lib`` = numpy or casadi so that the optimiser and the
simulator evaluate the same expressions.
"""
from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np

from .vehicle import G, Vehicle


@dataclass(frozen=True)
class DynVehicle(Vehicle):
    lr: float = 1.62             # rear axle -> CG [m]: front axle carries lr / L = 45 % statically
    Iz: float = 1000.0           # yaw inertia [kg m^2]
    h_cg: float = 0.30           # CG height [m]
    aero_front: float = 0.45     # share of the downforce on the front axle
    brake_front: float = 0.60    # brake balance (front share)
    B: float = 14.0              # tyre stiffness factor [1/rad]
    C: float = 1.6               # tyre shape factor

    @property
    def lf(self) -> float:
        return self.L - self.lr

    @property
    def alpha_peak(self) -> float:
        return float(np.tan(np.pi / (2.0 * self.C)) / self.B)


def _f(lib):
    if lib is np:
        return np.arctan, np.sin, np.cos, np.sqrt
    return ca.atan, ca.sin, ca.cos, ca.sqrt


def tyre_forces(veh: DynVehicle, vx, vy, r, delta, f_drive, f_brake, lib=np):
    """Slip angles, normal loads and (pure-slip) tyre forces of both axles."""
    atan, sin, cos, sqrt = _f(lib)
    lf, lr, L, m = veh.lf, veh.lr, veh.L, veh.m
    v2 = vx * vx + vy * vy
    resist = veh.q_drag * vx * vx + veh.c_rr * (m * G + veh.q_lift * v2)
    ax = (f_drive + f_brake - resist) / m
    fzf = m * G * lr / L + veh.aero_front * veh.q_lift * v2 - m * ax * veh.h_cg / L
    fzr = m * G * lf / L + (1.0 - veh.aero_front) * veh.q_lift * v2 + m * ax * veh.h_cg / L
    af = delta - atan((vy + lf * r) / vx)
    ar = -atan((vy - lr * r) / vx)
    fyf = veh.mu_y * fzf * sin(veh.C * atan(veh.B * af))
    fyr = veh.mu_y * fzr * sin(veh.C * atan(veh.B * ar))
    fxf = veh.brake_front * f_brake
    fxr = f_drive + (1.0 - veh.brake_front) * f_brake
    return dict(af=af, ar=ar, fzf=fzf, fzr=fzr, fxf=fxf, fxr=fxr, fyf=fyf, fyr=fyr)


def ellipse_usage(veh: DynVehicle, F: dict, lib=np):
    """Friction-ellipse usage of the front and rear axle."""
    _, _, _, sqrt = _f(lib)
    uf = sqrt((F["fxf"] / (veh.mu_x * F["fzf"])) ** 2 + (F["fyf"] / (veh.mu_y * F["fzf"])) ** 2)
    ur = sqrt((F["fxr"] / (veh.mu_x * F["fzr"])) ** 2 + (F["fyr"] / (veh.mu_y * F["fzr"])) ** 2)
    return uf, ur


def body_accel(veh: DynVehicle, vx, vy, r, delta, F: dict, lib=np):
    """(vx', vy', r') for given tyre forces."""
    _, sin, cos, _ = _f(lib)
    m = veh.m
    resist = veh.q_drag * vx * vx + veh.c_rr * (F["fzf"] + F["fzr"])
    sd, cd = sin(delta), cos(delta)
    dvx = (F["fxr"] + F["fxf"] * cd - F["fyf"] * sd - resist) / m + r * vy
    dvy = (F["fyr"] + F["fyf"] * cd + F["fxf"] * sd) / m - r * vx
    dr = (veh.lf * (F["fyf"] * cd + F["fxf"] * sd) - veh.lr * F["fyr"]) / veh.Iz
    return dvx, dvy, dr


def saturate(veh: DynVehicle, F: dict) -> tuple[dict, float, float]:
    """Simulator: scale each axle's (F_x, F_y) back onto its ellipse if outside
    (the tyre slides); returns the forces and the usages before saturation."""
    uf, ur = ellipse_usage(veh, F)
    G2 = dict(F)
    if uf > 1.0:
        G2["fxf"], G2["fyf"] = F["fxf"] / uf, F["fyf"] / uf
    if ur > 1.0:
        G2["fxr"], G2["fyr"] = F["fxr"] / ur, F["fyr"] / ur
    return G2, float(uf), float(ur)
