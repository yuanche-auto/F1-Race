"""Joint state and parameter estimation with an extended Kalman filter (L6, round 3).

Against the bias of the least squares of L4 under sensor noise (noisy states are
both regressors and, after differencing, targets – an errors-in-variables
problem): the filter runs the model forward with the applied inputs and corrects
it with the noisy measurements, so that no velocity is differenced.

Filter state  s = (v_x, v_y, r, delta, theta_1..theta_7)   (theta: the factors of
``identify.PARAMS``), discrete model: one RK4 step of the dynamic single-track model
with the vehicle of ``vehicle_from(theta)``, delta' = delta_rate (the applied
command), theta a random walk. Measurements (v_x, v_y, r, delta) with the sensor
noise. Samples where the plant's tyres are saturated (the same rule as the least
squares) carry no information on theta: there the velocity states are reset to the
measurement and the parameters are only propagated. Parameters are projected onto
the bounds of the least squares after every update.
"""
from __future__ import annotations

from dataclasses import replace

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from .identify import HI, LO, PARAMS, X0, vehicle_from


def _step_function(nominal: DynVehicle, h: float) -> ca.Function:
    s = ca.SX.sym("s", 4 + len(PARAMS))
    u = ca.SX.sym("u", 3)
    fac = dict(zip(PARAMS, [s[4 + i] for i in range(len(PARAMS))]))  # vehicle_from, symbolic
    veh = replace(nominal, mu_x=nominal.mu_x * fac["mu"], mu_y=nominal.mu_y * fac["mu"], m=nominal.m * fac["m"],
                  ClA=nominal.ClA * fac["ClA"], CdA=nominal.CdA * fac["CdA"], Iz=nominal.Iz * fac["Iz"],
                  B=nominal.B * fac["B"], aero_front=nominal.aero_front + fac["aero_front"])

    def f(z):
        vx, vy, r, d = z[0], z[1], z[2], z[3]
        F = tyre_forces(veh, vx, vy, r, d, u[1], u[2], lib=ca)
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
        return ca.vertcat(dvx, dvy, dr, u[0])

    z = s[:4]
    k1 = f(z); k2 = f(z + h / 2 * k1); k3 = f(z + h / 2 * k2); k4 = f(z + h * k3)
    sn = ca.vertcat(z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4), s[4:])
    return ca.Function("step", [s, u], [sn, ca.jacobian(sn, s)])


def ekf_identify(state: np.ndarray, inputs: np.ndarray, usage: np.ndarray, dt: float, nominal: DynVehicle,
                 sigma: np.ndarray, accel_sd: float = 0.5, yaw_accel_sd: float = 0.05,
                 theta_walk_sd: float = 1e-5, usage_max: float = 0.98) -> dict:
    """``state`` (K, 7) noisy logged states, ``inputs`` (K, 3) applied inputs, ``usage``
    (K, 2) plant ellipse usage, ``sigma`` (7) measurement noise of the state entries.
    Process noise: accelerations with ``accel_sd`` (v_x, v_y) and ``yaw_accel_sd`` (r)
    over a step, parameters a random walk with ``theta_walk_sd`` per step."""
    step = _step_function(nominal, dt)
    p = len(PARAMS)
    n = 4 + p
    R = np.diag(np.asarray(sigma, float)[3:7] ** 2)
    H = np.zeros((4, n)); H[:, :4] = np.eye(4)
    Q = np.diag(np.r_[(accel_sd * dt) ** 2, (accel_sd * dt) ** 2, (yaw_accel_sd * dt) ** 2, 1e-12,
                      np.full(p, theta_walk_sd ** 2)])
    s = np.r_[state[0, 3:7], X0]
    P = np.diag(np.r_[np.asarray(sigma, float)[3:7] ** 2, ((HI - LO) / 4.0) ** 2])
    skipped = 0
    for k in range(len(state) - 1):
        sn, F = step(s, inputs[k])
        s, F = np.asarray(sn).ravel(), np.asarray(F)
        P = F @ P @ F.T + Q
        y = state[k + 1, 3:7]
        if max(usage[k].max(), usage[k + 1].max()) >= usage_max or state[k, 3] <= 5.0:
            s[:4] = y                                   # no information on the parameters here
            P[:4, :] = 0.0; P[:, :4] = 0.0
            P[:4, :4] = R
            skipped += 1
            continue
        S = H @ P @ H.T + R
        K = np.linalg.solve(S, H @ P).T
        s = s + K @ (y - s[:4])
        P = (np.eye(n) - K @ H) @ P
        P = 0.5 * (P + P.T)
        s[4:] = np.clip(s[4:], LO, HI)
    theta = s[4:]
    return {"theta": theta, "std": np.sqrt(np.diag(P)[4:]), "vehicle": vehicle_from(theta, nominal),
            "skipped": skipped, "samples": len(state) - 1 - skipped}
