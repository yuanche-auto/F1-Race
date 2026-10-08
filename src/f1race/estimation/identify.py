"""Identification of the dynamic single-track model from driving data (L4).

The unknowns are the same factors as the L2 plant perturbation: friction (both
coefficients), mass, downforce and drag areas, yaw inertia, tyre stiffness B
(multiplicative) and the front share of the downforce (additive). The data are
the logged states and applied inputs of a lap; accelerations are differences
of the logged velocities over each simulator step. Samples where an axle is saturated are
excluded (the simulator then scales the forces onto the ellipse, which the
model does not), as are the samples around a stop. Nonlinear least squares on
the body accelerations (vx', vy', r'), each normalised by its spread.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from ..dynamics import DynVehicle, body_accel, ellipse_usage, tyre_forces

PARAMS = ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front")
X0 = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0])
LO = np.array([0.7, 0.8, 0.6, 0.6, 0.5, 0.5, -0.15])
HI = np.array([1.3, 1.3, 1.4, 1.4, 1.5, 1.5, 0.15])


def vehicle_from(theta, nominal: DynVehicle) -> DynVehicle:
    f = dict(zip(PARAMS, np.asarray(theta, float)))
    return replace(nominal, mu_x=nominal.mu_x * f["mu"], mu_y=nominal.mu_y * f["mu"],
                   m=nominal.m * f["m"], ClA=nominal.ClA * f["ClA"], CdA=nominal.CdA * f["CdA"],
                   Iz=nominal.Iz * f["Iz"], B=nominal.B * f["B"],
                   aero_front=nominal.aero_front + f["aero_front"])


def theta_of(plant_factors: dict) -> np.ndarray:
    """L2 plant description (dict of factors) -> parameter vector."""
    return np.array([plant_factors.get(k, 0.0 if k == "aero_front" else 1.0) for k in PARAMS])


@dataclass
class Data:
    state: np.ndarray        # (K, 7) at the sample times
    inputs: np.ndarray       # (K, 3)
    accel: np.ndarray        # (K, 3) vx', vy', r'


def data_from_run(r, dt: float, usage_max: float = 0.98, skip_end: int = 20) -> Data:
    """Accelerations over each simulator step, (x[i+1] - x[i]) / dt, paired with the
    mid-step state and the input held over the step; saturated samples and the end of
    the run (a spin or an excursion is not representative) are dropped."""
    X, U = r.state, r.inputs
    i = np.arange(0, max(len(X) - skip_end - 1, 0))
    acc = (X[i + 1, 3:6] - X[i, 3:6]) / dt
    mid = 0.5 * (X[i] + X[i + 1])
    keep = ((r.usage[i].max(axis=1) < usage_max) & (r.usage[i + 1].max(axis=1) < usage_max)
            & (X[i, 3] > 5.0))
    return Data(mid[keep], U[i][keep], acc[keep])


def model_accel(veh: DynVehicle, d: Data) -> np.ndarray:
    _, _, _, vx, vy, r, delta = d.state.T
    F = tyre_forces(veh, vx, vy, r, delta, d.inputs[:, 1], d.inputs[:, 2])
    return np.column_stack(body_accel(veh, vx, vy, r, delta, F))


def identify(d: Data, nominal: DynVehicle, x0=None) -> dict:
    scale = np.maximum(d.accel.std(axis=0), 1e-3)

    def res(theta):
        return ((model_accel(vehicle_from(theta, nominal), d) - d.accel) / scale).ravel()

    sol = least_squares(res, X0 if x0 is None else x0, bounds=(LO, HI), x_scale="jac")
    J = sol.jac
    dof = max(J.shape[0] - J.shape[1], 1)
    s2 = 2.0 * sol.cost / dof
    try:
        cov = np.linalg.inv(J.T @ J) * s2
        std = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        std = np.full(len(PARAMS), np.nan)
    veh = vehicle_from(sol.x, nominal)
    F = tyre_forces(veh, *d.state[:, 3:7].T, d.inputs[:, 1], d.inputs[:, 2])
    uf, ur = ellipse_usage(veh, F)
    return {"theta": sol.x, "std": std, "rms": float(np.sqrt(2.0 * sol.cost / J.shape[0])),
            "samples": int(len(d.accel)), "max_usage_in_data": float(np.max(np.maximum(uf, ur))),
            "vehicle": veh}
