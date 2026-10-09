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


def lag_filter(dc: np.ndarray, h: float, tau: float, a0: float | None = None) -> np.ndarray:
    """Road-wheel angle of a first-order steering lag driven by the sampled commanded
    angle ``dc`` (a ramp between samples, exact): a[i+1] = E a[i] + b0 dc[i+1] + b1 dc[i]."""
    from scipy.signal import lfilter, lfiltic

    tau = max(float(tau), 1e-6)
    E = np.exp(-h / tau)
    q = tau * (1.0 - E) / h
    b, a = [1.0 - q, q - E], [1.0, -E]
    y0 = dc[0] if a0 is None else a0
    zi = lfiltic(b, a, [y0], [dc[0]])
    return lfilter(b, a, dc, zi=zi)[0] if len(dc) else dc


def identify_with_lag(run, dt: float, nominal: DynVehicle, usage_max: float = 0.98, skip_end: int = 20,
                      tau_starts=(0.005, 0.01, 0.02, 0.04, 0.08, 0.16), tau_max: float = 0.3) -> dict:
    """The L4 identification on the model with the correct structure (L6): the steering
    slot of the logged state is the commanded angle, the model's road-wheel angle follows
    it with a first-order lag whose time constant is identified together with the seven
    parameter factors. ``run``: the run as the controller logged it (7-dim states). The
    least squares is started from each of ``tau_starts`` (the other factors from the
    identification without lag); the best fit is kept."""
    X, U = run.state, run.inputs
    i = np.arange(0, max(len(X) - skip_end - 1, 0))
    acc = (X[i + 1, 3:6] - X[i, 3:6]) / dt
    keep = ((run.usage[i].max(axis=1) < usage_max) & (run.usage[i + 1].max(axis=1) < usage_max)
            & (X[i, 3] > 5.0))
    mid = 0.5 * (X[i] + X[i + 1])
    dc = X[:, 6]
    scale = np.maximum(acc[keep].std(axis=0), 1e-3)

    def data_for(tau):
        a = lag_filter(dc, dt, tau)
        st = mid.copy()
        st[:, 6] = 0.5 * (a[i] + a[i + 1])
        return Data(st[keep], U[i][keep], acc[keep])

    def res(p):
        d = data_for(p[-1])
        return ((model_accel(vehicle_from(p[:-1], nominal), d) - d.accel) / scale).ravel()

    plain = identify(Data(mid[keep], U[i][keep], acc[keep]), nominal)
    lo, hi = np.append(LO, 1e-4), np.append(HI, tau_max)
    best = None
    for t0 in tau_starts:
        sol = least_squares(res, np.append(plain["theta"], t0), bounds=(lo, hi), x_scale="jac")
        if best is None or sol.cost < best.cost:
            best = sol
    theta, tau = best.x[:-1], float(best.x[-1])
    return {"theta": theta, "tau": tau, "rms": float(np.sqrt(2.0 * best.cost / best.fun.size)),
            "rms_without_lag": plain["rms"], "samples": int(keep.sum()),
            "vehicle": vehicle_from(theta, nominal), "on_bounds": [bool(np.isclose(v, l) or np.isclose(v, h))
                                                                  for v, l, h in zip(best.x, lo, hi)]}


def smoothed(run, window: int = 11, order: int = 3):
    """The run with the logged velocities and steering angle smoothed (Savitzky–Golay)
    before the accelerations are differenced (L6, round 3)."""
    from dataclasses import is_dataclass
    from types import SimpleNamespace

    from scipy.signal import savgol_filter

    st = run.state.copy()
    st[:, 3:7] = savgol_filter(st[:, 3:7], window, order, axis=0)
    if is_dataclass(run):
        return replace(run, state=st)
    return SimpleNamespace(**{**vars(run), "state": st})
