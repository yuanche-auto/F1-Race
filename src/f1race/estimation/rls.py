"""Online parameter estimation for the adaptive NMPC (control expansion phase D, docs/CX_D_protocol.md
sections 3.1-3.2).

**Step model.** The data of one simulator step k are the state x_k, the input u_k the plant applied and the
body velocities y_{k+1} = (v_x, v_y, r) of x_{k+1}. The prediction y_hat(theta) is one RK4 step of h (the
simulator's 10 ms) of the dynamic single-track model with the car ``identify.vehicle_from(theta)`` (theta: the
seven factors of ``identify.PARAMS``) **with the simulator's tyre saturation** (an axle's forces outside its
friction ellipse scaled onto it, ``dynamics.saturate``): the simulator's own step (``sim.dynamic.DynamicSim``),
restricted to the states it depends on, z = (v_x, v_y, r, delta) – the position and heading do not enter the
body accelerations. ``step_function`` writes it with CasADi (the saturation as a division by max(usage, 1),
which is the simulator's rule) and returns y_hat and dy_hat / dtheta; ``step_np`` is the same step in numpy,
batched over cars (the model bank).

**RLS** (``RecursiveLS``): the recursive Gauss-Newton (prediction-error) form of least squares with forgetting
(literature D.1, D.2), RLS for a model linear in theta. With eps = Lambda^-1/2 (y - y_hat(theta)) and
psi = Lambda^-1/2 dy_hat/dtheta (3 x 7) at the estimate,

    L = P psi^T (I_3 + psi P psi^T)^-1,  theta <- clip(theta + L eps, LO, HI),  P <- D (I - L psi) P D,

D = diag(lambda_i^-1/2) (vector-type forgetting, D.2 eq. 12, written symmetrically: equal factors give the
classical P / lambda), then P bounded by the prior, P <- P0^1/2 min(P0^-1/2 P P0^-1/2, I) P0^1/2 (eigenvalues
clipped at 1, against the wind-up of D.2). lambda_mu < 1 for the friction factor, 1 for the six car constants.
Samples below 5 m/s are skipped (L4's rule); none is skipped for saturation (the model contains it).
"""
from __future__ import annotations

import time
from dataclasses import replace

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from .identify import HI, LO, PARAMS, X0

SIGMA = np.array([0.1, 0.05, 0.01])        # residual scaling of (v_x, v_y, r): the L3 measurement noise
V_MIN = 5.0


def _vehicle(nominal: DynVehicle, th) -> DynVehicle:
    """``identify.vehicle_from`` for symbolic factors or for arrays of factors (one car per entry)."""
    return replace(nominal, mu_x=nominal.mu_x * th[0], mu_y=nominal.mu_y * th[0], m=nominal.m * th[1],
                   ClA=nominal.ClA * th[2], CdA=nominal.CdA * th[3], Iz=nominal.Iz * th[4], B=nominal.B * th[5],
                   aero_front=nominal.aero_front + th[6])


def _rates(veh: DynVehicle, z, u, lib):
    """(v_x', v_y', r', delta') of the simulator's model at z = (v_x, v_y, r, delta) with the input held."""
    vx, vy, r, d = z
    F = tyre_forces(veh, vx, vy, r, d, u[1], u[2], lib=lib)
    uf2 = (F["fxf"] / (veh.mu_x * F["fzf"])) ** 2 + (F["fyf"] / (veh.mu_y * F["fzf"])) ** 2
    ur2 = (F["fxr"] / (veh.mu_x * F["fzr"])) ** 2 + (F["fyr"] / (veh.mu_y * F["fzr"])) ** 2
    mx, sq = (ca.fmax, ca.sqrt) if lib is ca else (np.maximum, np.sqrt)
    # saturate: scaled onto the ellipse when outside (usage = sqrt(u2), as ellipse_usage); sqrt(max(u2, 1)) keeps the
    # derivative finite where an axle carries no force (u2 = 0)
    sf, sr = sq(mx(uf2, 1.0)), sq(mx(ur2, 1.0))
    Fs = dict(F, fxf=F["fxf"] / sf, fyf=F["fyf"] / sf, fxr=F["fxr"] / sr, fyr=F["fyr"] / sr)
    dvx, dvy, dr = body_accel(veh, vx, vy, r, d, Fs, lib=lib)
    return [dvx, dvy, dr, u[0] + 0.0 * vx]


def step_function(nominal: DynVehicle, h: float) -> ca.Function:
    """CasADi (z (4), u (3, forces in N), theta (7)) -> (y_hat (3), dy_hat / dtheta (3 x 7))."""
    z = ca.SX.sym("z", 4)
    u = ca.SX.sym("u", 3)
    th = ca.SX.sym("th", len(PARAMS))
    veh = _vehicle(nominal, th)

    def f(q):
        return ca.vertcat(*_rates(veh, [q[0], q[1], q[2], q[3]], u, ca))

    k1 = f(z); k2 = f(z + h / 2 * k1); k3 = f(z + h / 2 * k2); k4 = f(z + h * k3)
    y = (z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))[:3]
    return ca.Function("cxd_step", [z, u, th], [y, ca.jacobian(y, th)])


def step_np(veh: DynVehicle, z, u, h: float) -> np.ndarray:
    """The same step in numpy; ``veh`` may hold arrays (one car per entry): returns (n, 3) or (3,)."""
    z = [np.asarray(q, float) for q in z]

    def f(q):
        return _rates(veh, q, u, np)

    k1 = f(z)
    k2 = f([a + h / 2 * b for a, b in zip(z, k1)])
    k3 = f([a + h / 2 * b for a, b in zip(z, k2)])
    k4 = f([a + h * b for a, b in zip(z, k3)])
    y = [a + h / 6 * (b + 2 * c + 2 * d + e) for a, b, c, d, e in zip(z, k1, k2, k3, k4)][:3]
    return np.stack(np.broadcast_arrays(*y), axis=-1)


class RecursiveLS:
    """RLS with vector-type forgetting on the simulator's step model (module doc). ``update(z, u, y)`` with z the
    state (v_x, v_y, r, delta) at step k, u the applied input (forces in N), y = (v_x, v_y, r) at step k + 1."""

    def __init__(self, nominal: DynVehicle | None = None, h: float = 0.01, lam_mu: float = 0.995,
                 sigma=SIGMA, theta0=None, lo=LO, hi=HI, v_min: float = V_MIN):
        self.nominal = nominal or DynVehicle()
        self.fn = step_function(self.nominal, h)
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.theta = np.array(X0 if theta0 is None else theta0, float)
        self.p0_sd = (self.hi - self.lo) / 4.0                  # L6's EKF prior
        self.P = np.diag(self.p0_sd ** 2)
        lam = np.ones(len(PARAMS))
        lam[PARAMS.index("mu")] = lam_mu
        self.d = lam ** -0.5
        self.sigma, self.v_min = np.asarray(sigma, float), v_min
        self.n_updates = self.n_skipped = self.n_clipped = 0
        self.sq_eps = 0.0                                        # sum of squared scaled prior residuals
        self.time = 0.0

    @property
    def mu(self) -> float:
        return float(self.theta[0])

    def update(self, z, u, y) -> None:
        t0 = time.perf_counter()
        if z[0] <= self.v_min:
            self.n_skipped += 1
            return
        yh, J = self.fn(z, u, self.theta)
        eps = (np.asarray(y, float) - np.asarray(yh).ravel()) / self.sigma
        psi = np.asarray(J) / self.sigma[:, None]
        Ppt = self.P @ psi.T
        L = np.linalg.solve(np.eye(3) + psi @ Ppt, Ppt.T).T
        self.theta = np.clip(self.theta + L @ eps, self.lo, self.hi)
        P = self.P - L @ (psi @ self.P)
        P = self.d[:, None] * P * self.d[None, :]
        P = 0.5 * (P + P.T)
        Pt = P / np.outer(self.p0_sd, self.p0_sd)                # in units of the prior
        w, V = np.linalg.eigh(Pt)
        if w[-1] > 1.0:
            self.n_clipped += 1
            P = ((V * np.minimum(w, 1.0)) @ V.T) * np.outer(self.p0_sd, self.p0_sd)
        self.P = P
        self.n_updates += 1
        self.sq_eps += float(eps @ eps)
        self.time += time.perf_counter() - t0

    def diagnostics(self) -> dict:
        n = max(self.n_updates, 1)
        return {"updates": self.n_updates, "skipped": self.n_skipped, "clipped_share": self.n_clipped / n,
                "one_step_rms_scaled": float(np.sqrt(self.sq_eps / (3 * n))),
                "update_us_mean": 1e6 * self.time / max(self.n_updates + self.n_skipped, 1)}


__all__ = ["SIGMA", "step_function", "step_np", "RecursiveLS"]
