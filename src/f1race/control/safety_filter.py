"""A cheap safety filter for L7: a control-barrier-function QP on a point-mass model.

Candidate for use *inside* RL training (the full NMPC filter, ``FrenetMPC(mode="filter")``,
is the other candidate). Model: the CG as a point mass in curvilinear coordinates,
driven by its longitudinal and lateral acceleration (a_x, a_y):

    n'  = v sin(xi)
    xi' = a_y / v - kappa(s) s',   s' = v cos(xi) / (1 - n kappa)
    v'  = a_x

so that n'' = a_x sin(xi) + cos(xi) (a_y - kappa v^2 cos(xi) / (1 - n kappa)) is affine in
(a_x, a_y). Constraints of the QP in (a_x, a_y, sigma):

* track edges as second-order barriers (relative degree 2 in the accelerations): for
  h = (w_l - b - n) and h = (n + w_r - b),  h'' + 2 alpha h' + alpha^2 h >= -sigma,
  sigma >= 0 heavily penalised (the QP stays feasible when the car is already in trouble);
* friction: (a_x, a_y) inside a regular 16-gon inscribed in the circle of radius
  eta mu_y (g + q_lift v^2 / m) (an inner approximation of the friction circle).

Objective: (a_y - a_y,des)^2 + w_x (a_x - a_x,des)^2 + w_sigma sigma. The proposed command
(steering rate, drive and brake force, as the tracker outputs it) is mapped to
accelerations with a kinematic steering model, a_y = v^2 tan(delta) / L, and back. The
risk estimate of L7 (identified structure, calibrated back-offs, conformal intervals)
enters as the back-off ``b`` and the friction factor ``eta``; this module only fixes the
structure, so that its solve time can be measured.
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle
from ..track import Track
from ..vehicle import G

N_POLY = 16


class CBFQPFilter:
    def __init__(self, track: Track, veh: DynVehicle, eta: float = 0.95, margin: float = 0.3,
                 alpha: float = 2.0, w_x: float = 0.1, w_sigma: float = 1e4, solver: str = "qpoases",
                 ctrl_dt: float = 0.01):
        self.track, self.veh, self.eta, self.alpha = track, veh, eta, alpha
        self.b = veh.half_width + margin
        self.w_x, self.w_sigma, self.ctrl_dt = w_x, w_sigma, ctrl_dt
        nc = 2 + N_POLY
        H = ca.DM([[2 * w_x, 0, 0], [0, 2.0, 0], [0, 0, 0]])
        A = ca.DM.ones(nc, 3)
        A[2:, 2] = 0
        opts = {"qpoases": {"printLevel": "none", "error_on_fail": False},
                "osqp": {"osqp": {"verbose": False}, "error_on_fail": False},
                "daqp": {"error_on_fail": False},
                "highs": {"error_on_fail": False, "highs": {"output_flag": False}},
                "proxqp": {"error_on_fail": False}}.get(solver, {"error_on_fail": False})
        self.qp = ca.conic("cbf", solver, {"h": H.sparsity(), "a": A.sparsity()}, opts)
        self.H = H
        th = 2 * np.pi * np.arange(N_POLY) / N_POLY
        self.poly = np.stack([np.cos(th), np.sin(th)], axis=1)
        self.poly_r = np.cos(np.pi / N_POLY)            # inscribed: apothem of the 16-gon
        self.stats: list[dict] = []

    def accel_of(self, x: np.ndarray, u: np.ndarray) -> tuple[float, float]:
        """(a_x, a_y) of a command (steering rate, drive, brake) at state x (7-dim)."""
        veh, v = self.veh, max(float(x[3]), 1.0)
        d_next = float(x[6]) + float(np.clip(u[0], -veh.delta_rate_max, veh.delta_rate_max)) * self.ctrl_dt
        return ((u[1] + u[2] - veh.resist(v * v)) / veh.m, v * v * np.tan(d_next) / veh.L)

    def command_of(self, x: np.ndarray, ax: float, ay: float) -> np.ndarray:
        veh, v = self.veh, max(float(x[3]), 1.0)
        d = float(np.arctan(ay * veh.L / (v * v)))
        f = veh.m * ax + veh.resist(v * v)
        rate = float(np.clip((d - float(x[6])) / self.ctrl_dt, -veh.delta_rate_max, veh.delta_rate_max))
        return np.array([rate, max(f, 0.0), min(f, 0.0)])

    def filter(self, frenet: tuple[float, float, float], x: np.ndarray, u: np.ndarray,
               backoff: float = 0.0) -> tuple[np.ndarray, dict]:
        """``frenet`` = (s, n, xi) of the CG (xi: course angle against the centre line);
        ``x``: the 7-dim state; ``u``: the proposed command. Returns the filtered command
        and an info dict (status, intervention, time)."""
        t0 = time.perf_counter()
        tr, veh, a = self.track, self.veh, self.alpha
        s, n, xi = frenet
        v = max(float(x[3]), 1.0)
        k = float(tr.kappa_at(s))
        wl, wr = float(tr.interp(tr.w_left, s)), float(tr.interp(tr.w_right, s))
        b = self.b + backoff
        sx, cx = np.sin(xi), np.cos(xi)
        nd = v * sx                                     # n'
        c0 = -cx * k * v * v * cx / (1.0 - n * k)       # n'' = sx a_x + cx a_y + c0
        # left edge h = wl - b - n:  -(sx ax + cx ay + c0) - 2a nd + a^2 h + sigma >= 0
        # right edge h = n + wr - b:  (sx ax + cx ay + c0) + 2a nd + a^2 h + sigma >= 0
        A = np.zeros((2 + N_POLY, 3))
        lba = np.full(2 + N_POLY, -np.inf)
        uba = np.full(2 + N_POLY, np.inf)
        A[0] = [-sx, -cx, 1.0]
        lba[0] = c0 + 2 * a * nd - a * a * (wl - b - n)
        A[1] = [sx, cx, 1.0]
        lba[1] = -c0 - 2 * a * nd - a * a * (n + wr - b)
        r = self.eta * veh.mu_y * (G + veh.q_lift * v * v / veh.m) * self.poly_r
        A[2:, :2] = self.poly
        uba[2:] = r
        ax_d, ay_d = self.accel_of(x, u)
        g = np.array([-2 * self.w_x * ax_d, -2 * ay_d, self.w_sigma])
        big = 1e20                                      # finite stand-in for infinity (DAQP needs it)
        sol = self.qp(h=self.H, g=g, a=A, lba=np.maximum(lba, -big), uba=np.minimum(uba, big),
                      lbx=[-big, -big, 0.0], ubx=[big, big, big])
        st = self.qp.stats()
        z = np.asarray(sol["x"]).ravel()
        ok = bool(st.get("success", False))
        if ok:
            u_out = self.command_of(x, z[0], z[1])
            moved = abs(z[0] - ax_d) + abs(z[1] - ay_d) > 1e-3   # [m/s^2], above solver tolerances
        else:                                           # no solution: brake, keep the steering
            u_out, moved = np.array([u[0], 0.0, -1e5]), True
        u_out = u_out if moved else np.asarray(u, float)
        info = {"ok": ok, "status": st.get("return_status", "?"), "intervened": bool(moved),
                "sigma": float(z[2]), "time": time.perf_counter() - t0, "u": u_out}
        self.stats.append(info)
        return u_out, info


__all__ = ["CBFQPFilter"]
