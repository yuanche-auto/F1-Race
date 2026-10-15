"""A cheap safety filter for L7: a control-barrier-function QP on a point-mass model.

Candidate for use *inside* RL training (the full NMPC filter, ``FrenetMPC(mode="filter")``,
is the other candidate). Model: the CG as a point mass in curvilinear coordinates,
driven by its longitudinal and lateral acceleration (a_x, a_y):

    n'  = v sin(xi)
    xi' = a_y / v - kappa(s) s',   s' = v cos(xi) / (1 - n kappa)
    v'  = a_x

so that n'' = a_x sin(xi) + cos(xi) (a_y - kappa v^2 cos(xi) / (1 - n kappa)) is affine in
(a_x, a_y). Decision variables (a_x, a_y, sigma_e, sigma_f); constraints:

* **track edges** as second-order barriers (relative degree 2 in the accelerations): for
  h_l = w_l(s) - b - n and h_r = n + w_r(s) - b,  h'' + 2 alpha h' + alpha^2 h >= -sigma_e,
  with the change of the track width along the lap in h' (w' s') and h'' (w'' s'^2; the
  w' s'' term is neglected). h uses the track's widths as they are; w' and w'' come from
  the widths smoothed along the lap (periodic Gaussian, ``width_smoothing`` = 20 m) – the
  raw widths are piecewise linear between points ~5 m apart, and their second difference
  has spikes at the points of up to ~350 m/s^2 in w'' s'^2 (review of 39b62c4, r-m1); the
  smoothed width differs from the raw one by < 1 m and its w'' v^2 stays below ~4 m/s^2. The barrier is on the CG; the car's extent enters the margin
  b = half width + margin + l_f |sin(xi_h)| + l_f^2 |kappa| / 2 (the front axle swung out by
  the heading angle xi_h against the centre line, and the chord of the curve, as the NMPC's
  axle points), held constant within a solve (its derivative is neglected);
* **friction**: (a_x, a_y) inside a regular 16-gon inscribed in the circle of radius
  eta mu_y (g + q_lift v^2 / m), softened by sigma_f (so that the QP stays solvable when the
  car is already beyond the limit);
* **what the actuators can reach** within one control step (``ctrl_dt``): a_y between
  v^2 tan(delta_lo) / L and v^2 tan(delta_hi) / L, delta_lo/hi the steering angle moved by
  the rate limit and clipped to the angle limit; a_x at most (P_max / v - resistance) / m.
  Without these bounds the QP could pick an a_y that the steering cannot produce, and the
  barrier would fail exactly when a large correction is needed.

Objective: (a_y - a_y,des)^2 + w_x (a_x - a_x,des)^2 + w_sigma (sigma_e + sigma_f). The
proposed command (steering rate, drive and brake force, as the tracker outputs it) is
mapped to accelerations with a kinematic steering model, a_y = v^2 tan(delta) / L, and
back. The risk estimate of L7 (identified structure, calibrated back-offs, conformal
intervals) enters as the back-off and the friction factor ``eta``. If the QP solver reports
a failure (with the softened constraints this should not happen), the proposed command is
passed through unchanged and the failure is counted.
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle
from ..track import Track
from ..vehicle import G

N_POLY = 16
BIG = 1e20          # finite stand-in for infinity (DAQP needs finite bounds)


def width_derivatives(track: Track, sigma: float = 20.0, h: float = 1.0):
    """w' and w'' of both track widths, smoothed along the lap by a periodic Gaussian of
    standard deviation ``sigma`` [m], on a grid of about ``h`` m:
    (grid, {side: (w', w'', smoothed width)})."""
    n = int(round(track.length / h))
    h = track.length / n
    g = np.arange(n) * h
    k = np.arange(n)
    k = np.minimum(k, n - k) * h                       # periodic distance to the first point
    ker = np.exp(-0.5 * (k / sigma) ** 2) if sigma > 0 else (k == 0).astype(float)
    ker /= ker.sum()
    out = {}
    for side, w0 in (("left", track.w_left), ("right", track.w_right)):
        w = np.fft.irfft(np.fft.rfft(track.interp(w0, g)) * np.fft.rfft(ker), n)
        out[side] = ((np.roll(w, -1) - np.roll(w, 1)) / (2 * h),
                     (np.roll(w, -1) - 2 * w + np.roll(w, 1)) / h**2, w)
    return g, out


class CBFQPFilter:
    def __init__(self, track: Track, veh: DynVehicle, eta: float = 0.95, margin: float = 0.3,
                 alpha: float = 2.0, w_x: float = 0.1, w_sigma: float = 1e4, solver: str = "qpoases",
                 ctrl_dt: float = 0.01, width_smoothing: float = 20.0):
        self.track, self.veh, self.eta, self.alpha = track, veh, eta, alpha
        self.width_grid, self.width_derivs = width_derivatives(track, width_smoothing)
        self.b = veh.half_width + margin
        self.w_x, self.w_sigma, self.ctrl_dt = w_x, w_sigma, ctrl_dt
        nc = 2 + N_POLY
        H = ca.DM([[2 * w_x, 0, 0, 0], [0, 2.0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]])
        A = ca.DM.zeros(nc, 4)
        A[:2, :3] = 1                                   # barriers: a_x, a_y, sigma_e
        A[2:, [0, 1, 3]] = 1                            # friction: a_x, a_y, sigma_f
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
        d_next = float(np.clip(float(x[6]) + float(np.clip(u[0], -veh.delta_rate_max, veh.delta_rate_max))
                               * self.ctrl_dt, -veh.delta_max, veh.delta_max))
        return ((u[1] + u[2] - veh.resist(v * v)) / veh.m, v * v * np.tan(d_next) / veh.L)

    def command_of(self, x: np.ndarray, ax: float, ay: float) -> np.ndarray:
        veh, v = self.veh, max(float(x[3]), 1.0)
        d = float(np.arctan(ay * veh.L / (v * v)))
        f = veh.m * ax + veh.resist(v * v)
        rate = float(np.clip((d - float(x[6])) / self.ctrl_dt, -veh.delta_rate_max, veh.delta_rate_max))
        return np.array([rate, max(f, 0.0), min(f, 0.0)])

    def _width(self, side: str, s: float) -> tuple[float, float, float]:
        """Width (raw), w' and w'' (smoothed) of one side at arclength s."""
        tr, g = self.track, self.width_grid
        w = float(tr.interp(tr.w_left if side == "left" else tr.w_right, s))
        d1, d2, _ = self.width_derivs[side]
        sm = s % tr.length
        return w, float(np.interp(sm, g, d1, period=tr.length)), float(np.interp(sm, g, d2, period=tr.length))

    def filter(self, frenet: tuple, x: np.ndarray, u: np.ndarray,
               backoff: float = 0.0) -> tuple[np.ndarray, dict]:
        """``frenet`` = (s, n, xi, xi_h) of the CG: arclength, lateral position, course angle
        and heading angle against the centre line (a 3-tuple means xi_h = xi); ``x``: the
        7-dim state; ``u``: the proposed command. Returns the filtered command and an info
        dict (status, intervention, slacks, time)."""
        t0 = time.perf_counter()
        tr, veh, a = self.track, self.veh, self.alpha
        s, n, xi = frenet[:3]
        xi_h = frenet[3] if len(frenet) > 3 else xi
        v = max(float(x[3]), 1.0)
        k = float(tr.kappa_at(s))
        wl, dwl, ddwl = self._width("left", s)
        wr, dwr, ddwr = self._width("right", s)
        lf = veh.L - veh.lr
        b = self.b + backoff + lf * abs(np.sin(xi_h)) + 0.5 * lf * lf * abs(k)
        sx, cx = np.sin(xi), np.cos(xi)
        nd = v * sx                                     # n'
        sd = v * cx / (1.0 - n * k)                     # s'
        c0 = -cx * k * v * sd                           # n'' = sx a_x + cx a_y + c0
        A = np.zeros((2 + N_POLY, 4))
        lba = np.full(2 + N_POLY, -BIG)
        uba = np.full(2 + N_POLY, BIG)
        # left edge h = wl - b - n:   -(sx ax + cx ay + c0) + wl'' s'^2 + 2a (wl' s' - n') + a^2 h + sigma_e >= 0
        A[0, :3] = [-sx, -cx, 1.0]
        lba[0] = c0 - ddwl * sd * sd - 2 * a * (dwl * sd - nd) - a * a * (wl - b - n)
        # right edge h = n + wr - b:  (sx ax + cx ay + c0) + wr'' s'^2 + 2a (n' + wr' s') + a^2 h + sigma_e >= 0
        A[1, :3] = [sx, cx, 1.0]
        lba[1] = -c0 - ddwr * sd * sd - 2 * a * (nd + dwr * sd) - a * a * (n + wr - b)
        r = self.eta * veh.mu_y * (G + veh.q_lift * v * v / veh.m) * self.poly_r
        A[2:, :2] = self.poly
        A[2:, 3] = -1.0
        uba[2:] = r
        # what the steering and the engine can reach within one control step
        d = float(x[6])
        step = veh.delta_rate_max * self.ctrl_dt
        d_lo, d_hi = max(d - step, -veh.delta_max), min(d + step, veh.delta_max)
        ay_lo, ay_hi = v * v * np.tan(d_lo) / veh.L, v * v * np.tan(d_hi) / veh.L
        ax_hi = (veh.P_max / v - veh.resist(v * v)) / veh.m
        ax_d, ay_d = self.accel_of(x, u)
        g = np.array([-2 * self.w_x * ax_d, -2 * ay_d, self.w_sigma, self.w_sigma])
        sol = self.qp(h=self.H, g=g, a=A, lba=lba, uba=uba, lbx=[-BIG, ay_lo, 0.0, 0.0],
                      ubx=[ax_hi, ay_hi, BIG, BIG])
        st = self.qp.stats()
        z = np.asarray(sol["x"]).ravel()
        ok = bool(st.get("success", False))
        moved = ok and abs(z[0] - ax_d) + abs(z[1] - ay_d) > 1e-3   # [m/s^2], above solver tolerances
        u_out = self.command_of(x, z[0], z[1]) if moved else np.asarray(u, float)
        info = {"ok": ok, "status": st.get("return_status", "?"), "intervened": bool(moved),
                "sigma_edge": float(z[2]) if ok else float("nan"),
                "sigma_friction": float(z[3]) if ok else float("nan"),
                "time": time.perf_counter() - t0, "u": u_out}
        self.stats.append(info)
        return u_out, info


__all__ = ["CBFQPFilter", "width_derivatives"]
