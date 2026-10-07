"""Minimum-lap-time optimal control of the kinematic model (direct multiple shooting).

The rear axle is described in curvilinear (Frenet) coordinates of the track
centreline, with the centreline arclength s as independent variable:

    states    n (lateral offset), xi (heading relative to centreline), v, delta
    controls  delta_rate, F_x (longitudinal tyre force), piecewise constant
    dt/ds   = (1 - n kappa_c) / (v cos xi)
    dn/ds   = (1 - n kappa_c) tan xi
    dxi/ds  = (1 - n kappa_c) tan(delta) / (L cos xi) - kappa_c
    dv/ds   = a_x dt/ds,  a_x = (F_x - q_drag v^2 - c_rr Fz) / m
    ddelta/ds = delta_rate dt/ds

Constraints (at both ends of every interval for the held force): friction
ellipse and power; at the nodes: steering angle
and rate, track limits for the car body around the rear axle and around the
front axle (n + L sin xi, with an extra L^2 |kappa_c| / 2 for the chord on
curves). The lap is closed: the state at s = length equals the state at s = 0.
The objective is the lap time plus a tiny smoothing term on the controls.
Each interval is integrated with one RK4 step; solved with IPOPT.
"""
from __future__ import annotations

from dataclasses import dataclass

import casadi as ca
import numpy as np

from ..track import Track
from ..vehicle import G, Vehicle


@dataclass
class OCPResult:
    s: np.ndarray        # centreline arclength of the nodes
    n: np.ndarray
    xi: np.ndarray
    v: np.ndarray
    delta: np.ndarray
    delta_rate: np.ndarray
    fx: np.ndarray
    lap_time: float
    success: bool
    status: str
    iterations: int
    solve_time: float

    def xy(self, track: Track) -> np.ndarray:
        return track.point_at(self.s, self.n)


def solve_min_time(track: Track, vehicle: Vehicle, init: dict, ds: float = 4.0,
                   margin: float = 0.3, eta: float = 1.0, max_iter: int = 3000,
                   verbose: bool = False) -> OCPResult:
    """``init``: initial guess as arrays over centreline arclength ``init['s']`` with
    keys 'n', 'xi', 'v', 'delta', 'fx' (see ``initial_guess_from_path``)."""
    veh = vehicle
    N = int(round(track.length / ds))
    h = track.length / N
    s = np.arange(N) * h
    kap0 = track.kappa_at(s)
    kapm = track.kappa_at(s + 0.5 * h)
    kap1 = np.roll(kap0, -1)
    wl = track.interp(track.w_left, s)
    wr = track.interp(track.w_right, s)
    L, m = veh.L, veh.m
    FS = 1e4                                       # force scaling

    x = ca.MX.sym("x", 4)
    u = ca.MX.sym("u", 2)
    k = ca.MX.sym("k")
    n_, xi_, v_, d_ = x[0], x[1], x[2], x[3]
    fx = u[1] * FS
    sf = (1 - n_ * k) / (v_ * ca.cos(xi_))
    v2 = v_ * v_
    ax = (fx - veh.q_drag * v2 - veh.c_rr * (m * G + veh.q_lift * v2)) / m
    rhs = ca.vertcat((1 - n_ * k) * ca.tan(xi_),
                     (1 - n_ * k) * ca.tan(d_) / (L * ca.cos(xi_)) - k,
                     ax * sf,
                     u[0] * sf)
    f = ca.Function("f", [x, u, k], [rhs, sf]).map(N)

    opti = ca.Opti()
    X = opti.variable(4, N)
    U = opti.variable(2, N)
    Xn = ca.horzcat(X[:, 1:], X[:, :1])
    k1, t1 = f(X, U, kap0.reshape(1, -1))
    k2, t2 = f(X + 0.5 * h * k1, U, kapm.reshape(1, -1))
    k3, t3 = f(X + 0.5 * h * k2, U, kapm.reshape(1, -1))
    k4, t4 = f(X + h * k3, U, kap1.reshape(1, -1))
    opti.subject_to(Xn == X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4))
    dt = h / 6 * (t1 + 2 * t2 + 2 * t3 + t4)
    T = ca.sum2(dt)

    n, xi, v, d = X[0, :], X[1, :], X[2, :], X[3, :]
    drate, F = U[0, :], U[1, :] * FS
    v2 = v * v
    fz = m * G + veh.q_lift * v2
    fy = m * v2 * ca.tan(d) / L
    mux, muy = eta * veh.mu_x, eta * veh.mu_y
    opti.subject_to((F / (mux * fz)) ** 2 + (fy / (muy * fz)) ** 2 <= 1)
    # the force is held over the interval: also check it with the end-of-interval state
    v2e = Xn[2, :] ** 2
    fze = m * G + veh.q_lift * v2e
    fye = m * v2e * ca.tan(Xn[3, :]) / L
    opti.subject_to((F / (mux * fze)) ** 2 + (fye / (muy * fze)) ** 2 <= 1)
    opti.subject_to(F * v <= veh.P_max)
    opti.subject_to(F * Xn[2, :] <= veh.P_max)
    opti.subject_to(opti.bounded(-veh.delta_max, d, veh.delta_max))
    opti.subject_to(opti.bounded(-veh.delta_rate_max, drate, veh.delta_rate_max))
    opti.subject_to(v >= 2.0)
    body = veh.half_width + margin
    opti.subject_to(opti.bounded((-wr + body).reshape(1, -1), n, (wl - body).reshape(1, -1)))
    chord = 0.5 * L**2 * np.abs(kap0)
    nf = n + L * ca.sin(xi)
    opti.subject_to(opti.bounded((-wr + body + chord).reshape(1, -1), nf,
                                 (wl - body - chord).reshape(1, -1)))

    reg = 1e-3 * ca.sumsqr(drate) * h / track.length + 1e-3 * ca.sumsqr(ca.diff(U[1, :])) / N
    opti.minimize(T + reg)

    def g(key):
        return np.interp(s, np.asarray(init['s']), np.asarray(init[key]), period=track.length)

    opti.set_initial(X[0, :], g("n"))
    opti.set_initial(X[1, :], g("xi"))
    opti.set_initial(X[2, :], np.maximum(g("v"), 5.0))
    opti.set_initial(X[3, :], g("delta"))
    opti.set_initial(U[0, :], 0.0)
    opti.set_initial(U[1, :], g("fx") / FS)

    opts = {"expand": True, "print_time": False}
    ipopt = {"max_iter": max_iter, "tol": 1e-8, "acceptable_tol": 1e-6,
             "mu_strategy": "adaptive", "print_level": 5 if verbose else 0,
             "linear_solver": "mumps"}
    opti.solver("ipopt", opts, ipopt)
    import time
    t0 = time.time()
    try:
        sol = opti.solve()
        ok = True
    except RuntimeError:
        sol = opti.debug
        ok = False
    st = sol.stats()
    val = sol.value
    return OCPResult(s, np.asarray(val(n)).ravel(), np.asarray(val(xi)).ravel(),
                     np.asarray(val(v)).ravel(), np.asarray(val(d)).ravel(),
                     np.asarray(val(drate)).ravel(), np.asarray(val(F)).ravel(),
                     float(val(T)), ok, st.get("return_status", "?"),
                     int(st.get("iter_count", -1)), time.time() - t0)


def initial_guess_from_path(track: Track, vehicle: Vehicle, idx: np.ndarray, alpha: np.ndarray,
                            path, v: np.ndarray, a: np.ndarray) -> dict:
    """Initial guess from a path whose points are p_i = c[idx_i] + alpha_i n[idx_i]
    (e.g. the minimum-curvature line) and a speed profile on those points."""
    xi = np.angle(np.exp(1j * (path.psi - track.psi[idx])))
    return {"s": track.s[idx], "n": alpha, "xi": xi, "v": v,
            "delta": np.arctan(vehicle.L * path.kappa),
            "fx": vehicle.m * a + vehicle.resist(v**2)}
