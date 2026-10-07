"""Minimum-lap-time optimal control of the dynamic single-track model (L1).

Curvilinear coordinates of the CG along the centre line, arclength as the
independent variable:

    states    n, xi, vx, vy, r, delta
    controls  delta_rate, F_drive (>= 0, rear axle), F_brake (<= 0, balance-split)
    dt/ds   = (1 - n kappa_c) / (vx cos xi - vy sin xi)
    dn/ds   = (vx sin xi + vy cos xi) dt/ds
    dxi/ds  = r dt/ds - kappa_c
    d(vx, vy, r)/ds = body accelerations (dynamics.py) * dt/ds
    ddelta/ds = delta_rate dt/ds

Constraints: friction ellipse of each axle (scaled by eta) and power at both
ends of every interval (the forces are held over the interval), slip angles
on the stable side of the tyre peak, steering angle/rate, track limits for the
car body around the front and rear axle. Periodic lap, objective = lap time.
Each interval is integrated with ``nsub`` RK4 sub-steps (the lateral tyre
dynamics are stiff at low speed).
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, ellipse_usage, tyre_forces
from ..track import Track

FS = 1e4  # force scaling of the decision variables


@dataclass
class DynOCPResult:
    s: np.ndarray
    n: np.ndarray
    xi: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    delta: np.ndarray
    delta_rate: np.ndarray
    f_drive: np.ndarray
    f_brake: np.ndarray
    lap_time: float
    success: bool
    status: str
    iterations: int
    solve_time: float

    def xy(self, track: Track) -> np.ndarray:
        return track.point_at(self.s, self.n)


def _rhs(veh: DynVehicle):
    x = ca.MX.sym("x", 6)
    u = ca.MX.sym("u", 3)
    k = ca.MX.sym("k")
    n, xi, vx, vy, r, d = (x[i] for i in range(6))
    F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
    dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
    sf = (1 - n * k) / (vx * ca.cos(xi) - vy * ca.sin(xi))
    rhs = ca.vertcat((vx * ca.sin(xi) + vy * ca.cos(xi)) * sf, r * sf - k,
                     dvx * sf, dvy * sf, dr * sf, u[0] * sf)
    return ca.Function("rhs", [x, u, k], [rhs, sf])


def _interval(veh: DynVehicle, h: float, nsub: int):
    """One interval of length h: nsub RK4 steps; kappa given at 2*nsub+1 points."""
    f = _rhs(veh)
    x = ca.MX.sym("x", 6)
    u = ca.MX.sym("u", 3)
    kv = ca.MX.sym("kv", 2 * nsub + 1)
    hs = h / nsub
    xk, t = x, 0
    for j in range(nsub):
        k1, t1 = f(xk, u, kv[2 * j])
        k2, t2 = f(xk + 0.5 * hs * k1, u, kv[2 * j + 1])
        k3, t3 = f(xk + 0.5 * hs * k2, u, kv[2 * j + 1])
        k4, t4 = f(xk + hs * k3, u, kv[2 * j + 2])
        xk = xk + hs / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        t = t + hs / 6 * (t1 + 2 * t2 + 2 * t3 + t4)
    return ca.Function("interval", [x, u, kv], [xk, t])


def _limits(veh: DynVehicle, X, U, eta):
    vx, vy, r, d = X[2, :], X[3, :], X[4, :], X[5, :]
    F = tyre_forces(veh, vx, vy, r, d, U[1, :] * FS, U[2, :] * FS, lib=ca)
    sq = lambda a: a * a
    uf2 = sq(F["fxf"] / (veh.mu_x * F["fzf"])) + sq(F["fyf"] / (veh.mu_y * F["fzf"]))
    ur2 = sq(F["fxr"] / (veh.mu_x * F["fzr"])) + sq(F["fyr"] / (veh.mu_y * F["fzr"]))
    return F, uf2, ur2


def solve_min_time_dynamic(track: Track, veh: DynVehicle, init: dict, ds: float = 2.5,
                           nsub: int = 2, margin: float = 0.3, eta: float = 1.0,
                           max_iter: int = 3000, max_cpu_time: float = 900.0,
                           verbose: bool = False) -> DynOCPResult:
    """``init``: arrays over centre-line arclength init['s'] with keys
    n, xi, vx, vy, r, delta, f_drive, f_brake."""
    N = int(round(track.length / ds))
    h = track.length / N
    s = np.arange(N) * h
    sk = s[:, None] + np.linspace(0.0, h, 2 * nsub + 1)[None, :]
    KV = track.kappa_at(sk).T                       # (2 nsub + 1, N)
    kap0 = KV[0]
    wl = track.interp(track.w_left, s)
    wr = track.interp(track.w_right, s)

    step = _interval(veh, h, nsub).map(N)
    opti = ca.Opti()
    X = opti.variable(6, N)
    U = opti.variable(3, N)
    Xn = ca.horzcat(X[:, 1:], X[:, :1])
    Xe, dt = step(X, U, KV)
    opti.subject_to(Xn == Xe)
    T = ca.sum2(dt)

    n, xi, vx, vy, r, d = (X[i, :] for i in range(6))
    for XX in (X, Xn):                               # forces held over the interval
        F, uf2, ur2 = _limits(veh, XX, U, eta)
        opti.subject_to(uf2 <= eta**2)
        opti.subject_to(ur2 <= eta**2)
        opti.subject_to(U[1, :] * FS * XX[2, :] <= veh.P_max)
        opti.subject_to(opti.bounded(-veh.alpha_peak, F["af"], veh.alpha_peak))
        opti.subject_to(opti.bounded(-veh.alpha_peak, F["ar"], veh.alpha_peak))
    opti.subject_to(U[1, :] >= 0)
    opti.subject_to(U[2, :] <= 0)
    opti.subject_to(opti.bounded(-veh.delta_max, d, veh.delta_max))
    opti.subject_to(opti.bounded(-veh.delta_rate_max, U[0, :], veh.delta_rate_max))
    opti.subject_to(vx >= 3.0)
    body = veh.half_width + margin
    for arm in (veh.lf, -veh.lr):                    # front and rear axle points
        chord = 0.5 * arm**2 * np.abs(kap0)
        na = n + arm * ca.sin(xi)
        opti.subject_to(opti.bounded((-wr + body + chord).reshape(1, -1), na,
                                     (wl - body - chord).reshape(1, -1)))

    reg = (1e-3 * ca.sumsqr(U[0, :]) * h / track.length
           + 1e-3 * (ca.sumsqr(ca.diff(U[1, :])) + ca.sumsqr(ca.diff(U[2, :]))) / N)
    opti.minimize(T + reg)

    g = lambda key: np.interp(s, np.asarray(init["s"]), np.asarray(init[key]), period=track.length)
    for i, key in enumerate(["n", "xi", "vx", "vy", "r", "delta"]):
        opti.set_initial(X[i, :], g(key))
    opti.set_initial(U[0, :], 0.0)
    opti.set_initial(U[1, :], np.maximum(g("f_drive"), 0.0) / FS)
    opti.set_initial(U[2, :], np.minimum(g("f_brake"), 0.0) / FS)

    opti.solver("ipopt", {"expand": True, "print_time": False},
                {"max_iter": max_iter, "max_cpu_time": max_cpu_time,
                 "tol": 1e-6, "acceptable_tol": 1e-4, "acceptable_iter": 10,
                 "mu_strategy": "adaptive", "print_level": 5 if verbose else 0,
                 "linear_solver": "mumps"})
    t0 = time.time()
    try:
        sol, ok = opti.solve(), True
    except RuntimeError:
        sol, ok = opti.debug, False
    st = sol.stats()
    val = lambda e: np.asarray(sol.value(e)).ravel()
    return DynOCPResult(s, val(n), val(xi), val(vx), val(vy), val(r), val(d), val(U[0, :]),
                        val(U[1, :]) * FS, val(U[2, :]) * FS, float(sol.value(T)), ok,
                        st.get("return_status", "?"), int(st.get("iter_count", -1)),
                        time.time() - t0)


def initial_guess_from_kinematic(track: Track, veh: DynVehicle, ocp0) -> dict:
    """Initial guess from the L0 (rear-axle, kinematic) OCP solution."""
    v = ocp0.v
    r = v * np.tan(ocp0.delta) / veh.L
    fx = ocp0.fx
    return {"s": ocp0.s, "n": ocp0.n + veh.lr * np.sin(ocp0.xi), "xi": ocp0.xi,
            "vx": v, "vy": veh.lr * r, "r": r, "delta": ocp0.delta,
            "f_drive": np.maximum(fx, 0.0), "f_brake": np.minimum(fx, 0.0)}


def initial_guess_from_dynamic(o: DynOCPResult) -> dict:
    """Warm start from another dynamic solution (e.g. eta = 1 for a run with a margin)."""
    return {"s": o.s, "n": o.n, "xi": o.xi, "vx": o.vx, "vy": o.vy, "r": o.r, "delta": o.delta,
            "f_drive": o.f_drive, "f_brake": o.f_brake}
