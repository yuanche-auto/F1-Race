"""Receding-horizon minimum-time MPC in the spatial domain (L2).

At every call the dynamic L1 minimum-time problem is solved over the next
``N * ds`` metres of centre line, starting from the measured state: no
reference line is tracked, only the track limits, the friction ellipses and
a terminal speed bound (a speed profile known to be brakeable, e.g. the ideal
L1 plan) shape the motion. A spatial horizon of ~200 m covers the longest
braking zone, which a short time-domain horizon at 85 m/s does not.

States per node: n, xi, vx, vy, r, delta at s0 + k ds; piecewise-constant
inputs delta_rate, F_drive, F_brake; ``nsub`` RK4 sub-steps per interval
(stiff lateral tyre dynamics at low speed). Track limits are softened with a
heavily penalised slack. The friction/slip/power limits are imposed at the start
of every interval (``both_ends=True`` adds the end, as in the offline OCP; the
low-level traction control of the controller covers the rest). The first input
is applied until the next solve.
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dyn_tracking import traction_limit
from .nmpc import _periodic_interpolant

FS = 1e4


class SpatialMinTimeMPC:
    def __init__(self, track: Track, veh: DynVehicle, v_terminal: dict, N: int = 40, ds: float = 5.0,
                 nsub: int = 3, eta: float = 0.97, margin: float = 0.3, max_iter: int = 150,
                 both_ends: bool = False):
        """``v_terminal``: {'s': centre-line arclength, 'vx': speed} bound for the last node.
        It must be brakeable with the MPC's own friction margin, e.g. an offline plan
        solved with the same ``eta`` (a bound from a plan with more grip makes the
        problem infeasible in braking zones)."""
        self.track, self.veh, self.N, self.ds, self.eta = track, veh, N, ds, eta
        L = track.length
        kap = _periodic_interpolant("kap", track.s, track.kappa, L)
        wl = _periodic_interpolant("wl", track.s, track.w_left, L)
        wr = _periodic_interpolant("wr", track.s, track.w_right, L)
        vterm = _periodic_interpolant("vterm", v_terminal["s"], v_terminal["vx"], L)

        x = ca.SX.sym("x", 6)
        u = ca.SX.sym("u", 3)
        k = ca.SX.sym("k")
        n, xi, vx, vy, r, d = (x[i] for i in range(6))
        F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
        sf = (1 - n * k) / (vx * ca.cos(xi) - vy * ca.sin(xi))
        f = ca.Function("f", [x, u, k], [ca.vertcat((vx * ca.sin(xi) + vy * ca.cos(xi)) * sf, r * sf - k,
                                                   dvx * sf, dvy * sf, dr * sf, u[0] * sf), sf])

        X = ca.SX.sym("X", 6, N + 1)
        U = ca.SX.sym("U", 3, N)
        S = ca.SX.sym("S", 2, N + 1)
        P = ca.SX.sym("P", 6 + 1 + 3)                 # x0, s0, previous input
        s0 = P[6]
        hs = ds / nsub
        g, lbg, ubg = [X[:, 0] - P[:6]], [0] * 6, [0] * 6
        J = 0
        body = veh.half_width + margin
        for i in range(N):
            xk, t_i = X[:, i], 0
            for j in range(nsub):
                sa = s0 + i * ds + j * hs
                ka, km, kb = kap(sa), kap(sa + hs / 2), kap(sa + hs)
                k1, t1 = f(xk, U[:, i], ka)
                k2, t2 = f(xk + hs / 2 * k1, U[:, i], km)
                k3, t3 = f(xk + hs / 2 * k2, U[:, i], km)
                k4, t4 = f(xk + hs * k3, U[:, i], kb)
                xk = xk + hs / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
                t_i = t_i + hs / 6 * (t1 + 2 * t2 + 2 * t3 + t4)
            g.append(X[:, i + 1] - xk); lbg += [0] * 6; ubg += [0] * 6
            J += t_i
            for XX in ((X[:, i], X[:, i + 1]) if both_ends else (X[:, i],)):
                Fi = tyre_forces(veh, XX[2], XX[3], XX[4], XX[5], U[1, i] * FS, U[2, i] * FS, lib=ca)
                uf2 = (Fi["fxf"] / (veh.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (veh.mu_y * Fi["fzf"])) ** 2
                ur2 = (Fi["fxr"] / (veh.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (veh.mu_y * Fi["fzr"])) ** 2
                g += [uf2, ur2, Fi["af"], Fi["ar"], U[1, i] * FS * XX[2]]
                lbg += [-ca.inf, -ca.inf, -veh.alpha_peak, -veh.alpha_peak, -ca.inf]
                ubg += [eta**2, eta**2, veh.alpha_peak, veh.alpha_peak, veh.P_max]
            du = U[:, i] - (P[7:] if i == 0 else U[:, i - 1])
            J += 1e-3 * (U[0, i] ** 2 + du[1] ** 2 + du[2] ** 2)
        for i in range(1, N + 1):
            si = s0 + i * ds
            ki = kap(si)
            for j, arm in enumerate((veh.lf, -veh.lr)):
                chord = 0.5 * arm**2 * ca.fabs(ki)
                na = X[0, i] + arm * ca.sin(X[1, i])
                g += [na + (wr(si) - body - chord) + S[j, i], (wl(si) - body - chord) - na + S[j, i]]
                lbg += [0, 0]; ubg += [ca.inf, ca.inf]
            J += 1e3 * ca.sumsqr(S[:, i]) + 1e3 * ca.sum1(S[:, i])
        g.append(X[2, N] - vterm(s0 + N * ds)); lbg.append(-ca.inf); ubg.append(0.0)

        z = ca.vertcat(ca.vec(X), ca.vec(U), ca.vec(S))
        nx, nu = 6 * (N + 1), 3 * N
        self.Xi = np.arange(nx).reshape(N + 1, 6)
        self.Ui = nx + np.arange(nu).reshape(N, 3)
        self.Si = nx + nu + np.arange(2 * (N + 1)).reshape(N + 1, 2)
        lbz = np.full(z.shape[0], -np.inf)
        ubz = np.full(z.shape[0], np.inf)
        lbz[self.Xi[:, 2]] = 2.0
        lbz[self.Xi[:, 5]], ubz[self.Xi[:, 5]] = -veh.delta_max, veh.delta_max
        lbz[self.Ui[:, 0]], ubz[self.Ui[:, 0]] = -veh.delta_rate_max, veh.delta_rate_max
        lbz[self.Ui[:, 1]] = 0.0
        ubz[self.Ui[:, 2]] = 0.0
        lbz[self.Si] = 0.0
        ubz[self.Si[0]] = 0.0
        self.lbz, self.ubz = lbz, ubz
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        self.solver = ca.nlpsol("rhmpc", "ipopt", {"x": z, "f": J, "g": ca.vertcat(*g), "p": P}, {
            "print_time": False,
            "ipopt": {"print_level": 0, "max_iter": max_iter, "tol": 1e-4, "acceptable_tol": 1e-3,
                      "acceptable_iter": 5, "warm_start_init_point": "yes", "mu_init": 1e-3,
                      "linear_solver": "mumps"}})
        self.z = None
        self.s_prev = None
        self.lam_g = self.lam_x = None
        self.stats: list[dict] = []

    def _guess(self, x0, s0):
        N, ds = self.N, self.ds
        z = np.zeros(self.lbz.shape)
        if self.z is None:
            for i in range(N + 1):
                z[self.Xi[i]] = x0
            for i in range(N):
                z[self.Ui[i]] = [0.0, 0.1, 0.0]
            return z, False
        shift = s0 - self.s_prev                        # re-interpolate the previous solution
        grid = np.arange(N + 1) * ds
        X = self.z[self.Xi]
        for c in range(6):
            z[self.Xi[:, c]] = np.interp(grid + shift, grid, X[:, c])
        U = self.z[self.Ui]
        for c in range(3):
            z[self.Ui[:, c]] = np.interp(grid[:-1] + shift, grid[:-1], U[:, c])
        z[self.Xi[0]] = x0
        return z, True

    def solve(self, x0: np.ndarray, s0: float, u_prev: np.ndarray):
        z0, warm = self._guess(x0, s0)
        p = np.concatenate([x0, [s0], [u_prev[0], u_prev[1] / FS, u_prev[2] / FS]])
        t0 = time.time()
        args = {"lam_g0": self.lam_g, "lam_x0": self.lam_x} if warm else {}
        sol = self.solver(x0=z0, p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg, **args)
        st = self.solver.stats()
        ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        if ok or self.z is None:
            z = np.asarray(sol["x"]).ravel()
            self.lam_g, self.lam_x = np.asarray(sol["lam_g"]).ravel(), np.asarray(sol["lam_x"]).ravel()
        else:
            z = z0
        self.z, self.s_prev = z, s0
        u = z[self.Ui[0]] * np.array([1.0, FS, FS])
        info = {"status": st["return_status"], "iter": st["iter_count"], "time": time.time() - t0,
                "slack": float(np.max(z[self.Si])), "fallback": not ok}
        self.stats.append(info)
        return u, info


class SpatialMPCController:
    """Plant interface for SpatialMinTimeMPC (re-solve every ``period`` seconds)."""

    def __init__(self, mpc: SpatialMinTimeMPC, sim_dt: float, period: float = 0.05,
                 tc_eta: float | None = 0.99):
        self.mpc, self.sim_dt, self.period, self.tc_eta = mpc, sim_dt, period, tc_eta
        self.track = mpc.track
        self.u = np.zeros(3)
        self.t_next = 0.0
        self.s_unwrapped = None
        self.log: list[tuple] = []

    def __call__(self, t: float, x: np.ndarray):
        tr = self.track
        px, py, psi, vx, vy, r, d = x
        s, n = tr.project(np.array([px]), np.array([py]))
        s, n = float(s[0]), float(n[0])
        if self.s_unwrapped is None:
            su = s
        else:
            prev = self.s_unwrapped
            su = prev + ((s - prev % tr.length + 0.5 * tr.length) % tr.length - 0.5 * tr.length)
        self.s_unwrapped = su
        if t + 1e-9 >= self.t_next:
            xi = float(np.angle(np.exp(1j * (psi - tr.heading_at(s)))))
            u, info = self.mpc.solve(np.array([n, xi, vx, vy, r, d]), su, self.u)
            self.u = u
            self.t_next = t + self.period
            self.log.append((t, su, n, info["time"], info["status"], info["slack"]))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = d + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb
