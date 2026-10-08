"""Nonlinear MPC for the dynamic single-track model in curvilinear coordinates (L2).

State   x = [s, n, xi, vx, vy, r, delta]    (s: centre-line arclength of the CG, unwrapped)
Input   u = [delta_rate, F_drive, F_brake]  (forces scaled by 1e4 inside the NLP)

    s'  = (vx cos xi - vy sin xi) / (1 - n kappa(s))
    n'  = vx sin xi + vy cos xi
    xi' = r - kappa(s) s'
    (vx, vy, r)' from dynamics.body_accel,  delta' = delta_rate

Two objectives on the same constraints:

* ``mode="track"``: follow a reference (n_ref, xi_ref, v_ref as functions of s),
* ``mode="mpcc"``: maximise the progress s_N - s_0 (no reference line); the
  terminal speed is bounded by a reference speed profile so that the short
  horizon cannot run into a corner it can no longer brake for.

Constraints: friction ellipse of both axles (factor ``eta``), slip angles within
the tyre peak, power, steering angle and rate, and track limits for the front
and rear axle points (softened with a heavily penalised slack so that the NLP
stays feasible after a disturbance). Multiple shooting with one RK4 step per MPC step, IPOPT with warm
start; if a solve fails, the shifted previous plan is applied instead.
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dyn_tracking import traction_limit

FS = 1e4


def _periodic_interpolant(name: str, s: np.ndarray, f: np.ndarray, length: float, laps: int = 3):
    """Linear interpolant of a periodic field over [-length, laps*length]."""
    grid = np.concatenate([s + k * length for k in range(-1, laps)])
    vals = np.tile(f, laps + 1)
    return ca.interpolant(name, "linear", [grid], vals)


class FrenetMPC:
    def __init__(self, track: Track, veh: DynVehicle, ref: dict, mode: str = "track",
                 N: int = 30, dt: float = 0.05, eta=0.97, margin: float = 0.3,
                 weights: dict | None = None, max_iter: int = 200, backoff=None):
        """``ref``: arrays over centre-line arclength ref['s'] with keys n, xi, vx (the
        reference trajectory, e.g. an L1 plan; in MPCC mode only vx is used, as a
        terminal speed bound). ``eta``: friction factor, a number or one value per
        stage (N); ``backoff``: optional extra distance [m] kept from the track edges
        per predicted state (N + 1 values, the first unused), e.g. stochastic
        constraint tightening (L3)."""
        eta_k = np.broadcast_to(np.asarray(eta, float), (N,)).copy()
        b_k = np.zeros(N + 1) if backoff is None else np.asarray(backoff, float)
        self.track, self.veh, self.mode, self.N, self.dt = track, veh, mode, N, dt
        self.eta, self.eta_k, self.backoff = eta, eta_k, b_k
        w = {"n": 1.0, "xi": 10.0, "v": 0.2, "drate": 1.0, "dF": 0.5, "slack": 1e4, "progress": 1.0}
        w.update(weights or {})
        self.w = w
        L = track.length
        kap = _periodic_interpolant("kap", track.s, track.kappa, L)
        wl = _periodic_interpolant("wl", track.s, track.w_left, L)
        wr = _periodic_interpolant("wr", track.s, track.w_right, L)
        nref = _periodic_interpolant("nref", ref["s"], ref["n"], L)
        xiref = _periodic_interpolant("xiref", ref["s"], ref["xi"], L)
        vref = _periodic_interpolant("vref", ref["s"], ref["vx"], L)

        x = ca.SX.sym("x", 7)
        u = ca.SX.sym("u", 3)
        s, n, xi, vx, vy, r, d = (x[i] for i in range(7))
        k = kap(s)
        F = tyre_forces(veh, vx, vy, r, d, u[1] * FS, u[2] * FS, lib=ca)
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F, lib=ca)
        sdot = (vx * ca.cos(xi) - vy * ca.sin(xi)) / (1 - n * k)
        f = ca.Function("f", [x, u], [ca.vertcat(sdot, vx * ca.sin(xi) + vy * ca.cos(xi),
                                                 r - k * sdot, dvx, dvy, dr, u[0])])
        h = dt
        k1 = f(x, u); k2 = f(x + h / 2 * k1, u); k3 = f(x + h / 2 * k2, u); k4 = f(x + h * k3, u)
        step = ca.Function("step", [x, u], [x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)])

        X = ca.SX.sym("X", 7, N + 1)
        U = ca.SX.sym("U", 3, N)
        S = ca.SX.sym("S", 2, N + 1)                  # track-limit slacks (front, rear points)
        P = ca.SX.sym("P", 7 + 3)                     # initial state, previous input
        g, lbg, ubg = [], [], []
        J = 0
        g.append(X[:, 0] - P[:7]); lbg += [0] * 7; ubg += [0] * 7
        body = veh.half_width + margin
        for i in range(N):
            g.append(X[:, i + 1] - step(X[:, i], U[:, i])); lbg += [0] * 7; ubg += [0] * 7
            Fi = tyre_forces(veh, X[3, i], X[4, i], X[5, i], X[6, i], U[1, i] * FS, U[2, i] * FS, lib=ca)
            uf2 = (Fi["fxf"] / (veh.mu_x * Fi["fzf"])) ** 2 + (Fi["fyf"] / (veh.mu_y * Fi["fzf"])) ** 2
            ur2 = (Fi["fxr"] / (veh.mu_x * Fi["fzr"])) ** 2 + (Fi["fyr"] / (veh.mu_y * Fi["fzr"])) ** 2
            g += [uf2, ur2, Fi["af"], Fi["ar"], U[1, i] * FS * X[3, i]]
            lbg += [-ca.inf, -ca.inf, -veh.alpha_peak, -veh.alpha_peak, -ca.inf]
            ubg += [eta_k[i]**2, eta_k[i]**2, veh.alpha_peak, veh.alpha_peak, veh.P_max]
            du = U[:, i] - (P[7:] if i == 0 else U[:, i - 1])
            J += h * (w["drate"] * U[0, i] ** 2 + w["dF"] * (du[1] ** 2 + du[2] ** 2) / h)
        for i in range(1, N + 1):
            si, ni, xii = X[0, i], X[1, i], X[2, i]
            ki = kap(si)
            for j, arm in enumerate((veh.lf, -veh.lr)):
                chord = 0.5 * arm**2 * ca.fabs(ki)
                na = ni + arm * ca.sin(xii)
                g += [na + (wr(si) - body - chord) + S[j, i], (wl(si) - body - chord) - na + S[j, i]]
                lbg += [b_k[i], b_k[i]]; ubg += [ca.inf, ca.inf]
            J += w["slack"] * h * ca.sumsqr(S[:, i]) + 10 * w["slack"] * h * ca.sum1(S[:, i])
            if mode == "track":
                J += h * (w["n"] * (ni - nref(si)) ** 2 + w["xi"] * (xii - xiref(si)) ** 2
                          + w["v"] * (X[3, i] - vref(si)) ** 2)
        if mode == "mpcc":
            J += -w["progress"] * (X[0, N] - X[0, 0])
            g.append(X[3, N] - vref(X[0, N])); lbg.append(-ca.inf); ubg.append(0.0)

        z = ca.vertcat(ca.vec(X), ca.vec(U), ca.vec(S))
        self.nx = 7 * (N + 1)
        self.nu = 3 * N
        lbz = np.full(z.shape[0], -np.inf)
        ubz = np.full(z.shape[0], np.inf)
        Xi = np.arange(self.nx).reshape(N + 1, 7)          # vec() is column-major: X[:, i] contiguous
        Ui = self.nx + np.arange(self.nu).reshape(N, 3)
        Si = self.nx + self.nu + np.arange(2 * (N + 1)).reshape(N + 1, 2)
        lbz[Xi[:, 3]] = 2.0
        lbz[Xi[:, 6]], ubz[Xi[:, 6]] = -veh.delta_max, veh.delta_max
        lbz[Ui[:, 0]], ubz[Ui[:, 0]] = -veh.delta_rate_max, veh.delta_rate_max
        lbz[Ui[:, 1]], ubz[Ui[:, 1]] = 0.0, np.inf
        lbz[Ui[:, 2]], ubz[Ui[:, 2]] = -np.inf, 0.0
        lbz[Si] = 0.0
        ubz[Si[0]] = 0.0                                # slack at the (fixed) initial state unused
        self.Xi, self.Ui, self.Si = Xi, Ui, Si
        self.lbz, self.ubz = lbz, ubz
        self.lbg, self.ubg = np.array(lbg, float), np.array(ubg, float)
        nlp = {"x": z, "f": J, "g": ca.vertcat(*g), "p": P}
        self.solver = ca.nlpsol("mpc", "ipopt", nlp, {
            "print_time": False,
            "ipopt": {"print_level": 0, "max_iter": max_iter, "tol": 1e-4, "acceptable_tol": 1e-3,
                      "acceptable_iter": 5, "warm_start_init_point": "yes", "mu_init": 1e-3,
                      "linear_solver": "mumps"}})
        self.z = None
        self.lam_g = None
        self.lam_x = None
        self.stats: list[dict] = []

    def initial_guess(self, x0: np.ndarray, u0: np.ndarray) -> np.ndarray:
        """Roll the nominal model forward with a constant input for the first solve."""
        z = np.zeros(self.lbz.shape)
        xs = np.array(x0, float)
        for i in range(self.N + 1):
            z[self.Xi[i]] = xs
            xs = xs.copy()
            xs[0] += self.dt * xs[3]
        for i in range(self.N):
            z[self.Ui[i]] = [0.0, u0[1] / FS, u0[2] / FS]
        return z

    def solve(self, x0: np.ndarray, u_prev: np.ndarray) -> tuple[np.ndarray, dict]:
        p = np.concatenate([x0, [u_prev[0], u_prev[1] / FS, u_prev[2] / FS]])
        if self.z is None:
            z0 = self.initial_guess(x0, u_prev)
            args = {}
        else:                                           # shift the previous solution
            z0 = self.z.copy()
            X = self.z[self.Xi]
            U = self.z[self.Ui]
            z0[self.Xi[:-1]] = X[1:]
            z0[self.Ui[:-1]] = U[1:]
            z0[self.Xi[0]] = x0
            args = {"lam_g0": self.lam_g, "lam_x0": self.lam_x}
        t0 = time.time()
        sol = self.solver(x0=z0, p=p, lbx=self.lbz, ubx=self.ubz, lbg=self.lbg, ubg=self.ubg, **args)
        st = self.solver.stats()
        ok = st["return_status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
        fallback = False
        if ok or self.z is None:
            z = np.asarray(sol["x"]).ravel()
            self.z, self.lam_g, self.lam_x = z, np.asarray(sol["lam_g"]).ravel(), np.asarray(sol["lam_x"]).ravel()
        else:                                           # keep the shifted previous plan
            z, fallback = z0, True
            self.z = z0
        u = z[self.Ui[0]] * np.array([1.0, FS, FS])
        info = {"status": st["return_status"], "iter": st["iter_count"], "time": time.time() - t0,
                "slack": float(np.max(z[self.Si])), "fallback": fallback}
        self.stats.append(info)
        return u, info


    def prediction(self) -> tuple[np.ndarray, np.ndarray]:
        """Predicted states (N + 1, 7) and inputs (N, 3, forces in N) of the last solve."""
        return self.z[self.Xi].copy(), self.z[self.Ui] * np.array([1.0, FS, FS])


class MPCController:
    """Plant interface: converts the simulator state to curvilinear coordinates,
    re-solves the MPC every ``mpc.dt`` and holds the input in between."""

    def __init__(self, mpc: FrenetMPC, sim_dt: float, tc_eta: float | None = 0.99,
                 record: bool = False):
        """``tc_eta``: optional low-level traction control / ABS applied at every
        simulator step (the MPC input is held for mpc.dt while the state moves).
        ``record``: keep every prediction (for calibrating constraint tightening, L3)."""
        self.mpc, self.sim_dt, self.tc_eta = mpc, sim_dt, tc_eta
        self.record = record
        self.predictions: list[tuple[float, np.ndarray, np.ndarray]] = []
        self.track = mpc.track
        self.u = np.zeros(3)
        self.t_next = 0.0
        self.s_unwrapped = None
        self.log: list[tuple] = []

    def frenet(self, x: np.ndarray) -> np.ndarray:
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
        xi = float(np.angle(np.exp(1j * (psi - tr.heading_at(s)))))
        return np.array([su, n, xi, vx, vy, r, d])

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            u, info = self.mpc.solve(xf, self.u)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            if self.record:
                self.predictions.append((t, *self.mpc.prediction()))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb
