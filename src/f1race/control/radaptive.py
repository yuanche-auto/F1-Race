"""The robust adaptive NMPC of control expansion phase J (docs/CX_J_protocol.md, revision 0 with its clarifications): phase
C's tube around phase D's adaptive model.

``AdaptiveTube`` is ``tube.Tube`` (phase C: linearisation along the shifted prediction, time-varying LQR, vertex
disturbance box, reachable-set bounds, caps) with three changes:

* the model is the estimate theta-hat – the Jacobians, the disturbance vertices and the ellipse usage use it
  (``FrenetMPC(param_model=True, param_tightening=True)``);
* the parameter box is given per solve, [lo, hi] around theta-hat (clipped to the estimator's bounds), and a per-stage
  term d_s (7) is added to the disturbance box W_k (section 3.4, the systematic part);
* an initial state-error box e0 (7) enters every bound through the closed-loop transitions, b_k(C) = sum_j ... + |C Phi_{k,0}| e0
  (section 3.4, the measurement part). Stage 0 is not tightened (clarification m2): eta_0 = eta, back-off 0 unused.

``RobustAdaptiveMPCController`` drives it: every 50 ms it measures the first-stage prediction error of its previous solve
(skipped after a fallback), takes the component-wise 0.9-quantile q of the last 40 (0 with fewer than ten), and hands the
tube d_s = rho_s q and e0 = rho0 q. The box half-widths are h_i = min(c_i, kappa sigma_i) (``shrink``) or c_i, with
sigma_i the RLS standard deviations and c the symmetric cap of section 3.3. Nothing here claims coverage or recursive
feasibility (protocol section 3.6).
"""
from __future__ import annotations

import time

import casadi as ca
import numpy as np

from ..dynamics import tyre_forces
from ..estimation.identify import HI, LO
from .adaptive_mpc import AdaptiveMPCController
from .dyn_tracking import traction_limit
from .mppi import _periodic
from .nmpc import FS, FrenetMPC, param_vehicle
from .tube import PARAM_ORDER, frenet_step, tv_lqr, vertex_vehicle

KAPPA = 2.0                                          # two standard deviations (section 3.3), not tuned
CAP = np.array([0.08, 0.05, 0.10, 0.10, 0.15, 0.15, 0.03])   # the L2 ranges' largest distance from the nominal car
WINDOW, MIN_RESIDUALS, QUANTILE = 40, 10, 0.9


def reach_bounds_e0(Acl: np.ndarray, W: np.ndarray, C: np.ndarray, e0: np.ndarray) -> np.ndarray:
    """``tube.reach_bounds`` with an initial error in the box |e_0,i| <= e0_i: b_k = max |C_k e_k|; b_0 = 0 by the rule of
    clarification m2 (stage 0 is not tightened)."""
    N = Acl.shape[0]
    b = np.zeros(N + 1)
    for k in range(1, N + 1):
        r = C[k].copy()
        tot = 0.0
        for j in range(k - 1, -1, -1):
            tot += float(np.abs(r) @ W[j])
            r = r @ Acl[j]
        b[k] = tot + float(np.abs(r) @ e0)
    return b


class AdaptiveTube:
    """Per-solve tightening for a ``FrenetMPC(param_model=True, param_tightening=True)`` (module doc)."""

    def __init__(self, mpc: FrenetMPC, margin: float, eta: float = 0.95, c_R: float = 1.0, eta_min: float = 0.70,
                 b_max: float | None = None):
        if not (mpc.param_tightening and mpc.param_model):
            raise ValueError("the MPC needs param_model and param_tightening")
        if mpc.steer_lag is not None:
            raise ValueError("the tube is defined for the 7-state model")
        self.mpc, self.veh, self.N, self.h = mpc, mpc.veh, mpc.N, mpc.dt
        self.eta, self.eta_min = float(eta), float(eta_min)
        tr = mpc.track
        self.kap = _periodic(tr.s, tr.kappa, tr.length)
        body = mpc.veh.half_width + float(margin)
        self.b_max = float(np.min(0.5 * (tr.w_left + tr.w_right)) - body) if b_max is None else float(b_max)
        w, h = mpc.w, self.h
        self.Q = h * np.diag([0.0, w["n"], w["xi"], w["v"], 0.0, 0.0, 0.0])
        self.R = float(c_R) * np.diag([h * w["drate"], w["dF"], w["dF"]])
        x, u, th = ca.SX.sym("x", 7), ca.SX.sym("u", 3), ca.SX.sym("th", 7)
        y = mpc._step(x, u, th)
        self._jac = ca.Function("atube_jac", [x, u, th], [ca.jacobian(y, x), ca.jacobian(y, u)]).map(self.N)
        v = param_vehicle(mpc.veh, th)
        F = tyre_forces(v, x[3], x[4], x[5], x[6], u[1] * FS, u[2] * FS, lib=ca)
        g = ca.vertcat((F["fxf"] / (v.mu_x * F["fzf"])) ** 2 + (F["fyf"] / (v.mu_y * F["fzf"])) ** 2,
                       (F["fxr"] / (v.mu_x * F["fzr"])) ** 2 + (F["fyr"] / (v.mu_y * F["fzr"])) ** 2)
        self._gjac = ca.Function("atube_gjac", [x, u, th], [ca.jacobian(g, x), ca.jacobian(g, u)]).map(self.N)

    def _theta(self, theta: np.ndarray) -> np.ndarray:
        return np.repeat(np.asarray(theta, float)[:, None], self.N, axis=1)

    def linearise(self, X: np.ndarray, U: np.ndarray, theta: np.ndarray):
        A, B = self._jac(X[:-1].T, (U / np.array([1.0, FS, FS])).T, self._theta(theta))
        return (np.asarray(A).reshape(7, self.N, 7).transpose(1, 0, 2),
                np.asarray(B).reshape(7, self.N, 3).transpose(1, 0, 2))

    def disturbance(self, X: np.ndarray, U: np.ndarray, theta: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
        """W (N, 7): the largest one-step mismatch over the 128 vertices of [lo, hi] against the model at theta."""
        ver_veh, _ = vertex_vehicle(self.veh, {k: (float(lo[i]), float(hi[i])) for i, k in enumerate(PARAM_ORDER)})
        with np.errstate(all="ignore"):
            nom = frenet_step(param_vehicle(self.veh, theta), self.kap, X[:-1], U, self.h)
            ver = frenet_step(ver_veh, self.kap, X[None, :-1], U[None], self.h)
        return np.max(np.abs(ver - nom[None]), axis=0)

    def compute(self, X: np.ndarray, U: np.ndarray, theta: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                d_s: np.ndarray | None = None, e0: np.ndarray | None = None) -> dict:
        """Tightening for a nominal prediction X (N + 1, 7), U (N, 3) in N at theta with the parameter box [lo, hi]."""
        t0 = time.perf_counter()
        N, veh = self.N, self.veh
        theta = np.asarray(theta, float)
        d_s = np.zeros(7) if d_s is None else np.asarray(d_s, float)
        e0 = np.zeros(7) if e0 is None else np.asarray(e0, float)
        A, B = self.linearise(X, U, theta)
        K = tv_lqr(A, B, self.Q, self.R)
        Acl = A + np.einsum("kij,kjl->kil", B, K)
        W = self.disturbance(X, U, theta, lo, hi) + d_s
        b_track = np.zeros(N + 1)
        for arm in (veh.lf, -veh.lr):
            C = np.zeros((N + 1, 7))
            C[:, 1] = 1.0
            C[:, 2] = arm * np.cos(X[:, 2])
            b_track = np.maximum(b_track, reach_bounds_e0(Acl, W, C, e0))
        b_raw = b_track.copy()
        b_track = np.minimum(b_track, self.b_max)
        Gx, Gu = self._gjac(X[:-1].T, (U / np.array([1.0, FS, FS])).T, self._theta(theta))
        Gx = np.asarray(Gx).reshape(2, N, 7).transpose(1, 0, 2)
        Gu = np.asarray(Gu).reshape(2, N, 3).transpose(1, 0, 2)
        D = np.zeros(N + 1)
        for ax in range(2):
            C = np.zeros((N + 1, 7))
            C[:N] = Gx[:, ax] + np.einsum("kj,kjl->kl", Gu[:, ax], K)
            D = np.maximum(D, reach_bounds_e0(Acl, W, C, e0))
        eta_k = np.sqrt(np.maximum(self.eta ** 2 - D[:N], self.eta_min ** 2))
        capped_eta = (self.eta ** 2 - D[:N]) < self.eta_min ** 2
        return {"eta": eta_k, "backoff": b_track, "b_raw": b_raw, "D": D, "W": W, "K": K,
                "capped_track_share": float(np.mean(b_raw[1:] > self.b_max)),
                "capped_eta_share": float(np.mean(capped_eta[1:])), "time": time.perf_counter() - t0}


class RobustAdaptiveMPCController(AdaptiveMPCController):
    """``AdaptiveMPCController`` with the tube (module doc). ``shrink``: the box h = min(CAP, KAPPA sigma) instead of CAP;
    ``rho0``, ``rho_s``: the residual coefficients of section 3.4."""

    def __init__(self, mpc: FrenetMPC, tube: AdaptiveTube, sim_dt: float, plan_etas, estimator, rho0: float = 0.0,
                 rho_s: float = 0.0, shrink: bool = True, cap: np.ndarray = CAP, kappa: float = KAPPA,
                 margin_eta: float = 0.95, tc_eta: float | None = 0.99):
        super().__init__(mpc, sim_dt, plan_etas, estimator=estimator, source="estimate", margin_eta=margin_eta,
                         tc_eta=tc_eta)
        self.tube, self.rho0, self.rho_s, self.shrink = tube, float(rho0), float(rho_s), bool(shrink)
        self.cap, self.kappa = np.asarray(cap, float), float(kappa)
        self.x1_pred = None                              # the first-stage prediction of the last solve, if it was a solve
        self.residuals: list[np.ndarray] = []
        self.tube_log: list[dict] = []

    def box(self, theta: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(lo, hi, h) around theta, inside the estimator's bounds."""
        h = self.cap.copy()
        if self.shrink:
            h = np.minimum(h, self.kappa * np.sqrt(np.maximum(np.diag(self.est.P), 0.0)))
        return np.maximum(theta - h, LO), np.minimum(theta + h, HI), h

    def quantile(self) -> np.ndarray:
        if len(self.residuals) < MIN_RESIDUALS:
            return np.zeros(7)
        return np.quantile(np.array(self.residuals[-WINDOW:]), QUANTILE, axis=0)

    def _nominal(self, xf: np.ndarray):
        mpc = self.mpc
        if mpc.z is None:
            z = mpc.initial_guess(xf, self.u)
            return z[mpc.Xi], z[mpc.Ui] * np.array([1.0, FS, FS])
        X, U = mpc.z[mpc.Xi], mpc.z[mpc.Ui] * np.array([1.0, FS, FS])
        return np.vstack([xf[None], X[2:], X[-1:]]), np.vstack([U[1:], U[-1:]])

    def __call__(self, t: float, x: np.ndarray):
        x = np.asarray(x, float)
        if self.prev is not None:
            self.est.update(self.prev[0], self.prev[1], x[3:6])
        xf = self.frenet(x)
        th, mu = self.current(xf[0])
        if t + 1e-9 >= self.t_next:
            if self.x1_pred is not None:
                self.residuals.append(np.abs(xf - self.x1_pred))
            q = self.quantile()
            j = self.plan_coordinate(mu)
            lo, hi, h = self.box(th)
            Xn, Un = self._nominal(xf)
            tb = self.tube.compute(Xn, Un, th, lo, hi, self.rho_s * q, self.rho0 * q)
            u, info = self.mpc.solve(xf, self.u, eta=tb["eta"], backoff=tb["backoff"], theta=th, plan=j)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.x1_pred = None if info["fallback"] else self.mpc.z[self.mpc.Xi[1]].copy()
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            self.trace.append((t, xf[0], j, mu, *th))
            self.tube_log.append({"t": t, "s": float(xf[0]), "h": h, "theta": th.copy(), "q": q, "eta": tb["eta"].copy(),
                                  "b_all": tb["backoff"].copy(), "tube_time": tb["time"],
                                  "sigma": np.sqrt(np.maximum(np.diag(self.est.P), 0.0)),
                                  "capped_track": tb["capped_track_share"], "capped_eta": tb["capped_eta_share"],
                                  "X_pred": self.mpc.z[self.mpc.Xi[:11]].copy()})
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            from ..estimation.identify import vehicle_from
            veh = vehicle_from(th, self.mpc.veh)
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        from .adaptive_mpc import applied_input
        self.prev = (x[3:7].copy(), applied_input(self.mpc.veh, x, (drate, fd, fb)))
        return drate, fd, fb


__all__ = ["AdaptiveTube", "RobustAdaptiveMPCController", "reach_bounds_e0", "CAP", "KAPPA", "WINDOW", "MIN_RESIDUALS",
           "QUANTILE"]
