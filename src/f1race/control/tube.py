"""Constraint tightening of the NMPC by an error tube (control expansion, phase C; docs/CX_C_protocol.md,
revision 0 with its clarifications).

Linear tube MPC with a fixed ancillary feedback (Mayne, Seron, Rakovic 2005; Chisci, Rossiter, Zappa 2001), carried
over to ``FrenetMPC``'s nonlinear problem by linearising along the nominal prediction - the "approximate tube of
reachable sets" of Wischnewski et al. (VSD 2022) - with a disturbance set from the L2 parameter ranges. Before every
solve, along the previous solution shifted by one stage (z_k, v_k), k = 0 ... N - 1:

1. A_k = df/dx, B_k = df/du of the NMPC's own discrete step (one RK4 step of h, CasADi Jacobians; inputs scaled as in
   the NLP: forces in 1e4 N).
2. Ancillary feedback: time-varying LQR along the prediction, backward Riccati recursion with the NMPC's stage weights
   Q = h diag(0, w_n, w_xi, w_v, 0, 0, 0), R = c_R diag(h w_drate, w_dF, w_dF), P_N = Q; K_k; A_cl,k = A_k + B_k K_k.
   K is **not applied** in closed loop (protocol clarification m1): the NMPC re-solves from the measured state every
   step (e_0 = 0) and applies its own first input; K only shapes the error reachable sets.
3. Disturbance set: the box W_k with half-widths w_k,i = max over the 128 vertices of the L2 parameter box
   (``experiments.l2.PERTURBATION``) of |f_theta(z_k, v_k) - f(z_k, v_k)|_i - the one-step mismatch at the nominal point
   (one RK4 step of the same Frenet model with the vertex car; numpy, broadcast over vertices and stages).
4. Reachable-set bounds of an output y = C e at stage k (e_0 = 0, e_{k+1} = A_cl,k e_k + w_k, w_k in W_k):
   b_k(C) = sum_{j<k} sum_i |(C Phi_{k,j+1})_i| w_j,i, Phi the closed-loop transitions - the support function of the
   box-driven reachable set.
5. Tightening: the track back-off b_k = max over the front and rear axle points of b_k(e_n + arm cos(xi_k) e_xi), capped
   at ``b_max``; the friction factor eta_k = sqrt(max(eta^2 - D_k, eta_min^2)) with D_k = max over the axles of
   b_k((dg/dx + dg/du K_k)), g the axle's ellipse usage^2 (``track_only``: eta_k = eta). Under ``param_tightening``
   eta_k also bounds the axles' longitudinal-force limits (protocol clarification m2).

The tightening goes to ``FrenetMPC(param_tightening=True)`` as ``eta`` (N) and ``backoff`` (N + 1, stage 0 unused).
What is not claimed (protocol section 3.5): a linearisation, a vertex-maximum disturbance bound at the nominal point,
caps, no terminal set - no recursive feasibility or guaranteed constraint satisfaction.
"""
from __future__ import annotations

import itertools
import time
from dataclasses import replace

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from .mppi import _periodic
from .nmpc import FS, FrenetMPC, MPCController, traction_limit

PARAM_ORDER = ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front")


def vertex_vehicle(nominal: DynVehicle, box: dict) -> tuple[DynVehicle, np.ndarray]:
    """One ``DynVehicle`` whose perturbed fields are arrays of shape (2^7, 1) - the vertices of the parameter box
    (multiplicative factors, ``aero_front`` additive; as ``l2.sample_plants``) - and the vertex table (128, 7)."""
    V = np.array(list(itertools.product(*(box[k] for k in PARAM_ORDER))), float)
    c = {k: V[:, i:i + 1] for i, k in enumerate(PARAM_ORDER)}
    veh = replace(nominal, mu_x=nominal.mu_x * c["mu"], mu_y=nominal.mu_y * c["mu"], m=nominal.m * c["m"],
                  ClA=nominal.ClA * c["ClA"], CdA=nominal.CdA * c["CdA"], Iz=nominal.Iz * c["Iz"],
                  B=nominal.B * c["B"], aero_front=nominal.aero_front + c["aero_front"])
    return veh, V


def frenet_step(veh: DynVehicle, kap, X: np.ndarray, U: np.ndarray, h: float) -> np.ndarray:
    """One RK4 step of ``FrenetMPC``'s model (inputs held; U = [delta_rate, F_drive, F_brake] in N) for states X
    (..., 7); vehicle fields may be arrays that broadcast against the leading axes."""
    def f(x):
        s, n, xi, vx, vy, r, d = (x[..., i] for i in range(7))
        k = kap(s)
        F = tyre_forces(veh, vx, vy, r, d, U[..., 1], U[..., 2])
        dvx, dvy, dr = body_accel(veh, vx, vy, r, d, F)
        sdot = (vx * np.cos(xi) - vy * np.sin(xi)) / (1.0 - n * k)
        return np.stack(np.broadcast_arrays(sdot, vx * np.sin(xi) + vy * np.cos(xi), r - k * sdot, dvx, dvy, dr,
                                            U[..., 0] + 0.0 * vx), axis=-1)
    k1 = f(X)
    k2 = f(X + 0.5 * h * k1)
    k3 = f(X + 0.5 * h * k2)
    k4 = f(X + h * k3)
    return X + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def reach_bounds(Acl: np.ndarray, W: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Support-function bounds b_k = max |C_k e_k| over e_0 = 0, e_{j+1} = Acl_j e_j + w_j, |w_j,i| <= W_j,i.
    Acl (N, n, n), W (N, n), C (N + 1, n) (row k: the output at stage k); returns b (N + 1,), b_0 = 0."""
    N = Acl.shape[0]
    b = np.zeros(N + 1)
    for k in range(1, N + 1):
        r = C[k].copy()
        tot = 0.0
        for j in range(k - 1, -1, -1):          # r = C_k Phi_{k, j+1}; w_j enters e_{j+1}
            tot += float(np.abs(r) @ W[j])
            r = r @ Acl[j]
        b[k] = tot
    return b


def tv_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Time-varying LQR gains K (N, m, n) for x_{k+1} = A_k x_k + B_k u_k by the backward Riccati recursion with stage
    weights Q, R and terminal weight Q; u_k = K_k x_k."""
    N, n, m = A.shape[0], A.shape[1], B.shape[2]
    P = Q.copy()
    K = np.zeros((N, m, n))
    for k in range(N - 1, -1, -1):
        Ak, Bk = A[k], B[k]
        K[k] = -np.linalg.solve(R + Bk.T @ P @ Bk, Bk.T @ P @ Ak)
        P = Q + Ak.T @ P @ (Ak + Bk @ K[k])
        P = 0.5 * (P + P.T)
    return K


class Tube:
    """Per-solve tightening for a ``FrenetMPC`` built with ``param_tightening=True``."""

    def __init__(self, mpc: FrenetMPC, box: dict, margin: float, eta: float = 0.95, c_R: float = 1.0,
                 eta_min: float = 0.70, b_max: float | None = None, track_only: bool = False, null: bool = False):
        """``box``: the parameter ranges (``l2.PERTURBATION``); ``margin``: the NMPC's track margin (``b_max`` = half the
        narrowest track width minus half the car and the margin, unless given). ``null`` (development only, review c1 of
        packet 2): the tube is computed as usual but the tightening handed to the NLP is none (eta_k = eta, back-off 0)."""
        if not mpc.param_tightening:
            raise ValueError("the MPC needs param_tightening=True")
        if mpc.steer_lag is not None:
            raise ValueError("the tube is defined for the 7-state model")
        self.mpc, self.veh, self.N, self.h = mpc, mpc.veh, mpc.N, mpc.dt
        self.eta, self.c_R, self.eta_min, self.track_only = float(eta), float(c_R), float(eta_min), bool(track_only)
        self.null = bool(null)
        tr = mpc.track
        self.kap = _periodic(tr.s, tr.kappa, tr.length)
        body = mpc.veh.half_width + float(margin)
        self.b_max = float(np.min(0.5 * (tr.w_left + tr.w_right)) - body) if b_max is None else float(b_max)
        self.vertex_veh, self.vertices = vertex_vehicle(mpc.veh, box)
        w = mpc.w
        h = self.h
        self.Q = h * np.diag([0.0, w["n"], w["xi"], w["v"], 0.0, 0.0, 0.0])
        self.R = self.c_R * np.diag([h * w["drate"], w["dF"], w["dF"]])
        x, u = ca.SX.sym("x", 7), ca.SX.sym("u", 3)
        y = mpc._step(x, u)
        self._jac = ca.Function("tube_jac", [x, u], [ca.jacobian(y, x), ca.jacobian(y, u)]).map(self.N)
        F = tyre_forces(mpc.veh, x[3], x[4], x[5], x[6], u[1] * FS, u[2] * FS, lib=ca)
        uf2 = (F["fxf"] / (mpc.veh.mu_x * F["fzf"])) ** 2 + (F["fyf"] / (mpc.veh.mu_y * F["fzf"])) ** 2
        ur2 = (F["fxr"] / (mpc.veh.mu_x * F["fzr"])) ** 2 + (F["fyr"] / (mpc.veh.mu_y * F["fzr"])) ** 2
        g = ca.vertcat(uf2, ur2)
        self._gjac = ca.Function("tube_gjac", [x, u], [ca.jacobian(g, x), ca.jacobian(g, u)]).map(self.N)

    # --------------------------------------------------------------- pieces
    def linearise(self, X: np.ndarray, U: np.ndarray):
        """A (N, 7, 7), B (N, 7, 3) at X (N + 1, 7), U (N, 3) in N (B per scaled input, 1e4 N)."""
        Us = U / np.array([1.0, FS, FS])
        A, B = self._jac(X[:-1].T, Us.T)
        A = np.asarray(A).reshape(7, self.N, 7).transpose(1, 0, 2)
        B = np.asarray(B).reshape(7, self.N, 3).transpose(1, 0, 2)
        return A, B

    def lqr(self, A: np.ndarray, B: np.ndarray) -> np.ndarray:
        """Time-varying LQR gains K (N, 3, 7) along the prediction, P_N = Q."""
        return tv_lqr(A, B, self.Q, self.R)

    def disturbance(self, X: np.ndarray, U: np.ndarray) -> np.ndarray:
        """W (N, 7): the largest one-step mismatch over the vertices at (X_k, U_k)."""
        with np.errstate(all="ignore"):
            nom = frenet_step(self.veh, self.kap, X[:-1], U, self.h)
            ver = frenet_step(self.vertex_veh, self.kap, X[None, :-1], U[None], self.h)
        return np.max(np.abs(ver - nom[None]), axis=0)

    # ------------------------------------------------------------ tightening
    def compute(self, X: np.ndarray, U: np.ndarray) -> dict:
        """Tightening for a nominal prediction X (N + 1, 7), U (N, 3) in N: eta (N,), backoff (N + 1,) and diagnostics."""
        t0 = time.perf_counter()
        N, veh = self.N, self.veh
        A, B = self.linearise(X, U)
        K = self.lqr(A, B)
        Acl = A + np.einsum("kij,kjl->kil", B, K)
        W = self.disturbance(X, U)
        b_track = np.zeros(N + 1)
        for arm in (veh.lf, -veh.lr):
            C = np.zeros((N + 1, 7))
            C[:, 1] = 1.0
            C[:, 2] = arm * np.cos(X[:, 2])
            b_track = np.maximum(b_track, reach_bounds(Acl, W, C))
        b_raw = b_track.copy()
        b_track = np.minimum(b_track, self.b_max)
        eta_k = np.full(N, self.eta)
        D = np.zeros(N + 1)
        if not self.track_only:
            Gx, Gu = self._gjac(X[:-1].T, (U / np.array([1.0, FS, FS])).T)
            Gx = np.asarray(Gx).reshape(2, N, 7).transpose(1, 0, 2)          # (N, 2, 7)
            Gu = np.asarray(Gu).reshape(2, N, 3).transpose(1, 0, 2)          # (N, 2, 3)
            for ax in range(2):
                C = np.zeros((N + 1, 7))
                C[:N] = Gx[:, ax] + np.einsum("kj,kjl->kl", Gu[:, ax], K)
                D = np.maximum(D, reach_bounds(Acl, W, C))
            eta_k = np.sqrt(np.maximum(self.eta ** 2 - D[:N], self.eta_min ** 2))
        capped_track = b_raw[1:] > self.b_max
        capped_eta = (self.eta ** 2 - D[:N]) < self.eta_min ** 2 if not self.track_only else np.zeros(N, bool)
        if self.null:                                     # review c1: the same path without tightening
            eta_k, b_track = np.full(N, self.eta), np.zeros(N + 1)
        return {"eta": eta_k, "backoff": b_track, "b_raw": b_raw, "D": D, "W": W, "K": K,
                "capped_track_share": float(np.mean(capped_track)), "capped_eta_share": float(np.mean(capped_eta[1:])),
                "time": time.perf_counter() - t0}


class TubeMPCController(MPCController):
    """``MPCController`` whose every solve uses the tube's tightening, computed along the previous solution shifted by
    one stage (the first solve: the NMPC's constant-input initial guess) - as ``CautiousMPCController`` (L6)."""

    def __init__(self, mpc: FrenetMPC, tube: Tube, sim_dt: float, tc_eta: float | None = 0.99, keep: int = 10):
        super().__init__(mpc, sim_dt, tc_eta)
        self.tube, self.keep = tube, int(keep)
        self.tube_log: list[dict] = []

    def _nominal(self, xf: np.ndarray):
        mpc = self.mpc
        if mpc.z is None:
            z = mpc.initial_guess(xf, self.u)
            return z[mpc.Xi], z[mpc.Ui] * np.array([1.0, FS, FS])
        X, U = mpc.z[mpc.Xi], mpc.z[mpc.Ui] * np.array([1.0, FS, FS])
        X = np.vstack([xf[None], X[2:], X[-1:]])            # shifted; stage 0 the measured state
        U = np.vstack([U[1:], U[-1:]])
        return X, U

    def __call__(self, t: float, x: np.ndarray):
        xf = self.frenet(x)
        if t + 1e-9 >= self.t_next:
            Xn, Un = self._nominal(xf)
            tb = self.tube.compute(Xn, Un)
            u, info = self.mpc.solve(xf, self.u, eta=tb["eta"], backoff=tb["backoff"])
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            Xp, _ = self.mpc.prediction()
            self.tube_log.append({"t": t, "tube_time": tb["time"], "b": tb["backoff"][: self.keep + 1].copy(),
                                  "b_N": float(tb["backoff"][-1]), "b_all": tb["backoff"].copy(), "eta": tb["eta"].copy(),
                                  "capped_track": tb["capped_track_share"], "capped_eta": tb["capped_eta_share"],
                                  "X_pred": Xp[: self.keep + 1].copy()})
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = self.mpc.veh
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        return drate, fd, fb
