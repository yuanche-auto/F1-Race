"""Learned residual dynamics with predictive uncertainty (L6, round 2).

What the (identified) model gets wrong about the body accelerations (v_x', v_y',
r') is learned from logged driving data as a function of

    z = (v_x, v_y, r, delta, delta_rate_cmd, F_drive, F_brake)

by Bayesian linear regression on a constant, the 7 standardised inputs and
``n_rff`` random Fourier features of a squared-exponential kernel (an
approximation of a Gaussian process; Rahimi & Recht 2007). Per output, the prior
precision and the noise precision are found by evidence maximisation (MacKay's
fixed-point updates); the common length scale (on standardised inputs) is the
candidate with the largest total evidence. Nothing is tuned on closed-loop laps.

The predictive standard deviation includes the noise term: it is the spread of a
new observation around the mean, which is what a constraint tightening and the
coverage check need.
"""
from __future__ import annotations

import casadi as ca
import numpy as np

from ..dynamics import DynVehicle
from .identify import Data, model_accel

LENGTH_SCALES = (0.5, 1.0, 2.0, 4.0, 8.0, 16.0)


def inputs_of(d: Data) -> np.ndarray:
    """(K, 7) regression inputs from identification data (mid-step state, held input)."""
    return np.column_stack([d.state[:, 3:7], d.inputs])


def targets_of(d: Data, veh: DynVehicle) -> np.ndarray:
    """(K, 3) residual accelerations: measured minus the model's."""
    return d.accel - model_accel(veh, d)


def _evidence_fit(Phi: np.ndarray, y: np.ndarray, iters: int = 200):
    """MacKay's evidence maximisation for y = Phi w + e, w ~ N(0, 1/alpha), e ~ N(0, 1/beta)."""
    N, D = Phi.shape
    PtP, Pty = Phi.T @ Phi, Phi.T @ y
    eig = np.linalg.eigvalsh(PtP)
    alpha, beta = 1.0, 1.0 / max(np.var(y), 1e-12)
    for _ in range(iters):
        A = alpha * np.eye(D) + beta * PtP
        m = beta * np.linalg.solve(A, Pty)
        lam = beta * eig
        gamma = float(np.sum(lam / (alpha + lam)))
        err = float(np.sum((y - Phi @ m) ** 2))
        a_new = gamma / max(float(m @ m), 1e-12)
        b_new = (N - gamma) / max(err, 1e-12)
        done = abs(np.log(a_new / alpha)) < 1e-6 and abs(np.log(b_new / beta)) < 1e-6
        alpha, beta = a_new, b_new
        if done:
            break
    A = alpha * np.eye(D) + beta * PtP
    m = beta * np.linalg.solve(A, Pty)
    err = float(np.sum((y - Phi @ m) ** 2))
    _, logdet = np.linalg.slogdet(A)
    ev = (0.5 * D * np.log(alpha) + 0.5 * N * np.log(beta) - 0.5 * beta * err - 0.5 * alpha * float(m @ m)
          - 0.5 * logdet - 0.5 * N * np.log(2 * np.pi))
    return m, np.linalg.inv(A), alpha, beta, float(ev)


class ResidualModel:
    def __init__(self, n_rff: int = 100, seed: int = 0, length_scales=LENGTH_SCALES):
        self.n_rff, self.seed, self.length_scales = n_rff, seed, tuple(length_scales)

    def _features(self, Z: np.ndarray, ell: float) -> np.ndarray:
        Zs = (Z - self.z_mu) / self.z_sd
        rff = np.sqrt(2.0 / self.n_rff) * np.cos(Zs @ self.W / ell + self.b)
        return np.column_stack([np.ones(len(Zs)), Zs, rff])

    def fit(self, Z: np.ndarray, Y: np.ndarray) -> "ResidualModel":
        rng = np.random.default_rng(self.seed)
        self.z_mu, self.z_sd = Z.mean(axis=0), np.maximum(Z.std(axis=0), 1e-6)
        self.y_mu, self.y_sd = Y.mean(axis=0), np.maximum(Y.std(axis=0), 1e-9)
        self.W = rng.standard_normal((Z.shape[1], self.n_rff))
        self.b = rng.uniform(0.0, 2.0 * np.pi, self.n_rff)
        Ys = (Y - self.y_mu) / self.y_sd
        best = None
        for ell in self.length_scales:
            Phi = self._features(Z, ell)
            fits = [_evidence_fit(Phi, Ys[:, j]) for j in range(Y.shape[1])]
            total = sum(f[4] for f in fits)
            if best is None or total > best[0]:
                best = (total, ell, fits)
        self.evidence, self.ell, fits = best
        self.m = np.column_stack([f[0] for f in fits])            # (D, 3)
        self.S = [f[1] for f in fits]                              # posterior covariances
        self.alpha = np.array([f[2] for f in fits])
        self.beta = np.array([f[3] for f in fits])
        self.n_samples = len(Z)
        return self

    def predict(self, Z: np.ndarray):
        """Mean (K, 3) and predictive standard deviation (K, 3) in physical units."""
        Phi = self._features(np.atleast_2d(Z), self.ell)
        mean = Phi @ self.m * self.y_sd + self.y_mu
        var = np.column_stack([np.einsum("ij,jk,ik->i", Phi, S, Phi) + 1.0 / b
                               for S, b in zip(self.S, self.beta)])
        return mean, np.sqrt(var) * self.y_sd

    def casadi_mean(self) -> ca.Function:
        """z (7) -> residual mean (3), for the MPC dynamics."""
        z = ca.SX.sym("z", len(self.z_mu))
        zs = (z - self.z_mu) / self.z_sd
        rff = np.sqrt(2.0 / self.n_rff) * ca.cos(ca.mtimes(self.W.T, zs) / self.ell + self.b)
        phi = ca.vertcat(1.0, zs, rff)
        out = ca.mtimes(self.m.T, phi) * self.y_sd + self.y_mu
        return ca.Function("residual", [z], [out])

    def summary(self) -> dict:
        return {"ell": self.ell, "evidence": self.evidence, "alpha": self.alpha.tolist(),
                "noise_sd": (self.y_sd / np.sqrt(self.beta)).tolist(), "n_samples": self.n_samples,
                "n_rff": self.n_rff}


def coverage(model: ResidualModel, Z: np.ndarray, Y: np.ndarray, z_score: float = 1.96) -> dict:
    """Fraction of the residuals inside the nominal 95 % prediction interval, per output."""
    mean, sd = model.predict(Z)
    inside = np.abs(Y - mean) <= z_score * sd
    return {"coverage": inside.mean(axis=0).tolist(), "n": int(len(Y)),
            "rms_error": np.sqrt(((Y - mean) ** 2).mean(axis=0)).tolist(),
            "mean_sd": sd.mean(axis=0).tolist(), "inside": inside}
