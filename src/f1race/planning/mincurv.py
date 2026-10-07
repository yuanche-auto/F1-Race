"""Minimum-curvature racing line as a bounded linear least-squares problem.

Points of the line are p_i = c_i + alpha_i n_i (c: centreline sample, n: left
normal). The objective is the sum of squared periodic second differences,
weighted by 1/ds_i^4 so that it approximates sum(kappa_i^2) for the current
point spacing; the weights are updated over a few iterations. Each subproblem

    min || W (D2 (c + N alpha)) ||^2   s.t.  lb <= alpha <= ub

is solved as a box-constrained QP with OSQP (through CasADi).
"""
from __future__ import annotations

import casadi as ca
import numpy as np
import scipy.sparse as sp


def _to_casadi(M: sp.spmatrix) -> ca.DM:
    M = sp.csc_matrix(M)
    M.sort_indices()
    spars = ca.Sparsity(M.shape[0], M.shape[1], M.indptr.tolist(), M.indices.tolist())
    return ca.DM(spars, M.data)


def solve_box_qp(H: sp.spmatrix, f: np.ndarray, lb: np.ndarray, ub: np.ndarray,
                 x0: np.ndarray | None = None) -> np.ndarray:
    """min 1/2 x'Hx + f'x  s.t. lb <= x <= ub  (OSQP with solution polishing)."""
    n = len(f)
    Hc = _to_casadi(0.5 * (H + H.T))
    I = _to_casadi(sp.eye(n))
    solver = ca.conic("qp", "osqp", {"h": Hc.sparsity(), "a": I.sparsity()},
                      {"osqp": {"eps_abs": 1e-8, "eps_rel": 1e-8, "max_iter": 200000,
                                "polish": True, "verbose": False}, "error_on_fail": False})
    args = dict(h=Hc, g=f, a=I, lba=lb, uba=ub)
    if x0 is not None:
        args["x0"] = x0
    return np.asarray(solver(**args)["x"]).ravel()

from ..path import Path
from ..track import Track
from ..vehicle import Vehicle


def lateral_bounds(track: Track, vehicle: Vehicle, margin: float,
                   kappa_path: np.ndarray | None = None, idx=slice(None)):
    """Admissible offsets of the rear axle so that the car body (half width) and the
    front axle (overhang ~ L^2 |kappa| / 2 on curves) stay inside the edges."""
    extra = vehicle.half_width + margin
    if kappa_path is not None:
        extra = extra + 0.5 * vehicle.L**2 * np.abs(kappa_path)
    lb = -(track.w_right[idx] - extra)
    ub = track.w_left[idx] - extra
    return lb, ub


def min_curvature_line(track: Track, vehicle: Vehicle, margin: float = 0.3,
                       step: int = 3, iters: int = 4, smooth: float = 0.02) -> tuple[np.ndarray, Path]:
    """Return (alpha on every ``step``-th centreline sample, smoothed path through them).

    ``smooth`` is the RMS deviation [m] allowed when fitting the final quintic spline;
    it removes the curvature kinks the QP produces where bounds become active.
    """
    idx = np.arange(0, len(track.s), step)
    c = np.column_stack([track.x[idx], track.y[idx]])
    nrm = track.normal[idx]
    N = len(idx)
    e = np.ones(N)
    D2 = sp.diags([e, -2 * e, e], [-1, 0, 1], shape=(N, N), format="lil")
    D2[0, N - 1] = 1.0
    D2[N - 1, 0] = 1.0
    D2 = D2.tocsr()
    Ax = D2 @ sp.diags(nrm[:, 0])
    Ay = D2 @ sp.diags(nrm[:, 1])
    bx, by = D2 @ c[:, 0], D2 @ c[:, 1]

    alpha = np.zeros(N)
    kappa_path = None
    for _ in range(iters):
        p = c + nrm * alpha[:, None]
        dsi = np.hypot(*(np.roll(p, -1, axis=0) - p).T)
        dsm = 0.5 * (dsi + np.roll(dsi, 1))
        w = sp.diags(1.0 / dsm**2)
        A = sp.vstack([w @ Ax, w @ Ay]).tocsr()
        b = -np.concatenate([w @ bx, w @ by])
        lb, ub = lateral_bounds(track, vehicle, margin, kappa_path, idx)
        lb, ub = np.minimum(lb, ub - 1e-6), np.maximum(ub, lb + 1e-6)
        H = 2.0 * (A.T @ A) + 1e-9 * sp.eye(N)
        f = -2.0 * (A.T @ b)
        alpha = solve_box_qp(H, f, lb, ub, x0=np.clip(alpha, lb, ub))
        path = Path.from_points(c + nrm * alpha[:, None], smooth=smooth)
        kappa_path = np.abs(path.kappa)
        # local max over ~10 m for the front-axle term
        win = max(1, int(round(10.0 / (step * track.ds))))
        kappa_path = np.max([np.roll(kappa_path, k) for k in range(-win, win + 1)], axis=0)
    return alpha, Path.from_points(c + nrm * alpha[:, None], smooth=smooth)
