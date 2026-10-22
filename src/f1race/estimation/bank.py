"""Model bank with look-back selection (control expansion phase D, docs/CX_D_protocol.md section 3.5; after
LLA-MPC, literature D.9).

N_b parameter vectors drawn uniformly in [LO, HI] (``identify``'s bounds) plus the nominal car (index 0). At
every simulator step every model predicts y_{k+1} = (v_x, v_y, r) from (x_k, u_k) with the simulator's step
(``rls.step_np``, batched); a model's cost is the sum of |Lambda^-1/2 (y - y_hat)|^2 over the last W seconds;
the model with the smallest cost is selected (hard switching, as D.9; the nominal car until the window is
full). The friction used for the plan is smoothed, mu_s <- gamma mu_sel + (1 - gamma) mu_s per step (D.9
smooths its friction estimate without reporting gamma; gamma = 0.05 here). D.9 scores the plain Euclidean
state error; here the error is Lambda-weighted as for the RLS.
"""
from __future__ import annotations

import time

import numpy as np

from ..dynamics import DynVehicle
from .identify import HI, LO, PARAMS, X0
from .rls import SIGMA, V_MIN, _vehicle, step_np

BANK_SIZE, BANK_SEED, GAMMA = 20_000, 336, 0.05


class ModelBank:
    def __init__(self, nominal: DynVehicle | None = None, h: float = 0.01, window: float = 0.2,
                 n: int = BANK_SIZE, seed: int = BANK_SEED, gamma: float = GAMMA, sigma=SIGMA, lo=LO, hi=HI,
                 v_min: float = V_MIN):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        rng = np.random.default_rng(seed)
        self.thetas = np.vstack([X0, lo + (hi - lo) * rng.random((n, len(PARAMS)))])
        self.veh = _vehicle(nominal or DynVehicle(), self.thetas.T)
        self.h, self.W = h, max(int(round(window / h)), 1)
        self.buf = np.zeros((self.W, len(self.thetas)))
        self.k = 0
        self.sel = 0
        self.gamma, self.sigma, self.v_min = gamma, np.asarray(sigma, float), v_min
        self.mu_s = float(self.thetas[0, 0])
        self.switches = 0
        self.n_skipped = 0
        self.time = 0.0

    @property
    def theta(self) -> np.ndarray:
        return self.thetas[self.sel].copy()

    @property
    def mu(self) -> float:
        """The smoothed friction factor (for the plan)."""
        return self.mu_s

    def update(self, z, u, y) -> None:
        t0 = time.perf_counter()
        if z[0] <= self.v_min:
            self.n_skipped += 1
            return
        yh = step_np(self.veh, z, u, self.h)
        self.buf[self.k % self.W] = (((np.asarray(y, float) - yh) / self.sigma) ** 2).sum(axis=1)
        self.k += 1
        if self.k >= self.W:
            sel = int(np.argmin(self.buf.sum(axis=0)))
            self.switches += int(sel != self.sel)
            self.sel = sel
        self.mu_s = self.gamma * float(self.thetas[self.sel, 0]) + (1.0 - self.gamma) * self.mu_s
        self.time += time.perf_counter() - t0

    def diagnostics(self) -> dict:
        return {"updates": self.k, "skipped": self.n_skipped, "switches": self.switches, "bank_size": len(self.thetas),
                "update_us_mean": 1e6 * self.time / max(self.k + self.n_skipped, 1)}


__all__ = ["ModelBank", "BANK_SIZE", "BANK_SEED", "GAMMA"]
