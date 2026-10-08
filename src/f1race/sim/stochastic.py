"""Stochastic disturbances for L3 (all seeded, so that every controller meets the same
realisations).

* ``FrictionField``: friction factor along the track, f(s) = 1 + sigma g(s) with g a
  zero-mean, unit-variance, periodic Gaussian process with squared-exponential
  correlation (length ``corr_len``), sampled by a random Fourier series on the loop.
* ``Gusts``: lateral force at the CG (and the yaw moment of a force acting ``arm``
  metres ahead of it), an Ornstein-Uhlenbeck process in time.
* ``NoisyController``: the controller sees the state plus white Gaussian noise.

``StochasticSim`` is the L1/L2 simulator with the friction coefficients of both
axles scaled by f(s) at the car's position and the gust added to the body
accelerations. The controllers keep using the nominal model.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from ..dynamics import DynVehicle, body_accel, tyre_forces
from ..track import Track
from .dynamic import DynamicSim


class FrictionField:
    def __init__(self, length: float, sigma: float, corr_len: float, seed: int, n_modes: int = 400,
                 clip: float = 3.0):
        self.length, self.sigma = length, sigma
        rng = np.random.default_rng(seed)
        k = np.arange(1, n_modes + 1)
        w = np.exp(-0.5 * (2.0 * np.pi * k * corr_len / length) ** 2)
        w /= w.sum()                                     # unit variance of g
        self.k, self.amp = k, np.sqrt(w)
        self.a, self.b = rng.standard_normal(n_modes), rng.standard_normal(n_modes)
        self.clip = clip
        grid = np.linspace(0.0, length, int(length) + 1)
        self._grid, self._f = grid, self._eval(grid)

    def _eval(self, s: np.ndarray) -> np.ndarray:
        ph = 2.0 * np.pi * np.outer(np.asarray(s, float), self.k) / self.length
        g = (np.cos(ph) * self.a + np.sin(ph) * self.b) @ self.amp
        return 1.0 + self.sigma * np.clip(g, -self.clip, self.clip)

    def __call__(self, s):
        return np.interp(np.mod(s, self.length), self._grid, self._f)


class Gusts:
    """Lateral force F (N) on a sampled time grid; F(t) is held over each step."""

    def __init__(self, sigma: float, tau: float, dt: float, t_max: float, seed: int, arm: float = 0.3):
        rng = np.random.default_rng(seed)
        n = int(np.ceil(t_max / dt)) + 2
        a = np.exp(-dt / tau)
        F = np.empty(n)
        F[0] = sigma * rng.standard_normal()
        xi = rng.standard_normal(n)
        for i in range(1, n):
            F[i] = a * F[i - 1] + sigma * np.sqrt(1.0 - a * a) * xi[i]
        self.F, self.dt, self.arm = F, dt, arm

    def at(self, t: float) -> float:
        return float(self.F[min(int(t / self.dt + 1e-9), len(self.F) - 1)])


@dataclass
class Disturbances:
    """One realisation. ``sigma_mu = 0`` / ``gust_sigma = 0`` / ``noise = None`` switch a
    component off."""
    friction: FrictionField | None
    gusts: Gusts | None
    noise_sigma: np.ndarray | None
    noise_seed: int


NOISE_SIGMA = np.array([0.05, 0.05, 0.005, 0.1, 0.05, 0.01, 0.001])   # x, y, psi, vx, vy, r, delta


def sample_disturbances(track: Track, seed: int, sigma_mu: float = 0.03, corr_len: float = 100.0,
                        gust_sigma: float = 300.0, gust_tau: float = 1.0, noise: bool = True,
                        dt: float = 0.01, t_max: float = 120.0) -> Disturbances:
    ss = np.random.SeedSequence(seed).spawn(3)
    seeds = [int(s.generate_state(1)[0]) for s in ss]
    fr = FrictionField(track.length, sigma_mu, corr_len, seeds[0]) if sigma_mu > 0 else None
    gu = Gusts(gust_sigma, gust_tau, dt, t_max, seeds[1]) if gust_sigma > 0 else None
    return Disturbances(fr, gu, NOISE_SIGMA.copy() if noise else None, seeds[2])


class NoisyController:
    def __init__(self, controller, sigma: np.ndarray | None, seed: int):
        self.ctl, self.sigma = controller, sigma
        self.rng = np.random.default_rng(seed)

    def __call__(self, t, x):
        if self.sigma is None:
            return self.ctl(t, x)
        return self.ctl(t, x + self.sigma * self.rng.standard_normal(len(x)))

    def __getattr__(self, name):                         # log, mpc, ... of the wrapped controller
        return getattr(self.ctl, name)


class StochasticSim(DynamicSim):
    def __init__(self, track: Track, vehicle: DynVehicle, dist: Disturbances, dt: float = 0.01):
        super().__init__(track, vehicle, dt)
        self.nominal, self.dist = vehicle, dist
        self.t = 0.0
        self.f_now = 1.0
        self.gust_now = 0.0

    def _update(self, x):
        if self.dist.friction is not None:
            s, _ = self.track.project(np.array([x[0]]), np.array([x[1]]))
            f = float(self.dist.friction(s[0]))
            if f != self.f_now:
                self.veh = replace(self.nominal, mu_x=self.nominal.mu_x * f, mu_y=self.nominal.mu_y * f)
                self.f_now = f
        if self.dist.gusts is not None:
            self.gust_now = self.dist.gusts.at(self.t)

    def _f(self, x, u):
        _, _, psi, vx, vy, r, d = x
        F, _, _ = self._forces(x, u)
        dvx, dvy, dr = body_accel(self.veh, vx, vy, r, d, F)
        if self.gust_now:
            dvy += self.gust_now / self.veh.m
            dr += self.gust_now * self.dist.gusts.arm / self.veh.Iz
        c, s = np.cos(psi), np.sin(psi)
        return np.array([vx * c - vy * s, vx * s + vy * c, r, dvx, dvy, dr, u[0]])

    def step(self, x, u):
        self._update(x)
        xn = super().step(x, u)
        self.t += self.dt
        return xn

    def run(self, controller, x0, t_max: float = 300.0, laps: float = 1.0):
        self.t = 0.0
        self._update(np.asarray(x0, float))
        ctl = NoisyController(controller, self.dist.noise_sigma, self.dist.noise_seed)
        return super().run(ctl, x0, t_max, laps)


def nominal_usage(veh: DynVehicle, state: np.ndarray, inputs: np.ndarray) -> np.ndarray:
    """Front/rear ellipse usage of recorded states and inputs evaluated with the nominal
    friction (the simulator records the usage of the plant, i.e. with f(s))."""
    _, _, _, vx, vy, r, d = state.T
    F = tyre_forces(veh, vx, vy, r, d, inputs[:, 1], inputs[:, 2])
    uf = np.hypot(F["fxf"] / (veh.mu_x * F["fzf"]), F["fyf"] / (veh.mu_y * F["fzf"]))
    ur = np.hypot(F["fxr"] / (veh.mu_x * F["fzr"]), F["fyr"] / (veh.mu_y * F["fzr"]))
    return np.column_stack([uf, ur])


__all__ = ["FrictionField", "Gusts", "Disturbances", "sample_disturbances", "NoisyController",
           "StochasticSim", "nominal_usage", "NOISE_SIGMA"]
