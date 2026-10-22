"""The adaptive NMPC of control expansion phase D (docs/CX_D_protocol.md section 3.3).

``AdaptiveMPCController`` drives a ``FrenetMPC(param_model=True)`` built on a plan library (a list of references
ordered by their friction margin eta_p, ascending): at every simulator step it feeds its estimator the data of
the previous step (the state it saw, the input the plant applied – its own command after its traction control
and after the simulator's input limits, recomputed here – and the body velocities now); at every MPC step it
hands the NLP the estimate theta and the plan coordinate j of eta_p = 0.95 mu (clipped to the library's range,
linear in the eta_p grid). The traction control uses the car of the current estimate.

``source``: ``"estimate"`` (``adaptive``, ``adaptive_bank``), ``"null"`` (``nmpc_param``: the estimator runs, the
nominal car and the 5 % plan are handed to the NLP and the traction control) or ``"oracle"`` (``adaptive_oracle``:
``oracle(s)`` gives the true factors with the friction factor at the car's position; no estimator).
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from ..estimation.identify import vehicle_from
from .dyn_tracking import traction_limit
from .nmpc import THETA_NOMINAL, FrenetMPC, MPCController


def applied_input(veh, x, u) -> np.ndarray:
    """The input the simulator applies to the command u at state x (``DynamicSim.apply_limits``; with the steering
    lag the 7-dim state the controller sees holds the commanded angle, which is the one the stop acts on)."""
    drate = float(np.clip(u[0], -veh.delta_rate_max, veh.delta_rate_max))
    d = x[6]
    if (d >= veh.delta_max and drate > 0) or (d <= -veh.delta_max and drate < 0):
        drate = 0.0
    fd = float(np.clip(u[1], 0.0, veh.P_max / max(x[3], 1.0)))
    fb = float(min(u[2], 0.0))
    return np.array([drate, fd, fb])


class AdaptiveMPCController(MPCController):
    def __init__(self, mpc: FrenetMPC, sim_dt: float, plan_etas, estimator=None, source: str = "estimate",
                 oracle: Callable[[float], np.ndarray] | None = None, margin_eta: float = 0.95,
                 tc_eta: float | None = 0.99):
        if not (mpc.param_model and mpc.library):
            raise ValueError("the MPC needs param_model=True and a plan library")
        if source not in ("estimate", "null", "oracle"):
            raise ValueError(source)
        if (source == "oracle") != (oracle is not None) or (source != "oracle" and estimator is None):
            raise ValueError("source 'oracle' needs oracle(s) and no estimator; the others an estimator")
        super().__init__(mpc, sim_dt, tc_eta)
        self.etas = np.asarray(plan_etas, float)
        if len(self.etas) != mpc.n_plans or np.any(np.diff(self.etas) <= 0):
            raise ValueError("plan_etas: one per plan, ascending")
        self.est, self.source, self.oracle, self.margin_eta = estimator, source, oracle, margin_eta
        self.prev = None                                   # (z, applied input) of the previous step
        self.trace: list[tuple] = []                       # per solve: t, s, j, mu for the plan, theta (7)

    def plan_coordinate(self, mu: float) -> float:
        e = float(np.clip(self.margin_eta * mu, self.etas[0], self.etas[-1]))
        return float(np.interp(e, self.etas, np.arange(len(self.etas), dtype=float)))

    def current(self, s: float) -> tuple[np.ndarray, float]:
        """(theta for the NLP and the traction control, mu for the plan)."""
        if self.source == "oracle":
            th = np.asarray(self.oracle(s), float)
            return th, float(th[0])
        if self.source == "null":
            return THETA_NOMINAL.copy(), 1.0
        return self.est.theta.copy(), float(self.est.mu)

    def __call__(self, t: float, x: np.ndarray):
        x = np.asarray(x, float)
        if self.prev is not None and self.est is not None:
            self.est.update(self.prev[0], self.prev[1], x[3:6])
        xf = self.frenet(x)
        th, mu = self.current(xf[0])
        if t + 1e-9 >= self.t_next:
            j = self.mpc.plan_default if self.source == "null" else self.plan_coordinate(mu)
            u, info = self.mpc.solve(xf, self.u, theta=th, plan=j)
            self.u = u
            self.t_next = t + self.mpc.dt
            self.log.append((t, xf[0], xf[1], info["time"], info["status"], info["slack"]))
            self.trace.append((t, xf[0], j, mu, *th))
        drate, fd, fb = float(self.u[0]), float(self.u[1]), float(self.u[2])
        if self.tc_eta is not None:
            veh = vehicle_from(th, self.mpc.veh)
            d_next = x[6] + np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max) * self.sim_dt
            fd, fb = traction_limit(veh, x, d_next, fd, fb, self.tc_eta)
        self.prev = (x[3:7].copy(), applied_input(self.mpc.veh, x, (drate, fd, fb)))
        return drate, fd, fb


__all__ = ["AdaptiveMPCController", "applied_input"]
