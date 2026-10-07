"""Path and speed tracking for the kinematic model.

Steering: rear-axle path-following law (Lyapunov based, see e.g. Paden et al.,
"A survey of motion planning and control techniques for self-driving urban
vehicles", 2016)

    kappa_cmd = kappa cos(dpsi) / (1 - kappa e) - k_psi dpsi - k_e e sin(dpsi)/dpsi

which gives e'' + k_psi e' + k_e e = 0 in arclength for small errors; the gains
are scheduled with speed (bounded temporal bandwidth). The law is evaluated
half a step ahead and the resulting steering angle is commanded through the
rate-limited actuator, capped by a friction governor: the commanded angle never
asks for more than ``eta_exec`` of the lateral grip at the current speed.

Speed: feed-forward of the planned acceleration (scaled by v / v_ref because the
reference is a function of arclength) plus proportional feedback;
the longitudinal force is then clipped to what the friction ellipse leaves
at the current lateral demand (times ``eta_exec``) and to the power limit.
"""
from __future__ import annotations

import numpy as np

from ..path import Path
from ..vehicle import Vehicle


class PathTracker:
    def __init__(self, path: Path, v_ref: np.ndarray, a_ref: np.ndarray, vehicle: Vehicle,
                 dt: float, eta_exec: float = 0.995, wn_max: float = 0.1, wt_max: float = 3.0,
                 zeta: float = 1.0, kv: float = 8.0, kappa_ref: np.ndarray | None = None):
        self.path, self.veh, self.dt = path, vehicle, dt
        # curvature feed-forward: the plan's own steering (tan(delta)/L) if given,
        # otherwise the geometric curvature of the path spline
        self.kappa_ref = np.asarray(path.kappa if kappa_ref is None else kappa_ref)
        self.v_ref, self.a_ref = np.asarray(v_ref), np.asarray(a_ref)
        self.eta_exec, self.wn_max, self.wt_max, self.zeta, self.kv = eta_exec, wn_max, wt_max, zeta, kv
        self.idx = None
        self.log: list[tuple] = []

    def _project(self, px: float, py: float):
        p = self.path
        N = len(p.s)
        if self.idx is None:
            j = int(np.argmin((p.x - px) ** 2 + (p.y - py) ** 2))
        else:
            w = (self.idx + np.arange(-10, 40)) % N
            j = int(w[np.argmin((p.x[w] - px) ** 2 + (p.y[w] - py) ** 2)])
        best = None
        for a in ((j - 1) % N, j):
            b = (a + 1) % N
            vx, vy = p.x[b] - p.x[a], p.y[b] - p.y[a]
            L2 = vx * vx + vy * vy
            lam = min(max(((px - p.x[a]) * vx + (py - p.y[a]) * vy) / L2, 0.0), 1.0)
            qx, qy = p.x[a] + lam * vx, p.y[a] + lam * vy
            d2 = (px - qx) ** 2 + (py - qy) ** 2
            if best is None or d2 < best[0]:
                best = (d2, a, b, lam, qx, qy)
        _, a, b, lam, qx, qy = best
        self.idx = a
        th = p.psi[a] + lam * np.angle(np.exp(1j * (p.psi[b] - p.psi[a])))
        lerp = lambda f: f[a] + lam * (f[b] - f[a])
        e = -(px - qx) * np.sin(th) + (py - qy) * np.cos(th)
        sp = (p.s[a] + lam * p.ds[a]) % p.length
        return sp, e, th, lerp(self.kappa_ref), lerp(p.dkappa), lerp(self.v_ref), lerp(self.a_ref)

    def __call__(self, t: float, x: np.ndarray) -> tuple[float, float]:
        veh, dt = self.veh, self.dt
        px, py, psi, v, delta = x
        # evaluate the reference half a step ahead (inputs act over the next step)
        px_p = px + 0.5 * dt * v * np.cos(psi)
        py_p = py + 0.5 * dt * v * np.sin(psi)
        psi_p = psi + 0.5 * dt * v * np.tan(delta) / veh.L
        sp, e, th, kap, dkap, vref, aref = self._project(px_p, py_p)
        dpsi = np.angle(np.exp(1j * (psi_p - th)))
        vv = max(v, 1.0)
        wn = min(self.wn_max, self.wt_max / vv)
        ke, kpsi = wn * wn, 2.0 * self.zeta * wn
        sc = np.sin(dpsi) / dpsi if abs(dpsi) > 1e-6 else 1.0
        kc = kap * np.cos(dpsi) / (1.0 - kap * e) - kpsi * dpsi - ke * e * sc
        # friction governor: never command more steering than the lateral grip allows at
        # the current speed (pure-lateral usage <= eta_exec); path accuracy is traded for grip
        v2 = v * v
        d_grip = np.arctan(veh.L * self.eta_exec * veh.mu_y * veh.fz(v2) / (veh.m * max(v2, 1e-6)))
        d_lim = min(veh.delta_max, d_grip)
        delta_des = float(np.clip(np.arctan(veh.L * kc), -d_lim, d_lim))
        drate = float(np.clip((delta_des - delta) / dt, -veh.delta_rate_max, veh.delta_rate_max))

        # v_ref is a function of arclength: d v_ref/dt = a_ref * v / v_ref
        fx = veh.m * (aref * v / max(vref, 1.0) + self.kv * (vref - v)) + veh.resist(v * v)
        delta_n = delta + drate * dt
        v2 = v * v
        fz = veh.fz(v2)
        fy = veh.m * v2 * np.tan(max(abs(delta), abs(delta_n))) / veh.L
        lat = fy / (veh.mu_y * fz * self.eta_exec)
        avail = self.eta_exec * veh.mu_x * fz * np.sqrt(max(1.0 - lat * lat, 0.0))
        fx = float(np.clip(fx, -avail, min(avail, veh.P_max / vv)))
        self.log.append((t, sp, e, dpsi, vref))
        return drate, fx
