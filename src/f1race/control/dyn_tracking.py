"""Feed-forward + feedback trajectory tracking for the dynamic model.

Steering: planned steering angle plus a look-ahead feedback on the lateral
error of the CG projected ahead (in the spirit of Kapania & Gerdes, "Design of
a feedback-feedforward steering controller for accurate path tracking and
stability at the limits of handling", 2015):

    delta_cmd = delta_ff(s) - k_p (e + x_la sin(dpsi)) - k_r (r - r_ff(s))

plus an ESC-like counter-steer k_esc (|a_r| - esc_frac a_peak) when the rear slip
angle approaches the tyre peak, and capped so that the front slip angle stays
within ``slip_frac`` of the tyre peak.

Longitudinal: planned force (acceleration part scaled by v / v_ref) plus speed
feedback, routed to the driven rear axle (positive) or to the brakes (negative),
then limited per axle to what its friction ellipse leaves at the current lateral
force (traction control / ABS, factor ``eta_exec``).
"""
from __future__ import annotations

import numpy as np

from ..dynamics import DynVehicle, tyre_forces
from ..path import Path


class DynTracker:
    def __init__(self, path: Path, ff: dict, vehicle: DynVehicle, dt: float,
                 k_p: float = 0.05, x_la: float = 12.0, k_r: float = 0.0, kv: float = 6.0,
                 eta_exec: float = 0.99, slip_frac: float = 1.0, k_esc: float = 0.0,
                 esc_frac: float = 0.85):
        """``ff``: arrays on the path points: vx, delta, r, f (net longitudinal force)."""
        self.path, self.veh, self.dt = path, vehicle, dt
        self.ff = {k: np.asarray(v) for k, v in ff.items()}
        self.k_p, self.x_la, self.k_r, self.kv = k_p, x_la, k_r, kv
        self.eta_exec = eta_exec
        self.slip_frac = slip_frac
        self.k_esc, self.esc_frac = k_esc, esc_frac
        self.idx = None
        self.log: list[tuple] = []

    def _project(self, px, py):
        p = self.path
        N = len(p.s)
        if self.idx is None:
            j = int(np.argmin((p.x - px) ** 2 + (p.y - py) ** 2))
        else:
            w = (self.idx + np.arange(-10, 60)) % N
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
        e = -(px - qx) * np.sin(th) + (py - qy) * np.cos(th)
        ff = {k: v[a] + lam * (v[b] - v[a]) for k, v in self.ff.items()}
        return (p.s[a] + lam * p.ds[a]) % p.length, e, th, ff

    def __call__(self, t, x):
        veh, dt = self.veh, self.dt
        px, py, psi, vx, vy, r, d = x
        # look half a step ahead (the command acts over the next step)
        c, s = np.cos(psi), np.sin(psi)
        pxp = px + 0.5 * dt * (vx * c - vy * s)
        pyp = py + 0.5 * dt * (vx * s + vy * c)
        sp, e, th, ff = self._project(pxp, pyp)
        # course angle (heading + side-slip) against the path tangent: the CG path is
        # tangent to the velocity, not to the body axis
        course = psi + 0.5 * dt * r + np.arctan2(vy, max(vx, 1.0))
        dpsi = np.angle(np.exp(1j * (course - th)))
        d_cmd = ff["delta"] - self.k_p * (e + self.x_la * np.sin(dpsi)) - self.k_r * (r - ff["r"])
        # rear-slip stabiliser (ESC-like): counter-steer in proportion to how far the rear
        # slip angle exceeds esc_frac of the tyre peak; the drive force is cut below
        a_r = -np.arctan((vy - veh.lr * r) / max(vx, 1.0))
        a_on = self.esc_frac * veh.alpha_peak
        esc = np.sign(a_r) * max(abs(a_r) - a_on, 0.0)
        d_cmd = d_cmd + self.k_esc * esc
        # front-slip governor: keep the front slip angle on the stable side of the tyre peak
        th_f = np.arctan((vy + veh.lf * r) / max(vx, 1.0))
        lim = self.slip_frac * veh.alpha_peak
        d_cmd = float(np.clip(d_cmd, th_f - lim, th_f + lim))
        d_cmd = float(np.clip(d_cmd, -veh.delta_max, veh.delta_max))
        drate = (d_cmd - d) / dt
        # the reference is a function of arclength: scale the planned acceleration by v / v_ref
        a_ff = (ff["f"] - veh.resist(ff["vx"] ** 2)) / veh.m
        f = ff["f"] + veh.m * a_ff * (vx / max(ff["vx"], 1.0) - 1.0) + veh.m * self.kv * (ff["vx"] - vx)
        d_next = d + float(np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max)) * dt
        fd, fb = traction_limit(veh, x, d_next, max(f, 0.0), min(f, 0.0), self.eta_exec)
        if esc != 0.0:
            fd = 0.0
        self.log.append((t, sp, e, dpsi, ff["vx"]))
        return drate, fd, fb


def traction_limit(veh: DynVehicle, x: np.ndarray, delta: float, fd: float, fb: float,
                   eta: float) -> tuple[float, float]:
    """Traction control / ABS: limit the drive (rear) and brake (balance-split) forces so
    that each axle stays inside ``eta`` of its friction ellipse, given the lateral force
    it carries at steering angle ``delta`` (nominal model)."""
    _, _, _, vx, vy, r, _ = x
    F = tyre_forces(veh, vx, vy, r, delta, fd, fb)

    def avail(fy, fz):
        lat = abs(fy) / (eta * veh.mu_y * fz)
        return eta * veh.mu_x * fz * np.sqrt(max(1.0 - lat * lat, 0.0))

    a_r, a_f = avail(F["fyr"], F["fzr"]), avail(F["fyf"], F["fzf"])
    kb = veh.brake_front
    if fd > 0.0:
        fd = min(fd, a_r)
    if fb < 0.0:
        lim = min(a_f / kb, a_r / (1.0 - kb)) if kb < 1.0 else a_f / kb
        fb = max(fb, -lim)
    return fd, fb
