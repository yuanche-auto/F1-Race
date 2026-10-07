"""Closed race track: smooth centreline, widths and Frenet projection.

Input format (TUM FTM racetrack-database): CSV with columns
``x_m, y_m, w_tr_right_m, w_tr_left_m`` describing a closed centreline
with the distances to the right/left track edge.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree

DATA_DIR = Path(__file__).resolve().parents[2] / "data" / "tracks"


def _wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Track:
    """Uniformly sampled closed centreline (the sample at s = length is omitted)."""

    name: str
    s: np.ndarray        # arclength of the samples [m], s[0] = 0
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray      # heading (unwrapped) [rad]
    kappa: np.ndarray    # curvature [1/m], > 0 for left turns
    w_left: np.ndarray   # distance centreline -> left edge [m]
    w_right: np.ndarray  # distance centreline -> right edge [m]
    length: float

    # ------------------------------------------------------------------ build
    @classmethod
    def from_csv(cls, path: str | Path, ds: float = 1.0, smoothing: float = 0.05,
                 name: str | None = None) -> "Track":
        """Fit a periodic smoothing spline (RMS deviation ~``smoothing`` m) and resample."""
        path = Path(path)
        raw = np.loadtxt(path, delimiter=",", comments="#")
        pts, wr, wl = raw[:, :2], raw[:, 2], raw[:, 3]
        if np.hypot(*(pts[0] - pts[-1])) < 1e-6:          # drop duplicated closing point
            pts, wr, wl = pts[:-1], wr[:-1], wl[:-1]
        n = len(pts)
        closed = np.vstack([pts, pts[:1]])
        tck, u = splprep([closed[:, 0], closed[:, 1]], s=n * smoothing**2, per=1)

        # exact arclength of the fitted curve via dense evaluation
        uu = np.linspace(0.0, 1.0, 20 * n + 1)
        xx, yy = splev(uu, tck)
        seg = np.hypot(np.diff(xx), np.diff(yy))
        sarc = np.concatenate([[0.0], np.cumsum(seg)])
        length = float(sarc[-1])
        m = int(round(length / ds))
        s = np.arange(m) * (length / m)
        us = np.interp(s, sarc, uu)
        x, y = splev(us, tck)
        dx, dy = splev(us, tck, der=1)
        ddx, ddy = splev(us, tck, der=2)
        psi = np.unwrap(np.arctan2(dy, dx))
        kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5

        # widths: interpolate the raw values along the parameter of the raw points
        u_raw = np.asarray(u)[:-1]
        s_raw = np.interp(u_raw, uu, sarc)
        s_ext = np.concatenate([s_raw - length, s_raw, s_raw + length])
        w_left = np.interp(s, s_ext, np.tile(wl, 3))
        w_right = np.interp(s, s_ext, np.tile(wr, 3))
        return cls(name or path.stem, s, np.asarray(x), np.asarray(y), psi, kappa,
                   w_left, w_right, length)

    @classmethod
    def load(cls, name: str, **kw) -> "Track":
        path = DATA_DIR / f"{name}.csv"
        if not path.exists():
            raise FileNotFoundError(f"{path} not found - run: python scripts/fetch_tracks.py {name}")
        return cls.from_csv(path, name=name, **kw)

    # ------------------------------------------------------------ evaluation
    @property
    def ds(self) -> float:
        return self.length / len(self.s)

    @property
    def normal(self) -> np.ndarray:
        """Unit left normal at the samples, shape (N, 2)."""
        return np.column_stack([-np.sin(self.psi), np.cos(self.psi)])

    def interp(self, field: np.ndarray, s) -> np.ndarray:
        """Periodic linear interpolation of a sampled field at arclength(s) s."""
        s = np.mod(np.asarray(s, dtype=float), self.length)
        sp = np.concatenate([self.s, [self.length]])
        fp = np.concatenate([field, field[:1]])
        return np.interp(s, sp, fp)

    def kappa_at(self, s):
        return self.interp(self.kappa, s)

    def heading_at(self, s):
        # heading is unwrapped: interpolate sin/cos to stay continuous across s = 0
        return np.arctan2(self.interp(np.sin(self.psi), s), self.interp(np.cos(self.psi), s))

    def point_at(self, s, n=0.0) -> np.ndarray:
        """Cartesian point(s) at arclength s and lateral offset n (left positive)."""
        th = self.heading_at(s)
        return np.column_stack([self.interp(self.x, s) - np.sin(th) * n,
                                self.interp(self.y, s) + np.cos(th) * n])

    def edges(self) -> tuple[np.ndarray, np.ndarray]:
        nrm = self.normal
        c = np.column_stack([self.x, self.y])
        return c + nrm * self.w_left[:, None], c - nrm * self.w_right[:, None]

    # ------------------------------------------------------------ projection
    def _tree(self) -> cKDTree:
        if not hasattr(self, "_kdt"):
            self._kdt = cKDTree(np.column_stack([self.x, self.y]))
        return self._kdt

    def project(self, px, py) -> tuple[np.ndarray, np.ndarray]:
        """Frenet coordinates (s, n) of points.

        On each segment the tangent is interpolated linearly between the two end
        samples and the foot point solves (p - c(lam)) . t(lam) = 0 (a quadratic in
        lam). Unlike a plain polyline projection this is continuous across the
        samples: on the inside of tight corners a polyline foot point jumps by about
        |n| kappa ds at every vertex.
        """
        px = np.atleast_1d(np.asarray(px, float))
        py = np.atleast_1d(np.asarray(py, float))
        _, idx = self._tree().query(np.column_stack([px, py]))
        N = len(self.s)
        tx, ty = np.cos(self.psi), np.sin(self.psi)
        best_d = np.full(px.shape, np.inf)
        s_out = np.zeros(px.shape)
        n_out = np.zeros(px.shape)
        for off in range(-3, 3):                 # segments around the nearest sample
            a = (idx + off) % N
            b = (a + 1) % N
            dx, dy = self.x[b] - self.x[a], self.y[b] - self.y[a]
            qx, qy = px - self.x[a], py - self.y[a]
            gx, gy = tx[b] - tx[a], ty[b] - ty[a]
            A = -(dx * gx + dy * gy)
            B = qx * gx + qy * gy - (dx * tx[a] + dy * ty[a])
            C = qx * tx[a] + qy * ty[a]
            lin = -C / np.where(np.abs(B) > 1e-12, B, -1e-12)
            disc = np.sqrt(np.maximum(B * B - 4 * A * C, 0.0))
            Asafe = np.where(np.abs(A) > 1e-12, A, 1.0)
            r1, r2 = (-B + disc) / (2 * Asafe), (-B - disc) / (2 * Asafe)
            quad = np.where(np.abs(r1 - lin) < np.abs(r2 - lin), r1, r2)
            lam = np.where(np.abs(A) > 1e-9 * np.maximum(np.abs(B), 1e-12), quad, lin)
            lam = np.clip(lam, 0.0, 1.0)
            cx, cy = self.x[a] + lam * dx, self.y[a] + lam * dy
            ux, uy = (1 - lam) * tx[a] + lam * tx[b], (1 - lam) * ty[a] + lam * ty[b]
            un = np.hypot(ux, uy)
            ux, uy = ux / un, uy / un
            ex, ey = px - cx, py - cy
            d = np.hypot(ex, ey)
            better = d < best_d
            best_d = np.where(better, d, best_d)
            seg = np.hypot(dx, dy)
            s_out = np.where(better, np.mod(self.s[a] + lam * seg, self.length), s_out)
            n_out = np.where(better, ux * ey - uy * ex, n_out)
        return s_out, n_out
