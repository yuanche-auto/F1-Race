"""Closed driving path (e.g. a racing line of the rear axle)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import splev, splprep


@dataclass
class Path:
    """Closed path sampled at (possibly non-uniform) points; the closing point is omitted.

    s[i] is the arclength of point i measured from point 0; ``length`` closes the loop.
    """

    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    psi: np.ndarray      # heading, unwrapped
    kappa: np.ndarray    # curvature, > 0 left
    dkappa: np.ndarray   # d kappa / ds
    length: float

    @classmethod
    def from_points(cls, xy: np.ndarray, resample: float | None = None,
                    smooth: float = 0.0, k: int = 5) -> "Path":
        """Periodic B-spline (degree ``k``) through / near the points.

        ``smooth`` is the allowed RMS deviation from the points [m] (0 interpolates).
        With ``resample`` the path is re-sampled uniformly in arclength, otherwise it
        is evaluated at the parameters of the given points (1:1 correspondence).
        """
        xy = np.asarray(xy, float)
        n = len(xy)
        closed = np.vstack([xy, xy[:1]])
        tck, u = splprep([closed[:, 0], closed[:, 1]], s=n * smooth**2, per=1, k=k)
        uu = np.linspace(0.0, 1.0, 20 * n + 1)
        xx, yy = splev(uu, tck)
        sarc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xx), np.diff(yy)))])
        length = float(sarc[-1])
        if resample:
            m = int(round(length / resample))
            s = np.arange(m) * (length / m)
            uq = np.interp(s, sarc, uu)
        else:
            uq = np.asarray(u)[:-1]
            s = np.interp(uq, uu, sarc)
        x, y = splev(uq, tck)
        d1x, d1y = splev(uq, tck, der=1)
        d2x, d2y = splev(uq, tck, der=2)
        d3x, d3y = splev(uq, tck, der=3)
        sp2 = d1x**2 + d1y**2
        cross = d1x * d2y - d1y * d2x
        kappa = cross / sp2**1.5
        dcross = d1x * d3y - d1y * d3x
        dsp2 = 2.0 * (d1x * d2x + d1y * d2y)
        dk_du = dcross / sp2**1.5 - 1.5 * cross * dsp2 / sp2**2.5
        dkappa = dk_du / np.sqrt(sp2)
        psi = np.unwrap(np.arctan2(d1y, d1x))
        return cls(s, np.asarray(x), np.asarray(y), psi, kappa, dkappa, length)

    @property
    def ds(self) -> np.ndarray:
        """Distance from point i to point i+1 (periodic), length N."""
        return np.diff(np.concatenate([self.s, [self.length]]))

    def interp(self, field: np.ndarray, s) -> np.ndarray:
        s = np.mod(np.asarray(s, dtype=float), self.length)
        return np.interp(s, np.concatenate([self.s, [self.length]]),
                         np.concatenate([field, field[:1]]))

    def heading_at(self, s):
        return np.arctan2(self.interp(np.sin(self.psi), s), self.interp(np.cos(self.psi), s))
