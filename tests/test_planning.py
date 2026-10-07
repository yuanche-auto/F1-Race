import numpy as np

from f1race.path import Path as RPath
from f1race.planning.mincurv import lateral_bounds
from f1race.planning.qss import pointwise_v2_max, speed_profile


def _steady_circle_v2(vehicle, k):
    """Sustainable speed^2 on a circle: the ellipse must leave F_x = drag + rolling."""
    lo, hi = 0.0, float(vehicle.v2_lateral_max(k))
    for _ in range(200):
        w = 0.5 * (lo + hi)
        if vehicle.fx_available(w, k) >= vehicle.resist(w):
            lo = w
        else:
            hi = w
    return lo


def test_qss_circle_matches_analytic(circle_path, vehicle):
    R, path = circle_path
    prof = speed_profile(path, vehicle)
    v_an = np.sqrt(_steady_circle_v2(vehicle, 1.0 / R))
    assert v_an < np.sqrt(vehicle.v2_lateral_max(1.0 / R))   # drag binds before the pure lateral limit
    assert np.allclose(prof.v, v_an, rtol=2e-3)
    assert np.isclose(prof.lap_time, path.length / v_an, rtol=2e-3)


def test_qss_feasible_and_maximal(mincurv, vehicle):
    from f1race.planning.qss import _bwd, _fwd

    _, path = mincurv
    prof = speed_profile(path, vehicle)
    w = prof.v**2
    wn = np.roll(w, -1)
    kn = np.roll(path.kappa, -1)
    ds = path.ds
    a = (wn - w) / (2 * ds)
    # physical feasibility: the segment acceleration is admissible at both ends
    for ww, kk in ((w, path.kappa), (wn, kn)):
        amin, amax = vehicle.accel_limits(ww, kk)
        assert np.all(a <= amax + 1e-6)
        assert np.all(a >= amin - 1e-6)
    wmax = pointwise_v2_max(path, vehicle)
    assert np.all(w <= wmax * (1 + 1e-9))
    # maximality: every point is limited by its own bound or by a neighbour's pass
    # (up to 0.1 %: the conservative two-end rule is not exactly symmetric between passes)
    lim_fwd = np.roll([_fwd(vehicle, wi, ki, kj, d) for wi, ki, kj, d in zip(w, path.kappa, kn, ds)], 1)
    lim_bwd = np.array([_bwd(vehicle, wj, kj, ki, d) for wj, kj, ki, d in zip(wn, kn, path.kappa, ds)])
    tight = np.minimum.reduce([wmax, lim_fwd, lim_bwd])
    assert np.allclose(w, tight, rtol=1e-3, atol=1e-6)


def test_mincurv_bounds_and_gain(track, vehicle, mincurv):
    alpha, path = mincurv
    idx = np.arange(0, len(track.s), 3)
    lb, ub = lateral_bounds(track, vehicle, 0.3, None, idx)
    assert np.all(alpha >= lb - 1e-4) and np.all(alpha <= ub + 1e-4)
    centre = RPath.from_points(np.column_stack([track.x[idx], track.y[idx]]))
    assert np.sum(path.kappa**2 * path.ds) < 0.7 * np.sum(centre.kappa**2 * centre.ds)
    assert speed_profile(path, vehicle).lap_time < speed_profile(centre, vehicle).lap_time - 3.0
