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
    _, path = mincurv
    prof = speed_profile(path, vehicle)
    w = prof.v**2
    wn = np.roll(w, -1)
    ds = path.ds
    a = (wn - w) / (2 * ds)
    # every segment respects the limits used by the passes (end-point evaluation)
    amin_end, _ = vehicle.accel_limits(wn, np.roll(path.kappa, -1))
    _, amax_start = vehicle.accel_limits(w, path.kappa)
    assert np.all(a <= amax_start + 1e-6)
    assert np.all(a >= amin_end - 1e-6)
    assert np.all(w <= pointwise_v2_max(path, vehicle) * (1 + 1e-9))
    # maximality: every point is limited by its own bound or by a neighbour's pass
    lim_fwd = np.roll(w + 2 * np.roll(ds, 0) * amax_start, 1)
    lim_bwd = wn - 2 * ds * amin_end
    wmax = pointwise_v2_max(path, vehicle)
    tight = np.minimum.reduce([wmax, lim_fwd, lim_bwd])
    assert np.allclose(w, tight, rtol=1e-6, atol=1e-6)


def test_mincurv_bounds_and_gain(track, vehicle, mincurv):
    alpha, path = mincurv
    idx = np.arange(0, len(track.s), 3)
    lb, ub = lateral_bounds(track, vehicle, 0.3, None, idx)
    assert np.all(alpha >= lb - 1e-4) and np.all(alpha <= ub + 1e-4)
    centre = RPath.from_points(np.column_stack([track.x[idx], track.y[idx]]))
    assert np.sum(path.kappa**2 * path.ds) < 0.7 * np.sum(centre.kappa**2 * centre.ds)
    assert speed_profile(path, vehicle).lap_time < speed_profile(centre, vehicle).lap_time - 3.0
