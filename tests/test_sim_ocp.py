import numpy as np
import pytest

from f1race.experiments.l0 import plan_from_points, run_closed_loop
from f1race.planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from f1race.planning.qss import speed_profile
from f1race.sim.kinematic import KinematicSim


def test_sim_circle_constant_steering(track, vehicle):
    sim = KinematicSim(track, vehicle, dt=0.01, stop_on_violation=False)
    delta, v = 0.05, 20.0
    x = np.array([0.0, 0.0, 0.0, v, delta])
    fx = vehicle.resist(v * v)                      # hold the speed
    R = vehicle.L / np.tan(delta)
    T = 2 * np.pi * R / v
    for _ in range(int(round(T / sim.dt))):
        x = sim.step(x, np.array([0.0, fx]))
    assert abs(x[3] - v) < 1e-9
    assert np.hypot(x[0], x[1]) < 0.6 * v * sim.dt          # back at the start (T/dt rounding)
    assert np.isclose(np.hypot(x[0], x[1] - R), R, atol=1e-6)


@pytest.mark.parametrize("dt", [0.02, 0.04])
def test_closed_loop_follows_qss_plan(track, vehicle, mincurv, dt):
    from f1race.path import Path as RPath

    _, path = mincurv
    dense = RPath.from_points(np.column_stack([path.x, path.y]), resample=0.5)
    prof = speed_profile(dense, vehicle, eta=0.97)
    r = run_closed_loop(track, vehicle, dense, prof.v, prof.a, None, dt=dt)
    assert r.completed
    assert abs(r.lap_time - prof.lap_time) < 0.3
    assert r.max_usage <= 0.995 + 1e-9 and r.min_margin > 0.0


@pytest.mark.slow
@pytest.mark.parametrize("dt", [0.02, 0.04])
def test_closed_loop_follows_ocp_plan(track, vehicle, mincurv, dt):
    alpha, path = mincurv
    prof = speed_profile(path, vehicle)
    idx = np.arange(0, len(track.s), 3)
    init = initial_guess_from_path(track, vehicle, idx, alpha, path, prof.v, prof.a)
    o = _ocp_cache(track, vehicle, init)
    dense, v, a, k = plan_from_points(o.xy(track), o.v, kappa=np.tan(o.delta) / vehicle.L)
    r = run_closed_loop(track, vehicle, dense, v, a, k, dt=dt)
    assert r.completed
    assert abs(r.lap_time - o.lap_time) < 0.3
    assert r.max_usage <= 0.995 + 1e-9 and r.min_margin > 0.0


_OCP = {}


def _ocp_cache(track, vehicle, init):
    if "plan" not in _OCP:
        _OCP["plan"] = solve_min_time(track, vehicle, init, ds=2.0, margin=0.3, eta=0.97)
        assert _OCP["plan"].success
    return _OCP["plan"]


@pytest.mark.slow
def test_ocp_beats_qss_on_mincurv(track, vehicle, mincurv):
    alpha, path = mincurv
    prof = speed_profile(path, vehicle)
    idx = np.arange(0, len(track.s), 3)
    init = initial_guess_from_path(track, vehicle, idx, alpha, path, prof.v, prof.a)
    res = solve_min_time(track, vehicle, init, ds=6.0, margin=0.3)
    assert res.success
    assert res.lap_time < prof.lap_time
    assert np.all(np.abs(res.delta) <= vehicle.delta_max + 1e-6)
