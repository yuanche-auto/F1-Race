"""Control expansion, phase A: the dynamic MPCC (control/mpcc.py)."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def _plan():
    return pickle.load(open(CACHE, "rb"))["plans"][0.95]


def _mpcc(track, **kw):
    from f1race.control.mpcc import DynamicMPCC
    from f1race.experiments.l2 import ref_from_plan

    return DynamicMPCC(track, DynVehicle(), ref_from_plan(_plan()), **kw)


def test_path_errors_signs(track):
    """Liniger eq. 11 on the centre line: a point 2 m left of the reference point has e_c = -2 and
    no lag error; a point 3 m further along the path has e_l = -3 and no contouring error."""
    from f1race.control.mpcc import path_errors

    s0 = np.array([1000.0])
    left = track.point_at(s0, 2.0)
    ec, el = path_errors(track, left[:, 0], left[:, 1], s0)
    assert abs(ec[0] + 2.0) < 1e-6 and abs(el[0]) < 1e-6
    ahead = track.point_at(s0 + 3.0)
    ec, el = path_errors(track, ahead[:, 0], ahead[:, 1], s0)
    assert abs(el[0] + 3.0) < 0.01 and abs(ec[0]) < 0.02          # chord of a curved path


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_model_matches_simulator(track):
    """Within the friction limits (no saturation in the simulator) the MPCC's continuous dynamics
    of (X, Y, psi, vx, vy, r, delta) are the simulator's, and theta' = v_theta."""
    from f1race.sim.dynamic import DynamicSim

    mpc = _mpcc(track)
    sim = DynamicSim(track, DynVehicle(), 0.01)
    x = np.array([10.0, 5.0, 0.3, 50.0, 0.5, 0.1, 0.02])
    u = np.array([0.1, 2000.0, 0.0])
    _, (uf, ur), _ = sim._forces(x, u)
    assert uf < 1.0 and ur < 1.0                                     # inside both ellipses
    fm = np.asarray(mpc._f(np.concatenate([x, [123.0]]), [u[0], u[1] / 1e4, u[2] / 1e4, 47.0])).ravel()
    assert np.allclose(fm[:7], sim._f(x, u), rtol=1e-10, atol=1e-10)
    assert fm[7] == pytest.approx(47.0)


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_spline_errors_match_the_track(track):
    """The NLP's errors (B-splines of the centre line) agree with the track's own interpolation,
    also across the lap boundary (theta > L)."""
    from f1race.control.mpcc import path_errors

    mpc = _mpcc(track)
    rng = np.random.default_rng(0)
    for th in [5.0, 1234.5, track.length - 0.5, track.length + 7.0]:
        p = track.point_at(np.array([th]), rng.uniform(-4, 4))[0] + rng.uniform(-2, 2, 2)
        ec_n, el_n = path_errors(track, [p[0]], [p[1]], np.array([th]))
        ec, el = (float(v) for v in mpc.errors_fn(p[0], p[1], th))
        assert abs(ec - ec_n[0]) < 0.01 and abs(el - el_n[0]) < 0.01


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_first_solve_feasible(track):
    """From the start of the 5 % plan: IPOPT converges, the predicted progress increases, both axle
    points stay within the half-planes, v_theta >= 0, the inputs within bounds and the terminal speed
    below the reference speed profile at the car's approximate projection."""
    from f1race.control.mpcc import MPCCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan

    o = _plan()
    veh = DynVehicle()
    path, _ = dyn_reference(track, veh, o)
    mpc = _mpcc(track)
    ctl = MPCCController(mpc, 0.01)
    u = ctl(0.0, x0_from_plan(o, path))
    info = mpc.stats[-1]
    assert info["status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level") and not info["fallback"]
    X, U = mpc.prediction()
    assert np.all(np.diff(X[:, 7]) > 0) and np.all(U[:, 3] >= -1e-8)
    assert info["slack"] < 1e-6
    assert abs(u[0]) <= veh.delta_rate_max + 1e-9 and u[1] >= 0.0 and u[2] <= 0.0
    for arm in (veh.lf, -veh.lr):
        px, py = X[1:, 0] + arm * np.cos(X[1:, 2]), X[1:, 1] + arm * np.sin(X[1:, 2])
        ec = np.array([float(mpc.errors_centre_fn(a, b, t)[0]) for a, b, t in zip(px, py, X[1:, 7])])
        th = np.mod(X[1:, 7], track.length)
        body = veh.half_width + 0.3
        assert np.all(ec >= -(track.interp(track.w_left, th) - body) - 1e-4)
        assert np.all(ec <= track.interp(track.w_right, th) - body + 1e-4)
    el_n = float(mpc.errors_fn(X[-1, 0], X[-1, 1], X[-1, 7])[1])     # bound at theta - e_l
    vref = np.interp(np.mod(X[-1, 7] - el_n, track.length), o.s, o.vx)
    assert X[-1, 3] <= vref + 1e-3


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_raceline_reference_and_stage_speed_bound(track):
    """``reference="raceline"``: a point on the plan's line has no contouring error w.r.t. the
    reference, and its centre-line contouring error is minus the plan's offset (the half-planes stay
    those of the centre line). ``speed_bound="stages"``: every predicted speed is at most the
    reference speed at the car's approximate projection theta - e_l."""
    from f1race.control.mpcc import MPCCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan

    o = _plan()
    mpc = _mpcc(track, reference="raceline", speed_bound="stages")
    for th in (300.0, 2000.0, 3900.0):
        n = float(np.interp(th, o.s, o.n))
        p = track.point_at(np.array([th]), n)[0]
        ec, el = (float(v) for v in mpc.errors_fn(p[0], p[1], th))
        ecc, _ = (float(v) for v in mpc.errors_centre_fn(p[0], p[1], th))
        assert abs(ec) < 0.02 and abs(el) < 0.02 and abs(ecc + n) < 0.02
    path, _ = dyn_reference(track, DynVehicle(), o)
    MPCCController(mpc, 0.01)(0.0, x0_from_plan(o, path))
    assert mpc.stats[-1]["status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    X, _ = mpc.prediction()
    el = np.array([float(mpc.errors_fn(a, b, t)[1]) for a, b, t in zip(X[:, 0], X[:, 1], X[:, 7])])
    vref = np.interp(np.mod(X[1:, 7] - el[1:], track.length), o.s, o.vx)
    assert np.all(X[1:, 3] <= vref + 1e-3)


def test_progress_unwraps_across_the_lap_end(track):
    """theta_0 continues past L at the start line instead of jumping back to 0."""
    from f1race.control.mpcc import MPCCController

    class _M:                                                # only .track is used by progress()
        pass

    m = _M()
    m.track = track
    ctl = MPCCController(m, 0.01)
    p1, p2 = track.point_at(np.array([track.length - 1.0]))[0], track.point_at(np.array([1.0]))[0]
    assert ctl.progress(np.array([p1[0], p1[1]])) == pytest.approx(track.length - 1.0, abs=0.05)
    assert ctl.progress(np.array([p2[0], p2[1]])) == pytest.approx(track.length + 1.0, abs=0.05)
