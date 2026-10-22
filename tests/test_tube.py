"""Control expansion, phase C: the tube tightening (control/tube.py) - CX-C protocol section 3, development checks
9.1-9.2."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"
W = {"n": 2.0, "xi": 20.0, "v": 0.4}            # nmpc's CXV choice c = 2 (phase A), margin 0.6
needs_plan = pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")


def _plan():
    return pickle.load(open(CACHE, "rb"))["plans"][0.95]


def _mpc(track):
    from f1race.control.nmpc import FrenetMPC
    from f1race.experiments.l2 import ref_from_plan

    return FrenetMPC(track, DynVehicle(), ref_from_plan(_plan()), mode="track", N=30, dt=0.05, eta=0.95, margin=0.6,
                     weights=W, param_tightening=True)


def _tube(mpc, **kw):
    from f1race.control.tube import Tube
    from f1race.experiments.l2 import PERTURBATION

    return Tube(mpc, PERTURBATION, margin=0.6, **kw)


def _x0(s):
    o = _plan()
    return np.array([s] + [float(np.interp(s, o.s, getattr(o, k))) for k in ("n", "xi", "vx", "vy", "r", "delta")])


def test_vertex_vehicle_is_the_l2_box():
    """2^7 vertices of l2.PERTURBATION, applied as sample_plants applies a draw."""
    from dataclasses import replace

    from f1race.control.tube import PARAM_ORDER, vertex_vehicle
    from f1race.experiments.l2 import PERTURBATION

    nom = DynVehicle()
    veh, V = vertex_vehicle(nom, PERTURBATION)
    assert V.shape == (128, 7) and len({tuple(v) for v in V}) == 128
    for i, k in enumerate(PARAM_ORDER):
        assert set(V[:, i]) == set(PERTURBATION[k])
    j = 37
    f = dict(zip(PARAM_ORDER, V[j]))
    ref = replace(nom, mu_x=nom.mu_x * f["mu"], mu_y=nom.mu_y * f["mu"], m=nom.m * f["m"], ClA=nom.ClA * f["ClA"],
                  CdA=nom.CdA * f["CdA"], Iz=nom.Iz * f["Iz"], B=nom.B * f["B"], aero_front=nom.aero_front + f["aero_front"])
    for k in ("mu_x", "mu_y", "m", "ClA", "CdA", "Iz", "B", "aero_front"):
        assert getattr(veh, k)[j, 0] == pytest.approx(getattr(ref, k), rel=1e-15)


@needs_plan
def test_frenet_step_is_the_nmpc_step(track):
    """The numpy step (nominal and, broadcast, the vertex cars) is FrenetMPC's CasADi step."""
    from dataclasses import replace

    from f1race.control.nmpc import FS, FrenetMPC
    from f1race.control.tube import frenet_step, vertex_vehicle
    from f1race.experiments.l2 import PERTURBATION, ref_from_plan

    mpc, tube = _mpc(track), None
    tube = _tube(mpc)
    rng = np.random.default_rng(0)
    X = np.column_stack([rng.uniform(0, track.length, 20), rng.uniform(-3, 3, 20), rng.uniform(-0.1, 0.1, 20),
                         rng.uniform(20, 85, 20), rng.uniform(-1, 1, 20), rng.uniform(-0.5, 0.5, 20),
                         rng.uniform(-0.2, 0.2, 20)])
    U = np.column_stack([rng.uniform(-1, 1, 20), rng.uniform(0, 1.5e4, 20), rng.uniform(-3e4, 0, 20)])
    ref = np.array([np.asarray(mpc._step(X[k], U[k] / np.array([1.0, FS, FS]))).ravel() for k in range(20)])
    assert np.max(np.abs(frenet_step(mpc.veh, tube.kap, X, U, 0.05) - ref) / np.maximum(np.abs(ref), 1.0)) < 1e-12
    veh_v, V = vertex_vehicle(DynVehicle(), PERTURBATION)
    j = 5
    car = replace(DynVehicle(), mu_x=veh_v.mu_x[j, 0], mu_y=veh_v.mu_y[j, 0], m=veh_v.m[j, 0], ClA=veh_v.ClA[j, 0],
                  CdA=veh_v.CdA[j, 0], Iz=veh_v.Iz[j, 0], B=veh_v.B[j, 0], aero_front=veh_v.aero_front[j, 0])
    mpc_j = FrenetMPC(track, car, ref_from_plan(_plan()), N=30, dt=0.05, eta=0.95, margin=0.6, weights=W)
    ref_j = np.array([np.asarray(mpc_j._step(X[k], U[k] / np.array([1.0, FS, FS]))).ravel() for k in range(20)])
    got = frenet_step(veh_v, tube.kap, X[None], U[None], 0.05)
    assert got.shape == (128, 20, 7)
    assert np.max(np.abs(got[j] - ref_j) / np.maximum(np.abs(ref_j), 1.0)) < 1e-12


@needs_plan
def test_jacobians_match_finite_differences(track):
    """Section 9.1: the CasADi Jacobians of the NMPC's step against central differences (relative 1e-6)."""
    from f1race.control.nmpc import FS

    mpc = _mpc(track)
    tube = _tube(mpc)
    X = np.array([_x0(s) for s in np.linspace(100, 4000, 31)])
    U = np.column_stack([np.linspace(-0.2, 0.2, 30), np.linspace(0, 8e3, 30), np.linspace(-2e4, 0, 30)])
    A, B = tube.linearise(X, U)
    step = lambda x, u: np.asarray(mpc._step(x, u / np.array([1.0, FS, FS]))).ravel()
    for k in (0, 7, 18, 29):
        Ad = np.column_stack([(step(X[k] + e, U[k]) - step(X[k] - e, U[k])) / (2 * 1e-6) for e in np.eye(7) * 1e-6])
        # B is per scaled input (rad/s, 1e4 N, 1e4 N); the differences are taken in rad/s and N
        Bd = np.column_stack([(step(X[k], U[k] + e) - step(X[k], U[k] - e)) / (2 * de) * sc for e, de, sc in
                              zip(np.diag([1e-6, 1e-2, 1e-2]), (1e-6, 1e-2, 1e-2), (1.0, FS, FS))])
        assert np.max(np.abs(A[k] - Ad)) < 1e-6 * max(1.0, np.abs(Ad).max())
        assert np.max(np.abs(B[k] - Bd)) < 1e-5 * max(1.0, np.abs(Bd).max())


def test_reach_bounds_are_the_support_function():
    """Section 9.2: random disturbance sequences in the box never exceed b_k; the sign-matched vertex sequence attains it."""
    from f1race.control.tube import reach_bounds

    rng = np.random.default_rng(1)
    N, n = 12, 4
    Acl = 0.9 * rng.normal(size=(N, n, n)) / np.sqrt(n)
    Wb = rng.uniform(0.01, 0.2, size=(N, n))
    C = rng.normal(size=(N + 1, n))
    b = reach_bounds(Acl, Wb, C)
    assert b[0] == 0.0
    E = np.zeros((10000, n))
    worst = np.zeros(N + 1)
    for j in range(N):
        E = E @ Acl[j].T + rng.uniform(-1, 1, size=(10000, n)) * Wb[j]
        worst[j + 1] = np.abs(E @ C[j + 1]).max()
    assert np.all(worst <= b + 1e-12)
    k = N                                                   # tightness at the last stage
    r, w = C[k].copy(), np.zeros((N, n))
    for j in range(k - 1, -1, -1):
        w[j] = np.sign(r) * Wb[j]
        r = r @ Acl[j]
    e = np.zeros(n)
    for j in range(k):
        e = Acl[j] @ e + w[j]
    assert C[k] @ e == pytest.approx(b[k], rel=1e-12)


def test_tv_lqr_converges_to_the_stationary_gain():
    """A long time-invariant horizon gives the discrete algebraic Riccati gain at stage 0."""
    from scipy.linalg import solve_discrete_are

    from f1race.control.tube import tv_lqr

    rng = np.random.default_rng(2)
    A = np.eye(3) + 0.1 * rng.normal(size=(3, 3))
    B = rng.normal(size=(3, 2))
    Q, R = np.eye(3), 0.5 * np.eye(2)
    K = tv_lqr(np.repeat(A[None], 300, 0), np.repeat(B[None], 300, 0), Q, R)
    P = solve_discrete_are(A, B, Q, R)
    assert np.allclose(K[0], -np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A), atol=1e-8)


@needs_plan
def test_compute_shapes_caps_and_track_only(track):
    """eta (N) within [eta_min, eta], back-off (N + 1) from 0 and at most b_max; track_only leaves eta alone."""
    mpc = _mpc(track)
    x0 = _x0(380.0)
    mpc.solve(x0, np.array([0.0, 2000.0, 0.0]), eta=np.full(30, 0.95), backoff=np.zeros(31))
    X, U = mpc.prediction()
    for kw in ({}, {"track_only": True}, {"b_max": 0.05}, {"c_R": 10.0}):
        tb = _tube(mpc, **kw).compute(X, U)
        assert tb["eta"].shape == (30,) and tb["backoff"].shape == (31,) and tb["backoff"][0] == 0.0
        assert np.all(tb["eta"] <= 0.95 + 1e-12) and np.all(tb["eta"] >= 0.70 - 1e-12)
        assert np.all(tb["backoff"] <= _tube(mpc, **kw).b_max + 1e-12) and np.all(tb["W"] >= 0)
        if kw.get("track_only"):
            assert np.all(tb["eta"] == 0.95) and tb["capped_eta_share"] == 0.0
        if "b_max" in kw:
            assert tb["capped_track_share"] > 0
    t1, t10 = _tube(mpc).compute(X, U), _tube(mpc, c_R=10.0).compute(X, U)
    assert t10["b_raw"][-1] != t1["b_raw"][-1]                                  # c_R changes the tube


@needs_plan
def test_closed_loop_smoke(track):
    """The tube controller for 1 s on the nominal car: tightening at every solve, prediction records kept."""
    from f1race.control.tube import TubeMPCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.sim.dynamic import DynamicSim

    o, veh = _plan(), DynVehicle()
    path, _ = dyn_reference(track, veh, o)
    mpc = _mpc(track)
    ctl = TubeMPCController(mpc, _tube(mpc), 0.01)
    res = DynamicSim(track, veh, 0.01).run(ctl, x0_from_plan(o, path), t_max=1.0)
    assert res.stop_reason == "time limit" and len(ctl.tube_log) == 20
    assert all(r["X_pred"].shape == (11, 7) and np.isfinite(r["tube_time"]) for r in ctl.tube_log)
