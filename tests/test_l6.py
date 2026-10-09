"""L6: plant with a steering actuator lag, lag model of the MPC, observer of the lag."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def _plan():
    return pickle.load(open(CACHE, "rb"))["plans"][0.95]


def test_lagged_step_matches_fine_integration(track):
    """The exact lag over one step agrees with a fine RK4 of the 8-dim equations."""
    from f1race.sim.lagged import LaggedSim

    veh = DynVehicle()
    sim = LaggedSim(track, veh, 0.03, dt=0.01)
    fine = LaggedSim(track, veh, 0.03, dt=0.01)
    x = np.array([0.0, 0.0, 0.1, 60.0, 0.3, 0.2, 0.02, 0.05])
    u = np.array([0.8, 2000.0, 0.0])
    xs = sim.step(x, u)
    xf, h = x.copy(), 1e-4
    for _ in range(100):                                     # plain RK4 on _f, step 1e-4
        k1 = fine._f(xf, u); k2 = fine._f(xf + h / 2 * k1, u)
        k3 = fine._f(xf + h / 2 * k2, u); k4 = fine._f(xf + h * k3, u)
        xf = xf + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    assert np.max(np.abs(xs[6:] - xf[6:])) < 1e-12           # the lag itself is exact
    h = 0.01                                                 # the rest: as accurate as plain RK4
    k1 = fine._f(x, u); k2 = fine._f(x + h / 2 * k1, u)
    k3 = fine._f(x + h / 2 * k2, u); k4 = fine._f(x + h * k3, u)
    xp = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    assert np.max(np.abs(xs[:6] - xf[:6])) <= 2.0 * np.max(np.abs(xp[:6] - xf[:6])) + 1e-9


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_tiny_lag_reproduces_the_plant_without_lag(track):
    from f1race.control.dyn_tracking import DynTracker
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.sim.dynamic import DynamicSim
    from f1race.sim.lagged import LaggedSim

    o, veh = _plan(), DynVehicle()
    path, ff = dyn_reference(track, veh, o)
    g = dict(k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    a = DynamicSim(track, veh, 0.01).run(DynTracker(path, ff, veh, 0.01, **g), x0_from_plan(o, path), t_max=8.0)
    for obs in ("command", "angle"):
        b = LaggedSim(track, veh, 1e-5, 0.01, observe=obs).run(DynTracker(path, ff, veh, 0.01, **g),
                                                              x0_from_plan(o, path), t_max=8.0)
        assert b.state.shape[1] == 8
        assert np.max(np.abs(a.state[-1, :6] - b.state[-1, :6])) < 1e-3


def test_observe_modes(track):
    from f1race.sim.lagged import LaggedSim

    x = np.arange(8, dtype=float)
    assert LaggedSim(track, DynVehicle(), 0.05, observe="command").observe(x)[6] == 7.0
    assert LaggedSim(track, DynVehicle(), 0.05, observe="angle").observe(x)[6] == 6.0
    assert len(LaggedSim.lift(np.arange(7.0))) == 8


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_lag_model_and_observer_match_the_plant(track):
    """The MPC's lag model predicts the plant's road-wheel angle exactly for a held rate,
    and the controller's observer reconstructs it from the commanded angle alone."""
    from f1race.control.nmpc import FrenetMPC, LagAwareMPCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, x0_from_plan
    from f1race.sim.lagged import LaggedSim

    o, veh = _plan(), DynVehicle()
    tau = 0.06
    mpc = FrenetMPC(track, veh, ref_from_plan(o), N=8, dt=0.05, eta=0.95, steer_lag=tau)
    ctl = LagAwareMPCController(mpc, 0.01)
    path, _ = dyn_reference(track, veh, o)
    sim = LaggedSim(track, veh, tau, 0.01)
    seen = []

    def wrapped(t, y):
        u = ctl(t, y)
        seen.append((t, ctl.d_hat))
        return u

    r = sim.run(wrapped, x0_from_plan(o, path), t_max=1.0)
    d_true = r.state[:, 6]
    d_hat = np.array([d for _, d in seen])[: len(d_true)]
    assert np.max(np.abs(d_hat - d_true)) < 1e-6
    X, U = mpc.prediction()
    assert X.shape == (9, 8)
    a0, c0, rho, h = X[0, 6], X[0, 7], U[0, 0], 0.05
    exact = c0 + rho * (h - tau) + (a0 - c0 + rho * tau) * np.exp(-h / tau)
    assert abs(X[1, 6] - exact) < 1e-6 and abs(X[1, 7] - (c0 + rho * h)) < 1e-9


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_env_with_lag_reproduces_the_tracker(track):
    pytest.importorskip("gymnasium")
    from f1race.control.dyn_tracking import DynTracker
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.rl.env import RacingEnv, evaluate_lap
    from f1race.sim.lagged import LaggedSim

    o, veh = _plan(), DynVehicle()
    env = RacingEnv(o, randomize_plant=False)
    r = evaluate_lap(None, env, t_max=60.0, steer_lag=0.03)
    assert len(env.x) == 8
    path, ff = dyn_reference(track, veh, o)
    ctl = DynTracker(path, ff, veh, 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    ref = LaggedSim(track, veh, 0.03, 0.01).run(ctl, x0_from_plan(o, path), t_max=60.0)
    assert r["stop_reason"] == ref.stop_reason
    assert abs(r["distance"] - ref.progress[-1]) < 4.0


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_lag_identification_recovers_tau_and_parameters(track):
    """The lag filter reproduces the plant's road-wheel angle from the commanded angle, and
    the identification on the model with the lag recovers tau and the parameters, while the
    identification without it does not."""
    from f1race.control.dyn_tracking import DynTracker
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import sample_plants, x0_from_plan
    from f1race.estimation.identify import data_from_run, identify, identify_with_lag, lag_filter, theta_of
    from f1race.experiments.l6 import observed_run
    from f1race.sim.lagged import LaggedSim

    o, nom = _plan(), DynVehicle()
    info, plant = sample_plants(nom, 3, 0)[2]
    path, ff = dyn_reference(track, nom, o)
    sim = LaggedSim(track, plant, 0.04, 0.01)
    ctl = DynTracker(path, ff, nom, 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    r = sim.run(ctl, x0_from_plan(o, path), t_max=12.0)
    assert np.max(np.abs(lag_filter(r.state[:, 7], 0.01, 0.04) - r.state[:, 6])) < 1e-9
    ro = observed_run(r, sim)
    ident = identify_with_lag(ro, 0.01, nom)
    assert abs(ident["tau"] - 0.04) < 1e-3
    assert np.max(np.abs(ident["theta"] - theta_of(info))) < 0.02
    plain = identify(data_from_run(ro, 0.01), nom)
    assert plain["rms"] > 20 * ident["rms"]


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_lag_aware_controller_with_measured_angle(track):
    """With the road-wheel angle measured, the controller's own commanded angle (from its
    rate commands and the actuator limits) equals the plant's."""
    from f1race.control.nmpc import FrenetMPC, LagAwareMPCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, x0_from_plan
    from f1race.sim.lagged import LaggedSim

    o, veh = _plan(), DynVehicle()
    mpc = FrenetMPC(track, veh, ref_from_plan(o), N=8, dt=0.05, eta=0.95, steer_lag=0.06)
    ctl = LagAwareMPCController(mpc, 0.01, observe="angle")
    path, _ = dyn_reference(track, veh, o)
    seen = []

    def wrapped(t, y):
        u = ctl(t, y)
        seen.append(ctl.c_hat)
        return u

    r = LaggedSim(track, veh, 0.06, 0.01, observe="angle").run(wrapped, x0_from_plan(o, path), t_max=1.0)
    c_true = r.state[1:, 7]
    assert np.max(np.abs(np.array(seen)[: len(c_true)] - c_true)) < 1e-12


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_parametric_tightening_and_zero_residual_match_the_plain_mpc(track):
    """Tightening passed as NLP parameters, and a residual that is zero, give the same
    solution as the formulation without these options (L3-style constants)."""
    import casadi as ca

    from f1race.control.nmpc import FrenetMPC
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, x0_from_plan
    from f1race.control.nmpc import MPCController

    o, veh = _plan(), DynVehicle()
    path, _ = dyn_reference(track, veh, o)
    N = 10
    eta = np.linspace(0.95, 0.90, N)
    b = np.r_[0.0, np.linspace(0.1, 0.5, N)]
    z = ca.SX.sym("z", 7)
    zero = ca.Function("zero", [z], [ca.DM.zeros(3) * z[0]])
    a = FrenetMPC(track, veh, ref_from_plan(o), N=N, dt=0.05, eta=eta, backoff=b)
    p = FrenetMPC(track, veh, ref_from_plan(o), N=N, dt=0.05, eta=0.95, param_tightening=True, residual=zero)
    xf = MPCController(a, 0.01).frenet(x0_from_plan(o, path))
    u_prev = np.array([0.0, 1000.0, 0.0])
    ua, _ = a.solve(xf, u_prev)
    up, _ = p.solve(xf, u_prev, eta=eta, backoff=b)
    assert np.allclose(ua, up, rtol=1e-5, atol=1e-3)
    assert np.allclose(a.z[a.Xi], p.z[p.Xi], atol=1e-5)


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_ekf_reduces_the_noise_bias(track):
    """With the L3 sensor noise on the logged states, the EKF estimate is closer to the
    true parameters than the least squares on the raw data (and is exact-ish without noise)."""
    from dataclasses import replace

    from f1race.control.dyn_tracking import DynTracker
    from f1race.estimation.ekf import ekf_identify
    from f1race.estimation.identify import data_from_run, identify, theta_of
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import sample_plants, x0_from_plan
    from f1race.sim.dynamic import DynamicSim
    from f1race.sim.stochastic import NOISE_SIGMA

    o, nom = _plan(), DynVehicle()
    info, plant = sample_plants(nom, 4, 0)[3]
    path, ff = dyn_reference(track, nom, o)
    ctl = DynTracker(path, ff, nom, 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    r = DynamicSim(track, plant, 0.01).run(ctl, x0_from_plan(o, path), t_max=20.0)
    true = theta_of(info)
    noisy = replace(r, state=r.state + NOISE_SIGMA * np.random.default_rng(7).standard_normal(r.state.shape))
    ls = identify(data_from_run(noisy, 0.01), nom)["theta"]
    ekf = ekf_identify(noisy.state, noisy.inputs, noisy.usage, 0.01, nom, NOISE_SIGMA)["theta"]
    assert np.abs(ekf - true).max() < 0.5 * np.abs(ls - true).max()
    assert np.abs(ekf - true).max() < 0.1
