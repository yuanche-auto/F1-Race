"""Control expansion phase D: the estimators' step model, the RLS, the model bank, the parameterised NLP and the adaptive
controller (docs/CX_D_protocol.md sections 3.1-3.5)."""
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle
from f1race.estimation.identify import HI, LO, X0, theta_of
from f1race.estimation.rls import RecursiveLS, _vehicle, step_function, step_np
from f1race.experiments.l2 import sample_plants
from f1race.sim.dynamic import DynamicSim

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"
needs_plan = pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
DT = 0.01


@pytest.fixture(scope="module")
def track():
    from f1race.track import Track
    return Track.load("Spielberg")


def _car(i=1):
    info, car = sample_plants(DynVehicle(), 5, 399)[i]
    return theta_of(info), car


def _samples(sim, rng, n):
    for _ in range(n):
        x = np.array([0, 0, 0, rng.uniform(20, 88), rng.uniform(-2.5, 2.5), rng.uniform(-0.7, 0.7), rng.uniform(-0.25, 0.25)])
        u = sim.apply_limits(x, [rng.uniform(-1, 1), rng.uniform(0, 2.5e4), rng.uniform(-5e4, 0)])
        yield x, u, sim.step(x, u)


def test_step_model_is_the_simulator_step(track):
    th, car = _car()
    sim = DynamicSim(track, car, DT)
    fn = step_function(DynVehicle(), DT)
    sat = 0
    for x, u, xn in _samples(sim, np.random.default_rng(0), 300):
        sat += max(sim._forces(x, u)[1]) > 1.0
        assert np.allclose(np.asarray(fn(x[3:7], u, th)[0]).ravel(), xn[3:6], rtol=0, atol=1e-12)
        assert np.array_equal(step_np(_vehicle(DynVehicle(), th), x[3:7], u, DT), xn[3:6])
    assert sat > 50                                    # the saturated branch is covered


def test_step_jacobian_matches_differences():
    th, _ = _car()
    fn = step_function(DynVehicle(), DT)
    x, u = np.array([60.0, 0.5, 0.3, 0.05]), np.array([0.1, 5e3, 0.0])
    J = np.asarray(fn(x, u, th)[1])
    for i in range(7):
        e = np.zeros(7)
        e[i] = 1e-6
        fd = (np.asarray(fn(x, u, th + e)[0]).ravel() - np.asarray(fn(x, u, th - e)[0]).ravel()) / 2e-6
        assert np.allclose(J[:, i], fd, rtol=1e-6, atol=1e-7 * np.abs(J).max())


def test_rls_identifies_a_car_from_exciting_noise_free_steps(track):
    th, car = _car(2)
    sim = DynamicSim(track, car, DT)
    est = RecursiveLS(DynVehicle(), DT, lam_mu=0.999)
    for x, u, xn in _samples(sim, np.random.default_rng(1), 400):
        est.update(x[3:7], u, xn[3:6])
    err = np.abs(est.theta - th)              # the recursion keeps its early linearisations: CdA (little information per
    assert err[3] < 0.02 and np.all(np.delete(err, 3) < 2e-3)     # 10 ms step) converges slowest
    assert est.n_updates == 400 and est.diagnostics()["one_step_rms_scaled"] > 0


def test_rls_forgetting_only_on_friction_and_covariance_bounded_by_the_prior(track):
    sim = DynamicSim(track, DynVehicle(), DT)
    est = RecursiveLS(DynVehicle(), DT, lam_mu=0.99)
    P0 = np.diag(((HI - LO) / 4.0) ** 2)
    for k in range(300):                              # straight line at constant speed: little information on mu, B
        x = np.array([0, 0, 0, 60.0, 0.0, 0.0, 0.0])
        u = sim.apply_limits(x, [0.0, 3e3, 0.0])
        est.update(x[3:7], u, sim.step(x, u)[3:6])
        S = np.diag(1.0 / np.sqrt(np.diag(P0)))
        assert np.linalg.eigvalsh(S @ est.P @ S).max() <= 1.0 + 1e-9
    assert est.P[0, 0] == pytest.approx(P0[0, 0], rel=1e-6)          # forgotten up to the prior: not excited
    assert est.n_clipped > 0
    est2 = RecursiveLS(DynVehicle(), DT, lam_mu=1.0)
    est2.update(np.array([3.0, 0, 0, 0]), np.zeros(3), np.zeros(3))  # below 5 m/s: skipped
    assert est2.n_skipped == 1 and np.array_equal(est2.theta, X0)


def test_bank_selects_the_true_model_after_the_window(track):
    from f1race.estimation.bank import ModelBank

    th, car = _car(3)
    rng = np.random.default_rng(2)
    thetas = np.vstack([X0, LO + (HI - LO) * rng.random((50, 7))])
    thetas[17] = th
    bank = ModelBank(DynVehicle(), DT, window=0.05, thetas=thetas)
    sim = DynamicSim(track, car, DT)
    for k, (x, u, xn) in enumerate(_samples(sim, rng, 8)):
        bank.update(x[3:7], u, xn[3:6])
        assert bank.sel == (0 if k < 4 else 17)        # nominal until the window (5 steps) is full
    assert np.array_equal(bank.theta, th)
    assert min(1.0, th[0]) < bank.mu < max(1.0, th[0])             # smoothed towards the selection
    big = ModelBank(DynVehicle(), DT)
    assert big.thetas.shape == (20_001, 7) and np.array_equal(big.thetas[0], X0)
    assert np.all(big.thetas[1:] >= LO) and np.all(big.thetas[1:] <= HI)


def test_applied_input_is_the_simulators(track):
    from f1race.control.adaptive_mpc import applied_input

    sim = DynamicSim(track, DynVehicle(), DT)
    rng = np.random.default_rng(3)
    for _ in range(200):
        x = np.array([0, 0, 0, rng.uniform(0.5, 90), 0, 0, rng.choice([-0.4, 0.4, rng.uniform(-0.4, 0.4)])])
        u = [rng.uniform(-2, 2), rng.uniform(-1e4, 5e4), rng.uniform(-5e4, 1e4)]
        assert np.array_equal(applied_input(DynVehicle(), x, u), sim.apply_limits(x, u))


@needs_plan
def test_param_model_nlp_equals_the_plain_nlp(track):
    from f1race.control.nmpc import FrenetMPC
    from f1race.estimation.identify import vehicle_from
    from f1race.experiments.l2 import ref_from_plan
    from f1race.experiments.l8 import plans_for

    pl = plans_for("Spielberg")
    etas = [0.88, 0.93, 0.95, 0.97, 1.0]
    kw = dict(mode="track", N=30, dt=0.05, eta=0.95, margin=0.6, weights={"n": 2.0, "xi": 20.0, "v": 0.4})
    pm = FrenetMPC(track, DynVehicle(), [ref_from_plan(pl[e]) for e in etas], param_model=True, ref_grid=2, **kw)
    o = pl[0.95]
    i = 300
    x = np.array([o.s[i], o.n[i], o.xi[i], o.vx[i], o.vy[i], o.r[i], o.delta[i]])
    th_v = np.array([0.92, 1.05, 0.9, 1.1, 0.85, 1.15, 0.03])
    for veh, ref, args in ((DynVehicle(), pl[0.95], {}), (vehicle_from(th_v, DynVehicle()), pl[0.95], {"theta": th_v}),
                           (DynVehicle(), pl[0.88], {"plan": 0.0})):
        plain = FrenetMPC(track, veh, ref_from_plan(ref), **kw)
        pm.z = None
        ua, _ = plain.solve(x, np.zeros(3))
        ub, _ = pm.solve(x, np.zeros(3), **args)
        assert np.allclose(ua, ub, rtol=1e-8, atol=1e-6)
        assert np.allclose(plain.z[plain.Xi], pm.z[pm.Xi], atol=1e-8)
    with pytest.raises(ValueError):
        FrenetMPC(track, DynVehicle(), ref_from_plan(o), **kw).solve(x, np.zeros(3), theta=X0)


@needs_plan
def test_adaptive_controller_plan_coordinate_and_sources(track):
    from f1race.control.adaptive_mpc import AdaptiveMPCController
    from f1race.control.nmpc import FrenetMPC
    from f1race.experiments.l2 import ref_from_plan
    from f1race.experiments.l8 import plans_for

    pl = plans_for("Spielberg")
    etas = [0.88, 0.93, 0.95, 0.97, 1.0]
    mpc = FrenetMPC(track, DynVehicle(), [ref_from_plan(pl[e]) for e in etas], mode="track", eta=0.95, margin=0.6,
                    param_model=True, ref_grid=2)
    est = RecursiveLS(DynVehicle(), DT)
    ctl = AdaptiveMPCController(mpc, DT, etas, estimator=est)
    assert ctl.plan_coordinate(1.0) == pytest.approx(2.0)            # 0.95 x 1.0 -> the 0.95 plan
    assert ctl.plan_coordinate(0.5) == 0.0 and ctl.plan_coordinate(1.2) == 4.0
    assert ctl.plan_coordinate(0.95) == pytest.approx((0.9025 - 0.88) / (0.93 - 0.88))
    null = AdaptiveMPCController(mpc, DT, etas, estimator=est, source="null")
    est.theta = np.array([0.8, 1, 1, 1, 1, 1, 0.0])
    assert np.array_equal(null.current(0.0)[0], X0) and ctl.current(0.0)[1] == pytest.approx(0.8)
    orc = AdaptiveMPCController(mpc, DT, etas, source="oracle", oracle=lambda s: np.array([0.9, 1, 1, 1, 1, 1, 0.0]))
    assert orc.current(10.0)[1] == pytest.approx(0.9)
    with pytest.raises(ValueError):
        AdaptiveMPCController(mpc, DT, etas)                         # an estimator or an oracle is needed
    with pytest.raises(ValueError):
        AdaptiveMPCController(mpc, DT, etas[::-1], estimator=est)
