"""L3: disturbance models, stochastic simulator, calibration of the back-offs."""
import numpy as np

from f1race.dynamics import DynVehicle
from f1race.experiments.l3 import L3Config, calibrate
from f1race.sim.dynamic import DynamicSim
from f1race.sim.stochastic import Disturbances, FrictionField, Gusts, StochasticSim, sample_disturbances


def test_friction_field_statistics(track):
    s = np.linspace(0.0, track.length, 3000, endpoint=False)
    v = np.array([FrictionField(track.length, 0.03, 100.0, seed)(s) for seed in range(100)])
    assert abs(v.mean() - 1.0) < 0.003
    assert abs(v.std() - 0.03) < 0.003
    f = FrictionField(track.length, 0.03, 100.0, 7)
    assert np.isclose(f(0.0), f(track.length))                       # periodic on the loop
    g = (v - 1.0) / 0.03
    lag = int(round(100.0 / (s[1] - s[0])))
    rho = np.mean(g[:, :-lag] * g[:, lag:]) / np.mean(g * g)
    assert abs(rho - np.exp(-0.5)) < 0.06                              # squared-exponential, 100 m


def test_gusts_ou_statistics():
    G = Gusts(300.0, 1.0, 0.01, 3000.0, 3)
    assert abs(G.F.std() - 300.0) < 30.0
    assert abs(np.corrcoef(G.F[:-100], G.F[100:])[0, 1] - np.exp(-1.0)) < 0.08
    assert G.at(0.0) == G.F[0] and G.at(0.015) == G.F[1]


def test_seeded_and_switchable(track):
    a, b = sample_disturbances(track, 5), sample_disturbances(track, 5)
    assert np.array_equal(a.gusts.F, b.gusts.F) and a.friction(123.0) == b.friction(123.0)
    off = sample_disturbances(track, 5, sigma_mu=0.0, gust_sigma=0.0, noise=False)
    assert off.friction is None and off.gusts is None and off.noise_sigma is None


def test_without_disturbances_equals_deterministic_sim(track):
    veh = DynVehicle()
    x0 = np.array([track.x[0], track.y[0], track.heading_at(0.0), 30.0, 0.0, 0.0, 0.0])
    ctl = lambda t, x: (0.0, 2000.0, 0.0)                              # noqa: E731
    r0 = DynamicSim(track, veh, 0.01).run(ctl, x0, t_max=2.0)
    r1 = StochasticSim(track, veh, Disturbances(None, None, None, 0), 0.01).run(ctl, x0, t_max=2.0)
    assert np.allclose(r0.state, r1.state)


def test_lower_friction_saturates_earlier(track):
    """A constant 10 % friction loss raises the plant's ellipse usage by 1/0.9."""
    veh = DynVehicle()
    fr = FrictionField(track.length, 0.0, 100.0, 0)
    fr._f = np.full_like(fr._f, 0.9)
    x0 = np.array([track.x[0], track.y[0], track.heading_at(0.0), 40.0, 0.0, 0.0, 0.0])
    ctl = lambda t, x: (0.0, 0.0, -8000.0)                             # noqa: E731
    r0 = StochasticSim(track, veh, Disturbances(None, None, None, 0), 0.01).run(ctl, x0, t_max=0.5)
    r1 = StochasticSim(track, veh, Disturbances(fr, None, None, 0), 0.01).run(ctl, x0, t_max=0.5)
    assert np.allclose(r1.usage[:5], r0.usage[:5] / 0.9, rtol=1e-6)


def test_calibration_quantiles():
    cfg = L3Config(N=3)
    rng = np.random.default_rng(0)
    n = 20000
    k = np.repeat(np.arange(4), n)
    up = np.where(k < 3, 0.9, np.nan)
    err = 0.01 * (k + 1) * rng.standard_normal(4 * n)
    s = {"k": k, "u_pred": up, "u_real": up + err, "f": np.ones(4 * n), "e_n": np.abs(err) * 10}
    c = calibrate([s], cfg, 0.05)
    z = 1.6449
    assert np.allclose(c["eta"], 1.0 - z * 0.01 * np.arange(1, 4), atol=0.002)
    assert np.allclose(c["backoff"][1:], 10 * 1.96 * 0.01 * np.arange(2, 5), rtol=0.03)
    assert c["backoff"][0] == 0.0
