import numpy as np
import pytest

from f1race.dynamics import DynVehicle, ellipse_usage, saturate, tyre_forces
from f1race.sim.dynamic import DynamicSim


@pytest.fixture(scope="module")
def dveh():
    return DynVehicle()


def test_tyre_peak(dveh):
    """The simplified magic formula peaks at mu_y Fz at alpha_peak."""
    a = np.linspace(0.0, 0.3, 3001)
    F = tyre_forces(dveh, 30.0, 0.0, 0.0, a, 0.0, 0.0)
    i = int(np.argmax(F["fyf"]))
    assert abs(a[i] - dveh.alpha_peak) < 2e-4
    assert np.isclose(F["fyf"][i], dveh.mu_y * F["fzf"], rtol=1e-6)


def test_load_transfer_and_split(dveh):
    F0 = tyre_forces(dveh, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Fb = tyre_forces(dveh, 40.0, 0.0, 0.0, 0.0, 0.0, -20000.0)
    assert Fb["fzf"] > F0["fzf"] and Fb["fzr"] < F0["fzr"]          # braking loads the front
    assert np.isclose(Fb["fzf"] + Fb["fzr"], F0["fzf"] + F0["fzr"])  # total load unchanged
    assert np.isclose(Fb["fxf"], dveh.brake_front * -20000.0)
    Fd = tyre_forces(dveh, 40.0, 0.0, 0.0, 0.0, 5000.0, 0.0)
    assert Fd["fxf"] == 0.0 and Fd["fxr"] == 5000.0                   # rear-wheel drive
    assert np.isclose(dveh.lr / dveh.L, 0.45)                         # 45 % on the front axle
    F1 = tyre_forces(dveh, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0)
    ax = -dveh.c_rr * 9.81                                            # rolling resistance only
    fzf = dveh.m * 9.81 * dveh.lr / dveh.L - dveh.m * ax * dveh.h_cg / dveh.L
    assert np.isclose(F1["fzf"], fzf, rtol=1e-6)


def test_saturation_scales_onto_ellipse(dveh):
    F = tyre_forces(dveh, 30.0, 0.0, 0.0, 0.08, 0.0, -30000.0)
    uf, ur = ellipse_usage(dveh, F)
    assert uf > 1.0
    G, uf0, _ = saturate(dveh, F)
    uf1, _ = ellipse_usage(dveh, G)
    assert np.isclose(uf1, 1.0) and np.isclose(uf0, uf)
    assert np.isclose(G["fyf"] / G["fxf"], F["fyf"] / F["fxf"])        # direction kept


def test_low_speed_steady_state_matches_kinematics(track, dveh):
    """At low speed and small steering the slip angles are small and the dynamic
    model's steady yaw rate approaches the kinematic one, v tan(delta) / L."""
    sim = DynamicSim(track, dveh, dt=0.005)
    v, delta = 8.0, 0.05
    x = np.array([0.0, 0.0, 0.0, v, 0.0, 0.0, delta])
    for _ in range(4000):                                   # 20 s, hold the speed
        f = dveh.m * 2.0 * (v - x[3]) + dveh.resist(x[3] ** 2)
        x = sim.step(x, np.array([0.0, max(f, 0.0), min(f, 0.0)]))
    r_kin = x[3] * np.tan(delta) / dveh.L
    assert abs(x[5] - r_kin) / r_kin < 0.03


def test_quasi_steady_guess_balances_cornering(dveh):
    """On a steady corner the quasi-steady guess has the lateral force and (almost) no yaw
    moment that the turn needs, and keeps the rear axle on the kinematic course."""
    from types import SimpleNamespace

    from f1race.planning.ocp_dynamic import initial_guess_quasi_steady

    v, R = np.array([30.0, 45.0]), 120.0
    d_kin = np.full(2, np.arctan(dveh.L / R))
    o0 = SimpleNamespace(s=np.zeros(2), n=np.zeros(2), xi=np.zeros(2), v=v, delta=d_kin,
                         fx=dveh.q_drag * v**2)
    g = initial_guess_quasi_steady(None, dveh, o0)
    F = tyre_forces(dveh, g["vx"], g["vy"], g["r"], g["delta"], g["f_drive"], g["f_brake"])
    lat = F["fyr"] + F["fyf"] * np.cos(g["delta"]) + F["fxf"] * np.sin(g["delta"])
    yaw = dveh.lf * F["fyf"] * np.cos(g["delta"]) - dveh.lr * F["fyr"]
    assert np.allclose(lat, dveh.m * g["vx"] * g["r"], rtol=0.01)
    assert np.all(np.abs(yaw) < 0.01 * dveh.lr * np.abs(F["fyr"]))
    assert np.all(np.abs(F["ar"]) < dveh.alpha_peak) and np.all(np.abs(F["af"]) < dveh.alpha_peak)
    assert np.allclose(g["xi"] - F["ar"], o0.xi)                      # rear-axle course unchanged
