import numpy as np

from f1race.vehicle import G


def test_track_geometry(track):
    assert abs(track.length - 4315.7) < 5.0
    # closed loop, driven clockwise: total turning -2 pi
    turning = np.sum(np.angle(np.exp(1j * np.diff(np.concatenate([track.psi, track.psi[:1]])))))
    assert abs(turning + 2.0 * np.pi) < 1e-3
    assert track.w_left.min() > 4.0 and track.w_right.min() > 4.0


def test_projection_roundtrip(track):
    s = track.s[::97]
    n = np.linspace(-3.0, 3.0, len(s))
    p = track.point_at(s, n)
    s2, n2 = track.project(p[:, 0], p[:, 1])
    ds = (s2 - s + track.length / 2) % track.length - track.length / 2
    assert np.abs(ds).max() < 0.05
    assert np.abs(n2 - n).max() < 0.02


def test_top_speed_balance(vehicle):
    v = vehicle.top_speed()
    assert abs(vehicle.P_max / v - vehicle.resist(v * v)) < 1.0


def test_lateral_limit_consistent(vehicle):
    k = 1.0 / 60.0
    w = vehicle.v2_lateral_max(k)
    fz = vehicle.fz(w)
    assert abs(vehicle.m * k * w - vehicle.mu_y * fz) < 1e-6 * fz
    assert vehicle.fx_available(w, k) < 1e-3 * fz          # nothing left for F_x
    # downforce dominates on gentle curves
    assert np.isinf(vehicle.v2_lateral_max(1e-4))


def test_usage(vehicle):
    v, d = 40.0, 0.02
    fz = vehicle.fz(v * v)
    fy = vehicle.m * v * v * np.tan(d) / vehicle.L
    assert np.isclose(vehicle.usage(v, d, 0.0), fy / (vehicle.mu_y * fz))
    assert np.isclose(vehicle.usage(v, 0.0, vehicle.mu_x * fz), 1.0)
    assert G == 9.81
