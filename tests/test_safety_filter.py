"""Safety filters prepared for L7: the CBF-QP and the NMPC in filter mode; NMPC retries."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_cbf_qp_passes_safe_commands_and_corrects_unsafe_ones(track):
    from f1race.control.safety_filter import CBFQPFilter

    veh = DynVehicle()
    f = CBFQPFilter(track, veh, eta=0.95)
    s = 50.0
    x = np.array([0.0, 0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
    # in the middle of the track, straight ahead, gentle drive: unchanged
    u = np.array([0.0, 2000.0, 0.0])
    out, info = f.filter((s, 0.0, 0.0), x, u)
    assert info["ok"] and not info["intervened"] and np.allclose(out, u)
    # next to the left edge, heading towards it, steering further left: the filter steers right
    wl = float(track.interp(track.w_left, s))
    n = wl - f.b - 0.2
    u = np.array([5.0, 0.0, 0.0])                   # steering rate towards the left
    out, info = f.filter((s, n, 0.05), x, u)
    assert info["ok"] and info["intervened"] and out[0] < u[0]
    # a lateral demand beyond the friction circle is cut back
    u = np.array([30.0, 0.0, 0.0])
    out, info = f.filter((s, 0.0, 0.0), x, u)
    ax, ay = f.accel_of(x, out)
    assert ay <= 0.95 * veh.mu_y * (9.81 + veh.q_lift * 900 / veh.m) + 1e-6


@pytest.mark.slow
@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_nmpc_filter_keeps_a_safe_command(track):
    from f1race.control.nmpc import FrenetMPC
    from f1race.experiments.l2 import ref_from_plan

    veh = DynVehicle()
    o = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    mpc = FrenetMPC(track, veh, ref_from_plan(o), mode="filter", N=30, dt=0.05, eta=0.95)
    x0 = np.array([o.s[10], o.n[10], o.xi[10], o.vx[10], o.vy[10], o.r[10], o.delta[10]])
    u_des = np.array([0.0, max(o.f_drive[10], 0.0), min(o.f_brake[10], 0.0)])
    u, info = mpc.solve(x0, u_des, u_des=u_des)
    assert info["status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    assert abs(u[0] - u_des[0]) < 0.05 and abs(u[1] - u_des[1]) < 500 and abs(u[2] - u_des[2]) < 500
    with pytest.raises(ValueError):
        mpc.solve(x0, u_des)                       # the filter needs a proposal


@pytest.mark.slow
@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_retry_rollout_and_brake_fallback(track):
    from f1race.control.nmpc import BRAKE_REQUEST, FrenetMPC
    from f1race.experiments.l2 import ref_from_plan

    veh = DynVehicle()
    o = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    mpc = FrenetMPC(track, veh, ref_from_plan(o), mode="track", N=30, dt=0.05, eta=0.95, max_iter=2,
                    retry=True, fallback="brake")
    x0 = np.array([o.s[10], o.n[10], o.xi[10], o.vx[10], o.vy[10], o.r[10], o.delta[10]])
    mpc.solve(x0, np.zeros(3))                     # first solve: kept even if not converged
    # the rollout reproduces the model's own prediction
    z = mpc.rollout(mpc.z, x0)
    assert np.allclose(z[mpc.Xi[0]], x0)
    X = z[mpc.Xi]
    nxt = np.asarray(mpc._step(X[0], z[mpc.Ui[0]])).ravel()
    assert np.allclose(X[1], nxt)
    # two iterations cannot converge: retried (2 more attempts), then the brake fallback
    u, info = mpc.solve(x0 + np.array([0.5, 0, 0, 0, 0, 0, 0]), np.zeros(3))
    assert info["fallback"] and len(info["attempts"]) == 3
    assert u[1] == 0.0 and u[2] == -BRAKE_REQUEST
