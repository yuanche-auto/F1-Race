"""Control expansion phase G (experiments/cx_g.py, the ``comp`` option of experiments/l9; docs/CX_G_protocol.md):
sets, the predictor over the delay (exact with the exact model), the d = 0 / L_hat = 0 identity, the summaries."""
from types import SimpleNamespace

import numpy as np
import pytest


def test_sets_and_summaries():
    from f1race.experiments import cx_g

    gt = cx_g.cases("GT1") + cx_g.cases("GT2")
    gd = cx_g.cases("GD")
    assert len(gt) == 105 and len(gd) == 12 and len(cx_g.cases("NOM")) == 1
    assert [c["dist_seed"] for c in gt if c["dist_seed"] is not None] == list(range(7700, 7730))
    assert [c["dist_seed"] for c in gd if c["dist_seed"] is not None] == [7790, 7791, 7792]
    assert len({c["name"] for c in gt + gd}) == 117 and cx_g.case_by_name("GT1_car3_tau60")["tau"] == 0.06
    assert set(cx_g.PRIMARY) <= set(cx_g.ARMS) and cx_g.ARMS["rti_dc20_d40"][2]["d"] == 0.02
    calls = [{"t": 0.0, "t_apply": 0.01, "t_plan": 0.01, "x_assumed": [1.0, 0, 0, 10, 0, 0, 0]},
             {"t": 0.05, "t_apply": 0.07, "t_plan": 0.06, "x_assumed": [0.0, 0, 0.1, 10, 0, 0, 0.01]}]
    states = np.zeros((10, 7))
    states[:, 3] = 10.0
    e = cx_g.prediction_errors(calls, states)
    assert e["n"] == 2 and e["pos_p50"] == pytest.approx(0.5) and e["heading_p95"] == pytest.approx(0.095)
    a = cx_g.application_delays(calls)
    assert a["histogram_ms"] == {"10": 1, "20": 1} and a["max_ms"] == pytest.approx(20.0)


def _ctl(lag: float | None, delay: float, comp: dict):
    from f1race.dynamics import DynVehicle
    from f1race.experiments import l9
    from f1race.track import Track

    tr = Track.load("Spielberg")
    mpc = SimpleNamespace(track=tr, veh=DynVehicle(), steer_lag=lag)
    cls = l9.RealTimeLagMPCController if lag else l9.RealTimeMPCController
    return cls(mpc, l9.DT, "realtime", delay, comp={**comp, "track": tr}), tr


@pytest.mark.parametrize("lag", [None, 0.03])
def test_prediction_is_exact_with_the_exact_model(lag):
    from f1race.dynamics import DynVehicle
    from f1race.sim.dynamic import DynamicSim
    from f1race.sim.lagged import LaggedSim

    ctl, tr = _ctl(lag, 0.02, {"d": 0.02, "lhat": 0.01})
    sim = LaggedSim(tr, DynVehicle(), lag, 0.01) if lag else DynamicSim(tr, DynVehicle(), 0.01)
    i = 200
    x = np.array([tr.x[i], tr.y[i], tr.psi[i], 40.0, 0.2, 0.05, 0.02], float)
    if lag:
        x = np.concatenate([x, [0.04]])                           # actual 0.02, commanded 0.04
    us = [np.array([0.3, 2000.0, 0.0]), np.array([-0.2, 0.0, -3000.0])]
    U = np.array([[0.1, 1000.0, 0.0]] * 30)
    xs = [x]
    for u in us:
        xs.append(sim.step(xs[-1], sim.apply_limits(xs[-1], u)))
    x_lhat = sim.step(xs[-1], sim.apply_limits(xs[-1], U[0]))
    for xi in xs:                                                # the sensor buffer: three measurements
        ctl._measure(sim.observe(xi) if lag else xi)
    ctl._out.extend(us)
    ctl.active = (0.02, U)
    xm = sim.observe(xs[0]) if lag else xs[0]
    d_act = float(xs[0][6]) if lag else None
    ctl._ctx = (xm, d_act, np.zeros(8 if lag else 7))
    xfp, xp, u_prev = ctl._predict(0.02, 0.01)
    assert np.allclose(xp, x_lhat, rtol=0, atol=1e-12)
    assert np.allclose(u_prev, U[0])
    assert ctl.s_unwrapped is None                                # the measurement path's unwrapping untouched
    assert xfp.shape == (8 if lag else 7,)


def test_no_delay_no_latency_is_the_identity():
    ctl, _ = _ctl(None, 0.0, {"d": 0.0, "lhat": 0.0})
    xf = np.arange(7.0)
    ctl._measure(np.ones(7))
    ctl._ctx = (np.ones(7), None, xf)
    ctl.u = np.array([0.1, 2.0, 3.0])
    out = ctl._predict(0.0, 0.0)
    assert out[0] is xf and out[2] is ctl.u
