"""Robust re-planning (planning/replan.py): the order of the steps, with a stub solver."""
from types import SimpleNamespace

import numpy as np

from f1race.dynamics import DynVehicle
from f1race.planning.replan import blend_vehicle, replan_robust


def plan(tag="w"):
    z = np.zeros(4)
    return SimpleNamespace(s=np.arange(4.0), n=z, xi=z, vx=z + 50, vy=z, r=z, delta=z, f_drive=z, f_brake=z,
                           tag=tag)


def stub(ok):
    """A solver that succeeds when ok(car, eta) is true; it records its calls."""
    calls = []

    def solve(track, car, guess, eta, max_iter):
        calls.append((car, float(eta)))
        good = ok(car, eta)
        o = plan()
        o.success, o.status, o.iterations, o.lap_time = good, "ok" if good else "fail", 1, 80.0
        o.car, o.eta = car, eta
        return o
    return solve, calls


WARM = [("nominal plan", plan("nominal")), ("ideal plan", plan("ideal"))]


def test_direct_success_is_unchanged():
    solve, calls = stub(lambda car, eta: True)
    o, log = replan_robust(None, DynVehicle(), WARM, 0.95, solve=solve)
    assert o is not None and len(log) == 1 and log[0]["step"] == "direct" and log[0]["warm"] == "nominal plan"


def test_eta_continuation_with_halving():
    # the direct solves fail; the ideal problem solves, and from there a step in eta
    # succeeds only if it is at most 0.01 (the planned steps are 0.0167: each is halved)
    state = {"last": None}

    def ok(car, eta):
        good = eta >= 1.0 or (state["last"] is not None and state["last"] - eta <= 0.0101)
        if good:
            state["last"] = eta
        return good
    solve, calls = stub(ok)
    o, log = replan_robust(None, DynVehicle(), WARM, 0.95, solve=solve, max_solves=30)
    assert [e["step"] for e in log[:3]] == ["direct", "direct", "eta continuation"] and log[2]["eta"] == 1.0
    assert o is not None and abs(o.eta - 0.95) < 1e-12
    assert sum(e["status"] == "fail" for e in log[3:]) == 3          # each of the three steps halved once


def test_parameter_continuation_path():
    nom = DynVehicle()
    target = blend_vehicle(nom, DynVehicle(Iz=1500.0), 1.0)
    assert target.Iz == 1500.0 and blend_vehicle(nom, target, 0.5).Iz == 1250.0
    # only the parameter continuation succeeds
    solve, calls = stub(lambda car, eta: eta < 1.0 and car.Iz < 1500.0 or (car.Iz == 1500.0 and len(calls) > 6))
    o, log = replan_robust(None, target, WARM, 0.95, nominal=nom, solve=solve, max_solves=30)
    steps = [e["step"] for e in log]
    assert "parameter continuation" in steps and o is not None and o.car.Iz == 1500.0


def test_gives_up_within_the_cap():
    solve, calls = stub(lambda car, eta: False)
    o, log = replan_robust(None, DynVehicle(Iz=1200.0), WARM, 0.95, nominal=DynVehicle(), solve=solve, max_solves=9)
    assert o is None and len(log) == 9
