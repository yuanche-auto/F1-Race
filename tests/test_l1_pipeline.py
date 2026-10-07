"""L1 closed loop on the plans cached by scripts/run_l1.py (skipped if absent)."""
import pickle
from pathlib import Path

import pytest

from f1race.dynamics import DynVehicle
from f1race.experiments.l1 import L1Config, run_dyn_closed_loop

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_l1_plan_closed_loop(track):
    plans = pickle.load(open(CACHE, "rb"))["plans"]
    assert plans[1.0].lap_time < plans[0.97].lap_time < plans[0.95].lap_time < plans[0.93].lap_time
    o = plans[0.95]
    r = run_dyn_closed_loop(track, DynVehicle(), o, 0.01, L1Config().tracker)
    assert r.completed
    assert abs(r.lap_time - o.lap_time) < 0.3
    assert r.max_usage <= 1.0 and r.min_margin > 0.0


@pytest.mark.slow
def test_dynamic_ocp_coarse_converges(track):
    from f1race.planning.ocp_dynamic import initial_guess_from_kinematic, solve_min_time_dynamic
    from f1race.planning.ocp_kinematic import initial_guess_from_path, solve_min_time
    from f1race.planning.mincurv import min_curvature_line
    from f1race.planning.qss import speed_profile
    import numpy as np

    veh = DynVehicle()
    alpha, mc = min_curvature_line(track, veh, margin=0.3)
    q = speed_profile(mc, veh)
    idx = np.arange(0, len(track.s), 3)
    o0 = solve_min_time(track, veh, initial_guess_from_path(track, veh, idx, alpha, mc, q.v, q.a), ds=4.0)
    o1 = solve_min_time_dynamic(track, veh, initial_guess_from_kinematic(track, veh, o0), ds=5.0, eta=0.97)
    assert o1.success
    assert o1.lap_time > o0.lap_time          # the dynamic car cannot be faster
    assert np.all(np.abs(o1.delta) <= veh.delta_max + 1e-6)
