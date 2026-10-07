"""L1 closed loop on the plans cached by scripts/run_l1.py (skipped if absent or stale)."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle
from f1race.experiments.l1 import L1Config, code_fingerprint, first_dynamic_plan, run_dyn_closed_loop

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def _cache():
    if not CACHE.exists():
        pytest.skip("run scripts/run_l1.py first")
    c = pickle.load(open(CACHE, "rb"))
    if c.get("code") != code_fingerprint():
        pytest.skip("results/L1/l1_plans.pkl was produced by different code: rerun scripts/run_l1.py")
    return c


def test_l1_plan_closed_loop(track):
    plans = _cache()["plans"]
    assert plans[1.0].lap_time < plans[0.97].lap_time < plans[0.95].lap_time < plans[0.93].lap_time
    o = plans[0.95]
    r = run_dyn_closed_loop(track, DynVehicle(), o, 0.01, L1Config().tracker)
    assert r.completed
    assert abs(r.lap_time - o.lap_time) < 0.3
    assert r.max_usage <= 0.99 + 1e-3 and r.min_margin > 0.0


@pytest.mark.slow
def test_first_dynamic_plan_converges(track):
    """The first, most fragile step of the L1 pipeline with the pipeline's settings
    (L0 on 2 m, dynamic OCP on 2.5 m; a few minutes)."""
    from f1race.planning.mincurv import min_curvature_line
    from f1race.planning.ocp_kinematic import initial_guess_from_path, solve_min_time
    from f1race.planning.qss import speed_profile

    cfg, veh = L1Config(), DynVehicle()
    alpha, mc = min_curvature_line(track, veh, margin=cfg.margin)
    q = speed_profile(mc, veh)
    idx = np.arange(0, len(track.s), 3)
    o0 = solve_min_time(track, veh, initial_guess_from_path(track, veh, idx, alpha, mc, q.v, q.a),
                        ds=cfg.ds_l0, margin=cfg.margin)
    log = []
    o1 = first_dynamic_plan(track, veh, o0, cfg.first_eta, log, ds=cfg.ds_l1, nsub=cfg.nsub,
                            margin=cfg.margin)
    assert o1.success, log
    assert o0.lap_time + 2.0 < o1.lap_time < o0.lap_time + 6.0   # local optima: 74.89 / 75.25 s
    assert np.all(np.abs(o1.delta) <= veh.delta_max + 1e-6)
