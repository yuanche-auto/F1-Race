import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle
from f1race.experiments.l2 import PERTURBATION, sample_plants, summarize

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_sample_plants_deterministic_and_in_range():
    nom = DynVehicle()
    a = sample_plants(nom, 5, seed=3)
    b = sample_plants(nom, 5, seed=3)
    assert a[0][0] == {"nominal": True} and a[0][1] == nom
    assert [x[0] for x in a] == [x[0] for x in b]
    for f, v in a[1:]:
        for k, (lo, hi) in PERTURBATION.items():
            assert lo <= f[k] <= hi
        assert np.isclose(v.mu_x / nom.mu_x, f["mu"]) and np.isclose(v.mu_y / nom.mu_y, f["mu"])
        assert np.isclose(v.aero_front, nom.aero_front + f["aero_front"])


def test_summarize_counts():
    rs = [{"controller": "c", "plant": {"nominal": True}, "completed": True, "lap_time": 75.0,
           "stop_reason": "lap completed", "max_usage": 0.99, "min_edge_margin": 0.2, "solve_ms_mean": None},
          {"controller": "c", "plant": {"mu": 0.95}, "completed": False, "lap_time": float("nan"),
           "stop_reason": "spin", "max_usage": 1.02, "min_edge_margin": 0.1, "solve_ms_mean": None}]
    s = summarize(rs)["c"]
    assert s["runs"] == 2 and s["completed"] == 1 and s["lap_nominal_plant"] == 75.0
    assert s["failures"] == ["spin"]


@pytest.mark.slow
@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_nmpc_tracks_plan_for_a_few_seconds(track):
    from f1race.control.nmpc import FrenetMPC, MPCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import ref_from_plan, x0_from_plan
    from f1race.sim.dynamic import DynamicSim

    veh = DynVehicle()
    o = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    mpc = FrenetMPC(track, veh, ref_from_plan(o), mode="track", N=30, dt=0.05, eta=0.95)
    ctl = MPCController(mpc, 0.01)
    path, _ = dyn_reference(track, veh, o)
    r = DynamicSim(track, veh, 0.01).run(ctl, x0_from_plan(o, path), t_max=8.0)
    assert r.stop_reason == "time limit"            # no spin / off track in 8 s (through turn 1)
    assert r.max_usage <= 1.0 and r.min_margin > 0.0
    assert not any(s["fallback"] for s in mpc.stats)
