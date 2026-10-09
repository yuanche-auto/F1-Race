"""L5: fast scalar projection, RL environment (skipped without the optional RL extras)."""
import pickle
from pathlib import Path

import numpy as np
import pytest

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_point_projector_matches_project(track):
    pytest.importorskip("gymnasium")
    from f1race.rl.env import PointProjector

    pp = PointProjector(track)
    rng = np.random.default_rng(0)
    i = rng.integers(0, len(track.s), 500)
    px, py = track.x[i] + rng.uniform(-6, 6, 500), track.y[i] + rng.uniform(-6, 6, 500)
    s1, n1 = track.project(px, py)
    hint = None
    for k in range(500):
        s2, n2, _ = pp.project(px[k], py[k], None)
        ds = (s1[k] - s2 + 0.5 * track.length) % track.length - 0.5 * track.length
        assert abs(ds) < 1e-9 and abs(n1[k] - n2) < 1e-9
    for k in range(0, len(track.s), 7):                      # warm-started along the track
        s2, n2, hint = pp.project(track.x[k] + 0.4, track.y[k] - 0.3, hint)
        s3, n3 = track.project(np.array([track.x[k] + 0.4]), np.array([track.y[k] - 0.3]))
        assert abs(n2 - n3[0]) < 1e-9


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_env_with_zero_action_reproduces_the_tracker(track):
    pytest.importorskip("gymnasium")
    from f1race.control.dyn_tracking import DynTracker
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import sample_plants, x0_from_plan
    from f1race.rl.env import FastMargins, RacingEnv, evaluate_lap
    from f1race.sim.dynamic import DynamicSim

    o = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    env = RacingEnv(o, randomize_plant=False)
    info, plant = sample_plants(DynVehicle(), 3, 0)[1]               # a car the tracker spins
    r = evaluate_lap(None, env, plant=plant, t_max=60.0)
    path, ff = dyn_reference(track, DynVehicle(), o)
    ctl = DynTracker(path, ff, DynVehicle(), 0.01, k_p=0.06, x_la=12.0, k_r=0.05, kv=8.0)
    ref = DynamicSim(track, plant, 0.01).run(ctl, x0_from_plan(o, path), t_max=60.0)
    assert r["stop_reason"] == ref.stop_reason
    assert abs(r["distance"] - ref.progress[-1]) < 4.0           # checks every 50 ms vs every 10 ms
    fm, sim = FastMargins(track, DynVehicle()), DynamicSim(track, DynVehicle())
    for x in ref.state[::200]:
        a, b = fm(x), sim.edge_margin(x)
        assert abs(a[1] - b[1]) < 1e-9 and abs(a[2] - b[2]) < 1e-9
