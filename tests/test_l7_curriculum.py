"""L7 Part C training environment (rl/curriculum.py): lag distribution, failure starts."""
import json
import pickle
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("gymnasium")
CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_lag_drawn_per_episode_and_failure_starts(tmp_path, track):
    from f1race.rl.curriculum import P_NO_LAG, TAU_RANGE, L7TrainEnv
    from f1race.sim.lagged import LaggedSim

    plan = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    env = L7TrainEnv(plan, randomize_plant=True, history=8, seed=1)
    taus = []
    for _ in range(60):
        env.reset()
        taus.append(env.sim.tau if isinstance(env.sim, LaggedSim) else 0.0)
    taus = np.array(taus)
    assert 0.15 < np.mean(taus == 0.0) < 0.55                     # P_NO_LAG = 1/3
    assert np.all((taus == 0.0) | ((taus >= TAU_RANGE[0]) & (taus <= TAU_RANGE[1])))
    assert P_NO_LAG == pytest.approx(1 / 3)
    # with disturbances: never a lag
    envd = L7TrainEnv(plan, randomize_plant=True, disturbances=True, history=8, seed=2)
    for _ in range(5):
        envd.reset()
        assert not isinstance(envd.sim, LaggedSim)
    # failure starts: always (p = 1) 50-150 m before the pool position
    pool = tmp_path / "pool.json"
    pool.write_text(json.dumps([2000.0]))
    envf = L7TrainEnv(plan, pool_file=str(pool), p_fail_start=1.0, history=8, seed=3)
    for _ in range(10):
        envf.reset()
        kind, s0 = envf.last_start
        assert kind == "failure" and 1850.0 - 1.0 <= s0 <= 1950.0 + 1.0
        assert abs(envf.s_prev - s0) < 1.5                       # the car starts there (path spacing)
    # step reports the lap position
    _, _, _, _, info = envf.step(np.zeros(2))
    assert "s" in info and abs(info["s"] - envf.s_prev) < 1e-9
    # group P: no pool, uniform starts
    pool.write_text("[]")
    envf.reset()
    assert envf.last_start[0] == "uniform"
