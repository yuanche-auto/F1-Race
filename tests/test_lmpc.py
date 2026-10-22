"""Control expansion phase E: the learning MPC's store (safe-set selection through the line, the regression and its
fallback), the NLP's terminal condition, and the sets (docs/CX_E_protocol.md)."""
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"
needs_plan = pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")


@pytest.fixture(scope="module")
def track():
    from f1race.track import Track
    return Track.load("Spielberg")


def _lap(L, v=50.0, dt=0.05, extra=250.0, n=0.5):
    """A synthetic stored lap: constant speed along the centre line, from s = 0 to L + extra."""
    t = np.arange(0.0, (L + extra) / v, dt)
    X = np.zeros((len(t), 7))
    X[:, 0], X[:, 1], X[:, 3] = v * t, n, v
    U = np.zeros((len(t), 3))
    return t, X, U, L / v


def test_store_selects_by_arclength_through_the_line():
    from f1race.control.lmpc import LapStore

    L = 4000.0
    store = LapStore(lambda x, u: x)
    store.add(*_lap(L, 50.0))
    store.add(*_lap(L, 60.0, n=-0.5))
    D, c, ids = store.select(1000.0, K=5, P=2)
    assert D.shape == (6, 10) and np.all(np.abs(D[0] - 1000.0) <= 5 * 3.0)
    assert set(ids) == {0, 1} and np.allclose(c[:5], 80.0 - D[0, :5] / 50.0) and np.allclose(c[5:], L / 60 - D[0, 5:] / 60.0)
    D, c, _ = store.select(L + 100.0, K=5, P=2)                   # past the line: stored points past it, negative c
    assert np.all(D[0] > L) and np.all(c < 0)
    one = LapStore(lambda x, u: x)
    one.add(*_lap(L, 50.0))
    D1, _, ids1 = one.select(500.0, K=4, P=2)                      # one stored lap fills both slots
    assert D1.shape == (6, 8) and set(ids1) == {0}


def test_regression_learns_an_affine_residual_and_falls_back_far_from_data():
    from f1race.control.lmpc import NZ, LapStore, regressor

    G_true = np.zeros((3, NZ))
    G_true[0, 0], G_true[1, 2], G_true[2, 7] = -0.001, 0.2, -0.002
    store = LapStore(lambda x, u: np.asarray(x, float), M=200, bandwidth=3.0)   # nominal step: the identity
    t = np.arange(0.0, 20.0, 0.05)
    U = np.column_stack([0.1 * np.sin(t), 2e3 * (1 + np.cos(t)), -1e3 * (1 + np.sin(0.5 * t))])
    X = np.zeros((len(t), 7))
    X[0, 3:6] = (50.0, 0.1, 0.2)
    X[:, 6] = 0.05 * np.sin(0.4 * t)
    for k in range(len(t) - 1):                                    # the data: the residual is G_true z exactly
        X[k + 1, 3:6] = X[k, 3:6] + G_true @ regressor(X[k:k + 1], U[k:k + 1])[0]
    store.add(t, X, U, 20.0)
    Zq = regressor(X[100:110], U[100:110])
    G, fb = store.regress(Zq)
    assert not fb.any()                                            # the residual is reproduced (the regressors are locally
    assert np.allclose(np.einsum("nij,nj->ni", G, Zq), Zq @ G_true.T, atol=1e-4)   # collinear: compare predictions)
    far = regressor(X[:1] + np.array([0, 0, 0, 500.0, 0, 0, 0]), U[:1])
    G2, fb2 = store.regress(far)
    assert fb2.all() and np.all(G2 == 0)


@needs_plan
def test_nlp_terminal_condition_and_simplex(track):
    """A stored lap of the nominal car driven by the NMPC: from one of its states, with its own continuation N steps later
    as the safe set, the LMPC's terminal condition holds and lambda is on the simplex."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    import cx_e_dev
    from f1race.control.lmpc import LapStore, LearningMPC

    trk, t, X, U, m = cx_e_dev._stored_nmpc_lap(__import__("f1race.experiments.cx_e", fromlist=["x"]).cases("NOM")[0], 0.95)
    lmpc = LearningMPC(trk, DynVehicle(), n_pts=10, N=30, dt=0.05)
    store = LapStore(lmpc._step)
    store.add(t, X, U, m["lap_time"])
    k = 300
    D, c, _ = store.select(X[k + 30, 0], 5, 2)
    u, info = lmpc.solve(X[k], U[k - 1], D, c)
    _, _, lam = lmpc.prediction()
    assert info["status"] in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    assert info["term_slack"] < 1e-3 and abs(lam.sum() - 1) < 1e-6 and lam.min() > -1e-8


def test_sets_and_seeds():
    from f1race.experiments import cx_a, cx_b, cx_c, cx_d, cx_e
    from f1race.experiments.l2 import sample_plants

    cet = cx_e.cases("CET")
    assert len(cet) == 50 and cet[0]["info"] == sample_plants(DynVehicle(), 50, 342)[1][0]
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))
    old = cx_a.cases("CXV") + cx_a.cases("CXT") + cx_b.cases("CBT") + cx_c.cases("CCT") + cx_d.cases("CDT")
    assert not ({key(c) for c in cet} & {key(c) for c in old})
    assert [c["info"] for c in cx_e.cases("CEV")] == [c["info"] for c in cx_a.cases("CXV")]
    assert all(c["tau"] == 0.03 for c in cx_e.cases("CET_LAG30"))
    assert cx_e.cases("CET_WET85")[2]["plant"].mu_x == pytest.approx(0.85 * cet[2]["plant"].mu_x)
    assert [g["K"] for g in cx_e.K_GRID] == [10, 20, 40] and cx_e.INIT_ETAS == (0.73, 0.79) and cx_e.N_LEARN == 10


def _rec(K, done):
    return [{"kind": "lmpc", "setting": {"K": K}, "setting_key": f"K={K}", "case": f"CEV_car{i}",
             "completed": d is not None, "lap_time": d if d is not None else float("nan")} for i, d in enumerate(done)]


def test_select_rule():
    from f1race.experiments import cx_e

    r = cx_e.select(_rec(10, [76.0, None]) + _rec(20, [None, None]) + _rec(40, [77.0, None]))
    assert r["setting"] == {"K": 10}                              # 1 : 0 : 1, mean on the common car 76 < 77
    r = cx_e.select(_rec(10, [76.0]) + _rec(20, [76.0]) + _rec(40, [76.0]))
    assert r["setting"] == {"K": 20}                              # all tied: K = 20 (E.2's)
    r = cx_e.select(_rec(10, [76.0, None]) + _rec(40, [None, 76.0]))
    assert r["setting"] == {"K": 10}                              # tied without a common car: the smaller K
    with pytest.raises(ValueError):
        cx_e.select(_rec(10, [76.0]) + _rec(20, [76.0, 77.0]))
