"""Control expansion phase I, candidate I-1 (docs/CX_I_protocol.md section 9.2): the speed bound per lap
(``LearningMPC(speed_bound=True)``, ``LapStore.v_ref``, ``LMPCController(eps=...)``), the sets and the tuning rule."""
import numpy as np
import pytest

from f1race.dynamics import DynVehicle


def test_v_ref_is_the_faster_lap_and_reaches_past_the_line():
    from f1race.control.lmpc import LapStore

    st = LapStore(None)
    s = np.linspace(-1.0, 4500.0, 901)                 # the unwrapped arclength, the run-out past L ~ 4318 m included
    X1, X2 = np.zeros((901, 7)), np.zeros((901, 7))
    X1[:, 0], X2[:, 0] = s, s + 1.0
    X1[:, 3] = 40.0 + 0.01 * s
    X2[:, 3] = np.where(s < 2000.0, 50.0, 30.0)
    st.add(np.arange(901) * 0.05, X1[::-1], np.zeros((901, 3)), 80.0)    # stored out of order: sorted by s inside
    st.add(np.arange(901) * 0.05, X2, np.zeros((901, 3)), 79.0)
    q = np.array([100.0, 1999.0, 3000.0, 4400.0])
    v = st.v_ref(q, 2)
    assert v == pytest.approx([50.0, 59.99, 70.0, 84.0], abs=1e-6)       # the larger of the two, past the line too
    assert st.v_ref(4400.0, 1)[0] == pytest.approx(30.0)                  # P = 1: the last lap only
    assert st.v_ref(9000.0, 1)[0] == pytest.approx(30.0)                  # constant beyond a lap's end


@pytest.fixture(scope="module")
def problems():
    from f1race.control.lmpc import LearningMPC
    from f1race.track import Track

    tr = Track.load("Spielberg")
    kw = dict(n_pts=4, N=10, dt=0.05)
    return tr, LearningMPC(tr, DynVehicle(), **kw), LearningMPC(tr, DynVehicle(), speed_bound=True, **kw)


def test_the_bound_off_keeps_phase_e_layout(problems):
    tr, off, on = problems
    N = off.N
    assert on._nlp["p"].shape[0] == off._nlp["p"].shape[0] + N
    assert on._nlp["x"].shape[0] == off._nlp["x"].shape[0] + N
    assert on._nlp["g"].shape[0] == off._nlp["g"].shape[0] + N
    assert len(off.Vi) == 0 and list(on.Vi) == list(range(off._nlp["x"].shape[0], on._nlp["x"].shape[0]))
    D, c = np.zeros((6, 4)), np.zeros(4)
    with pytest.raises(ValueError):
        off.solve(np.zeros(7), np.zeros(3), D, c, vbar=np.ones(N))
    with pytest.raises(ValueError):
        on.solve(np.zeros(7), np.zeros(3), D, c)


def _safe_set(m, tr):
    """From a straight at 30 m/s: the end states of an accelerating (A, cheaper to go) and a coasting (B) rollout."""
    from f1race.control.nmpc import FS

    i = 100 + int(np.argmin(np.abs(tr.kappa[100:600])))
    x0 = np.array([tr.s[i], 0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
    ends = []
    for f in (3000.0, 0.0):
        x = x0.copy()
        for _ in range(m.N):
            x = np.asarray(m._step(x, np.array([0.0, f, 0.0]) / np.array([1.0, FS, FS]))).ravel()
        ends.append(x[:6])
    A, B = ends
    D = np.column_stack([A, B, A + 0.01, B + 0.01])
    return x0, D, np.array([5.0, 5.5, 5.0, 5.5]), A, B


def test_an_inactive_bound_gives_the_unbounded_solution(problems):
    tr, off, on = problems
    x0, D, c, _, _ = _safe_set(off, tr)
    off.reset(); on.reset()
    u0, i0 = off.solve(x0, np.zeros(3), D, c)
    u1, i1 = on.solve(x0, np.zeros(3), D, c, vbar=np.full(on.N, 100.0))
    assert not i0["fallback"] and not i1["fallback"]
    assert i1["speed_slack"] < 1e-6 and i1["speed_near_share"] == 0.0
    X0, X1 = off.prediction()[0], on.prediction()[0]
    assert np.allclose(X0, X1, atol=1e-3)
    assert u1 == pytest.approx(u0, abs=1.0)                      # forces in N, steering rate in rad/s


def test_a_binding_bound_caps_the_predicted_speeds(problems):
    tr, off, on = problems
    x0, D, c, A, B = _safe_set(off, tr)
    vcap = 30.2
    assert A[3] > vcap + 0.5 and B[3] < vcap                     # the cheaper terminal point is faster than the cap
    off.reset(); on.reset()
    off.solve(x0, np.zeros(3), D, c)
    _, info = on.solve(x0, np.zeros(3), D, c, vbar=np.full(on.N, vcap))
    assert off.prediction()[0][:, 3].max() > vcap + 0.5         # unbounded: it takes the faster point
    assert not info["fallback"]
    assert on.prediction()[0][1:, 3].max() <= vcap + 0.02 and info["speed_slack"] < 0.02


def test_eps_and_the_bound_go_together(problems):
    from f1race.control.lmpc import LapStore, LMPCController

    _, off, on = problems
    st = LapStore(off._step)
    with pytest.raises(ValueError):
        LMPCController(off, st, 2, eps=0.04)
    with pytest.raises(ValueError):
        LMPCController(on, st, 2)
    assert LMPCController(on, st, 2, eps=0.04).eps == 0.04


def test_sets_seeds_and_kinds():
    from f1race.experiments import cx_a, cx_b, cx_c, cx_d, cx_e, cx_h, cx_i
    from f1race.experiments.l2 import sample_plants

    it = cx_i.cases("IT")
    assert len(it) == 50 and it[0]["info"] == sample_plants(DynVehicle(), 50, 383)[1][0]
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))  # noqa: E731
    old = (cx_a.cases("CXV") + cx_a.cases("CXT") + cx_b.cases("CBT") + cx_c.cases("CCT") + cx_d.cases("CDT")
           + cx_e.cases("CET") + cx_h.cases("HT"))
    assert not ({key(c) for c in it} & {key(c) for c in old})
    assert [c["info"] for c in cx_i.cases("IV")] == [c["info"] for c in cx_a.cases("CXV")]
    assert all(c["tau"] == 0.03 for c in cx_i.cases("IT_LAG30"))
    assert cx_i.cases("IT_WET85")[2]["plant"].mu_x == pytest.approx(0.85 * it[2]["plant"].mu_x)
    assert [g["eps"] for g in cx_i.EPS_GRID] == [0.02, 0.04, 0.08] and all(g["K"] == 20 for g in cx_i.EPS_GRID)
    assert cx_i.DEV_SETTING == {"K": 20, "eps": 0.04}
    nom = cx_i.cases("NOM")[0]
    with pytest.raises(ValueError):
        cx_i.learn(nom, "pb", {"K": 20})                         # pb needs eps
    with pytest.raises(ValueError):
        cx_i.learn(nom, "lmpc", {"K": 20, "eps": 0.04})          # lmpc is phase E's problem, no bound


def _rec(eps, done):
    return [{"kind": "pb", "setting": {"K": 20, "eps": eps}, "setting_key": f"K=20|eps={eps}", "case": f"IV_car{i}",
             "completed": d is not None, "lap_time": d if d is not None else float("nan")} for i, d in enumerate(done)]


def test_select_rule_over_eps():
    from f1race.experiments import cx_i

    r = cx_i.select(_rec(0.02, [76.0, None]) + _rec(0.04, [None, None]) + _rec(0.08, [77.0, None]))
    assert r["setting"]["eps"] == 0.02                           # 1 : 0 : 1, mean on the common car 76 < 77
    r = cx_i.select(_rec(0.02, [76.0]) + _rec(0.04, [76.0]) + _rec(0.08, [76.0]))
    assert r["setting"]["eps"] == 0.04                           # all tied: the development setting
    r = cx_i.select(_rec(0.02, [76.0, None]) + _rec(0.08, [None, 76.0]))
    assert r["setting"]["eps"] == 0.02                           # tied without a common car, no 0.04: the smaller eps
