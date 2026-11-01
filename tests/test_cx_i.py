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


# Revision 1, section 10.7 r1: the reproduction check's pieces


def test_v_c_rule_and_the_arms():
    from f1race.experiments import cx_i
    from f1race.track import Track

    assert cx_i.v_c_rule(Track.load("Spielberg"), DynVehicle()) == cx_i.V_C == 8.0
    assert set(cx_i.REPRO) == {"R1", "R2", "R3"}
    for arm, (kind, solver, s) in cx_i.REPRO.items():
        assert kind == "lmpc_nominal" and s["K"] == 20 and s["consecutive"] and s["n_learn"] == 40
        assert "eps" not in s                                        # no speed bound (section 10.3)
        assert (s.get("start") == "centre" and s["v_c"] == 8.0) == (arm != "R2")
        assert solver == ("atv" if arm == "R3" else "nlp")


def test_centre_start_needs_consecutive_laps():
    from f1race.experiments import cx_e

    nom = cx_e.cases("NOM")[0]
    with pytest.raises(ValueError):
        cx_e.learn(nom, "lmpc_nominal", {"K": 20, "start": "centre", "v_c": 8.0})
    with pytest.raises(ValueError):
        cx_e.learn(nom, "lmpc_nominal", {"K": 20, "start": "grid", "consecutive": True})


def test_time_limits():
    from f1race.experiments import cx_e

    assert cx_e.learn_time_limit([539.0, 541.0]) == pytest.approx(1.25 * 541.0)
    assert cx_e.learn_time_limit([86.9, 83.1]) == 121.0                  # R2: at least phase E's limit


def _straight_lap(track, s0, v, length):
    """A synthetic result along the centre line from arclength s0 at speed v (10 ms steps): t, progress, Cartesian state."""
    from types import SimpleNamespace

    p = np.arange(0.0, length, v * 0.01)
    s = np.mod(s0 + p, track.length)
    xy = track.point_at(s)
    X = np.column_stack([xy, track.heading_at(s), np.full(len(p), v), np.zeros((len(p), 3))])
    return SimpleNamespace(t=p / v, progress=p, state=X, completed=True, stop_reason="lap completed",
                           margin=np.ones((len(p), 2)), usage=np.zeros((len(p), 2)))


def test_handover_and_lap_timing():
    from f1race.experiments import cx_e
    from f1race.track import Track

    tr = Track.load("Spielberg")
    res = _straight_lap(tr, tr.length - 0.3, 20.0, tr.length + 300.0)
    rec_t = list(np.arange(0.0, res.t[-1], 0.05))
    rec_u = [np.array([0.0, 100.0 * i, 0.0]) for i in range(len(rec_t))]
    k, x, u = cx_e._handover(res, tr, rec_t, rec_u)
    assert res.progress[k] < tr.length <= res.progress[k + 1]           # the last 10 ms step before the line
    assert np.array_equal(x, res.state[k]) and x is not res.state[k]
    j = int(np.floor(res.t[k] / 0.05 + 1e-9))
    assert u[1] == pytest.approx(100.0 * j)                              # the command in force at that step
    m = cx_e._lap_metrics(res, tr, None, t_limit=1e4)
    assert m["completed"] and m["lap_time"] == pytest.approx(tr.length / 20.0, abs=1e-6)   # one track length
    assert not cx_e._lap_metrics(res, tr, None, t_limit=200.0)["completed"]             # over its time limit


def test_runout_check():
    from f1race.experiments import cx_e
    from f1race.track import Track

    tr = Track.load("Spielberg")
    prev = _straight_lap(tr, 0.0, 20.0, tr.length + 300.0)
    k = int(np.flatnonzero(prev.progress < tr.length)[-1])
    s_k = float(cx_e._frenet(tr, prev.state[k][None])[0, 0])
    same = _straight_lap(tr, s_k, 20.0, 300.0)
    c = cx_e._runout_check(prev, k, same, tr)
    assert c["dt"] == pytest.approx(0.0, abs=1e-6) and np.allclose(c["dstate_s_n_xi_vx_vy_r_delta"], 0.0, atol=1e-3)
    faster = _straight_lap(tr, s_k, 25.0, 300.0)
    c = cx_e._runout_check(prev, k, faster, tr)
    assert c["dt"] == pytest.approx(250.0 / 25.0 - 250.0 / 20.0, abs=1e-6)
    assert c["dstate_s_n_xi_vx_vy_r_delta"][3] == pytest.approx(5.0)
    assert cx_e._runout_check(prev, k, _straight_lap(tr, s_k, 20.0, 100.0), tr) is None   # lap j + 1 short of 250 m


def _seq(times, started=True, n=40):
    laps = [{"kind": "init", "completed": True, "lap_time": 540.0}] * 2
    laps += [{"kind": "learn", "completed": t is not None, "lap_time": t if t is not None else float("nan")}
             for t in times + [None] * (n - len(times))]
    return {"started": started, "laps": laps}


def test_readings():
    from f1race.experiments import cx_i

    r = lambda times, **kw: cx_i.reading(_seq(times, **kw))["reading"]     # noqa: E731
    assert r([200.0] * 30 + [78.0, 77.9, 77.8, 77.8, 77.7, 77.8, 77.7, 77.6, 77.6, 77.6]) == "a"
    assert r([200.0] * 35 + [78.0, 76.0, 79.0, 76.0, 79.0]) == "e"          # near, the last five 3.9 % apart
    assert r([200.0] * 40) == "d"
    assert r([200.0, 100.0, 78.6]) == "b"                                     # ends after a lap <= 78.69 s
    assert r([200.0, 100.0, 84.0]) == "c1"                                    # best <= 84.04 s, none near
    assert r([200.0, 100.0, 84.1]) == "c2"
    assert r([]) == "c2"                                                      # the first learning lap fails
    assert r([], started=False) == "init failed"
