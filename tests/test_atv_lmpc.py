"""Control expansion phase H (control/lmpc.py ``ATVLearningMPC``, ``ATVLMPCController``, ``LapStore.select_succ``;
docs/CX_H_protocol.md): the linearisation, the QP at a feasible candidate, the safe-set edges, the candidate."""
import numpy as np
import pytest


@pytest.fixture(scope="module")
def small():
    from f1race.control.lmpc import ATVLearningMPC
    from f1race.dynamics import DynVehicle
    from f1race.track import Track

    tr = Track.load("Spielberg")
    return ATVLearningMPC(tr, DynVehicle(), n_pts=4, N=5, dt=0.05), tr


def test_jacobian_matches_finite_differences(small):
    m, _ = small
    rng = np.random.default_rng(0)
    nz, npar = m._nlp["x"].shape[0], m._nlp["p"].shape[0]
    z = rng.standard_normal(nz) * 0.1
    z[m.Xi[:, 0]] = 100.0 + np.arange(m.N + 1)        # s
    z[m.Xi[:, 3]] = 40.0                               # v_x
    p = rng.standard_normal(npar) * 0.1
    J = np.asarray(m._jg(z, p).full())
    h = 1e-6
    for j in rng.choice(nz, 25, replace=False):
        e = np.zeros(nz)
        e[j] = h
        fd = (np.asarray(m._gf(z + e, p)).ravel() - np.asarray(m._gf(z - e, p)).ravel()) / (2 * h)
        assert np.allclose(J[:, j], fd, rtol=1e-4, atol=1e-5)


def test_qp_keeps_a_feasible_candidate_feasible(small):
    """A rollout of the nominal step along the track centre, its terminal state among the stored points: the QP's
    solution needs no slack (the candidate itself is feasible for the linearised rows)."""
    from f1race.control.nmpc import FS

    m, tr = small
    i = 300
    x = np.array([tr.s[i], 0.0, 0.0, 30.0, 0.0, 0.0, 0.0])
    U = np.tile([0.0, 2000.0, 0.0], (m.N, 1))
    X = [x]
    for k in range(m.N):
        X.append(np.asarray(m._step(X[-1], U[k] / np.array([1.0, FS, FS]))).ravel())
    X = np.array(X)
    D = np.column_stack([X[-1, :6]] + [X[-1, :6] + 0.5 * (j + 1) for j in range(m.n_pts - 1)])
    c = np.array([10.0, 11.0, 12.0, 13.0])
    u, info = m.solve_qp(X[0], U[0], D, c, None, m.candidate_vector(X, U))
    assert not info["fallback"], info
    assert info["term_slack"] < 1e-4 and info["slack"] < 1e-4 and info["limit_slack"] < 1e-4


def test_select_succ_excludes_the_last_point_and_returns_successors():
    from f1race.control.lmpc import LapStore

    st = LapStore(None)
    n = 40
    X = np.zeros((n, 7))
    X[:, 0] = np.arange(n) * 2.0
    X[:, 3] = 30.0 + np.arange(n)
    U = np.column_stack([np.zeros(n), np.arange(n) * 10.0, np.zeros(n)])
    st.add(np.arange(n) * 0.05, X, U, 1.5)
    st.add(np.arange(n) * 0.05, X + np.array([0.1, 0, 0, 0, 0, 0, 0]), U, 1.4)
    D, c, Dsucc, Usucc = st.select_succ(78.0, 5, 2)    # near the last point (s = 78)
    assert D.shape == (6, 10) and Dsucc.shape == (7, 10) and Usucc.shape == (3, 10)
    assert np.max(D[0]) < 78.0                          # the last point (s 78 / 78.1) is never selected
    assert np.allclose(Dsucc[0], D[0] + 2.0)            # the successor is the next 50 ms record
    assert np.allclose(Usucc[1], (D[3] - 30.0 + 1) * 10.0)
    assert st.fastest()["T"] == 1.4


def test_candidate_shift_and_terminal_from_lambda():
    from f1race.control.lmpc import ATVLMPCController

    class M:
        N, dt = 3, 0.05

    ctl = ATVLMPCController.__new__(ATVLMPCController)
    ctl.mpc = M()
    Xp = np.arange(4 * 7, dtype=float).reshape(4, 7)
    Up = np.arange(3 * 3, dtype=float).reshape(3, 3)
    lam = np.array([0.25, 0.75])
    Dsucc = np.array([[1.0, 3.0]] * 7)
    Usucc = np.array([[10.0, 30.0]] * 3)
    ctl.prev = (Xp, Up, lam, Dsucc, Usucc)
    Xc, Uc = ctl._candidate()
    assert np.allclose(Xc[:3], Xp[1:]) and np.allclose(Xc[3], 2.5)
    assert np.allclose(Uc[:2], Up[1:]) and np.allclose(Uc[2], 25.0)
