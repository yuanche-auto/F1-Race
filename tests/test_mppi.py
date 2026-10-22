"""Control expansion, phase B: MPPI (control/mppi.py) - CX-B protocol sections 3.1-3.5, development
checks 9.1-9.3."""
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"
W = {"n": 2.0, "xi": 20.0, "v": 0.4}            # nmpc's CXV choice c = 2 (phase A), margin 0.6
needs_plan = pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")


def _plan():
    return pickle.load(open(CACHE, "rb"))["plans"][0.95]


def _ref():
    from f1race.experiments.l2 import ref_from_plan

    return ref_from_plan(_plan())


def _mppi(track, **kw):
    from f1race.control.mppi import MPPI

    return MPPI(track, DynVehicle(), _ref(), eta=0.95, margin=0.6, weights=W, **kw)


def _nmpc(track):
    from f1race.control.nmpc import FrenetMPC

    return FrenetMPC(track, DynVehicle(), _ref(), mode="track", N=30, dt=0.05, eta=0.95, margin=0.6, weights=W)


def _x0(s=100.0):
    o = _plan()
    return np.array([s, float(np.interp(s, o.s, o.n)), 0.01, float(np.interp(s, o.s, o.vx)), 0.0, 0.0, 0.0])


def _states(track, rng, M):
    return np.column_stack([rng.uniform(0, track.length, M), rng.uniform(-3, 3, M), rng.uniform(-0.1, 0.1, M),
                            rng.uniform(20, 85, M), rng.uniform(-1, 1, M), rng.uniform(-0.5, 0.5, M),
                            rng.uniform(-0.2, 0.2, M)])


@needs_plan
def test_model_matches_frenet_mpc(track):
    """Section 9.1: the batched RK4 step is FrenetMPC's CasADi step (10^-9)."""
    from f1race.control.nmpc import FS

    mpc, mp = _nmpc(track), _mppi(track, K=8)
    rng = np.random.default_rng(0)
    X = _states(track, rng, 100)
    U = np.column_stack([rng.uniform(-1, 1, 100), rng.uniform(0, 15e3, 100), rng.uniform(-30e3, 0, 100)])
    xn, _ = mp.step(X, U)
    ref = np.array([np.asarray(mpc._step(X[k], U[k] / np.array([1.0, FS, FS]))).ravel() for k in range(100)])
    assert np.max(np.abs(xn - ref) / np.maximum(np.abs(ref), 1.0)) < 1e-9


@needs_plan
@pytest.mark.parametrize("inputs", ["net", "three"])
def test_cost_is_the_nlp_objective_with_eliminated_slacks(track, inputs):
    """Section 3.1 / 9.2: for the inputs of an IPOPT solution and for perturbed (penalised) samples, the states
    of the rollout with the eliminated slacks are feasible for the NLP, every positive slack is tight, and the
    NLP's objective there equals MPPI's cost."""
    import casadi as ca

    from f1race.control.nmpc import FS

    mpc, mp = _nmpc(track), _mppi(track, K=8, inputs=inputs)
    x0, u_prev = _x0(), np.array([0.0, 3000.0, 0.0])
    mpc.solve(x0, u_prev)
    Uz = mpc.z[mpc.Ui] * np.array([1.0, FS, FS])
    V = (Uz if inputs == "three" else np.stack([Uz[:, 0], Uz[:, 1] + Uz[:, 2]], axis=1))[None]
    rng = np.random.default_rng(1)
    V = np.concatenate([V, V + rng.normal(size=(4, 30, mp.nu)) * mp.sigma * np.array([3.0] + [2.5] * (mp.nu - 1))])
    r = mp.rollout(x0, V, u_prev, detail=True)
    J = ca.Function("J", [mpc._nlp["x"], mpc._nlp["p"]], [mpc._nlp["f"]])
    G = ca.Function("G", [mpc._nlp["x"], mpc._nlp["p"]], [mpc._nlp["g"]])
    p = np.concatenate([x0, u_prev / np.array([1.0, FS, FS])])
    checked = 0
    for k in range(len(V)):
        if not r["valid"][k]:
            continue
        z = mpc.z.copy()
        z[mpc.Xi], z[mpc.Ui] = r["X"][k], r["V3"][k] / np.array([1.0, FS, FS])
        z[mpc.Si], z[mpc.Ti] = r["sigma"][k], r["tau"][k]
        g = np.asarray(G(z, p)).ravel()
        assert np.max(mpc.lbg - g) < 1e-8 and np.max(g - mpc.ubg) < 1e-8
        assert np.max(mpc.lbz - z) < 1e-8 and np.max(z - mpc.ubz) < 1e-8
        assert float(J(z, p)) == pytest.approx(r["cost"][k], rel=1e-9)
        for idx in np.flatnonzero(z[np.concatenate([mpc.Si.ravel(), mpc.Ti.ravel()])] > 1e-6)[:5]:
            zz = z.copy()
            j = np.concatenate([mpc.Si.ravel(), mpc.Ti.ravel()])[idx]
            zz[j] -= 1e-6
            gg = np.asarray(G(zz, p)).ravel()
            assert max(np.max(mpc.lbg - gg), np.max(gg - mpc.ubg)) > 1e-9      # tight: less slack is infeasible
        checked += 1
    assert checked >= 3 and r["cost"][0] < np.inf and np.any(r["track_pen"] + r["limit_pen"] > 0)


@needs_plan
def test_clamp_keeps_the_hard_limits(track):
    """Section 3.1: clamped inputs meet the NMPC's hard limits (steering rate and angle, power, axle forces);
    inputs inside them are not changed."""
    from f1race.dynamics import tyre_forces

    mp, veh = _mppi(track, K=8), DynVehicle()
    rng = np.random.default_rng(2)
    X = _states(track, rng, 2000)
    X[:200, 6] = rng.choice([-1, 1], 200) * rng.uniform(0.39, 0.4, 200)       # near the angle limit
    V = np.column_stack([rng.normal(0, 2, 2000), rng.normal(0, 4e4, 2000)])
    V3 = mp.clamp(X, V)
    assert np.all(np.abs(V3[:, 0]) <= veh.delta_rate_max + 1e-12)
    assert np.all(np.abs(X[:, 6] + mp.dt * V3[:, 0]) <= veh.delta_max + 1e-12)
    assert np.all(V3[:, 1] >= 0) and np.all(V3[:, 2] <= 0) and np.all(V3[:, 1] * V3[:, 2] == 0)
    assert np.all(V3[:, 1] * X[:, 3] <= veh.P_max * (1 + 1e-12))
    F = tyre_forces(veh, X[:, 3], X[:, 4], X[:, 5], X[:, 6], V3[:, 1], V3[:, 2])
    for ax in ("f", "r"):
        assert np.all(np.abs(F[f"fx{ax}"] / (veh.mu_x * F[f"fz{ax}"])) <= 0.95 + 1e-9)
    inside = np.column_stack([rng.uniform(-0.05, 0.05, 2000), rng.uniform(-2000, 2000, 2000)])
    X[:, 6] = rng.uniform(-0.2, 0.2, 2000)
    V3 = mp.clamp(X, inside)
    assert np.allclose(V3[:, 0], inside[:, 0]) and np.allclose(V3[:, 1] + V3[:, 2], inside[:, 1])
    # three inputs (amendment B1): the same limits; the force pair keeps its direction; inside: unchanged
    W3 = np.column_stack([rng.normal(0, 2, 2000), np.abs(rng.normal(0, 3e4, 2000)), -np.abs(rng.normal(0, 4e4, 2000))])
    X[:, 6] = rng.uniform(-0.4, 0.4, 2000)
    C = mp.clamp(X, W3)
    assert np.all(C[:, 1] >= 0) and np.all(C[:, 2] <= 0) and np.all(C[:, 1] * X[:, 3] <= veh.P_max * (1 + 1e-12))
    F = tyre_forces(veh, X[:, 3], X[:, 4], X[:, 5], X[:, 6], C[:, 1], C[:, 2])
    for ax in ("f", "r"):
        assert np.all(np.abs(F[f"fx{ax}"] / (veh.mu_x * F[f"fz{ax}"])) <= 0.95 + 1e-9)
    pw = W3[:, 1] * X[:, 3] <= veh.P_max                      # not clipped by the power: scaled pair
    assert np.allclose(C[pw, 1] * W3[pw, 2], C[pw, 2] * W3[pw, 1], rtol=1e-9, atol=1e-3)
    assert np.mean(C[:, 1] * C[:, 2] < 0) > 0.5                  # drive and brake at once are possible
    small = np.column_stack([inside[:, 0], rng.uniform(0, 1500, 2000), rng.uniform(-1500, 0, 2000)])
    assert np.allclose(mp.clamp(X, small)[:, 1:], small[:, 1:])


@needs_plan
@pytest.mark.parametrize("inputs", ["net", "three"])
def test_iteration_rules(track, inputs):
    """Section 3.2 / 9.3: K0 = max(1, floor(alpha K)); lambda -> 0 picks the best sample; samples that all equal
    U leave it unchanged; the weights sum to one (U is a convex combination)."""
    assert [_mppi(track, K=k).K0 for k in (256, 512, 1024, 2048, 4096, 8)] == [2, 5, 10, 20, 40, 1]
    x0, u_prev = _x0(), np.zeros(3)
    rng = np.random.default_rng(3)
    mp = _mppi(track, K=16, lam=1e-12, inputs=inputs)
    U0 = np.column_stack([rng.uniform(-0.05, 0.05, 30), rng.uniform(-2000, 2000, 30)])
    if inputs == "three":
        U0 = np.column_stack([U0[:, 0], np.maximum(U0[:, 1], 0) + 300.0, np.minimum(U0[:, 1], 0) - 300.0])
    E = rng.normal(size=(16, 30, mp.nu)) * mp.sigma
    mp.U, mp.noise = U0.copy(), (lambda: E)
    V = U0[None] + E
    V[16 - mp.K0:] = E[16 - mp.K0:]
    r = mp.rollout(x0, V, u_prev)
    k = int(np.argmin(r["cost"]))
    rec = mp.iterate(x0, u_prev)
    best = r["V3"][k] if inputs == "three" else np.stack([r["V3"][k, :, 0], r["V3"][k, :, 1] + r["V3"][k, :, 2]], axis=1)
    assert np.allclose(mp.U, best)
    assert rec["ess"] == pytest.approx(1.0) and not rec["no_valid"]
    mp = _mppi(track, K=4, lam=1.0, inputs=inputs)
    mp.K0, mp.U, mp.noise = 0, U0.copy(), (lambda: np.zeros((4, 30, mp.nu)))
    mp.iterate(x0, u_prev)
    assert np.allclose(mp.U, U0, rtol=0, atol=1e-9)
    mp = _mppi(track, K=64, lam=1.0, inputs=inputs)
    mp.U = U0.copy()
    r0 = mp.rng.bit_generator.state
    E = mp.noise()
    mp.rng.bit_generator.state = r0
    V = U0[None] + E
    V[64 - mp.K0:] = E[64 - mp.K0:]
    r = mp.rollout(x0, V, u_prev)
    Vc = r["V3"] if inputs == "three" else np.stack([r["V3"][..., 0], r["V3"][..., 1] + r["V3"][..., 2]], axis=-1)
    mp.iterate(x0, u_prev)
    assert np.all(mp.U >= Vc.min(axis=0) - 1e-9) and np.all(mp.U <= Vc.max(axis=0) + 1e-9)


@needs_plan
@pytest.mark.parametrize("inputs", ["net", "three"])
def test_solve_shift_seed_and_no_valid_sample(track, inputs):
    """Section 3.2: apply u_0 of the updated sequence and shift it (last entry 0); the same seed gives the same
    inputs; clarification m1a: without a valid sample U is only shifted and its u_0 applied."""
    x0, u_prev = _x0(), np.zeros(3)
    a, b, c = (_mppi(track, K=32, seed=(2, 5, 0), inputs=inputs), _mppi(track, K=32, seed=(2, 5, 0), inputs=inputs),
               _mppi(track, K=32, seed=(2, 5, 1), inputs=inputs))
    ua = [a.solve(x0, u_prev)[0] for _ in range(3)]
    ub = [b.solve(x0, u_prev)[0] for _ in range(3)]
    uc = [c.solve(x0, u_prev)[0] for _ in range(3)]
    assert all(np.array_equal(p, q) for p, q in zip(ua, ub)) and not all(np.array_equal(p, q) for p, q in zip(ua, uc))
    assert a.stats[0]["iters"] == 20 and a.stats[1]["iters"] == 1
    assert np.array_equal(a.U, np.vstack([a.U_last[1:], np.zeros((1, a.nu))]))
    assert np.allclose(ua[-1], a.clamp(x0[None], a.U_last[:1])[0])
    slow = x0.copy()
    slow[3] = 1.0                                       # no sample reaches vx >= 2 m/s within 50 ms
    U_before = a.U.copy()
    u, info = a.solve(slow, u_prev)
    assert info["no_valid"] and info["status"] == "no valid sample" and info["valid"] == 0
    assert np.array_equal(a.U_last, U_before) and np.allclose(u, a.clamp(slow[None], U_before[:1])[0])


@needs_plan
def test_lowpass_noise(track):
    """Section 3.5: the filtered noise has the stationary standard deviation sigma, a transient at the first
    stages (clarification n2), and is correlated along the stages; mppi's noise is white."""
    lp, wh = _mppi(track, K=20000, lowpass=(1.0, 2)), _mppi(track, K=20000)
    E, Ew = lp.noise() / lp.sigma, wh.noise() / wh.sigma
    assert np.allclose(E[:, 29].std(axis=0), 1.0, atol=0.03) and np.all(E[:, 0].std(axis=0) < 0.2)
    corr = lambda A: np.mean([np.corrcoef(A[:, 20, j], A[:, 21, j])[0, 1] for j in range(2)])
    assert corr(E) > 0.8 and abs(corr(Ew)) < 0.05


@needs_plan
@pytest.mark.parametrize("inputs", ["net", "three"])
def test_closed_loop_smoke(track, inputs):
    """MPPI behind the NMPC's controller interface for 1 s on the nominal car."""
    from f1race.control.nmpc import MPCController
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.sim.dynamic import DynamicSim

    o, veh = _plan(), DynVehicle()
    path, _ = dyn_reference(track, veh, o)
    mp = _mppi(track, K=64, inputs=inputs)
    res = DynamicSim(track, veh, 0.01).run(MPCController(mp, 0.01), x0_from_plan(o, path), t_max=1.0)
    assert res.stop_reason == "time limit" and len(mp.stats) == 20
    assert all(np.isfinite(s["time"]) and np.isfinite(s["cost_after"]) for s in mp.stats)
