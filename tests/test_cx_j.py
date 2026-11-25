"""Control expansion phase J: the tube around the adaptive model, the combined NLP and the sets (docs/CX_J_protocol.md
section 9.1)."""
import math
import pickle
from pathlib import Path

import numpy as np
import pytest

from f1race.dynamics import DynVehicle
from f1race.estimation.identify import HI, LO, X0

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "results" / "L1" / "l1_plans.pkl"
LIB = ROOT / "results" / "CX" / "D" / "plans" / "plans_cxd.pkl"
needs_plan = pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
needs_lib = pytest.mark.skipif(not (CACHE.exists() and LIB.exists()), reason="run scripts/cx_d_plans.py first")
W = {"n": 2.0, "xi": 20.0, "v": 0.4}


@pytest.fixture(scope="module")
def track():
    from f1race.track import Track
    return Track.load("Spielberg")


def _plan():
    return pickle.load(open(CACHE, "rb"))["plans"][0.95]


def _ref():
    from f1race.experiments.l2 import ref_from_plan
    return ref_from_plan(_plan())


def _mpc(track, veh=None, param_model=False, soft=True):
    from f1race.control.nmpc import FrenetMPC
    return FrenetMPC(track, veh or DynVehicle(), _ref(), mode="track", N=30, dt=0.05, eta=0.95, margin=0.6, weights=W,
                     param_tightening=True, param_model=param_model, soft_limits=soft)


def _prediction(track, s=1500.0):
    from f1race.control.nmpc import FS
    mpc = _mpc(track)
    o = _plan()
    x = np.array([s] + [float(np.interp(s, o.s, getattr(o, k))) for k in ("n", "xi", "vx", "vy", "r", "delta")])
    mpc.solve(x, np.zeros(3))
    return x, mpc.z[mpc.Xi].copy(), mpc.z[mpc.Ui] * np.array([1.0, FS, FS])


# ---------------------------------------------------------------- the reachable set with an initial error
def test_reach_bounds_e0_is_the_support_function_and_c_without_e0():
    from f1race.control.radaptive import reach_bounds_e0
    from f1race.control.tube import reach_bounds
    rng = np.random.default_rng(0)
    N, n = 6, 4
    Acl = 0.9 * rng.normal(size=(N, n, n)) / 2
    Wb = np.abs(rng.normal(size=(N, n)))
    C = rng.normal(size=(N + 1, n))
    b0 = reach_bounds_e0(Acl, Wb, C, np.zeros(n))
    assert np.allclose(b0, reach_bounds(Acl, Wb, C), atol=1e-14) and b0[0] == 0.0
    e0 = np.abs(rng.normal(size=n))
    b = reach_bounds_e0(Acl, Wb, C, e0)
    assert b[0] == 0.0 and np.all(b[1:] >= b0[1:])          # stage 0 is not tightened
    for k in range(1, N + 1):
        worst = 0.0                                          # the sign-matched vertex attains the bound
        for _ in range(400):
            e = e0 * rng.uniform(-1, 1, n)
            for j in range(k):
                e = Acl[j] @ e + Wb[j] * rng.uniform(-1, 1, n)
            worst = max(worst, abs(C[k] @ e))
        assert worst <= b[k] + 1e-12
        Phi = np.eye(n)
        rows = [C[k] @ np.linalg.multi_dot([np.eye(n)] + [Acl[j] for j in range(k - 1, i, -1)] + [np.eye(n)])
                if i < k - 1 else C[k] for i in range(k)]
        tot = sum(np.abs(r) @ Wb[i] for i, r in enumerate(rows))
        r0 = C[k]
        for j in range(k - 1, -1, -1):
            r0 = r0 @ Acl[j]
        assert b[k] == pytest.approx(tot + np.abs(r0) @ e0, rel=1e-10)


# ---------------------------------------------------------------- the combined NLP
@needs_plan
def test_combined_nlp_equals_the_tightened_nlp_of_the_car(track):
    """Review m4: the constraint values (distance to their bounds) and the objective at the same point."""
    import casadi as ca
    from f1race.control.nmpc import param_vehicle
    from f1race.experiments.l2 import PERTURBATION

    th = np.array([PERTURBATION[k][j] for k, j in (("mu", 0), ("m", 1), ("ClA", 0), ("CdA", 1), ("Iz", 0), ("B", 1),
                                                    ("aero_front", 1))])
    nom = DynVehicle()
    plant = param_vehicle(nom, th)
    A = _mpc(track, param_model=True)
    B = _mpc(track, veh=DynVehicle(**{f: float(getattr(plant, f)) for f in plant.__dataclass_fields__}))
    rng = np.random.default_rng(1)
    eta = np.full(30, 0.9)
    back = np.concatenate([[0.0], np.full(30, 0.08)])
    x0 = np.array([1000.0, 0.2, 0.01, 60.0, 0.1, 0.05, 0.02])
    up = np.array([0.1, 3000.0, 0.0]) / np.array([1.0, 1e4, 1e4])
    pa = np.concatenate([x0, up, eta, back, th])
    pb = np.concatenate([x0, up, eta, back])
    fA = ca.Function("a", [A._nlp["x"], A._nlp["p"]], [A._nlp["f"], A._nlp["g"]])
    fB = ca.Function("b", [B._nlp["x"], B._nlp["p"]], [B._nlp["f"], B._nlp["g"]])
    for _ in range(4):
        z = np.clip(rng.normal(size=A.lbz.shape) * 0.1, -0.5, 0.5)
        z[A.Xi[:, 3]] = 50.0 + rng.normal(size=31)
        z[A.Xi[:, 0]] = 1000.0 + np.linspace(0, 75, 31)
        z[A.Ti] = np.abs(z[A.Ti])
        ja, ga = (np.asarray(v).ravel() for v in fA(z, pa))
        jb, gb = (np.asarray(v).ravel() for v in fB(z, pb))
        assert ja[0] == pytest.approx(jb[0], rel=1e-9)
        fin_u, fin_l = np.isfinite(A.ubg), np.isfinite(A.lbg)
        assert np.array_equal(fin_u, np.isfinite(B.ubg)) and np.array_equal(fin_l, np.isfinite(B.lbg))
        assert np.allclose((ga - A.ubg)[fin_u], (gb - B.ubg)[fin_u], atol=1e-9)
        assert np.allclose((A.lbg - ga)[fin_l], (B.lbg - gb)[fin_l], atol=1e-9)


@needs_plan
def test_combined_needs_soft_limits(track):
    with pytest.raises(ValueError):
        _mpc(track, param_model=True, soft=5)


# ---------------------------------------------------------------- the tube at the nominal car equals phase C's
@needs_plan
def test_tube_at_nominal_with_c_box_is_phase_cs(track):
    from f1race.control.radaptive import AdaptiveTube
    from f1race.control.tube import PARAM_ORDER, Tube
    from f1race.experiments.l2 import PERTURBATION

    x, X, U = _prediction(track)
    tc = Tube(_mpc(track), PERTURBATION, margin=0.6).compute(X, U)
    ta = AdaptiveTube(_mpc(track, param_model=True), margin=0.6)
    lo = np.array([PERTURBATION[k][0] for k in PARAM_ORDER])
    hi = np.array([PERTURBATION[k][1] for k in PARAM_ORDER])
    r = ta.compute(X, U, X0, lo, hi)
    assert np.allclose(r["backoff"], tc["backoff"], atol=1e-9) and np.allclose(r["eta"], tc["eta"], atol=1e-9)
    assert r["capped_eta_share"] == tc["capped_eta_share"]
    e = ta.compute(X, U, X0, lo, hi, np.zeros(7), np.zeros(7))
    assert np.array_equal(e["eta"], r["eta"])
    f = ta.compute(X, U, X0, lo, hi, np.full(7, 0.01), np.full(7, 0.01))
    assert f["eta"][0] == pytest.approx(0.95) and f["backoff"][0] == 0.0      # clarification m2: stage 0 not tightened
    assert np.all(f["backoff"][1:] >= r["backoff"][1:] - 1e-12)


@needs_plan
def test_cap_box_is_a_superset_of_phase_cs(track):
    from f1race.control.radaptive import CAP, AdaptiveTube
    from f1race.control.tube import PARAM_ORDER
    from f1race.experiments.l2 import PERTURBATION

    x, X, U = _prediction(track, s=2500.0)
    ta = AdaptiveTube(_mpc(track, param_model=True), margin=0.6)
    lo = np.array([PERTURBATION[k][0] for k in PARAM_ORDER])
    hi = np.array([PERTURBATION[k][1] for k in PARAM_ORDER])
    W_c = ta.disturbance(X, U, X0, lo, hi)
    W_cap = ta.disturbance(X, U, X0, X0 - CAP, X0 + CAP)
    assert np.all(W_cap >= W_c - 1e-12)


def test_cap_is_the_largest_distance_of_the_l2_range():
    from f1race.control.radaptive import CAP
    from f1race.control.tube import PARAM_ORDER
    from f1race.experiments.l2 import PERTURBATION
    for i, k in enumerate(PARAM_ORDER):
        lo, hi = PERTURBATION[k]
        assert CAP[i] == pytest.approx(max(hi - X0[i], X0[i] - lo))
    assert np.all(X0 - CAP >= LO) and np.all(X0 + CAP <= HI)


# ---------------------------------------------------------------- the controller's box and residual terms
def _stub(**kw):
    from f1race.control.radaptive import RobustAdaptiveMPCController
    c = RobustAdaptiveMPCController.__new__(RobustAdaptiveMPCController)
    from f1race.control.radaptive import CAP, KAPPA
    c.cap, c.kappa, c.shrink, c.residuals = CAP.copy(), KAPPA, True, []
    for k, v in kw.items():
        setattr(c, k, v)
    return c


def test_box_shrinks_with_sigma_and_is_capped_and_clipped():
    class Est:
        P = np.diag([1e-6] * 7)
    c = _stub(est=Est())
    lo, hi, h = c.box(X0.copy())
    assert np.allclose(h, 2 * 1e-3) and np.allclose(hi - X0, h)
    Est.P = np.diag([1.0] * 7)
    lo, hi, h = c.box(X0.copy())
    assert np.allclose(h, c.cap)
    c.shrink = False
    Est.P = np.diag([1e-9] * 7)
    assert np.allclose(c.box(X0.copy())[2], c.cap)
    edge = np.array([LO[0] + 0.01, *X0[1:]])
    assert c.box(edge)[0][0] == LO[0]                        # clipped to the estimator's bounds


def test_quantile_needs_ten_residuals_and_uses_the_last_forty():
    c = _stub()
    c.residuals = [np.full(7, 1.0)] * 9
    assert np.all(c.quantile() == 0.0)
    c.residuals = [np.full(7, 5.0)] * 100 + [np.full(7, 1.0)] * 40
    assert np.allclose(c.quantile(), 1.0)
    c.residuals = [np.full(7, float(i)) for i in range(40)]
    assert np.allclose(c.quantile(), np.quantile(np.arange(40.0), 0.9))


@needs_lib
def test_controller_short_drive_and_residuals(track):
    """One second on the nominal car: the terms are logged, a fallback's residual is skipped."""
    from f1race.experiments import cx_j

    case = cx_j.cases("NOM_C")[0]
    sim = cx_j.make_sim(case, track)
    ctl, mpc, x0, extra = cx_j.build("ra", cx_j.DEV_SETTING, track, case, sim)
    res = sim.run(ctl, x0, t_max=1.0)
    assert len(ctl.tube_log) == len(mpc.stats) >= 19
    assert len(ctl.residuals) == len(mpc.stats) - 1
    assert np.all(ctl.tube_log[0]["q"] == 0.0) and np.allclose(ctl.tube_log[0]["h"], ctl.cap)  # P at the prior
    assert all(r["eta"][0] == 0.95 and r["b_all"][0] == 0.0 for r in ctl.tube_log)
    assert len(res.t) >= 100


# ---------------------------------------------------------------- the sets and the rule
def test_sets_and_seeds():
    from f1race.experiments import cx_j
    from f1race.experiments.l2 import sample_plants

    jt = cx_j.cases("JT")
    assert len(jt) == 50 and jt[0]["drift"] is None and jt[0]["tau"] == 0.0 and jt[0]["noise_seed"] is None
    ref = sample_plants(DynVehicle(), 50, 391)[1:]
    assert all(abs(c["plant"].m - p.m) < 1e-12 for c, (_, p) in zip(jt, ref))
    jd, jn, jl = cx_j.cases("JD"), cx_j.cases("JN"), cx_j.cases("JL")
    assert all(c["drift"] for c in jd) and [c["noise_seed"] for c in jn] == [391_000 + i for i in range(1, 51)]
    assert all(c["tau"] == 0.03 for c in jl)
    assert [c["car_key"] for c in jt] == [c["car_key"] for c in jd] == [c["car_key"] for c in jn]
    for s, n in (("JV_C", 10), ("JV_D", 10), ("JV_N", 10), ("JV_L", 10), ("DEV_C", 5), ("DEV_N", 5), ("NOM_D", 1)):
        assert len(cx_j.cases(s)) == n
    assert cx_j.cases("JV_N")[0]["noise_seed"] == 301_001 and cx_j.cases("DEV_N")[4]["noise_seed"] == 399_005
    assert cx_j.cases("NOM_N")[0]["noise_seed"] == 399_900
    assert cx_j.cases("NOM_D")[0]["drift"] == {"s_on": 1600.0, "ell": 800.0, "f_end": 0.82}
    assert cx_j.case_by_name("JL_car7")["car"] == 7 and cx_j.case_by_name("NOM_L_car0")["tau"] == 0.03
    with pytest.raises(KeyError):
        cx_j.cases("JT_C")


def test_start_fixed_drift():
    from f1race.experiments.cx_j import StartFixedDrift
    L = 4315.7403
    d = StartFixedDrift(L, 1600.0, 800.0, 0.82)
    assert d(L - 1.4e-11) == 1.0 and d(0.0) == 1.0 and d(L - 1e-3) == pytest.approx(0.82)
    assert d(2000.0) == pytest.approx(1.0 - 0.18 * 400.0 / 800.0) and d(2500.0) == pytest.approx(0.82)
    assert np.allclose(d(np.array([L - 1.4e-11, 100.0])), 1.0)


def test_select_rule():
    from f1race.experiments import cx_j

    def rec(a, b, done, t=80.0):
        s = {"rho0": a, "rho_s": b}
        return [{"kind": "ra", "setting": s, "setting_key": cx_j.setting_key(s), "case": f"c{i}", "completed": i < done,
                 "lap_time": t + 0.1 * a + b if i < done else math.nan} for i in range(4)]
    grid = [rec(g["rho0"], g["rho_s"], 2) for g in cx_j.RHO_GRID]     # all complete the same: the lowest mean lap wins
    assert cx_j.select(sum(grid, []))["setting"] == {"rho0": 0.0, "rho_s": 0.0}
    recs = []
    for g in cx_j.RHO_GRID:
        recs += rec(g["rho0"], g["rho_s"], 3 if (g["rho0"], g["rho_s"]) == (4.0, 0.2) else 2)
    assert cx_j.select(recs)["setting"] == {"rho0": 4.0, "rho_s": 0.2}
    flat = []
    for g in cx_j.RHO_GRID:                                  # all equal: the middle wins
        s = {"rho0": g["rho0"], "rho_s": g["rho_s"]}
        flat += [{"kind": "ra", "setting": s, "setting_key": cx_j.setting_key(s), "case": f"c{i}", "completed": False,
                  "lap_time": math.nan} for i in range(4)]
    assert cx_j.select(flat)["setting"] == cx_j.DEV_SETTING
    with pytest.raises(ValueError):
        cx_j.select(flat[:-1])


@needs_lib
def test_settings_of_the_ablations(track):
    from f1race.experiments import cx_j

    case = cx_j.cases("NOM_C")[0]
    for kind, shrink, rho in (("ra", True, 2.0), ("ra_fixed", False, 0.0), ("ra_fixed_res", False, 2.0),
                              ("ra_shrink", True, 0.0)):
        ctl, mpc, x0, extra = cx_j.build(kind, cx_j.DEV_SETTING, track, case, cx_j.make_sim(case, track))
        assert ctl.shrink is shrink and ctl.rho0 == rho and ctl.rho_s == (0.1 if rho else 0.0)


# ---------------------------------------------------------------- the report's hypothesis tests
def _report():
    import importlib.util
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    spec = importlib.util.spec_from_file_location("report_cx_j", ROOT / "scripts" / "report_cx_j.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_report_h_j1_and_h_j2(tmp_path):
    m = _report()

    def rec(v, t=80.0):
        return {"completed": v, "lap_time": t if v else math.nan}
    jd = {"ra": {c: rec(c <= 45) for c in range(1, 51)}, "adaptive": {c: rec(c <= 34 or c == 50) for c in range(1, 51)}}
    r = m.h_j1(jd)
    assert r["only_first"] == 11 and r["only_second"] == 1 and r["improved"] == 11 and r["worsened"] == 1
    assert r["unchanged"] == 38 and r["completed"] == [45, 35]
    from scipy.stats import binomtest
    assert r["p"] == pytest.approx(binomtest(1, 12).pvalue)
    assert all(math.isnan(v) for v in m.interval(0, 0)) and m.interval(0, 5)[0] == 0.0 and m.interval(5, 5)[1] == 1.0
    assert m.interval(5, 10) == pytest.approx([0.187, 0.813], abs=1e-3)
    jn = {"ra": {c: rec(c <= 10) for c in range(1, 51)}, "tube": {c: rec(c <= 33) for c in range(1, 51)}}
    jl = {"ra": {c: rec(True) for c in range(1, 51)}, "tube": {c: rec(c <= 44) for c in range(1, 51)}}
    h = m.h_j2(jn, jl)                                       # per car: jn counts differ for 11..33 (-1); jl +1 for 45..50
    assert h["worsened"] == 23 and h["improved"] == 6 and h["unchanged"] == 21
    assert h["laps_completed"] == [60, 77] and h["n_nonzero"] == 29
    same = {"ra": jl["ra"], "tube": jl["ra"]}                # no difference anywhere
    z = m.h_j2(same, same)
    assert z["p"] == 1.0 and z["n_nonzero"] == 0 and z["unchanged"] == 50


def test_report_test_stage_requires_full_sets(tmp_path, monkeypatch):
    m = _report()
    (tmp_path / "test.jsonl").write_text("")
    with pytest.raises(SystemExit):
        m.test(tmp_path)
