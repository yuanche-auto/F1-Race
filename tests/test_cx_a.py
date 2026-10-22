"""Control expansion phase A: sets, grids, the tuning rule and the report (docs/CX_A_protocol.md rev. 1)."""
import json
import pickle
import sys
from pathlib import Path

import numpy as np
import pytest

from f1race.experiments import cx_a

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_sets_are_new_draws_within_the_l2_ranges():
    v, t = cx_a.cases("CXV"), cx_a.cases("CXT")
    assert len(v) == 10 and len(t) == 25 and len(cx_a.cases("NOM")) == 1
    assert len({c["name"] for c in v + t}) == 35
    for c in v + t:
        for k, (lo, hi) in cx_a.PERTURBATION.items():
            assert lo <= c["info"][k] <= hi
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))
    assert not ({key(c) for c in v} & {key(c) for c in t})                 # tuning and test cars disjoint
    from f1race.experiments.l2 import sample_plants
    from f1race.dynamics import DynVehicle

    assert v[0]["info"] == sample_plants(DynVehicle(), 10, 301)[1][0]       # protocol section 2 seeds
    assert t[-1]["info"] == sample_plants(DynVehicle(), 25, 302)[25][0]


def test_grids():
    assert len(cx_a.MPCC_GRID) == 24 and len(cx_a.NMPC_GRID) == 6
    assert {"c": 1.0, "margin": 0.3} in cx_a.NMPC_GRID                       # L2's setting is one point
    assert {"q_c": 0.05, "q_v": 1.0, "margin": 0.3} in cx_a.MPCC_GRID
    assert cx_a.setting_key({"q_v": 1.0, "q_c": 2.0, "margin": 0.3}) == "margin=0.3,q_c=2,q_v=1"


def _recs(kind, table):
    """table: setting -> list of lap times (None = not completed), one per car."""
    out = []
    for st, laps in table:
        for i, lt in enumerate(laps):
            out.append({"kind": kind, "setting": st, "setting_key": cx_a.setting_key(st), "case": f"CXV_car{i + 1}",
                        "completed": lt is not None, "lap_time": lt if lt is not None else float("nan")})
    return out


def test_select_rule():
    a, b, c = {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}, {"q_c": 0.5, "q_v": 1.0, "margin": 0.3}, \
        {"q_c": 8.0, "q_v": 2.0, "margin": 0.6}
    # most completed wins
    r = cx_a.select(_recs("mpcc", [(a, [76.0, None, 77.0]), (b, [76.5, 76.0, 77.5]), (c, [None, None, 70.0])]), "mpcc")
    assert r["setting"] == b
    # tie on completions -> lower mean lap time on the laps both complete (cars 1 and 3)
    r = cx_a.select(_recs("mpcc", [(a, [76.0, None, 77.0]), (b, [76.5, None, 77.5])]), "mpcc")
    assert r["setting"] == a and r["table"][cx_a.setting_key(a)]["mean_lap_common"] == pytest.approx(76.5)
    # tie on both -> the smaller q_c
    r = cx_a.select(_recs("mpcc", [(a, [76.0, 77.0]), (b, [76.0, 77.0])]), "mpcc")
    assert r["setting"] == b
    # nmpc: tie -> c nearest 1, then margin 0.3
    n1, n2, n3 = {"c": 2.0, "margin": 0.3}, {"c": 1.0, "margin": 0.6}, {"c": 1.0, "margin": 0.3}
    r = cx_a.select(_recs("nmpc", [(n1, [76.0]), (n2, [76.0]), (n3, [76.0])]), "nmpc")
    assert r["setting"] == n3
    # no common completed lap among the tied settings -> straight to closeness
    r = cx_a.select(_recs("mpcc", [(a, [76.0, None]), (b, [None, 75.0])]), "mpcc")
    assert r["setting"] == b
    with pytest.raises(ValueError):                                        # unequal lap counts
        cx_a.select(_recs("mpcc", [(a, [76.0, 77.0]), (b, [76.0])]), "mpcc")
    other = _recs("mpcc", [(b, [76.0, 77.0])])
    other[1]["case"] = "CXV_car9"                                         # same count, another car (n2)
    with pytest.raises(ValueError):
        cx_a.select(_recs("mpcc", [(a, [76.0, 77.0])]) + other, "mpcc")


def test_report_tune_and_test(tmp_path, monkeypatch):
    """report_cx_a end to end on synthetic records: selected.json carries the tuning file's SHA-256; the test
    report computes H-A1 / H-A2 with Holm and lists the discordant cars."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    import report_cx_a

    tune = []
    for kind in ("nmpc", "mpcc", "mpcc_centre"):
        for st in cx_a.grid(kind):
            for r in _recs(kind, [(st, [76.0 + 0.01 * i for i in range(10)])]):
                r["job"] = f"tune|{kind}|{r['setting_key']}|{r['case']}"
                tune.append(r)
    (tmp_path / "tune.jsonl").write_text("\n".join(json.dumps(r) for r in tune) + "\n")
    report_cx_a.tune(tmp_path)
    sel = json.loads((tmp_path / "selected.json").read_text())
    import hashlib
    assert sel["tune_sha256"] == hashlib.sha256((tmp_path / "tune.jsonl").read_bytes()).hexdigest()
    assert sel["kinds"]["nmpc"]["setting"] == {"c": 1.0, "margin": 0.3}
    test = []
    rng = np.random.default_rng(1)
    for kind in cx_a.KINDS:
        st = {} if kind == "tracker" else sel["kinds"][kind]["setting"]
        for i in list(range(1, 26)) + [0]:
            name, set_ = (f"CXT_car{i}", "CXT") if i else ("NOM_car0", "NOM")
            done = not (kind == "nmpc" and 1 <= i <= 6) and not (kind == "mpcc" and i == 7)
            test.append({"job": f"test|{kind}|x|{name}", "kind": kind, "set": set_, "case": name, "setting": st,
                         "completed": done, "lap_time": 76.0 + rng.normal(0, 0.1) - (0.2 if kind == "mpcc" else 0.0),
                         "stop_reason": "lap completed" if done else "off track", "failure_s": None if done else 3800.0,
                         "distance": 4315.0 if done else 3800.0, "min_edge_margin": 0.2,
                         "bound_active_stage0_measured": 0.3, "bound_active_stages_1_N": 0.4})
    write = lambda rs: (tmp_path / "test.jsonl").write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    write(test[:-1])                                                       # a car missing -> refused (m2)
    with pytest.raises(SystemExit):
        report_cx_a.test(tmp_path)
    bad = [dict(r) for r in test]
    bad[30]["setting"] = {"c": 2.0, "margin": 0.3}                       # nmpc with another setting -> refused
    write(bad)
    with pytest.raises(SystemExit):
        report_cx_a.test(tmp_path)
    write(test + [{"job": "test|mpcc|x|CXT_car3", "error": "boom"}])     # last record an error -> refused
    with pytest.raises(SystemExit):
        report_cx_a.test(tmp_path)
    write(test)
    report_cx_a.test(tmp_path)
    res = json.loads((tmp_path / "report" / "CX_A.json").read_text())
    assert res["H-A1"][:2] == [6, 1]                                        # cars 1-6 only mpcc, car 7 only nmpc
    assert res["H-A2"]["n"] == 18 and res["holm"]["H-A2"]["rejected"]      # mpcc 0.2 s faster on 18 cars
    md = (tmp_path / "report" / "CX_A.md").read_text(encoding="utf-8")
    assert "| CXT_car7 |" in md and "off track at s = 3800 m" in md
    assert "| nmpc | 19 lap completed, 6 off track |" in md                 # stop reasons (m3)


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_bound_recorder_one_solve(track):
    """The recorder's stage-0 entry is the measured speed against v_ref(theta_0 - e_l,0); its stage entry is
    the share of predicted stages 1 ... N at the bound - recomputed here directly."""
    from f1race.experiments.l1 import dyn_reference
    from f1race.experiments.l2 import x0_from_plan
    from f1race.dynamics import DynVehicle

    o = pickle.load(open(CACHE, "rb"))["plans"][0.95]
    ctl, mpc, x0 = cx_a.build("mpcc", {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}, track, o)
    ctl(0.0, x0)
    X, _ = mpc.prediction()
    vref = cx_a._vref_fn(o, track.length)
    el = np.array([float(mpc.errors_fn(a, b, t)[1]) for a, b, t in zip(X[:, 0], X[:, 1], X[:, 7])])
    assert ctl.active_stages[-1] == pytest.approx(np.mean(X[1:, 3] >= vref(X[1:, 7] - el[1:]) - cx_a.BOUND_TOL))
    el0 = float(mpc.errors_fn(x0[0], x0[1], ctl.ctl.theta)[1])
    assert ctl.active0[-1] == bool(x0[3] >= vref(ctl.ctl.theta - el0) - cx_a.BOUND_TOL)
    solve = mpc.solve                                                      # a fallback solve (n1)

    def failing(*a, **k):
        u, info = solve(*a, **k)
        info["fallback"] = mpc.stats[-1]["fallback"] = True
        return u, info
    mpc.solve = failing
    ctl(0.05, x0)
    assert len(ctl.active0) == 2 and len(ctl.active_stages) == 1 and ctl.fallback_solves == 1


def test_ca_grid_and_secondary_sets():
    """Amendment A1's grid keeps mpcc's base; the secondary sets of protocol section 2."""
    base = {"q_c": 2.0, "q_v": 1.0, "margin": 0.3}
    g = cx_a.ca_grid(base)
    assert len(g) == 6 and all({k: s[k] for k in base} == base for s in g)
    r = cx_a.select(_recs("mpcc_ca", [(g[5], [76.0]), (g[0], [76.0])]), "mpcc_ca")    # tie -> smaller w_ca
    assert r["setting"]["w_ca"] == 0.02
    lag, cxt = cx_a.cases("CXT_LAG30"), cx_a.cases("CXT")
    assert [c["info"] for c in lag] == [c["info"] for c in cxt] and {c["tau"] for c in lag} == {0.03}
    assert [c["dist_seed"] for c in cx_a.cases("CXD")] == list(range(7400, 7430))
    assert {c["track"] for c in cx_a.cases("CXL8")} == {"Zandvoort", "SaoPaulo", "Austin", "Nuerburgring"}
    assert all(c["tau"] == 0.0 and c["dist_seed"] is None and c["track"] == "Spielberg"
               for set_ in ("CXV", "CXT", "NOM") for c in cx_a.cases(set_))


def test_a2_grid_and_selection(tmp_path):
    """Amendment A2: the soft-bound grid, ca_grid keeping the flag, and report tune --a2 taking nmpc from
    tune.jsonl and the MPCCs from tune_a2.jsonl, with both SHA-256 in selected.json."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    import hashlib
    import report_cx_a

    assert len(cx_a.MPCC_GRID_A2) == 24 and all(g["soft_bound"] == 1 for g in cx_a.MPCC_GRID_A2)
    assert all(s["soft_bound"] == 1 for s in cx_a.ca_grid({"q_c": 0.5, "q_v": 1.0, "margin": 0.6, "soft_bound": 1}))
    write = lambda name, rs: (tmp_path / name).write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    hard, soft = [], []
    for kind, grid_, out in (("nmpc", cx_a.NMPC_GRID, hard), ("mpcc", cx_a.MPCC_GRID, hard), ("mpcc_centre", cx_a.MPCC_GRID, hard),
                             ("mpcc", cx_a.MPCC_GRID_A2, soft), ("mpcc_centre", cx_a.MPCC_GRID_A2, soft)):
        for st in grid_:
            lt = 76.0 if (kind == "mpcc" and st.get("soft_bound") and st["q_c"] == 2.0 and st["q_v"] == 1.0
                          and st["margin"] == 0.3) else None
            for r in _recs(kind, [(st, [lt, None])]):
                r["job"] = f"tune|{kind}|{r['setting_key']}|{r['case']}"
                out.append(r)
    write("tune.jsonl", hard)
    write("tune_a2.jsonl", soft)
    report_cx_a.tune(tmp_path, a2=True)
    sel = json.loads((tmp_path / "selected.json").read_text())
    assert sel["kinds"]["mpcc"]["setting"] == {"q_c": 2.0, "q_v": 1.0, "margin": 0.3, "soft_bound": 1}
    assert sel["kinds"]["nmpc"]["setting"] == {"c": 1.0, "margin": 0.3}            # from tune.jsonl
    assert sel["tune_a2_sha256"] == hashlib.sha256((tmp_path / "tune_a2.jsonl").read_bytes()).hexdigest()
