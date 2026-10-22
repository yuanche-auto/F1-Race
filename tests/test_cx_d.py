"""Control expansion phase D: sets, seeds, the drift, the tuning rule, the runner's jobs and guards and the report
(docs/CX_D_protocol.md)."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from f1race.experiments import cx_a, cx_b, cx_c, cx_d

ROOT = Path(__file__).resolve().parents[1]


def test_sets_seeds_and_conditions():
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants

    cdt, cdd = cx_d.cases("CDT"), cx_d.cases("CDD")
    assert len(cdt) == len(cdd) == 50 and len(cx_d.cases("CDV")) == 20 and len(cx_d.cases("DEV_D")) == 5
    assert cdt[0]["info"] == sample_plants(DynVehicle(), 50, 332)[1][0]
    assert [c["info"] for c in cdd] == [c["info"] for c in cdt]                   # the same cars (review m2)
    assert [c["car_key"] for c in cdd] == [c["car_key"] for c in cdt]
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))
    old = cx_a.cases("CXV") + cx_a.cases("CXT") + cx_b.cases("CBT") + cx_c.cases("CCT") + cx_c.cases("CXDEV")
    assert not ({key(c) for c in cdt} & {key(c) for c in old})
    assert [c["info"] for c in cx_d.cases("CDV_D")] == [c["info"] for c in cx_a.cases("CXV")]
    assert [c["info"] for c in cx_d.cases("DEV_C")] == [c["info"] for c in cx_c.cases("CXDEV")]
    assert all(c["drift"] is None for c in cdt) and all(c["drift"] for c in cdd)
    assert cdd[0]["drift"] == cx_d.draw_drifts(50, 333)[0] and cx_d.cases("CDV_D")[0]["drift"] == cx_d.draw_drifts(10, 334)[0]
    noise = cx_d.cases("CDT_NOISE")
    assert [c["noise_seed"] for c in noise] == list(range(332_001, 332_051)) and all(c["dist_seed"] is None for c in noise)
    wet = cx_d.cases("CDT_WET85")
    assert wet[3]["plant"].mu_x == pytest.approx(0.85 * cdt[3]["plant"].mu_x) and cx_d.true_theta(wet[3])[0] == \
        pytest.approx(0.85 * cdt[3]["info"]["mu"])
    assert all(c["tau"] == 0.03 for c in cx_d.cases("CDT_LAG30"))
    assert [c["patch_seed"] for c in cx_d.cases("CDP")] == list(range(30))
    assert [c["name"] for c in cx_d.cases("CXD")] == [c["name"] for c in cx_a.cases("CXD")]
    with pytest.raises(KeyError):
        cx_d.cases("CXL8")


def test_drift_field_and_draws():
    f = cx_d.DriftField(4000.0, 1000.0, 500.0, 0.8)
    assert f(999.0) == 1.0 and f(1250.0) == pytest.approx(0.9) and f(1500.0) == pytest.approx(0.8) and f(3999.0) == 0.8
    assert f(1250.0 + 4000.0) == pytest.approx(0.9)                               # periodic in s
    d = np.array([[x["s_on"], x["ell"], x["f_end"]] for x in cx_d.draw_drifts(2000, 7)])
    assert d[:, 0].min() >= 600 and d[:, 0].max() <= 2600 and d[:, 1].min() >= 100 and d[:, 1].max() <= 1500
    assert d[:, 2].min() >= 0.75 and d[:, 2].max() <= 0.90
    s = cx_d.drift_summary("CDD")
    assert s["n"] == 50 and 0.0 <= s["share_drift_ends_before_2700m"] <= 1.0 and s["cars_mu_f_end_below_0.70"] >= 0


def _recs(kind, table):
    out = []
    for st, laps in table:
        for i, lt in enumerate(laps):
            out.append({"kind": kind, "setting": st, "setting_key": cx_d.setting_key(st), "case": f"CDV_C_car{i + 1}",
                        "set": "CDV_C", "completed": lt is not None, "lap_time": lt if lt is not None else float("nan")})
    return out


def test_select_rule():
    lo, mid, hi = cx_d.ADAPTIVE_GRID
    assert cx_d.select(_recs("adaptive", [(lo, [76.0, None]), (mid, [None, None]), (hi, [None, 77.0])]),
                       "adaptive")["setting"] == lo                       # lo, hi tied, no common lap: equally far, smaller
    r = cx_d.select(_recs("adaptive", [(lo, [76.0, 77.0]), (mid, [76.5, None]), (hi, [76.4, 77.5])]), "adaptive")
    assert r["setting"] == lo and r["table"]["lam_mu=0.99"]["mean_lap_common"] == pytest.approx(76.5)
    assert cx_d.select(_recs("adaptive_bank", [(g, [76.0]) for g in cx_d.BANK_GRID]), "adaptive_bank")["setting"] == \
        {"W": 0.2}
    assert cx_d.select(_recs("adaptive", [(lo, [76.0]), (hi, [76.0])]), "adaptive")["setting"] == lo
    with pytest.raises(ValueError):
        cx_d.select(_recs("adaptive", [(lo, [76.0, 77.0]), (mid, [76.0])]), "adaptive")


def test_runner_jobs_and_guards(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_cx_d

    tj = run_cx_d.tune_jobs()
    assert len(tj) == 120 and {j[1] for j in tj} == {"adaptive", "adaptive_bank"}
    assert run_cx_d._key(tj[0]) == "tune|adaptive|lam_mu=0.99|CDV_C_car1"
    monkeypatch.setattr(run_cx_d, "OUT", tmp_path)
    with pytest.raises(SystemExit):
        run_cx_d._selected()
    st = run_cx_d.settings({"kinds": {"adaptive": {"setting": {"lam_mu": 0.999}}, "adaptive_bank": {"setting": {"W": 0.5}}}})
    assert st["nmpc_param"] == {"lam_mu": 0.999} and st["adaptive_bank"] == {"W": 0.5} and st["nmpc"] == {}
    jobs = run_cx_d.test_jobs(st)
    assert len(jobs) == 7 * 101 and {j[3] for j in jobs if j[1] == "identified"} == set(run_cx_d._names("CDT", "NOM", "CDD"))
    sec = run_cx_d.secondary_jobs(st)
    assert len(sec) == 4 * 210 and {j[1] for j in sec} == set(cx_d.SECONDARY_KINDS)
    for c in ("CDV_C_car3", "CDT_NOISE_car7", "CDP_patch4", "CXD_dist7401", "NOM_car0", "DEV_D_car2"):
        assert run_cx_d._case(c)["name"] == c


def _lap_rec(kind, case, set_, done, lap, st, adaptive):
    r = {"job": f"test|{kind}|x|{case}", "kind": kind, "set": set_, "case": case, "setting": st, "completed": done,
         "lap_time": lap if done else float("nan"), "stop_reason": "lap completed" if done else "off track",
         "failure_s": None if done else 2900.0, "distance": 4315.0 if done else 2900.0, "min_edge_margin": 0.2}
    if adaptive:
        r.update(mu_abs_err_p50_p95=[0.001, 0.01], mu_err_end=0.0, factor_err_end=[0.0] * 7,
                 plan_eta_p5_p50_p95=[0.9, 0.93, 0.95], too_fast_share=0.05, settle_from_start_s=3.0,
                 estimator={"one_step_rms_scaled": 0.0, "clipped_share": 0.1, "update_us_mean": 80.0})
        if set_ == "CDD":
            r.update(drift={"s_on": 1000.0}, drift_settle_delay_s=1.5,
                     mu_abs_err_before_during_after_p50_p95=[[0.0, 0.01], [0.01, 0.03], [0.0, 0.01]])
    return r


def test_report_tune_and_test(tmp_path):
    """report_cx_d end to end on synthetic records: selected.json carries the tuning file's SHA-256; the test report
    computes H-D1 / H-D2 with Holm, the gap to the oracle and the secondary comparisons; incomplete or wrongly configured
    records are refused."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import report_cx_d
    import run_cx_d

    tune = []
    for kind, grid in cx_d.GRIDS.items():
        for st in grid:
            for c in cx_d.cases("CDV"):
                tune.append({"job": f"tune|{kind}|{cx_d.setting_key(st)}|{c['name']}", "kind": kind, "setting": st,
                             "setting_key": cx_d.setting_key(st), "case": c["name"], "set": c["set"], "completed": True,
                             "lap_time": 77.0 + (0.1 if st != grid[1] else 0.0)})
    (tmp_path / "tune.jsonl").write_text("\n".join(json.dumps(r) for r in tune) + "\n")
    report_cx_d.tune(tmp_path)
    sel = json.loads((tmp_path / "selected.json").read_text())
    assert sel["tune_sha256"] == hashlib.sha256((tmp_path / "tune.jsonl").read_bytes()).hexdigest()
    assert sel["kinds"]["adaptive"]["setting"] == {"lam_mu": 0.995} and sel["kinds"]["adaptive_bank"]["setting"] == {"W": 0.2}
    st = run_cx_d.settings(sel)
    rng = np.random.default_rng(1)
    test = []
    for kind in cx_d.KINDS:
        for c in cx_d.cases("CDT") + cx_d.cases("NOM") + cx_d.cases("CDD"):
            i = c["car"]
            done = not (kind in ("nmpc", "nmpc_param", "tube", "identified") and 1 <= i <= 8) \
                and not (kind == "nmpc" and c["set"] == "CDD" and i <= 30)
            test.append(_lap_rec(kind, c["name"], c["set"], done, 77.0 + rng.normal(0, 0.1), st[kind],
                                 kind in cx_d.ADAPTIVE_KINDS))
    write = lambda rs: (tmp_path / "test.jsonl").write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    write(test[:-1])
    with pytest.raises(SystemExit):
        report_cx_d.test(tmp_path)
    bad = [dict(r) for r in test]
    bad[0]["setting"] = {"c": 1.0}
    write(bad)
    with pytest.raises(SystemExit):
        report_cx_d.test(tmp_path)
    write(test)
    report_cx_d.test(tmp_path)
    res = json.loads((tmp_path / "report" / "CX_D.json").read_text())
    assert res["primary"]["H-D1"]["mcnemar"][:2] == [8, 0] and res["primary"]["H-D2"]["mcnemar"][:2] == [30, 0]
    assert res["holm"]["H-D2"]["rejected"] and res["oracle_gap"]["CDT"]["mcnemar"][:2] == [0, 0]
    assert len(res["secondary"]) == 16                                           # 7 pairs + adaptive-nmpc, x 2 sets
    md = (tmp_path / "report" / "CX_D.md").read_text(encoding="utf-8")
    assert "| CDT_car5 |" in md and "not independent" in md and "Noise floor on CDT" in md


def test_lap_position_and_the_drift_start():
    """Packet 4: the start state projects to s = L - 1e-11. The drift field (s mod L) gives f_end there - the first
    step's artefact; the start-fixed field of the check gives 1; the onset diagnostics use the lap position."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import cx_d_dev

    L = 4315.74
    q = cx_d.lap_position([L - 1e-11, L + 0.76, L + 2000.0, 2 * L - 1.0], L)
    assert q[0] == pytest.approx(0.0, abs=1e-9) and np.allclose(np.diff(q), [0.76, 1999.24, L - 2001.0])
    assert np.allclose(cx_d.lap_position([0.5, 1.5], L), [0.5, 1.5])
    f, g = cx_d.DriftField(L, 1000.0, 500.0, 0.8), cx_d_dev._StartFixDrift(L, 1000.0, 500.0, 0.8)
    assert f(L - 1e-11) == pytest.approx(0.8) and g(L - 1e-11) == 1.0          # the artefact and its fix
    for s in (0.0, 999.0, 1250.0, 1500.0, L - 10.0):
        assert g(s) == f(s)


def test_drift_onset_diagnostics_from_the_onset_not_the_lap_start():
    from types import SimpleNamespace

    from f1race.track import Track

    track = Track.load("Spielberg")
    L = track.length
    case = {"info": {"mu": 1.0}, "wet": None, "noise_seed": None,
            "drift": {"s_on": 1000.0, "ell": 500.0, "f_end": 0.8}}
    field = cx_d.DriftField(L, 1000.0, 500.0, 0.8)
    t = np.arange(0.0, 40.0, 0.05)
    s = (L - 1e-11) + 60.0 * t                                     # unwrapped, as the controller's trace
    mu_hat = np.array([float(field(q)) for q in s]) + np.where(t > 30.0, 0.0, 0.03)   # the estimate settles at 30 s
    th = np.tile([1.0, 1, 1, 1, 1, 1, 0.0], (len(t), 1))
    th[:, 0] = mu_hat
    ctl = SimpleNamespace(trace=[(ti, si, 2.0, mi, *row) for ti, si, mi, row in zip(t, s, mu_hat, th)], est=None)
    sim = SimpleNamespace(dist=SimpleNamespace(friction=field))
    m = cx_d.adaptation_metrics(case, ctl, sim, np.array([0.88, 0.93, 0.95]), track)
    t_on = t[np.flatnonzero(s - (L - 1e-11) >= 1000.0)[0]]
    assert m["drift_onset_t"] == pytest.approx(t_on) and t_on > 16.0                     # not the lap start
    assert m["drift_settle_delay_s"] == pytest.approx(t[np.flatnonzero(t > 30.0)[0]] - t_on)
