"""Control expansion phase B: sets, grids, seeds, the tuning rule, the runner's guards and the report
(docs/CX_B_protocol.md)."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from f1race.experiments import cx_a, cx_b

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "results" / "L1" / "l1_plans.pkl"


def test_sets_seeds_and_grids():
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants

    t, v = cx_b.cases("CBT"), cx_b.cases("CXV")
    assert len(t) == 50 and len(v) == 10 and len(cx_b.cases("CBT_LAG30")) == 50
    assert t[0]["info"] == sample_plants(DynVehicle(), 50, 312)[1][0] and t[-1]["name"] == "CBT_car50"
    assert [c["name"] for c in v] == [c["name"] for c in cx_a.cases("CXV")]           # phase A's tuning set
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))
    old = cx_a.cases("CXV") + cx_a.cases("CXT")
    assert not ({key(c) for c in t} & {key(c) for c in old})                          # new cars
    assert all(c["tau"] == 0.03 for c in cx_b.cases("CBT_LAG30")) and all(c["tau"] == 0.0 for c in t)
    assert len(cx_b.MPPI_GRID) == 9 and cx_b.GRID_CENTRE in cx_b.MPPI_GRID
    assert [g["f_c"] for g in cx_b.lp_grid({"lam": 1.0, "c_sigma": 1.0, "K": 8, "f_c": 2.0})] == [1.0, 2.0, 4.0]
    assert cx_b.big_setting({"lam": 1.0, "c_sigma": 1.0, "K": 512, "f_c": 2.0}) == {"lam": 1.0, "c_sigma": 1.0, "K": 2048,
                                                                                    "f_c": 2.0}
    st1 = cx_b.lp_stage1(2048)                                                      # amendment B2.2
    assert len(st1) == 9 and {s_["f_c"] for s_ in st1} == {2.0} and {s_["K"] for s_ in st1} == {2048}
    assert cx_b.white_setting(2048) == {"lam": 1.0, "c_sigma": 1.0, "K": 2048} and cx_b.PRIMARY == "mppi_lp"
    assert cx_b.rng_seed(t[4], 2) == (2, 5, 2) and cx_b.rng_seed(cx_b.cases("NOM")[0], 0) == (0, 0, 0)
    assert cx_b.rng_seed(cx_b.cases("CXD")[0], 1) == (4, 7400, 1) and cx_b.rng_seed(cx_b.cases("CXL8")[3], 0)[0] == 5


def _recs(kind, table, rep=0):
    out = []
    for st, laps in table:
        for i, lt in enumerate(laps):
            out.append({"kind": kind, "setting": st, "setting_key": cx_b.setting_key(st), "case": f"CXV_car{i + 1}",
                        "replicate": rep, "completed": lt is not None, "lap_time": lt if lt is not None else float("nan")})
    return out


def test_select_rule():
    c = {"lam": 1.0, "c_sigma": 1.0, "K": 8}
    lo, hi, wide = {**c, "lam": 0.1}, {**c, "lam": 10.0}, {**c, "c_sigma": 2.0}
    assert cx_b.select(_recs("mppi", [(lo, [76.0, None]), (c, [None, None])]), "mppi")["setting"] == lo
    r = cx_b.select(_recs("mppi", [(lo, [76.0, None, 77.0]), (c, [76.5, None, 77.5])]), "mppi")
    assert r["setting"] == lo and r["table"][cx_b.setting_key(lo)]["mean_lap_common"] == pytest.approx(76.5)
    # full tie -> the smaller |log lam| first, then |log c_sigma|, then the smaller lam
    assert cx_b.select(_recs("mppi", [(lo, [76.0]), (c, [76.0]), (hi, [76.0])]), "mppi")["setting"] == c
    assert cx_b.select(_recs("mppi", [(wide, [76.0]), (lo, [76.0])]), "mppi")["setting"] == wide
    assert cx_b.select(_recs("mppi", [(wide, [76.0]), (c, [76.0])]), "mppi")["setting"] == c
    assert cx_b.select(_recs("mppi", [(lo, [76.0]), (hi, [76.0])]), "mppi")["setting"] == lo
    # replicates other than 0 are ignored; unequal laps stop it
    assert cx_b.select(_recs("mppi", [(lo, [76.0])]) + _recs("mppi", [(c, [70.0])], rep=1), "mppi")["setting"] == lo
    with pytest.raises(ValueError):
        cx_b.select(_recs("mppi", [(lo, [76.0, 77.0]), (c, [76.0])]), "mppi")
    lp = [{**c, "f_c": f} for f in (1.0, 2.0, 4.0)]                     # B2.2: f_c closest to 2 Hz, then higher
    assert cx_b.select(_recs("mppi_lp", [(s_, [76.0]) for s_ in lp]), "mppi_lp", "f_c")["setting"]["f_c"] == 2.0
    assert cx_b.select(_recs("mppi_lp", [(s_, [76.0]) for s_ in (lp[0], lp[2])]), "mppi_lp", "f_c")["setting"]["f_c"] == 4.0
    assert cx_b.select(_recs("mppi_lp", [(lp[0], [76.0]), (lp[1], [None])]), "mppi_lp", "f_c")["setting"]["f_c"] == 1.0


def test_runner_guards(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_cx_b

    monkeypatch.setattr(cx_b, "K_RT", None)
    with pytest.raises(SystemExit):
        run_cx_b.tune_jobs()
    monkeypatch.setattr(cx_b, "K_RT", 512)
    tj = run_cx_b.tune_jobs()
    assert len(tj) == 90 and {json.loads(j[5])["K"] for j in tj} == {512} and {j[1] for j in tj} == {"mppi_lp"}
    assert run_cx_b._key(tj[0]) == "tune|mppi_lp|K=512,c_sigma=0.5,f_c=2,lam=0.1|CXV_car1|r0"
    monkeypatch.setattr(run_cx_b, "OUT", tmp_path)
    with pytest.raises(SystemExit):
        run_cx_b._selected()
    sel = {"lam": 0.1, "c_sigma": 2.0, "K": 512, "f_c": 2.0}
    st = run_cx_b.settings({"kinds": {"mppi_lp": {"setting": sel}}}, {"kinds": {"mppi_lp": {"setting": {**sel, "f_c": 4.0}}}})
    assert st["mppi"] == {"lam": 1.0, "c_sigma": 1.0, "K": 512} and st["mppi_lp"]["f_c"] == 4.0     # B2.3
    assert st["mppi_big"] == {**st["mppi_lp"], "K": 2048}
    jobs = run_cx_b.test_jobs(st)
    assert len(jobs) == 51 * 8 and sum(j[1] == "mppi_lp" for j in jobs) == 153 and sum(j[1] == "mppi" for j in jobs) == 51
    assert {j[3] for j in jobs} == {c["name"] for c in cx_b.cases("CBT")} | {"NOM_car0"}


def _test_records(sel, sel_lp):
    import run_cx_b

    st = run_cx_b.settings(sel, sel_lp)
    rng = np.random.default_rng(1)
    out = []
    for kind in cx_b.KINDS:
        for rep in ((0, 1, 2) if kind == "mppi_lp" else (0,)):
            for i in list(range(1, 51)) + [0]:
                name, set_ = (f"CBT_car{i}", "CBT") if i else ("NOM_car0", "NOM")
                done = not (kind == "nmpc" and 1 <= i <= 8) and not (kind == "mppi_lp" and i == 9) \
                    and not (kind == "mppi_lp" and rep == 1 and i == 10)
                r = {"job": f"test|{kind}|x|{name}|r{rep}", "kind": kind, "set": set_, "case": name, "setting": st[kind],
                     "setting_key": cx_b.setting_key(st[kind]), "replicate": rep if kind in cx_b.MPPI_KINDS else None,
                     "completed": done, "lap_time": 76.0 + rng.normal(0, 0.1) - (0.2 if kind == "mppi_lp" else 0.0),
                     "stop_reason": "lap completed" if done else "off track", "failure_s": None if done else 2700.0,
                     "distance": 4315.0 if done else 2700.0, "min_edge_margin": 0.2, "overlap_N_p50_p90_max": [0, 0, 0]}
                if kind in cx_b.MPPI_KINDS:
                    r.update(step_ms_mean_p50_p95_p99_max=[30, 29, 35, 40, 60], steps_over_50ms=0.01,
                             first_call_ms_iters=[600, 20], no_valid_steps=0, no_penalty_free_sample_share=0.1,
                             best_sample_penalised_share=0.05, ess_p5_p50_p95=[2, 10, 40], penalised_sample_share_mean=0.3,
                             invalid_samples_total_max=[0, 0], cost_after_over_before_p5_p50_p95=[0.5, 0.9, 1.1],
                             cost_after_worse_share=0.3)
                out.append(r)
    return out


def test_report_tune_and_test(tmp_path, monkeypatch):
    """report_cx_b end to end on synthetic records: the selections carry their files' SHA-256; the test report
    computes H-B1 / H-B2 with Holm, lists the discordant cars and the replicate flips, and refuses incomplete
    or wrongly configured records."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import report_cx_b

    monkeypatch.setattr(cx_b, "K_RT", 512)
    tune = []
    for s_ in cx_b.lp_stage1(512):
        for r in _recs("mppi_lp", [(s_, [76.0 + 0.01 * i for i in range(10)])]):
            r.update(job=f"tune|mppi_lp|{r['setting_key']}|{r['case']}|r0", inputs="three", ess_p5_p50_p95=[1, 5, 9],
                     step_ms_mean_p50_p95_p99_max=[30, 29, 35, 40, 60], no_penalty_free_sample_share=0.1,
                     distance=4315.0)
            tune.append(r)
    (tmp_path / "tune.jsonl").write_text("\n".join(json.dumps(r) for r in tune) + "\n")
    report_cx_b.tune(tmp_path)
    sel = json.loads((tmp_path / "selected.json").read_text())
    assert sel["tune_sha256"] == hashlib.sha256((tmp_path / "tune.jsonl").read_bytes()).hexdigest()
    assert sel["kinds"]["mppi_lp"]["setting"] == {"lam": 1.0, "c_sigma": 1.0, "K": 512, "f_c": 2.0} and sel["K"] == 512
    base = sel["kinds"]["mppi_lp"]["setting"]
    lp = []
    for s_ in cx_b.lp_grid(base):
        laps = [76.0 + 0.01 * i for i in range(10)] if s_["f_c"] == 2.0 else [76.0 + 0.01 * i for i in range(9)] + [None]
        for r in _recs("mppi_lp", [(s_, laps)]):
            r.update(job=f"tune|mppi_lp|{r['setting_key']}|{r['case']}|r0", distance=4315.0)
            lp.append(r)
    (tmp_path / "tune_lp.jsonl").write_text("\n".join(json.dumps(r) for r in lp) + "\n")
    report_cx_b.tune_lp(tmp_path)
    sel_lp = json.loads((tmp_path / "selected_lp.json").read_text())
    assert sel_lp["kinds"]["mppi_lp"]["setting"]["f_c"] == 2.0                     # 10 of 10 against 9 of 10
    assert "repeat the first step's in 10 of 10" in (tmp_path / "report" / "tune_lp.md").read_text(encoding="utf-8")
    assert sel_lp["selected_json_sha256"] == hashlib.sha256((tmp_path / "selected.json").read_bytes()).hexdigest()
    test = _test_records(sel, sel_lp)
    write = lambda rs: (tmp_path / "test.jsonl").write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    write([r for r in test if not (r["kind"] == "mppi_lp" and r["replicate"] == 2 and r["case"] == "CBT_car3")])
    with pytest.raises(SystemExit):                                          # a replicate lap missing
        report_cx_b.test(tmp_path)
    bad = [dict(r) for r in test]
    bad[60]["setting"] = {"c": 1.0, "margin": 0.3}                          # nmpc with another setting
    write(bad)
    with pytest.raises(SystemExit):
        report_cx_b.test(tmp_path)
    write(test + [{"job": "test|mppi_lp|x|CBT_car3|r0", "error": "boom"}])  # last record an error
    with pytest.raises(SystemExit):
        report_cx_b.test(tmp_path)
    write(test)
    report_cx_b.test(tmp_path)
    res = json.loads((tmp_path / "report" / "CX_B.json").read_text())
    assert res["H-B1"][:2] == [8, 1]                                         # cars 1-8 only mppi_lp, car 9 only nmpc
    assert res["H-B2"]["n"] == 41 and res["holm"]["H-B2"]["rejected"]       # mppi_lp 0.2 s faster on 41 cars
    assert res["holm"]["H-B1"]["rejected"] and res["H-B1"][2] > 0.025       # p = 2 * P(X <= 1 | 9) = 0.039: Holm, not strict
    assert res["replicate_flips"] == ["CBT_car10"]
    md = (tmp_path / "report" / "CX_B.md").read_text(encoding="utf-8")
    assert "| CBT_car9 |" in md and "off track at s = 2700 m" in md and "12 comparisons" in md
    assert "| mppi vs mppi_lp |" in md and "| mppi_lp replicate 1 vs nmpc |" in md


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_baseline_records_come_from_phase_a_code(track):                    # the fixture provides the track data
    """cx_b.lap runs a baseline through cx_a.lap: the record equals cx_a.lap's in every field that does not
    measure time, plus the drive-and-brake overlap of the applied inputs (the tracker on the nominal car)."""
    case = cx_b.cases("NOM")[0]
    a, b = cx_a.lap(case, "tracker", {}), cx_b.lap(case, "tracker", {})
    drop = {"wall_s"}
    assert {k: v for k, v in a.items() if k not in drop} == {k: v for k, v in b.items() if k not in drop
                                                            and k in a}
    assert set(b) - set(a) == {"replicate", "overlap_N_p50_p90_max", "overlap_over_1kN_share"}
    from f1race import experiments

    assert experiments.l7.make_sim_for.__name__ == "make_sim_for" and experiments.l7.make_sim_for.__qualname__ == "make_sim_for"


def test_reproducibility_check_treats_nan_lap_times_as_equal(tmp_path, monkeypatch):
    """report_cx_b.secondary compares nmpc / mpcc laps with phase A's records field by field: a not-completed lap's
    lap time is NaN in both and must count as identical; a real difference must be listed (review n3 of packet 1)."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import report_cx_b

    rec = lambda kind, case, lt, d: {"kind": kind, "case": case, "set": case.split("_")[0], "completed": lt == lt,
                                     "lap_time": lt, "distance": d, "stop_reason": "x", "wall_s": 1.0}
    old = [dict(rec("nmpc", "CXD_dist7400", float("nan"), 3809.0), job="a"), dict(rec("mpcc", "CXL8_Austin", 90.0, 5.0), job="b")]
    new = [rec("nmpc", "CXD_dist7400", float("nan"), 3809.0), rec("mpcc", "CXL8_Austin", 90.5, 5.0)]
    pa = tmp_path / "phase_a.jsonl"
    pa.write_text("\n".join(json.dumps(r) for r in old) + "\n")
    monkeypatch.setattr(report_cx_b, "load", lambda p: [json.loads(l) for l in open(p)] if p == pa else
                        [dict(r, job=str(i), replicate=None, setting={}) for i, r in enumerate(new)])
    monkeypatch.setattr(report_cx_b, "_settings", lambda d: {})
    monkeypatch.setattr(report_cx_b, "_check", lambda recs, st, sets: None)
    monkeypatch.setattr(cx_b, "SECONDARY", ())
    report_cx_b.secondary(tmp_path, pa)
    md = (tmp_path / "report" / "CX_B_secondary.md").read_text(encoding="utf-8")
    assert "Identical in every field that does not measure time: 1 of 2 laps" in md
    assert "* mpcc CXL8_Austin: lap_time" in md and "CXD_dist7400" not in md.split("Differences")[1]
