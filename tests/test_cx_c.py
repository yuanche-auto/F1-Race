"""Control expansion phase C: sets, seeds, the tuning rule, the runner's guards and the report (docs/CX_C_protocol.md)."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from f1race.experiments import cx_a, cx_b, cx_c

ROOT = Path(__file__).resolve().parents[1]


def test_sets_seeds_and_settings():
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants

    t, dev = cx_c.cases("CCT"), cx_c.cases("CXDEV")
    assert len(t) == 50 and len(dev) == 5 and len(cx_c.cases("CCT_LAG30")) == 50
    assert t[0]["info"] == sample_plants(DynVehicle(), 50, 322)[1][0] and dev[-1]["info"] == sample_plants(DynVehicle(), 5, 399)[5][0]
    key = lambda c: tuple(round(c["info"][k], 12) for k in sorted(cx_a.PERTURBATION))
    old = cx_a.cases("CXV") + cx_a.cases("CXT") + cx_b.cases("CBT")
    assert not ({key(c) for c in t + dev} & {key(c) for c in old}) and not ({key(c) for c in t} & {key(c) for c in dev})
    assert [c["name"] for c in cx_c.cases("CXV")] == [c["name"] for c in cx_a.cases("CXV")]
    assert all(c["tau"] == 0.03 for c in cx_c.cases("CCT_LAG30"))
    assert cx_c.fixed_setting("nmpc") == {"c": 2.0, "margin": 0.6}
    assert cx_c.fixed_setting("nmpc_reserve") == {"c": 2.0, "margin": 0.6, "eta": 0.88}
    assert [g["c_R"] for g in cx_c.TUBE_GRID] == [0.1, 1.0, 10.0]


def _recs(kind, table):
    out = []
    for st, laps in table:
        for i, lt in enumerate(laps):
            out.append({"kind": kind, "setting": st, "setting_key": cx_c.setting_key(st), "case": f"CXV_car{i + 1}",
                        "completed": lt is not None, "lap_time": lt if lt is not None else float("nan")})
    return out


def test_select_rule():
    lo, c1, hi = {"c_R": 0.1}, {"c_R": 1.0}, {"c_R": 10.0}
    assert cx_c.select(_recs("tube", [(lo, [76.0, None]), (c1, [None, None]), (hi, [None, 77.0])]), "tube")["setting"] == lo
    r = cx_c.select(_recs("tube", [(lo, [76.0, None, 77.0]), (hi, [76.5, None, 77.5])]), "tube")
    assert r["setting"] == lo and r["table"]["c_R=0.1"]["mean_lap_common"] == pytest.approx(76.5)
    assert cx_c.select(_recs("tube", [(lo, [76.0]), (c1, [76.0]), (hi, [76.0])]), "tube")["setting"] == c1
    assert cx_c.select(_recs("tube", [(lo, [76.0]), (hi, [76.0])]), "tube")["setting"] == lo     # the narrower tube
    with pytest.raises(ValueError):
        cx_c.select(_recs("tube", [(lo, [76.0, 77.0]), (c1, [76.0])]), "tube")


def test_runner_jobs_and_guards(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_cx_c

    tj = run_cx_c.tune_jobs()
    assert len(tj) == 60 and {j[1] for j in tj} == {"tube", "tube_track"}
    assert run_cx_c._key(tj[0]) == "tune|tube|c_R=0.1|CXV_car1"
    monkeypatch.setattr(run_cx_c, "OUT", tmp_path)
    with pytest.raises(SystemExit):
        run_cx_c._selected()
    st = run_cx_c.settings({"kinds": {"tube": {"setting": {"c_R": 1.0}}, "tube_track": {"setting": {"c_R": 0.1}}}})
    jobs = run_cx_c.test_jobs(st)
    assert len(jobs) == 51 * 6 and {j[3] for j in jobs} == {c["name"] for c in cx_c.cases("CCT")} | {"NOM_car0"}
    assert st["nmpc_reserve"]["eta"] == 0.88 and st["tube_track"] == {"c_R": 0.1}
    assert st["nmpc_param"] == {"c_R": 1.0, "null": 1} and sum(j[1] == "nmpc_param" for j in jobs) == 51   # amendment C1
    assert len(run_cx_c.secondary_jobs(st)) == 5 * (50 + 30 + 4)                 # the secondary conditions: five kinds


def test_runner_runs_nmpc_param_on_the_tube_path(monkeypatch):
    """Amendment C1: the runner runs nmpc_param as cx_c.lap(case, "tube", {"c_R": 1, "null": 1}) and relabels the record."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import run_cx_c

    seen = []
    monkeypatch.setattr(cx_c, "lap", lambda case, kind, setting: seen.append((kind, setting)) or {"kind": kind, "x": 1})
    rec = run_cx_c.job(("test", "nmpc_param", "c_R=1,null=1", "NOM_car0", json.dumps(cx_c.NMPC_PARAM_SETTING)))
    assert seen == [("tube", {"c_R": 1.0, "null": 1})] and rec["kind"] == "nmpc_param"
    rec = run_cx_c.job(("test", "tube", "c_R=1", "NOM_car0", json.dumps({"c_R": 1.0})))
    assert seen[-1] == ("tube", {"c_R": 1.0}) and rec["kind"] == "tube"


def test_report_tune_and_test(tmp_path):
    """report_cx_c end to end on synthetic records: selected.json carries the tuning file's SHA-256; the test report
    computes H-C1 / H-C2 with Holm and lists the discordant cars; incomplete or wrongly configured records are refused."""
    sys.path.insert(0, str(ROOT / "scripts"))
    import report_cx_c
    import run_cx_c

    tune = []
    for kind in cx_c.TUBE_KINDS:
        for st in cx_c.TUBE_GRID:
            for r in _recs(kind, [(st, [76.0 + 0.01 * i for i in range(10)])]):
                r.update(job=f"tune|{kind}|{r['setting_key']}|{r['case']}", capped_track_share_mean=0.0,
                         capped_eta_share_mean=0.1, coverage_exceed_share=0.05)
                tune.append(r)
    (tmp_path / "tune.jsonl").write_text("\n".join(json.dumps(r) for r in tune) + "\n")
    report_cx_c.tune(tmp_path)
    sel = json.loads((tmp_path / "selected.json").read_text())
    assert sel["tune_sha256"] == hashlib.sha256((tmp_path / "tune.jsonl").read_bytes()).hexdigest()
    assert sel["kinds"]["tube"]["setting"] == {"c_R": 1.0}
    st = run_cx_c.settings(sel)
    rng = np.random.default_rng(1)
    test = []
    for kind in cx_c.TEST_KINDS:
        for i in list(range(1, 51)) + [0]:
            name, set_ = (f"CCT_car{i}", "CCT") if i else ("NOM_car0", "NOM")
            done = not (kind == "nmpc" and 1 <= i <= 8) and not (kind == "tube" and i == 9)
            r = {"job": f"test|{kind}|x|{name}", "kind": kind, "set": set_, "case": name, "setting": st[kind],
                 "completed": done, "lap_time": 76.0 + rng.normal(0, 0.1) + (0.5 if kind == "tube" else 0.0),
                 "stop_reason": "lap completed" if done else "off track", "failure_s": None if done else 3830.0,
                 "distance": 4315.0 if done else 3830.0, "min_edge_margin": 0.2}
            if kind in cx_c.TUBE_KINDS:
                r.update(tube_ms_mean_p95_p99_max=[30, 40, 50, 60], backoff_m_p5_p50_p95=[0.0, 0.2, 0.6],
                         eta_k_p5_p50_p95=[0.7, 0.9, 0.95], capped_track_share_mean=0.0, capped_eta_share_mean=0.3,
                         coverage_exceed_share=0.1)
            test.append(r)
    write = lambda rs: (tmp_path / "test.jsonl").write_text("\n".join(json.dumps(r) for r in rs) + "\n")
    write(test[:-1])
    with pytest.raises(SystemExit):
        report_cx_c.test(tmp_path)
    bad = [dict(r) for r in test]
    bad[60]["setting"] = {"c": 1.0, "margin": 0.3}
    write(bad)
    with pytest.raises(SystemExit):
        report_cx_c.test(tmp_path)
    write(test)
    report_cx_c.test(tmp_path)
    res = json.loads((tmp_path / "report" / "CX_C.json").read_text())
    assert res["H-C1"][:2] == [8, 1]                        # cars 1-8 only tube, car 9 only nmpc
    assert res["H-C2"]["n"] == 41 and res["holm"]["H-C2"]["rejected"]      # tube 0.5 s slower on 41 cars
    md = (tmp_path / "report" / "CX_C.md").read_text(encoding="utf-8")
    assert "| CCT_car9 |" in md and "10 comparisons" in md and "| tube vs nmpc_reserve |" in md
    assert "| nmpc_param vs nmpc |" in md and "the same code path, only the tightening differs" in md
