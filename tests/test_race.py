"""The race front end (src/f1race/race.py, scripts/race.py): classification, report, replay,
the plan cache, and one real race of the cheapest driver."""
import json
import re

import numpy as np
import pytest

from f1race.race import DRIVERS, Entry, RaceConfig, RaceResult, race, replay_data, report, write_replay
from f1race.sim.kinematic import SimResult


def fake_sim(lap, distance, completed, reason="lap completed"):
    t = np.linspace(0.0, lap if completed else 10.0, 11)
    state = np.zeros((11, 5))
    state[:, 0] = np.linspace(0, 100, 11)
    state[:, 3] = 80.0
    prog = np.linspace(0.0, distance, 11)
    return SimResult(t, state, np.zeros((11, 2)), prog, np.full(11, 0.9), np.full((11, 2), 1.0),
                     lap if completed else float("nan"), completed, reason)


def fake_result(track):
    e = [Entry(DRIVERS["centre"], 78.8, 0.5, False, fake_sim(78.84, track.length, True)),
         Entry(DRIVERS["ocp"], 71.3, 20.0, False, fake_sim(0.0, 3842.0, False, "off track")),
         Entry(DRIVERS["mincurv"], 72.1, 16.0, True, fake_sim(72.43, track.length, True))]
    return RaceResult(RaceConfig(grip=1.0), track, e)


def test_classification_and_report(track):
    res = fake_result(track)
    assert [e.driver.key for e in res.classification()] == ["mincurv", "centre", "ocp"]
    text = report(res)
    lines = text.splitlines()
    assert lines[2].split()[:2] == ["1", "MCV"] and "1:12.430" in lines[2]
    assert "+6.410" in lines[3]
    assert lines[4].startswith("  DNF") and "off track at 3,842 m" in lines[4]
    text.encode("ascii")                              # printable on any console


def test_replay_html_roundtrip(track, tmp_path):
    res = fake_result(track)
    out = write_replay(res, tmp_path / "r.html")
    html = out.read_text(encoding="utf-8")
    m = re.search(r'<script id="race-data" type="application/json">(.*?)</script>', html, re.S)
    data = json.loads(m.group(1))
    assert data == json.loads(json.dumps(replay_data(res)))
    assert [c["code"] for c in data["cars"]] == ["CTR", "OCP", "MCV"]
    assert data["cars"][1]["finished"] is False and data["cars"][1]["lap"] is None
    assert data["cars"][2]["t"][-1] == pytest.approx(72.43)       # the last sample is always kept


def test_unknown_driver_is_refused(track):
    with pytest.raises(KeyError):
        race(RaceConfig(drivers=("nobody",)), log=lambda *a: None)


def test_centre_line_race_and_cache(track, tmp_path):
    cfg = RaceConfig(drivers=("centre",), cache_dir=tmp_path)
    first = race(cfg, log=lambda *a: None)
    e = first.entries[0]
    assert e.finished and not e.cached
    assert 78.0 < e.lap_time < 80.0                   # the centre line + QSS at 97 % grip: 78.84 s
    assert e.max_usage <= 0.995 + 1e-9                # the tracker's own limit
    again = race(cfg, log=lambda *a: None).entries[0]
    assert again.cached and again.lap_time == e.lap_time
