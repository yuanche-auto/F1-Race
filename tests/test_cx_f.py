"""Control expansion phase F (experiments/cx_f.py; docs/CX_F_protocol.md): sets, the shield's monitor,
``run_lap_shield`` against ``l7.run_lap``, the L7 options (retry, command log) and the statistics."""
from pathlib import Path

import numpy as np
import pytest

CACHE = Path(__file__).resolve().parents[1] / "results" / "L1" / "l1_plans.pkl"


def test_sets_and_signed_rank():
    from f1race.experiments import cx_f

    fv, fd = cx_f.cases("FV"), cx_f.cases("FD")
    assert len(fv) == 40 and len(fd) == 12 and len(cx_f.cases("NOM")) == 1
    assert sorted({c["tau"] for c in fv}) == [0.0, 0.03, 0.06]
    assert [c["dist_seed"] for c in fv if c["dist_seed"] is not None] == list(range(7500, 7510))
    assert [c["dist_seed"] for c in fd if c["dist_seed"] is not None] == [7590, 7591, 7592]
    assert len({c["name"] for c in fv + fd}) == 52
    assert cx_f.case_by_name("FV_car3_tau30")["plant"] is not None
    # L7's H-B: nine negative differences -> W+ = 0, p = 2 / 512
    r = cx_f.exact_signed_rank([-11, -71, -49, -38, -12, -52, -32, -8, -6])
    assert r["n"] == 9 and r["W_plus"] == 0.0 and r["p"] == pytest.approx(2 / 512)
    assert cx_f.exact_signed_rank([0, 0])["p"] == 1.0
    assert cx_f.runs([0, 1, 1, 0, 1, 0, 0, 1, 1, 1]) == [2, 1, 3]
    a = cx_f.lap_audit({"changed": [0, 1, 1, 0, 1], "s": [0, 10, 20, 30, 40], "time": [0.01, 0.06, 0.2, 0.01, 0.01],
                        "fail": [0, 1, 1, 1, 0]}, completed=False, length=80.0)
    assert a["on_switches"] == 2 and a["longest_run"] == 2 and a["time_over_50ms"] == 2
    assert a["time_over_100ms"] == 1 and a["fallback_runs3"] == 1 and a["interventions_before_failure"] == 3
    assert a["by_eighth"] == [0, 1, 1, 0, 1, 0, 0, 0]


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_shield_lap_matches_run_lap_and_forced_backup():
    pytest.importorskip("gymnasium")
    from f1race.experiments import cx_f, l7

    case = cx_f.cases("NOM")[0]
    env = l7.make_env()
    ref = l7.run_lap(None, env, case, None, t_max=4.0)
    sh = cx_f.run_lap_shield(None, env, case, cx_f.PassShield(), t_max=4.0)
    assert np.array_equal(ref["log"].state, sh["log"].state) and np.array_equal(ref["log"].inputs, sh["log"].inputs)
    n_dec = len(sh["log"].t) // 5                                  # one decision per 5 steps of 10 ms
    assert not sh["decisions"]["changed"].any() and len(sh["decisions"]["t"]) == n_dec
    # the command log of run_lap changes nothing and records one command per decision
    lg = l7.run_lap(None, env, case, None, t_max=4.0, log_commands=True)
    assert np.array_equal(ref["log"].inputs, lg["log"].inputs) and len(lg["decisions"]["u_prop"]) == n_dec
    # forced interventions = the backup action at every decision (the tracker's command recomputed
    # from the same measurement and state; the policy observes the applied a_b, review m1)
    seen = []

    def pol(obs):
        seen.append(np.array(obs[-2:]))
        return np.zeros(2)

    back = l7.run_lap(lambda obs: np.array(cx_f.A_B), env, case, None, t_max=4.0)
    fo = cx_f.run_lap_shield(pol, env, case, cx_f.AlwaysShield(env), t_max=4.0)
    assert fo["decisions"]["changed"].all()
    assert np.array_equal(back["log"].state, fo["log"].state) and np.array_equal(back["log"].inputs, fo["log"].inputs)
    assert np.allclose(seen[1], cx_f.A_B) and np.allclose(seen[0], 0.0)
    assert not np.array_equal(fo["decisions"]["u_prop"], fo["decisions"]["u_appl"])


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_monitor_accepts_plan_rejects_pushed_state():
    pytest.importorskip("gymnasium")
    import copy

    from f1race.experiments import cx_f, l7

    env = l7.make_env()
    env.reset(options={"plant": cx_f.cases("NOM")[0]["plant"], "start_index": 0})
    for _ in range(20):
        env.step(np.zeros(2))
    sh = cx_f.Shield(env, cx_f.ShieldConfig(margin=0.5))
    y = np.array(env._meas, float)
    ok, info = sh.check(copy.copy(env.ctl), y, env.steer_max)
    assert ok, info
    # 3 m towards the left edge with a heading error towards it: not recoverable
    yb = y.copy()
    c, s = np.cos(y[2]), np.sin(y[2])
    yb[0] -= 3.0 * s
    yb[1] += 3.0 * c
    yb[2] += 0.15
    ok2, info2 = sh.check(copy.copy(env.ctl), yb, env.steer_max)
    assert not ok2 and info2["reason"] in ("margin", "spin", "not settled")


def test_retry_option_default_off():
    pytest.importorskip("casadi")
    if not CACHE.exists():
        pytest.skip("run scripts/run_l1.py first")
    from f1race.dynamics import DynVehicle
    from f1race.experiments import l7
    from f1race.track import Track

    tr = Track.load("Spielberg")
    assert l7.NMPCFilter(tr, DynVehicle(), l7.plan(0.95), record=False, horizon=5).mpc.retry is False
    assert l7.NMPCFilter(tr, DynVehicle(), l7.plan(0.95), record=False, horizon=5, retry=True).mpc.retry is True


@pytest.mark.skipif(not CACHE.exists(), reason="run scripts/run_l1.py first")
def test_plan_backup_mode_drives_with_the_backup_tracker():
    pytest.importorskip("gymnasium")
    from f1race.experiments import cx_f, l7

    env = l7.make_env()

    class Forced(cx_f.Shield):
        def check(self, ctl, y, steer_bound):
            return False, {"reason": "forced"}

    sh = Forced(env, cx_f.ShieldConfig(margin=0.0, backup_eta=0.85))
    assert sh.mode == "plan" and sh.ctl_b is not None
    seen = []
    r = cx_f.run_lap_shield(lambda o: (seen.append(np.array(o[-2:])), np.array([0.0, 1.0]))[1], env,
                            cx_f.cases("NOM")[0], sh, t_max=4.0)
    d = r["decisions"]
    assert d["changed"].all() and not np.allclose(d["u_prop"], d["u_appl"])
    assert np.allclose(seen[1], [0.0, 1.0])                 # plan mode: the env keeps the policy's action
    assert not r["completed"] and r["stop_reason"] == "time limit"
    with pytest.raises(ValueError):
        cx_f.backup_plan(0.84)
