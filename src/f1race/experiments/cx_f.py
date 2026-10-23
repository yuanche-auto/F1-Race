"""Control expansion phase F: safety filters around the L5 policies (docs/CX_F_protocol.md,
revision 0 with its clarifications m1–m3, n1, n2).

* Sets (section 2): ``T1`` / ``T2`` – L7's test sets (``experiments.l7.cases``); ``FV`` – tuning,
  ``sample_plants(nominal, 10, seed 351)[1:]`` × τ ∈ {0, 30, 60 ms} and the nominal car with the L3
  disturbances, seeds 7500–7509 (40 laps); ``FD`` – development only, seed 359, three cars × τ and
  disturbance seeds 7590–7592 (12 laps); ``NOM`` – the nominal car.
* ``Shield`` (section 3.1): model predictive shielding (literature F.1). At every 50 ms decision, from
  the measured state y the tracker sees and a copy of the tracker: the proposal step (the tracker with
  the policy's action, 5 steps of 10 ms) and then the backup rollout (the tracker with a_b = (0, −1):
  steering offset 0, speed factor 0.85, for T_b = 3 s), both on the nominal model ``DynVehicle()``
  without lag (``DynamicSim``'s RK4 step and limits). Recoverable: at every step both axle margins
  (``FastMargins`` of the nominal car) ≥ m, |β| ≤ 0.3 rad and v_x ≥ 5 m/s, and the last state settled –
  the tracker's |e| ≤ 1 m, |Δψ| ≤ 0.1 rad, v_x ≤ 0.85 v_plan(s) + 2 m/s. Not recoverable → a_b for this
  decision; then the backup from y itself is checked too (3.05 s): failing it is a decision with **no
  recoverable option**.
* ``run_lap_shield``: ``experiments.l7.run_lap`` with the shield at the first 10 ms step of every
  decision (the measurement and the tracker command of that step, as L7's filters). An intervention
  sets the tracker's action to a_b for the decision and recomputes the step's command from the same
  measurement and the tracker's state before it (its projection index restored); the environment's
  last action – what the policy observes – is a_b (review m1). Snapshots for the false interventions
  as L7's A8 (review n2). With a shield that never intervenes the lap is ``run_lap``'s lap (tests).
* Audit helpers (section 5.1) on per-decision records, and the exact signed-rank of L7's H-B.
"""
from __future__ import annotations

import copy
import itertools
import math
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from ..dynamics import DynVehicle
from .l2 import sample_plants
from . import l7

ROOT = Path(__file__).resolve().parents[3]
TAUS = l7.TAUS
SETS = {"FV": (351, 10, tuple(range(7500, 7510))), "FD": (359, 3, tuple(range(7590, 7593)))}
FINALS = [f"{g}_s{s}" for g in "ABC" for s in (1, 2, 3)]
PROPOSERS = FINALS + ["B_s1_500k", "tracker"]
A_B = (0.0, -1.0)                       # the backup action: steering offset 0, speed factor 1 - 0.15
M_GRID = (0.25, 0.5, 1.0)               # the shield's margin (section 4)
FAIL_WINDOW = 60                        # decisions (3 s): a failure "after an intervention" (review m2)


# ---------------------------------------------------------------- sets
def cases(name: str) -> list[dict]:
    """The laps of a set (section 2), as ``l7.cases`` builds them."""
    if name in ("T1", "T2"):
        return l7.cases(name)
    nom = DynVehicle()
    if name == "NOM":
        return [{"set": "NOM", "car": 0, "info": {"nominal": True}, "plant": nom, "tau": 0.0,
                 "dist_seed": None, "name": "NOM"}]
    seed, n, dist = SETS[name]
    cars = sample_plants(nom, n, seed)
    out = []
    for tau in TAUS:
        for i in range(1, len(cars)):
            out.append({"set": name, "car": i, "info": cars[i][0], "plant": cars[i][1], "tau": tau,
                        "dist_seed": None, "name": f"{name}_car{i}_tau{int(round(tau * 1000))}"})
    for d in dist:
        out.append({"set": name, "car": None, "info": {"nominal": True}, "plant": nom, "tau": 0.0,
                    "dist_seed": d, "name": f"{name}_dist{d}"})
    return out


def case_by_name(name: str) -> dict:
    st = name.split("_")[0]
    return next(c for c in cases(st) if c["name"] == name)


# ---------------------------------------------------------------- proposers
def policy_path(name: str) -> Path:
    if name == "B_s1_500k":
        return ROOT / "models" / "L5" / f"{name}.zip"
    return ROOT / "results" / "L5" / "multiseed" / f"{name}.zip"


def steer_mode(name: str) -> str:
    return "lateral" if name.startswith("C_") else "angle"


def load_policy(name: str):
    """obs -> action of an L5 checkpoint (deterministic); None for the bare tracker."""
    if name == "tracker":
        return None
    from stable_baselines3 import SAC

    model = SAC.load(policy_path(name), device="cpu")
    return lambda obs: model.predict(obs, deterministic=True)[0]


# ---------------------------------------------------------------- the shield
@dataclass
class ShieldConfig:
    margin: float = 0.5
    a_b: tuple = A_B
    t_b: float = 3.0
    beta_max: float = 0.3
    v_min: float = 5.0
    e_max: float = 1.0
    dpsi_max: float = 0.1
    dv: float = 2.0
    backup_eta: float | None = None       # None: a_b in the policy's action space (revision 0); a plan's
    #                                       eta: the bare tracker on that plan (development variant)


def backup_plan(eta: float):
    """A plan of phase D's library (L1 / L2's and ``plans_cxd.pkl``), by its eta (exact)."""
    from .cx_d import plan_library

    etas, plans = plan_library()
    i = int(np.argmin(np.abs(etas - eta)))
    if abs(etas[i] - eta) > 1e-9:
        raise ValueError(f"no plan at eta {eta}")
    return plans[i]


class Shield:
    """The monitor of section 3.1 on the nominal model; ``check`` returns (recoverable, info).
    ``mode`` "action": the backup is a_b, an action of the policy's tracker (revision 0); "plan"
    (``cfg.backup_eta``): the backup is a second tracker - the L1 tracker with zero action on the
    plan at ``backup_eta`` with its own feed-forward - whose command replaces the policy's tracker's
    at every 10 ms step of an intervened decision (the environment keeps the policy's action, as
    with L7's filters)."""

    def __init__(self, env, cfg: ShieldConfig | None = None):
        from ..rl.env import FastMargins, ResidualTracker
        from ..sim.dynamic import DynamicSim
        from .l1 import dyn_reference

        self.cfg = cfg or ShieldConfig()
        nom = DynVehicle()
        self.sim = DynamicSim(env.track, nom, env.dt)
        self.fm = FastMargins(env.track, nom)
        self.dt, self.ctrl_every = env.dt, env.ctrl_every
        self.speed_down, self.speed_up = env.speed_down, env.speed_up
        self.n_b = int(round(self.cfg.t_b / env.dt))
        self.mode, self.ctl_b = "action", None
        if self.cfg.backup_eta is not None:
            self.mode = "plan"
            path_b, ff_b = dyn_reference(env.track, nom, backup_plan(self.cfg.backup_eta))
            self.ctl_b = ResidualTracker(path_b, ff_b, nom, env.dt, **env.tracker_gains)

    def backup_params(self, steer_bound: float) -> tuple[float, float]:
        """(steering offset, speed factor) of a_b, as ``RacingEnv.step`` maps an action."""
        a = self.cfg.a_b
        return steer_bound * a[0], 1.0 + (self.speed_up if a[1] > 0 else self.speed_down) * a[1]

    def rollout(self, y, phases) -> tuple[bool, str, np.ndarray]:
        """From y: ``phases`` = [(tracker, d_offset, v_scale, steps, fresh), ...], each tracker copied
        (``fresh``: its projection searched anew); True if every state keeps the margins and the last
        one is settled for the last tracker; (ok, reason, last state)."""
        cfg, sim, fm = self.cfg, self.sim, self.fm
        fm.reset()
        x = np.array(y[:7], float)
        c, v_sc = None, 1.0
        for ctl, d_off, v_sc, n, fresh in phases:
            c = copy.copy(ctl)                             # its state: idx, d_offset, v_scale, last
            if fresh:
                c.idx = None
            c.d_offset, c.v_scale = d_off, v_sc
            for _ in range(n):
                x = sim.step(x, sim.apply_limits(x, c(0.0, x)))
                _, mr, mf = fm(x)
                if min(mr, mf) < cfg.margin:
                    return False, "margin", x
                if abs(math.atan2(x[4], max(x[3], 1e-3))) > cfg.beta_max or x[3] < cfg.v_min:
                    return False, "spin", x
        c(0.0, x)                                          # the tracker's errors at the last state
        _, e, dpsi, ff = c.last
        if abs(e) > cfg.e_max or abs(dpsi) > cfg.dpsi_max or x[3] > v_sc * ff["vx"] + cfg.dv:
            return False, "not settled", x
        return True, "", x

    def _backup_phase(self, ctl, steer_bound: float, n: int) -> tuple:
        if self.mode == "plan":
            return (self.ctl_b, 0.0, 1.0, n, True)
        d_b, v_b = self.backup_params(steer_bound)
        return (ctl, d_b, v_b, n, False)

    def check(self, ctl, y, steer_bound: float) -> tuple[bool, dict]:
        """The proposal (the tracker's current action) for one decision, then the backup."""
        ok, why, _ = self.rollout(y, [(ctl, ctl.d_offset, ctl.v_scale, self.ctrl_every, False),
                                      self._backup_phase(ctl, steer_bound, self.n_b)])
        return ok, {"reason": why}

    def backup_ok(self, ctl, y, steer_bound: float) -> bool:
        """The backup from y itself (one decision plus T_b): False = no recoverable option."""
        return self.rollout(y, [self._backup_phase(ctl, steer_bound, self.ctrl_every + self.n_b)])[0]


class PassShield:
    """Never intervenes (tests: ``run_lap_shield`` then reproduces ``l7.run_lap``)."""

    cfg = ShieldConfig()
    mode = "action"

    def check(self, ctl, y, steer_bound):
        return True, {"reason": ""}


class AlwaysShield(PassShield):
    """Always intervenes (tests: the lap of a_b at every decision)."""

    def __init__(self, env):
        self.speed_down, self.speed_up = env.speed_down, env.speed_up

    backup_params = Shield.backup_params

    def check(self, ctl, y, steer_bound):
        return False, {"reason": "forced"}

    def backup_ok(self, ctl, y, steer_bound):
        return True


def _steer_bound(env) -> float:
    """``RacingEnv.step``'s bound on the steering offset at the current measurement."""
    bound = env.steer_max
    if env.steer_mode == "lateral":
        v = max(float(env._meas[3]), 1.0)
        bound = min(bound, env.a_corr * env.nominal.L / (v * v))
    return bound


def run_lap_shield(policy, env, case: dict, shield, t_max: float = 120.0, snap_at=(),
                   log_commands: bool = True) -> dict:
    """``l7.run_lap`` with ``shield`` deciding at the first 10 ms step of every decision (module
    docstring). Returns the same fields; ``decisions`` has t, changed, time (the check), s,
    no_option, reason and, with ``log_commands``, u_prop / u_appl."""
    episode_s = env.episode_s
    env.episode_s = t_max
    opts = {"plant": case["plant"], "start_index": 0, "disturbances": case["dist_seed"] is not None,
            "dist_seed": case["dist_seed"] or 0}
    if case["tau"] > 0:
        opts["steer_lag"] = case["tau"]
    obs, _ = env.reset(options=opts)
    sim = env.sim
    orig = sim.apply_limits
    st = {"first": False, "idx": env.ctl.idx, "bound": 0.0, "active": False}
    T, X, U, US = [], [], [], []
    dec = {"t": [], "changed": [], "time": [], "s": [], "no_option": [], "reason": []}
    if log_commands:
        dec.update({"u_prop": [], "u_appl": []})

    def apply_limits(x, u):
        out = orig(x, u)
        y = np.asarray(env._meas, float)                 # what the tracker saw at this step
        if st["first"]:
            st["first"] = False
            ctl = env.ctl
            pre = copy.copy(ctl)
            pre.idx = st["idx"]                          # the tracker's state before this step's call
            t0 = time.perf_counter()
            ok, info = shield.check(pre, y, st["bound"])
            u_prop = out
            no_option = False
            if not ok and shield.mode == "plan":            # the backup tracker drives this decision
                if not st["active"]:
                    shield.ctl_b.idx = None                 # its projection searched anew
                st["active"] = True
                out = orig(x, shield.ctl_b(env.t, y))
                no_option = not shield.backup_ok(pre, y, st["bound"])
            elif not ok:
                ctl.idx = st["idx"]
                ctl.d_offset, ctl.v_scale = shield.backup_params(st["bound"])
                out = orig(x, ctl(env.t, y))
                pre.d_offset, pre.v_scale = ctl.d_offset, ctl.v_scale
                no_option = not shield.backup_ok(pre, y, st["bound"])
            else:
                st["active"] = False
            dec["t"].append(env.t)
            dec["changed"].append(not ok)
            dec["time"].append(time.perf_counter() - t0)
            dec["s"].append(ctl.last[0] if ctl.last is not None else float("nan"))
            dec["no_option"].append(no_option)
            dec["reason"].append(info["reason"])
            if log_commands:
                dec["u_prop"].append(np.array(u_prop, float))
                dec["u_appl"].append(np.array(out, float))
        elif st["active"]:
            out = orig(x, shield.ctl_b(env.t, y))
        st["idx"] = env.ctl.idx
        T.append(env.t)
        X.append(np.array(x[:7], float))
        U.append(np.array(out, float))
        US.append(sim._forces(x, out)[1])
        return out

    sim.apply_limits = apply_limits
    L = env.track.length
    done, info = False, {}
    pending = sorted(snap_at)
    snapshots = []
    while not done:
        a = np.zeros(2) if policy is None else policy(obs)
        snap = l7.env_snapshot(env) if pending and env.progress >= pending[0] else None
        n_dec = len(dec["t"])
        st["first"], st["bound"] = True, _steer_bound(env)
        obs, _, term, trunc, info = env.step(a)
        intervened = len(dec["changed"]) > n_dec and dec["changed"][-1]
        if intervened and not term and shield.mode == "action":    # review m1: the policy observes a_b
            env._last_a = np.asarray(shield.cfg.a_b, np.float32)
            obs = env._stacked()
        if snap is not None and intervened:
            snapshots.append({"at": pending.pop(0), "progress": snap["progress"], "snap": snap})
        done = term or trunc or env.progress >= L
    env.episode_s = episode_s
    sim.apply_limits = orig
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)
    log = SimpleNamespace(t=np.array(T), state=np.array(X), inputs=np.array(U), usage=np.array(US),
                          completed=bool(completed))
    return {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
            "stop_reason": "lap completed" if completed else (info.get("reason") or "time limit"),
            "distance": float(env.progress), "log": log,
            "decisions": {k: np.array(v) for k, v in dec.items()}, "snapshots": snapshots}


# ---------------------------------------------------------------- statistics and audit
def exact_signed_rank(diffs) -> dict:
    """L7's H-B test (``scripts/report_l7_partB.py``, the same code): zero differences dropped,
    mid-ranks, the exact null distribution of W+ by enumerating the 2^n signs; two-sided."""
    from scipy.stats import rankdata

    d = np.asarray([x for x in diffs if x != 0], float)
    n = len(d)
    if n == 0:
        return {"n": 0, "W_plus": 0.0, "p": 1.0}
    r = rankdata(np.abs(d))
    w_obs = float(r[d > 0].sum())
    mu = n * (n + 1) / 4.0
    dev = abs(w_obs - mu)
    hits = 0
    for signs in itertools.product((0, 1), repeat=n):
        w = float(np.dot(signs, r))
        hits += abs(w - mu) >= dev - 1e-9
    return {"n": n, "W_plus": w_obs, "p": hits / 2 ** n}


def runs(flags) -> list[int]:
    """Lengths of the runs of True."""
    out, n = [], 0
    for f in np.asarray(flags, bool):
        if f:
            n += 1
        elif n:
            out.append(n)
            n = 0
    if n:
        out.append(n)
    return out


def lap_audit(dec: dict, completed: bool, length: float) -> dict:
    """Section 5.1 / 5 for one lap's decision record (L7's ``dec`` npz or phase F's): interventions
    by eighth of the lap, on-switches, the longest intervention run, fallback runs (``fail``), the
    filter's time against the 50 ms period, and – for a failed lap – the interventions within the last
    ``FAIL_WINDOW`` decisions (followed by the failure within 3 s, review m2)."""
    ch = np.asarray(dec["changed"], bool)
    s = np.asarray(dec["s"], float)
    tm = np.asarray(dec["time"], float)
    n = len(ch)
    eighth = np.clip((np.nan_to_num(s) % length) // (length / 8), 0, 7).astype(int) if n else np.zeros(0, int)
    out = {"decisions": n, "interventions": int(ch.sum()),
           "by_eighth": [int(ch[eighth == j].sum()) for j in range(8)],
           "decisions_by_eighth": [int((eighth == j).sum()) for j in range(8)],
           "on_switches": int(np.sum(ch[1:] & ~ch[:-1]) + (ch[0] if n else 0)),
           "longest_run": max(runs(ch), default=0),
           "time_over_50ms": int(np.sum(tm > 0.05)), "time_over_100ms": int(np.sum(tm > 0.10)),
           "time_mean_ms": float(tm.mean() * 1e3) if n else float("nan")}
    if "fail" in dec:
        fb = np.asarray(dec["fail"], bool)
        r = runs(fb)
        out.update({"fallbacks": int(fb.sum()), "fallback_runs3": int(sum(x >= 3 for x in r)),
                    "longest_fallback_run": max(r, default=0)})
    if "no_option" in dec:
        out["no_option"] = int(np.asarray(dec["no_option"], bool).sum())
    out["interventions_before_failure"] = 0 if completed else int(ch[max(n - FAIL_WINDOW, 0):].sum())
    return out


def false_interventions(records: list[dict], proposer: str, length: float) -> dict:
    """L7's A8 summary (``scripts/report_l7_partB.py``, the same procedure – review n2): the snapshots
    of ``proposer``'s records in record order, 200 drawn uniformly with seed 0 (all if fewer);
    the share whose unfiltered 3 s rollout did not crash, and per eighth of the lap."""
    snaps = [s for r in records if r.get("policy") == proposer for s in r.get("false_interventions", [])]
    if not snaps:
        return {"snapshots": 0, "sampled": 0, "false": 0}
    rng = np.random.default_rng(0)
    idx = rng.choice(len(snaps), size=min(200, len(snaps)), replace=False)
    smp = [snaps[i] for i in sorted(idx)]
    per = []
    for j in range(8):
        g = [s for s in smp if int(round(s["at"] / (length / 8))) == j]
        per.append([sum(not s["crashed"] for s in g), len(g)])
    return {"snapshots": len(snaps), "sampled": len(smp), "false": sum(not s["crashed"] for s in smp),
            "per_eighth": per}


def attribute(dec: dict) -> str:
    """L7's pb-m2 attribution of a failed filtered lap (its last 60 decisions)."""
    fb = np.asarray(dec.get("fail", np.zeros(len(dec["changed"]))), bool)[-FAIL_WINDOW:]
    ch = np.asarray(dec["changed"], bool)[-FAIL_WINDOW:]
    best = max(runs(fb), default=0)
    if best >= 3:
        return "fallback run"
    return "intervention" if ch.any() else "neither"


__all__ = ["cases", "case_by_name", "Shield", "ShieldConfig", "PassShield", "AlwaysShield", "run_lap_shield",
           "backup_plan", "exact_signed_rank", "lap_audit", "runs", "false_interventions", "attribute", "load_policy", "steer_mode", "PROPOSERS", "FINALS"]
