"""A race: a few drivers on one track, one flying lap on the kinematic car (L0's setting).

A *driver* is a way to plan the lap; the plan is then driven in the time-domain simulator by
the L0 path/speed tracker (``experiments.l0.run_closed_loop``), so what counts is the lap the
car actually drives, not the plan:

* ``ocp``      the minimum-time OCP – racing line and speed optimised together
* ``mincurv``  the minimum-curvature line with a quasi-steady speed profile
* ``centre``   the centre line with a quasi-steady speed profile

``grip`` is the share of the friction ellipse the plans use (L0's executed plans: 0.97);
the tracker never commands more than 99.5 % of it, so with ``grip = 1`` a plan has no room
left for its tracking errors. Plans are cached per track and setting (``results/race/cache``),
so a second race with the same setting takes seconds.

    python scripts/race.py                       # the command-line front end
"""
from __future__ import annotations

import contextlib
import hashlib
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path as FsPath

import numpy as np

from .experiments.l0 import plan_from_points, run_closed_loop
from .path import Path
from .planning.mincurv import min_curvature_line
from .planning.ocp_kinematic import initial_guess_from_path, solve_min_time
from .planning.qss import speed_profile
from .sim.kinematic import SimResult
from .track import Track
from .vehicle import Vehicle

ROOT = FsPath(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Driver:
    key: str
    code: str
    name: str
    style: str
    color: str


DRIVERS = {
    "ocp": Driver("ocp", "OCP", "Minimum-time OCP", "line and speed optimised together", "#3987e5"),
    "mincurv": Driver("mincurv", "MCV", "Min. curvature + QSS", "geometric racing line, then the speed", "#d95926"),
    "centre": Driver("centre", "CTR", "Centre line + QSS", "no racing line", "#199e70"),
}


@dataclass
class RaceConfig:
    track: str = "Spielberg"
    drivers: tuple = ("ocp", "mincurv", "centre")
    grip: float = 0.97           # share of the friction ellipse the plans use
    margin: float = 0.3          # planned distance to the edges [m]
    dt: float = 0.02             # simulator step [s]
    cache: bool = True
    cache_dir: FsPath | None = None   # default: results/race/cache


@dataclass
class Entry:
    driver: Driver
    plan_lap: float              # lap time of the plan itself [s]
    plan_seconds: float          # time it took to compute the plan (0 when cached) [s]
    cached: bool
    sim: SimResult

    @property
    def finished(self) -> bool:
        return bool(self.sim.completed)

    @property
    def lap_time(self) -> float:
        return float(self.sim.lap_time)

    @property
    def distance(self) -> float:
        return float(self.sim.progress[-1])

    @property
    def top_speed_kmh(self) -> float:
        return float(self.sim.state[:, 3].max() * 3.6)

    @property
    def max_usage(self) -> float:
        return self.sim.max_usage

    @property
    def min_margin(self) -> float:
        return self.sim.min_margin


@dataclass
class RaceResult:
    config: RaceConfig
    track: Track
    entries: list = field(default_factory=list)
    seconds: float = 0.0

    def classification(self) -> list:
        """Finishers by lap time, then the others by the distance they covered."""
        done = sorted((e for e in self.entries if e.finished), key=lambda e: e.lap_time)
        out = sorted((e for e in self.entries if not e.finished), key=lambda e: -e.distance)
        return done + out


# --------------------------------------------------------------------------- planning
@contextlib.contextmanager
def _quiet_stdout():
    """Silence file descriptor 1 (output written from C libraries) for the block."""
    try:
        fd = sys.stdout.fileno()
    except (AttributeError, OSError, ValueError):     # no real stdout (e.g. captured): nothing to do
        yield
        return
    sys.stdout.flush()
    saved = os.dup(fd)
    try:
        with open(os.devnull, "w") as null:
            os.dup2(null.fileno(), fd)
            yield
    finally:
        sys.stdout.flush()
        os.dup2(saved, fd)
        os.close(saved)


def _fingerprint(track_name: str) -> str:
    """Changes when the track file or the planning / tracking code changes (stale caches)."""
    h = hashlib.sha256()
    csv = ROOT / "data" / "tracks" / f"{track_name}.csv"
    if csv.exists():
        h.update(csv.read_bytes())
    src = FsPath(__file__).resolve().parent
    for rel in ("planning/mincurv.py", "planning/qss.py", "planning/ocp_kinematic.py", "path.py", "track.py",
                "vehicle.py"):
        p = src / rel
        if p.exists():
            h.update(p.read_bytes())
    return h.hexdigest()[:16]


_PATH_FIELDS = ("s", "x", "y", "psi", "kappa", "dkappa", "length")     # the fields of a Path, in order


def _cache_file(cfg: RaceConfig, key: str) -> FsPath:
    base = FsPath(cfg.cache_dir) if cfg.cache_dir is not None else ROOT / "results" / "race" / "cache"
    return base / f"{cfg.track}_{key}_grip{cfg.grip:.3f}_margin{cfg.margin:.2f}.npz"


def _load(cfg: RaceConfig, key: str, fp: str):
    f = _cache_file(cfg, key)
    if not (cfg.cache and f.exists()):
        return None
    d = np.load(f, allow_pickle=False)
    if str(d["fingerprint"]) != fp:
        return None
    path = Path(*(d[f"path_{n}"] for n in _PATH_FIELDS[:-1]), length=float(d["path_length"]))
    k = d["kappa"] if d["kappa"].size else None
    return path, d["v"], d["a"], k, float(d["plan_lap"])


def _save(cfg: RaceConfig, key: str, fp: str, path: Path, v, a, k, plan_lap: float) -> None:
    if not cfg.cache:
        return
    f = _cache_file(cfg, key)
    f.parent.mkdir(parents=True, exist_ok=True)
    np.savez(f, **{f"path_{n}": getattr(path, n) for n in _PATH_FIELDS}, v=v, a=a,
             kappa=np.array([]) if k is None else k, plan_lap=plan_lap, fingerprint=fp)


def plan_driver(track: Track, veh: Vehicle, key: str, cfg: RaceConfig, shared: dict):
    """(dense path, v, a, curvature feed-forward or None, plan lap time) of one driver.
    ``shared`` keeps the minimum-curvature line, which the OCP starts from."""
    if key == "centre":
        cl = Path.from_points(np.column_stack([track.x[::3], track.y[::3]]), resample=0.5)
        q = speed_profile(cl, veh, cfg.grip)
        return cl, q.v, q.a, None, q.lap_time
    if "mc" not in shared:
        shared["mc"] = min_curvature_line(track, veh, margin=cfg.margin, step=3)
    alpha, mc = shared["mc"]
    if key == "mincurv":
        dense = Path.from_points(np.column_stack([mc.x, mc.y]), resample=0.5)
        q = speed_profile(dense, veh, cfg.grip)
        return dense, q.v, q.a, None, q.lap_time
    if key == "ocp":
        idx = np.arange(0, len(track.s), 3)
        qk = speed_profile(mc, veh, 1.0)
        init = initial_guess_from_path(track, veh, idx, alpha, mc, qk.v, qk.a)
        with _quiet_stdout():                            # IPOPT's banner, printed from C
            o = solve_min_time(track, veh, init, ds=2.0, margin=cfg.margin, eta=cfg.grip)
        if not o.success:
            raise RuntimeError(f"the minimum-time OCP did not converge on {track.name}: {o.status}")
        path, v, a, k = plan_from_points(o.xy(track), o.v, kappa=np.tan(o.delta) / veh.L)
        return path, v, a, k, o.lap_time
    raise KeyError(f"unknown driver {key!r}; known: {sorted(DRIVERS)}")


# --------------------------------------------------------------------------- race
def race(cfg: RaceConfig = RaceConfig(), log=print) -> RaceResult:
    t_all = time.time()
    for k in cfg.drivers:
        if k not in DRIVERS:
            raise KeyError(f"unknown driver {k!r}; known: {sorted(DRIVERS)}")
    track = Track.load(cfg.track)
    veh = Vehicle()
    fp = _fingerprint(cfg.track)
    shared: dict = {}
    plans = {}
    log("Practice - each driver plans the lap")
    order = sorted(cfg.drivers, key=lambda k: ("centre", "mincurv", "ocp").index(k))   # cheap ones first
    for key in order:
        d = DRIVERS[key]
        t0 = time.time()
        got = _load(cfg, key, fp)
        cached = got is not None
        if not cached:
            got = plan_driver(track, veh, key, cfg, shared)
            _save(cfg, key, fp, *got[:4], got[4])
        plans[key] = (got, time.time() - t0, cached)
        how = "cached" if cached else f"{time.time() - t0:.1f} s"
        log(f"  {d.code}  {d.name:<22} plan {got[4]:6.2f} s   ({how})")
    log("Race - one flying lap each, in the simulator")
    res = RaceResult(cfg, track)
    for key in cfg.drivers:
        (path, v, a, k, plan_lap), secs, cached = plans[key]
        sim = run_closed_loop(track, veh, path, v, a, k, cfg.dt)
        res.entries.append(Entry(DRIVERS[key], plan_lap, 0.0 if cached else secs, cached, sim))
    res.seconds = time.time() - t_all
    return res


def fmt_time(t: float) -> str:
    return f"{int(t // 60)}:{t % 60:06.3f}"


def report(res: RaceResult) -> str:
    """The classification as plain text (ASCII only: any console can print it)."""
    cfg, tr = res.config, res.track
    rows = res.classification()
    best = next((e.lap_time for e in rows if e.finished), None)
    lines = [f"Classification - {tr.name}, {tr.length / 1000:.2f} km, grip used by the plans {cfg.grip:.0%}",
             f"  {'POS':<4} {'DRIVER':<28} {'LAP TIME':>9} {'GAP':>8} {'TOP SPEED':>10} {'MAX GRIP':>9} {'MIN EDGE':>9}"]
    for i, e in enumerate(rows):
        d = e.driver
        who = f"{d.code}  {d.name}"
        if e.finished:
            gap = "-" if e.lap_time == best else f"+{e.lap_time - best:.3f}"
            lines.append(f"  {i + 1:<4} {who:<28} {fmt_time(e.lap_time):>9} {gap:>8} {e.top_speed_kmh:>6.0f} km/h"
                         f" {e.max_usage:>8.1%} {e.min_margin:>7.2f} m")
        else:
            lines.append(f"  {'DNF':<4} {who:<28} {e.sim.stop_reason} at {e.distance:,.0f} m")
    return "\n".join(lines)


# --------------------------------------------------------------------------- replay
def replay_data(res: RaceResult, every: int = 5) -> dict:
    """What the HTML replay draws: the track edges every ~4 m and each car every
    ``every`` simulator steps (0.1 s at dt = 0.02)."""
    tr = res.track
    left, right = tr.edges()
    step = max(1, int(round(4.0 / tr.ds)))
    p0 = tr.point_at(np.array([0.0]), np.array([0.0]))[0]
    rnd = lambda a: np.round(np.asarray(a, float), 2).tolist()     # noqa: E731
    cars = []
    for e in res.entries:
        s = e.sim
        idx = np.unique(np.append(np.arange(0, len(s.t), every), len(s.t) - 1))
        cars.append(dict(code=e.driver.code, name=e.driver.name, style=e.driver.style, color=e.driver.color,
                         finished=e.finished, lap=None if not e.finished else round(e.lap_time, 3),
                         reason=s.stop_reason, distance=round(e.distance, 1), plan=round(e.plan_lap, 3),
                         top=round(e.top_speed_kmh, 1), t=rnd(s.t[idx]), x=rnd(s.state[idx, 0]),
                         y=rnd(s.state[idx, 1]), v=rnd(s.state[idx, 3] * 3.6), p=rnd(s.progress[idx])))
    return dict(track=dict(name=tr.name, length=round(tr.length, 1), left=rnd(left[::step]), right=rnd(right[::step]),
                           start=rnd(p0), normal=rnd(tr.normal[0])),
                grip=res.config.grip, margin=res.config.margin, cars=cars)


def write_replay(res: RaceResult, path) -> FsPath:
    from .replay_html import TEMPLATE

    path = FsPath(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(replay_data(res), separators=(",", ":")).replace("</", "<\\/")
    path.write_text(TEMPLATE.replace("/*RACE_DATA*/", data), encoding="utf-8")
    return path
