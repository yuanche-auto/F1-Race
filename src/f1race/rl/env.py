"""Gymnasium environment for residual reinforcement learning on top of the L1 tracker (L5).

The base controller is the L1 feed-forward/feedback tracker on the nominal 5 %
plan. Every 50 ms the policy chooses two corrections, held until the next
decision:

* a steering offset (up to +-``steer_max`` rad) added to the tracker's command,
* a factor on the reference speed (between ``1 - speed_down`` and ``1 + speed_up``):
  the policy can learn where to keep a reserve and where it can go faster.

Episodes start at a random point of the plan (state of the plan there) and last
``episode_s`` seconds or until the car leaves the track or spins. The plant is
re-sampled every episode: parameters from the L2 perturbation ranges and,
optionally, the L3 disturbances (friction field, gusts, measurement noise).
Reward: progress along the track (per metre / 5), minus ``crash_penalty`` when the
episode ends off track or in a spin, minus a small penalty on the action.
"""
from __future__ import annotations

import math
from bisect import bisect_right
from dataclasses import replace

import gymnasium as gym
import numpy as np

from ..control.dyn_tracking import DynTracker, traction_limit
from ..dynamics import DynVehicle, tyre_forces
from ..experiments.l1 import dyn_reference
from ..experiments.l2 import PERTURBATION
from ..sim.dynamic import DynamicSim
from ..sim.stochastic import StochasticSim, sample_disturbances
from ..track import Track


class ResidualTracker(DynTracker):
    """L1 tracker (DynTracker.__call__, line by line) with an external steering offset and
    reference-speed factor; with zero action it reproduces DynTracker exactly."""

    d_offset = 0.0
    v_scale = 1.0

    def __call__(self, t, x):
        veh, dt = self.veh, self.dt
        px, py, psi, vx, vy, r, d = x
        c, s = np.cos(psi), np.sin(psi)
        pxp = px + 0.5 * dt * (vx * c - vy * s)
        pyp = py + 0.5 * dt * (vx * s + vy * c)
        sp, e, th, ff = self._project(pxp, pyp)
        course = psi + 0.5 * dt * r + np.arctan2(vy, max(vx, 1.0))
        dpsi = np.angle(np.exp(1j * (course - th)))
        d_cmd = (ff["delta"] - self.k_p * (e + self.x_la * np.sin(dpsi)) - self.k_r * (r - ff["r"])
                 + self.d_offset)
        a_r = -np.arctan((vy - veh.lr * r) / max(vx, 1.0))          # as DynTracker: ESC term and
        esc = np.sign(a_r) * max(abs(a_r) - self.esc_frac * veh.alpha_peak, 0.0)   # drive cut
        d_cmd = d_cmd + self.k_esc * esc
        th_f = np.arctan((vy + veh.lf * r) / max(vx, 1.0))
        lim = self.slip_frac * veh.alpha_peak
        d_cmd = float(np.clip(d_cmd, th_f - lim, th_f + lim))
        d_cmd = float(np.clip(d_cmd, -veh.delta_max, veh.delta_max))
        drate = (d_cmd - d) / dt
        k = self.v_scale
        v_ref = k * ff["vx"]
        a_ref = k * k * (ff["f"] - veh.resist(ff["vx"] ** 2)) / veh.m
        f = veh.resist(v_ref ** 2) + veh.m * a_ref * vx / max(v_ref, 1.0) + veh.m * self.kv * (v_ref - vx)
        d_next = d + float(np.clip(drate, -veh.delta_rate_max, veh.delta_rate_max)) * dt
        fd, fb = traction_limit(veh, x, d_next, max(f, 0.0), min(f, 0.0), self.eta_exec)
        if esc != 0.0:
            fd = 0.0
        self.last = (sp, e, dpsi, ff)
        return drate, fd, fb


class PointProjector:
    """``Track.project`` for a single point in plain Python floats (about 15 times faster
    than the vectorised version for one point; same formula and results). ``hint``:
    index of the nearest sample at the previous call; the nearest sample is searched in
    a window around it (the KD-tree is used without a hint or when the minimum lies at
    the window edge). Returns (s, n, nearest sample index)."""

    def __init__(self, track: Track):
        self.track = track
        track._tree()
        self._xl, self._yl, self._sl = track.x.tolist(), track.y.tolist(), track.s.tolist()
        self._txl, self._tyl = np.cos(track.psi).tolist(), np.sin(track.psi).tolist()
        self.N, self.length = len(track.s), track.length

    def project(self, px: float, py: float, hint: int | None = None) -> tuple[float, float, int]:
        x, y, N = self._xl, self._yl, self.N
        if hint is None:
            _, idx = self.track._tree().query((px, py))
            idx = int(idx)
        else:
            best, idx = math.inf, hint
            for k in range(hint - 8, hint + 25):
                j = k % N
                d2 = (x[j] - px) ** 2 + (y[j] - py) ** 2
                if d2 < best:
                    best, idx = d2, j
            if (idx - hint) % N in (N - 8, 24):
                return self.project(px, py, None)
        tx, ty, sl, ln = self._txl, self._tyl, self._sl, self.length
        best_d, s_out, n_out = math.inf, 0.0, 0.0
        for off in range(-3, 3):
            a = (idx + off) % N
            b = (a + 1) % N
            dx, dy = x[b] - x[a], y[b] - y[a]
            qx, qy = px - x[a], py - y[a]
            gx, gy = tx[b] - tx[a], ty[b] - ty[a]
            A = -(dx * gx + dy * gy)
            B = qx * gx + qy * gy - (dx * tx[a] + dy * ty[a])
            C = qx * tx[a] + qy * ty[a]
            lin = -C / (B if abs(B) > 1e-12 else -1e-12)
            if abs(A) > 1e-9 * max(abs(B), 1e-12):
                disc = math.sqrt(max(B * B - 4 * A * C, 0.0))
                r1, r2 = (-B + disc) / (2 * A), (-B - disc) / (2 * A)
                lam = r1 if abs(r1 - lin) < abs(r2 - lin) else r2
            else:
                lam = lin
            lam = min(max(lam, 0.0), 1.0)
            cx, cy = x[a] + lam * dx, y[a] + lam * dy
            ux, uy = (1 - lam) * tx[a] + lam * tx[b], (1 - lam) * ty[a] + lam * ty[b]
            un = math.hypot(ux, uy)
            ux, uy = ux / un, uy / un
            ex, ey = px - cx, py - cy
            d = math.hypot(ex, ey)
            if d < best_d:
                best_d = d
                s_out = (sl[a] + lam * math.hypot(dx, dy)) % ln
                n_out = ux * ey - uy * ex
        return s_out, n_out, idx



class FastMargins:
    """``DynamicSim.edge_margin`` with the scalar ``PointProjector`` and a warm-start
    index per point (same geometry and results, much faster in a stepping loop)."""

    def __init__(self, track: Track, veh: DynVehicle):
        self.tr, self.veh = track, veh
        self.proj = PointProjector(track)
        self.sp = track.s.tolist() + [track.length]
        self.wl = track.w_left.tolist() + [float(track.w_left[0])]
        self.wr = track.w_right.tolist() + [float(track.w_right[0])]
        self.reset()

    def reset(self):
        self.hint = [None, None, None]

    def _w(self, s):
        i = min(bisect_right(self.sp, s) - 1, len(self.sp) - 2)
        f = (s - self.sp[i]) / (self.sp[i + 1] - self.sp[i])
        return (self.wl[i] + f * (self.wl[i + 1] - self.wl[i]),
                self.wr[i] + f * (self.wr[i + 1] - self.wr[i]))

    def __call__(self, x):
        veh = self.veh
        c, s = math.cos(x[2]), math.sin(x[2])
        pts = ((x[0] - veh.lr * c, x[1] - veh.lr * s), (x[0] + veh.lf * c, x[1] + veh.lf * s), (x[0], x[1]))
        out = []
        for k, (px, py) in enumerate(pts):
            sk, nk, self.hint[k] = self.proj.project(float(px), float(py), self.hint[k])
            out.append((sk, nk))
        m = []
        for sk, nk in out[:2]:
            wl, wr = self._w(sk)
            m.append(min(wl - nk, wr + nk) - veh.half_width)
        return out[2][0], m[0], m[1]


def sample_plant(nominal: DynVehicle, rng: np.random.Generator) -> DynVehicle:
    f = {k: float(rng.uniform(*r)) for k, r in PERTURBATION.items()}
    return replace(nominal, mu_x=nominal.mu_x * f["mu"], mu_y=nominal.mu_y * f["mu"],
                   m=nominal.m * f["m"], ClA=nominal.ClA * f["ClA"], CdA=nominal.CdA * f["CdA"],
                   Iz=nominal.Iz * f["Iz"], B=nominal.B * f["B"],
                   aero_front=nominal.aero_front + f["aero_front"])


class RacingEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, plan, track: str = "Spielberg", randomize_plant: bool = True,
                 disturbances: bool = False, dt: float = 0.01, ctrl_every: int = 5,
                 episode_s: float = 20.0, steer_max: float = 0.02, speed_down: float = 0.15,
                 speed_up: float = 0.05, crash_penalty: float = 50.0, tracker: dict | None = None,
                 history: int = 1, seed: int | None = None):
        super().__init__()
        self.track = Track.load(track)
        self.nominal = DynVehicle()
        self.plan = plan
        self.path, self.ff = dyn_reference(self.track, self.nominal, plan)
        self.randomize_plant, self.disturbances = randomize_plant, disturbances
        self.dt, self.ctrl_every, self.episode_s = dt, ctrl_every, episode_s
        self.steer_max, self.speed_down, self.speed_up = steer_max, speed_down, speed_up
        self.crash_penalty = crash_penalty
        self.tracker_gains = tracker or {"k_p": 0.06, "x_la": 12.0, "k_r": 0.05, "kv": 8.0}
        self.rng = np.random.default_rng(seed)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), np.float32)
        # history > 1: the last `history` observations and the last action, so that the
        # policy can infer how this particular car responds (implicit identification)
        self.history = history
        dim = 19 if history == 1 else 19 * history + 2
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (dim,), np.float32)
        self._hist: list[np.ndarray] = []
        self._last_a = np.zeros(2, np.float32)
        self._preview = np.array([10.0, 25.0, 50.0, 100.0, 150.0])

    # -- helpers -------------------------------------------------------------------
    def _state_on_plan(self, i: int) -> np.ndarray:
        o, p = self.plan, self.path
        s = p.s[i]
        j = lambda a: float(np.interp(s, p.s, a, period=p.length))       # noqa: E731
        vx, vy, r, d = j(self.ff["vx"]), 0.0, j(self.ff["r"]), j(self.ff["delta"])
        vy = float(np.interp(s, np.linspace(0, p.length, len(o.vy), endpoint=False), o.vy, period=p.length))
        return np.array([p.x[i], p.y[i], p.psi[i] - np.arctan2(vy, vx), vx, vy, r, d])

    def _obs(self, x) -> np.ndarray:
        tr, veh, p = self.track, self.nominal, self.path
        sp, e, dpsi, ff = self.ctl.last
        _, _, _, vx, vy, r, d = x
        F = tyre_forces(veh, vx, vy, r, d, 0.0, 0.0)
        a_f, a_r = float(F["af"]), float(F["ar"])
        kap = np.interp((sp + self._preview) % p.length, p.s, p.kappa, period=p.length)
        vpre = np.interp((sp + self._preview[1:4]) % p.length, p.s, self.ff["vx"], period=p.length)
        _, mr, mf = self.margins(x)
        return np.array([e / 0.5, dpsi / 0.1, vx / 50.0, vy / 2.0, r, d / veh.delta_max,
                         (ff["vx"] - vx) / 5.0, a_f / veh.alpha_peak, a_r / veh.alpha_peak,
                         *(kap * 50.0), *(vpre / 50.0), min(mr, mf), self.ctl.v_scale - 1.0],
                        dtype=np.float32)

    # -- gym API -------------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = options or {}
        plant = options.get("plant")
        if plant is None:
            plant = sample_plant(self.nominal, self.rng) if self.randomize_plant else self.nominal
        i0 = options.get("start_index", int(self.rng.integers(len(self.path.s))))
        self.ctl = ResidualTracker(self.path, self.ff, self.nominal, self.dt, **self.tracker_gains)
        dist_seed = options.get("dist_seed", int(self.rng.integers(2**31)))
        if self.disturbances or options.get("disturbances", False):
            dist = sample_disturbances(self.track, dist_seed, t_max=max(self.episode_s, 150.0))
            self.sim = StochasticSim(self.track, plant, dist, self.dt)
            self.noise = dist.noise_sigma
            self.noise_rng = np.random.default_rng(dist.noise_seed)
        else:
            self.sim = DynamicSim(self.track, plant, self.dt)
            self.noise = None
        self.x = self._state_on_plan(i0)
        self.t = 0.0
        self.margins = FastMargins(self.track, plant)
        self.s_prev, _, _ = self.margins(self.x)
        self.progress = 0.0
        self.ctl(0.0, self.x)               # initialises the tracker (no noise draw: the noise
        o = self._obs(self.x)                # sequence then matches the L3 simulator's)
        self._hist = [o] * self.history
        self._last_a = np.zeros(2, np.float32)
        return self._stacked(), {}

    def _stacked(self) -> np.ndarray:
        if self.history == 1:
            return self._hist[-1]
        return np.concatenate(self._hist + [self._last_a]).astype(np.float32)

    def _measure(self, x):
        if self.noise is None:
            return x
        return x + self.noise * self.noise_rng.standard_normal(len(x))

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        self.ctl.d_offset = self.steer_max * a[0]
        self.ctl.v_scale = 1.0 + (self.speed_up if a[1] > 0 else self.speed_down) * a[1]
        tr = self.track
        ds_total, crashed, reason = 0.0, False, ""
        for _ in range(self.ctrl_every):
            u = self.sim.apply_limits(self.x, self.ctl(self.t, self._measure(self.x)))
            self.x = self.sim.step(self.x, u)
            self.t += self.dt
            s, mr, mf = self.margins(self.x)
            ds = (s - self.s_prev + 0.5 * tr.length) % tr.length - 0.5 * tr.length
            self.s_prev = s
            ds_total += ds
            beta = np.arctan2(self.x[4], max(self.x[3], 1e-3))
            if min(mr, mf) < 0.0:
                crashed, reason = True, "off track"
                break
            if abs(beta) > 0.5 or self.x[3] < 1.0:
                crashed, reason = True, "spin"
                break
        self.progress += ds_total
        reward = ds_total / 5.0 - 0.01 * float(a @ a)
        if crashed:
            reward -= self.crash_penalty
        truncated = (not crashed) and self.t >= self.episode_s
        info = {"progress": self.progress, "t": self.t, "reason": reason}
        if crashed:
            obs = np.zeros(self.observation_space.shape, np.float32)
        else:
            self._hist = self._hist[1:] + [self._obs(self.x)]
            self._last_a = a.astype(np.float32)
            obs = self._stacked()
        return obs, float(reward), crashed, truncated, info


def evaluate_lap(policy, env: RacingEnv, plant=None, disturbances: bool = False, dist_seed: int = 0,
                 t_max: float = 120.0) -> dict:
    """One flying lap from the start of the plan with a deterministic policy (``policy``:
    obs -> action, or None for the bare tracker)."""
    episode_s = env.episode_s
    env.episode_s = t_max
    obs, _ = env.reset(options={"plant": plant if plant is not None else env.nominal, "start_index": 0,
                                "disturbances": disturbances, "dist_seed": dist_seed})
    L = env.track.length
    done, info = False, {}
    while not done:
        a = np.zeros(2) if policy is None else policy(obs)
        obs, _, term, trunc, info = env.step(a)
        done = term or trunc or env.progress >= L
    env.episode_s = episode_s
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)       # back to the finish line
    return {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
            "stop_reason": "lap completed" if completed else (info.get("reason") or "time limit"),
            "distance": float(env.progress)}
