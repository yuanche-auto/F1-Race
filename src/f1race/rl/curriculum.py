"""L7 Part C: the training environment of groups P and F (docs/L7_protocol.md section 5, A7).

``L7TrainEnv`` is ``RacingEnv`` with

* **the L7 training distribution**: an environment without the L3 disturbances draws a
  steering lag per episode – none with probability 1/3, otherwise tau ~ U(10 ms, 80 ms) – and
  the policy sees the commanded angle; an environment with the disturbances has no lag (as
  in L5);
* **failure starts (group F)**: with probability ``p_fail_start`` (0.3) an episode starts 50–150 m
  before a position drawn uniformly from the failure pool – the last 200 failure positions
  (failures of the curriculum laps C at every checkpoint and crash points of training
  episodes), kept by the training process in a JSON file that every environment reads at
  reset. Otherwise (and in group P) the start is uniform over the lap, as in L5.

``step`` adds the lap position of the car to ``info`` (``"s"``), so that the training process
can collect crash points.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .env import RacingEnv

TAU_RANGE = (0.010, 0.080)
P_NO_LAG = 1.0 / 3.0
BACK_RANGE = (50.0, 150.0)


class L7TrainEnv(RacingEnv):
    def __init__(self, plan, pool_file: str | None = None, p_fail_start: float = 0.0, **kw):
        super().__init__(plan, **kw)
        self.pool_file = Path(pool_file) if pool_file else None
        self.p_fail_start = p_fail_start
        self.last_start = None
        # failure positions are centre-line arclengths (lap distance); the start index is a
        # point of the plan's path: its centre-line arclength, to pick the start
        self._path_track_s, _ = self.track.project(self.path.x, self.path.y)

    def _pool(self) -> list[float]:
        if self.pool_file is None:
            return []
        try:
            return [float(x) for x in json.loads(self.pool_file.read_text())]
        except (OSError, ValueError):
            return []

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        options = dict(options or {})
        if not self.disturbances and "steer_lag" not in options:
            options["steer_lag"] = 0.0 if self.rng.random() < P_NO_LAG else float(self.rng.uniform(*TAU_RANGE))
        if "start_index" not in options:
            pool = self._pool() if self.p_fail_start > 0 else []
            if pool and self.rng.random() < self.p_fail_start:
                p = pool[int(self.rng.integers(len(pool)))]
                L = self.track.length
                s0 = (p - float(self.rng.uniform(*BACK_RANGE))) % L
                d = np.abs((self._path_track_s - s0 + 0.5 * L) % L - 0.5 * L)
                options["start_index"] = int(np.argmin(d))
                self.last_start = ("failure", s0)
            else:
                self.last_start = ("uniform", None)
        obs, info = super().reset(seed=None, options=options)
        return obs, info

    def step(self, action):
        obs, rew, term, trunc, info = super().step(action)
        info["s"] = float(self.s_prev)
        return obs, rew, term, trunc, info


__all__ = ["L7TrainEnv", "TAU_RANGE", "P_NO_LAG", "BACK_RANGE"]
