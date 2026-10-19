"""L9 Part C (docs/L9_protocol.md section 6): a neural approximation of the NMPC by DAgger.

* Expert: the acados SQP-RTI ``nmpc`` (nominal model, 5 % plan, eta 0.95) in the offline mode,
  following the visited states (its warm start tracks them).
* Student: an MLP (2 x 256, tanh output) from the RL policy's observation without the history (the
  19 features of ``RacingEnv._obs``) to the NMPC command (steering rate / delta_rate_max, forces /
  20 kN), held for 50 ms with the NMPC's traction control at every simulator step.
* Data: episodes of 40 s from a uniform start on the 5 % plan, cars from the L2 ranges, half of the
  episodes with the L3 disturbances (as the L5 training distribution); 10 DAgger iterations of
  20 000 decisions; iteration 1 driven by the expert, iterations 2-10 by the student (beta 1, then
  0); the expert labels every visited state (a state where its QP fails gets no label – the
  command there is the shifted previous plan, not the expert's; counted); the MLP is refit on the aggregate after every
  iteration (Adam 1e-3, 20 epochs, batch 256, MSE).
* ``student_lap``: one lap of the student on a case in the offline or real-time mode (its inference
  timed, the previous command held, as for the RL policies – A1).
"""
from __future__ import annotations

import math
import time

import numpy as np

from ..dynamics import DynVehicle
from .l9 import DT, PERIOD, T_MAX, RealTimeMPCController, make_mpc, plans, timing_summary

SCALE = np.array([DynVehicle().delta_rate_max, 2e4, 2e4])
ITERS, DECISIONS, EPISODE_S = 10, 20_000, 40.0


def _env(seed: int, history: int = 1):
    from ..rl.env import RacingEnv

    return RacingEnv(plans()[0.95], randomize_plant=True, history=history, episode_s=EPISODE_S, steer_mode="angle",
                     observe="command", seed=seed)


def _features(env) -> np.ndarray:
    """The 19 observation features at the current measurement (the tracker is called only to update
    its projection; its command is not used)."""
    env.ctl(env.t, env._meas)
    return env._obs(env._meas)


def _apply(env, cmd: np.ndarray, expert=None) -> tuple[bool, str]:
    """One decision (5 simulator steps) with the command held and the traction control; the expert
    controller (if given) is called at every step to follow the states. Returns (crashed, reason)."""
    from ..control.dyn_tracking import traction_limit

    veh, tr = env.nominal, env.track
    for _ in range(env.ctrl_every):
        meas = env._measure(env._view(env.x))
        if expert is not None:
            expert(env.t, meas)
        drate = float(np.clip(cmd[0], -veh.delta_rate_max, veh.delta_rate_max))
        d_next = meas[6] + drate * env.dt
        fd, fb = traction_limit(veh, meas, d_next, max(float(cmd[1]), 0.0), min(float(cmd[2]), 0.0), 0.99)
        u = env.sim.apply_limits(env.x, (drate, fd, fb))
        env.x = env.sim.step(env.x, u)
        env.t += env.dt
        s, mr, mf = env.margins(env.x)
        ds = (s - env.s_prev + 0.5 * tr.length) % tr.length - 0.5 * tr.length
        env.s_prev = s
        env.progress += ds
        beta = np.arctan2(env.x[4], max(env.x[3], 1e-3))
        if min(mr, mf) < 0.0:
            return True, "off track"
        if abs(beta) > 0.5 or env.x[3] < 1.0:
            return True, "spin"
    return False, ""


def _expert_mpc(name: str):
    from ..track import Track

    return make_mpc("rti", Track.load("Spielberg"), DynVehicle(), plans()[0.95], 0.95, name=name)


def _expert(mpc):
    """A fresh controller around the (once-built) expert solver, its warm start reset."""
    mpc.initialised, mpc._last_plan = False, None
    return RealTimeMPCController(mpc, DT, "offline")


class MLP:
    def __init__(self, seed: int):
        import torch

        torch.manual_seed(seed)
        self.net = torch.nn.Sequential(torch.nn.Linear(19, 256), torch.nn.ReLU(), torch.nn.Linear(256, 256),
                                       torch.nn.ReLU(), torch.nn.Linear(256, 3), torch.nn.Tanh())

    def fit(self, X: np.ndarray, Y: np.ndarray, seed: int, epochs: int = 20, batch: int = 256) -> float:
        import torch

        g = torch.Generator().manual_seed(seed)
        x, y = torch.as_tensor(X, dtype=torch.float32), torch.as_tensor(Y, dtype=torch.float32)
        opt = torch.optim.Adam(self.net.parameters(), lr=1e-3)
        loss = torch.tensor(0.0)
        for _ in range(epochs):
            perm = torch.randperm(len(x), generator=g)
            for i in range(0, len(x), batch):
                idx = perm[i:i + batch]
                loss = torch.nn.functional.mse_loss(self.net(x[idx]), y[idx])
                opt.zero_grad()
                loss.backward()
                opt.step()
        return float(loss)

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        import torch

        with torch.no_grad():
            return self.net(torch.as_tensor(obs, dtype=torch.float32)).numpy() * SCALE


def train(seed: int, name: str, iters: int = ITERS, decisions: int = DECISIONS, log=print) -> tuple[MLP, list]:
    import torch

    torch.set_num_threads(1)
    rng = np.random.default_rng(90_000 + seed)
    student = MLP(seed)
    mpc = _expert_mpc(name)
    X, Y, hist = [], [], []
    for it in range(iters):
        n, eps, crashes, excluded = 0, 0, 0, 0
        while n < decisions:
            env = _env(int(rng.integers(2**31)))
            env.reset(options={"disturbances": eps % 2 == 1, "steer_lag": 0.0})
            expert = _expert(mpc)
            done = False
            while not done and n < decisions:
                o = _features(env)
                expert(env.t, env._meas)                  # the expert's command at this state
                ue = np.array(expert.u, float)
                if expert.calls and expert.calls[-1]["fallback"]:
                    excluded += 1                         # a failed QP: no expert label (the shifted plan)
                else:
                    X.append(o)
                    Y.append(np.clip(ue / SCALE, -1.0, 1.0))
                cmd = ue if it == 0 else student(o)
                crashed, _ = _apply(env, cmd, expert)
                n += 1
                done = crashed or env.t >= EPISODE_S
                crashes += crashed
            eps += 1
        loss = student.fit(np.array(X), np.array(Y), seed + 1000 * it)
        hist.append({"iteration": it + 1, "data": len(X), "episodes": eps, "crashes": crashes,
                     "labels_excluded_failed_qp": excluded, "loss": loss})
        log(hist[-1])
    _cleanup(name)
    return student, hist


def _cleanup(name: str) -> None:
    import os
    import shutil
    from pathlib import Path

    for d in Path(os.path.expanduser("~/l9_acados_build")).glob(f"{name}*"):
        shutil.rmtree(d, ignore_errors=True)


def student_lap(student, case: dict, mode: str, t_max: float = T_MAX) -> dict:
    """One flying lap of the student under the rule of section 3 (no plan: the previous command
    held while it computes; its inference timed in the real-time mode)."""
    from .l9 import make_env

    env = make_env("angle")
    env.episode_s = t_max
    opts = {"plant": case["plant"], "start_index": 0, "disturbances": case["dist_seed"] is not None,
            "dist_seed": case["dist_seed"] or 0}
    if case["tau"] > 0:
        opts["steer_lag"] = case["tau"]
    env.reset(options=opts)
    L = env.track.length
    cmd = np.zeros(3)
    calls, reason = [], ""
    while True:
        o = _features(env)
        t0 = time.perf_counter()
        new = student(o)
        c = time.perf_counter() - t0
        k = 0 if mode == "offline" else math.ceil(c / env.dt - 1e-9)
        if mode == "realtime":
            calls.append({"t": env.t, "latency": c, "busy": c, "miss": c > PERIOD, "fallback": False, "status": 0})
        crashed = False
        for i in range(env.ctrl_every):                  # previous command for k steps, then the new one
            if i == k:
                cmd = new
            crashed, reason = _apply_one(env, cmd)
            if crashed:
                break
        if k >= env.ctrl_every:
            cmd = new
        if crashed or env.progress >= L or env.t >= t_max:
            break
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)
    out = {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
           "stop_reason": "lap completed" if completed else (reason or "time limit"), "distance": float(env.progress)}
    if calls:
        out.update(timing_summary(calls))
    return out


def _apply_one(env, cmd) -> tuple[bool, str]:
    saved = env.ctrl_every
    env.ctrl_every = 1
    try:
        return _apply(env, cmd)
    finally:
        env.ctrl_every = saved


__all__ = ["MLP", "train", "student_lap", "SCALE", "ITERS", "DECISIONS"]
