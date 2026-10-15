"""Robust re-planning of the minimum-time OCP for an identified car (preparation for L7).

``experiments.l4.replan`` solves once from each warm start (a nominal plan, then the
ideal plan) and gives up when both fail. In L6 that happened three times (round 1,
car 20 with the lag identified at 60 ms; round 3, cars 20 and 21 with the EKF), and
those cars were lost although a plan exists for every car (driving slowly is always
feasible). ``replan_robust`` first does exactly what ``l4.replan`` does – so every
re-plan that succeeded in L6 gives the same plan – and only if that fails goes on:

1. **continuation in the friction margin** (as in L1): solve the ideal problem
   (eta = 1, the largest feasible set) from each warm start, then lower eta in steps
   to the target, each step warm-started from the previous solution; a step that
   fails is halved (at most ``max_halvings`` times in a row);
2. **continuation in the parameters**: from the nominal car (whose plan is the warm
   start) to the identified car along a straight line in the parameter factors,
   at the target eta, with the same step halving;
3. **slower warm starts**: the warm starts with their speed scaled down (``speed_scales``).

Every solve is logged; the number of solves is capped (``max_solves``). The module
does not touch ``planning/ocp_dynamic.py`` (its hash is part of the L1 cache key).
"""
from __future__ import annotations

from dataclasses import fields, replace

import numpy as np

from ..dynamics import DynVehicle
from ..track import Track
from .ocp_dynamic import DynOCPResult, initial_guess_from_dynamic, solve_min_time_dynamic

STEPS = ("direct", "eta continuation", "parameter continuation", "slower warm start")


def scaled_guess(o: DynOCPResult, speed_scale: float) -> dict:
    """Warm start from a dynamic solution at a reduced speed: speeds and yaw rate scale
    with the speed, forces with its square (as ``initial_guess_from_kinematic``)."""
    g = initial_guess_from_dynamic(o)
    for k in ("vx", "vy", "r"):
        g[k] = g[k] * speed_scale
    for k in ("f_drive", "f_brake"):
        g[k] = g[k] * speed_scale**2
    return g


def blend_vehicle(a: DynVehicle, b: DynVehicle, lam: float) -> DynVehicle:
    """The car a + lam (b - a) in every numeric parameter (lam = 0: a, lam = 1: b)."""
    out = {}
    for f in fields(a):
        va, vb = getattr(a, f.name), getattr(b, f.name)
        if isinstance(va, (int, float)) and not isinstance(va, bool) and va != vb:
            out[f.name] = va + lam * (vb - va)
    return replace(a, **out)


def _path(start: float, end: float, n_steps: int) -> list[float]:
    return list(np.linspace(start, end, n_steps + 1)[1:])


def replan_robust(track: Track, veh: DynVehicle, warm, eta: float, nominal: DynVehicle | None = None,
                  eta_steps: int = 3, lam_steps: int = 2, max_halvings: int = 2,
                  speed_scales=(0.95, 0.9), max_solves: int = 16, max_iter: int = 1000,
                  steps=STEPS, solve=solve_min_time_dynamic):
    """Plan for ``veh`` at friction factor ``eta``. ``warm``: list of (name, plan) warm
    starts (as for ``l4.replan``); ``nominal``: the car the warm starts were planned for
    (needed for the parameter continuation). Returns (plan or None, log); each log entry
    has the step, the warm start, eta, the continuation parameter, status, iterations
    and lap time."""
    log: list[dict] = []

    def run(step: str, name: str, guess: dict, e: float, car: DynVehicle, lam: float = 1.0):
        if len(log) >= max_solves:
            return None
        o = solve(track, car, guess, eta=e, max_iter=max_iter)
        log.append({"step": step, "warm": name, "eta": float(e), "lambda": float(lam), "status": o.status,
                    "iterations": o.iterations, "lap_time": o.lap_time})
        return o if o.success else None

    def continuation(step: str, name: str, start: DynOCPResult, x0: float, path: list[float], solve_at):
        """Follow ``path`` from the solution ``start`` (at x0); a failed step is halved."""
        cur, cur_x = start, x0
        todo = list(path)
        halvings = 0
        while todo:
            x = todo[0]
            o = solve_at(step, name, cur, x)
            if o is not None:
                cur, cur_x, halvings = o, x, 0
                todo.pop(0)
                continue
            if len(log) >= max_solves or halvings >= max_halvings:
                return None
            todo.insert(0, 0.5 * (cur_x + x))           # halve the step towards the failed point
            halvings += 1
        return cur

    # 0. as l4.replan: one solve from each warm start
    if "direct" in steps:
        for name, w in warm:
            o = run("direct", name, initial_guess_from_dynamic(w), eta, veh)
            if o is not None:
                return o, log
    # 1. continuation in eta: the ideal problem first, then down to the target
    if "eta continuation" in steps:
        for name, w in warm:
            o1 = run("eta continuation", name, initial_guess_from_dynamic(w), 1.0, veh)
            if o1 is None:
                continue
            if eta >= 1.0:
                return o1, log
            o = continuation("eta continuation", name, o1, 1.0, _path(1.0, eta, eta_steps),
                             lambda st, nm, cur, e: run(st, nm, initial_guess_from_dynamic(cur), e, veh))
            if o is not None:
                return o, log
    # 2. continuation in the parameters, nominal -> identified, at the target eta
    if "parameter continuation" in steps and nominal is not None:
        for name, w in warm:
            o = continuation("parameter continuation", name, w, 0.0, _path(0.0, 1.0, lam_steps),
                             lambda st, nm, cur, lam: run(st, nm, initial_guess_from_dynamic(cur), eta,
                                                          blend_vehicle(nominal, veh, lam), lam))
            if o is not None:
                return o, log
    # 3. slower warm starts
    if "slower warm start" in steps:
        for sc in speed_scales:
            for name, w in warm:
                o = run("slower warm start", f"{name} x{sc}", scaled_guess(w, sc), eta, veh)
                if o is not None:
                    return o, log
    return None, log


__all__ = ["replan_robust", "blend_vehicle", "scaled_guess", "STEPS"]
