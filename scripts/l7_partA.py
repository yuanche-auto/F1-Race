"""L7 Part A: diagnosis of the nine L5 runs (docs/L7_protocol.md section 3).

    .venv-rl/Scripts/python scripts/l7_partA.py drive --ckpt A_s1      # re-drive the 81 L5 test laps, log every decision
    .venv-rl/Scripts/python scripts/l7_partA.py drive --all --workers 11
    .venv-rl/Scripts/python scripts/l7_partA.py analyze                 # A0-A4 -> results/L7/partA/partA.md, partA.json

Amendment A2: the L5 test cars 0-24 at tau = 0 and the L3 seeds 0-29 are the L7 test sets T1
(tau = 0) and T2; every finding is reported in two parts, (i) L5 test cars 25-50 (and the
validation outcomes for A1), (ii) the laps of T1 / T2 (``part_of``). Only (i) may change Part C.

Checkpoints: the final checkpoint of each of the nine runs (``results/L5/multiseed/<g>_s<k>.zip``,
the model saved after 600 000 decisions – identical to ``<run>_600k``, checked for A_s2) and
the two selected checkpoints that are not final (``models/L5/B_s1_500k``, ``C_s2_450k``).
The intermediate checkpoints were not restored from the old machine (protocol section 3).

Per decision (50 ms) the log holds: lap position, lateral error, course-angle error, speed,
plan speed, front / rear slip angle relative to the tyre peak (nominal model, as the policy
observes them), the friction usage of the true car under the applied command, the action
(steering offset, speed factor; in [-1, 1]), the reward, and the critic's min(Q1, Q2) of the
observation and the action.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "L7" / "partA"
L5 = ROOT / "results" / "L5" / "multiseed"
GROUP_STEER = {"A": "angle", "B": "angle", "C": "lateral"}
FINALS = [f"{g}_s{s}" for g in "ABC" for s in (1, 2, 3)]
SELECTED = ["B_s1_500k", "C_s2_450k"]                    # selected and not final (A_s2_600k = A_s2)
GAMMA = 0.99
WINDOW = 100.0                                           # [m] blind-spot window
BEFORE = 60                                              # decisions = 3 s before a failure
TRUNC = 300                                              # decisions before the lap end excluded in A4


PARTS = ("i", "ii")


def part_of(case) -> str:
    """Amendment A2: (i) L5 test cars 25-50, outside the L7 test sets; (ii) L5 test cars 0-24
    and the L3 seeds 0-29 = T1 at tau = 0 and T2."""
    kind, idx = case
    return "i" if kind == "test_car" and idx >= 25 else "ii"


def ckpt_path(name: str) -> Path:
    return (ROOT / "models" / "L5" / f"{name}.zip") if name in SELECTED else (L5 / f"{name}.zip")


def test_name(name: str) -> str:
    """The checkpoint's name in the L5 test.json."""
    return name if name in SELECTED else f"{name}_600k"


def test_cases():
    return [("test_car", i) for i in range(51)] + [("test_dist", s) for s in range(30)]


# ---------------------------------------------------------------- driving
def drive_lap(model, env, plant=None, disturbances=False, dist_seed=0, t_max=120.0) -> dict:
    """``rl.env.evaluate_lap`` with a log of every decision."""
    import torch

    from f1race.dynamics import ellipse_usage, tyre_forces

    episode_s = env.episode_s
    env.episode_s = t_max
    opts = {"plant": plant if plant is not None else env.nominal, "start_index": 0,
            "disturbances": disturbances, "dist_seed": dist_seed}
    obs, _ = env.reset(options=opts)
    true_car = env.sim.veh
    applied = {}
    orig = env.sim.apply_limits

    def apply_limits(x, u):
        out = orig(x, u)
        if "u" not in applied:
            applied["u"], applied["x"] = out, np.array(x, float)
        return out
    env.sim.apply_limits = apply_limits
    L = env.track.length
    log = {k: [] for k in ("pos", "e", "dpsi", "vx", "v_plan", "a_f", "a_r", "usage", "act", "reward",
                           "obs", "progress")}
    done, info = False, {}
    while not done:
        a = model.predict(obs, deterministic=True)[0]
        sp, e, dpsi, ff = env.ctl.last
        log["obs"].append(obs.copy())
        log["pos"].append(float(sp))
        log["e"].append(float(e))
        log["dpsi"].append(float(dpsi))
        log["vx"].append(float(env._meas[3]))
        log["v_plan"].append(float(ff["vx"]))
        o = obs[-21:-2] if len(obs) > 19 else obs           # the newest observation of the history
        log["a_f"].append(float(o[7]))
        log["a_r"].append(float(o[8]))
        log["act"].append(np.array(a, float))
        applied.clear()
        obs, rew, term, trunc, info = env.step(a)
        if "u" in applied:
            x, u = applied["x"], applied["u"]
            F = tyre_forces(true_car, x[3], x[4], x[5], x[6], u[1], u[2])
            log["usage"].append(float(np.max(ellipse_usage(true_car, F))))
        else:
            log["usage"].append(float("nan"))
        log["reward"].append(float(rew))
        log["progress"].append(float(env.progress))
        done = term or trunc or env.progress >= L
    env.episode_s = episode_s
    completed = env.progress >= L
    lap = env.t - (env.progress - L) / max(env.x[3], 1.0)
    out = {k: np.array(v) for k, v in log.items()}
    with torch.no_grad():                                 # the critic: min(Q1, Q2)(s, a)
        o_t = torch.as_tensor(out["obs"], dtype=torch.float32)
        a_t = torch.as_tensor(out["act"], dtype=torch.float32)
        q = torch.min(*model.critic(o_t, a_t)).numpy().ravel()
    out["q"] = q
    out.pop("obs")
    return {"completed": bool(completed), "lap_time": float(lap) if completed else float("nan"),
            "stop_reason": "lap completed" if completed else (info.get("reason") or "time limit"),
            "distance": float(env.progress), "log": out}


def drive(name: str) -> dict:
    import pickle

    from stable_baselines3 import SAC

    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.rl.env import RacingEnv

    plan = pickle.load(open(ROOT / "results" / "L1" / "l1_plans.pkl", "rb"))["plans"][0.95]
    model = SAC.load(ckpt_path(name), device="cpu")
    env = RacingEnv(plan, randomize_plant=False, history=8, steer_mode=GROUP_STEER[name[0]])
    cars = sample_plants(DynVehicle(), 50, seed=0)
    laps, arrays = [], {}
    t0 = time.time()
    for kind, idx in test_cases():
        if kind == "test_car":
            r = drive_lap(model, env, plant=cars[idx][1])
        else:
            r = drive_lap(model, env, disturbances=True, dist_seed=idx)
        lg = r.pop("log")
        r.update({"checkpoint": name, "case": [kind, idx], "decisions": len(lg["pos"])})
        laps.append(r)
        for k, v in lg.items():
            arrays.setdefault(k, []).append(v)
    OUT.mkdir(parents=True, exist_ok=True)
    lap_id = np.concatenate([np.full(len(v), i) for i, v in enumerate(arrays["pos"])])
    np.savez_compressed(OUT / f"laps_{name}.npz", lap_id=lap_id,
                        **{k: np.concatenate(v) for k, v in arrays.items()})
    (OUT / f"laps_{name}.json").write_text(json.dumps(laps, indent=1))
    print(f"{name}: {sum(r['completed'] for r in laps)}/{len(laps)} completed, {time.time() - t0:.0f} s", flush=True)
    return {"checkpoint": name, "laps": laps}


# ---------------------------------------------------------------- analysis
def blind_spot(positions, L: float, window: float = WINDOW):
    """The 100 m window with the most failures; a blind spot if it holds at least half of
    them and there are at least 3. Returns (start [m] or None, failures in it, failures)."""
    p = np.sort(np.mod(np.asarray(positions, float), L))
    n = len(p)
    if n == 0:
        return None, 0, 0
    ext = np.concatenate([p, p + L])
    best, start = 0, None
    for i in range(n):
        k = int(np.searchsorted(ext, ext[i] + window, side="right") - i)
        if k > best:
            best, start = k, float(ext[i])
    is_spot = n >= 3 and best >= 0.5 * n
    return (start if is_spot else None), best, n


def returns(r: np.ndarray, gamma: float = GAMMA) -> np.ndarray:
    g = np.zeros_like(r)
    acc = 0.0
    for i in range(len(r) - 1, -1, -1):
        acc = r[i] + gamma * acc
        g[i] = acc
    return g


def analyze() -> None:
    from f1race.track import Track

    L = Track.load("Spielberg").length
    test = json.loads((L5 / "test.json").read_text())
    val = json.loads((L5 / "validation.json").read_text())
    res: dict = {"A0": {}, "A1": {}, "A2": {}, "A3": {}, "A4": {}}
    md = ["# L7 Part A – diagnosis of the L5 runs (results)", ""]

    # A0: reproduction of the L5 test laps
    md += ["## A0 reproduction of the L5 test laps", "",
           "| checkpoint | laps | same outcome | failures: same position (±5 m) | completed here / in L5 |",
           "|---|---|---|---|---|"]
    redriven = {}
    for name in FINALS + SELECTED:
        f = OUT / f"laps_{name}.json"
        if not f.exists():
            continue
        laps = json.loads(f.read_text())
        redriven[name] = laps
        old = {tuple(r["case"]): r for r in test if r["checkpoint"] == test_name(name)}
        same = pos = nf = 0
        for r in laps:
            o = old[tuple(r["case"])]
            same += r["completed"] == o["completed"]
            if not r["completed"] and not o["completed"]:
                nf += 1
                pos += abs(r["distance"] - o["distance"]) <= 5.0
        nc, oc = sum(r["completed"] for r in laps), sum(o["completed"] for o in old.values())
        res["A0"][name] = {"laps": len(laps), "same_outcome": same, "failures_both": nf, "same_position": pos,
                           "completed_here": nc, "completed_L5": oc}
        md.append(f"| {name} | {len(laps)} | {same} | {pos} of {nf} | {nc} / {oc} |")

    # A1: blind spots over training (validation outcomes) and on the test laps
    md += ["", "## A1 blind spots", "",
           "Validation laps (20 per checkpoint): failures, and the 100 m window holding at least half of "
           "them (start, failures in it) if there are at least 3.", "",
           "| run | " + " | ".join(f"{k}k" for k in range(50, 601, 50)) + " |",
           "|---|" + "---|" * 12]
    for name in FINALS:
        cells = []
        res["A1"][name] = []
        for k in range(50, 601, 50):
            rs = [r for r in val if r["checkpoint"] == f"{name}_{k}k"]
            fails = [r["distance"] for r in rs if not r["completed"]]
            st, inside, n = blind_spot(fails, L)
            res["A1"][name].append({"k": k, "failures": n, "spot": st, "in_spot": inside})
            cells.append(f"{n}" + (f" @{st:.0f} ({inside})" if st is not None else ""))
        md.append(f"| {name} | " + " | ".join(cells) + " |")
    md += ["", "Test laps (L5 test.json), split by amendment A2: (i) L5 test cars 25–50 (not in the L7 test "
           "sets), (ii) L5 test cars 0–24 and L3 seeds 0–29 (= T1 at τ = 0 and T2):", "",
           "| checkpoint | part | laps | failures | blind spot (start, failures in it) | other failure positions |",
           "|---|---|---|---|---|---|"]
    res["A1"]["test"] = {}
    for name in FINALS + SELECTED:
        for part in PARTS:
            rs = [r for r in test if r["checkpoint"] == test_name(name) and part_of(r["case"]) == part]
            fails = [r["distance"] for r in rs if not r["completed"]]
            st, inside, n = blind_spot(fails, L)
            res["A1"]["test"].setdefault(name, {})[part] = {"laps": len(rs), "failures": n, "spot": st,
                                                             "in_spot": inside, "positions": sorted(fails)}
            rest = [f for f in fails if st is None or not (0 <= (f - st) % L <= WINDOW)]
            md.append(f"| {name} | ({part}) | {len(rs)} | {n} | "
                      + (f"{st:.0f}–{st + WINDOW:.0f} m ({inside})" if st is not None else "–")
                      + f" | {', '.join(f'{x:.0f}' for x in sorted(rest)) or '–'} |")

    # A2-A4 on the re-driven laps, per part (amendment A2)
    md += ["", "## A2 states before the failures, A3 saturated actions, A4 critic error", "",
           "Per part (i) / (ii) of amendment A2; failing laps and the comparison laps are taken from the same "
           "part. Failing: the last 3 s (60 decisions) before each failure. Window: the same 100 m before the "
           "failure position on the laps of the same checkpoint and part that complete. Elsewhere: all other "
           "decisions of completed laps. Medians (A2), fractions (A3), Q − G (A4; G the discounted return, "
           "γ = 0.99, truncated at the lap end, so decisions in the last 300 of a completed lap are excluded).", ""]
    hdr = ("| checkpoint | part | failures | v / v_plan: fail / window | lateral error [m] | rear slip / peak | "
           "usage | speed factor | steer offset | saturated (fail / window / elsewhere) | Q − G (fail / elsewhere) |")
    md += [hdr, "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, laps in redriven.items():
        z = np.load(OUT / f"laps_{name}.npz")
        lid = z["lap_id"]
        act = z["act"]
        speed_factor = np.where(act[:, 1] > 0, 1 + 0.05 * act[:, 1], 1 + 0.15 * act[:, 1])
        sat = np.any(np.abs(act) >= 0.99, axis=1)
        ratio = z["vx"] / np.maximum(z["v_plan"], 1.0)
        err = np.zeros(len(lid))
        for i in range(len(laps)):
            m = np.flatnonzero(lid == i)
            err[m] = z["q"][m] - returns(z["reward"][m])
        for part in PARTS:
            ids = [i for i, r in enumerate(laps) if part_of(r["case"]) == part]
            keep_else = np.zeros(len(lid), bool)
            fail_idx, win_idx = [], []
            for i in ids:
                r = laps[i]
                m = np.flatnonzero(lid == i)
                if r["completed"]:
                    keep_else[m[:-TRUNC]] = True
                else:
                    fail_idx.append(m[-BEFORE:])
                    pf = r["distance"] % L
                    for j in ids:
                        if laps[j]["completed"]:
                            mm = np.flatnonzero(lid == j)
                            d = (pf - z["pos"][mm]) % L
                            win_idx.append(mm[d <= WINDOW])
            key = f"{name} ({part})"
            if not fail_idx:
                q_else = float(np.median(err[keep_else])) if keep_else.any() else float("nan")
                s_else = float(sat[keep_else].mean()) if keep_else.any() else float("nan")
                res["A3"][key] = {"saturated": [None, None, s_else]}
                res["A4"][key] = {"q_minus_g": [None, q_else]}
                md.append(f"| {name} | ({part}) | 0 | – | – | – | – | – | – | – / – / {s_else:.3f} | – / {q_else:.1f} |")
                continue
            fi = np.concatenate(fail_idx)
            wi = np.unique(np.concatenate(win_idx)) if win_idx else np.array([], int)
            else_idx = np.flatnonzero(keep_else & ~np.isin(np.arange(len(lid)), wi))
            med = lambda a, ix: float(np.nanmedian(a[ix])) if len(ix) else float("nan")   # noqa: E731
            mean = lambda a, ix: float(a[ix].mean()) if len(ix) else float("nan")       # noqa: E731
            pct = lambda a, ix: float(np.percentile(a[ix], 90)) if len(ix) else float("nan")  # noqa: E731
            row = {"failures": len(fail_idx), "window_decisions": int(len(wi)),
                   "v_ratio": [med(ratio, fi), med(ratio, wi)], "e": [med(z["e"], fi), med(z["e"], wi)],
                   "rear_slip": [med(np.abs(z["a_r"]), fi), med(np.abs(z["a_r"]), wi)],
                   "usage": [med(z["usage"], fi), med(z["usage"], wi)],
                   "speed_factor": [med(speed_factor, fi), med(speed_factor, wi)],
                   "steer_offset": [med(act[:, 0], fi), med(act[:, 0], wi)],
                   "saturated": [mean(sat, fi), mean(sat, wi), mean(sat, else_idx)],
                   "sat_component_fail": [mean(np.abs(act[:, 0]) >= 0.99, fi), mean(np.abs(act[:, 1]) >= 0.99, fi)],
                   "q_minus_g": [med(err, fi), med(err, else_idx)],
                   "q_minus_g_q90": [pct(err, fi), pct(err, else_idx)]}
            res["A2"][key] = {k: row[k] for k in ("failures", "window_decisions", "v_ratio", "e", "rear_slip",
                                                   "usage", "speed_factor", "steer_offset")}
            res["A3"][key] = {"saturated": row["saturated"], "component_fail": row["sat_component_fail"]}
            res["A4"][key] = {"q_minus_g": row["q_minus_g"], "q_minus_g_q90": row["q_minus_g_q90"]}
            f2 = lambda p: f"{p[0]:.2f} / {p[1]:.2f}"             # noqa: E731
            md.append(f"| {name} | ({part}) | {row['failures']} | {f2(row['v_ratio'])} | {f2(row['e'])} | "
                      f"{f2(row['rear_slip'])} | {f2(row['usage'])} | {f2(row['speed_factor'])} | "
                      f"{f2(row['steer_offset'])} | {row['saturated'][0]:.2f} / {row['saturated'][1]:.2f} / "
                      f"{row['saturated'][2]:.3f} | {row['q_minus_g'][0]:.1f} / {row['q_minus_g'][1]:.1f} |")
    (OUT / "partA.json").write_text(json.dumps(res, indent=1))
    (OUT / "partA.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["drive", "analyze"])
    ap.add_argument("--ckpt")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--workers", type=int, default=11)
    a = ap.parse_args()
    if a.what == "analyze":
        analyze()
    elif a.all:
        from multiprocessing import get_context

        names = [n for n in FINALS + SELECTED if not (OUT / f"laps_{n}.npz").exists()]
        with get_context("spawn").Pool(min(a.workers, len(names)), maxtasksperchild=1) as pool:
            for _ in pool.imap_unordered(drive, names):
                pass
    else:
        drive(a.ckpt)


if __name__ == "__main__":
    main()
