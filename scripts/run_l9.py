"""L9 runs (docs/L9_protocol.md), in WSL (source ~/f1race-env.sh). One fresh process per job, pinned
to one vCPU with taskset; single-threaded.

    python scripts/run_l9.py run --mode offline  --slots 20                           # T1new + T2new
    python scripts/run_l9.py run --mode realtime --slots 8                            # interleaved batches
    python scripts/run_l9.py run --mode realtime --kinds lag_identified --delay 0.02 --tag delay
    python scripts/run_l9.py run --mode realtime --kinds lag_identified --explore-mode offline \\
        --plan-dir results/L9/offline --tag offline_explore
    python scripts/run_l9.py run --mode realtime --repeat 20 --tag repeat
    python scripts/run_l9.py jobs ...                                                 # print the job list

Stages of a run (so that no re-plan runs while a timed lap runs, l9-M1):
1. ``explore`` – the exploration laps of the identification controllers (``nmpc_reserve``, one per
   implementation, mode and case, shared by ``identified`` and ``lag_identified``); timed in the
   real-time mode;
2. ``plan`` – identification and re-plan (untimed pit stop; at most ``--plan-slots`` at once, 3:
   one re-plan needs ~7.4 GB and WSL has 31 GB);
3. ``eval`` – the evaluated laps of every controller; timed in the real-time mode.
Real-time stages: at most ``--slots`` (8) jobs at once, each on its own vCPU, the two implementations
interleaved in case order (every batch of 8 jobs: 4 laps x ipopt, rti); each job first times a fixed
workload (calibration) and records it with its vCPU; the machine load is sampled every 60 s (WSL
/proc/stat and the Windows processor counters) into load.jsonl.

Records in results/L9/<tag or mode>/: explore.jsonl, plans.jsonl, model.jsonl (key ``job``),
explore/*.pkl and plans/*.pkl (the logged exploration laps and the identified models and plans).
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import subprocess
import sys
import threading
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "L9"
KINDS = ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified")
IDENT = ("identified", "lag_identified")
IMPLS = ("ipopt", "rti")


def calibration() -> float:
    """The fixed workload of scripts/l9_wsl/core_speed.py, timed once (s)."""
    import numpy as np

    a = np.random.default_rng(0).standard_normal((120, 120))
    t0 = time.perf_counter()
    s = 0.0
    for _ in range(300):
        s += float(np.linalg.solve(a + 120 * np.eye(120), a[:, 0])[0])
    x = 0
    for i in range(400_000):
        x += i * i % 7
    return time.perf_counter() - t0


def key(j: dict) -> str:
    return "|".join(str(j[k]) for k in ("stage", "kind", "impl", "mode", "case", "delay", "explore_mode", "rep"))


def find_case(name: str) -> dict:
    from f1race.experiments.l9 import cases

    return next(c for s in ("T1new", "T2new", "V", "X") for c in cases(s) if c["name"] == name)


def run_job(j: dict, d: Path) -> dict:
    from f1race.experiments.l9 import drive, explore_lap, explore_summary, identify_and_plan
    from f1race.track import Track

    case = find_case(j["case"])
    cpu = sorted(os.sched_getaffinity(0))
    cal = calibration()
    t0 = time.time()
    name = f"{j['stage']}_{j['kind']}_{j['impl']}_{j['mode']}_{j['case']}_{os.getpid()}"
    em = j["explore_mode"]
    if j["stage"] == "explore":
        ex = explore_lap(case, Track.load("Spielberg"), j["impl"], j["mode"], name)
        (d / "explore").mkdir(exist_ok=True)
        pickle.dump(ex, open(d / "explore" / f"{j['impl']}_{j['mode']}_{j['case']}.pkl", "wb"))
        out = explore_summary(ex)
    elif j["stage"] == "plan":
        ex = pickle.load(open(d / "explore" / f"{j['impl']}_{em}_{j['case']}.pkl", "rb"))
        planned = identify_and_plan(ex, j["kind"])
        (d / "plans").mkdir(exist_ok=True)
        pickle.dump(planned, open(d / "plans" / f"{j['kind']}_{j['impl']}_{em}_{j['case']}.pkl", "wb"))
        out = {k: v for k, v in planned.items() if k not in ("plan", "vehicle")}
    else:
        planned = None
        if j["kind"] in IDENT:
            pd = Path(j["plan_dir"]) if j.get("plan_dir") else d
            planned = pickle.load(open(pd / "plans" / f"{j['kind']}_{j['impl']}_{em}_{j['case']}.pkl", "rb"))
        out = drive(case, j["kind"], j["impl"], j["mode"], delay=j["delay"], explore_mode=em, name=name,
                    planned=planned)
    out.update({"job": key(j), **{k: j[k] for k in ("stage", "kind", "impl", "mode", "delay", "explore_mode", "rep")},
                "case": j["case"], "set": case["set"], "car": case["car"], "tau": case["tau"],
                "dist_seed": case["dist_seed"], "cpu": cpu, "calibration_s": cal, "wall_s": time.time() - t0})
    for b in Path(os.path.expanduser("~/l9_acados_build")).glob(f"{name}*"):      # generated code not kept
        subprocess.run(["rm", "-rf", str(b)], check=False)
    return out


def stage_jobs(a, stage: str) -> list[dict]:
    from f1race.experiments.l9 import cases

    names = [c["name"] for s in a.sets.split(",") for c in cases(s)]
    if a.repeat:
        names = names[:a.repeat]
    kinds, impls = a.kinds.split(","), a.impls.split(",")
    em = a.explore_mode or a.mode
    base = {"mode": a.mode, "delay": a.delay, "explore_mode": em, "rep": 1 if a.repeat else 0,
            "plan_dir": a.plan_dir}
    if stage == "explore":
        if a.explore_mode or not any(k in IDENT for k in kinds):
            return []
        ks = ["explore"]
    elif stage == "plan":
        if a.explore_mode:
            return []
        ks = [k for k in kinds if k in IDENT]
    else:
        ks = kinds
    jobs = []
    for k in ks:
        if a.mode == "realtime" and stage != "plan":           # interleaved: 4 laps x implementations
            order = [(n, i) for b in range(0, len(names), 4) for n in names[b:b + 4] for i in impls]
        else:
            order = [(n, i) for i in impls for n in names]
        jobs += [{**base, "stage": stage, "kind": k, "impl": i, "case": n} for n, i in order]
    return jobs


def load_sampler(path: Path, stop: threading.Event, stage: dict) -> None:
    def proc_stat():
        out = {}
        for line in open("/proc/stat"):
            if line.startswith("cpu") and line[3].isdigit():
                f = line.split()
                vals = list(map(int, f[1:]))
                out[f[0]] = (sum(vals), vals[3] + vals[4])
        return out
    prev = proc_stat()
    while not stop.wait(60.0):
        cur = proc_stat()
        wsl = {c: round(100.0 * (1 - (cur[c][1] - prev[c][1]) / max(cur[c][0] - prev[c][0], 1)), 1) for c in cur}
        prev = cur
        try:
            ps = subprocess.run(["powershell.exe", "-NoProfile", "-Command",
                                 "(Get-Counter '\\Processor(*)\\% Processor Time').CounterSamples | "
                                 "ForEach-Object { $_.InstanceName + '=' + [int]$_.CookedValue }"],
                                capture_output=True, text=True, timeout=30).stdout.split()
            win = dict(x.split("=") for x in ps if "=" in x)
        except Exception as e:                       # noqa: BLE001
            win = {"error": str(e)}
        with open(path, "a") as fo:
            fo.write(json.dumps({"time": time.time(), "stage": stage["now"], "wsl": wsl, "windows": win}) + "\n")


def execute(jobs: list[dict], rec: Path, slots: int, cpu0: int, batches: bool = False) -> None:
    """Run jobs, one process each on its own vCPU. ``batches`` (timed stages of the real-time
    mode, l9-M1 b): strict batches of ``slots`` jobs – the next batch starts when the whole batch
    has finished, so that the two interleaved implementations always run side by side."""
    done = set()
    if rec.exists():
        done = {json.loads(l)["job"] for l in rec.read_text().splitlines() if l.strip() and '"error"' not in l}
    queue = [j for j in jobs if key(j) not in done]
    print(f"{len(queue)} jobs -> {rec}", flush=True)
    free, running = list(range(cpu0, cpu0 + slots)), {}
    with open(rec, "a") as fo:
        while queue or running:
            start = not (batches and running)                    # batches: a new batch only when all are done
            while start and queue and free:
                j, cpu = queue.pop(0), free.pop(0)
                cmd = ["taskset", "-c", str(cpu), sys.executable, str(Path(__file__)), "job", "--spec", json.dumps(j),
                       "--dir", str(rec.parent)]
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                running[p] = (j, cpu)
            time.sleep(0.5)
            for p in [p for p in running if p.poll() is not None]:
                j, cpu = running.pop(p)
                free.append(cpu)
                lines = [l for l in p.stdout.read().splitlines() if l.startswith("{")]
                r = json.loads(lines[-1]) if lines else {"job": key(j), "error": f"exit {p.returncode}"}
                fo.write(json.dumps(r, default=float) + "\n")
                fo.flush()
                print(r["job"], r.get("stop_reason") or r.get("failed") or "", "error" if "error" in r else "",
                      flush=True)


def run(a) -> None:
    d = OUT / (a.tag or a.mode)
    d.mkdir(parents=True, exist_ok=True)
    stop, stage = threading.Event(), {"now": "start"}
    if a.mode == "realtime":
        threading.Thread(target=load_sampler, args=(d / "load.jsonl", stop, stage), daemon=True).start()
    for s, slots in (("explore", a.slots), ("plan", a.plan_slots), ("eval", a.slots)):
        stage["now"] = s
        jobs = stage_jobs(a, s)
        if jobs:
            execute(jobs, d / {"explore": "explore.jsonl", "plan": "plans.jsonl", "eval": "model.jsonl"}[s], slots,
                    a.cpu0, batches=a.mode == "realtime" and s != "plan")
    stop.set()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["run", "jobs", "job"])
    ap.add_argument("--mode", default="offline", choices=["offline", "realtime"])
    ap.add_argument("--sets", default="T1new,T2new")
    ap.add_argument("--kinds", default=",".join(KINDS))
    ap.add_argument("--impls", default=",".join(IMPLS))
    ap.add_argument("--delay", type=float, default=0.0)
    ap.add_argument("--explore-mode", default=None, choices=[None, "offline", "realtime"])
    ap.add_argument("--plan-dir", default=None, help="plans of another run (with --explore-mode)")
    ap.add_argument("--repeat", type=int, default=0, help="only the first n laps, recorded as a repeat")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--slots", type=int, default=8)
    ap.add_argument("--plan-slots", type=int, default=3)
    ap.add_argument("--cpu0", type=int, default=0)
    ap.add_argument("--spec", default=None)
    ap.add_argument("--dir", default=None)
    a = ap.parse_args()
    if a.what == "job":
        j = json.loads(a.spec)
        try:
            r = run_job(j, Path(a.dir))
        except Exception:                            # noqa: BLE001
            import traceback

            r = {"job": key(j), "error": traceback.format_exc()}
        print(json.dumps(r, default=float))
    elif a.what == "jobs":
        for s in ("explore", "plan", "eval"):
            js = stage_jobs(a, s)
            print(f"{s}: {len(js)} jobs" + (f"; first 8: " + "; ".join(f"{j['kind']}/{j['impl']}/{j['case']}"
                                                                         for j in js[:8]) if js else ""))
    else:
        run(a)


if __name__ == "__main__":
    main()
