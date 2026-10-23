"""Control expansion phase G runs (docs/CX_G_protocol.md, revision 0 with its clarifications), in WSL
(source ~/f1race-env.sh) – L9's environment and rule. One fresh process per job, pinned to one vCPU with taskset;
single-threaded; each job first times L9's fixed workload (calibration); the machine load is sampled every 60 s
(``run_l9.load_sampler``). Records in results/CX/G/<tag>/ (key ``job`` = stage|arm|case|rep).

    python scripts/run_cx_g.py run --tag test                         # explore -> plan -> the four primary arms
    python scripts/run_cx_g.py run --tag test --stages eval --arms rti_d40,rti_dc_d40,rti_dc20_d40   # S1
    python scripts/run_cx_g.py run --tag test --stages eval --arms nmpc_rti_d20,nmpc_rti_dc_d20      # S2
    python scripts/run_cx_g.py run --tag test --stages rl                                            # S3
    python scripts/run_cx_g.py run --tag test --stages eval --repeat 20                              # S4 (m3)
    python scripts/run_cx_g.py run --tag dev --sets NOM,GD                                           # section 9
    python scripts/run_cx_g.py jobs ...                                                               # job lists

Stages: ``explore`` – the exploration lap of `lag_identified` (`nmpc_reserve`, RTI, real time, no sensor delay),
one per case, shared by every arm (section 3); ``plan`` – identification and re-plan (untimed pit stop, at most
``--plan-slots`` = 3 at once, as L9); ``eval`` – the evaluated laps, in batches of ``--slots`` (8) jobs that put the
arms of a case side by side (the next batch starts when the whole batch is done); ``rl`` – S3.
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
sys.path.insert(0, str(ROOT / "scripts"))
OUT = ROOT / "results" / "CX" / "G"
FILES = {"explore": "explore.jsonl", "plan": "plans.jsonl", "eval": "eval.jsonl", "rl": "rl.jsonl"}


def key(j: dict) -> str:
    return "|".join(str(j.get(k, "")) for k in ("stage", "arm", "case", "rep"))


def run_job(j: dict, d: Path) -> dict:
    import numpy as np

    from f1race.experiments import cx_g, l9
    from f1race.track import Track
    from run_l9 import calibration

    case = cx_g.case_by_name(j["case"])
    cpu = sorted(os.sched_getaffinity(0))
    cal = calibration()
    t0 = time.time()
    name = f"cxg_{j['stage']}_{j.get('arm', '')}_{j['case']}_{os.getpid()}"
    if j["stage"] == "explore":
        ex = l9.explore_lap(case, Track.load("Spielberg"), "rti", "realtime", name)
        (d / "explore").mkdir(exist_ok=True)
        pickle.dump(ex, open(d / "explore" / f"{j['case']}.pkl", "wb"))
        out = l9.explore_summary(ex)
    elif j["stage"] == "plan":
        ex = pickle.load(open(d / "explore" / f"{j['case']}.pkl", "rb"))
        planned = l9.identify_and_plan(ex, "lag_identified")
        (d / "plans").mkdir(exist_ok=True)
        pickle.dump(planned, open(d / "plans" / f"{j['case']}.pkl", "wb"))
        out = {k: v for k, v in planned.items() if k not in ("plan", "vehicle")}
    elif j["stage"] == "rl":
        import torch
        from stable_baselines3 import SAC

        torch.set_num_threads(1)
        model = SAC.load(ROOT / "models" / "L5" / "B_s1_500k.zip", device="cpu")
        env = l9.make_env("angle", delay=0.02)
        out = l9.rl_lap(lambda obs: model.predict(obs, deterministic=True)[0], env, case, "realtime")
    else:
        kind = cx_g.ARMS[j["arm"]][0]
        planned = (pickle.load(open(d / "plans" / f"{j['case']}.pkl", "rb")) if kind == "lag_identified" else None)
        out = cx_g.drive_arm(case, j["arm"], planned, name)
    out.update({"job": key(j), **{k: j.get(k) for k in ("stage", "arm", "rep")}, "case": j["case"],
                "set": case["set"], "car": case["car"], "tau": case["tau"], "dist_seed": case["dist_seed"],
                "cpu": cpu, "calibration_s": cal, "wall_s": time.time() - t0})
    for b in Path(os.path.expanduser("~/l9_acados_build")).glob(f"{name}*"):      # generated code not kept
        subprocess.run(["rm", "-rf", str(b)], check=False)
    return out


def stage_jobs(a, stage: str) -> list[dict]:
    from f1race.experiments import cx_g

    names = [c["name"] for s in a.sets.split(",") for c in cx_g.cases(s)]
    rep = 1 if a.repeat else 0
    if a.repeat:
        names = names[:a.repeat]
    if stage in ("explore", "plan"):
        return [{"stage": stage, "case": n, "rep": 0, "batch": -1} for n in names]
    if stage == "rl":
        per = a.slots
        return [{"stage": "rl", "arm": "B_s1_500k_d20", "case": n, "rep": rep, "batch": b // per}
                for b, n in enumerate(names)]
    arms = a.arms.split(",")
    per = max(a.slots // len(arms), 1)                       # cases per batch; every arm of a case side by side
    jobs, batch = [], 0
    for b in range(0, len(names), per):
        jobs += [{"stage": "eval", "arm": arm, "case": n, "rep": rep, "batch": batch} for n in names[b:b + per]
                 for arm in arms]
        batch += 1
    return jobs


def execute(jobs: list[dict], rec: Path, slots: int, cpu0: int, batches: bool) -> None:
    """``run_l9.execute`` for this script: one process per job on its own vCPU; ``batches``: the next batch
    starts when the whole batch has finished."""
    done = set()
    if rec.exists():
        done = {json.loads(l)["job"] for l in rec.read_text().splitlines() if l.strip() and '"error"' not in l}
    queue = [j for j in jobs if key(j) not in done]
    print(f"{len(queue)} jobs -> {rec}", flush=True)
    free, running = list(range(cpu0, cpu0 + slots)), {}
    with open(rec, "a") as fo:
        while queue or running:
            start = not (batches and running)
            head = queue[0].get("batch", -1) if queue else None
            while start and queue and free and (not batches or queue[0].get("batch", -1) == head):
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
    from run_l9 import load_sampler

    d = OUT / a.tag
    d.mkdir(parents=True, exist_ok=True)
    stop, stage = threading.Event(), {"now": "start"}
    threading.Thread(target=load_sampler, args=(d / "load.jsonl", stop, stage), daemon=True).start()
    for s in a.stages.split(","):
        stage["now"] = s
        jobs = stage_jobs(a, s)
        if jobs:
            execute(jobs, d / (FILES[s] if not a.repeat else f"repeat_{FILES[s]}"),
                    a.plan_slots if s == "plan" else a.slots, a.cpu0, batches=s != "plan")
    stop.set()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["run", "jobs", "job"])
    ap.add_argument("--sets", default="GT1,GT2")
    ap.add_argument("--stages", default="explore,plan,eval")
    ap.add_argument("--arms", default="rti_d0,rti_dc_d0,rti_d20,rti_dc_d20")
    ap.add_argument("--repeat", type=int, default=0, help="S4: only the first n laps, recorded as a repeat")
    ap.add_argument("--tag", default="test")
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
        for s in a.stages.split(","):
            js = stage_jobs(a, s)
            print(f"{s}: {len(js)} jobs; first batch: " +
                  "; ".join(f"{j.get('arm', '')}/{j['case']}" for j in js if j.get("batch", -1) == js[0].get("batch", -1)))
    else:
        run(a)


if __name__ == "__main__":
    main()
