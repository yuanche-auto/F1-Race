"""L7 Part B runs: calibration set K, risk estimates (a)-(c), CBF-QP alpha, acceptance on A
(docs/L7_protocol.md section 4, amendment A3). Run with .venv-rl (needs stable-baselines3).

    python scripts/run_l7.py explore --set K          # exploration laps (A3.7) -> results/L7/<set>/explore/
    python scripts/run_l7.py nofilter --set K         # B_s1_500k without a filter -> nofilter.jsonl
    python scripts/run_l7.py kfilter                  # B_s1_500k + NMPC filter (default) on K, predictions recorded
    python scripts/run_l7.py calibrate                # (b) eta_k, b_k; (c) residual models, q_k -> results/L7/K/estimates.json
    python scripts/run_l7.py cbf-alpha                # alpha rule of section 4.1 on K
    python scripts/run_l7.py estimate-a --set A       # (a) per case from its exploration lap
    python scripts/run_l7.py accept --estimate b      # B_s1_500k + NMPC filter with the estimate on A
    python scripts/run_l7.py report                   # tables -> results/L7/partB_K.md

Every job runs in a fresh process; single-threaded BLAS. Result runs from a worktree pinned
to one commit (results/L7/RUN_COMMIT).
"""
from __future__ import annotations

import argparse
import json
import os
import pickle
import sys
import time
import traceback

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from multiprocessing import get_context
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "L7"
POLICY = "B_s1_500k"
ALPHAS = (1.0, 2.0, 4.0, 8.0)
NL = chr(10)


def policy_fn():
    from stable_baselines3 import SAC

    model = SAC.load(ROOT / "models" / "L5" / POLICY, device="cpu")
    return lambda obs: model.predict(obs, deterministic=True)[0]


def case_by_name(name: str) -> dict:
    from f1race.experiments.l7 import cases

    st = name.split("_")[0]
    return next(c for c in cases(st) if c["name"] == name)


def outcome(case: dict, r: dict, **extra) -> dict:
    d = r.get("decisions", {})
    out = {"case": case["name"], "set": case["set"], "car": case["car"], "tau": case["tau"],
           "dist_seed": case["dist_seed"], "completed": r["completed"], "lap_time": r["lap_time"],
           "stop_reason": r["stop_reason"], "distance": r["distance"]}
    if len(d.get("t", [])):
        out.update({"decisions": int(len(d["t"])), "interventions": int(d["changed"].sum()),
                    "intervention_rate": float(d["changed"].mean()), "filter_failures": int(d["fail"].sum()),
                    "slack_decisions": int(((d["slack"] > 1e-3) | (d["limit_slack"] > 1e-3)).sum()),
                    "time_ms": [float(np.mean(d["time"]) * 1e3), float(np.percentile(d["time"], 95) * 1e3),
                                float(np.percentile(d["time"], 99) * 1e3), float(np.max(d["time"]) * 1e3)]})
    out.update(extra)
    return out


# ---------------------------------------------------------------- jobs (each in a fresh process)
def job_explore(args):
    name, outdir = args
    from f1race.experiments.l7 import explore_lap
    from f1race.track import Track

    case = case_by_name(name)
    t0 = time.time()
    r = explore_lap(case, Track.load("Spielberg"))
    d = Path(outdir)
    d.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(d / f"{name}.npz", t=r.t, state=r.state, inputs=r.inputs, usage=r.usage,
                        noisy=r.noisy)
    return {"case": name, "completed": r.completed, "stop_reason": r.stop_reason, "distance": r.distance,
            "wall_s": time.time() - t0}


def job_lap(args):
    """One lap of the policy on a case, with a filter spec: None, {"kind": "nmpc", ...} or
    {"kind": "cbf", ...}; records predictions and samples when asked."""
    name, spec, record_dir = args
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l7 import CBFFilter, NMPCFilter, band_c, make_env, plan, rollout_errors, run_lap
    from f1race.track import Track

    case = case_by_name(name)
    track = Track.load("Spielberg")
    env = make_env()
    pol = policy_fn()
    filt = None
    nom = DynVehicle()
    if spec is not None and spec["kind"] == "nmpc":
        model, tau_m, band = nom, None, None
        if spec.get("estimate") == "a":
            est = pickle.load(open(spec["a_file"], "rb"))
            model, tau_m = est["vehicle"], max(est["tau"], 1e-4)
        if spec.get("estimate") == "c":
            res = pickle.load(open(spec["c_file"], "rb"))
            band = band_c(res, np.asarray(spec["q"], float))
        filt = NMPCFilter(track, model, plan(0.95), tau_model=tau_m, eta=spec.get("eta", 0.95),
                          backoff=spec.get("backoff"), band=band)
    elif spec is not None and spec["kind"] == "cbf":
        filt = CBFFilter(track, nom, alpha=spec["alpha"], margin=spec["margin"], eta=spec["eta"])
    t0 = time.time()
    r = run_lap(pol, env, case, filt)
    out = outcome(case, r, wall_s=time.time() - t0, filter=None if spec is None else
                  {k: spec[k] for k in ("kind", "estimate", "alpha") if k in spec})
    if record_dir and filt is not None and getattr(filt, "predictions", None):
        d = Path(record_dir)
        d.mkdir(parents=True, exist_ok=True)
        # A4: the errors of the filter's model rolled out under the applied inputs, one row per
        # (decision p, stage k); p indexes the recorded predictions (same filter call)
        smp = rollout_errors(track, filt.mpc, filt.states, r["log"])
        P = filt.predictions
        np.savez_compressed(d / f"{name}.npz", t=np.array([p[0] for p in P]), X=np.stack([p[1] for p in P]),
                            U=np.stack([p[2] for p in P]), b=np.stack([p[3] for p in P]),
                            log_t=r["log"].t, log_state=r["log"].state, log_usage=r["log"].usage,
                            log_inputs=r["log"].inputs, xf=np.stack([q[3] for q in filt.states]),
                            **{f"smp_{k}": v for k, v in smp.items() if isinstance(v, np.ndarray)},
                            smp_t_end=smp["t_end"], smp_completed=smp["completed"],
                            dec_t=r["decisions"]["t"], dec_changed=r["decisions"]["changed"],
                            dec_s=r["decisions"]["s"])
    return out


def job_estimate_a(args):
    name, explore_dir, outdir = args
    from f1race.experiments.l7 import estimate_a
    from f1race.track import Track

    z = np.load(Path(explore_dir) / f"{name}.npz")
    run = SimpleNamespace(t=z["t"], state=z["state"], inputs=z["inputs"], usage=z["usage"], noisy=bool(z["noisy"]))
    t0 = time.time()
    est = estimate_a(Track.load("Spielberg"), run)
    d = Path(outdir)
    d.mkdir(parents=True, exist_ok=True)
    pickle.dump(est, open(d / f"{name}.pkl", "wb"))
    return {"case": name, "tau_hat": est["tau"], "theta": est["theta"], "b10": est["backoff"][10],
            "b20": est["backoff"][20], "starts": est["starts"], "wall_s": time.time() - t0}


def job_residual(args):
    name, explore_dir, outdir = args
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l7 import residual_for

    z = np.load(Path(explore_dir) / f"{name}.npz")
    run = SimpleNamespace(t=z["t"], state=z["state"], inputs=z["inputs"], usage=z["usage"])
    res = residual_for(run, DynVehicle())
    d = Path(outdir)
    d.mkdir(parents=True, exist_ok=True)
    pickle.dump(res, open(d / f"{name}.pkl", "wb"))
    return {"case": name, "residual": res.summary()}


def safe(args):
    fn, a = args
    try:
        return fn(a)
    except Exception:                                       # noqa: BLE001
        return {"case": a[0], "error": traceback.format_exc()}


def pool_run(fn, jobs, path: Path, workers: int):
    """Run jobs in fresh processes, append the records to ``path`` (jsonl); skip finished cases."""
    path.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if path.exists():
        done = {json.loads(l)["case"] for l in path.read_text().splitlines() if l.strip() and "error" not in l}
    todo = [j for j in jobs if j[0] not in done]
    print(f"{len(todo)} jobs -> {path}", flush=True)
    if not todo:
        return
    with get_context("spawn").Pool(min(workers, len(todo)), maxtasksperchild=1) as pool, \
            open(path, "a", encoding="utf-8") as fo:
        for rec in pool.imap_unordered(safe, [(fn, j) for j in todo]):
            fo.write(json.dumps(rec, default=float) + NL)
            fo.flush()
            print({k: v for k, v in rec.items() if k != "residual"}, flush=True)


def load_jsonl(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


# ---------------------------------------------------------------- steps
def calibrate() -> dict:
    """(b) from the K rollout errors of the default filter (A3.4, A4), (c) q_k per stage (A3.5)
    from the same errors and the same cut, sigma(k) along the filter's prediction at that
    decision."""
    from f1race.experiments.l3 import calibrate as l3_calibrate
    from f1race.experiments.l7 import N, cases, conformal_qk, sigma_k

    rec_dir = OUT / "K" / "kfilter"
    samples, ks, ratios = [], [], []
    for c in cases("K"):
        z = np.load(rec_dir / f"{c['name']}.npz")
        smp = {k[4:]: z[k] for k in z.files if k.startswith("smp_")}
        smp["t_end"], smp["completed"] = float(smp["t_end"]), bool(smp["completed"])
        ok = (smp["t"] + N * 0.05 <= smp["t_end"] - 2.0) if not smp["completed"] else np.ones(len(smp["k"]), bool)
        cut = {k: (v[ok] if isinstance(v, np.ndarray) and v.shape[:1] == ok.shape else v) for k, v in smp.items()}
        cut["completed"] = True                              # the cut is already applied
        samples.append(cut)
        res = pickle.load(open(OUT / "K" / "residual" / f"{c['name']}.pkl", "rb"))
        # sigma(k) along each recorded prediction; the samples are ordered prediction by prediction
        sig_all = []
        for X, U in zip(z["X"], z["U"]):
            sig_all.append(sigma_k(res, X[:, :7].astype(float), U.astype(float)))
        sig_all = np.array(sig_all)                          # (n_pred, N + 1)
        s = sig_all[smp["p"].astype(int), smp["k"].astype(int)]
        m = ok & (smp["k"] >= 1)
        ks.append(smp["k"][m])
        ratios.append(smp["e_n"][m] / np.maximum(s[m], 1e-12))
    cfg = SimpleNamespace(N=N, u_threshold=0.85)
    b = l3_calibrate(samples, cfg, eps=0.05, drop_before_failure=0.0)
    ks, ratios = np.concatenate(ks).astype(int), np.concatenate(ratios)
    q = conformal_qk(ks, ratios)
    est = {"b": {"eta": b["eta"], "backoff": b["backoff"], "samples_per_stage": b["samples_per_stage"]},
           "c": {"q": q.tolist(), "n_per_stage": [int((ks == k).sum()) for k in range(N + 1)]}}
    (OUT / "K" / "estimates.json").write_text(json.dumps(est, indent=1))
    print(json.dumps({"b_eta_10": b["eta"][10], "b_10": b["backoff"][10], "b_20": b["backoff"][20],
                      "q_10": q[10], "q_20": q[20]}), flush=True)
    return est


def coverage_report(est_name: str, set_name: str = "A") -> dict:
    """Acceptance (section 4.2, A3.8, A4): coverage of the rollout errors by the band b_k the
    filter used at that decision, k = 10 and 20; predictions reaching into the last 2 s of a
    failed lap dropped; also without the cut (a3-m3) and near the limit (rollout usage at
    stage k >= 0.85)."""
    from f1race.experiments.l7 import N, cases

    d = OUT / set_name / f"accept_{est_name}"
    out = {}
    for k in (10, 20):
        tot = {"n": 0, "inside": 0, "n_all": 0, "inside_all": 0, "n_near": 0, "inside_near": 0, "dropped": 0}
        for c in cases(set_name):
            f = d / f"{c['name']}.npz"
            if not f.exists():
                continue
            z = np.load(f)
            kk, p, e = z["smp_k"].astype(int), z["smp_p"].astype(int), z["smp_e_n"]
            band = z["b"][p, kk]
            m = kk == k
            ok = np.ones(len(kk), bool)
            if not bool(z["smp_completed"]):
                ok = z["smp_t"] + N * 0.05 <= float(z["smp_t_end"]) - 2.0
            ins = np.abs(e) <= band
            near = z["smp_u_pred"] >= 0.85
            tot["n"] += int((m & ok).sum())
            tot["inside"] += int((m & ok & ins).sum())
            tot["n_all"] += int(m.sum())
            tot["inside_all"] += int((m & ins).sum())
            tot["n_near"] += int((m & ok & near).sum())
            tot["inside_near"] += int((m & ok & near & ins).sum())
            tot["dropped"] += int((m & ~ok).sum())
        tot["coverage"] = tot["inside"] / max(tot["n"], 1)
        tot["coverage_all"] = tot["inside_all"] / max(tot["n_all"], 1)
        tot["coverage_near"] = tot["inside_near"] / max(tot["n_near"], 1)
        tot["passes"] = 0.90 <= tot["coverage"] <= 0.98
        out[str(k)] = tot
    out["accepted"] = all(out[str(k)]["passes"] for k in (10, 20))
    return out


def acceptance_report(set_name: str = "A") -> dict:
    """Section 4.2 with A3.8, A4, A5: per estimate the coverage (criterion) and, reported only,
    the completions with the filter (and without, B_s1_500k), the intervention rate, the
    solver failures, the share of decisions using slack and the share of decisions whose
    back-offs leave a corridor narrower than the car at some stage (track width minus both
    back-offs minus the car's width < 0; it leaves out the constraint's 0.3 m margin and the
    chord, so it underestimates the narrow corridors); then the main estimate (the first of
    (a), (b), (c) whose coverage passes)."""
    from f1race.dynamics import DynVehicle
    from f1race.track import Track

    track, veh = Track.load("Spielberg"), DynVehicle()
    base = OUT / set_name
    nf = load_jsonl(base / "nofilter.jsonl")
    out = {"nofilter_completed": sum(r["completed"] for r in nf), "laps": len(nf), "estimates": {}}
    md = [f"# L7 acceptance test on {set_name}", "",
          f"B_s1_500k without a filter: {out['nofilter_completed']} of {len(nf)} laps completed.", "",
          "| estimate | coverage k=10 | coverage k=20 | passes | without the 2 s cut k=10 / 20 (dropped) | near the limit k=10 / 20 | "
          "completed (of laps) | intervention rate | fallbacks | slack decisions | corridor < car width |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    main = None
    for e in ("a", "b", "c"):
        laps = load_jsonl(base / f"accept_{e}.jsonl")
        if not laps:
            continue
        cov = coverage_report(e, set_name)
        dec = sum(r.get("decisions", 0) for r in laps)
        narrow = n_dec = 0
        for c in laps:
            f = base / f"accept_{e}" / f"{c['case']}.npz"
            if not f.exists():
                continue
            z = np.load(f)
            S, B = z["X"][:, 1:, 0].astype(float), z["b"][:, 1:].astype(float)
            W = track.interp(track.w_left, S % track.length) + track.interp(track.w_right, S % track.length)
            narrow += int(np.any(W - 2 * B - 2 * veh.half_width < 0, axis=1).sum())
            n_dec += len(S)
        rec = {"coverage": cov, "completed": sum(r["completed"] for r in laps), "laps": len(laps),
               "intervention_rate": sum(r.get("interventions", 0) for r in laps) / max(dec, 1),
               "fallbacks": sum(r.get("filter_failures", 0) for r in laps),
               "slack_share": sum(r.get("slack_decisions", 0) for r in laps) / max(dec, 1),
               "narrow_share": narrow / max(n_dec, 1)}
        out["estimates"][e] = rec
        if main is None and cov["accepted"]:
            main = e
        c10, c20 = cov["10"], cov["20"]
        md.append(f"| ({e}) | {c10['coverage']:.3f} ({c10['n']}) | {c20['coverage']:.3f} ({c20['n']}) | "
                  f"{'yes' if cov['accepted'] else 'no'} | {c10['coverage_all']:.3f} / {c20['coverage_all']:.3f} "
                  f"({c10['dropped']} / {c20['dropped']}) | {c10['coverage_near']:.3f} ({c10['n_near']}) / "
                  f"{c20['coverage_near']:.3f} ({c20['n_near']}) | {rec['completed']} ({rec['laps']}) | "
                  f"{rec['intervention_rate']:.3f} | {rec['fallbacks']} | {rec['slack_share']:.3f} | {rec['narrow_share']:.3f} |")
    out["main_estimate"] = main
    md += ["", f"Main estimate (first of (a), (b), (c) that passes 90–98 % at k = 10 and 20): "
           f"**{main if main else 'none – the filters are evaluated with (b), labelled \'risk estimate not accepted\''}**."]
    (base / "acceptance.json").write_text(json.dumps(out, indent=1))
    (base / "acceptance.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {base / 'acceptance.md'}", flush=True)
    return out


def main() -> None:
    from f1race.experiments.l7 import cases

    ap = argparse.ArgumentParser()
    ap.add_argument("what")
    ap.add_argument("--set", default="K")
    ap.add_argument("--estimate", default="b")
    ap.add_argument("--workers", type=int, default=20)
    ap.add_argument("--cases", default=None, help="comma-separated case names (default: the whole set)")
    a = ap.parse_args()
    names = [c["name"] for c in cases(a.set)] if a.what not in ("kfilter", "calibrate", "cbf-alpha") \
        else [c["name"] for c in cases("K")]
    if a.cases:
        names = a.cases.split(",")
    base = OUT / a.set
    if a.what == "explore":
        pool_run(job_explore, [(n, str(base / "explore")) for n in names], base / "explore.jsonl", a.workers)
        if a.set == "K":
            pool_run(job_residual, [(n, str(base / "explore"), str(base / "residual")) for n in names],
                     base / "residual.jsonl", a.workers)
    elif a.what == "nofilter":
        pool_run(job_lap, [(n, None, None) for n in names], base / "nofilter.jsonl", a.workers)
    elif a.what == "kfilter":
        spec = {"kind": "nmpc", "estimate": "default", "eta": 0.95}
        pool_run(job_lap, [(n, spec, str(OUT / "K" / "kfilter")) for n in names], OUT / "K" / "kfilter.jsonl",
                 a.workers)
    elif a.what == "calibrate":
        calibrate()
    elif a.what == "cbf-alpha":
        est = json.loads((OUT / "K" / "estimates.json").read_text())
        nf = {r["case"]: r for r in load_jsonl(OUT / "K" / "nofilter.jsonl")}
        ok = [n for n in names if nf[n]["completed"]]
        for al in ALPHAS:
            spec = {"kind": "cbf", "alpha": al, "margin": est["b"]["backoff"][10], "eta": est["b"]["eta"][10]}
            pool_run(job_lap, [(n, spec, None) for n in ok], OUT / "K" / f"cbf_alpha{al:g}.jsonl", a.workers)
    elif a.what == "estimate-a":
        pool_run(job_estimate_a, [(n, str(base / "explore"), str(base / "est_a")) for n in names],
                 base / "est_a.jsonl", a.workers)
        if a.set == "A":
            pool_run(job_residual, [(n, str(base / "explore"), str(base / "residual")) for n in names],
                     base / "residual.jsonl", a.workers)
    elif a.what == "accept":
        est = json.loads((OUT / "K" / "estimates.json").read_text())
        jobs = []
        for n in names:
            if a.estimate == "a":
                spec = {"kind": "nmpc", "estimate": "a", "a_file": str(base / "est_a" / f"{n}.pkl"), "eta": 0.95}
                spec["backoff"] = pickle.load(open(base / "est_a" / f"{n}.pkl", "rb"))["backoff"]
            elif a.estimate == "b":
                spec = {"kind": "nmpc", "estimate": "b", "eta": est["b"]["eta"], "backoff": est["b"]["backoff"]}
            else:
                spec = {"kind": "nmpc", "estimate": "c", "c_file": str(base / "residual" / f"{n}.pkl"),
                        "q": est["c"]["q"], "eta": 0.95}
            jobs.append((n, spec, str(base / f"accept_{a.estimate}")))
        pool_run(job_lap, jobs, base / f"accept_{a.estimate}.jsonl", a.workers)
        rep = coverage_report(a.estimate, a.set)
        (base / f"accept_{a.estimate}_coverage.json").write_text(json.dumps(rep, indent=1))
        print(json.dumps(rep), flush=True)
    elif a.what == "acceptance-report":
        acceptance_report(a.set)
    else:
        raise SystemExit(f"unknown step {a.what}")


if __name__ == "__main__":
    main()
