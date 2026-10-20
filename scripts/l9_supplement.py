"""L9 supplement to report_l9.py, written after the report had been generated (post hoc; no decision
depends on it):

* the exact signed-rank test of section 8 for paired lap times (real-time − offline, laps both
  complete), which report_l9.py at RUN_COMMIT a881aee does not compute: zero differences dropped,
  mid-ranks, the exact null distribution of W+ over all 2^n sign assignments by dynamic programming
  over the doubled ranks (identical to enumerating them; report_l7_partB enumerates and cannot reach n = 71);
* where the laps end (distance P10 / P50 / P90) per controller, implementation and run, descriptive;
* the re-run of the crashed real-time RL job (calibration, vCPU).

    python scripts/l9_supplement.py --dir results/L9
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

NL = chr(10)
KINDS = ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified")
IMPLS = ("ipopt", "ipopt_dc", "rti")
CRASHED = "rl|B_s3|policy|realtime|T2new_dist6024|0.0|realtime|0|plan"


def load(p: Path) -> list[dict]:
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()] if p.exists() else []


def last_by_job(p: Path) -> dict:
    out = {}
    for r in load(p):
        out[r["job"]] = r                                  # a re-run after an error record replaces it
    return {k: r for k, r in out.items() if "error" not in r}


def exact_signed_rank(diffs) -> dict:
    """Two-sided exact Wilcoxon signed-rank p, P(|W+ - n(n+1)/4| >= |W+_obs - n(n+1)/4|)."""
    from scipy.stats import rankdata

    d = np.asarray([x for x in diffs if x != 0], float)
    n = len(d)
    if n == 0:
        return {"n": 0, "W_plus": 0.0, "p": 1.0}
    r2 = np.rint(2 * rankdata(np.abs(d))).astype(int)    # doubled mid-ranks are integers
    w2_obs = int(r2[d > 0].sum())
    tot = int(r2.sum())
    dist = np.zeros(tot + 1, dtype=object)               # exact counts (Python ints)
    dist[0] = 1
    for v in r2:
        dist[v:] = dist[v:] + dist[:tot + 1 - v].copy()
    mu2 = tot / 2.0
    dev = abs(w2_obs - mu2)
    hits = sum(int(c) for w, c in enumerate(dist) if abs(w - mu2) >= dev - 1e-9)
    return {"n": n, "W_plus": w2_obs / 2.0, "p": min(1.0, hits / 2.0 ** n)}


def pct(x) -> str:
    x = np.asarray(x, float)
    return "–" if not len(x) else "/".join(f"{np.percentile(x, q):.0f}" for q in (10, 50, 90))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="results/L9")
    a = ap.parse_args()
    d = Path(a.dir)
    md = ["# L9 supplement (post hoc, after report_l9.py; no decision depends on it)", ""]

    # ---- signed-rank on paired lap times
    md += ["## Lap time real-time − offline on laps both complete (exact signed-rank, section 8)", "",
           "| controller | impl | n | mean diff s | median diff s | W+ | p |", "|---|---|---|---|---|---|---|"]
    off = {(r["kind"], r["impl"], r["case"]): r for r in last_by_job(d / "offline" / "model.jsonl").values()
           if r.get("stage") == "eval"}
    rt = {(r["kind"], r["impl"], r["case"]): r for r in last_by_job(d / "realtime" / "model.jsonl").values()
          if r.get("stage") == "eval"}
    n_tests = 0
    for k in KINDS:
        for i in ("ipopt", "rti"):
            both = [c for (kk, ii, c), r in rt.items() if kk == k and ii == i and r["completed"]
                    and off.get((k, i, c), {}).get("completed")]
            if not both:
                md.append(f"| {k} | {i} | 0 | – | – | – | – |")
                continue
            diffs = [rt[(k, i, c)]["lap_time"] - off[(k, i, c)]["lap_time"] for c in both]
            t = exact_signed_rank(diffs)
            n_tests += 1
            md.append(f"| {k} | {i} | {len(both)} | {np.mean(diffs):+.3f} | {np.median(diffs):+.3f} | "
                      f"{t['W_plus']:.1f} | {t['p']:.3g} |")
    for f, lab in (("rl.jsonl", "policy"), ("student.jsonl", "student")):
        ro = {(r["kind"], r["case"]): r for r in last_by_job(d / "offline" / f).values()}
        rr = {(r["kind"], r["case"]): r for r in last_by_job(d / "realtime" / f).values()}
        for p in sorted({k for k, _ in rr}):
            both = [c for (kk, c), r in rr.items() if kk == p and r["completed"] and ro.get((p, c), {}).get("completed")]
            if not both:
                continue
            diffs = [rr[(p, c)]["lap_time"] - ro[(p, c)]["lap_time"] for c in both]
            t = exact_signed_rank(diffs)
            n_tests += 1
            md.append(f"| {p} | {lab} | {len(both)} | {np.mean(diffs):+.3f} | {np.median(diffs):+.3f} | "
                      f"{t['W_plus']:.1f} | {t['p']:.3g} |")
    md += ["", f"{n_tests} comparisons; p uncorrected, no claim across them.", ""]

    # ---- where the laps end
    md += ["## Where the laps end: distance P10 / P50 / P90 in m (lap 4315 m; completed laps count as their length)", "",
           "| run | stage | controller | impl | completed | distance P10/P50/P90 |", "|---|---|---|---|---|---|"]
    for tag in ("offline", "realtime", "repeat", "delay", "offline_explore", "hold_last"):
        for f in ("explore.jsonl", "model.jsonl"):
            rs = list(last_by_job(d / tag / f).values())
            for k in ("explore",) + KINDS:
                for i in IMPLS:
                    g = [r for r in rs if r.get("kind") == k and r.get("impl") == i]
                    if g:
                        md.append(f"| {tag} | {f.split('.')[0]} | {k} | {i} | {sum(bool(r['completed']) for r in g)}/{len(g)} | "
                                  f"{pct([r['distance'] for r in g])} |")
    for tag in ("offline", "realtime", "delay"):
        rs = list(last_by_job(d / tag / "rl.jsonl").values())
        for p in sorted({r["kind"] for r in rs}):
            g = [r for r in rs if r["kind"] == p]
            md.append(f"| {tag} | rl | {p} | policy | {sum(bool(r['completed']) for r in g)}/{len(g)} | "
                      f"{pct([r['distance'] for r in g])} |")

    # ---- the question of section 1: does real-time RTI keep what offline IPOPT achieved?
    from report_l7_partB import mcnemar

    md += ["", "## Real-time RTI against offline IPOPT, same controller (exact McNemar; post hoc, uncorrected)", "",
           "| controller | real-time RTI | offline IPOPT | only RTI real-time : only IPOPT offline, p |", "|---|---|---|---|"]
    for k in KINDS:
        a_ = {c: bool(r["completed"]) for (kk, ii, c), r in rt.items() if kk == k and ii == "rti"}
        b_ = {c: bool(r["completed"]) for (kk, ii, c), r in off.items() if kk == k and ii == "ipopt"}
        x, y, p = mcnemar(a_, b_)
        md.append(f"| {k} | {sum(a_.values())}/{len(a_)} | {sum(b_.values())}/{len(b_)} | {x} : {y}, {p:.3g} |")

    # ---- machine load during every real-time run
    md += ["", "## Machine load during the real-time runs (one sample a minute)", "",
           "| run | stage | samples | WSL mean over vCPUs % (max) | Windows total % mean (max) |", "|---|---|---|---|---|"]
    for tag in ("realtime", "repeat", "delay", "offline_explore", "hold_last"):
        ld = load(d / tag / "load.jsonl")
        for stage in ("explore", "plan", "eval", "rl", "student"):
            s = [x for x in ld if x.get("stage") == stage]
            if s:
                wsl = [np.mean(list(x["wsl"].values())) for x in s]
                win = [float({k.lower(): v for k, v in x["windows"].items()}.get("_total", "nan"))
                       for x in s if isinstance(x.get("windows"), dict)]
                md.append(f"| {tag} | {stage} | {len(s)} | {np.mean(wsl):.1f} ({np.max(wsl):.1f}) | "
                          f"{np.nanmean(win):.1f} ({np.nanmax(win):.1f}) |")

    # ---- the re-run of the crashed job
    rl = load(d / "realtime" / "rl.jsonl")
    recs = [(n, r) for n, r in enumerate(rl) if r["job"] == CRASHED]
    md += ["", "## The crashed real-time RL job and its re-run", ""]
    for n, r in recs:
        if "error" in r:
            md.append(f"* line {n + 1}: error record `{r['error']}`")
        else:
            md.append(f"* line {n + 1}: re-run, vCPU {r.get('cpu')}, calibration {1e3 * r.get('calibration_s', math.nan):.1f} ms, "
                      f"latency mean / P99 / max {r['latency_ms'][0]:.2f} / {r['latency_ms'][2]:.2f} / {r['latency_ms'][3]:.2f} ms, "
                      f"wall {r.get('wall_s', math.nan):.1f} s (alone; the load sampler's minute had no sample in it)")
    out = d / "report" / "L9_supplement.md"
    out.write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {out}")


if __name__ == "__main__":
    main()
