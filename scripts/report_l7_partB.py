"""L7 Part B report: tables and the pre-registered test of H-B from the T1 + T2 runs
(docs/L7_protocol.md sections 4.3, 6, 7; amendments A5, A7, A8; review pb-m2, pe-M1).

    .venv-rl/Scripts/python scripts/report_l7_partB.py --none <dir> --filtered <dir> --out results/L7/partB_report

``--none``: the partB directory with rl_none.jsonl and baselines.jsonl; ``--filtered``: the one
with rl_cbf.jsonl, rl_main.jsonl and dec/.

Fixed before any result was read (2026-10-16):
* H-B: over the nine final L5 checkpoints, the paired numbers of completed T1 + T2 laps with
  and without the main filter, Wilcoxon signed-rank (two-sided, zero differences dropped);
  H-B holds if p is below its Holm level and the median difference is positive. The Holm
  level depends on S1 (Part C); here p is reported with both possible levels (0.025 / 0.05).
* False interventions (A8): per policy 200 snapshots drawn uniformly with seed 0 from all
  (all if fewer); rate with a Clopper-Pearson interval and per eighth of the lap.
* Attribution of the failures of filtered laps (pb-m2), the last 60 decisions (3 s) before
  the failure: "fallback run" if they contain at least three consecutive fallbacks,
  otherwise "intervention" if the filter changed at least one command, otherwise "neither";
  and whether the same lap completes without the filter.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
FINALS = [f"{g}_s{s}" for g in "ABC" for s in (1, 2, 3)]
POLICIES = FINALS + ["B_s1_500k", "tracker"]
NL = chr(10)


def load(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def ci(k: int, n: int) -> tuple[float, float]:
    from scipy.stats import binomtest

    if n == 0:
        return float("nan"), float("nan")
    c = binomtest(k, n).proportion_ci(method="exact")
    return float(c.low), float(c.high)


def mcnemar(a: dict, b: dict) -> tuple[int, int, float]:
    from scipy.stats import binomtest

    keys = sorted(set(a) & set(b))
    x = sum(a[k] and not b[k] for k in keys)
    y = sum(b[k] and not a[k] for k in keys)
    return x, y, float(binomtest(min(x, y), x + y).pvalue) if x + y else 1.0


def attribute(dec: dict) -> str:
    fb = np.asarray(dec["fail"], bool)[-60:]
    ch = np.asarray(dec["changed"], bool)[-60:]
    run, best = 0, 0
    for f in fb:
        run = run + 1 if f else 0
        best = max(best, run)
    if best >= 3:
        return "fallback run"
    return "intervention" if ch.any() else "neither"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--none", required=True)
    ap.add_argument("--filtered", required=True)
    ap.add_argument("--out", default=str(ROOT / "results" / "L7" / "partB_report"))
    a = ap.parse_args()
    dn, df, out = Path(a.none), Path(a.filtered), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    from f1race.track import Track

    L = Track.load("Spielberg").length
    rl = {c: load((dn if c == "none" else df) / f"rl_{c}.jsonl") for c in ("none", "main", "cbf")}
    errors = {c: sum("error" in r for r in v) for c, v in rl.items()}
    res = {"errors": errors}
    by = {c: {(r["policy"], r["case"]): r for r in v if "error" not in r} for c, v in rl.items()}
    md = ["# L7 Part B – evaluation on T1 + T2", "", f"Error records: {errors}.", "",
          "## Completion and lap time", "",
          "| policy | condition | T1 (75) | T2 (30) | T1 + T2 (95 % interval) | mean lap of completed |",
          "|---|---|---|---|---|---|"]
    comp = {}
    for p in POLICIES:
        for c in ("none", "main", "cbf"):
            rs = [r for (pp, _), r in by[c].items() if pp == p]
            if not rs:
                continue
            t1 = sum(r["completed"] for r in rs if r["set"] == "T1")
            t2 = sum(r["completed"] for r in rs if r["set"] == "T2")
            n = len(rs)
            lo, hi = ci(t1 + t2, n)
            laps = [r["lap_time"] for r in rs if r["completed"]]
            comp[(p, c)] = {r["case"]: r["completed"] for r in rs}
            md.append(f"| {p} | {c} | {t1} | {t2} | {t1 + t2} / {n} ({lo:.2f}–{hi:.2f}) | "
                      f"{np.mean(laps):.2f} |" if laps else f"| {p} | {c} | {t1} | {t2} | {t1 + t2} / {n} | – |")
    res["completion"] = {f"{p}|{c}": sum(v.values()) for (p, c), v in comp.items()}

    # H-B (pre-registered)
    from scipy.stats import wilcoxon

    diffs = [sum(comp[(p, "main")].values()) - sum(comp[(p, "none")].values()) for p in FINALS
             if (p, "main") in comp and (p, "none") in comp]
    nz = [d for d in diffs if d != 0]
    p_hb = float(wilcoxon(nz).pvalue) if len(nz) else 1.0
    res["H-B"] = {"differences": diffs, "nonzero": len(nz), "median": float(np.median(diffs)) if diffs else None,
                  "p": p_hb, "holds_at_0.025": bool(p_hb < 0.025 and np.median(diffs) > 0),
                  "holds_at_0.05": bool(p_hb < 0.05 and np.median(diffs) > 0)}
    md += ["", "## H-B (pre-registered): nine final checkpoints, completed T1 + T2 laps with minus without the main filter", "",
           f"Differences: {diffs}; non-zero {len(nz)}; median {res['H-B']['median']}; Wilcoxon two-sided p = {p_hb:.4g}. "
           f"Holds at 0.025: {res['H-B']['holds_at_0.025']}; at 0.05: {res['H-B']['holds_at_0.05']} "
           "(the level follows Holm with S1, Part C).", "",
           "Per policy, exact McNemar (completed only with the filter : only without):", "",
           "| policy | main | p | cbf | p |", "|---|---|---|---|---|"]
    for p in POLICIES:
        row = [p]
        for c in ("main", "cbf"):
            if (p, c) in comp and (p, "none") in comp:
                x, y, pv = mcnemar(comp[(p, c)], comp[(p, "none")])
                row += [f"{x} : {y}", f"{pv:.3g}"]
            else:
                row += ["–", "–"]
        md.append("| " + " | ".join(row) + " |")

    # interventions, fallbacks, slack, time, lap-time cost
    md += ["", "## The filters in the loop", "",
           "| policy | condition | interventions per decision | fallbacks (share) | decisions with slack | ms mean / P99 | "
           "lap-time cost on laps both complete (mean, n) |", "|---|---|---|---|---|---|---|"]
    for p in POLICIES:
        for c in ("main", "cbf"):
            rs = [r for (pp, _), r in by[c].items() if pp == p and "decisions" in r]
            if not rs:
                continue
            dec = sum(r["decisions"] for r in rs)
            cost = [r["lap_time"] - by["none"][(p, r["case"])]["lap_time"] for r in rs
                    if r["completed"] and (p, r["case"]) in by["none"] and by["none"][(p, r["case"])]["completed"]]
            tm = np.average([r["time_ms"][0] for r in rs], weights=[r["decisions"] for r in rs])
            p99 = max(r["time_ms"][2] for r in rs)
            md.append(f"| {p} | {c} | {sum(r['interventions'] for r in rs) / dec:.3f} | "
                      f"{sum(r['filter_failures'] for r in rs)} ({sum(r['filter_failures'] for r in rs) / dec:.3f}) | "
                      f"{sum(r['slack_decisions'] for r in rs) / dec:.3f} | {tm:.1f} / {p99:.0f} | "
                      f"{(np.mean(cost) if cost else float('nan')):+.2f} s ({len(cost)}) |")

    # false interventions (A8)
    md += ["", "## False interventions (A8): 200 per policy drawn with seed 0; rate per eighth of the lap", "",
           "| policy | condition | snapshots | sampled | false (95 % interval) | per eighth (false / sampled) |",
           "|---|---|---|---|---|---|"]
    res["false"] = {}
    for p in POLICIES:
        for c in ("main", "cbf"):
            snaps = [s for (pp, _), r in by[c].items() if pp == p for s in r.get("false_interventions", [])]
            if not snaps:
                continue
            rng = np.random.default_rng(0)
            idx = rng.choice(len(snaps), size=min(200, len(snaps)), replace=False)
            smp = [snaps[i] for i in sorted(idx)]
            k = sum(not s["crashed"] for s in smp)
            lo, hi = ci(k, len(smp))
            per = []
            for j in range(8):
                g = [s for s in smp if int(round(s["at"] / (L / 8))) == j]
                per.append(f"{sum(not s['crashed'] for s in g)}/{len(g)}")
            res["false"][f"{p}|{c}"] = {"snapshots": len(snaps), "sampled": len(smp), "false": k}
            md.append(f"| {p} | {c} | {len(snaps)} | {len(smp)} | {k / len(smp):.2f} ({lo:.2f}–{hi:.2f}) | {' '.join(per)} |")

    # attribution of filtered failures (pb-m2)
    md += ["", "## Failures with a filter (pb-m2): the last 3 s", "",
           "| condition | failures | fallback run | intervention | neither | of them completed without the filter |",
           "|---|---|---|---|---|---|"]
    for c in ("main", "cbf"):
        counts = {"fallback run": 0, "intervention": 0, "neither": 0}
        alone = 0
        n = 0
        for (p, case), r in by[c].items():
            if r["completed"]:
                continue
            f = df / "dec" / c / f"{p}__{case}.npz"
            if not f.exists():
                continue
            z = np.load(f)
            counts[attribute({"fail": z["fail"], "changed": z["changed"]})] += 1
            n += 1
            alone += bool(by["none"].get((p, case), {}).get("completed"))
        md.append(f"| {c} | {n} | {counts['fallback run']} | {counts['intervention']} | {counts['neither']} | {alone} |")

    # baselines
    bl = load(dn / "baselines.jsonl")
    md += ["", "## Model-based baselines (no filter, A8)", "", "| baseline | T1 | T2 | T1 + T2 (95 %) | mean lap | fallbacks |",
           "|---|---|---|---|---|---|"]
    for kind in ("nmpc", "smpc", "lag_identified"):
        rs = [r for r in bl if r.get("kind") == kind and "error" not in r]
        if not rs:
            continue
        t1 = sum(r["completed"] for r in rs if r["set"] == "T1")
        t2 = sum(r["completed"] for r in rs if r["set"] == "T2")
        lo, hi = ci(t1 + t2, len(rs))
        laps = [r["lap_time"] for r in rs if r["completed"]]
        md.append(f"| {kind} | {t1} | {t2} | {t1 + t2} / {len(rs)} ({lo:.2f}–{hi:.2f}) | "
                  f"{(np.mean(laps) if laps else float('nan')):.2f} | {sum(r.get('fallbacks', 0) or 0 for r in rs)} |")
    (out / "partB.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "partB.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"written {out / 'partB.md'}")


if __name__ == "__main__":
    main()
