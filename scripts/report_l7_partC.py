"""L7 Part C report: S1, S3 and the secondary results from the evaluation of the trained
checkpoints on T1 + T2 (docs/L7_protocol.md sections 5–7, amendments A7, A8).

    .venv-rl/Scripts/python scripts/report_l7_partC.py --eval <dir> --models <dir> \\
        --partB-none <dir> --partB-filtered <dir> --out results/L7/partC_report

``--eval``: the output of ``scripts/run_l7_partC.py`` (rl_none.jsonl, rl_main.jsonl, dec/,
selection.json); ``--models``: the training output (one directory per run with training.json,
validation.jsonl); ``--partB-none`` / ``--partB-filtered``: Part B's directories (H-B, the bare
tracker under the main filter, the model-based baselines, the L5 policies for reference).

Fixed before any Part C test result was read (2026-10-16):
* **Median seed** of each group (section 5, the L5 ranking rule): the five final checkpoints
  (600 000) ranked by completed T1 + T2 laps without the filter (more first), ties by the lower
  mean lap time of the completed laps, then by the lower seed; the median seed is the third.
* **S1 (primary)**: exact McNemar (two-sided binomial test on the discordant pairs) between the
  median seeds of F and P, without the filter, over the 105 paired laps. Holm over the two
  primary tests H-B and S1 (section 7): the two-sided p values sorted, the smaller tested at
  0.025, the larger at 0.05 only if the smaller is rejected. H-B's p is recomputed here from
  Part B's records (exact signed-rank, as in report_l7_partB). S1 holds if its null hypothesis
  is rejected by Holm and the median seed of F completes more laps (more laps only F
  completes than only P completes); H-B holds if rejected and its median difference is
  positive. A9.1: H-B's two-sided null can be rejected in the direction "fewer laps", which
  lets S1 be tested at 0.05 (the literal reading of section 7, Holm). The report gives this
  decision and the stricter one (S1 tested at 0.025 whatever H-B gives) side by side; where
  they differ, both are stated.
* **5 x 5 pairwise** (scope of S1): for every final F seed against every final P seed,
  completed laps without the filter and exact McNemar; counts of pairs in which F completes
  more / fewer / equally many, and of pairs significant at 0.05 in either direction
  (uncorrected, 25 comparisons, no claim across them).
* **S3**: the median seed of F with the main filter (a) completes at least as many T1 + T2 laps
  as the bare tracker under the main filter (Part B) and (b) on the laps that both it (with
  the main filter) and the lag-aware NMPC with τ and parameters identified (Part B baseline,
  no filter) complete, its mean lap time is at most 4.6 s above the NMPC's (paired, point
  estimate of the mean difference). If no lap is completed by both, (b) and so S3 is not shown.
* **Secondary**: all final checkpoints with and without the main filter (T1, T2, together;
  exact 95 % intervals); the validation-selected checkpoint per group (selection.json, V and
  test); F against P with the main filter (median seeds, exact McNemar); per policy McNemar
  with / without the filter; the filter's interventions, fallbacks, slack, solve time and
  lap-time cost; false interventions (A8: 200 per policy drawn with seed 0, per eighth);
  attribution of filtered failures (pb-m2, with the overlap of pb2-m5); the training
  records (validation laps per checkpoint, F's pool).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_l7_partB import FINALS as L5_FINALS, attribute, ci, exact_signed_rank, load, mcnemar  # noqa: E402
from run_l7_partC import FINALS, GROUPS, RUNS, SEEDS                                              # noqa: E402

RL_GAP_S = 4.6
NL = chr(10)


def holm(pvals: dict, alpha: float = 0.05) -> dict:
    """Holm's step-down over the given two-sided p values: name -> (level, rejected)."""
    order = sorted(pvals, key=lambda k: pvals[k])
    out, go = {}, True
    for i, k in enumerate(order):
        level = alpha / (len(order) - i)
        rej = go and pvals[k] <= level
        out[k] = {"p": pvals[k], "level": level, "rejected": bool(rej)}
        go = rej
    return out


def summary(rs: list[dict]) -> dict:
    done = [r["lap_time"] for r in rs if r["completed"]]
    return {"n": len(rs), "completed": len(done), "T1": sum(r["completed"] for r in rs if r["set"] == "T1"),
            "T2": sum(r["completed"] for r in rs if r["set"] == "T2"),
            "lap_mean": float(np.mean(done)) if done else float("inf")}


def ranking(by_none: dict, group: str) -> list[dict]:
    rows = []
    for s in SEEDS:
        ck = f"{group}_s{s}_600k"
        rs = [r for (p, _), r in by_none.items() if p == ck]
        rows.append({"checkpoint": ck, "seed": s, **summary(rs)})
    rows.sort(key=lambda r: (-r["completed"], r["lap_mean"], r["seed"]))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval", required=True)
    ap.add_argument("--models", required=True)
    ap.add_argument("--partB-none", required=True)
    ap.add_argument("--partB-filtered", required=True)
    ap.add_argument("--out", default=str(ROOT / "results" / "L7" / "partC_report"))
    a = ap.parse_args()
    de, dm, bn, bf, out = (Path(x) for x in (a.eval, a.models, a.partB_none, a.partB_filtered, a.out))
    out.mkdir(parents=True, exist_ok=True)
    from f1race.track import Track

    L = Track.load("Spielberg").length
    rl = {c: load(de / f"rl_{c}.jsonl") for c in ("none", "main")}
    errors = {c: sum("error" in r for r in v) for c, v in rl.items()}
    by = {c: {(r["policy"], r["case"]): r for r in v if "error" not in r} for c, v in rl.items()}
    pb = {c: {(r["policy"], r["case"]): r for r in load((bn if c == "none" else bf) / f"rl_{c}.jsonl") if "error" not in r}
          for c in ("none", "main")}
    baselines = [r for r in load(bn / "baselines.jsonl") if "error" not in r]
    sel = json.loads((de / "selection.json").read_text()) if (de / "selection.json").exists() else {}
    res = {"errors": errors, "selection": sel}
    md = ["# L7 Part C – evaluation on T1 + T2", "", f"Error records: {errors}.", ""]

    def comp(src: dict, p: str) -> dict:
        return {case: r["completed"] for (pp, case), r in src.items() if pp == p}

    # median seeds (L5 ranking rule)
    md += ["## Median seeds (final checkpoints, without the filter; L5 ranking rule)", "",
           "| group | rank | checkpoint | T1 (75) | T2 (30) | T1 + T2 | mean lap |", "|---|---|---|---|---|---|---|"]
    med = {}
    for g in GROUPS:
        rk = ranking(by["none"], g)
        med[g] = rk[2]["checkpoint"]
        res[f"ranking_{g}"] = rk
        for i, r in enumerate(rk):
            mark = " **(median)**" if i == 2 else ""
            md.append(f"| {g} | {i + 1} | {r['checkpoint']}{mark} | {r['T1']} | {r['T2']} | {r['completed']} / {r['n']} | "
                      f"{r['lap_mean']:.2f} |")
    res["median"] = med

    # S1 and Holm with H-B
    x, y, p_s1 = mcnemar(comp(by["none"], med["F"]), comp(by["none"], med["P"]))
    hb_d = [sum(comp(pb["main"], p).values()) - sum(comp(pb["none"], p).values()) for p in L5_FINALS]
    hb = exact_signed_rank(hb_d)
    h = holm({"H-B": hb["p"], "S1": p_s1})
    s1_holds = h["S1"]["rejected"] and x > y
    s1_strict = p_s1 <= 0.025 and x > y                     # A9.1: the stricter reading
    hb_holds = h["H-B"]["rejected"] and float(np.median(hb_d)) > 0
    res["S1"] = {"F": med["F"], "P": med["P"], "only_F": x, "only_P": y, "p": p_s1, "holm": h["S1"], "holds": s1_holds,
                 "holds_strict_0.025": s1_strict}
    res["H-B"] = {"differences": hb_d, **hb, "median": float(np.median(hb_d)), "holm": h["H-B"], "holds": hb_holds}
    md += ["", "## S1 (primary): median seed of F against median seed of P, without the filter", "",
           f"{med['F']} against {med['P']}: completed only by F : only by P = {x} : {y}; exact McNemar two-sided "
           f"p = {p_s1:.4g}.", "",
           "Holm over the two primary tests (section 7):", "",
           "| test | p (two-sided) | Holm level | null rejected | direction | holds |", "|---|---|---|---|---|---|",
           f"| H-B (Part B) | {hb['p']:.4g} | {h['H-B']['level']:.3g} | {h['H-B']['rejected']} | median {np.median(hb_d):+g} | "
           f"**{hb_holds}** |",
           f"| S1 | {p_s1:.4g} | {h['S1']['level']:.3g} | {h['S1']['rejected']} | F {'more' if x > y else 'not more'} "
           f"({x} : {y}) | **{s1_holds}** |", "",
           f"Stricter reading (A9.1: S1 at 0.025 whatever H-B gives): S1 {'holds' if s1_strict else 'does not hold'}"
           + (" – **the two readings differ**." if s1_strict != s1_holds else " – the same decision as Holm.")]

    # 5 x 5 pairwise
    md += ["", "## 5 x 5 pairwise, final checkpoints without the filter (scope of S1; 25 comparisons, uncorrected)", "",
           "Cells: completed F − completed P; only F : only P; p.", "",
           "| F \\ P | " + " | ".join(f"P_s{s}" for s in SEEDS) + " |", "|---" * (len(SEEDS) + 1) + "|"]
    cnt = {"F more": 0, "F fewer": 0, "equal": 0, "sig F more": 0, "sig F fewer": 0}
    for sf in SEEDS:
        cf = comp(by["none"], f"F_s{sf}_600k")
        row = [f"F_s{sf}"]
        for sp in SEEDS:
            cp = comp(by["none"], f"P_s{sp}_600k")
            d = sum(cf.values()) - sum(cp.values())
            xx, yy, pp = mcnemar(cf, cp)
            cnt["F more" if d > 0 else "F fewer" if d < 0 else "equal"] += 1
            if pp < 0.05:
                cnt["sig F more" if xx > yy else "sig F fewer"] += 1
            row.append(f"{d:+d}; {xx} : {yy}; {pp:.2g}")
        md.append("| " + " | ".join(row) + " |")
    res["pairwise"] = cnt
    md += ["", f"F completes more in {cnt['F more']}, fewer in {cnt['F fewer']}, equally many in {cnt['equal']} of 25 pairs; "
           f"significant at 0.05 (uncorrected): F more {cnt['sig F more']}, F fewer {cnt['sig F fewer']}."]

    # S3
    f_main = {case: r for (p, case), r in by["main"].items() if p == med["F"]}
    tr_main = {case: r for (p, case), r in pb["main"].items() if p == "tracker"}
    lag = {r["case"]: r for r in baselines if r["kind"] == "lag_identified"}
    n_f, n_tr = sum(r["completed"] for r in f_main.values()), sum(r["completed"] for r in tr_main.values())
    both = [c for c in f_main if f_main[c]["completed"] and c in lag and lag[c]["completed"]]
    dt = [f_main[c]["lap_time"] - lag[c]["lap_time"] for c in both]
    s3a = n_f >= n_tr
    s3b = bool(dt) and float(np.mean(dt)) <= RL_GAP_S
    res["S3"] = {"F_main": n_f, "tracker_main": n_tr, "a": s3a, "laps_both": len(both),
                 "mean_dt": float(np.mean(dt)) if dt else None, "median_dt": float(np.median(dt)) if dt else None,
                 "b": s3b, "holds": s3a and s3b}
    md += ["", "## S3: median seed of F with the main filter", "",
           f"(a) completed T1 + T2 laps: {med['F']} with the main filter {n_f} against the bare tracker with the main filter "
           f"{n_tr} → {'met' if s3a else 'not met'}.", "",
           f"(b) lap time against the lag-aware NMPC (τ and parameters identified, no filter) on the {len(both)} laps both "
           f"complete: mean difference {(np.mean(dt) if dt else float('nan')):+.2f} s (median "
           f"{(np.median(dt) if dt else float('nan')):+.2f} s), limit +{RL_GAP_S} s → "
           f"{'met' if s3b else 'not met' if dt else 'not evaluable (no lap completed by both)'}.", "",
           f"**S3 {'holds' if s3a and s3b else 'not shown'}.**"]

    # completion of all policies (secondary; section 6)
    pols = FINALS + [v["checkpoint"] for v in sel.values() if v["checkpoint"] not in FINALS]
    md += ["", "## Completion and lap time (all final checkpoints; secondary: selected; reference: Part B)", "",
           "| policy | condition | T1 (75) | T2 (30) | T1 + T2 (95 % interval) | mean lap of completed |", "|---|---|---|---|---|---|"]
    res["completion"] = {}

    def row(name: str, cond: str, rs: list[dict]) -> None:
        s = summary(rs)
        lo, hi = ci(s["completed"], s["n"])
        res["completion"][f"{name}|{cond}"] = s
        md.append(f"| {name} | {cond} | {s['T1']} | {s['T2']} | {s['completed']} / {s['n']} ({lo:.2f}–{hi:.2f}) | "
                  f"{s['lap_mean']:.2f} |" if s["completed"] else
                  f"| {name} | {cond} | {s['T1']} | {s['T2']} | {s['completed']} / {s['n']} ({lo:.2f}–{hi:.2f}) | – |")

    for p in pols:
        for c in ("none", "main"):
            rs = [r for (pp, _), r in by[c].items() if pp == p]
            if rs:
                row(p, c, rs)
    for p in ("B_s1_500k", "tracker"):
        for c in ("none", "main"):
            rs = [r for (pp, _), r in pb[c].items() if pp == p]
            if rs:
                row(f"{p} (Part B)", c, rs)
    for g in GROUPS:
        tot = [sum(comp(by["none"], f"{g}_s{s}_600k").values()) for s in SEEDS]
        totm = [sum(comp(by["main"], f"{g}_s{s}_600k").values()) for s in SEEDS]
        res[f"group_{g}"] = {"none": tot, "main": totm}
        md.append(f"| group {g}, five finals | none / main | | | {tot} / {totm} | |")

    # F vs P with the main filter; per policy with / without
    xm, ym, pm = mcnemar(comp(by["main"], med["F"]), comp(by["main"], med["P"]))
    res["F_vs_P_main"] = {"only_F": xm, "only_P": ym, "p": pm}
    md += ["", "## Secondary comparisons", "",
           f"F against P with the main filter (median seeds): only F : only P = {xm} : {ym}, p = {pm:.3g}.", "",
           "Per policy, main filter against none (only with : only without, exact McNemar):", "",
           "| policy | with : without | p |", "|---|---|---|"]
    for p in pols:
        xx, yy, pp = mcnemar(comp(by["main"], p), comp(by["none"], p))
        md.append(f"| {p} | {xx} : {yy} | {pp:.3g} |")
    if sel:
        md += ["", "Validation-selected checkpoint per group (secondary; V = 40 laps):", "",
               "| group | checkpoint | V completed | V mean lap | test none | test main |", "|---|---|---|---|---|---|"]
        for g, v in sel.items():
            cn = sum(comp(by["none"], v["checkpoint"]).values())
            cm = sum(comp(by["main"], v["checkpoint"]).values())
            md.append(f"| {g} | {v['checkpoint']} | {v['val_completed']} / {v['val_laps']} | {v['val_lap_mean']:.2f} | "
                      f"{cn} | {cm} |")

    # the filter in the loop
    md += ["", "## The main filter in the loop", "",
           "| policy | interventions per decision | fallbacks (share) | decisions with slack | ms mean / P99 | "
           "lap-time cost on laps both complete (mean, n) |", "|---|---|---|---|---|---|"]
    for p in pols:
        rs = [r for (pp, _), r in by["main"].items() if pp == p and "decisions" in r]
        if not rs:
            continue
        dec = sum(r["decisions"] for r in rs)
        cost = [r["lap_time"] - by["none"][(p, r["case"])]["lap_time"] for r in rs
                if r["completed"] and (p, r["case"]) in by["none"] and by["none"][(p, r["case"])]["completed"]]
        tm = np.average([r["time_ms"][0] for r in rs], weights=[r["decisions"] for r in rs])
        md.append(f"| {p} | {sum(r['interventions'] for r in rs) / dec:.3f} | "
                  f"{sum(r['filter_failures'] for r in rs)} ({sum(r['filter_failures'] for r in rs) / dec:.3f}) | "
                  f"{sum(r['slack_decisions'] for r in rs) / dec:.3f} | {tm:.1f} / {max(r['time_ms'][2] for r in rs):.0f} | "
                  f"{(np.mean(cost) if cost else float('nan')):+.2f} s ({len(cost)}) |")

    # false interventions (A8)
    md += ["", "## False interventions (A8): 200 per policy drawn with seed 0; per eighth of the lap", "",
           "| policy | snapshots | sampled | false (95 % interval) | per eighth (false / sampled) |", "|---|---|---|---|---|"]
    res["false"] = {}
    for p in pols:
        snaps = [s for (pp, _), r in by["main"].items() if pp == p for s in r.get("false_interventions", [])]
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
        res["false"][p] = {"snapshots": len(snaps), "sampled": len(smp), "false": k}
        md.append(f"| {p} | {len(snaps)} | {len(smp)} | {k / len(smp):.2f} ({lo:.2f}–{hi:.2f}) | {' '.join(per)} |")

    # attribution of filtered failures (pb-m2, pb2-m5)
    counts = {"fallback run": 0, "intervention": 0, "neither": 0}
    overlap = alone = n = 0
    for (p, case), r in by["main"].items():
        if r["completed"] or p not in pols:
            continue
        f = de / "dec" / "main" / f"{p}__{case}.npz"
        if not f.exists():
            continue
        z = np.load(f)
        kind = attribute({"fail": z["fail"], "changed": z["changed"]})
        counts[kind] += 1
        fb, ch = np.asarray(z["fail"], bool)[-60:], np.asarray(z["changed"], bool)[-60:]
        overlap += kind == "fallback run" and bool((ch & ~fb).any())
        n += 1
        alone += bool(by["none"].get((p, case), {}).get("completed"))
    res["attribution"] = {**counts, "failures": n, "fallback_run_with_intervention": overlap, "completed_without": alone}
    md += ["", "## Failures with the main filter (pb-m2): the last 3 s", "",
           f"Of {n}: fallback run {counts['fallback run']} (of them {overlap} also with an intervention at a decision whose "
           f"solve succeeded), intervention {counts['intervention']}, neither {counts['neither']}; {alone} of the {n} laps "
           "complete without the filter."]

    # training records
    md += ["", "## Training (validation laps V per checkpoint, of 40; F: pool at the end)", "",
           "| run | V completed at 50k … 600k | minutes | pool (train / C) | C failures at the last checkpoint |",
           "|---|---|---|---|---|"]
    for r in RUNS:
        f = dm / r / "training.json"
        if not f.exists():
            continue
        tr = json.loads(f.read_text())
        v = [str(x["val_completed"]) for x in tr if "val_completed" in x]
        last = tr[-1]
        cur = [x["cur_failures"] for x in tr if "cur_failures" in x]
        md.append(f"| {r} | {' '.join(v)} | {last['minutes']:.0f} | "
                  f"{last.get('pool_train', 0)} / {last.get('pool_C', 0)} | {cur[-1] if cur else '–'} |")

    (out / "partC.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "partC.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"written {out / 'partC.md'}")


if __name__ == "__main__":
    main()
