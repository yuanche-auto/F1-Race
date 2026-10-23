"""Control expansion phase F report (docs/CX_F_protocol.md, revision 0 with its clarifications m1–m3, n1, n2).

    python scripts/report_cx_f.py audit     # section 5.1 on L7's records (results/L7/partB_*) -> results/CX/F/report/audit_l7.*
    python scripts/report_cx_f.py tune      # section 4 -> results/CX/F/selected.json (with the SHA-256 of tune.jsonl)
    python scripts/report_cx_f.py test      # amendment F1: H-F2, the checks of section 3, section 5 -> results/CX/F/report/CX_F.*

``audit`` also checks that the false-intervention summary of ``experiments/cx_f.false_interventions`` reproduces
L7's report (``results/L7/partB_report/partB.json``) – the one procedure of review n2.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "results" / "CX" / "F"
L7B = ROOT / "results" / "L7"
NL = chr(10)


def load(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def _length() -> float:
    from f1race.track import Track

    return Track.load("Spielberg").length


def _sum_audit(rows: list[dict]) -> dict:
    keys = ("decisions", "interventions", "on_switches", "time_over_50ms", "time_over_100ms", "fallbacks",
            "fallback_runs3", "interventions_before_failure")
    out = {k: int(sum(r.get(k, 0) for r in rows)) for k in keys}
    out["laps"] = len(rows)
    out["by_eighth"] = [int(sum(r["by_eighth"][j] for r in rows)) for j in range(8)]
    out["decisions_by_eighth"] = [int(sum(r["decisions_by_eighth"][j] for r in rows)) for j in range(8)]
    out["longest_run_max"] = int(max((r["longest_run"] for r in rows), default=0))
    out["longest_fallback_run_max"] = int(max((r.get("longest_fallback_run", 0) for r in rows), default=0))
    return out


def audit() -> None:
    """Section 5.1: L7's main and CBF-QP records, per proposer and in total."""
    from f1race.experiments import cx_f

    L = _length()
    out = OUT / "report"
    out.mkdir(parents=True, exist_ok=True)
    res, md = {}, ["# CX-F audit of L7's filter records (protocol section 5.1)", "",
                   "Records: `results/L7/partB_filtered/` (L7 Part B, run at bb420e5); descriptive. The solve times were "
                   "measured under L7's load (~26 processes on 24 cores).", ""]
    l7_report = json.loads((L7B / "partB_report" / "partB.json").read_text())
    for cond in ("main", "cbf"):
        recs = load(L7B / "partB_filtered" / f"rl_{cond}.jsonl")
        per = {}
        attrib = {"fallback run": 0, "intervention": 0, "neither": 0}
        fail_by = {}
        for r in recs:
            z = np.load(L7B / "partB_filtered" / "dec" / cond / f"{r['policy']}__{r['case']}.npz")
            d = {k: z[k] for k in z.files}
            a = cx_f.lap_audit(d, r["completed"], L)
            per.setdefault(r["policy"], []).append(a)
            if not r["completed"]:
                attrib[cx_f.attribute(d)] += 1
            g = "T2" if r["set"] == "T2" else f"tau{int(round(r['tau'] * 1000))}"
            fb = fail_by.setdefault(g, [0, 0])
            fb[0] += a["interventions"]
            fb[1] += a["interventions_before_failure"]
        tot = _sum_audit([a for v in per.values() for a in v])
        fi = {p: cx_f.false_interventions(recs, p, L) for p in cx_f.PROPOSERS}
        repro = all(fi[p]["false"] == l7_report["false"][f"{p}|{cond}"]["false"] and
                    fi[p]["sampled"] == l7_report["false"][f"{p}|{cond}"]["sampled"] for p in cx_f.PROPOSERS)
        res[cond] = {"total": tot, "per_proposer": {p: _sum_audit(v) for p, v in per.items()},
                     "attribution": attrib, "interventions_followed_by_failure_3s": fail_by,
                     "false_interventions": fi, "false_reproduces_L7_report": repro}
        d = tot["decisions"]
        md += [f"## {cond}", "",
               f"* laps {tot['laps']}, decisions {d}, interventions {tot['interventions']} ({tot['interventions'] / d:.3f} "
               f"per decision), on-switches {tot['on_switches']} ({tot['on_switches'] / tot['laps']:.1f} per lap), the "
               f"longest intervention run {tot['longest_run_max']} decisions",
               f"* interventions by eighth of the lap (per decision, of n decisions): "
               + ", ".join(f"{b / max(n, 1):.2f} (n {n})" for b, n in zip(tot["by_eighth"], tot["decisions_by_eighth"])),
               f"* solver fallbacks {tot['fallbacks']} ({tot['fallbacks'] / d:.3f}), runs of >= 3: {tot['fallback_runs3']}, "
               f"the longest run {tot['longest_fallback_run_max']} decisions",
               f"* filter time above 50 ms: {tot['time_over_50ms'] / d:.3f} of the decisions; above 100 ms: "
               f"{tot['time_over_100ms'] / d:.3f}",
               f"* failures (last 3 s, L7's pb-m2): {attrib}",
               "* interventions followed by a failure within 3 s, by condition (of all interventions): "
               + ", ".join(f"{g} {v[1]}/{v[0]}" for g, v in sorted(fail_by.items())),
               f"* false interventions (200 per proposer, seed 0): reproduce L7's report: **{repro}**", ""]
    (out / "audit_l7.json").write_text(json.dumps(res, indent=1, default=float))
    (out / "audit_l7.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(NL.join(md))


def tune() -> None:
    """Section 4: the m with the most completed laps summed over the nine finals on FV; ties -> the fewer
    interventions per decision; then the larger m (review n1)."""
    from f1race.experiments import cx_f

    recs = [r for r in load(OUT / "tune.jsonl") if "error" not in r]
    errors = sum("error" in r for r in load(OUT / "tune.jsonl"))
    rows = []
    for m in cx_f.M_GRID:
        rs = [r for r in recs if r["condition"] == "shield" and r["setting"].get("m") == m and r["policy"] in cx_f.FINALS]
        dec = sum(r.get("decisions", 0) for r in rs)
        rows.append({"m": m, "laps": len(rs), "completed": sum(r["completed"] for r in rs),
                     "interventions_per_decision": sum(r.get("interventions", 0) for r in rs) / max(dec, 1)})
    best = sorted(rows, key=lambda r: (-r["completed"], r["interventions_per_decision"], -r["m"]))[0]
    none = [r for r in recs if r["condition"] == "none"]
    back = [r for r in recs if r["condition"] == "backup"]
    sel = {"m": best["m"], "rows": rows, "errors": errors,
           "none_completed": f"{sum(r['completed'] for r in none)} / {len(none)}",
           "backup_completed": f"{sum(r['completed'] for r in back)} / {len(back)}",
           "tune_sha256": hashlib.sha256((OUT / "tune.jsonl").read_bytes()).hexdigest()}
    (OUT / "selected.json").write_text(json.dumps(sel, indent=1))
    print(json.dumps(sel, indent=1))


def test() -> None:
    """Amendment F1 (option 1, g2): H-F2 – the only primary test, α 0.05 (its verdict at 0.025 reported too);
    the `none` regression and the reproduction check of `main` (section 3); the secondary comparison retry – none;
    the tables of section 5 for `main` and `retry`."""
    from scipy.stats import binomtest

    from f1race.experiments import cx_f

    L = _length()
    raw = load(OUT / "test.jsonl")
    errors = sum("error" in r for r in raw)
    recs = [r for r in raw if "error" not in r]
    l7none = {(r["policy"], r["case"]): r for r in load(L7B / "partB_none" / "rl_none.jsonl")}
    l7main = {(r["policy"], r["case"]): r for r in load(L7B / "partB_filtered" / "rl_main.jsonl")}
    by = {}
    for r in recs:
        by.setdefault(r["condition"], {})[(r["policy"], r["case"])] = r
    res = {"errors": errors}
    md = ["# CX-F – L7's main filter with retried solves on T1 + T2 (amendment F1: the shield part closed)", "",
          f"Error records: {errors}. H-F1 (the shield) was withdrawn before any T1 / T2 lap because the development "
          "gate failed (protocol section 9, amendment F1); no FV, `shield` or `backup` lap was run.", ""]
    # the regression of `none` (section 3: outcome, and the failure position within 0.013 m)
    mism, dlap = [], []
    for k, r in by.get("none", {}).items():
        o = l7none.get(k)
        if o is None or o["completed"] != r["completed"] or (not r["completed"] and abs(o["distance"] - r["distance"]) > 0.013):
            mism.append(k)
        elif r["completed"]:
            dlap.append(abs(o["lap_time"] - r["lap_time"]))
    res["none_regression"] = {"laps": len(by.get("none", {})), "mismatches": [list(k) for k in mism],
                              "max_lap_time_difference_s": max(dlap, default=0.0)}
    # the reproduction check of `main` (section 3)
    rep_bad = []
    for k, r in by.get("main_repro", {}).items():
        o = l7main[k]
        ok = o["completed"] == r["completed"] and o["stop_reason"] == r["stop_reason"] and (
            abs(o["lap_time"] - r["lap_time"]) <= 0.01 if r["completed"] else abs(o["distance"] - r["distance"]) <= 1.0)
        if not ok:
            rep_bad.append(k)
    use_l7 = len(by.get("main_repro", {})) == 22 and not rep_bad
    res["main_reproduction"] = {"laps": len(by.get("main_repro", {})), "mismatches": [list(k) for k in rep_bad],
                                "use_L7_records": use_l7}
    main = l7main if use_l7 else by.get("main_full", {})
    md += [f"* Regression of `none` against L7's `rl_none.jsonl`: {res['none_regression']['laps']} laps, "
           f"{len(mism)} mismatches (outcome; failure position within 0.013 m); the largest lap-time difference of the "
           f"completed laps {res['none_regression']['max_lap_time_difference_s']:.2e} s.",
           f"* Reproduction check of `main` (22 laps, rng 0): {len(rep_bad)} mismatches -> "
           + ("L7's records are used." if use_l7 else "`main` re-run in full (`main_full`)."), ""]
    conds = {"none": by.get("none", {}), "main": main, "retry": by.get("retry", {})}
    comp = {(p, c): {case: bool(r["completed"]) for (pp, case), r in d.items() if pp == p}
            for c, d in conds.items() for p in cx_f.PROPOSERS}
    comp = {k: v for k, v in comp.items() if v}

    def total(p, c):
        return sum(comp.get((p, c), {}).values())

    diffs = [total(p, "retry") - total(p, "main") for p in cx_f.FINALS if (p, "retry") in comp and (p, "main") in comp]
    ex = cx_f.exact_signed_rank(diffs)
    med = float(np.median(diffs)) if diffs else None
    res["H-F2"] = {"differences": diffs, "median": med, **ex, "significant_0.05": bool(ex["p"] < 0.05),
                   "significant_0.025": bool(ex["p"] < 0.025), "complete": len(diffs) == 9}
    md += ["## H-F2 (primary; the only one after amendment F1): nine final checkpoints, completed T1 + T2 laps with "
           "`retry` minus with `main`", "",
           f"Differences {diffs}; median {med}; non-zero n {ex['n']}; W+ {ex['W_plus']}; exact two-sided p {ex['p']:.4g}. "
           f"At α 0.05: **{'significant' if ex['p'] < 0.05 else 'not significant'}**; at 0.025: "
           f"{'significant' if ex['p'] < 0.025 else 'not significant'}. **Not cross-checked** (the second environment is "
           "deferred).", ""]
    d2 = [total(p, "retry") - total(p, "none") for p in cx_f.FINALS if (p, "retry") in comp and (p, "none") in comp]
    res["secondary_retry_minus_none"] = {"differences": d2, **cx_f.exact_signed_rank(d2)}
    md += [f"Secondary (uncorrected): `retry` − `none` over the nine finals: {res['secondary_retry_minus_none']}.", "",
           "## Completion (of 105; T1 / T2) and per-proposer McNemar", "",
           "| proposer | none | main | retry | retry : main (only retry : only main, p) | mean lap retry | lap-time cost retry − none (laps both complete) |",
           "|---|---|---|---|---|---|---|"]

    def mcn(a, b):
        keys = sorted(set(a) & set(b))
        x = sum(a[k] and not b[k] for k in keys)
        y = sum(b[k] and not a[k] for k in keys)
        return x, y, float(binomtest(min(x, y), x + y).pvalue) if x + y else 1.0

    def split(p, c):
        d = conds[c]
        t1 = sum(r["completed"] for (pp, _), r in d.items() if pp == p and r["set"] == "T1")
        t2 = sum(r["completed"] for (pp, _), r in d.items() if pp == p and r["set"] == "T2")
        return f"{t1 + t2} ({t1} / {t2})"

    res["per_proposer"] = {}
    for p in cx_f.PROPOSERS:
        if (p, "retry") not in comp:
            continue
        x, y, pv = mcn(comp[(p, "retry")], comp.get((p, "main"), {}))
        laps = [r["lap_time"] for (pp, _), r in conds["retry"].items() if pp == p and r["completed"]]
        cost = [r["lap_time"] - conds["none"][(p, c)]["lap_time"] for (pp, c), r in conds["retry"].items()
                if pp == p and r["completed"] and conds["none"].get((p, c), {}).get("completed")]
        res["per_proposer"][p] = {c: total(p, c) for c in conds} | {"mcnemar_retry_main": [x, y, pv]}
        md.append(f"| {p} | {split(p, 'none')} | {split(p, 'main')} | {split(p, 'retry')} | {x} : {y} ({pv:.3g}) | "
                  f"{np.mean(laps) if laps else float('nan'):.2f} | {np.mean(cost) if cost else float('nan'):+.2f} s ({len(cost)}) |")
    # the filters in the loop (section 5)
    md += ["", "## The filters in the loop (section 5)", "",
           "| proposer | condition | interventions per decision | on-switches per lap | fallbacks (share) | runs >= 3 | "
           "ms mean / P99 max | false interventions (sampled) | retries: first failed / by retry 1 / by retry 2 / fallback |",
           "|---|---|---|---|---|---|---|---|---|"]
    res["loop"] = {}
    for c in ("main", "retry"):
        for p in cx_f.PROPOSERS:
            rs = [r for (pp, _), r in conds[c].items() if pp == p and r.get("decisions")]
            if not rs:
                continue
            dec = sum(r["decisions"] for r in rs)
            aud = [cx_f.lap_audit({k: z[k] for k in z.files}, r["completed"], L)
                   for r in rs for z in [np.load(_dec_path(c, use_l7, p, r["case"]))]]
            fi = cx_f.false_interventions(rs, p, L)
            row = {"interventions_per_decision": sum(r["interventions"] for r in rs) / dec,
                   "on_switches_per_lap": sum(a["on_switches"] for a in aud) / len(rs),
                   "fallbacks": sum(r.get("filter_failures", 0) for r in rs), "fallback_share":
                   sum(r.get("filter_failures", 0) for r in rs) / dec, "fallback_runs3": sum(a.get("fallback_runs3", 0) for a in aud),
                   "time_ms_mean": float(np.average([r["time_ms"][0] for r in rs], weights=[r["decisions"] for r in rs])),
                   "time_ms_p99_max": max(r["time_ms"][2] for r in rs), "false": fi}
            if c == "retry":
                row["retry"] = {k: sum(r["retry"][k] for r in rs) for k in rs[0]["retry"]}
            res["loop"][f"{p}|{c}"] = row
            rt = row.get("retry")
            md.append(f"| {p} | {c} | {row['interventions_per_decision']:.3f} | {row['on_switches_per_lap']:.1f} | "
                      f"{row['fallbacks']} ({row['fallback_share']:.3f}) | {row['fallback_runs3']} | {row['time_ms_mean']:.0f} / "
                      f"{row['time_ms_p99_max']:.0f} | {fi['false']} / {fi['sampled']} | "
                      + (f"{rt['first_attempt_failed']} / {rt['recovered_retry1']} / {rt['recovered_retry2']} / {rt['fallbacks']}"
                         if rt else "–") + " |")
    # failures with a filter (L7's pb-m2) and interventions followed by a failure within 3 s, by condition (review m2)
    md += ["", "## Failures with a filter (L7's pb-m2: the last 3 s) and interventions followed by a failure within 3 s", "",
           "| condition | failures | fallback run | intervention | neither | of them completed without a filter | "
           "interventions followed by a failure within 3 s (tau 0 / 30 / 60 ms / T2) |", "|---|---|---|---|---|---|---|"]
    res["failures"] = {}
    for c in ("main", "retry"):
        cnt, alone, n = {"fallback run": 0, "intervention": 0, "neither": 0}, 0, 0
        fb = {}
        for (p, case), r in conds[c].items():
            z = np.load(_dec_path(c, use_l7, p, case))
            d = {k: z[k] for k in z.files}
            a = cx_f.lap_audit(d, r["completed"], L)
            g = "T2" if r["set"] == "T2" else f"tau{int(round(r['tau'] * 1000))}"
            v = fb.setdefault(g, [0, 0])
            v[0] += a["interventions"]
            v[1] += a["interventions_before_failure"]
            if r["completed"]:
                continue
            cnt[cx_f.attribute(d)] += 1
            n += 1
            alone += bool(conds["none"].get((p, case), {}).get("completed"))
        res["failures"][c] = {"n": n, **cnt, "completed_without_filter": alone, "interventions_then_failure": fb}
        md.append(f"| {c} | {n} | {cnt['fallback run']} | {cnt['intervention']} | {cnt['neither']} | {alone} | "
                  + " / ".join(f"{fb.get(g, [0, 0])[1]} of {fb.get(g, [0, 0])[0]}" for g in ("tau0", "tau30", "tau60", "T2")) + " |")
    md += ["", "Note (review of packet 2): every decision of L7's `main` took longer than 50 ms under L7's load (97 % longer "
           "than 100 ms; audit, section 5.1), while the simulation applied the filtered command at once – an idealisation "
           "of both `main` and `retry` here; real time is phase G's subject."]
    out = OUT / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_F.json").write_text(json.dumps(res, indent=1, default=float))
    (out / "CX_F.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(NL.join(md))


def _dec_path(condition: str, use_l7: bool, proposer: str, case: str) -> Path:
    """The per-decision record of a lap: L7's for `main` (if its records are used), phase F's otherwise."""
    if condition == "main" and use_l7:
        return L7B / "partB_filtered" / "dec" / "main" / f"{proposer}__{case}.npz"
    name = "main_full" if condition == "main" else condition
    return OUT / "test_dec" / name / "default" / f"{proposer}__{case}.npz"


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["audit", "tune", "test"])
    a = ap.parse_args()
    {"audit": audit, "tune": tune, "test": test}[a.what]()


if __name__ == "__main__":
    main()
