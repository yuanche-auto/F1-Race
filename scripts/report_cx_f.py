"""Control expansion phase F report (docs/CX_F_protocol.md, revision 0 with its clarifications m1–m3, n1, n2).

    python scripts/report_cx_f.py audit     # section 5.1 on L7's records (results/L7/partB_*) -> results/CX/F/report/audit_l7.*
    python scripts/report_cx_f.py tune      # section 4 -> results/CX/F/selected.json (with the SHA-256 of tune.jsonl)
    python scripts/report_cx_f.py test      # sections 1, 5, 6 -> results/CX/F/report/CX_F.{md,json}

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
    """The primary tests H-F1, H-F2 (Holm), the reproduction check of main, the secondary comparisons and
    the tables of section 5."""
    from scipy.stats import binomtest

    from f1race.experiments import cx_f

    L = _length()
    recs = load(OUT / "test.jsonl")
    errors = sum("error" in r for r in recs)
    recs = [r for r in recs if "error" not in r]
    l7none = {(r["policy"], r["case"]): r for r in load(L7B / "partB_none" / "rl_none.jsonl")}
    l7main_list = load(L7B / "partB_filtered" / "rl_main.jsonl")
    l7main = {(r["policy"], r["case"]): r for r in l7main_list}
    by = {}
    for r in recs:
        by.setdefault(r["condition"], {})[(r["policy"], r["case"])] = r
    res = {"errors": errors}
    md = ["# CX-F – safety filters around the L5 policies on T1 + T2", "", f"Error records: {errors}.", ""]
    # regression: none vs L7's rl_none
    mism = []
    for k, r in by.get("none", {}).items():
        o = l7none.get(k)
        if o is None or o["completed"] != r["completed"] or (not r["completed"] and abs(o["distance"] - r["distance"]) > 0.013) \
                or (r["completed"] and abs(o["lap_time"] - r["lap_time"]) > 1e-6):
            mism.append(k)
    res["none_regression"] = {"laps": len(by.get("none", {})), "mismatches": [list(k) for k in mism]}
    # reproduction of main (section 3)
    rep_bad = []
    for k, r in by.get("main_repro", {}).items():
        o = l7main[k]
        ok = o["completed"] == r["completed"] and o["stop_reason"] == r["stop_reason"] and (
            abs(o["lap_time"] - r["lap_time"]) <= 0.01 if r["completed"] else abs(o["distance"] - r["distance"]) <= 1.0)
        if not ok:
            rep_bad.append(k)
    res["main_reproduction"] = {"laps": len(by.get("main_repro", {})), "mismatches": [list(k) for k in rep_bad],
                                "use_L7_records": len(by.get("main_repro", {})) == 22 and not rep_bad}
    md += [f"Regression of `none` against L7's records: {len(by.get('none', {}))} laps, mismatches {len(mism)}.",
           f"Reproduction check of `main` (22 laps): mismatches {len(rep_bad)} -> L7's records used: "
           f"{res['main_reproduction']['use_L7_records']}.", ""]
    main = by.get("main_full") or l7main if res["main_reproduction"]["use_L7_records"] else by.get("main_full", {})
    comp = {}
    for cond, d in (("none", by.get("none", {})), ("main", main), ("retry", by.get("retry", {})),
                    ("shield", by.get("shield", {})), ("backup", by.get("backup", {}))):
        for (p, case), r in d.items():
            comp.setdefault((p, cond), {})[case] = bool(r["completed"])

    def total(p, c):
        return sum(comp.get((p, c), {}).values())

    tests = {}
    for name, (a, b) in (("H-F1", ("shield", "none")), ("H-F2", ("retry", "main"))):
        diffs = [total(p, a) - total(p, b) for p in cx_f.FINALS if (p, a) in comp and (p, b) in comp]
        ex = cx_f.exact_signed_rank(diffs)
        tests[name] = {"differences": diffs, "median": float(np.median(diffs)) if diffs else None, **ex}
    ps = sorted(tests, key=lambda k: tests[k]["p"])
    for i, k in enumerate(ps):                               # Holm over two
        tests[k]["holm_level"] = 0.05 / (2 - i)
        tests[k]["significant"] = bool(tests[k]["p"] < tests[k]["holm_level"] and
                                       all(tests[q]["p"] < tests[q]["holm_level"] for q in ps[:i]))
        tests[k]["below_0.025"] = bool(tests[k]["p"] < 0.025)
    res["primary"] = tests
    md += ["## Primary tests (Holm over H-F1 and H-F2)", ""]
    for k in ("H-F1", "H-F2"):
        t = tests[k]
        md.append(f"* **{k}**: differences {t['differences']}; median {t['median']}; non-zero n {t['n']}; W+ {t['W_plus']}; "
                  f"exact two-sided p {t['p']:.4g}; Holm level {t['holm_level']:.3f}: significant {t['significant']} "
                  f"(below 0.025: {t['below_0.025']}). **Not cross-checked** (second environment deferred).")
    sec = {}
    for name, (a, b) in (("shield-main", ("shield", "main")), ("retry-none", ("retry", "none"))):
        diffs = [total(p, a) - total(p, b) for p in cx_f.FINALS if (p, a) in comp and (p, b) in comp]
        sec[name] = {"differences": diffs, **cx_f.exact_signed_rank(diffs)}
    res["secondary"] = sec
    md += ["", "## Secondary (uncorrected)", ""] + [f"* {k}: {v}" for k, v in sec.items()]
    # completion table
    md += ["", "## Completion (of 105) and lap time", "", "| proposer | none | main | retry | shield | McNemar shield:none | "
           "McNemar retry:main |", "|---|---|---|---|---|---|---|"]

    def mcn(a, b):
        keys = sorted(set(a) & set(b))
        x = sum(a[k] and not b[k] for k in keys)
        y = sum(b[k] and not a[k] for k in keys)
        return x, y, float(binomtest(min(x, y), x + y).pvalue) if x + y else 1.0

    for p in cx_f.PROPOSERS:
        cells = [str(total(p, c)) if (p, c) in comp else "-" for c in ("none", "main", "retry", "shield")]
        m1 = mcn(comp[(p, "shield")], comp[(p, "none")]) if (p, "shield") in comp and (p, "none") in comp else None
        m2 = mcn(comp[(p, "retry")], comp[(p, "main")]) if (p, "retry") in comp and (p, "main") in comp else None
        md.append(f"| {p} | " + " | ".join(cells) + " | " + (f"{m1[0]} : {m1[1]} ({m1[2]:.3g})" if m1 else "-") + " | "
                  + (f"{m2[0]} : {m2[1]} ({m2[2]:.3g})" if m2 else "-") + " |")
    if ("backup", "backup") in comp:
        md.append(f"| backup alone | {total('backup', 'backup')} / 105 | | | | | |")
    # the filters in the loop, false interventions, failures after interventions by condition
    md += ["", "## The filters in the loop", ""]
    res["loop"] = {}
    for cond, d in (("main", main), ("retry", by.get("retry", {})), ("shield", by.get("shield", {}))):
        rl = [r for r in d.values()] if isinstance(d, dict) else []
        for p in cx_f.PROPOSERS:
            rs = [r for r in rl if r["policy"] == p and r.get("decisions")]
            if not rs:
                continue
            dec = sum(r["decisions"] for r in rs)
            fi = cx_f.false_interventions(rs, p, L)
            row = {"interventions_per_decision": sum(r["interventions"] for r in rs) / dec,
                   "false": fi, "time_ms_mean": float(np.average([r["time_ms"][0] for r in rs],
                                                                 weights=[r["decisions"] for r in rs]))}
            if cond in ("retry", "shield"):
                row["on_switches_per_lap"] = sum(r.get("on_switches", 0) for r in rs) / len(rs)
            if cond == "shield":
                row["no_option"] = sum(r.get("no_option", 0) for r in rs) / dec
            if cond == "retry":
                row["retry"] = {k: sum(r["retry"][k] for r in rs) for k in rs[0]["retry"]}
            res["loop"][f"{p}|{cond}"] = row
            md.append(f"* {p} | {cond}: {json.dumps(row, default=float)}")
    out = OUT / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_F.json").write_text(json.dumps(res, indent=1, default=float))
    (out / "CX_F.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(NL.join(md))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["audit", "tune", "test"])
    a = ap.parse_args()
    {"audit": audit, "tune": tune, "test": test}[a.what]()


if __name__ == "__main__":
    main()
