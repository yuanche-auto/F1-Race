"""L8 report: H-G, H-D and the tables of docs/L8_protocol.md (sections 1, 4-8).

    .venv-rl/Scripts/python scripts/report_l8.py --eval results/L8/eval --l7-none results/L7/partB_none \\
        --l7-partc results/L7/partC_eval --out results/L8/report

``--eval``: the records of ``scripts/run_l8.py`` (A_model / A_rl / A0_model / B_model / B_rl, and
ft_<target>_rl for Part C); ``--l7-none``: L7 Part B's records without a filter (rl_none.jsonl,
baselines.jsonl; 375a418); ``--l7-partc``: L7 Part C's (rl_none.jsonl; f71c9c6). The Spielberg
reference of a case A0_car<i> is L7's T1_car<i>_tau0, of A0_dist<d> L7's T2_dist<d> (d 0-9).

Fixed before any L8 test result was read (2026-10-17):
* **H-G**: the 100 laps of set A with a car (25 cars x 4 test tracks); ``lag_identified`` against
  ``B_s1_500k``; exact McNemar (two-sided binomial test on the discordant laps).
* **H-D**: the 50 laps of set B with a wet level (25 cars x 0.85, 0.75); the same pair and test.
* **Holm** over H-G and H-D (``report_l7_partC.holm``: the smaller p at 0.025, the other at 0.05
  only if the first is rejected) and the stricter decision (each at 0.025). A hypothesis holds
  if its null is rejected and more laps are completed only by ``lag_identified`` than only by
  ``B_s1_500k``; where the two decisions differ, both are stated.
* Secondary (reported with p and the number of comparisons): the RL distribution first – per
  policy the completion on Spielberg (the 25 cars), the test tracks, the wet levels and the
  patches, and over the nine final L5 checkpoints the exact signed-rank test of the change in
  completed share against Spielberg on the same cars (test tracks; wet); the fixed set per
  condition with exact intervals, its ranking (more laps first, ties by the lower mean lap time)
  and Kendall's tau against Spielberg's; McNemar of every fixed controller against
  ``lag_identified``; the identification diagnostics (l8-M1); failure positions; lap times;
  NMPC solve statistics; Part C.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_l7_partB import ci, exact_signed_rank, load, mcnemar  # noqa: E402
from report_l7_partC import holm  # noqa: E402
from run_l8 import FT_SEEDS, FT_TARGETS, L5_FINALS, L7_FINALS, L7_SELECTED  # noqa: E402

FIXED = ["tracker", "nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified", "B_s1_500k"]
NL = chr(10)


def l7_name(a0_case: str) -> str:
    if "_car" in a0_case:
        return f"T1_car{a0_case.split('_car')[1]}_tau0"
    return f"T2_dist{a0_case.split('_dist')[1]}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval", required=True)
    ap.add_argument("--l7-none", required=True)
    ap.add_argument("--l7-partc", required=True)
    ap.add_argument("--out", default=str(ROOT / "results" / "L8" / "report"))
    a = ap.parse_args()
    de, out = Path(a.eval), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    from f1race.experiments.l8 import TEST, cases

    # ---- records: rec[(controller, case)] -> record
    rec: dict = {}
    errors = {}
    for f, key in (("A_model", "kind"), ("A0_model", "kind"), ("B_model", "kind"), ("A_rl", "policy"), ("B_rl", "policy")):
        rs = load(de / f"{f}.jsonl")
        errors[f] = sum("error" in r for r in rs)
        for r in rs:
            if "error" not in r:
                rec[(r[key], r["case"])] = r
    a0 = [c["name"] for c in cases("A0")]
    l7b = {(r["kind"], r["case"]): r for r in load(Path(a.l7_none) / "baselines.jsonl") if "error" not in r}
    l7r = {(r["policy"], r["case"]): r for r in load(Path(a.l7_none) / "rl_none.jsonl") if "error" not in r}
    l7c = {(r["policy"], r["case"]): r for r in load(Path(a.l7_partc) / "rl_none.jsonl") if "error" not in r}
    for c in a0:
        n7 = l7_name(c)
        for k in ("nmpc", "smpc", "lag_identified"):
            if (k, n7) in l7b:
                rec[(k, c)] = l7b[(k, n7)]
        for src in (l7r, l7c):
            for (p, cc), r in src.items():
                if cc == n7:
                    rec[(p, c)] = r
    res = {"errors": errors}
    md = ["# L8 – generalisation: new tracks and wet Spielberg", "", f"Error records: {errors}.", ""]

    groups = {
        "Spielberg cars (25)": [c for c in a0 if "_car" in c],
        "Spielberg L3 (10)": [c for c in a0 if "_dist" in c],
        **{f"{t} cars (25)": [c["name"] for c in cases("A") if c["track"] == t and c["car"] is not None] for t in TEST},
        "test tracks cars (100)": [c["name"] for c in cases("A") if c["car"] is not None],
        "test tracks L3 (40)": [c["name"] for c in cases("A") if c["dist_seed"] is not None],
        "wet 0.85 (25)": [c["name"] for c in cases("B") if c["wet"] == 0.85],
        "wet 0.75 (25)": [c["name"] for c in cases("B") if c["wet"] == 0.75],
        "wet both (50)": [c["name"] for c in cases("B") if c["wet"] is not None],
        "patches (30)": [c["name"] for c in cases("B") if c["patch_seed"] is not None],
    }

    def comp(ctrl: str, names: list[str]) -> dict:
        return {n: bool(rec[(ctrl, n)]["completed"]) for n in names if (ctrl, n) in rec}

    def cell(ctrl: str, g: str) -> str:
        d = comp(ctrl, groups[g])
        if not d:
            return "–"
        k = sum(d.values())
        lo, hi = ci(k, len(d))
        return f"{k}/{len(d)}" + (f" ({lo:.2f}–{hi:.2f})" if len(d) >= 50 else "")

    # ---- primary
    prim = {}
    for hname, g in (("H-G", "test tracks cars (100)"), ("H-D", "wet both (50)")):
        x, y, p = mcnemar(comp("lag_identified", groups[g]), comp("B_s1_500k", groups[g]))
        prim[hname] = {"laps": g, "only_lag_identified": x, "only_B_s1_500k": y, "p": p,
                       "lag_identified": sum(comp("lag_identified", groups[g]).values()),
                       "B_s1_500k": sum(comp("B_s1_500k", groups[g]).values())}
    h = holm({k: v["p"] for k, v in prim.items()})
    md += ["## Primary tests (H-G, H-D): `lag_identified` against `B_s1_500k`, exact McNemar", "",
           "| test | laps | lag_identified | B_s1_500k | only lag_identified : only B_s1_500k | p | Holm level | rejected | "
           "holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|---|---|---|"]
    for k, v in prim.items():
        holds = h[k]["rejected"] and v["only_lag_identified"] > v["only_B_s1_500k"]
        strict = v["p"] <= 0.025 and v["only_lag_identified"] > v["only_B_s1_500k"]
        v.update({"holm": h[k], "holds": holds, "holds_strict": strict})
        md.append(f"| {k} | {v['laps']} | {v['lag_identified']} | {v['B_s1_500k']} | {v['only_lag_identified']} : "
                  f"{v['only_B_s1_500k']} | {v['p']:.4g} | {h[k]['level']:.3g} | {h[k]['rejected']} | **{holds}** | "
                  f"{strict}{' – **the two readings differ**' if strict != holds else ''} |")
    res["primary"] = prim

    # ---- RL distribution first
    pols = L5_FINALS + ["B_s1_500k"] + L7_FINALS + L7_SELECTED + ["tracker"]
    cols = ["Spielberg cars (25)", "test tracks cars (100)", *[f"{t} cars (25)" for t in TEST], "test tracks L3 (40)",
            "wet 0.85 (25)", "wet 0.75 (25)", "patches (30)"]
    md += ["", "## RL policies (unselected distribution first), completed laps", "",
           "| policy | " + " | ".join(cols) + " |", "|---" * (len(cols) + 1) + "|"]
    for p in pols:
        md.append(f"| {p} | " + " | ".join(cell(p, g) for g in cols) + " |")
    res["rl_change"] = {}
    for lab, g in (("test tracks", "test tracks cars (100)"), ("wet", "wet both (50)")):
        d = []
        for p in L5_FINALS:
            cs, ct = comp(p, groups["Spielberg cars (25)"]), comp(p, groups[g])
            if cs and ct:
                d.append(sum(ct.values()) / len(ct) - sum(cs.values()) / len(cs))
        ex = exact_signed_rank([round(x, 9) for x in d]) if d else {"n": 0, "W_plus": 0, "p": 1.0}
        res["rl_change"][lab] = {"changes": d, **ex, "median": float(np.median(d)) if d else None}
        md += ["", f"Nine final L5 checkpoints, completed share on {lab} minus on Spielberg's 25 cars: "
               f"{[round(x, 3) for x in d]}; median {np.median(d) if d else float('nan'):+.3f}; exact signed-rank "
               f"n = {ex['n']}, W+ = {ex['W_plus']}, p = {ex['p']:.4g} (secondary)."]

    # ---- fixed set per condition, ranking, Kendall
    from scipy.stats import kendalltau

    cond = ["Spielberg cars (25)", "Spielberg L3 (10)", "test tracks cars (100)", "test tracks L3 (40)", "wet 0.85 (25)",
            "wet 0.75 (25)", "patches (30)"]
    md += ["", "## The fixed baseline set", "", "| controller | " + " | ".join(cond) + " |", "|---" * (len(cond) + 1) + "|"]
    for c in FIXED:
        md.append(f"| {c} | " + " | ".join(cell(c, g) for g in cond) + " |")

    def lap_mean(c: str, g: str) -> float:
        t = [rec[(c, n)]["lap_time"] for n in groups[g] if (c, n) in rec and rec[(c, n)]["completed"]]
        return float(np.mean(t)) if t else float("inf")

    md += ["", "Ranking (more laps first, ties by the lower mean lap time) and Kendall's tau of the completed counts "
           "against Spielberg's 25 cars (descriptive):", ""]
    ref = [sum(comp(c, groups["Spielberg cars (25)"]).values()) for c in FIXED]
    res["ranking"] = {}
    for g in cond:
        k = [sum(comp(c, groups[g]).values()) for c in FIXED]
        order = sorted(FIXED, key=lambda c: (-sum(comp(c, groups[g]).values()), lap_mean(c, g)))
        tau = kendalltau(ref, k).statistic if len(set(k)) > 1 and len(set(ref)) > 1 else float("nan")
        res["ranking"][g] = {"order": order, "kendall_tau": float(tau)}
        md.append(f"* {g}: {' > '.join(order)}; tau = {tau:.2f}")

    md += ["", "McNemar of each fixed controller against `lag_identified` (only it : only lag_identified, p; "
           f"{(len(FIXED) - 1) * 2} comparisons, uncorrected):", "",
           "| controller | test tracks cars (100) | wet both (50) |", "|---|---|---|"]
    for c in FIXED:
        if c == "lag_identified":
            continue
        row = [c]
        for g in ("test tracks cars (100)", "wet both (50)"):
            x, y, p = mcnemar(comp(c, groups[g]), comp("lag_identified", groups[g]))
            row.append(f"{x} : {y}, {p:.3g}")
        md.append("| " + " | ".join(row) + " |")

    # ---- identification diagnostics (l8-M1)
    md += ["", "## Identification controllers (l8-M1)", "",
           "| controller | condition | exploration completed | exploration distance (median) | stop reasons | "
           "|mu_hat - mu| (median) | laps with a parameter on its bound |", "|---|---|---|---|---|---|---|"]
    for c in ("identified", "lag_identified"):
        for g in cond:
            rs = [rec[(c, n)] for n in groups[g] if (c, n) in rec]
            if not rs:
                continue
            ex = [r["explore"] for r in rs if "explore" in r]
            idn = [r for r in rs if "identification" in r]
            mu_err = [abs(r["identification"]["theta"][0] - r["true_mu"]) for r in idn if "true_mu" in r]
            bounds = sum(any(r["identification"].get("on_bounds", [])) for r in idn)
            sr = Counter(r["stop_reason"] for r in rs)
            md.append(f"| {c} | {g} | {sum(e['completed'] for e in ex)}/{len(ex)} | "
                      f"{np.median([e['distance'] for e in ex]) if ex else float('nan'):.0f} m | {dict(sr)} | "
                      f"{np.median(mu_err) if mu_err else float('nan'):.3f} | {bounds}/{len(idn)} |")

    # ---- failure positions, lap times, solver
    md += ["", "## Failure positions (the three most frequent 100 m bins per controller and track)", ""]
    for t in TEST:
        names = [c["name"] for c in cases("A") if c["track"] == t]
        for c in FIXED + L5_FINALS:
            f = [int(rec[(c, n)]["distance"] // 100 * 100) for n in names if (c, n) in rec and not rec[(c, n)]["completed"]]
            if f:
                md.append(f"* {t}, {c}: {len(f)} failures; " + ", ".join(f"{b} m ×{k}" for b, k in Counter(f).most_common(3)))
    md += ["", "## Lap times against `lag_identified` on laps both complete (mean / median difference, n)", "",
           "| controller | test tracks cars (100) | wet both (50) |", "|---|---|---|"]
    for c in FIXED:
        if c == "lag_identified":
            continue
        row = [c]
        for g in ("test tracks cars (100)", "wet both (50)"):
            d = [rec[(c, n)]["lap_time"] - rec[("lag_identified", n)]["lap_time"] for n in groups[g]
                 if (c, n) in rec and ("lag_identified", n) in rec and rec[(c, n)]["completed"]
                 and rec[("lag_identified", n)]["completed"]]
            row.append(f"{np.mean(d):+.2f} / {np.median(d):+.2f} s ({len(d)})" if d else "–")
        md.append("| " + " | ".join(row) + " |")
    md += ["", "## NMPC solver (fallback share of solves; solve time mean / P99 / max in ms)", "",
           "| controller | condition | fallbacks | ms |", "|---|---|---|---|"]
    for c in ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified"):
        for g in ("test tracks cars (100)", "wet both (50)"):
            rs = [rec[(c, n)] for n in groups[g] if (c, n) in rec and rec[(c, n)].get("solves")]
            if rs:
                fb = sum(r["fallbacks"] for r in rs) / max(sum(r["solves"] for r in rs), 1)
                md.append(f"| {c} | {g} | {fb:.3f} | {np.mean([r['solve_ms'][0] for r in rs]):.0f} / "
                          f"{max(r['solve_ms'][2] for r in rs):.0f} / {max(r['solve_ms'][3] for r in rs):.0f} |")

    # ---- Part C
    md += ["", "## Part C: fine-tuned `B_s1_500k` (secondary; ~5 000 s of driving on the target, other cars)", "",
           "| target | laps | B_s1_500k | lag_identified | fine-tuned seeds (completed; McNemar against B_s1_500k) |",
           "|---|---|---|---|---|"]
    res["partC"] = {}
    for t in FT_TARGETS:
        rs = {(r["policy"], r["case"]): r for r in load(de / f"ft_{t}_rl.jsonl") if "error" not in r}
        if not rs:
            continue
        names = (groups["wet both (50)"] + groups["patches (30)"]) if t == "wet" else groups[f"{t} cars (25)"] + [
            c["name"] for c in cases("A") if c["track"] == t and c["dist_seed"] is not None]
        base = comp("B_s1_500k", names)
        li = comp("lag_identified", names)
        cells = []
        for k in FT_SEEDS:
            ft = {n: bool(rs[(f"ft_{t}_s{k}", n)]["completed"]) for n in names if (f"ft_{t}_s{k}", n) in rs}
            x, y, p = mcnemar(ft, base)
            cells.append(f"s{k}: {sum(ft.values())} ({x} : {y}, {p:.3g})")
            res["partC"][f"{t}_s{k}"] = {"completed": sum(ft.values()), "only_ft": x, "only_base": y, "p": p}
        md.append(f"| {t} | {len(names)} | {sum(base.values())} | {sum(li.values())} | {'; '.join(cells)} |")

    (out / "L8.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "L8.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"written {out / 'L8.md'}")


if __name__ == "__main__":
    main()
