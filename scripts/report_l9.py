"""L9 report: H-RT1, H-RT2 and the tables of docs/L9_protocol.md (sections 1, 5, 7-9).

    python scripts/report_l9.py --dir results/L9 --out results/L9/report

Reads results/L9/{offline,realtime}/ (model.jsonl, explore.jsonl, load.jsonl; rl.jsonl when the
RL part has run), and the secondary runs delay/, offline_explore/, repeat/ when present.

Fixed before any T1new / T2new result was read:
* **H-RT1**: ``lag_identified`` with IPOPT, real-time against offline mode, on the 105 laps of
  T1new + T2new; exact McNemar (two-sided binomial test on the discordant laps). Holds if its null is
  rejected at its Holm level and more laps are completed only offline than only in real time.
* **H-RT2**: ``lag_identified`` in the real-time mode, RTI against IPOPT, the same laps and test.
  Holds if rejected and more laps are completed only by RTI.
* **Holm** over the two (``report_l7_partC.holm``) and the stricter decision (each at 0.025); where
  they differ, both are stated.
* The main real-time rule is the plan buffer of amendment A1; the rule of revision 1 (the first
  command held) is the secondary run hold_last/.
* Secondary: completion and lap time per controller, implementation and mode; computation time
  (latency and busy time: mean, P95, P99, max) and deadline-miss rate; the loss against the offline
  mode (McNemar, lap time on laps both complete); RTI against IPOPT offline; the ranking per mode
  and Kendall's tau; the exploration laps; the calibration times of the two arms and the machine
  load (l9-M1); the repeat (outcome flips); the sensor delay; the offline exploration (l9-m2).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_l7_partB import ci, load, mcnemar  # noqa: E402
from report_l7_partC import holm  # noqa: E402

KINDS = ("nmpc", "nmpc_reserve", "smpc", "identified", "lag_identified")
IMPLS = ("ipopt", "ipopt_dc", "rti")           # ipopt_dc: the A2 arm (real-time only)
NL = chr(10)


def records(path: Path) -> dict:
    """(kind, impl, case) -> record of a model.jsonl (eval stage, no error)."""
    return {(r["kind"], r["impl"], r["case"]): r for r in load(path) if "error" not in r and r.get("stage") == "eval"}


def comp(rec: dict, kind: str, impl: str) -> dict:
    return {c: bool(r["completed"]) for (k, i, c), r in rec.items() if k == kind and i == impl}


def lap_mean(rec: dict, kind: str, impl: str) -> float:
    t = [r["lap_time"] for (k, i, _), r in rec.items() if k == kind and i == impl and r["completed"]]
    return float(np.mean(t)) if t else float("inf")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=str(ROOT / "results" / "L9"))
    ap.add_argument("--out", default=str(ROOT / "results" / "L9" / "report"))
    a = ap.parse_args()
    d, out = Path(a.dir), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    off, rt = records(d / "offline" / "model.jsonl"), records(d / "realtime" / "model.jsonl")
    errors = {m: sum("error" in r for r in load(d / m / "model.jsonl")) for m in ("offline", "realtime")}
    res = {"errors": errors}
    md = ["# L9 – real-time control under an equal compute budget", "", f"Error records (eval): {errors}.", ""]

    # ---- primary
    x1, y1, p1 = mcnemar(comp(off, "lag_identified", "ipopt"), comp(rt, "lag_identified", "ipopt"))
    x2, y2, p2 = mcnemar(comp(rt, "lag_identified", "rti"), comp(rt, "lag_identified", "ipopt"))
    h = holm({"H-RT1": p1, "H-RT2": p2})
    prim = {"H-RT1": {"a": "offline", "b": "real-time", "only_a": x1, "only_b": y1, "p": p1},
            "H-RT2": {"a": "RTI", "b": "IPOPT", "only_a": x2, "only_b": y2, "p": p2}}
    md += ["## Primary tests: `lag_identified`, exact McNemar on the 105 laps of T1new + T2new", "",
           "| test | comparison | only first : only second | p | Holm level | rejected | holds (Holm) | holds (each at 0.025) |",
           "|---|---|---|---|---|---|---|---|"]
    for k, v in prim.items():
        holds = h[k]["rejected"] and v["only_a"] > v["only_b"]
        strict = v["p"] <= 0.025 and v["only_a"] > v["only_b"]
        v.update({"holm": h[k], "holds": holds, "holds_strict": strict})
        lab = "IPOPT: offline vs real-time" if k == "H-RT1" else "real-time: RTI vs IPOPT"
        md.append(f"| {k} | {lab} | {v['only_a']} : {v['only_b']} | {v['p']:.4g} | {h[k]['level']:.3g} | "
                  f"{h[k]['rejected']} | **{holds}** | {strict}{' – **the two readings differ**' if strict != holds else ''} |")
    res["primary"] = prim

    # ---- completion
    md += ["", "## Completion and lap time", "",
           "| controller | impl | mode | T1new (75) | T2new (30) | total (95 %) | mean lap |", "|---|---|---|---|---|---|---|"]
    res["completion"] = {}
    for k in KINDS:
        for i in IMPLS:
            for m, rec in (("offline", off), ("realtime", rt)):
                rs = [r for (kk, ii, _), r in rec.items() if kk == k and ii == i]
                if not rs:
                    continue
                n = sum(r["completed"] for r in rs)
                lo, hi = ci(n, len(rs))
                t1 = sum(r["completed"] for r in rs if r["set"] == "T1new")
                t2 = sum(r["completed"] for r in rs if r["set"] == "T2new")
                lm = lap_mean(rec, k, i)
                res["completion"][f"{k}|{i}|{m}"] = n
                md.append(f"| {k} | {i} | {m} | {t1} | {t2} | {n}/{len(rs)} ({lo:.2f}–{hi:.2f}) | "
                          f"{lm:.2f} |" if math.isfinite(lm) else f"| {k} | {i} | {m} | {t1} | {t2} | {n}/{len(rs)} | – |")

    # ---- timing
    md += ["", "## Computation (per call; latency = feedback for RTI; busy = feedback + preparation)", "",
           "| controller | impl | mode | latency ms mean / P95 / P99 / max | busy ms mean / P99 / max | miss rate | fallbacks | "
           "laps / steps past the plan (A1.2) |",
           "|---|---|---|---|---|---|---|---|"]
    for k in KINDS:
        for i in IMPLS:
            for m, rec in (("offline", off), ("realtime", rt)):
                rs = [r for (kk, ii, _), r in rec.items() if kk == k and ii == i and r.get("latency_ms")]
                if not rs:
                    continue
                w = np.array([r["calls"] for r in rs], float)
                lat = np.array([r["latency_ms"] for r in rs])
                busy = np.array([r["busy_ms"] for r in rs])
                miss = (float(np.average([r["miss_rate"] for r in rs], weights=w)) if "miss_rate" in rs[0] else float("nan"))
                md.append(f"| {k} | {i} | {m} | {np.average(lat[:, 0], weights=w):.1f} / {np.median(lat[:, 1]):.1f} / "
                          f"{np.max(lat[:, 2]):.1f} / {np.max(lat[:, 3]):.1f} | {np.average(busy[:, 0], weights=w):.1f} / "
                          f"{np.max(busy[:, 2]):.1f} / {np.max(busy[:, 3]):.1f} | {miss:.3f} | "
                          f"{sum(r['fallbacks'] for r in rs)} | {sum(r.get('plan_exhausted_steps', 0) > 0 for r in rs)} / "
                          f"{sum(r.get('plan_exhausted_steps', 0) for r in rs)} |")

    # ---- A2: the delay-compensated IPOPT arm (secondary, no Holm)
    if any(i == "ipopt_dc" for (_, i, _) in rt):
        md += ["", "## A2: IPOPT with delay compensation (L_hat = 100 ms; real-time; secondary, no Holm)", "",
               "| controller | completed: dc / IPOPT / RTI | dc : IPOPT (only), p | dc : RTI (only), p | latency > L_hat | "
               "solves held to t_k + L_hat / of them a solve would have restarted (a2-M1) |",
               "|---|---|---|---|---|---|"]
        res["A2"] = {}
        for k in ("nmpc", "lag_identified"):
            cd, ci_, cr = comp(rt, k, "ipopt_dc"), comp(rt, k, "ipopt"), comp(rt, k, "rti")
            if not cd:
                continue
            x1, y1, p1 = mcnemar(cd, ci_)
            x2, y2, p2 = mcnemar(cd, cr)
            late = [r.get("late_rate", float("nan")) for (kk, ii, _), r in rt.items() if kk == k and ii == "ipopt_dc"]
            held = sum(r.get("held_to_plan", 0) for (kk, ii, _), r in rt.items() if kk == k and ii == "ipopt_dc")
            rst = sum(r.get("would_restart", 0) for (kk, ii, _), r in rt.items() if kk == k and ii == "ipopt_dc")
            res["A2"][k] = {"dc": sum(cd.values()), "ipopt": sum(ci_.values()), "rti": sum(cr.values()),
                            "dc_vs_ipopt": [x1, y1, p1], "dc_vs_rti": [x2, y2, p2]}
            md.append(f"| {k} | {sum(cd.values())} / {sum(ci_.values())} / {sum(cr.values())} of {len(cd)} | "
                      f"{x1} : {y1}, {p1:.3g} | {x2} : {y2}, {p2:.3g} | {np.nanmean(late):.3f} | {held} / {rst} |")

    # ---- loss against offline; RTI vs IPOPT offline; ranking
    md += ["", "## Loss against the offline mode, and RTI against IPOPT offline (exact McNemar; uncorrected)", "",
           "| controller | impl | offline : real-time (only), p | lap time real-time − offline (mean, n) | RTI : IPOPT offline (only), p |",
           "|---|---|---|---|---|"]
    for k in KINDS:
        for i in IMPLS:
            co, cr = comp(off, k, i), comp(rt, k, i)
            if not co or not cr:
                continue
            x, y, p = mcnemar(co, cr)
            dt = [rt[(k, i, c)]["lap_time"] - off[(k, i, c)]["lap_time"] for c in co
                  if co[c] and cr.get(c) and (k, i, c) in rt]
            cell = "–"
            if i == "rti":
                xr, yr, pr = mcnemar(comp(off, k, "rti"), comp(off, k, "ipopt"))
                cell = f"{xr} : {yr}, {pr:.3g}"
            md.append(f"| {k} | {i} | {x} : {y}, {p:.3g} | {(np.mean(dt) if dt else float('nan')):+.2f} s ({len(dt)}) | {cell} |")
    from scipy.stats import kendalltau

    ctrls = [(k, i) for k in KINDS for i in IMPLS if comp(off, k, i) and comp(rt, k, i)]
    md += ["", "Ranking per mode (more laps first, ties by lower mean lap time); Kendall's tau of the counts between "
           "the modes (descriptive):", ""]
    for m, rec in (("offline", off), ("realtime", rt)):
        order = sorted(ctrls, key=lambda c: (-sum(comp(rec, *c).values()), lap_mean(rec, *c)))
        md.append(f"* {m}: " + " > ".join(f"{k}/{i}" for k, i in order))
    if len(ctrls) > 1:
        tau = kendalltau([sum(comp(off, *c).values()) for c in ctrls], [sum(comp(rt, *c).values()) for c in ctrls]).statistic
        md.append(f"* tau = {tau:.2f}")

    # ---- exploration laps
    md += ["", "## Exploration laps (`nmpc_reserve`, shared by identified / lag_identified)", "",
           "| impl | mode | completed | distance median | latency ms mean | miss rate |", "|---|---|---|---|---|---|"]
    for m in ("offline", "realtime"):
        ex = [r for r in load(d / m / "explore.jsonl") if "error" not in r]
        for i in IMPLS:
            rs = [r for r in ex if r["impl"] == i]
            if rs:
                md.append(f"| {i} | {m} | {sum(r['completed'] for r in rs)}/{len(rs)} | {np.median([r['distance'] for r in rs]):.0f} m | "
                          f"{np.mean([r['latency_ms'][0] for r in rs if r.get('latency_ms')]):.1f} | "
                          f"{np.mean([r.get('miss_rate', float('nan')) for r in rs]):.3f} |")

    # ---- timing fidelity (l9-M1)
    md += ["", "## Timing fidelity (l9-M1): calibration times of the two arms; machine load", ""]
    for stage, f in (("eval", "model.jsonl"), ("explore", "explore.jsonl")):
        rs = [r for r in load(d / "realtime" / f) if "error" not in r and "calibration_s" in r]
        for i in IMPLS:
            c = np.array([r["calibration_s"] for r in rs if r["impl"] == i]) * 1e3
            if len(c):
                md.append(f"* real-time {stage}, {i}: calibration median {np.median(c):.1f} ms, P5–P95 "
                          f"{np.percentile(c, 5):.1f}–{np.percentile(c, 95):.1f} ms, n = {len(c)}; vCPUs "
                          f"{sorted(Counter(x for r in rs if r['impl'] == i for x in r['cpu']).items())}")
    ld = load(d / "realtime" / "load.jsonl")
    for stage in ("explore", "eval"):
        s = [x for x in ld if x.get("stage") == stage]
        if s:
            wsl = [np.mean(list(x["wsl"].values())) for x in s]
            win = [float({k.lower(): v for k, v in x["windows"].items()}.get("_total", "nan"))  # Get-Counter: "_total"
                   for x in s if isinstance(x.get("windows"), dict)]
            md.append(f"* load during real-time {stage} ({len(s)} samples): WSL mean over vCPUs {np.mean(wsl):.1f} % "
                      f"(max {np.max(wsl):.1f} %), Windows total {np.nanmean(win):.1f} % (max {np.nanmax(win):.1f} %)")

    # ---- RL policies, bare tracker (no plan: the previous action held), Part C students
    for part, f, lab in (("rl", "rl.jsonl", "RL policies and the bare tracker (the nine L5 finals first)"),
                         ("student", "student.jsonl", "Part C: DAgger students of the RTI `nmpc` (secondary)")):
        ro = {(r["kind"], r["case"]): r for r in load(d / "offline" / f) if "error" not in r}
        rr = {(r["kind"], r["case"]): r for r in load(d / "realtime" / f) if "error" not in r}
        if not ro and not rr:
            continue
        md += ["", f"## {lab}", "",
               "| policy | offline (95 %) | real-time (95 %) | offline : real-time (only), p | latency ms mean / P99 / max | miss rate |",
               "|---|---|---|---|---|---|"]
        names = sorted({k for k, _ in ro} | {k for k, _ in rr}, key=lambda n: (n == "B_s1_500k", n == "tracker", n))
        for p in names:
            co = {c: bool(r["completed"]) for (k, c), r in ro.items() if k == p}
            cr = {c: bool(r["completed"]) for (k, c), r in rr.items() if k == p}
            cells = []
            for cc in (co, cr):
                n = sum(cc.values())
                lo, hi = ci(n, len(cc)) if cc else (float("nan"), float("nan"))
                cells.append(f"{n}/{len(cc)} ({lo:.2f}–{hi:.2f})" if cc else "–")
            x, y, pv = mcnemar(co, cr) if co and cr else (0, 0, float("nan"))
            lat = [r for (k, _), r in rr.items() if k == p and r.get("latency_ms")]
            tl = (f"{np.mean([r['latency_ms'][0] for r in lat]):.2f} / {max(r['latency_ms'][2] for r in lat):.2f} / "
                  f"{max(r['latency_ms'][3] for r in lat):.2f}") if lat else "–"
            mr = f"{np.mean([r['miss_rate'] for r in lat]):.3f}" if lat else "–"
            md.append(f"| {p} | {cells[0]} | {cells[1]} | {x} : {y}, {pv:.3g} | {tl} | {mr} |")

    # ---- repeat, delay, offline exploration
    rep = records(d / "repeat" / "model.jsonl")
    if rep:
        md += ["", "## Repeat of the first 20 laps in the real-time mode (outcome flips)", ""]
        for k in KINDS:
            for i in IMPLS:
                a1, a2 = comp(rt, k, i), comp(rep, k, i)
                common = [c for c in a2 if c in a1]
                if common:
                    md.append(f"* {k}/{i}: {sum(a1[c] != a2[c] for c in common)} of {len(common)} outcomes differ")
        ex1, ex2 = ({(r["impl"], r["case"]): bool(r["completed"]) for r in load(d / m / "explore.jsonl") if "error" not in r}
                    for m in ("realtime", "repeat"))
        for i in IMPLS:                                  # the exploration laps too (A2.6)
            common = [c for (i_, c) in ex2 if i_ == i and (i, c) in ex1]
            if common:
                md.append(f"* exploration/{i}: {sum(ex1[(i, c)] != ex2[(i, c)] for c in common)} of {len(common)} "
                          f"outcomes differ")
    for tag, lab in (("delay", "20 ms sensor delay (real-time)"),
                     ("offline_explore", "real-time evaluated lap, offline exploration (l9-m2)"),
                     ("hold_last", "the rule of revision 1: the first command held, no pre-start plan (A1, secondary)")):
        sec = records(d / tag / "model.jsonl")
        if sec:
            md += ["", f"## {lab}", "", "| controller | impl | completed | against the main real-time run (only this : only main), p |",
                   "|---|---|---|---|"]
            for k in KINDS:
                for i in IMPLS:
                    cs = comp(sec, k, i)
                    if cs:
                        x, y, p = mcnemar(cs, comp(rt, k, i))
                        md.append(f"| {k} | {i} | {sum(cs.values())}/{len(cs)} | {x} : {y}, {p:.3g} |")
            srl = {(r["kind"], r["case"]): bool(r["completed"]) for r in load(d / tag / "rl.jsonl") if "error" not in r}
            mrl = {(r["kind"], r["case"]): bool(r["completed"]) for r in load(d / "realtime" / "rl.jsonl")
                   if "error" not in r}
            for p_ in sorted({k for k, _ in srl}):
                cs = {c: v for (k, c), v in srl.items() if k == p_}
                cm = {c: v for (k, c), v in mrl.items() if k == p_}
                x, y, p = mcnemar(cs, cm)
                md.append(f"| {p_} | policy | {sum(cs.values())}/{len(cs)} | {x} : {y}, {p:.3g} |")

    (out / "L9.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "L9.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"written {out / 'L9.md'}")


if __name__ == "__main__":
    main()
