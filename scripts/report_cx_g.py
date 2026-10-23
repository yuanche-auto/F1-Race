"""Control expansion phase G report (docs/CX_G_protocol.md, revision 0 with its clarifications m1–m3, n1, n2).

    python scripts/report_cx_g.py --tag test        # -> results/CX/G/<tag>/report/CX_G.{md,json}

H-G1 (d = 20 ms) and H-G2 (d = 0): `rti_dc` against `rti`, exact McNemar, Holm (each also at 0.025); the noise floor
of S4 (flips per arm, review m3) next to the discordant counts; S1–S3; per arm the completions by tau / GT2, lap times,
the timing decomposition (section 5), the application delays (m1), the prediction errors.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
NL = chr(10)


def load(p: Path) -> list[dict]:
    """The last record of every job (a job re-run after an error appends a new record; the error record stays in
    the file, as in L9)."""
    last: dict = {}
    for l in (p.read_text().splitlines() if p.exists() else []):
        if l.strip():
            r = json.loads(l)
            last[r["job"]] = r
    return list(last.values())


def mcnemar(a: dict, b: dict) -> tuple[int, int, float]:
    from scipy.stats import binomtest

    keys = sorted(set(a) & set(b))
    x = sum(a[k] and not b[k] for k in keys)
    y = sum(b[k] and not a[k] for k in keys)
    return x, y, float(binomtest(min(x, y), x + y).pvalue) if x + y else 1.0


def ci(k: int, n: int) -> tuple[float, float]:
    from scipy.stats import binomtest

    if not n:
        return float("nan"), float("nan")
    c = binomtest(k, n).proportion_ci(method="exact")
    return float(c.low), float(c.high)


def group(r: dict) -> str:
    return "dist" if r["dist_seed"] is not None else f"tau{int(round(r['tau'] * 1000))}"


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="test")
    a = ap.parse_args()
    from f1race.experiments import cx_g

    d = ROOT / "results" / "CX" / "G" / a.tag
    raw = load(d / "eval.jsonl")
    rep = load(d / "repeat_eval.jsonl")
    rl = load(d / "rl.jsonl")
    errors = {"eval": sum("error" in r for r in raw), "repeat": sum("error" in r for r in rep),
              "rl": sum("error" in r for r in rl)}
    for f, k in (("eval.jsonl", "eval"), ("repeat_eval.jsonl", "repeat"), ("rl.jsonl", "rl")):  # incl. re-run ones
        fp = d / f
        errors[f"{k}_error_records_in_file"] = sum('"error"' in l for l in fp.read_text().splitlines()) if fp.exists() else 0
    ev = [r for r in raw if "error" not in r]
    by = {}
    for r in ev:
        by.setdefault(r["arm"], {})[r["case"]] = r
    comp = {arm: {c: bool(r["completed"]) for c, r in v.items()} for arm, v in by.items()}
    res = {"errors": errors}
    md = [f"# CX-G – delay-compensated RTI under L9's rule ({a.tag})", "", f"Jobs whose last record is an error, and error records in the files (re-run jobs included): {errors}.", "",
          "## Completion per arm (of the laps run; GT1 by tau, GT2)", "",
          "| arm | completed (95 %) | tau 0 | tau 30 | tau 60 | disturbances (GT2) | mean lap | failures: distance P10 / P50 / P90 m |",
          "|---|---|---|---|---|---|---|---|"]
    for arm in list(cx_g.PRIMARY) + list(cx_g.S1) + list(cx_g.S2):
        if arm not in by:
            continue
        rs = list(by[arm].values())
        k, n = sum(r["completed"] for r in rs), len(rs)
        lo, hi = ci(k, n)
        g = {x: sum(r["completed"] for r in rs if group(r) == x) for x in ("tau0", "tau30", "tau60", "dist")}
        gn = {x: sum(group(r) == x for r in rs) for x in g}
        laps = [r["lap_time"] for r in rs if r["completed"]]
        fd = [r["distance"] for r in rs if not r["completed"]]
        dist = " / ".join(f"{np.percentile(fd, q):.0f}" for q in (10, 50, 90)) if fd else "–"
        md.append(f"| {arm} | {k} / {n} ({lo:.2f}–{hi:.2f}) | " + " | ".join(f"{g[x]} / {gn[x]}" for x in g)
                  + f" | {np.mean(laps) if laps else float('nan'):.2f} | {dist} |")
    # primary tests (Holm)
    tests = {}
    for name, (ad, ab) in (("H-G1", ("rti_dc_d20", "rti_d20")), ("H-G2", ("rti_dc_d0", "rti_d0"))):
        if ad in comp and ab in comp:
            x, y, p = mcnemar(comp[ad], comp[ab])
            tests[name] = {"only_dc": x, "only_rti": y, "p": p, "completed_dc": sum(comp[ad].values()),
                           "completed_rti": sum(comp[ab].values()), "laps": len(set(comp[ad]) & set(comp[ab]))}
    ps = sorted(tests, key=lambda k: tests[k]["p"])
    for i, k in enumerate(ps):
        tests[k]["holm_level"] = 0.05 / (len(ps) - i)
        tests[k]["significant"] = bool(all(tests[q]["p"] < tests[q]["holm_level"] for q in ps[:i + 1]))
        tests[k]["below_0.025"] = bool(tests[k]["p"] < 0.025)
    res["primary"] = tests
    md += ["", "## Primary tests (exact McNemar, Holm over H-G1 and H-G2)", "",
           "| test | rti_dc | rti | only rti_dc : only rti | p | Holm level | significant | at 0.025 |",
           "|---|---|---|---|---|---|---|---|"]
    for k in ("H-G1", "H-G2"):
        if k in tests:
            t = tests[k]
            md.append(f"| **{k}** | {t['completed_dc']} | {t['completed_rti']} | {t['only_dc']} : {t['only_rti']} | "
                      f"{t['p']:.3g} | {t['holm_level']:.3f} | {t['significant']} | {t['below_0.025']} |")
    md.append("**Not cross-checked** (the second environment is deferred; the WSL environment is L9's acados "
              "environment, not the second one).")
    # noise floor (S4, review m3)
    if rep:
        rp = {(r["arm"], r["case"]): bool(r["completed"]) for r in rep if "error" not in r}
        flips = {}
        for arm in cx_g.PRIMARY:
            pairs = [(comp[arm][c], v) for (aa, c), v in rp.items() if aa == arm and c in comp.get(arm, {})]
            flips[arm] = {"laps": len(pairs), "flips": sum(x != y for x, y in pairs)}
        res["repeat_flips"] = flips
        md += ["", "## Noise floor (S4: the first laps of every primary arm run again; review m3)", "",
               "| arm | repeated laps | flips |", "|---|---|---|"] + \
              [f"| {k} | {v['laps']} | {v['flips']} |" for k, v in flips.items()]
        md.append("Discordant cases of the primary tests for comparison: "
                  + "; ".join(f"{k} {t['only_dc']} : {t['only_rti']}" for k, t in tests.items()) + ".")
    # secondary comparisons
    sec = {}
    for name, (x1, x2) in (("S1 rti_dc_d40 vs rti_d40", ("rti_dc_d40", "rti_d40")),
                           ("S1 rti_dc20_d40 (wrong d) vs rti_d40", ("rti_dc20_d40", "rti_d40")),
                           ("S1 rti_dc_d40 vs rti_dc20_d40", ("rti_dc_d40", "rti_dc20_d40")),
                           ("S2 nmpc rti_dc vs rti (d 20)", ("nmpc_rti_dc_d20", "nmpc_rti_d20")),
                           ("rti_dc_d20 vs rti_d0", ("rti_dc_d20", "rti_d0"))):
        if x1 in comp and x2 in comp:
            sec[name] = dict(zip(("only_first", "only_second", "p"), mcnemar(comp[x1], comp[x2])))
    res["secondary"] = sec
    md += ["", "## Secondary (exact McNemar, uncorrected, descriptive)", ""] + [f"* {k}: {v}" for k, v in sec.items()]
    if rl:
        rs = [r for r in rl if "error" not in r]
        res["S3_rl"] = {"completed": sum(r["completed"] for r in rs), "laps": len(rs)}
        md.append(f"* S3 `B_s1_500k` at d = 20 ms (context, no compensation): {res['S3_rl']['completed']} / {len(rs)}")
    # lap times on laps both complete
    from scipy.stats import wilcoxon

    md += ["", "## Lap time, `rti_dc` − `rti`, on laps both complete", ""]
    for x1, x2 in (("rti_dc_d0", "rti_d0"), ("rti_dc_d20", "rti_d20")):
        if x1 in by and x2 in by:
            dd = [by[x1][c]["lap_time"] - by[x2][c]["lap_time"] for c in by[x1]
                  if c in by[x2] and by[x1][c]["completed"] and by[x2][c]["completed"]]
            p = float(wilcoxon(dd).pvalue) if len([v for v in dd if v != 0]) > 0 else float("nan")
            md.append(f"* {x1} − {x2}: n {len(dd)}, mean {np.mean(dd) if dd else float('nan'):+.3f} s, "
                      f"signed-rank p {p:.3g}")
    # timing decomposition, application delays, prediction errors (section 5, m1)
    md += ["", "## Timing (section 5), application delays (m1), prediction errors", "",
           "| arm | feedback latency ms mean / P99 max | busy ms mean / P99 max | prediction ms mean / max | late share | "
           "application delay ms: mean, histogram | end-to-end ms (d + application delay, mean) | prediction error "
           "P50 / P95: position m, heading rad, delta rad |", "|---|---|---|---|---|---|---|---|"]
    res["timing"] = {}
    for arm in list(cx_g.PRIMARY) + list(cx_g.S1) + list(cx_g.S2):
        rs = [r for r in by.get(arm, {}).values() if r.get("latency_ms")]
        if not rs:
            continue
        w = [r["calls"] for r in rs]
        lat = float(np.average([r["latency_ms"][0] for r in rs], weights=w))
        busy = float(np.average([r["busy_ms"][0] for r in rs], weights=w))
        tp = [r["time_pred_ms"] for r in rs if r.get("time_pred_ms")]
        late = [r["late_share"] for r in rs if "late_share" in r]
        hist: dict = {}
        tot, n = 0.0, 0
        for r in rs:
            ad = r.get("application_delay", {})
            for k, v in ad.get("histogram_ms", {}).items():
                hist[k] = hist.get(k, 0) + v
                tot += float(k) * v
                n += v
        pe = [r["prediction_error"] for r in rs if r.get("prediction_error")]

        def med(key):
            return float(np.median([p[key] for p in pe])) if pe else float("nan")
        d_ms = cx_g.ARMS[arm][1] * 1e3
        row = {"latency_ms_mean": lat, "busy_ms_mean": busy, "late_share": float(np.mean(late)) if late else None,
               "application_hist_ms": hist, "application_mean_ms": tot / n if n else None,
               "end_to_end_ms_mean": d_ms + (tot / n if n else 0.0),
               "prediction_error_lap_medians": {k: med(k) for k in ("pos_p50", "pos_p95", "heading_p50", "heading_p95",
                                                                     "delta_p50", "delta_p95")}}
        res["timing"][arm] = row
        md.append(f"| {arm} | {lat:.2f} / {max(r['latency_ms'][2] for r in rs):.1f} | {busy:.2f} / "
                  f"{max(r['busy_ms'][2] for r in rs):.1f} | "
                  + (f"{np.mean([t[0] for t in tp]):.3f} / {max(t[2] for t in tp):.2f}" if tp else "–") + " | "
                  + (f"{row['late_share']:.3f}" if late else "–") + f" | {row['application_mean_ms']:.1f}, "
                  + ", ".join(f"{k}: {v}" for k, v in sorted(hist.items(), key=lambda kv: int(kv[0]))) + " | "
                  + f"{row['end_to_end_ms_mean']:.1f} | "
                  + " / ".join(f"{med(k):.3g}" for k in ("pos_p50", "pos_p95", "heading_p50", "heading_p95",
                                                          "delta_p50", "delta_p95")) + " |")
    md.append("(Prediction errors: per lap the median and P95 of |error| over its solves; the table gives the median of "
              "these per-lap values. Actuation delay: the cars' steering lag tau, and the identified one in the plans.)")
    out = d / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_G.json").write_text(json.dumps(res, indent=1, default=float))
    (out / "CX_G.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(NL.join(md))


if __name__ == "__main__":
    main()
