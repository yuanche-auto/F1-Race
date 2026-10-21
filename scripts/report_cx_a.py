"""Control expansion phase A report (docs/CX_A_protocol.md, revision 1).

    python scripts/report_cx_a.py tune     # CXV: the rule of section 4 -> results/CX/A/selected.json + report/tune.md
    python scripts/report_cx_a.py test     # CXT: H-A1, H-A2 (Holm and the stricter decision), discordant cars,
                                           # the metrics of section 5 per controller; NOM apart -> report/CX_A.md

H-A1: exact McNemar, mpcc vs nmpc on the 25 CXT cars. H-A2: exact signed-rank (mid-ranks, the exact
distribution by dynamic programming, scripts/l9_supplement.py) on the lap times of the cars both complete.
Holm over the two (alpha 0.05) and each at 0.025. Everything else uncorrected, with the number of comparisons.
The speed-bound activity at stage 0 is a *measured* quantity (the bound acts on stages 1 ... N).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from l9_supplement import exact_signed_rank  # noqa: E402
from report_l7_partB import ci, mcnemar  # noqa: E402
from report_l7_partC import holm  # noqa: E402

from f1race.experiments.cx_a import KINDS, select  # noqa: E402

NL = chr(10)


def load(p: Path) -> list[dict]:
    """Records of a jsonl file, the last record of each job (a re-run after an error replaces it)."""
    last = {}
    if p.exists():
        for l in p.read_text(encoding="utf-8").splitlines():
            if l.strip():
                r = json.loads(l)
                last[r["job"]] = r
    return list(last.values())


def fmt_ci(k: int, n: int) -> str:
    lo, hi = ci(k, n)
    return f"{k}/{n} ({lo:.2f}–{hi:.2f})" if n else "–"


def tune(d: Path) -> None:
    path = d / "tune.jsonl"
    recs = load(path)
    errors = [r["job"] for r in recs if "error" in r]
    if errors:
        raise SystemExit(f"{len(errors)} error records in tune.jsonl: {errors[:3]}")
    out = {"tune_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "kinds": {}}
    md = ["# CX-A tuning on CXV (protocol section 4)", ""]
    for kind in ("nmpc", "mpcc", "mpcc_centre"):
        sel = select(recs, kind)
        out["kinds"][kind] = {"setting": sel["setting"], "setting_key": sel["setting_key"]}
        md += [f"## {kind} – chosen: `{sel['setting_key']}`", "", "| setting | completed of CXV | mean lap on the laps all tied settings complete |",
               "|---|---|---|"]
        for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
            m = t.get("mean_lap_common")
            md.append(f"| `{k}` | {t['completed']}/{t['laps']} | {'' if m is None else f'{m:.3f} s'} |")
        md.append("")
    (d / "selected.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def _row(rs: list[dict], kind: str) -> str:
    n = len(rs)
    done = [r for r in rs if r["completed"]]
    lt = f"{np.mean([r['lap_time'] for r in done]):.3f}" if done else "–"
    def m(key, i=None, f=np.mean):
        v = [r[key] if i is None else r[key][i] for r in rs if key in r]
        return f"{f(v):.3f}" if v else "–"
    solve = "–"
    if any("solve_ms_mean_p95_p99_max" in r for r in rs):
        solve = (f"{m('solve_ms_mean_p95_p99_max', 0)} / {m('solve_ms_mean_p95_p99_max', 1)} / "
                 f"{m('solve_ms_mean_p95_p99_max', 2, np.max)} / {m('solve_ms_mean_p95_p99_max', 3, np.max)}")
    return (f"| {kind} | {fmt_ci(len(done), n)} | {lt} | {m('min_edge_margin', None, np.min)} | "
            f"{m('tyre_usage_max_front_rear', 0)} / {m('tyre_usage_max_front_rear', 1)} | "
            f"{m('tyre_usage_mean_front_rear', 0)} / {m('tyre_usage_mean_front_rear', 1)} | "
            f"{m('centre_offset_abs_mean_max', 0)} | {m('plan_line_offset_abs_mean_max', 0)} / {m('plan_line_offset_abs_mean_max', 1, np.max)} | "
            f"{m('speed_over_plan_p5_p50_p95', 0)} / {m('speed_over_plan_p5_p50_p95', 1)} / {m('speed_over_plan_p5_p50_p95', 2)} | "
            f"{m('steer_rate_rms')} | {m('force_increment_rms_N')} | {solve} | {m('solves_over_50ms')} | "
            f"{sum(r.get('not_converged', 0) for r in rs)} / {sum(r.get('fallbacks', 0) for r in rs)} | "
            f"{m('bound_active_stage0_measured')} | {m('bound_active_stages_1_N')} |")


def test(d: Path) -> None:
    recs = load(d / "test.jsonl")
    sel = json.loads((d / "selected.json").read_text())
    errors = [r["job"] for r in recs if "error" in r]
    cxt = {k: {r["case"]: r for r in recs if "error" not in r and r["kind"] == k and r["set"] == "CXT"} for k in KINDS}
    comp = {k: {c: bool(r["completed"]) for c, r in v.items()} for k, v in cxt.items()}
    md = ["# CX-A – dynamic MPCC against the NMPC (docs/CX_A_protocol.md, revision 1)", "",
          f"Error records: {len(errors)} {errors[:5]}. Settings chosen on CXV (selected.json, tune.jsonl SHA-256 "
          f"{sel['tune_sha256'][:12]}…): " + "; ".join(f"{k} `{v['setting_key']}`" for k, v in sel["kinds"].items()) + ".", ""]

    # ---- primary tests
    x, y, p1 = mcnemar(comp["mpcc"], comp["nmpc"])
    both = sorted(c for c in comp["mpcc"] if comp["mpcc"][c] and comp["nmpc"].get(c))
    diffs = [cxt["mpcc"][c]["lap_time"] - cxt["nmpc"][c]["lap_time"] for c in both]
    sr = exact_signed_rank(diffs)
    hb = holm({"H-A1": p1, "H-A2": sr["p"]})
    md += ["## Primary tests (25 CXT cars)", "",
           "| test | comparison | result | p | Holm level | holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|",
           f"| H-A1 | completed laps, mpcc : nmpc (only one completes) | {x} : {y} (mpcc {sum(comp['mpcc'].values())}, "
           f"nmpc {sum(comp['nmpc'].values())} of {len(comp['mpcc'])}) | {p1:.4g} | {hb['H-A1']['level']:.3f} | "
           f"**{hb['H-A1']['rejected']}** | {p1 <= 0.025} |",
           f"| H-A2 | lap time mpcc − nmpc on the {len(both)} cars both complete | mean {np.mean(diffs) if diffs else math.nan:+.3f} s, "
           f"median {np.median(diffs) if diffs else math.nan:+.3f} s, W+ {sr['W_plus']:.1f} (n {sr['n']}) | {sr['p']:.4g} | "
           f"{hb['H-A2']['level']:.3f} | **{hb['H-A2']['rejected']}** | {sr['p'] <= 0.025} |", "",
           "H-A2 is conditional on both completing (cx-m2); the discordant cars of H-A1:", "",
           "| car | mpcc | nmpc |", "|---|---|---|"]
    for c in sorted(set(comp["mpcc"]) & set(comp["nmpc"])):
        if comp["mpcc"][c] != comp["nmpc"][c]:
            cell = lambda r: (f"{r['lap_time']:.3f} s" if r["completed"] else f"{r['stop_reason']} at s = {r['failure_s']:.0f} m "
                              f"({r['distance']:.0f} m)")
            md.append(f"| {c} | {cell(cxt['mpcc'][c])} | {cell(cxt['nmpc'][c])} |")

    # ---- secondary comparisons (uncorrected)
    md += ["", "## Secondary comparisons on CXT (exact McNemar, uncorrected; 3 comparisons)", "",
           "| comparison | only first : only second | p |", "|---|---|---|"]
    for a_, b_ in (("mpcc_centre", "nmpc"), ("mpcc_centre", "mpcc"), ("tracker", "nmpc")):
        xx, yy, pp = mcnemar(comp[a_], comp[b_])
        md.append(f"| {a_} vs {b_} | {xx} : {yy} | {pp:.4g} |")

    # ---- section 5 metrics
    head = ("| controller | completed (95 %) | mean lap s | min edge margin m | max tyre use F / R | mean tyre use F / R | "
            "mean centre offset m | plan-line offset mean / max m | speed / plan speed P5 / P50 / P95 | steer-rate RMS | "
            "force-increment RMS N | solve ms mean / P95 / P99 / max | share > 50 ms | not converged / fallbacks | "
            "bound active stage 0 (measured) | bound active stages 1…N |")
    sep = "|" + "---|" * 16
    md += ["", "## Metrics of section 5 – CXT (means over the laps; P99 and max: the largest)", "", head, sep]
    for k in KINDS:
        md.append(_row(list(cxt[k].values()), k))
    md += ["", "## The nominal car (development car; not part of a test)", "", head, sep]
    for k in KINDS:
        md.append(_row([r for r in recs if "error" not in r and r["kind"] == k and r["set"] == "NOM"], k))
    md += ["", "Stage 0 of the bound is the measured speed against v_ref(θ₀ − e_l,0) − 0.01 m/s; the constraint acts on "
           "stages 1 … N only.", "", "Second environment: not run – H-A1 and H-A2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_A.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_A.json").write_text(json.dumps({"H-A1": [x, y, p1], "H-A2": sr, "holm": hb}, indent=1, default=float))
    print(f"written {out / 'CX_A.md'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "A"))
    a = ap.parse_args()
    (tune if a.what == "tune" else test)(Path(a.dir))


if __name__ == "__main__":
    main()
