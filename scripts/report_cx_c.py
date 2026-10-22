"""Control expansion phase C report (docs/CX_C_protocol.md, revision 0 with its clarifications).

    python scripts/report_cx_c.py tune        # CXV: the rule of section 4 -> results/CX/C/selected.json + report/tune.md
    python scripts/report_cx_c.py test        # CCT: H-C1, H-C2 (Holm and the stricter decision), discordant cars, the 10
                                              # secondary comparisons (amendment C1), tube vs nmpc_param (descriptive),
                                              # the metrics and tube diagnostics; NOM apart
                                              # -> report/CX_C.md, report/CX_C.json
    python scripts/report_cx_c.py secondary [--phase-b PATH]   # CCT_LAG30, CXD, CXL8 (uncorrected) and nmpc's
                                              # reproducibility against phase B's secondary.jsonl -> report/CX_C_secondary.md

H-C1: exact McNemar, tube vs nmpc on the 50 CCT cars. H-C2: exact signed-rank (mid-ranks, the exact distribution,
scripts/l9_supplement.py) on the lap times of the cars both complete. Holm over the two (alpha 0.05) and each at 0.025.
Everything else uncorrected, with the number of comparisons.
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
from report_cx_a import fmt_ci, load  # noqa: E402
from report_cx_b import HEAD, SEP, _cell, _m, _row  # noqa: E402
from report_l7_partB import mcnemar  # noqa: E402
from report_l7_partC import holm  # noqa: E402

from f1race.experiments import cx_c  # noqa: E402

NL = chr(10)
TIMING_KEYS = ("wall_s", "wall_job_s", "solve_ms_mean_p95_p99_max", "solves_over_50ms", "job", "stage", "replicate")
THEAD = ("| controller | tube ms mean / P95 / P99 / max | tube + solve ms mean / P95 | track back-off m P5 / P50 / P95 | "
         "back-off at stage N m P50 | η_k P5 / P50 / P95 | η at stage N P50 | capped stages: track / η | "
         "coverage: realised error above the back-off | realised / back-off P50 / P95 | realised lateral error m P50 / P95 / max |")
TSEP = "|" + "---|" * 11


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _errors(recs: list[dict], name: str) -> None:
    errors = [r["job"] for r in recs if "error" in r]
    if errors:
        raise SystemExit(f"{len(errors)} error records in {name}, e.g. {errors[:3]} - re-run them first")


def tune(d: Path) -> None:
    path = d / "tune.jsonl"
    recs = load(path)
    _errors(recs, "tune.jsonl")
    out = {"tune_sha256": _sha(path), "kinds": {}}
    md = ["# CX-C tuning on CXV (protocol section 4)", ""]
    for kind in cx_c.TUBE_KINDS:
        sel = cx_c.select(recs, kind)
        out["kinds"][kind] = {"setting": sel["setting"], "setting_key": sel["setting_key"]}
        md += [f"## {kind} – chosen: `{sel['setting_key']}`", "", "| setting | completed of CXV | mean lap on the laps all tied "
               "settings complete | capped track / η (mean) | coverage exceed share (median over laps) |", "|---|---|---|---|---|"]
        for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
            rs = [r for r in recs if r["kind"] == kind and r["setting_key"] == k]
            m = t.get("mean_lap_common")
            md.append(f"| `{k}` | {t['completed']}/{t['laps']} | {'' if m is None else f'{m:.3f} s'} | "
                      f"{_m(rs, 'capped_track_share_mean')} / {_m(rs, 'capped_eta_share_mean')} | "
                      f"{_m(rs, 'coverage_exceed_share', None, np.median)} |")
        md.append("")
    (d / "selected.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def _settings(d: Path) -> dict:
    sel = json.loads((d / "selected.json").read_text())
    if sel.get("tune_sha256") != _sha(d / "tune.jsonl"):
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    return {**{k: cx_c.fixed_setting(k) for k in ("tracker", "nmpc", "nmpc_param", "nmpc_reserve")},
            **{k: sel["kinds"][k]["setting"] for k in cx_c.TUBE_KINDS}}


def _check(recs: list[dict], st: dict, sets: tuple[str, ...], kinds=cx_c.KINDS) -> None:
    _errors(recs, "the records")
    need = {c["name"] for s_ in sets for c in cx_c.cases(s_)}
    for k in kinds:
        have = {r["case"] for r in recs if r["kind"] == k}
        if have != need:
            raise SystemExit(f"{k}: {len(need - have)} laps missing, {len(have - need)} unexpected")
        bad = [r["case"] for r in recs if r["kind"] == k and r["setting"] != st[k]]
        if bad:
            raise SystemExit(f"{k}: records with a setting other than the selected one ({bad[:3]})")


def _trow(rs: list[dict], kind: str) -> str:
    return (f"| {kind} | {_m(rs, 'tube_ms_mean_p95_p99_max', 0)} / {_m(rs, 'tube_ms_mean_p95_p99_max', 1)} / "
            f"{_m(rs, 'tube_ms_mean_p95_p99_max', 2, np.max)} / {_m(rs, 'tube_ms_mean_p95_p99_max', 3, np.max)} | "
            f"{_m(rs, 'step_ms_tube_plus_solve_mean_p95', 0)} / {_m(rs, 'step_ms_tube_plus_solve_mean_p95', 1)} | "
            f"{_m(rs, 'backoff_m_p5_p50_p95', 0)} / {_m(rs, 'backoff_m_p5_p50_p95', 1)} / {_m(rs, 'backoff_m_p5_p50_p95', 2)} | "
            f"{_m(rs, 'backoff_stageN_m_p5_p50_p95', 1)} | {_m(rs, 'eta_k_p5_p50_p95', 0)} / {_m(rs, 'eta_k_p5_p50_p95', 1)} / "
            f"{_m(rs, 'eta_k_p5_p50_p95', 2)} | {_m(rs, 'eta_stageN_p5_p50_p95', 1)} | {_m(rs, 'capped_track_share_mean')} / "
            f"{_m(rs, 'capped_eta_share_mean')} | {_m(rs, 'coverage_exceed_share')} | "
            f"{_m(rs, 'coverage_err_over_bound_p50_p95', 0)} / {_m(rs, 'coverage_err_over_bound_p50_p95', 1)} | "
            f"{_m(rs, 'realised_lateral_error_p50_p95_max_m', 0)} / {_m(rs, 'realised_lateral_error_p50_p95_max_m', 1)} / "
            f"{_m(rs, 'realised_lateral_error_p50_p95_max_m', 2, np.max)} |")


def test(d: Path) -> None:
    recs = load(d / "test.jsonl")
    st = _settings(d)
    K = cx_c.TEST_KINDS
    _check(recs, st, ("CCT", "NOM"), K)
    cct = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == "CCT"} for k in K}
    comp = {k: {c: bool(r["completed"]) for c, r in v.items()} for k, v in cct.items()}
    n = len(comp["tube"])
    md = ["# CX-C – tube MPC against the NMPC (docs/CX_C_protocol.md)", "",
          f"Error records: none; every controller has the {n} CCT cars and the nominal car with the settings chosen on CXV: "
          + "; ".join(f"{k} `{cx_c.setting_key(v)}`" for k, v in st.items()) + ".", ""]
    x, y, p1 = mcnemar(comp["tube"], comp["nmpc"])
    both = sorted(c for c in comp["tube"] if comp["tube"][c] and comp["nmpc"].get(c))
    diffs = [cct["tube"][c]["lap_time"] - cct["nmpc"][c]["lap_time"] for c in both]
    sr = exact_signed_rank(diffs)
    hb = holm({"H-C1": p1, "H-C2": sr["p"]})
    md += [f"## Primary tests ({n} CCT cars)", "",
           "| test | comparison | result | p | Holm level | holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|",
           f"| H-C1 | completed laps, tube : nmpc (only one completes) | {x} : {y} (tube {sum(comp['tube'].values())}, "
           f"nmpc {sum(comp['nmpc'].values())} of {n}) | {p1:.4g} | {hb['H-C1']['level']:.3f} | **{hb['H-C1']['rejected']}** | "
           f"{p1 <= 0.025} |",
           f"| H-C2 | lap time tube − nmpc on the {len(both)} cars both complete | "
           + (f"mean {np.mean(diffs):+.3f} s, median {np.median(diffs):+.3f} s, W+ {sr['W_plus']:.1f} (n {sr['n']})" if diffs
              else "no car completed by both – not testable")
           + f" | {sr['p']:.4g} | {hb['H-C2']['level']:.3f} | **{hb['H-C2']['rejected']}** | {sr['p'] <= 0.025} |", "",
           "H-C2 is conditional on both completing; the discordant cars of H-C1:", "", "| car | tube | nmpc |", "|---|---|---|"]
    for c in sorted(set(comp["tube"]) & set(comp["nmpc"])):
        if comp["tube"][c] != comp["nmpc"][c]:
            md.append(f"| {c} | {_cell(cct['tube'][c])} | {_cell(cct['nmpc'][c])} |")
    pairs = [("tube", "nmpc_reserve"), ("tube_track", "nmpc"), ("tube_track", "tube"), ("nmpc_reserve", "nmpc"),
             ("nmpc_param", "nmpc")]                                                  # the last: amendment C1
    md += ["", "## Secondary comparisons on CCT (uncorrected; 10 comparisons: 5 pairs × McNemar and signed-rank; "
           "`nmpc_param` vs `nmpc` is the numerical-path noise floor of amendment C1)", "",
           "| comparison | only first : only second | McNemar p | both complete | lap time first − second mean / median s | signed-rank p |",
           "|---|---|---|---|---|---|"]
    out_sec = {}
    for a_, b_ in pairs:
        xx, yy, pp = mcnemar(comp[a_], comp[b_])
        cc = sorted(c for c in comp[a_] if comp[a_][c] and comp[b_].get(c))
        dd = [cct[a_][c]["lap_time"] - cct[b_][c]["lap_time"] for c in cc]
        s2 = exact_signed_rank(dd)
        out_sec[f"{a_}-{b_}"] = {"mcnemar": [xx, yy, pp], "signed_rank": s2}
        md.append(f"| {a_} vs {b_} | {xx} : {yy} | {pp:.4g} | {len(cc)} | "
                  f"{(np.mean(dd) if dd else math.nan):+.3f} / {(np.median(dd) if dd else math.nan):+.3f} | {s2['p']:.4g} |")
    xx, yy, pp = mcnemar(comp["tube"], comp["nmpc_param"])
    cc = sorted(c for c in comp["tube"] if comp["tube"][c] and comp["nmpc_param"].get(c))
    dd = [cct["tube"][c]["lap_time"] - cct["nmpc_param"][c]["lap_time"] for c in cc]
    out_sec["tube-nmpc_param (descriptive)"] = {"mcnemar": [xx, yy, pp], "signed_rank": exact_signed_rank(dd)}
    floor = out_sec["nmpc_param-nmpc"]["mcnemar"]
    md += ["", f"Descriptive (amendment C1): `tube` vs `nmpc_param` – the same code path, only the tightening differs: only one "
           f"completes {xx} : {yy}; on the {len(cc)} cars both complete `tube` − `nmpc_param` mean "
           f"{(np.mean(dd) if dd else math.nan):+.3f} s. Discordant cars of `nmpc_param` vs `nmpc` (the noise floor): "
           f"{floor[0] + floor[1]}, against {x + y} for H-C1."]
    md += ["", "## Stop reasons and failure positions on CCT", "",
           "| controller | stop reasons | not completed: car, s at failure (distance) |", "|---|---|---|"]
    for k in K:
        rs = sorted(cct[k].values(), key=lambda r: r["case"])
        counts = {}
        for r in rs:
            counts[r["stop_reason"]] = counts.get(r["stop_reason"], 0) + 1
        fails = ", ".join(f"{r['case']} {r['failure_s']:.0f} m ({r['distance']:.0f} m)" for r in rs if not r["completed"]) or "–"
        md.append(f"| {k} | " + ", ".join(f"{v} {c}" for c, v in sorted(counts.items())) + f" | {fails} |")
    md += ["", "## Metrics of section 5 – CCT (means over the laps; P99, max and maxima: the largest)", "", HEAD, SEP]
    md += [_row(list(cct[k].values()), k) for k in K]
    md += ["", "### Tube diagnostics – CCT", "", THEAD, TSEP] + [_trow(list(cct[k].values()), k) for k in cx_c.TUBE_KINDS]
    nom = {k: [r for r in recs if r["kind"] == k and r["set"] == "NOM"] for k in K}
    md += ["", "## The nominal car (development car; not part of a test)", "", HEAD, SEP] + [_row(nom[k], k) for k in K]
    md += ["", THEAD, TSEP] + [_trow(nom[k], k) for k in cx_c.TUBE_KINDS]
    md += ["", "Definitions: *plan-line offset* and *speed / plan speed* against the 5 % plan for every controller (`nmpc_reserve` "
           "tracks the 12 % plan). *`nmpc_param`* (amendment C1) – `nmpc`'s problem in the tube's parameterised NLP and code path "
           "with no tightening; its differences from `nmpc` are numerical-path noise (protocol section 9, item 7). *Track back-off* – the tube's lateral bound added to the margin, over the stages 1 … N of "
           "every solve; *η_k* the tightened friction factor (it also bounds the axles' longitudinal forces, clarification m2); "
           "*capped* – the share of stages at b_max or η_min (not robust in the tube's sense). *Coverage* – the realised lateral "
           "error of the axle points 1 … 10 stages after a solve against that stage's back-off: the realised error is the "
           "re-solving NMPC's, the bound assumes the ancillary feedback acting, so it is a diagnostic of the tube's scale, not a "
           "test of a guarantee (clarification m1). *Overlap* and *force increment* as phases A / B. All times include the load "
           "of 20 parallel jobs.", "",
           "What is not claimed (protocol section 3.5): a linearisation-based tube, a vertex-maximum disturbance bound at the "
           "nominal point, caps, no terminal set – no recursive feasibility or guaranteed constraint satisfaction.", "",
           "Second environment: not run – H-C1 and H-C2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_C.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_C.json").write_text(json.dumps({"H-C1": [x, y, p1], "H-C2": sr, "holm": hb, "secondary": out_sec},
                                              indent=1, default=float))
    print(f"written {out / 'CX_C.md'}")


def _comparable(r: dict) -> dict:
    return {k: v for k, v in r.items() if k not in TIMING_KEYS}


def secondary(d: Path, phase_b: Path | None) -> None:
    """Section 2's secondary conditions; tube against nmpc by exact McNemar (uncorrected, 3 comparisons); clarification m3:
    nmpc's CXD / CXL8 laps against phase B's records, compared as JSON text field by field (NaN lap times included)."""
    recs = load(d / "secondary.jsonl")
    st = _settings(d)
    _check(recs, st, cx_c.SECONDARY)
    md = ["# CX-C – secondary conditions (protocol section 2; uncorrected)", ""]
    for set_ in cx_c.SECONDARY:
        bs = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_} for k in cx_c.KINDS}
        md += [f"## {set_} ({len(bs['tube'])} laps)", "", "| controller | completed (95 %) | mean lap s | not completed: lap, s at failure |",
               "|---|---|---|---|"]
        for k in cx_c.KINDS:
            rs = sorted(bs[k].values(), key=lambda r: r["case"])
            done = [r for r in rs if r["completed"]]
            lt = f"{np.mean([r['lap_time'] for r in done]):.3f}" if done else "–"
            fails = ", ".join(f"{r['case']} {r['stop_reason']} {r['failure_s']:.0f} m" for r in rs if not r["completed"]) or "–"
            md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {lt} | {fails} |")
        x, y, p = mcnemar({c: bool(r["completed"]) for c, r in bs["tube"].items()},
                          {c: bool(r["completed"]) for c, r in bs["nmpc"].items()})
        md += ["", f"tube : nmpc (only one completes) {x} : {y}, exact McNemar p = {p:.4g} (uncorrected).", ""]
    md += ["## Reproducibility of nmpc on CXD and CXL8 against phase B (clarification m3)", ""]
    if phase_b is None or not phase_b.exists():
        md.append("Phase B's secondary.jsonl not given – not checked.")
    else:
        old = {(r["kind"], r["case"]): r for r in load(phase_b) if "error" not in r}
        same, diff = 0, []
        for r in recs:
            if r["kind"] == "nmpc" and r["set"] in ("CXD", "CXL8"):
                o = old.get((r["kind"], r["case"]))
                if o is None:
                    diff.append(f"nmpc {r['case']}: missing in phase B")
                    continue
                a_, b_ = _comparable(r), _comparable(o)
                bad = [k for k in sorted(set(a_) & set(b_))
                       if json.dumps(a_[k], sort_keys=True) != json.dumps(b_[k], sort_keys=True)]
                if bad:
                    diff.append(f"nmpc {r['case']}: {', '.join(bad)}")
                else:
                    same += 1
        md.append(f"Phase B records: `{phase_b}` (SHA-256 {_sha(phase_b)[:12]}…). Identical in every field that does not "
                  f"measure time: {same} of {same + len(diff)} laps." + (" Differences (not used as results):" if diff else ""))
        md += [f"* {x_}" for x_ in diff]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_C_secondary.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {out / 'CX_C_secondary.md'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test", "secondary"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "C"))
    ap.add_argument("--phase-b", default=None, help="secondary: phase B's secondary.jsonl")
    a = ap.parse_args()
    d = Path(a.dir)
    if a.what == "secondary":
        secondary(d, Path(a.phase_b) if a.phase_b else None)
    else:
        {"tune": tune, "test": test}[a.what](d)


if __name__ == "__main__":
    main()
