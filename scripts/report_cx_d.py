"""Control expansion phase D report (docs/CX_D_protocol.md, revision 0 with its clarifications).

    python scripts/report_cx_d.py tune        # CDV: the rule of section 4 -> results/CX/D/selected.json + report/tune.md
    python scripts/report_cx_d.py test        # CDT (H-D1) and CDD (H-D2): Holm and the stricter decision, discordant cars,
                                              # the gap to adaptive_oracle first (review m1), the secondary comparisons
                                              # (14 McNemar, 16 signed-rank), adaptation diagnostics; NOM apart
                                              # -> report/CX_D.md, report/CX_D.json
    python scripts/report_cx_d.py secondary [--phase-c PATH]   # CDT_NOISE, CDT_LAG30, CDT_WET85, CDP, CXD (10 McNemar,
                                              # uncorrected), the bias of mu on CDT_NOISE, nmpc's reproducibility on CXD
                                              # against phase C's secondary.jsonl -> report/CX_D_secondary.md

H-D1 / H-D2: exact McNemar, adaptive vs nmpc. Holm over the two (alpha 0.05) and each at 0.025; CDT and CDD are the same
cars, so the two are not independent pieces of evidence (review m2). Everything else uncorrected, with the number of
comparisons.
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

from f1race.experiments import cx_d  # noqa: E402

NL = chr(10)
TIMING_KEYS = ("wall_s", "wall_job_s", "solve_ms_mean_p95_p99_max", "solves_over_50ms", "job", "stage", "replicate")
PRIMARY = {"H-D1": "CDT", "H-D2": "CDD"}
PAIRS = [("nmpc_param", "nmpc"), ("adaptive", "tube"), ("adaptive", "identified"), ("adaptive", "adaptive_oracle"),
         ("adaptive_bank", "adaptive"), ("tube", "nmpc"), ("identified", "nmpc")]
AHEAD = ("| controller | |μ̂ − μf| P50 / P95 | μ̂ − μf at the end (mean) | |factor error| at the end, mean over the 7 "
         "(mean) | plan η_p P5 / P50 / P95 | too-fast share | settled from the start (laps; median s) | drift: settle delay "
         "after onset median s (never) | drift: |error| P95 before / during / after | one-step RMS (scaled) | clipped "
         "covariance share | estimator µs per update |")
ASEP = "|" + "---|" * 12


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
    md = ["# CX-D tuning on CDV (protocol section 4)", ""]
    for kind in cx_d.GRIDS:
        sel = cx_d.select(recs, kind)
        out["kinds"][kind] = {"setting": sel["setting"], "setting_key": sel["setting_key"]}
        md += [f"## {kind} – chosen: `{sel['setting_key']}`", "", "| setting | completed of CDV (constant / drift) | mean lap on "
               "the laps all tied settings complete | |μ̂ − μf| P95 (mean) | drift settle delay median s (never) |",
               "|---|---|---|---|---|"]
        for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
            rs = [r for r in recs if r["kind"] == kind and r["setting_key"] == k]
            c_ = sum(r["completed"] for r in rs if r["set"] == "CDV_C")
            d_ = sum(r["completed"] for r in rs if r["set"] == "CDV_D")
            m = t.get("mean_lap_common")
            md.append(f"| `{k}` | {t['completed']}/{t['laps']} ({c_} / {d_}) | {'' if m is None else f'{m:.3f} s'} | "
                      f"{_m(rs, 'mu_abs_err_p50_p95', 1)} | {_delay(rs)} |")
        md.append("")
    (d / "selected.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def _delay(rs: list[dict]) -> str:
    v = [r.get("drift_settle_delay_s") for r in rs if r.get("drift") is not None and "drift_settle_delay_s" in r]
    if not v:
        return "–"
    ok = [x for x in v if x is not None]
    return f"{np.median(ok):.2f} ({len(v) - len(ok)})" if ok else f"– ({len(v)})"


def _settings(d: Path) -> dict:
    sel = json.loads((d / "selected.json").read_text())
    if sel.get("tune_sha256") != _sha(d / "tune.jsonl"):
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    lam = sel["kinds"]["adaptive"]["setting"]
    return {"nmpc": {}, "tube": {}, "identified": {}, "adaptive_oracle": {}, "adaptive": lam,
            "adaptive_bank": sel["kinds"]["adaptive_bank"]["setting"], "nmpc_param": dict(lam)}


def _check(recs: list[dict], st: dict, sets: tuple[str, ...], kinds) -> None:
    _errors(recs, "the records")
    need = {c["name"] for s_ in sets for c in cx_d.cases(s_)}
    for k in kinds:
        have = {r["case"] for r in recs if r["kind"] == k}
        if have != need:
            raise SystemExit(f"{k}: {len(need - have)} laps missing, {len(have - need)} unexpected")
        bad = [r["case"] for r in recs if r["kind"] == k and r["setting"] != st[k]]
        if bad:
            raise SystemExit(f"{k}: records with a setting other than the selected one ({bad[:3]})")


def _arow(rs: list[dict], kind: str) -> str:
    fe = [float(np.mean(np.abs(r["factor_err_end"]))) for r in rs if r.get("factor_err_end")]
    st = [r.get("settle_from_start_s") for r in rs if "settle_from_start_s" in r]
    st_ok = [x for x in st if x is not None]
    dr = [r for r in rs if r.get("mu_abs_err_before_during_after_p50_p95")]
    seg = lambda i: _m([{"v": r["mu_abs_err_before_during_after_p50_p95"][i][1]} for r in dr], "v")
    est = [r["estimator"] for r in rs if r.get("estimator")]
    return (f"| {kind} | {_m(rs, 'mu_abs_err_p50_p95', 0)} / {_m(rs, 'mu_abs_err_p50_p95', 1)} | {_m(rs, 'mu_err_end')} | "
            f"{f'{np.mean(fe):.4f}' if fe else '–'} | {_m(rs, 'plan_eta_p5_p50_p95', 0)} / {_m(rs, 'plan_eta_p5_p50_p95', 1)} / "
            f"{_m(rs, 'plan_eta_p5_p50_p95', 2)} | {_m(rs, 'too_fast_share')} | {len(st_ok)}/{len(st)}; "
            f"{f'{np.median(st_ok):.2f}' if st_ok else '–'} | {_delay(rs)} | "
            f"{(seg(0) + ' / ' + seg(1) + ' / ' + seg(2)) if dr else '–'} | {_m(est, 'one_step_rms_scaled')} | "
            f"{_m(est, 'clipped_share')} | {_m(est, 'update_us_mean')} |")


def _pair(by: dict, a_: str, b_: str) -> dict:
    ca = {c: bool(r["completed"]) for c, r in by[a_].items()}
    cb = {c: bool(r["completed"]) for c, r in by[b_].items()}
    x, y, p = mcnemar(ca, cb)
    cc = sorted(c for c in ca if ca[c] and cb.get(c))
    dd = [by[a_][c]["lap_time"] - by[b_][c]["lap_time"] for c in cc]
    return {"mcnemar": [x, y, p], "both": len(cc), "diffs": dd, "signed_rank": exact_signed_rank(dd)}


def _fail(r: dict) -> str:
    s = f"{r['case']} {r['failure_s']:.0f} m" if r.get("failure_s") is not None else f"{r['case']} ({r['stop_reason']})"
    if r.get("failure_s_minus_onset") is not None:
        s += f" (onset {r['failure_s_minus_onset']:+.0f} m)"
    return s


def test(d: Path) -> None:
    recs = load(d / "test.jsonl")
    st = _settings(d)
    K = cx_d.KINDS
    _check(recs, st, ("CDT", "NOM", "CDD"), K)
    by = {s_: {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == s_} for k in K} for s_ in ("CDT", "CDD")}
    n = {s_: len(by[s_]["adaptive"]) for s_ in by}
    md = ["# CX-D – the adaptive NMPC against the NMPC (docs/CX_D_protocol.md)", "",
          "Error records: none; every controller has the CDT cars, the nominal car and the CDD cases with the settings "
          "chosen on CDV: " + "; ".join(f"{k} `{cx_d.setting_key(v)}`" for k, v in st.items()) + ".", ""]
    prim = {h: _pair(by[s_], "adaptive", "nmpc") for h, s_ in PRIMARY.items()}
    hb = holm({h: v["mcnemar"][2] for h, v in prim.items()})
    md += ["## Primary tests", "",
           "| test | set | only adaptive : only nmpc | adaptive / nmpc completed | p | Holm level | holds (Holm) | holds (each "
           "at 0.025) |", "|---|---|---|---|---|---|---|---|"]
    for h, s_ in PRIMARY.items():
        x, y, p = prim[h]["mcnemar"]
        ca, cn = (sum(r["completed"] for r in by[s_][k].values()) for k in ("adaptive", "nmpc"))
        md.append(f"| {h} | {s_} ({n[s_]}) | {x} : {y} | {ca} / {cn} of {n[s_]} | {p:.4g} | {hb[h]['level']:.3f} | "
                  f"**{hb[h]['rejected']}** | {p <= 0.025} |")
    md += ["", "CDT and CDD are the same 50 cars: the two tests are not independent pieces of evidence (review m2).", ""]
    gap = {s_: _pair(by[s_], "adaptive", "adaptive_oracle") for s_ in by}
    md += ["## The gap to `adaptive_oracle` (review m1: the main quantity for reading H-D1)", "",
           "| set | only adaptive : only adaptive_oracle | adaptive / oracle completed | lap time adaptive − oracle on the laps "
           "both complete: n, mean / median s, signed-rank p |", "|---|---|---|---|"]
    for s_ in by:
        g = gap[s_]
        dd = g["diffs"]
        ca, co = (sum(r["completed"] for r in by[s_][k].values()) for k in ("adaptive", "adaptive_oracle"))
        md.append(f"| {s_} | {g['mcnemar'][0]} : {g['mcnemar'][1]} | {ca} / {co} | {g['both']}, "
                  f"{(np.mean(dd) if dd else math.nan):+.3f} / {(np.median(dd) if dd else math.nan):+.3f}, "
                  f"{g['signed_rank']['p']:.4g} |")
    for h, s_ in PRIMARY.items():
        md += ["", f"Discordant cases of {h} ({s_}):", "", "| case | adaptive | nmpc |", "|---|---|---|"]
        for c in sorted(by[s_]["adaptive"]):
            if by[s_]["adaptive"][c]["completed"] != by[s_]["nmpc"][c]["completed"]:
                md.append(f"| {c} | {_cell(by[s_]['adaptive'][c])} | {_cell(by[s_]['nmpc'][c])} |")
    out_sec = {}
    md += ["", "## Secondary comparisons (uncorrected; 14 McNemar and 16 signed-rank: 7 pairs × 2 sets, and the lap time of "
           "`adaptive` – `nmpc` on both sets; `nmpc_param` vs `nmpc` is the numerical-path noise floor)", "",
           "| set | comparison | only first : only second | McNemar p | both complete | lap time first − second mean / median s | "
           "signed-rank p |", "|---|---|---|---|---|---|---|"]
    for s_ in by:
        for a_, b_ in [("adaptive", "nmpc")] + PAIRS:
            r_ = prim[[h for h, v in PRIMARY.items() if v == s_][0]] if (a_, b_) == ("adaptive", "nmpc") else _pair(by[s_], a_, b_)
            out_sec[f"{s_}:{a_}-{b_}"] = {k: v for k, v in r_.items() if k != "diffs"}
            dd = r_["diffs"]
            mc = "(primary)" if (a_, b_) == ("adaptive", "nmpc") else f"{r_['mcnemar'][2]:.4g}"
            md.append(f"| {s_} | {a_} vs {b_} | {r_['mcnemar'][0]} : {r_['mcnemar'][1]} | {mc} | {r_['both']} | "
                      f"{(np.mean(dd) if dd else math.nan):+.3f} / {(np.median(dd) if dd else math.nan):+.3f} | "
                      f"{r_['signed_rank']['p']:.4g} |")
    for s_, h in (("CDT", "H-D1"), ("CDD", "H-D2")):
        fl = out_sec[f"{s_}:nmpc_param-nmpc"]["mcnemar"]
        md.append("")
        md.append(f"Noise floor on {s_}: `nmpc_param` vs `nmpc` has {fl[0] + fl[1]} discordant cases, against "
                  f"{sum(prim[h]['mcnemar'][:2])} for {h}.")
    for s_ in by:
        md += ["", f"## Stop reasons and failure positions – {s_}", "", "| controller | stop reasons | not completed: case, s at "
               "failure (relative to the drift onset) |", "|---|---|---|"]
        for k in K:
            rs = sorted(by[s_][k].values(), key=lambda r: r["case"])
            counts: dict = {}
            for r in rs:
                counts[r["stop_reason"]] = counts.get(r["stop_reason"], 0) + 1
            fails = ", ".join(_fail(r) for r in rs if not r["completed"]) or "–"
            md.append(f"| {k} | " + ", ".join(f"{v} {c}" for c, v in sorted(counts.items())) + f" | {fails} |")
        md += ["", f"### Metrics of section 5 – {s_} (means over the laps; P99, max and maxima: the largest)", "", HEAD, SEP]
        md += [_row(list(by[s_][k].values()), k) for k in K]
        md += ["", f"### Adaptation diagnostics – {s_}", "", AHEAD, ASEP]
        md += [_arow(list(by[s_][k].values()), k) for k in cx_d.ADAPTIVE_KINDS]
    idr = [r for r in recs if r["kind"] == "identified"]
    ex_done = sum(1 for r in idr if r["set"] == "CDT" and r.get("identified", {}).get("explore", {}).get("completed"))
    fails = sorted({r["case"] + ": " + r["stop_reason"] for r in idr if r["stop_reason"] in ("identification failed", "plan failed")})
    md += ["", "## `identified`: the exploration laps and the identification", "",
           f"Exploration laps completed (CDT cars): {ex_done} of {n['CDT']}. Identification / plan failures: "
           + (", ".join(fails) if fails else "none") + ". CDD reuses each car's cached exploration lap, identification and "
           "plan (computed for CDT; `identified.cached`)."]
    nom = {k: [r for r in recs if r["kind"] == k and r["set"] == "NOM"] for k in K}
    md += ["", "## The nominal car (development car; not part of a test)", "", HEAD, SEP] + [_row(nom[k], k) for k in K]
    md += ["", AHEAD, ASEP] + [_arow(nom[k], k) for k in cx_d.ADAPTIVE_KINDS]
    md += ["", "Definitions: *μf* – the true friction factor at the car's position (the car's μ times the drift f(s)); "
           "*factor error* – estimate minus truth of the seven factors at the last solve; *plan η_p* – the friction margin of "
           "the plan coordinate; *too fast* – solves where η_p > 0.95 μf + 0.01 (the plan asked for more grip than the car "
           "had); *settled* – the error below 0.02 at every later solve; the drift's settle delay is counted from the first "
           "solve past the onset, *never* when the error at the last solve is still ≥ 0.02. *`nmpc_param`* – `adaptive`'s "
           "code path with the nominal car and the 5 % plan handed to the NLP (the RLS runs): numerical-path noise. "
           "*`adaptive_oracle`* gets the true car and friction (oracle). Plan-line offset and speed / plan speed against the "
           "5 % plan for every controller. All times include the load of the parallel jobs.", "",
           "What is not claimed (protocol section 3.8): certainty equivalence, no excitation management, exact states and the "
           "simulator's step as the estimator's model (an upper bound), a plan library over friction only, no guarantee.", "",
           "Second environment: not run – H-D1 and H-D2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_D.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_D.json").write_text(json.dumps({"primary": {h: {k: v for k, v in r_.items() if k != "diffs"}
                                                           for h, r_ in prim.items()}, "holm": hb,
                                               "oracle_gap": {s_: {k: v for k, v in g.items() if k != "diffs"}
                                                              for s_, g in gap.items()}, "secondary": out_sec},
                                              indent=1, default=float))
    print(f"written {out / 'CX_D.md'}")


def _comparable(r: dict) -> dict:
    return {k: v for k, v in r.items() if k not in TIMING_KEYS}


def secondary(d: Path, phase_c: Path | None) -> None:
    """Section 2's secondary conditions with the four controllers; adaptive against nmpc and tube by exact McNemar
    (uncorrected, 10); the bias of mu on CDT_NOISE; nmpc's CXD laps against phase C's records in every field both have
    that does not measure time (JSON text)."""
    recs = load(d / "secondary.jsonl")
    st = _settings(d)
    _check(recs, st, cx_d.SECONDARY, cx_d.SECONDARY_KINDS)
    md = ["# CX-D – secondary conditions (protocol section 2; uncorrected, 10 McNemar)", ""]
    out = {}
    for set_ in cx_d.SECONDARY:
        bs = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_} for k in cx_d.SECONDARY_KINDS}
        md += [f"## {set_} ({len(bs['adaptive'])} laps)", "", "| controller | completed (95 %) | mean lap s | not completed: "
               "lap, s at failure |", "|---|---|---|---|"]
        for k in cx_d.SECONDARY_KINDS:
            rs = sorted(bs[k].values(), key=lambda r: r["case"])
            done = [r for r in rs if r["completed"]]
            lt = f"{np.mean([r['lap_time'] for r in done]):.3f}" if done else "–"
            fails = ", ".join(f"{r['case']} {r['stop_reason']} {r['failure_s']:.0f} m" for r in rs if not r["completed"]) or "–"
            md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {lt} | {fails} |")
        md.append("")
        for b_ in ("nmpc", "tube"):
            r_ = _pair(bs, "adaptive", b_)
            out[f"{set_}:adaptive-{b_}"] = r_["mcnemar"]
            md.append(f"adaptive : {b_} (only one completes) {r_['mcnemar'][0]} : {r_['mcnemar'][1]}, exact McNemar p = "
                      f"{r_['mcnemar'][2]:.4g} (uncorrected).")
        md += ["", AHEAD, ASEP] + [_arow(list(bs[k].values()), k) for k in ("adaptive", "adaptive_oracle")]
        if set_ == "CDT_NOISE":
            b = [r["mu_bias_last_half"] for r in bs["adaptive"].values() if r.get("mu_bias_last_half") is not None]
            if b:
                md += ["", f"Bias of μ̂ under the measurement noise (`adaptive`, mean of μ̂ − μ over the last half of each lap's "
                       f"solves; {len(b)} laps): mean {np.mean(b):+.4f}, median {np.median(b):+.4f}, range "
                       f"{np.min(b):+.4f} … {np.max(b):+.4f} (the errors-in-variables effect L6 found; review m1)."]
        md.append("")
    md += ["## Reproducibility of nmpc on CXD against phase C", ""]
    if phase_c is None or not phase_c.exists():
        md.append("Phase C's secondary.jsonl not given – not checked.")
    else:
        old = {(r["kind"], r["case"]): r for r in load(phase_c) if "error" not in r}
        same, diff = 0, []
        for r in recs:
            if r["kind"] == "nmpc" and r["set"] == "CXD":
                o = old.get((r["kind"], r["case"]))
                if o is None:
                    diff.append(f"nmpc {r['case']}: missing in phase C")
                    continue
                a_, b_ = _comparable(r), _comparable(o)
                keys = sorted((set(a_) & set(b_)) - {"setting", "setting_key", "set", "plant"})
                bad = [k for k in keys if json.dumps(a_[k], sort_keys=True) != json.dumps(b_[k], sort_keys=True)]
                if bad:
                    diff.append(f"nmpc {r['case']}: {', '.join(bad)}")
                else:
                    same += 1
        md.append(f"Phase C records: `{phase_c}` (SHA-256 {_sha(phase_c)[:12]}…). Identical in every field both records have "
                  f"that does not measure time (the setting's representation and the set name excepted): {same} of "
                  f"{same + len(diff)} laps." + (" Differences (not used as results):" if diff else ""))
        md += [f"* {x_}" for x_ in diff]
    o = d / "report"
    o.mkdir(exist_ok=True)
    (o / "CX_D_secondary.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (o / "CX_D_secondary.json").write_text(json.dumps(out, indent=1, default=float))
    print(f"written {o / 'CX_D_secondary.md'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test", "secondary"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "D"))
    ap.add_argument("--phase-c", default=None, help="secondary: phase C's secondary.jsonl")
    a = ap.parse_args()
    d = Path(a.dir)
    if a.what == "secondary":
        secondary(d, Path(a.phase_c) if a.phase_c else None)
    else:
        {"tune": tune, "test": test}[a.what](d)


if __name__ == "__main__":
    main()
