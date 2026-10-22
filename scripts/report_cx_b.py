"""Control expansion phase B report (docs/CX_B_protocol.md, revision 0 with its clarifications and amendments).

    python scripts/report_cx_b.py tune       # CXV: the rule of section 4 -> results/CX/B/selected.json + report/tune.md
    python scripts/report_cx_b.py tune_lp    # mppi_lp on CXV -> results/CX/B/selected_lp.json + report/tune_lp.md
    python scripts/report_cx_b.py test       # CBT: H-B1, H-B2 (Holm and the stricter decision), discordant cars, the
                                             # 12 secondary comparisons, the replicates, the metrics of section 5;
                                             # NOM apart -> report/CX_B.md, report/CX_B.json
    python scripts/report_cx_b.py secondary [--phase-a PATH]   # CBT_LAG30, CXD, CXL8 (uncorrected) and the
                                             # reproducibility check against phase A's secondary.jsonl -> report/CX_B_secondary.md

H-B1: exact McNemar, mppi (replicate 0) vs nmpc on the 50 CBT cars. H-B2: exact signed-rank (mid-ranks, the
exact distribution, scripts/l9_supplement.py) on the lap times of the cars both complete. Holm over the two
(alpha 0.05) and each at 0.025. Everything else uncorrected, with the number of comparisons.
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
from report_l7_partB import mcnemar  # noqa: E402
from report_l7_partC import holm  # noqa: E402

from f1race.experiments import cx_b  # noqa: E402

NL = chr(10)
TIMING_KEYS = ("wall_s", "wall_job_s", "solve_ms_mean_p95_p99_max", "solves_over_50ms", "step_ms_mean_p50_p95_p99_max",
               "steps_over_50ms", "first_call_ms_iters", "job", "stage", "replicate")


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _errors(recs: list[dict], name: str) -> None:
    errors = [r["job"] for r in recs if "error" in r]
    if errors:
        raise SystemExit(f"{len(errors)} error records in {name}, e.g. {errors[:3]} - re-run them first")


def _table(sel: dict) -> list[str]:
    md = ["| setting | completed of CXV | mean lap on the laps all tied settings complete |", "|---|---|---|"]
    for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
        m = t.get("mean_lap_common")
        md.append(f"| `{k}` | {t['completed']}/{t['laps']} | {'' if m is None else f'{m:.3f} s'} |")
    return md


def _diag(recs: list[dict], key: str) -> dict[str, str]:
    """Per setting: the median over the laps of a step statistic (description only)."""
    out = {}
    for k in {r["setting_key"] for r in recs}:
        v = [r[key][1] if isinstance(r[key], list) else r[key] for r in recs if r["setting_key"] == k and r.get(key) is not None]
        out[k] = f"{np.median(v):.3g}" if v else "–"
    return out


def tune(d: Path) -> None:
    path = d / "tune.jsonl"
    recs = load(path)
    _errors(recs, "tune.jsonl")
    Ks = {r["setting"]["K"] for r in recs}
    if len(Ks) != 1 or Ks != {cx_b.K_RT}:
        raise SystemExit(f"tuning records with K {sorted(Ks)}, K_RT is {cx_b.K_RT}")
    sel = cx_b.select(recs, "mppi")
    out = {"tune_sha256": _sha(path), "K": cx_b.K_RT, "inputs": recs[0].get("inputs"),
           "kinds": {"mppi": {"setting": sel["setting"], "setting_key": sel["setting_key"]}}}
    step, ess, nofree = (_diag(recs, "step_ms_mean_p50_p95_p99_max"), _diag(recs, "ess_p5_p50_p95"),
                         _diag(recs, "no_penalty_free_sample_share"))
    md = ["# CX-B tuning of `mppi` on CXV (protocol section 4)", "", f"K = {cx_b.K_RT}, inputs `{out['inputs']}`; "
          f"chosen: `{sel['setting_key']}`", ""] + _table(sel)
    md += ["", "Description (median over the 10 laps; not part of the rule): step time median ms, ESS median, share of "
           "steps without a penalty-free sample.", "", "| setting | step ms | ESS | no penalty-free sample |", "|---|---|---|---|"]
    md += [f"| `{k}` | {step[k]} | {ess[k]} | {nofree[k]} |" for k in sorted(step)]
    (d / "selected.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def tune_lp(d: Path) -> None:
    path = d / "tune_lp.jsonl"
    recs = load(path)
    _errors(recs, "tune_lp.jsonl")
    base = json.loads((d / "selected.json").read_text())["kinds"]["mppi"]["setting"]
    if any({k: r["setting"][k] for k in base} != base for r in recs):
        raise SystemExit("tune_lp.jsonl has records whose base is not mppi's choice in selected.json")
    sel = cx_b.select(recs, "mppi_lp")
    out = {"tune_lp_sha256": _sha(path), "selected_json_sha256": _sha(d / "selected.json"),
           "kinds": {"mppi_lp": {"setting": sel["setting"], "setting_key": sel["setting_key"]}}}
    md = ["# CX-B tuning of `mppi_lp` on CXV (protocol sections 3.5 and 4)", "", f"chosen: `{sel['setting_key']}`", ""]
    (d / "selected_lp.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune_lp.md").write_text(NL.join(md + _table(sel)) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def _settings(d: Path) -> dict:
    sel = json.loads((d / "selected.json").read_text())
    sel_lp = json.loads((d / "selected_lp.json").read_text())
    m = sel["kinds"]["mppi"]["setting"]
    return {**{k: cx_b.baseline_setting(k) for k in cx_b.BASELINES}, "mppi": m,
            "mppi_lp": sel_lp["kinds"]["mppi_lp"]["setting"], "mppi_big": cx_b.big_setting(m)}


def _check(recs: list[dict], st: dict, sets: tuple[str, ...]) -> None:
    """No error records; every controller has every case of ``sets`` (mppi on CBT / NOM with replicates 0-2)
    with the setting chosen on CXV (selected.json / selected_lp.json)."""
    _errors(recs, "the records")
    need = {c["name"] for s_ in sets for c in cx_b.cases(s_)}
    for k in cx_b.KINDS:
        reps = (0, 1, 2) if (k == "mppi" and "CBT" in sets) else (0,)
        for rep in reps:
            have = {r["case"] for r in recs if r["kind"] == k and (r.get("replicate") or 0) == rep}
            if have != need:
                raise SystemExit(f"{k} r{rep}: {len(need - have)} laps missing, {len(have - need)} unexpected")
        bad = [r["case"] for r in recs if r["kind"] == k and r["setting"] != st[k]]
        if bad:
            raise SystemExit(f"{k}: records with a setting other than the selected one ({bad[:3]})")


def _m(rs, key, i=None, f=np.mean):
    v = [r[key] if i is None else r[key][i] for r in rs if r.get(key) is not None]
    v = [x for x in v if x is not None and np.isfinite(x)]
    return f"{f(v):.3f}" if v else "–"


def _row(rs: list[dict], kind: str) -> str:
    done = [r for r in rs if r["completed"]]
    lt = f"{np.mean([r['lap_time'] for r in done]):.3f}" if done else "–"
    if any("step_ms_mean_p50_p95_p99_max" in r for r in rs):
        t = (f"{_m(rs, 'step_ms_mean_p50_p95_p99_max', 0)} / {_m(rs, 'step_ms_mean_p50_p95_p99_max', 2)} / "
             f"{_m(rs, 'step_ms_mean_p50_p95_p99_max', 3, np.max)} / {_m(rs, 'step_ms_mean_p50_p95_p99_max', 4, np.max)}")
        over = _m(rs, "steps_over_50ms")
    elif any("solve_ms_mean_p95_p99_max" in r for r in rs):
        t = (f"{_m(rs, 'solve_ms_mean_p95_p99_max', 0)} / {_m(rs, 'solve_ms_mean_p95_p99_max', 1)} / "
             f"{_m(rs, 'solve_ms_mean_p95_p99_max', 2, np.max)} / {_m(rs, 'solve_ms_mean_p95_p99_max', 3, np.max)}")
        over = _m(rs, "solves_over_50ms")
    else:
        t, over = "–", "–"
    slack = "–"
    if any("track_slack_max_mean" in r for r in rs):
        slack = (f"{_m(rs, 'track_slack_max_mean', 0, np.max)} / {_m(rs, 'track_slack_max_mean', 1)} / "
                 f"{_m(rs, 'limit_slack_max', None, np.max)}")
    nc = (f"{sum(r.get('not_converged', 0) for r in rs)} / {sum(r.get('fallbacks', 0) for r in rs)}"
          if any("fallbacks" in r for r in rs) else "–")
    return (f"| {kind} | {fmt_ci(len(done), len(rs))} | {lt} | {_m(rs, 'min_edge_margin', None, np.min)} | {slack} | "
            f"{_m(rs, 'tyre_usage_max_front_rear', 0)} / {_m(rs, 'tyre_usage_max_front_rear', 1)} | "
            f"{_m(rs, 'tyre_usage_mean_front_rear', 0)} / {_m(rs, 'tyre_usage_mean_front_rear', 1)} | "
            f"{_m(rs, 'centre_offset_abs_mean_max', 0)} / {_m(rs, 'centre_offset_abs_mean_max', 1, np.max)} | "
            f"{_m(rs, 'plan_line_offset_abs_mean_max', 0)} / {_m(rs, 'plan_line_offset_abs_mean_max', 1, np.max)} | "
            f"{_m(rs, 'speed_over_plan_p5_p50_p95', 0)} / {_m(rs, 'speed_over_plan_p5_p50_p95', 1)} / "
            f"{_m(rs, 'speed_over_plan_p5_p50_p95', 2)} | {_m(rs, 'steer_rate_rms')} | {_m(rs, 'force_increment_rms_N')} | "
            f"{_m(rs, 'force_increment_rms_50ms_N')} | {_m(rs, 'overlap_N_p50_p90_max', 0)} / "
            f"{_m(rs, 'overlap_N_p50_p90_max', 1)} / {_m(rs, 'overlap_N_p50_p90_max', 2, np.max)} | {t} | {over} | {nc} |")


HEAD = ("| controller | completed (95 %) | mean lap s | min edge margin m | track slack max / mean, limit slack max | "
        "max tyre use F / R | mean tyre use F / R | centre offset mean / max m | plan-line offset mean / max m | "
        "speed / plan speed P5 / P50 / P95 | steer-rate RMS | force increment RMS N (10 ms) | force increment RMS N (50 ms) | "
        "drive-and-brake overlap N P50 / P90 / max | step or solve ms mean / P95 / P99 / max | share > 50 ms | "
        "not converged / fallbacks |")
SEP = "|" + "---|" * 17
MHEAD = ("| controller | first call ms (iterations) | steps without a valid sample | no penalty-free sample (share of steps) | "
         "best sample penalised (share) | ESS P5 / P50 / P95 | penalised samples (mean share) | invalid samples total / max per step | "
         "cost after / before P5 / P50 / P95 | update made the nominal worse (share) |")


def _mrow(rs: list[dict], kind: str) -> str:
    fc = _m(rs, "first_call_ms_iters", 0)
    its = {r["first_call_ms_iters"][1] for r in rs if r.get("first_call_ms_iters")}
    return (f"| {kind} | {fc} ({'/'.join(map(str, sorted(its)))}) | {sum(r['no_valid_steps'] for r in rs)} | "
            f"{_m(rs, 'no_penalty_free_sample_share')} | {_m(rs, 'best_sample_penalised_share')} | "
            f"{_m(rs, 'ess_p5_p50_p95', 0)} / {_m(rs, 'ess_p5_p50_p95', 1)} / {_m(rs, 'ess_p5_p50_p95', 2)} | "
            f"{_m(rs, 'penalised_sample_share_mean')} | {sum(r['invalid_samples_total_max'][0] for r in rs)} / "
            f"{max(r['invalid_samples_total_max'][1] for r in rs)} | {_m(rs, 'cost_after_over_before_p5_p50_p95', 0)} / "
            f"{_m(rs, 'cost_after_over_before_p5_p50_p95', 1)} / {_m(rs, 'cost_after_over_before_p5_p50_p95', 2)} | "
            f"{_m(rs, 'cost_after_worse_share')} |")


def _cell(r: dict) -> str:
    return (f"{r['lap_time']:.3f} s" if r["completed"]
            else f"{r['stop_reason']} at s = {r['failure_s']:.0f} m ({r['distance']:.0f} m)")


def test(d: Path) -> None:
    recs = load(d / "test.jsonl")
    st = _settings(d)
    _check(recs, st, ("CBT", "NOM"))
    by = lambda k, rep=0, set_="CBT": {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_
                                        and (r.get("replicate") or 0) == rep}
    cbt = {k: by(k) for k in cx_b.KINDS}
    comp = {k: {c: bool(r["completed"]) for c, r in v.items()} for k, v in cbt.items()}
    n = len(comp["mppi"])
    md = ["# CX-B – MPPI against the NMPC (docs/CX_B_protocol.md)", "",
          f"Error records: none; every controller has the {n} CBT cars and the nominal car (mppi with replicates 0–2) with "
          "the settings chosen on CXV: " + "; ".join(f"{k} `{cx_b.setting_key(v)}`" for k, v in st.items()) + ".", ""]
    # ---- primary
    x, y, p1 = mcnemar(comp["mppi"], comp["nmpc"])
    both = sorted(c for c in comp["mppi"] if comp["mppi"][c] and comp["nmpc"].get(c))
    diffs = [cbt["mppi"][c]["lap_time"] - cbt["nmpc"][c]["lap_time"] for c in both]
    sr = exact_signed_rank(diffs)
    hb = holm({"H-B1": p1, "H-B2": sr["p"]})
    md += [f"## Primary tests ({n} CBT cars; mppi replicate 0)", "",
           "| test | comparison | result | p | Holm level | holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|",
           f"| H-B1 | completed laps, mppi : nmpc (only one completes) | {x} : {y} (mppi {sum(comp['mppi'].values())}, "
           f"nmpc {sum(comp['nmpc'].values())} of {n}) | {p1:.4g} | {hb['H-B1']['level']:.3f} | **{hb['H-B1']['rejected']}** | "
           f"{p1 <= 0.025} |",
           f"| H-B2 | lap time mppi − nmpc on the {len(both)} cars both complete | mean "
           f"{np.mean(diffs) if diffs else math.nan:+.3f} s, median {np.median(diffs) if diffs else math.nan:+.3f} s, "
           f"W+ {sr['W_plus']:.1f} (n {sr['n']}) | {sr['p']:.4g} | {hb['H-B2']['level']:.3f} | **{hb['H-B2']['rejected']}** | "
           f"{sr['p'] <= 0.025} |", "", "H-B2 is conditional on both completing; the discordant cars of H-B1:", "",
           "| car | mppi | nmpc |", "|---|---|---|"]
    for c in sorted(set(comp["mppi"]) & set(comp["nmpc"])):
        if comp["mppi"][c] != comp["nmpc"][c]:
            md.append(f"| {c} | {_cell(cbt['mppi'][c])} | {_cell(cbt['nmpc'][c])} |")
    # ---- secondary (12)
    rep = {r_: {c: bool(rr["completed"]) for c, rr in by("mppi", r_).items()} for r_ in (1, 2)}
    pairs = [("mppi", "mpcc"), ("mppi_lp", "mppi"), ("mppi_lp", "nmpc"), ("mppi_big", "mppi"), ("mppi_big", "nmpc")]
    md += ["", "## Secondary comparisons on CBT (uncorrected; 12 comparisons: 5 pairs × McNemar and signed-rank, "
           "replicates 1 and 2 of mppi against nmpc)", "",
           "| comparison | only first : only second | McNemar p | both complete | lap time first − second mean / median s | signed-rank p |",
           "|---|---|---|---|---|---|"]
    out_sec = {}
    for a_, b_ in pairs:
        xx, yy, pp = mcnemar(comp[a_], comp[b_])
        cc = sorted(c for c in comp[a_] if comp[a_][c] and comp[b_].get(c))
        dd = [cbt[a_][c]["lap_time"] - cbt[b_][c]["lap_time"] for c in cc]
        s2 = exact_signed_rank(dd)
        out_sec[f"{a_}-{b_}"] = {"mcnemar": [xx, yy, pp], "signed_rank": s2}
        md.append(f"| {a_} vs {b_} | {xx} : {yy} | {pp:.4g} | {len(cc)} | "
                  f"{(np.mean(dd) if dd else math.nan):+.3f} / {(np.median(dd) if dd else math.nan):+.3f} | {s2['p']:.4g} |")
    for r_ in (1, 2):
        xx, yy, pp = mcnemar(rep[r_], comp["nmpc"])
        out_sec[f"mppi_r{r_}-nmpc"] = {"mcnemar": [xx, yy, pp]}
        md.append(f"| mppi replicate {r_} vs nmpc | {xx} : {yy} | {pp:.4g} | – | – | – |")
    # ---- replicates
    reps = {0: comp["mppi"], **rep}
    flip = sorted(c for c in comp["mppi"] if len({reps[r_][c] for r_ in reps}) > 1)
    all3 = sorted(c for c in comp["mppi"] if all(reps[r_][c] for r_ in reps))
    spread = [np.ptp([by("mppi", r_)[c]["lap_time"] for r_ in reps]) for c in all3]
    md += ["", "## Repeatability of mppi (replicates 0, 1, 2 – different random streams)", "",
           f"Completed: {', '.join(f'r{r_} {sum(v.values())}/{n}' for r_, v in reps.items())}. Cars whose outcome changes "
           f"between replicates: {len(flip)}" + (f" ({', '.join(flip)})" if flip else "") + f". Lap-time range over the "
           f"replicates on the {len(all3)} cars all three complete: "
           + (f"median {np.median(spread):.3f} s, max {np.max(spread):.3f} s." if spread else "–")]
    # ---- stop reasons
    md += ["", "## Stop reasons and failure positions on CBT", "",
           "| controller | stop reasons | not completed: car, s at failure (distance) |", "|---|---|---|"]
    for k in cx_b.KINDS:
        rs = sorted(cbt[k].values(), key=lambda r: r["case"])
        counts = {}
        for r in rs:
            counts[r["stop_reason"]] = counts.get(r["stop_reason"], 0) + 1
        fails = ", ".join(f"{r['case']} {r['failure_s']:.0f} m ({r['distance']:.0f} m)" for r in rs if not r["completed"]) or "–"
        md.append(f"| {k} | " + ", ".join(f"{v} {c}" for c, v in sorted(counts.items())) + f" | {fails} |")
    # ---- metrics
    md += ["", "## Metrics of section 5 – CBT (means over the laps; P99, max and maxima: the largest; mppi replicate 0)", "",
           HEAD, SEP] + [_row(list(cbt[k].values()), k) for k in cx_b.KINDS]
    md += ["", "### MPPI diagnostics – CBT (replicate 0)", "", MHEAD, "|" + "---|" * 10]
    md += [_mrow(list(cbt[k].values()), k) for k in cx_b.MPPI_KINDS]
    nom = {k: [r for r in recs if r["kind"] == k and r["set"] == "NOM" and (r.get("replicate") or 0) == 0] for k in cx_b.KINDS}
    md += ["", "## The nominal car (development car; not part of a test)", "", HEAD, SEP] + [_row(nom[k], k) for k in cx_b.KINDS]
    md += ["", MHEAD, "|" + "---|" * 10] + [_mrow(nom[k], k) for k in cx_b.MPPI_KINDS]
    md += ["", "Definitions: *step ms* (MPPI) – the time of the sampling iteration and of computing the applied input, the "
           "first call (20 iterations) left out and reported in the diagnostics; the diagnostic rollouts of the nominal sequence "
           "are outside it. *solve ms* (nmpc, mpcc) – IPOPT. The benchmark runs 20 jobs at once, so all times include that load. "
           "*Overlap* – min(F_drive, −F_brake) of the applied inputs at every simulator step (amendment B1). *No penalty-free "
           "sample* – every valid sample carries a track or limit penalty; *best sample penalised* – the minimum-cost sample does. "
           "*Cost after / before* – the cost of the nominal sequence after and before the step's update (from the same state). "
           "*Plan-line offset* – the lateral distance between the car and the 5 % plan's line at the car's projection (the tracking "
           "error of nmpc and of the MPPIs, which solve nmpc's problem). *Force increment* – as phase A (10 ms not comparable "
           "across controllers; 50 ms descriptive).", "",
           "Second environment: not run – H-B1 and H-B2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_B.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_B.json").write_text(json.dumps({"H-B1": [x, y, p1], "H-B2": sr, "holm": hb, "secondary": out_sec,
                                               "replicate_flips": flip}, indent=1, default=float))
    print(f"written {out / 'CX_B.md'}")


def _comparable(r: dict) -> dict:
    return {k: v for k, v in r.items() if k not in TIMING_KEYS and not k.startswith("overlap")}


def secondary(d: Path, phase_a: Path | None) -> None:
    """Protocol section 2's secondary conditions; mppi against nmpc by exact McNemar (uncorrected, 3 comparisons);
    clarification n3: nmpc's and mpcc's CXD / CXL8 laps against phase A's records, every field that does not measure
    time expected identical."""
    recs = load(d / "secondary.jsonl")
    st = _settings(d)
    _check(recs, st, cx_b.SECONDARY)
    md = ["# CX-B – secondary conditions (protocol section 2; uncorrected)", ""]
    for set_ in cx_b.SECONDARY:
        bs = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_} for k in cx_b.KINDS}
        md += [f"## {set_} ({len(bs['mppi'])} laps)", "", "| controller | completed (95 %) | mean lap s | not completed: lap, s at failure |",
               "|---|---|---|---|"]
        for k in cx_b.KINDS:
            rs = sorted(bs[k].values(), key=lambda r: r["case"])
            done = [r for r in rs if r["completed"]]
            lt = f"{np.mean([r['lap_time'] for r in done]):.3f}" if done else "–"
            fails = ", ".join(f"{r['case']} {r['stop_reason']} {r['failure_s']:.0f} m" for r in rs if not r["completed"]) or "–"
            md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {lt} | {fails} |")
        x, y, p = mcnemar({c: bool(r["completed"]) for c, r in bs["mppi"].items()},
                          {c: bool(r["completed"]) for c, r in bs["nmpc"].items()})
        md += ["", f"mppi : nmpc (only one completes) {x} : {y}, exact McNemar p = {p:.4g} (uncorrected).", ""]
    md += ["## Reproducibility of nmpc and mpcc on CXD and CXL8 against phase A (clarification n3)", ""]
    if phase_a is None or not phase_a.exists():
        md.append("Phase A's secondary.jsonl not given – not checked.")
    else:
        old = {(r["kind"], r["case"]): r for r in load(phase_a) if "error" not in r}
        same, diff = 0, []
        for r in recs:
            if r["kind"] in ("nmpc", "mpcc") and r["set"] in ("CXD", "CXL8"):
                o = old.get((r["kind"], r["case"]))
                a_, b_ = _comparable(r), (_comparable(o) if o else None)
                if b_ is not None:
                    b_.pop("replicate", None)
                    keys = sorted((set(a_) & set(b_)) - {"replicate"})
                    bad = [k for k in keys if a_[k] != b_[k]]
                else:
                    bad = ["missing in phase A"]
                if bad:
                    diff.append(f"{r['kind']} {r['case']}: {', '.join(bad)}")
                else:
                    same += 1
        md.append(f"Phase A records: `{phase_a}` (SHA-256 {_sha(phase_a)[:12]}…). Identical in every field that does not "
                  f"measure time: {same} of {same + len(diff)} laps." + (" Differences (not used as results):" if diff else ""))
        md += [f"* {x_}" for x_ in diff]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_B_secondary.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {out / 'CX_B_secondary.md'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "tune_lp", "test", "secondary"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "B"))
    ap.add_argument("--phase-a", default=None, help="secondary: phase A's secondary.jsonl")
    a = ap.parse_args()
    d = Path(a.dir)
    if a.what == "secondary":
        secondary(d, Path(a.phase_a) if a.phase_a else None)
    else:
        {"tune": tune, "tune_lp": tune_lp, "test": test}[a.what](d)


if __name__ == "__main__":
    main()
