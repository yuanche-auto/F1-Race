"""Control expansion phase J report (docs/CX_J_protocol.md, revision 0 with its clarifications).

    python scripts/report_cx_j.py tune    # section 4: the selection of ra's (rho0, rho_s) on JV -> selected.json, report/tune.md
    python scripts/report_cx_j.py test    # section 6: H-J1 (JD, ra against adaptive, exact McNemar), H-J2 (JN + JL, ra against
                                          # tube, per-car exact signed-rank), Holm and the stricter decision, the secondary
                                          # comparisons (with their number), the lap times of JN and JL on the cars both
                                          # complete (description), the adaptation and tube diagnostics; NOM apart
                                          # -> report/CX_J.md, report/CX_J.json

H-J1: exact McNemar. H-J2: per car the number of completed laps of JN and JL (0, 1, 2); Wilcoxon signed-rank on the
differences, zero differences dropped, mid-ranks, the exact distribution. The report gives the numbers of cars improved,
worsened and unchanged and the exact 95 % interval of the improved share among the changed ones. "Not significant" is not
"no worse": no non-inferiority is claimed.
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

from f1race.experiments import cx_j  # noqa: E402

KINDS = cx_j.TEST_KINDS
SETS = {"C": "JT", "D": "JD", "N": "JN", "L": "JL"}


def _errors(recs: list[dict], name: str) -> None:
    bad = [r["job"] for r in recs if "error" in r]
    if bad:
        raise SystemExit(f"{len(bad)} error records in {name}: {bad[:3]}")


def tune(d: Path) -> None:
    path = d / "tune.jsonl"
    recs = load(path)
    _errors(recs, "tune.jsonl")
    sel = cx_j.select(recs, full=True)
    (d / "report").mkdir(parents=True, exist_ok=True)
    out = {"tune_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "ra": sel}
    (d / "selected.json").write_text(json.dumps(out, indent=1, default=float))
    md = ["# CX-J – tuning on JV", "", f"{len(recs)} laps; chosen: {sel['setting']}", "",
          "| setting | completed | mean lap on the common cars |", "|---|---|---|"]
    for k, t in sorted(sel["table"].items()):
        md.append(f"| {k} | {t['completed']} / {t['laps']} | {t.get('mean_lap_common', float('nan')):.3f} |")
    (d / "report" / "tune.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md))


def _by(recs: list[dict], set_: str) -> dict:
    out: dict = {}
    for r in recs:
        if r["case"].startswith(f"{set_}_car") and r.get("kind"):
            out.setdefault(r["kind"], {})[int(r["case"].rsplit("_car", 1)[1])] = r
    return out


def _done(rs: dict) -> dict:
    return {c: bool(r["completed"]) for c, r in rs.items()}


def _mc(a: dict, b: dict) -> dict:
    x, y, p = mcnemar(_done(a), _done(b))
    return {"only_first": x, "only_second": y, "p": p}


def _times(a: dict, b: dict) -> list[float]:
    return [a[c]["lap_time"] - b[c]["lap_time"] for c in sorted(a) if c in b and a[c]["completed"] and b[c]["completed"]]


def interval(k: int, n: int, level: float = 0.95) -> list[float]:
    """Exact (Clopper-Pearson) interval of k of n."""
    from scipy.stats import beta
    if n == 0:
        return [math.nan, math.nan]
    a = 1 - level
    lo = 0.0 if k == 0 else float(beta.ppf(a / 2, k, n - k + 1))
    hi = 1.0 if k == n else float(beta.ppf(1 - a / 2, k + 1, n - k))
    return [lo, hi]


def h_j1(jd: dict) -> dict:
    """H-J1: ra against adaptive on JD (one lap per car)."""
    r = _mc(jd["ra"], jd["adaptive"])
    n = r["only_first"] + r["only_second"]
    return {**r, "completed": [sum(_done(jd["ra"]).values()), sum(_done(jd["adaptive"]).values())],
            "improved_share_interval": interval(r["only_first"], n), "improved": r["only_first"], "worsened": r["only_second"],
            "unchanged": len(jd["ra"]) - n}


def h_j2(jn: dict, jl: dict) -> dict:
    """H-J2: the per-car number of completed JN + JL laps, ra against tube."""
    cars = sorted(jn["ra"])
    a = [int(jn["ra"][c]["completed"]) + int(jl["ra"][c]["completed"]) for c in cars]
    b = [int(jn["tube"][c]["completed"]) + int(jl["tube"][c]["completed"]) for c in cars]
    d = [x - y for x, y in zip(a, b)]
    sr = exact_signed_rank(d)
    up, dn = sum(x > 0 for x in d), sum(x < 0 for x in d)
    return {"p": sr["p"], "n_nonzero": sr["n"], "W_plus": sr["W_plus"], "improved": up, "worsened": dn,
            "unchanged": len(d) - up - dn, "mean_diff": float(np.mean(d)),
            "improved_share_interval": interval(up, up + dn), "laps_completed": [sum(a), sum(b)], "cars": len(cars)}


def _guards(recs: list[dict], selected: dict) -> None:
    """Review m3: no case twice for a kind; each set, kind and car number 1 ... 50 (``adaptive_oracle`` on JT and JD); the ``ra``
    kinds' settings the selected one (the others use none)."""
    seen: set = set()
    for r in recs:
        key = (r["kind"], r["case"])
        if key in seen:
            raise SystemExit(f"{key} twice")
        seen.add(key)
        if r["kind"] in cx_j.RA_KINDS and cx_j.setting_key(r["setting"]) != cx_j.setting_key(selected):
            raise SystemExit(f"{key}: setting {r['setting']} is not the selected {selected}")
    for c in "CDNL":
        kinds = KINDS + (("adaptive_oracle",) if c in "CD" else ())
        for k in kinds:
            cars = {int(x.rsplit("_car", 1)[1]) for kk, x in seen if kk == k and x.startswith(f"{SETS[c]}_car")}
            if cars != set(range(1, 51)):
                raise SystemExit(f"{SETS[c]} {k}: {len(cars)} cars instead of 1 ... 50")


N_SECONDARY = 21                                        # section 6: 3 + 2 + 2 + 2 + 12


def _num(rs: list[dict], f) -> float:
    v = [x for x in (f(r) for r in rs) if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return float(np.mean(v)) if v else math.nan


def _failures(rs: dict) -> str:
    """Where the lost laps end: the number by corner of the failure positions (m, rounded to 50) and the stop reasons."""
    out: dict = {}
    for r in rs.values():
        if not r["completed"]:
            pos = r.get("failure_s")
            key = f"{50 * round(pos / 50):.0f} m" if pos is not None else r["stop_reason"]
            out[key] = out.get(key, 0) + 1
    return ", ".join(f"{k} {v}" for k, v in sorted(out.items(), key=lambda t: -t[1])) or "-"


def diagnostics(S: dict) -> list[str]:
    """Section 5 for the ``ra`` kinds (means over the 50 laps of a set) and the solver's work for every kind."""
    md = ["## Diagnostics (means over the laps of a set; description)", "",
          "| set | kind | solve ms | tube ms | fallbacks / lap | h_mu at cap | err <= h (mu) | q n [m] | q vx [m/s] | e0 n | d_s n | eta_k P50 | back-off P50 [m] | eta capped | track capped | coverage exceed |",
          "|---|---|" + "---|" * 14]
    for c in "CDNL":
        for k in (*KINDS, *(("adaptive_oracle",) if c in "CD" and "adaptive_oracle" in S[c] else ())):
            rs = list(S[c][k].values())
            ra = k in cx_j.RA_KINDS
            row = [SETS[c], k, f"{_num(rs, lambda r: r['solve_ms_mean_p95_p99_max'][0]):.0f}",
                   f"{_num(rs, lambda r: r['tube_ms_mean_p95_max'][0]):.1f}" if ra else "-",
                   f"{_num(rs, lambda r: r['fallbacks']):.1f}"]
            if ra:
                q = [_num(rs, lambda r, i=i: r["q_p50_p95_per_component"][i][0]) for i in range(7)]
                rho = rs[0]["setting"] if k in ("ra", "ra_fixed_res") else {"rho0": 0.0, "rho_s": 0.0}
                row += [f"{_num(rs, lambda r: r['at_cap_share_per_param'][0]):.2f}",
                        f"{_num(rs, lambda r: r['err_within_h_share_per_param'][0]):.2f}", f"{q[1]:.3f}", f"{q[3]:.3f}",
                        f"{rho['rho0'] * q[1]:.3f}", f"{rho['rho_s'] * q[1]:.4f}",
                        f"{_num(rs, lambda r: r['eta_k_p5_p50_p95'][1]):.3f}",
                        f"{_num(rs, lambda r: r['backoff_m_p5_p50_p95'][1]):.3f}",
                        f"{_num(rs, lambda r: r['capped_eta_share_mean']):.3f}",
                        f"{_num(rs, lambda r: r['capped_track_share_mean']):.3f}",
                        f"{_num(rs, lambda r: r['coverage_exceed_share']):.3f}"]
            else:
                row += ["-"] * 11
            md.append("| " + " | ".join(row) + " |")
    md += ["", "Where the lost laps end (number of laps by position, rounded to 50 m):", ""]
    for c in "CDNL":
        for k in KINDS:
            md.append(f"* {SETS[c]} `{k}`: {_failures(S[c][k])}")
    md += ["", "Parameter error at the lap end against sigma, and the shares |error| <= h per parameter (mu, m, ClA, CdA, Iz, B, aero_front):", ""]
    for c in "CDNL":
        rs = list(S[c]["ra"].values())
        md.append(f"* {SETS[c]} `ra`: |error| <= h on "
                  + ", ".join(f"{_num(rs, lambda r, i=i: r['err_within_h_share_per_param'][i]):.2f}" for i in range(7)) + "; sigma P50 "
                  + ", ".join(f"{_num(rs, lambda r, i=i: r['sigma_p50_per_param'][i]):.3f}" for i in range(7)) + "; mean mu error at the lap end "
                  f"{_num(rs, lambda r: r['mu_err_end']):+.3f}; too-fast share {_num(rs, lambda r: r['too_fast_share']):.2f}")
    return md


COMP = ("s", "n", "xi", "vx", "vy", "r", "delta")


def always_reported(S: dict) -> list[str]:
    """Section 5's list for every controller (review of packet 3, m2): the laps, the limits, the effort and the solver – means over
    the 50 laps of a set (lap time, failure distance and stop reasons on their own laps); the residual terms' per-component P50
    and P95 (q, e0 = rho0 q, d_s = rho_s q) and the adaptation metrics of the adaptive controllers."""
    md = ["## Always reported (section 5; means over the 50 laps of a set; description)", "",
          "| set | kind | lap time mean / median / min / max [s] | failure distance median [m] | stop reasons of the lost laps | edge margin min [m] | centre offset mean / max [m] | plan-line offset mean / max [m] | tyre usage max f / r | steer rate rms | force incr. rms [N] | track slack max | limit slack max | solve ms P95 / P99 / max | not converged | fallbacks |",
          "|---|---|" + "---|" * 14]
    for c in "CDNL":
        for k in (*KINDS, *(("adaptive_oracle",) if c in "CD" and "adaptive_oracle" in S[c] else ())):
            rs = list(S[c][k].values())
            done = [r["lap_time"] for r in rs if r["completed"]]
            lost = [r for r in rs if not r["completed"]]
            stops: dict = {}
            for r in lost:
                stops[r["stop_reason"]] = stops.get(r["stop_reason"], 0) + 1
            dist = [r["distance"] for r in lost]
            m = lambda f: _num(rs, f)                                             # noqa: E731
            md.append("| " + " | ".join([
                SETS[c], k, (f"{np.mean(done):.2f} / {np.median(done):.2f} / {min(done):.2f} / {max(done):.2f}" if done else "-"),
                f"{np.median(dist):.0f}" if dist else "-", ", ".join(f"{a} {n}" for a, n in sorted(stops.items())) or "-",
                f"{m(lambda r: r['min_edge_margin']):.2f}",
                f"{m(lambda r: r['centre_offset_abs_mean_max'][0]):.2f} / {m(lambda r: r['centre_offset_abs_mean_max'][1]):.2f}",
                f"{m(lambda r: r['plan_line_offset_abs_mean_max'][0]):.2f} / {m(lambda r: r['plan_line_offset_abs_mean_max'][1]):.2f}",
                f"{m(lambda r: r['tyre_usage_max_front_rear'][0]):.2f} / {m(lambda r: r['tyre_usage_max_front_rear'][1]):.2f}",
                f"{m(lambda r: r['steer_rate_rms']):.3f}", f"{m(lambda r: r['force_increment_rms_N']):.0f}",
                f"{m(lambda r: r['track_slack_max_mean'][0]):.3f}", f"{m(lambda r: r['limit_slack_max']):.4f}",
                f"{m(lambda r: r['solve_ms_mean_p95_p99_max'][1]):.0f} / {m(lambda r: r['solve_ms_mean_p95_p99_max'][2]):.0f} / {m(lambda r: r['solve_ms_mean_p95_p99_max'][3]):.0f}",
                f"{m(lambda r: r['not_converged']):.1f}", f"{m(lambda r: r['fallbacks']):.1f}"]) + " |")
    md += ["", "Residual terms of the `ra` kinds with residual terms, per component (mean of the laps' P50 / P95; q, then e0 = rho0 q and"
           " d_s = rho_s q with the lap's setting):", ""]
    for c in "CDNL":
        for k in ("ra", "ra_fixed_res"):
            rs = list(S[c][k].values())
            s = rs[0]["setting"]
            for i, name in enumerate(COMP):
                q50 = _num(rs, lambda r, i=i: r["q_p50_p95_per_component"][i][0])
                q95 = _num(rs, lambda r, i=i: r["q_p50_p95_per_component"][i][1])
                if i == 0:
                    line = f"* {SETS[c]} `{k}` (rho0 {s['rho0']:g}, rho_s {s['rho_s']:g}): "
                line = line + f"{name} q {q50:.4g} / {q95:.4g}, e0 {s['rho0'] * q50:.4g} / {s['rho0'] * q95:.4g}, d_s {s['rho_s'] * q50:.4g} / {s['rho_s'] * q95:.4g}; "
            md.append(line.rstrip("; "))
    md += ["", "Adaptation metrics (phase D's; `adaptive`, `adaptive_oracle` and the `ra` kinds):", "",
           "| set | kind | mu error P50 / P95 | mu error at the lap end | too-fast share | settled from [s] (median of those that settle) | never settled | plan eta P50 | drift: settle delay [s] (median) |",
           "|---|---|---|---|---|---|---|---|---|"]
    for c in "CDNL":
        for k in (("adaptive", "adaptive_oracle", *cx_j.RA_KINDS) if c in "CD" else ("adaptive", *cx_j.RA_KINDS)):
            rs = list(S[c].get(k, {}).values())
            if not rs or "mu_abs_err_p50_p95" not in rs[0]:
                continue
            st = [r["settle_from_start_s"] for r in rs if r.get("settle_from_start_s") is not None]
            dl = [r["drift_settle_delay_s"] for r in rs if r.get("drift_settle_delay_s") is not None]
            md.append(f"| {SETS[c]} | {k} | {_num(rs, lambda r: r['mu_abs_err_p50_p95'][0]):.3f} / {_num(rs, lambda r: r['mu_abs_err_p50_p95'][1]):.3f} | "
                      f"{_num(rs, lambda r: r['mu_err_end']):+.3f} | {_num(rs, lambda r: r['too_fast_share']):.2f} | "
                      f"{(f'{np.median(st):.1f}' if st else '-')} | {sum(r.get('settle_from_start_s') is None for r in rs)} | "
                      f"{_num(rs, lambda r: r['plan_eta_p5_p50_p95'][1]):.3f} | " + (f"{np.median(dl):.1f}" if dl else "-") + " |")
    return md


PAR = ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front")


def box_and_tightening(S: dict) -> list[str]:
    """The rest of section 5's list for the ``ra`` kinds (added after the first report, description only): each h_i (P5 / P50 / P95)
    and its share of steps at the cap, sigma_i against |error_i| at the lap end, the back-off and eta_k (P5 / P50 / P95), the tube's
    time, the coverage ratio; means over the 50 laps of a set. And the laps with many fallbacks."""
    md = ["## Box and tightening of the `ra` kinds (section 5; means over the 50 laps of a set; description)", "",
          "| set | kind | h per parameter, P5 / P50 / P95 | at the cap, share per parameter | abs. error and sigma at the lap end, per parameter |"
          " back-off P5 / P50 / P95 [m] | eta_k P5 / P50 / P95 | tube ms mean / P95 / max | realised error over bound P50 / P95 |",
          "|---|---|" + "---|" * 7]
    for c in "CDNL":
        for k in cx_j.RA_KINDS:
            rs = list(S[c][k].values())
            tri = lambda key, n=3: " / ".join(f"{_num(rs, lambda r, j=j: r[key][j]):.3f}" for j in range(n))      # noqa: E731
            h = "; ".join(PAR[i] + " " + " / ".join(f"{_num(rs, lambda r, i=i, j=j: r['h_p5_p50_p95_per_param'][i][j]):.3f}" for j in range(3))
                          for i in range(7))
            cap = "; ".join(f"{PAR[i]} {_num(rs, lambda r, i=i: r['at_cap_share_per_param'][i]):.2f}" for i in range(7))
            es = "; ".join(f"{PAR[i]} {_num(rs, lambda r, i=i: abs(r['err_end_vs_sigma_end'][i][0])):.3f}, "
                           f"{_num(rs, lambda r, i=i: r['err_end_vs_sigma_end'][i][1]):.3f}" for i in range(7))
            tm = " / ".join(f"{_num(rs, lambda r, j=j: r['tube_ms_mean_p95_max'][j]):.1f}" for j in range(3))
            md.append(f"| {SETS[c]} | {k} | {h} | {cap} | {es} | {tri('backoff_m_p5_p50_p95')} | {tri('eta_k_p5_p50_p95')} | {tm} | "
                      f"{tri('coverage_err_over_bound_p50_p95', 2)} |")
    md += ["", "Laps with 20 fallbacks or more (lost among them), and the most on one lap:", ""]
    for c in "CDNL":
        md.append(f"* {SETS[c]}: " + ", ".join(
            f"`{k}` {sum(r['fallbacks'] >= 20 for r in S[c][k].values())} "
            f"({sum(r['fallbacks'] >= 20 and not r['completed'] for r in S[c][k].values())}), max {max(r['fallbacks'] for r in S[c][k].values())}"
            for k in KINDS))
    return md


def nominal(recs: list[dict]) -> list[str]:
    """NOM in the four conditions, apart from the tests."""
    md = ["## NOM (apart; one lap per condition)", "", "| kind | constant | drift | noise | lag |", "|---|---|---|---|---|"]
    by: dict = {}
    for r in recs:
        if r["case"].startswith("NOM_"):
            by.setdefault(r["kind"], {})[r["case"].split("_")[1]] = r
    for k in KINDS:
        md.append(f"| {k} | " + " | ".join(
            (f"{by[k][c]['lap_time']:.2f} s" if by[k][c]["completed"] else f"off ({by[k][c].get('failure_s', 0):.0f} m)") if c in by.get(k, {}) else "-"
            for c in "CDNL") + " |")
    return md


def _complete(d: Path, n: int = 1528) -> None:
    """Review of packet 3, n1: the file's lines (``load`` keeps the last record of a job) – ``n`` lines, distinct jobs, the 28 NOM
    laps (seven kinds in four conditions), no error record."""
    rows = [json.loads(l) for l in (d / "test.jsonl").read_text(encoding="utf-8").splitlines() if l.strip()]
    jobs = [r["job"] for r in rows]
    if len(rows) != n or len(set(jobs)) != n:
        raise SystemExit(f"test.jsonl: {len(rows)} lines, {len(set(jobs))} distinct jobs instead of {n}")
    nom = {(r["kind"], r["case"]) for r in rows if r["case"].startswith("NOM_")}
    want = {(k, f"NOM_{c}_car0") for k in KINDS for c in "CDNL"}
    if nom != want:
        raise SystemExit(f"NOM laps: {sorted(nom ^ want)[:4]}")


def test(d: Path) -> None:
    _complete(d)
    recs = load(d / "test.jsonl")
    _errors(recs, "test.jsonl")
    _guards(recs, json.loads((d / "selected.json").read_text())["ra"]["setting"])
    S = {c: _by(recs, SETS[c]) for c in "CDNL"}
    h1, h2 = h_j1(S["D"]), h_j2(S["N"], S["L"])
    hm = holm({"H-J1": h1["p"], "H-J2": h2["p"]})
    out = {"H-J1": h1, "H-J2": h2, "holm": hm, "each_at_0.025": {"H-J1": h1["p"] <= 0.025, "H-J2": h2["p"] <= 0.025}}
    sec: dict = {"completions": {}, "lap_time": {}, "ablations": {}, "jn_jl_lap_time_both_complete": {}}
    for k in ("adaptive", "tube", "nmpc"):
        sec["completions"][f"JT ra - {k}"] = _mc(S["C"]["ra"], S["C"][k])
    for k in ("tube", "adaptive"):
        dd = _times(S["C"]["ra"], S["C"][k])
        sec["lap_time"][f"JT ra - {k}"] = {"n": len(dd), "mean": float(np.mean(dd)) if dd else math.nan, **exact_signed_rank(dd)}
    sec["completions"]["JD ra - tube"] = _mc(S["D"]["ra"], S["D"]["tube"])
    sec["completions"]["JD ra - adaptive_oracle"] = _mc(S["D"]["ra"], S["D"]["adaptive_oracle"]) \
        if "adaptive_oracle" in S["D"] else None
    for c in "NL":
        sec["completions"][f"{SETS[c]} ra - adaptive"] = _mc(S[c]["ra"], S[c]["adaptive"])
    for c in "CDNL":
        for k in ("ra_fixed", "ra_shrink", "ra_fixed_res"):
            sec["ablations"][f"{SETS[c]} ra - {k}"] = _mc(S[c]["ra"], S[c][k])
    for c in "NL":
        for k in ("tube", "adaptive"):
            dd = _times(S[c]["ra"], S[c][k])
            sec["jn_jl_lap_time_both_complete"][f"{SETS[c]} ra - {k}"] = {
                "n": len(dd), "mean": float(np.mean(dd)) if dd else math.nan, "median": float(np.median(dd)) if dd else math.nan}
    out["secondary"] = sec
    out["completed"] = {SETS[c]: {k: sum(_done(S[c][k]).values()) for k in KINDS} for c in "CDNL"}
    for k in ("adaptive_oracle",):
        for c in "CD":
            if k in S[c]:
                out["completed"][SETS[c]][k] = sum(_done(S[c][k]).values())
    (d / "report").mkdir(parents=True, exist_ok=True)
    (d / "report" / "CX_J.json").write_text(json.dumps(out, indent=1, default=float))
    all_kinds = KINDS + (("adaptive_oracle",) if "adaptive_oracle" in out["completed"]["JT"] else ())
    md = ["# CX-J – robust adaptive MPC", "", f"H-J1 (JD, `ra` against `adaptive`): {h1['only_first']} : {h1['only_second']}, p {h1['p']:.4g}; "
          f"completed {h1['completed'][0]} / {h1['completed'][1]}; cars improved / worsened / unchanged "
          f"{h1['improved']} / {h1['worsened']} / {h1['unchanged']}, improved share {h1['improved_share_interval']}",
          f"H-J2 (JN + JL, `ra` against `tube`): per-car signed-rank p {h2['p']:.4g} (n {h2['n_nonzero']}), laps {h2['laps_completed']}; "
          f"cars improved / worsened / unchanged {h2['improved']} / {h2['worsened']} / {h2['unchanged']}, improved share "
          f"{h2['improved_share_interval']}", f"Holm: {hm}", "",
          "| set | " + " | ".join(all_kinds) + " |", "|---|" + "---|" * len(all_kinds)]
    for c in "CDNL":
        md.append(f"| {SETS[c]} | " + " | ".join(fmt_ci(out["completed"][SETS[c]][k], 50) if k in out["completed"][SETS[c]] else "-"
                                               for k in all_kinds) + " |")
    jl = sec["jn_jl_lap_time_both_complete"]
    md += ["", "**H-J2 beside the lap times of the same laps (description, no test; review of packet 2):** where both `ra` and `tube`"
           " complete, the lap time of `ra` minus `tube`:"]
    for key, t in jl.items():
        md.append(f"* {key}: " + (f"{t['n']} cars, mean {t['mean']:+.2f} s, median {t['median']:+.2f} s" if t["n"] else "no car completes both – no lap time"))
    md += ["", f"**Secondary comparisons (uncorrected; {N_SECONDARY} comparisons):**", "```", json.dumps(
        {k: v for k, v in sec.items() if k != "jn_jl_lap_time_both_complete"}, indent=1, default=float), "```", ""]
    md += diagnostics(S) + [""] + always_reported(S) + [""] + box_and_tightening(S) + [""] + nominal(recs)
    (d / "report" / "CX_J.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md[:24]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test"])
    a = ap.parse_args()
    {"tune": tune, "test": test}[a.what](cx_j.OUT)


if __name__ == "__main__":
    main()
