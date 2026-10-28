"""Control expansion phase H report (docs/CX_H_protocol.md, revision 0 with its clarifications m1–m3, n1, n2).

    python scripts/report_cx_h.py tune        # HV: the rule of section 4 -> results/CX/H/selected.json + report/tune.md
    python scripts/report_cx_h.py test        # HT: H-H1 (lap time, tenth learning lap vs identified, both complete) and
                                              # H-H2 (completions), Holm and each at 0.025; the secondary comparisons; the
                                              # lap-time curves, survival, per-lap completion, the cars completed vs not,
                                              # initialisation failures apart; the linearisation error per lap with its
                                              # outcome (n1); NOM apart -> report/CX_H.md, report/CX_H.json
    python scripts/report_cx_h.py secondary   # lmpc (phase E's NLP) against atv on HT; HT_LAG30, HT_WET85
    python scripts/report_cx_h.py dev         # section 9: the same tables on the development records (NOM + DEV, K 20)
                                              # and the gate -> results/CX/H/dev/report/
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

from f1race.experiments import cx_h  # noqa: E402

NL = chr(10)
FACTORS = ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front")
LEARN = ("atv", "atv_nominal", "lmpc")


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _errors(recs: list[dict], name: str) -> None:
    errors = [r["job"] for r in recs if "error" in r]
    if errors:
        raise SystemExit(f"{len(errors)} error records in {name}, e.g. {errors[:3]} - re-run them first")


def _last(recs: list[dict]) -> list[dict]:
    """The last record per job (a job re-run after an error appends a record; the error record stays in the file)."""
    out: dict = {}
    for r in recs:
        out[r["job"]] = r
    return list(out.values())


def tune(d: Path) -> None:
    path = d / "tune.jsonl"
    recs = _last(load(path))
    _errors(recs, "tune.jsonl")
    sel = cx_h.select(recs)
    out = {"tune_sha256": _sha(path), "kinds": {"atv": {"setting": sel["setting"], "setting_key": sel["setting_key"]}}}
    md = ["# CX-H tuning on HV (protocol section 4)", "", f"## atv – chosen: `{sel['setting_key']}`", "",
          "| setting | tenth learning laps completed | mean tenth lap on the cars all tied settings complete | started | "
          "first failed learning lap (cars) | lap-time curve, median over the completed cars |", "|---|---|---|---|---|---|"]
    for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
        rs = [r for r in recs if r["kind"] == "atv" and r["setting_key"] == k]
        ff = sorted(r["first_failed_learning_lap"] for r in rs if r["first_failed_learning_lap"] is not None)
        curve = []
        for j in range(12):
            v = [r["lap_times"][j] for r in rs if r["lap_times"][j] is not None]
            curve.append(f"{np.median(v):.1f}" if v else "–")
        m = t.get("mean_lap_common")
        md.append(f"| `{k}` | {t['completed']}/{t['laps']} | {'' if m is None else f'{m:.3f} s'} | "
                  f"{sum(r['started'] for r in rs)} | {ff or '–'} | {' '.join(curve)} |")
    (d / "selected.json").write_text(json.dumps(out, indent=1))
    (d / "report").mkdir(exist_ok=True)
    (d / "report" / "tune.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(json.dumps(out["kinds"], indent=1))


def _settings(d: Path) -> dict:
    sel = json.loads((d / "selected.json").read_text())
    if sel.get("tune_sha256") != _sha(d / "tune.jsonl"):
        raise SystemExit("selected.json was not made from the present tune.jsonl")
    k = sel["kinds"]["atv"]["setting"]
    return {"nmpc": {}, "identified": {}, "atv": k, "atv_nominal": dict(k), "lmpc": dict(k)}


def _check(recs: list[dict], st: dict, sets: tuple[str, ...], kinds: tuple[str, ...]) -> None:
    _errors(recs, "the records")
    need = {c["name"] for s_ in sets for c in cx_h.cases(s_)}
    for k in kinds:
        have = {r["case"] for r in recs if r["kind"] == k}
        if have != need:
            raise SystemExit(f"{k}: {len(need - have)} laps missing, {len(have - need)} unexpected")
        bad = [r["case"] for r in recs if r["kind"] == k and r["setting"] != st[k]]
        if bad:
            raise SystemExit(f"{k}: records with a setting other than the selected one ({bad[:3]})")


def _pair(a: dict, b: dict) -> dict:
    ca = {c: bool(r["completed"]) for c, r in a.items()}
    cb = {c: bool(r["completed"]) for c, r in b.items()}
    x, y, p = mcnemar(ca, cb)
    both = sorted(c for c in ca if ca[c] and cb.get(c))
    dd = [a[c]["lap_time"] - b[c]["lap_time"] for c in both]
    return {"mcnemar": [x, y, p], "both": len(both), "diffs": dd, "signed_rank": exact_signed_rank(dd)}


def _cell(r: dict) -> str:
    if r["completed"]:
        return f"{r['lap_time']:.3f} s"
    if r.get("started") is False:
        return "not started (initialisation failed)"
    if r.get("first_failed_learning_lap"):
        return f"failed at learning lap {r['first_failed_learning_lap']}"
    return f"{r['stop_reason']}" + (f" at s = {r['failure_s']:.0f} m" if r.get("failure_s") is not None else "")


def _curves(rs: list[dict]) -> list[str]:
    lines = ["| lap | " + " | ".join(["init 0.73", "init 0.79"] + [f"L{j}" for j in range(1, 11)]) + " |",
             "|" + "---|" * 13]
    alive = [sum(1 for r in rs if r["started"] and (r["first_failed_learning_lap"] is None or r["first_failed_learning_lap"] > j))
             for j in range(0, 11)]
    entered = [sum(1 for r in rs if r["started"] and (r["first_failed_learning_lap"] is None or r["first_failed_learning_lap"] >= j))
               for j in range(1, 11)]
    done = [sum(1 for r in rs if r["lap_times"][j + 1] is not None) for j in range(1, 11)]
    med = [np.median([r["lap_times"][j] for r in rs if r["lap_times"][j] is not None]) if any(
        r["lap_times"][j] is not None for r in rs) else math.nan for j in range(12)]
    lines.append("| median lap s (completed) | " + " | ".join(f"{m:.2f}" if np.isfinite(m) else "–" for m in med) + " |")
    lines.append("| completed | " + " | ".join(str(sum(r["lap_times"][j] is not None for r in rs)) for j in range(12)) + " |")
    lines.append("| entered (learning) | – | – | " + " | ".join(str(e) for e in entered) + " |")
    lines.append("| completed / entered | – | – | " + " | ".join(f"{dn / e:.2f}" if e else "–" for dn, e in zip(done, entered)) + " |")
    lines.append("| still driving after the lap (survival) | – | " + " | ".join(str(a) for a in alive) + " |")
    return lines


def _lin_err(rs: list[dict]) -> list[str]:
    """Review n1: the linearisation error per learning lap, next to the lap's outcome (median over the laps of the
    per-lap median and P95 of |ds|, |dn|, |dv_x| at N), and the terminal slack share."""
    rows = {"completed": [], "failed": []}
    for r in rs:
        for lp in r["laps"]:
            if lp.get("kind") != "learn" or "lin_err_ds_dn_dvx_p50" not in lp:
                continue
            rows["completed" if lp["completed"] else "failed"].append(lp)
    lines = ["| learning laps | n | |ds| m P50 / P95 | |dn| m P50 / P95 | |dv_x| m/s P50 / P95 | terminal slack > 1e-3 (share) | "
             "QP fallbacks per lap |", "|---|---|---|---|---|---|---|"]
    for lab, lps in rows.items():
        if not lps:
            lines.append(f"| {lab} | 0 | – | – | – | – | – |")
            continue
        p50 = np.median([lp["lin_err_ds_dn_dvx_p50"] for lp in lps], axis=0)
        p95 = np.median([lp["lin_err_ds_dn_dvx_p95"] for lp in lps], axis=0)
        ts = np.median([lp.get("term_slack_share_above_1e-3", math.nan) for lp in lps])
        fb = np.mean([lp.get("fallbacks", 0) for lp in lps])
        lines.append(f"| {lab} | {len(lps)} | {p50[0]:.3g} / {p95[0]:.3g} | {p50[1]:.3g} / {p95[1]:.3g} | "
                     f"{p50[2]:.3g} / {p95[2]:.3g} | {ts:.3f} | {fb:.1f} |")
    return lines


def _report(recs: list[dict], st: dict, test_set: str, kinds: tuple[str, ...], title: str) -> tuple[list[str], dict]:
    by = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == test_set} for k in kinds}
    n = len(by["atv"])
    h1 = _pair(by["atv"], by["identified"])
    p1, p2 = h1["signed_rank"]["p"], h1["mcnemar"][2]
    hb = holm({"H-H1": p1, "H-H2": p2})
    dd = h1["diffs"]
    md = [title, "",
          "Settings: " + "; ".join(f"{k} `{cx_h.setting_key(st[k])}`" for k in kinds) + ".", "",
          "## Primary tests (the tenth learning lap of `atv` against `identified`)", "",
          "| test | comparison | result | p | Holm level | holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|",
          f"| H-H1 | lap time `atv` − `identified` on the {h1['both']} cars both complete | "
          + (f"mean {np.mean(dd):+.3f} s, median {np.median(dd):+.3f} s, W+ {h1['signed_rank']['W_plus']:.1f} (n {h1['signed_rank']['n']})"
             if dd else "no car completed by both – not testable")
          + f" | {p1:.4g} | {hb['H-H1']['level']:.3f} | **{hb['H-H1']['rejected']}** | {p1 <= 0.025} |",
          f"| H-H2 | completed laps, only `atv` : only `identified` | {h1['mcnemar'][0]} : {h1['mcnemar'][1]} "
          f"(`atv` {sum(r['completed'] for r in by['atv'].values())}, `identified` "
          f"{sum(r['completed'] for r in by['identified'].values())} of {n}) | {p2:.4g} | {hb['H-H2']['level']:.3f} | "
          f"**{hb['H-H2']['rejected']}** | {p2 <= 0.025} |", "",
          "H-H2's exposure is unequal (phase E's m1, carried over): `atv`'s tenth lap is completed only after ten learning "
          "laps without a crash, `identified` has one scored lap. H-H1 is conditional on both completing (E m2).", ""]
    lm = list(by["atv"].values())
    init_fail = [r["case"] for r in lm if not r["started"]]
    learn_fail = [r for r in lm if r["started"] and not r["completed"]]
    md += ["## Failures of `atv` (initialisation apart)", "",
           f"* initialisation failed (both laps; `atv` not started): {len(init_fail)} – {', '.join(init_fail) or '–'}",
           f"* failed during the learning laps: {len(learn_fail)} – "
           + (", ".join(f"{r['case']} (lap {r['first_failed_learning_lap']}, {r['laps'][1 + r['first_failed_learning_lap']]['stop_reason']}"
                        f"{'' if r['laps'][1 + r['first_failed_learning_lap']].get('failure_s') is None else ' at %.0f m' % r['laps'][1 + r['first_failed_learning_lap']]['failure_s']})"
                        for r in learn_fail) or "–"), ""]
    for k in [k for k in LEARN if k in kinds]:
        md += [f"### Lap-time curve, survival and per-lap completion – `{k}`", ""] + _curves(list(by[k].values()))
        md += ["", f"Linearisation error per learning lap with its outcome – `{k}` (review n1)", ""] + _lin_err(list(by[k].values())) + [""]
    first_beat = []
    for c, r in by["atv"].items():
        ri = by["identified"][c]
        if not ri["completed"]:
            continue
        j = next((j for j in range(1, 11) if r["lap_times"][j + 1] is not None and r["lap_times"][j + 1] < ri["lap_time"]), None)
        first_beat.append(j)
    md += ["", f"First learning lap faster than `identified`'s lap (cars `identified` completes): "
           f"{sum(j is not None for j in first_beat)} of {len(first_beat)} cars ever; laps {sorted(j for j in first_beat if j)}", ""]
    md += ["### The cars `atv` completes and those it does not (a description)", "",
           "| group | cars | `identified` lap s median (range) | " + " | ".join(FACTORS) + " (median) |", "|" + "---|" * (3 + len(FACTORS))]
    for lab, grp in (("tenth lap completed", [r for r in lm if r["completed"]]), ("not completed", [r for r in lm if not r["completed"]])):
        idl = [by["identified"][r["case"]]["lap_time"] for r in grp if by["identified"][r["case"]]["completed"]]
        fac = [np.median([r["plant"].get(f, 0.0) for r in grp]) if grp else math.nan for f in FACTORS]
        md.append(f"| {lab} | {len(grp)} | " + (f"{np.median(idl):.2f} ({min(idl):.2f}–{max(idl):.2f})" if idl else "–")
                  + " | " + " | ".join(f"{v:.3f}" for v in fac) + " |")
    sec = {"atv-nmpc": _pair(by["atv"], by["nmpc"]), "atv_nominal-atv": _pair(by["atv_nominal"], by["atv"])}
    imp = [r["lap_time"] - r["first_learning_lap_time"] for r in lm if r["completed"] and np.isfinite(r["first_learning_lap_time"])]
    sec["tenth-first"] = {"n": len(imp), "signed_rank": exact_signed_rank(imp), "mean": float(np.mean(imp)) if imp else math.nan}
    md += ["", "## Secondary comparisons (uncorrected)", "", "| comparison | only first : only second | McNemar p | both complete | "
           "lap time first − second mean / median s | signed-rank p |", "|---|---|---|---|---|---|"]
    for name in ("atv-nmpc", "atv_nominal-atv"):
        s_ = sec[name]
        md.append(f"| {name.replace('-', ' vs ')} | {s_['mcnemar'][0]} : {s_['mcnemar'][1]} | {s_['mcnemar'][2]:.4g} | {s_['both']} | "
                  f"{(np.mean(s_['diffs']) if s_['diffs'] else math.nan):+.3f} / {(np.median(s_['diffs']) if s_['diffs'] else math.nan):+.3f} | "
                  f"{s_['signed_rank']['p']:.4g} |")
    md.append(f"| `atv` tenth − first learning lap | – | – | {len(imp)} | {sec['tenth-first']['mean']:+.3f} / "
              f"{(np.median(imp) if imp else math.nan):+.3f} | {sec['tenth-first']['signed_rank']['p']:.4g} |")
    md += ["", "The tenth − first improvement is measured on the cars that survived all ten learning laps (E n1).", ""]
    md += [f"## Completions and lap times – {test_set}", "", "| controller | completed (95 %) | mean lap s (completed) |", "|---|---|---|"]
    for k in kinds:
        rs = list(by[k].values())
        done = [r["lap_time"] for r in rs if r["completed"]]
        md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {np.mean(done):.3f} |" if done else f"| {k} | {fmt_ci(0, len(rs))} | – |")
    res = {"H-H1": {k: v for k, v in h1.items() if k != "diffs"}, "holm": hb,
           "secondary": {k: {kk: vv for kk, vv in v.items() if kk != "diffs"} for k, v in sec.items()}}
    return md, res


def _nom(recs: list[dict], kinds: tuple[str, ...]) -> list[str]:
    md = ["", "## The nominal car (development car; not part of a test)", ""]
    for k in kinds:
        r = next((r for r in recs if r["kind"] == k and r["set"] == "NOM"), None)
        if r is None:
            continue
        md.append(f"* {k}: " + (" ".join("–" if v is None else f"{v:.2f}" for v in r["lap_times"]) if k in LEARN else _cell(r)))
    return md


def test(d: Path) -> None:
    recs = _last(load(d / "test.jsonl"))
    st = _settings(d)
    _check(recs, st, ("HT", "NOM"), cx_h.PRIMARY_KINDS)
    md, res = _report(recs, st, "HT", cx_h.PRIMARY_KINDS,
                      "# CX-H – the ATV-QP learning MPC against the one-lap identification (docs/CX_H_protocol.md)")
    md += _nom(recs, cx_h.PRIMARY_KINDS)
    md += ["", "What is not claimed (protocol section 3.2): no guarantee; not an equal-information comparison.", "",
           "Second environment: not run – H-H1 and H-H2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_H.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_H.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"written {out / 'CX_H.md'}")


def secondary(d: Path) -> None:
    recs = _last(load(d / "secondary.jsonl"))
    test_recs = _last(load(d / "test.jsonl"))
    st = _settings(d)
    _check(recs, st, cx_h.SECONDARY, cx_h.PRIMARY_KINDS)
    _check([r for r in recs if r["set"] == "HT"], st, ("HT",), ("lmpc",))
    md = ["# CX-H – secondary (protocol sections 4, 6; uncorrected)", "", "## `lmpc` (phase E's NLP) against `atv` on HT – "
          "the formulation ablation", ""]
    atv = {r["case"]: r for r in test_recs if r["kind"] == "atv" and r["set"] == "HT"}
    nlp = {r["case"]: r for r in recs if r["kind"] == "lmpc" and r["set"] == "HT"}
    for lab, j in (("first learning lap", 2), ("tenth learning lap", 11)):
        a_ = {c: r["lap_times"][j] is not None for c, r in atv.items()}
        b_ = {c: r["lap_times"][j] is not None for c, r in nlp.items()}
        x, y, p = mcnemar(a_, b_)
        md.append(f"* {lab}: `atv` {sum(a_.values())}, `lmpc` {sum(b_.values())} of {len(a_)}; only `atv` : only `lmpc` "
                  f"{x} : {y}, exact McNemar p = {p:.4g}")
    md += ["", "### `atv` (HT)", ""] + _curves(list(atv.values())) + ["", "### `lmpc` (HT)", ""] + _curves(list(nlp.values())) + [""]
    for set_ in cx_h.SECONDARY:
        bs = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_} for k in cx_h.PRIMARY_KINDS}
        md += [f"## {set_}", "", "| controller | completed (95 %) | mean lap s |", "|---|---|---|"]
        for k in cx_h.PRIMARY_KINDS:
            rs = list(bs[k].values())
            done = [r["lap_time"] for r in rs if r["completed"]]
            md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {np.mean(done):.3f} |" if done else f"| {k} | {fmt_ci(0, len(rs))} | – |")
        p_ = _pair(bs["atv"], bs["identified"])
        md += ["", f"`atv` : `identified` (only one completes) {p_['mcnemar'][0]} : {p_['mcnemar'][1]}, exact McNemar p = "
               f"{p_['mcnemar'][2]:.4g} (uncorrected). `atv` not started (initialisation failed): "
               f"{sum(not r['started'] for r in bs['atv'].values())}.", ""]
        md += ["### `atv` lap-time curve", ""] + _curves(list(bs["atv"].values())) + [""]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_H_secondary.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {out / 'CX_H_secondary.md'}")


def dev(d: Path) -> None:
    """Section 9: the development records (NOM + DEV, K 20; `nmpc` and `identified` for the tables) and the gate –
    judged on these registered laps only (clarification m1)."""
    dd = d / "dev"
    recs = _last(load(dd / "laps.jsonl"))
    _errors(recs, "dev/laps.jsonl")
    st = {"nmpc": {}, "identified": {}, "atv": cx_h.DEV_SETTING, "atv_nominal": dict(cx_h.DEV_SETTING)}
    _check(recs, st, ("DEV", "NOM"), cx_h.PRIMARY_KINDS)
    md, res = _report(recs, st, "DEV", cx_h.PRIMARY_KINDS, "# CX-H – development checks (section 9; development data, not a test)")
    md += _nom(recs, cx_h.PRIMARY_KINDS)
    nom = next(r for r in recs if r["kind"] == "atv" and r["set"] == "NOM")
    devs = [r for r in recs if r["kind"] == "atv" and r["set"] == "DEV"]
    started = [r for r in devs if r["started"]]
    g = {"nom_all_ten": bool(nom["completed"] and nom["first_failed_learning_lap"] is None),
         "nom_tenth_faster_than_first": bool(nom["completed"] and np.isfinite(nom["first_learning_lap_time"])
                                             and nom["lap_time"] < nom["first_learning_lap_time"]),
         "dev_tenth_completed": f"{sum(r['completed'] for r in started)} of {len(started)} started",
         "dev_at_least_3": sum(r["completed"] for r in started) >= 3}
    g["passed"] = g["nom_all_ten"] and g["nom_tenth_faster_than_first"] and g["dev_at_least_3"]
    md += ["", "## Gate (phase E's, unchanged; these registered laps only – clarification m1)", "", f"{json.dumps(g)}"]
    out = dd / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_H_dev.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "gate.json").write_text(json.dumps({"gate": g, **res}, indent=1, default=float))
    print(NL.join(md))


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test", "secondary", "dev"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "H"))
    a = ap.parse_args()
    {"tune": tune, "test": test, "secondary": secondary, "dev": dev}[a.what](Path(a.dir))


if __name__ == "__main__":
    main()
