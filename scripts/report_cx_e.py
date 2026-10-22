"""Control expansion phase E report (docs/CX_E_protocol.md, revision 0 with its clarifications).

    python scripts/report_cx_e.py tune        # CEV: the rule of section 4 -> results/CX/E/selected.json + report/tune.md
    python scripts/report_cx_e.py test        # CET: H-E1 (lap time, tenth learning lap vs identified, both complete) and
                                              # H-E2 (completions), Holm and the stricter decision; the 5 secondary tests;
                                              # the lap-time curves, the survival curve, per-lap completion, the cars
                                              # completed vs not (m2), initialisation failures apart (m4); NOM apart
                                              # -> report/CX_E.md, report/CX_E.json
    python scripts/report_cx_e.py secondary   # CET_LAG30, CET_WET85: completions, lmpc - identified (2, uncorrected)
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

from f1race.experiments import cx_e  # noqa: E402

NL = chr(10)
FACTORS = ("mu", "m", "ClA", "CdA", "Iz", "B", "aero_front")


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
    sel = cx_e.select(recs, "lmpc")
    out = {"tune_sha256": _sha(path), "kinds": {"lmpc": {"setting": sel["setting"], "setting_key": sel["setting_key"]}}}
    md = ["# CX-E tuning on CEV (protocol section 4)", "", f"## lmpc – chosen: `{sel['setting_key']}`", "",
          "| setting | tenth learning laps completed | mean tenth lap on the cars all tied settings complete | started | "
          "first failed learning lap (cars) | lap-time curve, median over the completed cars |", "|---|---|---|---|---|---|"]
    for k, t in sorted(sel["table"].items(), key=lambda kv: (-kv[1]["completed"], kv[0])):
        rs = [r for r in recs if r["kind"] == "lmpc" and r["setting_key"] == k]
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
    k = sel["kinds"]["lmpc"]["setting"]
    return {"nmpc": {}, "identified": {}, "lmpc": k, "lmpc_nominal": dict(k)}


def _check(recs: list[dict], st: dict, sets: tuple[str, ...]) -> None:
    _errors(recs, "the records")
    need = {c["name"] for s_ in sets for c in cx_e.cases(s_)}
    for k in cx_e.KINDS:
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


def test(d: Path) -> None:
    recs = load(d / "test.jsonl")
    st = _settings(d)
    _check(recs, st, ("CET", "NOM"))
    by = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == "CET"} for k in cx_e.KINDS}
    n = len(by["lmpc"])
    h1 = _pair(by["lmpc"], by["identified"])
    p1 = h1["signed_rank"]["p"]
    p2 = h1["mcnemar"][2]
    hb = holm({"H-E1": p1, "H-E2": p2})
    dd = h1["diffs"]
    md = ["# CX-E – the learning MPC against the one-lap identification (docs/CX_E_protocol.md)", "",
          "Error records: none; every controller has the CET cars and the nominal car with the settings chosen on CEV: "
          + "; ".join(f"{k} `{cx_e.setting_key(v)}`" for k, v in st.items()) + ".", "",
          "## Primary tests (the tenth learning lap of `lmpc` against `identified`)", "",
          "| test | comparison | result | p | Holm level | holds (Holm) | holds (each at 0.025) |", "|---|---|---|---|---|---|---|",
          f"| H-E1 | lap time `lmpc` − `identified` on the {h1['both']} cars both complete | "
          + (f"mean {np.mean(dd):+.3f} s, median {np.median(dd):+.3f} s, W+ {h1['signed_rank']['W_plus']:.1f} (n {h1['signed_rank']['n']})"
             if dd else "no car completed by both – not testable")
          + f" | {p1:.4g} | {hb['H-E1']['level']:.3f} | **{hb['H-E1']['rejected']}** | {p1 <= 0.025} |",
          f"| H-E2 | completed laps, only `lmpc` : only `identified` | {h1['mcnemar'][0]} : {h1['mcnemar'][1]} "
          f"(`lmpc` {sum(r['completed'] for r in by['lmpc'].values())}, `identified` "
          f"{sum(r['completed'] for r in by['identified'].values())} of {n}) | {p2:.4g} | {hb['H-E2']['level']:.3f} | "
          f"**{hb['H-E2']['rejected']}** | {p2 <= 0.025} |", "",
          "H-E2's exposure is unequal (clarification m1): `lmpc`'s tenth lap is completed only after ten learning laps without a "
          "crash, `identified` has one scored lap. H-E1 is conditional on both completing (m2).", ""]
    lm = list(by["lmpc"].values())
    init_fail = [r["case"] for r in lm if not r["started"]]
    learn_fail = [r for r in lm if r["started"] and not r["completed"]]
    md += ["## Failures of `lmpc` (clarification m4)", "",
           f"* initialisation failed (both laps; `lmpc` not started): {len(init_fail)} – {', '.join(init_fail) or '–'}",
           f"* failed during the learning laps: {len(learn_fail)} – "
           + (", ".join(f"{r['case']} (lap {r['first_failed_learning_lap']}, {r['laps'][1 + r['first_failed_learning_lap']]['stop_reason']}"
                        f"{'' if r['laps'][1 + r['first_failed_learning_lap']].get('failure_s') is None else ' at %.0f m' % r['laps'][1 + r['first_failed_learning_lap']]['failure_s']})"
                        for r in learn_fail) or "–"), "",
           "### Lap-time curve, survival and per-lap completion – `lmpc` (CET)", ""] + _curves(lm)
    md += ["", "### Lap-time curve – `lmpc_nominal` (CET)", ""] + _curves(list(by["lmpc_nominal"].values()))
    first_beat = []
    for c, r in by["lmpc"].items():
        ri = by["identified"][c]
        if not ri["completed"]:
            continue
        j = next((j for j in range(1, 11) if r["lap_times"][j + 1] is not None and r["lap_times"][j + 1] < ri["lap_time"]), None)
        first_beat.append(j)
    md += ["", f"First learning lap faster than `identified`'s lap (cars `identified` completes): "
           f"{sum(j is not None for j in first_beat)} of {len(first_beat)} cars ever; laps {sorted(j for j in first_beat if j)}", ""]
    md += ["### The cars `lmpc` completes and those it does not (clarification m2; a description)", "",
           "| group | cars | `identified` lap s median (range) | " + " | ".join(FACTORS) + " (median) |", "|" + "---|" * (3 + len(FACTORS))]
    for lab, grp in (("tenth lap completed", [r for r in lm if r["completed"]]), ("not completed", [r for r in lm if not r["completed"]])):
        idl = [by["identified"][r["case"]]["lap_time"] for r in grp if by["identified"][r["case"]]["completed"]]
        fac = [np.median([r["plant"].get(f, 0.0) for r in grp]) if grp else math.nan for f in FACTORS]
        md.append(f"| {lab} | {len(grp)} | " + (f"{np.median(idl):.2f} ({min(idl):.2f}–{max(idl):.2f})" if idl else "–")
                  + " | " + " | ".join(f"{v:.3f}" for v in fac) + " |")
    sec = {"lmpc-nmpc": _pair(by["lmpc"], by["nmpc"]), "lmpc_nominal-lmpc": _pair(by["lmpc_nominal"], by["lmpc"])}
    imp = [r["lap_time"] - r["first_learning_lap_time"] for r in lm if r["completed"] and np.isfinite(r["first_learning_lap_time"])]
    sec["tenth-first"] = {"n": len(imp), "signed_rank": exact_signed_rank(imp), "mean": float(np.mean(imp)) if imp else math.nan}
    md += ["", "## Secondary comparisons (uncorrected; 5)", "", "| comparison | only first : only second | McNemar p | both complete | "
           "lap time first − second mean / median s | signed-rank p |", "|---|---|---|---|---|---|"]
    for name in ("lmpc-nmpc", "lmpc_nominal-lmpc"):
        s_ = sec[name]
        md.append(f"| {name.replace('-', ' vs ')} | {s_['mcnemar'][0]} : {s_['mcnemar'][1]} | {s_['mcnemar'][2]:.4g} | {s_['both']} | "
                  f"{(np.mean(s_['diffs']) if s_['diffs'] else math.nan):+.3f} / {(np.median(s_['diffs']) if s_['diffs'] else math.nan):+.3f} | "
                  f"{s_['signed_rank']['p']:.4g} |")
    md.append(f"| `lmpc` tenth − first learning lap | – | – | {len(imp)} | {sec['tenth-first']['mean']:+.3f} / "
              f"{(np.median(imp) if imp else math.nan):+.3f} | {sec['tenth-first']['signed_rank']['p']:.4g} |")
    md += ["", "The tenth − first improvement is measured on the cars that survived all ten learning laps (clarification n1).", ""]
    md += ["## Completions and lap times – CET", "", "| controller | completed (95 %) | mean lap s (completed) |", "|---|---|---|"]
    for k in cx_e.KINDS:
        rs = list(by[k].values())
        done = [r["lap_time"] for r in rs if r["completed"]]
        md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {np.mean(done):.3f} |" if done else f"| {k} | {fmt_ci(0, len(rs))} | – |")
    nom = {k: next(r for r in recs if r["kind"] == k and r["set"] == "NOM") for k in cx_e.KINDS}
    md += ["", "## The nominal car (development car; not part of a test)", ""]
    for k in cx_e.KINDS:
        r = nom[k]
        md.append(f"* {k}: " + (" ".join("–" if v is None else f"{v:.2f}" for v in r["lap_times"]) if k in cx_e.LEARN_KINDS
                                 else _cell(r)))
    md += ["", "What is not claimed (protocol section 3.4): no guarantee (the local convex hull, the learned correction and the soft "
           "terminal condition give up E.1's); not an equal-information comparison.", "",
           "Second environment: not run – H-E1 and H-E2 are not cross-checked (protocol section 7)."]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_E.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "CX_E.json").write_text(json.dumps({"H-E1": {k: v for k, v in h1.items() if k != "diffs"}, "holm": hb,
                                               "secondary": {k: {kk: vv for kk, vv in v.items() if kk != "diffs"}
                                                             for k, v in sec.items()}}, indent=1, default=float))
    print(f"written {out / 'CX_E.md'}")


def secondary(d: Path) -> None:
    recs = load(d / "secondary.jsonl")
    st = _settings(d)
    _check(recs, st, cx_e.SECONDARY)
    md = ["# CX-E – secondary conditions (protocol section 2; uncorrected, 2 McNemar)", ""]
    for set_ in cx_e.SECONDARY:
        bs = {k: {r["case"]: r for r in recs if r["kind"] == k and r["set"] == set_} for k in cx_e.KINDS}
        md += [f"## {set_}", "", "| controller | completed (95 %) | mean lap s |", "|---|---|---|"]
        for k in cx_e.KINDS:
            rs = list(bs[k].values())
            done = [r["lap_time"] for r in rs if r["completed"]]
            md.append(f"| {k} | {fmt_ci(len(done), len(rs))} | {np.mean(done):.3f} |" if done else f"| {k} | {fmt_ci(0, len(rs))} | – |")
        p_ = _pair(bs["lmpc"], bs["identified"])
        md += ["", f"`lmpc` : `identified` (only one completes) {p_['mcnemar'][0]} : {p_['mcnemar'][1]}, exact McNemar p = "
               f"{p_['mcnemar'][2]:.4g} (uncorrected). `lmpc` not started (initialisation failed): "
               f"{sum(not r['started'] for r in bs['lmpc'].values())}.", ""]
        md += ["### `lmpc` lap-time curve", ""] + _curves(list(bs["lmpc"].values())) + [""]
    out = d / "report"
    out.mkdir(exist_ok=True)
    (out / "CX_E_secondary.md").write_text(NL.join(md) + NL, encoding="utf-8")
    print(f"written {out / 'CX_E_secondary.md'}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test", "secondary"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "E"))
    a = ap.parse_args()
    {"tune": tune, "test": test, "secondary": secondary}[a.what](Path(a.dir))


if __name__ == "__main__":
    main()
