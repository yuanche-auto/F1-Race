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


def test(d: Path) -> None:
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
    md = ["# CX-J – robust adaptive MPC", "", f"H-J1 (JD, `ra` against `adaptive`): {h1['only_first']} : {h1['only_second']}, p {h1['p']:.4g}; "
          f"completed {h1['completed'][0]} / {h1['completed'][1]}; cars improved / worsened / unchanged "
          f"{h1['improved']} / {h1['worsened']} / {h1['unchanged']}, improved share {h1['improved_share_interval']}",
          f"H-J2 (JN + JL, `ra` against `tube`): per-car signed-rank p {h2['p']:.4g} (n {h2['n_nonzero']}), laps {h2['laps_completed']}; "
          f"cars improved / worsened / unchanged {h2['improved']} / {h2['worsened']} / {h2['unchanged']}, improved share "
          f"{h2['improved_share_interval']}", f"Holm: {hm}", "",
          "| set | " + " | ".join(KINDS) + " |", "|---|" + "---|" * len(KINDS)]
    for c in "CDNL":
        md.append(f"| {SETS[c]} | " + " | ".join(fmt_ci(out["completed"][SETS[c]][k], 50) for k in KINDS) + " |")
    md += ["", "Secondary (uncorrected):", "```", json.dumps(sec, indent=1, default=float), "```"]
    (d / "report" / "CX_J.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md[:4] + md[4:14]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["tune", "test"])
    a = ap.parse_args()
    {"tune": tune, "test": test}[a.what](cx_j.OUT)


if __name__ == "__main__":
    main()
