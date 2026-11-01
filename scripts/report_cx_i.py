"""Control expansion phase I report (docs/CX_I_protocol.md, revision 0 with its clarifications m1, m2, n1, n2; candidate
I-1; revision 1, the reproduction check). No benchmark report exists: the benchmark was never registered (clarification
m1; the gate failed, and the user closed the LMPC line – section 11).

    python scripts/report_cx_i.py dev    # section 9: stage A (pb on NOM at each eps), stage B (pb, pb_nominal at eps
                                         # 0.04 on NOM + DEV; pb_nominal descriptive, n2), the bound's activity per lap
                                         # (section 5), phase E's and H's records beside them, and the gate
                                         # -> results/CX/I/dev/report/
    python scripts/report_cx_i.py repro  # revision 1, section 10: the reproduction check's arms, per-lap rows, the
                                         # run-out check and the readings -> results/CX/I/repro/report/
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_cx_a import load  # noqa: E402
from report_cx_h import _cell, _curves, _errors, _last  # noqa: E402

from f1race.experiments import cx_i  # noqa: E402

NL = chr(10)


def _fmt(v) -> str:
    return "–" if v is None else f"{v:.2f}"


def _first_lap(r: dict | None) -> str:
    """The first learning lap of a sequence: its time, or where it stopped."""
    if r is None:
        return "–"
    lp = next((l_ for l_ in r["laps"] if l_["lap"] == "learn_1"), None)
    if lp is None:
        return "–"
    if lp["completed"]:
        return f"{lp['lap_time']:.2f} s"
    return f"{lp['stop_reason']} at {lp.get('distance', 0.0):.0f} m"


def _bound_rows(r: dict) -> list[str]:
    """Section 5: per learning lap the bound's activity, the lap over the previous lap, and at a failure the bound and the
    stored laps' speed at the last solve against the car's speed."""
    out = []
    prev = r["lap_times"][1]                               # the second initialisation lap
    for j, lp in enumerate([l_ for l_ in r["laps"] if l_["kind"] == "learn"], start=1):
        if "speed_slack_share_above_1e-3" not in lp:
            break
        ratio = (lp["lap_time"] / prev) if (lp["completed"] and prev) else math.nan
        tail = ""
        if not lp["completed"]:
            s_, vx, vr, vb = lp["last_solve_s_vx_vref_vbar1"]
            tail = f"{lp['stop_reason']} at {lp.get('distance', 0.0):.0f} m; last solve s {s_:.0f} m: v_x {vx:.1f}, v_ref {vr:.1f}, vbar_1 {vb:.1f} m/s"
        out.append(f"| {r['case']} | {j} | {'%.2f' % lp['lap_time'] if lp['completed'] else '–'} | "
                   f"{'%.4f' % ratio if np.isfinite(ratio) else '–'} | {lp['speed_slack_share_above_1e-3']:.3f} | "
                   f"{lp['speed_near_share']:.3f} | {lp['speed_slack_max']:.3g} | {lp.get('term_slack_share_above_1e-3', math.nan):.3f} | "
                   f"{lp.get('regression_fallback_share', math.nan):.2f} | {lp.get('fallbacks', 0)}/{lp.get('solves', 0)} | "
                   f"{lp['solve_ms_mean_p95_max'][0]:.0f} | {tail} |")
        prev = lp["lap_time"] if lp["completed"] else None
    return out


def _other(path: Path, kind: str, key: str) -> dict:
    if not path.exists():
        return {}
    return {r["case"]: r for r in _last(load(path)) if r.get("kind") == kind and r.get("setting_key") == key and "error" not in r}


def dev(d: Path) -> None:
    dd = d / "dev"
    recs = _last(load(dd / "laps.jsonl"))
    _errors(recs, "dev/laps.jsonl")
    dev_key = cx_i.setting_key(cx_i.DEV_SETTING)
    nom_dev = [c["name"] for s_ in ("NOM", "DEV") for c in cx_i.cases(s_)]
    by = lambda kind, key=None: {r["case"]: r for r in recs if r["kind"] == kind and (key is None or r["setting_key"] == key)}  # noqa: E731
    need = {("pb", dev_key): nom_dev, ("pb_nominal", dev_key): nom_dev, ("nmpc", "default"): nom_dev,
            ("identified", "default"): nom_dev}
    need.update({("pb", cx_i.setting_key(s_)): ["NOM_car0"] for s_ in cx_i.EPS_GRID})
    for (k, key), names in need.items():
        missing = set(names) - set(by(k, key))
        if missing:
            raise SystemExit(f"{k} {key}: missing {sorted(missing)}")
    e_lmpc = _other(ROOT / "results" / "CX" / "E" / "dev" / "laps.jsonl", "lmpc", "K=20")
    h_atv = _other(ROOT / "results" / "CX" / "H" / "dev" / "laps.jsonl", "atv", "K=20")
    pb, pbn, nm, idf = by("pb", dev_key), by("pb_nominal", dev_key), by("nmpc"), by("identified")

    md = ["# CX-I – development checks of I-1, the speed bound per lap (section 9; development data, not a test)", "",
          f"Records: `results/CX/I/dev/laps.jsonl` ({len(recs)} jobs). K = 20 throughout.", "",
          "## Stage A – `pb` on NOM, the first learning lap at each eps (review i3's precondition)", "",
          "| eps | init 0.73 | init 0.79 | " + " | ".join(f"L{j}" for j in range(1, 11)) + " |", "|" + "---|" * 13]
    stage_a = {}
    for s_ in cx_i.EPS_GRID:
        r = by("pb", cx_i.setting_key(s_))["NOM_car0"]
        stage_a[s_["eps"]] = bool(r["lap_times"][2] is not None)
        md.append(f"| {s_['eps']} | " + " | ".join(_fmt(v) for v in r["lap_times"]) + " |")
    md += ["", "Expected under clarification m2 (NOM, T₀.₇₉ = 83.16 s): the first learning lap at about T₀.₇₉ / (1 + eps) "
           "– 81.5, 80.0, 77.0 s.", ""]

    md += ["## Stage B – eps 0.04 on NOM and DEV: the first learning lap and the sequence", "",
           "| car | `pb` – lap 1 | `pb` – sequence | `pb_nominal` – lap 1 (descriptive, n2) | `pb_nominal` – sequence | "
           "phase E's `lmpc` K 20 – lap 1 | phase H's `atv` K 20 – lap 1 | `nmpc` | `identified` |", "|" + "---|" * 9]
    for c in nom_dev:
        md.append(f"| {c} | {_first_lap(pb[c])} | {_cell(pb[c])} | {_first_lap(pbn[c])} | {_cell(pbn[c])} | "
                  f"{_first_lap(e_lmpc.get(c))} | {_first_lap(h_atv.get(c))} | {_cell(nm[c])} | {_cell(idf[c])} |")
    for k, rs in (("pb", pb), ("pb_nominal", pbn)):
        md += ["", f"### Lap-time curve, survival and per-lap completion – `{k}` (NOM + DEV)", ""] + _curves(list(rs.values()))
    md += ["", "## The bound per learning lap (section 5)", "",
           "| car | lap | time s | over the previous lap | share sigma > 1e-3 | share v_x within 1 % of vbar | max sigma | "
           "share terminal slack > 1e-3 | regression fallback share | solver failures / solves | solve ms mean | at a failure |",
           "|" + "---|" * 12]
    for k, rs in (("pb", pb), ("pb_nominal", pbn)):
        md += [f"| **{k}** |" + " |" * 11]
        for c in nom_dev:
            md += _bound_rows(rs[c])
    for e in (0.02, 0.08):
        md += [f"| **pb eps {e} (NOM)** |" + " |" * 11] + _bound_rows(by("pb", cx_i.setting_key({"K": 20, "eps": e}))["NOM_car0"])

    nom = pb["NOM_car0"]
    devs = [pb[c] for c in nom_dev if c != "NOM_car0"]
    started = [r for r in devs if r["started"]]
    g = {"A_nom_first_lap_each_eps": stage_a, "A_passed": all(stage_a.values()),
         "B_nom_all_ten": bool(nom["completed"] and nom["first_failed_learning_lap"] is None),
         "B_nom_tenth_faster_than_first": bool(nom["completed"] and np.isfinite(nom["first_learning_lap_time"])
                                               and nom["lap_time"] < nom["first_learning_lap_time"]),
         "B_dev_tenth_completed": f"{sum(r['completed'] for r in started)} of {len(started)} started",
         "B_dev_at_least_3": sum(r["completed"] for r in started) >= 3}
    g["B_passed"] = g["B_nom_all_ten"] and g["B_nom_tenth_faster_than_first"] and g["B_dev_at_least_3"]
    g["passed"] = g["A_passed"] and g["B_passed"]
    md += ["", "## Gate of I-1 (section 9.4; these registered laps only, `pb` only – n2)", "", f"{json.dumps(g)}"]
    out = dd / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_I_dev.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "gate.json").write_text(json.dumps({"gate": g}, indent=1, default=float))
    print(NL.join(md))


def _g(v, f="{:.2f}") -> str:
    return "–" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f.format(v)


def repro(d: Path, sub: str = "repro") -> None:
    """Revision 1, section 10: per arm the lap-time curve, the per-lap rows of section 10.4, the run-out check (m3), the
    failure (n2) and the reading (section 10.5 with m1) -> results/CX/I/<sub>/report/."""
    dd = d / sub
    recs = {r["job"].split("|")[1]: r for r in _last(load(dd / "laps.jsonl")) if "error" not in r}
    errs = [r for r in load(dd / "laps.jsonl") if "error" in r]
    nmpc = 76.40
    md = ["# CX-I – the reproduction check under the published LMPCs' conditions (revision 1, section 10; NOM, diagnostic)",
          "", f"Records: `results/CX/I/{sub}/laps.jsonl` ({len(recs)} arms; {len(errs)} error records). "
          f"`nmpc` on NOM: {nmpc} s; near the limit ≤ {cx_i.NEAR_LIMIT} s, approaching ≤ {cx_i.APPROACH} s; converged: the "
          f"last five completed learning laps within {cx_i.CONVERGED:.1%}.", "",
          "## Readings", "", "| arm | completed learning laps | best | near the limit | converged | last five | reading | "
          "wall h |", "|" + "---|" * 8]
    readings = {}
    for arm in sorted(recs):
        r = recs[arm]
        g = cx_i.reading(r)
        readings[arm] = g
        md.append(f"| {arm} | {g['completed']} of {g['learning_laps']} | {_g(g['best'])} | {g['near']} | {g['converged']} | "
                  + ", ".join(f"{t:.2f}" for t in g["last_five"]) + f" | **{g['reading']}** | {r['wall_s'] / 3600:.1f} |")
    for arm in sorted(recs):
        r = recs[arm]
        md += ["", f"## {arm} – per lap (section 10.4)", "",
               "| lap | time s | over the previous | start s m | term. slack > 1e-3 | term. slack max | fallbacks / solves | "
               "solve ms mean | tyre usage max f / r | min edge margin m | run-out check: dt s, d v_x m/s | stop | wall min |",
               "|" + "---|" * 13]
        prev = None
        for lp in r["laps"]:
            if "solves" not in lp and not lp.get("completed"):
                md.append(f"| {lp['lap']} | – | – | – | – | – | – | – | – | – | – | {lp.get('stop_reason')} | – |")
                continue
            t = lp["lap_time"] if lp["completed"] else None
            ratio = (t / prev) if (t is not None and prev) else None
            rc = lp.get("runout_check")
            us = lp.get("tyre_usage_max_front_rear") or [math.nan, math.nan]
            stop = "lap completed" if lp["completed"] and lp.get("run_out_completed") else (
                f"{lp.get('stop_reason')} / run-out {lp.get('run_out_stop')} at {lp.get('distance', 0):.0f} m; "
                f"{lp.get('failure_class', '')} ({lp.get('fallbacks_last20', '–')} of last 20 failed); v_x "
                f"{_g(lp.get('failure_vx'), '{:.1f}')}, stored {_g(lp.get('failure_v_stored'), '{:.1f}')}")
            md.append(f"| {lp['lap']} | {_g(t)} | {_g(ratio, '{:.4f}')} | {_g(lp.get('start_s'), '{:.2f}')} | "
                      f"{_g(lp.get('term_slack_share_above_1e-3'), '{:.3f}')} | {_g(lp.get('term_slack_max'), '{:.3g}')} | "
                      f"{lp.get('fallbacks', '–')} / {lp.get('solves', '–')} | "
                      f"{_g((lp.get('solve_ms_mean_p95_max') or [None])[0], '{:.0f}')} | {us[0]:.3f} / {us[1]:.3f} | "
                      f"{_g(lp.get('min_edge_margin'), '{:.2f}')} | "
                      + (f"{rc['dt']:+.3f}, {rc['dstate_s_n_xi_vx_vy_r_delta'][3]:+.2f}" if rc else "–")
                      + f" | {stop} | {_g((lp.get('wall_lap_s') or math.nan) / 60, '{:.0f}')} |")
            prev = t if t is not None else prev
    out = dd / "report"
    out.mkdir(parents=True, exist_ok=True)
    (out / "CX_I_repro.md").write_text(NL.join(md) + NL, encoding="utf-8")
    (out / "readings.json").write_text(json.dumps(readings, indent=1, default=float))
    print(NL.join(md))


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["dev", "repro"])
    ap.add_argument("--dir", default=str(ROOT / "results" / "CX" / "I"))
    ap.add_argument("--sub", default="repro")
    a = ap.parse_args()
    if a.what == "repro":
        repro(Path(a.dir), a.sub)
        return
    dev(Path(a.dir))


if __name__ == "__main__":
    main()
