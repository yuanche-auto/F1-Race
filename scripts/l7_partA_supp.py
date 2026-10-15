"""L7 Part A, two supplementary measures (part (i) of amendment A2 only, and (ii) for the record):

* A4b: the critic's Q itself in the last 3 s before a failure against Q at the same 100 m of
  track on the laps of the same checkpoint that complete – does the critic tell the failing
  state from a passing one at the same place?
* A2b: the parameters of the cars that fail against those that complete (L2 test cars only).

    .venv-rl/Scripts/python scripts/l7_partA_supp.py      # -> results/L7/partA/partA_supp.md / .json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from l7_partA import BEFORE, FINALS, OUT, PARTS, SELECTED, WINDOW, part_of  # noqa: E402


def main() -> None:
    from f1race.dynamics import DynVehicle
    from f1race.experiments.l2 import sample_plants
    from f1race.track import Track

    L = Track.load("Spielberg").length
    cars = sample_plants(DynVehicle(), 50, seed=0)
    res, md = {}, ["# L7 Part A – supplementary measures", "",
                   "A4b: median Q in the last 3 s before each failure, and median Q of the completed laps of the "
                   "same checkpoint and part on the same 100 m before the failure position; the difference "
                   "fail − window. A2b: median friction factor μ and yaw-inertia factor of the failing and the "
                   "completing L2 test cars (part (i): cars 25–50; part (ii): cars 0–24).", "",
                   "| checkpoint | part | failures | Q fail | Q window | fail − window | μ failing / completing | "
                   "Iz failing / completing |", "|---|---|---|---|---|---|---|---|"]
    for name in FINALS + SELECTED:
        laps = json.loads((OUT / f"laps_{name}.json").read_text())
        z = np.load(OUT / f"laps_{name}.npz")
        lid, q, pos = z["lap_id"], z["q"], z["pos"]
        for part in PARTS:
            ids = [i for i, r in enumerate(laps) if part_of(r["case"]) == part]
            fq, wq = [], []
            for i in ids:
                if laps[i]["completed"]:
                    continue
                m = np.flatnonzero(lid == i)
                fq.append(q[m[-BEFORE:]])
                pf = laps[i]["distance"] % L
                for j in ids:
                    if laps[j]["completed"]:
                        mm = np.flatnonzero(lid == j)
                        wq.append(q[mm[((pf - pos[mm]) % L) <= WINDOW]])
            carl = [laps[i] for i in ids if laps[i]["case"][0] == "test_car"]
            fac = lambda rs, k: float(np.median([cars[r["case"][1]][0].get(k, 1.0) for r in rs])) if rs else float("nan")  # noqa: E731
            fail_c = [r for r in carl if not r["completed"]]
            done_c = [r for r in carl if r["completed"]]
            row = {"failures": len(fq), "mu": [fac(fail_c, "mu"), fac(done_c, "mu")],
                   "Iz": [fac(fail_c, "Iz"), fac(done_c, "Iz")]}
            if fq:
                a, b = float(np.median(np.concatenate(fq))), float(np.median(np.concatenate(wq))) if wq else float("nan")
                row.update({"q_fail": a, "q_window": b})
                qs = f"{a:.1f} | {b:.1f} | {a - b:+.1f}"
            else:
                qs = "– | – | –"
            res[f"{name} ({part})"] = row
            md.append(f"| {name} | ({part}) | {len(fq)} | {qs} | {row['mu'][0]:.3f} / {row['mu'][1]:.3f} | "
                      f"{row['Iz'][0]:.2f} / {row['Iz'][1]:.2f} |")
    (OUT / "partA_supp.json").write_text(json.dumps(res, indent=1))
    (OUT / "partA_supp.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"written {OUT / 'partA_supp.md'}")


if __name__ == "__main__":
    main()
