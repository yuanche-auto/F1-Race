"""Control expansion phase I – development checks of candidate I-1 (docs/CX_I_protocol.md section 9; NOM and DEV only).
Windows `.venv`; records in results/CX/I/dev/ with the commit and the dirty flag (``meta_<what>.json``).

    python scripts/cx_i_dev.py nlp-repro     # 9.1: phase E's NLP unchanged (the bound off) – lmpc K 20 on NOM against
                                             # phase E's record
    python -m pytest tests/test_cx_i.py      # 9.2: the bound (unit tests)
    python scripts/run_cx_i.py dev           # 9.3: stages A and B, nmpc and identified on NOM + DEV
    python scripts/report_cx_i.py dev        # 9.4: the tables and the gate
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
DEV = ROOT / "results" / "CX" / "I" / "dev"
E_DEV = ROOT / "results" / "CX" / "E" / "dev" / "laps.jsonl"


def meta(what: str) -> None:
    DEV.mkdir(parents=True, exist_ok=True)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "scripts", "tests"], cwd=ROOT,
                                capture_output=True, text=True).stdout.strip())
    (DEV / f"meta_{what}.json").write_text(json.dumps({"commit": commit, "dirty": dirty,
                                                       "time": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def nlp_repro() -> None:
    from f1race.experiments import cx_e

    meta("nlp_repro")
    ref = next(json.loads(l) for l in E_DEV.read_text().splitlines()
               if l.strip() and json.loads(l).get("job") == "dev|lmpc|K=20|NOM_car0")
    t0 = time.time()
    r = cx_e.learn(cx_e.cases("NOM")[0], "lmpc", {"K": 20})
    pick = lambda rec: [(l["lap"], l["completed"], l.get("distance"), l.get("stop_reason")) for l in rec["laps"][:4]]  # noqa: E731
    out = {"wall_s": time.time() - t0,
           "phase_e": {"lap_times": ref["lap_times"], "first_failed": ref["first_failed_learning_lap"], "laps": pick(ref)},
           "now": {"lap_times": r["lap_times"], "first_failed": r["first_failed_learning_lap"], "laps": pick(r)}}
    out["equal"] = out["phase_e"]["lap_times"] == out["now"]["lap_times"] and out["phase_e"]["laps"] == out["now"]["laps"]
    (DEV / "nlp_repro.json").write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps(out, indent=1, default=float))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["nlp-repro"])
    ap.parse_args()
    nlp_repro()


if __name__ == "__main__":
    main()
