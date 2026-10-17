"""L8: compute the plans of a track (``f1race.experiments.l8.compute_plans``).

    python scripts/l8_plans.py Monza            # -> results/L8/plans/Monza.pkl
"""
from __future__ import annotations

import json
import os
import sys

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.experiments.l8 import compute_plans  # noqa: E402

if __name__ == "__main__":
    for name in sys.argv[1:]:
        d = compute_plans(name)
        print(json.dumps({"track": name, "minutes": round(d["minutes"], 1),
                          "lap_times": {str(k): round(v.lap_time, 3) for k, v in sorted(d["plans"].items())},
                          "log": d["log"]}, indent=1, default=str), flush=True)
