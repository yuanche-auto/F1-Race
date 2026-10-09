"""Figures of the L5 multi-seed redo (docs/figures/L5/multiseed.png).

    python scripts/l5_figures.py     # after scripts/l5_multiseed.py and scripts/l5_baselines.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "results" / "L5" / "multiseed"
GROUPS = {"A": ("20 s, ±0.02 rad", "tab:blue"), "B": ("40 s, ±0.02 rad", "tab:orange"),
          "C": ("40 s, speed-dependent bound", "tab:green")}


def main() -> None:
    val = json.loads((D / "validation.json").read_text())
    test = json.loads((D / "test.json").read_text())
    base = json.loads((ROOT / "results" / "L5" / "baselines.json").read_text())
    ks = list(range(50, 601, 50))
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.8))
    for g, (label, col) in GROUPS.items():
        for s, ls in zip((1, 2, 3), ("-", "--", ":")):
            n = [sum(r["completed"] for r in val if r["checkpoint"] == f"{g}_s{s}_{k}k") for k in ks]
            axs[0].plot(ks, n, ls, color=col, label=f"{g} ({label}), seed {s}" if s == 1 else f"{g}, seed {s}")
    axs[0].set_xlabel("training decisions [thousands]"); axs[0].set_ylabel("validation laps completed (of 20)")
    axs[0].grid(alpha=0.3); axs[0].legend(fontsize=7, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.14))
    x = 0
    ticks = []
    for g, (label, col) in GROUPS.items():
        for s in (1, 2, 3):
            stem = f"{g}_s{s}_600k"
            l2 = sum(r["completed"] for r in test if r["checkpoint"] == stem and r["case"][0] == "test_car")
            l3 = sum(r["completed"] for r in test if r["checkpoint"] == stem and r["case"][0] == "test_dist")
            axs[1].bar(x - 0.2, 100 * l2 / 51, 0.4, color=col)
            axs[1].bar(x + 0.2, 100 * l3 / 30, 0.4, color=col, alpha=0.5)
            ticks.append((x, f"{g}{s}"))
            x += 1
        x += 0.5
    refs = {"NMPC, nominal model (L2)": (22 / 51, "k"), "NMPC, η 0.82 + 1.3 m (L2)": (sum(
        r["completed"] for r in base if r["controller"] == "nmpc_0.82_m1.3" and r["set"] == "L2") / 51, "tab:red"),
        "tracker × 0.85 (L2)": (16 / 51, "tab:gray")}
    for name, (v, c) in refs.items():
        axs[1].axhline(100 * v, color=c, ls="--", lw=1, label=name)
    axs[1].set_xticks([t for t, _ in ticks]); axs[1].set_xticklabels([n for _, n in ticks])
    axs[1].set_ylabel("test laps completed [%]  (dark: 51 L2 cars, light: 30 L3 laps)")
    axs[1].set_title("final checkpoint of every training run", fontsize=10)
    axs[1].grid(alpha=0.3, axis="y"); axs[1].legend(fontsize=7, loc="lower left")
    out = ROOT / "docs" / "figures" / "L5"
    out.mkdir(parents=True, exist_ok=True)
    fig.savefig(out / "multiseed.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
