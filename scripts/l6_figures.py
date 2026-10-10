"""Figures of L6 (docs/figures/L6/): the lag sweep on the nominal car, completion on the
25 cars, and the parameters the identification without the lag finds.

    python scripts/l6_figures.py [--results DIR]     # after scripts/run_l6.py and scripts/l6_rl.py
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
FIG = ROOT / "docs" / "figures" / "L6"
LAP = 4315.0

STYLE = {"tracker": ("L1 tracker", "tab:gray", "x"), "nmpc": ("NMPC, nominal", "tab:blue", "o"),
         "explore": ("NMPC, 12 % reserve", "tab:cyan", "s"), "smpc": ("SMPC (L3)", "tab:purple", "D"),
         "identified": ("NMPC, identified (L4)", "tab:orange", "^"),
         "lag_nominal": ("NMPC, nominal param. + lag model", "tab:green", "v"),
         "lag_identified": ("NMPC + lag, τ identified", "tab:olive", "P"),
         "lag_half": ("NMPC + lag model 0.5 τ", "yellowgreen", "<"),
         "lag_double": ("NMPC + lag model 2 τ", "darkgreen", ">"),
         "oracle_param": ("NMPC, true parameters", "tab:brown", "h"),
         "oracle_struct": ("NMPC, true parameters + lag", "black", "*"),
         "rl": ("residual RL (L5)", "tab:red", "X")}


def load(d: Path, name: str) -> list[dict]:
    p = d / name
    return [json.loads(l) for l in open(p, encoding="utf-8") if l.strip()] if p.exists() else []


def sweep(res: list[dict]) -> None:
    nom = [r for r in res if r["car"] == 0 and r["observe"] == "command"]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for c, (label, col, mk) in STYLE.items():
        rs = sorted((r for r in nom if r["controller"] == c), key=lambda r: r["tau"])
        if len(rs) < 2:
            continue
        t = [1000 * r["tau"] for r in rs]
        d = [100 * min(r["distance"], LAP) / LAP for r in rs]
        ax.plot(t, d, "-", marker=mk, color=col, label=label, alpha=0.85)
    ax.set_xscale("log"); ax.set_xlabel("steering lag τ [ms]"); ax.set_ylabel("lap distance covered [%]")
    ax.set_ylim(0, 105); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="lower left")
    ax.set_title("Nominal car: how far each controller gets with a steering lag")
    fig.tight_layout(); fig.savefig(FIG / "sweep.png", dpi=130); plt.close(fig)


def completion(res: list[dict]) -> None:
    taus = sorted({r["tau"] for r in res if r["car"] == 24 and r["observe"] == "command"})
    names = [c for c in STYLE if any(r["controller"] == c and r["car"] == 24 for r in res)]
    if not taus or not names:
        return
    fig, ax = plt.subplots(figsize=(10, 4.5))
    w = 0.8 / len(taus)
    for i, t in enumerate(taus):
        n = [sum(r["completed"] for r in res if r["tau"] == t and r["controller"] == c and r["observe"] == "command")
             for c in names]
        ax.bar(np.arange(len(names)) + (i - (len(taus) - 1) / 2) * w, n, w, label=f"τ = {1000 * t:.0f} ms")
    ax.set_xticks(np.arange(len(names)), [STYLE[c][0] for c in names], rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("laps completed (of 25 cars)"); ax.set_ylim(0, 26); ax.grid(axis="y", alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(FIG / "completion.png", dpi=130); plt.close(fig)


def identification(res: list[dict]) -> None:
    from f1race.estimation.identify import HI, LO, PARAMS

    ex = sorted((r for r in res if r["car"] == 0 and r["controller"] == "explore" and r["observe"] == "command"
                 and "identification" in r), key=lambda r: r["tau"])
    if not ex:
        return
    t = [1000 * r["tau"] for r in ex]
    th = np.array([r["identification"]["theta"] for r in ex])
    fig, axs = plt.subplots(1, len(PARAMS), figsize=(16, 3.2), sharex=True)
    for j, (ax, name) in enumerate(zip(axs, PARAMS)):
        ax.plot(t, th[:, j], "o-", color="tab:orange", label="without lag (L4)")
        li = sorted((r for r in res if r["car"] == 0 and r["controller"] == "lag_identified"
                     and "identification_lag" in r), key=lambda r: r["tau"])
        if li:
            ax.plot([1000 * r["tau"] for r in li], [r["identification_lag"]["theta"][j] for r in li], "s--",
                    color="tab:olive", label="with lag")
        ax.axhline(0.0 if name == "aero_front" else 1.0, color="k", lw=0.8)
        ax.axhline(LO[j], color="r", lw=0.6, ls=":"); ax.axhline(HI[j], color="r", lw=0.6, ls=":")
        ax.set_xscale("log"); ax.set_title(name, fontsize=9); ax.grid(alpha=0.3)
    axs[0].set_ylabel("identified factor"); axs[len(PARAMS) // 2].set_xlabel("steering lag τ [ms] (nominal car)")
    axs[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(FIG / "identification.png", dpi=130); plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", default=str(ROOT / "results" / "L6"))
    d = Path(ap.parse_args().results)
    FIG.mkdir(parents=True, exist_ok=True)
    res = load(d, "round1.jsonl") + load(d, "rl.jsonl") + load(d, "round2.jsonl")
    sweep(res)
    completion(res)
    identification(res)
    print("figures in", FIG)


if __name__ == "__main__":
    main()
