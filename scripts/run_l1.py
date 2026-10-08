"""Reproduce the L1 results and figures (dynamic single-track model).

    python scripts/fetch_tracks.py Spielberg
    python scripts/run_l1.py            # ~20 min; writes results/L1/ and docs/figures/L1/
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from f1race.dynamics import ellipse_usage, tyre_forces  # noqa: E402
from f1race.experiments.l1 import L1Config, run  # noqa: E402


def figures(ctx, figdir: Path) -> None:
    figdir.mkdir(parents=True, exist_ok=True)
    tr, veh, o0, o1 = ctx["track"], ctx["veh"], ctx["o0"], ctx["o1"]

    # 1) speed: kinematic vs dynamic optimum
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(o0.s, o0.v * 3.6, label=f"L0 kinematic OCP ({o0.lap_time:.2f} s)")
    ax.plot(o1.s, np.hypot(o1.vx, o1.vy) * 3.6, label=f"L1 dynamic OCP ({o1.lap_time:.2f} s)")
    ax.set_xlabel("centreline arclength [m]")
    ax.set_ylabel("speed [km/h]")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(figdir / "speed.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 2) tyre usage, slip angles and side-slip of the dynamic optimum
    F = tyre_forces(veh, o1.vx, o1.vy, o1.r, o1.delta, o1.f_drive, o1.f_brake)
    uf, ur = ellipse_usage(veh, F)
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axs[0].plot(o1.s, uf, lw=0.8, label="front axle")
    axs[0].plot(o1.s, ur, lw=0.8, label="rear axle")
    axs[0].set_ylabel("ellipse usage")
    axs[1].plot(o1.s, np.degrees(F["af"]), lw=0.8, label="front")
    axs[1].plot(o1.s, np.degrees(F["ar"]), lw=0.8, label="rear")
    axs[1].axhline(np.degrees(veh.alpha_peak), color="k", ls=":", lw=0.8)
    axs[1].axhline(-np.degrees(veh.alpha_peak), color="k", ls=":", lw=0.8)
    axs[1].set_ylabel("slip angle [deg]")
    axs[2].plot(o1.s, np.degrees(np.arctan2(o1.vy, o1.vx)), lw=0.8)
    axs[2].set_ylabel("side-slip [deg]")
    axs[2].set_xlabel("centreline arclength [m]")
    for a in axs:
        a.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[1].legend(fontsize=8)
    fig.savefig(figdir / "tyres.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 3) lines of the two optima near the tightest corner
    left, right = tr.edges()
    i = int(np.argmax(np.abs(tr.kappa)))
    fig, axs = plt.subplots(1, 2, figsize=(13, 6))
    for ax, (cx, cy) in zip(axs, [(tr.x[i], tr.y[i]), (tr.x[len(tr.s) * 7 // 10], tr.y[len(tr.s) * 7 // 10])]):
        ax.plot(*left.T, "k", lw=0.8)
        ax.plot(*right.T, "k", lw=0.8)
        ax.plot(*o0.xy(tr).T, label="L0 rear axle (kinematic)")
        ax.plot(*o1.xy(tr).T, label="L1 CG (dynamic)")
        ax.set_xlim(cx - 80, cx + 80)
        ax.set_ylim(cy - 80, cy + 80)
        ax.set_aspect("equal")
        ax.legend(fontsize=8)
    fig.savefig(figdir / "lines.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 4) closed loop of both plans on the dynamic car
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for name, r in [("L1 plan + dynamic tracker", ctx["cl_l1"]), ("L0 plan + kinematic tracker", ctx["cl_l0"])]:
        lbl = f"{name}: " + (f"{r.lap_time:.2f} s" if r.completed else r.stop_reason)
        axs[0].plot(r.progress, r.usage.max(axis=1), lw=0.8, label=lbl)
        axs[1].plot(r.progress, r.margin.min(axis=1), lw=0.8, label=name)
        axs[2].plot(r.progress, np.hypot(r.state[:, 3], r.state[:, 4]) * 3.6, lw=0.8, label=name)
    axs[0].axhline(1.0, color="k", ls=":", lw=0.8)
    axs[0].set_ylabel("max axle usage")
    axs[1].set_ylabel("edge margin [m]")
    axs[2].set_ylabel("speed [km/h]")
    axs[2].set_xlabel("distance [m]")
    for a in axs:
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.savefig(figdir / "closed_loop.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    res, ctx = run(L1Config(), outdir=ROOT / "results" / "L1")
    figures(ctx, ROOT / "docs" / "figures" / "L1")


if __name__ == "__main__":
    main()
