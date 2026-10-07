"""Reproduce the L0 results and figures.

    python scripts/fetch_tracks.py Spielberg
    python scripts/run_l0.py            # writes results/L0/ and docs/figures/L0/
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

from f1race.experiments.l0 import L0Config, run  # noqa: E402
from f1race.vehicle import G  # noqa: E402


def figures(ctx, figdir: Path) -> None:
    figdir.mkdir(parents=True, exist_ok=True)
    tr, veh, mc, q, o, oi = ctx["track"], ctx["veh"], ctx["mc"], ctx["q_plan"], ctx["o_plan"], ctx["o_ideal"]
    left, right = tr.edges()
    oxy = oi.xy(tr)

    # 1) track with lines, coloured by the ideal OCP speed; inset on the tightest corner
    fig, axs = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [1.4, 1]})
    for ax in axs:
        ax.plot(*left.T, "k", lw=0.8)
        ax.plot(*right.T, "k", lw=0.8)
        ax.plot(tr.x, tr.y, "k--", lw=0.5, label="centreline")
        ax.plot(mc.x, mc.y, color="tab:blue", lw=1.0, label="minimum curvature")
        sc = ax.scatter(oxy[:, 0], oxy[:, 1], c=oi.v * 3.6, s=3, cmap="turbo", label="minimum time (OCP)")
        ax.set_aspect("equal")
    i = int(np.argmax(np.abs(tr.kappa)))
    cx, cy = tr.x[i], tr.y[i]
    axs[1].set_xlim(cx - 60, cx + 60)
    axs[1].set_ylim(cy - 60, cy + 60)
    axs[1].set_title("tightest corner")
    axs[0].legend(loc="lower left", fontsize=8)
    axs[0].set_title(f"{tr.name}: lines (rear axle), colour = OCP speed")
    fig.colorbar(sc, ax=axs, label="speed [km/h]", shrink=0.8)
    fig.savefig(figdir / "lines.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 2) speed profiles over centreline arclength
    md = ctx["mc_dense"]
    s_md, _ = tr.project(md.x, md.y)
    order = np.argsort(s_md)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(s_md[order], ctx["q_ideal"].v[order] * 3.6,
            label=f"min-curvature + QSS ({ctx['q_ideal'].lap_time:.2f} s)")
    ax.plot(oi.s, oi.v * 3.6, label=f"minimum-time OCP ({oi.lap_time:.2f} s)")
    ax.set_xlabel("centreline arclength [m]")
    ax.set_ylabel("speed [km/h]")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.savefig(figdir / "speed.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 3) g-g usage of the ideal OCP solution
    v2 = oi.v**2
    fz = veh.fz(v2)
    ax_n = (oi.fx - veh.resist(v2)) / veh.m / G
    ay_n = v2 * np.tan(oi.delta) / veh.L / G
    use = np.hypot(oi.fx / (veh.mu_x * fz), veh.m * v2 * np.tan(oi.delta) / veh.L / (veh.mu_y * fz))
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))
    s0 = axs[0].scatter(ay_n, ax_n, c=oi.v * 3.6, s=4, cmap="turbo")
    axs[0].set_xlabel("lateral acceleration [g]")
    axs[0].set_ylabel("longitudinal acceleration [g]")
    axs[0].set_title("g-g diagram (OCP, eta = 1)")
    axs[0].grid(alpha=0.3)
    fig.colorbar(s0, ax=axs[0], label="speed [km/h]")
    axs[1].plot(oi.s, use)
    axs[1].set_xlabel("centreline arclength [m]")
    axs[1].set_ylabel("friction-ellipse usage")
    axs[1].grid(alpha=0.3)
    fig.savefig(figdir / "gg.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # 4) closed-loop verification
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for name, (path, v, r) in ctx["sims"].items():
        axs[0].plot(r.progress, r.usage, lw=0.8, label=f"{name} ({r.lap_time:.2f} s)")
        axs[1].plot(r.progress, r.margin.min(axis=1), lw=0.8, label=name)
        axs[2].plot(r.progress, r.state[:, 3] * 3.6, lw=0.8, label=name)
    axs[0].set_ylabel("ellipse usage")
    axs[1].set_ylabel("edge margin [m]")
    axs[2].set_ylabel("speed [km/h]")
    axs[2].set_xlabel("distance [m]")
    for a in axs:
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.savefig(figdir / "closed_loop.png", dpi=130, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    res, ctx = run(L0Config(), outdir=ROOT / "results" / "L0")
    figures(ctx, ROOT / "docs" / "figures" / "L0")


if __name__ == "__main__":
    main()
