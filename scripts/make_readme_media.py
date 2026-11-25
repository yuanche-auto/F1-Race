"""Generate the README media (docs/media/) from the project's own code and results.

The animations, the map and the 3D model are drawn from a fresh L0 run (the numbers of
the L0 table) and, for the spin animation, from the L1 dynamic plans; the results chart
takes its numbers from the README tables (RESULTS below, each row with its source).

    python scripts/fetch_tracks.py Spielberg
    python scripts/make_readme_media.py                    # everything (~6 min, + ~10 min for L1)
    python scripts/make_readme_media.py chart banner       # only some outputs
    python scripts/make_readme_media.py --no-l1            # skip the spin animation

Outputs in docs/media/:
  ghost_race.webp / .mp4     the three ideal L0 plans driven side by side (x4 playback)
  spin.webp / .mp4           L1: the kinematic optimum on the dynamic car against the dynamic plan
  spielberg[.zh].geojson     the minimum-time line on a map (georeferenced, see GEO_ANCHOR)
  speed_landscape.stl        the same line in 3D, height = speed
  banner_{en,zh}.svg         animated header (a dot drives the minimum-time lap)
  results_{en,zh}_{light,dark}.svg   completed laps, baseline -> the layer's method
  winners_{en,zh}_{light,dark}.svg   who wins where: laps per controller and test condition
  snippets/                  the compact GeoJSON / STL blocks inlined in the READMEs

The L1 plans are read from results/L1/l1_plans.pkl when it exists (scripts/run_l1.py),
otherwise the three solves the eta = 0.95 plan needs are run here. ffmpeg must be on PATH
for the GIF / MP4 files.
"""
from __future__ import annotations

import argparse
import json
import pickle
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "docs" / "media"

# Latitude / longitude of the origin of the TUM track frame (x east, y north, metres) for
# Spielberg. Fitted once by ICP against the Red Bull Ring outline of bacinger/f1-circuits
# (MIT): RMS distance 1.8 m, rotation 0.05 deg (neglected), scale 1.
GEO_ANCHOR = (47.220328, 14.766711)
EARTH_R = 6378137.0

# chart / animation colours (dark broadcast style; categorical slots 1-3, dark steps)
BG = "#0d1117"
PANEL = "#161b22"
INK = "#f0efec"
INK2 = "#c3c2b7"
MUTED = "#898781"
GRID = "#2c2c2a"
ASPHALT = "#30363d"
SERIES = ["#3987e5", "#d95926", "#199e70"]
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


# --------------------------------------------------------------------------- data
def l0_lines(cache: Path | None = None):
    """The three ideal plans of the L0 table, each as (xy, t, v) on a closed path.
    ``cache``: a pickle of the lines, written on the first call and read afterwards."""
    from f1race.experiments.l0 import L0Config, run
    from f1race.path import Path as CPath
    from f1race.planning.qss import speed_profile
    from f1race.track import Track

    if cache is not None and cache.exists():
        with open(cache, "rb") as fh:
            lines = pickle.load(fh)
        return Track.load(L0Config().track), None, labelled(lines)

    cfg = L0Config()
    res, ctx = run(cfg, outdir=None, verbose=False)
    tr, veh = ctx["track"], ctx["veh"]
    cl = CPath.from_points(np.column_stack([tr.x[::cfg.mincurv_step], tr.y[::cfg.mincurv_step]]),
                           resample=cfg.qss_ds)
    q_cl = speed_profile(cl, veh)
    md, q_mc = ctx["mc_dense"], ctx["q_ideal"]
    oi = ctx["o_ideal"]
    oxy = oi.xy(tr)
    # time along the OCP line from its arclength and speed, scaled to the OCP's own lap time
    seg = np.hypot(*np.diff(np.vstack([oxy, oxy[:1]]), axis=0).T)
    dt = 2.0 * seg / (oi.v + np.roll(oi.v, -1))
    t_o = np.concatenate([[0.0], np.cumsum(dt)[:-1]]) * oi.lap_time / dt.sum()
    lines = [
        dict(key="ocp", lap=oi.lap_time,
             xy=oxy, t=t_o, v=oi.v),
        dict(key="mincurv", lap=q_mc.lap_time, xy=np.column_stack([md.x, md.y]), t=q_mc.t, v=q_mc.v),
        dict(key="centre", lap=q_cl.lap_time,
             xy=np.column_stack([cl.x, cl.y]), t=q_cl.t, v=q_cl.v),
    ]
    for ln in lines:                                   # centreline progress of every point
        s, _ = tr.project(ln["xy"][:, 0], ln["xy"][:, 1])
        ln["s"] = np.unwrap(s, period=tr.length)
        ln["s"] -= ln["s"][0]
    print("L0 lap times:", {ln["key"]: round(ln["lap"], 2) for ln in lines})
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        with open(cache, "wb") as fh:
            pickle.dump(lines, fh)
    return tr, veh, labelled(lines)


LABELS = {"ocp": ("Minimum-time OCP", "line + speed optimised"),
          "mincurv": ("Min. curvature + QSS", "geometric line first"),
          "centre": ("Centre line + QSS", "no racing line")}


def labelled(lines):
    for ln in lines:
        ln["name"], ln["sub"] = LABELS[ln["key"]]
    return lines


def closed(a):
    return np.concatenate([a, a[:1]])


def at_time(ln, t):
    """Position, speed and centreline progress of a line at time t (clamped to the lap)."""
    tt = np.append(ln["t"], ln["lap"])
    t = min(t, ln["lap"])
    x = np.interp(t, tt, closed(ln["xy"][:, 0]))
    y = np.interp(t, tt, closed(ln["xy"][:, 1]))
    v = np.interp(t, tt, closed(ln["v"]))
    s = np.interp(t, tt, np.append(ln["s"], ln["s"][0] + ln["s_len"]))
    return x, y, v, s


def ffmpeg_pipe(path: Path, w: int, h: int, fps: int):
    return subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}",
         "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22",
         "-preset", "slow", "-movflags", "+faststart", str(path)], stdin=subprocess.PIPE)


def to_webp(src, out: Path, fps: int = 15, width: int = 960, quality: int = 75, src_fps: int | None = None,
            pad_16_9: bool = False):
    """Animated WebP for the README (about half the size of a GIF, full colour; GitHub plays it
    like a GIF). ``src`` is a video file, or an image sequence pattern with ``src_fps``;
    ``pad_16_9`` letterboxes a wider clip to 16:9 on the page background."""
    inp = ["-framerate", str(src_fps), "-i", str(src)] if src_fps else ["-i", str(src)]
    vf = f"fps={fps},scale={width}:-2:flags=lanczos"
    if pad_16_9:
        vf += f",pad={width}:{round(width * 9 / 16 / 2) * 2}:(ow-iw)/2:(oh-ih)/2:color=0x0d1117"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *inp, "-vf", vf,
                    "-c:v", "libwebp_anim", "-loop", "0", "-q:v", str(quality), "-compression_level", "6",
                    "-preset", "drawing", str(out)], check=True)
    print(f"{out.name}: {out.stat().st_size / 1e6:.2f} MB")


def style_axes(ax):
    ax.set_facecolor(BG)
    for sp in ax.spines.values():
        sp.set_visible(False)


# --------------------------------------------------------------------------- ghost race
def ghost_race(tr, lines, speedup: float = 4.0, fps: int = 30, hold: float = 3.0):
    W, H, DPI = 1280, 720, 125                         # 125 dpi: text legible at README width
    for ln in lines:
        ln["s_len"] = tr.length
    lead_lap = min(ln["lap"] for ln in lines)
    t_end = max(ln["lap"] for ln in lines)
    dt_frame = speedup / fps
    n_run = int(np.ceil(t_end / dt_frame)) + 1
    n_hold = int(hold * fps)

    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.025, 0.945, "Spielberg · one lap · three ways to drive it", color=INK, fontsize=17,
             fontweight="bold", va="center")
    fig.text(0.025, 0.905, "kinematic car model, ideal plans (friction fully used) — the L0 table",
             color=MUTED, fontsize=10.5, va="center")
    fig.text(0.975, 0.945, f"×{speedup:g} playback", color=INK2, fontsize=11, ha="right", va="center",
             bbox=dict(boxstyle="round,pad=0.35", fc=PANEL, ec=GRID))

    # map
    ax = fig.add_axes([0.01, 0.235, 0.57, 0.64])
    style_axes(ax)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    left, right = tr.edges()
    ax.fill(np.concatenate([left[:, 0], right[::-1, 0]]), np.concatenate([left[:, 1], right[::-1, 1]]),
            color=ASPHALT, lw=0, zorder=1)
    # the fill of a closed band leaves a seam; re-draw the band as a thick line to hide it
    pad = 40
    ax.set_xlim(tr.x.min() - pad, tr.x.max() + pad)
    ax.set_ylim(tr.y.min() - pad, tr.y.max() + pad)
    ax.figure.canvas.draw()
    px_per_m = ax.transData.transform([1, 0])[0] - ax.transData.transform([0, 0])[0]
    width_pt = 12.0 * px_per_m * 72 / DPI
    for poly in (left, right):
        ax.plot(*closed(poly).T, color="#484f58", lw=0.6, zorder=2)
    ax.plot(*closed(np.column_stack([tr.x, tr.y])).T, color=ASPHALT, lw=width_pt, zorder=1,
            solid_capstyle="round")
    # start / finish line
    p0 = tr.point_at(np.array([0.0]), np.array([0.0]))[0]
    nrm = tr.normal[0]
    w0 = 8.0
    ax.plot([p0[0] - w0 * nrm[0], p0[0] + w0 * nrm[0]], [p0[1] - w0 * nrm[1], p0[1] + w0 * nrm[1]],
            color=INK, lw=2.0, zorder=3)
    ax.annotate("START / FINISH", p0, xytext=(0, -22), textcoords="offset points", color=INK2,
                fontsize=8, ha="center", zorder=3)

    # inset: tightest corner
    i_k = int(np.argmax(np.abs(tr.kappa)))
    cx, cy = tr.x[i_k], tr.y[i_k]
    ins = ax.inset_axes([-60, 350, 400, 370], transform=ax.transData)   # the empty top-right corner
    style_axes(ins)
    ins.set_facecolor(PANEL)
    ins.set_aspect("equal")
    ins.set_xticks([])
    ins.set_yticks([])
    ins.set_xlim(cx - 22, cx + 98)                     # apex on the left, both legs in view
    ins.set_ylim(cy - 92, cy + 28)
    ins.fill(np.concatenate([left[:, 0], right[::-1, 0]]), np.concatenate([left[:, 1], right[::-1, 1]]),
             color=ASPHALT, lw=0)
    for poly in (left, right):
        ins.plot(*poly.T, color="#484f58", lw=0.8)
    for ln, c in zip(lines, SERIES):
        ins.plot(*closed(ln["xy"]).T, color=c, lw=1.4, alpha=0.85)
    ins.text(0.04, 0.95, "tightest corner", transform=ins.transAxes, color=INK2, fontsize=8.5, va="top")
    ax.plot([cx], [cy], marker="s", ms=14, mfc="none", mec=MUTED, mew=0.8, zorder=3)

    dots, trails, idots = [], [], []
    for ln, c in zip(lines, SERIES):
        trail, = ax.plot([], [], color=c, lw=2.2, alpha=0.75, zorder=4, solid_capstyle="round")
        dot, = ax.plot([], [], "o", ms=9, mfc=c, mec=BG, mew=2, zorder=6)
        idot, = ins.plot([], [], "o", ms=8, mfc=c, mec=PANEL, mew=1.5, zorder=6)
        trails.append(trail)
        dots.append(dot)
        idots.append(idot)

    # timing tower
    x0 = 0.615
    fig.text(x0, 0.84, "LAP TIME", color=MUTED, fontsize=10, fontweight="bold")
    clock = fig.text(x0, 0.775, "", color=INK, fontsize=30, fontweight="bold", family="DejaVu Sans Mono",
                     va="center")
    fig.text(x0, 0.70, "POS   LINE", color=MUTED, fontsize=9, fontweight="bold")
    fig.text(0.905, 0.70, "SPEED", color=MUTED, fontsize=9, fontweight="bold", ha="right")
    fig.text(0.975, 0.70, "GAP", color=MUTED, fontsize=9, fontweight="bold", ha="right")
    rows = []
    for k in range(3):
        y = 0.635 - k * 0.085
        r = dict(
            pos=fig.text(x0 + 0.005, y, "", color=INK, fontsize=15, fontweight="bold", va="center"),
            bar=fig.add_artist(plt.Rectangle((x0 + 0.03, y - 0.026), 0.005, 0.052, transform=fig.transFigure,
                                             color=SERIES[k], lw=0)),
            name=fig.text(x0 + 0.045, y + 0.012, "", color=INK, fontsize=11.5, va="center"),
            sub=fig.text(x0 + 0.045, y - 0.017, "", color=MUTED, fontsize=8.5, va="center"),
            v=fig.text(0.905, y, "", color=INK, fontsize=12, va="center", ha="right", family="DejaVu Sans Mono"),
            gap=fig.text(0.975, y, "", color=INK2, fontsize=12, va="center", ha="right",
                         family="DejaVu Sans Mono"),
        )
        rows.append(r)
    note = fig.text(x0, 0.405, "", color=INK2, fontsize=10, va="top", linespacing=1.35)

    # speed traces over centreline distance
    axs = fig.add_axes([0.055, 0.075, 0.92, 0.15])
    style_axes(axs)
    axs.set_xlim(0, tr.length / 1000)
    axs.set_ylim(40, 340)
    axs.set_yticks([100, 200, 300])
    axs.set_xticks(np.arange(0, 4.5, 0.5))
    axs.tick_params(colors=MUTED, labelsize=8.5, length=0)
    axs.grid(color=GRID, lw=0.8)
    axs.set_axisbelow(True)
    axs.text(1.0, 1.08, "distance along the centre line [km]", transform=axs.transAxes, color=MUTED, fontsize=9,
             ha="right")
    axs.text(-0.04, 1.08, "SPEED [km/h]", transform=axs.transAxes, color=MUTED, fontsize=9, fontweight="bold")
    traces, tdots = [], []
    for ln, c in zip(lines[::-1], SERIES[::-1]):
        tl, = axs.plot([], [], color=c, lw=1.6)
        traces.insert(0, tl)
    for c in SERIES:
        td, = axs.plot([], [], "o", ms=6, mfc=c, mec=BG, mew=1.5, zorder=5)
        tdots.append(td)
    fig.text(0.975, 0.012, "github.com/yuanche-auto/F1-Race · scripts/make_readme_media.py", color="#6e7681",
             fontsize=8, ha="right")

    def lead_time_at(s):
        ref = lines[0]                                 # the leader on every frame here is the OCP
        return np.interp(s, np.append(ref["s"], ref["s_len"]), np.append(ref["t"], ref["lap"]))

    mp4 = OUT / "ghost_race.mp4"
    proc = ffmpeg_pipe(mp4, W, H, fps)
    for f in range(n_run + n_hold):
        t = min(f * dt_frame, t_end)
        state = []
        for k, ln in enumerate(lines):
            x, y, v, s = at_time(ln, t)
            done = t >= ln["lap"]
            tt = np.linspace(max(0.0, min(t, ln["lap"]) - 2.0), min(t, ln["lap"]), 25)
            pts = np.array([at_time(ln, q)[:2] for q in tt])
            trails[k].set_data(pts[:, 0], pts[:, 1])
            dots[k].set_data([x], [y])
            idots[k].set_data([x], [y])
            m = ln["t"] <= t
            traces[k].set_data(ln["s"][m] / 1000, ln["v"][m] * 3.6)
            tdots[k].set_data([s / 1000], [v * 3.6])
            gap = (ln["lap"] - lead_lap) if done else t - lead_time_at(s)
            state.append((s if not done else 1e9 + 1.0 / ln["lap"], k, v, gap, done))
        order = sorted(state, key=lambda z: -z[0])
        clock.set_text(f"{int(t // 60)}:{t % 60:06.3f}")
        for p, (s, k, v, gap, done) in enumerate(order):
            r, ln = rows[p], lines[k]
            r["pos"].set_text(str(p + 1))
            r["bar"].set_color(SERIES[k])
            r["name"].set_text(ln["name"])
            r["sub"].set_text(ln["sub"])
            if done:
                r["v"].set_text(f"{ln['lap']:.2f} s")
                r["gap"].set_text("LEADER" if p == 0 else f"+{gap:.2f}")
            else:
                r["v"].set_text(f"{v * 3.6:5.0f}")
                r["gap"].set_text("—" if p == 0 else f"+{max(gap, 0):.2f}")
        if t >= t_end:
            note.set_text(f"racing line alone: −{lines[2]['lap'] - lines[1]['lap']:.2f} s\n"
                          f"line + speed together: another −{lines[1]['lap'] - lines[0]['lap']:.2f} s")
        fig.canvas.draw()
        proc.stdin.write(fig.canvas.buffer_rgba().tobytes())
    proc.stdin.close()
    proc.wait()
    plt.close(fig)
    print(f"{mp4.name}: {mp4.stat().st_size / 1e6:.2f} MB")
    to_webp(mp4, OUT / "ghost_race.webp")


# --------------------------------------------------------------------------- L1 spin
def l1_runs(tr):
    """Closed-loop runs of L1: the L0 plan (eta 0.97) on the dynamic car with the L0 tracker,
    and the dynamic plan (eta 0.95) with the L1 tracker."""
    from f1race.experiments import l1 as L1

    cfg = L1.L1Config()
    veh = L1.DynVehicle()
    cache = ROOT / "results" / "L1" / "l1_plans.pkl"
    if cache.exists():
        with open(cache, "rb") as fh:
            d = pickle.load(fh)
        o0p, o95 = d["o0p"], d["plans"][cfg.eta_plan]
    else:
        alpha, mc = L1.min_curvature_line(tr, veh, margin=cfg.margin)
        q = L1.speed_profile(mc, veh, 1.0)
        idx = np.arange(0, len(tr.s), 3)
        init0 = L1.initial_guess_from_path(tr, veh, idx, alpha, mc, q.v, q.a)
        o0 = L1.solve_min_time(tr, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=1.0)
        o0p = L1.solve_min_time(tr, veh, init0, ds=cfg.ds_l0, margin=cfg.margin, eta=cfg.eta_l0_plan)
        kw = dict(ds=cfg.ds_l1, nsub=cfg.nsub, margin=cfg.margin)
        o97 = L1.first_dynamic_plan(tr, veh, o0, cfg.first_eta, [], **kw)
        o1 = L1.solve_min_time_dynamic(tr, veh, L1.initial_guess_from_dynamic(o97), eta=1.0, **kw)
        o95 = L1.solve_min_time_dynamic(tr, veh, L1.initial_guess_from_dynamic(o1), eta=cfg.eta_plan, **kw)
    r_dyn = L1.run_dyn_closed_loop(tr, veh, o95, cfg.dt, cfg.tracker)
    r_kin = L1.run_l0_plan_on_dyn(tr, veh, o0p, cfg.dt)
    print(f"L1: dynamic plan {r_dyn.lap_time:.2f} s ({r_dyn.stop_reason}); "
          f"L0 plan {r_kin.stop_reason} at {r_kin.progress[-1]:.0f} m, t = {r_kin.t[-1]:.2f} s")
    return veh, r_kin, r_dyn


def car_polygon(x, y, psi, length=5.4, width=2.0):
    c, s = np.cos(psi), np.sin(psi)
    pts = np.array([[length * 0.55, 0], [length * 0.35, width / 2], [-length / 2, width / 2],
                    [-length / 2, -width / 2], [length * 0.35, -width / 2]])
    return np.column_stack([x + pts[:, 0] * c - pts[:, 1] * s, y + pts[:, 0] * s + pts[:, 1] * c])


def spin(tr, veh, r_kin, r_dyn, fps: int = 30, slow: float = 0.5, after: float = 1.5, hold: float = 2.5):
    W, H, DPI = 1280, 720, 125                         # 125 dpi: text legible at README width
    runs = [dict(r=r_kin, name="L0 kinematic optimum + L0 tracker", c=SERIES[1], short="kinematic plan"),
            dict(r=r_dyn, name="L1 dynamic plan (5 % margin) + L1 tracker", c=SERIES[0], short="dynamic plan")]
    t_spin = float(r_kin.t[-1])
    t0 = max(0.0, t_spin - 4.0)                        # start on the straight, ~4 s before the spin
    t1 = t_spin + after
    for d in runs:
        st = d["r"].state
        d["beta"] = np.degrees(np.arctan2(st[:, 4], st[:, 3]))
        d["v"] = np.hypot(st[:, 3], st[:, 4]) * 3.6

    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI, facecolor=BG)
    fig.text(0.025, 0.945, "Same car, two plans: why L1 needs a dynamic model", color=INK, fontsize=17,
             fontweight="bold", va="center")
    fig.text(0.025, 0.905, "dynamic single-track car, Pacejka tyres, load transfer: the kinematic plan spins at turn 1",
             color=MUTED, fontsize=10.5, va="center")
    badge = fig.text(0.975, 0.945, "", color=INK2, fontsize=11, ha="right", va="center",
                     bbox=dict(boxstyle="round,pad=0.35", fc=PANEL, ec=GRID))

    ax = fig.add_axes([0.01, 0.06, 0.60, 0.80])
    style_axes(ax)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    left, right = tr.edges()
    ax.fill(np.concatenate([left[:, 0], right[::-1, 0]]), np.concatenate([left[:, 1], right[::-1, 1]]),
            color=ASPHALT, lw=0)
    for poly in (left, right):
        ax.plot(*closed(poly).T, color="#6e7681", lw=1.2)
    # kerb-like centre dashes
    ax.plot(tr.x, tr.y, color="#484f58", lw=0.8, ls=(0, (6, 8)))
    for d in runs:
        st = d["r"].state
        d["path"], = ax.plot([], [], color=d["c"], lw=1.6, alpha=0.8)
        d["poly"] = ax.add_patch(Polygon(car_polygon(0, 0, 0), closed=True, fc=d["c"], ec=BG, lw=1.0, zorder=5))
        d["vel"] = ax.annotate("", xy=(0, 0), xytext=(0, 0), zorder=6,
                               arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.3, mutation_scale=12))
        d["label"] = ax.text(0, 0, d["short"], color=INK, fontsize=9.5, zorder=7,
                             bbox=dict(boxstyle="round,pad=0.25", fc=PANEL, ec=d["c"], lw=1.0))
    spin_txt = ax.text(0.5, 0.08, "", transform=ax.transAxes, color=INK, fontsize=18, fontweight="bold",
                       ha="center", bbox=dict(boxstyle="round,pad=0.4", fc=SERIES[1], ec="none"))
    ax.text(0.02, 0.02, "arrow = velocity direction; the angle between body and arrow is the side-slip angle",
            transform=ax.transAxes, color=MUTED, fontsize=9)

    # telemetry
    x0 = 0.645
    fig.text(x0, 0.835, "SIDE-SLIP ANGLE β [deg]", color=MUTED, fontsize=9.5, fontweight="bold")
    axb = fig.add_axes([x0 + 0.02, 0.50, 0.31, 0.31])
    fig.text(x0, 0.42, "SPEED [km/h]", color=MUTED, fontsize=9.5, fontweight="bold")
    axv = fig.add_axes([x0 + 0.02, 0.12, 0.31, 0.28])
    lim_deg = np.degrees(0.5)
    for a in (axb, axv):
        style_axes(a)
        a.tick_params(colors=MUTED, labelsize=8.5, length=0)
        a.grid(color=GRID, lw=0.8)
        a.set_axisbelow(True)
        a.set_xlim(t0, t1)
    axb.set_ylim(-35, 35)
    axb.axhline(lim_deg, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    axb.axhline(-lim_deg, color=MUTED, lw=1.0, ls=(0, (4, 3)))
    axb.text(t0 + 0.1, lim_deg + 1.5, "spin: |β| > 0.5 rad", color=INK2, fontsize=8.5)
    axb.set_xticklabels([])
    axv.set_xlabel("time [s]", color=MUTED, fontsize=9)
    vmin = min(d["v"][(d["r"].t >= t0) & (d["r"].t <= t1)].min() for d in runs)
    vmax = max(d["v"][(d["r"].t >= t0) & (d["r"].t <= t1)].max() for d in runs)
    axv.set_ylim(vmin - 15, vmax + 15)
    for d in runs:
        d["bl"], = axb.plot([], [], color=d["c"], lw=1.8)
        d["vl"], = axv.plot([], [], color=d["c"], lw=1.8)
    for k, d in enumerate(runs):
        fig.add_artist(plt.Rectangle((x0 + 0.02 + k * 0.165, 0.035), 0.012, 0.012, transform=fig.transFigure,
                                     color=d["c"], lw=0))
        fig.text(x0 + 0.037 + k * 0.165, 0.041, d["short"], color=INK2, fontsize=9, va="center")

    mp4 = OUT / "spin.mp4"
    proc = ffmpeg_pipe(mp4, W, H, fps)
    n_run = int(np.ceil((t1 - t0) / (slow / fps))) + 1
    n_hold = int(hold * fps)
    for f in range(n_run + n_hold):
        t = min(t0 + f * slow / fps, t1)
        cams = []
        for d in runs:
            r = d["r"]
            tt = min(t, float(r.t[-1]))
            st = np.array([np.interp(tt, r.t, r.state[:, j]) for j in range(r.state.shape[1])])
            px, py, psi, vx, vy = st[:5]
            d["poly"].set_xy(car_polygon(px, py, psi))
            vdir = psi + np.arctan2(vy, vx)
            d["vel"].xy = (px + 9 * np.cos(vdir), py + 9 * np.sin(vdir))
            d["vel"].set_position((px, py))
            d["label"].set_position((px + 4, py + (6 if d is runs[0] else -9)))
            m = (r.t >= t0 - 3) & (r.t <= tt)
            d["path"].set_data(r.state[m, 0], r.state[m, 1])
            mb = (r.t >= t0) & (r.t <= tt)
            d["bl"].set_data(r.t[mb], d["beta"][mb])
            d["vl"].set_data(r.t[mb], d["v"][mb])
            cams.append((px, py))
        if t >= t_spin:
            spin_txt.set_text(f"SPIN at {r_kin.progress[-1]:.0f} m (turn 1)")
        badge.set_text(f"×{slow:g} slow motion" if t < t1 else
                       f"the dynamic plan completes the lap: {r_dyn.lap_time:.1f} s")
        cxy = np.mean(cams, axis=0) if np.hypot(*np.subtract(*cams)) < 70 else np.array(cams[0])
        half = 48
        ax.set_xlim(cxy[0] - half * 1.05, cxy[0] + half * 1.05)
        ax.set_ylim(cxy[1] - half * 0.95, cxy[1] + half * 0.95)
        fig.canvas.draw()
        proc.stdin.write(fig.canvas.buffer_rgba().tobytes())
    proc.stdin.close()
    proc.wait()
    plt.close(fig)
    print(f"{mp4.name}: {mp4.stat().st_size / 1e6:.2f} MB")
    to_webp(mp4, OUT / "spin.webp")


# --------------------------------------------------------------------------- map / 3D
def resample_line(ln, ds: float):
    xy = closed(ln["xy"])
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    q = np.arange(0.0, s[-1], ds)
    v = np.interp(q, s, closed(ln["v"]))
    return np.column_stack([np.interp(q, s, xy[:, 0]), np.interp(q, s, xy[:, 1])]), v, q


def to_lonlat(xy):
    lat0, lon0 = GEO_ANCHOR
    lat = lat0 + np.degrees(xy[:, 1] / EARTH_R)
    lon = lon0 + np.degrees(xy[:, 0] / (EARTH_R * np.cos(np.radians(lat0))))
    return np.column_stack([lon, lat])


MAP_TEXT = {
    "en": dict(band="speed band", start="Start / finish",
               start_d="Minimum-time lap {lap:.2f} s (L0, kinematic model, ideal)",
               top="Top speed", top_d="{v:.0f} km/h", brk="Hardest braking",
               brk_d="up to {g:.1f} g of deceleration (downforce adds grip at speed)",
               both="Top speed, then the hardest braking", both_d="{v:.0f} km/h, then braking at {b}",
               low="Slowest point: the tightest corner", low_d="{v:.0f} km/h at the apex",
               fail="Failure hot spot: {name}",
               fail_d="One of the few corners where failed test laps leave the track; in the control-expansion "
                      "benchmark the same corners for every controller",
               names=("Turn 1 (≈ 460 m)", "≈ 2.2 km", "≈ 2.7 km", "≈ 3.83 km")),
    "zh": dict(band="速度区间", start="起点 / 终点", start_d="最短时间圈 {lap:.2f} s（L0，运动学模型，理想）",
               top="最高速度", top_d="{v:.0f} km/h", brk="最大制动",
               brk_d="最高 {g:.1f} g 的减速度（高速时下压力增加抓地力）",
               both="最高速度，随即最大制动", both_d="{v:.0f} km/h，随后以{b}制动",
               low="最低速度点：最急的弯", low_d="弯心 {v:.0f} km/h",
               fail="失败高发点：{name}",
               fail_d="测试圈冲出赛道的少数几个弯之一；在控制扩展的基准测试中，所有控制器都在这些弯失败",
               names=("1 号弯（≈ 460 m）", "≈ 2.2 km", "≈ 2.7 km", "≈ 3.83 km")),
}


def geojson(tr, ln, ds: float = 10.0):
    """The minimum-time line on a map: segments by speed band (simplestyle colours) and
    markers with the numbers a reader clicks on; one file per README language."""
    xy, v, q = resample_line(ln, ds)
    kmh = v * 3.6
    edges = [0, 120, 170, 220, 270, 1000]
    cols = ["#86b6ef", "#3987e5", "#256abf", "#184f95", "#0d366b"]
    band = np.digitize(kmh, edges) - 1
    ll = to_lonlat(xy)
    runs = []                                           # (band, indices) of each run of one speed band
    i, n = 0, len(xy)
    while i < n:
        j = i
        while j + 1 < n and band[j + 1] == band[i]:
            j += 1
        runs.append((band[i], list(range(i, j + 2)) if j + 1 < n else list(range(i, n)) + [0]))
        i = j + 1

    # markers from the line at its own resolution (the 10 m copy smooths the braking peaks)
    nxy, nv = closed(ln["xy"]), closed(ln["v"])
    nll = to_lonlat(nxy)
    nseg = np.hypot(*np.diff(nxy, axis=0).T)
    n_acc = np.diff(nv**2) / (2.0 * nseg) / 9.81
    n_kmh = nv * 3.6
    k_top, k_low, k_brk = int(np.argmax(n_kmh)), int(np.argmin(n_kmh)), int(np.argmin(n_acc))
    k_fail = [int(np.argmin(np.abs(ln["s"] - s_f))) for s_f in (460.0, 2200.0, 2730.0, 3830.0)]
    print(f"markers: top {n_kmh[k_top]:.0f} km/h at {ln['s'][k_top % len(ln['s'])]:.0f} m, "
          f"slowest {n_kmh[k_low]:.0f} km/h, braking {-n_acc[k_brk]:.2f} g at {ln['s'][k_brk]:.0f} m")

    def marker(k, title, desc, color, symbol):
        return {"type": "Feature",
                "properties": {"title": title, "description": desc, "marker-color": color,
                               "marker-symbol": symbol, "marker-size": "medium"},
                "geometry": {"type": "Point", "coordinates": [round(nll[k, 0], 6), round(nll[k, 1], 6)]}}

    snip = OUT / "snippets"
    snip.mkdir(exist_ok=True)
    for lang, tx in MAP_TEXT.items():
        feats = []
        for b, idx in runs:
            lo, hi = edges[b], edges[b + 1]
            label = f"< {hi} km/h" if lo == 0 else (f"> {lo} km/h" if hi == 1000 else f"{lo}–{hi} km/h")
            feats.append({"type": "Feature",
                          "properties": {tx["band"]: label, "stroke": cols[b], "stroke-width": 5,
                                         "stroke-opacity": 0.95},
                          "geometry": {"type": "LineString",
                                       "coordinates": [[round(a, 6), round(c, 6)] for a, c in ll[idx]]}})
        feats.append(marker(0, tx["start"], tx["start_d"].format(lap=ln["lap"]), "#0b0b0b", "racetrack"))
        brake = tx["brk_d"].format(g=-n_acc[k_brk])
        if np.hypot(*(nxy[k_top] - nxy[k_brk])) < 50.0:  # one marker when they (nearly) coincide
            feats.append(marker(k_top, tx["both"], tx["both_d"].format(v=n_kmh[k_top], b=brake), "#2a78d6",
                                "triangle"))
        else:
            feats += [marker(k_top, tx["top"], tx["top_d"].format(v=n_kmh[k_top]), "#2a78d6", "triangle"),
                      marker(k_brk, tx["brk"], brake, "#2a78d6", "cross")]
        feats.append(marker(k_low, tx["low"], tx["low_d"].format(v=n_kmh[k_low]), "#eb6834", "circle"))
        # the corners where failed test laps leave the track (CONTROL_EXPANSION_BENCHMARK.md, phase A "Reading")
        for k, name in zip(k_fail, tx["names"]):
            feats.append(marker(k, tx["fail"].format(name=name), tx["fail_d"], "#e34948", "danger"))
        fc = {"type": "FeatureCollection", "features": feats}
        suffix = "" if lang == "en" else f".{lang}"
        (OUT / f"spielberg{suffix}.geojson").write_text(json.dumps(fc, ensure_ascii=False, indent=1) + "\n")
        for ft in feats:                                # the inline copy: ~1 m coordinates
            ft["geometry"]["coordinates"] = np.round(np.array(ft["geometry"]["coordinates"]), 5).tolist()
        body = ",\n".join(json.dumps(ft, ensure_ascii=False, separators=(",", ":")) for ft in feats)
        out = snip / f"map{suffix}.geojson"
        out.write_text('{"type":"FeatureCollection","features":[\n' + body + "\n]}\n")
        print(f"{out.name}: {len(feats)} features, {out.stat().st_size / 1e3:.1f} kB")


def simplify_idx(p: np.ndarray, tol: float) -> list[int]:
    """Douglas-Peucker on an open polyline; returns the kept indices."""
    keep = np.zeros(len(p), bool)
    keep[[0, -1]] = True
    stack = [(0, len(p) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        d = p[b] - p[a]
        L = np.linalg.norm(d)
        rel = p[a + 1:b] - p[a]
        if L < 1e-12:
            dist = np.linalg.norm(rel, axis=1)
        else:
            proj = rel @ d / L**2
            dist = np.linalg.norm(rel - np.outer(proj, d), axis=1)
        k = int(np.argmax(dist))
        if dist[k] > tol:
            keep[a + 1 + k] = True
            stack += [(a, a + 1 + k), (a + 1 + k, b)]
    return list(np.flatnonzero(keep))


def stl(ln, path: Path, tol: float, z_per_kmh: float = 0.6, name: str = "speed_landscape", road: bool = True):
    """A double-sided wall along the minimum-time line, its height the speed (0.6 m per km/h),
    optionally on a thin road ribbon; facets in ASCII STL."""
    xy, v, _ = resample_line(ln, 2.0)
    xy = xy - xy.mean(axis=0)
    z = v * 3.6 * z_per_kmh
    pts3 = np.column_stack([xy, z])
    idx = simplify_idx(np.vstack([pts3, pts3[:1]]), tol)
    idx = [i % len(xy) for i in idx[:-1]]
    P, Z = xy[idx], z[idx]
    n = len(P)
    tang = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    nrm = np.column_stack([-tang[:, 1], tang[:, 0]])
    half_w = 6.0
    facets = []

    def tri(a, b, c):
        nv = np.cross(b - a, c - a)
        ln_ = np.linalg.norm(nv)
        if ln_ < 1e-9:
            return
        facets.append((nv / ln_, a, b, c))

    for i in range(n):
        j = (i + 1) % n
        a0, b0 = np.append(P[i], 0.0), np.append(P[j], 0.0)
        a1, b1 = np.append(P[i], Z[i]), np.append(P[j], Z[j])
        tri(a0, b0, b1)                                 # wall, both sides
        tri(a0, b1, a1)
        tri(a0, b1, b0)
        tri(a0, a1, b1)
        if road:
            l0, r0 = np.append(P[i] + half_w * nrm[i], 0.0), np.append(P[i] - half_w * nrm[i], 0.0)
            l1, r1 = np.append(P[j] + half_w * nrm[j], 0.0), np.append(P[j] - half_w * nrm[j], 0.0)
            tri(r0, r1, l1)                             # road, facing up
            tri(r0, l1, l0)

    def f(x):
        return f"{x:.0f}" if abs(x - round(x)) < 0.05 else f"{x:.1f}"

    lines = [f"solid {name}"]
    for nv, a, b, c in facets:
        lines.append("facet normal " + " ".join(f"{q:.3g}" for q in nv))
        lines.append("outer loop")
        for p in (a, b, c):
            lines.append("vertex " + " ".join(f(q) for q in p))
        lines.append("endloop")
        lines.append("endfacet")
    lines.append(f"endsolid {name}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    print(f"{path.name}: {n} stations, {len(facets)} facets, {path.stat().st_size / 1e3:.0f} kB")


# --------------------------------------------------------------------------- banner
def banner(tr, ln):
    """Animated SVG header: the minimum-time line coloured by speed, a dot driving it at
    the plan's own (relative) speed, 4x faster than real time."""
    xy, v, q = resample_line(ln, 6.0)
    t = np.concatenate([[0.0], np.cumsum(2 * np.diff(q) / (v[:-1] + v[1:]))])
    lap_t = t[-1] + 2 * (q[1] - q[0]) / (v[-1] + v[0])
    Wb, Hb = 1280, 340
    box = (800, 28, 430, 262)                           # x, y, w, h of the track area
    span = xy.max(0) - xy.min(0)
    sc = min(box[2] / span[0], box[3] / span[1])
    off = np.array([box[0] + (box[2] - sc * span[0]) / 2, box[1] + (box[3] + sc * span[1]) / 2])
    P = np.column_stack([off[0] + (xy[:, 0] - xy[:, 0].min()) * sc, off[1] - (xy[:, 1] - xy[:, 1].min()) * sc])
    kmh = v * 3.6
    ramp = ["#184f95", "#256abf", "#2a78d6", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb"]
    bands = np.clip(((kmh - 60) / (330 - 60) * len(ramp)).astype(int), 0, len(ramp) - 1)
    segs = []                                           # one polyline per run of a speed band
    i = 0
    while i < len(P):
        j = i
        while j + 1 < len(P) and bands[j + 1] == bands[i]:
            j += 1
        pts = [P[k % len(P)] for k in range(i, j + 2)]
        segs.append(f'<polyline points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in pts)}" '
                    f'stroke="{ramp[bands[i]]}"/>')
        i = j + 1
    d_path = "M" + " L".join(f"{x:.1f},{y:.1f}" for x, y in P) + " Z"
    # keyPoints: fraction of the path length reached at each fraction of the lap time
    seg = np.hypot(*np.diff(np.vstack([P, P[:1]]), axis=0).T)
    frac_len = np.concatenate([[0.0], np.cumsum(seg)])[:-1] / seg.sum()
    frac_t = t / lap_t
    sel = np.arange(0, len(P), 4)
    kp = ";".join(f"{a:.4f}" for a in np.append(frac_len[sel], 1.0))
    kt = ";".join(f"{a:.4f}" for a in np.append(frac_t[sel], 1.0))
    dur = lap_t / 4.0
    texts = {
        "en": ("Minimum-lap-time racing, layer by layer",
               "From optimal racing lines to MPC under uncertainty, system identification,",
               "reinforcement learning and real-time control — compared on the same car, track and test laps."),
        "zh": ("逐层递进的最小圈速赛车研究",
               "从最优赛车线到不确定性下的 MPC、系统辨识、强化学习与实时控制",
               "所有方法在同一辆车、同一条赛道、同一批测试圈上对比"),
    }
    chips = {"en": [("L0 – L9 done", 128), ("9 control phases", 160), ("pre-registered tests", 190)],
             "zh": [("L0 – L9 已完成", 136), ("9 个控制扩展阶段", 166), ("预注册的统计检验", 166)]}
    caption = {"en": f"Spielberg · minimum-time line, colour = speed · {ln['lap']:.2f} s",
               "zh": f"Spielberg 赛道 · 最短时间赛车线，颜色 = 速度 · {ln['lap']:.2f} s"}
    for lang, (t1, t2, t3) in texts.items():
        cx, chip_svg = 60, []
        for label, w in chips[lang]:
            chip_svg.append(f'<rect x="{cx}" y="270" width="{w}" height="30" rx="15" fill="#21262d"/>'
                            f'<text x="{cx + w / 2:.0f}" y="290" text-anchor="middle">{label}</text>')
            cx += w + 10
        svg = f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {Wb} {Hb}" width="{Wb}" height="{Hb}" role="img" aria-label="F1-Race: {t1}">
<defs>
  <linearGradient id="bg" x1="0" y1="0" x2="1" y2="1">
    <stop offset="0" stop-color="#0d1117"/><stop offset="1" stop-color="#161b22"/>
  </linearGradient>
  <filter id="glow" x="-200%" y="-200%" width="500%" height="500%">
    <feGaussianBlur stdDeviation="5" result="b"/>
    <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge>
  </filter>
</defs>
<rect width="{Wb}" height="{Hb}" rx="18" fill="url(#bg)"/>
<g font-family="system-ui,-apple-system,'Segoe UI','PingFang SC','Microsoft YaHei','Noto Sans CJK SC',sans-serif">
  <text x="56" y="118" fill="#f0efec" font-size="78" font-weight="800" letter-spacing="-1">F1-Race</text>
  <text x="60" y="170" fill="#f0efec" font-size="27" font-weight="600">{t1}</text>
  <text x="60" y="214" fill="#c3c2b7" font-size="17">{t2}</text>
  <text x="60" y="240" fill="#c3c2b7" font-size="17">{t3}</text>
  <g font-size="14" fill="#c3c2b7">{"".join(chip_svg)}</g>
  <text x="{box[0] + box[2]:.0f}" y="{Hb - 14}" fill="#6e7681" font-size="12" text-anchor="end">{caption[lang]}</text>
</g>
<g stroke-width="11" stroke="#30363d" stroke-linecap="round" fill="none"><path d="{d_path}"/></g>
<g stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round" fill="none">{"".join(segs)}</g>
<circle r="6.5" fill="#f0efec" filter="url(#glow)">
  <animateMotion dur="{dur:.2f}s" repeatCount="indefinite" calcMode="linear" keyPoints="{kp}" keyTimes="{kt}" path="{d_path}"/>
</circle>
</svg>
'''
        (OUT / f"banner_{lang}.svg").write_text(svg)
    print(f"banner: dot lap {dur:.1f} s, {(OUT / 'banner_en.svg').stat().st_size / 1e3:.0f} kB")


# --------------------------------------------------------------------------- results chart
# (group, layer, setting, n, baseline, k_base, method, k_method); numbers from the README tables
RESULTS = [
    ("L", "L2", ("model mismatch", "模型失配"), 51, ("plan + tracking", "离线规划 + 跟踪"), 3,
     ("NMPC re-planning", "NMPC 滚动重规划"), 22),
    ("L", "L3", ("friction field, gusts, noise", "随机摩擦、阵风、噪声"), 30, ("NMPC, 5 % reserve", "NMPC，5% 裕度"), 5,
     ("stochastic MPC", "随机 MPC"), 30),
    ("L", "L4", ("learning the car", "学习车辆参数"), 25, ("NMPC, nominal", "NMPC，名义模型"), 13,
     ("NMPC, identified", "NMPC，辨识模型"), 25),
    ("L", "L5", ("residual RL", "残差强化学习"), 51, ("NMPC, nominal", "NMPC，名义模型"), 22,
     ("RL, selected run", "RL，验证集选优"), 51),
    ("L", "L6", ("30 ms steering lag", "30 ms 转向延迟"), 25, ("NMPC, nominal", "NMPC，名义模型"), 3,
     ("NMPC + lag model", "NMPC + 延迟模型"), 25),
    ("L", "L7", ("safety filter on RL", "RL 安全滤波器"), 105, ("RL policy alone", "RL 策略本身"), 102,
     ("+ NMPC filter", "+ NMPC 滤波"), 51),
    ("L", "L8", ("4 unseen tracks", "4 条新赛道"), 100, ("NMPC, nominal", "NMPC，名义模型"), 49,
     ("identify + re-plan", "辨识 + 重规划"), 100),
    ("L", "L9", ("real time, 50 ms", "实时 50 ms"), 105, ("NMPC, IPOPT", "NMPC，IPOPT"), 0,
     ("acados RTI", "acados RTI"), 79),
    ("C", "CX-A", ("MPCC", "MPCC 轮廓控制"), 25, ("NMPC", "NMPC"), 10, ("MPCC", "MPCC"), 9),
    ("C", "CX-B", ("sampling MPC", "采样 MPC"), 50, ("NMPC", "NMPC"), 12, ("MPPI", "MPPI"), 0),
    ("C", "CX-C", ("robust tube MPC", "鲁棒管道 MPC"), 50, ("NMPC", "NMPC"), 16, ("tube MPC", "管道 MPC"), 49),
    ("C", "CX-D", ("online adaptation", "在线自适应"), 50, ("NMPC", "NMPC"), 15, ("adaptive NMPC", "自适应 NMPC"), 42),
    ("C", "CX-G", ("20 ms sensor delay", "20 ms 传感器延迟"), 105, ("RTI (L9)", "RTI（L9）"), 5,
     ("delay-compensated", "延迟补偿 RTI"), 73),
]


def results_chart():
    themes = {
        "light": dict(ink="#0b0b0b", ink2="#52514e", muted="#6b6a65", grid="#e1e0d9", base="#9a9893",
                      method="#2a78d6", worse="#eb6834", band="#f3f2ee", bg="#fcfcfb", edge="#e1e0d9"),
        "dark": dict(ink="#f0efec", ink2="#c3c2b7", muted="#898781", grid="#2c2c2a", base="#6e6d68",
                     method="#3987e5", worse="#d95926", band="#161b22", bg="#0d1117", edge="#30363d"),
    }
    words = {
        "en": dict(title="Laps completed: the baseline → the method each layer studies",
                   sub="share of each layer's test laps; every row its own test set and cars; "
                       "orange = the method did worse",
                   base="baseline", method="method studied", worse="method, worse",
                   gL="Layers L2 – L9", gC="Control expansion (phases with a benchmark)", x="laps completed"),
        "zh": dict(title="完赛率：基线 → 该层研究的方法",
                   sub="各层测试圈中完成的比例；每一行使用各自的测试集与车辆；橙色 = 方法更差",
                   base="基线", method="研究的方法", worse="方法（更差）",
                   gL="研究层 L2 – L9", gC="控制扩展（有基准测试的阶段）", x="完赛比例"),
    }
    W = 1000
    x_lab, x_set = 24, 92
    x0, x1 = 400, 820
    x_val = 836
    row_h = 40
    top = 108
    for lang, wd in words.items():
        for th, c in themes.items():
            li = 0 if lang == "en" else 1
            y = top
            body = []
            for gi, g in enumerate(("L", "C")):
                rows = [r for r in RESULTS if r[0] == g]
                body.append(f'<text x="{x_lab}" y="{y + 4}" fill="{c["ink"]}" font-size="14" font-weight="700">'
                            f'{wd["gL"] if g == "L" else wd["gC"]}</text>')
                y += 16
                for r in rows:
                    _, layer, setting, n, bname, kb, mname, km = r
                    yc = y + row_h / 2
                    fb, fm = kb / n, km / n
                    xb, xm = x0 + fb * (x1 - x0), x0 + fm * (x1 - x0)
                    col = c["method"] if km >= kb else c["worse"]
                    body.append(f'<rect x="{x_lab - 8}" y="{y + 2}" width="{W - 2 * x_lab + 16}" height="{row_h - 4}" '
                                f'rx="6" fill="{c["band"]}" opacity="{0.0 if (RESULTS.index(r) % 2) else 1}"/>')
                    body.append(f'<text x="{x_lab}" y="{yc - 2}" fill="{c["ink"]}" font-size="13.5" '
                                f'font-weight="700">{layer}</text>')
                    body.append(f'<text x="{x_set}" y="{yc - 3}" fill="{c["ink"]}" font-size="13">{setting[li]}</text>')
                    body.append(f'<text x="{x_set}" y="{yc + 13}" fill="{c["muted"]}" font-size="11.5">'
                                f'{bname[li]} → {mname[li]}</text>')
                    body.append(f'<line x1="{min(xb, xm):.1f}" y1="{yc}" x2="{max(xb, xm):.1f}" y2="{yc}" '
                                f'stroke="{col}" stroke-width="3" stroke-linecap="round" opacity="0.45"/>')
                    body.append(f'<circle cx="{xb:.1f}" cy="{yc}" r="6" fill="{c["base"]}"/>')
                    body.append(f'<circle cx="{xm:.1f}" cy="{yc}" r="7" fill="{col}"/>')
                    body.append(f'<text x="{x_val}" y="{yc + 5}" fill="{c["ink"]}" font-size="13" '
                                f'font-variant-numeric="tabular-nums">{kb} → <tspan font-weight="700">{km}</tspan>'
                                f'<tspan fill="{c["muted"]}"> / {n}</tspan></text>')
                    body.append(f'<title>{layer}: {bname[li]} {kb}/{n} → {mname[li]} {km}/{n}</title>')
                    y += row_h
                y += 14
            # axis
            ax_y = y + 4
            grid = []
            for p in (0, 25, 50, 75, 100):
                gx = x0 + p / 100 * (x1 - x0)
                grid.append(f'<line x1="{gx:.1f}" y1="{top + 10}" x2="{gx:.1f}" y2="{ax_y - 8}" stroke="{c["grid"]}" '
                            f'stroke-width="1"/>')
                grid.append(f'<text x="{gx:.1f}" y="{ax_y + 8}" fill="{c["muted"]}" font-size="12" '
                            f'text-anchor="middle">{p} %</text>')
            H = ax_y + 30
            legend_y = 82
            leg = (f'<circle cx="{x0 + 6}" cy="{legend_y - 4}" r="6" fill="{c["base"]}"/>'
                   f'<text x="{x0 + 18}" y="{legend_y}" fill="{c["ink2"]}" font-size="13">{wd["base"]}</text>'
                   f'<circle cx="{x0 + 126}" cy="{legend_y - 4}" r="7" fill="{c["method"]}"/>'
                   f'<text x="{x0 + 139}" y="{legend_y}" fill="{c["ink2"]}" font-size="13">{wd["method"]}</text>'
                   f'<circle cx="{x0 + 286}" cy="{legend_y - 4}" r="7" fill="{c["worse"]}"/>'
                   f'<text x="{x0 + 299}" y="{legend_y}" fill="{c["ink2"]}" font-size="13">{wd["worse"]}</text>')
            svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
                   f'role="img" aria-label="{wd["title"]}">'
                   f'<g font-family="system-ui,-apple-system,\'Segoe UI\',\'PingFang SC\',\'Microsoft YaHei\','
                   f'\'Noto Sans CJK SC\',sans-serif">'
                   f'<rect x="0.5" y="0.5" width="{W - 1}" height="{H - 1}" rx="12" fill="{c["bg"]}" '
                   f'stroke="{c["edge"]}"/>'
                   f'<text x="{x_lab}" y="34" fill="{c["ink"]}" font-size="19" font-weight="700">{wd["title"]}</text>'
                   f'<text x="{x_lab}" y="58" fill="{c["ink2"]}" font-size="13">{wd["sub"]}</text>'
                   f'{leg}{"".join(grid)}{"".join(body)}</g></svg>\n')
            (OUT / f"results_{lang}_{th}.svg").write_text(svg)
    print("results charts written")


# --------------------------------------------------------------------------- who wins where
# completed laps per controller (columns) and test condition (rows); None = not run in that layer.
# Every number is from the README tables of the layer named in the row.
WIN_COLS = [("tracker", ("L1 tracker", "L1 跟踪器"), (("plan +", "tracking"), ("规划 +", "跟踪"))),
            ("nmpc", ("NMPC", "NMPC"), (("nominal", "model"), ("名义", "模型"))),
            ("smpc", ("Stochastic MPC", "随机 MPC"), (("calibrated", "back-offs"), ("标定", "回退量"))),
            ("ident", ("NMPC", "NMPC"), (("identified", "parameters"), ("辨识", "参数"))),
            ("lag", ("NMPC", "NMPC"), (("identified", "lag model"), ("辨识", "延迟模型"))),
            ("rl", ("Residual RL", "残差 RL"), (("selected on", "validation"), ("验证集", "选优")))]
WIN_ROWS = [
    (("Car differs from the model", "车辆与模型不符"), [
        (("L2", "51 cars", "51 辆车"), 51, dict(tracker=3, nmpc=22, rl=51)),
        (("L4", "25 cars", "25 辆车"), 25, dict(nmpc=13, ident=25))]),
    (("Random friction, gusts, noise", "随机摩擦、阵风、噪声"), [
        (("L3", "30 laps", "30 圈"), 30, dict(tracker=0, nmpc=5, smpc=30, rl=30))]),
    (("Steering lag the models lack", "模型中没有的转向延迟"), [
        (("L6", "τ = 10 ms", "τ = 10 ms"), 25, dict(nmpc=12, smpc=22, ident=22, lag=25, rl=25)),
        (("L6", "τ = 30 ms", "τ = 30 ms"), 25, dict(nmpc=3, smpc=22, ident=0, lag=25, rl=24)),
        (("L6", "τ = 60 ms", "τ = 60 ms"), 25, dict(nmpc=0, smpc=9, ident=0, lag=24, rl=23))]),
    (("New tracks, wet grip", "新赛道、湿滑路面"), [
        (("L8", "4 new tracks", "4 条新赛道"), 100, dict(tracker=4, nmpc=49, smpc=81, lag=100, rl=0)),
        (("L8", "wet, grip × 0.85", "湿地，抓地 × 0.85"), 25, dict(tracker=0, nmpc=0, smpc=0, lag=25, rl=8)),
        (("L8", "wet, grip × 0.75", "湿地，抓地 × 0.75"), 25, dict(tracker=0, nmpc=0, smpc=0, lag=25, rl=0))]),
    (("50 ms budget on one core", "单核 50 ms 实时预算"), [
        (("L9", "real time", "实时"), 105, dict(lag=79, rl=98))]),
]


def _mix(c0, c1, a):
    h = lambda c: [int(c[i:i + 2], 16) for i in (1, 3, 5)]
    x, y = h(c0), h(c1)
    return "#" + "".join(f"{int(round(x[i] + (y[i] - x[i]) * a)):02x}" for i in range(3))


def _ramp(stops, a):
    a = min(max(a, 0.0), 1.0) * (len(stops) - 1)
    i = min(int(a), len(stops) - 2)
    return _mix(stops[i], stops[i + 1], a - i)


def winner_heatmap():
    """Who wins where: one row per test condition, one column per controller, the cell
    coloured by the share of laps completed (sequential blue), the best of each row ringed."""
    themes = {
        "light": dict(bg="#fcfcfb", edge="#e1e0d9", ink="#0b0b0b", ink2="#52514e", muted="#6b6a65",
                      empty="#f0efec", ring="#0b0b0b",
                      ramp=["#e8f1fc", "#b7d3f6", "#6da7ec", "#2a78d6", "#184f95", "#0d366b"]),
        "dark": dict(bg="#0d1117", edge="#30363d", ink="#f0efec", ink2="#c3c2b7", muted="#898781",
                     empty="#161b22", ring="#f0efec",
                     ramp=["#14243a", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#b7d3f6"]),
    }
    words = {
        "en": dict(title="Who wins where? Laps completed, per controller and test condition",
                   sub="each row is one test set of the layer named; a ring marks the most reliable controller "
                       "of the row; – = not run in that layer",
                   rt="RTI; IPOPT 0", low="0 %", high="100 % of the row's laps"),
        "zh": dict(title="谁在什么情形下胜出？各控制器在各测试条件下完成的圈数",
                   sub="每一行是对应层的一个测试集；圆环标出该行最可靠的控制器；– = 该层未测试",
                   rt="RTI；IPOPT 为 0", low="0 %", high="该行测试圈的 100 %"),
    }
    W = 1000
    x_cols = 318
    cw, ch, gap = 106, 38, 6
    top = 160
    for lang, wd in words.items():
        li = 0 if lang == "en" else 1
        for th, c in themes.items():
            out = []
            # column headers
            for k, (_, name, sub) in enumerate(WIN_COLS):
                cx = x_cols + k * (cw + gap) + cw / 2
                out.append(f'<text x="{cx:.0f}" y="{top - 44}" fill="{c["ink"]}" font-size="13.5" font-weight="700" '
                           f'text-anchor="middle">{name[li]}</text>')
                for r, part in enumerate(sub[li]):
                    out.append(f'<text x="{cx:.0f}" y="{top - 27 + 14 * r}" fill="{c["muted"]}" font-size="11.5" '
                               f'text-anchor="middle">{part}</text>')
            y = top
            for gname, rows in WIN_ROWS:
                out.append(f'<text x="24" y="{y + 15}" fill="{c["ink"]}" font-size="13.5" font-weight="700">'
                           f'{gname[li]}</text>')
                y += 24
                for (layer, en, zh), n, vals in rows:
                    best = max(vals.values())
                    out.append(f'<text x="36" y="{y + ch / 2 + 5:.0f}" fill="{c["ink"]}" font-size="13" '
                               f'font-weight="700">{layer}</text>')
                    out.append(f'<text x="70" y="{y + ch / 2 + 5:.0f}" fill="{c["ink2"]}" font-size="13">'
                               f'{en if lang == "en" else zh}</text>')
                    for k, (key, _, _) in enumerate(WIN_COLS):
                        x = x_cols + k * (cw + gap)
                        v = vals.get(key)
                        if v is None:
                            out.append(f'<rect x="{x}" y="{y}" width="{cw}" height="{ch}" rx="6" fill="{c["empty"]}"/>')
                            out.append(f'<text x="{x + cw / 2:.0f}" y="{y + ch / 2 + 5:.0f}" fill="{c["muted"]}" '
                                       f'font-size="13" text-anchor="middle">–</text>')
                            continue
                        a = v / n
                        fill = _ramp(c["ramp"], a)
                        lum = sum(int(fill[i:i + 2], 16) * w for i, w in ((1, .299), (3, .587), (5, .114))) / 255
                        txt = "#0b0b0b" if lum > 0.55 else "#ffffff"
                        out.append(f'<rect x="{x}" y="{y}" width="{cw}" height="{ch}" rx="6" fill="{fill}"/>')
                        if v == best and best > 0:
                            out.append(f'<rect x="{x - 2.5}" y="{y - 2.5}" width="{cw + 5}" height="{ch + 5}" rx="8" '
                                       f'fill="none" stroke="{c["ring"]}" stroke-width="2"/>')
                        label = f'{v}<tspan font-size="11" font-weight="400" opacity="0.8"> / {n}</tspan>'
                        out.append(f'<text x="{x + cw / 2:.0f}" y="{y + ch / 2 + 5:.0f}" fill="{txt}" font-size="15" '
                                   f'font-weight="700" text-anchor="middle">{label}</text>')
                        if layer == "L9" and key == "lag":
                            out.append(f'<text x="{x + cw / 2:.0f}" y="{y + ch + 14:.0f}" fill="{c["muted"]}" '
                                       f'font-size="11" text-anchor="middle">{wd["rt"]}</text>')
                        out.append(f'<title>{layer} {en}: {WIN_COLS[k][1][0]} ({" ".join(WIN_COLS[k][2][0])}) '
                                   f'{v}/{n}</title>')
                    y += ch + gap
                y += 10
            # legend: the ramp
            ly = y + 18
            lx0, lx1 = x_cols, x_cols + 300
            stops = "".join(f'<stop offset="{i / (len(c["ramp"]) - 1):.2f}" stop-color="{col}"/>'
                            for i, col in enumerate(c["ramp"]))
            out.append(f'<defs><linearGradient id="g{lang}{th}">{stops}</linearGradient></defs>')
            out.append(f'<rect x="{lx0}" y="{ly}" width="{lx1 - lx0}" height="10" rx="5" fill="url(#g{lang}{th})"/>')
            out.append(f'<text x="{lx0}" y="{ly + 27}" fill="{c["muted"]}" font-size="12">{wd["low"]}</text>')
            out.append(f'<text x="{lx1}" y="{ly + 27}" fill="{c["muted"]}" font-size="12" text-anchor="end">'
                       f'{wd["high"]}</text>')
            Hh = ly + 46
            svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {Hh}" width="{W}" height="{Hh}" '
                   f'role="img" aria-label="{wd["title"]}">'
                   f'<g font-family="system-ui,-apple-system,\'Segoe UI\',\'PingFang SC\',\'Microsoft YaHei\','
                   f'\'Noto Sans CJK SC\',sans-serif">'
                   f'<rect x="0.5" y="0.5" width="{W - 1}" height="{Hh - 1}" rx="12" fill="{c["bg"]}" '
                   f'stroke="{c["edge"]}"/>'
                   f'<text x="24" y="38" fill="{c["ink"]}" font-size="19" font-weight="700">{wd["title"]}</text>'
                   f'<text x="24" y="62" fill="{c["ink2"]}" font-size="13">{wd["sub"]}</text>'
                   f'{"".join(out)}</g></svg>\n')
            (OUT / f"winners_{lang}_{th}.svg").write_text(svg)
    print("winner heatmaps written")


# --------------------------------------------------------------------------- READMEs
README_BLOCKS = {"map.geojson": "geojson", "speed_landscape_inline.stl": "stl"}


def update_readmes():
    """Copy the inline snippets into README.md and README.zh-CN.md, between the markers
    <!-- BEGIN name --> and <!-- END name -->."""
    for readme in (ROOT / "README.md", ROOT / "README.zh-CN.md"):
        if not readme.exists():
            continue
        text = readme.read_text()
        for name, lang in README_BLOCKS.items():
            snip = OUT / "snippets" / name
            if readme.name == "README.zh-CN.md":        # a translated copy when there is one
                zh = snip.with_name(snip.stem + ".zh" + snip.suffix)
                snip = zh if zh.exists() else snip
            begin, end = f"<!-- BEGIN {name} -->", f"<!-- END {name} -->"
            if not snip.exists() or begin not in text:
                continue
            i, j = text.index(begin) + len(begin), text.index(end)
            text = text[:i] + f"\n```{lang}\n{snip.read_text().rstrip()}\n```\n" + text[j:]
        readme.write_text(text)
        print(f"{readme.name}: inline blocks updated")


# --------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("only", nargs="*", help="ghost spin map stl banner chart winners (default: all)")
    ap.add_argument("--no-l1", action="store_true", help="skip the spin animation (needs the L1 plans)")
    ap.add_argument("--cache", type=Path, default=None,
                    help="pickle of the L0 lines (written once, then reused), e.g. results/media/l0_lines.pkl")
    args = ap.parse_args(argv)
    todo = set(args.only or ["ghost", "spin", "map", "stl", "banner", "chart", "winners"])
    if args.no_l1:
        todo.discard("spin")
    OUT.mkdir(parents=True, exist_ok=True)
    if {"ghost", "spin"} & todo and shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is needed for the animations")
    if "chart" in todo:
        results_chart()
    if "winners" in todo:
        winner_heatmap()
    if todo - {"chart", "winners"}:
        tr, veh, lines = l0_lines(args.cache)
        ocp = lines[0]
        if "banner" in todo:
            banner(tr, ocp)
        if "map" in todo:
            geojson(tr, ocp)
        if "stl" in todo:
            stl(ocp, OUT / "speed_landscape.stl", tol=1.5)
            stl(ocp, OUT / "snippets" / "speed_landscape_inline.stl", tol=4.0, road=False)
        if "ghost" in todo:
            ghost_race(tr, lines)
        if "spin" in todo:
            veh1, r_kin, r_dyn = l1_runs(tr)
            spin(tr, veh1, r_kin, r_dyn)
        if {"map", "stl"} & todo:
            update_readmes()


if __name__ == "__main__":
    main()
