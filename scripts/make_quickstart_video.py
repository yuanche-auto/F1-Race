"""Record and render the quick-start video tutorial of the README.

    python scripts/make_quickstart_video.py record    # ~5 min: a fresh clone, venv, install, L0, tests
    python scripts/make_quickstart_video.py render    # docs/media/quickstart_{en,zh}.gif / .mp4

``record`` runs the README's five quick-start steps for real, in a temporary directory, and
writes docs/media/quickstart.cast (asciinema v2: the output with its timing, the typed commands,
and a marker at the start of every step; `asciinema play docs/media/quickstart.cast` replays it).
``render`` turns the recording into the video: a terminal window with the commands typed out,
long steps fast-forwarded with their real duration on screen, a caption per step in English or
Chinese, the L0 figure after step 4 and an end card. ffmpeg must be on PATH; the Chinese
captions need a CJK font (Noto Sans CJK, WenQuanYi, Microsoft YaHei or PingFang).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / "docs" / "media"
CAST = MEDIA / "quickstart.cast"
REPO_URL = "https://github.com/yuanche-auto/F1-Race.git"

STEPS = [
    dict(key="clone", cmds=[f"git clone {REPO_URL}", "cd F1-Race"],
         en=("Get the code", "clone the repository"),
         zh=("获取代码", "克隆仓库")),
    dict(key="install", cmds=["python -m venv .venv && source .venv/bin/activate", 'pip install -e ".[dev]"'],
         en=("Install", "a virtual environment, then the package with its test extras"),
         zh=("安装", "创建虚拟环境，再安装本项目及测试依赖")),
    dict(key="track", cmds=["python scripts/fetch_tracks.py Spielberg"],
         en=("Download the track", "from the TUM racetrack-database: pinned commit, SHA-256 checked"),
         zh=("下载赛道", "来自 TUM racetrack-database：固定上游版本，校验 SHA-256")),
    dict(key="l0", cmds=["python scripts/run_l0.py"],
         en=("Run the first layer, L0", "racing lines, the minimum-time OCP and the closed-loop check"),
         zh=("运行第一层 L0", "赛车线、最短时间最优控制与闭环验证")),
    dict(key="test", cmds=['pytest -m "not slow"'],
         en=("Run the tests", "the fast suite (the full one: pytest)"),
         zh=("运行测试", "快速测试集（完整测试：pytest）")),
]


# --------------------------------------------------------------------------- record
def record(workdir: Path | None = None) -> None:
    """Run every step's commands in one shell state (working directory, virtual environment)
    and write the cast. Paths under the working directory are shown as ~."""
    work = Path(workdir or tempfile.mkdtemp(prefix="f1race-quickstart-")).resolve()
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PYTHONUNBUFFERED="1", PIP_DISABLE_PIP_VERSION_CHECK="1", PIP_NO_INPUT="1",
               OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", TERM="dumb")
    env.pop("VIRTUAL_ENV", None)
    cwd = work
    events = []
    t0 = time.monotonic()

    def now():
        return round(time.monotonic() - t0, 3)

    def clean(text: str) -> str:
        return text.replace(str(work), "~")

    for i, step in enumerate(STEPS):
        events.append([now(), "m", json.dumps({"step": i, "key": step["key"]})])
        for cmd in step["cmds"]:
            shown = "~" + clean(str(cwd))[1:] if str(cwd).startswith(str(work)) else str(cwd)
            events.append([now(), "o", f"{shown} $ {cmd}\r\n"])
            events.append([now(), "m", json.dumps({"cmd": cmd, "cwd": shown})])
            script = ("[ -f .venv/bin/activate ] && source .venv/bin/activate\n"
                      f"{cmd}\nstatus=$?\necho \"__PWD__$(pwd)\"\nexit $status\n")
            proc = subprocess.Popen(["bash", "-c", script], cwd=cwd, env=env, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1)
            for line in proc.stdout:
                if line.startswith("__PWD__"):
                    cwd = Path(line[len("__PWD__"):].strip())
                    continue
                events.append([now(), "o", clean(line.rstrip("\n")) + "\r\n"])
            if proc.wait() != 0:
                raise SystemExit(f"step {step['key']}: {cmd!r} failed with {proc.returncode}")
            events.append([now(), "m", json.dumps({"done": cmd})])
            print(f"[{now():7.1f} s] {cmd}")
    header = {"version": 2, "width": 104, "height": 24, "timestamp": int(time.time()),
              "title": "F1-Race quick start", "env": {"SHELL": "/bin/bash", "TERM": "xterm-256color"}}
    MEDIA.mkdir(parents=True, exist_ok=True)
    with open(CAST, "w") as fh:
        fh.write(json.dumps(header) + "\n")
        for e in events:
            fh.write(json.dumps(e, ensure_ascii=False) + "\n")
    print(f"{CAST.relative_to(ROOT)}: {len(events)} events, {events[-1][0]:.0f} s")


# --------------------------------------------------------------------------- render
W, H, FPS = 1280, 720, 30
BG, WIN, BAR = (13, 17, 23), (22, 27, 34), (33, 38, 45)
INK, INK2, MUTED = (240, 246, 252), (201, 209, 217), (139, 148, 158)
GREEN, BLUE, YELLOW, ORANGE, RED = (63, 185, 80), (88, 166, 255), (210, 153, 34), (217, 89, 38), (248, 81, 73)
ACCENT = (57, 135, 229)
COLS, ROWS = 100, 20
TYPE_CPS = 32.0


def font_path(kind: str) -> str:
    import matplotlib
    from matplotlib import font_manager

    ttf = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    if kind == "mono":
        return str(ttf / "DejaVuSansMono.ttf")
    if kind == "bold":
        return str(ttf / "DejaVuSans-Bold.ttf")
    if kind == "sans":
        return str(ttf / "DejaVuSans.ttf")
    for name in ("Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Zen Hei", "Microsoft YaHei", "PingFang SC",
                 "SimHei"):
        try:
            p = font_manager.findfont(font_manager.FontProperties(family=name), fallback_to_default=False)
            return p
        except Exception:
            continue
    for p in ("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        if Path(p).exists():
            return p
    raise SystemExit("no CJK font found for the Chinese captions")


def load_cast():
    lines = CAST.read_text().splitlines()
    return json.loads(lines[0]), [json.loads(x) for x in lines[1:]]


def build_timeline(events):
    """Scenes on the video clock: per step its commands (typed), their output (time-compressed
    when the command ran long) and pauses. Returns (scenes, total duration)."""
    steps = []
    for t, kind, data in events:
        if kind == "m":
            m = json.loads(data)
            if "step" in m:
                steps.append(dict(index=m["step"], cmds=[]))
            elif "cmd" in m:
                steps[-1]["cmds"].append(dict(cmd=m["cmd"], cwd=m["cwd"], t0=t, out=[]))
            elif "done" in m:
                steps[-1]["cmds"][-1]["t1"] = t
        elif kind == "o" and steps and steps[-1]["cmds"] and "t1" not in steps[-1]["cmds"][-1]:
            c = steps[-1]["cmds"][-1]
            if data.startswith(f"{c['cwd']} $ "):
                continue
            c["out"].append((t - c["t0"], data.rstrip("\r\n")))
    clock = 2.8                                          # title card
    scenes = []
    for st in steps:
        sc = dict(index=st["index"], start=clock, cmds=[])
        clock += 0.9                                     # caption first
        for c in st["cmds"]:
            real = c["t1"] - c["t0"]
            typing = max(0.6, len(c["cmd"]) / TYPE_CPS)
            shown = real if real < 2.0 else min(2.0 + 0.06 * real, 5.0)
            c.update(type0=clock, type1=clock + typing, out0=clock + typing + 0.25, real=real, shown=shown,
                     speed=real / shown)
            clock = c["out0"] + shown + 0.6
            sc["cmds"].append(c)
        if STEPS[st["index"]]["key"] == "l0":
            sc["figure"] = (clock, clock + 4.2)
            clock += 4.2
        sc["end"] = clock + 0.4
        clock = sc["end"]
        scenes.append(sc)
    return scenes, clock + 4.5                           # end card


def wrap(text: str) -> list[str]:
    return textwrap.wrap(text, COLS, replace_whitespace=False, drop_whitespace=False) or [""]


def style_for(text: str):
    low = text.lower()
    if "successfully installed" in low or re.search(r"\b\d+ passed\b", low):
        return GREEN
    if re.search(r"\berror\b|failed", low) and "0 failed" not in low:
        return RED
    if '"ocp_ideal"' in text or '"lap_time"' in text or '"centreline_qss"' in text or '"mincurv_qss_ideal"' in text:
        return YELLOW
    if text.startswith(("Collecting", "Downloading", "Obtaining", "Installing", "Building", "Preparing",
                        "Requirement", "Using cached", "Getting requirements", "Created wheel", "Stored in",
                        "Attempting", "Checking")):
        return MUTED
    return INK2


def l0_numbers(scenes) -> dict:
    for sc in scenes:
        if STEPS[sc["index"]]["key"] == "l0":
            text = "\n".join(t for _, t in sc["cmds"][0]["out"])
            j = json.loads(text[text.index("{"):text.rindex("}") + 1])
            return dict(ocp=j["ocp_ideal"], mc=j["mincurv_qss_ideal"], centre=j["centreline_qss"],
                        cl=j["closed_loop"]["ocp"]["lap_time"])
    return {}


TEXT = {
    "en": dict(title="Quick start", sub="from a fresh clone to the first lap in five steps",
               real="{real} of real time in {video} s", step="STEP {i}/{n}", ff="▶▶ ×{x:.0f} · {t} real",
               fig="docs/figures/L0/lines.png", call="minimum-time lap {ocp:.2f} s · closed loop {cl:.2f} s",
               done="You are set.", next="Next steps",
               nexts=["python scripts/run_l1.py      the dynamic car, ~20 min",
                      "docs/L0.md … docs/L9.md       what each layer found",
                      "python scripts/make_readme_media.py   this README's media"]),
    "zh": dict(title="快速上手", sub="从全新克隆到跑完第一圈，只需五步",
               real="真实耗时 {real}，视频 {video} 秒", step="第 {i}/{n} 步", ff="▶▶ ×{x:.0f} · 真实 {t}",
               fig="docs/figures/L0/lines.png", call="最短时间圈 {ocp:.2f} s · 闭环 {cl:.2f} s",
               done="完成！", next="下一步",
               nexts=["python scripts/run_l1.py      动力学车辆，约 20 分钟",
                      "docs/L0.md … docs/L9.md       每一层的发现",
                      "python scripts/make_readme_media.py   本 README 的媒体素材"]),
}


def mmss(sec: float) -> str:
    return f"{int(sec // 60)}:{int(sec % 60):02d}"


def render(lang: str) -> None:
    from PIL import Image, ImageDraw, ImageFilter, ImageFont

    _, events = load_cast()
    scenes, total = build_timeline(events)
    tx = TEXT[lang]
    nums = l0_numbers(scenes)
    real_total = sum(c["real"] for sc in scenes for c in sc["cmds"])
    cap_font = font_path("cjk") if lang == "zh" else font_path("bold")
    sub_font = font_path("cjk") if lang == "zh" else font_path("sans")
    F = dict(mono=ImageFont.truetype(font_path("mono"), 17), mono_b=ImageFont.truetype(font_path("mono"), 17),
             cap=ImageFont.truetype(cap_font, 34), sub=ImageFont.truetype(sub_font, 20),
             chip=ImageFont.truetype(cap_font if lang == "zh" else font_path("bold"), 16),
             small=ImageFont.truetype(sub_font, 15), big=ImageFont.truetype(font_path("bold"), 84),
             mid=ImageFont.truetype(cap_font, 36), body=ImageFont.truetype(sub_font, 22),
             ff=ImageFont.truetype(font_path("mono"), 15))
    fig_img = Image.open(ROOT / "docs" / "figures" / "L0" / "lines.png").convert("RGB")
    tw = (W - 120, H - 150)                               # terminal window size
    tx0, ty0 = 60, 128
    line_h = 23
    char_w = F["mono"].getlength("M")

    def rounded(draw, box, r, fill, outline=None):
        draw.rounded_rectangle(box, r, fill=fill, outline=outline)

    def base():
        im = Image.new("RGB", (W, H), BG)
        return im, ImageDraw.Draw(im)

    def title_card(t):
        im, d = base()
        a = min(1.0, t / 0.6)
        col = tuple(int(BG[k] + (INK[k] - BG[k]) * a) for k in range(3))
        d.text((W / 2, 250), "F1-Race", font=F["big"], fill=col, anchor="mm")
        d.text((W / 2, 340), tx["title"], font=F["mid"], fill=col, anchor="mm")
        d.text((W / 2, 395), tx["sub"], font=F["body"], fill=tuple(int(BG[k] + (INK2[k] - BG[k]) * a)
                                                                   for k in range(3)), anchor="mm")
        d.text((W / 2, 450), tx["real"].format(real=mmss(real_total), video=round(total)), font=F["small"],
               fill=MUTED, anchor="mm")
        for k in range(len(STEPS)):
            x = W / 2 + (k - 2) * 46
            d.ellipse((x - 6, 500 - 6, x + 6, 500 + 6), fill=ACCENT if a >= 1 else BAR)
        return im

    def end_card(t):
        im, d = base()
        d.text((W / 2, 200), "✓", font=F["big"], fill=GREEN, anchor="mm")
        d.text((W / 2, 290), tx["done"], font=F["mid"], fill=INK, anchor="mm")
        d.text((W / 2, 350), tx["next"], font=F["sub"], fill=MUTED, anchor="mm")
        box_w = 760
        rounded(d, (W / 2 - box_w / 2, 380, W / 2 + box_w / 2, 380 + 44 * len(tx["nexts"]) + 24), 14, WIN)
        for k, line in enumerate(tx["nexts"]):
            cmd, _, desc = line.partition("   ")
            y = 404 + 44 * k
            d.text((W / 2 - box_w / 2 + 28, y), cmd.strip(), font=F["mono"], fill=INK)
            d.text((W / 2 + box_w / 2 - 28, y), desc.strip(), font=F["small"] if lang == "en" else F["small"],
                   fill=MUTED, anchor="ra")
        d.text((W / 2, 640), "github.com/yuanche-auto/F1-Race", font=F["small"], fill=MUTED, anchor="mm")
        return im

    def terminal(d, sc, t):
        rounded(d, (tx0, ty0, tx0 + tw[0], ty0 + tw[1]), 14, WIN)
        d.rounded_rectangle((tx0, ty0, tx0 + tw[0], ty0 + 36), 14, fill=BAR)
        d.rectangle((tx0, ty0 + 22, tx0 + tw[0], ty0 + 36), fill=BAR)
        for k, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
            d.ellipse((tx0 + 18 + 22 * k, ty0 + 12, tx0 + 30 + 22 * k, ty0 + 24), fill=c)
        d.text((tx0 + tw[0] / 2, ty0 + 18), "bash — F1-Race", font=F["small"], fill=MUTED, anchor="mm")
        rows = []                                         # (segments) ; segment = (text, colour)
        cursor = None
        badge = None
        for c in sc["cmds"]:
            if t < c["type0"]:
                break
            n = int((t - c["type0"]) * TYPE_CPS) if t < c["type1"] else len(c["cmd"])
            prompt = [(c["cwd"], BLUE), (" $ ", GREEN), (c["cmd"][:n], INK)]
            rows.append(prompt)
            if t < c["out0"]:
                cursor = len(rows) - 1
                break
            tt = (t - c["out0"]) * c["speed"]
            for rt, text in c["out"]:
                if rt > tt:
                    break
                for piece in wrap(text):
                    rows.append([(piece, style_for(text))])
            if t < c["out0"] + c["shown"]:
                if c["speed"] > 1.5:
                    badge = tx["ff"].format(x=c["speed"], t=mmss(min(tt, c["real"])))
                cursor = None
                break
        visible = rows[-ROWS:]
        y = ty0 + 52
        for k, segs in enumerate(visible):
            x = tx0 + 22
            for text, col in segs:
                d.text((x, y), text, font=F["mono"], fill=col)
                x += F["mono"].getlength(text)
            if cursor is not None and k == len(visible) - 1 and int(t * 2.5) % 2 == 0:
                d.rectangle((x + 1, y + 1, x + char_w, y + line_h - 4), fill=INK)
            y += line_h
        if badge:
            bw = F["ff"].getlength(badge) + 24
            rounded(d, (tx0 + tw[0] - bw - 14, ty0 + 48, tx0 + tw[0] - 14, ty0 + 76), 8, (56, 36, 20))
            d.text((tx0 + tw[0] - bw / 2 - 14, ty0 + 62), badge, font=F["ff"], fill=(255, 166, 87), anchor="mm")

    def caption(d, sc, t):
        i, n = sc["index"] + 1, len(STEPS)
        title, sub = STEPS[sc["index"]][lang]
        chip = tx["step"].format(i=i, n=n)
        cw = F["chip"].getlength(chip) + 26
        rounded(d, (60, 30, 60 + cw, 60), 15, ACCENT)
        d.text((60 + cw / 2, 45), chip, font=F["chip"], fill=(255, 255, 255), anchor="mm")
        d.text((60 + cw + 16, 45), title, font=F["cap"], fill=INK, anchor="lm")
        d.text((60, 90), sub, font=F["sub"], fill=MUTED, anchor="lm")
        for k in range(n):                                # step progress
            x = W - 60 - (n - k) * 38
            d.rounded_rectangle((x, 40, x + 30, 48), 4, fill=ACCENT if k < i else BAR)

    def figure_overlay(im, sc, t):
        f0, f1 = sc["figure"]
        a = min(1.0, (t - f0) / 0.35)
        box_w = int(980 * (0.92 + 0.08 * a))
        img = fig_img.copy()
        img.thumbnail((box_w - 32, 470))
        bw, bh = img.size[0] + 32, img.size[1] + 92
        x0, y0 = (W - bw) // 2, ty0 + (tw[1] - bh) // 2 + int((1 - a) * 30)
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        dl = ImageDraw.Draw(layer)
        dl.rounded_rectangle((x0 + 6, y0 + 10, x0 + bw + 6, y0 + bh + 10), 16, fill=(0, 0, 0, int(140 * a)))
        layer = layer.filter(ImageFilter.GaussianBlur(10))
        im.paste(Image.new("RGB", (W, H), (0, 0, 0)), (0, 0), layer)
        d = ImageDraw.Draw(im)
        d.rounded_rectangle((x0, y0, x0 + bw, y0 + bh), 14, fill=(250, 250, 250))
        d.text((x0 + 18, y0 + 18), tx["fig"], font=F["mono"], fill=(80, 80, 80))
        im.paste(img, (x0 + 16, y0 + 48))
        if nums:
            call = tx["call"].format(**nums)
            d.text((x0 + bw / 2, y0 + bh - 24), call, font=F["sub"], fill=(30, 30, 30), anchor="mm")
        if a < 1:
            im = Image.blend(im, im, 1.0)
        return im

    mp4 = MEDIA / f"quickstart_{lang}.mp4"
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r",
         str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "slow",
         "-movflags", "+faststart", str(mp4)], stdin=subprocess.PIPE)
    n_frames = int(math.ceil(total * FPS))
    for f in range(n_frames):
        t = f / FPS
        if t < scenes[0]["start"]:
            im = title_card(t)
        elif t >= scenes[-1]["end"]:
            im = end_card(t - scenes[-1]["end"])
        else:
            sc = next(s for s in scenes if s["start"] <= t < s["end"])
            im, d = base()
            caption(d, sc, t)
            terminal(d, sc, t)
            if "figure" in sc and sc["figure"][0] <= t < sc["figure"][1] + 0.4:
                im = figure_overlay(im, sc, t)
        proc.stdin.write(im.tobytes())
    proc.stdin.close()
    proc.wait()
    gif = MEDIA / f"quickstart_{lang}.gif"
    vf = ("fps=12,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64:stats_mode=diff[p];"
          "[b][p]paletteuse=dither=none:diff_mode=rectangle")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4), "-vf", vf, "-loop", "0", str(gif)],
                   check=True)
    print(f"{mp4.name}: {mp4.stat().st_size / 1e6:.2f} MB, {total:.1f} s; {gif.name}: {gif.stat().st_size / 1e6:.2f} MB")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["record", "render", "all"])
    ap.add_argument("--workdir", type=Path, default=None, help="where record clones the repository (default: temp)")
    ap.add_argument("--lang", nargs="*", default=["en", "zh"])
    args = ap.parse_args(argv)
    if args.what in ("record", "all"):
        record(args.workdir)
    if args.what in ("render", "all"):
        for lang in args.lang:
            render(lang)


if __name__ == "__main__":
    sys.exit(main())
