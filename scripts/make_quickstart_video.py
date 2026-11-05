"""Record and render the quick-start video of the README: your first race.

    python scripts/make_quickstart_video.py record    # ~3 min: a fresh clone, install, three races
    python scripts/make_quickstart_video.py render    # docs/media/quickstart_{en,zh}.gif / .mp4
    python scripts/make_quickstart_video.py all

``record`` runs the README's quick start for real in a temporary directory and writes
docs/media/quickstart.cast (asciinema v2: the output with its timing, the typed commands and a
marker per step; `asciinema play docs/media/quickstart.cast` replays it). After the steps that
show the replay it also films the race's HTML replay in a headless browser (Node with the
`playwright` package and its Chromium), frames in results/media/quickstart/ (not committed).
``--ref`` checks out a branch or commit right after the clone (hidden in the video), to record
a change before it is merged; the commands shown are the README's.

``render`` turns the recording into the video: a terminal window with the commands typed out,
long steps fast-forwarded with their real duration on screen, a caption per step in English or
Chinese, the browser replays and an end card. ffmpeg must be on PATH; the Chinese captions need
a CJK font (Noto Sans CJK, WenQuanYi, Microsoft YaHei or PingFang).
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
FRAMES = ROOT / "results" / "media" / "quickstart"
REPO_URL = "https://github.com/yuanche-auto/F1-Race.git"
CLIP_FPS = 15
VIEWPORT = (1200, 800)                                  # the clip shows the map and the timing tower

STEPS = [
    dict(key="setup",
         cmds=[f"git clone {REPO_URL} && cd F1-Race",
               "python -m venv .venv && source .venv/bin/activate && pip install -e ."],
         en=("Set up, once", "clone the repository, a virtual environment and the package"),
         zh=("准备（只需一次）", "克隆仓库，创建虚拟环境并安装")),
    dict(key="race", cmds=["python scripts/race.py"], hold=3.4,
         en=("Start your first race", "three drivers, one flying lap at Spielberg; the track downloads on first use"),
         zh=("开始你的第一场比赛", "三位车手在 Spielberg 跑一个飞驰圈；首次运行会自动下载赛道")),
    dict(key="replay", cmds=["python scripts/race.py --open"], hold=0.8,
         browser=dict(html="results/race/Spielberg_grip0.97.html", speed=14.0, follow=False, until="end", tail=1.8),
         en=("Watch the replay", "the plans are cached, so a rerun takes seconds; --open shows the replay"),
         zh=("观看回放", "规划已缓存，重跑只需几秒；--open 直接打开回放")),
    dict(key="limit", cmds=["python scripts/race.py --grip 1.0"], hold=3.2,
         browser=dict(html="results/race/Spielberg_grip1.00.html", speed=2.0, follow=True, until="dnf", before=7.0,
                      tail=1.6),
         en=("Push to the limit", "--grip 1.0: the plans use all of the grip. Who still finishes?"),
         zh=("挑战极限", "--grip 1.0：规划用尽全部抓地力，谁还能完赛？")),
]

CAPTURE_JS = r"""
const { chromium } = require('playwright');
const fs = require('fs');
(async () => {
  const [html, out, fps, speed, follow, until, before, w, h] = process.argv.slice(2);
  const b = await chromium.launch();
  const p = await b.newPage({ viewport: { width: +w, height: +h }, deviceScaleFactor: 1 });
  await p.goto('file://' + html);
  const info = await p.evaluate(() => { replay.pause(); return { end: replay.end, cars: replay.cars }; });
  let t0 = 0, t1 = info.end + 0.3;
  if (until === 'dnf') {
    const dnf = info.cars.filter(c => !c.finished);
    if (!dnf.length) throw new Error('no car left the track');
    t1 = dnf[0].tEnd + 1.5; t0 = Math.max(0, dnf[0].tEnd - +before);
  }
  if (follow === '1') await p.evaluate(() => replay.setFollow(true));
  const n = Math.ceil((t1 - t0) / +speed * +fps);
  const box = await p.locator('main').boundingBox();
  fs.mkdirSync(out, { recursive: true });
  for (let i = 0; i <= n; i++) {
    await p.evaluate(t => replay.seek(t), t0 + i * +speed / +fps);
    await p.screenshot({ path: `${out}/f_${String(i).padStart(4, '0')}.png`, clip: box });
  }
  console.log(JSON.stringify({ frames: n + 1, t0, t1 }));
  await b.close();
})().catch(e => { console.error(e); process.exit(1); });
"""


# --------------------------------------------------------------------------- record
def capture(html: Path, key: str, spec: dict) -> dict:
    out = FRAMES / key
    if out.exists():
        for f in out.glob("f_*.png"):
            f.unlink()
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as fh:
        fh.write(CAPTURE_JS)
        js = fh.name
    try:
        res = subprocess.run(["node", js, str(html), str(out), str(CLIP_FPS), str(spec["speed"]),
                              "1" if spec.get("follow") else "0", spec["until"], str(spec.get("before", 0)),
                              str(VIEWPORT[0]), str(VIEWPORT[1])], capture_output=True, text=True, check=True)
    finally:
        os.unlink(js)
    return json.loads(res.stdout.strip().splitlines()[-1])


def record(workdir: Path | None = None, ref: str | None = None) -> None:
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
            shown = clean(str(cwd))
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
            if ref and cmd.startswith("git clone"):
                subprocess.run(["git", "checkout", "-q", ref], cwd=cwd, check=True)
                print(f"           (checked out {ref})")
        if "browser" in step:
            spec = step["browser"]
            got = capture(cwd / spec["html"], step["key"], spec)
            events.append([now(), "m", json.dumps({"browser": step["key"], "frames": got["frames"],
                                                   "url": f"{clean(str(cwd))}/{spec['html']}"})])
            print(f"           filmed the replay: {got['frames']} frames")
    header = {"version": 2, "width": 104, "height": 24, "timestamp": int(time.time()),
              "title": "F1-Race quick start: your first race",
              "env": {"SHELL": "/bin/bash", "TERM": "xterm-256color"}}
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
GREEN, BLUE, YELLOW, RED = (63, 185, 80), (88, 166, 255), (210, 153, 34), (248, 81, 73)
ACCENT = (57, 135, 229)
COLS, ROWS = 90, 18
TYPE_CPS = 32.0


def font_path(kind: str) -> str:
    import matplotlib
    from matplotlib import font_manager

    ttf = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    if kind in ("mono", "bold", "sans"):
        return str(ttf / {"mono": "DejaVuSansMono.ttf", "bold": "DejaVuSans-Bold.ttf", "sans": "DejaVuSans.ttf"}[kind])
    for name in ("Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Zen Hei", "Microsoft YaHei", "PingFang SC",
                 "SimHei"):
        try:
            return font_manager.findfont(font_manager.FontProperties(family=name), fallback_to_default=False)
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
    when the command ran long), a hold to read the result, and the browser replay if the step
    has one. Returns (scenes, total duration)."""
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
            elif "browser" in m:
                steps[-1]["browser"] = m
        elif kind == "o" and steps and steps[-1]["cmds"] and "t1" not in steps[-1]["cmds"][-1]:
            c = steps[-1]["cmds"][-1]
            if data.startswith(f"{c['cwd']} $ "):
                continue
            c["out"].append((t - c["t0"], data.rstrip("\r\n")))
    clock = 2.8                                          # title card
    scenes = []
    for st in steps:
        spec = STEPS[st["index"]]
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
        clock += spec.get("hold", 0.0)
        b = st.get("browser")
        if b and (FRAMES / spec["key"]).exists():
            dur = b["frames"] / CLIP_FPS + spec["browser"].get("tail", 1.0)
            sc["browser"] = dict(b, t0=clock, t1=clock + dur, dir=FRAMES / spec["key"])
            clock += dur
        elif b:
            print(f"no frames for {spec['key']} in {FRAMES}: run `record` first; the replay is left out")
        sc["end"] = clock + 0.3
        clock = sc["end"]
        scenes.append(sc)
    return scenes, clock + 4.8                           # end card


def wrap(text: str) -> list[str]:
    return textwrap.wrap(text, COLS, replace_whitespace=False, drop_whitespace=False) or [""]


def style_for(text: str):
    low = text.lower()
    if "successfully installed" in low or re.match(r"^\s+1\s", text):
        return GREEN
    if "dnf" in low or re.search(r"\berror\b|failed", low):
        return RED
    if text.startswith("Replay:"):
        return BLUE
    if text.startswith(("F1-Race |", "Practice", "Race -", "Classification", "Downloading")):
        return INK
    if text.lstrip().startswith(("POS ", "Collecting", "Downloading", "Obtaining", "Installing", "Building",
                                 "Preparing", "Requirement", "Using cached", "Getting requirements", "Created wheel",
                                 "Stored in", "Attempting", "Checking", "Cloning")):
        return MUTED
    return INK2


TEXT = {
    "en": dict(title="Your first race", sub="from git clone to the chequered flag in four steps",
               real="{real} of real time in {video} s", step="STEP {i}/{n}", ff="▶▶ ×{x:.0f} · {t} real",
               done="Chequered flag!", next="Make it your race",
               nexts=["python scripts/race.py --track Monza   another circuit",
                      "python scripts/race.py --drivers ocp centre   pick the field",
                      "docs/L1.md   why a plan spins on a car with tyre slip"]),
    "zh": dict(title="你的第一场比赛", sub="从 git clone 到冲过终点，只需四步",
               real="真实耗时 {real}，视频 {video} 秒", step="第 {i}/{n} 步", ff="▶▶ ×{x:.0f} · 真实 {t}",
               done="完赛！", next="打造你自己的比赛",
               nexts=["python scripts/race.py --track Monza   换一条赛道",
                      "python scripts/race.py --drivers ocp centre   选择参赛车手",
                      "docs/L1.md   为什么规划在有轮胎滑移的车上会打滑"]),
}


def mmss(sec: float) -> str:
    return f"{int(sec // 60)}:{int(sec % 60):02d}"


def render(lang: str) -> None:
    from PIL import Image, ImageDraw, ImageFilter, ImageFont

    _, events = load_cast()
    scenes, total = build_timeline(events)
    tx = TEXT[lang]
    real_total = sum(c["real"] for sc in scenes for c in sc["cmds"])
    cap_font = font_path("cjk") if lang == "zh" else font_path("bold")
    sub_font = font_path("cjk") if lang == "zh" else font_path("sans")
    F = dict(mono=ImageFont.truetype(font_path("mono"), 21), endmono=ImageFont.truetype(font_path("mono"), 19),
             cap=ImageFont.truetype(cap_font, 34), sub=ImageFont.truetype(sub_font, 20),
             chip=ImageFont.truetype(cap_font, 16), small=ImageFont.truetype(sub_font, 17),
             big=ImageFont.truetype(font_path("bold"), 84), mid=ImageFont.truetype(cap_font, 36),
             body=ImageFont.truetype(sub_font, 22),
             ff=ImageFont.truetype(font_path("cjk") if lang == "zh" else font_path("mono"), 17))
    tw = (W - 120, H - 150)                               # terminal / browser window size
    tx0, ty0 = 60, 128
    line_h = 27
    char_w = F["mono"].getlength("M")
    frame_cache: dict = {}

    def base():
        im = Image.new("RGB", (W, H), BG)
        return im, ImageDraw.Draw(im)

    def title_card(t):
        im, d = base()
        a = min(1.0, t / 0.6)
        mix = lambda c: tuple(int(BG[k] + (c[k] - BG[k]) * a) for k in range(3))   # noqa: E731
        d.text((W / 2, 250), "F1-Race", font=F["big"], fill=mix(INK), anchor="mm")
        d.text((W / 2, 340), tx["title"], font=F["mid"], fill=mix(INK), anchor="mm")
        d.text((W / 2, 395), tx["sub"], font=F["body"], fill=mix(INK2), anchor="mm")
        d.text((W / 2, 450), tx["real"].format(real=mmss(real_total), video=round(total)), font=F["small"],
               fill=MUTED, anchor="mm")
        for k in range(len(STEPS)):
            x = W / 2 + (k - (len(STEPS) - 1) / 2) * 46
            d.ellipse((x - 6, 494, x + 6, 506), fill=ACCENT if a >= 1 else BAR)
        return im

    def end_card(t):
        im, d = base()
        d.text((W / 2, 196), "✓", font=F["big"], fill=GREEN, anchor="mm")
        d.text((W / 2, 286), tx["done"], font=F["mid"], fill=INK, anchor="mm")
        d.text((W / 2, 346), tx["next"], font=F["sub"], fill=MUTED, anchor="mm")
        box_w = 1000
        d.rounded_rectangle((W / 2 - box_w / 2, 376, W / 2 + box_w / 2, 376 + 46 * len(tx["nexts"]) + 22), 14,
                            fill=WIN)
        for k, line in enumerate(tx["nexts"]):
            cmd, _, desc = line.partition("   ")
            y = 399 + 46 * k
            d.text((W / 2 - box_w / 2 + 28, y), cmd.strip(), font=F["endmono"], fill=INK)
            d.text((W / 2 + box_w / 2 - 28, y), desc.strip(), font=F["small"], fill=MUTED, anchor="ra")
        d.text((W / 2, 640), "github.com/yuanche-auto/F1-Race", font=F["small"], fill=MUTED, anchor="mm")
        return im

    def window(d, title):
        d.rounded_rectangle((tx0, ty0, tx0 + tw[0], ty0 + tw[1]), 14, fill=WIN)
        d.rounded_rectangle((tx0, ty0, tx0 + tw[0], ty0 + 36), 14, fill=BAR)
        d.rectangle((tx0, ty0 + 22, tx0 + tw[0], ty0 + 36), fill=BAR)
        for k, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
            d.ellipse((tx0 + 18 + 22 * k, ty0 + 12, tx0 + 30 + 22 * k, ty0 + 24), fill=c)
        d.text((tx0 + tw[0] / 2, ty0 + 18), title, font=F["small"], fill=MUTED, anchor="mm")

    def terminal(d, sc, t):
        window(d, "bash — F1-Race")
        rows, cursor, badge = [], None, None
        for c in sc["cmds"]:
            if t < c["type0"]:
                break
            n = int((t - c["type0"]) * TYPE_CPS) if t < c["type1"] else len(c["cmd"])
            rows.append([(c["cwd"], BLUE), (" $ ", GREEN), (c["cmd"][:n], INK)])
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
            d.rounded_rectangle((tx0 + tw[0] - bw - 14, ty0 + 48, tx0 + tw[0] - 14, ty0 + 76), 8, fill=(56, 36, 20))
            d.text((tx0 + tw[0] - bw / 2 - 14, ty0 + 62), badge, font=F["ff"], fill=(255, 166, 87), anchor="mm")

    def browser(im, sc, t):
        b = sc["browser"]
        a = min(1.0, (t - b["t0"]) / 0.35)
        k = min(int((t - b["t0"]) * CLIP_FPS), b["frames"] - 1)
        if k not in frame_cache:
            frame_cache.clear()
            img = Image.open(b["dir"] / f"f_{k:04d}.png").convert("RGB")
            sc_ = min(tw[0] / img.size[0], (tw[1] - 36) / img.size[1])
            frame_cache[k] = img.resize((int(img.size[0] * sc_), int(img.size[1] * sc_)), Image.LANCZOS)
        img = frame_cache[k]
        dy = int((1 - a) * 24)
        layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
        ImageDraw.Draw(layer).rounded_rectangle((tx0 + 4, ty0 + 12 + dy, tx0 + tw[0] + 4, ty0 + tw[1] + 12 + dy), 16,
                                                fill=(0, 0, 0, int(160 * a)))
        im.paste(Image.new("RGB", (W, H), (0, 0, 0)), (0, 0), layer.filter(ImageFilter.GaussianBlur(10)))
        d = ImageDraw.Draw(im)
        d.rounded_rectangle((tx0, ty0 + dy, tx0 + tw[0], ty0 + tw[1] + dy), 14, fill=(13, 17, 23), outline=BAR)
        d.rounded_rectangle((tx0, ty0 + dy, tx0 + tw[0], ty0 + 36 + dy), 14, fill=BAR)
        d.rectangle((tx0, ty0 + 22 + dy, tx0 + tw[0], ty0 + 36 + dy), fill=BAR)
        for j, c in enumerate([(255, 95, 86), (255, 189, 46), (39, 201, 63)]):
            d.ellipse((tx0 + 18 + 22 * j, ty0 + 12 + dy, tx0 + 30 + 22 * j, ty0 + 24 + dy), fill=c)
        url = "file://" + b["url"]
        uw = F["small"].getlength(url) + 30
        d.rounded_rectangle((W / 2 - uw / 2, ty0 + 6 + dy, W / 2 + uw / 2, ty0 + 30 + dy), 12, fill=WIN)
        d.text((W / 2, ty0 + 18 + dy), url, font=F["small"], fill=INK2, anchor="mm")
        im.paste(img, (int(tx0 + (tw[0] - img.size[0]) / 2), int(ty0 + 36 + dy + (tw[1] - 36 - img.size[1]) / 2)))
        return im

    def caption(d, sc):
        i, n = sc["index"] + 1, len(STEPS)
        title, sub = STEPS[sc["index"]][lang]
        chip = tx["step"].format(i=i, n=n)
        cw = F["chip"].getlength(chip) + 26
        d.rounded_rectangle((60, 30, 60 + cw, 60), 15, fill=ACCENT)
        d.text((60 + cw / 2, 45), chip, font=F["chip"], fill=(255, 255, 255), anchor="mm")
        d.text((60 + cw + 16, 45), title, font=F["cap"], fill=INK, anchor="lm")
        d.text((60, 90), sub, font=F["sub"], fill=MUTED, anchor="lm")
        for k in range(n):                                # step progress
            x = W - 60 - (n - k) * 38
            d.rounded_rectangle((x, 40, x + 30, 48), 4, fill=ACCENT if k < i else BAR)

    mp4 = MEDIA / f"quickstart_{lang}.mp4"
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r",
         str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "slow",
         "-movflags", "+faststart", str(mp4)], stdin=subprocess.PIPE)
    for f in range(int(math.ceil(total * FPS))):
        t = f / FPS
        if t < scenes[0]["start"]:
            im = title_card(t)
        elif t >= scenes[-1]["end"]:
            im = end_card(t - scenes[-1]["end"])
        else:
            sc = next(s for s in scenes if s["start"] <= t < s["end"])
            im, d = base()
            caption(d, sc)
            terminal(d, sc, t)
            if "browser" in sc and t >= sc["browser"]["t0"]:
                im = browser(im, sc, t)
        proc.stdin.write(im.tobytes())
    proc.stdin.close()
    proc.wait()
    gif = MEDIA / f"quickstart_{lang}.gif"
    vf = ("fps=12,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];"
          "[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4), "-vf", vf, "-loop", "0", str(gif)],
                   check=True)
    print(f"{mp4.name}: {mp4.stat().st_size / 1e6:.2f} MB, {total:.1f} s; {gif.name}: {gif.stat().st_size / 1e6:.2f} MB")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["record", "render", "all"])
    ap.add_argument("--workdir", type=Path, default=None, help="where record clones the repository (default: temp)")
    ap.add_argument("--ref", default=None, help="branch or commit to check out after the clone (hidden)")
    ap.add_argument("--lang", nargs="*", default=["en", "zh"])
    args = ap.parse_args(argv)
    if args.what in ("record", "all"):
        record(args.workdir, args.ref)
    if args.what in ("render", "all"):
        for lang in args.lang:
            render(lang)


if __name__ == "__main__":
    sys.exit(main())
