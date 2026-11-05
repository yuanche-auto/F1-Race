"""The HTML replay of a race (``race.write_replay``): one self-contained page, no network.
The race data replaces the /*RACE_DATA*/ marker."""

TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>F1-Race replay</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --line:#30363d; --ink:#f0efec; --ink2:#c3c2b7; --muted:#898781;
          --asphalt:#30363d; --edge:#6e7681; --accent:#3987e5; }
  * { box-sizing: border-box; }
  html, body { margin: 0; background: var(--bg); color: var(--ink);
               font: 14px/1.4 system-ui, -apple-system, "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif; }
  header { display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap; padding: 16px 20px 4px; }
  header h1 { font-size: 20px; margin: 0; }
  header .sub { color: var(--muted); }
  main { display: grid; grid-template-columns: minmax(0, 1fr) 340px; gap: 16px; padding: 8px 20px; }
  @media (max-width: 860px) { main { grid-template-columns: 1fr; } }
  .card { background: var(--panel); border: 1px solid var(--line); border-radius: 12px; }
  #map { width: 100%; height: 470px; display: block; }
  #tower { padding: 14px 16px; }
  .clock { font: 700 34px/1 ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace; margin: 4px 0 12px; }
  .lbl { color: var(--muted); font-size: 11px; font-weight: 700; letter-spacing: .06em; }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 7px 4px; border-top: 1px solid var(--line); vertical-align: middle; }
  td.pos { font-weight: 700; width: 34px; font-size: 16px; }
  td.num { text-align: right; white-space: nowrap; font-variant-numeric: tabular-nums;
           font-family: ui-monospace, Menlo, Consolas, monospace; }
  .who { display: flex; align-items: center; gap: 8px; }
  .sw { flex: none; width: 4px; height: 30px; border-radius: 2px; }
  .nm small { display: block; color: var(--muted); font-size: 11px; line-height: 1.25; }
  .dnf { color: #f85149; font-weight: 700; }
  #chart { width: 100%; height: 150px; display: block; }
  .bottom { padding: 0 20px 8px; }
  .controls { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; padding: 12px 20px 20px; }
  button { background: var(--panel); color: var(--ink); border: 1px solid var(--line); border-radius: 8px;
           padding: 6px 12px; font: inherit; cursor: pointer; }
  button.on { background: var(--accent); border-color: var(--accent); color: #fff; }
  input[type=range] { flex: 1; min-width: 160px; accent-color: var(--accent); }
  #t { font-family: ui-monospace, Menlo, Consolas, monospace; color: var(--ink2); min-width: 92px; }
  label { color: var(--ink2); display: flex; gap: 6px; align-items: center; }
  #final { margin-top: 14px; color: var(--ink2); font-size: 13px; }
</style>
</head>
<body>
<header>
  <h1 id="title">F1-Race replay</h1>
  <span class="sub" id="sub"></span>
</header>
<main>
  <div class="card"><canvas id="map"></canvas></div>
  <div class="card" id="tower">
    <div class="lbl">LAP TIME</div>
    <div class="clock" id="clock">0:00.000</div>
    <table><tbody id="rows"></tbody></table>
    <div id="final"></div>
  </div>
</main>
<div class="bottom"><div class="card"><canvas id="chart"></canvas></div></div>
<div class="controls">
  <button id="play">❚❚ Pause</button>
  <span id="speeds"></span>
  <input type="range" id="seek" min="0" step="0.05" value="0" aria-label="race time">
  <span id="t"></span>
  <label><input type="checkbox" id="follow"> follow the leader</label>
</div>
<script id="race-data" type="application/json">/*RACE_DATA*/</script>
<script>
(() => {
  const D = JSON.parse(document.getElementById("race-data").textContent);
  const cars = D.cars, L = D.track.length;
  const end = Math.max(...cars.map(c => c.t[c.t.length - 1]));
  const duration = end + 2.5;
  const best = Math.min(...cars.filter(c => c.finished).map(c => c.lap));
  document.getElementById("title").textContent = `${D.track.name} · one flying lap`;
  document.getElementById("sub").textContent =
    `${(L / 1000).toFixed(2)} km · kinematic car · the plans use ${(D.grip * 100).toFixed(0)} % of the grip`;
  const fmt = t => `${Math.floor(t / 60)}:${(t % 60).toFixed(3).padStart(6, "0")}`;

  // ---- sampling
  function idx(arr, v) {                       // last i with arr[i] <= v
    let lo = 0, hi = arr.length - 1;
    if (v <= arr[0]) return 0;
    if (v >= arr[hi]) return hi;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (arr[m] <= v) lo = m; else hi = m; }
    return lo;
  }
  function at(c, t) {
    const i = idx(c.t, t), j = Math.min(i + 1, c.t.length - 1);
    const a = j > i ? Math.min(1, Math.max(0, (t - c.t[i]) / (c.t[j] - c.t[i]))) : 0;
    const f = k => c[k][i] + (c[k][j] - c[k][i]) * a;
    return { x: f("x"), y: f("y"), v: f("v"), p: f("p"), done: t >= c.t[c.t.length - 1] };
  }
  const timeAt = (c, p) => { const i = idx(c.p, p), j = Math.min(i + 1, c.p.length - 1);
    const a = j > i && c.p[j] > c.p[i] ? (p - c.p[i]) / (c.p[j] - c.p[i]) : 0;
    return c.t[i] + (c.t[j] - c.t[i]) * a; };

  // ---- map
  const map = document.getElementById("map"), mctx = map.getContext("2d");
  const pts = D.track.left.concat(D.track.right);
  const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
  const bx = [Math.min(...xs), Math.max(...xs)], by = [Math.min(...ys), Math.max(...ys)];
  let follow = false;
  function view(lead) {
    const w = map.clientWidth, h = map.clientHeight, pad = 24;
    if (follow && lead) {
      const s = Math.min(w, h) / 220;
      return { s, ox: w / 2 - lead.x * s, oy: h / 2 + lead.y * s };
    }
    const s = Math.min((w - 2 * pad) / (bx[1] - bx[0]), (h - 2 * pad) / (by[1] - by[0]));
    return { s, ox: (w - s * (bx[0] + bx[1])) / 2, oy: (h + s * (by[0] + by[1])) / 2 };
  }
  function drawMap(t, states, lead) {
    const dpr = window.devicePixelRatio || 1, w = map.clientWidth, h = map.clientHeight;
    if (map.width !== w * dpr) { map.width = w * dpr; map.height = h * dpr; }
    const c = mctx; c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
    const V = view(lead), X = p => V.ox + p[0] * V.s, Y = p => V.oy - p[1] * V.s;
    c.beginPath();
    for (const edge of [D.track.left, D.track.right]) {
      edge.forEach((p, k) => k ? c.lineTo(X(p), Y(p)) : c.moveTo(X(p), Y(p))); c.closePath();
    }
    c.fillStyle = getComputedStyle(document.documentElement).getPropertyValue("--asphalt"); c.fill("evenodd");
    c.strokeStyle = "#6e7681"; c.lineWidth = 1; c.stroke();
    const s0 = D.track.start, n0 = D.track.normal;
    c.strokeStyle = "#f0efec"; c.lineWidth = 3; c.beginPath();
    c.moveTo(X([s0[0] - 8 * n0[0], s0[1] - 8 * n0[1]]), Y([s0[0] - 8 * n0[0], s0[1] - 8 * n0[1]]));
    c.lineTo(X([s0[0] + 8 * n0[0], s0[1] + 8 * n0[1]]), Y([s0[0] + 8 * n0[0], s0[1] + 8 * n0[1]])); c.stroke();
    cars.forEach((car, k) => {                   // trails: the last 2.5 s
      c.strokeStyle = car.color; c.globalAlpha = 0.75; c.lineWidth = 3; c.lineCap = "round"; c.beginPath();
      const t1 = Math.min(t, car.t[car.t.length - 1]);
      for (let q = Math.max(0, t1 - 2.5), m = 0; q <= t1 + 1e-9; q += 0.1, m++) {
        const s = at(car, q); m ? c.lineTo(X([s.x, s.y]), Y([s.x, s.y])) : c.moveTo(X([s.x, s.y]), Y([s.x, s.y]));
      }
      c.stroke(); c.globalAlpha = 1;
    });
    cars.forEach((car, k) => {
      const s = states[k], px = X([s.x, s.y]), py = Y([s.x, s.y]);
      if (!car.finished && s.done) {             // where it stopped
        c.strokeStyle = "#f85149"; c.lineWidth = 3; c.beginPath();
        c.moveTo(px - 7, py - 7); c.lineTo(px + 7, py + 7); c.moveTo(px + 7, py - 7); c.lineTo(px - 7, py + 7); c.stroke();
        return;
      }
      c.beginPath(); c.arc(px, py, 7, 0, 2 * Math.PI); c.fillStyle = car.color; c.fill();
      c.lineWidth = 2; c.strokeStyle = "#0d1117"; c.stroke();
      c.fillStyle = "#f0efec"; c.font = "600 12px system-ui, sans-serif"; c.fillText(car.code, px + 10, py - 6 - 14 * k);
    });
  }

  // ---- speed chart
  const chart = document.getElementById("chart"), cctx = chart.getContext("2d");
  const vmax = Math.max(...cars.map(c => Math.max(...c.v)));
  function drawChart(t, states) {
    const dpr = window.devicePixelRatio || 1, w = chart.clientWidth, h = chart.clientHeight;
    if (chart.width !== w * dpr) { chart.width = w * dpr; chart.height = h * dpr; }
    const c = cctx; c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, w, h);
    const l = 44, r = 12, tp = 22, b = 24, X = p => l + (w - l - r) * p / L, Y = v => h - b - (h - tp - b) * v / (vmax * 1.05);
    c.font = "11px system-ui, sans-serif"; c.fillStyle = "#898781"; c.strokeStyle = "#2c2c2a"; c.lineWidth = 1;
    for (let v = 100; v < vmax * 1.05; v += 100) { c.beginPath(); c.moveTo(l, Y(v)); c.lineTo(w - r, Y(v)); c.stroke(); c.fillText(v, 12, Y(v) + 4); }
    for (let km = 0; km <= L / 1000; km += 0.5) { c.fillText(km.toFixed(1), X(km * 1000) - 8, h - 6); }
    c.fillText("SPEED [km/h] over the distance along the centre line [km]", l, 14);
    cars.forEach((car, k) => {
      const n = idx(car.t, t);
      c.strokeStyle = car.color; c.lineWidth = 2; c.beginPath();
      for (let i = 0; i <= n; i++) i ? c.lineTo(X(car.p[i]), Y(car.v[i])) : c.moveTo(X(car.p[i]), Y(car.v[i]));
      c.stroke();
      const s = states[k]; c.beginPath(); c.arc(X(s.p), Y(s.v), 4, 0, 2 * Math.PI); c.fillStyle = car.color; c.fill();
    });
  }

  // ---- timing tower
  const rows = document.getElementById("rows"), clock = document.getElementById("clock"), fin = document.getElementById("final");
  function drawTower(t, states) {
    const order = cars.map((c, k) => k).sort((a, b) => {
      const A = cars[a], B = cars[b], sa = states[a], sb = states[b];
      const key = (C, S) => C.finished && S.done ? -1e6 + C.lap : (S.done ? 1e6 - S.p : -S.p);   // DNF last
      return key(A, sa) - key(B, sb);
    });
    const lead = cars[order[0]];
    rows.innerHTML = order.map((k, pos) => {
      const c = cars[k], s = states[k];
      let speed, gap;
      if (c.finished && s.done) { speed = c.lap.toFixed(2) + "&nbsp;s"; gap = c.lap === best ? "WINNER" : "+" + (c.lap - best).toFixed(2); }
      else if (s.done) { speed = ""; gap = `<span class="dnf">DNF</span>`; }
      else { speed = s.v.toFixed(0); gap = pos === 0 ? "—" : "+" + Math.max(0, t - timeAt(lead, s.p)).toFixed(2); }
      return `<tr><td class="pos">${!c.finished && s.done ? "–" : pos + 1}</td>` +
        `<td><div class="who"><span class="sw" style="background:${c.color}"></span>` +
        `<span class="nm">${c.name}<small>${c.style}</small></span></div></td>` +
        `<td class="num">${speed}</td><td class="num">${gap}</td></tr>`;
    }).join("");
    clock.textContent = fmt(Math.min(t, end));
    const dnfs = cars.filter((c, k) => !c.finished && states[k].done);
    fin.innerHTML = dnfs.map(c => `<div><span class="dnf">DNF</span> ${c.name}: ${c.reason} at ${Math.round(c.distance).toLocaleString("en")} m</div>`).join("");
    return order;
  }

  // ---- loop and controls
  let t = 0, speed = 4, playing = true, last = null;
  const seek = document.getElementById("seek"), tl = document.getElementById("t"), playB = document.getElementById("play");
  seek.max = duration.toFixed(2);
  const sp = document.getElementById("speeds");
  [1, 2, 4, 8, 16].forEach(x => { const b = document.createElement("button"); b.textContent = x + "×";
    b.onclick = () => { speed = x; [...sp.children].forEach(c => c.classList.toggle("on", c === b)); };
    if (x === speed) b.classList.add("on"); sp.appendChild(b); });
  function render() {
    const states = cars.map(c => at(c, t));
    const order = drawTower(t, states);
    drawMap(t, states, states[order[0]]);
    drawChart(t, states);
    seek.value = t; tl.textContent = `${fmt(Math.min(t, end))} / ${fmt(end)}`;
  }
  function setPlaying(p) { playing = p; playB.textContent = p ? "❚❚ Pause" : "▶ Play"; last = null; }
  playB.onclick = () => { if (!playing && t >= duration) t = 0; setPlaying(!playing); };
  seek.oninput = () => { t = +seek.value; render(); };
  document.getElementById("follow").onchange = e => { follow = e.target.checked; render(); };
  document.addEventListener("keydown", e => {
    if (e.code === "Space") { e.preventDefault(); playB.click(); }
    if (e.code === "ArrowRight") { t = Math.min(duration, t + 2); render(); }
    if (e.code === "ArrowLeft") { t = Math.max(0, t - 2); render(); }
  });
  function frame(now) {
    if (playing) { if (last !== null) t += (now - last) / 1000 * speed; last = now;
      if (t >= duration) { t = duration; setPlaying(false); } render(); }
    requestAnimationFrame(frame);
  }
  window.addEventListener("resize", render);
  window.replay = { duration, end, cars: cars.map(c => ({ code: c.code, finished: c.finished, tEnd: c.t[c.t.length - 1] })),
                    seek: x => { t = x; render(); }, play: () => setPlaying(true), pause: () => setPlaying(false),
                    setSpeed: x => { speed = x; },
                    setFollow: f => { follow = f; document.getElementById("follow").checked = f; render(); } };
  render(); requestAnimationFrame(frame);
})();
</script>
</body>
</html>
"""
