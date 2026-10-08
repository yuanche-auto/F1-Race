# F1-Race — minimum-lap-time racing, layer by layer

A self-contained study of how to drive a racing car around a track as fast as
possible, built up in layers. Each layer adds one source of difficulty and
compares several solution methods on the same problem:

| Layer | Setting | Methods compared | Status |
|---|---|---|---|
| **L0** | deterministic kinematic model | minimum-curvature line + quasi-steady speed profile; minimum-time optimal control; closed-loop verification | ✅ done ([docs/L0.md](docs/L0.md)) |
| **L1** | dynamic single-track model with tyre slip (magic formula) and load transfer | minimum-time optimal control on the dynamic model; feed-forward/feedback tracking; what the kinematic model misses | ✅ done ([docs/L1.md](docs/L1.md)) |
| **L2** | closed loop with model mismatch | offline plan + tracking vs NMPC, constraint tightening, Monte Carlo and one-at-a-time sensitivity | ✅ done ([docs/L2.md](docs/L2.md)) |
| **L3** | stochastic disturbances (friction varying along the track, gusts, sensor noise) | NMPC with uniform reserves vs stochastic MPC with back-offs calibrated from measured prediction errors; ablations | 🔍 in review ([branch `layer/L3`](https://github.com/yuanche-auto/F1-Race/tree/layer/L3)) |
| **L4** | learning the car | identify the parameters from an exploration lap, re-plan, drive with the identified model; noise sensitivity | ✅ approved, merges after L3 ([branch `layer/L4`](https://github.com/yuanche-auto/F1-Race/tree/layer/L4)) |
| **L5** | reinforcement learning | residual soft actor-critic on top of the L1 tracker, with and without an observation history, trained on randomised cars and disturbances | 🔍 in review ([branch `layer/L5`](https://github.com/yuanche-auto/F1-Race/tree/layer/L5)) |

The guiding question of the whole series: *in which settings does a
learning-based controller beat a model-based one, and why?*

## Quick start

```bash
python -m venv .venv && .venv/Scripts/activate      # or: source .venv/bin/activate
pip install -e .[dev]                                # or first: pip install -r requirements-lock.txt
python scripts/fetch_tracks.py Spielberg             # downloads the track (see "Data")
python scripts/run_l0.py                             # ~65 s; writes results/L0 and docs/figures/L0
python scripts/l0_studies.py                         # ~4 min; grid / step studies
python scripts/run_l1.py                             # ~20 min; writes results/L1 and docs/figures/L1
python scripts/l1_studies.py                         # ~40 min; grid / model / tracker studies
python scripts/run_l2.py                             # ~3 h (8 processes); Monte Carlo over 51 cars
python scripts/l2_studies.py                         # ~20 min; one parameter at a time
pytest -m "not slow"                                 # ~2 min (full suite ~12 min, ~30 min if the first L1 guess fails)
```

## L0 results (Spielberg, generic open-wheel car with downforce)

| Plan | Lap time (ideal, friction fully used) | Lap time with 3 % friction margin | Closed loop (simulator) |
|---|---|---|---|
| centre line + speed profile | 77.89 s | – | – |
| minimum-curvature line + speed profile | 72.14 s | 72.89 s | 72.90 s |
| minimum-time optimal control | **71.26 s** | 72.07 s | **72.10 s** |

OCP on a 2 m grid (grid limit ≈ 71.0 s), speed profiles at 0.5 m; the closed
loop stays within 99.5 % of the friction ellipse for control steps from 5 to
40 ms. See the grid and step studies in [docs/L0.md](docs/L0.md).

![lines](docs/figures/L0/lines.png)

Details, model equations and discussion: [docs/L0.md](docs/L0.md).

## L1 results (same track and car, dynamic single-track model)

| Plan / run | Lap time |
|---|---|
| kinematic optimum (L0) | 71.26 s |
| dynamic optimum (ideal) | **73.91 s** (+2.6 s: rear-wheel drive, per-axle friction, slip) |
| dynamic plan with 5 % margin, driven by feed-forward/feedback tracking | **75.57 s** |
| kinematic plan driven on the dynamic car | spins at the first corner |

The tracker completes the lap only in a narrow window of plan margins and gains;
see [docs/L1.md](docs/L1.md).

## L2 results (plant parameters differ from the controllers' model)

| Controller | Laps completed on 51 cars (95 % interval) | Nominal car |
|---|---|---|
| offline plan (5 % reserve) + tracking | 3 / 51 (1–16 %) | 75.57 s |
| NMPC re-planning every 50 ms (5 % reserve) | **22 / 51** (29–58 %) | 76.07 s |
| the same NMPC with hard friction / slip constraints | 15 / 51 (17–44 %) | 75.88 s |
| NMPC with a 12 % reserve | 27 / 51 (38–67 %) | 78.91 s |

Re-planning completes 19 cars the tracker cannot (and none the other way round).
Hard constraints on the measured slip angles make the MPC problem infeasible and
cost 7 cars. A larger reserve helps little: the remaining failures are cars that
turn differently from the model (softer tyres, different aero balance), mostly
without reaching the friction limit; see [docs/L2.md](docs/L2.md).

## Work in review (L3–L5)

L3–L5 are complete and published on their branches, but not yet merged here:
every layer reaches `main` only after the independent review (see
[Development process](#development-process)). Preliminary results, which may
still change in review:

* **L3, stochastic disturbances** ([docs](https://github.com/yuanche-auto/F1-Race/blob/layer/L3/docs/L3.md)): on 30
  disturbed laps a stochastic MPC whose reserves grow along the prediction
  horizon completes 29–30 of 30, the best uniform reserve 16 of 30 (and 2.2 s
  slower). The review asked for checks of the calibration (it was taken from
  mostly failed laps); with three alternative calibrations the stochastic MPC
  completes 28–30 of 30.
* **L4, learning the car** ([docs](https://github.com/yuanche-auto/F1-Race/blob/layer/L4/docs/L4.md), approved):
  identifying the car from one exploration lap and re-planning completes 25 of 25
  cars, against 13 of 25 for the nominal model, as fast as with the true
  parameters; with sensor noise on the data 22 of 25.
* **L5, reinforcement learning** ([docs](https://github.com/yuanche-auto/F1-Race/blob/layer/L5/docs/L5.md)): a
  residual RL policy with an observation history completes all 51 L2 cars and all
  30 L3 laps, but is 3–6 s per lap slower than the model-based controllers that
  learn the car (L4) or calibrate their uncertainty (L3).

## Repository layout

```
src/f1race/
  track.py          closed track: smoothing spline, widths, Frenet projection
  vehicle.py        vehicle parameters, friction ellipse with downforce, power limit
  dynamics.py       dynamic single-track model (L1): tyres, load transfer, axle ellipses
  path.py           closed paths (racing lines) with curvature and its derivative
  planning/         minimum-curvature QP, QSS speed profile, minimum-time OCP
  control/          path/speed tracking (L0), dynamic tracker (L1), NMPC and experimental
                    receding-horizon min-time MPC (L2)
  sim/              time-domain simulator with constraint monitoring
  experiments/      one module per layer
scripts/            data download and per-layer reproduction scripts
tests/              pytest suite
docs/               per-layer write-ups and figures, review workflow
```

## Data

Track centre lines and widths come from the
[TUM FTM racetrack-database](https://github.com/TUMFTM/racetrack-database)
(LGPL-3.0). They are **not** redistributed here: `scripts/fetch_tracks.py`
downloads them from a pinned upstream commit and checks SHA-256 sums.

## Development process

Every layer is developed on its own branch and merged only after an
independent review (see [docs/workflow.md](docs/workflow.md) and
[STATUS.md](STATUS.md)).

## License

MIT for the code in this repository (see [LICENSE](LICENSE)). The track data
keeps its upstream license.
