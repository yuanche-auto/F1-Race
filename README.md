# F1-Race — minimum-lap-time racing, layer by layer

A self-contained study of how to drive a racing car around a track as fast as
possible, built up in layers. Each layer adds one source of difficulty and
compares several solution methods on the same problem:

| Layer | Setting | Methods compared | Status |
|---|---|---|---|
| **L0** | deterministic kinematic model | minimum-curvature line + quasi-steady speed profile; minimum-time optimal control; closed-loop verification | ✅ done ([docs/L0.md](docs/L0.md)) |
| **L1** | dynamic single-track model with tyre slip (magic formula) and load transfer | minimum-time optimal control on the dynamic model; feed-forward/feedback tracking; what the kinematic model misses | ✅ done ([docs/L1.md](docs/L1.md)) |
| **L2** | closed loop with model mismatch | offline plan + tracking vs NMPC, constraint tightening, Monte Carlo and one-at-a-time sensitivity | ✅ done ([docs/L2.md](docs/L2.md)) |
| **L3** | stochastic disturbances (friction varying along the track, gusts, sensor noise) | NMPC with uniform reserves vs stochastic MPC with back-offs calibrated from measured prediction errors (four calibrations); ablations | ✅ done ([docs/L3.md](docs/L3.md)) |
| L4 | learning and adaptation | online parameter estimation + adaptive MPC, Gaussian-process residual models, learning MPC | planned |
| L5 | reinforcement learning | SAC/PPO with domain randomisation, residual RL, RL value functions as MPC terminal cost | planned |

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
python scripts/run_l3.py                             # ~8 h (12 processes) incl. calibrate / iterate / failures; 18 controllers x 30 seeds
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

## L3 results (random friction field, gusts and sensor noise; 30 laps each)

| Controller | Laps completed (95 % interval) | Mean lap |
|---|---|---|
| offline plan + tracking | 0 / 30 (0–12 %) | – |
| NMPC, uniform 5 % friction reserve | 5 / 30 (6–35 %) | 77.45 s |
| NMPC, uniform 11 % and 0.8 m track reserve | 16 / 30 (34–72 %) | 81.43 s |
| **stochastic MPC, ε = 0.05** | **29 / 30** (83–100 %) | 78.26 s |
| **stochastic MPC, ε = 0.01** | **30 / 30** (88–100 %) | 79.20 s |

Back-offs calibrated from the measured prediction errors, growing along the
horizon and covering friction and track limits, make the MPC both safer and
faster than any uniform reserve; with three other calibrations it completes
28–30 of 30, while the same average tightening applied uniformly along the
horizon completes 10 of 30. See [docs/L3.md](docs/L3.md).

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
  sim/              time-domain simulator with constraint monitoring; stochastic
                    disturbances (L3)
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
