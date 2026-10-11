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
| **L4** | learning the car | identify the parameters from an exploration lap (least squares on the logged dynamics), re-plan, drive with the identified model; noise sensitivity | ✅ done ([docs/L4.md](docs/L4.md)) |
| **L5** | reinforcement learning | residual soft actor-critic on top of the L1 tracker with an observation history, trained on randomised cars and disturbances; nine training runs, checkpoints selected on full validation laps; compared on the L2 and L3 test sets | ✅ done ([docs/L5.md](docs/L5.md)) |
| **L6** | structural model error (steering actuator lag) | the fixed baselines of L1–L5; parameter identification vs identification of the lag; correct-structure NMPC; learned residual dynamics with uncertainty; EKF under sensor noise | in review ([docs/L6.md](docs/L6.md), roadmap L6–L10: [docs/ROADMAP_L6-L10.md](docs/ROADMAP_L6-L10.md)) |

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
python scripts/run_l4.py                             # ~3 h (7 processes); identify + re-plan on 25 cars (noise: run_l4.py noise)
pip install -e .[rl] && python scripts/l5_multiseed.py   # L5: 9 training runs, validation, tests (~8 h; selected policies in models/L5)
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

## L4 results (the L2 cars, learned from one exploration lap)

| Controller | Laps completed on 25 cars (95 % interval) | Mean lap |
|---|---|---|
| NMPC, nominal model | 13 / 25 (31–72 %) | 76.35 s |
| exploration lap (NMPC, 12 % reserve) | 16 / 25 (43–82 %) | 79.21 s |
| **NMPC, identified model, re-planned** | **25 / 25** (86–100 %) | 76.87 s |
| NMPC, true model, re-planned | 25 / 25 (86–100 %) | 76.89 s |

The parameters are identified almost exactly from the exploration lap, even
when that lap ends off track; with them the controller is as good as one that
knows the car (12 to 0 against the nominal model, p = 0.0005). With realistic
sensor noise the naive estimate is biased but still completes 22 of 25. See
[docs/L4.md](docs/L4.md).

## L5 results (residual reinforcement learning, same test sets as L2 and L3)

| Controller | L2: 51 cars | L3: 30 disturbed laps |
|---|---|---|
| tracker alone; with a fixed 15 % slow-down | 3 / 51; 16 / 51 | 0 / 30; 14 / 30 |
| NMPC, nominal model (L2) / SMPC ε = 0.01 (L3) | 22 / 51, 76.3 s | 30 / 30, 79.2 s |
| NMPC slowed down by a uniform reserve | 29 / 51, 84.0 s | 17 / 30, 85.7 s |
| RL, final checkpoints of nine training runs | 19–50 / 51 (median runs 30–46) | 14–30 / 30 (median runs 27–29) |
| **RL, checkpoint selected on validation laps** | **51 / 51**, 81.2 s | **30 / 30**, 81.2 s |

A learned policy can be more robust than the model-based controller without a
model of the actual car, but a single training run is not enough: almost every
run beats the nominal NMPC, only about one in three beats an NMPC slowed to its
pace, and most runs have a blind spot, one corner where they fail on almost
every car, invisible in training. With several runs and a validation on full
laps each design yielded a good policy; it is still slower than the
model-based controllers that identify the car (L4, in a favourable setting) or
calibrate their reserves (L3). See
[docs/L5.md](docs/L5.md).

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
  estimation/       parameter identification from driving data (L4)
  rl/               Gymnasium environment for residual RL on the tracker (L5)
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
