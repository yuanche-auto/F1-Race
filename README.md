# F1-Race — minimum-lap-time racing, layer by layer

A self-contained study of how to drive a racing car around a track as fast as
possible, built up in layers. Each layer adds one source of difficulty and
compares several solution methods on the same problem:

| Layer | Setting | Methods compared | Status |
|---|---|---|---|
| **L0** | deterministic kinematic model | minimum-curvature line + quasi-steady speed profile; minimum-time optimal control; closed-loop verification | ✅ done ([docs/L0.md](docs/L0.md)) |
| L1 | dynamic single-track model with tyre slip (Pacejka) and load transfer | minimum-time optimal control on the dynamic model; how much the kinematic assumption costs | planned |
| L2 | closed loop with model mismatch | offline trajectory + tracking, NMPC, MPCC, constraint tightening | planned |
| L3 | stochastic disturbances | stochastic MPC (chance constraints), scenario MPC | planned |
| L4 | learning and adaptation | online parameter estimation + adaptive MPC, Gaussian-process residual models, learning MPC | planned |
| L5 | reinforcement learning | SAC/PPO with domain randomisation, residual RL, RL value functions as MPC terminal cost | planned |

The guiding question of the whole series: *in which settings does a
learning-based controller beat a model-based one, and why?*

## Quick start

```bash
python -m venv .venv && .venv/Scripts/activate      # or: source .venv/bin/activate
pip install -e .[dev]
python scripts/fetch_tracks.py Spielberg             # downloads the track (see "Data")
python scripts/run_l0.py                             # ~40 s; writes results/L0 and docs/figures/L0
pytest                                               # ~1-2 min
```

## L0 results (Spielberg, generic open-wheel car with downforce)

| Plan | Lap time (ideal, friction fully used) | Lap time with 3 % friction margin | Closed loop (simulator) |
|---|---|---|---|
| centre line + speed profile | 78.35 s | – | – |
| minimum-curvature line + speed profile | 72.33 s | 73.09 s | 73.17 s |
| minimum-time optimal control | **71.43 s** | 72.25 s | **72.28 s** |

![lines](docs/figures/L0/lines.png)

Details, model equations and discussion: [docs/L0.md](docs/L0.md).

## Repository layout

```
src/f1race/
  track.py          closed track: smoothing spline, widths, Frenet projection
  vehicle.py        vehicle parameters, friction ellipse with downforce, power limit
  path.py           closed paths (racing lines) with curvature and its derivative
  planning/         minimum-curvature QP, QSS speed profile, minimum-time OCP
  control/          path + speed tracking controller
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
