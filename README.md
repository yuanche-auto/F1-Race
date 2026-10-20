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
| **L6** | structural model error (steering actuator lag) | the fixed baselines of L1–L5; parameter identification vs identification of the lag; correct-structure NMPC; learned residual dynamics with uncertainty; EKF under sensor noise | ✅ done ([docs/L6.md](docs/L6.md); roadmap L6–L10: [docs/ROADMAP_L6-L10.md](docs/ROADMAP_L6-L10.md)) |
| **L7** | reliable learning | diagnosis of the L5 runs' blind spots; an NMPC safety filter with calibrated risk estimates on top of the learned policies; training that restarts episodes before failure positions | ✅ done ([docs/L7.md](docs/L7.md)) |
| **L8** | generalisation: new tracks and wet friction, one at a time | the fixed baselines and the trained policies on four unseen tracks and on wet Spielberg; identification + re-planning; short fine-tuning | ✅ done ([docs/L8.md](docs/L8.md)) |
| **L9** | real time: every controller on one core within the 50 ms control period | the offline implementations (IPOPT) against acados real-time iterations of the same OCP; delay compensation; sensor delay; distilled policies | ✅ done ([docs/L9.md](docs/L9.md)) |
| **L10** | interactive racing (exploratory) | one opponent of fixed behaviour first | next: protocol ([roadmap](docs/ROADMAP_L6-L10.md)) |

Every layer from L5 on was planned in a pre-registered protocol
(`docs/L*_protocol.md`, amendments listed at its end) before any test lap ran.

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
python scripts/run_l6.py plans && python scripts/run_l6.py round1 --taus 0.06   # L6 (rounds 1-4 and report: see the script)
# L7-L9 run from worktrees pinned to one commit; the steps and commands are in each report:
#   L7: docs/L7.md (scripts/l7_partA.py, run_l7.py, run_l7_partB.py, train_l7.py, run_l7_partC.py; RL extras)
#   L8: docs/L8.md (scripts/fetch_tracks.py <track>, l8_plans.py, run_l8.py, report_l8.py)
#   L9: docs/L9.md (WSL with acados, scripts/l9_wsl/; run_l9.py, report_l9.py)
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

## L6 results (a steering actuator lag the models do not have; the 25 cars of L4)

Laps completed of 25, with the controllers seeing their own commanded angle:

| Controller | τ = 10 ms | τ = 30 ms | τ = 60 ms |
|---|---|---|---|
| NMPC, nominal model | 12 | 3 | 0 |
| SMPC (L3, calibration unchanged) | 22 | 22 | 9 |
| NMPC, parameters identified (L4) | 22 | 0 | 0 |
| **NMPC + lag model, τ and parameters identified** | **25** | **25** | **24** |
| NMPC + lag model, true car (reference) | 25 | 24 | 23 |
| residual RL (L5, unchanged) | 25 | 24 | 23 |

Parameter errors and the structural error fail separately and both have to be
fixed: identifying parameters in the wrong structure fails from 30 ms on, while
identifying the lag together with the parameters matches the correct-structure
reference (lap times 76.8–77.4 s; the RL policy is as reliable, ~4 s per lap
slower). A learned residual model was not calibrated well enough to carry a
safety layer (no-go for L7). See [docs/L6.md](docs/L6.md).

## L7 results (reliable learning)

* **Diagnosis of the nine L5 runs:** the blind spots are a few hard corners, and
  which one a run fails at depends on the run; the failing cars mostly have less
  grip. Whether the cause is too little experience or an observation that cannot
  reveal the grip in time is not separable from these data.
* **Safety layer (H-B, not supported – the opposite):** an NMPC safety filter
  with a risk estimate that passed its acceptance test lowered the completions of
  all nine final L5 checkpoints on the test laps (exact signed-rank p = 0.0039; the
  selected policy 102 → 51 of 105); 88–99 % of its interventions were not needed.
  More than half of the filtered failures followed runs of failed solves. The
  result is for this filter (no solver retries); it is not a statement about
  safety filters in general.
* **Training from failures (H-C, not supported – the opposite):** restarting 30 %
  of the episodes before recent failure positions gave a median seed that
  completes fewer test laps than plain training (50 vs 80 of 105, 4 : 34,
  p = 6·10⁻⁷); at the level of seeds the two groups do not differ significantly
  (p = 0.69), the curriculum's seeds are far more spread. Checkpoints selected on
  validation laps reached 104 and 105 of 105 (a secondary result).

See [docs/L7.md](docs/L7.md) and the protocol [docs/L7_protocol.md](docs/L7_protocol.md).

## L8 results (four unseen tracks; wet Spielberg)

| Controller | new tracks, 25 cars × 4 | wet, friction × 0.85 / × 0.75 (25 cars each) |
|---|---|---|
| L1 tracker | 4 / 100 | 0 / 0 |
| NMPC, nominal | 49 / 100 | 0 / 0 |
| SMPC (Spielberg calibration) | 81 / 100 | 0 / 0 |
| **NMPC + lag model, identified, re-planned** | **100 / 100** | **25 / 25** |
| residual RL (L5 selected) | 0 / 100 | 8 / 0 |

Both primary tests hold (new tracks 100 : 0, p = 1.6·10⁻³⁰; wet 42 : 0,
p = 4.5·10⁻¹³) – for these two controllers, one of which drives an exploration
lap and re-plans. None of the 22 trained policies transfers: at most 2 of 100
laps on the new tracks. With the L3 disturbances on the new tracks the SMPC is the
most reliable (33 / 40). Short fine-tuning on a new track helps some seeds and
none on one track. Test tracks drawn by seed (Zandvoort, São Paulo, Austin,
Nürburgring; Norisring replaced by the rule when its plans failed). See
[docs/L8.md](docs/L8.md).

## L9 results (real time: one core, 50 ms control period, T1new + T2new)

| Controller (lag model, τ and parameters identified) | offline | real time |
|---|---|---|
| NMPC with IPOPT (as in L2–L8) | 88 / 105 | **0 / 105** |
| the same with a fixed 100 ms delay compensation | – | 0 / 105 |
| **the same OCP in acados, real-time iterations** | 74 / 105 | **79 / 105** |

Both primary tests hold: the time limit costs the IPOPT implementation every lap
(88 : 0, p = 6.5·10⁻²⁷; every IPOPT controller fails within the first 237 m, with
every solve late), and real-time iterations keep the controller on the track
(79 : 0, p = 3.3·10⁻²⁴; feedback ~2.4 ms per step) – though below the converged
IPOPT offline. A 20 ms sensor delay removes every RTI lap. The RL policies, at
~0.2 ms per decision, lose little to the time limit: the selected one 99 → 98 of
105, the nine L5 final checkpoints between 0 and 15 laps each. This compares two implementations of one controller on
this machine (WSL, acados v0.6.0) and under this rule. See [docs/L9.md](docs/L9.md).

**Not yet done:** the protocols of L7–L9 ask for the primary results to be
re-run in a second Python environment; that is deferred (see [STATUS.md](STATUS.md)).

## Repository layout

```
src/f1race/
  track.py          closed track: smoothing spline, widths, Frenet projection
  vehicle.py        vehicle parameters, friction ellipse with downforce, power limit
  dynamics.py       dynamic single-track model (L1): tyres, load transfer, axle ellipses
  path.py           closed paths (racing lines) with curvature and its derivative
  planning/         minimum-curvature QP, QSS speed profile, minimum-time OCP;
                    robust re-planning by continuation (replan.py, before L7)
  control/          path/speed tracking (L0), dynamic tracker (L1), NMPC and experimental
                    receding-horizon min-time MPC (L2); safety-filter mode of the NMPC
                    and a CBF-QP filter (L7, safety_filter.py); the same OCP in acados
                    with real-time iterations (L9, acados_mpc.py)
  sim/              time-domain simulator with constraint monitoring; stochastic
                    disturbances (L3); steering actuator lag (L6, lagged.py)
  estimation/       parameter identification from driving data (L4); identification
                    with the lag, EKF, residual models (L6)
  rl/               Gymnasium environment for residual RL on the tracker (L5);
                    failure-start curriculum (L7, curriculum.py)
  experiments/      one module per layer (l9_distill.py: the L9 DAgger students)
scripts/            data download and per-layer reproduction scripts; l9_wsl/: the WSL
                    environment of L9 (acados build, versions, run driver)
tests/              pytest suite
docs/               per-layer write-ups and figures, review workflow
```

## Data

Track centre lines and widths come from the
[TUM FTM racetrack-database](https://github.com/TUMFTM/racetrack-database)
(LGPL-3.0). They are **not** redistributed here: `scripts/fetch_tracks.py`
downloads them from a pinned upstream commit and checks SHA-256 sums.
L0–L7 and L9 use Spielberg; L8 also uses the validation tracks Monza and Sakhir
and the test tracks Zandvoort, SaoPaulo, Austin and Nuerburgring
(`python scripts/fetch_tracks.py <name>`). Results (`results/`) are not
committed either.

## Development process

Every layer is developed on its own branch and merged only after an
independent review (see [docs/workflow.md](docs/workflow.md) and
[STATUS.md](STATUS.md)).

## License

MIT for the code in this repository (see [LICENSE](LICENSE)). The track data
keeps its upstream license.
