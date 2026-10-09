# L6 protocol – model mismatch with a structural error

Fixed before the result-producing runs (2026-10-09). Deviations will be listed
at the end with their reason and date. Roadmap: [ROADMAP_L6-L10.md](ROADMAP_L6-L10.md).

**Disclosure of what was run before this protocol:** feasibility checks on the
nominal car only (no lap of any other car): the L1 tracker at τ = 30, 60 and
120 ms (off track or spin before 2 730 m at every τ), the nominal NMPC at τ =
30 ms (off track at 2 739 m), 60 and 120 ms (off track at about 460 m), and the
lag-aware NMPC with nominal parameters at the same τ (laps of 76.08, 76.02 and
76.05 s; 76.07 s without a lag in L2). The τ grid below was chosen after these
checks (section 4): the roadmap's 30 / 60 / 120 ms would leave every
lag-unaware controller at or near zero, so smaller lags were added.

## 1. Question

When the controller's model is wrong in its *structure* – not only in its
parameters – which of the methods of L2–L5 still work: re-planning with
reserves, calibrated tightening, parameter identification, residual learning
with its uncertainty, the residual RL policy? How far is each from a controller
with the correct structure?

## 2. Plant and what the controllers see

* The cars of L4: the nominal car and the first 24 cars of L2 (seed 0), 25
  cars. On the nominal car the lag is the only error; on the other 24 it comes
  on top of the parameter errors of L2.
* Structural error: a first-order steering actuator lag (`sim/lagged.py`). The
  controllers command a steering rate as before; it moves the commanded angle
  δ_c (same rate and angle limits); the road-wheel angle follows,
  δ' = (δ_c − δ)/τ; the tyre forces depend on δ. No parameter of the
  controllers' model can reproduce it.
* **Main observation condition ("command")**: the controllers receive their
  own commanded angle in the steering slot of the state; the road-wheel angle
  is not measured, the lag is hidden in the car's response (a servo without
  feedback, or tyre forces building up behind a measured angle). The
  identification of L4 therefore sees δ_c as the steering angle.
* Ablation ("angle"): the road-wheel angle is measured instead (section 4).
* Everything else as in L4: no disturbances, true state otherwise (noise is
  round 3), 10 ms simulator step, NMPC every 50 ms over 30 steps.

## 3. Controllers (round 1)

| name | model parameters | lag in the model | plan | note |
|---|---|---|---|---|
| `tracker` | nominal | no | 5 % | L1 tracker |
| `nmpc` | nominal | no | 5 % | L2 NMPC, η = 0.95 |
| `explore` | nominal | no | 12 % | uniform 12 % reserve (η = 0.88); the exploration lap of L4 |
| `smpc` | nominal | no | 12 % | L3 SMPC ε = 0.01, L3 calibration unchanged (calibrated on disturbances, not on a lag) |
| `identified` | identified from the `explore` lap as logged (δ_c) | no | re-planned, 5 % | L4 pipeline unchanged |
| `oracle_param` | true | no | re-planned with the true car, 5 % | what parameter knowledge buys (L4 "oracle") |
| `lag_nominal` | nominal | yes, true τ | 5 % | what structure knowledge buys |
| `oracle_struct` | true | yes, true τ | true-car plan, 5 % | **upper bound** |
| `rl` | – | – | 5 % | the selected L5 policy B_s1_500k, unchanged; observes δ_c |

The lag-aware MPCs integrate the lag exactly within each MPC step and
reconstruct the road-wheel angle with an open-loop observer from their own
commands (exact without noise; `LagAwareMPCController`). The plans are the
minimum-time plans without a lag (references for the tracking MPC).
`oracle_param` and `lag_nominal` together with `nmpc` and `oracle_struct` form a
2 × 2 design (parameters × structure).

## 4. Runs of round 1

**1a – lag sweep on the nominal car** (the lag is the only error): τ = 5, 10,
15, 20, 30, 45, 60, 90, 120 ms; `tracker`, `nmpc`, `explore`, `smpc`,
`identified`, `lag_nominal` (on the nominal car the same as `oracle_struct`) and
`rl`. Shows up to which lag each controller works.

**1b – the 25 cars** (lag and parameter errors): τ = 10, 30 and 60 ms (60 ms is
the main level of the later rounds), all controllers of section 3.

Order of the runs: 1a, then 1b at 60, 30, 10 ms.

Ablation: observation "angle" at τ = 60 ms for `tracker` and `nmpc` on the 25
cars.

Reference τ = 0 (parameter error only): the L4 results for `nmpc`, `explore`,
`identified`, `oracle_param` on the same cars (same code); `tracker` and
`smpc` at τ = 0 are run here. A re-run of the four L4 controllers at τ = 0 in
the L6 pipeline is done last, as a reproduction check, if time allows.

## 5. Later rounds (fixed now, details may be amended before they run)

**Round 2 – residual learning with uncertainty** (at the round-1 τ levels of
the 25-car runs). On top of the identified model, a residual of the body
accelerations (v_x', v_y', r') is learned from the exploration lap (the same
data as the identification, every 5th sample, i.e. one per 50 ms):

* inputs z = (v_x, v_y, r, δ, δ̇_cmd, F_drive, F_brake), standardised on the
  data; features: a constant, the 7 linear terms, and 100 random Fourier
  features of a squared-exponential kernel (fixed seed 0);
* Bayesian linear regression per output, prior precision and noise precision
  by evidence maximisation, the common length scale chosen from
  {0.5, 1, 2, 4} by the evidence – no tuning on laps;
* `residual`: the identified model plus the residual mean in the NMPC dynamics;
* `residual_cautious`: in addition, per-stage tightening from the predictive
  standard deviation σ of the lateral acceleration at the previous solution's
  prediction: friction factor η_k = max(0.80, 0.95 − 1.96 m σ_k /
  (μ_y F_z,k)), track back-off b_k = 1.96 Σ_{j<k} (k − j − ½) h² σ_j
  (errors assumed fully correlated over the horizon).

**Calibration of the residual uncertainty (go / no-go for L7):** coverage of
the nominal 95 % prediction interval of each acceleration on (i) the last
20 % of the exploration lap, held out from fitting (a separate fit on the
first 80 %), and (ii) the closed-loop laps driven by `residual` (the
distribution the controller actually visits). **Go** for L7 if the pooled
coverage of (ii) lies in 90–98 % at every τ of round 2; otherwise **no-go**:
L7 may not use this uncertainty directly (it would need recalibration, e.g.
empirical back-offs as in L3 or conformal intervals). Coverage per car is
reported too.

**Round 3 – noise only** (τ = 0, the L3 sensor noise on the logged
exploration lap, one noise draw per car as in L4): identification by the L4
least squares on raw data (biased, L4: 22 of 25), by least squares on
smoothed signals (Savitzky–Golay, window 11 samples, cubic, before
differencing), and by an extended Kalman filter estimating the velocities and
the 7 parameter factors jointly (parameters as random walks with a small fixed
variance, initial covariance from the L4 bounds). Metrics: parameter error,
parameters on their bounds, completion and lap time after re-planning.

**Round 4 – combined** (τ = 60 ms, noise, the L2 parameter errors): the
methods of rounds 2 and 3 composed; the noise-robust identification used there
is the one with the smaller parameter error in round 3 (a choice on
identification error, not on laps, fixed before round 4 runs).

## 6. Metrics and statistics

* Laps completed out of 25 with exact (Clopper–Pearson) 95 % intervals; mean
  lap time of completed laps; lap time on the nominal car; stop reasons.
* Lap time against the upper bound on cars both complete (paired).
* Identification: identified parameters against the true ones per τ,
  parameters on their bounds, residual rms.
* Model quality: mean and 95th percentile of the lateral-position error of the
  MPC predictions at horizon steps 5, 10, 20 and 30 (descriptive).
* Steering: largest gap between commanded and road-wheel angle.
* Pre-registered paired comparisons (exact McNemar, same cars, per τ):
  `oracle_struct`–`oracle_param` (value of the structure), `lag_nominal`–`nmpc`,
  `identified`–`nmpc`, `identified`–`oracle_param` (does identification absorb
  the lag?), `oracle_param`–`nmpc`, `smpc`–`nmpc`, `rl`–`nmpc`; round 2:
  `residual`–`identified`, `residual_cautious`–`residual`,
  `oracle_struct`–`residual`. p-values without correction; a conclusion needs
  p < 0.01 or the same direction at every τ.

## 7. Expectations (written down to be checked, not criteria)

* Completion of every controller without the lag in its model falls with τ.
* `oracle_struct` completes (nearly) every car at every τ.
* The identification absorbs the lag as a less responsive car (smaller tyre
  stiffness, larger yaw inertia or a rearward aero balance); `identified` is
  not better than `oracle_param` at the largest τ.
* The residual mean recovers part of the gap; the residual uncertainty is too
  small on the closed-loop laps (distribution shift).

## 8. Reproduction

`.venv` (Python 3.12) is the reference. In the second environment `.venv-311`
(Python 3.11, numpy 2.4.6, scipy 1.17.1): the lag sweep 1a, and `nmpc` and
`oracle_struct` on the 25 cars at τ = 60 ms. Differences are reported; the main claims must hold in both.

## 9. Compute

About 8–11 min per NMPC lap with 8 processes (L4 and the feasibility check),
15 min per re-plan. Round 1: about 45 laps and 9 re-plans (1a) and 525 laps
and 100 re-plans (1b), 14–16 h on 8 processes.

## Deviations

(none yet)
