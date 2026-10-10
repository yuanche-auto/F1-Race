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
  identification of L4 therefore sees δ_c as the steering angle. This is the
  harder of the two settings: many real cars measure the rack or road-wheel
  angle.
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
| `oracle_struct` | true | yes, true τ | true-car plan, 5 % | **correct-structure reference** (not a strict upper bound: its plan has no lag) |
| `lag_identified` | identified with τ from the `explore` lap (8 unknowns) | yes, identified τ | re-planned, 5 % | the correct structure with τ to be identified (A1) |
| `lag_half`, `lag_double` | nominal | yes, 0.5 τ / 2 τ | 5 % | sensitivity to a wrong time constant (A1; 1a only) |
| `rl` | – | – | 5 % | the selected L5 policy B_s1_500k, unchanged; observes δ_c |

The lag-aware MPCs integrate the lag exactly within each MPC step and
reconstruct the road-wheel angle with an open-loop observer from their own
commands (exact without noise; `LagAwareMPCController`). The plans are the
minimum-time plans without a lag (references for the tracking MPC).
`oracle_param` and `lag_nominal` together with `nmpc` and `oracle_struct` form a
2 × 2 design (parameters × structure); there "structure" means the correct
structure *with the exact τ*, so it overstates what knowing the structure buys
in practice – `lag_identified` and the τ-mismatch runs measure that (A1). A
true upper bound would need a minimum-time plan with the lag (optional, not
planned).

## 4. Runs of round 1

**1a – lag sweep on the nominal car** (the lag is the only error): τ = 5, 10,
15, 20, 30, 45, 60, 90, 120 ms; `tracker`, `nmpc`, `explore`, `smpc`,
`identified`, `lag_nominal` (on the nominal car the same as `oracle_struct`),
`lag_identified`, `lag_half`, `lag_double` and `rl`. Shows up to which lag each
controller works.

**1b – the 25 cars** (lag and parameter errors): τ = 10, 30 and 60 ms (60 ms is
the main level of the later rounds), all controllers of section 3 except
`lag_half` / `lag_double`.

Order of the runs: 1a, then 1b at 60, 30, 10 ms.

Ablation: observation "angle" at τ = 60 ms on the 25 cars for `tracker`, `nmpc`,
`explore`, `identified` (the identification then sees the road-wheel angle)
and `lag_nominal` (the controller keeps its commanded angle from its own rate
commands) (A3).

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
  {0.5, 1, 2, 4, 8, 16} by the evidence – no tuning on laps (A8);
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
distribution the controller actually visits). On a failed closed-loop lap the
samples of the last 2 s before the failure are dropped, and for the k-step
intervals every prediction whose horizon overlaps those 2 s (the definition of
the L3 truncation). **Go** for L7 if, at every τ of round 2, the pooled
coverage of (ii) lies in 90–98 % **and** so does the coverage on the
near-limit samples of (ii) (friction-ellipse usage of the model ≥ 0.85 on
either axle, the threshold of the L3 calibration) (A4); otherwise **no-go**:
L7 may not use this uncertainty directly (it would need recalibration, e.g.
empirical back-offs as in L3 or conformal intervals). Also reported, as a
secondary check: coverage of the k-step lateral-position intervals of
`residual_cautious` (±b_k, k = 10 and 20) on its own laps – the tightening uses
these propagated intervals, not the one-step ones; if they cover less than
90 % while the one-step check passes, the go is conditional (L7 then needs
horizon-calibrated back-offs). Coverage per car is reported too.

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
identification error, not on laps, fixed before round 4 runs). Fixed in A9:
the L3 sensor noise (seed 1000 + car) on the saved 60 ms exploration laps of
round 1; `noisy_lag_ls` (least squares with the lag on the noisy data) and
`noisy_lag_ekf` (the EKF of round 3 extended by the lag: the road-wheel angle a
hidden state, τ estimated with the parameters); each re-planned and driven by
the lag-aware NMPC with its identified τ; the controllers see the noise-free
state. Metrics: τ and parameter errors, completion, lap time against
`lag_identified` and `oracle_struct` of round 1 (paired).

## 6. Metrics and statistics

* Laps completed out of 25 with exact (Clopper–Pearson) 95 % intervals; mean
  lap time of completed laps; lap time on the nominal car; stop reasons.
* Lap time against `oracle_struct` on cars both complete (paired).
* Identification: identified parameters against the true ones per τ,
  parameters on their bounds, residual rms.
* Model quality: mean and 95th percentile of the lateral-position error of the
  MPC predictions at horizon steps 5, 10, 20 and 30 (descriptive).
* Steering: largest gap between commanded and road-wheel angle.
* Pre-registered paired comparisons (exact McNemar, same cars, per τ):
  `oracle_struct`–`oracle_param` (value of the structure), `lag_nominal`–`nmpc`,
  `lag_identified`–`identified`, `oracle_struct`–`lag_identified`,
  `identified`–`nmpc`, `identified`–`oracle_param` (does identification absorb
  the lag?), `oracle_param`–`nmpc`, `smpc`–`nmpc`, `rl`–`nmpc`; round 2:
  `residual`–`identified`, `residual_cautious`–`residual`,
  `oracle_struct`–`residual`, `lag_identified`–`residual`. p-values without
  correction; a conclusion needs p < 0.01 at a single τ, or an exact sign test
  with p < 0.01 on the discordant pairs pooled over the τ levels together with
  the same direction at every τ (A5).

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
(Python 3.11, numpy 2.4.6, scipy 1.17.1): the lag sweep 1a, and `nmpc`,
`oracle_struct` and `identified` on the 25 cars at τ = 60 ms (A6: the
identification and the first re-planning solve are the most
environment-sensitive steps). Differences are reported; the main claims must hold in both.

## 9. Compute

About 8–11 min per NMPC lap with 8 processes (L4 and the feasibility check),
15 min per re-plan. Round 1: about 70 laps and 18 re-plans (1a) and about
700 laps and 200 re-plans (1b with A1 and A3), about 20 h on 8 processes.

## Deviations and amendments

Amendments A1–A7, 2026-10-09, after the reviewer's design review of 75865d8,
made while 1a was running and **before 1b started**:

* **A1** (review MAJOR-1: "knowing the structure" was "knowing the structure
  and the exact τ"): added `lag_identified` (1a and 1b) and `lag_half` /
  `lag_double` (1a). Disclosure: before adding it, the identification with τ
  was tried on the two exploration laps of 1a that had already ended (nominal
  car, 45 and 60 ms; both off track after about 465 m): τ recovered to 0.2 %,
  the parameters to 0.2 %; the identification without the lag put three
  factors on their bounds (inertia, mass, aero balance).
* **A2**: `oracle_struct` is described as the correct-structure reference, not
  an upper bound (its plan has no lag); a plan with the lag is optional.
* **A3**: the "angle" ablation extended to `explore`, `identified` and
  `lag_nominal`.
* **A4**: coverage check – truncation of failed laps fixed (L3 definition),
  near-limit coverage added to the go criterion, k-step position-interval
  coverage added as a secondary check.
* **A5**: statistical rule tightened. It was "p < 0.01 or the same direction at
  every τ" (too loose: 1:0 at every τ would count).
* **A6**: `identified` at 60 ms added to the second-environment check.
* **A7**: the "command" condition is stated to be the harder setting.
* **A8** (2026-10-09, before round 2): the length-scale grid of the residual
  model extended from {0.5, 1, 2, 4} to {0.5, 1, 2, 4, 8, 16}: in a smoke test of
  the code on one 1a exploration lap (nominal car, 90 ms) the evidence chose the
  largest candidate, 4. No lap was driven with a residual model.

Code checks before rounds 2 and 3 (disclosed; nothing was changed because of
them):

* the round-2 job end to end on the nominal car at 30 ms (from its 1a
  exploration lap): `residual` and `residual_cautious` both off track at about
  470 m; held-out one-step coverage 0.62 / 0.74 / 0.69 (v_x', v_y', r'),
  closed-loop 0.87 / 0.94 / 0.96 on 500 samples; 1.1 s per solve (a full lap
  takes about 30 min). This lap is a result of round 2 and will be re-run
  identically in the round;
* the EKF and the smoothed least squares on 25 s of an exploration-like lap of
  car 5 at τ = 0 with one noise draw (not the round-3 data): largest parameter
  error 0.015 (EKF), 0.61 (least squares raw and smoothed: inertia on its
  bound).
* Identification notes: the samples where the plant's tyres are saturated are
  excluded with the plant's true ellipse usage, as in L4 (privileged
  information, disclosed there); the same rule is used by the lag
  identification and the EKF.

* **A9** (2026-10-10, before round 4 ran; round 3 had finished 12 of 25 cars):
  round 4 fixed as written in section 5. The choice of the EKF follows the rule
  of section 5 on the 12 cars available (median largest parameter error 0.006
  against 0.41 smoothed and 0.54 raw least squares; it will be checked on all
  25 cars). Two deviations from "the methods of rounds 2 and 3 composed":
  (1) no identification without the lag – at 60 ms even the true parameters
  without a lag model completed 0 of 25 in round 1; (2) the residual model is
  deferred: round 2 ran during an overload of the machine (A10), its results
  are not yet verified, and whether it is added to round 4 is decided after
  that verification. Code check before the run (disclosed): on 25 s of a
  simulated tracker lap of car 3 with one noise draw, the EKF with the lag found
  τ = 60.3 ms (true 60) and the parameters to 0.009, at 30 ms τ = 26.6 ms and
  0.11; the least squares with the lag 41.7 / 17.2 ms and 0.56.
* **A10** (2026-10-10): overload. After round 1, too many pools ran at once
  (about 18 processes; one minimum-time OCP solve needs about 5.5 GB) and the
  machine ran out of memory between about 03:00 and 09:00: the angle ablation
  failed, round 3 stopped after 12 of 25 cars, the machine froze. Round 1
  (finished before) and the RL runs are unaffected. Since then at most 8
  processes and at most two simultaneous re-plans (`replan_slot`), and a failing
  job no longer aborts a pool. Records with a solver fallback or a retried
  re-plan that were produced during the overload can be affected (a memory
  failure inside IPOPT looks like a solver failure); records without either are
  deterministic and valid. Round 2 ran entirely during the overload. Per the
  user's instruction the unfinished parts are completed first; which of the
  affected records are replayed is decided afterwards. Replays done so far: 6
  round-1 laps at 10 ms, all identical to the originals.
