# Safety-filter solve-time benchmark (before L7)

ROADMAP section 3 / 7: before L7, measure only the single-solve time of the candidate
safety filters and decide which one may run inside RL training and which one is only for
evaluation. This page fixes the candidates, the measurement and the decision rule
**before** any timing is taken; the results are appended below the rule.

## Candidates

1. **CBF-QP** (`control/safety_filter.py`, `CBFQPFilter`): point mass in curvilinear
   coordinates, decision variables (a_x, a_y, σ); two second-order barrier constraints for
   the track edges and a 16-gon inner approximation of the friction circle; the proposed
   command (steering rate, drive, brake) is mapped to accelerations and back. Solvers that
   ship with CasADi 3.8.1: qpOASES, OSQP, DAQP, PROXQP, HiGHS.
2. **NMPC filter** (`FrenetMPC(mode="filter")`): the L2/L6 NMPC (N = 30, 50 ms, soft
   track / ellipse / slip limits, η = 0.95) with the objective replaced by the deviation of
   the first input from the proposed one and the terminal speed bounded by the plan speed;
   variants without and with the steering-lag model (the identified structure of L6), the
   latter on laps with a lag. Warm-started along the lap, as it would run.

## Measurement

* States: closed-loop laps of the selected L5 policy (`B_s1_500k`, the residual RL policy
  that L7 starts from) on (nominal car, no lag), (L2 car 5, τ = 30 ms) and (L2 car 12,
  τ = 60 ms); at every 50 ms decision the state and the command proposed by
  tracker + policy are logged (`scripts/l7_filter_bench.py collect`, `.venv-rl`).
* The proposed command enters the filters after the plant's actuator limits
  (`DynamicSim.apply_limits`: steering rate and angle, engine power), i.e. as it would act.
  (Added after the first smoke runs: the tracker's raw command asks for steering rates
  above 1 rad/s and drive forces above the power limit, which the plant clips later; the
  filters, which respect these limits, then "changed" 62 % of the decisions for that
  reason alone. Timings were not affected.)
* Each filter is run on every logged decision (`bench`): the filter call as the training
  loop would make it (QP: build the matrices, solve, map back; NMPC: parameter vector,
  IPOPT, fallback handling). Reported: mean, median, P95, P99 and max time, solver
  failures, the fraction of decisions where the filter changes the command.
* Two load conditions on this machine (24 cores): one process alone, and 24 processes at
  once (as when three training runs of 8 environments share the machine). Single-threaded
  BLAS, a fresh process per job.

## Decision rule (fixed before the measurement)

The L5 training runs took ~2.5 h for 600 000 decisions: 75 000 steps of 8 environments
that run in parallel subprocesses, ~120 ms per vector step. A filter inside every
environment adds its own time to each vector step (the environments run in parallel).
Budget for L7: about 12 training runs, three at once, within about three days of machine
time, i.e. at most ~2.5× the L5 run time per run.

* **Admissible in training** if, under the 24-process load, the mean call time is at most
  150 ms (≈ +3 h on 75 000 vector steps), the P99 at most 1 s, and at most 1 % of the
  calls fail to solve. If both candidates pass, both may be used in training; the cheaper
  one is the default.
* **Evaluation only** otherwise.
* A filter whose calls fail in more than 1 % of the decisions is reported as not usable
  as it stands, whatever its time.
* Real-time capability (a 50 ms control period) is not decided here; that is L9.

## Results

2026-10-14, three RL laps of `B_s1_500k` (all completed; 1 617 / 1 616 / 1 619 decisions),
`python scripts/l7_filter_bench.py report`. "Alone": one benchmark process at a time
(two or three single-threaded preparation runs were going on other cores). "changed":
decisions where the filter changes the command (steering rate > 0.01 rad/s or a force
> 100 N) – before any calibration of the filters, on laps that complete without them.

| filter | load | calls | mean ms | median | P95 | P99 | max | failed | changed |
|---|---|---|---|---|---|---|---|---|---|
| CBF-QP, qpOASES | alone | 4 852 | 0.46 | 0.41 | 0.71 | 1.09 | 2.5 | 0 % | 40.4 % |
| CBF-QP, OSQP | alone | 4 852 | 0.55 | 0.51 | 0.81 | 1.18 | 4.2 | 0.04 % | 43.1 % |
| CBF-QP, PROXQP | alone | 4 852 | 0.52 | 0.47 | 0.77 | 1.37 | 2.7 | 1.05 % | 40.4 % |
| CBF-QP, HiGHS | alone | 4 852 | 0.91 | 0.81 | 1.27 | 1.57 | 3.7 | 0 % | 40.4 % |
| CBF-QP, DAQP | alone | 4 852 | 0.48 | 0.50 | 0.73 | 1.10 | 2.0 | **100 %** | – |
| NMPC filter, no lag model | alone | 4 852 | 108.8 | 106.9 | 141.3 | 258.5 | 746 | 0.06 % | 26.6 % |
| NMPC filter, lag model (laps with a lag) | alone | 3 235 | 122.9 | 118.3 | 157.2 | 293.5 | 918 | 0.03 % | 26.4 % |

| CBF-QP, qpOASES | 24 processes | 38 816 | 0.71 | 0.64 | 1.48 | 7.13 | 38.1 | 0 % | 40.4 % |
| NMPC filter, no lag model | 24 processes | 4 800 | 219.2 | 200.2 | 343.6 | 754.8 | 1 169 | 0 % | 25.5 % |
| NMPC filter, lag model | 24 processes | 9 600 | 235.3 | 215.5 | 394.4 | 1 077.5 | 1 779 | 0.08 % | 26.4 % |

"24 processes": 24 benchmark processes at once (NMPC: 8 on lap 0 without the lag model,
8 each on laps 1 and 2 with it, the first 600 decisions of each lap; CBF-QP: 8 per lap,
whole laps), with two or three single-threaded preparation runs on the machine as well
(26–27 processes on 24 cores).

* DAQP through CasADi 3.8.1 reports failure on every call (also with finite bounds);
  not investigated further – it is not needed.
* **Not a timing matter, but important for L7:** uncalibrated, the CBF-QP changes ~40 %
  of the decisions of a policy that completes these laps without it (three quarters
  through the edge barrier: the racing line runs along the plan's 1.3 m edge margin), the
  NMPC filter ~26 % (checked on the first 600 decisions of lap 0: the longitudinal forces
  in 23 %, the steering rate in 8 %; in 59 % of the changed decisions the proposal uses
  more than the filter's hard longitudinal friction limit η = 0.95 – the policy brakes and
  drives up to 0.99 –, the terminal speed bound is active in only 5 %). Both are far too
  intrusive as they stand; the protocol fixes their margins on the calibration set before
  any L7 run.

## CBF-QP revised after the review (layer/L7)

The review of 89f81fb found that the QP did not constrain a_y to what the steering can
reach in one step, so a filtered command could be unrealisable exactly when a large
correction is needed, and that the barrier ignored the car's extent and the change of the
track width (`control/safety_filter.py` now: a_y and a_x bounds from the steering rate and
angle limits and the engine power, the friction polygon softened by its own slack, margin
+ l_f |sin ξ_h| + l_f² |κ| / 2, w′ and w″ in the barrier). Re-timed on the same three laps
(qpOASES):

| CBF-QP | load | calls | mean ms | median | P95 | P99 | max | failed | changed |
|---|---|---|---|---|---|---|---|---|---|
| revised | alone | 4 852 | 0.44 | 0.42 | 0.61 | 1.02 | 1.6 | 0 % | 45.6 % |
| revised | 24 processes | 38 816 | 0.68 | 0.59 | 1.36 | 4.53 | 23.4 | 0 % | 45.6 % |

Same verdict. The share of changed decisions rose (45.6 % against 40.4 %), as expected from
the wider margin; it is uncalibrated and is fixed on K (protocol section 4.1).

The review of this version (39b62c4, r-m1) found that w″ from the raw widths (piecewise
linear between points ~5 m apart) has spikes at the points of up to ~350 m/s² in w″ ṡ².
With w′ and w″ from widths smoothed along the lap (periodic Gaussian, 20 m), alone on the
same three laps: 0.58 ms mean, 1.27 ms P99, 0 failures, 42.9 % of the decisions changed
(the reviewer measured 43.3 % with w″ = 0).

## Sensitivity of the decision to the rule

The rule was written before the timings were measured, but not committed separately, and
the smoke runs of each filter (which came first) already showed the NMPC filter at
~125–140 ms. The decision does not depend on the exact thresholds: with the budget stated
above (at most ~2.5× the L5 run time, i.e. +1.5 × 2.5 h = 3.75 h per run), the largest
admissible mean is 3.75 h / 75 000 ≈ 180 ms per call; the NMPC filter's 219–235 ms under
load exceeds it as well, and the CBF-QP's 0.7 ms is far below any threshold that could be
argued for. (Review of 89f81fb.)

## Decision (by the rule above)

| filter | mean under load | P99 under load | failed | verdict |
|---|---|---|---|---|
| CBF-QP (qpOASES) | 0.71 ms ≤ 150 ms | 7.1 ms ≤ 1 s | 0 % ≤ 1 % | **admissible in training** (default solver qpOASES; OSQP and HiGHS also usable, PROXQP fails 1.05 % – just above the limit) |
| NMPC filter | 219–235 ms > 150 ms | 0.75–1.08 s | ≤ 0.08 % | **evaluation only** |

At ~120 ms per vector step of the L5 training, the CBF-QP adds well under 1 % to a
training run; the NMPC filter would add ~4.6–4.9 h per run (75 000 vector steps at
~220–235 ms), beyond the budget. The NMPC filter in this form is not real-time at a 50 ms
control period (its mean is ~2× the period even alone; the CBF-QP is far below it) – that
is L9's question.
