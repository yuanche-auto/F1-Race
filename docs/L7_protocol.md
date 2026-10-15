# L7 protocol – reliable learning: diagnosis, a safety layer, and whether reliability transfers to the policy

Draft, written before any result-producing run of L7 (2026-10-14). It becomes binding
when the reviewer has confirmed it; deviations after that are listed at the end as
amendments with their date and reason, and the runs they affect are named. Roadmap:
[ROADMAP_L6-L10.md](ROADMAP_L6-L10.md) (section 4, L7, and section 7, the strategy update
after L6).

**What was run before this protocol** (preparation, not L7 results):

* the reproduction of the L6 numerical failures and the robustness changes of the NMPC and
  the re-plan ([prep_robustness.md](prep_robustness.md));
* the solve-time benchmark of the two candidate safety filters on three closed-loop laps of
  the selected L5 policy ([L7_filter_benchmark.md](L7_filter_benchmark.md); its decision
  rule was written before its measurements). These laps are the only RL laps run so far on
  this machine; they are on the nominal car and L2 cars 5 and 12 of the L6 set, which are
  also test cars below – they were used for timing only, no outcome of them enters L7.

## 1. Question and hypotheses

*When both the model and the learned policy can be wrong, can the control be reliable –
and does the reliability come from the safety layer or from the policy?*

* **H-B (safety layer):** a safety filter with a calibrated risk estimate raises the
  completion rate of the L5 policies on the L7 test set, at a lap-time cost that is
  reported.
* **H-C (transfer, the candidate method):** training that feeds failures and filter
  interventions back (section 5) makes the policy *without* the filter more reliable than
  plain training under otherwise identical conditions.

H-C is the falsifiable claim. If it fails while H-B holds, the conclusion is pre-written:
*the safety layer works but does not transfer reliability to the policy* – a result that
is reported as such.

The combination of a disturbance observer, residual model learning and control barrier
functions for safe RL is published with an F1/10 car (Kalaria, Lin, Dolan, IROS 2025;
ROADMAP section 5). L7 does not claim that combination; what it adds is the controlled
measurement of where the reliability comes from (filter on/off, unselected seeds, false
interventions, against the model-based baselines of L2–L6). The literature check is
repeated before the L7 report.

## 2. Conditions, test, validation and calibration sets

All laps: Spielberg, flying lap from the start of the 5 % plan, 10 ms simulator step,
decisions every 50 ms, single-threaded BLAS, a fresh process per job, result runs from a
worktree pinned to one commit (`results/L7/RUN_COMMIT`).

| set | content | laps | used for |
|---|---|---|---|
| **test T1** | the 25 cars of L4/L6 (`sample_plants(nominal, 24, seed 0)`) × steering lag τ ∈ {0, 30, 60 ms}, controllers see their own commanded angle (L6 main condition) | 75 | all conclusions |
| **test T2** | nominal car, L3 disturbances, seeds 0–29 | 30 | all conclusions |
| validation V | 10 cars `sample_plants(nominal, 10, seed 1)` (no nominal car) × τ ∈ {0, 30, 60 ms}, + 10 disturbance seeds 2000–2009 | 40 | checkpoint selection (secondary results only) |
| calibration K | 10 cars `sample_plants(nominal, 10, seed 7)` × τ ∈ {0, 30, 60 ms}, + disturbance seeds 3000–3009 | 40 | risk estimates (b) and (c) |
| acceptance A | 5 cars `sample_plants(nominal, 5, seed 8)` × τ ∈ {0, 30, 60 ms}, + disturbance seeds 4000–4004 | 20 | the acceptance test of each risk estimate |

The sets are disjoint from each other and from the training distribution's samples
(training draws cars and disturbances from the L2/L3 ranges with its own seeds). V extends
the L5 validation set by the lag conditions (L5 policies were selected on its 20 laps
without a lag; for them the old selection stands). The test laps are run only after
everything they evaluate is fixed.

## 3. Part A – diagnosis of the nine L5 runs (no new training)

Mechanism first, method second. Data available on this machine: the validation outcomes
of all 108 checkpoints (`results/L5/multiseed/validation.json`: 9 runs × 12 checkpoints ×
20 laps, with failure positions), the test outcomes of the final and selected checkpoints
(`test.json`, 81 laps each), and the policies of the final checkpoints of the nine runs
(`results/L5/multiseed/*.zip`) and of the three selected ones (`models/L5`). **Gap:** the
intermediate checkpoints of the nine runs were not restored from the old machine, so the
state-level analyses (A2–A4) are on the final and selected checkpoints only; the movement
of the blind spots during training (A1) is on the logged validation outcomes.

* **A0 reproduction.** Re-drive the final checkpoints on the L5 test laps they failed and
  on as many completed ones (deterministic evaluation); report agreement with `test.json`
  (outcome and failure position within 5 m). Disagreements are reported and the analysis
  uses the re-driven laps.
* **A1 blind spots and their movement.** Failure position along the lap for every failed
  validation / test lap. A *blind spot* of a checkpoint: a 100 m window that holds at least
  half of its failures, if it has at least 3 failures. Reported per run: the blind spot of
  each checkpoint over training (50k … 600k), whether it stays, moves or disappears, and
  whether the windows of different runs coincide (the same corner for several runs would
  point to the track, different corners to the run).
* **A2 states before the failures.** For each failed test lap of a final checkpoint: the
  last 3 s before the failure against the same 100 m window on the completed laps of the
  same checkpoint: speed relative to the plan, lateral error, front / rear slip angle
  relative to the tyre peak, friction usage, the speed factor and steering offset chosen.
* **A3 saturated actions.** Fraction of decisions with |a_i| ≥ 0.99 (steering offset,
  speed factor) in the 3 s before failures, in the blind-spot window on completed laps, and
  elsewhere.
* **A4 critic error.** Along the re-driven laps: the critic's estimate min(Q₁, Q₂)(s, a)
  against the realised discounted return (γ = 0.99, the training reward incl. the crash
  penalty; truncated at the lap end); the error in the last 3 s before a failure against
  elsewhere. Expected if the blind spot is a value error: Q overestimates there (the crash
  is not anticipated).

A has no pass/fail criterion; it is reported before Part C starts. Part C's design is
fixed in this protocol now; if the diagnosis points to a mechanism that Part C does not
address, a change is written as an amendment, reviewed, and labelled as decided after
seeing A.

## 4. Part B – the safety layer

### 4.1 Filters

Decided by the solve-time benchmark ([L7_filter_benchmark.md](L7_filter_benchmark.md);
under the load of 24 processes: CBF-QP 0.71 ms mean, 7 ms P99; NMPC filter 219–235 ms
mean, 0.75–1.08 s P99):

* **CBF-QP** (`control/safety_filter.py`, qpOASES): **admissible in training**; always
  evaluated. As benchmarked (barrier at the plan's own edge margin of 1.3 m, α = 2)
  it changes ~40 % of the decisions of the L5 policy on laps that complete without it,
  three quarters of them through the edge barrier – the racing line runs along that
  margin. Its two free parameters are therefore fixed on the calibration set K before
  any other L7 run, by this rule: barrier margin = half the car's width plus the risk
  estimate's back-off (not the plan's 0.3 m), and α = the smallest value in {1, 2, 4, 8}
  for which the filtered `B_s1_500k` changes at most 10 % of the decisions on the K laps
  that `B_s1_500k` completes without the filter (if none does: α = 8, reported);
* **NMPC filter** (`FrenetMPC(mode="filter")`, N = 30, 50 ms, with the steering-lag model
  where the risk estimate provides τ): **evaluation only** (it misses the training budget:
  ~+4.6–4.9 h per run). As benchmarked it changes ~26 % of the decisions, mostly the
  longitudinal force where the policy uses more than its hard friction limit η = 0.95;
  its η and back-offs come from the risk estimate (section 4.2), not from this default.

Both filter the command of tracker + policy at every 50 ms decision (the command proposed
for the first 10 ms step; the filtered command replaces it for the decision). Every
NMPC-based controller and filter of L7 uses the NMPC failure handling chosen by the rule
in [prep_robustness.md](prep_robustness.md) section 4 (written before the comparison it
decides: the L6 settings), every re-plan `replan_robust`; fallbacks are counted and
reported.

### 4.2 Risk estimate (the filter's uncertainty), in the order of ROADMAP section 7

The risk estimate is a band δn(k) on the predicted lateral position at prediction step
k = 1 … 30 (50 ms each) and a friction factor η; the NMPC filter uses δn(k) as per-stage
track back-offs and η_k as per-stage friction factors, the CBF-QP the band at k = 10
(0.5 s) as its back-off and η_10 as its friction factor.

* **(a) identified structure.** Per car and condition: one exploration lap (L6 `explore`,
  12 % reserve), τ and the seven parameter factors identified from its first two thirds
  (as logged, possibly cut short by a failure) – by `identify_with_lag` (L6 round 1) where
  the measurements are noise-free (T1, K, A without disturbances), by the EKF with the lag
  (`ekf_identify_lag`, L6 round 4) where they carry the L3 measurement noise (T2 and the
  disturbance laps of K and A); the filter's model is the identified car with the
  identified lag; δn(k) = the 95 % quantile of the k-step lateral prediction errors of this
  model on the last third. Not available in training (one exploration lap per episode is
  too costly); evaluation only.
* **(b) L3-type calibrated back-offs.** Nominal model (no lag); δn(k) and the per-stage
  friction factors η_k from the L3 calibration procedure (`experiments/l3.calibrate`,
  ε = 0.05, the L3 near-limit threshold and truncation before failures) re-run on the
  closed-loop predictions of the NMPC filter on the calibration set K (with lags).
* **(c) conformal residual uncertainty.** The L6 residual model (random-feature Bayesian
  regression on the exploration lap); its predictive k-step spread σ(k) scaled by the
  split-conformal factor q̂ = the ⌈0.95 (n + 1)⌉-th smallest of |error| / σ over the n
  calibration predictions on K; δn(k) = q̂ σ(k). The residual *mean* is not used in the
  dynamics unless round 2 of L6 has first been replayed at 10 ms (ROADMAP section 7 point
  1; not planned).

**Acceptance test (the L6 go criterion, kept):** the 95 % band covers 90–98 % of the
k-step lateral prediction errors (k = 10 and 20) on the closed-loop laps of the acceptance
set A, which neither identifies nor calibrates; coverage near the friction limit (usage ≥
0.85) is reported alongside. All three estimates are computed and tested (offline, cheap).
**The main L7 filter uses the first estimate in the order (a), (b), (c) that passes**; the
others are reported as secondary. If none passes, the filters are still evaluated with (b)
and every result is labelled "risk estimate not accepted".

In training (Part C) the filter uses (b), the only estimate available per episode without
an exploration lap, whether or not it is the main one.

### 4.3 Evaluation of Part B

Policies: the nine final L5 checkpoints (unselected) and the selected `B_s1_500k`.
Baselines on the same laps: the bare tracker under the same filter (no learning), the
lag-aware NMPC with τ and parameters identified (the best model-based controller of L6;
L6 failure handling, plans by `replan_robust`), the L3 SMPC, the nominal NMPC. Each on T1 + T2 (105 laps),
without and with the main filter (and with the secondary filter and estimates where the
compute allows; listed in the report).

## 5. Part C – training from failures and interventions

Three groups, three training seeds each (9 runs), all with the L5 group-B settings (SAC,
history of 8 observations + last action, 40 s episodes, steering offset ±0.02 rad, speed
factor 0.85–1.05, 600 000 decisions in 8 parallel environments, a checkpoint every 50 000)
and the **L7 training distribution**: cars from the L2 ranges; in the four environments
without L3 disturbances a steering lag τ = 0 with probability 1/3, otherwise τ ~ U(10,
80 ms), the policy seeing the commanded angle; in the other four the L3 disturbances
(no lag) as in L5.

| group | what is fed back |
|---|---|
| **P** plain | nothing (the control: L5 training on the L7 distribution) |
| **F** failure starts | 30 % of the episodes start 50–150 m before a recent failure position: failures of the validation laps run at every checkpoint (V, without filter) and crash points of training episodes, the last 200 positions |
| **M** method | F, plus the training filter in the loop: the environment applies the filtered command, the reward has a cost 0.5 · (changed decision) per intervention, and the positions of interventions join the failure positions for the episode starts |

F separates the curriculum from the filter. The cost weight 0.5 (about the progress
reward of one decision at racing speed: 2.5 m / 5) and the 30 % are fixed here, not tuned.

**Pre-registered success criteria** (on T1 + T2, 105 laps):

* **S1 (primary): the policy without the filter.** The median seed of M (ranking rule of
  L5: completed test laps without filter, ties by lower mean lap time, then lower seed),
  final checkpoint, completes significantly more test laps *without the filter* than the
  median seed of P (exact McNemar, two-sided, α = 0.05).
* **S2: interventions fall during training.** On the validation laps with the filter, the
  intervention rate of the M checkpoints decreases with training (Spearman ρ < 0, p < 0.05)
  for at least 2 of the 3 seeds.
* **S3: not worse than the filtered MPC alone.** The median seed of M *with* the filter
  completes at least as many test laps as the bare tracker under the same filter and, on
  the laps both complete, is not slower than the lag-aware NMPC baseline by more than its
  L5/L6 gap (4.5 s per lap). (The ROADMAP's example "not slower than the safety-filtered
  MPC alone" is made concrete this way: the RL policies were 4.3–4.6 s slower than the
  NMPC in L5 and L6, so "not slower" literally would fail before any run.)

L7's method claim holds if S1 and S2 hold; S3 is reported. Secondary: F vs P (S1 with F
in place of M), all nine final checkpoints (the unselected distribution, reported first),
the validation-selected checkpoint per group, M vs P with the filter.

## 6. Always reported

For every policy and baseline, on T1 and T2 separately and together:

* completion **without and with the filter** (exact 95 % intervals) and lap time;
* **intervention rate**: fraction of decisions where the filter changes the command
  (steering rate by > 0.01 rad/s or a force by > 100 N), and the positions along the lap;
* **false interventions**: for a sample of up to 200 interventions per policy (all if
  fewer), the unfiltered policy is rolled out from the state at the intervention for 3 s;
  if it neither leaves the track nor spins, the intervention counts as false. Rate and
  interval reported;
* **lap-time cost of the filter**: lap with minus lap without, on laps both complete;
* **computation time** of the filter per call (mean, P95, P99, max) under the load of the
  run, and the number of solver fallbacks;
* the comparison with the model-based baselines (section 4.3) on the same laps.

## 7. Statistics

Exact McNemar tests on paired lap outcomes, exact binomial (Clopper–Pearson) 95 %
intervals, paired lap-time differences on laps both complete with the Wilcoxon signed-rank
test. A relative perturbation of 1e-9 of the initial state changes an NMPC lap by up to
0.34 s (preparation, car 4 at 60 ms), so lap-time differences of single laps below about
0.4 s are not interpreted; lap-time statements rest on means and paired tests over laps.
S1 is the only primary test; everything else is reported with its p value and the number
of comparisons made, without a claim of significance across the family.

## 8. Second environment

A Python 3.11 environment with the RL extras (`.venv-311-rl`, built from the lock file
with Python 3.11 versions; to be set up before the first main result) re-runs: the
acceptance test of the main risk estimate; T1 + T2 with and without the main filter for
`B_s1_500k` and for the median seeds of P and M. Training itself is not bit-reproducible
(parallel environments); evaluation is, and differences are reported as in L6 section 7.

## 9. Compute and order

1. Part A (CPU, RL evaluation only; ~1 day including the report).
2. Risk estimates (a)–(c) and their acceptance test; Part B evaluation (~1–2 days: the
   NMPC filter costs about 0.15–0.3 s per decision, ~1 600 decisions per lap).
3. Part C training (9 runs, three at a time on 24 cores; ~1–2 days depending on the
   training filter), validation of all checkpoints, test evaluation.
4. Second environment, report, review.

Runs that make re-plans respect the machine-wide re-plan slots (`replan_slot`, 4 here).

## 10. Known limitations, fixed in advance

* One track, one structural error (the steering lag), the L2/L3 ranges of cars and
  disturbances; the training distribution contains the test conditions' kinds of error
  (different samples), the model-based baselines get no such prior (as in L5).
* Round 2 of L6 is unverified; no L7 decision rests on its laps.
* The L6 numerical failures depend on rounding (preparation); fallbacks are counted in
  every NMPC-based lap of L7.
* The intermediate L5 checkpoints are missing (Part A).

## Amendments

(none yet)
