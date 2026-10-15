# L7 protocol – reliable learning: diagnosis, a safety layer, and whether reliability transfers to the policy

Written before any result-producing run of L7 (2026-10-14); revision 1 after the design
review of d1fa964 (CHANGES_REQUESTED), confirmed by the reviewer on 39b62c4 (APPROVE) with
three minors fixed in revision 1.1 (see the end). Binding from revision 1.1; deviations after that are listed at the end as
amendments with their date and reason, and the runs they affect are named. Roadmap:
[ROADMAP_L6-L10.md](ROADMAP_L6-L10.md) (section 4, L7, and section 7, the strategy update
after L6).

**What was run before this protocol** (preparation, not L7 results):

* the reproduction of the L6 numerical failures and the robustness changes of the NMPC and
  the re-plan ([prep_robustness.md](prep_robustness.md));
* the solve-time benchmark of the two candidate safety filters on three closed-loop laps of
  the selected L5 policy ([L7_filter_benchmark.md](L7_filter_benchmark.md)).

**Test laps touched before this protocol (p-m2 of the review).** All on cars of the test set
T1 below: NMPC laps of the preparation on the nominal car (τ 120 ms – not a T1 condition),
cars 4, 18, 20, 21 at 30 / 60 ms and cars 20, 21 at τ = 0 (EKF plans); RL laps of
`B_s1_500k` on the nominal car (τ = 0), car 5 (30 ms) and car 12 (60 ms) for the
benchmark. No outcome of them enters L7. The CBF calibration rule of section 4.1 (the α
grid, the 10 % threshold) was written after seeing that the uncalibrated filters change
40 % / 26 % of the decisions on those three RL laps; no parameter is set on them – α and
every back-off are fixed on the calibration set K.

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
| validation V | 10 cars `sample_plants(nominal, 10, seed 1)` (no nominal car) × τ ∈ {0, 30, 60 ms}, + 10 disturbance seeds 2000–2009 | 40 | checkpoint selection (secondary results only); S2 |
| curriculum C | 10 cars `sample_plants(nominal, 10, seed 9)` × τ ∈ {0, 30, 60 ms}, + disturbance seeds 5000–5009 | 40 | failure positions for the episode starts of F and M (section 5) – so that V stays independent of their training |
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
under the load of 24 processes: CBF-QP 0.68 ms mean, 4.5 ms P99 (revised version);
NMPC filter 219–235 ms mean, 0.75–1.08 s P99). Roles (P-M2 of the review):

* **The main filter of Part B is the NMPC filter** (`FrenetMPC(mode="filter")`, N = 30,
  50 ms, with the steering-lag model where the risk estimate provides τ): it can carry
  every risk estimate of section 4.2 as it is defined (a model, per-stage back-offs and
  friction factors), and the acceptance test is made on its own predictions. **Evaluation
  only** – it misses the training budget (~+4.6–4.9 h per run). "With the filter" in H-B,
  S3 and section 6 means this filter with the main risk estimate. As benchmarked (η = 0.95,
  no back-offs) it changes ~26 % of the decisions, mostly the longitudinal force where the
  policy uses more than its hard friction limit; its η_k and back-offs come from the risk
  estimate, not from this default.
* **The CBF-QP is the training filter** (`control/safety_filter.py`, qpOASES; Part C) and
  the **secondary** evaluation filter (reported next to the main one, no hypothesis rests
  on it). It was revised after the review (P-M1): a_y and a_x are bounded by what the
  steering (rate and angle limits within one 10 ms step) and the engine can reach, the
  friction polygon has its own slack, and the barrier margin includes the front axle swung
  out by the heading angle, the chord of the curve and the change of the track width.
  Uncalibrated (plan margin 0.3 m, α = 2) it changes ~46 % of the decisions of the L5
  policy on laps that complete without it, mostly through the edge barrier – the racing
  line runs along that margin. Its free parameters are fixed on K before any other L7 run
  (after the back-off of (b), section 4.2): margin = the back-off of (b) at k = 10 (instead
  of the plan's 0.3 m; the car's half width and extent are added inside the filter),
  friction factor η_10 of (b), and α = the smallest value in {1, 2, 4, 8} for which the
  filtered `B_s1_500k` changes at most 10 % of the decisions on the K laps that
  `B_s1_500k` completes without the filter (if none does: α = 8, reported). The band of (b)
  is calibrated on the NMPC filter's predictions, not on the point-mass model's: for the
  CBF-QP it is a heuristic transfer, and the CBF-QP has no acceptance test of its own – it
  is judged by its outcomes (completion, interventions, false interventions) only.

Both filter the command of tracker + policy at every 50 ms decision (the command proposed
for the first 10 ms step; the filtered command replaces it for the decision). **In training
and in evaluation the filters receive the command after the plant's actuator limits**
(`DynamicSim.apply_limits` / `LaggedSim.apply_limits`: steering rate and angle, engine
power), as in the benchmark – otherwise the tracker's raw rate and drive commands, which
the plant clips anyway, would count as interventions (62 % in the benchmark's smoke runs),
and M's intervention cost and every intervention rate would include them (r-m3). Every
NMPC-based controller and filter of L7 uses the NMPC failure handling chosen by the rule
in [prep_robustness.md](prep_robustness.md) section 4.3 – the L6 settings (for the rule's
timing see there) – and every re-plan `replan_robust`; fallbacks are counted and reported.

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

**Closed loops used for calibration and acceptance** (P-M2 (iii)): on K and on A the laps
are driven by `B_s1_500k` as the proposer with the NMPC filter. For (b) and (c) the K laps
run the filter with its default (nominal model, η = 0.95, no back-offs); its k-step
predictions against the realised path give the samples of (b) and the calibration errors
of (c). For (a) the identification uses each car's own exploration lap.

**Acceptance test (the L6 go criterion, kept):** on the laps of the acceptance set A,
driven by `B_s1_500k` with the NMPC filter *using the estimate under test* (its model, η_k
and back-offs), the 95 % band covers 90–98 % of the filter's own k-step lateral prediction
errors **both at k = 10 and at k = 20** (p-m4); coverage near the friction limit (usage ≥
0.85) is reported alongside. A neither identifies nor calibrates. All three estimates are
computed and tested.

**The main risk estimate is the first in the order (a), (b), (c) that passes; the main
filter uses it**; the others are reported as secondary. If none passes, the main filter
is still evaluated with (b) and every result is labelled "risk estimate not accepted".

In training (Part C) the filter uses (b), the only estimate available per episode without
an exploration lap, whether or not it is the main one.

### 4.3 Evaluation of Part B

Policies: the nine final L5 checkpoints (unselected) and the selected `B_s1_500k`.

**Pre-registered test of H-B (primary, P-M2 (ii)):** over the nine final checkpoints, the
paired numbers of completed T1 + T2 laps with and without the main filter; H-B holds if the
Wilcoxon signed-rank test (two-sided, zero differences dropped) is significant at the
level given in section 7 and the median difference is positive. (With n non-zero
differences the smallest attainable two-sided p is 2/2ⁿ: n = 6 gives 0.031, n = 7 gives
0.016. H-B is reported as not shown if n < 7 when Holm tests it at 0.025, or n < 6 when it
is tested at 0.05.) Per policy: exact McNemar with /
without (secondary, all ten reported).

Baselines on the same laps: the bare tracker under the main filter (no learning), the
lag-aware NMPC with τ and parameters identified (the best model-based controller of L6;
L6 failure handling, plans by `replan_robust`), the L3 SMPC, the nominal NMPC. Each on
T1 + T2 (105 laps), without and with the main filter, and with the CBF-QP (secondary); the
secondary risk estimates where the compute allows (listed in the report).

## 5. Part C – training from failures and interventions

Three groups – **P and M with five training seeds each, F with three** (13 runs; P-M3 of
the review: the L5 seeds of one group differed from 19 to 50 of 51 completed laps, so two
median seeds of three could differ by seed luck alone) – all with the L5 group-B settings
(SAC, history of 8 observations + last action, 40 s episodes, steering offset ±0.02 rad, speed
factor 0.85–1.05, 600 000 decisions in 8 parallel environments, a checkpoint every 50 000)
and the **L7 training distribution**: cars from the L2 ranges; in the four environments
without L3 disturbances a steering lag τ = 0 with probability 1/3, otherwise τ ~ U(10,
80 ms), the policy seeing the commanded angle; in the other four the L3 disturbances
(no lag) as in L5.

| group | what is fed back |
|---|---|
| **P** plain | nothing (the control: L5 training on the L7 distribution) |
| **F** failure starts | 30 % of the episodes start 50–150 m before a recent failure position: failures of the curriculum laps C run without filter at every checkpoint, and crash points of training episodes; the last 200 positions |
| **M** method | F, plus the training filter (CBF-QP, calibrated as in section 4.1) in the loop: the environment applies the filtered command, the reward has a cost 0.5 · (changed decision) per intervention, and the positions of interventions join the failure positions for the episode starts |

F separates the curriculum from the filter. The cost weight 0.5 (about the progress
reward of one decision at racing speed: 2.5 m / 5) and the 30 % are fixed here, not tuned.

**Pre-registered success criteria** (on T1 + T2, 105 laps):

* **S1 (primary): the policy without the filter.** The median seed of M (of five; ranking
  rule of L5: completed test laps without filter, ties by lower mean lap time, then lower
  seed), final checkpoint, completes significantly more test laps *without the filter*
  than the median seed of P (of five) – exact McNemar, two-sided, at the level of section 7.
  Reported with it, to show the scope: the 5 × 5 pairwise comparisons of the final
  checkpoints (how many M seeds beat how many P seeds without the filter).
* **S2: interventions fall during training.** On the validation laps V with the training
  filter, the intervention rate of the M checkpoints decreases with training (Spearman
  ρ < 0, p < 0.05) for at least 3 of the 5 seeds.
* **S3: not worse than the filtered MPC alone.** The median seed of M *with the main
  filter* completes at least as many test laps as the bare tracker under the main filter
  and, on the laps both complete, is not slower than the lag-aware NMPC baseline by more
  than the RL gap of L5 and L6: 4.6 s per lap (L5 section 3: the selected policy 4.6 s
  slower than the nominal NMPC on the L2 cars and L3 seeds both complete; L6 section 3: the
  residual RL policy 4.3–4.4 s slower than the correct-structure reference; the larger is
  taken). (The
  ROADMAP's example "not slower than the safety-filtered MPC alone" is made concrete this
  way: "not slower" literally would fail before any run.)

L7's method claim holds if S1 and S2 hold; S3 is reported. Secondary: F vs P (S1 with F
in place of M), all final checkpoints (the unselected distribution, reported first),
the validation-selected checkpoint per group, M vs P with the main filter.

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
* **decisions the filter cannot guarantee**, separately from solver failures (p-m5): the
  NMPC filter is softly constrained, so a converged solve may still use track or friction
  slack (> 1e-3) – reported as a rate and with positions; for the CBF-QP the decisions with
  σ_edge or σ_friction > 0;
* **computation time** of the filter per call (mean, P95, P99, max) under the load of the
  run, and the number of solver failures (NMPC: fallbacks to the shifted plan);
* the comparison with the model-based baselines (section 4.3) on the same laps.

## 7. Statistics

Exact McNemar tests on paired lap outcomes, exact binomial (Clopper–Pearson) 95 %
intervals, paired lap-time differences on laps both complete with the Wilcoxon signed-rank
test. A relative perturbation of 1e-9 of the initial state changes an NMPC lap by up to
0.34 s (preparation, car 4 at 60 ms), so lap-time differences of single laps below about
0.4 s are not interpreted; lap-time statements rest on means and paired tests over laps.
Two primary tests: H-B (section 4.3) and S1 (section 5), with Holm's correction over the
two (family-wise α = 0.05: the smaller p value is tested at 0.025, the other then at 0.05).
Everything else is reported with its p value and the number of comparisons made, without
a claim of significance across the family.

## 8. Second environment

A Python 3.11 environment with the RL extras (`.venv-311-rl`, built from the lock file
with Python 3.11 versions; to be set up before the first main result) re-runs: the
acceptance test of the main risk estimate; T1 + T2 with and without the main filter for
`B_s1_500k` and for the median seeds of P and M (S1). Training itself is not bit-reproducible
(parallel environments); evaluation is, and differences are reported as in L6 section 7.

## 9. Compute and order

1. Part A (CPU, RL evaluation only; ~1 day including the report).
2. Risk estimates (a)–(c) and their acceptance test; Part B evaluation (~1–2 days: the
   NMPC filter costs about 0.15–0.3 s per decision, ~1 600 decisions per lap).
3. Part C training (13 runs, three at a time on 24 cores, ~2.5–3 h each: ~13–15 h; the
   CBF-QP adds < 1 %), validation and curriculum laps at every checkpoint, test evaluation.
   The laps at the checkpoints: an RL lap takes ~2 s here (measured, `B_s1_500k`, one
   process), ~3 s with the CBF-QP (1 600 calls × 0.6 ms); V (40 laps) for all 13 runs, C
   (40) for F and M (8 runs), V with the training filter for M (S2, 5 runs), each at 12
   checkpoints: 13·12·40·2 s + 8·12·40·2 s + 5·12·40·3 s ≈ 7.6 CPU-h; per run, if the laps
   run inside the training process: P ~0.27 h, F ~0.53 h, M ~0.93 h (amendment A1).
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

## Revisions before the protocol is binding

* **Revision 1 (2026-10-14), design review of d1fa964:** P-M1 the CBF-QP made realisable
  (steering / engine reach, softened friction, axle swing, chord, w′ / w″) and re-timed;
  P-M2 the NMPC filter is the main filter of Part B, the CBF-QP the training and secondary
  filter; a pre-registered test of H-B (co-primary with S1, Holm); the closed loops for K
  and A named; P-M3 five seeds for P and M; p-m1 a separate curriculum set C; p-m2 the test
  laps touched in the preparation listed; p-m3 the CBF-QP's band declared a heuristic;
  p-m4 k = 10 and 20 both in the acceptance band; p-m5 slack-using decisions reported
  apart from solver failures; p-m6 the reference to the prep rule corrected; nit: the
  source of the 4.6 s gap.

* **Revision 1.1 (2026-10-14), confirmation of 39b62c4 (APPROVE) with three minors to fix
  before the first K run:** r-m1 the CBF-QP's w′ / w″ from widths smoothed along the lap
  (the raw piecewise-linear widths gave spikes of up to ~350 m/s² in w″ ṡ²; test added;
  changed decisions on the benchmark laps 45.6 % → 42.9 %); r-m2 the attainability of the
  H-B test corrected (n < 7 at 0.025, n < 6 at 0.05); r-m3 the filters see the command after
  the actuator limits, in training and evaluation; nit: the cost of the checkpoint laps in
  section 9. **The protocol is binding from this revision on.**

## Amendments

* **A1 (2026-10-14), after the confirmation of 58ad42e (reviewer's nit 1):** section 9, the
  cost of the checkpoint laps per run given per group (the total of 7.6 CPU-h was right,
  "0.5–0.8 h per run" was too low for M). No run affected. Also nit 2: the width test now
  asserts max |smoothed − raw width| < 1 m directly (code: `width_derivatives` returns the
  smoothed width as well; the filter's numbers are unchanged).
