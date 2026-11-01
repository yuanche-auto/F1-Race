# CX-E protocol – learning MPC, lap to lap (control expansion, phase E)

Status: **binding, revision 0 (2026-10-22; d1c2043, APPROVE)** with the clarifications of its review (m1–m4, n1, n2;
marked *"clarification of revision 0"*), written down before any phase-E code. **Closed by amendment E1 (option 1):** the
development gate failed (section 9); phase E ends without the benchmark – **H-E1 and H-E2 are not tested**; no CEV, CET or
secondary lap was run. Revision 0 was written after the literature
check ([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §E) and before any phase-E code: no module, test,
runner or lap of phase E existed (section 9). Branch `cx/E-lmpc` (from `main` ae87f41, the merge of phase D).
Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

## 0. What L0–L9 and phases A–D left

* Every controller so far drives **one lap with what it knows before the lap**: the nominal model and an offline plan
  (`nmpc`), a tube from the parameter ranges (phase C), an estimate updated during the lap (phase D), or one exploration
  lap, an identification and a full re-plan (`identified`, L4 / L8). None uses the laps it has driven to drive the next
  one faster. On new L2 cars `identified` completes every lap (phase D's CDT: 50 of 50, 78.0 s), and the online
  `adaptive` controller of phase D is limited by its friction-only plan (its oracle: 42 of 50).
* Learning MPC (literature E.1, E.2, E.4) is the iterative counterpart: each lap's closed-loop data become a safe set and
  a cost-to-go for the next lap, so the car is driven by its own, demonstrated trajectories – not by a plan of a model –
  and is meant to get faster lap by lap. Whether that holds on these cars, with their model mismatch, is the question.

## 1. Question and hypotheses

*Does a learning MPC – the NMPC's model and constraints with a terminal set and cost-to-go built from the car's own
previous laps, and a local regression of the model's error – reach a lap time on new L2 cars after ten learning laps
that differs from the one-lap identification with a full re-plan (`identified`), and does it complete as many laps?*

* **H-E1 (primary):** on the test cars CET (section 2), the lap time of `lmpc`'s tenth learning lap and `identified`'s
  lap differ, on the cars both complete. Exact signed-rank (mid-ranks, exact distribution), two-sided.
* **H-E2 (primary):** `lmpc`'s tenth learning lap and `identified` differ in the number of completed laps on CET. Exact
  McNemar, two-sided.

Holm over H-E1 and H-E2 (α 0.05), each at 0.025 alongside. No direction is predicted. Power: H-E1 has up to 50 pairs;
for H-E2 at least 7 discordant cars, all one way, are needed (`identified` completed 50 of 50 phase-D cars, so H-E2 may
well be untestable in one direction – reported as it comes).

*Clarification of revision 0 (review m1):* **the exposure of H-E2 is unequal.** `lmpc`'s tenth learning lap is completed
only if its two initialisation laps and all ten learning laps run without leaving the track (a failure ends the car's
sequence, section 2) – ten learning laps in a row – while `identified` has one scored lap (its exploration lap's failure
is counted apart, as L8). H-E2 is therefore systematically against `lmpc`; this is intended (a crash while learning is a
failure), and section 5 reports the survival curve and the per-lap completion rates.
*Clarification of revision 0 (review m2):* **H-E1 is conditional** on both completing; with `identified` near 50 of 50,
these are essentially the cars on which `lmpc` survives its ten learning laps – possibly the easier cars. Section 5
describes how the cars `lmpc` completes and those it does not differ (in `identified`'s lap time and in the car's
parameters).

Scope: Spielberg, the nominal model, η = 0.95 (the friction margin of every NMPC here), N = 30 × 50 ms, IPOPT, offline
(no time budget). **Information budget:** `lmpc` gets the track, the nominal model, the measured state, its own inputs and
**every lap it has driven on this car** – two initialisation laps and the learning laps before the current one (up to 11
laps for the tenth); `identified` gets one exploration lap and an offline minimum-time re-plan; `nmpc` gets none of these.
The comparison is between a controller that learns over many laps and one that identifies from one lap – not an equal
budget; the lap-time curve over the learning laps (section 5) shows where `lmpc` passes `identified`, if it does.

## 2. Sets

* **Tuning set CEV:** phase A's CXV cars (`cx_a.cases("CXV")`, 10 cars), constant.
* **Test set CET** (new): `sample_plants(nominal, 50, seed 342)[1:]`, constant parameters. Seed 342 is used by no
  L-layer and by no phase A–D set (checked in src, scripts, tests and docs at ae87f41).
* **Development cars DEV:** phase C's CXDEV cars (seed 399) – development checks only.
* **NOM:** the nominal car (reported separately; the lap-time improvement where the model is exact).
* **Secondary conditions** (after the primary report): `CET_LAG30` (the CET cars with the L6 30 ms steering lag – a
  model error the error regression can learn) and `CET_WET85` (μ × 0.85 – a friction error the safe set, built from the
  car's own laps, contains), with `nmpc`, `identified`, `lmpc`, `lmpc_nominal`.
* **One iteration = one lap from the same initial state** (the iterative task of E.1): the state of the 5 % plan at
  s = 0 (the flying start of phases A–D), one lap scored (time at s = L, interpolated); every lap is driven on for 250 m
  past the line, unscored, so that the stored data cover the region beyond the finish (E.2 uses consecutive laps for
  this). A lap that leaves the track or spins ends the car's sequence: the remaining learning laps count as not completed.
  *Clarification of revision 0 (review m3):* **arclength through the line:** a stored lap's s is unwrapped from 0 at
  its start; the 250 m past the line are stored as s ∈ (L, L + 250]; the time-to-go c = T_l − t is negative there. The
  controller's s is unwrapped the same way (as `MPCController`); when the predicted s_N passes L, the local safe set is
  selected by that unwrapped s, i.e. among the stored points past the line.
* Time limit 121 s per lap, as phases A–D.

## 3. Controllers

| controller | model | plan / terminal set | role |
|---|---|---|---|
| `nmpc` | nominal | 5 % plan | baseline (phases A–D) |
| `identified` | identified offline | full re-plan (L8's pipeline, c 2, margin 0.6, as phase D) | primary comparison |
| `lmpc` | nominal + local error regression (E.4) | local safe set and cost-to-go of the car's laps (E.2) | **primary** |
| `lmpc_nominal` | nominal | the same | secondary (the regression removed) |

### 3.1 Initialisation (E.1's feasible first trajectory; E.2's slow laps)

Two laps of `nmpc`'s NMPC (c 2, margin 0.6, η 0.95, the nominal model) tracking the plans of phase D's library at
η_p = 0.73 and 0.79 (86.8 and 83.0 s for the nominal car), from the common initial state. Two different speeds give the
safe set some spread at every s. A failed initialisation lap is not stored; with none stored, `lmpc` is not started and
all its learning laps count as not completed.

### 3.2 The learning MPC (E.2, with the model of E.4)

At every solve (50 ms), with the stored laps l (their 10 ms states x, the time-to-go T_l − t of each – negative after
the finish – and the 250 m past it):

* **Local safe set:** around the candidate terminal arclength s̃ (the previous solution's s_N advanced by one step; at the
  first solve s_0 + N h v_x), the K nearest stored states by s from each of the last P = 2 stored laps (P·K points D,
  costs-to-go c).
* **NLP:** the NMPC's multiple-shooting problem (`FrenetMPC`'s dynamics, RK4 of 50 ms, the friction ellipses at η 0.95,
  slip, power and steering limits, track limits with margin 0.6, soft limits as `nmpc`) with extra variables λ ∈ R^{P·K},
  λ ≥ 0, 1ᵀλ = 1, the terminal condition (s, n, ξ, v_x, v_y, r)_N = Dλ (soft, an exactly penalised slack as the NMPC's
  other soft constraints; the slack is reported), and the cost  w_T cᵀλ + the NMPC's input-rate costs; no reference
  line or speed. The stage cost of E.2 (1 per step) is the constant N h here and is left out; the terminal cost is the
  time to the finish from the horizon's end (E.2's convex combination of costs-to-go, an approximation). w_T = 100 per
  second (fixed; time dominates the input costs). *Clarification of revision 0 (review n2):* w_T is not tuned because it
  only has to make the time term much larger than the input-rate terms (a 0.1 s gain against input costs of order
  10⁻² per solve); the order of magnitude is checked on NOM's development laps (section 9), not selected.
  *Clarification of revision 0 (review m3):* δ, the seventh state, is **not** in the terminal condition: it is the
  integral of the input δ̇ and E.2's model has the steering angle as an input, not a state; constraining it to a convex
  combination of stored angles could leave the terminal state at a steering angle no stored trajectory continues from
  smoothly, and leaving it free lets the optimiser choose it within the angle and rate limits.
* **Model (E.4):** the NMPC's nominal step plus, for (v_x, v_y, r) at stage k, an affine correction Γ_k z_k,
  z = (v_x, v_y, r, δ, δ̇, F_drive, F_brake, 1), regressed before each solve from the stored laps' 50 ms transitions (the
  state, the input held, the state 50 ms later; the residual against the nominal step): the M = 80 nearest samples to
  the candidate (z̄_k) by a scaled distance, Epanechnikov weights of bandwidth h_r, ridge ε; no sample in the bandwidth –
  Γ_k = 0, the nominal model (E.4's fallback). `lmpc_nominal`: Γ ≡ 0.
* Traction control (0.99) on the nominal car, as `nmpc`. The applied input is the first; the candidate trajectory for the
  next solve is the shifted solution.

### 3.3 Tuning parameters

`lmpc`: K ∈ {10, 20, 40} (points per stored lap; E.2 uses 20 over two laps); `lmpc_nominal` uses `lmpc`'s choice.
h_r = 1 in the scaled distance (each component of z divided by its spread over the stored data), ε = 10⁻⁴ – fixed.

### 3.4 What is and is not claimed

* **No guarantee:** E.1's recursive feasibility and non-increasing lap time need the exact model and the full, discrete
  safe set; the local convex hull, the learned model and the soft terminal condition give that up (E.2, E.4 say so). The
  lap-time curve and the completions are measured, not proven.
* **Deviations from E.2 / E.4:** a nonlinear NLP (IPOPT) with the nominal nonlinear model instead of an ATV QP; the
  correction affine in a fixed regressor (E.4: rows of the linearised error dynamics); each iteration from the same
  initial state with an unscored run-out past the line instead of consecutive laps; the terminal condition soft.
* The comparison with `identified` is not an equal-information comparison (section 1).

## 4. Tuning (CEV only)

`lmpc` with K ∈ {10, 20, 40}: 10 cars × (2 initialisation + 10 learning laps) = 120 laps per setting, 360 laps. Rule: the
most completed tenth learning laps; ties by the mean of the tenth laps' times on the cars every tied setting completes;
then K = 20 (E.2's), then the smaller K. The choice and all records go to the reviewer before any CET lap.

## 5. Always reported

As phases A–D, for every scored lap; and per car the **lap-time curve** over the initialisation and learning laps (lap
time, completed, stop reason), the iteration at which `lmpc` first beats `identified`'s lap (if ever), the terminal slack
(share of solves above 10⁻³), the number of stored points used, the regression's fallback share (stages with Γ = 0) and its
one-step residual before / after the correction on the next lap's data, the solve time.
*Clarifications of revision 0:* (review m1) the **survival curve** – the number of cars still driving after each learning
lap and the lap of the first failure – and the **completion rate of each learning lap conditional on entering it**;
(review m2) for `lmpc`'s tenth learning lap, the cars it completes against those it does not: `identified`'s lap time and
the seven parameter factors (medians, ranges – a description); (review m4) **failures of the initialisation** (both
initialisation laps failed – `lmpc` not started) **counted apart from failures during the learning laps**; H-E2 counts
both as not completed, as registered, and the reading separates them.

## 6. Statistics

Exact signed-rank and McNemar, exact intervals, Holm over H-E1 and H-E2. Secondary, uncorrected (5): on CET `lmpc` –
`nmpc` (McNemar and signed-rank, `lmpc`'s tenth learning lap), `lmpc_nominal` – `lmpc` (both, tenth laps), and `lmpc`'s
tenth – first learning lap (signed-rank on the cars that complete both: the improvement). *Clarification of revision 0
(review n1):* the cars that complete the tenth lap have survived all ten – the improvement is measured on survivors. The secondary conditions:
completions per controller and `lmpc` – `identified` McNemar per condition (2), uncorrected.

## 7. Second environment

As before (deferred by the user): H-E1 and H-E2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). No code before its APPROVE.
2. `control/lmpc.py` (the NLP of section 3.2, the safe-set and regression helpers), `experiments/cx_e.py` (sets, the
   iteration loop, the lap records), `scripts/run_cx_e.py`, `scripts/report_cx_e.py`, `scripts/cx_e_dev.py`, unit
   tests; the development checks of section 9; review (packet 2) before any CEV lap.
3. CEV tuning (360 laps); results to the reviewer.
4. CET + NOM from a pinned worktree: `nmpc` and `identified` 51 laps each, `lmpc` and `lmpc_nominal` 51 × 12 laps
   (~1 330 laps; one job per car and controller, its laps in sequence); report, review.
5. Secondary conditions (~2 × 50 × 26 laps), review, final review, merge.

## 9. Development checks (planned; none has run)

On NOM and DEV only (records in `results/CX/E/dev/` with commit and dirty flag):

1. **NLP:** with the safe set made of one stored `nmpc` lap's states at the solve's own predicted terminal state and
   Γ ≡ 0, one solve's prediction is feasible and its terminal slack zero; the convex-hull rows and λ's simplex hold.
   *Clarification of revision 0 (review m3):* also one solve near the line (s_N past L): the terminal condition is
   feasible and λ selects stored points past the line.
2. **Regression:** on stored laps of a DEV car, the corrected step's one-step residual in (v_x, v_y, r) against the
   nominal step's (in- and out-of-sample on the next lap); Γ = 0 where no sample is in the bandwidth.
3. **Initialisation:** the two laps on NOM and the DEV cars (completed? lap times).
4. **Learning laps:** NOM and DEV with K = 20, 10 learning laps each: the lap-time curves, completions, terminal slack,
   solve times.
5. **Gate:** on NOM `lmpc` completes all 10 learning laps and its tenth lap is faster than its first; on DEV it completes
   the tenth lap on at least 3 of the cars whose initialisation completed; otherwise an amendment before any CEV lap.


### 9. Results of the development checks (2026-10-22; packet 2) – **the gate failed**

NOM and the DEV cars only; records in `results/CX/E/dev/` (git-ignored): `laps.jsonl` (24 sequences at d64eb47, dirty false –
`laps_meta.json`), `diag.jsonl` (12 sequences at 7507114, `diag_meta.json`), `nlp.json` (5068328), `regression.json`
(7507114), `summary.json`; no error record.

1. **NLP** (`nlp.json`): from states of a stored `nmpc` lap of the nominal car (76.40 s), with that lap's own states around
   its state 30 steps later as the safe set (K = 10, the lap in both P slots) and Γ = 0: every solve converges with the
   terminal slack below 10⁻⁸ and λ on the simplex (sum 1, min ≥ 3 · 10⁻⁵); near the line (s = 4 257 m, s_N = 4 370 m)
   all of λ's weight lies on stored points past the line (clarification m3).
2. **Regression** (`regression.json`, DEV car 1; the two initialisation laps stored): the one-step residual of
   (v_x, v_y, r) on a stored lap 0.019 / 0.030 / 0.0071 → 0.0004 / 0.0009 / 0.0002 (in sample); on a faster lap (the
   0.95 plan, out of sample) 0.027 / 0.034 / 0.012 → 0.018 / 0.022 / 0.0096, with 15 % of the samples outside the
   bandwidth (Γ = 0). In the learning laps 38–62 % of the stages fall back to the nominal model.
3. **Initialisation:** both laps complete on NOM and on all five DEV cars (86.9–87.0 s and 83.1–83.5 s).
4. **Learning laps** (`laps.jsonl`; 10 learning laps were planned for each):

   | variant | NOM | DEV cars 1–5 |
   |---|---|---|
   | `lmpc` K = 20 (the registered development setting) | lap 1 off at 442 m | all five off on lap 1 (465–470 m, turn 1) |
   | `lmpc` K = 10 | lap 1 77.0 s, lap 2 off (343 m) | all five off on lap 1 (471 m – 3 838 m) |
   | `lmpc` K = 5 | lap 1 77.0 s, lap 2 off (406 m) | all five off on lap 1 (470 m – 2 974 m) |
   | `lmpc_nominal` K = 10 | lap 1 76.6 s, lap 2 off (471 m) | 2 complete lap 1 (77.6, 77.2 s) and leave on lap 2; 3 leave on lap 1 |

   Solve time 132–681 ms mean per lap (`nmpc`'s initialisation laps 137–186 ms), under 20 parallel jobs.
5. **Gate: failed** for every setting – no sequence completes more than one learning lap on NOM or the DEV cars, none
   reaches the tenth.

**Mechanism (development diagnostics, `diag.jsonl`; options added for them only, the registered settings unchanged):**

* *On the nominal car the model is exact* – one solve's prediction and the measured state 50 ms later agree to 10⁻³ while
  the limits hold – yet the learning laps leave the track. The first learning lap jumps from the initialisation laps' 83 s to 76.4–77.0 s: every
  solve places the terminal state at the far end of the window (the lowest time-to-go), so the target moves ahead of the
  stored lap's timeline at each solve. Near the limit – braking into turn 1 – the terminal condition becomes unreachable
  (the share of solves with terminal slack above 10⁻³ grows over the laps, e.g. 0.03 → 0.17 → 0.33) and the car leaves.
* A **convex combination of stored states is a feasible state for a linear (or linearised, E.2's ATV) model, not for the
  nonlinear model here**: over a window of tens of metres (K = 20: 442 m on NOM in lap 1) or across two laps of different
  speed and line, the combined terminal state lies off every stored trajectory. Fewer points help: K ≤ 10 completes lap 1
  on NOM; the last lap only (P = 1) and the steering angle in the terminal condition (against clarification m3's
  reasoning) add at most one or two laps: the best sequence, `lmpc_nominal` K 5, P 2, δ in the terminal condition,
  completes three learning laps on NOM (76.5, 76.4, 76.4 s) and leaves on the fourth (483 m); with P = 1 NOM completes two
  (76.4, 75.9 s) and DEV car 1 two (77.1, 76.6 s), then turn 1.
* *On the DEV cars* the first learning lap plans with the nominal model at η 0.95 – `nmpc`'s aggressiveness – and leaves
  where `nmpc` leaves (DEV car 4: 3 838–3 841 m with every `lmpc_nominal` variant – the corner `nmpc` loses on it; 468–473 m and 2 597 m
  with the regression); the regression has data only
  at the initialisation laps' speeds and falls back on 38–62 % of the stages, and with it the cars leave on lap 1 as well.

## Amendment E1 (proposed 2026-10-22, packet 2; **decided: option 1** – see the decision below)

*Cause:* the gate of section 9 failed (no sequence beyond the second learning lap; the registered development setting
K = 20 fails the first on every car). The method as adapted (E.2's local convex hull with the nominal nonlinear model in
the NLP) does not learn safely here; the development diagnostics point at the relaxation's loss of feasibility near the
limit and at the first lap's model-based aggressiveness on mismatched cars.

Options (the reviewer decides; nothing is run before):

1. **Close phase E without the benchmark** (as phase B's option B3.2): the result is the development finding above,
   reported in CONTROL_EXPANSION_BENCHMARK.md as a negative development result – the racing LMPC of E.2 / E.4, adapted to
   this car's nonlinear model and a 1.5 s horizon at up to 85 m/s, does not complete ten learning laps on the nominal car
   or the development cars; no test claim. *Proposed.*
2. **Run the benchmark with the gate waived** (as phase B's B3.1), with the grid K ∈ {5, 10, 20} (E.2's 20 over two laps
   is K = 10 here) and the tie broken to K = 10: H-E1 / H-E2 as registered. Expected (from the development cars): `lmpc`
   completes few or no tenth laps, H-E1 is untestable, H-E2 measures how fast it fails. Cost ~1 300 laps (the LMPC
   sequences end early).
3. **Re-design and re-register** (a new protocol revision before any lap): E.2's ATV-QP formulation (linearised along the
   candidate, so that the convex hull is a feasible set of the model it uses), or a bound on the progress per lap; both
   are larger changes than an amendment and would be reviewed as revision 1.

### Decision on amendment E1 (the review of packet 2, 2026-10-22): option 1, with e1–e4

**Phase E is closed without the benchmark; H-E1 and H-E2 are recorded as *not tested*.** The development records and the
gate's result of section 9 are accepted. Option 2 would spend ~1 300 laps to measure at which learning lap the method fails
– information the development laps already give (not even the nominal car completes ten); option 3 is a new design and a
new protocol, listed as the first follow-up (e3).

* **e1 – the scope of the negative result:** it concerns **this project's implementation** – an NLP solved by IPOPT, the
  nominal nonlinear model, the convex hull of stored states as the terminal set, the terminal condition soft – which does
  not complete ten learning laps on the nominal car or the development cars. It is **not** a result about the racing LMPC
  of E.2 or E.4 as such. The most likely cause – that a convex combination of stored states need not be feasible for a
  nonlinear model, while it is for E.2's ATV linearisation – is a **hypothesis, not verified** here.
* **e2 – what was observed (section 9):** both initialisation laps complete on every car; the first learning lap jumps
  from 83 s to 76.4–77.0 s (about the 5 % plan's lap time); the share of solves with terminal slack above 10⁻³ rises from
  lap to lap (0.03 → 0.17 → 0.33); the regression falls back to the nominal model on 38–62 % of the stages of the learning
  laps; the development cars leave where `nmpc` leaves. In the diagnostics `lmpc_nominal` (K 5, P 1) drove its second
  learning lap on the nominal car in **75.9 s, faster than `nmpc`'s 76.40 s** – development data, a description only:
  the mechanism can beat the tracked plan, but not lap after lap.
  *Note (2026-10-28, on the phase-I branch):* "the development cars leave where `nmpc` leaves" holds for some
  `lmpc_nominal` variants (~3.84 km, section 9). With the registered K 20 they left at turn 1 (465–470 m), where `nmpc`
  does not leave.
* **e3 – follow-up candidates** (CONTROL_METHOD_MATRIX.md): (1) E.2's ATV-QP implementation (the model linearised along
  the candidate trajectory, so that the convex hull is feasible for the model the problem uses); (2) a bound on the progress
  per lap (e.g. the terminal time-to-go not below the previous lap's at the corresponding point minus a limit); (3)
  consecutive laps instead of every lap from the same initial state.
* **e4 – the code:** `control/lmpc.py`, `experiments/cx_e.py`, the scripts and the tests are merged as usual, with the
  registered settings and the development options (off by default), so that section 9 can be reproduced; the records
  of section 9 map to the commits named there.
