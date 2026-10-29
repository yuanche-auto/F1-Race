# CX-I protocol – the learning MPC's follow-ups, a progress bound per lap first (control expansion, phase I – frontier 2)

Status: **binding, revision 0 (2026-10-28; bdc02d0, APPROVE)** with the clarifications of its review (m1, m2, n1, n2;
marked *"clarification of revision 0"*), written down before any phase-I code. **I-1's development gate failed (section
9; stage A passed, stage B failed): I-1 is closed for this phase (rule 4); no benchmark registered, no IV or IT lap.**
Written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §I, on top of §E and §H) and before any phase-I code: no
module, test, runner or lap of phase I exists. Branch `cx/I-lmpc-followups` from `main` 98f8134 (the merge of phase H).

The user's instruction (2026-10-28, translated): *"First try the methods in the method matrix; if they still do not work,
we will discuss what to do next."* The methods are the three follow-up candidates of CX-H amendment H1 (review i3),
listed in CONTROL_METHOD_MATRIX.md in this order:
1. a bound on the progress per lap, or a slower safe set;
2. E.2's own local-regression model, with a trust region and a robust QP solver;
3. consecutive laps.

Review i3 adds a precondition for any later LMPC phase: show on NOM that its first learning lap completes reliably
before a benchmark is registered.

## 0. What phases E and H left

* **Both closed without the benchmark.** Neither the NLP (phase E, IPOPT) nor the ATV-QP (phase H, OSQP; a diagnostic
  with qpOASES and a step box) completed ten learning laps on the nominal car NOM or the development cars DEV. With the
  registered settings, every car left the track on learning lap 1. Phase E's best diagnostic variant drove three
  learning laps on NOM.
* **The common observation** (CX-H §9, review i2; an observation, not a tested cause):
  * The two initialisation laps take 86.9 and 83.1 s. `nmpc` on the η 0.95 plan takes 76.40 s.
  * Driven by the stored laps' time-to-go, the first learning lap plans at the nominal model's η 0.95 limit. Where it
    completes, it takes 76.4–77.0 s.
  * On the mismatched DEV cars it leaves at turn 1 (465–474 m). `nmpc` on the η 0.95 plan does not leave there: it
    completes two of the five DEV cars and leaves the other three at ~3.83 km (CX-H §9, corrected).
  * Phase E's regression had data only at the initialisation laps' speeds. It fell back to the nominal model on 38–62 %
    of the stages of the learning laps.
* **The literature (§I):**
  * The published racing LMPCs start far from the limit: E.2 and I.1 at ~1.2 m/s, E.4 at 10 m/s on the centre line. They
    improve gradually over tens of laps. E.4 calls its input-rate cost a "learning rate".
  * No source found bounds the progress per lap by speed or lap time. I.1 widens a lateral-state envelope lap by lap,
    gated by the previous lap's outcome, on a model equal to the plant.
* **The solvers:**
  * Phase E's NLP failed 0.1–17 % of the solves of a learning lap in its development records. Its sequences ended with
    the car leaving the track.
  * Phase H's registered QP failed 55–98 % (OSQP's iteration limit).

## 1. The sequence of candidates (fixed in advance)

| candidate | what changes against phase E | registered |
|---|---|---|
| **I-1** | a **speed bound per lap**: every stage at most ε faster than the stored laps at that point (section 3.1) | this revision |
| I-2 | **E.2's own model** (local linear regression of the velocity states), one QP per step with a **trust region** and a robust QP solver (section 3.3) | by revision, if reached |
| I-3 | **consecutive laps** (section 3.3) | by revision, if reached |

Rules:
1. **The order is fixed.** A candidate is developed only after the previous one's gate failed and the review accepted
   that result.
2. **Each candidate has its own development gate** (section 9), judged only on its registered development laps.
   Diagnostic variants may be run as diagnostics. They cannot pass a gate.
3. **The first candidate to pass its gate gets the benchmark** (sections 4–6). Review i3 applies: the benchmark is
   registered – made binding – by a revision after that candidate's stage A passes (clarification m1 below: after A
   and B), and before any tuning or test lap.
   The later candidates are not run in this phase; they stay in the matrix.
4. **If a candidate's gate fails**, its development results are reported as they are, and the next candidate's design
   is registered by a revision (reviewed) before its code. The revision states whether it keeps I-1's bound, decided
   from I-1's development records, with the reason.
5. **If all three fail**, phase I is closed without the benchmark, and the user decides the next step (the user's
   instruction).

*Clarification of revision 0 (review m1) – when the benchmark is registered:* the revision that registers the benchmark
is written only **after both stages of the candidate's gate (A and B, section 9) have passed**, not after stage A alone.
A candidate whose stage A passes and whose stage B fails has failed its gate (rule 4); no benchmark is registered for
it. No IV or IT lap is run before that revision's APPROVE.

## 2. Question and hypotheses

*Phase E's LMPC is kept, but no lap may jump to the limit: each stage's speed may exceed that of the faster of the last
two stored laps at the same point by at most ε. Does the LMPC then learn over ten laps on these cars? And how does its
tenth learning lap compare with the one-lap identification and full re-plan?*

The hypotheses are phase E's and H's, for the candidate that passes its gate (`pb` for I-1). They are **planned**; by
rule 3 they become binding only by the revision that registers the benchmark.
* **H-I1 (primary):** on the test cars IT (section 4), the lap time of the candidate's tenth learning lap and
  `identified`'s lap differ, on the cars both complete. Exact signed-rank (mid-ranks, the exact distribution),
  two-sided.
* **H-I2 (primary):** the candidate's tenth learning lap and `identified` differ in the number of completed laps on IT.
  Exact McNemar, two-sided.

Holm over H-I1 and H-I2 (α 0.05), each also against 0.025. No direction is predicted. Phase E's clarifications carry
over:
* H-I2's exposure is unequal against the candidate: its tenth lap counts only if all ten learning laps survive (E m1).
* H-I1 is conditional on both completing, so it is measured on survivors (E m2).
* The comparison is not an equal-information one (E section 1).

Scope: Spielberg, the nominal model, η = 0.95, N = 30 × 50 ms, offline (no time budget; L9's rule is not applied).

## 3. Controllers

| controller | model | terminal set / plan | role |
|---|---|---|---|
| `nmpc` | nominal | 5 % plan | baseline |
| `identified` | identified offline | full re-plan (L8's pipeline, c 2, margin 0.6, as phases D, E, H) | primary comparison |
| `pb` | phase E's NLP (nominal + E.4's error regression) **with the speed bound per lap** | local safe set and cost-to-go (E.2), as phase E | **primary** (I-1) |
| `pb_nominal` | the same without the regression (Γ ≡ 0) | the same | secondary |
| `lmpc` | phase E's NLP, unchanged (no bound) | the same | secondary: the bound's ablation |

**Initialisation** as phase E (3.1): two laps of `nmpc`'s NMPC tracking the plans at η_p 0.73 and 0.79. A failed
initialisation lap is not stored.

### 3.1 I-1: the progress-bounded LMPC (`pb`)

**Phase E's optimal control problem, row for row** (`control/lmpc.LearningMPC`, CX-E section 3.2):
* the dynamics with E.4's correction;
* the friction ellipses at η 0.95, and the slip, power and axle-force rows;
* the track limits with margin 0.6;
* the soft terminal convex hull and the cost.

It is solved by IPOPT as in phase E, with phase E's weights, regression (M = 80, h_r = 1, ridge 10⁻⁴), w_T = 100,
P = 2, candidate and fallback. **Added: one soft row per stage i = 1 … N**

  v_x,i ≤ v̄_i + σ_i, σ_i ≥ 0, cost w_slack · h · (σ_i² + 10 σ_i) (the track slacks' exact penalty, w_slack = 10⁴),

with

  v̄_i = (1 + ε) · v_ref(ŝ_i):
* **v_ref(s)** is the largest v_x at s among the stored laps the safe set uses (the last P = 2). Each lap's 50 ms
  records are interpolated linearly in s; the stored laps cover the lap and its 250 m run-out.
* **ŝ_i** is the predicted arclength of stage i, from the previous solution shifted by one step. At a lap's first solve
  it is s_0 + i Δt v_x,0, as phase E's initial guess.

**What the bound does:**
* Every stage's speed is at most ε above the faster of the last two stored laps at that point, so each lap can be at
  most about ε faster than the stored laps, point by point.
* For the first learning lap the reference is the faster of the two initialisation laps at each point (mostly the
  second, 83.1 s on NOM). The bound is expected to bind where the first learning laps of phases E and H went faster
  than the stored laps – into turn 1, where they left the track; the development checks show where it binds.
* Where the friction rows bind before the speed bound, the problem is phase E's.
* The regression (E.4) is evaluated at speeds close to the data it was fitted on, and every completed lap adds data a
  little faster.

**What it does not do:**
* It does not keep a lap away from the nominal model's limit for long. The speed ratio between the η_p 0.79 and 0.95
  plans in a corner at the limit is √(0.95 / 0.79) ≈ 1.10. At ε = 0.02, 1.02⁵ ≈ 1.10, so after about five laps the bound
  no longer stops the LMPC from planning at η 0.95.
* Whether the learned correction keeps a mismatched car on the track by then is what the gate and the benchmark show.

**Why a speed bound, and why on phase E's NLP:**
* *Not a slower safe set:* the first learning lap's safe set already holds only the two slow initialisation laps, and
  the jump happened anyway.
* *Not I.1's envelope:* it acts on the sideslip and yaw rate, aims at drift, and was shown on a model equal to the plant.
* *Not E.4's rate cost:* its effect on the lap time is indirect, and E's and H's weights were registered.
* *The speed bound* acts on the quantity that failed – the speed into turn 1 – and its step ε reads as a lap time.
* *Phase E's NLP and not phase H's QP:* the bound does not depend on the formulation. E's NLP solved 83–99.9 % of
  its solves; H's registered QP solved 2–45 %. With E's NLP, phase E's records are directly the "no bound" comparison
  (`lmpc`).

**Fixed now, not tuned:**
* the slack penalty;
* v_ref as the maximum over the last two laps, interpolated linearly;
* ŝ from the shifted solution;
* K = 20 (E.2's value; phase E's and H's development setting);
* everything inherited from phase E.

**Tuned** (section 4): ε ∈ {0.02, 0.04, 0.08}. Development setting: ε = 0.04.

*Clarification of revision 0 (review m2) – the first learning lap, and the reference past the line:*
* At the first learning lap the last two stored laps are the **two initialisation laps** (`nmpc` on the plans at
  η_p 0.73 and 0.79). v_ref is therefore, almost everywhere, the speed of the η_p 0.79 lap, and the first learning lap's
  bound is **about 1.02–1.08 times that lap's speed** at each point.
* **Expected, so that the development records show whether the bound acts:** the first learning lap takes about the
  η_p 0.79 lap's time or a little less – roughly T₀.₇₉ / (1 + ε), on NOM about 81.5, 80.0 and 77.0 s for ε = 0.02, 0.04
  and 0.08 (T₀.₇₉ = 83.16 s). It should not jump to ~77 s at ε = 0.02 or 0.04, as phase E's first learning laps did.
  The figures are approximate: the bound is on v_x at each s, not on the lap time, and a line through the inside of a
  corner gains arclength rate (ṡ = (v_x cos ξ − v_y sin ξ) / (1 − nκ)).
* **Past the line (s > L):** v_ref is taken from the stored laps' records of their 250 m run-out, by the unwrapped
  arclength (s from about 0 at the start, beyond L after the line), interpolated linearly as before the line. A
  prediction never reaches beyond a stored lap's run-out: the horizon is 1.5 s, at most ~135 m at 90 m/s.

**Attribution:** the module's docstring cites E.1, E.2, E.4 (the LMPC and the error regression) and I.1 (the lap-gated
expansion that motivated a per-lap bound). The speed bound itself is this project's design (literature §I (a)).

### 3.2 What is and is not claimed

* **No guarantee:** E.2, E.4 and I.1 state none for the relaxed, learned scheme. The bound adds none.
* **The bound is a design choice of this project.** No published racing LMPC was found to use it (§I (a)).
* **The deviations from E.2 are phase E's:**
  * the nominal model with E.4's correction;
  * the soft terminal condition;
  * N = 30 × 50 ms;
  * every lap from the same initial state with a 250 m run-out.

### 3.3 I-2 and I-3 (sketches; each registered by a revision before its code, if reached)

* **I-2 – E.2's own model:**
  * the velocity states (v_x, v_y, r) from a local linear regression of the stored data (Epanechnikov kernel, nearest
    neighbours; E.2, H.1), the kinematic Frenet states linearised analytically – no nominal physics model;
  * one QP per step along E.2's candidate (phase H's structure);
  * qpOASES with a box on the input step. Phase H's diagnostic with these settings failed 0–18 QPs per lap of
    ~150–1 000 solves.
  * Whether it keeps I-1's bound: rule 4.
* **I-3 – consecutive laps (E.2, E.4, E.6):**
  * the next lap starts from the previous lap's state at the line, shifted back by the track length;
  * the safe set includes the stored points past the line, their time-to-go continued;
  * ten learning laps in one run;
  * on top of I-1 or I-2 (rule 4).
  * The comparison with `nmpc` and `identified` – a flying lap against a lap from the plan's start state – is defined
    in its revision.

## 4. The benchmark (planned; binding only by the revision of rule 3)

**Sets:**
* **Tuning IV:** phase A's CXV cars (10; the tuning set of phases A–C, E and H), constant.
* **Test IT** (new): `sample_plants(nominal, 50, seed 383)[1:]`, constant parameters.
  * No L-layer or phase uses seed 383 (checked in `src/`).
  * Phase E's CET (342) and phase H's HT (382) were registered and never driven. They are not reused, to keep the
    registrations apart.
* **Development DEV:** phase C's CXDEV cars (seed 399; phases E's and H's development cars) and **NOM**.
* **Secondary conditions** (after the primary report): `IT_LAG30` (the L6 30 ms steering lag) and `IT_WET85`
  (μ × 0.85), with `nmpc`, `identified`, `pb` and `pb_nominal`.

**One iteration = one lap from the same initial state**, as phase E section 2 (the 5 % plan's state at s = 0, the
time at s = L scored, 250 m past the line driven and stored unscored, a crash ends the car's sequence, time limit 121 s).

**Tuning** (IV only): `pb` with ε ∈ {0.02, 0.04, 0.08}, K = 20: 30 sequences, 360 laps. The rule, phase E's:
1. the most completed tenth learning laps;
2. ties: the mean tenth lap time on the cars every tied setting completes;
3. then ε = 0.04;
4. then the smaller ε.

`pb_nominal` uses `pb`'s ε. The choice and the records go to the reviewer before any IT lap.

*Clarification of revision 0 (review n1):* the tuned parameter is **ε only**, over the grid {0.02, 0.04, 0.08}. **K is
fixed at 20.** Ties after step 2 go to the development setting ε = 0.04, then to the smaller ε (phase E's rule, with ε
in place of K).

**Primary** on IT (and NOM, shown apart): `nmpc` and `identified` (one lap each); `pb` and `pb_nominal` (twelve laps each).

**Secondary** on IT, after the primary report:
* `lmpc` (phase E's NLP, K 20, no bound) – the bound's ablation;
* the secondary conditions.

## 5. Always reported

As CX-H section 5, with `pb` in place of `atv`. Per car:
* the lap-time curve over the initialisation and learning laps;
* the survival curve and the first failed learning lap;
* the completion rate of each learning lap, conditional on entering it;
* initialisation failures counted apart from failures while learning;
* the first learning lap faster than `identified`;
* the terminal slack;
* the regression's fallback share;
* solver failures (fallbacks) and the solve time;
* the tenth lap: the cars completed against those not completed (E m2).

**Added for the bound:**
* per lap, the share of stages with σ > 10⁻³ and the share with v_x within 1 % of v̄; the largest σ;
* each lap's time over the previous lap's;
* for a failed lap, the bound v̄ and the stored laps' speed at the failure point, against the car's speed there.

## 6. Statistics

Exact signed-rank and McNemar, exact intervals, Holm over H-I1 and H-I2.

Secondary, uncorrected:
* on IT, `pb` – `nmpc` (`pb`'s tenth lap);
* `pb_nominal` – `pb` (tenth laps);
* `pb`'s tenth – first learning lap (survivors, E n1);
* `pb` – `lmpc`: the completions of the first and the tenth learning lap (McNemar), and the survival curves;
* the secondary conditions: completions per controller, and `pb` – `identified` McNemar per condition.

## 7. Second environment

As before (deferred by the user): H-I1 and H-I2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and literature §I (packet 1). **No code before its APPROVE.**
2. Code:
   * `control/lmpc.py`: the speed bound as an option of `LearningMPC`, off by default. Phase E's NLP path is unchanged,
     checked by reproducing a phase-E development record.
   * the controller computes v̄ from the store;
   * `experiments/cx_i.py`; `scripts/run_cx_i.py`, `scripts/report_cx_i.py`, `scripts/cx_i_dev.py`; unit tests.
3. The development checks of section 9 (about 14 sequences and 12 fixed laps; a few hours with 20 workers). Packet 2
   then carries the results and either the revision that registers the benchmark (stage A passed, rule 3), or I-2's
   revision (the gate failed, rule 4).
4. IV tuning: 30 sequences. Phase E's records give ~0.2–0.6 s per IPOPT solve on a learning lap, so a full
   twelve-lap sequence is estimated at ~2 h, ~5 h with 20 workers. The choice goes to the reviewer.
5. IT from a pinned worktree (Windows `.venv`, single-threaded BLAS, one process per job): 100 learning sequences and
   102 fixed laps (~12 h estimated). Then the report and its review.
6. Secondary: `lmpc` on IT; IT_LAG30 and IT_WET85 with the four primary controllers (~1–1.5 days estimated). Then the
   review, the final review and the merge.

## 9. Development checks of I-1 (planned)

On NOM and DEV only. Records in `results/CX/I/dev/`, each with the commit and a dirty flag.

1. **Phase E's NLP unchanged:** with the bound off, phase E's development record `lmpc` K 20 on NOM is reproduced.
2. **The bound** (unit tests):
   * with v̄ above every reachable speed, one solve equals the unbounded solve (σ = 0);
   * with v̄ below the current speed profile, the predicted speeds are capped up to a small σ;
   * v_ref's interpolation across two laps and into the run-out.
3. **Laps (K = 20):**
   * **stage A:** `pb` on NOM at ε = 0.02, 0.04 and 0.08;
   * **stage B:** `pb` and `pb_nominal` at ε = 0.04 on NOM and the five DEV cars;
   * `nmpc` and `identified` on the same cars, for the tables;
   * phase E's and H's records beside them, for development only.
4. **Gate (fixed in advance; the gate of I-1):**
   * **A – the precondition of review i3**: on NOM, `pb` completes learning lap 1 at **each** ε of the grid (3 of 3).
     "Reliably" is read as: not dependent on one value of the tuned parameter.
   * **B – phase E's gate:**
     * on NOM at ε = 0.04, `pb` completes all ten learning laps, and its tenth is faster than its first;
     * on DEV at ε = 0.04, it completes the tenth on at least 3 of the cars whose initialisation completed.
   * The gate is judged on these registered laps only. No other setting may be tried in order to pass it.
   * *Clarification of revision 0 (review n2):* `pb_nominal`'s laps in stage B are **descriptive only**; they take no
     part in the gate, which is judged on `pb` alone.
   * **A and B pass:** packet 2, with the revision that registers the benchmark (rule 3, clarification m1).
   * **Either fails:** I-1 is closed for this phase. Packet 2 reports it and carries I-2's revision (rule 4).

### 9. Results of I-1's development checks (2026-10-29; packet 2) – **stage A passed, stage B failed: the gate of I-1 failed**

Records (git-ignored) in `results/CX/I/dev/`, at 62fc41d (clean; `meta_*.json`, `RUN_COMMIT_dev`):
* `nlp_repro.json`;
* `laps.jsonl` – 26 jobs, 0 errors, run in 51 min with 20 workers;
* `report/` (`report_cx_i.py dev`, `CX_I_dev.md`, `gate.json`).

Before the registered laps there was one scratch smoke, `pb` ε 0.04 on NOM with only one learning lap
(`results/CX/I/smoke/`): learning lap 1 took 80.83 s. The registered sequence reproduced it exactly.

1. **Phase E's NLP unchanged (9.1):** with the bound off, phase E's `lmpc` K 20 on NOM reproduces its record exactly
   (`equal: true`).
2. **The bound (9.2):** `tests/test_cx_i.py`, 7 passed:
   * an inactive bound gives the unbounded solution;
   * a binding bound caps the predicted speeds;
   * v_ref is the faster lap, including past the line;
   * the layout with the bound off is phase E's;
   * the sets and the ε rule.
3. **Stage A – `pb` on NOM, learning lap 1 at each ε: 3 of 3 completed.**

   | ε | learning lap 1 | expected (m2) | learning lap 2 |
   |---|---|---|---|
   | 0.02 | 81.84 s | ~81.5 s | off at 435 m (turn 1) |
   | 0.04 | 80.83 s | ~80.0 s | off at 2 613 m |
   | 0.08 | 79.07 s | ~77.0 s | spin at 2 165 m |

   The bound acts as m2 expected: the first learning lap no longer jumps to ~77 s. Phases E and H left the track on
   this lap.
4. **Stage B – ε 0.04 on NOM and DEV.**

   | car | `pb` (the gate) | `pb_nominal` (descriptive, n2) | phase E's `lmpc` K 20 – lap 1 | `nmpc` | `identified` |
   |---|---|---|---|---|---|
   | NOM | 80.83, then off on lap 2 (2 613 m) | 80.42, 78.51, 77.60, 77.65, then off on lap 5 (3 830 m) | off at 442 m | 76.40 s | 76.42 s |
   | DEV_car1 | off on lap 1 (2 922 m) | 81.82, 79.37, 78.10, 77.30, then off on lap 5 (441 m) | off at 469 m | 76.25 s | 76.63 s |
   | DEV_car2 | spin on lap 1 (2 961 m) | 80.53, 79.23, then off on lap 3 (2 735 m) | off at 469 m | off at 3 829 m | 77.75 s |
   | DEV_car3 | off on lap 1 (2 667 m) | 80.74, 80.14, then off on lap 3 (2 746 m) | off at 465 m | off at 3 829 m | 78.75 s |
   | DEV_car4 | 81.42, then off on lap 2 (405 m) | 80.93, 80.11, then off on lap 3 (2 737 m) | off at 468 m | off at 3 828 m | 78.70 s |
   | DEV_car5 | off on lap 1 (2 686 m) | 81.56, 78.80, 77.73, then off on lap 4 (2 686 m) | off at 470 m | 75.91 s | 75.72 s |

   * **`pb`'s solver failures:** 9–51 per learning lap, at most 11.5 % of a lap's solves. Its regression fell back
     to the nominal model on 27–45 % of the stages.
   * **`pb` exceeds its bound before it fails.** The largest planned excess σ per lap was 2.6–15.4 m/s on the DEV
     cars. At the last solve before leaving, DEV_car3 drove 51.9 m/s where the stored laps had 42.7 m/s (bound 44.6),
     and NOM at ε 0.02 drove 35.1 m/s into turn 1 against a bound of 26.7. The record holds the planned excess; it does
     not show why the car got there.
   * **`pb_nominal`** (the same bound, no regression):
     * 0–15 solver failures per lap;
     * every completed learning lap was faster than the one before (ratio 0.966–0.993), except NOM's lap 4 (0.05 s
       slower than lap 3);
     * the share of stages near the bound fell from ~0.55 on lap 1 to ~0.1 by lap 4, as the bound stopped binding;
     * the cars that lasted longest (NOM, DEV_car1, DEV_car5) reached 77.3–77.7 s. That is not faster than `nmpc`
       (76.40 s on NOM). DEV_car2–4 left on lap 3, their best laps 79.2–80.1 s;
     * on the last laps the share of solves with terminal slack above 10⁻³ rose to 0.11–0.19 – phase E's failure near
       the limit (CX-E §9);
     * on NOM, where the model equals the plant up to the integration, it left on lap 5 at 3 830 m.
5. **Gate of I-1 (`gate.json`):**
   * A passed (3 of 3).
   * B failed: NOM does not complete ten learning laps (`pb` leaves on lap 2), and DEV completes the tenth on 0 of 5.
   * **The gate failed. By rule 4, I-1 is closed for this phase.** No benchmark is registered (clarification m1), and
     no IV or IT lap was run.

**Reading (development data; observations, not tested causes):**
* **The bound did what it was designed for.** The first learning lap completed on NOM at every ε, near the expected
  time. With `pb_nominal` the laps improved gradually, as in the published LMPCs (§I (b)).
* **The error regression (E.4) made the sequences shorter on every car.** `pb` failed on lap 1 or 2, `pb_nominal` on
  laps 3–5. In phase E's development records, `lmpc_nominal` never did worse than `lmpc` at the same K. With the
  regression the cars got faster than the stored laps and than the bound. This is consistent with the regression
  mis-predicting the speed at speeds above its data. The records do not test that.
* **Near the limit the LMPC fails even without model error.** On NOM, `pb_nominal` reached 77.6 s and left on lap 5,
  at 3 830 m – the corner where `nmpc` leaves three DEV cars. On the DEV cars it left at ~2.7 km (four cars) or at turn 1
  (DEV_car1).
  * The bound no longer binds there, and the terminal slack grows lap by lap, as in phase E.
  * So the bound postpones phase E's failure by a few laps; it does not remove it.
* **For the next candidates (rule 4):**
  * I-2 regresses the velocity states entirely (E.2). The I-1 records show the partial regression of E.4 shortening the
    sequences.
  * I-3 (consecutive laps) adds data past the line; it does not change the formulation near the limit.
  * Neither addresses the near-limit failure seen on NOM. Their revisions must say how they would.
