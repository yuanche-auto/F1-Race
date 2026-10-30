# CX-I protocol – the learning MPC's follow-ups, a progress bound per lap first (control expansion, phase I – frontier 2)

Status: **binding, revision 0 (2026-10-28; bdc02d0, APPROVE)** with the clarifications of its review (m1, m2, n1, n2;
marked *"clarification of revision 0"*), written down before any phase-I code. **I-1's development gate failed (section
9; stage A passed, stage B failed): I-1 is closed for this phase (rule 4); no benchmark registered, no IV or IT lap.**
**Revision 1 (2026-10-29; 1e2bf96, APPROVE – packet 3) with the clarifications of its review (m1–m3, n1–n3, marked
*"clarification of revision 1"*), written before any code of it: section 10, a reproduction check under the published
LMPCs' conditions on NOM – a diagnostic at the user's request; I-2 and I-3 not registered.**
**Its results (section 10.9; packet 4): R1 reads (c1) – learning laps of 150.8, 88.6 and 80.4 s, then off on learning
lap 4 with the solver succeeding, as in phases E and I. R2 and R3 read (c2) and were examined: R2 is the same failure on
learning lap 1 (K 20), and R3 is phase H's QP solver at its iteration limit from the first solve.**
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

**Review of packet 2 (437ee9e): APPROVE** (2026-10-29). I-1 is closed for this phase; no benchmark is registered. The
review: the I-2 and I-3 revisions must first state how they address the failure near the limit on NOM with an exact
model, or there is no reason to expect them to pass the same gate – passed to the user as written, as the basis for
the decision.

## 10. Revision 1 – a reproduction check under the published conditions (a diagnostic; the user's request)

**Why.**
* The user chose to discuss before I-2 and asked:
  * which papers the method follows;
  * whether the original would pass;
  * how ours differs.
* The answer (literature §E, §H, §I, I.5):
  * The published racing LMPCs start far from the limit, run consecutive laps over tens of laps, and report no failure
    statistics. The exception is E.4: on BARC, 1 of 10 trials failed, and E.2's regression failed 5 of 10 at h = 3.
  * None starts near the limit or compares with a planner-based controller.
  * Nobody has run the original under phase I's conditions, so whether it would pass is not known.
* The user's instruction (2026-10-29, translated): *"Do the reproduction check first."* It is a diagnostic under the
  published conditions, on NOM.

**Its place among phase I's rules:**
* It is a diagnostic (rule 2). It has no gate, cannot register a benchmark, and does not reopen I-1.
* I-2 and I-3 stay unregistered. After the check the user decides: close the LMPC line, or ask for a candidate that a
  revision registers on the check's result.
* No lap of the check runs before this revision's APPROVE and its code.

### 10.1 Question

*Driven under the published racing LMPCs' conditions, does phase E's LMPC learn on NOM to a lap near the limit and keep
completing laps there?* Phase E's LMPC here means the NLP with the nominal model, K 20 and P 2.
* **If it does,** phases E's and I's failures belong to their conditions: a start near the limit, every lap from the
  same state, ten laps.
* **If it does not,** the failure near the limit belongs to the formulation as implemented here, on this car.

### 10.2 The conditions carried over

| condition | papers (literature) | phases E and I | this check |
|---|---|---|---|
| start | two laps of path following at a constant ~1.2 m/s (E.2, I.1); three laps on the centre line at 10 m/s (E.4) | `nmpc`'s tracking NMPC on the η_p 0.73 / 0.79 plans: 86.9 / 83.1 s, against `nmpc`'s 76.40 s | two laps of the same tracking NMPC on the **centre line at a constant v_c** |
| laps | consecutive, with data past the line (E.2, E.4) | every lap from the plan's start state, with a 250 m run-out | **consecutive**: each lap starts from the previous lap's state at the line |
| how many | converged after 29 (E.2) | 10 | **40 learning laps** |
| model | regression (E.2); nominal plus regression (E.4); equal to the plant (I.1) | nominal plus E.4's regression (`pb`, `lmpc`); nominal (`pb_nominal`, `lmpc_nominal`) | **nominal on NOM**, equal to the plant up to the integration (I.1's condition) |
| solver | one QP per step (E.2, H.1, I.1); an NLP or a QP (H.2) | NLP (E, I); QP (H) | NLP (R1, R2); QP (R3) |

**v_c (fixed now).** v_c is the constant speed at which the centre line's largest lateral acceleration, v² max|κ|, is
half the friction limit μ_y g, with downforce not counted:

  v_c = ⌊√(0.5 μ_y g / max|κ|)⌋ = ⌊√(0.5 · 1.6 · 9.81 / 0.1159)⌋ = **8 m/s**.

* Spielberg's largest |κ| is 0.116 1/m (R ≈ 8.6 m), at s ≈ 1 399 m.
* A lap at 8 m/s takes about 540 s, about seven times `nmpc`'s lap.
* The tyre usage is about 0.5 at that point and far lower elsewhere, so the car is far from the limit everywhere.
* If an initialisation lap at 8 m/s does not complete, v_c is lowered in steps of 1 m/s (recorded; check r2).

### 10.3 The arms (NOM only; one deterministic run each)

| arm | problem | start | laps |
|---|---|---|---|
| **R1** (decides the reading) | `lmpc_nominal`: phase E's NLP without the regression, K 20, P 2 | two centre-line laps at v_c | 40 learning laps, consecutive |
| R2 | the same | phase E's initialisation (η_p 0.73, then 0.79 from the first lap's end state) | the same |
| R3 | `atv_nominal`: phase H's ATV-QP without the regression (OSQP, as registered in phase H) | as R1 | the same |

* R2 separates the start from the consecutive laps.
* R3 is E.2's form of the problem: one QP per step along E.2's candidate.
* No speed bound: it is not in the papers.
* Everything else is phase E's:
  * N = 30 × 50 ms, η 0.95, margin 0.6, w_T 100;
  * the weights;
  * the soft terminal hull;
  * the fallback.

**The start state at v_c (R1, R3):** the centre line's point at s = 0 with its heading, v_x = v_c, and v_y = r = δ = 0.
The line lies on the main straight. The reference of the two initialisation laps is n = 0 and ξ = 0 at v_x = v_c
everywhere.

**Consecutive laps (fixed now):**
* **What is stored.** Each lap is simulated from its start state through the line and 250 m beyond, as in phases E and
  I. The lap and its run-out are stored, with the time-to-go continued past the line as now.
* **Where the next lap starts.** The next lap starts from the simulator state at the last 10 ms step before the line,
  within one step (< 1 m) of L.
  * It has a fresh controller. The controller's previous command is the earlier lap's command at that step.
  * Its time is measured over one track length from that state.
* **An approximation of continuous driving.** The data past the line come from the earlier lap's own run-out, driven
  with the store as it was during that lap. They do not come from the next lap's first 250 m.
* **A failure ends the sequence**, as in phases E and I. A failure is off track, a spin, or the time limit, in the lap
  or its run-out.
  * A retry would only repeat it: the simulation and the controller are deterministic, and the store does not change.
  * I.1 continues after a rejected lap by narrowing its envelope; E.4 counts failed trials on hardware. Neither carries
    over without a new mechanism.
* **A lap's time limit** is 1.25 × the slower initialisation lap's time, and at least 121 s.

*Clarification of revision 1 (review m2) – how the laps follow each other, the initialisation laps included:*
* **R1 and R3:**
  * The first v_c lap starts from the start state at v_c.
  * The second v_c lap starts, consecutively as above, from the first lap's state at its last step before the line.
  * Learning lap 1 starts from the second v_c lap's state at its last step before the line.
* **R2:**
  * The η_p 0.73 lap starts from phase E's start state.
  * The η_p 0.79 lap starts from the 0.73 lap's state before the line.
  * Learning lap 1 starts from the 0.79 lap's state before the line, consecutively, as R2 is meant to be.
* **The initialisation laps' controller** is `nmpc`'s tracking NMPC (`FrenetMPC`, mode "track"), with phase E's
  initialisation settings: η 0.95, N = 30 × 50 ms, margin 0.6, and the weights `NMPC_TRACK_W` × c 2. For R1 and R3 its
  reference is n = 0, ξ = 0, v_x = v_c everywhere.
* **The previous command** is carried into the next lap's controller in the same way, for the initialisation laps too.

*Clarification of revision 1 (review m3, part 1) – the flying start is not new.* Each new controller's initial guess
and warm start are phase E's: the constant-speed propagation of the start state at the first solve, then the shifted
solution. Phase E's laps also start at speed, from the 5 % plan's state at the line. The cold first solve of a
consecutive lap is therefore no difference from phases E and I. The only difference is the carried previous command,
and it enters only the first stage's input-rate cost.

*Clarification of revision 1 (review n1) – the size of the time-to-go:* after the 540 s initialisation laps the stored
time-to-go reaches ~540 s, against ~80 s in phase E, so the terminal cost w_T c^T λ is about seven times as large in
the NLP's cost. **w_T stays 100, not scaled.** The smoke and check r2 report the solver failures and the shares of
terminal slack, to show whether this matters.

### 10.4 Reported, per arm

* The lap-time curve over the initialisation and learning laps, and each lap's time over the previous lap's.
* The number of completed learning laps.
* At a failure: the lap, the arclength, the stop reason, and the car's speed against the stored laps' speed there.
* Per lap:
  * the share of solves with terminal slack above 10⁻³, and its maximum;
  * solver failures (fallbacks) and solve times;
  * the largest tyre usage (front, rear) and the smallest edge margin.
* The best lap and the last five laps, against `nmpc`'s 76.40 s and I-1's `pb_nominal` best lap on NOM (77.60 s).
* For R3, the QP failures per lap.

*Clarification of revision 1 (review m3, part 2) – a check of the run-out approximation.* For each pair of consecutive
laps j, j + 1 (descriptive only):
* the time over the first 250 m of lap j + 1, against the time over the same stretch in lap j's stored run-out (L to
  L + 250 m);
* the difference of the states (s, n, ξ, v_x, v_y, r, δ) at the end of that stretch.

*Clarification of revision 1 (review n2) – the cause of a failure.*
* Every failure record carries the stop reason and the number of failed solves (fallbacks) among the last 20 solves
  (1 s) before the stop.
* A failure is classed **"after solver failures"** if at least one of those 20 solves failed, and **"solved, still
  left"** otherwise. For R3 the solver failure is the QP stopping at OSQP's iteration limit; phase H's experience is
  that this kind dominates.
* The same classes are reported for R1 and R2 (the NLP's failures).

### 10.5 Readings (fixed now, before any lap)

**Definitions:**
* **Near the limit:** a learning lap within 3 % of `nmpc`'s 76.40 s, i.e. ≤ 78.69 s. I-1's `pb_nominal` left NOM
  after laps of 77.60 and 77.65 s.
* **Converged:** the last five completed learning laps lie within 0.5 % of each other (max / min − 1 ≤ 0.005).

| reading | condition | what it says about the question |
|---|---|---|
| **(a) reproduced** | 40 of 40 completed, at least one lap near the limit, converged | Under the published conditions the LMPC works on NOM, so phases E's and I's failures belong to their conditions. R2 tells whether the slow start is needed. |
| **(b) fails near the limit** | the sequence ends after at least one lap near the limit | The failure near the limit persists under the published conditions with an exact model. It belongs to this formulation on this car; the differences of 10.6 are untested. |
| **(c) fails before the limit** (split into (c1) and (c2) by clarification m1 below) | it ends before any lap near the limit | This points to a defect in the implementation or in the low-speed regime. It is examined as a diagnostic before any conclusion. |
| **(d) stays away** | 40 completed, no lap near the limit | The method does not reach the limit within 40 laps, so the question is not answered. |
| **(e) near, not converged** | 40 completed, a lap near the limit, not converged | Reported as such; there is no conclusion on stability at convergence. |

R1 decides the check's reading. R2 and R3 are read the same way and reported beside it.

*Clarification of revision 1 (review m1) – reading (c) split in two.* The original (c) would call a sequence that ends
with its best lap between 78.69 s and the starting speed – e.g. 80 s – a problem of the implementation or of the low
speeds. 80 s is not low speed; phases E and I failed in that range. So (c) becomes:

| reading | condition | what it says about the question |
|---|---|---|
| **(c1) fails while approaching the limit** | the best learning lap is ≤ 1.10 × 76.40 = **84.04 s**, but no lap is ≤ 78.69 s | The car fails farther from the limit than in (b), of the same kind as phases E's and I's failures. This is **not** read as an implementation defect. |
| **(c2) fails far from the limit** | the best learning lap is > 84.04 s, or the first learning lap fails | As the original (c): this points to a defect in the implementation or in the low-speed regime, and it is examined as a diagnostic before any conclusion. |

**For R2,** which starts from the 83.1 s lap, (c2) can in practice occur only as a failure on the first learning lap.
A completed learning lap slower than 84.04 s would also count, but phase E's first learning laps took 76.4–77.0 s.

### 10.6 Not claimed; the differences that remain

* **Scope.** There is one deterministic run per arm, on NOM only. So there is no statement about model mismatch (DEV),
  and there are no statistics.
* **A pass is not a candidate.** It has no gate and registers no benchmark. Whether an LMPC under these conditions
  should be benchmarked against `identified` is the user's decision. Such a benchmark would need more than ten laps;
  H-I1 and H-I2 as written compare the tenth.
* **A failure does not refute E.2 or E.4.** These remain unreproduced:
  * the cars' scale and hardware (1:10 BARC; the IAC car only qualitatively);
  * E.2's own regression model (I-2);
  * H.1's safe-set details – the four fastest laps, contiguous windows, the L1 distance on the full state;
  * soft track bounds that let a car pass the edge without ending the run (H.1's lane slack, I.1's −0.02 m acceptance).

### 10.7 Development checks before the registered runs

Records go to `results/CX/I/repro/`, each with the commit and a dirty flag.

* **r0 – the default path unchanged.** With the new options off, these records are reproduced exactly:
  * phase E's `lmpc` K 20 and `lmpc_nominal` K 10 development records on NOM;
  * phase H's `atv_nominal` K 20 record on NOM.
* **r1 – unit tests:**
  * the centre-line reference at v_c and its start state;
  * the consecutive start: the state, its arclength within one step of L, and the earlier lap's command carried over;
  * the lap timed over one track length;
  * the time-limit rule.
* **r2 – the two initialisation laps.** Recorded before any learning lap: v_c's value, and the two laps at v_c on NOM
  (completed, times, tyre usage). They are R1's and R3's first laps.
* **One scratch smoke** of R1 with two learning laps is allowed. It is disclosed and not used for the reading.

### 10.8 Compute and order

1. Review of this revision (packet 3). No code before its APPROVE.
2. Code:
   * options of `cx_e.learn`, off by default: consecutive laps, a centre-line start at v_c, the number of learning
     laps, and the lap's time limit;
   * the arms in `experiments/cx_i.py`;
   * `scripts/run_cx_i.py repro` and `scripts/report_cx_i.py repro`;
   * unit tests.
3. r0–r2, then R1–R3 in parallel: three processes, Windows `.venv`, single-threaded BLAS, one process per arm, in the
   main checkout (a diagnostic, as the development checks).
   * Phase I's records give ~0.2 s per NLP solve, so a 540 s lap is ~11 000 solves, ~35–40 min.
   * R1's 40 learning laps are estimated at 5–12 h, depending on how fast it learns.
4. The results and the reading go to the reviewer (packet 4), then to the user.

*Clarification of revision 1 (review n3) – no wall-clock limit.* If R1 learns slowly, the first learning laps take
close to 540 s each (35–40 min), and 40 laps may take more than 12 h. **No wall-clock limit is set.** A run that takes
longer than the estimate is completed as usual, and the overrun is noted in the records.

### 10.9 Results (2026-10-30; packet 4) – **R1 reads (c1); R2 and R3 read (c2), each examined: neither is a low-speed or consecutive-lap defect**

**Records** (git-ignored) are in `results/CX/I/repro/`. Each carries its commit and a clean dirty flag (`meta_*.json`).

| record | commit | content |
|---|---|---|
| `laps.jsonl`, `progress_R*.jsonl`, `log.txt` | b34eb97 | R1–R3, run in parallel 19:40–22:42; 0 error records. Wall time: R1 3.0 h, R2 0.1 h, R3 2.6 h |
| `report/` (`report_cx_i.py repro`) | – | `CX_I_repro.md`, `readings.json` |
| `r0.json`, `r2.json` | b34eb97 | checks r0 and r2 |
| `r2_usage.json`, `r2_usage.npz` | 418bf3a | r2: where the v_c laps' tyre usage arises (descriptive) |
| `r2_diag.json` | bca7802 | R2's (c2) examined: two neighbours |
| `snap/` | bca7802 | R1 driven again through learning lap 3, the store pickled after each lap; its lap times equal R1's record |
| `replay_R3_learn1.json`, `replay_R1_learn4.json` | aae7f30 | R3's first and R1's fourth learning lap driven again from the snapshots, each equal to its record |

The diagnostics (`r2_usage`, `r2_diag`, `snap`, the replays) were written and run after the readings were known. They
are descriptive and change no reading.

**The scratch smoke** (`results/CX/I/repro_smoke/`, b34eb97) was R1 with two learning laps: 538.51, 538.77, 150.81 and
88.63 s. The registered R1 reproduced these laps exactly. The smoke is not used for the reading.

#### 10.9.1 Checks r0–r2

1. **r0 – the default path is unchanged.** With the new options off, the three NOM records are reproduced lap for lap
   (`all_equal: true`):
   * phase E's `lmpc` K 20: 86.88 and 83.16 s, then off on learning lap 1;
   * phase E's `lmpc_nominal` K 10: 86.88, 83.16 and 76.64 s, then off on learning lap 2;
   * phase H's `atv_nominal` K 20: 86.88 and 83.16 s, then off on learning lap 1.
2. **r1 – unit tests.** `tests/test_cx_i.py` has 13 tests, 6 of them new. The full suites passed at b34eb97, and at
   aae7f30 as the packet quotes.
3. **r2 – the two initialisation laps.**
   * **v_c and completion.** v_c's rule gives 8 m/s; max|κ| is 0.1159 1/m at s ≈ 1 399 m. Both laps at v_c completed:

     | lap | time | NMPC fallbacks | smallest edge margin | largest tyre usage (front / rear) |
     |---|---|---|---|---|
     | 1 | 538.51 s | 118 of 11 392 | 3.33 m | 0.935 / 0.950 |
     | 2 | 538.77 s | 105 of 11 405 | 3.46 m | 0.951 / 0.955 |

     v_c stays 8 m/s; section 10.2 lowers it only if a lap does not complete.
   * **The tyre usage is not the "about 0.5" section 10.2 expected.** The first lap, driven again
     (`repro-r2-usage`, 538.5055 s, as recorded), shows that the peaks are longitudinal:
     * They come from the rear axle on the straights (κ ≈ 0). The tracking NMPC holds v_c with drive forces of
       ~6.8 kN, often with brake force applied at the same time. v_x varies between 5.87 and 9.05 m/s.
     * The longitudinal part exceeds 0.5 on 29 % of the steps. The lateral part exceeds 0.5 on 0.2 % of them.
     * The lateral part's largest values:
       * front axle: 0.745 at turn-in (s 459 m), and 0.73 at the hairpin (s ≈ 1 399 m);
       * rear axle: 0.38.
     * So section 10.2's estimate holds for the cornering load on the rear axle. It does not hold on the front axle, and
       not for the tyre as a whole: the start is far from the cornering limit, not from the tyre's limit.

#### 10.9.2 The arms (section 10.4)

| arm | initialisation laps | learning laps completed | lap times (learning) | end | reading |
|---|---|---|---|---|---|
| **R1** (NLP, start at v_c) | 538.51, 538.77 s | **3 of 40** | 150.81, 88.63, **80.39** | lap 4: off at 3 973 m, *solved, still left* (0 of the last 20 solves failed); v_x 33.7 m/s, the stored laps 31.6 | **(c1)** |
| R2 (NLP, phase E's start) | 86.88, 83.20 s | 0 of 40 | – | lap 1: off at 471 m (turn 1), *after solver failures* (1 of 20); v_x 26.0, stored 26.5 | (c2) |
| R3 (ATV-QP, start at v_c) | 538.51, 538.77 s (as R1) | 0 of 40 | – | lap 1: off at 215 m at 8.1 m/s, *after solver failures* (20 of 20) | (c2) |

**Per learning lap (R1, and R2's and R3's single failed lap):**

| | R1 lap 1 | R1 lap 2 | R1 lap 3 | R1 lap 4 (failed) | R2 lap 1 | R3 lap 1 |
|---|---|---|---|---|---|---|
| time over the previous lap | 0.280 | 0.588 | 0.907 | – | – | – |
| share of solves with terminal slack > 10⁻³ | 0.009 | 0.040 | 0.152 | 0.215 | 0.257 | 0.000 (the QP failed) |
| largest terminal slack | 6.15 | 6.95 | 8.16 | 8.80 | 13.3 | 3·10⁻⁷ |
| solver failures / solves | 5 / 3 198 | 4 / 1 951 | 8 / 1 717 | 5 / 1 439 | 7 / 148 | **533 / 538** |
| mean solve time | 244 ms | 189 ms | 177 ms | 172 ms | 377 ms | 286 ms |
| largest tyre usage (front / rear) | 0.980 / 0.991 | 1.011 / 0.983 | 1.018 / 0.990 | 1.004 / 0.994 | 1.030 / 0.990 | 0.880 / 0.565 |
| smallest edge margin | 2.90 m | 1.66 m | 0.59 m | −0.04 m | −0.05 m | −0.00 m |

* **Clarification n1 (the 540 s time-to-go):**
  * R1's NLP failed on at most 0.5 % of a learning lap's solves.
  * Its terminal slack grew lap by lap (0.9 % → 21.5 % of the solves), as in phases E and I.
  * For R3, the time-to-go's size is not the cause either; see the probe in 10.9.3.
* **Clarification m3 – the run-out check.** Lap j's stored run-out is compared with lap j + 1's first 250 m: the time
  over the 250 m, and the speed difference at 250 m (lap j + 1 minus lap j).

  | pair | run-out of lap j | first 250 m of lap j + 1 | Δv_x at 250 m |
  |---|---|---|---|
  | v_c 1 → v_c 2 | 31.08 s | 31.20 s | −0.15 m/s |
  | v_c 2 → learning 1 | 31.44 s | 9.40 s | +21.95 m/s |
  | learning 1 → 2 | 9.05 s | 4.84 s | +52.90 m/s |
  | learning 2 → 3 | 8.86 s | 4.37 s | +65.29 m/s |
  | learning 3 → 4 | 5.41 s | 3.47 s | +69.43 m/s |
  | R2: 0.73 → 0.79 | 3.18 s | 3.18 s | +0.09 m/s |
  | R2: 0.79 → learning 1 | 3.14 s | 3.14 s | +0.10 m/s |

  * After the slow start, every learning lap's run-out braked back towards the v_c laps' speeds after the line. The data
    past the line it was driven to are the earlier laps' run-outs, which keep the v_c laps' 8 m/s.
  * The next laps' first 250 m took 3.5–4.8 s, so the laps crossed the line at speed.
  * The run-out approximation is therefore poor after the slow start, as section 10.3 allowed. It stays close for R2's
    start, as in phases E and I.
  * Whether the slow data past the line cost time before the line is not measured.

#### 10.9.3 Reading (c2) examined (clarification m1)

**R2 – two neighbours** (`repro-r2-diag`, two learning laps each):

| run | learning lap 1 | learning lap 2 |
|---|---|---|
| R2 as registered (K 20, consecutive laps) | off at 471 m | – |
| K 20 from phase E's start (no consecutive laps) | off at 442 m | – |
| K 10 with consecutive laps | 76.72 s | off at 471 m (*after solver failures*; v_x 26.5, stored 26.5) |
| phase E's `lmpc_nominal` K 10 (record) | 76.64 s | off at 470.8 m |

* K 20 also fails on lap 1 from phase E's start (at 442 m, where phase E's `lmpc` K 20 left). **So the consecutive
  start is not the cause.**
* With K 10, the consecutive laps repeat phase E's K 10 within 0.08 s and 0.3 m.
* R2's first learning lap jumps from 83.2 s to the limit (K 10: 76.72 s ≤ 78.69 s) and fails at turn 1, on its first
  or second lap. This is phase E's and I's failure near the limit. It is not a defect of the implementation or of low
  speeds. Mechanically R2 still reads (c2), because its K 20 failed on lap 1.

**R3 – its first learning lap driven again** (`repro-replay --which R3_learn1`). The replay equals the record: off at
215.17 m, 538 solves, 533 failed.
* **Solver status.** All 533 failures are OSQP's *maximum iterations reached* (20 000, phase H's setting); 5 QPs solved.
* **What the failures do.** A failed QP keeps the candidate (phase H), so the car is driven nearly open loop on the
  stored lap's shifted inputs. The prediction's median lateral error at N was 0.46 m. The car drifted off at 8 m/s.
* **Probes on the first QP:**

  | probe | time-to-go c | result |
  |---|---|---|
  | as run | 536.6–537.7 s | iteration limit |
  | c shifted by its minimum (the same QP's solution, since the λ sum to one) | 0–1.17 s | **iteration limit** |
  | OSQP's iteration limit × 10 (200 000) | 536.6–537.7 s | **solved, in 2.4 s** (48 × the 50 ms step) |

* **So R3's (c2) is the QP solver's convergence** – phase H's dominant failure kind – present already at 8 m/s and with
  the first QP. It is not clarification n1's scale, and it is not the learning.
  * Phase H's ATV-QP with its OSQP settings cannot drive the v_c start. R3 therefore says nothing about the LMPC.
  * Why OSQP converges so slowly here is not tested. One untested possibility: the two v_c laps are nearly identical,
    so the 40 hull points come in near-duplicate pairs.

#### 10.9.4 R1's failure, driven again (descriptive; `repro-replay --which R1_learn4`)

The replay equals the record: off at 3 973.21 m, 1 439 solves, 5 failed. IPOPT solved 1 434 problems; 5 stopped at its
iteration limit, one of them at 3 930.6 m.

**Approaching the corner at ~3 970 m** (braking from 59.3 m/s at 3 913 m):
* From s ≈ 3 922 m (56.7 m/s), the terminal slack became nonzero and rose to 8.2.
* From s ≈ 3 936 m, the soft track bound's slack in the prediction grew to 1.55 m by the edge. The margin is 0.6 m, so
  the optimiser's own plan crossed the edge.
* IPOPT's iterations rose from ~25 to ~120.
* *"Solved, still left"* therefore means that the soft-constrained problem was solved, and its solution left the track.
  The car arrived faster than the stored laps (33.7 against 31.6 m/s at the edge).

**Earlier on the same lap:**
* At turn 1 (s 407–446 m), the prediction's track slack reached 0.91 m – past the edge by ~0.3 m – and the car stayed
  on.
* The terminal slack was nonzero in seven stretches of the lap, the largest 8.8 at turn 1.

**This is phase E's and I's failure near the limit**, on NOM with an exact model up to the integration:
* The terminal set is a soft convex hull of the K nearest stored states in s.
* It gives no recursive feasibility once the car is faster than the stored laps.

#### 10.9.5 Reading of the check, and what is not claimed

**R1 decides the check's reading: (c1) – it fails while approaching the limit.** The best learning lap was 80.39 s,
5.2 % above `nmpc`'s 76.40 s, and the fourth learning lap left the track.
* **Under the published conditions, our LMPC learns quickly and then fails as in phases E and I.** The published
  conditions here are a slow start far from the cornering limit, consecutive laps, and an exact model on NOM.
  * The learning laps went 538 → 151 → 89 → 80 s.
  * The failure came on learning lap 4, with the solver succeeding: the solution itself crossed the edge.
* **So phases E's and I's failure does not belong to their start conditions.**
  * The slow start and the consecutive laps do not remove it. They only move it from lap 1 or 2 to lap 4.
  * By clarification m1, (c1) is not read as an implementation defect.
* **R2 and R3, beside it:**
  * R2's (c2) is the same failure on the first lap: the K 10 neighbour reaches the limit on lap 1.
  * R3's (c2) is phase H's QP solver, which cannot drive even the v_c start.

**Not claimed (section 10.6):**
* one deterministic run per arm, on NOM only, with no statistics;
* a pass would not have been a candidate, and there is no gate;
* E.2 and E.4 are not refuted. These remain unreproduced:
  * their scale and hardware;
  * E.2's regression;
  * H.1's safe-set details;
  * soft track bounds that let a car pass the edge without ending the run.
  The last is exactly where R1 ended: its own plan crossed the edge by ~1 m.

**Also disclosed:**
* **Section 10.2 misjudged the start's tyre usage** (10.9.1). The start is far from the cornering limit, but the
  tracking NMPC used up to 0.95 of the tyre longitudinally.
* **The run-out approximation is poor after the slow start** (10.9.2, m3).
* **The failure records' code path** (n2) ran for the first time in the registered runs. R1's lap 4, R2 and R3 each
  produced a complete record.
* **The pytest results** are quoted from pytest's summary line (`-rs`).
  * At b34eb97: `.venv` 144 passed, 7 skipped (gymnasium) in 698.06 s; `.venv-rl` 151 passed in 719.24 s.
  * At aae7f30: `.venv` 144 passed, 7 skipped (gymnasium) in 599.07 s; `.venv-rl` 151 passed in 612.20 s.
  * `test_no_course_content` passed. No results or data file is tracked.

**What follows is the user's decision (section 10, "Its place among phase I's rules"):**
* close the LMPC line; or
* ask for a candidate, registered by a revision on this result.

Per packet 2's review, such a candidate would have to say how it removes this failure near the limit on NOM: the soft
terminal hull of K nearest states without recursive feasibility once the car is faster than its data.
