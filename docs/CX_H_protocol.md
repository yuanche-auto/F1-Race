# CX-H protocol – the ATV-QP learning MPC (control expansion, phase H – the first frontier method)

Status: **binding, revision 0 (2026-10-28; 1a5c538, APPROVE)** with the clarifications of its review (m1–m3, n1, n2;
marked *"clarification of revision 0"*), written down before any phase-H code. Written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §H, on top of §E) and before any phase-H code: no module,
test, runner or lap of phase H exists. Branch `cx/H-atv-lmpc` from `main` c157738 (the merge of phase G). The user chose
this method (2026-10-23) from the follow-ups of CONTROL_METHOD_MATRIX.md: phase E's first follow-up candidate (CX-E
amendment E1, review e3).

## 0. What phase E left

* Phase E's `lmpc` failed its development gate and was closed without the benchmark – **H-E1 and H-E2 were not tested**
  (docs/CX_E_protocol.md, amendment E1).
  * `lmpc` is the NMPC's NLP (IPOPT, the nominal nonlinear model with E.4's affine error correction) with a soft
    terminal convex hull of stored states and their costs-to-go.
  * With the registered settings no sequence completed more than one learning lap (the best diagnostic variant
    three). The first learning lap jumped from 83 s to ~76.5 s, and the terminal slack grew lap by lap until the car
    left the track near the limit.
  * Review e1 put the result's scope on that implementation. Its suspected cause is a hypothesis: **a convex
    combination of stored states is a feasible state for a linearised (affine time-varying) model, as E.2 solves the
    problem, but need not be for the nonlinear one.**
* The published codes (§H) solve the racing LMPC as a **QP on a model linearised along a shifted candidate** – H.1, and
  H.2's linearised mode. H.1 / E.2 build the candidate's terminal point from the previous λ's successors. Phase E
  differed in both respects.
* `identified` (one exploration lap, identification, full re-plan; L8's pipeline) completed 50 of 50 cars in phase D.
  It is the comparison phase E registered.

## 1. Question and hypotheses

*With the learning MPC solved as E.2 does – one QP per step on the model linearised along E.2's candidate, everything else
as phase E registered – does it learn over ten laps where the NLP did not, and how does its tenth learning lap compare
with the one-lap identification and full re-plan?*

The phase is gated: the development gate of phase E (section 9) must be passed before any test lap.

*Clarification of revision 0 (review m1) – the gate, fixed in advance:*
* The gate is judged **only on the registered development laps of section 9.3** – K = 20, `atv` and `atv_nominal`, NOM
  and the five DEV cars. No other setting may be tried in order to pass it. Diagnostic variants may be run, as
  diagnostics only.
* **If the gate fails, the default is to close phase H without the benchmark**, as amendment E1's option 1, and to
  report the development results as they are. Only a substantially new design would be registered, as a new protocol.
  It would not be an amendment of this one.

* **H-H1 (primary):** on the test cars HT (section 2), the lap time of `atv`'s tenth learning lap and `identified`'s lap
  differ, on the cars both complete. Exact signed-rank (mid-ranks, the exact distribution), two-sided.
* **H-H2 (primary):** `atv`'s tenth learning lap and `identified` differ in the number of completed laps on HT. Exact
  McNemar, two-sided.

Holm over H-H1 and H-H2 (α 0.05), each also against 0.025. No direction is predicted. Phase E's clarifications carry
over:
* H-H2's exposure is unequal against `atv` – the tenth lap counts only if all ten learning laps survive (E m1).
* H-H1 is conditional on both completing, so it is measured on survivors (E m2).
* The comparison is not an equal-information one (E section 1).

Scope: Spielberg, the nominal model, η = 0.95, N = 30 × 50 ms, offline (no time budget; L9's rule is not applied).

## 2. Sets

* **Tuning HV:** phase A's CXV cars (10; the tuning set of phases A–C and E's CEV), constant.
* **Test HT** (new): `sample_plants(nominal, 50, seed 382)[1:]`, constant parameters.
  * Seed 382 is used by no L-layer or phase (car seeds in use: 0–10, 99, 301–379).
  * Phase E's CET (seed 342) is not used. It was registered for E's benchmark and never driven; a new seed keeps the two
    registrations apart.
* **Development DEV:** phase C's CXDEV cars (seed 399; the cars of phase E's development checks, whose NLP records
  exist) and **NOM** (the nominal car).
* **Secondary conditions** (after the primary report): `HT_LAG30` (the HT cars with the L6 30 ms steering lag) and
  `HT_WET85` (μ × 0.85), with `nmpc`, `identified`, `atv`, `atv_nominal` – phase E's secondary design.
* **One iteration = one lap from the same initial state**, as phase E section 2:
  * the 5 % plan's state at s = 0;
  * the time at s = L scored;
  * 250 m past the line driven and stored, unscored;
  * a crash ends the car's sequence;
  * the unwrapped arclength through the line as E's clarification m3;
  * time limit 121 s.

## 3. Controllers

| controller | model | terminal set / plan | role |
|---|---|---|---|
| `nmpc` | nominal | 5 % plan | baseline |
| `identified` | identified offline | full re-plan (L8's pipeline, c 2, margin 0.6, as phases D / E) | primary comparison |
| `atv` | nominal + E.4's affine error regression, **linearised** | local safe set and cost-to-go (E.2) | **primary** |
| `atv_nominal` | nominal, linearised | the same | secondary (the regression removed) |
| `lmpc` | phase E's NLP, unchanged | the same | secondary: the formulation ablation (section 4) |

**Initialisation** as phase E (3.1): two laps of `nmpc`'s NMPC tracking the plans at η_p 0.73 and 0.79. A failed
initialisation lap is not stored.

### 3.1 The ATV-QP learning MPC (`atv`; E.2's formulation, literature E.2, E.4, H.1, H.2)

**The optimal control problem is phase E's, row for row** (`control/lmpc.LearningMPC`, CX-E section 3.2):
* the dynamics – RK4 of 50 ms plus E.4's correction Γ_k z_k on (v_x, v_y, r);
* the friction ellipses at η 0.95, slip, power and axle-force rows with their slacks;
* the axle track limits with margin 0.6 and their slacks;
* λ ≥ 0, 1ᵀλ = 1, and the soft terminal condition (s, n, ξ, v_x, v_y, r)_N = Dλ + slack;
* the cost w_T cᵀλ + the input-rate costs + the exact slack penalties;
* w_T, the weights and the regression (M = 80, h_r = 1, ε = 10⁻⁴) are phase E's values.

What changes is how it is solved, and the candidate.

* **One QP per solve (50 ms)**, from the candidate z̄:
  * every constraint row g is replaced by its first-order expansion g(z̄) + ∇g(z̄)(z − z̄) – the dynamics, the ellipses,
    the slip and axle-force rows, the track limits (with sin ξ, the track widths and the curvature at the candidate's s);
  * the cost is phase E's, which is quadratic and linear in the variables, so it is kept exactly, plus a regularisation
    10⁻⁶ ‖z − z̄‖² for strict convexity;
  * the bounds are phase E's.
  This is one Gauss–Newton (RTI-type) step of phase E's NLP. Without the constraints' curvature it is the
  affine-time-varying QP of E.2 / H.1.
* **The candidate (E.2, H.1):**
  * the previous QP solution's states and inputs shifted by one step;
  * its terminal state z_t = Σ λ_i succ(D_i) – the stored states one 50 ms step after the selected points, weighted with
    the previous λ – and its last input the stored successors' inputs, weighted the same way;
  * at a lap's first solve, the fastest stored lap's first N + 1 states and N inputs;
  * slacks at 0.
  * *Clarification of revision 0 (review m2) – the edges:*
    * each stored lap's **last point** (the end of its 250 m run-out) has no successor, so it is **excluded from the
      local safe set**; the K nearest are chosen among the others;
    * if a lap's **first** QP fails, there is no previous plan to shift, so **the candidate's first input** (the fastest
      stored lap's first input) is applied; it is counted as a fallback.
* **The local safe set:** the K nearest stored states by s to z_t's s from each of the last P = 2 stored laps (phase E's
  rule, with E.2's query point z_t instead of phase E's s_N + h v_x).
* **The regression** is evaluated at the candidate (phase E: at the shifted solution). `atv_nominal` uses Γ ≡ 0.
* **Solver:** OSQP (CasADi's conic interface; eps_abs = eps_rel = 10⁻⁵, polish on, max_iter 20 000). A QP that does not
  return "solved" applies the shifted previous plan (L6's handling, as phase E) and is counted.
* Traction control (0.99), the applied first input, and the recording of the lap are as in phase E.
* *Clarification of revision 0 (review m3) – attribution:* the module's docstring states that the formulation follows
  E.2 (and E.4 for the error model), that urosolia/RacingLMPC (H.1) and MPC-Berkeley/Racing-LMPC-ROS2 (H.2) were read and
  no code was copied, and cites the papers (E.1, E.2, E.4). H.1's licence asks for attribution to UC Berkeley and the
  papers. Any structural borrowing – the successor-weighted terminal candidate is H.1's / E.2's – is attributed there.

**Fixed now, not tuned:** the regularisation 10⁻⁶; the OSQP settings; P = 2; everything inherited from phase E.

### 3.2 What is and is not claimed

* **No guarantee:** E.2 states that feasibility holds only if the linearisation is accurate over the local set; the
  soft terminal condition and the learned correction give up the rest (E.4).
* **Deviations from E.2:**
  * the nominal physics model with E.4's correction, as phase E, not E.2's full regression of the velocity states;
  * an exact-penalty-softened terminal condition (as H.1, H.2 and phase E; E.2's paper states it as hard);
  * N = 30 × 50 ms against E.2's 12 × 100 ms;
  * every lap from the same initial state with a 250 m run-out, not consecutive laps.
* The ablation against phase E's NLP (`lmpc`, section 4) shows whether the formulation – the hypothesis of e1 – is what
  changes the outcome. It does not separate the linearisation from E.2's candidate; the two change together, as one
  design.

## 4. Tuning (HV only) and secondary arms

* **Tuning:** `atv` with K ∈ {10, 20, 40}, 10 cars × 12 laps × 3 = 360 laps. Phase E's rule:
  * the most completed tenth learning laps;
  * ties by the mean tenth lap time on the cars every tied setting completes;
  * then K = 20, then the smaller K.
  `atv_nominal` and `lmpc` use `atv`'s K. The choice and the records go to the reviewer before any HT lap.
  *Clarification of revision 0 (review n2):* the grid stays phase E's {10, 20, 40} and is not changed to the {5, 10, 20}
  tried in E's development diagnostics. E's grid was registered, and the candidate is defined differently here, so E's
  observation under the NLP that smaller K did better need not carry over.
* **Secondary on HT**, after the primary report:
  * `lmpc` (phase E's NLP, the chosen K) – the formulation ablation;
  * the secondary conditions of section 2.

## 5. Always reported

As phase E section 5, per car:
* the **lap-time curve** over the initialisation and learning laps (time, completed, stop reason);
* the **survival curve** (cars still driving after each learning lap), the first failed learning lap, and the completion
  rate of each learning lap conditional on entering it;
* initialisation failures counted apart from failures while learning;
* the learning lap at which `atv` first beats `identified` (if ever);
* the terminal slack (the share of solves above 10⁻³; the maximum);
* the regression's fallback share (stages with Γ = 0);
* QP failures (fallbacks) and the solve time;
* **the linearisation error** of each solve – the nonlinear model's rollout of the QP's inputs against the QP's predicted
  states, at N (position, v_x), as a median and P95 per lap. The candidate's validity is the mechanism under test.
  *Clarification of revision 0 (review n1):* it is reported **per lap, next to that lap's outcome**, so that a failed
  lap's linearisation error can be compared with that of the completed laps;
* for the tenth lap, the cars `atv` completes against those it does not (`identified`'s lap time, the seven parameter
  factors; E m2).

## 6. Statistics

Exact signed-rank and McNemar, exact intervals, Holm over H-H1 and H-H2.

Secondary, uncorrected:
* on HT, `atv` – `nmpc` (McNemar and signed-rank, `atv`'s tenth lap);
* `atv_nominal` – `atv` (tenth laps);
* `atv`'s tenth – first learning lap (signed-rank on the cars completing both; survivors, E n1);
* `atv` – `lmpc`:
  * completions of the first and of the tenth learning lap (McNemar);
  * the survival curves side by side;
* the secondary conditions: completions per controller and `atv` – `identified` McNemar per condition.

## 7. Second environment

As before (deferred by the user): H-H1 and H-H2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). **No code before its APPROVE.**
2. Code:
   * `control/lmpc.py` gets an ATV-QP solver of `LearningMPC`'s problem (a new class or option; phase E's NLP path
     unchanged, checked by reproducing a phase-E development record);
   * the candidate construction and z_t-centred selection in the controller;
   * `experiments/cx_h.py` (sets, arms; phase E's `learn` reused with the controller swapped);
   * `scripts/run_cx_h.py`, `scripts/report_cx_h.py`, `scripts/cx_h_dev.py`, unit tests;
   * the development checks of section 9; packet 2 before any HV lap.
3. HV tuning (360 laps; a QP solve is expected to take milliseconds, a car's twelve laps minutes); the choice to the
   reviewer.
4. HT from a pinned worktree (Windows `.venv`, single-threaded BLAS, one process per job):
   * `nmpc` and `identified`, 51 laps each;
   * `atv` and `atv_nominal`, 51 × 12 laps;
   then the report and its review.
5. Secondary (`lmpc` on HT; HT_LAG30 and HT_WET85, 2 × 50 × the four controllers); review; final review; merge.

## 9. Development checks (planned)

On NOM and DEV only (records in `results/CX/H/dev/` with commit and dirty flag):

1. **Linearisation:**
   * the QP's constraint Jacobian against finite differences of phase E's constraint function at random candidates;
   * at a candidate that satisfies the nonlinear constraints (a stored lap's own states, with its terminal state among
     the selected points), the QP accepts Δz = 0 – all slacks and the terminal slack 0.
2. **Phase E's NLP unchanged:** one phase-E development sequence re-run reproduces its record.
3. **Laps:** `atv` and `atv_nominal` with K = 20 on NOM and the five DEV cars – lap-time curves, terminal slacks,
   linearisation errors, solve times. The same cars' phase-E NLP records (`results/CX/E/dev/laps.jsonl`) are shown
   beside them, for development only.
4. **Gate (phase E's, unchanged):**
   * on NOM, `atv` completes all ten learning laps and its tenth lap is faster than its first;
   * on DEV, it completes the tenth lap on at least 3 of the cars whose initialisation completed.
   Otherwise an amendment before any HV lap.
