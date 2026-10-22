# CX-E protocol – learning MPC, lap to lap (control expansion, phase E)

Status: **draft, revision 0 (2026-10-22), for review** – written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §E) and **before any phase-E code**: no module, test, runner
or lap of phase E exists (section 9). Branch `cx/E-lmpc` (from `main` ae87f41, the merge of phase D).
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
  second (fixed; time dominates the input costs).
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

## 6. Statistics

Exact signed-rank and McNemar, exact intervals, Holm over H-E1 and H-E2. Secondary, uncorrected (5): on CET `lmpc` –
`nmpc` (McNemar and signed-rank, `lmpc`'s tenth learning lap), `lmpc_nominal` – `lmpc` (both, tenth laps), and `lmpc`'s
tenth – first learning lap (signed-rank on the cars that complete both: the improvement). The secondary conditions:
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
2. **Regression:** on stored laps of a DEV car, the corrected step's one-step residual in (v_x, v_y, r) against the
   nominal step's (in- and out-of-sample on the next lap); Γ = 0 where no sample is in the bandwidth.
3. **Initialisation:** the two laps on NOM and the DEV cars (completed? lap times).
4. **Learning laps:** NOM and DEV with K = 20, 10 learning laps each: the lap-time curves, completions, terminal slack,
   solve times.
5. **Gate:** on NOM `lmpc` completes all 10 learning laps and its tenth lap is faster than its first; on DEV it completes
   the tenth lap on at least 3 of the cars whose initialisation completed; otherwise an amendment before any CEV lap.
