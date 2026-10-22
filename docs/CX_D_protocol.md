# CX-D protocol – adaptive MPC and online identification (control expansion, phase D)

Status: **binding, revision 0 (2026-10-22; 314129d, APPROVE)** with the clarifications of its review (m1–m3, n1–n3;
marked *"clarification of revision 0"*), written down before any phase-D code. Revision 0 was written after the
literature check ([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §D) and before any phase-D code: no module,
test, runner, plan or lap of phase D existed (section 9). Branch `cx/D-adaptive` (from `main` 4b4e091, the merge of phase C).
Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

## 0. What L0–L9 and phases A–C left

* **L4 / L6 / L8 identify the car offline:** one exploration lap (the NMPC on the 12 % plan), least squares (noise-free)
  or an EKF (under the L3 noise) on its data, a full re-plan of the minimum-time OCP with the identified car (~1 min,
  ~7.4 GB), then the NMPC with the identified model. It closes the gap to the true model on the L2 / L4 cars (L4: 25 of
  25 against the nominal NMPC's 13) and on wet Spielberg (L8: 50 of 50), but it needs the extra lap, it fixes one
  parameter vector for the whole lap, and so it fails where the condition changes along the lap: L8's wet patches,
  11 of 30 (docs/L8.md). Nothing in the project updates the model **while it drives**.
* **Phase C** robustified `nmpc` against the L2 parameter ranges by a tube: 49 of 50 new L2 cars against `nmpc`'s 16,
  at +1.61 s – with the ranges handed to it as a prior, and no protection outside them.
* The roadmap's risk for this phase: "keeping static identification, state estimation and online model update apart".
  Here the states are measured (noise-free in the primary sets); the phase is about the **online parameter update**
  and what the controller does with it. Static identification (`identified`) is a comparison, not a component.

## 1. Question and hypotheses

*Does an NMPC that estimates the car's parameters while it drives – and updates its prediction model and its plan
from the estimate – complete more laps than `nmpc`, (a) on new cars whose parameters are unknown but constant, without
an exploration lap, and (b) on the same cars when the grip falls during the lap?*

* **H-D1 (primary):** on the 50 test cars CDT (section 2, constant parameters), `adaptive` and `nmpc` differ in the
  number of completed laps. Exact McNemar, two-sided; holds if significant at its Holm level; direction reported.
* **H-D2 (primary):** on the drift set CDD (the same 50 cars, the friction factor falling during the lap, section 2),
  `adaptive` and `nmpc` differ in the number of completed laps. Exact McNemar, two-sided.

Holm over H-D1 and H-D2 (family-wise α = 0.05), the stricter decision (each at 0.025) alongside. No direction is
predicted. *Clarification of revision 0 (review m2):* CDT and CDD are the same 50 cars, so H-D1 and H-D2 are not
independent; Holm stays valid under any dependence, but the two tests are not read as two independent pieces of
evidence. Lap time is secondary here (section 6): the phase's question is whether the lap is completed. Power: as
phases B / C (at least 7 discordant cars, all one way, for Holm's first level); `nmpc` completed 16 of 50 phase-C test
cars and 0 of 50 wet L8 laps, so many discordant cars are possible in both sets.

Scope: Spielberg, the nominal plan family (section 3.3), `nmpc`'s objective and margin (phase A's c 2, margin 0.6),
η = 0.95, offline as phases A–C (IPOPT without a time budget). **Information budget:** the `adaptive*` controllers get
the track, the plan library, the measured state, their own applied inputs, the model *structure* and the parameter
bounds of L4's least squares (`identify.LO / HI`, wider than the L2 box) – no exploration lap, no true parameter;
`adaptive_oracle` gets the true parameters and the true friction factor at the car's position (oracle); `tube` gets
the L2 ranges (phase C); `identified` gets one exploration lap under the car's initial condition; `nmpc` gets none of
these. The estimator's model is the simulator's own step (section 3.1): with exact states and the exact structure,
the primary sets measure **the ceiling of online identification**, not its robustness – structure mismatch (the
steering lag) and noise are secondary conditions. *Clarification of revision 0 (review m1):* with seven parameters,
three outputs per step and exact states, the RLS is expected to identify the car quickly wherever the tyres are
excited, and L4 already showed that the NMPC with the true model completes these cars (25 of 25). What H-D1 adds is
only that this is reached **online, without the exploration lap**; the main quantity for reading H-D1 is therefore the
gap `adaptive` – `adaptive_oracle` (completions and lap time), which the report and the Reading state first.

## 2. Sets

* **Tuning set CDV** (20 laps per setting): phase A's CXV cars (`cx_a.cases("CXV")`, seed 301, 10 cars) with constant
  parameters (`CDV_C`), and the same 10 cars with one friction drift each (`CDV_D`, drift seed 334).
* **Test set CDT** (new): `sample_plants(nominal, 50, seed 332)[1:]`, constant parameters.
* **Test set CDD** (new): the CDT cars, each with one friction drift (drift seed 333, one draw per car in car order).
* **Friction drift:** the friction factor f(s) on both axles' μ_x, μ_y at the car's position (as L8's wet patches,
  `StochasticSim` with a friction field and no gusts, no noise): f = 1 up to the onset s_on, then linear to f_end over a
  length ℓ, then f_end to the end of the lap; s_on ~ U(600, 2 600) m, ℓ ~ U(100, 1 500) m (from a sudden drop, as a
  rain shower or a wet patch, to a gradual one, as tyre wear), f_end ~ U(0.75, 0.90) (L8's wet levels span 0.75–0.85).
  The plant's friction is the car's μ factor times f(s). The drift is one-way within the lap (it does not recover).
  The final grip μ·f_end lies in [0.69, 0.93]; below 0.70 (L4's lower bound for μ, section 3.1) an estimate sits on
  the bound, as for 9 of L8's wet 0.75 cars, which all completed. *Clarification of revision 0 (review n3):* the drift
  only falls within the lap and changes only μ_x and μ_y; "only the grip changes" is also the premise of `adaptive`'s
  forgetting structure (λ_μ < 1, λ = 1 for the rest, section 3.2) – a design that fits this condition by construction,
  which favours `adaptive` on CDD.
* **Development set DEV:** phase C's CXDEV cars (`sample_plants(nominal, 5, seed 399)[1:]`), constant and with drifts
  (drift seed 335) – for the development checks of section 9 only; never tuned on, never tested on.
* **NOM:** the nominal car, constant (reported separately; in the CDT run).
* Seeds 332–336 are used by no L-layer, by phases A–C or by L8's patch seeds (checked in src, scripts, tests and docs
  at 4b4e091). Seed 336: the model bank (section 3.5).
* **Secondary conditions** (after the primary report, as phases A–C), with `nmpc`, `tube`, `adaptive` and
  `adaptive_oracle` (their CDT settings): `CDT_NOISE` (*clarification of revision 0, review m1:* the CDT cars with the
  L3 measurement noise only – no gusts, no friction field – so that it tests the estimator under noise; the bias of μ̂
  is reported, the errors-in-variables effect L6 found; noise seed 332 000 + car), `CDT_LAG30` (the CDT cars with the
  L6 steering lag of 30 ms – a structure the estimator's model does not have), `CDT_WET85` (the CDT cars with μ × 0.85 from the start – the grip is wrong before
  any corner has excited it), `CDP` (the nominal car with L8's wet patches, patch seeds 0–29 as L8 set B) and `CXD`
  (the nominal car with the L3 disturbances, seeds 7400–7429 – measurement noise, gusts, friction field; `nmpc` there
  reproduces phase C's records, expected identical in every field that does not measure time; the nominal car needs no
  adaptation, so CXD is the reproducibility control, not the noise test – that is CDT_NOISE). CXL8 (new tracks) is
  not run: the plan library exists for Spielberg only.
* Start, laps and time limit as phases A–C (flying start on the 5 % plan, one lap, 121 s).

## 3. Controllers

Shared: N = 30 × 50 ms, IPOPT, the L2 traction control (0.99), no retries; `nmpc`'s objective (c 2: w_n 2, w_ξ 20,
w_v 0.4; margin 0.6) and η = 0.95.

| controller | model in the NMPC | plan | role |
|---|---|---|---|
| `nmpc` | nominal | 5 % plan | baseline (phases A–C, unchanged) |
| `nmpc_param` | nominal, through the parameterised NLP | 5 % plan, through the plan coordinate | numerical-path control (as phase C's amendment C1) |
| `adaptive` | RLS estimate (3.2) | from the friction estimate (3.3) | **primary** |
| `adaptive_bank` | the model bank's selection (3.5) | from its smoothed friction | secondary |
| `adaptive_oracle` | the true car and friction factor | from the true friction | secondary, **oracle** (the architecture's ceiling) |
| `tube` | nominal, tube-tightened (phase C, c_R 1) | 5 % plan | secondary (robust against the L2 box) |
| `identified` | identified offline from an exploration lap | re-planned with the identified car | secondary (L8's pipeline; one more lap of information) |

### 3.1 Data and the estimator's model

At every simulator step k (10 ms) the controller sees the state x_k (CG position, heading, v_x, v_y, r, δ) and knows the
input u_k the plant applied: its own command after its traction control and after the simulator's input limits
(steering-rate clip, the angle stop, the power limit P_max / v_x, F_brake ≤ 0 – limits that are not uncertain; the
controller recomputes them). The data pair at step k + 1 is (x_k, u_k, y_{k+1}), y = (v_x, v_y, r) of x_{k+1}.

The estimator's prediction ŷ_{k+1}(θ) is one RK4 step of 10 ms of the dynamic single-track model with the vehicle
`identify.vehicle_from(θ)` – θ the seven factors of L4 (μ, m, ClA, CdA, I_z, B multiplicative, aero_front additive)
– **with the simulator's tyre saturation** (forces outside an axle's ellipse scaled onto it): the simulator's own step,
written with CasADi (`dynamics` with `lib=ca`, the saturation by `if_else`). Unlike L4's least squares no sample is
excluded for saturation (the model contains it; a real controller does not know the plant's usage) – only samples
below 5 m/s, as L4. Residual scaling Λ = diag(0.1², 0.05², 0.01²) (the L3 measurement noise of v_x, v_y, r).

### 3.2 `adaptive`: recursive least squares with forgetting (the primary estimator)

The recursive Gauss–Newton (prediction-error) form of RLS (D.1, D.2), which is RLS for a model linear in θ: with
ε = Λ^{−1/2}(y_{k+1} − ŷ_{k+1}(θ̂)) and ψ = Λ^{−1/2} ∂ŷ/∂θ (3 × 7, CasADi) at θ̂,

    L = P ψᵀ (I₃ + ψ P ψᵀ)⁻¹,   θ̂ ← Π[LO, HI](θ̂ + L ε),   P ← D (I − L ψ) P D,   D = diag(λ_i^{−1/2}),

then P bounded by the prior: P ← P₀^{1/2} min(P₀^{−1/2} P P₀^{−1/2}, I) P₀^{1/2} (eigenvalues clipped at 1 – against
the wind-up of D.2). **Vector-type forgetting** (D.2 eq. 12, written symmetrically so that equal factors give the
classical P/λ): λ_μ < 1 for the friction factor, λ = 1 for the six car constants (mass, aero, inertia, tyre stiffness,
balance – the drift model is that only the grip changes). Start θ̂₀ = nominal, P₀ = diag(((HI − LO)/4)²) (L6's EKF
prior). One update per simulator step. **Tuning parameter:** λ_μ ∈ {0.99, 0.995, 0.999} (memory ~1, 2, 10 s at 100 Hz).

### 3.3 The adaptive NMPC and the plan library

* **Parameterised NLP** – a new option of `FrenetMPC` (`param_model=True`, default off: the existing formulation is
  unchanged, as phase C's `param_tightening`): the seven factors θ are parameters of each solve; the model, the friction
  ellipses, the longitudinal force limits and the slip-angle limits (α_peak = tan(π/2C)/B, moved from the bounds into
  the constraint function) use them.
* **Plan library** – L1's dynamic minimum-time plans of the nominal car at the friction margins η_p ∈ {1.00, 0.97,
  0.95, 0.93} (L1 cache) and 0.88 (L2's 12 % plan), and new plans at η_p ∈ {0.91, 0.85, 0.82, 0.79, 0.76, 0.73, 0.70,
  0.67} made by the L1 pipeline's continuation in η from the nearest plan (`scripts/cx_d_plans.py`, outputs in
  `results/CX/D/plans/`, git-ignored, SHA-256 in the run's input list). A plan that fails is dropped from the grid,
  reported before any CDV lap. The plan of a friction margin η_p is the plan of a car whose grip is η_p times the
  nominal: a library over **friction only**, as D.9 / D.10's velocity-profile libraries (ours: full dynamic plans, line
  and speed); the other parameters are left to the NMPC's model (L4's full re-plan with every identified parameter is
  `identified`'s).
* **Plan coordinate** – a second NLP parameter j: the reference (n, ξ, v_x) at centre-line s is the linear interpolation
  between the library plans ⌊j⌋ and ⌈j⌉ (a 2-D linear interpolant on a common s grid; integer j gives that plan
  exactly) – D.9 / D.10 interpolate their profiles the same way. Before every solve j is set from η_p = 0.95 μ̂, clipped
  to [0.67, 1.00], by linear interpolation in the η_p grid.
* **Per solve** (every 50 ms): θ̂ and j from the estimator's latest estimate; the traction control uses
  `vehicle_from(θ̂)`. Start: θ̂ nominal, j at 0.95 (`nmpc`'s start). *Clarification of revision 0 (review n1):* the
  estimation error therefore also enters the low-level traction control, while `adaptive_oracle`'s uses the true car –
  one source of the gap between the two.
* **`nmpc_param`** (numerical-path control, decided now after phase C's amendment C1): `adaptive`'s code path with the
  RLS running (at `adaptive`'s selected λ_μ) but θ ≡ nominal and j ≡ 0.95 handed to the NLP and the traction control –
  `nmpc`'s problem written differently. Its discordant cars against `nmpc` are the noise floor of H-D1 / H-D2.

### 3.4 MHE – removed (*clarification of revision 0, review m3*)

The draft's `adaptive_mhe` (a moving-horizon estimate of the parameters only, the states being exact) is removed: with
exact states it is a windowed weighted least squares with an arrival cost, which carries almost the same information as
the RLS with forgetting, and the condition where an MHE would differ – noise, with the states as decision variables
(D.5–D.7) – was not to be run with it. It moves to the frontier list (CONTROL_METHOD_MATRIX.md) as a full MHE (states and
parameters) for the noisy conditions. No module `estimation/mhe.py` in this phase.

### 3.5 `adaptive_bank`: model bank with look-back selection (secondary, after D.9)

N_b = 20 000 parameter vectors drawn uniformly in [LO, HI] (seed 336) plus the nominal car. At every simulator step
every model predicts y_{k+1} (numpy, batched, the step of section 3.1); each model's cost is the sum of
‖Λ^{−1/2}(y − ŷ)‖² over the last W seconds; the model with the smallest cost is the NMPC's model at the next solve
(hard switching, as D.9; the nominal car until the window is full). The friction for the plan coordinate is smoothed,
μ_s ← γ μ_sel + (1 − γ) μ_s per step with γ = 0.05 (D.9 smooths the friction but does not report γ; ours stated). D.9
scores the plain Euclidean state error; ours is Λ-weighted, as the other estimators (stated). **Tuning parameter:**
W ∈ {0.1, 0.2, 0.5} s (D.9's 0.2 s in the middle).

### 3.6 `adaptive_oracle`

The architecture of section 3.3 handed the true car (`vehicle_from` of the case's factors) with the friction factor
times f(s) at the car's position at every solve, for the model, the plan coordinate and the traction control. Not
tuned. What the adaptive NMPC can do at best with this plan library; the gap `adaptive_oracle` – `adaptive` is the cost
of estimating.

### 3.7 `tube` and `identified`

* **`tube`** – phase C's `tube` with its selected c_R 1, unchanged (`cx_c.build`); its tightening is sized for the L2
  box (μ ≥ 0.92), which CDD's drift leaves.
* **`identified`** – L8's pipeline (`l8.drive`'s steps): the exploration lap (the L2 NMPC on the 12 % plan, η 0.88) under
  the car's **initial** condition (constant; on CDD before the drift – the drift is what changed after the
  identification), L4's least squares on its data, `replan_robust` with the identified car at 0.95, then the NMPC with
  the identified model – with this phase's objective (c 2, margin 0.6; L8 used the defaults c 1, margin 0.3) and the
  traction control on the identified model. The exploration lap is information, not a scored lap. "Identification
  failed" / "plan failed" as L8 (not completed, no retry). On CDT and CDD the exploration lap and identification of a
  car are the same; they are computed once per car and reused (deterministic).

### 3.8 What is and is not claimed (deviations from the literature, stated)

* **Certainty equivalence:** the NMPC uses the point estimate as if it were true – no robustness against the estimation
  error (unlike D.12 / D.13's robust adaptive MPC); a tube around the adaptive model is a follow-up candidate.
* **Excitation is not managed:** friction is observable only where the tyres work (D.6, D.2); no estimator here excites
  the car on purpose (no dual control). CDT_WET85 shows the consequence where the grip is wrong from the start.
* **Ideal measurement and structure in the primary sets:** exact states, the simulator's own step as the estimator's
  model – the result is an upper bound for these estimators; CDT_LAG30 (unmodelled lag) and CDT_NOISE (measurement
  noise) are the secondary checks; under noise the RLS on measured states is biased (errors in variables, as L6 found for L4's least
  squares) – reported, not corrected.
* **The plan adapts to friction only** (a library over η_p with linear blending – a blend of two minimum-time plans is
  not itself a minimum-time plan); the forgetting assumes that only the grip changes.
* No convergence or stability guarantee is claimed for any estimator–NMPC combination.

## 4. Tuning (CDV only, one lap per case and setting)

* `adaptive`: λ_μ ∈ {0.99, 0.995, 0.999}; `adaptive_bank`: W ∈ {0.1, 0.2, 0.5} s – 6 settings × 20 cases = 120 laps
  (*clarification of revision 0, review m3:* without the MHE). `nmpc`, `nmpc_param`, `adaptive_oracle`, `tube`,
  `identified` are not tuned.
* Rule (phases A–C): the most completed CDV laps (constant and drift together); ties by the mean lap time on the cases
  every tied setting completes (skipped if none); then the middle setting of the grid; different lap counts stop the
  selection.
* The choices and all CDV records go to the reviewer before any CDT / CDD lap.

## 5. Always reported

As phases A–C: completed laps (exact 95 % intervals), lap time, distance to failure, stop reason; offsets from the plan
line; constraint use (minimum edge margin, slacks); tyre usage; control effort; NMPC solve time (mean, P95, P99, max),
non-converged solves and fallbacks. **For the adaptive controllers**, per lap: the estimation error of each factor at
the lap end (CDT) and of the friction factor μ̂ − μ·f(s) along the lap (P50 / P95 of the absolute error; on CDD before,
during and after the drift); on CDD the delay from the drift onset until |μ̂ − μ·f| < 0.02 (or never); the plan
coordinate along the lap and the **too-fast share** (stages where η_p > 0.95 μ·f + 0.01: the plan asked for more grip
than the car had); the scaled one-step prediction RMS; the share of RLS updates with a clipped covariance; the failure
position relative to the drift onset; the estimator time per update (RLS, bank) next to the NMPC time. On CDT_NOISE the bias of μ̂ (mean of μ̂ − μ over the
last half of the lap) and of every factor at the lap end.

## 6. Statistics

Exact McNemar, exact intervals, exact signed-rank (mid-ranks); Holm over H-D1 and H-D2. Secondary, uncorrected (number
stated): on CDT and on CDD each, McNemar of `nmpc_param` – `nmpc` (the noise floor), `adaptive` – `tube`, `adaptive` –
`identified`, `adaptive` – `adaptive_oracle`, `adaptive_bank` – `adaptive`, `tube` – `nmpc`, `identified` – `nmpc` (14),
and the signed-rank of the lap time on the laps both complete for these pairs and for `adaptive` – `nmpc` (16). The
secondary conditions: completed laps per controller and `adaptive` – `nmpc` / `adaptive` – `tube` McNemar per condition
(10: five conditions), uncorrected. (*Clarification of revision 0, review m3 / m1:* without the MHE, with CDT_NOISE.)

## 7. Second environment

As before (deferred by the user): H-D1 and H-D2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). No code before its APPROVE.
2. `estimation/rls.py`, `estimation/bank.py` (the estimators and the step model of section 3.1),
   `control/adaptive_mpc.py` (the controller), `FrenetMPC(param_model=True)` with the plan coordinate (`control/nmpc.py`,
   defaults unchanged), `experiments/cx_d.py` (sets, drift field, controllers, lap), `scripts/cx_d_plans.py`,
   `scripts/run_cx_d.py`, `scripts/report_cx_d.py`, `scripts/cx_d_dev.py`, unit tests; the development checks of
   section 9; review (packet 2) before any CDV lap.
3. CDV tuning: 6 settings × 20 cases = 120 laps; results to the reviewer.
4. CDT + NOM and CDD from a pinned worktree: 7 controllers × 101 = 707 laps (`identified`'s re-plans at most 6 in
   parallel – ~7.4 GB each, 63.5 GB here); report, review.
5. Secondary conditions: 4 controllers × 210 laps (CDT_NOISE 50, CDT_LAG30 50, CDT_WET85 50, CDP 30, CXD 30) = 840
   laps; review, final review, merge.

## 9. Development checks (planned; none has run)

On NOM and DEV only (records in `results/CX/D/dev/` with commit and dirty flag):

1. **Step model:** the CasADi step of section 3.1 against the simulator's step (`DynamicSim.step`, and `StochasticSim`
   with a friction factor) at states along `nmpc`'s NOM lap and at saturated states – (v_x, v_y, r) equal to 10⁻¹²;
   ∂ŷ/∂θ against central differences (10⁻⁶ relative, away from the saturation switch).
2. **Parameterised NLP:** one cold solve with θ nominal and j at 0.95 equals `nmpc`'s solve from four states (10⁻⁸ in
   the input and the prediction); with θ of a vertex car equals `FrenetMPC` built with that car; with j at a library
   point equals `FrenetMPC` built on that plan.
3. **Plan library:** the new plans (status, lap time of the plan, solve time); the 2-D interpolant reproduces each grid
   plan at integer j.
4. **Estimator replays:** the RLS and the bank fed offline with the data of recorded laps (`nmpc` on NOM and on
   the DEV cars, `adaptive_oracle` on the DEV drifts): convergence of every factor along the lap (distance until within
   2 %), friction tracking after the onset, per setting – development data, no selection.
5. **Timing:** per update of each estimator; the NMPC solve with `param_model`.
6. **Development laps:** NOM with the 6 settings, `adaptive_oracle` and `nmpc_param`; DEV constant and drift (10 cases)
   with `nmpc`, `adaptive` (λ_μ 0.995) and `adaptive_oracle`. **Gate:** `adaptive_oracle` completes NOM and at least 4
   of the 5 DEV drift cases (the architecture – model update and plan library – works when the estimate is right), and
   `adaptive` completes NOM with at least one λ_μ; otherwise the design is revised by an amendment before any CDV lap.
   *Clarification of revision 0 (review n2):* the gate's report lists `nmpc`'s results on the DEV drifts next to it, as
   the background that shows whether the gate is easy to pass.
7. *Clarification of revision 0 (review m2):* **the drifts drawn for CDD** (and CDV), reported after the implementation
   and before any CDV lap – they depend on no controller: the distributions of s_on, ℓ and f_end, the number of cars with
   μ·f_end < 0.70, and the shares of drifts that end (s_on + ℓ) before the corners at ~2.7 km and ~3.83 km.

### 9. Results of the development checks (2026-10-22; packet 2)

NOM and the development cars only; records in `results/CX/D/dev/` (git-ignored), each with its commit (fb8305b;
`unchanged.json` ea4ac82) and `dirty = false`.

0. **The existing NLPs are unchanged** (`unchanged.json`): `FrenetMPC` of this branch and of 55c5925 (before
   `param_model`), for the default, `param_tightening`, `steer_lag`, `mode="filter"`, `mode="mpcc"` and
   `soft_limits=5` – objective and constraint function equal to the bit at 5 random points each, all bounds identical.
1. **Step model** (`step.json`, `tests/test_adaptive.py`): 3 cars (two DEV cars and the nominal) × friction factor 1.0
   and 0.8 × 500 random states and inputs (444–463 of each 500 saturated): the CasADi step differs from the
   simulator's by at most 4.4 · 10⁻¹⁶ in (v_x, v_y, r), the numpy step not at all; ∂ŷ/∂θ against central differences
   1.4–5.7 · 10⁻⁸ relative. The saturation is written as a division by sqrt(max(u², 1)) – the simulator's rule; the
   first version (through `ellipse_usage`) had an infinite derivative where an axle carries no force (a straight with
   no front force), found by the unit test.
2. **Parameterised NLP** (`nlp.json`): the 13-plan library, built in 0.38 s; at 4 states of `nmpc`'s NOM lap one cold
   solve against `FrenetMPC` with the nominal car and the 5 % plan, with a vertex car, with the 0.88 plan and with the
   0.67 plan: inputs at most 3.7 · 10⁻¹⁰ N apart (relative ≤ 9.3 · 10⁻¹⁴), predictions ≤ 2.3 · 10⁻¹³, the same IPOPT
   status and iteration count every time.
3. **Plan library** (`plans.json`, `plans/plans_cxd_log.json`): all eight new plans solved, each by one direct solve
   from the next plan above (1.1–1.5 min each); none dropped. Lap times of the 13 plans:

   | η_p | 0.67 | 0.70 | 0.73 | 0.76 | 0.79 | 0.82 | 0.85 | 0.88 | 0.91 | 0.93 | 0.95 | 0.97 | 1.00 |
   |---|---|---|---|---|---|---|---|---|---|---|---|---|---|
   | lap s | 91.33 | 88.96 | 86.80 | 84.84 | 83.05 | 81.39 | 79.87 | 78.45 | 77.15 | 76.35 | 75.60 | 74.89 | 73.91 |

   All on the s grid of the 0.95 plan; the plan coordinate's interpolant reproduces every plan's v_x exactly at
   integer j. (The plan file records commit fb8305b: the script started at 6f54f30 and records HEAD when it writes;
   the planning code is the same – its fingerprint is checked at every load.)
4. **Estimator replays** (`replay.json`): the RLS (3 λ_μ) and the bank (3 W) fed offline with 11 recorded laps (`nmpc`
   on NOM and on DEV_C – which leaves at 3.83 km on cars 2–4 –, `adaptive_oracle` on DEV_D). Development data, no
   selection.
   * **RLS, constant cars:** on NOM exact from the start; on 4 of the 5 DEV cars every factor within 2 % from 0–500 m
     of the lap (the first corners) for every λ_μ – except car 4's ClA with λ_μ 0.999 (from 1.36 km). **DEV car 3** (μ 0.929, ClA 1.065): μ and ClA move
     together along a ridge (μ·F_z nearly preserved: μ̂ high, ClÂ low) – μ within 2 % only from 2.2 km (λ_μ 0.995),
     0.44 km (0.999), 3.7 km (0.99); ClA never with 0.99. Forgetting only on μ lets μ follow recent data along that
     ridge.
   * **RLS, drifts** (λ_μ 0.995 / 0.999 / 0.99): settled (|μ̂ − μf| < 0.02 at every later step) 4.1–54.7 s after the
     onset / 14.6 s – never (2 of 5) / 4.0 s – never (1 of 5); |μ̂ − μf| after the onset P95 0.015–0.083 / 0.023–0.092 /
     0.012–0.088.
   * **Model bank (D.9): it does not identify these cars.** |μ̂ − μ| P50 0.05–0.16, P95 0.25–0.50 (drifts: up to 0.59);
     almost no factor ever within 2 %; 330–1 150 model switches per lap (fewer with longer W); exact on NOM, where the
     nominal car is a member. Two mechanisms: (a) directions without excitation – μ on the straights (D.6) – leave many
     members with nearly the same window cost, and the argmin switches among them; (b) 20 001 random vectors in the
     7-dimensional box of L4's bounds are about a quarter of each range apart (20 000^(1/7) ≈ 4 per dimension), so no
     member is within 2 % of a car. This is D.9's method with this phase's seven parameters and bounds (D.9: 8 tyre and
     drag parameters of a scale car, bounds set relative to the truth). **Proposed: keep `adaptive_bank` as registered**
     (a secondary controller; its result is informative as it is); the reviewer may decide otherwise.
5. **Timing** (under 20 parallel jobs): RLS 266–283 µs per update (100 per second); bank 13.6–14.0 ms per update (~110 s
   of a lap's wall time); NMPC solve with `param_model` 184–195 ms mean against `nmpc`'s 173–195 ms in the same runs.
6. **Development laps** (`laps.jsonl`, `gate.json`; 39 laps, no error):

   | set | `nmpc` | `adaptive` (λ_μ 0.995) | `adaptive_oracle` |
   |---|---|---|---|
   | NOM | 76.400 s | 76.399 s (every λ_μ) | 76.399 s |
   | DEV_C (constant) | 2 of 5 (cars 1, 5; cars 2–4 off at 3 828–3 829 m, as phase C's CXDEV) | 5 of 5 | 5 of 5 |
   | DEV_D (drift) | 0 of 5 (off at 1 401–2 740 m) | 5 of 5 | 5 of 5 |

   On NOM also `adaptive_bank` (every W) and `nmpc_param` 76.399 s. Lap time `adaptive` − `adaptive_oracle`: DEV_C
   −0.06 … +0.88 s, DEV_D −0.60 … +0.16 s. Too-fast share of `adaptive`: up to 0.27 on four DEV_C cars, 1.0 on car 3
   (the ridge of item 4), 0.07–0.79 on DEV_D. **Gate: passed** – `adaptive_oracle` completes NOM and 5 of the 5 DEV
   drifts, `adaptive` completes NOM with every λ_μ. Background (review n2): `nmpc` completes 0 of the 5 DEV drifts,
   `adaptive` 5 of 5. Runner smoke (`results/CX/D/smoke/`): `nmpc` 76.400 s, `tube` 78.279 s – phase C's laps.
7. **The drifts drawn** (`drifts.json`, review m2; independent of any controller):

   | set | s_on min / median / max m | ℓ min / median / max m | f_end min / median / max | μ·f_end min / median / max | μ·f_end < 0.70 | drift ends before 2.7 km / 3.83 km |
   |---|---|---|---|---|---|---|
   | CDD (50) | 624 / 1 768 / 2 524 | 183 / 1 152 / 1 486 | 0.751 / 0.843 / 0.900 | 0.718 / 0.805 / 0.906 | 0 | 46 % / 100 % |
   | CDV_D (10) | 1 018 / 1 512 / 2 448 | 342 / 946 / 1 493 | 0.758 / 0.843 / 0.883 | 0.730 / 0.806 / 0.893 | 0 | 60 % / 90 % |

   Every CDD drift has begun before 2.7 km and ended before 3.83 km, so every CDD lap meets the corner at ~3.83 km –
   where L2's failures concentrate – at the final, lowest grip.

**Tuning tie (code, for the reviewer):** section 4 breaks a remaining tie by "the middle setting of the grid"; when the
lowest and the highest setting are tied without the middle one, both are equally far from it – `cx_d.select` then takes
the smaller value (λ_μ 0.99, W 0.1). Stated here before any CDV lap.

**The drift's start (found after the CDT / CDD report; packet 4).** The lap's start state projects to s = L − 1.4 · 10⁻¹¹,
just before the line, so the drift field (a function of s mod L) gave f_end for the first 10 ms simulator step of every
drift lap (DEV_D above, CDV_D, CDD) – the same for every controller – instead of f = 1 up to s_on as section 2 defines
it; and the lap's onset diagnostics (`cx_d.adaptation_metrics`) detected the onset at the lap's first solve. The
diagnostics now use the lap position (`cx_d.lap_position`, c8035cb; the CDT / CDD report's drift-delay columns and the
CDV report's are therefore not used – `dev/cdd_rerun.json` has the corrected ones). Checked at c8035cb
(`dev/cdd_rerun.json`; the decision rule set by the review before the check): `adaptive` on CDD again with the field as
run – all 50 laps identical to `test.jsonl`; with the start fixed (`cx_d_dev._StartFixDrift`) – `nmpc` 0 → 0, `adaptive`
34 → 34, `adaptive_oracle` 44 → 44, `tube` 2 → 2 completed, no car's outcome changed (completed laps up to 0.79 s apart).
The replays of item 4 were not affected (they read the onset from the recorded states after the first one); redone at
c8035cb they are identical in every field (`dev/replay.json`; the first record kept as
`dev/replay_v1_onset_at_lap_start.json`).
