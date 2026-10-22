# CX-C protocol – robust tube MPC (control expansion, phase C)

Status: **binding, revision 0 (2026-10-22; 9006b4a, APPROVE)** with the clarifications of its review (m1–m3, n1–n3;
marked *"clarification of revision 0"*), written down before any development check of section 9 – written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §C) and **before any tube-MPC code**: no module, test,
runner or lap of phase C exists (section 9). Branch `cx/C-tube` (from `main` 2858db4, the merge of phase B).
Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

## 0. What L0–L9 and phases A–B left

The NMPC of L2–L9 is robustified only empirically: a uniform friction reserve in the plan (L2: 7 % and 12 %) and
stochastic back-offs calibrated from closed-loop errors (L3). L2 found that the L2 cars fail mostly because *the car
does not turn as the model predicts* (softer tyres, aero balance, less downforce) – "a friction reserve does not cover an
error in how the car turns" (docs/L2.md §4–5); 22 of the 24 failures with the 12 % reserve happen without either axle
reaching its friction limit, at the same three corners. Phases A and B kept the nominal problem; no controller uses a
**model of the uncertainty** (the parameter ranges) to decide how far from the track edges and the friction limits
its nominal prediction must stay. Tube MPC does exactly that: an error tube around the nominal prediction, propagated
from a disturbance set under an ancillary feedback, shrinks the constraints the nominal problem sees.

## 1. Question and hypotheses

*Does tightening `nmpc`'s constraints by a tube – the reachable set of the prediction error under an ancillary
feedback, driven by the model mismatch that the L2 parameter ranges allow – complete more laps than `nmpc` on new L2
cars, and at what lap time?*

* **H-C1 (primary):** on the 50 test cars CCT (section 2), `tube` and `nmpc` differ in the number of completed laps.
  Exact McNemar, two-sided; holds if significant at its Holm level; direction reported.
* **H-C2 (primary):** on the test laps both complete, `tube` and `nmpc` differ in lap time. Exact signed-rank (mid-ranks,
  exact distribution), two-sided; conditional on both completing – the discordant cars listed with lap time or
  failure position and stop reason.

Holm over H-C1 and H-C2 (family-wise α = 0.05), the stricter decision (each at 0.025) alongside. No direction is
predicted for H-C1; for H-C2 a tightened controller is expected to be slower where both complete, and the size of
that cost is the quantity of interest. Power: as phase B §1 (at least 7 discordant cars, all one way, for Holm's first
level).

Scope: the nominal model, Spielberg, η = 0.95, `nmpc`'s objective and margin (phase A's CXV choice c 2, margin 0.6); the
tube of section 3 (a linearisation-based approximation – no robustness guarantee is claimed, section 3.5); the L2
parameter ranges as the uncertainty. **Information budget:** `tube` knows the *ranges* of the L2 parameters (a prior
the protocol hands it), not the parameters of the car; `nmpc` does not use them; `nmpc_reserve` uses a fixed reserve.
No oracle. Offline, as phases A and B (IPOPT without a time budget). *Clarification of revision 0 (review n3):* the
tuning asymmetry runs the other way this time – `tube` has 3 settings on CXV, `nmpc`'s objective was chosen from 6
(phase A).

## 2. Sets

* **Tuning set CXV** (phase A's, reused as in phase B): `sample_plants(nominal, 10, seed 301)[1:]` – `nmpc`'s objective
  was chosen there; `tube` tunes only its own parameters there.
* **Test set CCT** (new): `sample_plants(nominal, 50, seed 322)[1:]`. Seed 322 is used by no L-layer (0, 1, 7, 8, 9,
  10, 99), by phase A (301, 302) or phase B (312) (checked in src, scripts and docs at 2858db4).
* **Development cars CXDEV:** `sample_plants(nominal, 5, seed 399)[1:]` – for the development checks of section 9 only;
  never tuned on, never tested on (the roadmap allows development on sets no protocol tests on).
* **NOM:** the nominal car, reported separately.
* **Secondary conditions** (after the primary report, as phases A / B): `CCT_LAG30` (the CCT cars with the L6 30 ms
  steering lag), `CXD` (nominal car, L3 disturbances, seeds 7400–7429), `CXL8` (nominal car, the four L8 tracks).
  *Clarification of revision 0 (review m3):* the secondary conditions run the five controllers of section 3 (`tracker`,
  `nmpc`, `nmpc_reserve`, `tube`, `tube_track`, with their CCT settings); `nmpc` there serves as the reproducibility
  check against phase B's records (expected identical in every field that does not measure time; `mpcc` is not run in
  phase C).
* Start, laps, time limit as phases A / B.

## 3. Controllers

Shared: the nominal model, η = 0.95, the 5 % plan, N = 30 × 50 ms, IPOPT, the L2 traction control (0.99), no retries.

* **`tracker`** – plan + L1 tracker (baseline, not tuned).
* **`nmpc`** – `FrenetMPC` tracking the 5 % plan, c 2, margin 0.6 (phases A / B, unchanged).
* **`nmpc_reserve`** (secondary baseline) – a variant of L2's "NMPC on plan 12 %" (review n2: L2 used the weights c 1,
  margin 0.3): `nmpc`'s objective (c 2, margin 0.6) tracking the η = 0.88 plan (`results/L2/plan_088.pkl`) with η = 0.88
  in the MPC – the uniform, uncertainty-blind reserve.
* **`tube`** (primary) – `nmpc` with the per-solve tightening of sections 3.1–3.4 (`FrenetMPC(param_tightening=True)`,
  as L6's cautious NMPC).
* **`tube_track`** (secondary, ablation) – `tube` with the track back-offs only (η not tightened): is it the lateral
  tube that matters (L2's finding)?

### 3.1 Error model and ancillary feedback

Error e = x − z of the Frenet state (s, n, ξ, vx, vy, r, δ) about the nominal prediction z. Before each solve, along the
previous solution shifted by one stage (the first solve: the initial guess), the discrete model (one RK4 step of 50 ms,
the NMPC's `step`) is linearised: A_k = ∂f/∂x, B_k = ∂f/∂u at (z_k, v_k), k = 0 … N − 1 (CasADi Jacobians of the NMPC's
own step function). The ancillary feedback is a time-varying LQR along the prediction: backward Riccati recursion with
`nmpc`'s stage weights, Q = h · diag(0, w_n, w_ξ, w_v, 0, 0, 0) and R = c_R · diag(h w_δ̇, w_dF, w_dF) (c = 2: w_n 2,
w_ξ 20, w_v 0.4, w_δ̇ 1, w_dF 0.5; forces in 10⁴ N, as the NLP), terminal P_N = Q; gains K_k. Closed-loop error dynamics
e_{k+1} = (A_k + B_k K_k) e_k + w_k. (Linear tube MPC with a fixed feedback, C.1 / C.2; along a nonlinear prediction by
linearisation, as the "approximate tube of reachable sets" of C.6.) *Clarification of revision 0 (review m1):* K is
**not applied** in closed loop – the applied input is the NMPC's u*_{0|t} and the nominal restarts from the measured
state at every solve (e_0 = 0), as in C.2 / C.4's constraint tightening; K only shapes the error reachable sets, i.e.
the tube is the error bound the system *would* have if K acted between solves.

### 3.2 Disturbance set W_k (the L2 parameter ranges)

w_k is the one-step model mismatch: W_k = the box with half-widths w̄_k,i = max over the 2⁷ = 128 vertices θ of the L2
parameter box (μ × [0.92, 1.03], m × [1.00, 1.05], ClA, CdA × [0.90, 1.10], Iz, B × [0.85, 1.15], aero_front + [−0.03, 0.03])
of |f_θ(z_k, v_k) − f(z_k, v_k)|_i (one RK4 step, the nominal input), evaluated per solve along the previous prediction
– state- and input-dependent, as C.4's w̃(z, v) and C.8's vertex bound. (The vertex maximum is the box maximum only
where the mismatch is monotone in each parameter; checked by sampling in section 9.)

### 3.3 Tube and tightening

Error reachable sets from e_0 = 0 (the nominal starts at the measured state at every solve, as C.4): for an output
y = Cₒ e its bound at stage k is b_k(Cₒ) = Σ_{j<k} Σ_i |(Cₒ Φ_{k,j+1})_i| w̄_j,i, Φ the closed-loop transition matrices
(the exact support function of the box-driven reachable set of the linear error system). Tightened quantities:

* **Track limits** – the lateral positions of the front and rear axle points (n + arm sin ξ, linearised: Cₒ = e_n + arm
  cos ξ_k e_ξ): back-off b_k^track = max of the two, added to the NMPC's margin (`backoff`, per stage).
* **Friction ellipses** – each axle's usage² (linearised in (e, u − v = K e) at (z_k, v_k)): back-off Δ_k = max over the
  two axles; the per-stage friction factor becomes η_k = sqrt(max(η² − Δ_k, η_min²)), η_min = 0.70 (`eta`, per stage).
  *Clarification of revision 0 (review m2):* under `param_tightening`, η_k also enters the hard limits on each axle's
  longitudinal force (|F_x / (μ_x F_z)| ≤ η_k, `nmpc.py`), so the tube tightens those too, and η_min = 0.70 bounds them
  as well – a property of the existing interface, kept and read as part of the tube; `tube_track` leaves η at 0.95
  everywhere, so the ablation separates exactly the track part from the friction-and-force part.
* **Caps:** b_k^track ≤ b_max = half of the narrowest track width minus body (otherwise no nominal trajectory remains),
  η_k ≥ η_min. A capped stage is not robust in the sense of the tube; the share of capped stages is reported
  (robust-feasibility diagnostic, section 5).

### 3.4 Tuning parameters and the per-solve computation

`tube` and `tube_track` have one tuning parameter: the LQR input weight scale c_R ∈ {0.1, 1, 10} (c_Q = 1): a larger c_R
gives a weaker feedback and a wider tube. The tube is computed from the previous solution before each IPOPT solve
(30 Jacobians, 128 × 30 model steps, one Riccati recursion) – reported as part of the step time.

### 3.5 What is and is not claimed (deviations from the literature, stated)

The tube is an approximation: (a) it uses the linearisation along the previous prediction, not an incremental
Lyapunov function or a contraction metric (C.4, C.8 need SDP / SOS solvers, which are not available here – literature
"Tools note"); (b) the disturbance set is the vertex maximum of the one-step mismatch, not a proven bound, and it is
evaluated at the nominal (z_k, v_k) – the mismatch at the perturbed points (z_k + e, v_k + K e) that the error actually
visits is not covered (review n1); (c) the
caps can remove robustness; (d) no terminal set. The scheme is therefore a **constraint-tightened NMPC with a
model-based tube**, in the spirit of C.2 / C.6, with no claim of recursive feasibility or guaranteed constraint
satisfaction. Section 9 measures how well the tube covers the actual prediction errors of cars from the L2 box.

## 4. Tuning (CXV only, one lap per car and setting)

* `tube`: c_R ∈ {0.1, 1, 10} (3 settings); `tube_track`: the same 3 (6 settings, 60 laps).
* Rule (phases A / B): the most completed CXV laps; ties by the mean lap time on the cars every tied setting completes
  (skipped if none); then the setting closest to c_R = 1 (the smaller |log c_R|), then the smaller c_R (the stronger
  feedback, the narrower tube). Different lap counts stop the selection. `nmpc`, `nmpc_reserve`, `tracker` are not tuned.
* The choices and all CXV records go to the reviewer before any CCT lap.

## 5. Always reported

As phases A / B: completed laps (exact 95 % intervals), lap time, distance to failure, stop reason; contouring / plan-line
offsets; constraint use (minimum edge margin, slacks); tyre usage; control effort; solve time (IPOPT) and tube time
(mean, P95, P99, max), offline share above 50 ms; non-converged solves and fallbacks; the drive-and-brake overlap.
**For the tube controllers**, per lap: the track back-off and the friction factor η_k (P5 / P50 / P95 over stages and
solves, and at stage N), the share of capped stages, and the **coverage**: for every solve, the realised error between
the measured state 1 … 10 stages later and the solve's prediction, against that stage's back-off (share of stages
where the realised lateral error exceeds b_k^track). *Clarification of revision 0 (review m1):* the realised error is
that of the re-solving NMPC with the nominal input held open loop over those stages, the bound assumes K acting – two
different systems; the coverage is a diagnostic of the tube's scale, not a test of a guarantee.

## 6. Statistics

Exact McNemar, exact intervals, exact signed-rank; Holm over the two primary tests. Secondary, uncorrected (number
stated): McNemar and signed-rank of `tube` – `nmpc_reserve`, `tube_track` – `nmpc`, `tube_track` – `tube`,
`nmpc_reserve` – `nmpc` (8).

## 7. Second environment

As before (deferred by the user): H-C1 and H-C2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). No code before its APPROVE.
2. `control/tube.py` (the tube computation; the controller wraps `FrenetMPC(param_tightening=True)` as
   `CautiousMPCController` does), unit tests, the development checks of section 9, the runner and report
   (`experiments/cx_c.py`, `scripts/run_cx_c.py`, `scripts/report_cx_c.py`); review (packet 2) before any CXV lap.
3. CXV tuning: 6 settings × 10 cars (60 laps, ~4–5 min each); results to the reviewer.
4. CCT + NOM from a pinned worktree: `tracker`, `nmpc`, `nmpc_reserve`, `tube`, `tube_track` – 51 × 5 = 255 laps
   (~20 CPU-h; ~1 h with 20 workers); report, review.
5. Secondary conditions, review, final review, merge.

## 9. Development checks (planned; none has run)

On NOM and the development cars CXDEV only (records in `results/CX/C/dev/` with commit and dirty flag):

1. **Linearisation:** the Jacobians against central finite differences of the NMPC's step (relative 10⁻⁶).
2. **Reachable-set bound:** for the linear error system of a recorded prediction, 10⁴ random disturbance sequences in
   the box (and its vertices) never exceed b_k, and the vertex sequences attain it (tightness).
3. **Disturbance set:** at states along `nmpc`'s nominal lap, the one-step mismatch of 1 000 random cars from the L2
   box against the vertex maximum (share exceeding it).
4. **Coverage on CXDEV:** `nmpc` and `tube` (c_R = 1) laps on the five development cars; the realised prediction errors
   against the back-offs (section 5); the track back-off and η_k along the lap.
5. **Timing** of the tube computation per solve.
6. **Development laps:** NOM with the 6 settings. **Gate:** `tube` must complete the nominal lap with at least one
   setting, and its tightening must not be capped on more than half of the stages of the lap; otherwise the tube is
   revised by an amendment before any CXV lap (the roadmap's main risk: "a disturbance set that is neither empty nor
   so large that the tightened problem is infeasible").

### 9. Results of the development checks (2026-10-22; packet 2)

Nominal car and CXDEV only; records in `results/CX/C/dev/` (git-ignored), each with its commit and `dirty = false`.

1. **Linearisation** – `tests/test_tube.py::test_jacobians_match_finite_differences`: the CasADi Jacobians of the NMPC's
   step against central differences at four stages of a prediction (A to 10⁻⁶, B to 10⁻⁵ relative); the numpy step
   used for the disturbance set equals the NMPC's step for the nominal car and for a vertex car (10⁻¹²).
2. **Reachable-set bound** – `test_reach_bounds_are_the_support_function`: 10⁴ random box-disturbance sequences never
   exceed b_k; the sign-matched vertex sequence attains b_N (10⁻¹²); `test_tv_lqr_converges_to_the_stationary_gain`.
3. **Disturbance set** (`wcheck.json`, db3b075): at 100 states of `nmpc`'s nominal lap (phase B's record), the one-step
   mismatch of 1 000 random cars from the L2 box exceeds the vertex maximum in no component (largest ratio 0.97);
   median vertex bounds per 50 ms step: vx 0.036 m/s, vy 0.062 m/s, r 0.011 rad/s, n 1.5 mm.
4. **Development cars CXDEV** (`cxdev.jsonl`, db3b075): `nmpc` completes 2 of 5 (cars 1, 5: 76.25, 75.91 s; cars 2–4 off
   at 3 828–3 829 m); `tube` (c_R 1) completes 5 of 5 (77.76–79.11 s); its η is capped on ~16 % of the stages, the track
   back-off almost never; the realised lateral error exceeds the back-off on 5.6–10.4 % of the covered stages.
   Development data – no test, no selection.
5. **Timing** (the NOM laps of 6, six in parallel): the tube computation 6.2–8.7 ms per solve (mean; P99 11–16 ms,
   max 14–19 ms), on top of IPOPT's ~136 ms mean.
6. **Development laps on NOM** (`laps.jsonl`, db3b075):

   | setting | `tube` lap | `tube_track` lap | `tube`: capped η share, track | coverage exceed share `tube` / `tube_track` |
   |---|---|---|---|---|
   | c_R 0.1 | 78.70 s | 76.75 s | 0.17, 0 | 10.5 % / 4.0 % |
   | c_R 1 | 78.28 s | 77.04 s | 0.16, 0 | 6.0 % / 3.0 % |
   | c_R 10 | 78.05 s | 77.35 s | 0.15, 0 | 4.3 % / 2.5 % |

   `nmpc` on the same car: 76.40 s. Track back-off P50 0.03–0.07 m, P95 0.17–0.99 m; `tube`'s η_k P5 0.70 (the cap),
   P50 0.89–0.92. **Gate (6): passed** – `tube` completes the nominal lap with every setting, with no more than 17 % of
   the stages capped. A stronger feedback (smaller c_R) narrows the track tube but widens the friction tube (the
   feedback's input deviations K e enter the usage), so `tube`'s laps are slower at c_R 0.1.
   Runner smoke (`results/CX/C/smoke/`): `tracker` 75.574 s, `nmpc` 76.400 s (as phase B), `nmpc_reserve` 79.048 s,
   `tube_track` 77.036 s, `tube` 78.279 s (as the development lap – deterministic).
