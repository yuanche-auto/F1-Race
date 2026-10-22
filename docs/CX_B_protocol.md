# CX-B protocol – model predictive path integral control (control expansion, phase B)

Status: **binding, revision 0 (2026-10-21; 0b94d1a, APPROVE)** with the clarifications of its review (m1, m2,
n1–n4; marked *"clarification of revision 0"* below), written down before any MPPI timing or development lap.
Written after the literature check ([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §B) and
**before any MPPI code**: at 0b94d1a no MPPI module, test, runner or lap existed (section 9). Branch `cx/B-mppi`
(from `main` b68d0a4, the merge of phase A).
Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

## 0. What L0–L9 and phase A left

Every model-based controller of the project is gradient-based: IPOPT (NMPC of L2–L9, MPCC of phase A) or
acados SQP / RTI (L9). No sampling-based MPC exists. Two of the project's known weaknesses are properties of
that solver: IPOPT needs ~130 ms per solve on this machine (phase A development laps; L9: IPOPT fails the
50 ms budget), and a hard constraint that the measured state already violates makes the NLP infeasible
(L2 §4, CX-A amendment A2). MPPI (Williams et al., literature B.1–B.3) optimises by sampling input
sequences, rolling the model out and averaging the samples with exponential weights: its cost per step is
fixed by the number of samples, it needs no derivatives, and it has no notion of infeasibility – limits are
costs or clamps. Its weaknesses are as well known (B.3, B.11): a finite sample of a 60-dimensional input
sequence, constraints only as penalties, noisy inputs.

## 1. Question and hypotheses

*Does MPPI – minimising **the same optimal control problem as the NMPC** (the NMPC's model, horizon,
tracking objective and limits, its soft limits as their exact penalties), with a fixed sample budget that
fits a 50 ms control period on one core (by the median step time, section 3.4) – complete as many laps as the NMPC, which solves that problem to
convergence with IPOPT, and at what lap time, under the model mismatch of L2?*

The comparison isolates the optimiser: same information, same objective, same model; the NMPC gets IPOPT
without a time budget (offline, as in L2–L8), MPPI gets one sampling iteration per control step at a budget
set before any tuning lap (section 3.4). A difference in laps can come from MPPI's approximate optimum (a
finite sample), from its different handling of limits (penalties on the samples instead of constraints;
no infeasible solves, no fallbacks), or from its sampling noise in the applied inputs; section 5 reports
quantities that separate these.

*Since amendment B2 (approved 2026-10-22) the primary MPPI is `mppi_lp` (section 3.5): read `mppi_lp` for `mppi` in
H-B1 and H-B2; the white-noise `mppi` is a secondary, untuned controller.* *Clarification (review n1 of packet 2):*
"fits a 50 ms control period" is meant by the median step time (section 3.4). At the K in force, K_RT = 1 024
(`mppi_lp`, section 9.5, `timing_lp.json`), the nominal lap's steps took median 25.0 ms, P95 41.0 ms, P99 48.3 ms,
with 0.8 % above 50 ms (review n1 of packet 3); for comparison, at K = 2 048 `mppi_lp`'s median was 44.6 ms, P95
61.1 ms, 27 % above 50 ms. The report lists the step-time percentiles and the share of steps above 50 ms.

* **H-B1 (primary):** on the 50 test cars CBT (section 2), `mppi` and `nmpc` differ in the number of
  completed laps. Exact McNemar, two-sided; holds if significant at its Holm level; direction reported.
* **H-B2 (primary):** on the test laps both complete, `mppi` and `nmpc` differ in lap time. Exact
  signed-rank (mid-ranks, the exact distribution), two-sided; same rule. Conditional on both completing:
  next to H-B1 the report lists, for every discordant car, each controller's lap time or failure position
  and stop reason.

Holm over H-B1 and H-B2 (family-wise α = 0.05), the stricter decision (each at 0.025) reported alongside.
No direction is predicted. **Power:** with d discordant cars the smallest two-sided McNemar p is 2 · 0.5^d;
H-B1 can hold at Holm's first level (0.025) only if at least 7 cars are discordant and all in one direction
(2 · 0.5⁷ = 0.016). Phase A had 3 discordant cars of 25; the test set is doubled for this reason – no power
calculation is possible without data, and a "not supported" may again be a statement about power.

Scope: these two controllers, the nominal model, Spielberg, η = 0.95, `nmpc`'s objective and margin as chosen
on CXV in phase A; MPPI with the budget of section 3.4 – a **CPU, numpy implementation**, whose K is set by the
speed of this implementation on this machine (review n4), so the results do not carry over to another
implementation or machine; no claim about MPPI with a GPU, with a learned model,
with another cost (e.g. a progress reward), or about sampling-based MPC in general. **Tuning freedom:** `nmpc`
was tuned on CXV over 6 objective settings (phase A); `mppi` solves the chosen objective and tunes only its
sampling parameters on the same 10 CXV cars, 9 settings (section 4) – not a symmetric budget, stated as in
phase A. *Since amendment B2 (review c3 of packet 2):* the primary `mppi_lp` is tuned on CXV over 9 + 3 = 12
settings (λ × c_σ at 2 Hz, then f_c) against `nmpc`'s 6, and the primary controller itself was changed after
the development gate on the nominal car failed for the white-noise `mppi` (section 9, B2) – a second asymmetry,
stated.

## 2. Sets

* **Tuning set CXV** (phase A's tuning set, reused): `sample_plants(nominal, 10, seed 301)[1:]` – the cars on
  which `nmpc`'s objective was chosen; MPPI's sampling parameters are tuned on the same cars. Its phase A
  results are known to the builder (disclosed; it is a tuning set).
* **Test set CBT** (new draw): `sample_plants(nominal, 50, seed 312)[1:]` – 50 cars from the L2 ranges,
  Spielberg, one lap each, no lag, no disturbances. Car seed 312 is used by no L-layer, by L10's protocol or by
  phase A (checked: L-layers use car seeds 0, 1, 7, 8, 9, 10, 99 – 10 is L7 / L9's T1new, added in the review (n1); phase A
  301, 302).
* **NOM:** the nominal car – development (section 9) and reported separately; not part of a test.
* **Secondary conditions** (after the primary report; as phase A, so that rows are comparable):
  `CBT_LAG30` – the CBT cars with the L6 steering lag τ = 30 ms (commanded angle observed; nominal-model
  controllers, as L6's rows without a lag model); `CXD` – the nominal car under the L3 disturbances, seeds
  7400–7429 (phase A's set; with L3's measurement noise; tuning-free); `CXL8` – the nominal car on the four
  L8 test tracks with their L8 plans (time limit 1.6 × the plan). `CXD` and `CXL8` repeat phase A's
  conditions; the phase-A records of `nmpc` and `mpcc` there are compared with this phase's re-runs as a
  reproducibility check (the code of both is unchanged). *Clarification of revision 0 (review n3):* with the
  code and the inputs unchanged the re-runs are expected to be **identical** to phase A's records in every
  quantity that does not measure time (outcome, lap time, distance, trajectory metrics; phase A showed these
  controllers to be deterministic); any difference is reported as such and the re-run is then not used as a
  new result.
* Start: the state of the 5 % plan at s = 0; one lap; time limit 121 s on Spielberg (1.6 × the plan).

## 3. Controllers

Shared (as phase A): the nominal model (`DynVehicle()`), η = 0.95, the 5 % plan, N = 30 stages of 50 ms, the
input held for 50 ms, the L2 traction control (0.99) on the executed input at every 10 ms simulator step; all
see the true state (no noise or delay except in `CXD` / `CBT_LAG30`); none knows the car's parameters (no
oracle).

* **`tracker`** – the 5 % plan and the L1 dynamic tracker (L1 baseline; not tuned).
* **`nmpc`** – `FrenetMPC` tracking the 5 % plan, **phase A's CXV choice: tracking-weight scale c = 2, margin
  0.6 m**, IPOPT to convergence, the shifted plan on a failed solve (phase A, unchanged).
* **`mpcc`** (secondary baseline) – `DynamicMPCC`, phase A's choice after amendment A2 (racing line, soft stage
  speed bound, q_c 8, q_v 2, margin 0.3 m), unchanged.
* **`mppi`** (primary) – section 3.1–3.4, new module `control/mppi.py`.
* **`mppi_lp`** (secondary) – `mppi` with low-pass-filtered sampling noise after LP-MPPI (B.5), section 3.5.
* **`mppi_big`** (secondary) – `mppi` with 4 × its sample budget and its CXV setting, untuned: whether a lap
  `mppi` loses is lost to the sample budget (offline; not real-time).

### 3.1 The optimal control problem (`nmpc`'s, unchanged)

State x = [s, n, ξ, vx, vy, r, δ] (Frenet, as `FrenetMPC`), one RK4 step per 50 ms stage of the NMPC's model
(`dynamics.tyre_forces` / `body_accel`, no force saturation), the curvature and the reference (n_ref, ξ_ref,
v_ref of the 5 % plan) as the NMPC's periodic linear interpolants. Cost of an input sequence from the measured
state, with the NMPC's weights at c = 2 (w_n 2, w_ξ 20, w_v 0.4, w_δ̇ 1, w_dF 0.5 per (10⁴ N)², track 10⁴,
limits 10⁶) and margin 0.6 m:

    J = Σ_{i=1..N} h [w_n (n_i − n_ref)² + w_ξ (ξ_i − ξ_ref)² + w_v (vx_i − v_ref)²]
      + Σ_{i=0..N−1} [h w_δ̇ δ̇_i² + w_dF (ΔF_d,i² + ΔF_b,i²)]          (ΔF from the previous stage; stage 0 from the
                                                                      previous control step's output; forces in 10⁴ N)
      + Σ_{i=1..N} Σ_{axle points} w_track h (σ² + 10 σ),  σ = max(0, violation of the track half-width − body − chord)
      + Σ_{i=0..N−1} Σ_{4 limits} w_limit h (τ² + 10 τ),   τ = max(0, u_axle² − η²) for each ellipse,
                                                          max(0, |α_axle| / α_peak − 1) for each slip angle

This is `FrenetMPC(mode="track")`'s NLP with its slacks eliminated: for given inputs the optimal slacks are
exactly these σ and τ, so J equals the NLP's objective at that point (checked numerically, section 9).
The NMPC's **hard** limits are enforced by clamping in the rollout (the samples' g(v) of B.3): steering rate
to ±δ̇_max and so that δ stays within ±δ_max over the stage (δ̇ is constant in a stage, so this is exact);
drive force ≥ 0 and ≤ P_max / vx (power); each axle's longitudinal force ≤ η μ_x F_z of that axle (solved in
closed form – the normal loads depend on the force through the load transfer, linearly). The bound vx ≥ 2 m/s
(a state bound of the NMPC that cannot be clamped) is enforced by giving a sample that violates it, or whose
cost is not finite, the weight 0.

**Input parameterisation (deviation from `nmpc`, stated) – superseded by amendment B1 (approved 2026-10-22: MPPI
samples the NMPC's three inputs; the paragraph is kept as the record of revision 0):** MPPI samples u = [δ̇, F_x] with one net
longitudinal force, split as F_d = max(F_x, 0), F_b = min(F_x, 0) (the NMPC's two forces, so J is
evaluated on the NMPC's three inputs). Sampling F_d and F_b separately would clip half of all noise at
zero. The NMPC can drive and brake at once; MPPI cannot. Whether the NMPC's optimum does so, and by how
much, is checked on `nmpc`'s nominal lap (section 9).

### 3.2 The MPPI iteration (B.3, Algorithms 1–2; the update as implemented in B.9)

Given the nominal sequence U = (u_0 … u_{N−1}) and the measured Frenet state x_0:

1. Draw ε_k,i ~ N(0, Σ), Σ = diag(σ_δ̇², σ_F²), independent over k = 1 … K and i (white noise, B.3).
2. v_k,i = u_i + ε_k,i for the first K − K₀ samples, v_k,i = ε_k,i (zero mean) for the last K₀, α = 0.01 (B.3:
   "less than one percent"). *Clarification of revision 0 (review m2):* K₀ = max(1, ⌊α K⌋) – 2 of 256, 5 of 512,
   10 of 1 024, 20 of 2 048, 40 of 4 096.
3. Roll out all K samples in one batched (numpy) RK4 integration with the clamped inputs v′_k,i = g(x_k,i, v_k,i).
4. S_k = J(v′_k) (section 3.1); ρ = min_k S_k; w_k = exp(−(S_k − ρ)/λ) / Σ_j exp(−(S_j − ρ)/λ).
5. U ← Σ_k w_k v′_k (B.9's form; equals U + Σ w (v′ − U)); apply u_0; shift U by one stage, the last
   entry (0, 0) (B.3, Table II).

*Clarification of revision 0 (review m1a):* if **no sample is valid** – every sample has vx < 2 m/s at some
stage or a cost that is not finite, so every weight is 0 and step 4 would divide by zero – step 5's average is
skipped: U is left as it was (the sequence shifted at the end of the previous step), its u_0 is applied and U
is shifted. Such steps are counted (section 5) and the case is covered by a unit test; during the 20
iterations of a lap's first call an iteration without a valid sample leaves U unchanged in the same way.

One iteration per 50 ms control step (B.3). At the first call of a lap, U = 0 and 20 iterations (the NMPC's
first solve also starts from a crude guess and runs to convergence). Random numbers: one
`numpy.random.default_rng` per lap, seeded with (set code, car index, replicate) – NOM 0, CXV 1, CBT 2,
CBT_LAG30 3, CXD 4 (car index = disturbance seed), CXL8 5 (car index = track index) – fixed here, so every lap
is reproducible. *Clarification (review n3 of packet 2):* the seed does not depend on the setting, so on CXV all
settings of a controller see the same random stream on the same car (common random numbers) – the comparison
of settings in the tuning is paired.

**Deviations from B.3, stated:**
(a) **no importance-sampling control-cost term** (γ uᵀΣ⁻¹v, γ = 0): in the derivation it is the control cost
(λ/2) uᵀΣ⁻¹u – a penalty on the input *magnitude* (here forces of up to ~3 · 10⁴ N) that the NMPC's objective
does not contain; with it MPPI would minimise a different problem. LP-MPPI (B.5) has no such term either;
B.3's own γ = 0.1 at λ = 12.5 was small;
(b) **no Savitzky–Golay filter** on the update: smoothness comes from the NMPC's input-rate penalties in J,
and a smoothing variant is tested separately (`mppi_lp`);
(c) the model is the NMPC's analytic model with nominal parameters (B.3: learned models);
(d) the clamping includes the state-dependent limits of the NMPC (power, axle forces) inside the rollout.

### 3.3 Interface

The NMPC's controller interface (`MPCController`: Frenet transform of the measured state, unwrapped s,
re-solve every 50 ms, hold, traction control 0.99). There is no failed solve and no fallback: every step
applies the updated u_0. Section 5 counts the steps in which no sample was free of penalties.

### 3.4 Sample budget K (fixed before any CXV lap)

K is the largest of {256, 512, 1 024, 2 048, 4 096} whose **median** MPPI step time on the nominal development
lap (section 9; one process, no other job running, single-threaded BLAS) is at most **40 ms** – 80 % of the
50 ms control period of L9's rule, on this machine's one core. *Clarification of revision 0 (review m1b):* the
timing uses the grid's centre setting (λ 1, c_σ 1; the step time hardly depends on λ); if even K = 256 has a
median above 40 ms, the rule selects nothing and an amendment goes to the reviewer – neither a K below 256
nor a looser 40 ms is adopted without it. K is then fixed for `mppi` and `mppi_lp`;
`mppi_big` uses 4 K. The measured step times of every K go to the reviewer with the code (packet 2). The
benchmark itself runs offline with 20 jobs in parallel, so its step times include that load and are reported
as such (the real-time question belongs to phase G).

### 3.5 `mppi_lp` (B.5; choices where LP-MPPI is silent, stated)

`mppi` with each noise sequence ε_k,·,j (per sample k and input j, along the 30 stages) filtered by a **causal**
digital Butterworth low-pass filter of order 2 and cut-off f_c (sampling rate 20 Hz; `scipy.signal.butter`,
`lfilter`, zero initial state) and **rescaled** by 1 / ‖h‖₂ (h the filter's impulse response over 30 stages), so
that the filtered noise has the stationary standard deviation σ again and only its spectrum changes (LP-MPPI
states neither; without rescaling a 1 Hz filter would cut the exploration by ~3×, confounding the filter with
the noise level). *Clarification of revision 0 (review n2):* with the zero initial state the filter has a
transient – the first stages' filtered noise has a smaller variance than σ² (the first value is b₀ ε₀) – and
the 1 / ‖h‖₂ scaling corrects only the stationary variance; the applied input u_0 is therefore perturbed less
in `mppi_lp` than in `mppi`. Accepted as part of the variant (LP-MPPI's applied control is not filtered either),
and stated. Everything else as `mppi`, with `mppi`'s CXV setting.

## 4. Tuning (CXV only, one lap per car and setting, replicate 0)

* **`mppi`:** λ ∈ {0.1, 1, 10} × noise scale c_σ ∈ {0.5, 1, 2} on the base σ_δ̇ = 0.03 rad/s, σ_F = 2 000 N
  (9 settings). Basis of the values (no lap has run): the NMPC's cost of a good horizon is of order 1 – lateral
  errors of ~0.5 m cost h · 2 · 0.25 per stage – so λ = 0.1 / 1 / 10 make cost differences of 1 decisive /
  moderate / weak; σ_δ̇ = 0.03 rad/s moves δ by ~0.008 rad over the horizon (σ h √N), which is a lateral
  acceleration of ~15 m/s² at 80 m/s and ~1.4 m/s² at 25 m/s; σ_F = 2 kN is ~7–15 % of the braking force range.
  Whether these scales are sensible is the purpose of the development check of section 9.
* **`mppi_lp`:** f_c ∈ {1, 2, 4} Hz (Nyquist 10 Hz) on `mppi`'s CXV choice (3 settings, after `mppi`'s
  selection).
* **Rule (phase A's, with its clarification):** the most completed CXV laps; ties by the mean lap time on the
  CXV cars **every** tied setting completes (skipped if none); then the setting closest to the grid's centre –
  for `mppi` the smaller |log λ|, then the smaller |log c_σ|, then the smaller λ, then the smaller c_σ; for
  `mppi_lp` the higher f_c (closest to `mppi`); settings with different numbers of CXV laps stop the
  selection with an error.
* `nmpc`, `mpcc`, `tracker` keep their phase-A settings (not re-tuned); `mppi_big` is not tuned.
* The chosen settings and all CXV records go to the reviewer before any CBT lap.

## 5. Always reported

Per controller and set, as phase A §5: completed laps (exact 95 % intervals), lap time, distance to failure
and stop reason; contouring error to the racing line and to the centre line; tracking error to the plan;
constraint use (minimum edge margin, track and limit penalties of the executed prediction); tyre utilisation
(max and mean ellipse usage per axle); control effort (steering-rate RMS, force-increment RMS per step and over
50 ms); step / solve time (mean, P95, P99, max) and the share above 50 ms; for `nmpc` / `mpcc`
non-converged solves and fallbacks; the speed / plan-speed ratio (P5, median, P95) and the lateral distance to
the plan's line. **For the MPPIs in addition**, per lap: the effective sample size 1 / Σ w_k² (P5, median, P95
over the steps), the share of steps in which **no sample is free of penalties** (ρ contains a track or limit
penalty – the case B.11 describes), the share of samples with a penalty, the number of samples given weight 0
(vx < 2 m/s or a non-finite cost), the steps without any valid sample (clarification m1a; U only shifted), and
the cost of the nominal sequence before and after each update.
**Repeatability:** `mppi` runs CBT with replicates 0, 1, 2 (only replicate 0 enters H-B1 / H-B2); reported:
completions per replicate, the cars whose outcome changes between replicates, the lap-time spread of the cars
all three complete.

## 6. Statistics

Exact McNemar on paired laps, exact intervals, exact signed-rank on paired lap times (mid-ranks, the exact
distribution); the two primary tests with Holm. Secondary, uncorrected, with the number of comparisons
stated (12): McNemar and, on the common completions, signed-rank of `mppi` – `mpcc`, `mppi_lp` – `mppi`,
`mppi_lp` – `nmpc`, `mppi_big` – `mppi`, `mppi_big` – `nmpc`; replicates 1 and 2 of `mppi` against `nmpc`
(McNemar only, descriptive of H-B1's sensitivity to the random stream). No claim across them.

## 7. Second environment

As L7–L9 and phase A (deferred by the user): H-B1 and H-B2 are marked as not cross-checked until the
Python 3.11 environment exists.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). No MPPI code before its APPROVE.
2. `control/mppi.py` (formulation in the module docstring), unit tests, the development checks of section 9,
   the runner and report code (`experiments/cx_b.py`, `scripts/run_cx_b.py`, `scripts/report_cx_b.py`,
   phase A's structure and checks); review (packet 2) before any CXV lap; K fixed there (section 3.4).
3. CXV tuning: 9 `mppi` settings × 10 cars, then 3 `mppi_lp` settings × 10 cars (120 laps, ~1–2 min each at
   K's budget); results to the reviewer (packet 3).
4. CBT and NOM from a pinned worktree (`C:\Users\yuanche\F1-Race-worktrees\`, input SHA-256 list,
   single-threaded BLAS, one process per job, `results/CX/B/`): `tracker`, `nmpc`, `mpcc`, `mppi` × 3
   replicates, `mppi_lp`, `mppi_big` – 51 cases × 8 runs ≈ 408 laps, ~16 CPU-h (the IPOPT controllers ~4 min
   per lap, `mppi_big` ~4–5 min); report in CONTROL_EXPANSION_BENCHMARK.md, review (packet 4).
5. Secondary conditions (`CBT_LAG30`, `CXD`, `CXL8`; all controllers, replicate 0), review (packet 5), final
   review, merge of `cx/B-mppi`.

Deviations from the roadmap's phase-B row, stated: its "validation conditions" (nominal; L3 disturbances; L6
lag; L9 budget) become NOM (development), L2 mismatch (primary – so that the NMPC baseline is the one of
phase A), L6 lag, L3 disturbances and L8 tracks (secondary), and the 50 ms budget through K (section 3.4); the
full L9 real-time rule (deadline misses on a pinned core in WSL) is phase G. The roadmap's "then
risk-sensitive / robust / adaptive covariance" variants are not in this protocol: RA-MPPI's 3 · 10⁵ rollouts
per step do not fit a CPU budget (B.6), Tube-MPPI / RMPPI need the ancillary controller of phase C (B.7), the
adaptive covariance (B.10) is a candidate for a later amendment or the frontier list.

## 9. Development checks (planned; none has run)

No MPPI code exists at revision 0. After the protocol's APPROVE and before any CXV lap, on the nominal car
only (records in `results/CX/B/dev/` with the commit and a dirty flag), reported in packet 2:

1. **Model:** the batched Frenet RK4 step equals `FrenetMPC`'s CasADi step to 10⁻⁹ on random states and inputs.
2. **Cost:** J of section 3.1 at an IPOPT solution of `nmpc` (states from its nominal lap) equals the NLP's
   objective there (relative 10⁻⁶) – the slack elimination of section 3.1; the clamps keep every input inside
   the NMPC's hard limits; the split F_x → (F_d, F_b): whether `nmpc`'s nominal lap ever drives and brakes at
   once (and by how much).
3. **Iteration:** weights sum to 1; λ → 0 picks the best sample; a single zero-noise sample leaves U unchanged;
   the shift; the seeding (identical laps for the same seed).
4. **Optimality gap (diagnostic, no threshold):** at 20 states of `nmpc`'s nominal lap, J of MPPI's sequence
   after 1, 5 and 50 iterations from the shifted IPOPT solution, and from U = 0, against IPOPT's optimum.
5. **K timing** (section 3.4).
6. **Development laps:** NOM with each of the 9 settings of section 4 at the chosen K (and `mppi_lp` at the
   grid centre). **Gate:** if no setting completes the nominal lap, the grid or the formulation is revised by
   an amendment reviewed before any CXV lap (as CX-A amendment A2) – not tuned on CXV until something works.
   These laps choose nothing: the selection is section 4's, on CXV.

### 9. Results of the development checks (2026-10-22; packet 2)

Nominal car only; records in `results/CX/B/dev/` (git-ignored), each with its commit and `dirty = false`.

1. **Model** (`checks.json`, 57146e4): 2 000 random states and inputs, largest relative difference to
   `FrenetMPC`'s CasADi step 6.7 · 10⁻¹⁶ (absolute 1.4 · 10⁻¹⁴).
2. **Cost** (`checks.json`, `nmpc_nom.npz`, 57146e4): at 20 solves of `nmpc`'s nominal lap (c 2, margin 0.6: 76.40 s,
   1 528 solves) the NLP's objective with the slacks eliminated equals MPPI's cost on the rollout of IPOPT's
   inputs to 1.3 · 10⁻¹⁴ (relative; constraint violation < 10⁻¹⁵); at IPOPT's own iterate it does not (B1.4).
   The drive-and-brake question led to amendment B1. The clamps: unit tests (below).
3. **Iteration** – unit tests at 2695112: `tests/test_mppi.py` 12 (model; cost = NLP objective for both
   parameterisations; the clamp of revision 0; the clamp of B1 – review c2: hard limits, fixed point,
   single-force clipping, direction; K₀; λ → 0 picks the best sample; samples equal to U leave it unchanged;
   U a convex combination; shift; seeding; no valid sample (m1a); the low-pass noise and its transient; a
   closed-loop smoke for both parameterisations) and `tests/test_cx_b.py` 5. Determinism in closed loop: the
   development laps repeated with the same seed (below, `nom_diagnostic.jsonl` replicate 0) give the same
   distance / lap time.
4. **Optimality gap** (`gap.json`, 8d2051b; three inputs, λ 1, c_σ 1, K 2 048; J* = the cost of IPOPT's inputs on
   the rollout, B1.4), 20 solves of `nmpc`'s nominal lap:

   | sequence | median cost | median / J* | worse than the warm start |
   |---|---|---|---|
   | IPOPT's solution (J*) | 0.158 | 1 | – |
   | warm start: IPOPT's previous solution shifted | 0.99 | 3.7 | – |
   | after 1 MPPI iteration from it | 27.7 | 63 | 16 of 20 |
   | after 5 / 50 iterations | 14.6 / 10.2 | 53 / 51 | 16 / 16 of 20 |
   | from U = 0 after 50 iterations | 9.95 | 48 | – |

   One MPPI iteration from a near-optimal sequence makes it worse in 16 of 20 cases (the samples are drawn
   around U, none is U itself, and their weighted mean carries their noise); the iteration settles at about
   50 × J* whether it starts warm or from zero.
5. **K timing** (`timing.json`, 2695112; three inputs; the grid centre; one closed-loop NOM lap per K, one after
   the other, each in a fresh process, nothing else running): median step 10.1 / 14.9 / 22.0 / **38.4** /
   103.9 ms for K = 256 / 512 / 1 024 / **2 048** / 4 096 → **K = 2 048** (section 3.4). Every one of these laps
   left the track at turn 1 (463–473 m, ~150 steps each), so the medians come from the straight and the
   braking zone of turn 1 only. *Informal timings before this one (review request, disclosed):* (i) the first
   version of `control/mppi.py` (two inputs, `np.interp(…, period=L)`), not yet committed, one static state
   (s = 100 m), 15 calls, first iterations 1, diagnostics off: 19.3 / 23.7 / 31.9 / 45.6 / 116.2 ms for K = 256
   … 4 096; (ii) a profile (K 1 024) showed `np.interp` with `period` taking ~38 % (it re-sorts the grid at every
   call) – `_periodic` was changed to a pre-tiled grid over [−L, 3L] with the breakpoints and values of
   `nmpc._periodic_interpolant`, already in 57146e4; re-timed (two inputs): 9.1 / 13.6 / 19.7 / 33.8 / 84.9 ms;
   (iii) after the three-input option (working tree before d80e1a6, which also moved the net-force bounds into
   their branch): two inputs 9.8 / 13.4 / 20.0 / 35.2 / 85.6 ms, three inputs 10.0 / 14.5 / 21.4 / 36.6 /
   91.8 ms. No other change was made for speed.
   **Re-timing on the primary (review c1 of packet 2, after amendment B2; `timing_lp.json`, b3742e7):** `mppi_lp` at
   the grid centre (2 Hz), the same conditions, one complete nominal lap per K:

   | K | lap | steps | step ms mean / P50 / P95 / P99 / max | steps above 50 ms |
   |---|---|---|---|---|
   | 1 024 | off track at 3 831 m | 1 432 | 27.3 / **25.0** / 41.0 / 48.3 / 59.0 | 0.8 % |
   | 2 048 | 82.21 s (as in 9.6) | 1 645 | 46.4 / **44.6** / 61.1 / 71.0 / 82.1 | 27 % |

   The low-pass filtering of the K × 30 × 3 noise values adds ~6 ms at K 2 048 (44.6 against the white noise's
   38.4 ms); the median at K 2 048 is above 40 ms, so by the rule of c1 **K_RT = 1 024** (no further amendment),
   and `mppi_big` = 4 K = **4 096**. `mppi` (white, untuned, B2.3) runs at the same K = 1 024. The development laps
   of 6 and 7 were at K 2 048; at K 1 024 the primary's nominal lap above left the track at 3 831 m (the 3.84 km
   corner of 7).
6. **Development laps** (`laps.jsonl`, 2695112; K 2 048; 10 jobs in parallel, so the step times carry that load):

   | controller, setting | result | ESS median | no penalty-free sample | overlap N P50 / P90 |
   |---|---|---|---|---|
   | `mppi` λ 0.1, c_σ 0.5 | off track at 1 398 m | 1.0 | 0.18 | 243 / 1 680 |
   | `mppi` λ 0.1, c_σ 1 / 2 | off at 465 / 440 m | 1.0 / 1.0 | 0.23 / 0.19 | 441 / 3 367; 882 / 5 398 |
   | `mppi` λ 1, c_σ 0.5 / 1 / 2 | off at 467 / 471 / 458 m | 12 / 10 / 1.1 | 0.25 / 0.23 / 0.26 | 574–898 / 1 167–4 492 |
   | `mppi` λ 10, c_σ 0.5 / 1 / 2 | off at 437 / 462 / 459 m | 1 028 / 416 / 80 | 0.17 / 0.24 / 0.24 | 1 108–2 281 / 1 395–3 388 |
   | `mppi_lp` λ 1, c_σ 1, f_c 2 Hz | **82.21 s** | 6.6 | 0.07 | 235 / 837 |

   **Gate (6): no `mppi` setting completes the nominal lap** – 8 of 9 leave at turn 1. `mppi_lp` at the grid
   centre does. `nmpc` on the same car: 76.40 s, overlap median 35 N.
7. **After the gate – diagnostics on the nominal car** (to find the cause before proposing an amendment;
   `nom_diagnostic.jsonl` at 0495c4b and `nom_diagnostic_big.jsonl` at 8d2051b; λ 1, c_σ 1; replicates 0–2):

   | variant | laps completed | where the others leave |
   |---|---|---|
   | `mppi`, K 2 048 (replicates 0–2; 0 repeats 9.6 exactly) | 0 of 3 | 471, 472, 1 396 m |
   | `mppi`, K 512 with 4 iterations per step (~ the same time) | 0 of 3 | 462–465 m |
   | `mppi`, K 8 192 (offline, 4 K) | 0 of 3 | 472–474 m |
   | `mppi_lp` f_c 2 Hz, K 2 048 (0 repeats 9.6 exactly: 82.21 s) | 1 of 3 | 3 836, 3 836 m |
   | `mppi_lp` f_c 1 / 4 Hz, K 2 048 (replicate 0) | 0 of 2 | 3 838 / 1 398 m |
   | `mppi_lp` f_c 2 Hz, K 8 192 (offline) | 1 of 3 (80.70 s) | 3 839, 3 846 m |

   Reading (the traces of replicate 0 of `mppi` and replicate 1 of `mppi_lp`, K 2 048 – first seen in scratchpad
   scripts, then recorded for review c2 of packet 2 by `cx_b_dev.py trace`: `traces.npz`, `traces.json`, b3742e7;
   the numbers below are the recorded ones): with white noise the car is up to 8.2 m/s below the plan on the
   main straight (first 300 m; applied overlap median 971 N), brakes for turn 1 in time, but turns in too slowly;
   from 424 m to the end of its lap no sample is free of penalties (ESS 1.0 throughout) and it leaves at the exit. More samples or more iterations per step do not change this; low-pass noise does –
   every low-pass lap at 2 Hz passes turn 1. A white-noise perturbation spread over 30 stages moves a smooth
   input change (a steering or braking ramp) by only ~σ / √30 per iteration; the low-pass noise puts its
   variance there (LP-MPPI's argument, B.5). The low-pass laps then leave at ~3.84 km in 5 of 8: in the corner
   at 3.70–3.85 km the rear axle reaches 0.96 of its ellipse (usage before saturation) with vy up to +2.43 m/s,
   from 3 755 m to the end no sample is free of penalties (ESS 1.0) and the car slides wide at the exit – the corner where phase A's MPCC with a low
   contouring weight also left by millimetres.
8. **The gate at the K in force (review g1 of packet 3; `gate_k1024.jsonl`, 99955f4):** `mppi_lp`'s nine first-step
   settings (2 Hz, K_RT = 1 024) on the nominal car, replicate 0, development laps (no selection; 9 jobs in
   parallel):

   | λ \ c_σ | 0.5 | 1 | 2 |
   |---|---|---|---|
   | 0.1 | off at 3 839 m | off at 470 m | off at 456 m |
   | 1 | off at 3 839 m | off at 3 831 m | off at 461 m |
   | 10 | off at 462 m | off at 1 400 m | off at 462 m |

   **None completes the nominal lap: the gate of 9.6 fails at K_RT = 1 024 for the primary as well** (and for the
   white-noise `mppi`, 9 / 9 at K 2 048). Five leave at turn 1 (the larger noise or the extreme temperatures), three
   at the 3.84 km corner, one at 1.4 km. Median ESS 1.0–1.3 at λ 0.1, 4.5–32 at λ 1 (c_σ ≤ 1), 21–256 at λ 10; step
   medians 25.5–33.0 ms (9 laps in parallel). Amendment B3 (proposed) follows.

## Amendments

* **B1 (2026-10-22; proposed in 8d86ff2, APPROVED by the reviewer 2026-10-22), MPPI samples the NMPC's three
  inputs – after development check 9.2, before the K timing and any development lap.** From here on it
  supersedes section 3.1's paragraph "Input parameterisation (deviation from `nmpc`, stated)": MPPI's input is
  [δ̇, F_d, F_b] and section 3.1 is the NMPC's problem without a restriction. The alternative B1.3 is not
  adopted (review).
  *Cause:* section 3.1 replaced the NMPC's two longitudinal inputs by one net force F_x and left open whether
  the NMPC's optimum drives and brakes at once (checked in 9.2). It does. On `nmpc`'s nominal lap (c 2, margin
  0.6: 76.40 s, 1 528 solves; `results/CX/B/dev/checks.json`, code 57146e4) the overlap min(F_d, −F_b) of the
  applied stage-0 input is mostly negligible – median 35 N, 21 N in the solves whose net force drives – but it is
  above 1 kN in 7.9 % of all solves and in 31 % of the braking solves (median 200 N, P90 1.8 kN, max 5.7 kN), in
  the braking zones at 0.2–0.4, 1.2, 2.0, 2.5–2.9 and 3.6–3.9 km – where the L2 cars fail (CX-A: turn 1, ~2.7 km,
  ~3.83 km). Driving the rear axle while braking moves the effective brake balance forward (the brake split is
  fixed at 60 / 40): a degree of freedom the net force cannot express. Evaluated through the net-force
  parameterisation (overlap removed), IPOPT's inputs cost 1.3 · 10⁴ to 1.6 · 10⁶ (ellipse penalties) instead of
  < 10 at 6 of the 20 checked solves. The small overlaps elsewhere come from the input-rate cost, a sum over the
  two force channels (a change split over both costs half).
  1. *Definition:* MPPI samples u = [δ̇, F_d, F_b] with independent noise N(0, diag(σ_δ̇², σ_F², σ_F²)) – the
     NMPC's own inputs and input set; the noise scale c_σ multiplies all three. Clamps: the steering rate and
     angle as section 3.1; F_d ≥ 0, F_b ≤ 0 and F_d ≤ P_max / vx by clipping; the two axles' longitudinal-force
     limits – linear in (F_d, F_b) at a given state – by scaling the force pair towards (0, 0) by the largest
     factor in [0, 1] that meets both (direction-preserving; with one force non-zero it is the clipping of
     revision 0). J is evaluated on the clamped three inputs as before (section 3.1 is then the NMPC's problem
     without a restriction). Everything else in sections 3.2–3.5 unchanged (U, ε, v have three columns).
  2. *Known consequence, reported:* clipping at F_d = 0 and F_b = 0 makes the perturbations one-sided there
     (mean +0.4 σ_F before weighting), so where the cost is indifferent – the overlap direction, e.g. on a
     straight below the power limit – the update can drift towards overlap until the rate and tracking costs
     stop it. Section 5 adds per lap the overlap min(F_d, −F_b) of the applied inputs (median, P90, max) for
     MPPI **and** the NMPC (review c3), with the remark that driving and braking at once is a freedom of this
     model (fixed 60 / 40 brake split, rear-wheel drive) – in effect left-foot braking or a brake-balance
     adjustment – which both controllers have and which says nothing about what a real car would do.
  3. *Alternative, not proposed:* keep the net force of revision 0 and state that MPPI solves the NMPC's problem
     restricted to F_d · F_b = 0 – the comparison of optimisers would then include that restriction exactly in
     the braking zones where the laps are decided.
  4. *Also from 9.2, for the record:* MPPI's cost equals the NLP's objective with the slacks eliminated on the
     rollout of IPOPT's inputs (largest relative difference 1.3 · 10⁻¹⁴ at the 20 solves; constraint violation
     < 10⁻¹⁵). At IPOPT's own iterate it does not: IPOPT's slack variables sit at about −10⁻⁸ (its bound
     relaxation), which the linear penalty terms (10 w h per slack, w up to 10⁶) turn into an objective of about
     −0.35 instead of ~0.1, and its dynamics defects reach 4.5 · 10⁻⁴. The optimality-gap check (9.4) therefore
     uses as IPOPT's optimum J* the cost of IPOPT's three inputs on the rollout.
  5. The K timing (section 3.4) and every development lap run after the reviewer's decision, with the decided
     parameterisation (three inputs: 90 sampled values per sequence instead of 60; the K rule is unchanged).
  6. *Review conditions (2026-10-22):* c1 this status note before the K timing; c2 unit tests of the clamp – the
     NMPC's hard limits met within tolerance, feasible inputs a fixed point, with one force non-zero the
     clipping of revision 0, the direction of the force pair kept (packet 2); c3 the overlap for MPPI and the
     NMPC with the remark of B1.2 (report).
* **B2 (2026-10-22; proposed in eea5a49, APPROVED by the reviewer 2026-10-22 with packet 2, c3fca85) – after the
  development gate (9.6) failed for `mppi`: the low-pass-noise MPPI becomes the primary controller.** Review
  conditions: c1 K is re-timed with `mppi_lp` (the grid centre, 2 Hz) for K = 1 024 and 2 048 under section 3.4's
  conditions before any CXV lap (2 048 if its median is ≤ 40 ms, otherwise 1 024 without a further amendment;
  section 9.5); c2 the trace details of 9.7 regenerated as records or marked as unarchived; c3 the scope's tuning
  freedom (done). B2.7 not adopted. *Cause:* section 9, results 4, 6 and 7: no setting of `mppi` (white noise,
  T-RO's sampling) completes the nominal lap – 9 of 9 grid settings, and the grid centre with two more random
  streams, with 4 iterations per step and with 4 K samples, 17 laps, all off track, 15 of them at turn 1; one
  MPPI iteration from a near-optimal sequence makes it worse in 16 of 20 checked solves. `mppi_lp` (LP-MPPI, B.5)
  passes turn 1 in every lap at 2 Hz and completes 2 of 8 low-pass laps (82.21 s at K 2 048, 80.70 s at K 8 192;
  `nmpc` 76.40 s); the others leave at ~3.84 km (5) or 1.4 km (1, f_c 4 Hz). The gate's condition ("a setting
  completes the nominal lap") is met only by the low-pass variant.
  1. *Primary:* H-B1 and H-B2 compare **`mppi_lp`** (section 3.5) with `nmpc`; their wording, test, Holm and the
     power statement of section 1 otherwise unchanged; `mppi_lp` runs CBT with replicates 0, 1, 2 (replicate 0 in
     the tests).
  2. *Tuning (CXV):* `mppi_lp` over λ ∈ {0.1, 1, 10} × c_σ ∈ {0.5, 1, 2} at f_c = 2 Hz (9 settings; 2 Hz is the
     cut-off of the development laps), then f_c ∈ {1, 2, 4} Hz on the chosen (λ, c_σ) (3 settings; the 2 Hz lap
     is the first step's). Section 4's rule; ties in the first step by section 4's order for (λ, c_σ), in the second
     the f_c closest to 2 Hz (|log(f_c / 2)|), then the higher. 120 laps.
  3. *`mppi` (white noise)* becomes a secondary controller, **not tuned** (every development setting failed, so CXV
     laps could only select by the closeness rule): the grid centre (λ 1, c_σ 1, K_RT) on CBT, replicate 0 – the
     T-RO baseline next to the low-pass variant. `mppi_big`: 4 K with `mppi_lp`'s CXV setting (the primary's
     sample budget).
  4. *Secondary comparisons* (12, uncorrected): McNemar and signed-rank of `mppi_lp` – `mpcc`, `mppi` – `mppi_lp`,
     `mppi` – `nmpc`, `mppi_big` – `mppi_lp`, `mppi_big` – `nmpc`; replicates 1 and 2 of `mppi_lp` against `nmpc`.
  5. *Unchanged:* the optimal control problem (section 3.1 with B1), the K rule (re-applied to `mppi_lp` by review c1:
     K_RT = 1 024, section 9.5), one iteration per step, the sets, the
     statistics, the secondary conditions (with `mppi_lp` in `mppi`'s place for the uncorrected McNemar there).
  6. *Disclosure:* decided after development laps on the nominal car, which is in no test; no CXV or CBT lap has
     run. The gap result (9.4) and the 3.84 km failures make it likely that `mppi_lp` completes fewer test cars
     than `nmpc`; the benchmark measures by how much, at this budget.
  7. *Alternative, not proposed:* close phase B without a benchmark and report the development result (T-RO MPPI
     does not complete the nominal lap at the real-time CPU budget; LP-MPPI 2 of 8) – H-B1 / H-B2 untested.
* **B3 (proposed 2026-10-22, for review) – after the gate failed at the K in force (section 9, result 8): how phase B
  ends.** *Cause:* at K_RT = 1 024 none of `mppi_lp`'s nine first-step settings completes the nominal lap; with
  B2's own evidence (K 2 048: 1 of 3; K 8 192 offline: 1 of 3) and the white-noise results (0 of 17), MPPI on the
  NMPC's problem does not reliably drive the nominal car at any budget tried, and not once at the real-time
  budget. The gate's question – is the problem solvable at this budget? – is answered: no, on the nominal car.
  Options:
  1. *Proposed:* **run the pre-registered tuning and benchmark unchanged at K_RT = 1 024, with the gate of 9.6
     waived by this amendment.** The development result is the finding; the benchmark turns it into the
     pre-registered comparison on 50 test cars (H-B1, H-B2 with Holm), with `mppi_big` (4 096, offline) as the
     budget sensitivity and the white-noise `mppi` alongside. Cost is small: MPPI laps that leave early end early,
     and the 51 `nmpc` / `mpcc` laps take ~15 min with 20 workers. The CXV tuning will likely see 0 of 10 for
     every setting and fall to the closeness rule (the grid centre); that is reported as such. The report states
     that the gate failed, that the benchmark ran under B3, and that H-B1 is expected to favour `nmpc`; no claim
     beyond "at this CPU budget".
  2. *Alternative:* close phase B without a benchmark – the development result (sections 9.4–9.8) is the phase's
     result; H-B1 / H-B2 untested.
  3. *Not proposed:* drop the real-time budget for the primary (e.g. K 8 192 offline) – a different question, and
     still 1 of 3 on the nominal car; or change the method again (the unperturbed U as a sample, an adaptive
     temperature or covariance, B.10) – development without a single literature-backed candidate that addresses
     the 3.84 km failure; these go to the frontier list.
