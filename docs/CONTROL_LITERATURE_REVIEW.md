# Control expansion – literature review

Part of the L1–L9 advanced-control expansion ([CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md)).
Every method gets an entry **before** it is implemented. Each entry records what was actually read:
*full text* (formulation checked against the paper), *abstract* (the arXiv / publisher page) or
*bibliographic* (title, authors and venue confirmed only). No method is called better than another
here on the strength of an abstract.

## Phase A – model predictive contouring control (MPCC)

### A.1 Lam, Manzie, Good – *Model predictive contouring control*, IEEE CDC 2010, 6137–6142 (bibliographic + abstract)

* Formulation: MPC for biaxial contouring that trades contouring accuracy against traversal time, with
  state and actuator constraints; a linear time-varying formulation for real time; stability by an
  added contraction constraint. Journal version: *Model Predictive Contouring Control for Biaxial
  Systems*, IEEE TCST 2013.
* Assumptions / setting: machine-tool XY table, a given path.
* Relevance: the origin of the contouring / lag error and the progress variable used by A.2.
* Not read in full (no open text found); cited for the origin of the method only.

### A.2 Liniger, Domahidi, Morari – *Optimization-based autonomous racing of 1:43 scale RC cars*, OCAM 36(5), 2015, 628–647 (arXiv:1711.07300; full text read, section 4)

* **Formulation:** the car's progress along a reference path is a state θ_A driven by a virtual input
  v (the "projected velocity"), θ_{k+1} = θ_k + T_s v_k. Contouring and lag error approximate the
  orthogonal distance to the path and the error of the projection (eq. 11):
  ê_c = sin Φ(θ)(X − X_ref(θ)) − cos Φ(θ)(Y − Y_ref(θ)), ê_l = −cos Φ(θ)(X − X_ref(θ)) − sin Φ(θ)(Y − Y_ref(θ)),
  Φ the path's tangent angle, the path a cubic spline in arclength.
* **Objective (eq. 13a):** Σ ‖ê_c‖²_{q_c} + ‖ê_l‖²_{q_l} − γ v_k T_s + ‖Δu‖²_{R_u} + ‖Δv‖²_{R_v}; q_c low (the car
  need not follow the centre line), q_l high (keeps the progress approximation accurate); − γ Σ v T_s
  replaces the final progress.
* **Constraints:** vehicle dynamics; track as two half-planes tangent to the borders at the reference
  point, softened with an ∞-norm slack penalty (eq. 12d, 14e); input and state bounds; 0 ≤ v ≤ v̄;
  obstacles through a corridor from a DP planner (not used here).
* **Model:** bicycle model without load transfer, simplified Pacejka lateral tyres, motor model for the
  rear longitudinal force.
* **Solver:** one QP per sample around the shifted previous solution (a real-time iteration; matrix
  exponential discretisation), FORCES interior-point QP solver; N = 40, T_s = 20 ms (0.8 s), ~15 ms per QP.
* **Open source:** github.com/alexliniger/MPCC (ETH Automatic Control Lab, Apache-2.0): C++ (QP by
  hpipm) and Matlab simulation environments of this paper's MPCC – its README checked, its code not
  used or read here.
* **Differences from our implementation (control/mpcc.py):** our NLP is solved to convergence by IPOPT
  (warm start: the shifted solution) instead of one QP; our model is the repository's dynamic model
  with load transfer, aero and friction ellipses (the same as the NMPC of L2–L9); the friction ellipse,
  slip-angle and power limits of the NMPC are added; a terminal speed bound v_x,N ≤ v_ref(θ_N) is added
  because a 1.5 s horizon does not cover this car's braking zones (see the module docstring); the
  half-planes are applied to the front and rear axle points; θ_0 is the projection of the measured CG.
* **Known limitations:** the error approximations hold only while θ stays near the projection (hence
  the high q_l); with a short horizon the progress term can drive the car into corners it cannot
  brake for; the weights (q_c, q_l, γ) are tuned by hand in the paper.

### A.3 Kabzan, Hewing, Liniger, Zeilinger – *Learning-based model predictive control for autonomous racing*, IEEE RA-L 4(4), 2019, 3363–3370 (bibliographic)

* An MPCC whose nominal model is corrected by a Gaussian-process residual learned from driving data
  (as cited by later work). Relevant to phases D / E (learned error dynamics) – to be read before
  those phases.

### A.4 Ma, Zhao, Jin, Wang, Wang – *Dynamic curvature-adaptive MPCC with physical feasibility constraints for high-speed racing under frequent curvature variations*, Control Engineering Practice 171 (2026) 106846 (abstract)

* A curvature-dependent weight function shifts the balance between contouring and lag error; friction
  ellipse constraints with curvature–velocity coupling constraints bound the feasible speed.
* Compared with feed-forward/feedback, MPCC, MPC-CBF and LPV-MPC (lap-time reductions of 1.7–4.3 s
  claimed in the abstract).
* The full text could not be obtained (2026-10-21); the curvature-aware MPCC of phase A is therefore based on
  CiMPCC (§A.8), whose formulation is open – CX-A amendment A1. This paper is not implemented.

### A.8 Li, Xie, Hu, Su – *Reduce Lap Time for Autonomous Racing with Curvature-Integrated MPCC Local Trajectory Planning Method* (CiMPCC), arXiv:2502.03695 (Feb 2025; full text read, HTML v1)

* **Formulation:** the centre line's curvature by finite differences (eq. 5), a moving-average filter (eq. 7) and
  min-max normalisation K ∈ [0, 1] (eq. 8); β = g(K) = exp(−α K²) at the centre-line point nearest the car
  (eqs. 9–10); the cost J_Ci = Σ_k (1 − β)‖v_k − v_low‖²_{R3} + β‖v_k − v_high‖²_{R3} added to a standard MPCC
  (contour and lag error weight Q = diag(800, 800), progress γ = 40, rate and input-reference penalties).
* **Assumptions / setting:** kinematic bicycle model, 1:10 car (F1TENTH stack), N = 10, CasADi NLP with multiple
  shooting and warm start; constant aggressive / safe speeds (safe ≈ 0.65 × aggressive).
* **Results (as reported):** 17 laps on one self-built track; mean lap 14.20 s against 16.11 s (standard MPCC)
  and 16.08 s (minimum-curvature line + velocity planning); the abstract's 11.4–12.5 % and the conclusion's
  11.8 % differ slightly; no statistical test; parameters tuned at runtime.
* **Open source:** none stated.
* **Differences from our `mpcc_ca`** (CX-A amendment A1): the dynamic model; v_high = the plan's speed at each
  stage (also the stage bound) and v_low = 0.65 v_high instead of two constants; a 50 ms, 30-stage horizon.
* **Known limitations:** one car, one track, no model mismatch evaluated (the authors list it as future work).

### A.5 Nguyen, Nguyen, Amine, Vo-Duy, Mangharam, Nghiem – *AD-MPCC: Adaptive Differentiable Model Predictive Contouring Control for Autonomous Racing*, arXiv:2607.00141 (v1 30 Jun 2026, v2 11 Sep 2026; abstract)

* Online estimation of a parameterised Pacejka magic formula by a regularised moving-horizon estimator
  with exponentially decaying weights; a differentiable MPCC (Diff-MPCC) tunes the objective weights
  against long-horizon costs, and a Pacejka-informed learned model adapts them online. Simulation only;
  no code mentioned. **Upgrade candidate** (differentiable MPCC, learned cost tuning, online tyre
  adaptation) after the standard MPCC; its MHE part overlaps phase D.

### A.6 AL-Sunni, Almubarak, Horng, Dolan – *LLA-MPC: Fast Adaptive Control for Autonomous Racing*, arXiv:2505.19512 (May 2025; abstract)

* A bank of candidate vehicle models; a look-back window selects the most accurate one, its friction
  estimate feeds a look-ahead planner inside an adaptive MPC; no learning phase. **Phase D candidate**
  (model-bank adaptation).

### A.7 L4acados – *Learning-based models for acados, applied to Gaussian process-based predictive control*, arXiv:2411.19258 (2024; search result only)

* GP-based predictive control in acados, tested on a 1:24 car and a full-scale vehicle (per the search
  result). To be read before phases D / E / G.

## Phase B – model predictive path integral control (MPPI)

Checked 2026-10-21. Full texts were read as arXiv HTML (arxiv.org/html or ar5iv); where only an
abstract or a search result was seen, the entry says so. Search for MPPI on a full-scale race car
(2024–2026): none found – the racing applications found are 1:5 (AutoRally, B.1–B.3), 1:10 (F1TENTH,
B.5) and 1:28 (B.6) scale cars, at 6–11 m/s or less; this car runs at up to ~90 m/s.

### B.1 Williams, Drews, Goldfain, Rehg, Theodorou – *Aggressive driving with model predictive path integral control*, IEEE ICRA 2016 (abstract)

* MPC for nonlinear systems with complex cost criteria from the relation between free energy and relative
  entropy (the path-integral framework); GPU implementation; fifth-scale AutoRally car, aggressive driving.
* The full text was not obtained (IEEE Xplore only); the algorithm is taken from B.2 and B.3, the same
  group's open texts. Cited as the origin of the name MPPI.

### B.2 Williams, Aldrich, Theodorou – *Model Predictive Path Integral Control using Covariance Variable Importance Sampling*, arXiv:1509.01149 (2015; full text read, ar5iv)

* **Formulation:** path-integral optimal control (the value function by the Feynman–Kac lemma as an
  expectation over trajectories), estimated by forward sampling; an importance-sampling scheme that changes
  the drift *and* the variance of the sampling distribution (exploration variance ν times the system's
  natural variance). Algorithm: K rollouts with perturbations δu around the current sequence, run in
  parallel; the running cost gains a likelihood-ratio term in δu (with the factor (1 − ν⁻¹)/2 on δuᵀRδu)
  and the control cost uᵀRδu; the update is the reward-weighted average of the perturbations; the
  sequence is shifted (u_i = u_{i+1}) and the last entry re-initialised; the first control is applied.
* **Assumptions:** control-affine dynamics with the noise entering through the actuated states only (stated
  as a limitation).
* **Setting:** cart-pole, a miniature race car on an elliptical track (~7 m/s), a quadrotor in a forest;
  50 Hz; 10 to 1 000 rollouts; compared with an MPC version of DDP. Only "with a very large increase in the
  exploration variance" did MPPI beat MPC-DDP on the race car (paraphrased from the text).
* **Relevance:** the variance is a tuning parameter, kept constant; adapting it online is named as future work.

### B.3 Williams, Drews, Goldfain, Rehg, Theodorou – *Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving*, IEEE T-RO 34(6), 2018 (arXiv:1707.02342; full text read, ar5iv – the experiments part by a second pass)

* **Formulation (Alg. 1, 2):** for k = 1 … K draw ε_k,t ~ N(0, Σ); v_k,t = u_t + ε_k,t for (1 − α)K samples and
  v_k,t = ε_k,t (zero mean) for the rest, α "less than one percent" (lets the controller recover after
  strong disturbances); roll the model out with the inputs clamped to their limits, g(v); cost
  S_k = φ(x_T) + Σ_t [c(x_t) + γ u_tᵀ Σ⁻¹ v_k,t] (γ = λ(1 − α) in the derivation, Sec. III-D2); weights
  w_k = exp(−(S_k − ρ)/λ) / Σ_j exp(−(S_j − ρ)/λ), ρ = min_k S_k; update u ← u + SGF(Σ_k w_k ε_k), SGF a
  Savitzky–Golay filter; apply u_0, shift the sequence, initialise the last entry ((0, 0) in Table II).
  The path-integral derivation needs control-affine dynamics; the information-theoretic one does not.
* **Constraints:** none in the optimiser – track limits as a map cost plus an impulse (10 000, decayed 0.9ᵗ
  over the horizon) outside the track; a slip-angle cost with an impulse beyond 0.75 rad; input limits by
  clamping the samples before the rollout ("simpler than solving a constrained QP online", paraphrased).
* **Setting (Table II):** 40 Hz, horizon 2 s, λ = 12.5, γ = 0.1, Σ = diag(0.0306, 0.0506) on normalised
  steering and throttle, cost = track + (v_x − v_des)² + slip terms, v_des 6 / 8.5 / 11 m/s; "a few
  thousand" samples on a GPU (K is not in the table); learned models (basis functions, a 2 × 32 neural
  network) from ~30 min of driving. Compared with a cross-entropy MPC.
* **Results (as reported):** 100-lap runs; the NN-model controller completes all 100 laps
  counter-clockwise at the 11 m/s target (top speed 9.06 m/s) and fails clockwise at 11 m/s from model error
  (predicts over-steer, the car under-steers); 7 of 24 settings were too reckless or unstable for 100 laps.
* **Known limitations (stated):** strong disturbances can put every sample in a poor region; too few
  samples give apparent local optima; Σ is constant (an adaptive Σ "not satisfactory"); learned-model
  errors from hidden variables.
* **Open source:** the AutoRally code base (named in B.9's benchmark); not used here.

### B.4 Kim, Park, Kwak, Bae, Lee – *Smooth Model Predictive Path Integral Control without Smoothing* (SMPPI), IEEE RA-L 2022 (arXiv:2112.09988, v8 Oct 2025; full text read, HTML v8)

* **Formulation:** input lifting – the sampled control is the derivative of the action, a_t = a_{t−1} + u_t Δt
  (eqs. 16–17); MPPI's derivation is unchanged in the lifted space; an action-smoothness cost
  Ω = Σ (a_t − a_{t−1})ᵀ ω (a_t − a_{t−1}) (eq. 15) added to S (eq. 18, with the term λ Σ uᵀ Σ⁻¹ ε).
* **Setting:** pendulum swing-up; a CarMaker Volvo XC90 on a 1 016 m kart-style circuit, K = 10 000, λ = 15,
  4 s horizon, 10 Hz, a learned NN model; baselines MPPI, MPPI + action cost, Savitzky–Golay on ε or on U.
* **Results (as reported, Table III):** MPPI variants fail by corner 2 or 3; SMPPI with Ω completes all six
  corner segments. No statistics; grid-searched parameters; simulation only.
* **Not implemented here:** this car's steering is already rate-controlled (the input is δ̇, δ a state –
  the lifting SMPPI adds); lifting the forces as well would add two states. Smoothness is tested with
  B.5 instead.

### B.5 Kicki – *LP-MPPI: Low-Pass Filtering for Efficient Model Predictive Path Integral Control*, arXiv:2503.11717 (v1 Mar 2025, v3 3 Feb 2026; accepted at ICRA 2026 per the arXiv page; full text read, HTML v3)

* **Formulation:** MPPI (Alg. 1: ε ~ N(0, Σ), U_i = U + ε_i, w_i ∝ exp(−λ J_i) with λ an inverse temperature,
  U = Σ w_i U_i, shift) with the sampled perturbations low-pass filtered before they are added
  (Alg. 2, line 5: ε^LP = LPFilter(ε, f_c, o)), a digital Butterworth filter (cut-off f_c, order o); the
  applied control is not filtered. No control-cost term in J; whether the filter is causal or zero-phase,
  and whether the filtered noise is rescaled, is not stated.
* **Setting:** Gymnasium (Hopper, Ant, HalfCheetah; Optuna, 100 trials), quadrupeds (Dial-MPC base),
  real F1TENTH time trials on a 14.2 m oval: dynamic single-track model with MF6.1 tyres, H = 30, dt 50 ms,
  30 Hz, N = 10 and 50 rollouts on one CPU core (i5-12500H), cost −v_f + soft track-edge term + slip and
  heading terms; baselines MPPI, SCP-MPPI (spline), SMPPI (B.4), coloured-noise MPPI.
* **Results (as reported):** best median distance in the 30 s trials (15 runs) at both N; the F1TENTH
  numbers are given only in a figure, and the margin is stated inconsistently (~30 cm in the text, "0.115 s"
  in the conclusion); filter parameters for F1TENTH not reported. Computation overhead +2.4 % over MPPI.
* **Used here:** the secondary controller `mppi_lp` (CX-B protocol §3) – our choices where the paper is silent
  are stated there.

### B.6 Yin, Zhang, Tsiotras – *Risk-Aware Model Predictive Path Integral Control Using Conditional Value-at-Risk* (RA-MPPI), arXiv:2209.12842 (2022; full text read, ar5iv)

* For each of M = 1 024 nominal samples, N = 300 disturbed rollouts estimate the CVaR of the running cost;
  a penalty A·CVaR (A = 10) is added where CVaR exceeds a bound C_u. 1:28 race car, 1 024 × 300 rollouts on
  an RTX 3090 at up to ~80 Hz; 10 hardware laps: collisions per lap 20.3 → 9.0, lap time 7.11 → 7.38 s against
  MPPI. CVaR is time-inconsistent – a strict bound hurts.
* **Not implemented in phase B:** 3 × 10⁵ rollouts per step are out of reach on one CPU core at 50 ms with this
  model. Candidate for the CVaR frontier item (roadmap section 5).

### B.7 Gandhi, Vlahov, Gibson, Williams, Theodorou – *Robust Model Predictive Path Integral Control: Analysis and Performance Guarantees* (RMPPI), IEEE RA-L 2021 (arXiv:2102.09027; abstract)

* A tracking controller (iLQG, contraction metrics) inside the sampled rollouts and a nominal initial state
  chosen by a constrained optimisation; a bound on the growth of the free energy; AutoRally experiments
  (RMPPI ahead of MPPI and Tube-MPPI, per the abstract). Needs an ancillary feedback controller – a phase C
  (tube MPC) topic; not implemented in phase B.

### B.8 Yin, Dawson, Fan, Tsiotras – *Shield Model Predictive Path Integral* (Shield-MPPI), IEEE RA-L 2023 (arXiv:2302.11719; abstract)

* MPPI with control barrier functions as a cost and a shielding filter on the output, few samples on a
  CPU; racing simulation and hardware with fewer constraint violations than robust MPPI variants and
  stochastic MPC (per the abstract). Phase F (safety) candidate; not implemented in phase B.

### B.9 Vlahov, Gibson, Gandhi, Theodorou – *MPPI-Generic: A CUDA Library for Stochastic Trajectory Optimization*, arXiv:2409.07563 (v1 Sep 2024, v4 Feb 2026; full text read in part, HTML v4: sections II–IV and the benchmark tables)

* The update as implemented: v = u + ε, J accumulates ℓ(y, v) − λ I(v) (an importance term with β ∈ {0, 1}),
  ρ = min_m J_m, w ∝ exp(−(J − ρ)/λ), U ← Σ w v (equivalently u + Σ w ε). Tube-MPPI (an iLQR tracking
  controller pulls the real system to a nominal one) and RMPPI (B.7) are implemented; coloured-noise
  and smooth-MPPI sampling are named but not defined in the parts read.
* **Timing (as reported, dt 0.02 s, 100-step horizon, i9-13900K):** the CPU-only Nav2 MPPI takes 0.39 ms for
  128 samples, 6.7 ms for 2 048 and 62 ms for 16 384 per optimisation – for that benchmark's model, not this
  car's; the GPU libraries stay below 1 ms up to 16 384 samples on an RTX 4090.
* **Relevance:** an order-of-magnitude reference for a CPU budget; the library itself (C++/CUDA) is not used.

### B.10 Yi, Pan, He, Qu, Shi – *CoVO-MPC: Theoretical Analysis of Sampling-based MPC and Optimal Covariance Design*, L4DC 2024, PMLR 242 (arXiv:2401.07369; abstract)

* Linear convergence of MPPI on quadratic (time-varying LQR) problems; a sampling covariance chosen to
  maximise the convergence rate instead of an isotropic one (43–54 % better than MPPI in the abstract,
  quadrotor tasks). Candidate for the adaptive-covariance follow-up of the roadmap; not implemented in
  phase B.

### B.11 *BC-MPPI: A Probabilistic Constraint Layer for Safe Model-Predictive Path-Integral Control*, arXiv:2510.00272 (2025; search result only)

* Argues that constraints enter MPPI only as penalties, so violations remain possible when no good feasible
  sample is drawn; adds a probabilistic constraint layer. Not read; noted because the same limitation applies
  to the penalty formulation of CX-B (§3) and is measured there (steps without a penalty-free sample).

## Phases C–G and the frontier methods

Entries are written at the start of each phase, before its code (roadmap section 3). Classic sources
to start from (not yet read; listed so that the check is planned, not as claims): tube MPC – Mayne, Seron, Raković (2005) and Mayne et
al. (nonlinear tube MPC, 2011); RLS / MHE – Rawlings, Mayne, Diehl (*Model Predictive Control: Theory,
Computation, and Design*); LMPC – Rosolia and Borrelli (iterative LMPC, IEEE TAC 2018) and the racing
application (Rosolia, Borrelli, 2019/2020); CBF / HOCBF / predictive safety filters – Ames et al.,
Xiao and Belta, Wabersich and Zeilinger; advanced-step NMPC – Zavala and Biegler (2009).
