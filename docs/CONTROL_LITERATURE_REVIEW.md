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

## Phase C – robust tube MPC

Checked 2026-10-22. Full texts as arXiv HTML; the journal classics of linear tube MPC were not obtained in full
(no open copy found) and are cited for their formulation as it is restated in the full texts read (C.4).

### C.1 Mayne, Seron, Raković – *Robust model predictive control of constrained linear systems with bounded disturbances*, Automatica 41(2), 2005, 219–224 (bibliographic + abstract)

* Linear discrete-time systems with bounded additive disturbances; the online problem has the nominal model's
  initial state as a decision variable (abstract). The tube formulation as restated in C.4 and in the textbook
  literature: x = z + e, the applied input u = v + K(x − z), e kept in a robust positively invariant set Z of
  e⁺ = (A + BK)e + w, the nominal problem solved with tightened constraints X ⊖ Z, U ⊖ KZ.
* Not read in full; cited as the origin of the "nominal plus ancillary feedback" tube.

### C.2 Chisci, Rossiter, Zappa – *Systems with persistent disturbances: predictive control with restricted constraints*, Automatica 37(7), 2001, 1019–1028 (bibliographic)

* Constraint tightening with a fixed stabilising feedback, the restricted constraints computed offline so that the
  online problem stays a nominal one (as described by secondary sources and by C.6's code documentation, which
  builds on it). Not read.

### C.3 Mayne, Kerrigan, van Wyk, Falugi – *Tube-based robust nonlinear model predictive control*, Int. J. Robust Nonlinear Control 21(11), 2011, 1341–1353 (abstract)

* Extends tube MPC to nonlinear systems with additive disturbances; the local linear controller is replaced by an
  ancillary MPC that keeps the disturbed trajectories in a tube around a nominal reference trajectory (abstract;
  conference version IFAC NOLCOS 2007). Two optimisation problems per step. Not read in full.

### C.4 Köhler, Soloperto, Müller, Allgöwer – *A computationally efficient robust model predictive control framework for uncertain nonlinear systems*, IEEE TAC 2021 (arXiv:1910.12081, extended version; full text read, ar5iv, two passes)

* **Assumptions:** a local incremental Lyapunov function V_δ(x, z, v) with c_δ,l ‖x − z‖² ≤ V_δ ≤ c_δ,u ‖x − z‖² (4a),
  ‖κ(x, z, v) − v‖² ≤ κ_max V_δ (4b) and V_δ(x⁺, z⁺, v⁺) ≤ ρ² V_δ (4c) for an incrementally stabilising feedback κ;
  the model mismatch d_w ∈ 𝒲(x, u) bounded through w̃_δ(z, v, c) (10a, b); constraints g_j Lipschitz with L_j.
* **Online problem (eq. 11):** the nominal model from the measured state (x_{0|t} = x_t), a scalar tube
  s_{k+1|t} = ρ s_{k|t} + w_{k|t}, s_{0|t} = 0, w_{k|t} ≥ w̃_δ(x_{k|t}, u_{k|t}, s_{k|t}), tightened constraints
  g_j(x_{k|t}, u_{k|t}) + c_j s_{k|t} ≤ 0 with c_j = L_j √(1/c_δ,l + κ_max), s ≤ s̄, terminal set; the applied input is
  the nominal u*_{0|t} – κ appears in the analysis only. Recursive feasibility, constraint satisfaction and
  practical stability (Theorem 1).
* **Offline:** constant P, K from LMIs (V_δ = ‖x − z‖²_P, κ = v + K(x − z); quadrotor example: ρ_c from a bisection,
  c_δ,l 0.016, c_δ,u 23.63, discrete ρ = 0.944 at h = 0.3 s, w̄ = 0.48), or control contraction metrics; c_j from a linear
  programme over the constraint polytope.
* **Results (as reported):** additive case at nominal-MPC cost, against ~500 × for an online-optimised tube; far
  less conservative than Lipschitz-based tubes (with L = 1.8, N = 10 the Lipschitz tube needs the disturbance cut by
  99 %); parametric uncertainty: ~4.5 × a nominal solve with the state-dependent bound.
* **Limitations (stated):** conservatism of the offline V_δ and of the simple stabilisability description; the nominal
  cost ignores the tube; dynamic uncertainty (e.g. an actuator lag) not covered ("still an open problem").
* **Relevance:** the scheme the racing papers C.8 / C.9 build on; it fits the existing NMPC – the tube enters as
  per-stage constraint back-offs, which `FrenetMPC(param_tightening=True)` already takes (L3, L6).

### C.5 Sasfi, Zeilinger, Köhler – *Robust adaptive MPC using control contraction metrics*, arXiv:2209.11713 (v2 Jul 2023; abstract)

* A homothetic tube from a control contraction metric for continuous-time nonlinear systems with bounded parametric
  uncertainty and additive disturbance, plus set-membership parameter adaptation; planar quadrotor. The tightening
  constants of C.8 follow it. Adaptive part: a phase D topic.

### C.6 Wischnewski, Euler, Gümüş, Lohmann – *Tube model predictive control for an autonomous race car*, Vehicle System Dynamics 60(9), 2022, 3151–3173 (online 2021; abstract + the open-source code's documentation)

* A friction-limited point-mass model with additive lateral and longitudinal disturbance; an approximate tube of
  reachable sets over the horizon tightens state and input constraints so that the problem stays a QP (OSQP);
  full-scale race car context (TUM, Roborace / Indy Autonomous Challenge). Hardware-in-the-loop with a nonlinear
  dual-track model: fewer constraint violations at comparable lap times against a nominal MPC and an infinite-horizon
  LQR (abstract). The code (github.com/TUMFTM/mod_vehicle_dynamics_control, documentation read) actuates
  accelerations through lower-level controllers and names C.1 and C.2 as its basis.
* The full text (and the TUM dissertation's chapter) was not read: no equations of the tube are taken from it.

### C.7 Wischnewski et al. – *A Tube-MPC Approach to Autonomous Multi-Vehicle Racing on High-Speed Ovals*, IEEE T-IV 8(1), 2022, 368–378 (bibliographic)

### C.8 Bongard, Krieger, Lohmann – *Dynamic Constraint Tightening for Nonlinear MPC for Autonomous Racing via Contraction Analysis*, arXiv:2602.04744 (v1 4 Feb 2026; full text read, HTML)

* **Model:** dynamic single-track model in curvilinear coordinates (s, d, Δψ, v_x, v_y, ψ̇, δ, throttle T, brake B;
  rate inputs), simplified Pacejka lateral tyres. **Uncertainty:** extra lateral tyre forces equivalent to ±2.5 % of
  the Pacejka peak factor, ±150 N lateral / longitudinal force disturbances.
* **Tube:** a state-dependent control contraction metric (no constant metric was found) from SOS programming (YALMIP,
  MOSEK) on a reduced model with linearised tyre curves; tube size σ̇ = −(β − L_E) σ + f_σ(x, u), f_σ the worst case
  over parameter / disturbance vertices in the metric; tightened constraints h_j + c_j σ ≤ 0 on the lateral deviation
  and an octagonal acceleration envelope; c_j, L_E from the metric (after C.5).
* **MPC:** 36 × 67 ms (2.4 s), RK4, acados SQP-RTI (HPIPM); solve median 2.25 ms against 1.86 ms nominal.
* **Results (simulation only, Yas Marina; plant with Pacejka B, C, D −5 % and 200 N disturbances):** no constraint
  violation with the tightening, violations and infeasibility without; in a rear-grip loss test the nominal
  controller spins, the tightened one recovers. No lap-time comparison.
* **Limitations (stated):** conservative metric, narrow uncertainty set chosen on purpose, simulation only.

### C.9 Bongard, Jank, Sagmeister, Lohmann – *Robust Nonlinear Trajectory Tracking Control for Autonomous Racing on Three-Dimensional Tracks*, arXiv:2604.19452 (v2 13 Jul 2026; full text read, HTML)

* C.8 extended to 3D tracks: the tube size σ is a state of the prediction model, σ̇ = −(β − L_E − C_σ) σ + f_σ(x, u),
  growing with v_x, v_y, yaw rate and the throttle / brake inputs; tightening of the path deviation, the acceleration
  envelope and the terminal speed bound; 36 × 60 ms, acados RTI at 100 Hz (~3.4 ms); high-fidelity double-track
  simulation of Monza at up to 255 km/h; results shown graphically. Simulation only; the tightening is 2D.

### C.10 Mi, Shao, Liu, Wang, Xu – *Integration of Motion Planning and Control for High-Performance Automated Vehicles Using Tube-Based Nonlinear MPC*, IEEE T-IV 9(2), 2024, 3859–3875 (abstract)

* Tube-based NMPC with boundary-layer sliding-mode ancillary control on a kinematic model with slip-angle terms;
  constraints tightened by tube coordinate transformation, interval analysis and robust optimisation; racing
  simulations over different road frictions (abstract).

**Tools note:** the offline designs of C.4 / C.5 / C.8 need an SDP or SOS solver (LMIs, CCM); none is installed here
(no cvxpy, MOSEK, SCS). The CX-C protocol states how its offline constants are obtained instead.

## Phase D – adaptive MPC and online identification

Checked 2026-10-22. Full texts as arXiv HTML / ar5iv or as the authors' PDFs (converted to text); the classics of
recursive identification and MHE were not obtained in full and are cited for what the full texts read restate.
What L4 / L6 / L8 already have (offline least squares and an EKF from one exploration lap, `estimation/identify.py`,
`estimation/ekf.py`) is not re-reviewed here.

### D.1 Ljung, Söderström – *Theory and Practice of Recursive Identification*, MIT Press, 1983 (bibliographic)

* The recursive prediction-error method (RPEM): a recursive Gauss–Newton step on the prediction-error criterion with
  the regressor replaced by the gradient of the prediction with respect to the parameters; for a model linear in the
  parameters it is RLS. Not read; cited as the standard form of "RLS" for a model that is nonlinear in its parameters,
  as phase D needs (the tyre model is nonlinear in B and, through saturation, in μ).

### D.2 Vahidi, Stefanopoulou, Peng – *Recursive least squares with forgetting for online estimation of vehicle mass and road grade: theory and experiments*, Vehicle System Dynamics 43(1), 2005, 31–55 (full text read, authors' PDF: sections 1 and 4.1–4.2)

* **RLS with exponential forgetting** (eq. 9–11): loss Σ λ^{k−i} (y(i) − φᵀ(i)θ̂)², gain L = Pφ(λ + φᵀPφ)⁻¹,
  P(k) = (I − Lφᵀ)P(k − 1)/λ. Without forgetting P → 0 and the estimator cannot follow a change.
* **Covariance wind-up:** under poor excitation old information is forgotten while little new arrives and P grows
  exponentially; the remedies listed (Fortescue's variable forgetting, resetting, an upper bound on P, directional
  forgetting) are called ad hoc; no tracking proofs exist for most of them.
* **Vector-type forgetting** (eq. 12): P scaled by a diagonal matrix of per-parameter forgetting factors, for parameters
  that change at different rates (a constant mass with a varying grade); the authors' own "multiple forgetting" splits
  the error by parameter (eq. 13–17). Mass within 5 % on truck data when the input is persistently exciting; lack of
  excitation in parts of the run limits accuracy (abstract, section 1).
* **Used in phase D:** RLS with forgetting only on the parameter that is expected to change (friction), the others with
  λ = 1, and P bounded above by the prior (protocol section 3.2).

### D.3 Ljung – *Asymptotic behavior of the extended Kalman filter as a parameter estimator for linear systems*, IEEE TAC 24(1), 1979, 36–50 (bibliographic)

* The EKF on the state augmented by the parameters as a recursive parameter estimator, its convergence and the
  modifications that make it consistent (as described by secondary sources). Not read; the L6 EKF (`estimation/ekf.py`)
  is this kind of estimator, used offline there.

### D.4 Rao, Rawlings, Mayne – *Constrained state estimation for nonlinear discrete-time systems: stability and moving horizon approximations*, IEEE TAC 48(2), 2003, 246–258 (bibliographic)

* Moving horizon estimation (MHE): a least-squares problem over the last N measurements with an arrival cost summarising
  the data before the window; stability conditions on the arrival cost. Not read; cited for the structure that D.5 and
  D.6 restate (window, arrival cost, constraints).

### D.5 Kühl, Diehl, Kraus, Schlöder, Bock – *A real-time algorithm for moving horizon state and parameter estimation*, Computers & Chemical Engineering 35(1), 2011, 71–83 (abstract)

* MHE for states and parameters together, with measurement noise only or measurement and state noise; the real-time
  iteration scheme (one Gauss–Newton iteration per measurement, preparation and estimation phases); an arrival-cost
  update by one QR factorisation; chemical-process examples (abstract). Not read in full.

### D.6 Zanon, Frasch, Diehl – *Nonlinear moving horizon estimation for combined state and friction coefficient estimation in autonomous driving*, ECC 2013, 4130–4135 (full text read, PDF)

* MHE over a 10 m horizon (10 shooting intervals) with a multi-body car model and a Pacejka tyre model; the friction
  coefficient is a parameter of the MHE; least-squares measurement residuals weighted by the inverse noise variances,
  an arrival cost on the initial state; RTI with direct multiple shooting, ACADO code generation; MHE and NMPC each
  below 50 ms per step in Matlab.
* Simulation: obstacle avoidance at 10 m/s, friction jumping from 0.3 to 0.5 after 80 m, initial guess 0.5. **The
  friction estimate is good while the car steers and degrades on straight stretches – with no torque and no steering,
  no slip occurs and friction is not observable**; a rigorous observability analysis is left open.
* **For phase D:** the excitation problem is real for every estimator here – friction is learned only where the tyres
  work, i.e. in the corners that need it (protocol section 3.8).

### D.7 Malvezzi, Musiu, Mascaro, Iacovacci, Bertogna – *RAGE: Radar-Aided Grip Estimator for Autonomous Race Cars*, arXiv:2604.02892 (v1 3 Apr 2026; full text read, HTML)

* MHE over a 150 ms window (a state every 10 ms), estimating body velocities, IMU biases and the Pacejka coefficients
  (B, C, D, E, S_h, S_v per axle) together; priors on the first state and the parameters act as the arrival cost; box
  bounds on the parameters; Ceres solver, at most 3 iterations, 2.3 ms mean on one core.
* Simulation hot lap with randomised initial Pacejka parameters: the parameters converge in about 20 s, lateral-force
  RMSE below 500 N; on the real EAV-24 car (Yas Marina, up to 70 m/s) cornering stiffness varies with tyre temperature
  and pressure. No comparison with other estimators; observability of the friction peak not analysed.

### D.8 Multiple-model adaptive estimation – Magill, *Optimal adaptive estimation of sampled stochastic processes*, IEEE TAC 10, 1965, 434–439; Lainiotis, *Optimal adaptive estimation: structure and parameter adaption*, IEEE TAC 16, 1971, 160–170 (bibliographic)

* A bank of filters, one per candidate parameter value, run in parallel; their residuals give posterior probabilities
  that weight the estimates; if a candidate equals the true parameter, the scheme identifies it (as described by Athans
  and Chang's 1976 Lincoln Laboratory note and Aguiar, Athans, Pascoal, found by search). Not read; the racing variant
  used in phase D is D.9's.

### D.9 AL-Sunni, Almubarak, Horng, Dolan – *LLA-MPC: Fast Adaptive Control for Autonomous Racing*, arXiv:2505.19512 (v1 May 2025; full text read, HTML v1 – upgrades entry A.6)

* **Model bank:** N = 20 000 (also 10 000) dynamic bicycle models, the 8 parameters (Pacejka B, C, D per axle, rolling
  and drag coefficients) drawn uniformly within assumed bounds; mass, inertia and geometry known.
* **Look-back selection:** at every step each model predicts the current state from the previous state and input; the
  squared Euclidean errors are summed over a window W = 0.2 s (10 steps of 20 ms); the model with the smallest sum is
  used (hard switching; nominal model until the window is full).
* **Friction and plan:** μ̄ = (D_f + D_r)/(2mg) from the selected model, exponentially smoothed (γ not reported), selects
  or interpolates a precomputed library of velocity profiles (one per friction value) on a minimum-curvature line.
  NMPC horizon 0.4 s at 20 ms; cost, constraints and solver not given.
* 1:43 simulator: friction decaying 2 %/s, dropping 40 % after lap 1 or within lap 1; against APACRace (D.10): shorter
  track-violation times (e.g. 0.44 vs 3.10 s) and ~0.03 vs ~0.06 s per iteration; 10 runs; simulation only.
* **Used in phase D** as `adaptive_bank` (protocol section 3.5), with this paper's window, bank size and hard switching.

### D.10 Kalaria, Lin, Dolan – *Adaptive Planning and Control with Time-Varying Tire Models for Autonomous Racing Using Extreme Learning Machine* (APACRace), arXiv:2303.08235 (Mar 2023), ICRA 2024, 10443–10449 per the search result (full text read, ar5iv HTML)

* Nominal kinematic model plus a learned residual: one extreme learning machine per axle maps slip angle to lateral
  force (40 random hidden units, output weights trained), with rolling and drag terms; SGD with momentum on the
  residuals of (v_x, v_y, ω) over the most recent K samples (300 / 2 000) – the sliding window is the forgetting; the
  nominal model alone for the first 6.2 s (44 s in CARLA).
* **Friction from the learned tyre curves selects / interpolates precomputed velocity profiles** (a library over
  friction on a fixed minimum-curvature line); NMPC with horizon 50, the corrected model inside; ~0.05 s per step.
* 1:43 simulator with tyre wear (peak-friction parameters changed) and a sudden "rain" drop: adaptation with variable
  speeds 0.46 s of track violations against 4.32 s without adaptation and 2.84 s with adaptation but constant speeds;
  the oracle 0. **The speed adaptation matters as much as the model adaptation.** Stated limits: no uncertainty
  model, no deliberate excitation, simulation only.

### D.11 Dikici, Ghignone, Hu, Baumann, Xie, Carron, Magno, Corno – *Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute*, arXiv:2411.17508 (Nov 2024; IEEE, vol. 10, no. 2, Feb 2025, 1984–1991 per the ETH Research Collection – most likely RA-L, not confirmed; full text read, HTML v1)

* ~30 s of on-track data (pure-pursuit driving), a small network correcting a nominal model, then Pacejka parameters
  fitted to virtual steady-state data generated with the corrected model, iterated (~6 iterations of 0.5 s); against
  nonlinear least squares on the one-step errors: 3.3× lower RMSE under noise in simulation; on a 1:10 car the NLS
  model did not complete a lap. Batch, not streaming; reliable only within the slip range the data cover.
* **For phase D:** a reminder that plain one-step least squares can underestimate tyre forces under noise (as L6
  found for L4's least squares); the primary sets of phase D are noise-free, the noisy condition is secondary.

### D.12 Adetola, DeHaan, Guay – *Adaptive model predictive control for constrained nonlinear systems*, Systems & Control Letters 58(5), 2009, 320–326 (abstract)

* Parameter adaptation combined with robust MPC (min-max or Lipschitz-based) so that the parameter-estimation error
  during the transient is accounted for, rather than relying on the inherent robustness of a certainty-equivalence
  NMPC; CSTR simulation (abstract). Not read in full. Phase D's controllers are certainty-equivalence (no such
  accounting) – stated in the protocol.

### D.13 Köhler, Kötting, Soloperto, Allgöwer, Müller – *A robust adaptive model predictive control framework for nonlinear uncertain systems*, Int. J. Robust Nonlinear Control (journal version 2020/21, volume not checked; arXiv:1911.02899; full text read in part, ar5iv: sections I–III; the numerical example not read)

* Parameters entering affinely, f(x, u) + G(x, u)θ, plus a bounded additive disturbance; set-membership estimation over
  a moving window of M measurements (2p linear programs per step) keeps a non-increasing hypercube containing the true
  parameter; an LMS point estimate in the cost; the tube of C.4 (incremental Lyapunov function, s⁺ = ρs + w̄) shrinks as
  the parameter set shrinks; recursive feasibility and constraint satisfaction under the model updates.
* Offline: an incremental Lyapunov function from LMIs (quasi-LPV) and Lipschitz-type constants. **Not used in phase D**
  (the parameters do not enter affinely – B inside the tyre curve – and no SDP solver is installed, "Tools note"
  above); the combination of phase C's tube with phase D's estimator is a follow-up candidate.

### D.14 Gustafsson – *Slip-based tire-road friction estimation*, Automatica 33(6), 1997, 1087–1099 (bibliographic + search-result summary)

* Friction from wheel slip with an adaptive estimator for a model linear in its parameters, designed for periods of
  poor excitation (per the search results). Not read; listed with D.6 for the excitation issue. Phase D's car model has
  no wheel-slip states, so slip-based estimation does not apply.

## Phase E – learning MPC (LMPC, lap-to-lap)

Checked 2026-10-22. Full texts as ar5iv / arXiv HTML; journal versions not obtained (their venues are noted as found, not
checked).

### E.1 Rosolia, Borrelli – *Learning Model Predictive Control for Iterative Tasks. A Data-Driven Control Framework*, arXiv:1609.01387 (v1 Sep 2016 … v7 Dec 2017; journal version IEEE TAC 2018 per secondary sources; full text read in part, ar5iv: the formulation, the theorems and the examples)

* An iterative task: every iteration starts at the same x_S and must reach the equilibrium x_F; the **sampled safe set**
  SS^j is the union of the stored closed-loop trajectories that reached x_F, the **Q-function** Q^j(x) the lowest stored
  cost-to-go at x (∞ outside SS^j).
* LMPC: the finite-horizon cost plus Q^{j−1}(x_N) as terminal cost, with x_N ∈ SS^{j−1} as terminal constraint.
  **Recursive feasibility, asymptotic stability of x_F and a non-increasing iteration cost** (given a feasible first
  trajectory); at a steady state, optimality for the finite-horizon problem with the learned terminal cost (local only
  for non-convex problems). **Assumption: the model equals the plant** (deterministic, exact).
* The discrete safe set makes the problem mixed-integer; examples: constrained LQR (cost 57.2 → 49.9), a minimum-time
  Dubins car (39 → 16 steps).

### E.2 Rosolia, Borrelli – *Learning How to Autonomously Race a Car: a Predictive Control Approach*, arXiv:1901.08184 (v4 Oct 2019; journal version in IEEE TCST per secondary sources, not checked; full text read, ar5iv)

* Racing as a minimum-time iterative task, one lap per iteration; stage cost 1 per step until s ≥ L; the cost-to-go of a
  stored state is the steps to that lap's finish.
* **Local safe set:** at a query state, the K = 20 nearest stored states by the arclength s (weight on s only) from the
  last two laps; the terminal state is a **convex combination** of them (λ ≥ 0, 1ᵀλ = 1 – the convex-hull relaxation of
  the mixed-integer set) and the terminal cost the same combination of their costs-to-go (an approximation, not a bound).
  The candidate terminal state is propagated from the previous solution's λ.
* **Model:** affine time-varying, linearised along the shifted candidate trajectory; the kinematic Frenet equations known,
  the velocity states (v_x, v_y, ω) from **local linear regression** (Epanechnikov kernel, bandwidth 10, 80 nearest
  neighbours). A QP (OSQP), N = 12, 10 Hz, ~10 ms.
* BARC 1:10 car: two path-following laps at ~1.2 m/s to start; the lap time converges after ~29 laps, lateral acceleration
  near 1 g. **Lost guarantees** stated: feasibility only if the system is linear or the linearisation accurate over the
  local set; no convergence proof for the relaxed, learned scheme.

### E.3 Rosolia, Borrelli – *Minimum Time Learning Model Predictive Control*, arXiv:1911.09239 (v4 Oct 2020; abstract)

* A time-varying safe set and terminal cost from closed-loop data for minimum-time problems, convexified; recursive
  constraint satisfaction, non-decreasing performance and finite-time convergence for a class of nonlinear systems with
  convex constraints; obstacle-avoidance and racing examples (abstract). Not read in full.

### E.4 Xue, Zhu, Dolan, Borrelli – *Learning Model Predictive Control with Error Dynamics Regression for Autonomous Racing*, ICRA 2024 (arXiv:2309.10716, v2 Mar 2024; full text read, HTML v2)

* The model is a **nominal physics model plus a local, linear regression of its error** in the velocity states:
  x⁺ = f(x, u) + e(x, u), the residual linearised about the candidate (A^e, B^e, C^e per velocity channel, one input
  each), from the M nearest samples of the previous laps, Epanechnikov kernel of bandwidth h, ridge-regularised weighted
  least squares; A = A^f + A^e etc. **Without nearby data the correction shrinks to zero and the nominal model remains**
  (the authors' argument against regressing the full dynamics, E.2).
* The LMPC of E.2 otherwise: K nearest states from each of the previous P laps, convex hull, convex combination of the
  costs-to-go; lap-time, input and input-rate costs; a convex program re-solved each step.
* Simulation with a kinematic nominal and a Pacejka plant (friction 1.2 vs 0.9): stable for small bandwidths and small
  rate costs where E.2's regression fails; BARC: 10 trials, failures 1 of 10 against 5 of 10 at h = 3; a full-scale IAC
  car (10 iterations, qualitative). **Limits:** no stability guarantee for an arbitrary nominal mismatch, the convex hull
  gives up persistent feasibility, hyperparameters partly unreported.

### E.5 Rosolia, Carvalho, Borrelli – *Autonomous Racing using Learning Model Predictive Control*, ACC 2017 (arXiv:1610.06534; bibliographic)

* The first racing LMPC (CarSim simulation), before the local safe set of E.2. Not read.

### E.6 Brunner, Rosolia, Gonzales, Borrelli – *Repetitive learning model predictive control: An autonomous racing example*, IEEE CDC 2017 (bibliographic)

* LMPC for repetitive (periodic) tasks such as consecutive laps. Not read.

### E.7 Rosolia, Zhang, Borrelli – *Robust Learning Model-Predictive Control for Linear Systems Performing Iterative Tasks*, IEEE TAC 2022 (bibliographic)

* LMPC for uncertain linear systems. Not read; phase C's tube around the LMPC would be its nonlinear counterpart.

### E.8 Kapania, Gerdes – *Path tracking of highly dynamic autonomous vehicle trajectories via iterative learning control*, ACC 2015 (arXiv:1902.00611; abstract)

* Iterative learning control (PD and quadratically optimal) of the steering feedforward from lap to lap on an Audi TTS at
  up to 8 m/s²; corrects repeatable tracking errors when the tyre limits are not exceeded. A lap-to-lap learning method
  that keeps the plan fixed – the counterpart of LMPC, which changes the trajectory. Not read in full.

### E.9 Zhao, Schlüter, Brunke, Schoellig – *Improving Drone Racing Performance Through Iterative Learning MPC*, IROS 2025 (arXiv:2508.01103; abstract)

* LMPC for drone racing with an adaptive cost (time-optimal vs centre line), a shifted local safe set against excessive
  shortcutting and Cartesian safety constraints; lap time improved by up to 60.85 % from weakly tuned starting
  controllers, 6.05 % on a real drone from MPCC++ (abstract). The most recent LMPC racing result found.

**For phase E:** the racing LMPC needs (a) a first, completed lap of the car (E.1's feasible first trajectory; E.2: slow
path following), (b) data beyond the finish line (E.2: consecutive laps), (c) a model whose error the local regression or
the nominal model keeps small (E.2, E.4) – the guarantees of E.1 do not survive the relaxations (E.2, E.4 state this).

## Phase F – safety-critical control (safety filters around a proposing controller)

Checked 2026-10-22. Full texts as ar5iv / arXiv HTML where stated; journal versions not obtained (their venues are noted as
found, not checked). L7's literature check (docs/L7.md, 2026-10-17 and the addendum) already covers the RL-specific
filters (Kalaria et al.'s DOB-CBF, Oh et al., Wu et al., Zongo and Wei, the conformal filters) – not repeated here; F.5
upgrades its abstract-only entry of Tearle et al.

### F.1 Bastani – *Safe Reinforcement Learning with Nonlinear Dynamics via Model Predictive Shielding*, arXiv:1905.10691 (v1 May 2019, first titled *Safe reinforcement learning via online shielding*; the published venue not checked; full text read, ar5iv)

* **Model predictive shielding (MPS):** at every step, simulate one step of the learned policy, x′ = f(x, π̂(x)), and test
  whether x′ is **recoverable** – whether the backup policy, simulated from x′ for at most N steps, reaches an invariant set
  X_inv while every state on the way is safe. If it is, apply π̂(x); otherwise apply the backup's action. Cost: O(N)
  simulation per step, nothing offline except the backup and its invariant set.
* **Guarantee (Theorem III.1):** from X₀ ⊆ X_inv every closed-loop trajectory is safe, by induction over recoverable
  states – **for a known, deterministic model** (stated as the main assumption; unknown dynamics left to future work).
* **Backup:** LQR around safe equilibria with SOS-certified invariant regions (polynomial dynamics; Taylor approximations
  otherwise void the formal guarantee), plus a recovery policy trained to reach them.
* **Experiments:** cart-pole only (100 rollouts). The shielded policies keep 100 % safety where the unshielded one drops to
  near zero on longer episodes; the shield **costs reward** (reported in figures only); the frequency of backup use is
  **not reported**.

### F.2 Banerjee, Rahmani, Biswas, Dillig – *Dynamic Model Predictive Shielding for Provably Safe Reinforcement Learning*, arXiv:2405.13863 (v2 Dec 2024; the venue is not stated on the page; full text read in part, HTML v2 up to the experiments)

* MPS's backup is conservative and task-oblivious; DMPS replaces it, when the proposal is not recoverable, by a **local
  planner** (an anytime MCTS, horizon 5) that searches action sequences keeping every state recoverable and scores them
  with the policy's Q-function; the fixed backup remains the fallback. Same assumption: deterministic, perfectly known
  dynamics.
* Thirteen navigation benchmarks: on the dynamic ones ~25 shield invocations per episode against ~124 for MPS and higher
  returns; planning ~0.4 s per triggered step (unoptimised Python). **Relevance:** the intervention count of a shield
  depends on how good its fallback is – the measure phase F reports.

### F.3 Agrawal, Chen, Panagou – *gatekeeper: Online Safety Verification and Control for Nonlinear Systems in Dynamic Environments*, arXiv:2211.14361 (v4 Aug 2024; full text read in part, HTML: up to the benchmarks; the conclusion not read)

* A nominal planner proposes a trajectory; candidates follow it up to a switch time T_S and then a **backup controller**
  that reaches a **backup set** C and stays there; a candidate is valid if it stays in the (shrunk) safe set until it
  reaches C. The valid candidate with the largest T_S is committed; if none is valid the previous committed trajectory is
  kept. **Safety for all time** follows from the backup set's invariance; with bounded disturbances and an ISS tracking
  controller the safe set is shrunk by the tracking-error bound R (and C kept R + r from the unsafe set).
* Median ~3.4 ms per iteration (grid of 10 switch times) against 27–78 ms for MPC-based alternatives; quadrotor
  simulations: goal reached in 13 of 15 worlds against 10 of 15 for an MPC safety filter. **Switching, not blending**, is
  why it follows the nominal plan more closely than a backup-blending filter (10 vs 15 m/s target speed in their firewatch
  case). Stated limitation: **a backup controller and set must exist**; robustness margins make it conservative.

### F.4 Wabersich, Zeilinger – *A predictive safety filter for learning-based control of constrained nonlinear dynamical systems*, arXiv:1812.05506 (Automatica 129, 2021, per the search result; full text read in part, ar5iv: the formulation, the theorem, the examples; the appendix not read)

* The **predictive safety filter (PSF):** min ‖u_L − v₀‖ over a backup plan of N steps of a nominal model, with
  constraints tightened along the horizon (ε_i = ε(1 − √ρ^i)/(1 − √ρ), from a contraction rate ρ), a model-confidence
  constraint and a terminal safe set with a known local controller. Infeasible → the tail of the previous plan, horizon
  shrinking to the terminal controller.
* Guarantee: **safety in probability** (the confidence p_S of a Bayesian model), given initial feasibility and small
  enough tightening constants. IPOPT / CasADi; pendulum swing-up, quadrotor landing. L7's main filter is this structure
  (minimal deviation over an NMPC plan, back-offs from estimate (b)), without the terminal set and with the shifted-plan
  fallback.

### F.5 Tearle, Wabersich, Carron, Zeilinger – *A predictive safety filter for learning-based racing control*, arXiv:2102.11907 (2021; the published venue not checked; full text read, ar5iv – upgrades L7's abstract-only entry)

* Dynamic bicycle model (Pacejka-type tyres, forward Euler), N = 60 at 80 Hz, acados RTI; objective: the deviation of the
  first input from the desired one plus an input-rate penalty. **Terminal set: ellipsoids around steady-state cornering
  equilibria** with a linear feedback designed by an SDP over 21 curvatures, invariance checked numerically (not a
  stopping set). Track and terminal constraints **soft**; no tightening for model error; no fallback for a failed solve
  described.
* 1:28 cars, manual driving and a DAgger policy (expert: MPCC). **No completion counts, intervention rates, violation
  counts or solve times are reported** – the evidence is trajectory heat maps. Stated limitations: invariance only on
  constant-curvature segments; a conservative linearised design; no robustness analysis.

### F.6 Chen, Jankovic, Santillo, Ames – *Backup Control Barrier Functions: Formulation and Comparative Study*, arXiv:2104.11332 (v1 Apr 2021, the only version; a Caltech record dated December 2021, the venue not checked; full text read, ar5iv)

* From a small invariant set S₀ and a backup policy π keeping it invariant, the implicit set S = {x: Φ_π(x, τ) ∈ C for
  τ ∈ [0, T], Φ_π(x, T) ∈ S₀} is **control invariant** (Theorem 1); its barrier h(x) = min(min_τ h_C(Φ(x, τ)), h_S(Φ(x, T)))
  is used in a CBF-QP, min ‖u − u⁰‖, with one constraint per sampled τ, the gradients from the **sensitivity of the flow**
  (variational equation Q̇ = (∂f_π/∂x) Q). The QP is feasible wherever h ≥ 0 (the backup input is a solution); the
  composed barrier has **relative degree 1** under mild conditions (Theorem 3) – the reason it sidesteps high-order CBFs.
* Lane keeping (Dubins car), aircraft avoidance, a 16-state quadrotor: the implicit set is close to the HJ result with a good
  backup; online cost 1.4 ms integration + 3.6 ms QP (Dubins), 4.9 + 28.3 ms (quadrotor). Limitations stated: the set
  depends on the backup; many QP constraints; **"not good at handling malicious disturbance"**; a known model.

### F.7 Gurriet, Mote, Ames, Feron – *An Online Approach to Active Set Invariance*, IEEE CDC 2018 (abstract, via the search result and the Caltech record)

* The origin of F.6: safety by staying within reach of a backup set, using online sensitivity analysis around the backup
  trajectory instead of an offline viable set; a linear inverted pendulum. Not read in full.

### F.8 Xiao, Belta – *Control Barrier Functions for Systems with High Relative Degree*, arXiv:1903.04706 (CDC 2019; the journal version *High-Order Control Barrier Functions*, IEEE TAC 67(7), 2022, not read; full text read, ar5iv)

* For a constraint b of relative degree m: ψ₀ = b, ψ_i = ψ̇_{i−1} + α_i(ψ_{i−1}); if ψ_m ≥ 0 is enforced (the HOCBF
  condition, input through L_g L_f^{m−1} b), the intersection of the sets {ψ_{i−1} ≥ 0} is forward invariant. Exponential
  CBFs are the linear-α special case. Assumptions: **control-affine dynamics**, a uniform relative degree; QP with a CLF.
* Adaptive cruise control (relative degree 2). Stated limitations: **conflicts with input bounds** can make the QP
  infeasible (penalty factors on α mitigate, not remove); the initial state must satisfy all ψ_i ≥ 0. L7's CBF-QP is of
  this kind: a second-order exponential barrier on the track edges for a point mass (relative degree 2 in the
  accelerations), with a robust margin b_10 and friction factor η_10 – it completed 0 of 105 laps for every proposer.

### F.9 Jankovic – *Robust control barrier functions for constrained stabilization of nonlinear systems*, Automatica 96, 2018, 359–367 (bibliographic and abstract, via the search result)

* Robust CBFs with bounded disturbances, combined with ISS-CLFs; removes the uniform relative-degree-1 assumption. Not read
  in full; listed as the robust-CBF reference of the roadmap.

### F.10 Hsu, Hu, Fisac – *The Safety Filter: A Unified View of Safety-Critical Control in Autonomous Systems*, arXiv:2309.05837 (Annual Review of Control, Robotics, and Autonomous Systems, per the arXiv comment; full text read in part, ar5iv: up to the open problems)

* Every filter is a **safety monitor** (a value function or a rollout), a **fallback policy** and an **intervention
  scheme** (switch to the fallback, or optimise the command close to the task's). The fallback never has to be used – the
  filter only keeps the option open (their Theorem 1). Least-restrictive = override only what would leave the maximal safe
  set.
* Classes: HJ reachability (maximal set; exponential in the state dimension, ~6 states in practice), CBFs (smooth, smaller
  sets, hard to find), **rollout-based** filters (MPS: binary switch, conservative, quality set by the fallback; tube /
  reachable-set MPC), learned critics (scalable, weak guarantees).
* **Switching filters chatter** near the boundary; **the fallback's quality sets the conservatism** ("freezing robot");
  probabilistic or robust guarantees rest on modelling assumptions; filter-aware task policies reduce unneeded
  interventions.

### F.11 Gacsi, Kiss, Molnar – *Braking within Barriers: Constructive Safety-Critical Control for Input-Constrained Vehicles via the Backup Set Method*, arXiv:2510.15797 (Oct 2025, submitted to IEEE T-ASE; abstract)

* Backup CBFs (F.6) for a braking vehicle on asymmetric surfaces, with friction-limited inputs; backup set and controller
  pairs built from feedback linearisation and Lyapunov equations; a four-wheel model in simulation (abstract). The search
  summary reports that the method needs an operating point where the inputs are not saturated – **not verified** (the
  full text was not read). The closest vehicle-dynamics backup-set work found.

### F.12 Other recent items (abstracts or search results only)

* Zhou, Zarrouki, Piccinini, Hu, Xie, Betz – *Safe Reinforcement Learning with a Predictive Safety Filter for Motion
  Planning and Control: A Drifting Vehicle Example*, arXiv:2506.22894 (Jun 2025; abstract): RL chooses drift states, a
  PSF modifies its actions (MATLAB–CarSim); no completion statistics in the abstract.
* Cao, Joa, Borrelli – *A Simple Approach to Constraint-Aware Imitation Learning with Application to Autonomous Racing*,
  IROS 2025, 9830–9837 (arXiv:2503.07737; abstract): constraints into the imitation objective instead of a filter.
* Chen, Vörös, Molnar, Orosz – a lag-compensating CBF for steer-by-wire lag in lane keeping, compared with high-order and
  backstepping CBFs (IEEE IV 2026 per the authors' page; **search result only, not read**) – the actuator-lag issue of L6
  in CBF form.

**For phase F:** (a) safety filters split into **minimal-modification optimisers** (F.4, F.5, F.8: PSF / NMPC filter,
CBF-QP – the command is changed whenever the model cannot certify it) and **monitor-and-switch shields** (F.1–F.3: the
proposal passes as long as a backup can still recover from where it leads). L7 tested only the first kind and measured
88–99 % false interventions and solver-fallback runs before 57 % of the filtered failures. (b) Every guarantee assumes an
exact (F.1, F.2, F.6) or bounded-error / probabilistic model (F.3, F.4); on these cars (L2 mismatch, lag, L3
disturbances) none holds, so phase F measures, it does not prove. (c) The fallback's quality sets a shield's conservatism
(F.1, F.2, F.6, F.10) and switching can chatter (F.10). (d) CBFs need control-affine dynamics and a known relative degree
(F.8); L7's point-mass CBF-QP was the F.8 / F.9 kind and failed at turn 1; for the single-track model the steering rate
reaches the lateral position with relative degree 3 through saturating, non-affine tyres – F.6's backup CBF (relative
degree 1 by construction) is the smooth counterpart of a shield. (e) No racing work was found that compares a shield with
a minimal-modification filter around learned policies, on unselected seeds, with completion and false-intervention counts.

## Phase G – real-time control: delay compensation, the real-time iteration and its relatives

Checked 2026-10-23. Full texts as PDF (text extracted with pdftotext) or ar5iv / arXiv HTML where stated; journal
versions not obtained (their venues are noted as found, not checked). L9's literature check (docs/L9_protocol.md,
2026-10-18: the RTI, Verschueren et al.'s racing RTI, acados, neural approximations) is not repeated here.

### G.1 Diehl, Bock, Schlöder et al. – the real-time iteration (Diehl et al., J. Process Control 12, 2002; Diehl, Bock, Schlöder, SIAM J. Control Optim. 43, 2005) (bibliographic, as cited in G.2)

* One full Gauss–Newton SQP step per sampling time, with the initial value embedded in the QP (the "tangential
  predictor"); nominal stability of the scheme is shown in these works (per G.2, not read).

### G.2 Gros, Zanon, Quirynen, Bemporad, Diehl – *From linear to nonlinear MPC: bridging the gap via the real-time iteration*, Int. J. Control 93(1), 2020, 62–80 (full text read in part, the SYSCOP PDF: sections 3–4)

* **Two timelines** (their Fig. 3):
  * A converged SQP can start from a model-based *prediction* of the state at the time the solution will be applied.
    It still rests on outdated information, because measurements that arrive during the iterations are ignored.
  * The RTI instead takes one Newton step per sampling time, always with the latest state estimate – "an approximate
    solution computed using the most up-to-date information".
* **Preparation and feedback phases:** the linearisation and condensing are done before the measurement arrives; after
  it, only the QP is solved. Their remark: **the delay introduced by the feedback time can be accommodated "as in
  linear MPC, by including a corresponding prediction in the state estimate".**
* The advanced-step controller (G.3) is described there: it solves the NLP to convergence, starting from a prediction
  of the future initial state, and corrects the solution with a tangential predictor when the actual state is known.

### G.3 Zavala, Biegler – *The advanced-step NMPC controller: optimality, stability and robustness*, Automatica 45(1), 2009, 86–93 (abstract, Optimization Online; mechanism as described in G.2)

* The NLP is solved in the background for the predicted next state. When the measurement arrives, the solution is
  updated by an NLP sensitivity step.
* The controller has the nominal stability of NMPC without computational delay, and ISS robustness (abstract).
* The converged-NLP counterpart of the RTI's split. Not read in full.

### G.4 Findeisen, Allgöwer – *Computational delay in nonlinear model predictive control*, IFAC ADCHEM 2003 (proceedings 2004, 427–432) (full text read in part, the PDF: sections 1–3)

* **Computation time neglected** in NMPC schemes becomes a delay between the state information and the input, which
  "can lead to a drastic performance decrease or even to instability".
* **Their approach:**
  * keep applying the input of the last recalculation during the (maximum) computation time δ_c;
  * either constrain the first δ_c of the new open-loop input to it, or **use that input to predict x(t_i + δ_c) and
    solve the problem from the predicted state**;
  * with a known maximum computation time and a horizon long enough, nominal stability follows (CSTR example).
* **Assumptions:** the model is exact (nominal), and the computation time has a known bound.

### G.5 Liniger, Domahidi, Morari – *Optimization-based autonomous racing of 1:43 scale RC cars*, arXiv:1711.07300 (the published venue not checked; full text read in part, ar5iv)

* The system delay of one sampling period (both controllers send the new input at the end of the 20 ms period) is
  compensated by integrating the nonlinear model forward over the delay (second-order Runge–Kutta) from the Kalman
  filter's state estimate.
* 50 Hz. **No comparison with and without the compensation is reported.**

### G.6 Kabzan et al. – *AMZ Driverless: The Full Autonomous Racing System*, arXiv:1905.05150 (v1 May 2019; full text read in part, the arXiv PDF: the MPC section and the lessons learned)

* MPCC after Liniger et al. (2015): ForcesPro NLP, 50 ms, N = 40 (2 s look-ahead). The SLAM's observation delay is
  compensated with the integrated odometry.
* **The total delay from a cone detection to the command reaching the car is about 300 ms.** At the handling limits
  this "can lead to severe instabilities" (e.g. a missed braking point). The workaround was to limit the top speed and
  the wheel torques. Reducing and handling delays consistently is named as the next season's focus.

### G.7 Kalaria, Lin, Dolan – *Delay-aware Robust Control for Safe Autonomous Driving and Racing*, arXiv:2208.13856 (extends their IEEE IV 2022 paper; full text read, ar5iv)

* **Method:**
  * the computation delay is estimated online with an adaptive Kalman filter (the bound is mean + 2 σ);
  * the initial state is shifted forward by the estimated delay (plus a fixed action delay) with the buffered nominal
    commands;
  * a first-order steering lag is modelled;
  * a tube MPC (LTV model, tightened constraints, CBFs) is used.
* **Results:** CARLA racing, single runs. With compensation, lap 35.34 s and no lane violation; without, 35.46 s and
  one violation (a crash). With a learned controller under a safety QP: 36.51 s / 0 against 38.04 s / 2.
* **Limits stated:** simulation only; no repeated runs; the delay values are not given in numbers.

### G.8 Dirckx, Bos, Vandewal, Vanroye, Decré, Swevers – *ASAP-MPC: An Asynchronous Update Scheme for Online Motion Planning with Nonlinear MPC*, arXiv:2402.06263 (v2 Mar 2024; abstract)

* An asynchronous update for varying, possibly large solve times: the successive NMPC solutions are merged into one
  continuous trajectory, which a linear state feedback tracks. Quadcopter (hardware in the loop) and a truck-trailer.
* It cites Findeisen–Allgöwer (G.4) and the advanced-step controller (G.3) as the two main strategies for
  computational delay (per the search result). Not read in full.

### G.9 Sambharya, Hall, Amos, Stellato – *End-to-End Learning to Warm-Start for Real-Time Quadratic Optimization*, arXiv:2212.08260 (Dec 2022; the published venue not checked; abstract)

* A network maps the QP parameters to a warm start for a fixed number of Douglas–Rachford iterations, trained through
  the iterations, with Rademacher-complexity generalisation bounds; fewer iterations in three real-time applications.
* The roadmap's "learned warm starts": aimed at iterative solvers with many iterations (L9's IPOPT); the RTI does one
  QP per period and needs no warm-start model.

**For phase G:**
* (a) The standard remedy for a known delay between measurement and actuation is a **model-based prediction of the
  state at the time the command takes effect**, using the inputs already decided (G.2, G.4–G.7). It is used for the
  RTI as for linear MPC (G.2).
* (b) **Its effect is rarely measured:** G.5 reports none; G.7 reports single simulated runs; G.6 names the delay as
  the bottleneck and works around it with lower speeds.
* (c) The guarantees assume an exact model and a known delay bound (G.4). On these cars the controller's model is the
  identified one and the delay is given, so the prediction carries the model error over the delay window. A
  mis-specified delay is the obvious robustness question.
* (d) The advanced-step controller (G.3) and learned warm starts (G.9) target converged or iterative solvers. Under
  L9's rule the converged IPOPT completes nothing, while the RTI already runs within the budget (2.4 ms feedback). So
  the open question L9 left is the delay, not the solver: a 20 ms sensor delay took the RTI `lag_identified` from 79 to
  0 of 105 (docs/L9.md).

## Phase H (frontier 1) – the ATV-QP learning MPC (E.2 / E.4's convex formulation)

Checked 2026-10-23. Phase E's entries (E.1–E.9) are the basis and are not repeated. Here: how the published racing
LMPCs are actually implemented – the two open-source codes were read, not copied (one carries an education / research
licence with attribution, the other LGPL-2.1) – and one recent item.

### H.1 Rosolia – *RacingLMPC* (github.com/urosolia/RacingLMPC, master; Python, OSQP / cvxopt; the code read: `initControllerParameters.py`, `main.py`, `PredictiveControllers.py`, `PredictiveModel.py` – its header permits education and research use with attribution to UC Berkeley and the papers E.1 / E.2)

* **The LMPC is a QP** solved each step (OSQP), on an affine time-varying model linearised along a candidate.
  * The candidate is the previous predicted trajectory shifted by one step. Its terminal point is
    **z_t = Σ λ_i succ(x_i)**, the successors (the next stored states) of the selected points weighted with the previous
    solution's λ, and its last input is formed the same way. At the first step of a lap, the candidate is the first
    N + 1 states of the stored lap.
* **The velocity states come from local linear regression.** v_x, v_y, ω are regressed on the stored data (7 points
  per lap, bandwidth 5); the kinematic Frenet states are linearised analytically. There is no physics nominal model –
  E.2's form.
* **The local safe set:**
  * from each of the 4 fastest stored laps, a contiguous window of 12 + 1 points around the point nearest to z_t (L1
    norm on the full state), 48 points in all;
  * terminal state = SS λ + slack, **soft**, with 500 per unit and per state;
  * λ ≥ 0, Σ λ = 1;
  * terminal cost = the stored costs-to-go weighted with λ, plus the time already spent in the current lap once the
    prediction crosses the line.
* **Other weights:** lane slack linear 25 and quadratic 5; input-rate weights 5 × (1, 10); no state cost; N = 14 at
  dt = 0.1 s. Before the LMPC laps: one PID lap, then MPC laps with an LTI and an LTV model regressed from it.

### H.2 Xue, Zhu et al. – *Racing-LMPC-ROS2* (github.com/MPC-Berkeley/Racing-LMPC-ROS2, branch humble-release; C++ / CasADi Opti; LGPL-2.1; E.4's code per the paper; the code read: `racing_mpc.cpp` and the parameter files)

* **Two modes in one code.**
  * `full_dynamics`: the nonlinear discrete dynamics as constraints, solved with **IPOPT** (`max_iter` 200, `tol`
    1e-3, `max_cpu_time` 0.085 s).
  * Otherwise: the dynamics **linearised** about a reference trajectory (the discrete dynamics' Jacobian), solved with
    **OSQP** (polish on).
  * The parameter files hold IPOPT settings, but **which mode the paper's experiments used is not stated**.
* **The convex hull is soft:** x_N = SS λ + slack, with a quadratic slack penalty per state (BARC: 40, 40, 4, 40, 40,
  4; IAC car: 200, 20, 2, 200, 2, 20). There are 96 points – 32 per lap from up to 3 stored laps. The terminal cost
  is the stored costs-to-go, offset by the first selected point's.
* **Phase E's `lmpc`** – an IPOPT-solved NLP with a soft convex hull – **corresponds to this code's full-dynamics
  mode.** The ATV-QP of H.1 / E.2 corresponds to its linearised mode.

### H.3 Zeng, Li, He, Sreenath, Zeng – *IteraOptiRacing*, arXiv:2507.09714 (Jul 2025; abstract)

* An iterative-task racing planner and controller built on i2LQR: it reuses the car's past laps, handles moving
  opponents, and has a constant computational load. Simulation only; no numbers are given in the abstract. Not an LMPC
  in E.2's sense; listed as the most recent iterative-task racing item found.

**For phase H:**
* (a) Both published codes keep the terminal convex hull **soft** and solve the racing LMPC as a **QP on a model
  linearised along the shifted candidate** (H.1), or in a mode that does so (H.2). H.1 builds its candidate's terminal
  point from the previous λ's successors (E.2).
* (b) Phase E solved the NLP (H.2's full-dynamics mode) with a candidate terminal arclength from the predicted end
  state, and failed its gate.
* (c) Phase E's hypothesis (e1) – that the convex hull is a feasible terminal set for the linearised model but need not
  be for the nonlinear one – is tested by changing exactly those two things (the linearisation, solved as one QP per
  step, and E.2's candidate) and keeping the rest of E's design.

## Phase I (frontier 2) – the learning MPC's follow-ups: a progress bound per lap first

Checked 2026-10-28. Phase E's and H's entries (E.1–E.9, H.1–H.3) are the basis and are not repeated. The question for this
check: is there a published way to keep a racing LMPC from jumping to the model's limit on its first learning lap –
a bound on the progress per lap, or a gradual expansion – and how do the published codes start and continue the laps?

### I.1 Zhao, Nguyen, Lu, Wu – *Vehicle Drift Emergence: Continuous Evolution from Grip Driving to the Handling Limit via Boundary Exploration Learning Model Predictive Control*, arXiv:2608.28723 (v1 Aug 2026; no venue listed; full text read in part, HTML: the formulation, Algorithm 1, the parameter table, the results and the limitations; the last ~4 % of the page not read)

* A racing LMPC in E.2's form that **widens a state envelope lap by lap**:
  * a level η_j ∈ [0, 1] interpolates the bounds on the sideslip angle (7° → 38°) and the yaw rate (2.4 → 6.0 rad/s);
  * after each lap η_{j+1} = clip(η_j + 0.20) if the lap is accepted ("recoverable"), clip(η_j − 0.10) otherwise;
  * a lap is accepted if its smallest track margin is ≥ −0.02 m, at most 15 % of its solves failed, its peak combined
    tyre utilisation is ≤ 1.08 and all states are finite; **only accepted laps enter the safe set**.
* A **fixed global speed bound** (10 m/s, above the observed ~2.9 m/s peaks) is the same on every lap. It is not a
  per-lap bound; the speed allocation is learned from the terminal cost.
* Safe set: 16 nearby samples from each of the last two accepted laps, convex hull, costs-to-go shifted to be
  non-negative per lap; the terminal condition **soft** (weight 800). ATV-QP (OSQP), N = 25 × 50 ms, linearised once per
  step along the warm start.
* Two path-following laps at 1.2 m/s seed the safe set; laps 3–12 are evaluated; the laps run in sequence (the initial
  state of each lap is not stated). At μ 0.6 the lap time falls from 49.95 s (lap 3) to 25.50 s (lap 12).
* **Limits stated by the authors:** the controller's model **equals the plant** (no model mismatch is tested); one
  track, deterministic, no statistics; the soft terminal condition gives no invariance guarantee.
* For phase I: the only source found with an **explicit lap-to-lap limit on how far the next lap may go**. Its limit acts
  on lateral states (aimed at drift) and is gated by the previous lap's outcome.

### I.2 E.4 re-read for this question (Xue, Zhu, Dolan, Borrelli, arXiv:2309.10716 v2, HTML)

* **No explicit speed, progress or hyperparameter schedule** limits an iteration. What limits it is implicit:
  * the terminal set – the convex hull of nearby stored states – lets each iteration go only as far as the data allow;
  * the hard track and input bounds;
  * the input and input-rate costs. The authors call the rate cost **"analogous to the 'learning rate' for LMPC"**: it
    penalises drastic input changes and keeps the car near the safe set.
* The first laps are slow: a tracking MPC on the centre line at 10 m/s for three laps. The authors describe the car as
  starting slow near the centre line and **incrementally** increasing its speed through the corners. The regression data
  are sparsest in the initial laps.
* When the state enters the finish set the car "has transitioned onto the next lap" (consecutive laps). Each lap's data
  are stored only up to the line; past it, the stage cost is zero.

### I.3 Zang, Amine, Kokolakis, Nghiem, Rosolia, Mangharam – *SIT-LMPC: Safe Information-Theoretic Learning Model Predictive Control for Iterative Tasks*, arXiv:2602.16187 (Feb 2026; IEEE RA-L 11(1), 2026, 986–993, per the arXiv page; abstract)

* An iterative-task MPC built on sampling-based (information-theoretic) MPC on a GPU. A normalising-flow value function is
  trained on the earlier iterations, and an adaptive penalty trades safety against performance. The abstract claims
  improvement over the iterations with robust constraint satisfaction. No per-iteration bound and no model mismatch are
  stated in the abstract. Not read in full; listed as the most recent LMPC variant found.

### I.4 Consecutive laps – Brunner, Rosolia, Gonzales, Borrelli, *Repetitive learning model predictive control: An autonomous racing example*, IEEE CDC 2017, 2545–2550 (bibliographic, via dblp – E.6); Brunke, *Learning Model Predictive Control for Competitive Autonomous Racing*, arXiv:2005.00826 (May 2020, a thesis; abstract)

* E.6 extends the racing LMPC to repetitive tasks. Its full text was not found. The thesis (I.4b) attributes to it the
  lap-to-lap initial condition: **the next lap starts from the previous lap's end state, shifted back by the track
  length**. This attribution was seen in the search result only; the thesis itself was read at the abstract only.
* The thesis's own subject is multi-agent LMPC: several initialisations for a richer safe set and a convex state
  selection. It is not used here.

**For phase I:**
* (a) **No source found bounds the progress of a racing LMPC per lap by speed or lap time.** The closest are:
  * I.1's lap-gated envelope on the lateral states (matched model, no mismatch);
  * E.4's implicit limits, its rate cost "as a learning rate".
  Phase I's speed bound per lap (CX_I_protocol.md §3) is therefore **this project's design**, motivated by I.1's
  gating and by E.2 / E.4 / I.1 starting far from the limit.
* (b) **The published LMPCs start slow:** E.2 at ~1.2 m/s (BARC), converging over ~29 laps; E.4 at 10 m/s on the centre
  line; I.1 at 1.2 m/s. Each lap improves gradually because the stored data are far from the limit.
  * Phases E and H started within ~9 % of the η 0.95 plan's lap time: the initialisation laps took 86.9 and 83.1 s,
    against `nmpc`'s 76.40 s. Their first learning lap went to the limit at once.
  * A per-lap bound restores the gradual approach without slowing the initialisation laps, which `identified` and
    `nmpc` do not need.
* (c) Consecutive laps (E.2, E.4, E.6) and E.2's own regression model are the matrix's follow-ups 2 and 3 (§I-2, I-3 of
  the protocol). They are registered there by revision, if reached.

### I.5 E.2 re-read for its lap set-up (Rosolia & Borrelli, arXiv:1901.08184, ar5iv; 2026-10-29)

A second pass over E.2 (read in full for phase E), this time through a summarising fetch of the ar5iv page, for the
reproduction check of CX_I_protocol.md section 10. The page renders some digits twice ("1010 Hz", "11 g"); they are read
as 10 Hz and 1 g, as in E.2's entry.
* **Start:** two laps of path following at constant speed, ~1.2 m/s, seed the safe set (section V).
* **Consecutive laps:** each iteration's data cover two consecutive laps, so the safe set and the Q-function are defined
  beyond the finish line. The terminal set is s ≥ L and the cost-to-go T^j − t. The safe set uses the last two
  iterations (table I).
* **Horizon:** N = 12 at 10 Hz; K = 20 (table I). Neither the tracks' lengths nor their radii are given (an oval and an
  L-shaped track).
* **Outcome:** the lap time decreases until it converges after 29 laps; laps 30–34 are shown as the steady state. The
  largest lateral acceleration is ~1 g, against the limit μ g with μ = 1 (aerodynamics neglected). No lap times or
  speeds are given as numbers.
* **Not reported:** any lap that failed or left the track, and the number of experiments or trials.

**For the reproduction check (protocol section 10):**
* Conditions of the published racing LMPCs that carry over to the simulation:
  * a slow constant-speed start (E.2, I.1 at ~1.2 m/s; E.4 on the centre line at 10 m/s);
  * consecutive laps with data past the line (E.2, E.4);
  * tens of laps (E.2: 29 to converge);
  * a model equal to the plant (I.1).
* Conditions that do not carry over: the cars' scale and hardware; soft track bounds that let a lap go past the edge
  without ending the run (H.1's lane slack, I.1's −0.02 m acceptance). E.2's own regression model is I-2's subject.

## Phases F–G and the frontier methods

Entries are written at the start of each phase, before its code (roadmap section 3). Classic sources
to start from (not yet read; listed so that the check is planned, not as claims): RLS / MHE – Rawlings, Mayne, Diehl (*Model Predictive Control: Theory,
Computation, and Design*); LMPC – Rosolia and Borrelli (iterative LMPC, IEEE TAC 2018) and the racing
application (Rosolia, Borrelli, 2019/2020); CBF / HOCBF / predictive safety filters – Ames et al.,
Xiao and Belta, Wabersich and Zeilinger; advanced-step NMPC – Zavala and Biegler (2009).
