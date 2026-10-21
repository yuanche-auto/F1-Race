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
* **The basis for the "curvature-aware MPCC" of phase A**, which is defined only after the full text
  has been read (an amendment of the phase-A protocol); the abstract does not give the weight function.

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

## Phases B–G and the frontier methods

Entries are written at the start of each phase, before its code (roadmap section 3). Classic sources
to start from (not yet read; listed so that the check is planned, not as claims): MPPI – Williams et
al. (information-theoretic MPC / MPPI, 2016–2018); tube MPC – Mayne, Seron, Raković (2005) and Mayne et
al. (nonlinear tube MPC, 2011); RLS / MHE – Rawlings, Mayne, Diehl (*Model Predictive Control: Theory,
Computation, and Design*); LMPC – Rosolia and Borrelli (iterative LMPC, IEEE TAC 2018) and the racing
application (Rosolia, Borrelli, 2019/2020); CBF / HOCBF / predictive safety filters – Ames et al.,
Xiao and Belta, Wabersich and Zeilinger; advanced-step NMPC – Zavala and Biegler (2009).
