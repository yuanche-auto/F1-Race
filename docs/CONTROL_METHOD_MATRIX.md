# Control method matrix – what L0–L9 implemented, what the expansion adds

Audit of the code on `main` c010887 (2026-10-21), for the advanced-control expansion
([CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md)). "Exists" means implemented, tested and
used in a reviewed layer; the file is given. Nothing listed as existing is re-implemented.

## Existing methods (L0–L9)

| family | method | where | layer, status |
|---|---|---|---|
| planning | minimum-curvature line (QP), quasi-steady-state speed profile, kinematic and dynamic minimum-time OCP; robust re-planning by continuation | `planning/mincurv.py`, `qss.py`, `ocp_kinematic.py`, `ocp_dynamic.py`, `replan.py` | L0–L1, prep for L7; approved |
| tracking | path / speed tracker (L0), dynamic tracker with feed-forward of the plan (L1) | `control/path_tracking.py`, `control/dyn_tracking.py` | L0–L1; approved |
| NMPC | Frenet NMPC (IPOPT, multiple shooting, RK4, soft limits, friction ellipses, warm start, retries / brake fallback); modes `track` (reference tracking), `mpcc` (progress maximisation in Frenet coordinates, **never evaluated**), `filter` (predictive safety filter) | `control/nmpc.py` (`FrenetMPC`) | L2–L9; `track` and `filter` approved |
| min-time MPC | spatial receding-horizon minimum-time MPC | `control/rh_mpc.py` | L2 – exploratory, failed in all 8 configurations (docs/L2.md §3) |
| uncertainty-aware | uniform friction reserve (12 % plan); stochastic MPC with calibrated back-offs (empirical chance-constraint tightening) | `FrenetMPC(backoff=…)`, `experiments/l3.py` | L2, L3; approved |
| identification | least squares from an exploration lap; identification with the steering lag; EKF parameter / lag estimation; re-planning with the identified car | `estimation/identify.py`, `estimation/ekf.py`, `experiments/l4.py`, `l6.py`, `l8.py` | L4, L6, L8; approved (offline, from one exploration lap – no online update) |
| learned dynamics | residual dynamics by random Fourier features with evidence-based hyperparameters; cautious NMPC (mean + tightening) | `estimation/residual.py`, `CautiousMPCController` | L6 – no-go for L7 (calibration insufficient) |
| structure | lag-aware NMPC (8-dim model with the actuator lag), open-loop lag observer | `FrenetMPC(steer_lag=…)`, `LagAwareMPCController` | L6–L9; approved |
| RL | residual SAC on the tracker with observation history; failure-start curriculum | `rl/env.py`, `rl/curriculum.py`, `experiments/l5.py`, `l7.py` | L5, L7; approved (L7 H-C not supported) |
| safety | NMPC predictive safety filter (L7 main filter, back-off estimates a/b/c); CBF-QP on a simplified model | `FrenetMPC(mode="filter")`, `experiments/l7.py` (`NMPCFilter`, `CBFFilter`), `control/safety_filter.py` | L7; approved (H-B not supported: the filter lowered completions) |
| real time | acados SQP-RTI and SQP of the same OCP; time-aligned plan buffer; warm start shifted by elapsed steps; delay-compensated IPOPT (fixed L̂); sensor-delay buffer; RL timing; DAgger students | `control/acados_mpc.py`, `experiments/l9.py`, `l9_distill.py` | L9; approved |

## Gap matrix (the seven core phases)

| phase | method | exists? | what is missing | depends on | first validation |
|---|---|---|---|---|---|
| A | **dynamic MPCC** (contouring / lag error, progress variable θ, Cartesian dynamic model) | no – `FrenetMPC(mode="mpcc")` only maximises Frenet progress, has no contouring / lag error or θ, and was never evaluated | the standard formulation (Liniger et al. 2015), curvature-aware variant, later adaptive / differentiable / learned-cost variants | the NMPC's model and limits (reused) | nominal car, Spielberg (development); phase-A protocol for the benchmark |
| B | **MPPI** (sampling-based MPC) | no | batched rollouts, correlated noise, path-integral weights, warm start, constraint handling (costs / projection) | the dynamic model in numpy (batched) | nominal car; then L3 disturbances, L6 lag, L9 timing |
| C | **robust tube MPC** | no – L2's uniform reserve and L3's back-offs tighten constraints empirically, without an error tube or ancillary feedback | nominal model, error dynamics, disturbance set, ancillary feedback, tube propagation, tightening, robust-feasibility diagnostics | phase A/NMPC structure; a disturbance set from L3's calibration | L2 cars, L3 disturbances, L6 lag |
| D | **adaptive MPC / online identification** | partly – offline least squares / EKF from one exploration lap (L4, L6, L8) | recursive least squares, moving-horizon estimation, adaptive NMPC (online model update), model-bank adaptation, drift tests | `estimation/identify.py` (regressors reused) | L4 cars, L6 lag, L8 wet / patches, parameter drift |
| E | **learning MPC** (iterative LMPC) | no – learned residual dynamics (L6) is not LMPC | sampled safe set, cost-to-go from stored laps, terminal constraint on the safe set, lap-by-lap iteration; then LMPC with local error-dynamics regression | phase A or NMPC model; stored closed-loop laps | nominal car first (lap-time improvement), then L4 / L6 cars |
| F | **safety-critical control** | partly – NMPC filter and CBF-QP (L7) | audit of L7's failures (intervention excess, delay, infeasibility runs, backup feasibility, discontinuity); robust CBF, high-order CBF, backup-set safety, reduced-order reachability where feasible | L7 modules (reused, not rewritten) | L7 laps (T1 / T2 as reported), then L3 / L6 / L9 |
| G | **real-time enhancement** | partly – RTI, shift warm start, delay compensation, plan buffer (L9) | learned warm starts, delay-aware state prediction for RTI (the gap L9's 20 ms result exposed), advanced-step MPC, deadline-aware fallback, anytime optimisation, solver-failure recovery; separate measurement of solver, end-to-end, sensor and actuation delay | `acados_mpc.py`, `experiments/l9.py` (reused) | L9's rule (50 ms, one vCPU) |

Frontier methods after the seven phases (roadmap section 5): chance-constrained / CVaR / distributionally
robust / scenario MPC and conformal calibration (L3); recurrent and context-conditioned residual RL,
model-based RL, MPC imitation (L5); GP residuals, deep ensembles, local error-dynamics regression,
physics-informed models (L6); online / meta adaptation, universal dynamics models (L8).
