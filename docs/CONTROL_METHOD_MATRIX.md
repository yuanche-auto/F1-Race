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
physics-informed models (L6); online / meta adaptation, universal dynamics models (L8). From phase D (CX-D review m3): a full
moving-horizon estimator of states and parameters (literature D.5–D.7) for the noisy conditions, where it can differ from
the RLS; a tube around the adaptive model (robust adaptive MPC, D.13); a plan library over more than friction (or an
online re-plan), since the friction-only library left the oracle below the full re-plan (CX-D: 42 against 50 of 50).
For any later phase that uses `cx_d.DriftField`: count a start state projected to s = L − ε as s = 0 (CX-D packet 4 – the
first 10 ms step of every drift lap ran at f_end; no outcome changed there, the definition was kept for CX-D's records).
From phase E (CX-E amendment E1, review e3; the phase was closed after its development gate failed): **the racing LMPC
as E.2 implements it – an ATV QP, linearised along the candidate trajectory, so that the convex hull of stored states is
feasible for the model the problem uses** (the first candidate); a bound on the progress per lap (e.g. the terminal
time-to-go not below the previous lap's at the corresponding point minus a limit); consecutive laps instead of every lap
from the same initial state.

### Follow-up candidates from phase B (MPPI; CX-B amendment B3, review d4)

Phase B found that MPPI on the NMPC's problem does not reliably complete the nominal lap at the real-time CPU
budget (docs/CX_B_protocol.md section 9). Candidates, not scheduled, each with its source:

* **The unperturbed sequence U as one of the samples** – keeps the update from making a near-optimal U worse
  (section 9.4: one iteration made it worse in 16 of 20 solves); used by some MPPI implementations (review n2 of
  packet 2; not read for this list).
* **Adaptive temperature** – λ set per step from the cost spread / an effective-sample-size target instead of a
  fixed grid value (the ESS of the development laps ranged from 1 to ~1 000 across the grid).
* **Adaptive / optimal sampling covariance** – CoVO-MPC (literature B.10, abstract read).
* **Input lifting for smoothness** – SMPPI (literature B.4, full text read).
* **A GPU implementation** – K of 10⁴–10⁵ within 50 ms (literature B.3, B.6, B.9); the RTX PRO 3000 of this machine is
  unused so far.
* **The failure mechanism at the 3.84 km corner** – the rear axle near saturation (usage 0.96, vy +2.4 m/s), every
  sample penalised (ESS 1), the car sliding wide at the exit (section 9.7, `traces.json`); the same corner where
  phase A's low-contouring-weight MPCC left by millimetres.
