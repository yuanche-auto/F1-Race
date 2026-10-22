# Control expansion roadmap – advanced control methods across L1–L9

Started 2026-10-21 at the user's direction: the development focus moves from L10 (interactive racing)
to a **horizontal expansion of the control methods of L1–L9**. It is not a new research layer – no
method gets a layer number, and L0–L9 stay the baseline. L10 is **PAUSED — Preserved for Future
Development** (its binding protocol revision 1, review record and branch `layer/L10` stay unchanged;
STATUS.md). Goals, in this order of priority: correctness, faithful implementation, closed-loop
validation, fair comparison, computational performance, recent research relevance, coverage. Not a
goal: new algorithms, paper contributions, or a single best lap time.

Companion documents: [CONTROL_METHOD_MATRIX.md](CONTROL_METHOD_MATRIX.md) (audit and gaps),
[CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md), [CONTROL_EXPANSION_BENCHMARK.md](CONTROL_EXPANSION_BENCHMARK.md)
(conventions and results), [CONTROL_EXPANSION_STATUS.md](CONTROL_EXPANSION_STATUS.md) (progress, review
state – the resume point), and one protocol per phase, `docs/CX_<phase>_protocol.md`.

## 1. Rules (inherited from the project; nothing here relaxes them)

* **The L0–L9 record is untouched.** Existing controllers, protocols, seeds, scenario sets, simulator,
  results and reports are not changed; their negative results (L7's filter lowering completions, L7's
  failure-start training, L8's non-transfer of the residual policies, L9's IPOPT failing the budget and
  RTI's collapse under a 20 ms sensor delay) stand. New methods are new modules or new options whose
  default leaves the existing behaviour unchanged, checked by a regression lap against an L-layer record
  where one exists. New results never replace a pre-registered L-layer comparison.
* **Protocol before benchmark.** Each phase gets `docs/CX_<phase>_protocol.md` (question, controllers,
  sets with new seeds, tuning on a tuning set only, primary tests with Holm, metrics), reviewed before
  any benchmark lap. Development checks before the protocol are allowed on the nominal car and on sets
  that no protocol uses as a test set, and are disclosed in the protocol. Multi-seed for anything
  trained; the unselected distribution first; the second Python environment stays an open item
  (deferred by the user for L7–L9; the same for the expansion) and every primary result is marked
  "not cross-checked" until it exists.
* **Review gates** (docs/workflow.md): Development → local verification → the original reviewer →
  fixes → reviewer acceptance → stage completed. The reviewer is the session that reviewed L7–L9,
  `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), reached by SendMessage. No stage is
  called completed before its APPROVE; findings, fixes and the commit of each round are kept
  (CONTROL_EXPANSION_STATUS.md and STATUS.md's review log).
* **Information budget.** Every controller states what it gets (track, plan, true state or measurement,
  own commands, model). A controller that reads true car parameters is labelled *oracle*.
* **Branches and outputs.** One branch per phase, `cx/<phase>-<method>` from `main`, merged after the
  phase's final APPROVE; results under `results/CX/<phase>/` (git-ignored, never on `main`); benchmark
  runs from a pinned worktree under `C:\Users\yuanche\F1-Race-worktrees\` with an input SHA-256 list,
  single-threaded BLAS, one process per job.

## 2. Phases (the seven core extensions, in this order)

| phase | method | module | validation conditions | main risk / cost |
|---|---|---|---|---|
| A | dynamic MPCC (contouring / lag error, progress θ); curvature-aware MPCC; later adaptive / differentiable / learned-cost MPCC | `control/mpcc.py` | L2 car mismatch (primary); L6 lag, L8 new tracks, L3 disturbances (secondary); L9 timing in phase G | a 1.5 s horizon at 85 m/s does not see the braking zones (L2 §4) – speed bound needed; aggressive use of the limits; ~0.1 s per IPOPT solve |
| B | MPPI (batched rollouts, correlated noise, path-integral weights, warm start, constraint costs); then risk-sensitive / robust / adaptive covariance | `control/mppi.py` | nominal; L3 disturbances; L6 lag; L9 budget | sample counts for a 7-state car at 50 ms in numpy; GPU optional (the RTX PRO 3000 is unused so far) |
| C | robust tube MPC (nominal model, error dynamics, disturbance set, ancillary feedback, tube propagation, tightening, robust-feasibility diagnostics) | `control/tube_mpc.py` | L2 cars, L3 disturbances, L6 lag | a disturbance set that is neither empty nor so large that the tightened problem is infeasible; nonlinear error dynamics |
| D | RLS, MHE, adaptive NMPC, model-bank adaptation (tyre, cornering stiffness, friction, aero, lag, drift) | `estimation/rls.py`, `estimation/mhe.py`, `control/adaptive_mpc.py` | L4 cars, L6 lag, L8 wet and patches, drift | keeping static identification, state estimation and online model update apart |
| E | iterative LMPC (sampled safe set, cost-to-go, terminal constraint, lap-by-lap); LMPC with local error-dynamics regression | `control/lmpc.py` | nominal (lap improvement), L4 / L6 cars | safe-set size and the mixed-integer flavour of the terminal constraint (convexified as in the racing LMPC papers) |
| F | audit of the L7 filters (interventions, delay, infeasibility runs, backup feasibility, discontinuity); robust CBF, high-order CBF, backup-set safety, reduced-order reachability where feasible | reuse `experiments/l7.py`, `control/safety_filter.py`; new `control/safety/` only for new methods | L7 laps; L3 / L6 / L9 | relative degree and model error of the CBFs; not repeating L7 |
| G | learned warm starts, delay-aware prediction for RTI, advanced-step MPC, deadline-aware fallback, anytime optimisation, solver-failure recovery; separate timing of solver, end-to-end, sensor and actuation delay | reuse `control/acados_mpc.py`, `experiments/l9.py` | L9's rule (50 ms, one vCPU) | the WSL environment of L9; timing noise (L9's repeat: ~1 flip in 20) |

Each phase: literature entries (CONTROL_LITERATURE_REVIEW.md) → mathematical formulation in the module
docstring → unit tests → numerical checks → a deterministic closed-loop development lap → the phase
protocol → review of code and protocol → tuning on the tuning set → (tuning results to the reviewer) →
benchmark on the test set from a pinned worktree → report section in CONTROL_EXPANSION_BENCHMARK.md →
review → merge. After the seven phases, the frontier methods of CONTROL_METHOD_MATRIX.md, chosen for a
clear paper basis and an interpretable mechanism.

## 3. Literature before code

Each method's entry records the paper (title, authors, year), the mathematical formulation, assumptions,
architecture, objective and constraints, solver, open-source availability, our differences and known
limitations – with what was actually read (full text, abstract, bibliographic). Methods from 2025–2026
are checked for existence and versions; no claim of superiority rests on an abstract.

## 4. Not in this expansion

L10 work of any kind (paused); changes to L0–L9 protocols or results; a reorganisation of the
repository; renaming the expansion's methods as layers.
