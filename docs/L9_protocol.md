# L9 protocol – real-time control under an equal compute budget

Status: **draft, revision 0 (2026-10-18)**, written before any L9 run. Branch `layer/L9` (from
`main` at 896b243). ROADMAP: section 4 (L9), rules of section 2. What was measured or seen before
this draft is listed in section 12.

## 0. What L2–L8 left

Every controller of L2–L8 was run **without a time limit**: the simulation waited for each solve.
The NMPC (casadi + IPOPT, N = 30, 50 ms steps) takes about 100 ms per solve on an idle core
(P95 146 ms, max 244 ms; section 12) and 155–443 ms on average under the load of L7's runs – two
to nine times its own control period. The RL policies (an MLP every 50 ms) and the L1 tracker cost
milliseconds or less. So every model-based result so far is an offline result, and the comparison
"model-based against learned" of L5–L8 was made at unequal computation.

## 1. Question and hypotheses

*When every controller has to deliver its command on one core within the 50 ms control period –
the previous command held until a new one is ready – does the ranking of L2–L8 change, and does a
real-time formulation of the same NMPC (acados, real-time iterations) keep what the offline NMPC
achieved?*

* **H-RT1 (primary): the budget costs the offline implementation.** Under the real-time rule of
  section 3, `lag_identified` with its L6 implementation (IPOPT) completes **fewer** of the 105
  test laps than the same controller without a time limit. Exact McNemar, two-sided, on the paired
  laps; holds if significant at its Holm level in that direction.
* **H-RT2 (primary): real-time iterations recover it.** Under the real-time rule,
  `lag_identified` with the acados SQP-RTI implementation of the same optimal control problem
  (section 4) completes **more** of the 105 test laps than the IPOPT implementation under the same
  rule. Same test and rule.

Holm over H-RT1 and H-RT2 (family-wise α = 0.05), with the stricter decision (each at 0.025)
reported alongside, as in L7 / L8. Scope: both compare implementations of one controller;
`lag_identified` is chosen because it is the best model-based controller of L6–L8 and the one that
tied with the learned policy on Spielberg's dry cars (L8).

Reasoning written before any run: IPOPT at ~100 ms per solve can update the command only every
second or third period and with a latency of ~100 ms; whether that costs laps at 50 ms dynamics is
the question of H-RT1 (it may not – the NMPC's plan is smooth). An RTI step costs a fraction of
that (casadi's own sqpmethod with OSQP: 44 ms, section 12; acados code-generated: expected a few
ms), at the price of one Gauss-Newton step per period instead of a converged solve – whether that
approximation costs laps is part of what H-RT2 and the secondary comparisons show.

**Literature check (ROADMAP rule 6, at the start of L9; abstracts and search results,
2026-10-18).** The real-time iteration (Diehl et al.; Gros, Zanon, Quirynen, Bemporad, Diehl,
*From linear to nonlinear MPC: bridging the gap via the real-time iteration*, IJC 2020) and its
use for racing: Verschueren et al. (CDC 2014) made minimum-time race-car driving tractable for
Gauss-Newton real-time NMPC on miniature cars; a full-scale racing controller in acados C code at
100 Hz with one linearisation and one QP per update (arXiv:2604.19452; from the search summary,
not read). acados itself:
Verschueren, Frison, Kouzoupis, ..., Diehl, *acados – a modular open-source framework for fast
embedded optimal control*, Math. Prog. Comp. 14 (2022) 147–183 (arXiv:1910.13753). Neural
approximations of racing MPC: DAgger with a long-horizon MPCC expert in Tearle et al. (RA-L 2021,
arXiv:2102.11907) and behavioural cloning of an NMPC (Freiburg, thesis). L9 claims no new method:
it measures what the real-time requirement does to the controllers of L2–L8 on the same laps, with
pre-registered tests.

## 2. Laps

* **Test (run once per controller and mode): T1new + T2new**, defined in L7 (amendment A2.3) and
  never run: the cars `sample_plants(nominal, 24, seed 10)` with the nominal car × steering lag
  τ ∈ {0, 30, 60 ms} (75 laps) and the nominal car with the L3 disturbances, seeds 6000–6029
  (30 laps); 105 laps on Spielberg. Disjoint from every set used for tuning or testing in L2–L8.
* **Validation (implementation choices and checks only):** L7's V (`sample_plants(nominal, 10,
  seed 1)` without the nominal car × τ, and L3 seeds 2000–2009; 40 laps) and the smoke set X.
* Spielberg only: L8 showed that the learned policies do not transfer to other tracks; L9 isolates
  the computation, with every controller on its home track.

## 3. The real-time rule

Two modes, on the same laps:

* **offline** – as in L2–L8: at every control instant t_k = k · 50 ms the controller is called
  with the measurement and its command takes effect at t_k; its computation time does not matter.
* **real-time** – the controller runs in its own process pinned to one vCPU, single-threaded (BLAS,
  casadi, torch), and its computation is timed (`time.perf_counter` around the call). A computation
  starts at a control instant t_k when the controller is idle, with the measurement at t_k. Its
  command takes effect at t_k + c_k, rounded up to the simulator's 10 ms step; **until then the
  previous command is held**, also across control instants. If c_k exceeds the 50 ms period (a
  **deadline miss**), the computation still completes and its command is applied when ready (late);
  no new computation starts before it finishes – the next one starts at the first control instant
  after it. For acados RTI, the *preparation* phase (linearisation, condensing) runs right after
  the previous feedback and counts against the previous period (preparation + feedback ≤ 50 ms or
  it is a miss); the *feedback* phase (QP solve with the new measurement) is the latency.
* Not budgeted, in both modes: the simulator; the 10 ms inner loops that already exist (the L1
  tracker's own feedback inside the RL environment, the NMPC's traction control at every simulator
  step); the exploration lap's identification and the re-plan of `identified` / `lag_identified`
  between the exploration lap and the evaluated lap (a pit stop, ~1 min).
* Actuator rate limits are already in the plant (steering rate, power, friction ellipse). **Sensor
  delay** (secondary condition, section 5): the measurement used at t_k is the one of t_k − 20 ms.

## 4. Controllers

| controller | implementations | notes |
|---|---|---|
| `tracker` | L1 tracker on the 5 % plan | cost negligible |
| `nmpc` | IPOPT (L2) / acados RTI | nominal model, 5 % plan, η 0.95 |
| `nmpc_reserve` | IPOPT / acados RTI | 12 % plan, η 0.88 (uniform reserve; also the exploration controller) |
| `smpc` | IPOPT / acados RTI | L3 back-offs and per-stage η unchanged |
| `identified` | IPOPT / acados RTI | L4 identification from the exploration lap, re-plan, NMPC on the identified model |
| `lag_identified` | IPOPT / acados RTI | τ and parameters identified, re-plan, lag-aware NMPC (8 states) |
| `B_s1_500k` | SAC policy on the tracker | L5 validation-selected |
| RL distribution | the nine final L5 checkpoints | reported first among the RL results |

* **The acados RTI implementation** solves the same optimal control problem as `FrenetMPC`: the
  same discrete dynamics (the RK4 step with the exact steering lag, given to acados as discrete
  dynamics), the same costs (tracking terms as nonlinear least squares; the input-rate terms on a
  state augmented by the previous input), the same hard bounds and constraints, and the same soft
  constraints with the same exact penalties (acados slacks with the corresponding linear and
  quadratic weights). Solver: SQP-RTI with the Gauss-Newton Hessian, HPIPM with partial condensing,
  the previous solution shifted as initial guess, one QP per period. If the QP fails, the shifted
  previous plan is applied (the L6 failure handling). Implementation choices not fixed here (QP
  tolerances, condensing horizon, regularisation) are made on the validation set only.
* **Equivalence check before any test run (pre-registered):** on 200 states logged on V (offline
  IPOPT laps), the acados SQP solved to convergence (not RTI) and IPOPT on the same problem give
  first commands that agree within 2 % of each input's range (steering rate ±δ̇_max, forces 20 kN)
  in at least 95 % of the states. If not, the acados model is fixed before anything else runs.
* `identified` / `lag_identified` take their exploration lap with `nmpc_reserve` in the **same
  implementation and mode** as the evaluated lap.
* **Not in L9:** the L7 safety filter (it lowered completions in L7; its NMPC costs 150–440 ms per
  call and would need its own RTI version) – listed as untested. The explicit or neural
  approximation of the MPC (ROADMAP) is a secondary part (section 6); L7 produced no distilled
  policy to reuse.

## 5. Conditions

| condition | controllers | modes | role |
|---|---|---|---|
| T1new + T2new, no sensor delay | all of section 4 | offline and real-time, both implementations | **H-RT1, H-RT2** and the ranking |
| the same with 20 ms sensor delay | `lag_identified` (IPOPT, RTI), `B_s1_500k`, `tracker` | real-time | secondary |

## 6. Part C – a neural approximation of the NMPC (secondary)

An MLP (two hidden layers of 256, the RL policy's observation without the history) trained by
DAgger to imitate the acados RTI `nmpc` (nominal model) on Spielberg, with cars from the L2 ranges
and half of the episodes with the L3 disturbances (as the L5 training distribution), three seeds,
10 DAgger iterations of 20 000 decisions each (the expert labels every visited state); evaluated
in both modes on T1new + T2new against its expert. Fixed now; reported as secondary, after the
primary laps have run.

## 7. Always reported

Per controller, implementation and mode: completion (exact 95 % intervals) on T1new, T2new and
together, lap time; per call the computation time (mean, P95, P99, max), the deadline-miss rate and
the latency distribution (real-time mode); the loss against the offline result of the same
controller (completed laps and lap time on laps both complete); NMPC fallbacks / failed QPs; the
ranking per mode (more laps first, ties by lower mean lap time) and Kendall's τ between the modes
(descriptive); the RTI against IPOPT comparison in the offline mode (what the approximation of one
iteration costs without latency).

## 8. Statistics

Exact McNemar on paired laps, exact intervals, exact signed-rank tests (mid-ranks, all 2ⁿ sign
assignments) for paired lap times. Two primary tests with Holm (section 1); everything else with
its p value and the number of comparisons, no claim across the family.

## 9. Timing fidelity and reproducibility

* L9 runs entirely in the WSL environment (Ubuntu 24.04, Python 3.12.3, the project's lock file –
  numpy 2.5.3, scipy 1.18.1, casadi 3.8.1 – and acados v0.6.0 built from source with tera_renderer
  0.2.1; section 12): one machine and one OS for every timed controller.
* WSL's vCPUs are scheduled by Windows onto the 285HX's performance and efficiency cores; pinning
  a process to a vCPU does not pin it to a physical core. Idle, every vCPU ran a fixed workload in
  29–32 ms (section 12). Real-time runs therefore run at most **8 jobs at once**, each pinned to its
  own vCPU, and each job first times the same fixed workload; that calibration time is recorded
  with the lap and reported (no lap is excluded for it).
* Outcomes in the real-time mode depend on measured times and are not bit-reproducible. To show
  how much, 20 laps of each IPOPT controller in the real-time mode are run twice (the first 20 test
  laps in case order) and the outcome flips are reported.
* The offline mode is deterministic; its laps may run with more jobs at once.

## 10. Second environment

The WSL environment uses the same package versions as the main one on another OS and compiler; it
is not the Python 3.11 environment of section 8 of L7 (still deferred, L7 A10 / L8 section 9).
H-RT1 and H-RT2 are re-run there when it exists; until then they are marked as not cross-checked.

## 11. Compute and order

1. acados implementation of the OCP; the equivalence check (section 4); RTI settings on V;
   timing checks on V and X. Code reviewed before any test lap.
2. Offline mode on T1new + T2new (IPOPT controllers ~3–4 min per lap, 20 jobs at once; RTI and RL
   laps much faster): ~1 000 IPOPT laps including exploration laps, ~60 CPU-h.
3. Real-time mode (8 jobs at once): ~600 IPOPT laps, ~40 CPU-h → ~5 h; RTI and RL laps short.
4. The repeat of section 9; the sensor-delay condition; Part C; report; review.

## 12. Measured or seen before this draft (disclosure)

* Feasibility probe on Windows (`.venv`, idle machine), Spielberg's nominal car along the 5 % plan
  for 25 s, the L2 NMPC (489 variables, 607 constraints): IPOPT 99.6 ms mean, P95 146 ms, max 244 ms
  per solve (500 solves); casadi's own `sqpmethod` with one iteration (RTI) and OSQP: 43.8 ms mean,
  P95 56.2 ms, max 58.2 ms (50 steps), its first command within 0.014 (median, scaled) of IPOPT's;
  with qrqp 3.7 s per step; HPIPM could not be used through casadi's generic interface.
* The WSL environment (user's approval, 2026-10-18): acados v0.6.0 (released 2026-08-06) cloned
  with its submodules and built with qpOASES and DAQP; tera_renderer v0.2.1 (2026-07-31,
  SHA-256 64a0a0f8…); a venv with pip 25.2 from its PyPI wheel, the lock file and acados_template.
  An acados example generated, compiled and solved; the project's tests pass there (the RL tests
  skip – torch is not yet installed in WSL). Per-vCPU speed of a fixed workload: 29.3–31.8 ms.
* No controller has run on T1new or T2new.
