# L9 protocol – real-time control under an equal compute budget

Status: **binding, revision 1 (2026-10-18; 0af2b60) with amendment A1** after the review of revision 0 (50e2a8d: l9-M1, l9-m1 to
l9-m4), written before any L9 run. Branch `layer/L9` (from
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

H-RT1 measures the controller as it would be deployed: in the real-time mode its exploration lap
runs in the real-time mode too, so a worse exploration lap and a worse identification are part of
the effect (l9-m2). A secondary comparison separates them: the real-time evaluated lap with the
identification and re-plan from the offline exploration lap (section 5).

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
Gauss-Newton real-time NMPC on miniature cars; Bongard, Jank, Sagmeister, Lohmann (arXiv:2604.19452)
track trajectories near the handling limits on three-dimensional tracks with a robust NMPC on a 3D
single-track model, simplified to stay real-time capable, with uncertainty-aware constraint
tightening (abstract; solver and control rate not named there; l9-m1). acados itself:
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
| T1new + T2new, real-time evaluated lap with the offline exploration's identification and re-plan | `lag_identified` (IPOPT, RTI) | real-time | secondary (l9-m2: direct against indirect effect of the budget) |

## 6. Part C – a neural approximation of the NMPC (secondary)

An MLP (two hidden layers of 256, the RL policy's observation without the history) trained by
DAgger to imitate the acados RTI `nmpc` (nominal model) on Spielberg, with cars from the L2 ranges
and half of the episodes with the L3 disturbances (as the L5 training distribution), three seeds,
10 DAgger iterations of 20 000 decisions each (the expert labels every visited state); evaluated
in both modes on T1new + T2new against its expert. In the real-time mode its inference is timed and
held to the same rule as every other controller (section 3; l9-m4). Fixed now; reported as secondary, after the
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
  own vCPU, and each job first times the same fixed workload; that calibration time and the vCPU
  are recorded with the lap and reported (no lap is excluded for it).
* **Machine load (l9-M1 a).** While real-time laps run, nothing else heavy runs on the machine –
  neither in WSL nor on the Windows side (no offline laps, training or other experiments); offline
  and real-time laps never run at the same time. The utilisation of every core is sampled once a
  minute during every real-time batch (WSL: /proc/stat; Windows: the processor counters) and
  reported with the results.
* **The two arms of a paired comparison under the same conditions (l9-M1 b).** In the real-time
  mode the two implementations of a controller run **interleaved in the same batches**: every batch
  of 8 jobs holds 4 laps × both implementations (IPOPT and RTI) of the same controller, in case
  order – never all IPOPT laps first and all RTI laps after – so that a drift of load or core
  assignment affects both arms alike. The report gives the calibration times of the two arms
  side by side. (H-RT1's other arm, the offline mode, is deterministic and not timed.)
* Outcomes in the real-time mode depend on measured times and are not bit-reproducible. To show
  how much, 20 laps of each IPOPT controller and of `lag_identified` RTI (the other arm of H-RT2;
  l9-M1) in the real-time mode are run twice (the first 20 test laps in case order), interleaved as
  above, and the outcome flips are reported.
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
  with its submodules and built with qpOASES and DAQP – the commits of acados and every submodule in
  `scripts/l9_wsl/acados_versions.txt` (a tag can move; l9-m3); tera_renderer v0.2.1 (2026-07-31,
  SHA-256 64a0a0f8…); a venv with pip 25.2 from its PyPI wheel (SHA-256 in the same file), the lock
  file and acados_template.
  An acados example generated, compiled and solved; the project's tests pass there (the RL tests
  skip – torch is not yet installed in WSL). Per-vCPU speed of a fixed workload: 29.3–31.8 ms.
* No controller has run on T1new or T2new.

## Amendments

* **A1 (2026-10-18), the real-time rule for a controller with a plan – committed before any lap on
  T1new / T2new; agreed with the reviewer after the smoke runs on X.**
  *Cause:* the inputs of the NMPC are the steering **rate** and the two forces. Under the rule of
  section 3 ("the previous command held until a new one is ready") a late IPOPT solve keeps a
  steering rate running for hundreds of milliseconds and the steering angle drifts. On the first two
  laps of the smoke set X in the real-time mode every IPOPT controller left the track within
  70–120 m (deadline misses 100 %, latency growing from ~120 ms to 1–3 s as the car left the plan),
  while the RTI controllers (~3 ms) drove as offline. No controller deployed with a slow solver is run
  that way, so H-RT1 would hold because of the rule. Nothing has been seen on T1new / T2new.
  1. **Plan buffer, time-aligned (the main rule):** a solution started at t_k and ready at t_k + c
     (rounded up to the simulator step) is executed from then on with its segment
     j = floor((t − t_k) / 50 ms) at time t – not from u₀, which was meant for t_k. The next solution
     replaces it as soon as it is ready.
  2. **Past the end of a plan** (j ≥ N, a latency above 1.5 s): steering rate 0 (the steering angle
     held), the two forces of the last segment; the number of laps and of 10 ms steps where this
     happens is reported.
  3. **Warm start by the elapsed steps (real-time mode):** a new solve starts from the previous
     solution shifted by the number of 50 ms steps since the previous solve started, not by one
     (`ShiftFrenetMPC` for IPOPT – FrenetMPC itself unchanged –, `AcadosFrenetMPC.solve(shift=)`
     re-aligning and re-preparing inside the timed feedback). The offline mode is unchanged (always one
     step).
  4. **A plan before the lap starts:** in the real-time mode the controller solves once before t = 0,
     untimed (IPOPT: one solve from its initial guess; RTI: 30 iterations at the initial state), so
     that a plan exists at t = 0; the timed computations start at t = 0. The offline mode, which
     solves at t = 0 with no delay, is unchanged.
  * The RL policies and the bare tracker have no plan: their previous action stays held (the
    tracker's own 10 ms loop runs as always).
  * **The rule of section 3 as written stays as a secondary condition** (`--hold last`, results in
    `hold_last/`): the first command of a solution applied when ready and held, no plan before the
    start (the warm start by elapsed steps, a property of the solvers, applies there too).
  * The rest of section 3 (timing, deadline misses, busy time, preparation of RTI) is unchanged.
* **A2 (2026-10-18), a delay-compensated IPOPT arm – secondary; committed before any lap on T1new /
  T2new; agreed with the reviewer.** *Cause:* under A1 every IPOPT controller still left the track
  within 94–180 m on the smoke laps of X. A step-by-step trace (Spielberg's nominal car,
  `nmpc_reserve`, IPOPT, the start straight; `scripts/l9_debug_rt.py`) shows the mechanism: each
  solve takes ~85–95 ms, so a new plan arrives every 100 ms and its segments 0–1 – meant for the time
  the solve was running – are never executed; with the steering *rate* as input, the skipped segments
  leave the real steering angle different from the plan's, and the steering-rate command roughly
  doubled every 0.1 s (−0.009 rad/s at 0.6 s, −0.117 at 1.0 s, −0.47 at 1.2 s) until the car left the
  track at 129 m after 1.68 s. The NMPC of L2–L8 has no delay compensation. Without an arm that has
  one, H-RT1 cannot separate "the solver is slow" from "the latency is not compensated". H-RT1 and
  H-RT2 stay as they are (this is decided after seeing X; the primary tests are not changed for it).
  1. At t_k the controller propagates the measured state over [t_k, t_k + L̂] with the inputs already
     committed there (the active plan's time-aligned segments, A1), with its own discrete model
     (`FrenetMPC._step`; for the lag-aware model the commanded angle and the angle together), to
     x̂(t_k + L̂), and solves from x̂ with u_prev = the committed segment of [t_k + L̂ − 50 ms,
     t_k + L̂) – the last input in effect before the new plan starts. The prediction is part of the
     timed computation.
  2. The new plan is aligned to t_k + L̂: at t ≥ t_k + L̂ its segment j = floor((t − (t_k + L̂)) / 50 ms)
     is executed. If the computation takes c < L̂, the switch happens at t_k + L̂ (not earlier); if
     c > L̂, at readiness as in A1, with the same formula for j. The share of solves with c > L̂ is
     reported. The warm start is shifted by the steps between the plans' alignment times. The controller
     stays busy until the plan's alignment time – busy until max(t_k + c, t_k + L̂), the next solve
     starting at t_k + L̂ – so that a solve finished early cannot be followed by another that would
     overwrite a plan not yet started (one plan per L̂; review finding a2-M1); reported: the number of
     solves that would have been free before t_k + L̂, and of them those that would have started
     another solve before it (a control instant before t_k + L̂ – the case that overwrote plans).
  3. **L̂ = 100 ms, fixed now and not tuned:** the median IPOPT latency on X was ~90 ms, rounded up to
     a multiple of 50 ms. Only for the IPOPT implementations of `lag_identified` and `nmpc`, in the
     real-time mode (`impl = "ipopt_dc"`, `run_l9.py --dc`); the exploration lap of `lag_identified`
     in this arm (`nmpc_reserve`) is compensated the same way (section 4: the same implementation and
     mode as the evaluated lap; `nmpc` has no exploration lap). It runs interleaved with the IPOPT
     main arm and RTI (section 9).
  4. Secondary, no Holm: reported against the IPOPT main arm and against RTI on the same laps (exact
     McNemar each, with p).
  5. The change rests on X only; nothing has been seen on T1new / T2new.
  6. **Repeat (section 9):** the compensated arm (`nmpc` and `lag_identified` with `ipopt_dc`, and
     the exploration laps of the latter) is part of the repeat of section 9 – the first 20 test laps
     run a second time, interleaved as in the main run (`run_l9.py run --mode realtime --dc --repeat 20
     --tag repeat`) – and its outcome flips are reported like those of the other arms.
  7. **Smoke runs on X before and after the a2-M1 fix (not a result; local records
     `results/L9/smoke_dc`, `smoke_dc2`, `smoke_dc3`).** The fix changes the schedule only when a
     solve takes c < 50 ms: then the next control instant lies before t_k + L̂ and the old code started
     another solve there. For 50 ms ≤ c ≤ L̂ the old code already started the next solve at t_k + L̂,
     after switching to the pending plan, and for c > L̂ max(t_k + c, t_k + L̂) = t_k + c, so both were
     unchanged. Distances of the compensated arm on X (cars 1 / 2, τ = 0; no lap completed in any
     run): before the fix (`smoke_dc`) `nmpc` 477 / 484 m, `lag_identified` 442 / 1410 m, the
     exploration lap 2872 / 2859 m; after the fix (`smoke_dc2`) `nmpc` 2664 / 2844 m,
     `lag_identified` 438 / 2652 m, the exploration lap 2655 / 2693 m. A third run of the fixed code
     (`smoke_dc3`, the same code with one more counter) gave 3168 m for `nmpc` on car 1 (2664 m in
     `smoke_dc2`), with no solve shorter than 50 ms (would_restart = 0 of 500). The runs before the
     fix recorded only the summary of the latencies (mean / P95 / P99 / max), not every solve, so
     whether a solve shorter than 50 ms occurred there cannot be checked. The mean latency of the
     `nmpc` laps was 127–131 ms before the fix and 106–107 ms after it; this can be cause or
     consequence, since IPOPT's solve time depends on the state. The difference most likely comes
     from the variation of the measured times between runs – this arm's latencies lie around L̂
     (share above L̂ 0.30–0.98), and the same fixed code gave 2664 and 3168 m on the same lap – but
     this cannot be confirmed, and it is not attributed to the fix. Every lap of this arm now records
     the counts held_to_plan and would_restart, so the case c < 50 ms can be checked lap by lap.
  * **Batches (section 9, l9-M1 b):** in the real-time mode `lag_identified` and `nmpc` run in
    batches of 2 laps × 3 implementations (IPOPT, IPOPT + compensation, RTI), the other controllers
    in batches of 4 laps × 2 implementations; every batch holds all implementations of the same laps,
    at most 8 jobs, and the next batch starts when the whole batch has finished.
  * **Section 12, correction (torch):** `.venv-rl` (Windows) and the WSL venv both have torch
    2.14.1+cpu – the Windows and the Linux wheel of the same CPU build (the WSL wheel
    `torch-2.14.1+cpu-cp312-cp312-manylinux_2_28_x86_64.whl` from the PyTorch CPU index, SHA-256
    5a636357…711f, `scripts/l9_wsl/acados_versions.txt`); the other RL packages have the same
    versions as `.venv-rl` (`scripts/l9_wsl/requirements-rl-wsl.txt`). The RL loop of L9 reproduces
    L7's records lap for lap in the offline mode (`B_s1_500k` and `C_s1` on T1's cars 0–5: 12 of 12).
