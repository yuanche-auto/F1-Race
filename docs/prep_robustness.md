# Numerical robustness before L7 (ROADMAP section 7, point 2)

Branch `prep/robustness`, 2026-10-14, on the new machine (STATUS.md). Goal set by the
user: find the last numerical failures of L6 – re-plans that converged from neither warm
start, bursts of failed NMPC solves – confirm the concrete cases before changing anything,
and make them disappear.

## 1. The failing cases of L6

From `results/L6_final` (the records give the identified parameters and τ, so the cases can
be rebuilt without re-running the exploration laps; `scripts/prep_robustness.py`):

| case | in L6 | kind |
|---|---|---|
| round 1, car 20, τ 60 ms, lag + parameters identified | plan failed (nominal warm start: 1000 iterations; ideal: "infeasible") | re-plan |
| round 3, car 20, no lag, EKF | plan failed (1000 iterations; "infeasible") | re-plan |
| round 3, car 21, no lag, EKF | plan failed (1000 iterations; "infeasible") | re-plan |
| round 1, nominal car, τ 120 ms, τ identified | off track at 1 406 m after 45 failed solves | NMPC burst |
| round 1, car 20, τ 30 ms, true model + lag (reference) | off track at 1 402 m, 52 failed solves | NMPC burst |
| round 1, car 20, τ 60 ms, reference | off track at 1 402 m, 70 failed solves | NMPC burst |
| round 1, car 4, τ 60 ms, reference | off track at 1 408 m, 34 failed solves | NMPC burst |
| round 1, cars 18 and 21, τ 60 ms, τ identified | completed, 16 / 24 failed solves, 3–7 s lost | NMPC burst |

## 2. Reproduction on this machine

Unperturbed, with the L6 settings and the code of `main`:

| case | L6 | here |
|---|---|---|
| re-plan, round 1, car 20, τ 60 ms identified | failed from both warm starts | **failed again** (1000 iterations; "infeasible" after 891) |
| re-plan, round 3, car 20, EKF | failed | converged from the nominal plan (395 iterations) |
| re-plan, round 3, car 21, EKF | failed | **failed again**; with a 1e-10 perturbation it converged (410 iterations) |
| lap, nominal car, 120 ms, τ identified | off at 1 406 m, 45 failed solves | completed, 76.00 s, 0 failed solves |
| lap, car 20, 30 ms, reference | off at 1 402 m, 52 | **off at 1 403 m**, 49 |
| lap, car 20, 60 ms, reference | off at 1 402 m, 70 | **off at 1 402 m**, 50 |
| lap, car 4, 60 ms, reference | off at 1 408 m, 34 | completed, 79.67 s, 0 |
| lap, car 18, 60 ms, τ identified | completed, 86.63 s, 16 | completed, 79.84 s, 0 |
| lap, car 21, 60 ms, τ identified | completed, 83.21 s, 24 | completed, 78.89 s, 6 |

**The failures depend on rounding.** L6 ran round 1 with multi-threaded BLAS (before
afe6160) and with the old L1 plan cache; the same cases here give different IPOPT paths.
A fix can therefore not be judged on one run of a case: every case was repeated with tiny
perturbations (trial k > 0: the initial state of the lap times 1 + 1e-9 N(0, 1), or the
warm-start plans times 1 + 1e-10 N(0, 1), seed k – the test L1 used for its first solve),
with the L6 settings (`old`) and with the new options.

## 3. Changes

* **NMPC, failed solves** (`FrenetMPC(retry=True)`): a solve that ends without
  `Solve_Succeeded` / `Solved_To_Acceptable_Level` is retried with a second IPOPT instance
  on the same NLP (no warm-started multipliers, IPOPT's default barrier start, twice the
  iterations), first from the shifted plan, then from a *rollout* – the shifted inputs
  simulated through the model from the measured state, so that the dynamics constraints
  hold exactly at the initial guess. In L2–L6 the warm start reused the multipliers of the
  last successful solve, unshifted, also after fallbacks.
* **NMPC, fallback** (`fallback="brake"`): if the retries fail too, the controller keeps
  the shifted plan's steering rate but brakes, with the traction control limiting the
  braking to the friction the tyres have left, instead of following the shifted plan's
  longitudinal forces open loop.
* **Re-plan** (`planning/replan.py`, `replan_robust`): first exactly `l4.replan` (one solve
  from each warm start, so every re-plan that succeeded before gives the same plan), then
  continuation in the friction margin from the ideal problem (η = 1 → target in three
  steps, a failed step halved up to twice in a row), then continuation in the parameters
  from the nominal car to the identified one (two steps, same halving), then slower warm
  starts (speed × 0.95, 0.9); at most 16 solves.
* Defaults unchanged: without the options the NMPC and the re-plan behave as in L2–L6
  (tests of L2 and L6 pass unchanged). `planning/ocp_dynamic.py` and `experiments/l1.py`
  are not touched, so the L1 plan cache stays valid.

## 4. Results

### 4.1 Laps: failed solves removed, laps not saved

The four lap cases that still have failed solves here, each unperturbed and with 3–4
perturbations, every variant on the same trials (paired):

| case | trials | old (L6) | retry | brake fallback | retry + brake |
|---|---|---|---|---|---|
| nominal car, 120 ms, τ identified | 4 | 3 completed, 256 fallbacks | 3, 0 | 3, 99 | 3, 0 |
| car 20, 30 ms, reference | 5 | 2, 306 | 2, 0 | 1, 227 | 2, 0 |
| car 20, 60 ms, reference | 5 | 1, 381 | 0, 3 | 0, 295 | 0, 4 |
| car 21, 60 ms, τ identified | 4 | 3, 120 | 1, 2 | 4, 102 | 1, 51 |
| **total** | 18 | **9**, 1 063 | **6**, 5 | **8**, 723 | **6**, 55 |

Discordant pairs against `old` (completed only by old : only by the variant): retry 3 : 0
(exact McNemar p = 0.25), brake 3 : 2 (p = 1), retry + brake 3 : 0 (p = 0.25). On the cases
without failed solves (car 4 and car 18 at 60 ms, 4 trials each) all variants give the
same laps to the last digit – the options act only when a solve fails.

* **The retries remove the failed solves** (1 063 fallbacks → 5): of the failed solves, the
  retry from the rollout converges almost always, the retry from the shifted plan with a
  fresh barrier rarely.
* **They do not save a single lap, and cost three** (not significant with 18 pairs). Every
  remaining failure is at the same place, 1 397–1 430 m: the Remus hairpin (apex at
  ~1 390–1 400 m, 12.7 m/s, planned steering −0.31 rad of the 0.4 rad limit, right after
  hard braking from 38 m/s). In the paired lap of car 21 (trial 2) the two runs are
  identical up to 1 391 m; there the old run applies the shifted plan twice and gets
  round, while the retried run applies a converged solution that slows the car further
  (to ~4 m/s), after which it slides (ellipse slack up to 3.5) and leaves the track. On car
  20 the car is already sliding at 1 399 m with converged solves (predicted ellipse
  violation 0.62) – the failed solves follow the loss of control, they do not cause it.
* **The brake fallback** saves one car-21 lap and loses one car-20 lap; its laps are slower
  (car 21: 80.0–86.3 s against 78.9–82.5 s).
* **Lap times are sensitive to rounding**: a relative 1e-9 change of the initial state moves
  a lap of car 4 at 60 ms between 79.37 and 79.71 s without any failed solve.
* Retries are expensive when they happen: single decisions took up to 27 s (with ~40
  processes on the machine; 0.26–0.28 s on average for laps without failures).

### 4.2 Re-plans

| case | trial | `l4.replan` (L6) | `replan_robust` |
|---|---|---|---|
| round 1, car 20, τ 60 ms identified | 0 | failed (nominal: 1000 iterations; ideal: "infeasible", 891) | **plan** after 6 solves: both direct solves fail as before, η continuation 1 → 0.983 → 0.967 → 0.95 converges at every step (72 / 474 / 41 / 38 iterations); plan 76.79 s |
| | 1 | failed (1000; "infeasible", 631) | **plan**, the same path, 76.79 s |
| round 3, car 21, EKF | 0 | failed (1000; "infeasible", 790) | **plan** after 6 solves, η continuation (63 / 253 / 42 / 44 iterations); plan 75.51 s |
| | 1 | converged directly (410 iterations), 76.03 s | the same (direct) |
| round 3, car 20, EKF | 0 | converged directly here (395 iterations) | – |

Laps with these plans (L6 controllers and settings, unperturbed):

| case | L6 | here |
|---|---|---|
| round 1, car 20, τ 60 ms, lag model with τ and parameters identified | plan failed | **completed, 77.19 s**, 0 failed solves (robust plan) |
| round 3, car 20, EKF, no lag | plan failed | completed, 78.68 s, 1 failed solve (direct plan) |
| round 3, car 21, EKF, no lag | plan failed | **completed, 76.26 s**, 0 failed solves (robust plan) |

* Where a re-plan still fails here, it fails from both warm starts the way it did in L6, and
  the continuation in the friction margin solves it every time (3 of 3); the parameter
  continuation and the slower warm starts were never needed.
* The two plans of car 21 differ (75.51 s from the continuation, 76.03 s from the
  perturbed direct solve): the minimum-time OCP has several local optima for this car.
* Cost: ~30–37 min per rescued re-plan under load, almost all of it the two failing direct
  solves (1 000 iterations and up to 891 before "infeasible"); the continuation itself took
  ~5 min. Lowering the iteration cap of the direct attempts would save most of it but
  changes the re-plans that converge late (up to 471 iterations in L6), so it is not done.

### 4.3 Decision for L7

**Rule for the L7 default, written after the first comparison (old against retry + brake:
14 laps completed by both, 9 by neither, 3 only by old, 0 only by the new settings) and
before the isolated variants (retry only, brake only) were run (2026-10-14 21:10):** the
NMPC setting for L7 is the variant with the most completed laps over the paired trials of
the four cases with failed solves; ties are broken by fewer fallbacks. A variant that
completes fewer paired laps than `old` is not used, whatever its fallback count. If no
variant completes more laps than `old`, L7 keeps the L6 settings and counts and reports
the fallbacks.

**Applied: no variant completes more laps than `old` (9 of 18) – L7 keeps the L6 failure
handling of the NMPC** (`retry=False`, `fallback="shift"`, the defaults). The options stay
in the code, off by default.

## 5. Conclusions

* **Re-plan: fixed.** The re-plan failures that still occur on this machine (2 of the 3
  of L6, with a perturbation 3 of 4 trials) are all solved by the continuation in the
  friction margin, and the three cars that L6 lost to failed re-plans each complete their
  lap here (77.19 s and 76.26 s with the robust plans, 78.68 s with a direct plan; one
  unperturbed lap each).
  `replan_robust` gives the same plan as before whenever the old re-plan succeeds, so it
  replaces `l4.replan` in L7.
* **NMPC failed solves: the symptom is fixed, the failures are not numerical.** The retries
  remove almost every failed solve, but no variant completes more laps than the L6
  settings (9 of 18 paired trials; retry 6, brake 8, both 6). All remaining lap failures are
  at the Remus hairpin with a steering lag of 30–120 ms; the failed solves start after
  the car is already losing grip there. This is a limit of the controller (a minimum-time
  plan without the lag, tracked at η = 0.95 with a 60 ms lag through a 12 m/s hairpin), not
  of the solver – and exactly the kind of state an L7 safety layer has to catch. L7 keeps
  the L6 failure handling and reports fallbacks.
* **The outcome of such laps depends on rounding** (single laps flip between completed
  and off track under a 1e-9 perturbation; lap times move by up to 0.34 s without any
  failed solve). L6's single-run outcomes for these cars – and any single-run outcome near
  this hairpin – are therefore uncertain; L7 counts laps over sets of cars, never a single
  lap.
* Not done: re-running all of L6 with `replan_robust` (the 60 ms identified result could
  move from 24/25 to 25/25 – one lap shown here, not a re-run of the round).

Reproduce: `python scripts/prep_robustness.py burst|replan --case <car>,<tau>,<controller>
--variant <old|retry|retry_brake|robust> --trial <k>`, tables
`python scripts/prep_robustness.py report` (results in `results/prep`, git-ignored).
