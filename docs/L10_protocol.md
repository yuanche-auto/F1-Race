# L10 protocol – interactive racing, step 1: one opponent of fixed behaviour

Status: **draft, revision 0 (2026-10-19)**, for review; written before any L10 code or run. Branch
`layer/L10` (from `main` at c010887). ROADMAP: section 4 (L10, exploratory), rules of section 2.
Nothing about opponents has been measured or seen before this draft (section 13).

## 0. What L0–L9 left

Every controller of L0–L9 drove alone. The best of them depend on a plan and a model of the own car
(L4–L8: identification and re-planning), or on a policy trained on solo laps (L5, L7). Under a
compute budget only the acados RTI implementation of the NMPC still runs, and the RL policies are
nearly unaffected (L9). ROADMAP section 4 sets L10's first step: **one opponent with fixed but unknown
behaviour**, and the question of **which parts of L6–L9 transfer** – collision constraints, opponent
prediction. Step 2 (game-theoretic planning, self-play, or stop) is decided after step 1 and a
literature check (section 9).

## 1. Question and hypotheses

*When a slower car with a fixed behaviour, unknown to the ego car, is on the track ahead, which of
the existing controllers complete their lap without contact – and what do a collision constraint and
the choice of opponent prediction contribute?*

One source of difficulty at a time (ROADMAP rule 2): the ego car is the **nominal car** (no model
error, no L3 disturbances, no actuator lag), so the opponent is the only new difficulty. The
combination with car errors is left to step 2.

The outcome of a scenario (section 4) is a **clean lap**: the ego completes one lap from its start
point within the time limit without contact, without leaving the track and without spinning.

* **H-I1 (primary): the opponent prediction matters.** On the 96 test scenarios (section 2), the NMPC
  with a collision constraint and the **racing-line prediction** (`nmpc_line`) and the same NMPC with
  the **constant-velocity prediction** (`nmpc_cv`) differ in the number of clean laps. Exact McNemar,
  two-sided, on the paired scenarios; holds if significant at its Holm level; the direction is
  reported. No direction is predicted: constant velocity extrapolates a lane change linearly (it
  overshoots it), the racing-line prediction ignores it (it keeps the current offset).
* **H-I2 (primary): learned against model-based, with the same opponent information.** The
  validation-selected RL policy (`B_s1_500k`, L5) behind the L7 NMPC safety filter with the same
  collision constraint and the constant-velocity prediction (`filter_cv`) and `nmpc_cv` differ in
  the number of clean laps. Same test and rule. In L7 the filter lowered the policy's completions on
  solo laps; here the policy does not observe the opponent at all, so the filter is its only way to
  avoid it.

Holm over H-I1 and H-I2 (family-wise α = 0.05), with the stricter decision (each at 0.025) reported
alongside, as in L7–L9. Scope: both compare controllers on these scenarios, with this opponent model
and these predictions; no claim about racing against reactive or strategic opponents in general.

Reasoning written before any run: an opponent-unaware controller on the same racing line runs into a
slower car ahead within seconds. A collision constraint lets an NMPC brake or pass; whether it passes
depends on the room the track leaves (Spielberg 10.2–13.7 m wide, the car 2.0 m) and on how well the
prediction anticipates the opponent's lateral moves. The RL policy behind the filter can only be
pushed away from the opponent by the filter, which in L7 changed about a third of its decisions.

## 2. Scenarios

A scenario: start point s₀ on the centre line, opponent behaviour (section 3), opponent speed
factor k, initial gap g, and (for the lane-change behaviour) a random seed.

* **Test set T (run once per controller): Spielberg.** s₀ ∈ {j · L / 8, j = 0 … 7} (L = 4 315.7 m),
  g ∈ {30, 60} m (centre to centre along the centre line, opponent ahead), k ∈ {0.85, 0.92},
  behaviour ∈ {hold, lane, block} – **96 scenarios**. Lane-change seed = the scenario's index.
* **Tuning set V: Monza** (an L8 validation track, its L8 plans): s₀ ∈ {j · L / 4, j = 0 … 3}, the
  same g, k and behaviours – 48 scenarios, lane-change seeds 1 000 + index. Every setting of
  section 5 that is not fixed in this protocol is chosen on V only (section 11).
* **Start:** a rolling start – the ego in the state of its 5 % plan at s₀ (as the L2 laps start at
  s = 0), the opponent at s₀ + g on its own path at speed k · v_plan. The ego drives one lap
  (L metres) from s₀; time limit 1.6 × the 5 % plan's lap time (121 s on Spielberg, as L8 / L9).
* **Alone:** every ego controller also drives the eight (T) and four (V) start points without an
  opponent; these laps are the reference for lap-time losses and show whether a start point is hard
  in itself.
* **Pre-screen of the opponent (before any ego lap; a rule fixed now):** the opponent drives each
  scenario alone for 121 s. A lane-change scenario whose opponent leaves the track or spins gets the
  next seed (index + 96 · m, m = 1, 2, 3); hold and block scenarios that fail are reported and kept
  out (block alone is hold). Replacements are listed.

## 3. The opponent

The nominal car (`DynVehicle()`), driven by the L1 dynamic tracker (`DynTracker`) along its own path
and speed profile, simulated in lockstep with the ego (10 ms steps, the same integrator). Its
behaviour is fixed per scenario; the ego is not told which.

* **hold:** the ego's 5 % racing line, speed k · v_plan(s).
* **lane (random lane changes):** the racing line shifted laterally by δ(s); δ switches between
  targets drawn uniformly from {−3, 0, +3} m at exponentially distributed intervals (mean 4 s),
  each switch a quintic transition over 1.5 s; the shifted line is clipped to 2.0 m from the edges.
  Speed min(k · v_plan(s), the quasi-steady-state speed of the shifted path at friction factor 0.85).
* **block** (the ROADMAP's "following", read as *following the ego's lateral position*): on the
  racing line at k · v_plan, but while the ego is behind within 40 m along the track the opponent's
  lateral target becomes the ego's lateral offset (clipped as above), reached at most at 2 m/s
  laterally with a 0.3 s reaction delay; the speed rule as for lane. Back to the racing line when
  the ego is ahead or more than 40 m behind. *Reviewer: if "following" was meant differently in the
  ROADMAP (for example an opponent that starts behind and follows the ego), this is the place to
  change it.*

If the opponent leaves the track or spins without contact during an ego lap, it is removed and the
ego's lap continues; such events are counted and reported per controller.

## 4. Contact and outcomes

* **Car body for contact:** a rectangle 5.6 m long (from 2.4 m behind to 3.2 m ahead of the CG,
  around the axles at −1.62 m and +1.98 m) and 2.0 m wide, for both cars. **Contact:** the two
  rectangles overlap at any 10 ms step (separating-axis test). The ego's lap ends at the first
  contact (outcome "contact").
* **Outcomes per ego lap:** clean lap (section 1); contact; off track; spin; time limit – the
  stop reasons of L2–L9 plus contact. Also recorded: **overtake** (the ego ahead of the opponent along
  the track at the end of its lap), the lap time, the minimum centre distance and the minimum body
  distance to the opponent, the time spent within 30 m behind it, and at a contact the relative
  position (ego behind, alongside, ahead) and which front or rear corner touched first (descriptive,
  no blame rule).

## 5. Controllers

**Unaware (the fixed baseline set, ROADMAP rule 1; they do not see the opponent):** the L1 tracker;
`nmpc` (nominal soft NMPC, the 5 % plan); `nmpc_reserve` (the 12 % plan); `smpc` (the L3 back-offs,
calibration unchanged); `B_s1_500k` (L5, validation-selected) and the nine final L5 checkpoints (the
unselected distribution first). The identification controllers of L4–L8 (`identified`,
`lag_identified`) are not run: on the nominal car there is nothing to identify, so they coincide with
`nmpc` up to identification noise (stated so that the reviewer can object). All as configured in L8
(no solver retries; a failed solve applies the shifted previous plan).

**Opponent-aware (they get the opponent's state, section 6):**

* **Collision constraint (common to all aware controllers):** for every stage i = 1 … N of the NMPC
  (N = 30, 50 ms), ((s_i − ŝ_i)/a_s)² + ((n_i − n̂_i)/a_n)² ≥ 1 − σ_i, σ_i ≥ 0, with the predicted
  opponent position (ŝ_i, n̂_i) as parameters of the NLP and the slack penalised like the track-limit
  slacks (quadratic + linear, weight w_c). Active while the opponent is within 150 m ahead or 20 m
  behind along the track; otherwise the parameters place it far away. a_s ∈ {6, 8} m and
  a_n ∈ {2.5, 3.0} m (body 5.6 × 2.0 m plus margin) and w_c ∈ {10⁴, 10⁵} are chosen on V
  (section 11); everything else is the L2 NMPC unchanged (tracking mode, 5 % plan as reference).
* **`nmpc_cv` – constant-velocity prediction:** ŝ_i = s_o + ṡ_o t_i, n̂_i = n_o + ṅ_o t_i (clipped to
  the track), from the opponent's measured Frenet position and velocity at the solve.
* **`nmpc_line` – racing-line prediction:** the opponent keeps its current speed ratio to the plan
  and its current lateral offset from the racing line: ŝ advances with k̂ · v_plan(ŝ),
  k̂ = v_o / v_plan(s_o); n̂_i = n_plan(ŝ_i) + (n_o − n_plan(s_o)).
* **`filter_cv` – RL behind the safety filter:** `B_s1_500k` proposes; the L7 NMPC filter
  (`FrenetMPC(mode="filter")`, N = 30, nominal model, η = 0.95, no back-offs – the car is the nominal
  one) with the collision constraint and the CV prediction applies the closest admissible input.
  Secondary: the nine final L5 checkpoints behind the same filter (the unselected distribution).

Information parity (ROADMAP rule 3): every controller knows the track and its 5 % plan (the RL
policies their L5 observation, which contains no opponent); the aware ones also get the opponent's
true pose and velocity at each solve – no noise, no delay, no knowledge of its behaviour type, speed
factor or seed. Offline mode as in L2–L8 (no compute budget; one source of difficulty at a time –
the real-time combination is a candidate for step 2).

## 6. What the aware controllers observe

At each solve (every 50 ms): the opponent's position projected on the centre line (s_o, n_o), its
heading relative to the track and its velocity, converted to (ṡ_o, ṅ_o) and v_o. Between solves the
NMPC's plan is applied as in L2–L8.

## 7. Always reported

Per controller, behaviour and k: clean laps (exact 95 % intervals), outcomes, overtakes, lap time on
clean laps and its loss against the controller's alone lap from the same start point (exact
signed-rank, paired), minimum distances, time behind the opponent, contacts by relative position;
solve time (mean, P95, P99, max), fallbacks and constraint slacks (track, collision) of the NMPCs;
the filter's intervention rate; opponent removals; the alone laps. The ranking of all controllers by
clean laps (ties by mean lap time) and Kendall's τ against the alone ranking (descriptive).

## 8. Statistics

Exact McNemar on paired scenarios, exact intervals, exact signed-rank tests (mid-ranks, the exact
distribution) for paired lap times. Two primary tests with Holm (section 1); everything else with its
p value and the number of comparisons, no claim across the family. The 96 scenarios share start
points and behaviours, so they are not independent draws of "racing situations"; the tests are about
these scenarios.

## 9. Go / no-go and step 2

* **Go for T:** after tuning, at least one aware NMPC (`nmpc_cv` or `nmpc_line`) completes at least
  24 of the 48 V scenarios clean. Otherwise no T lap runs; L10 step 1 reports V and the failure modes
  and stops there (ROADMAP rule 5).
* **Step 2** is decided with the user after the step-1 report and a literature check; the criteria,
  fixed now: if the best aware controller's contacts concentrate on the reactive opponent (block:
  more than twice the contact rate of hold), interaction modelling (iterated best response or a
  potential game, section 12) is the candidate; if `filter_cv` is not worse than `nmpc_cv`, training
  with opponents (self-play) is; if the best aware controller is clean on at least 95 % of T and
  overtakes in at least half of the hold scenarios, step 1 already answers the ROADMAP question and
  stopping is an option.

## 10. Second environment

As in L7–L9 (still deferred by the user): H-I1 and H-I2 are re-run in the Python 3.11 environment
when it exists; until then they are marked as not cross-checked.

## 11. Compute and order

1. Code: the two-car simulation (lockstep, contact test, opponent removal), the three behaviours and
   the pre-screen, the collision constraint and the two predictions in `FrenetMPC` (a new option;
   the formulation without it unchanged, checked against L8 records), the filter extension, the
   runner and the report code. Checked on V smoke scenarios only; **reviewed before any tuning run.**
2. Pre-screen of the opponents on V and T (opponent alone; no ego lap).
3. **Tuning on V:** the 2 × 2 × 2 = 8 settings of (a_s, a_n, w_c), each with `nmpc_cv`, `nmpc_line` and
   `filter_cv` on the 48 V scenarios; for each controller the setting with the most clean laps, ties
   by fewer contacts, then lower mean lap time. The chosen settings, the go / no-go result and the V
   records go to the reviewer before any T lap.
4. T: alone laps, then the 96 scenarios for every controller of section 5 (offline, deterministic up
   to the solvers; up to 20 jobs at once, one process per job, single-threaded BLAS).
5. Report, review, merge.

Estimate: an aware NMPC lap ~5–8 min (IPOPT with the collision rows); V tuning ~580 such laps
(~60 CPU-h), T ~1 150 (`nmpc_cv`, `nmpc_line`, `filter_cv` × 10 policies; ~130 CPU-h); the unaware
laps end at the first contact and are short. Windows `.venv` / `.venv-rl` (no acados needed),
pinned worktree, input SHA-256 list as in L7–L9.

## 12. Literature check (ROADMAP rule 6; 2026-10-19, abstracts and search results)

* **Game-theoretic racing:** Liniger and Lygeros, *A Non-Cooperative Game Approach to Autonomous
  Racing*, IEEE TCST 2020 (arXiv:1712.03913) – two-car racing as a non-zero-sum game in a receding
  horizon, Stackelberg and Nash equilibria, blocking as a payoff. Wang, Wang, Talbot, Gerdes,
  Schwager, *Game Theoretic Planning for Self-Driving Cars in Competitive Scenarios*, RSS 2019 –
  iterated best response with a sensitivity term (from the drone-racing planner of Spica et al.,
  RSS 2018). Kalaria, Maheshwari, Sastry, *α-RACER*, L4DC 2025, and Li et al., *SGTP*,
  arXiv:2607.25388 (2026) – both in the ROADMAP's check. Candidates for step 2.
* **Opponent prediction for overtaking with MPC:** Zhu, Busch, Johnson, Borrelli, *A Gaussian Process
  Model for Opponent Prediction in Autonomous Racing* (arXiv:2204.12533, submitted to IROS 2023) – a
  one-step GP of the opponent's closed-loop behaviour rolled out in the ego MPC; similar win rates and
  safe in up to three times as many races as literature predictors. Baumann et al., *Predictive
  Spliner: Data-Driven Overtaking in Autonomous Racing Using Opponent Trajectory Prediction*, IEEE
  RA-L 10(2), 2025 (arXiv:2410.04868) – GP opponent trajectory prediction and overtaking on 1:10 cars.
  Stahl, Wischnewski, Betz, Lienkamp, *Multilayer Graph-Based Trajectory Planning for Race Vehicles in
  Dynamic Scenarios*, ITSC 2019 (arXiv:2005.08664) – following, overtaking on a full-scale car. These
  works learn or plan the prediction; L10 step 1 compares two simple predictions on fixed behaviours,
  and a learned prediction (the identification idea of L4–L8 applied to the opponent) is a step-2
  candidate.
* **Learning to race with opponents:** Wurman et al., *Outracing champion Gran Turismo drivers with
  deep reinforcement learning*, Nature 602 (2022) 223–228 – model-free RL with mixed-scenario
  training and a sportsmanship reward. Steiner et al., *Accelerating Real-World Overtaking in F1TENTH
  Racing Employing Reinforcement Learning Methods* (arXiv:2510.26040, 2025) – training against
  opponents raised the overtaking rate from 56 % to 87 %. The L5 / L7 policies were trained alone;
  training with opponents is a step-2 candidate.
* **Survey:** Betz et al., *Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing*,
  IEEE OJ-ITS 3 (2022) 458–488.
* Found by search, abstract not read: FSDP (arXiv:2503.06075), a two-stage QP overtaking planner.

L10 step 1 claims no new method: it measures which of the existing controllers, and which of two
standard predictions, handle a slower opponent of fixed behaviour on the same scenarios, with
pre-registered tests.

## 13. Measured or seen before this draft (disclosure)

Nothing about opponents. Known from L2–L9 and used in the design: the solo results on Spielberg's
nominal car (the 5 % plan's lap ~75.6 s, the basis of the 121 s limit of L8 / L9), the
IPOPT solve times of L9's offline runs (mean ~165–185 ms under 20 jobs), Spielberg's width
(10.2–13.7 m, from the track data) and the car's dimensions (`DynVehicle`: wheelbase 3.6 m, half
width 1.0 m).
