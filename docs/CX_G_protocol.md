# CX-G protocol – real-time control: delay-compensated RTI under L9's rule (control expansion, phase G)

Status: **revision 0, proposed for review (packet 1, 2026-10-23)**. Written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §G) and before any phase-G code: no module, test, runner
or lap of phase G exists. Before this draft: a check that L9's WSL environment still imports (Python 3.12.3, casadi
3.8.1, acados_template; 24 vCPUs) – no lap, no solve. Branch `cx/G-realtime` from `main` 1922066 (the merge of phase
F). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md) (phase G: "learned warm starts, delay-aware
prediction for RTI, advanced-step MPC, deadline-aware fallback, anytime optimisation, solver-failure recovery; separate
timing of solver, end-to-end, sensor and actuation delay"; validation "L9's rule (50 ms, one vCPU)").

## 0. What L9 and phases A–F left

* **L9** (docs/L9.md), under its real-time rule (one vCPU per controller, 50 ms period, a command takes effect when its
  computation has finished, rounded up to the 10 ms step):
  * the IPOPT NMPC of L6–L8 completes nothing (0 of 105), with every solve late (147–344 ms);
  * the acados SQP-RTI implementation of the same problem completes 79 of 105 (`lag_identified`), with a feedback
    latency of 2.4 ms mean and no deadline miss.
* **The delay is the RTI's weak point:** with a 20 ms sensor delay (plus the 10 ms command latency), the RTI
  `lag_identified` completed **0 of 105** (79 without the delay), the laps ending around 2.7 km. L9's interpretation,
  not tested: "a delay-compensated RTI (predicting the state over the known latency, as A2 does for IPOPT) is the
  obvious next step". L9's A2 compensated IPOPT over a fixed 100 ms, which is useless when the solve takes 200 ms.
  RTI's latency is short and steady, the case the compensation is made for (§G (a)).
* Phases A–F did not touch timing; phase F left the open question of a filter with retries under 50 ms (its solves took
  191–306 ms on average under load).

## 1. Question and hypotheses

*Does predicting the state over the known delay – from the delayed measurement to the time the command takes effect,
with the controller's own model and the commands it has already applied – make the real-time RTI `lag_identified`
complete laps under a sensor delay, and does it change anything without one?*

* **H-G1 (primary):** with a 20 ms sensor delay, `rti_dc` (compensated) and `rti` (L9's RTI, uncompensated) differ in
  the number of completed laps on the test set GT (section 2).
* **H-G2 (primary):** without a sensor delay (only the command latency of the rule), `rti_dc` and `rti` differ in the
  number of completed laps on GT.

Both: exact McNemar (two-sided binomial test on the discordant pairs; the same identified model and plan for both arms of
a case, section 3), **Holm over H-G1 and H-G2** at α 0.05, each also against 0.025. No direction is predicted (L9's 0 of
105 makes H-G1's direction likely but not certain: the compensation can only use the controller's model, whose error
grows over the window). Each test needs at least 6 discordant cases, all one way, to reach 0.025.

## 2. Sets

* **Test: GT** (new, run once): GT1 – the nominal car and `sample_plants(nominal, 24, seed 371)` × steering lag
  τ ∈ {0, 30, 60 ms} (75 laps); GT2 – the nominal car with the L3 disturbances, seeds 7700–7729 (30 laps); 105 laps on
  Spielberg, built as L7 / L9's T1new + T2new. (Seeds checked against those in use: car seeds 0–10, 99 and 301–359,
  and disturbance seeds 0–29, 2000–6029, 7400–7429, 7500–7509, 7590–7592, 9999.)
* **Development: GD** (development only): `sample_plants(nominal, 3, seed 379)[1:]` × τ (9 laps) and seeds 7790–7792 (3
  laps); and **NOM** (the nominal car, τ = 0, no disturbances).
* No tuning set: nothing is tuned (section 3.2). L9's T1new + T2new are not reused – real-time outcomes are not
  bit-reproducible (L9's repeat: ~1 flip in 20), so both arms run here, side by side.

## 3. Controllers and conditions

**The controller:** L9's `lag_identified` in its acados SQP-RTI implementation, as L9 runs it.
* Before the evaluated lap: an exploration lap (`nmpc_reserve`, RTI, real time, **without** the sensor delay – as L9's
  delay condition); the identification of the parameters and of τ; a re-plan.
* **The exploration, identification and plan are done once per case and shared by every arm** (L9's staged runs), so
  the arms of a case differ only in what section 3.1 changes.
* Then the lag-aware NMPC on the identified model under L9's rule:
  * revision 1 with amendment A1 – the time-aligned plan buffer and the untimed pre-start plan;
  * one vCPU per job (`taskset`), single-threaded;
  * the feedback phase is the latency, and the preparation counts against the period;
  * traction control on the true state at every 10 ms step, not budgeted, as in L9.

| arm | sensor delay d | compensation |
|---|---|---|
| `rti`, d = 0 | 0 | none (L9's main real-time RTI) |
| `rti_dc`, d = 0 | 0 | over the command latency L̂ |
| `rti`, d = 20 ms | 20 ms | none (L9's delay condition) |
| `rti_dc`, d = 20 ms | 20 ms | over d + L̂ |

### 3.1 The compensation (`rti_dc`; literature G.2, G.4, G.5)

At a control instant t_k, when the controller is idle (L9's rule):
1. **The measurement** is that of t_k − d (with T2's noise). From it, the controller's state is formed as in L9 (the
   Frenet state; for the lag-aware model, the lag observer's estimate of the actual steering angle and the commanded
   angle).
2. **Prediction over the sensor delay:**
   * the controller's own model – the identified `DynVehicle` with the identified τ, no disturbances – is integrated
     with the simulator's RK4 step (10 ms) over the d / 10 ms steps from t_k − d to t_k;
   * the integration applies the commands the controller itself output in those steps (after its traction control);
     it keeps a record of its own outputs, which it knows.
3. **Prediction over the command latency:** one more 10 ms step, L̂ = 10 ms, with the command of the active plan at t_k
   (before traction control).
4. **The solve:** the RTI's feedback phase is solved from the predicted state. The new plan is time-aligned to
   t_k + L̂ (its segment 0 starts there). It takes effect at max(t_k + latency rounded up to 10 ms, t_k + L̂); until
   then, the previous plan continues (A1).

The prediction (steps 1–3) is computed after the measurement and counted in the latency.

**Fixed now, not tuned:**
* **L̂ = 10 ms.** L9's RTI feedback latency was 2.4 ms on average, with the largest per-lap P99 11 ms. A latency
  above L̂ is applied when ready and counted ("late").
* **d is given exactly** (20 ms): the delay is assumed known, as a property of the sensor chain (G.4, G.5). Section 5
  measures what a wrong d does.
* The model of the prediction is the controller's own (identified) model. **Information budget:** the same as `rti` –
  the controller's model, its own commands, the delayed measurement – plus the known delay. No oracle.

### 3.2 Not in phase G (and why)

* **Advanced-step NMPC (G.3), learned warm starts (G.9), anytime / deadline-capped IPOPT:** these target converged or
  iterative solvers. Under L9's rule the converged IPOPT completes nothing at 147–344 ms per solve; capping it near
  50 ms leaves a few interior-point iterations from a warm start – a cruder version of the RTI that already runs within
  the budget (2.4 ms).
* **Deadline-aware fallback and solver-failure recovery:** the RTI had no deadline miss and 0–7 failed QPs per 105
  laps in L9 (the shifted-plan fallback, L6's handling), so there is little to recover from on these laps.
* These stay follow-ups (CONTROL_METHOD_MATRIX.md). Phase G measures the timing decomposition the roadmap asks for
  (section 5).

## 4. Secondary conditions (descriptive, uncorrected; on GT with the same shared plans)

* **S1 – a larger and a mis-specified delay:** d = 40 ms with `rti` and with `rti_dc` compensating 40 ms (the true d)
  and 20 ms (a wrong d).
* **S2 – a controller without identification:** `nmpc` (nominal model, 5 % plan, RTI) at d = 20 ms, `rti` and
  `rti_dc`. It needs no exploration.
* **S3 – the RL policy for context:** `B_s1_500k` at d = 20 ms under L9's rule (L9: 88 of 105 with the delay, 98
  without). No compensation exists for it; it is listed as context, not a comparison.
* **S4 – repeat:** the first 20 laps of the four primary arms, run again (L9's A2.6: outcome flips as the noise floor).

## 5. Always reported

Per arm, on GT1 by τ and on GT2:
* completed laps (exact 95 % intervals), stop reasons, distances of the failures;
* mean lap time of the completed laps, and the paired lap-time difference on laps both arms complete;
* **the timing decomposition** (roadmap):
  * the sensor delay d;
  * the prediction's cost;
  * the feedback latency (solver), the preparation time, and their sum against the 50 ms period (misses);
  * the end-to-end delay from the measured state's time to the time its command takes effect (d + the latency
    rounded up);
  * the actuation delay – the car's steering lag τ (true) and the identified τ̂;
  * the share of "late" solves (latency above L̂);
* **the prediction's error:** the predicted state at t_k + L̂ against the true state then (position, heading, speeds,
  steering angle; median and P95). It shows the model error over the window;
* failed QPs (fallbacks), plan exhaustion;
* the calibration of the fixed workload per job and its vCPU;
* the machine load (WSL and Windows, once a minute).

## 6. Statistics

Exact McNemar for H-G1 and H-G2 with Holm. Secondary, uncorrected, all descriptive:
* the S1–S3 comparisons, by exact McNemar;
* the lap-time differences on laps both arms complete, by exact signed-rank;
* `rti_dc` at d = 20 ms against `rti` at d = 0 (how much of the no-delay performance the compensation keeps).

## 7. Second environment

As before (deferred by the user): H-G1 and H-G2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). **No code before its APPROVE.**
2. Code (Windows checkout; runs in WSL, L9's environment):
   * a compensation option of `experiments/l9._RealTime` (default off – L9's behaviour, checked by a regression lap
     against L9's offline record and by an equivalence test: d = 0 and L̂ = 0 reproduce `rti`);
   * `experiments/cx_g.py` (sets, the predictor, arms);
   * `scripts/run_cx_g.py`, reusing `run_l9.py`'s staged execution: pinning, calibration, load sampler, batches that
     interleave the arms of a case;
   * `scripts/report_cx_g.py`, `scripts/cx_g_dev.py`, unit tests;
   * the development checks of section 9; packet 2 before any GT lap.
3. GT from a pinned worktree in WSL:
   * exploration 105 laps (real time);
   * identification and re-plan (≤ 3 at once, as L9);
   * the four primary arms 420 laps;
   * S1 315, S2 210, S3 105 and S4 80 laps.
   Real-time stages run in batches of at most 8 jobs on vCPUs 0–7, with the arms of a case side by side, and nothing
   else heavy runs at the same time. RTI laps take about a minute or two each, so a few hours in all.
4. Report; review; final review; merge.

## 9. Development checks (planned)

On NOM and GD only (records in `results/CX/G/dev/` with commit and dirty flag):

1. **Equivalence:** in offline mode with d = 0, `rti_dc` with L̂ = 0 reproduces `rti` lap by lap (test).
2. **Predictor:** on NOM with the nominal model (the exact model, no lag, no noise) and d = 20 ms, the predicted state
   at t_k equals the true state up to numerical noise (the same RK4 step and the same commands). On GD cars, the
   prediction error over d + L̂ with the identified model is reported.
3. **Laps:** the four primary arms on NOM and GD (13 cases): completions, timing, prediction errors.
4. **Gate:** (i) `rti_dc` at d = 20 ms completes NOM; (ii) its feedback latency including the prediction stays
   below 10 ms on average. Otherwise an amendment before any GT lap.
