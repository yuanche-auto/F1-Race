# Roadmap L6–L10

Written 2026-10-09 after L0–L5 were merged. A first draft was criticised in
a written critique that the user forwarded (it did not come from the review
session that reviews this repository's commits); this version keeps what held
up, takes over most of the criticism, and says where it does not (section 3).
It is the reference for the reviews of L6–L10. Each layer still gets its own pre-registered protocol
(`docs/L<k>_protocol.md`) before any result-producing run.

## 1. What L0–L5 left open

* L4 showed that identifying the parameters works **when the model structure
  is right**; with noisy data the estimate is biased, and that was not fixed.
* The blind spots of the L5 policies were avoided only by training several
  times and selecting on validation laps; the policies did not become
  reliable by themselves.
* Every MPC was run offline in Python (about 0.1–0.3 s per solve), far from a
  real-time controller.
* Everything happened on one track, with one car, alone.

## 2. Rules for all five layers

1. **Fixed baseline set**, re-evaluated under every new difficulty: L1
   tracker, nominal soft NMPC, NMPC with a uniform reserve, L3 SMPC, L4
   identification + re-planning, the validation-selected L5 policy. What
   changes the *ranking* of these controllers is the main result of a layer.
2. **One source of error at a time**, then combined.
3. **Information parity**: every comparison states what each controller gets
   (whole track or local preview, true state or measurement, own commands).
4. **Protocol before runs**, tuning data separate from test data, a second
   Python environment for the main results, several seeds for anything
   trained; the unselected distribution is reported first.
5. **Go / no-go criterion per layer**: a layer that another layer depends on
   must meet its criterion first.
6. **Literature check at the start of each layer**, before its contribution
   is claimed (section 5).

## 3. Points of the critique taken over, and points not

| Point of the critique | Decision | Reason |
|---|---|---|
| Keep this L6–L10 frame, put "reliable learning" in L7 | taken over | keeps the "remove one idealisation per layer" structure and makes L6 → L7 one line of method |
| L6: separate parameter, noise and structure errors; add one structural error first; an upper bound with the correct structure | taken over | otherwise nothing can be attributed; the L4 noise study already had an inertia estimate on its bound – stable but physically wrong |
| An EKF does not remove structural bias | taken over, refined | the EKF (or smoothing / errors-in-variables) is kept, but only for the noise bias of L4 |
| A safety filter can hide the policy's problems: report filter-off results and intervention rates | taken over | the central measurement of L7 |
| A safety filter built on a wrong model can fail: use the uncertainty calibrated in L6 | taken over | links L6 and L7 |
| MPC distillation only as a control, or in L9 | taken over | it solves a computation problem |
| L8: separate geometric and dynamic generalisation, equalise information | partly | the residual policy follows a plan computed from the whole track and already sees 10–150 m of curvature preview; on a new track every method gets the new plan. The main L8 experiment gives all methods the same plan and preview; "local information only" is a supplementary condition |
| Benchmark real-time cost early | taken over, limited | only the solve time of the candidate safety filters before L7 (half a day); the full L9 stays later |
| L10 exploratory, starting with one opponent of fixed behaviour | taken over | scope |
| No GPU rewrite as a prerequisite; profile first, with consistency tests | taken over | the L5 environment once missed the tracker's drive cut |
| "Failure-aware learning" as an independent method contribution | **not as stated** | the combination of disturbance observer, residual learning and control barrier functions for safe RL has been published with an F1/10 car (section 5); a contribution needs a falsifiable hypothesis, see L7 |

Two more corrections of the first draft:

* A full nonlinear MPC safety filter inside the RL training loop is not
  affordable in Python (0.1–0.3 s per call, millions of calls). Training can
  only use a cheap filter (a control-barrier-function QP on a simplified
  model) or none; the full filter is for evaluation.
* Realistic effort on this machine (CPU only, including second-environment
  reruns and review): L6 3–5 days (about 400 MPC laps, ~30 CPU h per round),
  L7 1–2 weeks (10–15 training runs of 2.5 h plus filter evaluation),
  L8 4–6 days, L9 4–6 days (acados probably needs WSL), L10 1–2 weeks.

## 4. The layers

### L6 – Model mismatch: parameters, noise and structure separated

*Question:* when the model itself is wrong, under which conditions do
parameter identification, residual learning and uncertainty-based tightening
fail?

* Plant: the car of L2 plus **one structural error, a first-order steering
  actuator lag** (three strengths, e.g. 30 / 60 / 120 ms). Noise is a
  separate dimension (the L3 sensor noise). Each alone first, then combined.
* Methods: the fixed baselines; L4 identification (expected to absorb the lag
  into stable but wrong parameters); an EKF for the noise case only; residual
  dynamics learned from data, used in the MPC, with its predictive
  uncertainty used for tightening; **upper bound**: an MPC with the correct
  structure (lag state in the model).
* Metrics: completion, lap time against the upper bound, identified
  parameters (on their bounds? physically meaningful?), **calibration of the
  residual uncertainty** (coverage of its prediction intervals on held-out
  and on closed-loop data).
* **Go / no-go for L7:** the 95 % intervals of the residual model cover
  90–98 % of held-out closed-loop data; otherwise the L7 safety layer has no
  trustworthy risk estimate and L7 must use another one.
* Later extensions: four-wheel model, tyre load sensitivity.

### L7 – Reliable learning: diagnose the blind spots, a safety layer, and whether reliability transfers to the policy

*Question:* when both the model and the learned policy can be wrong, can the
control be reliable – and does the reliability come from the safety layer or
from the policy?

1. **Diagnosis** of the nine L5 runs: states before the failures, saturated
   actions, the critic's error near the failure points, how a blind spot moves
   during training. Mechanism first, method second.
2. **Safety layer** with the uncertainty calibrated in L6; a solve-time
   benchmark decides which filter is cheap enough for training and which is
   only for evaluation.
3. **Training from failures and interventions** (candidate method): failure
   positions from full validation laps and interventions of the safety layer
   are fed back into training (episodes started near them, a cost on
   interventions); aim: the policy *without* the filter becomes reliable.
   Pre-registered success criterion, for example: the median seed without the
   filter completes significantly more laps than plain training; the
   intervention rate falls during training; it is not slower than the
   safety-filtered MPC alone. If it fails, the conclusion is "the safety layer
   works but does not transfer reliability to the policy" – a useful negative
   result.

Always reported: completion with and without the filter, intervention rate,
positions and false interventions, lap-time cost of the filter, its
computation time, comparison with the safety-filtered MPC alone. MPC
distillation only as a control.

### L8 – Generalisation: new tracks and new conditions, separately

* **Geometric:** 3–5 unused tracks of the database for testing, 2 for
  validation; every method gets the plan re-computed for the new track and
  the same local preview.
* **Dynamic:** friction lowered to wet level, local wet patches, on Spielberg.
* **Online adaptation branch:** first-lap identification (L4), online update
  of the L6 residual, short RL fine-tuning.
* Supplementary condition: no offline plan, local information only.

### L9 – Real-time control under an equal compute budget

* Control period 50 ms, the previous input is held on a missed deadline;
  sensor delay, actuator rate limits; a fixed single-core budget.
* Methods: acados code-generated MPC with real-time iterations (preparation /
  feedback split); explicit or neural approximations of the MPC (including the
  L7 distilled policy); RL policies; the L7 safety layer.
* Metrics: mean, P95 / P99 and worst solve time, deadline-miss rate, its
  effect on completion and lap time, loss against the unconstrained-time
  results. *Does the ranking of L3–L7 change?*

### L10 – Interactive racing (exploratory)

* Step 1: one opponent with fixed but unknown behaviour (holding the line,
  following, random lane changes); which parts of L6–L9 transfer (collision
  constraints, opponent prediction)?
* Step 2, decided after step 1 and a literature check: game-theoretic
  planning, self-play, or stop.

## 5. Literature check (2026-10-09)

The works cited by the critique exist as cited (checked by the builder; the
repository's reviewer independently checked SGTP and DKMGP):

* J. Ning, M. Behl, *DKMGP: A Gaussian Process Approach to Multi-Task and
  Multi-Step Vehicle Dynamics Modeling in Autonomous Racing*, L4DC 2025,
  PMLR 283 ([proceedings](https://proceedings.mlr.press/v283/ning25a.html)) –
  multi-step GP residual correction, full-scale Indy car data. Relevant to L6.
* R. Rickenbach, A. Lahoud, E. Schaffernicht, M. Zeilinger, J. A. Stork,
  *ZipMPC: Compressed Context-Dependent MPC Cost via Imitation Learning*,
  CoRL 2025, PMLR 305 ([proceedings](https://proceedings.mlr.press/v305/rickenbach25a.html))
  – a short-horizon MPC with a learned cost imitating a long-horizon one,
  racing. Relevant to L7 (learned cost) and L9.
* D. Kalaria, C. Maheshwari, S. Sastry, *α-RACER: Real-Time Algorithm for
  Game-Theoretic Motion Planning and Control in Autonomous Racing using
  Near-Potential Function*, L4DC 2025, PMLR 283
  ([proceedings](https://proceedings.mlr.press/v283/kalaria25a.html)). L10.
* Z. Li et al., *SGTP: Sampling-based Game-Theoretic Planning for Real-Time
  Multi-Vehicle Autonomous Racing*, arXiv:2607.25388 (2026)
  ([arXiv](https://arxiv.org/abs/2607.25388)) – GPU sampling, open benchmark.
  L10.
* D. Kalaria, Q. Lin, J. M. Dolan, *Disturbance Observer-based Control Barrier
  Functions with Residual Model Learning for Safe Reinforcement Learning*,
  IROS 2025 ([arXiv:2410.06570](https://arxiv.org/abs/2410.06570)) – safe RL
  with a disturbance observer, residual model learning and CBFs, validated on
  an F1/10 car. The closest prior work for L7: its combination is not new
  here.

Further works for L6:

* L. Hewing, J. Kabzan, M. N. Zeilinger, *Cautious Model Predictive Control
  using Gaussian Process Regression*, IEEE TCST 2019
  ([arXiv:1705.10702](https://arxiv.org/abs/1705.10702)) – GP residual
  dynamics in an MPC with uncertainty-based tightening, miniature race cars.
  The method of L6 is in this line; L6 does not claim a new method.
* *Delay-aware Robust Control for Safe Autonomous Driving and Racing*
  ([arXiv:2208.13856](https://arxiv.org/abs/2208.13856)) – steering delay as a
  first-order lag handled in a robust tube MPC.
* *On learning racing policies with reinforcement learning*
  ([arXiv:2504.02420](https://arxiv.org/abs/2504.02420)) – first-order
  actuator models change when a policy starts to steer.

What L6 can contribute is therefore not a method but a controlled comparison:
the same fixed baselines under parameter, noise and structural error
separated, with an upper bound of the correct structure and a calibration
check of the learned uncertainty.

## 6. Order

L6 first round: steering lag only, three strengths, fixed baselines and the
upper bound – no RL, little computation, and its result decides whether the
L7 safety layer can rely on learned uncertainty. Then L6 rounds 2–4 (see
`docs/L6_protocol.md`), the safety-filter solve-time benchmark, L7.

## 7. Strategy update after L6 (2026-10-10, before the L6 review verdict)

L6 answered the question that L7 depended on, without needing the unverified
round-2 laps: the learned residual's uncertainty is not calibrated (held-out
coverage 67–79 % of nominal 95 % intervals), so the L7 safety layer cannot use it
as it is. The user's direction: where the existing results suffice, update the
strategy instead of re-running. Changes:

1. **Risk estimate for the L7 safety layer.** Instead of the raw residual
   uncertainty, in this order: (a) the identified structure – L6 showed that the
   lag and the parameters can be identified from one exploration lap, also under
   noise (EKF); (b) empirically calibrated back-offs as in L3, which already made
   the SMPC the most robust model-based controller without the right model;
   (c) a recalibrated residual uncertainty (split-conformal intervals on held-out
   laps) – the cheapest way to keep a learned component honest. Its calibration
   laps must be separate from the L7 evaluation laps, and if the residual mean
   goes into the dynamics, round 2 of L6 is replayed first (at least at 10 ms,
   the only τ with discordant cars). The go / no-go
   check of L6 is kept as the acceptance test for whichever is used.
2. **Numerical robustness first.** The last failures of the model-based path in
   L6 were numerical: re-plans that did not converge from either warm start and
   bursts of failed NMPC solves. Before L7: a re-plan with continuation in the
   friction margin (as in L1) and more warm starts, and a fallback that is safer
   than the shifted previous plan. This also benefits a model-based safety filter.
3. **Compute.** One minimum-time OCP re-plan needs ~6.8 GB and limits how many
   can run at once on this machine (~14–16 GB free). Profile and reduce it (the
   expression graph and its derivatives, not BLAS) before L7–L8, which need many
   re-plans; single-threaded BLAS and a fresh worker per job are now the default
   for runs.
4. **RL in L7.** The residual RL policy coped with the lag up to 120 ms without
   any model of it (L6) and with the parameter errors (L5) – the safety-layer
   study of L7 starts from it as planned; its cost is 4–4.5 s per lap against the
   correct-structure NMPC, the number a safety layer plus a better policy has to
   beat.
5. **Round 2 of L6** stays reported as "laps not verified" (review: APPROVE with
   that wording; no L7 decision may rest on its lap outcomes).
