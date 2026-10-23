# CX-F protocol – safety filters around the L5 policies: L7's filter audited, retried, and a backup shield (control expansion, phase F)

Status: **binding, revision 0 (2026-10-22; e2bacfd, APPROVE)** with the clarifications of its review (m1–m3, n1, n2;
marked *"clarification of revision 0"*), written down before any phase-F code. **Development gate failed (section 9):
the shield's backup does not drive the cars. Amendment F1, option 1 (the review of packet 2): the shield part is closed
without the benchmark – H-F1 is not tested; H-F2 is the only primary test (α 0.05); no FV, `shield` or `backup` lap.** Written after the literature check
([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §F) and before any phase-F code: no module, test, runner or
lap of phase F exists; the only measurement so far is a timing of the shield's rollout (no lap, section 9). Branch
`cx/F-safety` from `main` 490d33e (the merge of phase E). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md)
(phase F: "audit of the L7 filters …; robust CBF, high-order CBF, backup-set safety, reduced-order reachability where
feasible"; risk: "not repeating L7").

## 0. What L7 and phases A–E left

* **L7 Part B** (docs/L7.md): on L7's test sets T1 + T2 (105 laps), L7's **main filter** – `FrenetMPC(mode="filter")`,
  N = 30 × 50 ms, the nominal model with the back-offs of the accepted risk estimate (b), L6's failure handling (no retry;
  a failed solve applies the shifted previous plan) – made **every one of the nine final L5 checkpoints complete fewer
  laps** (−6 … −71; W⁺ = 0, exact p = 0.0039), and the **CBF-QP** (a second-order barrier on a point mass) completed none.
  Two mechanisms were measured: (i) solver-fallback runs (≥ 3 fallbacks in the last 3 s before 383 of the 667 filtered
  failures); (ii) unneeded interventions (a third of the decisions changed; 88–99 % of the sampled ones not needed by the
  3 s criterion). L7 limited its scope explicitly (pb2-m1): **a filter with retries was not evaluated – "whether retries
  change the result under the filter is open"**.
* Both of L7's filters are **minimal-modification optimisers** (§F, reading (a)): they change the command whenever their
  model cannot certify it over the horizon. The other family – a **shield** that lets the proposal pass as long as a
  backup can still recover from where it leads (MPS, F.1; gatekeeper, F.3) – was not tested.
* L7's per-decision records (`results/L7/partB_filtered/dec/{main,cbf}/*.npz`: t, changed, fail, slack, limit_slack,
  solve time, s) allow an audit of interventions, fallback runs and solve times without new laps; the commands themselves
  were **not** logged, so the size of the command jumps (the discontinuity of an intervention) is not in them.
* Phases A–E did not touch safety filters. The proposers here are fixed, trained without any filter (L5), as in L7.

## 1. Question and hypotheses

*(Q1) Does retrying the failed solves of L7's main filter change its completion count – is the negative result of L7
the solver's? (Q2) Does a shield that keeps the policy's action as long as a backup can still recover the car (in a
nominal-model rollout) make the L5 policies complete more or fewer laps than without a filter?*

* **H-F1 (primary):** over the nine final L5 checkpoints, the completed laps on T1 + T2 (of 105) with `shield` minus
  with `none` (d_i, i = 1 … 9) have a median different from zero.
* **H-F2 (primary):** over the nine final checkpoints, the completed laps with `retry` minus with `main` (L7's main
  filter) have a median different from zero.

Both: the exact Wilcoxon signed-rank test of L7's H-B (`scripts/report_l7_partB.py`: zero differences dropped,
mid-ranks, the exact distribution by enumeration), two-sided. **Holm over H-F1 and H-F2** at α 0.05, each also reported
against 0.025. No direction is predicted; the direction is reported with the test. With n = 9 the two-sided p is
0.0039, 0.0078, 0.0117, 0.0195, 0.027 for W⁺ (or W⁻) = 0 … 4 – the smaller Holm level 0.025 needs W⁺ ≤ 3 or W⁻ ≤ 3.

## 2. Sets

* **Test: T1 + T2**, L7's test sets (`experiments/l7.cases`, unchanged): T1 = the nominal car and the 24 cars of
  `sample_plants(nominal, 24, seed 0)` × steering lag τ ∈ {0, 30, 60 ms} (75 laps); T2 = the nominal car with the L3
  disturbances, seeds 0–29 (30 laps). They are reused **so that `none` and `main` are L7's** (section 3). They are not new
  data: their aggregate L7 results are known to the author of this protocol. Mitigation: no parameter of any phase-F
  condition is chosen on T1 / T2 (the shield's one tuned parameter on FV, section 4; everything else fixed below).
* **Tuning: FV** (new): `sample_plants(nominal, 10, seed 351)[1:]` × τ ∈ {0, 30, 60 ms} (30 laps) and the nominal car
  with the L3 disturbances, seeds 7500–7509 (10 laps) – 40 laps, built as L7's calibration set K.
* **Development: FD** (new, development only): `sample_plants(nominal, 3, seed 359)[1:]` × τ ∈ {0, 30, 60 ms} (9 laps)
  and disturbance seeds 7590–7592 (3 laps); and **NOM** (the nominal car, τ = 0, no disturbances).

(Seeds checked against those in use: car seeds 0–10, 99 and 301–399; disturbance seeds 0–29, 2000–6029 (L7, L8),
7400–7429 (phase A) and 9999.)

## 3. Proposers and conditions

**Proposers** (as L7 Part B): the nine final L5 checkpoints `A_s1` … `C_s3` (`results/L5/multiseed/<g>_s<k>.zip`; group C
with `steer_mode="lateral"`), the selected `B_s1_500k` (`models/L5/`) and the bare tracker (`tracker`, no policy). The
primary analyses use the nine finals (the unselected distribution); `B_s1_500k` and `tracker` are reported descriptively.
**Loop:** L7's (`experiments/l7.run_lap`: one flying lap from the plan's start, 120 s limit, the environment's off-track
and spin criteria, `RacingEnv` on the 0.95 plan with history 8, the policy acting every 50 ms through the L1 tracker at
10 ms).

| condition | what | laps on T1 + T2 | run |
|---|---|---|---|
| `none` | no filter | 11 × 105 | **re-run** at the phase-F commit (~5 s per lap): must reproduce L7's `rl_none.jsonl` lap by lap (outcome, and the failure position within 0.013 m as L7's check of L5) – the regression check; a mismatch is reported and phase F's own records are used |
| `main` | L7's main filter, unchanged | 11 × 105 | **L7's records** (`results/L7/partB_filtered/rl_main.jsonl`, run at bb420e5; 82.8 CPU hours), with a **reproduction check** (below) |
| `retry` | `main` with `FrenetMPC(retry=True)`: a failed solve is retried twice with a second IPOPT instance (cold multipliers, default barrier start, twice the iterations) – from the shifted plan, then from a rollout of its inputs (the pre-L7 option, docs/prep_robustness.md); if all fail, the shifted plan as before | 11 × 105 | new |
| `shield` | the backup shield of section 3.1 | 11 × 105 | new |
| `backup` | the shield's backup alone: the bare tracker with the action a_b at every decision | 105 | new |

**Reproduction check of `main`:** 22 laps – for each of the 11 proposers two of the 105 cases, drawn with
`numpy.random.default_rng(0)` (proposers in the order above, `choice(105, 2, replace=False)` each) – re-run at the
phase-F commit with command logging (section 5). Rule, fixed now: if all 22 have L7's outcome (completed / stop reason),
completed laps' times within 0.01 s and failures' positions within 1 m, L7's `main` records are used; otherwise `main` is
re-run in full at the phase-F commit (~5 h on 20 workers), that run is used, and the mismatch is reported.

`retry` changes only the failure handling – it isolates mechanism (i) of L7. A second fallback (braking within the
friction left, `fallback="brake"`) is not tested (one change at a time).

### 3.1 The shield (MPS, F.1; gatekeeper's margins, F.3)

At every 50 ms decision, with what the tracker sees – the measured state y (with T2's measurement noise; on the lag cars
the observed state with the commanded steering angle, as the tracker), a copy of the tracker's internal state, the nominal
car `DynVehicle()` without lag, the track:

1. The policy proposes its action a = π(obs) (steering offset, speed factor; the bare tracker proposes a = 0).
2. **Proposal step:** the tracker with action a drives the nominal model (`DynamicSim`'s RK4 step and `apply_limits`,
   10 ms) for one decision (5 steps) from y, giving y₁.
3. **Backup rollout:** from y₁ the tracker with the backup action **a_b = (0, −1)** – steering offset 0, speed factor
   0.85 (the most cautious action of the policy's own action space) – for **T_b = 3 s** (300 steps).
4. **Recoverable** if (i) at every step of 2 and 3 both axle margins (the environment's `FastMargins` of the nominal car:
   the front and rear axle points to the track edges – the environment's off-track criterion) are at least **m**, the
   side-slip |β| ≤ 0.3 rad (the environment's spin limit is 0.5) and v_x ≥ 5 m/s; and (ii) the last state is **settled**:
   the tracker's lateral error |e| ≤ 1.0 m, its course error |Δψ| ≤ 0.1 rad and v_x ≤ 0.85 v_plan(s) + 2 m/s.
5. Recoverable → the policy's action is applied. Not recoverable → **a_b is applied** for this decision (an
   intervention); the next decision is checked anew (no hysteresis – chattering is measured, section 5). At an
   intervention the shield also tests whether the backup from y itself (3.05 s) passes the same test; if not, the
   decision is counted as **"no recoverable option"** (a state outside the monitor's set: F.1's induction does not hold
   there, a_b is applied anyway).

**Tuned:** m ∈ {0.25, 0.5, 1.0} m (section 4). **Fixed now:** a_b, T_b = 3 s (the braking from 1.05 to 0.85 of the plan
speed should take well under 1 s at this car's deceleration – not measured; the rest is the tracker settling), the settled thresholds, |β| ≤
0.3, v_x ≥ 5 m/s.

**What it is and is not.** F.1's guarantee needs an exact model; this shield checks with the nominal model (no lag,
nominal parameters, no disturbances), so on these cars it is a **heuristic shield with a margin m**, not a certified one.
The settled set is **not shown to be invariant** under the backup: that the backup, once settled, keeps the car on the
track is an assumption, checked empirically by `backup` (whole laps) and in development (section 9). **Information
budget:** as L7's main filter (track, plan, measured state, nominal model) plus the tracker the policy acts through – not
an oracle. **Granularity:** the shield replaces the policy's action for one decision (the tracker then runs as usual at
10 ms); L7's filters replaced the first 10 ms command of a decision and held it for the decision – both count as "the
decision changed".

*Clarification of revision 0 (review m1) – the action in the policy's observation:* at an intervention the environment
advances with the **applied** action a_b, and a_b is what the policy's observation records as the last action (the
`RacingEnv` history with `history` > 1 includes the last action) – the closest to a deployment, where the car's own record
holds what was applied. This differs from L7's filters: they replaced the tracker's command below the environment, so the
environment – and the policy's observation – kept the policy's own action.

*Clarification of revision 0 (review m2) – lag:* the proposal step and the backup rollout run on the nominal model
**without the steering lag**, while two thirds of T1 (τ = 30 and 60 ms) and of FV have it. On those cars the shield is
expected to be **optimistic**: a state it judges recoverable may not be recoverable for the real car. Section 5 reports,
by τ, how often an intervention is still followed by a failure within 3 s.

### 3.2 Not in phase F (and why)

* **High-order CBF on the single-track model, robust CBF variants:** L7's CBF-QP already is a second-order (exponential,
  F.8) barrier with a robust margin (b_10) and friction factor (η_10) on a point mass, and it completed 0 of 105 laps for
  every proposer (all at turn 1); on the single-track model the steering rate reaches the lateral position with relative
  degree 3 through saturating tyres – not control-affine (F.8's assumption). **Backup CBF** (F.6) – the smooth
  counterpart of this shield, relative degree 1 by construction – is the follow-up if the shield works but chatters.
* **Reduced-order reachability:** HJ grids are limited to ~6 states (F.10); a point-mass reduction is what failed in L7.
* Training the policies with a filter (Oh et al., L7's literature) – the policies are fixed.
These go to CONTROL_METHOD_MATRIX.md as follow-ups; the phase reports them as not done.

## 4. Tuning (FV only)

`shield` with m ∈ {0.25, 0.5, 1.0}: nine finals × 40 laps × 3 = 1 080 laps; for reference also `none` (9 × 40) and
`backup` (40) on FV. **Rule:** the m with the most completed laps summed over the nine finals; ties → the fewer
interventions per decision (mean over the nine finals' laps); then the larger m (*clarification of revision 0, review n1:* at equal completions and intervention rate, the larger safety
margin is kept – the more cautious choice when the data cannot tell them apart). The choice and all records go to the
reviewer before any T1 / T2 lap of `shield`. `retry`, `none` and the reproduction check of `main` have no parameter; they
may run on T1 + T2 after packet 2's APPROVE, before or alongside the tuning.

## 5. Always reported

Per proposer and condition, on T1 + T2 (and by τ and T2): completed laps; mean lap time of the completed laps and the
lap-time cost on the laps both conditions complete; interventions per decision; **intervention runs** (on-switches per
lap – chattering); the **false-intervention rate** as L7's A8 (at the first intervention at or after each j·L/8 the
episode state is kept and the policy is rolled out without the filter for 3 s; no crash = not needed; 200 per proposer
drawn with seed 0 – an upper bound, as in L7; *clarification of revision 0, review n2:* the **same procedure and code**
for `shield`, `retry` and L7's `main` records – `experiments/l7.rollout_unfiltered` from the episode state kept before the
first intervention at or after each j·L/8, and the sampling of L7's report – so that the rates compare directly); solver fallbacks and runs of ≥ 3 (`main`, `retry`); the attribution of
failures as L7's pb-m2 (after a fallback run / after interventions / neither); the filter's time per decision (mean, P95,
P99, max, the share above 50 ms; with the machine load stated). For `retry`: the attempts per failed solve and the share
each retry recovers. For `shield`: the share of decisions with no recoverable option, the check time, and (*clarification of revision 0,
review m2*) **by τ (0, 30, 60 ms) and for T2, the share of interventions followed by a failure within 3 s** (the car still
left the track or spun within 60 decisions after the intervention – where the nominal check was too optimistic or acted
too late).
**Commands** (proposed and applied first 10 ms command of each decision) are logged for `retry`, `shield`, `backup` and the
reproduction subset of `main`: the jump of the applied command at the onset and end of an intervention.

### 5.1 Audit of L7's records (no new laps; descriptive)

From `results/L7/partB_filtered/` (`main` and `cbf`, 1 155 laps each): A1 interventions along the lap (by eighth),
intervention runs and on-switches per lap; A2 fallback runs (length, position, the share followed by a failure within
3 s); A3 the solve time against the 50 ms decision period – the share of decisions above 50 ms and 100 ms (under L7's
load, ~26 processes on 24 cores): in L7's simulation a filtered command acted at once, so a filter this slow is an
idealisation there (real time is phase G's subject); A4 the CBF-QP's last decisions before its turn-1 failures (slack use,
interventions). The command jumps are not in these records (section 0); they come from the reproduction subset.

## 6. Statistics

Exact signed-rank (n = 9) for H-F1 and H-F2, Holm over the two. Secondary, uncorrected (3): `shield` – `main` and
`retry` – `none` (signed-rank over the nine finals), `shield` – `backup` is not paired by proposer and is reported as
counts. Per proposer, exact McNemar (`shield` vs `none`, `retry` vs `main`) as a description, as L7's table.
`B_s1_500k` and `tracker`: descriptive.

## 7. Second environment

As before (deferred by the user): H-F1 and H-F2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). **No code before its APPROVE.**
2. Code: `experiments/cx_f.py` (FV / FD; the shield as a wrapper of the policy with access to the environment; a lap
   function with the shield's decision log, snapshots and command log; the audit functions); a `retry` option of
   `experiments/l7.NMPCFilter` and a command-log option of `experiments/l7.run_lap`, both off by default (L7's behaviour,
   checked by the regression and the reproduction check); `scripts/run_cx_f.py`, `scripts/report_cx_f.py`,
   `scripts/cx_f_dev.py`; unit tests; the development checks of section 9; review (packet 2) before any FV lap.
3. FV tuning (~1 480 laps; a shield lap ~1–2 min under load: ~2 h on 20 workers); results to the reviewer.
4. T1 + T2 from a pinned worktree (`.venv-rl`, as L7): `none` 1 155 laps (~1.5 CPU h), the `main` reproduction 22 laps
   (~1.6 CPU h), `retry` 1 155 laps (~85–100 CPU h, ~5 h on 20 workers), `shield` 1 155 laps (~25–35 CPU h), `backup` 105;
   the audit (section 5.1); report; review; final review; merge.

## 9. Development checks (planned)

On NOM and FD only (records in `results/CX/F/dev/` with commit and dirty flag):

1. **Monitor:** on NOM the shield accepts the plan's own states (the backup from them settles within 3 s) and rejects a
   state pushed off the line (e.g. 3 m lateral error with a heading error towards the edge); the check time.
2. **Backup alone:** `backup` on NOM and FD – completed?
3. **Shield:** with `B_s1_500k` and `tracker` on NOM and FD at m = 0.5: completions, interventions per decision, the share
   with no recoverable option, the check time.
4. **Retry:** `retry` with `B_s1_500k` on NOM: one lap – fallbacks, attempts.
5. **Gate:** (i) `backup` completes NOM; (ii) `shield` with `B_s1_500k` completes NOM with interventions in fewer than
   half of its decisions; (iii) the shield's mean check time in one process below 50 ms. Otherwise an amendment before
   any FV lap.
6. *Clarification of revision 0 (review m3) – the backup's own reliability:* `backup`'s completions on FD (9 + 3 laps)
   are reported as background, not as a pass condition. If it completes **fewer than half (< 6 of 12)**, packet 2
   discusses a more conservative a_b (e.g. a lower speed factor – below 0.85 it leaves the policy's action space; the
   tracker's speed factor allows it), decided **before any FV result** is seen. The bare tracker at the plan's full speed
   completes few laps on L2 cars (phase A's CXT 1 / 25, phase C's CCT 2 / 50, L7's T1 + T2 6 / 105) – the shield cannot
   protect better than its backup drives.

**Measured before this protocol** (disclosed, no lap): the rollout of step 3 on the nominal car, one process, `.venv-rl`:
11.6 ms per 1 s (100 steps), 33.6 ms per 3 s (300 steps) – about 1 600 checks per lap, ~1 min per lap.

### 9. Results of the development checks (2026-10-22; packet 2) – **the gate failed**

Records (git-ignored) in `results/CX/F/dev/`: `monitor.json` (7634e2d), `laps.jsonl` and `gate.json` (0570b1c),
`backup_plans.jsonl` (0570b1c), `monitor_variants.json` and `variants.jsonl` (4d356df); decision records in
`results/CX/F/dev_dec/` and `devvar_dec/`; each run's commit and dirty flag in `meta_<what>.json` (all clean). Two
diagnostics on NOM were first run as scratch scripts: the axle margins along the plans (reported below), and the
tracker at held speed factors, which `cx_f_dev.py backup-plans` later reproduced with the same outcomes. Numbers of
`B_s1_500k` and `tracker` without a filter on NOM and FD were run alongside as context (`laps.jsonl`, condition `none`).

1. **Monitor (9.1; m = 0.5, NOM):**
   * Pushed states: rejected 60 of 60.
   * The plan's own states: **accepted only 8 of 60** (51 rejected by the margin, 1 by the side-slip).
   * Why: the 0.95 plan itself keeps both axles less than 0.5 m from the edge on 23 % of the lap (its minimum is
     0.27 m). Every plan of the library has its minimum at 0.27–0.29 m. So with m = 0.5 or 1.0 the monitor rejects
     the plan itself, and of the registered grid only m = 0.25 lies below the plans' minimum.
   * Check time: 13.2 ms mean, 37.6 ms max, one process.
2. **Backup alone (9.2): `backup` does not complete NOM** (off track at 3 786 m); FD 5 of 12 (car 2 at every τ,
   disturbance seeds 7590 and 7591).
   * Diagnostic (`backup-plans`): the bare tracker with a held speed factor on the 0.95 plan completes NOM only at
     factor 1.0 (75.57 s). At 0.97 and 0.95 it spins at ~2.77 km; at 0.92, 0.90, 0.87 and 0.85 it leaves the track
     at 2.74–3.79 km.
   * Reading (not tested further): the tracker scales the reference speed and the longitudinal feed-forward by the
     factor but keeps the plan's steering feed-forward, which includes the slip at the plan's lateral acceleration.
     At a lower speed it therefore steers for the wrong lateral acceleration, and its feedback (k_p = 0.06) does not
     make up the difference. The policies vary the factor only locally.
3. **Shield (9.3; m = 0.5):**
   * With `B_s1_500k` and with `tracker` the shield intervenes in 87 % of the decisions, all by the margin, and in
     99 % of these there is no recoverable option. Both laps are therefore the backup's lap: off at 3 786 m on NOM,
     FD 5 of 12 each (the backup's cars).
   * Without the shield, `B_s1_500k` completes NOM (80.82 s) and 12 of 12 FD laps; `tracker` completes NOM only.
4. **Retry (9.4; NOM, `B_s1_500k`):** completed in 82.27 s (`none`: 80.82 s).
   * 0.286 interventions per decision.
   * 2 first attempts failed; one was recovered by each retry; no fallback.
   * Time per decision (20 processes): mean 127 ms, P99 336 ms, max 8.3 s.
   * The 8 false-intervention snapshots: none crashed.
5. **Gate:** (i) **failed**; (ii) **failed**; (iii) passed (13.2 ms). Review m3: `backup` completes 5 of 12 FD laps,
   fewer than 6, so a more conservative backup has to be discussed (below). **No FV, T1 or T2 lap has been run.**

**Development variant for the amendment** (`Shield(backup_eta)`, 4d356df):
* What it is: the backup is the bare tracker on a slower plan of phase D's library (η_b) with that plan's own
  feed-forward. Its command replaces the policy's tracker's at every 10 ms step of an intervened decision; the
  environment keeps the policy's action, as with L7's filters, so review m1's rule would change.
* Grid: η_b ∈ {0.85, 0.79, 0.73} × m ∈ {0.0, 0.1, 0.2}; the plans' axle margins are 0.27–0.29 m at the least.

* **The backup alone** (`backup-plans`, the bare tracker on a plan with zero action), NOM and FD:

  | η of the plan | 0.95 | 0.93 | 0.91 | 0.88 | 0.85 | 0.82 | 0.79 | 0.76 | 0.73 |
  |---|---|---|---|---|---|---|---|---|---|
  | NOM | yes | no (spin 2 747 m) | yes | yes | yes | yes | yes | yes | yes |
  | FD (of 12) | 0 | 1 | 1 | 4 | 6 | 8 | 7 | 8 | 9 |
  | of them τ = 60 ms (of 3) | 0 | 0 | 0 | 1 | 1 | 1 | 0 | 0 | 1 |

  Even the slowest plan leaves 3 of the 12 FD laps uncompleted, and on the 60 ms cars no plan completes more than one
  of three.
* **Monitor:** from the 0.95 plan's own states, the nominal-model rollout into the slower plan is recoverable at only
  **33–41 of 60** points (m 0–0.2). Even on the nominal model, the change of line and speed within 3 s fails at
  about a third of the lap. Pushed states: rejected 60 of 60 in every setting. Check time 28–33 ms in one process.
* **Laps with `B_s1_500k`** (FD of 12; without a filter it completes all 12 and NOM):

  | η_b \ m | 0.0 | 0.1 | 0.2 |
  |---|---|---|---|
  | 0.85 | 5 (NOM 81.25 s) | 4 (NOM 81.40 s) | 3 (NOM 81.38 s) |
  | 0.79 | 7 (NOM 81.91 s) | 7 (NOM 82.05 s) | 4 (NOM off track) |
  | 0.73 | 6 (NOM 82.26 s) | 5 (NOM 82.50 s) | 4 (NOM 83.15 s) |

  * Interventions per decision: 0.04–0.14 (median 85 per lap, range 9–254; 1–133 on-switches per lap).
  * Time per decision under 20 processes: 63–70 ms.
  * The failures (64 laps over the nine settings): in every setting the τ = 60 ms cars fail (0 of 3), and the
    τ = 30 ms cars complete at most 2 of 3.
    * 13 failures come at 33–42 m, all in the m = 0.2 settings of η_b 0.79 and 0.73 (NOM included): the monitor
      rejects the start state at once, and the backup's change from the 0.95 plan's line and speed to its own
      leaves the track within half a second, as the monitor's nominal rollout predicted (no recoverable option).
    * Most of the rest come at 2.5–2.7 km on the lag cars, where the bare tracker fails as well; a few come at turn 1
      (0.33–0.47 km), 1.56 km and 3.3–4.2 km.
    * Of a failed lap's interventions, 5–60 (median 32) fall in its last 3 s.
  * Mechanism (reading): the shield hands the car to a backup that cannot drive these cars exactly where the policy
    can. Review m2's optimism is visible: the nominal check accepts states that the lagged car cannot recover. The
    hand-over itself – a change of line and speed within one decision, with no blending – is part of the problem.
* **Laps with `tracker`:** NOM completed in eight of the nine settings (not with η_b 0.79, m 0.2); FD 0 of 12 in
  every setting (alone: 0 of 12).

**Reading.** With the trackers available here, no cheap backup is reliable on these cars:
* the backup of revision 0 fails even on the nominal car;
* a slower plan with its own feed-forward completes at most 9 of 12 FD laps and almost none of the 60 ms ones;
* from the policy's states it recovers on the nominal model at only about two thirds of the points.

A shield cannot protect better than its backup drives (review m3). Here the shield turned 12 of 12 FD laps of the
selected policy into 3–7.

## Amendment F1 (proposed 2026-10-22, packet 2; **decided: option 1** – see the decision below)

The development gate of section 9 failed for the shield, and review m3's condition holds (`backup` 5 of 12 on FD).
H-F2 (`retry` against `main`) does not depend on the shield, the gate or FV. Options:

1. **(proposed) Close the shield part without the benchmark.**
   * **H-F1 is recorded as *not tested***. The development records and the reading above are the result: a negative
     development result about a tracker-based backup on this car family, not about shields in general.
   * Phase F continues with **H-F2 as its only primary test** (α 0.05, Holm over one), the `none` regression, the
     reproduction check of `main` and the audit of section 5.1. The audit has already run on L7's records at b94e775:
     `results/CX/F/report/audit_l7.*`; it reproduces L7's false-intervention counts and failure attribution.
   * No FV laps; no `shield` or `backup` laps on T1 + T2.
   * The section-5 tables for `retry` stay as registered.
   * Follow-ups (CONTROL_METHOD_MATRIX.md):
     * a backup that drives the lag cars – e.g. L6's lag-aware identified NMPC, which completes 73 of 75 on T1 but
       is too expensive to roll out at every decision, so it would need a cheap surrogate;
     * a backup CBF (F.6), which needs such a backup as well;
     * a filter-aware or filter-trained policy (Oh et al.).
2. **H-F1 with the gate waived:**
   * FV tuning over η_b ∈ {0.85, 0.79, 0.73} × m ∈ {0.0, 0.1, 0.2} (9 × 360 shield laps + `none` and `backup`,
     ~5 h), then `shield` on T1 + T2 (1 155 laps, ~3 h).
   * The development laps predict the direction (fewer completions than `none` for the selected policy). It would
     measure that across the nine finals, as L7's H-B did for its filter.
   * Review m1's rule would be replaced: in the plan mode the environment keeps the policy's action.
3. **Revision 1, a re-design:** a backup reliable on lagged cars (an identified, lag-aware controller with a cheap
   rollout) – a new design and protocol, beyond phase F's scope as registered.

### Decision on amendment F1 (the review of packet 2, 2026-10-22): option 1, with g1–g4

**The shield part of phase F is closed without the benchmark; H-F1 is recorded as *not tested*. H-F2 continues as
registered.** The development records, the gate's result and the audit of section 9 / 5.1 are accepted. Option 2 would
spend ~3 000 FV laps on a direction the development laps already show; option 3 is a new design (a backup that drives
lagged cars), listed as a follow-up.

* **g1 – the scope of the negative result:** it concerns **an MPS shield with the L1 tracker as its backup and a check on
  the nominal model, on these cars** (L2 mismatch, steering lag, L3 disturbances) – not shields or MPS in general. A
  design flaw of section 3.1 / 4, stated as found in 9.1: the 0.95 plan itself keeps both axles closer than 0.5 m to the
  edge on 23 % of the lap (its minimum 0.27 m; every plan of the library 0.27–0.29 m), so of the registered grid only
  m = 0.25 lies below it – **the upper values of the grid were set without knowing this.**
* **g2 – the level of H-F2:** with H-F1 withdrawn, H-F2 is the only primary test and is decided at **α 0.05**; its
  verdict at 0.025 (the level each of the two would have had alongside) is reported too. H-F1 was withdrawn **because
  the development gate failed, before any T1 / T2 result existed** (no T1 / T2 lap of any phase-F condition had run).
* **g3 – what runs, as registered:** on T1 + T2, from a pinned worktree at one commit (RUN_COMMIT, input SHA-256 list,
  timeline): the `none` regression against L7's `rl_none.jsonl` (lap by lap), the reproduction check of `main` (22
  laps, rng 0, the rule of section 3 unchanged: any mismatch → `main` re-run in full) and `retry` (1 155 laps); the
  audit of section 5.1 (already run, b94e775). **No FV lap, no `shield` or `backup` lap.** The tables of section 5 for
  `retry` and `main` as registered; those of `shield` / `backup` fall away.
* **g4 – the code:** `Shield` in both modes (the action a_b of revision 0 and the slower-plan variant `backup_eta`) is
  merged as it is, off unless constructed, its docstring marked *development only; the shield part of phase F was closed
  without the benchmark (amendment F1)*. Follow-ups in CONTROL_METHOD_MATRIX.md: a backup that drives the lagged cars
  (e.g. a cheap surrogate of L6's lag-aware identified NMPC, which completes 73 of 75 T1 laps but is far too expensive to
  roll out at every decision); a backup CBF (F.6) on such a backup; policies trained with the filter they are deployed
  with (Oh et al., L7's literature).

*Reminders for the report (review of packet 2):* section 9.4's `retry` lap is development data – it shows the mechanism
works (two failed first attempts, each recovered by a retry), not H-F2's result; the audit shows every decision of
L7's `main` above 50 ms (97 % above 100 ms) under L7's load, while L7's simulation applied the filtered command at once –
an idealisation to be stated, and a subject of phase G.
