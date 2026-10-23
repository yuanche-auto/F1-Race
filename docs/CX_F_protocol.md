# CX-F protocol – safety filters around the L5 policies: L7's filter audited, retried, and a backup shield (control expansion, phase F)

Status: **revision 0, proposed for review (packet 1, 2026-10-22)**. Written after the literature check
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
interventions per decision (mean over the nine finals' laps); then the larger m. The choice and all records go to the
reviewer before any T1 / T2 lap of `shield`. `retry`, `none` and the reproduction check of `main` have no parameter; they
may run on T1 + T2 after packet 2's APPROVE, before or alongside the tuning.

## 5. Always reported

Per proposer and condition, on T1 + T2 (and by τ and T2): completed laps; mean lap time of the completed laps and the
lap-time cost on the laps both conditions complete; interventions per decision; **intervention runs** (on-switches per
lap – chattering); the **false-intervention rate** as L7's A8 (at the first intervention at or after each j·L/8 the
episode state is kept and the policy is rolled out without the filter for 3 s; no crash = not needed; 200 per proposer
drawn with seed 0 – an upper bound, as in L7); solver fallbacks and runs of ≥ 3 (`main`, `retry`); the attribution of
failures as L7's pb-m2 (after a fallback run / after interventions / neither); the filter's time per decision (mean, P95,
P99, max, the share above 50 ms; with the machine load stated). For `retry`: the attempts per failed solve and the share
each retry recovers. For `shield`: the share of decisions with no recoverable option, the check time.
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

**Measured before this protocol** (disclosed, no lap): the rollout of step 3 on the nominal car, one process, `.venv-rl`:
11.6 ms per 1 s (100 steps), 33.6 ms per 3 s (300 steps) – about 1 600 checks per lap, ~1 min per lap.
