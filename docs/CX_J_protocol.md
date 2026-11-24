# CX-J protocol – robust adaptive MPC: a tube around the adaptive model (control expansion, phase J – frontier 3)

Status: **draft, revision 0 (2026-11-24), for review (packet 1). Not binding; no code of phase J exists.** Written after
the literature check ([CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §J) and before any phase-J code: no
module, test, runner or lap of phase J exists. Branch `cx/J-robust-adaptive` from `main` 85059dc. The merge of phase I is
`main` **70a51f6** (recorded here, as agreed at its final review). The ten commits 70a51f6..85059dc are the README, media
and `race.py` work merged by the user; no file of this phase's scope changed in them.

The user's choice (2026-11-24, translated): *"Robust adaptive MPC."* It comes from the method matrix's phase-D list ("a
tube around the adaptive model (robust adaptive MPC, D.13)"; CX-D review m3).

## 0. What phases C and D left

* **Phase C's `tube`:** the NMPC's constraints tightened by an error tube for the L2 parameter box. 49 of 50 new L2
  cars against `nmpc`'s 16; also 45 of 50 under an unmodelled 30 ms steering lag and 28 of 30 under the L3
  disturbances. It knows the parameter ranges (a favourable prior) and costs +1.61 s.
* **Phase D's `adaptive`:** the model and the plan updated online (RLS). 42 of 50 constant cars, 34 of 50 with the grip
  falling during the lap. It is certainty-equivalent, with no margin.
* **The two are complementary in phase D's records** (completed of 50; every `adaptive` – `tube` comparison is
  uncorrected, and the last two rows are secondary conditions):

  | condition | `adaptive` | `tube` | oracle |
  |---|---|---|---|
  | constant (CDT) | 42 | 46 | 42 |
  | grip falling during the lap (CDD) | 34 | 2 | 44 |
  | measurement noise (CDT_NOISE) | 0 | 33 | 1 |
  | unmodelled 30 ms steering lag (CDT_LAG30) | 6 | 44 | 4 |

* **What this does not tell us.** `tube`'s robustness to noise and lag comes from a margin sized by a fixed prior box:
  neither condition is in its disturbance set. The oracle fails there too (1 and 4 of 50), so adaptation alone does not
  help. If the box is shrunk as the estimate improves, as in the robust adaptive MPC literature, the margin that covers
  noise and lag shrinks with it.

## 1. Question and hypotheses

*Can one controller keep what `adaptive` gains when the grip falls and what `tube` gains under noise and lag, when its
tube is sized online – from the estimate's uncertainty and from the prediction errors it sees – instead of from a fixed
prior box?*

* **H-J1 (primary):** on JD (50 new cars, the grip falling during the lap), `ra` and `adaptive` differ in the number of
  completed laps. Exact McNemar, two-sided.
* **H-J2 (primary):** on JN and JL (the same 50 cars with the measurement noise, and with the steering lag; 100 laps),
  `ra` and `tube` differ in the number of completed laps. Per car the count of completed laps is 0, 1 or 2; exact
  Wilcoxon signed-rank on the per-car differences (mid-ranks, the exact distribution, zero differences dropped),
  two-sided. The car is the unit: the two laps of a car are not independent.

Holm over H-J1 and H-J2 (family-wise α 0.05), each also against 0.025. No direction is predicted.
* **H-J2 can come out against `ra`.** That would mean the online tube does not replace the prior box – a real negative
  result of this phase. It is reported as such.
* "Not significant" is not "no worse". The report gives the discordant counts and the exact 95 % interval of the share
  of discordant laps in `ra`'s favour.
* JT (constant) is secondary: completions and lap time of `ra` against `tube` and `adaptive` (the price of the tube).
* **No composite of the four conditions is a primary.** It would weigh the conditions arbitrarily, and the conditions
  are chosen because one of the two parents is known to fail in each.

**Scope and information budget:** Spielberg, η 0.95, the nominal plan family, `nmpc`'s objective (c 2, margin 0.6),
N = 30 × 50 ms, offline (IPOPT without a time budget), as phases C and D. `ra` gets:
* what `adaptive` gets (plan library, measured state, its own applied inputs, the model structure, L4's bounds);
* the L2 half-widths as a **cap** on its box – the prior information `tube` has, no more;
* the residual bound of section 3.4, computed only from its own past predictions and the measured states.

It does not get the noise level, the lag, the drift or the true car. The adaptive part starts from the nominal car, as
`adaptive` does. `identified` (one exploration lap and a full re-plan; 50 of 50 on phase D's CDT, 0 of 50 under the drift) is
not run: it uses an extra lap, and its own phase-D numbers stand as background only.

## 2. Sets

Seeds 391 and 393 (and 391 000 + car for the noise) are used by no L-layer, by phases A–I or by L8 (checked in src,
scripts, tests and docs at 85059dc).

* **Cars JT** (new): `sample_plants(nominal, 50, seed 391)[1:]`. Four conditions on the same 50 cars:
  * **JT** – constant parameters, no noise, no lag;
  * **JD** – one friction drift per car (drift seed 393, the ranges of phase D: s_on ~ U(600, 2 600) m, ℓ ~ U(100, 1 500)
    m, f_end ~ U(0.75, 0.90));
  * **JN** – the L3 measurement noise only (noise seed 391 000 + car);
  * **JL** – the L6 steering lag of 30 ms (unmodelled).
* **Tuning set JV** (40 laps per setting): phase A's CXV cars (seed 301, 10 cars) in the same four conditions. Drift seed
  334 is phase D's CDV_D, so the drift laps are phase D's tuning cases; the noise seeds are 301 000 + car. These cars were
  used to tune phase D's λ_μ (0.995, kept), so they are background for `ra`'s own tuning, not an independent test.
* **Development set DEV:** phase C's CXDEV cars (seed 399), in the four conditions (drift seed 335; noise seeds
  399 000 + car), and **NOM** in the four conditions. Never tuned on, never tested on.
* **The drift's start, corrected:** a start state at s = L − ε counts as s = 0 (CX-D packet 4): the field is evaluated
  at s = 0 for positions within 10⁻⁶ m below L. This is a new field class; `cx_d.DriftField` is unchanged (phase D's
  records are its).
* Start, laps and time limit as phases A–D (flying start on the 5 % plan, one lap, 121 s).

## 3. Controllers

| controller | model in the NMPC | tube box half-widths around the model | extra disturbance | role |
|---|---|---|---|---|
| `nmpc` | nominal | none | none | baseline (phases A–D) |
| `tube` | nominal | the L2 box (phase C, c_R 1) | none | parent, **comparison** |
| `adaptive` | RLS estimate (phase D, λ_μ 0.995) | none | none | parent, **comparison** |
| **`ra`** | RLS estimate | shrinking (3.3) | residual bound (3.4) | **primary** |
| `ra_fixed` | RLS estimate | the L2 half-widths, always | none | ablation – the plain combination |
| `ra_shrink` | RLS estimate | shrinking | none | ablation – no residual bound |
| `adaptive_oracle` | the true car and friction | none | none | JT and JD only, the ceiling |

### 3.1 The combined problem

`FrenetMPC(param_model=True, param_tightening=True)` with the plan library: the estimate θ̂, the plan coordinate j
(from μ̂, η_p = 0.95 μ̂, as phase D) and the per-stage η_k and back-off b_k are parameters of each solve. The
combination is not implemented today (`nmpc.py` raises); the two options' parameter blocks are laid out in sequence, so
only the rejection and the rows are to be checked (section 9.1). Defaults unchanged: phases C and D reproduce (9.1).
The traction control uses `vehicle_from(θ̂)`, as `adaptive`.

### 3.2 The tube around the estimate (phase C's construction)

As phase C (CX_C 3.1–3.3), with the prediction model at θ̂ instead of the nominal car:
* the linearisation, the time-varying LQR (c_R 1; K is not applied in closed loop) and the reachable-set bounds along the
  previous solution shifted by one stage;
* the disturbance box W_k = the component-wise maximum over the **vertices** of the parameter box Θ̂ = θ̂ ± h (128
  vertices) of |f_θ(z_k, v_k) − f_θ̂(z_k, v_k)|, plus the residual bound d̄ of 3.4;
* the same two tightenings (track back-off, friction factor η_k ≥ 0.70) and caps. The axles' ellipse usage uses θ̂'s
  friction and loads.

The box is clipped to L4's bounds LO / HI. Parameters are handled by vertices, as in phase C: the maximum is the box
maximum only where the mismatch is monotone in each parameter.

### 3.3 The box half-widths h

With σ_i = sqrt(P_ii), the RLS covariance of phase D's estimator (P in the units of θ, after its forgetting and its
clipping to the prior):

  h_i = min(h_i^L2, κ σ_i).

* h^L2 are the L2 half-widths: 0.055 (μ), 0.025 (m), 0.10 (ClA, CdA), 0.15 (I_z, B), 0.03 (aero_front).
* At the start and where the tyres are not excited, P is at the prior and h at its cap. The tube is then as wide as
  phase C's, around θ̂.
* κ is tuned (section 4). It scales a **statistical** standard deviation of an estimator that is not calibrated for
  errors in the measured state or in the model structure. No coverage is claimed (3.6).

`ra_fixed` uses h = h^L2 always. `ra_shrink` is `ra` without 3.4.

### 3.4 The residual bound d̄

A per-component half-width of what the model did not predict, from the controller's own past predictions:
* At each MPC step k ≥ 1 whose previous solve did not fall back, the residual is r_k = |x_k − x̂_{1|k−1}|, the measured
  Frenet state minus the first-stage prediction of the previous solve (seven components).
* Over the last 40 residuals (2 s), d̄ = ρ · (the component-wise 0.9-quantile); with fewer than ten residuals, d̄ = 0.
* d̄ is added to W_k at every stage.

It contains everything the first 50 ms did not predict: parameter error, noise at both ends of the step, the lag, and
the plant's input limits and traction control. It is **not** a disturbance bound:
* the residuals are not exchangeable over a lap;
* a 0.9-quantile with a 2 s window covers about 90 % of the past only;
* the noise of the next solve's initial state is part of r_k but is not a process disturbance.

### 3.5 What the theory asks and what is done instead

The published robust adaptive schemes (literature §J) assume parameters that enter affinely, a set-membership or
non-falsified parameter set, an incremental Lyapunov function or a contraction metric, and exact states. None holds
here, so this is an approximation of the structure:

| theory | here |
|---|---|
| parameters enter affinely (D.13, C.5) | B enters inside the tyre curve, not affinely; D.13 itself lists nonlinearly parametrised systems as open |
| a parameter set containing θ* (set membership, non-increasing, hypercube) | a confidence box from the RLS covariance: no containment is claimed, and the box grows and shrinks (forgetting) |
| an offline V_δ / CCM and a scalar tube size | the linearised reachable sets of phase C |
| an additive disturbance bound d̄ given | estimated online from residuals |
| exact state, or an observer with an error bound (output feedback, J.5) | the measured state is used as it is; no state filter; noise is in d̄ |
| time-varying parameters by a dilation of the set (J.3, J.4) | forgetting of the RLS (λ_μ 0.995); the dilation is not modelled |

### 3.6 What is and is not claimed

* No recursive feasibility, no guaranteed constraint satisfaction, no coverage statement. The tube is an approximate
  constraint tightening, as phase C's.
* Under noise the RLS is biased (phase D: μ̂ − μ +0.082 on CDT_NOISE); the box is around the biased estimate. Nothing
  here corrects it.
* The plan coordinate uses μ̂ itself, without a margin, so that the tube is not counted twice.
* The caps (b_max, η ≥ 0.70) can remove robustness. The share of capped stages is reported.
* The adaptation adds the model update only (certainty equivalence in the plan); no dual control.

## 4. Tuning (JV only, one lap per car, condition and setting)

* **Grid:** κ ∈ {1, 2, 3}, ρ ∈ {0.5, 1, 2} – 9 settings × 40 laps = 360 laps. `ra_fixed` and the parents are not tuned
  (`adaptive` λ_μ 0.995 and `tube` c_R 1, as selected in phases D and C); `ra_shrink` uses `ra`'s κ.
* **Rule** (phases A–D): the most completed JV laps (all conditions together); ties by the mean lap time on the cases
  every tied setting completes (skipped if none); then the setting nearest the middle (κ 2, ρ 1); then the smaller κ,
  then the smaller ρ. Different lap counts stop the selection.
* The choices and all JV records go to the reviewer before any JT / JD / JN / JL lap (packet 2).

## 5. Always reported

As phases C and D, per controller: completed laps (exact 95 % intervals), lap time, distance to failure, stop reason;
offsets; constraint use and slacks; tyre usage; control effort; NMPC time (mean, P95, P99, max), non-converged solves,
fallbacks. In addition:
* for the adaptive controllers, phase D's adaptation metrics (μ̂ error, too-fast share, settling);
* for the `ra` controllers, per lap: h_μ and each h_i (P5 / P50 / P95), the share of steps at the cap, d̄ (P50 / P95 per
  component), the back-off and η_k (P5 / P50 / P95), the capped shares; σ_i against the true error |θ̂_i − θ_i| at the
  lap end, and the share of steps with |θ̂_i − θ_i| ≤ h_i (a description, not a calibration claim);
* the coverage diagnostic of phase C (the realised lateral error against the back-off);
* the tube's time per solve.

## 6. Statistics

Exact tests as section 1; exact 95 % intervals. Holm over H-J1 and H-J2.

Secondary, uncorrected (the number is stated):
* JT, completions: McNemar `ra` – `adaptive`, `ra` – `tube`, `ra` – `nmpc` (3);
* JT, lap time on the cars both complete: signed-rank `ra` – `adaptive`, `ra` – `tube` (2);
* JD, completions: `ra` – `tube`, `ra` – `adaptive_oracle` (2);
* JN and JL, completions: `ra` – `adaptive` per condition (2);
* the ablations: `ra` – `ra_fixed` and `ra` – `ra_shrink`, McNemar on each of the four sets (8);
* `ra_fixed` – `ra_shrink` and everything else: description only.

## 7. Second environment

As before (deferred by the user): H-J1 and H-J2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). **No code before its APPROVE.**
2. Code:
   * `FrenetMPC(param_model=True, param_tightening=True)`: the rejection removed, defaults unchanged;
   * `control/radaptive.py`: the parameterised tube, the box, the residual bound, the controller;
   * `experiments/cx_j.py`: sets, the corrected drift field, controllers, lap;
   * `scripts/run_cx_j.py`, `scripts/report_cx_j.py`, `scripts/cx_j_dev.py`;
   * unit tests; the development checks of section 9.
3. Packet 2: the code, tests, development checks and the gate; then the tuning records. No JT / JD / JN / JL lap before its
   APPROVE.
4. The test laps from a pinned worktree. Laps: 6 controllers × 200 (JT, JD, JN, JL) = 1 200, plus `adaptive_oracle` on JT
   and JD (100), plus NOM. About 5 min a lap; single-threaded BLAS, one process per job, 20 workers → about 6 h. The
   tuning is 360 laps, about 1.5 h. Estimates from phases C and D; the overrun is noted.
5. Report (packet 3), review, then the final review and the merge.

If a development check fails, the phase closes without a benchmark unless the review accepts an amended design (section 9).

## 9. Development checks (planned; none has run)

On NOM and DEV only (records in `results/CX/J/dev/` with commit and dirty flag).

1. **Code, against the parents.**
   * `param_model` with `param_tightening`: at η_k = η, b = 0, θ̂ nominal and j at 0.95, one cold solve from four states
     equals `FrenetMPC(param_model=True)`'s, and the combined solve with a vertex car and a tightening equals
     `FrenetMPC` built with that car and the same η_k and b_k (10⁻⁸ in the input and the prediction).
   * With the options off, phases C's and D's problems are unchanged: `tube` and `adaptive` reproduce one NOM record each
     (laps to the last digit).
   * The tube at θ̂ nominal, h = h^L2, d̄ = 0 equals phase C's `Tube` (back-off and η_k to 10⁻⁹).
   * Finite differences of the parameterised Jacobians.
   * The corrected drift field: f(L − 10⁻⁷) = 1; f at L − 10⁻³ = f_end.
   * The residual bound on synthetic predictions; a fallback is skipped; fewer than ten gives 0.
2. **The box in exact data** (replays of phase D's estimator on the DEV laps, no selection): σ_i and |θ̂_i − θ_i| along
   the laps – whether κ σ_i is ever below the cap, and how often |θ̂_i − θ_i| ≤ κ σ_i for κ 1, 2, 3. If κ σ_i stays at the
   cap for all κ, `ra` is `ra_fixed` with d̄ and the shrinking is empty; reported before the tuning.
3. **Timing:** the parameterised tube per solve, next to phase C's 11.5 ms, and the NLP.
4. **Development laps** (NOM and DEV in the four conditions: 6 cars × 4 = 24 laps each): `nmpc`, `tube`, `adaptive`,
   `ra` (κ 2, ρ 1), `ra_fixed`, `ra_shrink`.
   * **Gate.** All three:
     1. `ra` completes NOM in the constant condition.
     2. On the ten constant and drift laps of DEV, `ra` completes at least `adaptive`'s number minus one.
     3. On the ten noise and lag laps of DEV, `ra` completes at least half of `tube`'s number (rounded up).
   * **If the gate fails, the phase closes without a benchmark by default.** A new design needs a new revision, reviewed
     before any test lap. Diagnostic variants may be run, as diagnostics only; no other setting may be tried to pass.
   * The gate is a coarse check on few cars; it says nothing about H-J1 or H-J2.
5. **The drifts drawn for JD**, reported before any JV lap: s_on, ℓ, f_end distributions, the number of cars with
   μ · f_end < 0.70, and the shares of drifts that end before ~2.7 km and ~3.83 km.
