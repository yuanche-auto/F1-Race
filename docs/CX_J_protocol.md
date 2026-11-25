# CX-J protocol – robust adaptive MPC: a tube around the adaptive model (control expansion, phase J – frontier 3)

Status: **binding, revision 0 (2026-11-24; ab83afb, APPROVE – packet 1b)** with the clarifications of its review (m1, m2, n1–n3,
marked *"clarification of revision 0"*), written down before any phase-J code. No module, test, runner or lap of phase J existed.
The first version (2187b53) was reviewed as packet 1: CHANGES_REQUESTED (M1, M2, m1–m5, n1, n2). Its points are worked into this
version; the calculations behind M1 and M2 were repeated independently with phase C's existing code (section 3.7). Written after
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
* "Not significant" is not "no worse": it may not be read as `ra` keeping `tube`'s robustness. There is no non-inferiority
  claim; none was set in advance. The report gives, for H-J2, the number of **cars** that `ra` completes more laps of than
  `tube`, fewer, and as many; and the exact 95 % interval of the share of improved cars among the changed ones. For H-J1 the
  same with the laps of JD (one per car).
* JT (constant) is secondary: completions and lap time of `ra` against `tube` and `adaptive` (the price of the tube).
* **No composite of the four conditions is a primary.** It would weigh the conditions arbitrarily, and the conditions
  are chosen because one of the two parents is known to fail in each.

**Scope and information budget:** Spielberg, η 0.95, the nominal plan family, `nmpc`'s objective (c 2, margin 0.6),
N = 30 × 50 ms, offline (IPOPT without a time budget), as phases C and D. `ra` gets:
* what `adaptive` gets (plan library, measured state, its own applied inputs, the model structure, L4's bounds);
* the L2 ranges as a **symmetric cap** on its box (3.3). This is the same prior knowledge as `tube`'s, but not the same box:
  `tube`'s box is centred on the L2 ranges, not on the nominal car; `ra`'s starts at the nominal car and is at least as wide
  in every direction;
* the residual terms of section 3.4, computed only from its own past predictions and the measured states.

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
| **`ra`** | RLS estimate | shrinking (3.3) | residual terms (3.4) | **primary** |
| `ra_fixed` | RLS estimate | the cap, always | none | ablation – the plain combination |
| `ra_fixed_res` | RLS estimate | the cap, always | residual terms | ablation – residual terms without shrinking |
| `ra_shrink` | RLS estimate | shrinking | none | ablation – shrinking without residual terms |
| `adaptive_oracle` | the true car and friction | none | none | JT and JD only, the ceiling |

The four `ra` kinds are a 2 × 2: box {cap, shrinking} × residual terms {off, on} (review m1). `ra_fixed_res` uses `ra`'s selected
residual settings; `ra` was selected with them, which favours `ra` (stated where it is reported).

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
  vertices) of |f_θ(z_k, v_k) − f_θ̂(z_k, v_k)|, plus the per-stage residual term d_s of 3.4;
* the initial state-error box e₀ of 3.4, propagated through the closed-loop transitions (phase C has e₀ = 0): every output's
  bound at stage k gets Σ_i |(Cₒ Φ_{k,0})_i| e₀,i in addition;
* the same two tightenings (track back-off, friction factor η_k ≥ 0.70) and caps. The axles' ellipse usage uses θ̂'s
  friction and loads.

The box is clipped to L4's bounds LO / HI. Parameters are handled by vertices, as in phase C: the maximum is the box
maximum only where the mismatch is monotone in each parameter.

### 3.3 The box half-widths h

With σ_i = sqrt(P_ii), the RLS covariance of phase D's estimator (P in the units of θ, after its forgetting and its
clipping to the prior), and κ = 2 (fixed):

  h_i = min(c_i, κ σ_i),   Θ̂ = θ̂ ± h.

* **The cap (review M1).** c_i is the largest distance of the L2 range of parameter i from the nominal car: 0.08 (μ: the L2
  range is 0.92–1.03), 0.05 (m: 1.00–1.05), 0.10 (ClA, CdA), 0.15 (I_z, B), 0.03 (aero_front). The box is symmetric around
  θ̂. At θ̂ nominal and h = c it **contains** C's box, which is asymmetric (μ −0.08/+0.03, m 0/+0.05). No L2 car lies outside
  it at the start; with C's half-widths (0.055, 0.025) around the nominal car, 61 % of the L2 cars would (section 3.7).
* At the start P is at its prior standard deviation ((HI − LO)/4), which is above c in every parameter, so h = c: the tube starts
  as wide as C's box or a little wider (section 3.7). Where the tyres are not excited P returns to the prior, and so does h.
* κ is **not** tuned: 2 is two standard deviations, fixed to limit the tuning. P is a **statistical** covariance of an estimator
  that is not calibrated for errors in the measured state or in the model structure. No coverage is claimed (3.6).

`ra_fixed` and `ra_fixed_res` use h = c always.

*Clarification of revision 0 (packet-1b review m2) – stage 0.* Φ_{0,0} = I, so a non-zero e₀ would give stage 0 a bound that C does
not have. Decided: **stage 0 is not tightened, as in C** – η_0 = η, the back-off's stage 0 is unused, and the cap shares leave
stage 0 out. Its constraints depend on the measured state, which is given, so tightening them could only make them violated.
e₀ enters from stage 1. At e₀ = 0 and d_s = 0 the tube must equal C's (9.1).

### 3.4 The residual terms

What the model did not predict, from the controller's own past predictions:
* At each MPC step k ≥ 1 whose previous solve did not fall back, the residual is r_k = |x_k − x̂_{1|k−1}|, the measured
  Frenet state minus the first-stage prediction of the previous solve (seven components).
* q is the component-wise 0.9-quantile of the last 40 residuals (2 s); with fewer than ten residuals, q = 0.
* **An initial state error e₀ = ρ₀ q** (review M2, literature J.5): measurement noise makes the state the NMPC starts from wrong
  once, so it belongs in the initial error, propagated along the horizon, not in a disturbance repeated at every stage.
* **A per-stage term d_s = ρ_s q**, added to W_k: an error of the model that persists through the horizon, such as the lag or
  the estimate's error.
* **The residuals cannot tell the two apart.** ρ₀ and ρ_s are chosen by tuning (section 4), with 0 allowed for both.
* *Clarification of revision 0 (packet-1b review n2) – a feedback that is not analysed:* a tighter tube changes the trajectory,
  which changes the residuals and so q. No claim is made about its stability.

The first version put everything into the per-stage term with ρ ∈ {0.5, 1, 2}. The packet-1 review showed, and section 3.7
confirms, that a zero-mean noise repeated with one sign over 30 stages inflates the tube to its caps: the mean back-off at stage
10 goes from 0.21 m (C's box) to 1.0 m at ρ_s 0.5 and 1.8 m at ρ_s 1, with η at 0.70 on 58 % and 69 % of the stages.

It is **not** a disturbance bound:
* the residuals are not exchangeable over a lap;
* a 0.9-quantile with a 2 s window covers about 90 % of the past only;
* it contains parameter error, noise, lag and the plant's input limits and traction control together.

### 3.5 What the theory asks and what is done instead

The published robust adaptive schemes (literature §J) assume parameters that enter affinely, a set-membership or
non-falsified parameter set, an incremental Lyapunov function or a contraction metric, and exact states. None holds
here, so this is an approximation of the structure:

| theory | here |
|---|---|
| parameters enter affinely (D.13, C.5) | B enters inside the tyre curve, not affinely; D.13 itself lists nonlinearly parametrised systems as open |
| a parameter set containing θ* (set membership, non-increasing, hypercube) | a confidence box from the RLS covariance: no containment is claimed, and the box grows and shrinks (forgetting) |
| an offline V_δ / CCM and a scalar tube size | the linearised reachable sets of phase C |
| an additive disturbance bound given | estimated online from residuals, split into an initial and a per-stage part by tuning |
| exact state, or an observer with an error bound (output feedback, J.5) | the measured state is used as it is, with an initial state-error box e₀ from the residuals; no state filter, no bound |
| time-varying parameters by a dilation of the set (J.3, J.4) | forgetting of the RLS (λ_μ 0.995); the dilation is not modelled |

### 3.6 What is and is not claimed

* No recursive feasibility, no guaranteed constraint satisfaction, no coverage statement. The tube is an approximate
  constraint tightening, as phase C's.
* Under noise the RLS is biased (phase D: μ̂ − μ +0.082 on CDT_NOISE); the box is around the biased estimate. Nothing
  here corrects it. The box is ±0.08 wide only at the cap, so the bias can exceed it once the box has shrunk.
* The plan coordinate uses μ̂ itself, without a margin, so that the tube is not counted twice.
* The caps (b_max, η ≥ 0.70) can remove robustness. The share of capped stages is reported.
* The adaptation adds the model update only (certainty equivalence in the plan); no dual control.

### 3.7 What was calculated before this version (existing code only; no phase-J code)

With phase C's `Tube` (`cx_c.build`, c_R 1, nominal parameters, NOM) at 12 points of the 5 % plan (one cold solve each), and a
first-stage residual from the L3 noise alone (Frenet approximation of its standard deviations, 4 000 draws); scratch
scripts, not in the repository. They say nothing about closed-loop laps.

| tightening from | back-off at stage 10 | at stage N | share of stages at η 0.70 | mean η |
|---|---|---|---|---|
| C's box | 0.21 m | 0.37 m | 0.30 | 0.830 |
| the symmetric cap (3.3) | 0.23 m | 0.40 m | 0.33 | 0.826 |
| C's box + per-stage residual, ρ_s 0.5 / 1 | 1.00 / 1.79 m | 1.58 / 2.79 m | 0.58 / 0.69 | 0.769 / 0.745 |
| C's box + initial error only, ρ₀ 1 | 0.28 m | 0.37 m | 0.36 | 0.817 |
| 0.1 × C's box (a box shrunk to a tenth) | 0.02 m | 0.04 m | 0.00 | 0.933 |
| the same + initial error, ρ₀ 2 / 4 | 0.15 / 0.29 m | 0.04 / 0.05 m | 0.09 / 0.20 | 0.878 / 0.850 |
| the same + per-stage, ρ_s 0.1 / 0.2 | 0.18 / 0.34 m | 0.28 / 0.52 m | 0.06 / 0.28 | 0.869 / 0.830 |

* With the half-widths 0.055 and 0.025 around the nominal car, 22.5 % of 100 000 L2 cars have μ below 0.945, 50.2 % a mass
  above 1.025, and 61.4 % at least one factor outside; with the symmetric cap, none.
* **A shrunk box has almost no margin left:** η 0.933 against 0.830 for C's box. At least one of the residual terms must carry
  the margin (ρ₀ or ρ_s above zero – each alone gives some), or `ra` has the certainty-equivalent controller's margin under noise
  and lag. This is the question of H-J2.
* *The packet-1b review re-computed the table at its own 12 points* (sizes and order agree) *and the two grid settings that
  combine both terms:* 0.1 × C + ρ₀ 2, ρ_s 0.1: 0.39 / 0.28 m, 0.44, 0.787; + ρ₀ 4, ρ_s 0.2: 0.74 / 0.53 m, 0.65, 0.762. Under
  pure noise the grid's middle is about as tight as C's box. No laps.
* The figures of the review (a slightly different set of points) agree in size.

## 4. Tuning (JV only, one lap per car, condition and setting)

* **Grid:** ρ₀ ∈ {0, 2, 4} × ρ_s ∈ {0, 0.1, 0.2} – 9 settings × 40 laps = 360 laps. κ = 2 is fixed. `ra_fixed`, `ra_shrink` and
  the parents are not tuned (`adaptive` λ_μ 0.995 and `tube` c_R 1, as selected in phases D and C); `ra_fixed_res` uses
  `ra`'s ρ₀ and ρ_s. ρ₀ = ρ_s = 0 is `ra_shrink`, so "no residual term" can be selected; if it is, `ra` and `ra_shrink` are the
  same and the report says so.
* **Before the tuning, replays** (9.2) report q and, for each setting of the grid, the shares of stages at the caps per
  condition on the recorded DEV laps.
* **Rule** (phases A–D): the most completed JV laps (all conditions together, so the four conditions have equal weight – a
  weighting used for tuning only, and the reason there is no composite primary; review n2); ties by the mean lap time on the
  cases every tied setting completes (skipped if none); then the setting nearest the middle (ρ₀ 2, ρ_s 0.1); then the smaller
  ρ₀, then the smaller ρ_s. Different lap counts stop the selection.
* The choices and all JV records go to the reviewer before any JT / JD / JN / JL lap (packet 2).

## 5. Always reported

As phases C and D, per controller: completed laps (exact 95 % intervals), lap time, distance to failure, stop reason;
offsets; constraint use and slacks; tyre usage; control effort; NMPC time (mean, P95, P99, max), non-converged solves,
fallbacks. In addition:
* for the adaptive controllers, phase D's adaptation metrics (μ̂ error, too-fast share, settling);
* for the `ra` controllers, per lap: h_μ and each h_i (P5 / P50 / P95), the share of steps at the cap, q, e₀ and d_s (P50 /
  P95 per component), the back-off and η_k (P5 / P50 / P95), the capped shares; σ_i against the true error |θ̂_i − θ_i| at the
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
* the ablations: `ra` – `ra_fixed`, `ra` – `ra_shrink` and `ra` – `ra_fixed_res`, McNemar on each of the four sets (12);
* everything else, including the comparison of the ablations with each other: description only.

## 7. Second environment

As before (deferred by the user): H-J1 and H-J2 are marked as not cross-checked.

## 8. Compute and order

1. Review of this protocol and the literature entries (packet 1). **No code before its APPROVE.**
2. Code:
   * `FrenetMPC(param_model=True, param_tightening=True)`: the rejection removed, defaults unchanged;
   * `control/radaptive.py`: the parameterised tube, the box, the residual terms, the controller;
   * `experiments/cx_j.py`: sets, the corrected drift field, controllers, lap;
   * `scripts/run_cx_j.py`, `scripts/report_cx_j.py`, `scripts/cx_j_dev.py`;
   * unit tests; the development checks of section 9.
3. Packet 2: the code, tests, development checks and the gate; then the tuning records. No JT / JD / JN / JL lap before its
   APPROVE.
4. The test laps from a pinned worktree. Laps: 7 controllers × 200 (JT, JD, JN, JL) = 1 400, plus `adaptive_oracle` on JT
   and JD (100), plus NOM. About 5 min a lap; single-threaded BLAS, one process per job, 20 workers → about 7 h. The
   tuning is 360 laps, about 1.5 h. Estimates from phases C and D; the overrun is noted.
5. Report (packet 3), review, then the final review and the merge.

If a development check fails, the phase closes without a benchmark unless the review accepts an amended design (section 9).

## 9. Development checks (planned; none has run)

On NOM and DEV only (records in `results/CX/J/dev/` with commit and dirty flag).

1. **Code, against the parents.**
   * `param_model` with `param_tightening` (review m4): the **constraint and objective values at the same point** (four states,
     random points z within the variable bounds, a vertex car, a tightening) equal those of `FrenetMPC` built with that car and the same η_k
     and b_k, after the known shift of the slip rows (the parameterised form writes ap·(T + 1), the other form ap·T with the
     bound), to 10⁻⁹. Cold solves are compared only to IPOPT's tolerance (the applied input and the prediction differ by
     less than 10⁻³ in scaled units; reported).
   * With the options off, phases C's and D's problems are unchanged: `tube` and `adaptive` reproduce one NOM record each
     (laps to the last digit).
   * **The tube at θ̂ nominal with phase C's box** (passed as an explicit lower and upper box, not ± h), e₀ = 0 and d_s = 0 equals
     phase C's `Tube` (back-off and η_k to 10⁻⁹). With the symmetric cap instead, the back-off and the friction tightening
     are at least as large at every stage (a superset).
   * Finite differences of the parameterised Jacobians.
   * The corrected drift field: f(L − 10⁻⁷) = 1; f at L − 10⁻³ = f_end.
   * The initial-error propagation against an explicit reachable set on random box initial errors; the residual terms on
     synthetic predictions; a fallback is skipped; fewer than ten residuals give 0.
2. **The box and the residual terms before any tuning** (replays of phase D's RLS on recorded laps, no selection). The laps are
   `nmpc`, `tube` and `adaptive` on the DEV cars and NOM in **all four conditions**, including the noise laps, where the estimate is
   biased and |θ̂ − θ| ≤ κσ is informative. *Clarification of revision 0 (packet-1b review m1):*
   * these laps do not exist yet (phase D's DEV records have no `tube`, no noise and no lag laps, and no predictions). They are
     driven first, with the existing controllers, recording every solve's first-stage prediction (all solves) and the whole
     prediction (every tenth solve); they are also the reference laps of 9.4;
   * the q of `nmpc` and `tube` come from the nominal model, not from `ra`'s θ̂, and `ra`'s own q in closed loop will differ, so
     the replays are descriptive;
   * **order:** replays → the gate laps (9.4) → packet 2 → tuning. After the replays the grid and the gate's setting are not changed
     without a revision;
   * the share of capped stages is computed with the stage-0 rule of 3.3;
   * σ_i and |θ̂_i − θ_i| along the laps, whether 2σ_i is ever below the cap, and how often |θ̂_i − θ_i| ≤ 2σ_i;
   * q per component and condition; for each grid setting the share of stages at the caps.
   * **If 2σ_i is at the cap on more than 95 % of the steps of every lap, the shrinking is empty**, `ra` and `ra_fixed_res` are
     almost the same problem and `ra_shrink` and `ra_fixed` too (*clarification of revision 0, packet-1b review n1:* the remaining
     5 % of the steps let the closed loops diverge). Decided now (review n1): the phase goes on as
     registered, without a new review, and the reports and the reading say so. H-J1 and H-J2 stay on `ra`.
3. **Timing:** the parameterised tube per solve, next to phase C's 11.5 ms, and the NLP.
4. **Development laps** (NOM and DEV in the four conditions: 6 cars × 4 = 24 laps each): `nmpc`, `tube`, `adaptive`,
   `ra` (the middle of the grid: ρ₀ 2, ρ_s 0.1), `ra_fixed`, `ra_fixed_res`, `ra_shrink`. The gate is judged on this `ra`
   setting only; no other setting may be tried in order to pass it.
   * **Gate.** All three:
     1. `ra` completes NOM in the constant condition.
     2. On the ten constant and drift laps of DEV, `ra` completes at least `adaptive`'s number minus one.
     3. On the ten noise and lag laps of DEV, `ra` completes at least half of `tube`'s number (rounded up).
   * **The reference must carry information (review m2).** If `adaptive` completes fewer than 3 of the ten laps of condition 2, or
     `tube` fewer than 3 of the ten of condition 3, that condition has no information. It then counts as **not passed**.
   * **If the gate fails, the phase closes without a benchmark by default.** A new design needs a new revision, reviewed
     before any test lap. Diagnostic variants may be run, as diagnostics only; no other setting may be tried to pass.
   * The gate is a coarse check on few cars; it says nothing about H-J1 or H-J2.
5. **NOM's fixed values** (review m5): its drift is s_on 1 600 m, ℓ 800 m, f_end 0.82; its noise seed is 399 900. **The drifts
   drawn for JD**, reported before any JV lap: s_on, ℓ, f_end distributions, the number of cars with
   μ · f_end < 0.70, and the shares of drifts that end before ~2.7 km and ~3.83 km.

### 9. Results of the development checks (2026-11-24; packet 2) – **the gate passed**

Records (git-ignored) are in `results/CX/J/dev/`, all at 9b92813 (clean; `meta_*.json`): `repro.json`, `drifts.json`, `refs.jsonl`
(72 laps, 0 errors) with `replay_laps/`, `replay.json`, `gate.jsonl` (96 laps, 0 errors) and `gate.json`. The order was the one
of clarification m1: reproduction, references, replays, gate laps. After the replays neither the grid nor the gate's
setting was changed. These are development data on six cars; they support no claim about H-J1 or H-J2.

1. **Code against the parents (9.1).**
   * `repro.json`: with the new options off, phase C's `tube` (78.279 s) and phase D's `adaptive` (76.399 s) reproduce their NOM
     records in nine fields (time, stop reason, distance, margin, solves, fallbacks, usage, steering effort).
   * `tests/test_cx_j.py` (13): the initial-error reachable set against the support function and against random errors; the combined
     NLP's constraint distances and objective at the same point as the tightened NLP of the car (10⁻⁹); the tube at the nominal
     car with C's explicit box equals C's `Tube` (10⁻⁹) and the cap box's disturbance is at least as large; the box, the quantile,
     a short closed-loop drive; the sets and seeds; the drift's start; the selection rule. Three mutations (no e₀, no shrinking,
     disturbance × 0.9) each make a test fail.
   * Not checked against a car other than the nominal one in closed loop: a vertex car with a tightening in a lap.
2. **The box and the residual terms, replayed (9.2; 72 recorded laps, the RLS on what the controller saw).**
   * **The shrinking is not empty:** 2σ_μ is below the cap on about 47 % of the steps in exact data (other parameters 70–91 %); on 7–19 % of the steps
     every parameter is at the cap, in lag 8–46 %.
   * **In exact data σ is conservative:** the true error is within 2σ on 100 % of the steps for nearly every factor
     (`adaptive` ClA: 93 %), and σ_μ (0.043) is about ten times the typical |μ̂ − μ| (0.004–0.007).
   * **Under noise and lag the box does not contain the truth:** |μ̂ − μ| ≤ h on 13–59 % of the steps (noise; the RLS is biased, as
     phase D) and 43–56 % (lag). Lag also pulls I_z and the others far off. The box is not a reliable statement of uncertainty there; the margin
     must come from the rest of the tube.
   * **The residual q** (median over the laps): under noise 0.11 m (n), 0.23 m/s (v_x), 0.11 m/s (v_y); in exact data and under the
     lag small (lag: v_y 0.013–0.11 m/s, r 0.014–0.048 rad/s), so the residual terms hardly act on the lag.
   * **Capped shares with the recorded predictions** (mean over the laps; stage-0 rule): η at the cap on 25 % of the stages for the
     cap box without residuals in exact data, 4 % for the shrunk box; under noise the shrunk box 7.5 %, with (ρ₀ 2, ρ_s 0.1) 34 %,
     with (4, 0.2) 42 %; mean η 0.851 (cap box), 0.880 (shrunk), 0.824 and 0.805.
3. **Timing (9.3; gate laps, 20 processes in parallel):** the parameterised tube takes 11–15 ms per solve on average
   (P95 15–36 ms by kind; the largest single step 60–260 ms, under load), phase C's 11.5 ms; the NLP 187–323 ms (mean per condition).
   Offline, as before.
4. **Development laps and the gate (9.4).** Completed laps of five DEV cars and NOM per condition (in this order):

   | | constant (DEV / NOM) | drift | noise | lag |
   |---|---|---|---|---|
   | `nmpc` | 2 of 5 / 1 | 0 of 5 / 0 | 1 of 5 / 1 | 0 of 5 / 1 |
   | `tube` | 5 / 1 | 1 / 0 | 2 / 1 | 5 / 1 |
   | `adaptive` | 5 / 1 | 5 / 1 | 0 / 0 | 0 / 0 |
   | **`ra`** (2, 0.1) | **5 / 1** | **5 / 1** | **3 / 1** | **5 / 1** |
   | `ra_fixed` | 5 / 1 | 5 / 1 | 4 / 1 | 5 / 1 |
   | `ra_fixed_res` | 5 / 1 | 5 / 1 | 3 / 1 | 5 / 1 |
   | `ra_shrink` | 5 / 1 | 5 / 1 | 2 / 0 | 5 / 1 |

   * **Gate:** (1) `ra` completes NOM constant; (2) `ra` 10 of 10 on DEV constant + drift, `adaptive` 10 (at least 9 needed; the reference has
     information); (3) `ra` 8 of 10 on DEV noise + lag, `tube` 7 (at least 4 needed). **Passed.**
   * **Lap times on the completed laps (description):** constant `ra` 76.7–79.7 s, `adaptive` 76.4–79.7 s, `tube` 77.8–79.1 s; drift
     `ra` 79.0–88.2 s (`adaptive` 78.9–88.5 s); noise (DEV and NOM) `ra` 83.9–85.0 s, `tube` 78.2–80.1 s; **lag `ra` 82.6–88.9 s, `tube`
     77.5–79.3 s**.
   * **Why `ra` is slow under the lag (observation):** the RLS absorbs the lag; μ̂ ends the laps 0.09–0.32 below the true value
     (median |μ̂ − μ| 0.20) and the plan coordinate is about 0.74 on the median (`adaptive`: 0.84; it ends 0.01–0.26 low). So
     `ra`, like `ra_shrink`, completes with a plan that is slower than the car could drive. `adaptive` has the same kind of
     bias and still leaves the track on all six laps. These are laps, not proofs of the cause. (Under noise μ̂ ends 0.06–0.16 too
     high for `ra`, as phase D found for the RLS.)
   * **The gate does not separate the `ra` kinds**, and noise is the hard condition: 3 of 5 for `ra`, 4 of 5 for `ra_fixed`; no
     selection is made on these laps (the tuning and the test decide on their own data).
5. **The drifts drawn (9.5):** JD – s_on 681–2 580 m, ℓ 118–1 460 m, f_end 0.751–0.900, μ·f_end 0.698–0.909 (one of 50 cars
   below 0.70); all drifts have begun before 2.7 km and ended before 3.83 km, 66 % before 2.7 km. JV_D (ten cars): f_end
   as drawn for phase D; none below 0.70; 60 % end before 2.7 km, 90 % before 3.83 km.

**Not changed by these results:** the grid, the gate's setting, κ, the selection rule, the hypotheses. Two things the
replays and laps suggest, which the test will or will not show:
* the residual terms act on noise, not on lag, and the box is not informative there; H-J2's noise half and lag half may go
  different ways;
* under lag the completions of `ra` come with laps 4–10 s slower than `tube`'s, which McNemar on completions does not see.
  Section 6's secondary lap-time test covers only JT; **a description of the lap times of JN and JL on the cars both complete is
  added to the report** (no test; stated here before any JT / JD / JN / JL lap).
