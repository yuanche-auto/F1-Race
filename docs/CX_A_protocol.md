# CX-A protocol – dynamic MPCC (control expansion, phase A)

Status: **draft, revision 0 (2026-10-21)**, for review; written after the development checks of
section 12 and before any lap of the sets of section 2. Branch `cx/A-mpcc` (from `main` c010887);
code 0061e79. Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md); literature:
[CONTROL_LITERATURE_REVIEW.md](CONTROL_LITERATURE_REVIEW.md) §A.

## 0. What L0–L9 left

No contouring controller exists. L2 tried two progress-maximising MPCs: a spatial minimum-time MPC that
left the track in all eight configurations (docs/L2.md §3) and a time-domain progress-maximising MPC
that left at turn 1 because a 1.5 s horizon does not cover the braking zones (L2 §4). The NMPC of L2–L9
tracks the offline minimum-time plan. Phase A implements the standard model predictive contouring
control (Liniger, Domahidi, Morari 2015) with this repository's vehicle model and limits, and compares
it with the existing plan-based controllers.

## 1. Question and hypotheses

*Does a dynamic MPCC – which maximises progress itself instead of tracking the plan's trajectory, with
the same model, limits, horizon and solver as the NMPC – complete as many laps as the NMPC, and at what
lap time, under the model mismatch of L2?*

* **H-A1 (primary):** on the 25 test cars (section 2), `mpcc` and `nmpc` differ in the number of
  completed laps. Exact McNemar, two-sided; holds if significant at its Holm level; direction reported.
* **H-A2 (primary):** on the test laps both complete, `mpcc` and `nmpc` differ in lap time. Exact
  signed-rank (mid-ranks, the exact distribution), two-sided; same rule.

Holm over H-A1 and H-A2 (family-wise α = 0.05), with the stricter decision (each at 0.025) reported
alongside. No direction is predicted: the MPCC can choose its own line and speed within the limits
(faster, section 12), but it also uses more of the track and of the tyres, which leaves less room for a
car that differs from the model (L2: failures at three corners from how the car turns).

Scope: these two controllers with the nominal model, on Spielberg, at η = 0.95; no claim about MPCC in
general (its weights are tuned on section 2's tuning cars only).

## 2. Sets (new draws; none used by L0–L10)

* **Tuning set CXV:** `sample_plants(nominal, 10, seed 301)[1:]` – 10 cars from the L2 ranges,
  Spielberg, one lap each, no lag, no disturbances.
* **Test set CXT:** `sample_plants(nominal, 25, seed 302)[1:]` – 25 cars, the same conditions. The
  nominal car is reported separately (it was used for development, section 12) and is not part of a test.
* Secondary conditions on CXT (after the primary report): the L6 steering lag τ = 30 ms (nominal-model
  controllers, as L6's rows without a lag model); the L3 disturbances on the nominal car (seeds
  7400–7429; tuning-free); the four L8 test tracks with the nominal car (their L8 plans).
* Start: the state of the 5 % plan at s = 0 (as L2); one lap; time limit 1.6 × the plan's lap time
  (121 s on Spielberg).

## 3. Controllers

Shared: the nominal model (`DynVehicle()`), friction factor η = 0.95 in the constraints, N = 30 stages
of 50 ms, IPOPT (tol 10⁻⁴, warm start), track margin 0.3 m unless tuned, the L2 traction control
(0.99), no solver retries (a failed solve applies the shifted previous plan, as L8).

* **`tracker`** – the 5 % plan and the L1 dynamic tracker (time-optimal planning + tracking, L1).
* **`nmpc`** – `FrenetMPC` tracking the 5 % plan (L2, unchanged).
* **`mpcc`** – `DynamicMPCC(reference="raceline", speed_bound="stages")`: the standard contouring
  formulation with the plan's racing line as reference path and the plan's speed as an upper bound at
  every stage (section 12 shows why both are needed on this car); weights from section 4.
* **`mpcc_centre`** (secondary) – the same with Liniger's reference path, the centre line.
* **`mpcc_ca`** (secondary, curvature-aware) – defined by an amendment after the full text of Ma et al.
  (Control Engineering Practice 171, 2026; literature §A.4) has been read; if it cannot be obtained, the
  amendment states that and defines a variant of our own, named as such, or drops it.

Information budget: all four know the track and the 5 % plan; `nmpc` tracks its trajectory (position,
heading, speed), `mpcc` uses its line as the contouring reference and its speed only as an upper bound,
`mpcc_centre` only its speed; `tracker` follows it with feed-forward. All see the true state (no
noise, no delay); none knows the car's parameters (no oracle).

## 4. Tuning (CXV only)

A grid for `mpcc`: q_c ∈ {0.05, 0.5, 2, 8} × q_v ∈ {0.5, 1, 2} × margin ∈ {0.3, 0.6} m (24 settings ×
10 cars); the other weights fixed at the module defaults (q_l 50, rate weights as the NMPC). Choice: the
most completed laps; ties by the mean lap time on the laps every tied setting completes; then the
smaller q_c. The same grid and rule for `mpcc_centre`. `nmpc` and `tracker` are not tuned (L2 as is).
The chosen settings and the CXV records go to the reviewer before any CXT lap.

## 5. Always reported

Per controller and set: completed laps (exact 95 % intervals), lap time, distance to failure and stop
reason; contouring error to the racing line and to the centre line (mean, max |·|); for `nmpc` its
tracking error; constraint use (minimum edge margin, track and limit slacks); tyre utilisation (max and
mean ellipse usage per axle); control effort (steering-rate RMS, force-increment RMS); solve time
(mean, P95, P99, max) and the share of solves above 50 ms (offline: a would-be deadline miss);
non-converged solves and fallbacks; the adaptation data budget (none for these controllers).

## 6. Statistics

Exact McNemar on paired laps, exact intervals, exact signed-rank tests for paired lap times; two primary
tests with Holm; everything else with its p value and the number of comparisons, no claim across them.

## 7. Second environment

As L7–L9 (deferred by the user): H-A1 and H-A2 are marked as not cross-checked until the Python 3.11
environment exists.

## 8. Compute and order

1. Code review of `control/mpcc.py`, its tests and this protocol (0061e79 and the protocol commit).
2. Tuning on CXV: 48 settings × 10 cars ≈ 480 MPCC laps (~4 min each, ~32 CPU-h; 20 jobs at once);
   results to the reviewer.
3. CXT: 4 controllers × 25 cars (+ the nominal car), then the secondary conditions; pinned worktree,
   input SHA-256 list, single-threaded BLAS, one process per job, `results/CX/A/`.
4. Report section in CONTROL_EXPANSION_BENCHMARK.md, review, merge of `cx/A-mpcc`.

## 12. Development checks before this draft (disclosure)

Nominal car, Spielberg, one lap from the plan start, code 0061e79, records `results/CX/A/dev/`
(git-ignored). These laps chose the reference-path and speed-bound options of section 3; the weights
are left to section 4.

| controller (reference, speed bound, weights) | result | solve ms mean / P95 / P99 / max | fallbacks | max track slack in a prediction | min edge margin | max ellipse use F / R | steering-rate RMS |
|---|---|---|---|---|---|---|---|
| `nmpc` (L2, unchanged) | **76.05 s** (L2 recorded 76.07 s) | 130 / 181 / 216 / 318 | 0 | 0 | 0.287 m | 0.984 / 0.976 | 0.058 |
| `mpcc` centre, terminal (Liniger's path) | off track at **459 m** (turn 1) | 131 / 227 / 410 / 2 566 | 1 | 8.41 m | −0.104 m | 0.984 / 0.983 | 0.114 |
| `mpcc` raceline, terminal | off track at **461 m** | 115 / 174 / 311 / 334 | 0 | 7.34 m | −0.095 m | 0.983 / 0.988 | 0.115 |
| `mpcc` centre, stages | off track at **3 821 m** | 136 / 184 / 283 / 2 921 | 3 | 0.77 m | −0.045 m | 0.978 / 0.987 | 0.095 |
| `mpcc` raceline, stages (q_c 0.05) | off track at **3 839 m** | 133 / 190 / 287 / 557 | 0 | 0.33 m | −0.018 m | 0.974 / 0.992 | 0.093 |
| `mpcc` raceline, stages, margin 0.6 m | off track at **3 836 m** | 132 / 187 / 307 / 429 | 0 | 0.60 m | −0.002 m | 0.979 / 0.991 | 0.102 |
| `mpcc` raceline, stages, q_c 0.5 | **76.05 s** | 132 / 180 / 235 / 534 | 0 | 0 | 0.276 m | 0.977 / 0.990 | 0.079 |
| `mpcc` raceline, stages, q_c 2 | **75.76 s** | 130 / 179 / 234 / 421 | 0 | 0 | 0.267 m | 0.984 / 0.989 | 0.080 |

(8 laps run in parallel, so the solve times include the load of the others.)

What they showed, and how it shaped section 3:

* **Terminal bound only → turn 1.** A trace of the first version (centre line, terminal bound
  evaluated at θ_N; before 0061e79, `scratchpad` trace not kept as a record): from
  ~4.2 s the car is faster than the plan at the same s (61 against 58 m/s at 370 m, 57 against 46 m/s
  at 393 m) although every prediction meets v_N ≤ v_ref at its end; each solve postpones braking. In the
  last solves θ stops (the predicted progress over the horizon falls from ~67 m to 0) while the car
  moves on: a θ behind the car makes v_ref(θ_N) larger in a braking zone. The speed bound is therefore
  evaluated at θ − e_l, and "stages" bounds every predicted speed; with it both paths pass turn 1.
* **Centre line → the wrong line for the speed profile.** With the centre line the car stays at n ≈ 0
  on the straights and enters turn 1 on a tighter line than the plan (n ≈ +4 m) for which the speed
  bound was computed; with the racing line it stays near the plan's line (n 3.5–4.0 m before turn 1).
  Both variants are in section 3 (the centre line as the secondary, Liniger's choice).
* **Low contouring weight → the edge.** With q_c = 0.05 (Liniger: "low") both stage-bounded variants
  leave the track by 2–45 mm at the penultimate corner, at tyre use ~0.99; a larger margin (0.6 m) does
  not prevent it, q_c 0.5 and 2 do. Hence q_c and the margin are in section 4's grid; these two laps on
  the nominal car are not a selection of the final weights.
