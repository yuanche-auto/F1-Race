# L8 protocol – generalisation: new tracks and new conditions, separately

Status: **draft, revision 0 (2026-10-17)**, written before any run on a test track or under a
test condition. Branch `layer/L8` (from `main` at 4a9d92c). ROADMAP: section 4 (L8), rules of
section 2. What was seen before this draft is listed in section 12.

## 0. What L7 left, and what L8 asks

L0–L7 ran on one track, Spielberg, with one car and its parameter spread (L2), the L3
disturbances and the L6 steering lag. On Spielberg's dry cars without lag (T1 at τ = 0, 25 cars)
the two best controllers are tied: the lag-aware NMPC with τ and parameters identified from an
exploration lap and re-planned (`lag_identified`, L6) and the validation-selected L5 policy
`B_s1_500k`, both 25 / 25 (L7 Part B). The other L5 final checkpoints complete 10–24 of those
laps; under the L3 disturbances the SMPC is the most reliable model-based controller (29 / 30).

L8 asks whether this ranking holds when the **track** changes (geometry the policies never saw;
every controller gets the plan computed for the new track) and when the **friction** changes
(wet, below the range the policies were trained on), one at a time. *What changes the ranking of
the fixed baselines is the main result* (ROADMAP section 2.1).

## 1. Hypotheses

* **H-G (geometry, primary):** on the four test tracks (100 laps: the 25 cars × 4 tracks, no lag,
  no disturbances) the identification + re-planning controller `lag_identified` completes
  **more** laps than the validation-selected policy `B_s1_500k`. Exact McNemar, two-sided, over
  the 100 paired laps; H-G holds if the test is significant at its Holm level and more laps are
  completed only by `lag_identified` than only by `B_s1_500k`.
* **H-D (friction, primary):** under global wet friction on Spielberg (50 laps: the 25 cars ×
  the levels 0.85 and 0.75, section 2) `lag_identified` completes **more** laps than
  `B_s1_500k`. Same test and rule.

Reasoning written down before any run: `lag_identified` meets both changes by construction (it
re-plans on the new track and identifies the friction coefficient from the exploration lap,
section 3), while `B_s1_500k` learned its corrections on Spielberg with friction factors
0.92–1.03 (and the L3 friction field, σ = 0.03). If H-G or H-D fails, the learned policy's
reliability transferred as well as identification + re-planning (or both failed alike) – a
result reported as such.

Holm over H-G and H-D (family-wise α = 0.05: the smaller p at 0.025, the other at 0.05 only if
the first is rejected). As in L7 (A9.1), the report gives Holm's decision and the stricter one
(each at 0.025) side by side.

**Literature check (ROADMAP rule 6, at the start of L8; abstracts, 2026-10-17).** RL racing
policies are commonly evaluated on held-out tracks (e.g. drone racing on random layouts,
arXiv:2103.08624; Formula RL, arXiv:2104.11106). Closest to L8's structure, TC-Driver (Ghignone,
Baumann, Boss, Magno, Field Robotics 2023, arXiv:2205.09370) conditions an RL controller on a
planned trajectory and reports fewer crashes than an end-to-end policy on unseen tracks and under
tyre-model mismatch, and fewer than a non-learning MPC under mismatch. Friction adaptation in
racing: LLA-MPC (AL-Sunni, Almubarak, Horng, Dolan, arXiv:2505.19512) selects a model from a bank
by recent behaviour and re-plans under sudden friction changes; ensemble GP-MPC over friction
levels (arXiv:2303.13694); a risk-averse MPC on a wet turn (arXiv:2410.17183). Residual RL on
F1TENTH learned on board ("Drive Fast, Learn Faster", Hildisch et al., RLJ 2025 – found by search,
abstract not read) and RLPP (Ghignone et al., ICRA 2025, found by search). L8 does not claim a new
method: it is a controlled comparison of the fixed baselines under a new track and under wet
friction separately, with an identification baseline that re-plans, the unselected RL
distribution, and pre-registered tests.

## 2. Tracks, cars, conditions

**Track draw** (`scripts/l8_tracks.py`, committed with this draft): the 25 tracks of the TUM FTM
racetrack-database at the pinned commit, without Spielberg (L0–L7) and IMS (an oval), sorted by
name and permuted with `numpy.random.default_rng(8)`. Result:

| role | tracks |
|---|---|
| validation (pipeline checks, smoke tests; nothing pre-registered is measured on them) | Monza, Sakhir |
| **test** (run once) | Zandvoort, SaoPaulo, Norisring, Nuerburgring |
| reserve, in order | Austin, Silverstone, Shanghai, Montreal, Catalunya, … (17) |

The test tracks are not downloaded before this protocol is binding. Every plan-based controller
gets the plans of the track made by the L1 pipeline unchanged (`experiments/l8.compute_plans`:
η = 1.0, 0.97, 0.95, 0.93 by the continuation of L1, and the 12 % plan η = 0.88 from the ideal one,
as in L2). **If a plan of a test track fails, the track is replaced by the next reserve track**
(reported); the pipeline is not changed on test tracks. Changes of the pipeline may rest only on
the validation tracks and are made before this protocol is binding.

**Cars.** The 25 cars of T1 (`sample_plants(nominal, 24, seed 0)`, the nominal car first; L2
ranges), without steering lag – the cars of the Spielberg reference.

**Conditions** (one source of error at a time, then combined):

| part | condition | laps per controller | set |
|---|---|---|---|
| A | test tracks, the 25 cars, no lag, no disturbances | 4 × 25 = 100 | **H-G** |
| A | test tracks, nominal car with the L3 disturbances, seeds 0–9 per track (combined) | 4 × 10 = 40 | secondary |
| A0 | Spielberg reference: the 25 cars (T1, τ = 0) and the nominal car with L3 seeds 0–9 (T2) | 35 | reference |
| B | Spielberg, the 25 cars, friction × 0.85 ("damp") and × 0.75 ("wet") over the whole lap | 50 | **H-D** |
| B | Spielberg, nominal car, dry with four wet patches (× 0.75), patch seeds 0–29 | 30 | secondary |

* Global wet: the car's friction coefficients (both axles, x and y) are multiplied by the level;
  nothing else changes, nobody is told. The levels: 0.85 and 0.75 of dry grip – a damp and a wet
  track for tyres of this car, below the training range of the policies (0.92–1.03). 0.75 keeps
  the true friction factor (0.75 × 0.92–1.03 = 0.69–0.77) essentially inside the identification
  bounds of L4 (0.7–1.3; up to 1.4 % below), which stay unchanged.
* Wet patches: each 100 m at × 0.75 with 25 m linear ramps on both sides; four per lap, centres
  drawn uniformly over the lap with the patch seed, redrawn (same generator) until every two are
  at least 300 m apart. The friction field of the L3 simulator (`StochasticSim`) carries it; no
  gusts, no noise.
* Exploration laps of the identification controllers: on the same car and condition (wet level,
  patches; for the L3 seeds a separate draw of the disturbances, as in L6 / L7).

**Validation sets** (code checks; the RL fine-tuning of Part C may use them, section 6): on Monza
and Sakhir, the cars `sample_plants(nominal, 10, seed 1)[1:]` (L7's V cars) and L3 seeds
2000–2004; on Spielberg for Part B, the same cars at 0.85 / 0.75 and patch seeds 1000–1009.

## 3. Controllers and what they get (information parity)

Fixed baseline set (ROADMAP 2.1), re-evaluated unchanged; plans always those of the track:

| controller | plan / model | gets about this car and condition | L-origin |
|---|---|---|---|
| `tracker` | 5 % plan (η 0.95), L1 feed-forward / feedback tracker | nothing | L1 |
| `nmpc` | 5 % plan, nominal soft NMPC (N = 30, 50 ms) | nothing | L2 |
| `nmpc_reserve` | 12 % plan (η 0.88), the same NMPC with η 0.88 (uniform reserve) | nothing | L2 |
| `smpc` | L3 SMPC (ε = 0.01), back-offs calibrated on Spielberg, transferred unchanged | nothing | L3 |
| `identified` | parameters identified from one exploration lap (`nmpc_reserve`), re-plan (`replan_robust`), NMPC on the identified model | its own exploration lap | L4 (with L6's re-plan) |
| `lag_identified` | τ and parameters identified from the exploration lap, re-plan, lag-aware NMPC | its own exploration lap | L6 (best model-based of L6 / L7) |
| `B_s1_500k` | residual SAC policy on the tracker (history 8), trained on Spielberg | its observation history | L5 (validation-selected) |

All NMPC-based controllers with the L6 failure handling (retry off, fallback to the shifted plan;
the preparation's rule). Every controller gets the same track plan and curvature preview; none
gets the friction level or the patch positions; the identification controllers get one
exploration lap of the actual car under the actual condition (that is what they are).

**RL, unselected distribution (reported first, secondary):** the nine final L5 checkpoints and
the ten final L7 checkpoints (P, F), plus the two validation-selected L7 checkpoints
(`P_s5_500k`, `F_s5_450k`). Without a filter.

**Not in L8:** the L7 safety filter (it lowered completions in L7 and costs ~4 min per lap); the
"local information only" condition of the ROADMAP – the only local planner of the project (the
receding-horizon minimum-time MPC of L2) completed no lap, and the RL policies' observation
contains the plan's preview, so a fair condition needs a new local planner (work after L8); the
online update of the L6 residual – its uncertainty was not calibrated (L6), and L7 used the
identified structure instead. All three are listed as untested in the report.

## 4. Part A – new tracks

On each test track: the plans (section 2), then the fixed set and the RL policies on the 25 cars
and on the 10 disturbance seeds. **A0** completes the Spielberg reference on the same laps: the
controllers missing from L7 Part B (`nmpc_reserve`, `identified`) on T1 τ = 0 and T2 seeds 0–9;
the others are L7 Part B's records (deterministic evaluation, L7 Part B consistency checks).

Reported per controller: completion per track and over the four, with exact intervals, against
Spielberg on the same cars; failure positions per track; lap times; plan lap times per track; for
the identification controllers the exploration laps (completed / distance), the identified μ
against the true factor, and failed re-plans.

## 5. Part B – wet Spielberg

The 25 cars at 0.85 and 0.75 (H-D) and the 30 patch laps, every controller of section 3 and the
RL policies; the same reports as Part A, plus the identified friction factor against the true
one (global wet: identifiable in principle; patches: one μ for the whole lap cannot describe
them).

## 6. Part C – online adaptation (secondary)

* **First-lap identification** is what `identified` / `lag_identified` do (Parts A, B).
* **Short RL fine-tuning:** `B_s1_500k` continued with SAC at the L5 group-B settings for
  100 000 decisions (8 environments; a new replay buffer, 5 000 steps before learning) on
  (i) each test track and (ii) wet Spielberg with a friction factor drawn per episode from
  U(0.70, 0.90); cars from the L2 ranges, half of the environments with the L3 disturbances, as
  in L5. Three seeds per condition (15 runs); the final checkpoint is evaluated (no selection) on
  that condition's laps of Parts A / B. Reported against `B_s1_500k` without fine-tuning
  (McNemar) and against `lag_identified`. Information: ~100 000 × 50 ms ≈ 5 000 s of driving on
  the track or condition (about 60 laps' worth, other cars), against one exploration lap for the
  identification controllers – stated with every comparison.
* The validation tracks / sets are used only for code checks of the fine-tuning; no
  hyper-parameter is tuned.

## 7. Always reported

Per controller, condition and track: completion (exact 95 % intervals) and lap time, paired lap-
time differences on laps both complete; stop reasons and failure positions; NMPC fallbacks and
solve times (mean, P95, P99, max) under the run's load; plan lap times; exploration-lap outcomes
and identified parameters (μ estimate, bound hits); the ranking of the fixed set per condition
against Spielberg's (Kendall's τ, descriptive); the RL distribution first (per policy, per
condition, and the change against Spielberg on the same cars, with the exact signed-rank test
over the nine L5 final checkpoints as a secondary result).

## 8. Statistics

Exact McNemar tests on paired laps (same car / seed, track, condition), exact (Clopper–Pearson)
intervals, exact signed-rank tests (mid-ranks, all 2ⁿ sign assignments) for paired lap times and
across policies. Lap-time differences of single laps below ~0.4 s are not interpreted (L7 section
7). Two primary tests (H-G, H-D) with Holm (section 1); everything else is reported with its p
value and the number of comparisons, without a claim across the family. S1 of L7 showed that
lap-level p values do not contain the variance between training seeds; claims about "RL" as a
method rest on the unselected distribution, not on `B_s1_500k` alone.

## 9. Second environment

ROADMAP rule 2.4: the primary results re-run in a second Python environment. The user deferred
the Python 3.11 environment to the to-do list (L7 A10). For L8: H-G and H-D are re-run there when
it exists (the laps of `lag_identified` and `B_s1_500k` in Parts A and B); until then the L8
report marks them as not cross-checked, as L7 does.

## 10. Compute and order

1. Plans of the validation tracks (done before binding: 36 min for Monza, 30 min for Sakhir, both
   at once; section 12), then of the four test tracks (~30–40 min each, two to four at once; dynamic
   OCPs ~7.4 GB each, `replan_slot`).
2. Code for Parts A–C, checked on the validation sets only; reviewed before any test run.
3. A0 and Part A: ~735 NMPC-based laps (five NMPC controllers × 140, plus A0's 70) at ~3–4 min,
   ~45 CPU-h; ~350 exploration laps and re-plans (four at a time); RL laps (~22 policies × 175)
   ~2 s each.
4. Part B: ~400 NMPC-based laps, ~160 exploration laps and re-plans; RL laps.
5. Part C: 15 fine-tuning runs (~10 min each) and their evaluation laps.
6. Report, second environment (if available), review.

## 11. Known limitations, fixed in advance

* The tracks are centre lines with widths from one database (smoothed as for Spielberg), flat,
  without banking or kerbs; four test tracks.
* Wet is a friction factor only (no water film, no change of tyre characteristics, the same
  Pacejka shape); the patches are a deterministic friction field.
* The identification bounds of L4 (μ factor ≥ 0.7) are kept; at 0.75 a few cars' true factor lies
  up to 1.4 % below the bound.
* The RL policies get the plan of the nominal dry car on every track and condition, as every plan-
  based controller does; the identification controllers re-plan for their car.
* Round 2 of L6 remains unverified; no L8 decision rests on it. The second environment is pending
  (section 9).

## 12. What was seen before this draft (disclosure)

* The Spielberg reference numbers of section 0 (L7 Part B records, T1 at τ = 0 and T2), read to
  choose the controllers of H-G / H-D.
* The validation tracks Monza and Sakhir: downloaded (Sakhir's SHA-256 pinned in
  `scripts/fetch_tracks.py`), length and widths printed (Monza 5 790 m, 7.5–12.4 m; Sakhir
  5 406 m, 10.8–22.4 m), and their plans computed with `scripts/l8_plans.py`: all five plans of
  both tracks converged (Monza η 1.0 / 0.97 / 0.95 / 0.93 / 0.88: 89.22 / 90.22 / 90.94 / 91.71 /
  93.81 s; Sakhir 96.87 / 98.25 / 99.25 / 100.32 / 103.28 s). On both, the first two initial
  guesses of L1's first solve stopped at the 600-iteration cap and the third (quasi-steady at
  93 % speed) converged – inside L1's fall-back sequence of four, so the pipeline is unchanged;
  a test track on which all four fail is replaced (section 2).
* A closed-loop check of those plans with the nominal car, one lap each
  (`scripts/l8_validation_check.py`): the nominal NMPC completes both (Monza 91.40 s, Sakhir
  99.62 s, no fallback, ~0.13 s per solve); the bare L1 tracker completes Monza (90.93 s) and
  **spins on Sakhir at 1 916 m** with the nominal car. The tracker is a fixed baseline and is not
  changed; `B_s1_500k` is a residual on it – part of what H-G measures.
* The test tracks: only their names (the draw).
