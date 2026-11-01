# Control expansion – benchmark conventions and results

Results of the advanced-control expansion ([CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md)),
kept apart from the L0–L9 records. Only results produced under a reviewed phase protocol
(`docs/CX_<phase>_protocol.md`) appear in the "benchmark" sections; development checks are listed as
such and are never pooled with them.

## Conventions

* **Sets:** each phase draws its own tuning and test cars with seeds no L-layer used (phase A: car seeds
  301 tuning, 302 test; disturbance seeds 7300–7309 tuning, 7400–7429 test). L-layer sets are used only
  where a phase protocol reproduces an L-layer condition, and then as in that layer.
* **Outputs:** `results/CX/<phase>/` (git-ignored); development checks in `results/CX/<phase>/dev/`
  with the commit and a dirty flag in every record.
* **Metrics (every phase):** completion (exact 95 % intervals), lap time, distance to failure and stop
  reason, contouring / tracking error, constraint use (edge margin, slacks), tyre utilisation, control
  effort, solve time (mean, P95, P99, max), deadline misses (in real-time conditions; offline: the share
  of solves above 50 ms), non-converged solves and fallbacks, the adaptation data budget.
* **Statistics:** exact McNemar on paired laps, exact signed-rank (mid-ranks, exact distribution) on
  paired lap times, Holm over a phase's primary tests, uncorrected p elsewhere with the number of
  comparisons.

## Phase A – dynamic MPCC

Protocol [CX_A_protocol.md](CX_A_protocol.md) (revision 1, clarification m1, amendments A1 and A2). Runs from pinned
worktrees: the first CXV tuning at 8babe7b (`F1-Race-wt-CXA-tune`, 2026-10-21 19:03–19:51), the A2 re-tuning of the
MPCCs, the `mpcc_ca` tuning and CXT at c7c802f (`F1-Race-wt-CXA-run`, 20:21–21:50); input SHA-256 identical to the
main checkout; no error record. Records (git-ignored) in that worktree's `results/CX/A/`: `tune.jsonl`
(a60622d42bb5…), `tune_a2.jsonl` (0c26aa556763…), `tune_ca.jsonl` (1d925306f1a2…), `selected_first.json`,
`selected.json`, `selected_ca.json`, `test.jsonl` (2e560c407137…), `report/CX_A.md`, `timeline.tsv`.

**Settings chosen on CXV** (10 tuning cars; the rule of protocol §4): `nmpc` c 2, margin 0.6 m (3 of 10);
`mpcc` (soft bound, A2) q_c 8, q_v 2, margin 0.3 m (3 of 10, six settings tied, the lowest mean lap on the cars all
complete); `mpcc_centre` q_c 0.05, q_v 1, margin 0.3 m (0 of 10 in every setting – the rule falls to the closest to
the defaults); `mpcc_ca` α 3, w_ca 0.1 on `mpcc`'s base (3 of 10, three settings tied within 0.08 s). The first
tuning with the hard bound gave `mpcc` at most 2 of 10 with 6 442 fallbacks; A2 is in the protocol.

### Primary tests (25 test cars CXT): neither holds

| test | result | p | Holm |
|---|---|---|---|
| H-A1 completed laps, `mpcc` : `nmpc` | 9 / 25 against 10 / 25; only one completes 1 : 2 | 1 | not rejected |
| H-A2 lap time, the 8 cars both complete | `mpcc` faster on 6 of 8, mean −0.314 s, median −0.334 s (W⁺ 6) | 0.109 | not rejected |

Discordant cars: car 12 only `mpcc` (78.069 s; `nmpc` off at 3 830 m); cars 2 and 8 only `nmpc` (`mpcc` off at
462 / 464 m, turn 1). Not cross-checked in a second environment (protocol §7).

### Secondary (uncorrected, 5 comparisons) and description

| controller | completed (95 %) | mean lap of the completed | failures |
|---|---|---|---|
| `tracker` (plan + L1 tracker) | 1 / 25 (0.00–0.20) | 75.61 s | 18 spins, 6 off (turn 1 ~465 m, ~2.73 km, ~3.02 km) |
| `nmpc` | 10 / 25 (0.21–0.61) | 76.86 s | turn 1 (5), ~2.2 km (4), ~2.73 km (4), ~3.83 km (2) |
| `mpcc` | 9 / 25 (0.18–0.57) | 76.64 s | turn 1 (10), ~2.7 km (3), ~3.83 km (3) |
| `mpcc_ca` | 9 / 25 (the same cars as `mpcc`) | 76.75 s | as `mpcc` (one car fails at 2.71 km instead of 477 m) |
| `mpcc_centre` | 3 / 25 (0.03–0.31) | 77.62 s | turn 1 (20 of 22) |

The mean laps above come from different cars (the tracker's from one car) and are not comparable across the
rows; the only like-for-like lap-time comparison is H-A2's, on the 8 cars both `mpcc` and `nmpc` complete.

`mpcc_centre` against `nmpc` 0 : 7 (p 0.016) and against `mpcc` 0 : 6 (p 0.031); `tracker` against `nmpc` 0 : 9
(p 0.004); `mpcc_ca` against `mpcc` 0 : 0 and against `nmpc` 1 : 2 (p 1). Solver: `nmpc` 5 fallbacks over the 25
laps, `mpcc` 16, `mpcc_ca` 13, `mpcc_centre` 35; mean solve 174–197 ms, every solve above 50 ms (offline IPOPT);
the MPCCs' P99 reaches ~4 s. The speed bound of `mpcc` is active in 46 % of the solves at the measured stage 0 and
in 43 % of the predicted stages. On the nominal car (development car, not a test): `mpcc` 75.85 s, `mpcc_ca`
75.94 s, `nmpc` with its CXV setting 76.40 s, `tracker` 75.57 s, `mpcc_centre` off track.

### Secondary conditions (protocol §2; after the primary report; uncorrected, 3 comparisons)

Run at c7c802f in the same worktree, 2026-10-21 21:51–22:16 (`secondary.jsonl` f3a3a5399e0a…, `report/CX_A_secondary.md`);
`mpcc_ca` is not part of them (amendment A1: CXT and the nominal car only). **An input was missing at first:** the
worktree had no L8 track files and plans, so the tracker's four CXL8 jobs (the first in the queue) failed with
`FileNotFoundError` before any lap; the files were copied during the stage (SHA-256 identical to the main checkout and
to the L8 run, `SHA256_inputs_L8tracks.txt`), the later CXL8 jobs ran normally, and the four failed jobs were re-run
by resuming the stage (22:16:24–22:16:31); their error records stay in the file, the report uses the re-runs.

| condition | `tracker` | `nmpc` | `mpcc` | `mpcc_centre` | `mpcc` : `nmpc` (only one completes), p |
|---|---|---|---|---|---|
| CXT cars with the 30 ms steering lag (25) | 1 | 4 | 1 | 0 | 0 : 3, 0.25 |
| nominal car, L3 disturbances with measurement noise (30) | 0 | 4 | 10 | 1 | 7 : 1, 0.070 |
| nominal car on the four L8 test tracks (4) | 0 | 4 (mean 93.16 s) | 4 (mean 92.47 s) | 0 | 0 : 0, 1 |

* **Lag (30 ms, no lag model in any controller):** every controller loses most laps, as in L6's rows without a lag
  model; `nmpc`'s 21 failures are all at turn 1 (~460 m), `mpcc`'s 24 at turn 1 (15), ~2.7 km (7), 2.2 and
  3.8 km (one each).
* **L3 disturbances:** `mpcc` completes more laps than `nmpc` (10 against 4; 7 : 1, p = 0.070 uncorrected – not
  significant); `nmpc`'s 26 failures are spread over the lap (turn 1, 1.4, 2.2, 2.7, 3.8 km), 14 of `mpcc`'s 20
  are at 3.81–3.84 km (one more at 4.01 km). Mean laps 77.76 against 78.69 s, on different seeds (not comparable).
* **New tracks:** both MPC-based controllers complete all four with the nominal car; the plan-plus-tracker and the
  centre-line MPCC none.
* These are secondary and uncorrected; with 4–30 laps per condition none of the differences is significant.

### Reading

* **No difference in completion is shown, and this test had little power.** Both controllers complete about 40 %
  of the test cars, and only 3 cars separate them. A 1 : 2 split says nothing either way; it is not evidence that
  the two are equally reliable.
* **The MPCC is not slower where both complete** – faster on 6 of the 8 cars by about 0.3 s, not significant at
  n = 8. On the tuning cars the same order appeared (76.78 against 77.42 s on cars 1, 3, 9), but those numbers are
  after the selection and biased.
* **The failures sit at the same few corners for every controller** (turn 1, ~2.2 / 2.7 km, ~3.83 km), and the same
  cars fail – the cars of the L2 ranges are hard for all five controllers here (each plans or tracks at η = 0.95
  with the nominal model; no other controller was tested). The difference between the methods is small compared
  with that.
* **On this grid and these cars, changing the reference path to the centre line made a larger difference than any
  change of the weights within the grid** (secondary, uncorrected): with the centre line (Liniger's choice) 20 of
  the MPCC's 22 failures on CXT are at turn 1; on the nominal car the same formulation (stage bound) leaves the
  track at ~3.82 km, as in the development check (protocol §9: 3 821 m) – the turn-1 failure there was the
  terminal-bound version's.
* **The curvature-aware term adds nothing measurable here:** `mpcc_ca` completes the same cars as `mpcc`; its tuning
  ties were within 0.08 s. Its β is taken at the car's position for a 1.5 s horizon and cannot anticipate a corner
  (adaptation of CiMPCC, amendment A1).
* **Limits of the comparison:** both chosen settings lie on the edge of their grids (`nmpc` c 2, `mpcc` q_c 8 and
  q_v 2), so neither is shown to be tuned to its optimum; the tuning freedom was not symmetric (48 MPCC settings in
  two rounds with a change of formulation between them, 6 for the NMPC; protocol §1); the MPCC's speed is bounded by
  the plan's, so it chooses its line and its speed below the plan's, not its own speed profile; offline (no compute
  budget – an IPOPT MPCC solve takes ~0.2 s, see phase G).

## Phase B – MPPI (sampling-based MPC on the NMPC's problem)

Protocol [CX_B_protocol.md](CX_B_protocol.md) (revision 0, clarifications, amendments B1–B3).

**The development gate failed, and this benchmark ran under amendment B3.** Before any tuning lap, MPPI was required
to complete the nominal lap with at least one setting (protocol §9.6). The white-noise MPPI of Williams et al. (T-RO
2018) completed 0 of 17 nominal laps (K 2 048 / 8 192, more iterations per step); the low-pass MPPI (LP-MPPI) 2 of 8
at K 2 048 / 8 192, and 0 of 9 at the real-time budget K_RT = 1 024 that the timing rule gives it (median step ≤ 40 ms
on one core). B3 waived the gate so that the registered comparison could be made on new cars; H-B1 is therefore a
confirmatory test of a direction the development laps already suggested (in `nmpc`'s favour), not an independent
finding. **Every result below holds for this CPU, numpy implementation at K = 1 024 only.**

**The tuning carried no information.** On the 10 tuning cars CXV every setting completed 0 of 10 in both steps (9
settings of λ × c_σ at 2 Hz, then f_c ∈ {1, 2, 4} Hz), so the rule fell to the setting closest to the grid centre:
`mppi_lp` λ 1, c_σ 1, f_c 2 Hz, K 1 024. This is not a tuned setting. Descriptively the smallest noise scale (c_σ 0.5)
went furthest on CXV – often to the corners at 2.75 and 3.83 km – while c_σ 2 and λ 10 all left at turn 1, so the
better direction lies outside the grid (c_σ < 0.5). The CXV step times (medians 37–44 ms) carry the load of 20
parallel jobs; the K rule rests on the single-process timing (protocol §9.5).

Runs from the pinned worktree `F1-Race-worktrees/F1-Race-wt-CXB-run` at d93b5e9 (inputs identical to the main checkout
and to phase A's run): CXV 2026-10-22 01:56–01:59 (120 laps), CBT 02:00–02:22 (408 laps); no error record. Records
(git-ignored) in that worktree's `results/CX/B/`: `tune.jsonl` (34c8dee10bbe…), `tune_lp.jsonl` (f7b2a7a8296d…),
`selected.json`, `selected_lp.json`, `test.jsonl` (f869d47d3632…), `report/CX_B.md`, `timeline.tsv`.

### Primary tests (50 test cars CBT; `mppi_lp` replicate 0)

| test | result | p | Holm |
|---|---|---|---|
| H-B1 completed laps, `mppi_lp` : `nmpc` | 0 / 50 against 12 / 50; only one completes 0 : 12 | 0.00049 | **holds** (also at 0.025) – in `nmpc`'s favour |
| H-B2 lap time on the cars both complete | no car is completed by both | – | **not testable** (the report's p = 1 is the empty test) |

The 12 discordant cars: `mppi_lp` leaves at turn 1 on 7 of them (462–472 m), at ~1.4 km on 2, at 2.2 km on 1 and
at ~3.83 km on 2; `nmpc` completes them in 75.93–78.62 s. Not cross-checked in a second environment (protocol §7).

### Budget sensitivity: four times the samples (`mppi_big`, K 4 096, offline)

`mppi_big` completes 2 of 50 (cars 47 and 49, 80.60 and 81.00 s; `nmpc` 75.93 and 76.37 s there) against `mppi_lp`'s
0 (2 : 0, p = 0.5, uncorrected) and `nmpc`'s 12 (0 : 10, p = 0.002). **Four times the samples is not enough**: the
budget accounts at most for a small part of the gap, and where the larger budget completes it is ~4.6 s slower than
the NMPC. More of its failures move from turn 1 to the 3.83 km corner (turn 1 35, ~1.4 km 5, ~3.83 km 8). Its steps
take 145 ms (mean, under load) – not a real-time controller.

### Secondary (uncorrected; 12 comparisons) and description

| controller | completed (95 %) | where the others leave |
|---|---|---|
| `tracker` (plan + L1 tracker) | 3 / 50 (0.01–0.17) | turn 1 26, ~2.7 km 13, ~3.0 km 8 (35 spins) |
| `nmpc` | 12 / 50 (0.13–0.38) | turn 1 20, ~2.2 km 2, ~2.7 km 7, ~3.83 km 8, 4.0 km 1 |
| `mpcc` (phase A's choice) | 11 / 50 (0.12–0.36) | turn 1 23, ~1.4 km 1, ~2.7 km 8, ~3.83 km 7 |
| `mppi` (white noise, untuned) | 0 / 50 (0.00–0.07) | turn 1 49, ~1.4 km 1 |
| `mppi_lp` (primary), replicates 0 / 1 / 2 | 0 / 0 / 1 of 50 | mostly turn 1 (42 / 37 / 41), then ~1.4 km |
| `mppi_big` | 2 / 50 (0.00–0.14) | turn 1 35, ~1.4 km 5, ~3.83 km 8 |

Secondary McNemar (uncorrected): `mppi_lp` : `mpcc` 0 : 11 (p 0.001); `mppi` : `mppi_lp` 0 : 0; `mppi` : `nmpc` 0 : 12
(p 0.0005); replicates 1 and 2 of `mppi_lp` against `nmpc` 0 : 12 and 0 : 11. The one replicate completion (car 49,
replicate 2, 84.45 s) shows that a lap can depend on the random stream. `nmpc` and `mpcc` complete 12 and 11 cars,
9 of them the same (descriptive; phase A's comparison).

**Mechanism, as in development (protocol §9.4, §9.7):** in one sampling iteration per 50 ms the MPPI does not follow
the large, fast input changes of turn 1 (braking from ~86 to ~28 m/s and turning in); once the car is near a limit,
every sample carries a penalty (no penalty-free sample in ~20 % of all steps on CBT, ESS median 4.4) and the update
follows a single sample. The drive-and-brake overlap of the MPPIs' applied inputs (per-lap median, mean over the laps: 300 N for
`mppi_lp`, 865 N for the white noise, against `nmpc`'s 28 N) is the one-sided noise clipping of amendment B1.2; driving and braking at
once is a freedom of this model (fixed 60 / 40 brake split) that both controllers have, not a statement about a real
car. Step times under the benchmark's load: `mppi_lp` mean 41.9 ms, P95 52.3 ms, 8 % of steps above 50 ms; IPOPT
(`nmpc`) 179 ms mean.

### Reading

* **At this budget, MPPI on the NMPC's problem does not drive these cars**: 0 of 50 for the primary and the white-noise
  version, 12 of 50 for the NMPC that solves the same problem with IPOPT – the registered test confirms on new cars
  what the nominal car showed in development. H-B2 cannot be tested.
* **The failures are not the NMPC's:** the MPPIs leave mostly at turn 1, where the NMPC loses 20 cars but passes the
  rest; the corners that decide the NMPC's other laps (2.7 and 3.83 km) the MPPI at K 1 024 reaches on 2 of 50 cars
  (replicate 0; both leave at 3.83 km), the larger budget on 10 (it leaves at 3.83 km on 8 and completes 2).
* **Limits:** one CPU implementation and budget; an uninformative tuning whose better direction lies outside the grid;
  the T-RO form keeps the nominal sequence out of the samples (one iteration can make a near-optimal sequence worse,
  §9.4); the primary was changed after the gate failed (B2) and the gate itself was waived (B3). The follow-up
  candidates (the unperturbed sequence as a sample, adaptive temperature or covariance, SMPPI, a GPU implementation,
  the 3.84 km failure mechanism) are listed in [CONTROL_METHOD_MATRIX.md](CONTROL_METHOD_MATRIX.md).

### Secondary conditions (protocol §2; uncorrected; 3 McNemar comparisons)

Run 2026-10-22 02:25–02:56 at d93b5e9 in the same worktree, 504 laps, no error record (`secondary.jsonl`
2219e5634a68…; `report/CX_B_secondary.md`). All controllers with their CBT settings, replicate 0.

| condition | `tracker` | `nmpc` | `mpcc` | `mppi` | `mppi_lp` | `mppi_big` |
|---|---|---|---|---|---|---|
| CBT_LAG30 (50 cars, 30 ms steering lag) | 3 | 0 | 1 | 0 | 0 | 1 |
| CXD (nominal car, L3 disturbances, 30 seeds) | 0 | 4 | 10 | 0 | 1 | 4 |
| CXL8 (nominal car, 4 new tracks) | 0 | 4 | 4 | 0 | 0 | 1 (São Paulo, 84.21 s; `nmpc` 79.38 s) |

* `mppi_lp` : `nmpc` (only one completes): CBT_LAG30 0 : 0 (p = 1), CXD 1 : 4 (p = 0.375), CXL8 0 : 4 (p = 0.125).
* **With the 30 ms lag almost every lap fails** (`tracker` 3, `mpcc` 1, `mppi_big` 1 of 50): `nmpc` leaves at turn 1 on 48
  of 50 cars (its model has no lag, as L6's rows without a lag model); the MPPIs as without the lag, mostly at turn 1.
* **Under the L3 disturbances the larger budget matches `nmpc`'s count** (`mppi_big` 4, `nmpc` 4 of 30, on different
  seeds – none in common – while `mpcc` completes 10); `mppi_lp` at K 1 024 completes 1, and its failures move from turn 1
  to ~1.4 km (15 of 29).
* **New tracks:** `nmpc` and `mpcc` complete all four (as in phase A); the MPPIs at K 1 024 none, `mppi_big` one.
* **Reproducibility (clarification n3):** `nmpc`'s and `mpcc`'s 68 laps on CXD and CXL8 are identical to phase A's records
  (`F1-Race-wt-CXA-run/results/CX/A/secondary.jsonl`, f3a3a5399e0a…) in every field that does not measure time – 68 of 68.
  The report generated at d93b5e9 listed 46 "differences", all of them the NaN lap time of a not-completed lap compared
  as a number (NaN ≠ NaN); the comparison was corrected (baa69cb, with a test) and the report regenerated from the same
  records with that code from the main checkout – a hand note in `timeline.tsv`, the first version kept as
  `report/CX_B_secondary_d93b5e9_nanbug.md`.

## Phase C – robust tube MPC (constraint tightening by an error tube)

Protocol [CX_C_protocol.md](CX_C_protocol.md) (revision 0 with its clarifications; amendment C1). Runs from the pinned
worktree `F1-Race-worktrees/F1-Race-wt-CXC-run` (inputs identical to the main checkout and to phase B's run): CXV tuning
2026-10-22 05:57–06:13 at 8b6b40d (`RUN_COMMIT_tune`; 60 laps), CCT and NOM 06:16–07:06 at eab7c63 (`RUN_COMMIT`;
amendment C1 added `nmpc_param` to the runner and report tables only; 306 laps); no error record. Records (git-ignored)
in that worktree's `results/CX/C/`: `tune.jsonl` (ddcc9d8a4229…), `selected.json` (d38527d58885…), `test.jsonl`
(bcf5e9f743d8…), `report/CX_C.md`, `timeline.tsv`.

**Settings chosen on CXV** (10 tuning cars): `tube` c_R 1 (10 of 10 with every c_R; the lowest mean lap on the common
cars, 79.78 s), `tube_track` c_R 0.1 (3 of 10 with every c_R – the same three cars `nmpc` completed there in phase A).

### Primary tests (50 test cars CCT): both hold

| test | result | p | Holm |
|---|---|---|---|
| H-C1 completed laps, `tube` : `nmpc` | 49 / 50 against 16 / 50; only one completes 33 : 0 | 2.3 · 10⁻¹⁰ | **holds** (also at 0.025) – in `tube`'s favour |
| H-C2 lap time on the 16 cars both complete | `tube` slower on all 16, mean +1.61 s, median +1.68 s (0.51–2.20 s) | 3.1 · 10⁻⁵ | **holds** (also at 0.025) – `tube` slower |

`nmpc`'s 34 failures: turn 1 16, ~2.2 km 2, ~2.7 km 7, ~3.83 km 5, ~4.01 km 4; `tube` leaves the track once (car 39 at
3 820 m) and completes the other 33 in 78.3–82.3 s. Not cross-checked in a second environment (protocol §7).

**The numerical-path control (amendment C1).** `nmpc_param` – `nmpc`'s problem in the tube's parameterised NLP and code
path with no tightening – completes 18 of 50; against `nmpc` only one completes 2 : 0 (p 0.5) and the lap times on the 16
common cars differ by −0.06 s on average (p 0.19). The numerical path accounts for 2 discordant cars against H-C1's 33:
H-C1 is not an artefact of it. `tube` against `nmpc_param` (the same path, only the tightening differs; descriptive):
31 : 0, +1.65 s on the 18 cars both complete.

### Secondary (uncorrected; 10 comparisons) and description

| comparison | only first : only second | McNemar p | both complete | lap time first − second | signed-rank p |
|---|---|---|---|---|---|
| `tube` vs `nmpc_reserve` (uniform η 0.88, 12 % plan) | 28 : 0 | 7.5 · 10⁻⁹ | 21 | **−1.39 s** (median −1.25) | 9.5 · 10⁻⁷ |
| `tube_track` vs `nmpc` | 7 : 0 | 0.016 | 16 | +0.12 s | 0.065 |
| `tube_track` vs `tube` | 0 : 26 | 3.0 · 10⁻⁸ | 23 | −1.34 s | 2.4 · 10⁻⁷ |
| `nmpc_reserve` vs `nmpc` | 5 : 0 | 0.063 | 16 | +2.85 s | 3.1 · 10⁻⁵ |
| `nmpc_param` vs `nmpc` (noise floor) | 2 : 0 | 0.5 | 16 | −0.06 s | 0.19 |

| controller | completed (95 %) | where the others leave |
|---|---|---|
| `tracker` | 2 / 50 (0.00–0.14) | turn 1 23, ~1.4 km 1, ~2.7 km 16, ~3.0 km 8 |
| `nmpc` | 16 / 50 (0.20–0.47) | turn 1 16, ~2.2 km 2, ~2.7 km 7, ~3.83 km 5, ~4.01 km 4 |
| `nmpc_param` | 18 / 50 (0.23–0.51) | turn 1 16, ~2.2 km 2, ~2.7 km 8, ~3.83 km 5, ~4.01 km 1 |
| `nmpc_reserve` | 21 / 50 (0.28–0.57) | turn 1 16, ~2.2 km 1, ~2.7 km 3, ~3.83 km 9 |
| `tube` | **49 / 50 (0.89–1.00)** | ~3.83 km 1 |
| `tube_track` | 23 / 50 (0.32–0.61) | turn 1 11, ~2.7 km 7, ~3.83 km 9 |

**Tube diagnostics (CCT, `tube`):** track back-off P50 0.05 m, P95 0.25 m; η_k P5 0.70 (the cap), P50 0.90; 15 % of the
predicted stages capped at η 0.70, the track back-off never; the realised lateral error of the axle points exceeds the
back-off on 6.5 % of the covered stages (a diagnostic of scale: the realised error is the re-solving NMPC's, the bound
assumes the ancillary feedback – clarification m1). The tube adds 11.5 ms per solve (mean; P99 23 ms), IPOPT ~180 ms
(offline, 20 jobs in parallel).

**Where the friction tube is capped (review n2; development diagnostic `results/CX/C/dev/capmap.json`, 36d7a6a – the
nominal car and the five development cars, `tube` c_R 1, every predicted stage attributed to its predicted position):**
73 % of all capped stages lie in the fast sections 2.6–2.8 km (73 % of the stages there capped), 2.95–3.15 km (69 %) and
3.7–3.85 km (67 %) – the corners where `nmpc` leaves at ~2.7 and ~3.83 km – and 82 % including their approaches
(2.35–2.85, 2.95–3.2 and 3.7–3.9 km); another 5.5 % lie at 3.95–4.05 km (22 % of the stages there; `nmpc` leaves at
~4.01 km on 4 cars), 3 % at 1.05–1.2 km, and 2 % at turn 1 (400–550 m; 6 % of the stages there), where `nmpc` loses
most cars but the uncapped tightening suffices (review m1 of packet 4: an earlier version gave 82 % for the three
sections alone). At the fast corners the tube asks for more reserve than
the cap of 0.70 allows: there the controller is not robust in the tube's own sense, yet completes. (Development data on
cars outside the test set; it describes the mechanism, not the test.)

### Reading

* **Tightening by the model's uncertainty turns the L2 cars' failures into completions:** 49 of 50 against 16 of 50,
  33 : 0, at a cost of 1.6 s per lap (2 %) where both complete. The numerical path alone produces 2 discordant cars
  (`nmpc_param` against `nmpc`) – small against 33.
* **It is not a uniform reserve:** the uniform 12 % reserve (`nmpc_reserve`) gains 5 cars (5 : 0, p 0.06) and costs 2.9 s;
  the tube gains 33 and is 1.4 s *faster* than the uniform reserve where both complete. The tube's η_k varies along the
  horizon and the lap (P50 0.90, P95 0.95, capped at 0.70 in the fast corners) – reserve where the uncertainty acts,
  little where it does not.
* **The friction-and-force part does most of it:** the track tube alone (`tube_track`) gains 7 cars (7 : 0, p 0.016,
  uncorrected) and loses 26 against the full tube. Under `param_tightening` η_k also lowers the axles' longitudinal-force
  limits (clarification m2), so the friction reserve and the force limits act together here and cannot be separated.
* **Against L2's lesson** ("a friction reserve does not cover an error in how the car turns", docs/L2.md §4–5): a
  *uniform* reserve indeed helps little (5 cars here, 5 in L2); a reserve shaped by the propagated uncertainty helps a lot.
  The tube's disturbance set includes the tyre-stiffness, aero-balance and downforce ranges that L2 found decisive, and
  propagates their effect through the error dynamics into the friction usage of the following stages.
* **Limits:** the tube is told the L2 parameter ranges – exactly the support of the test cars' distribution (a
  favourable prior; a mis-specified box is untested); no robustness guarantee (a linearisation, vertex bounds at the
  nominal point, caps; protocol §3.5); one track at η 0.95; offline (IPOPT without a time budget, +11.5 ms for the tube);
  the tuning asymmetry runs the other way this time (3 tube settings against `nmpc`'s 6). On the development cars `tube`
  completed 5 of 5 and `nmpc` 2 of 5 – background only, not evidence (review n3).

### Secondary conditions (protocol §2; uncorrected; 3 McNemar comparisons)

Run 2026-10-22 07:27–08:23 at eab7c63 in the same worktree, 420 laps, no error record (`secondary.jsonl`
16d58eb9e999…; `report/CX_C_secondary.md`). The five controllers of protocol §3 with their CCT settings.

| condition | `tracker` | `nmpc` | `nmpc_reserve` | `tube` | `tube_track` | `tube` : `nmpc` (p) |
|---|---|---|---|---|---|---|
| CCT_LAG30 (50 cars, 30 ms steering lag) | 2 | 4 | 3 | **45** | 17 | 41 : 0 (9 · 10⁻¹³) |
| CXD (nominal car, L3 disturbances, 30 seeds) | 0 | 4 | 3 | **28** | 7 | 24 : 0 (1.2 · 10⁻⁷) |
| CXL8 (nominal car, 4 new tracks) | 0 | 4 | 4 | 4 | 3 | 0 : 0 (1) |

* **Under the steering lag** (which no controller models; L6's 30 ms) `nmpc` leaves at turn 1 on 45 of 50 cars, `tube`
  completes 45 (failures: ~3.82 km 4, ~1.4 km 1); the uniform reserve completes 3.
* **Under the L3 disturbances** (measurement noise, gusts, friction varying along the track – none of them in the tube's
  disturbance set, which is the L2 parameter box) `tube` completes 28 of 30 (failures at ~1.4 km and turn 1), `nmpc` 4.
* **New tracks:** `nmpc`, `nmpc_reserve` and `tube` complete all four, `tube_track` three (it leaves at Austin, 1 383 m);
  `tube` is slower than `nmpc` (mean 96.71 against 93.16 s; per track +2.8 to +4.9 s), about as slow as the uniform
  reserve (97.18 s).
* **Reproducibility (clarification m3):** `nmpc`'s 34 laps on CXD and CXL8 are identical to phase B's records
  (`F1-Race-wt-CXB-run/results/CX/B/secondary.jsonl`, 2219e5634a68…) in every field that does not measure time – 34 of 34.
* Reading: the tube's tightening, sized from the L2 parameter box, also covers conditions it was not built for (a
  steering lag, stochastic disturbances) on these cars and this track – a large margin effect; its price on the nominal
  car and the new tracks is 2–5 s per lap. These are secondary, uncorrected comparisons.

## Phase D – adaptive MPC and online identification

Protocol [CX_D_protocol.md](CX_D_protocol.md) (revision 0 with the clarifications m1–m3, n1–n3 of its review). Runs from
the pinned worktree `F1-Race-worktrees/F1-Race-wt-CXD-run` at cd484bd (inputs identical to the main checkout; track and
L1 / L2 plans identical to phase C's run): CDV tuning 2026-10-22 11:05–11:28 (120 laps), CDT + NOM and CDD 11:30–14:45
(707 laps); no error record. Records (git-ignored) in that worktree's `results/CX/D/`: `tune.jsonl` (f59485d0fb5c…),
`selected.json` (b7e631a95b6f…), `test.jsonl` (d7f85902eb45…), `report/CX_D.md`, `timeline.tsv`.

**Settings chosen on CDV** (the 10 CXV cars, constant and with a drift): `adaptive` λ_μ 0.995 (19 of 20; 0.99: 17, 0.999:
14), `adaptive_bank` W 0.2 (1 of 20; 0.1 and 0.5: 0). Background only (selection bias): on the same cars with constant
parameters phase A's `nmpc` completed 3 of 10, `adaptive` 10 of 10.

**The drift's start (found after the report, packet 4):** the lap's start state projects to s = L − 1.4 · 10⁻¹¹, so
the drift field (a function of s mod L) gave the final grip f_end for the **first 10 ms step** of every drift lap (CDD,
CDV_D, DEV_D), for every controller alike, and the onset diagnostics counted from the lap start. Checked at c8035cb
(`results/CX/D/dev/cdd_rerun.json`): `adaptive` on CDD again with the field as run reproduces all 50 laps of
`test.jsonl`; with the start fixed (f = 1 up to s_on, as protocol §2 defines it) `nmpc` 0 → 0, `adaptive` 34 → 34,
`adaptive_oracle` 44 → 44, `tube` 2 → 2 completed, **no car's outcome changes** (completed laps differ by up to 0.79 s –
one different step at the start is amplified over the lap, like phase C's numerical path). By the decision rule fixed
before that check, H-D2 stands as run and the fixed version is its sensitivity check. The drift diagnostics below are
the corrected ones (lap position instead of s mod L). The CDV tuning ran with the same first step; it selected by
completions only and is not re-run.

### Primary tests: both hold

| test | set | only `adaptive` : only `nmpc` | completed `adaptive` / `nmpc` | exact McNemar p | Holm | each at 0.025 |
|---|---|---|---|---|---|---|
| **H-D1** | CDT – 50 new L2 cars, constant | **27 : 0** | 42 / 15 | 1.5 · 10⁻⁸ | holds | holds |
| **H-D2** | CDD – the same cars, grip falling during the lap | **34 : 0** | 34 / 0 | 1.2 · 10⁻¹⁰ | holds | holds |

CDT and CDD are the same 50 cars: the two tests are not independent pieces of evidence (review m2). The numerical-path
control `nmpc_param` (the adaptive NLP with the nominal car and the 5 % plan) is 0 : 0 against `nmpc` on both sets.
Not cross-checked in a second environment.

### The gap to the oracle (review m1: the main quantity for H-D1) and the other comparisons (uncorrected)

| set | controller | completed | mean lap s (completed) | against `adaptive`: only it : only `adaptive`; lap time where both complete |
|---|---|---|---|---|
| CDT | `adaptive` | 42 | 77.81 | – |
| CDT | `adaptive_oracle` | 42 | 77.89 | 3 : 3; `adaptive` − oracle −0.04 s on 39 (p 0.62) |
| CDT | `identified` (one exploration lap, full re-plan) | **50** | 78.01 | 8 : 0 (p 0.008); `adaptive` − `identified` +0.13 s on 42 (p 0.03) |
| CDT | `tube` (phase C) | 46 | 79.17 | 5 : 1 (p 0.22); `adaptive` − `tube` −1.25 s on 41 (p 10⁻⁸) |
| CDT | `nmpc` | 15 | 76.83 | 0 : 27 |
| CDT | `adaptive_bank` | 1 | 75.68 | 0 : 41 |
| CDD | `adaptive` | 34 | 82.83 | – |
| CDD | `adaptive_oracle` | 44 | 82.29 | 13 : 3 (p 0.02); `adaptive` − oracle −0.01 s on 31 |
| CDD | `tube` | 2 | 81.82 | 0 : 32 |
| CDD | `identified` | 0 | – | 0 : 34 |
| CDD | `nmpc`, `nmpc_param`, `adaptive_bank` | 0 | – | 0 : 34 each |

Failure positions. CDT: `nmpc` – turn 1 (459–472 m) 16, ~2.2 km 2, ~2.75 km 6, ~3.83 km 11; `adaptive` – 466 m 1,
~2.75 km 2, ~3.83 km 5; `adaptive_oracle` – 3 823–3 840 m 8; `tube` – ~3.82 km 2, ~1.40 km 2. CDD (`cdd_failures.json`):
`adaptive`'s 16 – before the onset 1 (466 m), during the decline 7, after its end 8; at the corners ~2.75 km 9, ~3.82 km
6; `adaptive_oracle`'s 6 – during 2, after 4. Every CDD drift has ended before 3.83 km (§9 item 7), so a split by "drift
still under way at that corner" is empty.

Adaptation (corrected drift diagnostics): on CDT `adaptive`'s |μ̂ − μ| P50 0.010, P95 0.035, the seven factors' error at
the lap end 0.005 on average, settled (error < 0.02 from then on) on 48 of 50 laps, median 12.5 s after the start; the
plan asked for more grip than the car had ("too fast", η_p > 0.95 μ + 0.01) on a third of the solves on average. On CDD
the friction estimate settles 0–60 s after the onset (median 18.4 s; 4 laps never) – 18.4 s on the completed laps,
16.4 s on the failed ones; per-lap P95 of |μ̂ − μf|, medians: 0.034 before the onset (the identification at the start),
0.037 during the decline, 0.014 after it. The failed CDD laps had a higher too-fast share (median 0.51 against 0.13) and
a somewhat lower final grip (μ·f_end median 0.786 against 0.813).

### Reading

* **H-D1: online identification makes the NMPC complete 27 more of the 50 new cars than the nominal NMPC – and costs
  almost nothing against knowing the car.** `adaptive` and `adaptive_oracle` complete the same number (42), discordant
  3 : 3, with −0.04 s where both complete: the estimation error has no measurable cost here (exact states and the
  simulator's own step as the estimator's model – an upper bound, protocol §1).
* **But the architecture's ceiling is below a full re-plan:** the oracle, handed the true car, still leaves 8 cars at
  the corner at 3.83 km – where L2's failures concentrate – while `identified`, with one exploration lap and a full
  re-plan of the minimum-time OCP for the identified car, completes all 50. The plan library adapts the plan to friction
  only; tyre stiffness, aero balance and downforce are updated in the NMPC's model but not in the plan, and that is not
  enough at that corner.
* **"Without the exploration lap" has a price (review of packet 4):** against `identified` the online controller is
  0 : 8. H-D1's scope – that online identification reaches what identification gives, without the extra lap – is not
  borne out on these cars: it reaches the oracle of its own architecture, not the re-plan. Against `tube`, which knows
  the L2 ranges, `adaptive` loses 5 : 1 (not significant) and is 1.25 s faster.
* **H-D2: when the grip falls during the lap, only the adaptive controllers complete laps:** `adaptive` 34, the oracle
  44; none for `nmpc`, `nmpc_param`, the model bank and `identified` (its identification, made before the grip fell, is
  stale – the case protocol §3.7 foresaw); `tube` 2 (its tightening is sized for μ ≥ 0.92, which the drift leaves). The
  gap to the oracle, 13 : 3 (p 0.02, uncorrected), is the cost of tracking the falling grip with a ~2 s memory (λ_μ
  0.995); slow settling alone does not separate the failed laps from the completed ones (16 against 18 s), a plan that
  was too fast more often does.
* **The model bank (D.9) does not work here:** 1 of 50 and 0 of 50, mostly off at turn 1 before any drift – the bank's
  selected friction is far from the car's (|μ̂ − μ| P50 0.10, P95 0.28; hundreds of switches per lap), so the plan is too
  fast (two-thirds of the solves). Two reasons (§9 item 4): friction is unobservable on the straights, so many members
  tie and the argmin switches among them; and 20 001 random vectors in the seven-dimensional box of L4's bounds are about
  a quarter of each range apart, so no member is close to a car. D.9 builds its bank the same way (uniform random draws,
  not a grid), but over 8 tyre and drag parameters of a scale car, with bounds set relative to the true values (±150 %)
  and the unweighted full state as its score. This is D.9's method under this problem's parameters and bounds, not a
  refutation of D.9.
* **Excitation:** one CDT car (38), which `nmpc` takes past turn 1, leaves there with `adaptive` but not with the oracle –
  its estimate overshot before the first corner had excited the tyres (too-fast share 0.88). The 16 cars `nmpc` loses at
  turn 1 all pass it with `adaptive` (10 of them complete; 6 leave later, at ~2.75 or ~3.83 km). CDT_WET85 (secondary)
  tests the case where the grip is wrong from the start.
* **Limits:** exact states and the exact model structure in the primary sets (the ceiling of online identification;
  noise and an unmodelled lag are secondary conditions); certainty equivalence; a plan library over friction only; one
  track; offline (IPOPT without a time budget; the RLS adds ~0.3 ms per 10 ms step, the bank ~14 ms); not cross-checked
  in a second environment.

### Secondary conditions (protocol §2; after the primary report; uncorrected, 10 McNemar)

Run 2026-10-22 16:05–17:52 at cd484bd in the same worktree, 840 laps, no error record (`secondary.jsonl`
a4a39006e632…, `report/CX_D_secondary.md`). None of these conditions uses the drift field.

| condition (laps) | `nmpc` | `tube` | `adaptive` | `adaptive_oracle` | `adaptive` : `nmpc` (p) | `adaptive` : `tube` (p) |
|---|---|---|---|---|---|---|
| CDT_NOISE – the L3 measurement noise only (50) | 3 | **33** | 0 | 1 | 0 : 3 (0.25) | 0 : 33 (2 · 10⁻¹⁰) |
| CDT_LAG30 – unmodelled 30 ms steering lag (50) | 1 | **44** | 6 | 4 | 6 : 1 (0.13) | 0 : 38 (7 · 10⁻¹²) |
| CDT_WET85 – μ × 0.85 from the start (50) | 0 | 1 | 3 | **44** | 3 : 0 (0.25) | 2 : 0 (0.5) |
| CDP – nominal car, L8's wet patches (30) | 1 | 7 | 1 | **10** | 0 : 0 (1) | 0 : 6 (0.03) |
| CXD – nominal car, the L3 disturbances (30) | 4 | **28** | 2 | 7 | 2 : 4 (0.69) | 0 : 26 (3 · 10⁻⁸) |

* **Reproducibility:** `nmpc`'s 30 CXD laps are identical to phase C's records (`F1-Race-wt-CXC-run/results/CX/C/
  secondary.jsonl`, 16d58eb9e999…) in every field both records have that does not measure time – 30 of 30.
* **Measurement noise (CDT_NOISE, CXD) breaks every NMPC here, the oracle included:** with the true car, `adaptive_oracle`
  completes 1 of 50 (42 without the noise), `nmpc` 3 (15); many laps end at turn 1 (`nmpc` 35 of its 47 failures, the
  oracle 23 of 49). Only `tube` holds (33 of 50, 28 of 30) –
  its tightening also covers what the noise does to the measured state. The RLS on the noisy states is biased as L6 found
  for L4's least squares (errors in variables): μ̂ − μ over the last half of each lap +0.082 on average (+0.005 … +0.153),
  so its plan is too fast on 95 % of the solves – but with the oracle failing too, the noise's effect on the state
  feedback, not the estimate alone, decides these laps.
* **An unmodelled steering lag (CDT_LAG30):** `adaptive` 6, the oracle 4, `nmpc` 1, `tube` 44. The estimator's model has
  no lag, so the parameters absorb it (the factors end 0.19 off on average, μ̂ 0.17 low) – a structure error that online
  identification cannot remove (L6's lesson); the tube's margin covers it.
* **Grip wrong from the start (CDT_WET85):** the oracle 44 of 50 – the architecture copes when the grip is known – but
  `adaptive` 3: 46 of its 47 failures are at turn 1, before any corner has excited the tyres (|μ̂ − μ| P50 0.13: the estimate is still
  near the nominal grip). This is the excitation limit of protocol §3.8 (no dual control).
* **Short wet patches (CDP, 100 m at μ × 0.75):** `adaptive` 1 of 30, as `nmpc`; the oracle 10 and `tube` 7. A 2 s memory
  cannot follow a 1.5 s patch; even knowing the grip, the friction-only plan switch completes a third.
* Reading: online identification helps where the parameters are constant or change slowly and the tyres are excited –
  the primary sets; it does not help against noise, an unmodelled structure, a grip that is wrong before the first corner,
  or short patches, where the robust tube (sized for the L2 box) is the only controller that completes laps (noise, lag)
  or none does well (wet start, patches). Secondary, uncorrected comparisons.

## Phase E – learning MPC, lap to lap: closed without the benchmark (a negative development result)

Protocol [CX_E_protocol.md](CX_E_protocol.md) (revision 0 with clarifications m1–m4, n1, n2; **amendment E1, option 1**).
The development gate of protocol §9 failed, and phase E was closed by the review without the benchmark: **H-E1 and H-E2
are not tested**; no tuning, test or secondary lap was run. Development records (git-ignored) in the main checkout's
`results/CX/E/dev/`: `laps.jsonl` (24 sequences at d64eb47), `diag.jsonl` (12 diagnostic sequences at 7507114),
`nlp.json` (5068328), `regression.json` (7507114), `summary.json`.

**What was built:** `lmpc` – the NMPC's model and constraints (η 0.95, margin 0.6) without a reference, with the terminal
state a convex combination of K stored states per stored lap (the last P = 2 laps, nearest in s; soft), the cost-to-go
of those states as terminal cost (E.2), and an affine correction of the velocity states regressed from the stored laps
(E.4); two initialisation laps (`nmpc`'s NMPC on the plans at η_p 0.73 and 0.79), then up to ten learning laps from the
same initial state, each driven 250 m past the line.

**What happened** (the nominal car NOM and the five development cars):

| variant | NOM | development cars |
|---|---|---|
| `lmpc`, K = 20 (the registered development setting) | off on learning lap 1 (442 m) | all five off on lap 1 (turn 1) |
| `lmpc`, K = 10 / 5 | lap 1 77.0 s, off on lap 2 | all five off on lap 1 |
| `lmpc_nominal` (no regression), K = 10 | lap 1 76.6 s, off on lap 2 | 2 complete lap 1, then off; 3 off on lap 1 |
| best diagnostic variant (`lmpc_nominal`, K 5, P 2, the steering angle in the terminal condition) | laps 1–3: 76.5, 76.4, 76.4 s, off on lap 4 | – |

The initialisation laps complete on every car (86.9–87.0 s, 83.1–83.5 s). The first learning lap jumps to 76.4–77.0 s –
about the lap time of `nmpc` on the 5 % plan (76.40 s); in the diagnostics one learning lap took **75.9 s, faster than
`nmpc`** – yet no sequence keeps it up: near the limit (braking into turn 1) the terminal condition becomes unreachable
(the share of solves with terminal slack above 10⁻³ rising 0.03 → 0.17 → 0.33 over the laps) and the car leaves. On the
development cars the first learning lap plans with the nominal model at η 0.95 and leaves – at turn 1 with the registered
K 20, where `nmpc` does not leave; only some `lmpc_nominal` variants left at ~3.84 km, the corner where `nmpc` leaves
three of them (CX-E §9; *corrected 2026-10-28 on the phase-I branch*); the
regression has data only at the initialisation laps' speeds and falls back to the nominal model on 38–62 % of the stages.

**Reading (review e1):** the negative result is about **this implementation** – a nonlinear NLP (IPOPT) with the nominal
nonlinear model, a convex hull of stored states as the terminal set, the terminal condition soft – not about the racing
LMPC of E.2 / E.4 as such. The suspected cause, that a convex combination of stored states is a feasible state for E.2's
linearised (ATV) model but need not be for a nonlinear one, is a hypothesis that was not tested. Follow-ups (review e3):
E.2's ATV-QP implementation; a bound on the progress per lap; consecutive laps instead of a common initial state.

## Phase F – safety filters around the L5 policies: L7's filter with retried solves (the shield part closed)

Protocol [CX_F_protocol.md](CX_F_protocol.md) (revision 0 with clarifications m1–m3, n1, n2; **amendment F1, option 1**,
with g1–g4). Run from the pinned worktree `F1-Race-worktrees/F1-Race-wt-CXF-run` at **e0f0b0a**, 2026-10-22 21:53 –
2026-10-23 02:39 (20 processes, `.venv-rl`, one process per job); 2 332 laps, **0 error records**; inputs identical to
the main checkout's (`SHA256_inputs_*.txt`); records (git-ignored) `results/CX/F/test.jsonl` (SHA-256 248c584b…), per-lap
decision records `test_dec/`, report `report/CX_F.{md,json}` (`scripts/report_cx_f.py test` at e0f0b0a), the audit of
L7's records `report/audit_l7.*` (main checkout, b94e775). **Not cross-checked** (the second environment is deferred).

**The shield (H-F1) – closed without the benchmark.** The development gate of protocol §9 failed. The shield's backup
does not drive these cars:
* the backup of revision 0 (the L1 tracker at 0.85 of the plan speed) leaves the track even on the nominal car;
* the tracker on a slower plan completes at most 9 of 12 development laps, and almost none with a 60 ms lag;
* with it as the backup, the selected policy completes 3–7 of 12 development laps instead of 12.

H-F1 is **not tested**, and was withdrawn before any T1 / T2 lap existed. The negative result concerns an MPS shield
with the L1 tracker as its backup and a nominal-model check, on these cars – not shields or MPS in general (review g1).
Also stated as found: the monitor's margin grid (0.25 / 0.5 / 1.0 m) was set without knowing that the plans themselves
come within 0.27–0.29 m of the edge.

**Checks of protocol §3 (pre-registered):**
* the 1 155 `none` laps reproduce L7's `rl_none.jsonl` lap by lap – the same outcome everywhere, failure positions and
  lap times identical;
* the 22 laps of the reproduction check of `main` reproduce L7's outcome, stop reason, distance and lap time (to four
  decimals), so **L7's `main` records are used** and no full re-run of `main` was needed.

**H-F2 (primary, the only one after amendment F1; exact signed-rank, two-sided, α 0.05): holds – `retry` completes
more laps than `main`.**
* Over the nine final checkpoints, the completed T1 + T2 laps with `retry` minus with `main` are
  8, 14, 19, 18, 23, 12, 14, 4, −1 (median +14; W⁺ = 44 of 45; exact p = 0.0078).
* It is significant at α 0.05 and also at 0.025.

| proposer | none | main (L7) | retry | retry : main (McNemar, only retry : only main, p) | lap-time cost of `retry` against `none` (laps both complete) |
|---|---|---|---|---|---|
| A_s1 | 82 | 71 | 79 | 10 : 2 (0.039) | +1.38 s (63) |
| A_s2 | 98 | 27 | 41 | 16 : 2 (0.0013) | +3.86 s (38) |
| A_s3 | 66 | 17 | 36 | 21 : 2 (7 · 10⁻⁵) | +3.83 s (31) |
| B_s1 | 100 | 62 | 80 | 19 : 1 (4 · 10⁻⁵) | +2.17 s (79) |
| B_s2 | 58 | 46 | 69 | 24 : 1 (2 · 10⁻⁶) | +2.63 s (32) |
| B_s3 | 91 | 39 | 51 | 16 : 4 (0.012) | +2.12 s (50) |
| C_s1 | 72 | 40 | 54 | 17 : 3 (0.0026) | +2.46 s (45) |
| C_s2 | 74 | 66 | 70 | 4 : 0 (0.125) | +1.77 s (55) |
| C_s3 | 61 | 55 | 54 | 5 : 6 (1) | +1.81 s (39) |
| **nine finals** | **702** | **423** | **534** | | |
| `B_s1_500k` (selected) | 102 | 51 | 75 | 27 : 3 (8 · 10⁻⁶) | +3.78 s (74) |
| bare tracker | 6 | 14 | 22 | 11 : 3 (0.057) | +4.27 s (3) |

**`retry` against `none` (secondary, uncorrected):** −3, −57, −30, −20, +11, −40, −18, −4, −7 (W⁺ = 4, exact p = 0.027).
The filter with retries still costs completed laps for eight of the nine finals, and gains for one (B_s2: 69 against
58). Over the nine finals, the retries win back 111 of the 279 laps that L7's filter cost (40 %).

**What the retries change (protocol §5):**
* **Solver fallbacks:** from 0.5–6.8 % of the decisions (`main`) to **0.05–0.3 %** (`retry`) for the nine finals –
  below the 1 % of L7's benchmark rule for every proposer.
* **The retries themselves:** of the 4 350 failed first attempts of the nine finals, 1 161 were recovered by the first
  retry and 1 768 by the second; 1 421 still fell back.
* **The failures with the filter** (L7's pb-m2, the last 3 s): 667 → 524. Those after a fallback run drop from 383 to
  55; those after interventions (in the narrow and wide sense, ordered as L7) rise from 284 to 469. 301 of the 524
  completed without a filter.
* **Interventions:** 0.30–0.38 per decision (`main` 0.33–0.40); on-switches 38–75 per lap (28–69).
* **False interventions:** 0.89–0.995 by the 3 s criterion (`main` 0.875–0.99), the same procedure as L7's (review n2;
  for `main` the report reproduces L7's numbers exactly).
* **Interventions followed by a failure within 3 s** (review m2; all eleven proposers), τ = 0 / 30 / 60 ms / T2: `retry` 2.4 / 5.5 / 15.9 /
  3.9 % of the interventions; `main` 6.2 / 6.9 / 17.0 / 6.1 %.
* **Lap time:** +1.4 to +3.9 s against `none` on the laps both complete, for the nine finals.

**Reading:**
* L7's mechanism (i), the runs of failed solves with the shifted-plan fallback, is largely the solver's:
  * retrying the failed solves (a second IPOPT instance, cold start, from the shifted plan and then from a rollout)
    removes most of the fallback runs;
  * it raises the filter's completion for eight of the nine finals (H-F2, significant over the nine);
  * it makes the filter's fallback rate usable by L7's 1 % rule.
* L7's negative result does **not** disappear:
  * with retries the filter still makes eight of the nine finals complete fewer laps than without it;
  * its failures are now mostly after interventions, 469 of 524 – mechanism (ii);
  * about nine in ten of the sampled interventions are still not needed by the 3 s criterion (an upper bound, as in
    L7).
* A description, not a test (review of packet 3): the bare tracker completes 6 / 14 / 22 laps with `none` / `main` /
  `retry` – the filter helps the weak proposer and costs the L5 policies, the direction L7's hypothesis below would
  predict.
* What remains is the filter's caution against what the policies can do, not its solver. The interpretation that the
  policies are more robust than the filter's model certifies stays L7's hypothesis; it is not tested here.
* **Time, an idealisation of both filters (review of packet 2):**
  * every decision of `main` (L7's records) and practically every one of `retry` took longer than 50 ms under the run's
    load – `retry`: mean 191–306 ms, P99 up to ~21 s, the longest 26.6 s, because the cold retries are slow;
  * the simulation applied the filtered command at once, so a real car would get it many decisions late;
  * whether a filter with retries can meet the 50 ms period is phase G's subject, and `retry` makes it harder.
* §9.4's development lap of `retry` (82.27 s on the nominal car) was development data only.
* Follow-ups (CONTROL_METHOD_MATRIX.md): a backup that drives the lagged cars, a backup CBF on it, policies trained with
  their filter.

## Phase G – real-time control: the delay-compensated RTI under L9's rule

Protocol [CX_G_protocol.md](CX_G_protocol.md) (revision 0 with clarifications m1–m3, n1, n2).

**Run:**
* WSL (L9's acados environment: Ubuntu 24.04, Python 3.12.3, casadi 3.8.1, acados v0.6.0). Pinned worktree
  `F1-Race-worktrees/F1-Race-wt-CXG-run` at **5e11634**. Driver `results/CX/G/test/cxg_run.sh`, SHA-256 835abaee….
* Started 2026-10-23 04:00, done 05:39. One step at a time (explore 105, plan 105, the primary arms 420, S1 315,
  S2 210, S3 105, S4 80).
* Real-time jobs ran in batches of 8 on vCPUs 0–7, with the arms of a case side by side; the composition of every
  batch is in `batches.tsv` and in the records.
* **0 error records in every file.** Inputs are identical to the main checkout's (`SHA256_inputs_*.txt`).
* Records (git-ignored, `results/CX/G/test/`), SHA-256:

  | file | SHA-256 |
  |---|---|
  | `eval.jsonl` | 1c5f7884… |
  | `explore.jsonl` | 5a2e6298… |
  | `plans.jsonl` | 2cac8b33… |
  | `repeat_eval.jsonl` | a6a40a44… |
  | `rl.jsonl` | 49b746d6… |
  | `load.jsonl` | 5d74d487… |

* Report `report/CX_G.{md,json}` (`scripts/report_cx_g.py --tag test` at 5e11634), written before anything was read.
* **Not cross-checked:** the WSL environment is L9's, not the deferred second environment.

All 105 identifications and re-plans succeeded. The exploration laps (`nmpc_reserve`, RTI, real time, no delay) completed
16 of 105, median distance 472 m – as in L9, the identification rests on short logs.

**H-G1 (primary; exact McNemar, Holm): holds.** At a 20 ms sensor delay, the compensated RTI `lag_identified` completes
**73 of 105** and L9's uncompensated RTI **5 of 105** (68 : 0, p = 6.8 · 10⁻²¹; significant at its Holm level 0.025).
This reproduces L9's finding (79 → 0) and removes it.

**H-G2 (primary): not shown.** Without a sensor delay, 72 against 72: the discordant cases are symmetric, 7 : 7
(p = 1), and no difference is shown. The repeat of S4 flipped 0 of 20 laps in each d = 0 arm and 1 of 20 in each
d = 20 ms arm (review m3), so the run-to-run noise at d = 0 is not estimated by it (and between two arms that each carry
noise, the expected number of discordant cases is about twice one arm's flips). Whether the 14 discordant cars are
run-to-run noise or per-car differences that cancel cannot be told from these data (review h1). The lap times on the
65 laps both complete differ by +0.002 s (signed-rank p 0.92).

| arm | completed (95 %) | τ 0 / 30 / 60 ms (of 25) | disturbances (of 30) | mean lap | feedback latency ms (mean / largest P99) | position error of the state the solve assumed (median of the lap medians) |
|---|---|---|---|---|---|---|
| `rti`, d = 0 (L9's RTI) | 72 (0.59–0.77) | 23 / 23 / 24 | 2 | 77.26 s | 2.91 / 13.3 | 0 m |
| `rti_dc`, d = 0 | 72 (0.59–0.77) | 22 / 23 / 23 | 4 | 77.35 s | 3.37 / 15.0 | 4 · 10⁻⁷ m |
| `rti`, d = 20 ms | **5** (0.02–0.11) | 1 / 0 / 4 | 0 | 78.91 s | 3.02 / 22.7 | 1.21 m |
| `rti_dc`, d = 20 ms | **73** (0.60–0.78) | 23 / 24 / 24 | 2 | 77.23 s | 3.49 / 16.2 | 3 · 10⁻⁶ m |
| S1: `rti`, d = 40 ms | 0 | 0 / 0 / 0 | 0 | – | 3.67 / 22.0 | 3.23 m |
| S1: `rti_dc`, d = 40 ms | 74 (0.61–0.79) | 23 / 24 / 25 | 2 | 77.20 s | 3.26 / 14.1 | 9 · 10⁻⁶ m |
| S1: `rti_dc` assuming 20 ms, d = 40 ms | 3 (0.01–0.08) | 1 / 0 / 2 | 0 | 77.42 s | 3.23 / 16.2 | 1.21 m |
| S2: `nmpc` RTI, d = 20 ms | 0 | 0 / 0 / 0 | 0 | – | 2.90 / 15.5 | 1.55 m |
| S2: `nmpc` RTI compensated, d = 20 ms | 8 (0.03–0.14) | 8 / 0 / 0 | 0 | 76.06 s | 3.25 / 14.0 | 2.5 · 10⁻⁴ m |
| S3: `B_s1_500k`, d = 20 ms (context) | 89 | | | | | |

**Secondary (uncorrected, descriptive):**
* S1, compensated (true d) against uncompensated at 40 ms: 74 : 0. With a wrong delay (20 ms assumed, 40 ms real),
  the compensation keeps only 3 laps (71 : 0 against the true d). Its assumed state is then as far off as the
  uncompensated one at 20 ms (1.21 m).
* S2, `nmpc` (the nominal model, no identification): 8 : 0 for the compensation at 20 ms (p = 0.008). Its 8 laps are
  all on τ = 0 cars, and it is no better than L9's `nmpc` RTI without a delay (10 / 105).
* `rti_dc` at 20 ms against `rti` without a delay: 6 : 5 (p = 1) – the compensation takes the lap count back to the
  no-delay level.
* S3: the RL policy, which has no compensation, completes 89 of 105 at 20 ms (L9: 88 of 105).

**Timing (section 5; review m1):**
* **Application delay** – the time from a solve's start to its command taking effect: 10 ms for at least 99.5 % of the
  solves in every primary arm and 95–99.8 % in the secondary arms (20 ms for most of the rest; a handful of 30–120
  ms).
* **End-to-end delay** (d + application delay): 10, 30 and 50 ms for d = 0, 20, 40 ms.
* **Feedback latency** with the prediction: 3.2–3.5 ms on average (without: 2.9–3.7 ms). The prediction itself costs
  0.44–0.63 ms on average, its largest single value 29 ms. 0.3–0.6 % of the compensated solves were late (above L̂ =
  10 ms); 2.4 % with the wrong d.
* No deadline problem: busy (feedback + preparation) is 3.1–3.9 ms on average against the 50 ms period.
* **Actuation delay** is the cars' steering lag, τ ∈ {0, 30, 60 ms}, identified in the plans (`plans.jsonl`).

**Reading:**
* **A known sensor delay is the RTI's weakness, and a model-based prediction over it removes that weakness on these
  laps.** At 20 and 40 ms the compensated RTI completes as many laps as the uncompensated one without any delay (73 /
  74 against 72), while the uncompensated one collapses (5, 0). This is the step that L9's reading left open.
* **H-G2: what it compares (correcting clarification m1).** Under L9's rule with amendment A1, the plan is already
  aligned to the time the command takes effect, so both arms act after 10 ms (≥ 99.7 % of the solves at d = 0). H-G2 therefore
  compares not the application time but **whether the state is predicted to t_k + L̂ with the plan's command before the
  solve**. Without a sensor delay: 72 against 72, 7 : 7, no difference shown.
* **Why it works this well here – and its limit.** The state each compensated solve assumed was within micrometres of
  the true state: on these simulated cars the identified model is nearly exact, because the cars differ from the
  nominal one only in the parameters the identification estimates, including a first-order lag. Under that condition
  the prediction carries no model error over 30–50 ms. A real car's unmodelled dynamics would add error over the same
  window; this test does not measure that.
* **The delay must be known.** With a wrong delay the compensation loses almost everything (S1: 3 of 105), as wrong as
  no compensation at half the delay. G.7's online estimate of the delay is the matching follow-up.
* **Scope (review m2):** the identification came from exploration laps without the delay, shared by the arms, as in
  L9's delay condition. The effect measured is that on the evaluated lap. The L3 disturbance laps fail in every arm (0–4
  of 30); the compensation does not change that. Within GT, the no-delay RTI is close to L9's (72 against 79 on other
  laps; 70 of 75 against L9's 73 of 75 on the car laps, 2 against 6 of 30 on the disturbance laps).
* **Development data** (§9) on the development cars had shown the same picture (8 of 13 against 1 of 13).

## Phase H – the ATV-QP learning MPC (the first frontier method): closed without the benchmark (a negative development result)

Protocol [CX_H_protocol.md](CX_H_protocol.md): revision 0 with clarifications m1–m3, n1, n2; **amendment H1 accepted**,
with i1–i3. The development gate of protocol §9 – phase E's gate – failed, and phase H was closed without the benchmark,
by the default that clarification m1 fixed in advance. **H-H1 and H-H2 are not tested**; no tuning, test or secondary lap
was run.

Development records (git-ignored) are in the main checkout's `results/CX/H/dev/`:
* `qp.json`, `nlp_repro.json` and `laps.jsonl` (24 jobs) at 3c69eb0;
* `diag.jsonl` (6 diagnostic sequences) at 8b17829;
* `report/` (with `gate.json`) regenerated at cfe57ad.

**What was built:** `atv` solves phase E's problem row for row as one QP per step:
* the dynamics and constraints are linearised at a candidate z̄;
* the cost is kept exactly (its Hessian is constant), plus 10⁻⁶‖z − z̄‖²;
* the QP is solved with OSQP.

The candidate follows E.2:
* the previous solution shifted by one step, with the terminal state Σ λᵢ succ(xᵢ) of the previous λ, and its inputs
  formed the same way;
* at a lap's first step, the fastest stored lap's first N + 1 states.

The safe set holds the K nearest states by s around the candidate's terminal state, from the last P = 2 laps, without
each lap's last point. `atv_nominal` is the same without E.4's correction. The laps are as in phase E: two
initialisation laps, then up to ten learning laps from the same initial state, each driven 250 m past the line.

**What happened** (the nominal car NOM and the five development cars; K 20):

| variant | NOM | development cars |
|---|---|---|
| `atv` (registered: OSQP) | off on learning lap 1 (128 m) | all five off on lap 1 (101–439 m) |
| `atv_nominal` (registered: OSQP) | off on lap 1 (410 m) | all five off or spun on lap 1 (315–465 m) |
| phase E's `lmpc` (IPOPT; reproduced exactly) | off on lap 1 (442 m) | all five off on lap 1 (465–470 m) |
| diagnostic: `atv` with qpOASES and a box on the input step (0.2 rad/s, 1 000 N per stage) | off on lap 1 (2 765 m) | DEV_car1 off on lap 1 at 2 658 m; the other four at turn 1 (470–474 m) |

* The initialisation laps completed on every car.
* With OSQP, 21–115 QPs per lap stopped at the solver's iteration limit; the shifted plan was applied for them.
* With qpOASES and no box (a scratch run on NOM's first learning lap), every QP was solved, but the full step went to
  the input bounds, far outside the tyre model's
  linearisation.
* With the box, over full sequences:
  * the median linearisation error stayed small: |ds| 0.3–0.6 m and |dn| 0.6–1.4 m per lap;
  * its P95 in v_x was 13–80 m/s;
  * the terminal slack exceeded 10⁻³ in 15–39 % of the solves;
  * qpOASES failed no QP on the four cars that left at turn 1, but failed 18 on NOM and 17 on DEV_car1, the two that got
    to ~2.7 km. The shifted plan was applied for those.
* Solves took 280–335 ms on average (the diagnostic 346–507 ms), far above the 50 ms period. This is an offline
  benchmark, and phase G's real-time rule was not part of it.

**Reading (review i1, i2):**
* **Scope (i1):** the negative result concerns **an ATV-QP form of phase E's problem, solved with OSQP, with E.2's
  candidate, on these cars**. It is not a result about LMPC as such. Nor is it about E.2's own implementation, which has
  its own local-regression model of the velocity states, a hard terminal constraint, and N = 12 × 100 ms.
* **e1 (i2):** the diagnostic, which is not the registered design, also fails learning lap 1. On the mismatched cars it
  fails at much the same places as phase E's NLP. This **weakens** e1 – that the convex hull's infeasibility under the
  nonlinear model was the decisive cause – but **does not refute** it.
* **Observation, not a conclusion:** every variant plans its first learning lap at the nominal model's η 0.95 limit,
  driven by the stored laps' time-to-go.

**Follow-ups (i3):** see CONTROL_METHOD_MATRIX.md. Two LMPC phases have now failed at their development gate. **A later
LMPC phase must first show, on NOM, that its first learning lap completes reliably, before a benchmark is registered.**

## Phase I – the learning MPC's follow-ups: closed without a benchmark (I-1's gate failed; a reproduction check under the published conditions)

Protocol [CX_I_protocol.md](CX_I_protocol.md):
* revision 0, with clarifications m1, m2, n1, n2;
* revision 1 (section 10, the reproduction check), with m1–m3, n1–n3 and the clarifications of the packet-4 review.

**No benchmark was registered. H-I1 and H-I2 are not tested, and no IV or IT lap was run.** The user closed the LMPC
line after the reproduction check (2026-11-01; protocol section 11).

Records (git-ignored) are in the main checkout's `results/CX/I/`:
* `dev/` – I-1's development checks, at 62fc41d;
* `repro/` – the reproduction check at b34eb97, with its diagnostics at 418bf3a, bca7802 and aae7f30;
* `smoke/` and `repro_smoke/` – the disclosed scratch smokes;
* `final/` – the pytest outputs.

**I-1 – a soft speed bound per lap on phase E's NLP.** The bound is v_x ≤ (1 + ε) v_ref(s), where v_ref is the faster
of the last two stored laps.

| car | `pb`, ε 0.04 (the gate) | `pb_nominal`, no regression (descriptive) |
|---|---|---|
| NOM | 80.83 s, then off on lap 2 | 80.42, 78.51, 77.60, 77.65 s, then off on lap 5 |
| DEV (5 cars) | off on lap 1 or 2 (all five) | best laps 77.3–80.1 s, then off on laps 3–5 |

* Stage A passed: NOM's first learning lap completed at ε 0.02, 0.04 and 0.08 (81.84, 80.83, 79.07 s).
* Stage B failed, so I-1 was closed (rule 4).

**The reproduction check** – the user's request; a diagnostic on NOM, one deterministic run per arm. It used the
published racing LMPCs' conditions:
* two centre-line laps at 8 m/s;
* consecutive laps;
* 40 learning laps;
* the nominal model, equal to the plant.

| arm | learning laps | end | reading |
|---|---|---|---|
| R1 – phase E's NLP, start at 8 m/s | 150.81, 88.63, 80.39 s | off on lap 4 at 3 973 m with the solver succeeding: its own plan crossed the edge | (c1) |
| R2 – the same, from phase E's start | – | off on lap 1 at turn 1 (K 20; with K 10: 76.72 s, then off at turn 1) | (c2) |
| R3 – phase H's ATV-QP, start at 8 m/s | – | off on lap 1 at 8 m/s: OSQP at its iteration limit on 533 of 538 QPs | (c2) |

**Reading:**
* **Scope.** The negative result concerns this implementation: phase E's NLP with a soft convex hull of the K nearest
  stored states as its terminal set, on these simulated cars. The check ran one run per arm, on NOM only.
  * It is not a result about LMPC as such.
  * Nor is it about E.2's or E.4's implementations. Their scale and hardware, E.2's regression, H.1's safe-set details,
    and soft track bounds that let a car pass the edge remain unreproduced (CX_I §10.6).
* **Under the published conditions, this LMPC learned fast and failed while approaching the limit**, as in phase E and
  I-1. The slow start and the consecutive laps delayed the failure to learning lap 4; they did not remove it.
  * The explanation is a soft hull without recursive feasibility once the car is faster than its data.
  * It is consistent with the replay of the failed lap, but not tested.
* **On NOM, every LMPC sequence of phases E–I left the track by its fifth learning lap.** The only exception is the
  scratch smoke, which stopped after two laps by design. The sequences' fastest laps came near
  or below `nmpc`'s 76.40 s, and each was followed by leaving the track on the next lap:
  * 76.64 s with phase E's registered setting (`lmpc_nominal` K 10, learning lap 1);
  * 75.9 s in phase E's unregistered diagnostics (K 5, P 1; CX_E §9).
* **R3 is an implementation limit** (phase H's OSQP settings). E.2's QP form remains untested under the published
  conditions.
* **The run-out approximation** was poor after the slow start: the laps braked before the line. So whether R1 would
  first have reached the limit is open ((b) against (c1)); the conclusion is unchanged.
* Not cross-checked in a second environment.

**Follow-ups:** see CONTROL_METHOD_MATRIX.md ("From phase I"). The LMPC line is closed by the user's decision; the open
points are recorded there, not scheduled.
