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
* **New tracks:** every MPC completes all four; `tube` is slower than `nmpc` (mean 96.71 against 93.16 s; per track +2.8 to
  +4.9 s), about as slow as the uniform reserve (97.18 s); `tube_track` leaves at Austin (1 383 m).
* **Reproducibility (clarification m3):** `nmpc`'s 34 laps on CXD and CXL8 are identical to phase B's records
  (`F1-Race-wt-CXB-run/results/CX/B/secondary.jsonl`, 2219e5634a68…) in every field that does not measure time – 34 of 34.
* Reading: the tube's tightening, sized from the L2 parameter box, also covers conditions it was not built for (a
  steering lag, stochastic disturbances) on these cars and this track – a large margin effect; its price on the nominal
  car and the new tracks is 2–5 s per lap. These are secondary, uncorrected comparisons.
