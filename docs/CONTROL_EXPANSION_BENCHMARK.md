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

`mpcc_centre` against `nmpc` 0 : 7 (p 0.016) and against `mpcc` 0 : 6 (p 0.031); `tracker` against `nmpc` 0 : 9
(p 0.004); `mpcc_ca` against `mpcc` 0 : 0 and against `nmpc` 1 : 2 (p 1). Solver: `nmpc` 5 fallbacks over the 25
laps, `mpcc` 16, `mpcc_ca` 13, `mpcc_centre` 35; mean solve 174–197 ms, every solve above 50 ms (offline IPOPT);
the MPCCs' P99 reaches ~4 s. The speed bound of `mpcc` is active in 46 % of the solves at the measured stage 0 and
in 43 % of the predicted stages. On the nominal car (development car, not a test): `mpcc` 75.85 s, `mpcc_ca`
75.94 s, `nmpc` with its CXV setting 76.40 s, `tracker` 75.57 s, `mpcc_centre` off track.

### Reading

* **No difference in completion is shown, and this test had little power.** Both controllers complete about 40 %
  of the test cars, and only 3 cars separate them. A 1 : 2 split says nothing either way; it is not evidence that
  the two are equally reliable.
* **The MPCC is not slower where both complete** – faster on 6 of the 8 cars by about 0.3 s, not significant at
  n = 8. On the tuning cars the same order appeared (76.78 against 77.42 s on cars 1, 3, 9), but those numbers are
  after the selection and biased.
* **The failures sit at the same few corners for every controller** (turn 1, ~2.2 / 2.7 km, ~3.83 km), and the same
  cars fail – the cars of the L2 ranges are hard for any controller that plans at η = 0.95 with the nominal model.
  The difference between the methods is small compared with that.
* **The reference path matters more than the weights:** with the centre line as reference (Liniger's choice) the
  MPCC fails at turn 1 on almost every car, as it did on the nominal car in development.
* **The curvature-aware term adds nothing measurable here:** `mpcc_ca` completes the same cars as `mpcc`; its tuning
  ties were within 0.08 s. Its β is taken at the car's position for a 1.5 s horizon and cannot anticipate a corner
  (adaptation of CiMPCC, amendment A1).
* **Limits of the comparison:** both chosen settings lie on the edge of their grids (`nmpc` c 2, `mpcc` q_c 8 and
  q_v 2), so neither is shown to be tuned to its optimum; the tuning freedom was not symmetric (48 MPCC settings in
  two rounds with a change of formulation between them, 6 for the NMPC; protocol §1); the MPCC's speed is bounded by
  the plan's, so it chooses its line and its speed below the plan's, not its own speed profile; offline (no compute
  budget – an IPOPT MPCC solve takes ~0.2 s, see phase G).
