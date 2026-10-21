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

**Development checks only** (nominal car, Spielberg, code 0061e79; table and reading in
[CX_A_protocol.md](CX_A_protocol.md) §12): the standard formulation with a terminal speed bound leaves
the track at turn 1 with either reference path; with the racing line, a speed bound at every stage and
q_c ≥ 0.5 it completes the lap (76.05 s at q_c 0.5, 75.76 s at q_c 2; the NMPC on the same lap 76.05 s).
No benchmark yet: the protocol is in review.
