# Control expansion – status (resume point)

Last updated: 2026-10-22 (phase B: MPPI implemented and unit-tested; amendment B1 proposed, awaiting the reviewer). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** B – MPPI, branch `cx/B-mppi` from `main` b68d0a4: literature (§B of the literature review) and
  protocol revision 0 (`docs/CX_B_protocol.md`) **APPROVE** (0b94d1a) with clarifications m1, m2, n1–n4 written in;
  `control/mppi.py`, `experiments/cx_b.py`, the runner / report and their tests written (57146e4, d80e1a6); development
  checks 9.1 and 9.2 done – 9.2 found that the NMPC drives and brakes at once in its braking zones, so **amendment B1**
  (MPPI samples the NMPC's three inputs) was proposed (8d86ff2); the K timing and the development laps wait for the
  reviewer's decision. A done (final review of e2cea5d
  APPROVE; `cx/A-mpcc` merged to `main` --no-ff as **b68d0a4**, 2026-10-21).
* **Original reviewer:** session `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), the
  reviewer of L7–L9 – reached by SendMessage with this id (its pipe address changes after restarts).
* **L10:** PAUSED — Preserved for Future Development (branch `layer/L10`, protocol revision 1 binding,
  pause note 3ba9087). Nothing of L10 is touched by the expansion.

## Modules

| phase | module | developed | unit-tested | closed-loop checked | reviewed | accepted |
|---|---|---|---|---|---|---|
| A | `control/mpcc.py` (`DynamicMPCC`, `MPCCController`, `path_errors`) | yes (0061e79) | yes – `tests/test_mpcc.py`, 8 tests | development laps; CXV tuning; CXT benchmark; secondary conditions | **APPROVE** (f4d1ee9 … e2cea5d) | **yes – phase A completed** |
| A | runner and report (`experiments/cx_a.py`, `scripts/run_cx_a.py`, `scripts/report_cx_a.py`) | yes | yes – `tests/test_cx_a.py`, 7 tests (5 at 6bcc6ef) | smoke on the nominal car; the CXV / CXT / secondary runs | **APPROVE** (6bcc6ef … e2cea5d) | **yes** |
| A | `mpcc_ca` (CiMPCC-based, amendment A1) and the secondary conditions; soft speed bound (amendment A2) | yes | yes – `tests/test_mpcc.py` 8, `tests/test_cx_a.py` 7 | smoke on cases outside every set (`results/CX/A/dev/secondary_smoke.jsonl`); A2 diagnostic on CXV cars 2 / 5 / 7; CXT | **APPROVE** (94cba68 … e2cea5d) | **yes** |
| B | `control/mppi.py` (`MPPI`), `experiments/cx_b.py`, `scripts/run_cx_b.py`, `scripts/report_cx_b.py`, `scripts/cx_b_dev.py` | yes (57146e4, d80e1a6) | yes – `tests/test_mppi.py` 11, `tests/test_cx_b.py` 5 | development checks 9.1 (model = FrenetMPC's step, 7 · 10⁻¹⁶) and 9.2 (cost = NLP objective with eliminated slacks, 1.3 · 10⁻¹⁴); no MPPI lap yet | protocol rev 0 **APPROVE** (0b94d1a); amendment B1 proposed (8d86ff2) | – |
| C–G | – | not started | – | – | – | – |

## Review record

| date | commit | scope | verdict | findings / fixes |
|---|---|---|---|---|
| 2026-10-21 | f4d1ee9 | phase A packet 1: audit and roadmap docs, MPCC code and tests, development checks, CX-A protocol revision 0 | code, tests, math and workflow conventions **APPROVE**; protocol rev 0 CHANGES_REQUESTED | cx-M1 tuning asymmetry (nmpc untuned) → nmpc gets a 6-setting grid with the same rule; cx-M2 the speed is bounded by the plan → question and scope rewritten, speed-bound activity and speed / plan-speed ratio reported; cx-m1 q_c 0.05 kept in the grid, stated; cx-m2 H-A2 conditional on completion, discordant cars listed; cx-m3 mpcc_ca amendment before CXT; cx-m4 sections renumbered (development checks now §9) |
| 2026-10-21 | e8a5cd5 | CX-A protocol revision 1 | **APPROVE** (binding) | nit: state the grid sizes (24 / 6) and the selection bias in the scope – done; stage-0 bound activity is a measured quantity (the bound acts on stages 1 … N) |
| 2026-10-21 | 6bcc6ef | phase A packet 2: runner and report code, tests, nominal-car smoke | **APPROVE** | m1 the full tie-break order and the skip rule written into protocol §4 before CXV (done, protocol clarification); before the CXT report: m2 the report refuses missing cars / error records and checks the settings against selected.json, m3 the missing §5 columns, m4 the force-increment definition and a 50 ms difference column, n1 fallback solves out of the stage statistics, n2 the same cars for every setting asserted; secondary conditions and the mpcc_ca amendment reviewed before they run |
| 2026-10-21 | 94cba68 | phase A packet 3: CXV tuning results (8babe7b), review fixes m2–m4 / n1 / n2 (a1f0a73), amendment A1 (mpcc_ca after CiMPCC) and the secondary-condition code (549e4bd), amendment A2 (soft speed bound, re-tuning of the MPCCs on CXV) | **APPROVE** (conditions c1–c3, done in c7c802f; nits n3–n5 done) | the first CXV tuning accepted as the basis of A2; A2 and A1 agreed |
| 2026-10-21 | c7c802f (run) | phase A packet 4: A2 re-tuning of the MPCCs (480 laps) and the mpcc_ca tuning (60 laps) on CXV | **APPROVE** | report to state: choices on grid edges, mpcc_ca ties at noise level, hard cars / low power of H-A1, CXV lap times selection-biased; nit: timeline lacks 'start tune_ca' – hand note added |
| 2026-10-21 | c7c802f (run) | phase A packet 5: CXT (130 laps, test.jsonl 2e560c407137…) and its report; the secondary conditions running | **APPROVE** (all records and statistics recomputed) | m1 mpcc_centre's failure places on CXT vs the nominal car, n1 reference-path statement limited, n2 'all five controllers here', n3 mean laps on different cars – fixed in d7ea54d |
| 2026-10-21 | e2cea5d | phase A packet 6: the secondary conditions (236 laps, secondary.jsonl f3a3a5399e0a…; the missing CXL8 inputs and the 4 re-run jobs) and the final phase-A review request | **APPROVE** | the missing CXL8 inputs: failures before any simulation, inputs identical to the main checkout and the L8 run, handling and disclosure accepted |
| 2026-10-21 | e2cea5d | **phase A final review** (whole branch vs main c010887: 14 commits, 16 files, +2404 / −2; only STATUS.md among the existing files) | **APPROVE** | the three standing checks pass (.venv 70 + 4 skips, .venv-rl all); nit: status lines updated before the merge |
| 2026-10-21 | b68d0a4 | merge check of phase A (main) | correct | nit: the runner row's test count (5 vs 7) – fixed in d75f037 |
| 2026-10-21 | 0b94d1a | **phase B packet 1**: literature B.1–B.11, CX-B protocol revision 0 (no code) | **APPROVE** (binding) | m1 (a) no valid sample → U only shifted, counted, unit-tested; (b) K rule selects nothing → amendment, timing at the grid centre; m2 K₀ = max(1, ⌊αK⌋); n1 seed 10 (T1new) in the list; n2 mppi_lp filter transient; n3 CXD / CXL8 re-runs expected identical; n4 scope: CPU numpy implementation – all written into the protocol before any timing or development lap |

## Tests

`tests/test_mpcc.py` (8) and `tests/test_cx_a.py` (7) at e2cea5d (6 and 5 at 6bcc6ef), and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79 and the secondary-path smoke (549e4bd); `results/CX/A/smoke/` –
the runner's smoke on the nominal car. CXV tuning (8babe7b): worktree `F1-Race-worktrees/F1-Race-wt-CXA-tune`,
`results/CX/A/{tune.jsonl, selected.json, report/tune.md, RUN_COMMIT, SHA256_inputs*.txt, timeline.tsv}`.
The first CXV selection is kept as `selected_first.json` and `report/tune_first.md` when the A2 re-tuning
writes `selected.json` / `report/tune_a2.md` (review c1). CXT: `test.jsonl`, `report/CX_A.md` (2026-10-21 21:33–21:50); secondary conditions: `secondary.jsonl`,
`report/CX_A_secondary.md` (21:51–22:16). All git-ignored.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall.
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. Phase B (MPPI): protocol revision 0 APPROVE (0b94d1a; clarifications written in) – now `control/mppi.py`.
2. Then `control/mppi.py`, unit tests, the development checks of CX-B §9 (K fixed by §3.4), the runner and report code
   (packet 2); CXV tuning (packet 3); CBT (packet 4); secondary conditions (packet 5); final review, merge.
