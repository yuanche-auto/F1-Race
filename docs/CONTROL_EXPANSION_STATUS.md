# Control expansion – status (resume point)

Last updated: 2026-10-22 (after the review of packet 2). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** A – dynamic MPCC (`cx/A-mpcc`, from `main` c010887).
* **Original reviewer:** session `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), the
  reviewer of L7–L9 – reached by SendMessage with this id (its pipe address changes after restarts).
* **L10:** PAUSED — Preserved for Future Development (branch `layer/L10`, protocol revision 1 binding,
  pause note 3ba9087). Nothing of L10 is touched by the expansion.

## Modules

| phase | module | developed | unit-tested | closed-loop checked | reviewed | accepted |
|---|---|---|---|---|---|---|
| A | `control/mpcc.py` (`DynamicMPCC`, `MPCCController`, `path_errors`) | yes (0061e79) | yes – `tests/test_mpcc.py`, 6 tests | development laps only (nominal car, 8 configurations; CX_A_protocol §9) | **APPROVE** (code, tests, math; f4d1ee9) | code yes; the phase is not completed (benchmark pending) |
| A | runner and report (`experiments/cx_a.py`, `scripts/run_cx_a.py`, `scripts/report_cx_a.py`) | yes | yes – `tests/test_cx_a.py`, 5 tests | smoke on the nominal car (4 controllers, `results/CX/A/smoke/`) | **APPROVE** (6bcc6ef) | code yes; the phase is not completed |
| A | `mpcc_ca` (CiMPCC-based, amendment A1) and the secondary conditions; soft speed bound (amendment A2) | yes | yes – `tests/test_mpcc.py` 8, `tests/test_cx_a.py` 7 | smoke on cases outside every set (`results/CX/A/dev/secondary_smoke.jsonl`); A2 diagnostic on CXV cars 2 / 5 / 7 | requested (packet 3) | no |
| B–G | – | not started | – | – | – | – |

## Review record

| date | commit | scope | verdict | findings / fixes |
|---|---|---|---|---|
| 2026-10-21 | f4d1ee9 | phase A packet 1: audit and roadmap docs, MPCC code and tests, development checks, CX-A protocol revision 0 | code, tests, math and workflow conventions **APPROVE**; protocol rev 0 CHANGES_REQUESTED | cx-M1 tuning asymmetry (nmpc untuned) → nmpc gets a 6-setting grid with the same rule; cx-M2 the speed is bounded by the plan → question and scope rewritten, speed-bound activity and speed / plan-speed ratio reported; cx-m1 q_c 0.05 kept in the grid, stated; cx-m2 H-A2 conditional on completion, discordant cars listed; cx-m3 mpcc_ca amendment before CXT; cx-m4 sections renumbered (development checks now §9) |
| 2026-10-21 | e8a5cd5 | CX-A protocol revision 1 | **APPROVE** (binding) | nit: state the grid sizes (24 / 6) and the selection bias in the scope – done; stage-0 bound activity is a measured quantity (the bound acts on stages 1 … N) |
| 2026-10-21 | 6bcc6ef | phase A packet 2: runner and report code, tests, nominal-car smoke | **APPROVE** | m1 the full tie-break order and the skip rule written into protocol §4 before CXV (done, protocol clarification); before the CXT report: m2 the report refuses missing cars / error records and checks the settings against selected.json, m3 the missing §5 columns, m4 the force-increment definition and a 50 ms difference column, n1 fallback solves out of the stage statistics, n2 the same cars for every setting asserted; secondary conditions and the mpcc_ca amendment reviewed before they run |
| 2026-10-21 | (packet 3 commit) | phase A packet 3: CXV tuning results (8babe7b), review fixes m2–m4 / n1 / n2 (a1f0a73), amendment A1 (mpcc_ca after CiMPCC) and the secondary-condition code (549e4bd), amendment A2 (soft speed bound, re-tuning of the MPCCs on CXV) | requested | – |

## Tests

`tests/test_mpcc.py` (6) and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79 and the secondary-path smoke (549e4bd); `results/CX/A/smoke/` –
the runner's smoke on the nominal car. CXV tuning (8babe7b): worktree `F1-Race-worktrees/F1-Race-wt-CXA-tune`,
`results/CX/A/{tune.jsonl, selected.json, report/tune.md, RUN_COMMIT, SHA256_inputs*.txt, timeline.tsv}`.
The first CXV selection is kept as `selected_first.json` and `report/tune_first.md` when the A2 re-tuning
writes `selected.json` / `report/tune_a2.md` (review c1). All git-ignored. No CXT result.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall.
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. Protocol revision 1 binding (e8a5cd5).
2. The reviewer's verdict on packet 3; then (if A2 is approved) the A2 re-tuning of the MPCCs on CXV and the
   mpcc_ca tuning, their results to the reviewer; then CXT from a pinned worktree; the report; the secondary conditions.
3. Then phase B (MPPI): literature entries, formulation, tests, development laps, protocol.
