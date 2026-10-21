# Control expansion – status (resume point)

Last updated: 2026-10-21 (after the review of packet 1). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** A – dynamic MPCC (`cx/A-mpcc`, from `main` c010887).
* **Original reviewer:** session `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), the
  reviewer of L7–L9 – reached by SendMessage with this id (its pipe address changes after restarts).
* **L10:** PAUSED — Preserved for Future Development (branch `layer/L10`, protocol revision 1 binding,
  pause note 3ba9087). Nothing of L10 is touched by the expansion.

## Modules

| phase | module | developed | unit-tested | closed-loop checked | reviewed | accepted |
|---|---|---|---|---|---|---|
| A | `control/mpcc.py` (`DynamicMPCC`, `MPCCController`, `path_errors`) | yes (0061e79) | yes – `tests/test_mpcc.py`, 6 tests | development laps only (nominal car, 8 configurations; CX_A_protocol §9) | **APPROVE** (code, tests, math; f4d1ee9) | code yes; the phase is not completed (benchmark pending) |
| A | runner and report (`experiments/cx_a.py`, `scripts/run_cx_a.py`, `scripts/report_cx_a.py`) | yes | yes – `tests/test_cx_a.py`, 5 tests | smoke on the nominal car (4 controllers, `results/CX/A/smoke/`) | requested (packet 2) | no |
| A | curvature-aware MPCC | no – waits for the full text of Ma et al. 2026 (protocol §3) | – | – | – | – |
| B–G | – | not started | – | – | – | – |

## Review record

| date | commit | scope | verdict | findings / fixes |
|---|---|---|---|---|
| 2026-10-21 | f4d1ee9 | phase A packet 1: audit and roadmap docs, MPCC code and tests, development checks, CX-A protocol revision 0 | code, tests, math and workflow conventions **APPROVE**; protocol rev 0 CHANGES_REQUESTED | cx-M1 tuning asymmetry (nmpc untuned) → nmpc gets a 6-setting grid with the same rule; cx-M2 the speed is bounded by the plan → question and scope rewritten, speed-bound activity and speed / plan-speed ratio reported; cx-m1 q_c 0.05 kept in the grid, stated; cx-m2 H-A2 conditional on completion, discordant cars listed; cx-m3 mpcc_ca amendment before CXT; cx-m4 sections renumbered (development checks now §9) |
| 2026-10-21 | e8a5cd5 | CX-A protocol revision 1 | **APPROVE** (binding) | nit: state the grid sizes (24 / 6) and the selection bias in the scope – done; stage-0 bound activity is a measured quantity (the bound acts on stages 1 … N) |
| 2026-10-21 | (packet 2 commit) | phase A packet 2: runner and report code, tests, nominal-car smoke | requested | – |

## Tests

`tests/test_mpcc.py` (6) and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79; `results/CX/A/smoke/` – the runner's smoke on the
nominal car (4 laps). Both git-ignored. No benchmark results.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall.
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. Protocol revision 1 binding (e8a5cd5).
2. Review of the runner / report code (packet 2); then tuning on CXV (protocol §4) from a pinned worktree,
   results to the reviewer; the mpcc_ca amendment; then CXT.
3. Then phase B (MPPI): literature entries, formulation, tests, development laps, protocol.
