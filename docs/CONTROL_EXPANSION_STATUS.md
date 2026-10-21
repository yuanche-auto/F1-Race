# Control expansion – status (resume point)

Last updated: 2026-10-21. Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** A – dynamic MPCC (`cx/A-mpcc`, from `main` c010887).
* **Original reviewer:** session `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), the
  reviewer of L7–L9 – reached by SendMessage with this id (its pipe address changes after restarts).
* **L10:** PAUSED — Preserved for Future Development (branch `layer/L10`, protocol revision 1 binding,
  pause note 3ba9087). Nothing of L10 is touched by the expansion.

## Modules

| phase | module | developed | unit-tested | closed-loop checked | reviewed | accepted |
|---|---|---|---|---|---|---|
| A | `control/mpcc.py` (`DynamicMPCC`, `MPCCController`, `path_errors`) | yes (0061e79) | yes – `tests/test_mpcc.py`, 6 tests | development laps only (nominal car, 8 configurations; CX_A_protocol §12) | requested (packet 1) | no |
| A | curvature-aware MPCC | no – waits for the full text of Ma et al. 2026 (protocol §3) | – | – | – | – |
| B–G | – | not started | – | – | – | – |

## Review record

| date | commit | scope | verdict | findings / fixes |
|---|---|---|---|---|
| 2026-10-21 | (this commit) | phase A packet 1: audit and roadmap docs, MPCC code and tests, development checks, CX-A protocol revision 0 | requested | – |

## Tests

`tests/test_mpcc.py` (6) and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79 (git-ignored). No benchmark results.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall.
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. The reviewer's verdict on packet 1 (code, tests, protocol draft); fixes and re-review.
2. After APPROVE of the protocol: tuning on CXV (protocol §4), results to the reviewer, then CXT.
3. Then phase B (MPPI): literature entries, formulation, tests, development laps, protocol.
