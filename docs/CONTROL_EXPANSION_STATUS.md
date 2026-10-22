# Control expansion – status (resume point)

Last updated: 2026-10-22 (phase B done: final review APPROVE; merged to main; phase C next). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** B done – final review of b276f56 **APPROVE**, `cx/B-mppi` merged to `main` (--no-ff) 2026-10-22
  (merge hash recorded in phase C's first commit); next C – tube MPC (literature and protocol first).
  Phase B result: MPPI on the NMPC's problem at the real-time CPU budget (numpy, K 1 024) completes 0 of 50 test cars
  against the NMPC's 12 – **H-B1 holds for `nmpc`** (0 : 12, p 0.0005; confirmatory, after the development gate failed
  and was waived by amendment B3); H-B2 not testable; 4 K samples offline 2 of 50; not cross-checked.
  History of phase B:
  B – MPPI, branch `cx/B-mppi` from `main` b68d0a4: literature (§B of the literature review) and
    protocol revision 0 (`docs/CX_B_protocol.md`) **APPROVE** (0b94d1a) with clarifications m1, m2, n1–n4 written in;
    `control/mppi.py`, `experiments/cx_b.py`, the runner / report and their tests written (57146e4, d80e1a6); development
    checks 9.1 and 9.2 done – 9.2 found that the NMPC drives and brakes at once in its braking zones, so **amendment B1**
    (MPPI samples the NMPC's three inputs) was proposed (8d86ff2) and approved (c8806c1). K timing: K = 2 048. Development
    gate (9.6) **failed** for `mppi` (white noise: 0 of 17 nominal laps); `mppi_lp` 2 of 8 – **amendment B2** (the
    low-pass MPPI as the primary) proposed in eea5a49; packet 2 (c3fca85) and B2 **APPROVE**; review c1: the K rule
    on `mppi_lp` gives K_RT = 1 024 (K 2 048: median 44.6 ms); c2 traces recorded; c3 scope. Packet 3 (770cd7b) **APPROVE**;
    g1: the gate at K_RT fails for `mppi_lp` too (0 of 9 nominal laps) → **amendment B3** (e9fb7e1) **APPROVE**, option 1:
    the pre-registered tuning and benchmark run with the gate waived (d1, d4 in d93b5e9). CXV tuning step 1 started
    2026-10-22 01:56 in `F1-Race-worktrees/F1-Race-wt-CXB-run` at d93b5e9 (inputs identical to the main checkout and to
    phase A's run). CXV tuning done (01:56–01:59; 120 laps, no error record): every setting 0 of 10 in both steps – the
    rule falls to the grid centre, `mppi_lp` λ 1, c_σ 1, f_c 2 Hz, K 1 024 (an uninformative tuning, review d3);
    packet 4 (the selections) **APPROVE**. CBT 02:00–02:22 (408 laps, `test.jsonl` f869d47d3632…): **H-B1 holds for `nmpc`**
    (0 : 12, p 0.0005), H-B2 not testable; packet 5 **APPROVE**. Secondary conditions 02:25–02:56 (504 laps,
    `secondary.jsonl` 2219e5634a68…); phase-A reproducibility 68 of 68 (after the NaN-comparison fix baa69cb). Final review
    of `cx/B-mppi` requested. A done (final review of e2cea5d
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
| B | `control/mppi.py` (`MPPI`), `experiments/cx_b.py`, `scripts/run_cx_b.py`, `scripts/report_cx_b.py`, `scripts/cx_b_dev.py` | yes (57146e4 … baa69cb) | yes – `tests/test_mppi.py` 12, `tests/test_cx_b.py` 6 | development checks 9.1–9.8 on the nominal car; CXV tuning; CBT benchmark; secondary conditions | protocol rev 0, B1, B2, B3 and packets 2–6 **APPROVE**; final review b276f56 **APPROVE** | **yes – phase B completed** |
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
| 2026-10-22 | 8d86ff2 | amendment B1 (MPPI samples the NMPC's three inputs) | **APPROVE** (decision) | c1 status before the K timing (c8806c1), c2 clamp tests, c3 overlap for both controllers with the left-foot-braking remark; disclose the informal timings in packet 2 |
| 2026-10-22 | c3fca85 | **phase B packet 2**: MPPI code, runner / report, tests, development checks 9.1–9.6 and the post-gate diagnostics; amendment B2 | **APPROVE**; B2 **APPROVE** | c1 re-time K on mppi_lp (→ K_RT 1 024); c2 traces as records; c3 scope (12 vs 6 settings, primary changed after the gate); n1 median budget, n2 nominal-U limitation, n3 common random numbers |
| 2026-10-22 | 770cd7b | phase B packet 3: the B2 roles, K_RT 1 024 (c1), traces (c2), c3 / n1 / n3 | **APPROVE**; g1 blocking | g1 the 9.6 gate at K_RT (0 of 9, 99955f4); n1 budget numbers at 1 024, n2 track fixture (99955f4) |
| 2026-10-22 | e9fb7e1 | amendment B3 (the gate failed at K_RT) | **APPROVE**, option 1 | d1 H-B1 confirmatory, budget-limited; d2 report wording; d3 tuning by the rule even if uninformative; d4 frontier candidates (d93b5e9) |
| 2026-10-22 | worktree d93b5e9 | phase B packet 4: CXV tuning (120 laps; 0 of 10 everywhere, the grid centre) | **APPROVE** | report: the choice is the closeness rule, the better direction (c_σ < 0.5) lies outside the grid; CXV step times carry the load |
| 2026-10-22 | worktree d93b5e9; 82c31f9 … 78877f6 | phase B packet 5: CBT (408 laps) and its report / benchmark text | **APPROVE** | n1 mppi_big reaches the 2.7 / 3.83 km corners on 10 cars (fixed in 5fccae5) |
| 2026-10-22 | worktree d93b5e9; d813143, fc109a0, baa69cb | phase B packet 6: secondary conditions (504 laps), reproducibility 68 of 68 (after the NaN-comparison fix) | **APPROVE** | – |
| 2026-10-22 | b276f56 | **phase B final review** (whole branch vs main b68d0a4: 30 commits, 13 files, +2825 / −14) | **APPROVE** | the three standing checks pass (.venv 88 + 4 skips, .venv-rl 92); nit: status lines marked done before the merge |

## Tests

`tests/test_mpcc.py` (8) and `tests/test_cx_a.py` (7) at e2cea5d (6 and 5 at 6bcc6ef), and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79 and the secondary-path smoke (549e4bd); `results/CX/A/smoke/` –
the runner's smoke on the nominal car. CXV tuning (8babe7b): worktree `F1-Race-worktrees/F1-Race-wt-CXA-tune`,
`results/CX/A/{tune.jsonl, selected.json, report/tune.md, RUN_COMMIT, SHA256_inputs*.txt, timeline.tsv}`.
The first CXV selection is kept as `selected_first.json` and `report/tune_first.md` when the A2 re-tuning
writes `selected.json` / `report/tune_a2.md` (review c1). CXT: `test.jsonl`, `report/CX_A.md` (2026-10-21 21:33–21:50); secondary conditions: `secondary.jsonl`,
`report/CX_A_secondary.md` (21:51–22:16). All git-ignored.

Phase B: `results/CX/B/dev/` (main checkout) – the development checks of CX-B §9 (`checks.json`, `nmpc_nom.npz`,
`timing.json`, `timing_lp.json`, `gap.json`, `laps.jsonl`, `nom_diagnostic*.jsonl`, `traces.*`, `gate_k1024.jsonl`) and
the runner smoke (`smoke/`). Benchmark: worktree `F1-Race-worktrees/F1-Race-wt-CXB-run` at d93b5e9, `results/CX/B/`
(`tune.jsonl`, `tune_lp.jsonl`, `selected*.json`, `test.jsonl`, `secondary.jsonl`, `report/`, `RUN_COMMIT`,
`SHA256_inputs*.txt`, `timeline.tsv`). All git-ignored.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall (phase A used CiMPCC instead).
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. Phase C (tube MPC): branch `cx/C-tube` from the new `main`; record the phase-B merge hash in its first commit;
   literature entries (CONTROL_LITERATURE_REVIEW.md §C) and the protocol `docs/CX_C_protocol.md` for review; no code
   before the protocol is approved.
2. Phase B follow-up candidates (CONTROL_METHOD_MATRIX.md) stay unscheduled.
