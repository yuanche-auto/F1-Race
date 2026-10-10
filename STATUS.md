# STATUS — F1-Race (L0–L5 done, L6 in progress)

Builder session: "F1-Race 施工者（L0–L5 builder）" [f92a84]. Reviewer: "独立审查员身份和职责".
Rule: work on branch `layer/Lk`, request review by commit hash, merge to `main` and push only
after APPROVE. Workflow: [docs/workflow.md](docs/workflow.md).

## Layers

| Layer | Branch | State | Review |
|---|---|---|---|
| L0 deterministic kinematic | `layer/L0` → `main` (18a7a58) | **approved, pushed** to origin/main 2026-10-07 | round 2: APPROVE |
| L1 dynamic single-track + Pacejka | `layer/L1` → `main` (571a982) | **approved, pushed** to origin/main 2026-10-07 | round 2: APPROVE (m4, m5 fixed on `layer/L2`) |
| L2 model mismatch, closed loop | `layer/L2` → `main` (270fb2b) | **approved, pushed** to origin/main 2026-10-08 | round 2: APPROVE (m1, m2 fixed on `layer/L3`) |
| L3 stochastic disturbances | `layer/L3` → `main` | **approved, pushed** to origin/main 2026-10-08 | round 2: APPROVE (nit: truncation definition, fixed on `layer/L5`) |
| L4 learning / adaptation | `layer/L4` → `main` | **approved, pushed** to origin/main 2026-10-08 | round 1: APPROVE (minors fixed in 66f6cb3) |
| L5 reinforcement learning | `layer/L5` → `main` | **approved, pushed** to origin/main 2026-10-09 | APPROVE after the multi-seed redo (wording minors fixed in 4fec887) |
| L6 model mismatch with a structural error | `layer/L6` (worktree `../F1-Race-wt-L6`) | **in progress**: roadmap and protocol written, round 1 running | protocol sent for review |

## L0 summary

Spielberg, generic open-wheel car. Ideal lap: centre line 77.89 s, minimum curvature 72.14 s,
minimum-time OCP 71.26 s (2 m grid; limit ≈ 71.0 s). Closed loop with 3 % friction margin:
72.90 s / 72.10 s, ellipse usage ≤ 0.995 for control steps 5–40 ms.
Reproduce: `python scripts/fetch_tracks.py Spielberg && python scripts/run_l0.py && pytest`.

Verified environments (`requirements-lock.txt` pins the reference one; casadi 3.8.1 in all):

| environment | L0 ideal lap | first L1 solve (eta = 0.97) | final L1 plans 1.0 / 0.97 / 0.95 / 0.93 | closed loop |
|---|---|---|---|---|
| reference `.venv`: Python 3.12.10, numpy 2.5.3, scipy 1.18.1 | 71.26203387743065 | kinematic guess, 74.889 s | 73.9072 / 74.8891 / 75.5999 / 76.3494 s | 75.574 s |
| `.venv-311`: Python 3.11.15, numpy 2.4.6, scipy 1.17.1 (the reviewer's versions) | 71.26203387743058 | kinematic guess hits 600 iterations, quasi-steady (97 %) converges, 75.245 s | 73.9072 / 74.8891 / 75.5998 / 76.3494 s | 75.574 s |

Figures are not expected to be byte-identical across versions.

## Review log

| Date | Commit | Scope | Verdict | Notes |
|---|---|---|---|---|
| 2026-10-09 | 75865d8 | L6 design (roadmap, protocol, lag plant, lag-aware NMPC) | design review, no verdict yet | code verified line by line; MAJOR-1 (structure knowledge = exact τ) → `lag_identified`, τ-mismatch runs; stats rule, coverage details, naming, second-env scope; all fixed as protocol amendments A1–A7 before 1b; attribution of the roadmap critique corrected |
| 2026-10-09 | 4354579 | L5 redo | **APPROVE** | protocol followed (written before the runs); selection, median seeds, paired tests and blind spots recomputed; evaluation reproduced in Python 3.11; wording minors m1–m4 fixed in 4fec887 |
| 2026-10-08 | 6552180 | L5 | (preliminary) | first results withdrawn: single training run not reproducible; reviewer asked for non-learning and slowed-NMPC baselines and a multi-seed protocol |
| 2026-10-08 | 9ce0b25 | L3 fixes | **APPROVE** | recomputed from runs.json / failures.json; four calibrations 28–30/30; honest event rates; nit: truncation by prediction time vs horizon overlap |
| 2026-10-08 | 6ebbab7 | L4 | **APPROVE** | minors: bounds hit in the noise study, saturation filter uses the true usage (no effect), docstring; fixed in 66f6cb3 |
| 2026-10-08 | 6ec6007 | L3 | CHANGES_REQUESTED | MAJOR-1 back-offs calibrated on mostly failed laps, sensitive to the environment -> truncated / iterated / other-environment calibrations; minors: per-stage semantics, failure analysis, ablation scope |
| 2026-10-08 | 270fb2b | L2 fixes | **APPROVE** | statistics recomputed from runs.json; soft NMPC cross-checked in Python 3.11 (13 cars, same outcomes); hard ablation with 500 iterations still fails on the 7 cars; minors m1 (iterations of the ablation) and m2 (cross-check numbers) fixed on `layer/L3` |
| 2026-10-08 | 4e695ba | L2 | CHANGES_REQUESTED | MAJOR-1 fallback to the shifted plan hides infeasible NLPs (hard slip/ellipse limits on the measured state) and was not reported; MAJOR-2 13 cars cannot support "doubles the success rate"; minor: spatial-MPC trials not reproducible from a file |
| 2026-10-07 | 571a982 | L1 fixes | **APPROVE** | reproduced in the reviewer's environment (Python 3.11.15); minors m4 (first-solve attempts stopped by the CPU cap, so load-dependent -> iteration caps only) and m5 (run times under load) fixed on `layer/L2` |
| 2026-10-07 | a6b1cb5 | L1 | CHANGES_REQUESTED | BLOCKER-1 first dynamic solve (eta 0.97 from the kinematic guess) hits the CPU cap in another environment -> KeyError; minors: cache not tied to code, test usage bound loose, "5 of 8 gain sets" missing in the results section, run-time figures inconsistent; nits: nsub convergence numbers |
| 2026-10-06 | 18a7a58 | L0 fixes | **APPROVE** | nits n3 (failure mode at coarser steps = off track) and n4 (tighter maximality tolerance on dense paths) folded into L1 |
| 2026-10-06 | a514825 | L0 | CHANGES_REQUESTED | M1 OCP closed loop not robust to control step; M2 no OCP grid convergence; minors: margin < design, QSS end-point accel, projection jitter, front-axle approximation undocumented; nits: env versions, .gitattributes; Q1 author e-mail → user |

## Decisions by the user

* **Q1 (2026-10-07):** keep the author e-mail `yuanche@umich.edu`; push each layer to `main` as
  soon as the reviewer approves it.

## L5 redo (ran while the user was away; finished 2026-10-09 05:25; approved)

* Protocol, fixed before the runs: [docs/L5_protocol.md](docs/L5_protocol.md) (groups A/B/C,
  seeds 1–3, validation set, selection rule, reporting).
* Pipeline: `scripts/l5_multiseed.py` (worktree `../F1-Race-wt-L5`, venv `.venv-rl`); log
  `results/L5/multiseed/driver.log`. It is resumable: re-running it skips finished training
  runs (all 12 checkpoints present) and finished evaluations. If a training run was cut
  off, delete its partial checkpoints before re-running.
* Decisions taken without the user (also in the protocol): plain policy not retrained
  (more seeds instead); a_c = 4 m/s² fixed by physics, not tuned; 3 training runs in
  parallel with 8 environments each (unchanged training settings).
* After it finishes: write docs/L5.md from results/L5/multiseed/tables.md and the
  baselines, request the review, merge to main and push after APPROVE (user's rule).

## L6–L10 (started 2026-10-09, user away; decisions recorded here)

* The user asked for an L6–L10 plan, had it criticised (a written critique the user forwarded; not from the repository's review session) and asked for a revised
  plan "with critical inheritance"; then: save it and start. Roadmap:
  [docs/ROADMAP_L6-L10.md](docs/ROADMAP_L6-L10.md) (what was taken from the review and what not;
  the cited literature was checked, all five works exist as cited).
* L6 protocol: [docs/L6_protocol.md](docs/L6_protocol.md), written before the result-producing runs.
  Decisions taken without the user:
  * structural error = first-order steering actuator lag; main observation condition: the
    controllers see their own commanded angle (the lag is hidden in the car's response); the
    measured-angle case is an ablation;
  * the τ grid was changed from the roadmap's 30 / 60 / 120 ms after a feasibility check on the
    nominal car (disclosed in the protocol): every lag-unaware controller fails there already at
    30 ms, while the lag-aware NMPC laps at 76.0–76.1 s at every τ; hence a sweep 5–120 ms on the
    nominal car and 10 / 30 / 60 ms on the 25 cars;
  * the protocol is sent to the reviewer and round 1 starts at the same time (as for the L5 redo);
    if the review changes the design, the affected runs are repeated.
* Result-producing runs use a detached worktree `../F1-Race-wt-L6-run` pinned at the commit
  named in `results/L6/RUN_COMMIT` there, so that development on `layer/L6` cannot change code
  under a running pool; its `results/L6` is copied back for the report.
* Runs in progress (started 2026-10-09; check the `chain_*.log` files):
  * `../F1-Race-wt-L6-run` (20b314b): `results/L6/chain_round1.sh` – 1a (first controllers ran
    in `../F1-Race-wt-L6`, copied over), 1a additions, 1b at 60 → 30 → 10 ms, angle ablation;
  * `../F1-Race-wt-L6-run311` (20b314b, `.venv-311`, 3 workers): `chain_env311.sh` – the
    second-environment check (1a, and nmpc / identified / oracle_struct at 60 ms);
  * the RL baseline at 10 / 30 / 60 ms is done (`../F1-Race-wt-L6-run/results/L6/rl.jsonl`).
  * 2026-10-10 00:45, user: the machine is free tonight, use more of the CPU. Rounds 3 and 2
    (60 and 30 ms) were started right away in `../F1-Race-wt-L6-run23` (be9cdb6; same run code as
    a5dacce), 4 workers each, next to the 8 of round 1; `chain_round23.sh` was stopped. After
    round 1, `chain_after_round1.sh` there runs the RL sweep on the nominal car and round 2 at
    10 ms (8 workers, `results/L6_tau10`). Solve times recorded from now on are under full
    load (16 processes on 8 cores) and are not comparable to the earlier ones.
  The reviewer cleared 1b after amendments A1–A8 and expects a results review request.
* **Overload, 2026-10-10 03:00–09:00**: after round 1, round 2 at 10 ms (8 workers) started next
  to rounds 2 and 3 (4 + 4) and the RL sweep – about 18 processes, out of memory (`std::bad_alloc`);
  the angle ablation failed, round 3 aborted after 12 of 25 cars, the machine froze and the user
  quit the session. My mistake: too many parallel pools for ~12 GB of free memory. Kept: round 1
  (finished before), the RL sweep (no solver). Re-run in `../F1-Race-wt-L6-rerun` (fa7ba35: failing
  jobs no longer abort a pool), at most 8 processes in two lanes (`results/lanes.sh`, memory log
  `results/memlog.txt`): the whole angle ablation; round 3 for the 13 missing cars and cars 6
  and 7 (the two with solver fallbacks); round 2 at 60 and 30 ms entirely and the 8 jobs at 10 ms
  with solver fallbacks; the 8 round-1 laps at 10 ms with solver fallbacks. Records without any
  solver fallback or retried re-plan are deterministic and kept; every re-run is compared with
  the original (a determinism check).
  **Changed the same morning on the user's instruction: finish the unfinished parts first, no
  replays until the user has looked at the results.** A re-plan needs ~5.5 GB, so at most two run
  at once machine-wide (d783f90, `replan_slot`). Running now: `../F1-Race-wt-L6-rerun/results/lanes2.sh`
  (angle ablation + the 13 missing round-3 cars, 4 + 4 workers), then
  `../F1-Race-wt-L6-r4/results/L6/chain_round4.sh` (round 4, amendment A9; 8 workers). Replays done
  before the change: 6 round-1 laps at 10 ms, all identical. Pending decision of the user: replay
  round 2 (ran during the overload) and the other affected records.
* 2026-10-10 14:40 the machine was shut down (angle ablation finished at 12:20, round 3 at 24 of
  25 cars, round 4 not started). The low-memory guard (`../F1-Race-wt-L6-rerun/results/memguard.sh`)
  had reduced re-plans to one at a time at 12:02. User 15:05: continue. Restarted 15:15; within
  minutes 15 round-4 jobs and the round-3 job failed with `bad_alloc` / numpy `MemoryError` while
  8–9 GB were physically free. **Root cause:** a worker that has solved a minimum-time OCP keeps
  ~3 GB of committed (private) memory, and pool workers are reused, so the commit limit (RAM +
  page file, 50 GB, ~21 GB of it free for these runs) was exhausted – also the likely cause of
  the morning's overload. Fix 09fd8f4: a fresh worker process per job (`maxtasksperchild=1`),
  one re-plan at a time by default. Running since 15:24 (`../F1-Race-wt-L6-rerun/results/lanes3b.sh`,
  failed jobs retried once): round 3 car 24, round 4 (7 workers); the memory log now also
  records the free commit (`free_virtual_MB`).
* That was not enough: fresh workers still committed 1–3 GB each – numpy's and scipy's own
  OpenBLAS reserve ~0.5 GB each with 16 threads. Single-threaded BLAS (afe6160) brings a worker to
  ~140 MB; checked bit-identical on four nominal-car laps at 10 ms (NMPC, exploration +
  identification, identified with re-plan, lag model), and the laps run about a third faster. An
  OCP re-plan still needs ~6.8 GB (5.8 GB resident), so only one at a time (limit file
  `<temp>/f1race_replan_slots/limit` = 1, 4db4e39). Running since 15:53:
  `../F1-Race-wt-L6-rerun/results/lanes3c.sh` (round 3 car 24, round 4; two passes each).
* Pipeline: `python scripts/run_l6.py round1 ...` (resumable: finished laps in
  `results/L6/round1.jsonl` are skipped; errors in `results/L6/errors.log`), RL baseline
  `scripts/l6_rl.py` in `.venv-rl`, tables `python scripts/run_l6.py report`.

## Notes for resuming

* venv: `.venv` (Python 3.12). Track data in `data/` (git-ignored), fetched by script.
* A worktree `../F1-Race-wt-L0` holds `layer/L0` (used for the review fixes; can be removed).
* L1 (branch `layer/L1`, contains the approved L0 via merge): `dynamics.py`, `planning/ocp_dynamic.py`
  (iteration caps, no CPU-time cap), `sim/dynamic.py`, `control/dyn_tracking.py` (look-ahead feedback,
  TC/ABS `traction_limit`, front-slip governor, optional ESC term), `experiments/l1.py`
  (continuation over eta: 0.97 from L0 -> ideal -> all margins -> ideal), `scripts/run_l1.py`
  (~20 min, caches plans in results/L1/l1_plans.pkl), `scripts/l1_studies.py` (~40 min),
  tests `test_dynamics.py`, `test_l1_pipeline.py`.
* L2 (branch `layer/L2`, worktree `../F1-Race-wt-L2`, 0ae7547 + merge of the L1 fixes): `control/nmpc.py`,
  `control/rh_mpc.py` (experimental), `experiments/l2.py`, `scripts/run_l2.py`, `scripts/l2_studies.py`,
  docs/L2.md. The Monte Carlo is re-run on the final L1 plans before the L2 review.
* L3 (branch `layer/L3`, worktree `../F1-Race-wt-L3`, from `layer/L2`): `sim/stochastic.py` (friction field,
  gusts, noise), per-stage tightening in `control/nmpc.py`, `experiments/l3.py`, `scripts/run_l3.py`.
* Next: L4 (online estimation / adaptive MPC, GP residuals, learning MPC), then L5 (RL, RL + MPC).
