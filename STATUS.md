# STATUS — F1-Race (L0–L5)

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
