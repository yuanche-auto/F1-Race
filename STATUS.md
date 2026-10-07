# STATUS — F1-Race (L0–L5)

Builder session: "F1-Race 施工者（L0–L5 builder）" [f92a84]. Reviewer: "独立审查员身份和职责".
Rule: work on branch `layer/Lk`, request review by commit hash, merge to `main` and push only
after APPROVE. Workflow: [docs/workflow.md](docs/workflow.md).

## Layers

| Layer | Branch | State | Review |
|---|---|---|---|
| L0 deterministic kinematic | `layer/L0` → local `main` (18a7a58) | **approved**; push waits for the author-e-mail decision (Q1) | round 2: APPROVE |
| L1 dynamic single-track + Pacejka | `layer/L1` | results done (ideal 73.91 s, closed loop 75.57 s); studies running, then review | – |
| L2 model mismatch, closed loop | (on `layer/L1` for now) | NMPC (time domain) + receding-horizon min-time MPC (spatial) written; nominal laps running | – |
| L3 stochastic disturbances | – | planned | – |
| L4 learning / adaptation | – | planned | – |
| L5 reinforcement learning | – | planned | – |

## L0 summary

Spielberg, generic open-wheel car. Ideal lap: centre line 77.89 s, minimum curvature 72.14 s,
minimum-time OCP 71.26 s (2 m grid; limit ≈ 71.0 s). Closed loop with 3 % friction margin:
72.90 s / 72.10 s, ellipse usage ≤ 0.995 for control steps 5–40 ms.
Reproduce: `python scripts/fetch_tracks.py Spielberg && python scripts/run_l0.py && pytest`.

Reference environment: Windows 11, Python 3.12.10, numpy 2.5.3, scipy 1.18.1, casadi 3.8.1,
matplotlib 3.11.2. Numbers reproduce on other versions (reviewer: Python 3.11); figures are not
expected to be byte-identical.

## Review log

| Date | Commit | Scope | Verdict | Notes |
|---|---|---|---|---|
| 2026-10-06 | 18a7a58 | L0 fixes | **APPROVE** | nits n3 (failure mode at coarser steps = off track) and n4 (tighter maximality tolerance on dense paths) folded into L1 |
| 2026-10-06 | a514825 | L0 | CHANGES_REQUESTED | M1 OCP closed loop not robust to control step; M2 no OCP grid convergence; minors: margin < design, QSS end-point accel, projection jitter, front-axle approximation undocumented; nits: env versions, .gitattributes; Q1 author e-mail → user |

## Open decisions for the user

* **Q1 (blocks every push):** commits are authored as `Yuan <yuanche@umich.edu>` (the global git
  config). Pushing makes the address public and links the repository to the university. Keep it, or
  rewrite the unpushed commits to the GitHub no-reply address before the first push?

## Notes for resuming

* venv: `.venv` (Python 3.12). Track data in `data/` (git-ignored), fetched by script.
* A worktree `../F1-Race-wt-L0` holds `layer/L0` (used for the review fixes; can be removed).
* L1 (branch `layer/L1`, contains the approved L0 via merge): `dynamics.py`, `planning/ocp_dynamic.py`
  (CPU cap 15 min per solve), `sim/dynamic.py`, `control/dyn_tracking.py` (look-ahead feedback,
  TC/ABS `traction_limit`, front-slip governor, optional ESC term), `experiments/l1.py`
  (continuation over eta: 0.97 from L0 -> ideal -> all margins -> ideal), `scripts/run_l1.py`
  (~14 min, caches plans in results/L1/l1_plans.pkl), `scripts/l1_studies.py` (~40 min),
  tests `test_dynamics.py`, `test_l1_pipeline.py`.
* L2 groundwork (not committed yet): `control/nmpc.py` (Frenet time-domain NMPC, track/mpcc
  modes, solver-failure fallback), `control/rh_mpc.py` (spatial receding-horizon min-time
  MPC; terminal speed bound must come from a plan with the same eta), `experiments/l2.py`
  (plant perturbation sampler, multiprocessing Monte Carlo).
* Next after L1: L2 (plant with parameter mismatch; offline plan + tracking vs NMPC vs MPCC vs
  constraint tightening; Monte Carlo over plant parameters).
