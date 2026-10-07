# STATUS — F1-Race (L0–L5)

Builder session: "F1-Race 施工者（L0–L5 builder）" [f92a84]. Reviewer: "独立审查员身份和职责".
Rule: work on branch `layer/Lk`, request review by commit hash, merge to `main` and push only
after APPROVE. Workflow: [docs/workflow.md](docs/workflow.md).

## Layers

| Layer | Branch | State | Review |
|---|---|---|---|
| L0 deterministic kinematic | `layer/L0` | review round 1 fixes committed | round 1: CHANGES_REQUESTED (2 major) → fixed, re-review requested |
| L1 dynamic single-track + Pacejka | `layer/L1` | in progress | – |
| L2 model mismatch, closed loop | – | planned | – |
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
| 2026-10-06 | a514825 | L0 | CHANGES_REQUESTED | M1 OCP closed loop not robust to control step; M2 no OCP grid convergence; minors: margin < design, QSS end-point accel, projection jitter, front-axle approximation undocumented; nits: env versions, .gitattributes; Q1 author e-mail → user |

## Notes for resuming

* venv: `.venv` (Python 3.12). Track data in `data/` (git-ignored), fetched by script.
* Next: L1 — dynamic single-track model (states vx, vy, r; Pacejka lateral forces, load
  transfer, driven rear axle), minimum-time OCP on it, comparison with L0.
