# STATUS — F1-Race (L0–L5)

Builder session: "F1-Race 施工者（L0–L5 builder）" [f92a84]. Reviewer: "独立审查员身份和职责".
Rule: work on branch `layer/Lk`, request review by commit hash, merge to `main` and push only
after APPROVE. Workflow: [docs/workflow.md](docs/workflow.md).

## Layers

| Layer | Branch | State | Review |
|---|---|---|---|
| L0 deterministic kinematic | `layer/L0` | complete, review requested | pending |
| L1 dynamic single-track + Pacejka | `layer/L1` | in progress | – |
| L2 model mismatch, closed loop | – | planned | – |
| L3 stochastic disturbances | – | planned | – |
| L4 learning / adaptation | – | planned | – |
| L5 reinforcement learning | – | planned | – |

## L0 summary

Spielberg, generic open-wheel car. Ideal lap: centre line 78.35 s, minimum curvature 72.33 s,
minimum-time OCP 71.43 s. Closed loop with 3 % friction margin: 73.17 s / 72.28 s.
Reproduce: `python scripts/fetch_tracks.py Spielberg && python scripts/run_l0.py && pytest`.

## Review log

| Date | Commit | Scope | Verdict | Notes |
|---|---|---|---|---|

## Notes for resuming

* venv: `.venv` (Python 3.12). Track data in `data/` (git-ignored), fetched by script.
* Next: L1 — dynamic single-track model (states vx, vy, r; Pacejka lateral forces, load
  transfer, driven rear axle), minimum-time OCP on it, comparison with L0.
