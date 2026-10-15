# Review request: preparation for L7 and the L7 protocol

Branches: `prep/robustness` (89f81fb) and `layer/L7` (its head; = 89f81fb + the protocol
and this request). Base: `main` cd3ace5. New machine (STATUS.md, "Machine and environments").

Two things are asked:

1. **Review of the preparation** (`prep/robustness`): STATUS update for the new machine,
   numerical robustness (ROADMAP section 7 point 2), the safety-filter solve-time benchmark.
   Verdict APPROVE / CHANGES_REQUESTED as usual; after APPROVE it is merged into `main`.
2. **Confirmation of the L7 protocol** (`docs/L7_protocol.md` on `layer/L7`) before any L7
   run (design review, as for L6). No L7 result-producing run has been made.

## What changed

* `src/f1race/control/nmpc.py`: options `retry` (second IPOPT instance, retries from the
  shifted plan and from a rollout) and `fallback="brake"`, `mode="filter"` (predictive
  safety filter, `u_des` per solve); defaults unchanged.
* `src/f1race/planning/replan.py`: `replan_robust` (direct as `l4.replan`, then
  continuation in η, in the parameters, slower warm starts).
* `src/f1race/control/safety_filter.py`: `CBFQPFilter` (point mass in curvilinear
  coordinates, second-order barriers for the track edges, 16-gon friction circle).
* `scripts/prep_robustness.py` (reproduction and paired perturbation trials, `report`),
  `scripts/l7_filter_bench.py` (collect / bench / report).
* Tests: `tests/test_replan.py` (4, stub solver), `tests/test_safety_filter.py` (3).
* Docs: `docs/prep_robustness.md`, `docs/L7_filter_benchmark.md`, `docs/L7_protocol.md`,
  STATUS.md.
* `planning/ocp_dynamic.py` and `experiments/l1.py` untouched: the L1 cache fingerprint is
  unchanged.

## Claims to check

Preparation (`docs/prep_robustness.md`):

1. The L6 numerical failures depend on rounding: of 3 re-plan failures, 2 recur here (the
   third converges directly); of the NMPC-burst laps, those of the nominal car (120 ms),
   cars 4 and 18 complete here, car 20 (30 / 60 ms) still leaves the track at ~1 400 m.
2. `FrenetMPC(retry=True)` removes the failed solves (1 063 → 5 fallbacks over 18 paired
   trials) but completes fewer laps (6 vs 9 of 18; retry + brake 6, brake 8); by the rule
   written before the isolated variants ran, L7 keeps the L6 settings. Check the rule's
   timing (it is in the doc with its time stamp; the first comparison it mentions came
   before it) and the table (`prep_robustness.py report`).
3. The remaining lap failures are a control limit at the Remus hairpin (~1 390–1 410 m), not
   numerical: with every solve converged the car still slides off (car 20, retry runs;
   paired car-21 trial 2 diverges at 1 391 m).
4. `replan_robust` gives the same plan as `l4.replan` whenever that succeeds, and solved all
   3 failing re-plan trials by the η continuation; the three cars that L6 lost to failed
   re-plans complete one lap each here (77.19 / 76.26 / 78.68 s).
5. Default behaviour unchanged: `tests/test_l2.py`, `test_l6.py` pass; laps without failed
   solves are identical with and without the options (cars 4, 18).

Benchmark (`docs/L7_filter_benchmark.md`):

6. The decision rule was written before the measurement (the doc's history; the protocol
   preamble). The proposals are passed through the plant's actuator limits – a correction
   made after the smoke runs, disclosed in the doc; timings were unaffected.
7. Under 24-process load: CBF-QP (qpOASES) 0.71 ms mean, 7 ms P99, 0 failures → training;
   NMPC filter 219–235 ms mean, 0.75–1.08 s P99 → evaluation only.
8. Uncalibrated, the CBF-QP changes ~40 % and the NMPC filter ~26 % of the decisions of a
   policy that completes these laps without them (attributions in the doc).

Protocol (`docs/L7_protocol.md`, design review): the sets (T1, T2, V, K, A) and their
disjointness; the order and acceptance test of the risk estimates (a)–(c); the CBF
calibration rule on K; groups P / F / M and the pre-registered S1–S3 (S3 deviates from the
ROADMAP's literal wording, with the reason given); what is always reported; the gap of the
missing L5 intermediate checkpoints in Part A.

## Reproduction

```
python -m pytest -q                                     # .venv
python scripts/prep_robustness.py report                # tables from results/prep (git-ignored)
python scripts/prep_robustness.py burst --case 21,0.06,lag_identified --variant retry --trial 2
python scripts/prep_robustness.py replan --case 20,0.06,lag_identified --variant robust   # ~30 min, one re-plan slot
.venv-rl/Scripts/python scripts/l7_filter_bench.py collect
python scripts/l7_filter_bench.py bench --filter nmpc --lap 1 --tag check
python scripts/l7_filter_bench.py report
```

`results/prep` and `results/L7_bench` are on this machine only (git-ignored); I can publish
them on a data branch like `data/results-v2` if wanted.

## Standing checks

* no course content: `tests/test_no_course_content.py` passes;
* no track data or results committed (`git diff --stat cd3ace5` lists only code, tests,
  docs);
* `pytest` passes: `.venv` 56 passed, 3 skipped (RL tests); `.venv-rl` test_l5, test_l6,
  test_safety_filter, test_replan: 19 passed, none skipped.
