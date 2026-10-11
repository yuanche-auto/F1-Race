# Review request: L6 results (branch `layer/L6`, ced319e)

Scope since the design review (75865d8 → 20b314b, approved for 1b): the results of
all four rounds, the write-up and the code added for rounds 2–4.

* Write-up: `docs/L6.md`; protocol with amendments A1–A10: `docs/L6_protocol.md`;
  decisions and the run log: `STATUS.md` (sections "L6–L10" and the overload).
* Code: `estimation/residual.py` (BLR on random Fourier features, coverage),
  `estimation/ekf.py` (EKF for states and parameters, and with the lag),
  `estimation/identify.py` (`smoothed`, `lag_filter`, `identify_with_lag`),
  `control/nmpc.py` (residual dynamics, per-solve tightening,
  `CautiousMPCController`, `LagAwareMPCController(observe="angle")`),
  `experiments/l6.py` (rounds 2–4 jobs, `replan_slot`, `safe_job`),
  `scripts/run_l6.py` (round2/3/4, report, single-threaded BLAS, fresh worker per
  job), `scripts/l6_figures.py`, `tests/test_l6.py` (11 tests).
* Results: every record is in `results/L6_final/*.jsonl` of the builder's worktree
  `E:\09Project\F1Race\F1-Race-wt-L6` (git-ignored; `SOURCES.txt` says where and
  at which commit each part ran). Tables:
  `python scripts/run_l6.py report --results results/L6_final`.

## Claims (docs/L6.md)

1. Without the lag in the model every controller has a lag it cannot take (tracker
   from 15 ms, NMPC from 30 ms on the nominal car); the L3 SMPC tolerates 60 ms
   on the nominal car (on the 25 cars 22/22/9 of 25 at 10/30/60 ms).
2. Identifying parameters in the wrong structure hurts: the lag goes into the yaw
   inertia (on its bound from 15–30 ms); the identified NMPC fails earlier than the
   nominal one. Measuring the road-wheel angle removes the bias, not the failure.
3. Identifying τ with the parameters gives a controller as good as the true model
   (25/25/24 of 25 at 10/30/60 ms; 25:0 and 24:0 against the L4 identification);
   the failures of the lag model with nominal parameters are exactly the 12 L4
   parameter-error cars.
4. Under sensor noise the EKF removes the bias of the L4 least squares (0.006 vs
   0.50; the L4 numbers are reproduced exactly), and with the lag the EKF gives 25/25
   against 7/25 for the least squares.
5. The residual RL policy copes with the lag (up to 120 ms on the nominal car;
   25/24/23 of 25), 4.3–4.4 s per lap slower than the correct-structure NMPC.
6. The residual model's uncertainty is not calibrated (held-out coverage 67–79 %)
   → no-go for using it directly in L7.

## Known issues – please weigh them

* **Overload (A10).** On 2026-10-10 between about 00:45 and 09:00 too many pools
  ran at once; allocations failed (`bad_alloc`, logged), the angle ablation and
  round 3 had to be redone. Root causes found afterwards: workers that had solved a
  minimum-time OCP kept ~3 GB committed, and numpy's and scipy's OpenBLAS committed
  ~0.5 GB each with 16 threads. Fixed by a fresh worker per job, single-threaded BLAS
  (checked bit-identical on four laps including an identification and a re-plan) and
  one re-plan at a time.
* **Round 2 ran entirely during the overload and is not replayed** (user's
  decision: complete the unfinished parts first, then decide). docs/L6.md separates
  what cannot have been affected (coverage computed offline from logs recorded
  before the overload → claim 6) from what can (the residual laps; the 4 vs 22
  comparison at 10 ms is not claimed). Also from the overload window, with solver
  fallbacks and not replayed: round 3 cars 6 and 7 (least squares), five round-1
  laps at 10 ms (lag_nominal cars 16, 18, 22, 23; nmpc car 13). Six other round-1
  laps from that window were replayed: all bit-identical.
* Numerical fragility: failed re-plans (round 1 car 20; round 3 cars 20, 21) and
  bursts of failed lag-aware NMPC solves around 1 400 m cost the model-based
  controllers their last cars.

## Questions

(a) Are the claims, as worded, supported without replaying round 2 – in particular
claim 6 and the no-go, which rest on the held-out coverage? (b) If the user's
preferred route is to update the strategy rather than re-run: is it acceptable to
carry the no-go into L7 (the safety layer then uses the identified structure or
L3-style calibrated back-offs, or a recalibrated residual uncertainty) and to list
the round-2 laps as unverified? (c) Anything else that blocks a merge?
