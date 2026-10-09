# L5 redo — protocol (written before the runs)

Written on 2026-10-08, before any of the runs below were started, after the
first L5 results were withdrawn (a single training run per policy; retraining
with the same settings gave policies that fail systematically at one corner).
Everything here is fixed in advance; deviations will be reported in docs/L5.md.

## What was seen before this protocol

* First training run (observation = true state): history policy 51/51 L2 cars,
  30/30 L3 laps; plain policy 50/51, 30/30.
* Second run (observation = measured state, otherwise identical): history 13/51
  and 16/30, plain 0/51 and 0/30. Almost all failures at one place per policy:
  2384–2386 m (history) and 2545 m (plain), a fast right-hand curve (radius
  ≈ 210 m, ≈ 218 km/h, accelerating) and an almost straight section (radius
  ≈ 1100 m, ≈ 270 km/h). Training curves indistinguishable from the first run.
* Diagnosis (hypothesis): (1) training episodes of 20 s from random starts on
  the plan rarely bring the car to a given point in the state it reaches on a
  full lap (distribution shift), so a weakness can survive training unnoticed;
  (2) the steering offset (up to ±0.02 rad) changes the path curvature by
  δ/L, i.e. the lateral acceleration by v²δ/L – at 270 km/h the full offset is
  about 30 m/s², so a mis-learned offset on a fast section throws the car off.
* Baselines (done, not affected): tracker with constant speed factor 0.85:
  16/51 and 14/30; 0.90: 8/51 and 0/30; NMPC with a uniform reserve
  (η = 0.82, track margin 1.3 m, nominal lap 83.5 s): 29/51 and 17/30.

The two design changes below (episode length, speed-dependent steering bound)
are therefore made *after* seeing test-set failures; they are compared with the
unchanged design in the same protocol, and no setting of them is chosen on the
test sets.

## Groups (history policy, 8 observations + last action; SAC as before)

| group | episode length | steering offset bound |
|---|---|---|
| A (original) | 20 s | ±0.02 rad |
| B | 40 s | ±0.02 rad |
| C | 40 s | ±min(0.02 rad, a_c L / v²), a_c = 4 m/s² |

* a_c = 4 m/s² is fixed by physical reasoning, not tuned: the offset may change
  the lateral acceleration by at most about 0.4 g; below ≈ 100 km/h the bound is
  the old one (0.02 rad), at 270 km/h it is 0.15°.
* The plain policy (no history) is not retrained: both earlier runs show it no
  better than the history policy, and the compute goes to more seeds instead.
* Training seeds 1, 2, 3 per group (9 runs, 600 000 decisions each, 8 parallel
  environments, the same randomisation of cars and disturbances as before, all
  other hyper-parameters unchanged). A checkpoint is kept every 50 000
  decisions (12 per run).

## Validation set (disjoint from the test sets and from training)

* 10 cars: `sample_plants(nominal, 10, seed=1)` without the nominal car,
  no disturbances;
* 10 disturbance realisations on the nominal car: seeds 2000–2009.
* Full flying laps from the start of the plan, as in the test evaluation.

## Selection (per group, among its 3 × 12 checkpoints)

1. most validation laps completed (of 20);
2. ties: lowest mean lap time of the completed validation laps;
3. remaining ties: the later checkpoint.

## Test evaluation and reporting

Test sets as before: the 51 L2 cars (seed 0) and the 30 L3 seeds (0–29).

* (a) the selected checkpoint of each group: validation and test results (the
  difference shows the selection bias);
* (b) the final checkpoint (600 000) of every seed on the test sets – the
  unselected distribution;
* (c) the median seed of each group by (b).

Conclusions rest on (b) and (c); (a) is secondary. Paired comparisons with the
baselines use exact McNemar tests as in L2–L4.

## Decisions taken while the user is away

The user approved this plan and will review the results later; decisions not
covered here are logged in docs/L5.md with their reasons.
