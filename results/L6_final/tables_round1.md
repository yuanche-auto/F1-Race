### 1a – nominal car (lap time, or where it stopped)

| controller | 5 ms | 10 ms | 15 ms | 20 ms | 30 ms | 45 ms | 60 ms | 90 ms | 120 ms |
|---|---|---|---|---|---|---|---|---|---|
| L1 tracker, 5 % plan | 75.58 | 75.58 | off track @ 2730 m | off track @ 2725 m | off track @ 2725 m | spin @ 2721 m | spin @ 2712 m | spin @ 473 m | spin @ 460 m |
| NMPC, nominal model, 5 % | 76.05 | 76.11 | 76.27 | 76.93 | off track @ 2739 m | off track @ 463 m | off track @ 461 m | off track @ 458 m | off track @ 458 m |
| NMPC, lag + parameters identified | 76.08 | 76.00 | 76.18 | 76.03 | 76.03 | 76.05 | 76.02 | 76.00 | off track @ 1406 m |
| NMPC, nominal + lag model 0.5 τ | 76.03 | 76.05 | 76.08 | 76.35 | 76.40 | 76.49 | off track @ 2744 m | off track @ 2725 m | off track @ 2725 m |
| NMPC, nominal + lag model 2 τ | 76.08 | 76.02 | 76.06 | 76.10 | 76.42 | 77.05 | 77.04 | off track @ 3232 m | off track @ 2850 m |
| NMPC, nominal model, uniform 12 % | 78.89 | 78.78 | 78.84 | 78.98 | off track @ 3824 m | off track @ 466 m | off track @ 463 m | off track @ 1392 m | off track @ 459 m |
| SMPC ε = 0.01 (L3) | 78.88 | 78.92 | 78.84 | 78.92 | 78.86 | 78.82 | 78.96 | spin @ 186 m | spin @ 191 m |
| NMPC, identified model (L4) | 76.06 | 76.21 | off track @ 3823 m | off track @ 2753 m | off track @ 463 m | off track @ 462 m | off track @ 463 m | off track @ 459 m | off track @ 465 m |
| NMPC, true parameters, no lag | – | 76.09 | – | – | 77.37 | – | off track @ 466 m | – | – |
| NMPC, nominal parameters + lag | 76.01 | 76.03 | 76.03 | 76.01 | 76.08 | 76.06 | 76.02 | 76.07 | 76.05 |
| NMPC, true parameters + lag (correct structure) | – | 76.03 | – | – | 76.09 | – | 76.04 | – | – |
| residual RL (L5, B_s1_500k) | 80.83 | 80.88 | 80.93 | 80.98 | 81.03 | 81.02 | 81.01 | 81.05 | 81.27 |
| tracker in the RL environment (check) | 75.58 | 75.58 | off track @ 2730 m | off track @ 2725 m | off track @ 2725 m | spin @ 2721 m | spin @ 2712 m | spin @ 473 m | spin @ 460 m |

### 1b – 25 cars, observation: angle (laps completed, mean lap)

| controller | τ = 60 ms |
|---|---|
| L1 tracker, 5 % plan | 0/25 (0–14 %) |
| NMPC, nominal model, 5 % | 0/25 (0–14 %) |
| NMPC, nominal model, uniform 12 % | 0/25 (0–14 %) |
| NMPC, identified model (L4) | 0/25 (0–14 %) |
| NMPC, nominal parameters + lag | 13/25 (31–72 %), 76.43 s |

### 1b – 25 cars, observation: command (laps completed, mean lap)

| controller | τ = 10 ms | τ = 30 ms | τ = 60 ms |
|---|---|---|---|
| L1 tracker, 5 % plan | 2/25 (1–26 %), 75.58 s | 1/25 (0–20 %), 75.63 s | 2/25 (1–26 %), 75.59 s |
| NMPC, nominal model, 5 % | 12/25 (28–69 %), 76.37 s | 3/25 (3–31 %), 76.49 s | 0/25 (0–14 %) |
| NMPC, lag + parameters identified | 25/25 (86–100 %), 76.78 s | 25/25 (86–100 %), 76.90 s | 24/25 (80–100 %), 77.39 s |
| NMPC, nominal model, uniform 12 % | 16/25 (43–82 %), 79.30 s | 8/25 (15–54 %), 79.46 s | 1/25 (0–20 %), 79.77 s |
| SMPC ε = 0.01 (L3) | 22/25 (69–97 %), 79.11 s | 22/25 (69–97 %), 79.14 s | 9/25 (18–57 %), 79.21 s |
| NMPC, identified model (L4) | 22/25 (69–97 %), 76.73 s | 0/25 (0–14 %) | 0/25 (0–14 %) |
| NMPC, true parameters, no lag | 25/25 (86–100 %), 76.93 s | 4/25 (5–36 %), 77.68 s | 0/25 (0–14 %) |
| NMPC, nominal parameters + lag | 13/25 (31–72 %), 76.34 s | 13/25 (31–72 %), 76.37 s | 13/25 (31–72 %), 76.37 s |
| NMPC, true parameters + lag (correct structure) | 25/25 (86–100 %), 76.84 s | 24/25 (80–100 %), 76.76 s | 23/25 (74–99 %), 76.73 s |
| residual RL (L5, B_s1_500k) | 25/25 (86–100 %), 81.10 s | 24/25 (80–100 %), 81.07 s | 23/25 (74–99 %), 81.06 s |
| tracker in the RL environment (check) | 2/25 (1–26 %), 75.58 s | 1/25 (0–20 %), 75.63 s | 2/25 (1–26 %), 75.59 s |

| τ | comparison (same cars) | only the first | only the second | exact McNemar p |
|---|---|---|---|---|
| 10 ms | oracle_struct vs oracle_param | 0 | 0 | 1 |
| 10 ms | lag_nominal vs nmpc | 1 | 0 | 1 |
| 10 ms | lag_identified vs identified | 3 | 0 | 0.25 |
| 10 ms | oracle_struct vs lag_identified | 0 | 0 | 1 |
| 10 ms | identified vs nmpc | 10 | 0 | 0.00195 |
| 10 ms | identified vs oracle_param | 0 | 3 | 0.25 |
| 10 ms | oracle_param vs nmpc | 13 | 0 | 0.000244 |
| 10 ms | smpc vs nmpc | 10 | 0 | 0.00195 |
| 10 ms | rl vs nmpc | 13 | 0 | 0.000244 |
| 30 ms | oracle_struct vs oracle_param | 20 | 0 | 1.91e-06 |
| 30 ms | lag_nominal vs nmpc | 10 | 0 | 0.00195 |
| 30 ms | lag_identified vs identified | 25 | 0 | 5.96e-08 |
| 30 ms | oracle_struct vs lag_identified | 0 | 1 | 1 |
| 30 ms | identified vs nmpc | 0 | 3 | 0.25 |
| 30 ms | identified vs oracle_param | 0 | 4 | 0.125 |
| 30 ms | oracle_param vs nmpc | 4 | 3 | 1 |
| 30 ms | smpc vs nmpc | 19 | 0 | 3.81e-06 |
| 30 ms | rl vs nmpc | 21 | 0 | 9.54e-07 |
| 60 ms | oracle_struct vs oracle_param | 23 | 0 | 2.38e-07 |
| 60 ms | lag_nominal vs nmpc | 13 | 0 | 0.000244 |
| 60 ms | lag_identified vs identified | 24 | 0 | 1.19e-07 |
| 60 ms | oracle_struct vs lag_identified | 0 | 1 | 1 |
| 60 ms | identified vs nmpc | 0 | 0 | 1 |
| 60 ms | identified vs oracle_param | 0 | 0 | 1 |
| 60 ms | oracle_param vs nmpc | 0 | 0 | 1 |
| 60 ms | smpc vs nmpc | 9 | 0 | 0.00391 |
| 60 ms | rl vs nmpc | 23 | 0 | 2.38e-07 |

