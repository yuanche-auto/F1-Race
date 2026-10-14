### Round 2 – residual learning (25 cars, laps completed, mean lap)

| controller | τ = 10 ms | τ = 30 ms | τ = 60 ms |
|---|---|---|---|
| identified | 22/25, 76.73 s | 0/25 | 0/25 |
| residual | 4/25, 76.68 s | 0/25 | 0/25 |
| residual_cautious | 2/25, 77.71 s | 0/25 | 0/25 |
| lag_identified | 25/25, 76.78 s | 25/25, 76.90 s | 24/25, 77.39 s |
| oracle_struct | 25/25, 76.84 s | 24/25, 76.76 s | 23/25, 76.73 s |

Coverage of the 95 % intervals in % (v_x' / v_y' / r'), pooled over the cars:

| τ | held-out (exploration lap) | closed loop, `residual` | near-limit, `residual` | closed loop, `residual_cautious` | k-step ±b_k (k = 10 / 20), `residual_cautious` | go? |
|---|---|---|---|---|---|---|
| 10 ms | 72.0 / 67.0 / 72.5 (n = 6637) | 76.6 / 76.5 / 74.2 (n = 117477) | 59.2 / 60.8 / 57.9 (n = 60595) | 77.7 / 78.9 / 76.2 | 53.6 / 54.9 | no-go |
| 30 ms | 71.4 / 69.6 / 70.6 (n = 5589) | 86.8 / 85.6 / 85.0 (n = 13378) | 72.4 / 71.2 / 70.5 (n = 3356) | 83.3 / 83.3 / 83.6 | 78.5 / 86.1 | no-go |
| 60 ms | 72.4 / 74.9 / 78.8 (n = 2774) | 69.7 / 69.6 / 74.7 (n = 3744) | 66.3 / 66.3 / 65.0 (n = 326) | 66.1 / 67.9 / 71.2 | 54.8 / 77.2 | no-go |

Go / no-go for L7 (protocol section 5, A4): **no-go**.

| τ | comparison (same cars) | only the first | only the second | exact McNemar p |
|---|---|---|---|---|
| 10 ms | residual vs identified | 0 | 18 | 7.63e-06 |
| 10 ms | residual_cautious vs residual | 0 | 2 | 0.5 |
| 10 ms | oracle_struct vs residual | 21 | 0 | 9.54e-07 |
| 10 ms | lag_identified vs residual | 21 | 0 | 9.54e-07 |
| 30 ms | residual vs identified | 0 | 0 | 1 |
| 30 ms | residual_cautious vs residual | 0 | 0 | 1 |
| 30 ms | oracle_struct vs residual | 24 | 0 | 1.19e-07 |
| 30 ms | lag_identified vs residual | 25 | 0 | 5.96e-08 |
| 60 ms | residual vs identified | 0 | 0 | 1 |
| 60 ms | residual_cautious vs residual | 0 | 0 | 1 |
| 60 ms | oracle_struct vs residual | 23 | 0 | 2.38e-07 |
| 60 ms | lag_identified vs residual | 24 | 0 | 1.19e-07 |

### Round 3 – noise only (τ = 0, 25 cars)

| identification | laps completed | mean lap | largest parameter error (median over cars) | cars with a factor on its bound | mean error per factor (mu, m, ClA, CdA, Iz, B, aero_front) |
|---|---|---|---|---|---|
| noisy_ls | 22/25 | 76.59 s | 0.501 | 21 | +0.066, -0.031, -0.149, +0.005, +0.495, -0.052, -0.000 |
| noisy_ls_smooth | 24/25 | 77.00 s | 0.394 | 7 | +0.023, -0.035, -0.093, -0.001, +0.379, -0.014, -0.003 |
| noisy_ekf | 22/25 | 76.94 s | 0.006 | 0 | +0.000, -0.000, +0.004, -0.000, -0.002, -0.001, +0.001 |

| comparison (same cars) | only the first | only the second | exact McNemar p |
|---|---|---|---|
| noisy_ls_smooth vs noisy_ls | 2 | 0 | 0.5 |
| noisy_ekf vs noisy_ls | 3 | 3 | 1 |

### Round 4 – lag (60 ms), parameter errors and sensor noise (25 cars)

| identification | laps completed | mean lap | identified τ (median, range) | largest parameter error (median) | cars with a factor on its bound |
|---|---|---|---|---|---|
| noisy_lag_ls | 7/25 | 77.51 s | 41.8 ms (36.9–50.8) | 0.501 | 25 |
| noisy_lag_ekf | 25/25 | 77.50 s | 59.6 ms (55.1–62.0) | 0.034 | 0 |

| comparison (same cars) | only the first | only the second | exact McNemar p |
|---|---|---|---|
| noisy_lag_ekf vs noisy_lag_ls | 18 | 0 | 7.63e-06 |
| noisy_lag_ekf vs lag_identified | 1 | 0 | 1 |
| noisy_lag_ekf vs oracle_struct | 2 | 0 | 0.5 |

Lap time noisy_lag_ekf minus lag_identified on the 24 cars both complete: mean +0.10 s, range -6.67 to +3.31 s.

Lap time noisy_lag_ekf minus oracle_struct on the 23 cars both complete: mean +0.68 s, range -2.24 to +9.26 s.

