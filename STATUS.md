# STATUS — F1-Race (L0–L9 done; L10 PAUSED; control expansion: phases A–F done (E closed without benchmark; F's shield part closed), phase G protocol approved, code next)

Since 2026-10-14 the project runs on a new machine (below). Reviewer on this machine: Claude
session **"F1 Race 阶段审查"** (`local_30f94a40-03cb-490d-85b4-12ad1d351d3a`); the reviewer and
builder sessions named in the older entries below ran on the old machine and are not used any
more. Rule: work on branch `layer/Lk` (preparation work: `prep/...`), request review by commit
hash, merge to `main` and push only after APPROVE. Workflow: [docs/workflow.md](docs/workflow.md).

## Machine and environments (from 2026-10-14)

* Dell Pro Max 16 Plus: Core Ultra 9 285HX (24 cores / 24 threads), 63.5 GB RAM, commit limit
  ~73 GB (~46 GB free when idle); GPU RTX PRO 3000 Blackwell 12 GB (not used so far). WSL2
  Ubuntu-24.04 available (24 cores, ~31 GB, gcc / cmake / python3) – the place for acados (L9).
* `.venv`: Python 3.12, rebuilt from `requirements-lock.txt` (same versions as the reference
  environment). `.venv-rl`: CPU torch 2.14.1, gymnasium 1.4.0, stable-baselines3 2.9.0. The
  second environment (Python 3.11) is not set up yet; it is built when the first main results
  of L7 need the cross-check.
* Verified on this machine: `pytest` passes in `.venv`; `test_l5.py`, `test_l6.py` pass in
  `.venv-rl` (nothing skipped); `scripts/run_l0.py` reproduces the L0 numbers below (ideal lap
  71.262 s, closed loop 72.90 / 72.10 s).
* Runs: single-threaded BLAS (`OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`), a fresh process
  per job.
* **Re-plan concurrency: 4.** One minimum-time OCP re-plan (casadi Opti, default `expand: True`,
  SX expansion) commits ~7.4 GB and takes ~65 s here. Limit file
  `%TEMP%\f1race_replan_slots\limit` = 4 and user environment variable `L6_REPLAN_SLOTS=4`
  (`replan_slot` in `src/f1race/experiments/l6.py`).
* Measured, not adopted: without expansion (`expand=False`) a re-plan commits 0.22 GB; the
  η = 0.97 plan is identical (difference 1e-13), the η = 0.93 plan differs slightly (lap time
  1e-7 s), iterations 131 → 200, ~3× the time; 16 at once ≈ 500 re-plans/h, 24 at once ≈ 560/h.
  A full L6 round has ~460 re-plans, so this saves about one hour – kept off for now, to be
  revisited if re-planning limits L7. Changing `planning/ocp_dynamic.py` changes
  `code_fingerprint()` (`experiments/l1.py` hashes the whole file): `results/L1/l1_plans.pkl`
  becomes stale, `scripts/run_l1.py` must be re-run (~13 min) and the data branch re-published.

## Results and data

* `results/` and `data/` are git-ignored and never committed to `main`. Track data:
  `python scripts/fetch_tracks.py` (checked against the pinned SHA-256).
* Results of the old machine: orphan branch **`origin/data/results-v2`** (only `results/`,
  `MANIFEST.txt` with SHA-256; **never merge into `main`**). Restore in the repository root with
  `git archive --format=zip -o r.zip origin/data/results-v2 results` and unzip (write with `-o`;
  a PowerShell pipe corrupts the binary).
* `results/L1/l1_plans.pkl` was regenerated on this machine on `main` (cd3ace5); its fingerprint
  matches the code. The η = 0.93 plan converged to a neighbouring local solution of the old
  cache (lateral offset ≤ 2.3 cm, same lap time); L2–L6 ran on the old cache, so re-runs that
  involve η = 0.93 will not be bit-identical (noted in the MANIFEST).
* Known gaps: the `oracle_plans.pkl` of the individual L6 run worktrees cannot be compared with
  each other (those worktrees stayed on the old machine; `results/L6/oracle_plans.pkl` is the
  restored one); the per-seed intermediate checkpoints of L5 were not restored (the three
  selected policies are in `models/L5`).
* Round 2 of L6: lap outcomes **not verified** (ran during the overload); no L7 decision may rest
  on them.

## Layers

| Layer | Branch | State | Review |
|---|---|---|---|
| L0 deterministic kinematic | `layer/L0` → `main` (18a7a58) | **approved, pushed** to origin/main 2026-10-07 | round 2: APPROVE |
| L1 dynamic single-track + Pacejka | `layer/L1` → `main` (571a982) | **approved, pushed** to origin/main 2026-10-07 | round 2: APPROVE (m4, m5 fixed on `layer/L2`) |
| L2 model mismatch, closed loop | `layer/L2` → `main` (270fb2b) | **approved, pushed** to origin/main 2026-10-08 | round 2: APPROVE (m1, m2 fixed on `layer/L3`) |
| L3 stochastic disturbances | `layer/L3` → `main` | **approved, pushed** to origin/main 2026-10-08 | round 2: APPROVE (nit: truncation definition, fixed on `layer/L5`) |
| L4 learning / adaptation | `layer/L4` → `main` | **approved, pushed** to origin/main 2026-10-08 | round 1: APPROVE (minors fixed in 66f6cb3) |
| L5 reinforcement learning | `layer/L5` → `main` | **approved, pushed** to origin/main 2026-10-09 | APPROVE after the multi-seed redo (wording minors fixed in 4fec887) |
| L6 model mismatch with a structural error | `layer/L6` → `main` | **approved, pushed** to origin/main 2026-10-10 | results review: APPROVE (minors m1–m4 fixed) |
| Preparation for L7 (numerical robustness, filter benchmark) | `prep/robustness` → `main` (a4739c2) | **approved, pushed** to origin/main 2026-10-14 | APPROVE (minors m1–m3, n1–n3 fixed on `layer/L7`) |
| L7 reliable learning | `layer/L7` → `main` | **approved, pushed** to origin/main 2026-10-17 – report [docs/L7.md](docs/L7.md), protocol [docs/L7_protocol.md](docs/L7_protocol.md) (revision 1.1, amendments A1–A10). **H-B not supported:** the main filter (NMPC, N = 30, nominal model + back-offs of estimate (b), L6 failure handling) lowers the completions of all nine L5 checkpoints (exact p = 0.0039; scope: this filter). **H-C (narrowed to the failure-start curriculum, A7) not supported:** the pre-registered median-seed comparison is significant in the opposite direction (F_s4 50 vs P_s1 80 of 105, 4 : 34, p = 6·10⁻⁷); at the seed level no significant difference (p = 0.69), F far more spread (3–103). S3 not shown (+6.29 s > 4.6 s). Secondary: validation-selected checkpoints 104 / 105 of 105 without the filter; training unstable between checkpoints. **Open:** section 8 second environment (A10, deferred by the user); two works of the literature check found but not read. Smoke-test set X (code checks only): cars `sample_plants(nominal, 2, seed 99)[1:]` × τ ∈ {0, 30, 60 ms} and disturbance seed 9999 | protocol d1fa964 CHANGES_REQUESTED → revision 1 APPROVE; Parts A, B, C APPROVE; final review of 328d5ab: **APPROVE** |
| L8 generalisation (new tracks, wet) | `layer/L8` → `main` | **approved, pushed** to origin/main 2026-10-18 – report [docs/L8.md](docs/L8.md), protocol [docs/L8_protocol.md](docs/L8_protocol.md) (revision 1). Test tracks Zandvoort, SaoPaulo, Austin (replaces Norisring: plans failed; a selection effect fixed in advance), Nuerburgring. **H-G holds:** on the new tracks `lag_identified` 100/100 vs `B_s1_500k` 0/100 (p = 1.6·10⁻³⁰); **H-D holds:** wet Spielberg 50/50 vs 8/50 (p = 4.5·10⁻¹³) – scope: these two controllers. The 22 trained policies complete ~0 on the new tracks (nine L5 finals: p = 0.0039); the bare tracker is not worse there (4/100 vs 3/25); SMPC most reliable under L3 on the new tracks (33/40); Part C fine-tuning seed-dependent (0–34 of 35), none on Austin, worse in the wet. **Open:** second environment (H-G, H-D) | protocol fc53b4f CHANGES_REQUESTED → revision 1 APPROVE; code e21806b APPROVE; results b019a0a APPROVE; final review of b908686: **APPROVE** |
| L9 real-time control (equal compute budget) | `layer/L9` → `main` | **approved, pushed** to origin/main 2026-10-19 – report [docs/L9.md](docs/L9.md), protocol [docs/L9_protocol.md](docs/L9_protocol.md) (revision 1, amendments A1, A2, A2.6/A2.7). Runs in WSL at a881aee, 2026-10-18/19. **H-RT1 holds:** `lag_identified` IPOPT 88/105 offline vs 0/105 under the real-time rule (88 : 0, p = 6.5·10⁻²⁷); **H-RT2 holds:** real-time acados RTI 79/105 vs IPOPT 0/105 (79 : 0, p = 3.3·10⁻²⁴). Every IPOPT controller 0/105 in real time (latency 147–344 ms, every deadline missed), also with the fixed 100 ms delay compensation (A2); RTI loses nothing to the budget but completes fewer laps than converged IPOPT offline; RL nearly unaffected (B_s2 the exception); 20 ms sensor delay takes RTI `lag_identified` 79 → 0; Part C students 4–7 offline, 0 real-time. One real-time RL job crashed (exit −11, re-run). **Open:** second environment (H-RT1, H-RT2) | protocol 50e2a8d CHANGES_REQUESTED → revision 1 0af2b60; A1, A2 agreed; a2-M1 fixed 0d52ade APPROVE; a881aee APPROVE (runs released); results 00527f2 APPROVE (minors l9r-m1, l9r-m2 fixed); final review of 598df40: **APPROVE** |

## Open items (to do)

* **L7 section 8 – second environment** (amendment A10, deferred by the user on 2026-10-17): install Python 3.11, build `.venv-311-rl` from the lock file; re-run the acceptance test of estimate (b) and T1 + T2 with and without the main filter for `B_s1_500k` and the median seeds P_s1 / F_s4; report as an addendum to docs/L7.md.
* **L8 section 9 – second environment** (same deferral): re-run H-G and H-D there – the laps of `lag_identified` and `B_s1_500k` in sets A (25 cars x 4 test tracks) and B (wet 0.85 / 0.75); report as an addendum to docs/L8.md.
* **L9 – second environment** (section 10; same deferral): re-run H-RT1 and H-RT2 there (`lag_identified` IPOPT offline / real-time, RTI real-time on T1new + T2new); report as an addendum to docs/L9.md. The second environment is still open for the primary tests of L7, L8 and L9 alike; L9's WSL environment (same package versions on another OS and compiler) is not it and does not replace it. WSL environment (Ubuntu 24.04): `~/acados` (v0.6.0), `~/f1race-venv` (lock file, acados_template, torch 2.14.1+cpu, SB3 2.9.0, gymnasium 1.4.0), `source ~/f1race-env.sh`; setup and the run driver in `scripts/l9_wsl/`. Worktrees for pinned runs go under `C:\Users\yuanche\F1-Race-worktrees\`.
* **L10 – PAUSED — Preserved for Future Development** (2026-10-21, the user's direction): the protocol `docs/L10_protocol.md` revision 1 is binding on branch `layer/L10` (cd4a789 APPROVE, a961b29; pause note 3ba9087); no L10 code or run while paused; nothing of L10 is removed or changed.
* **Control expansion (L1–L9 advanced control methods; not a layer)** – started 2026-10-21: roadmap [docs/CONTROL_EXPANSION_ROADMAP.md](docs/CONTROL_EXPANSION_ROADMAP.md), progress and review state in [docs/CONTROL_EXPANSION_STATUS.md](docs/CONTROL_EXPANSION_STATUS.md), results in [docs/CONTROL_EXPANSION_BENCHMARK.md](docs/CONTROL_EXPANSION_BENCHMARK.md). **Phase A (dynamic MPCC) done** – final review of e2cea5d **APPROVE**, `cx/A-mpcc` merged to `main` (--no-ff) 2026-10-21 as **b68d0a4**. Protocol [docs/CX_A_protocol.md](docs/CX_A_protocol.md) revision 1 with amendments A1 (curvature-aware MPCC after CiMPCC) and A2 (soft speed bound, after the first tuning). **H-A1 not supported** (25 new test cars: `mpcc` 9, `nmpc` 10 completed; 1 : 2, p = 1 – low power, only 3 cars discordant); **H-A2 not supported** (−0.31 s on the 8 cars both complete, p = 0.11); the centre-line MPCC 3 / 25, the plan + tracker 1 / 25; secondary conditions uncorrected, none significant. Not cross-checked in a second environment. **Phase B (MPPI) done** – final review of b276f56 **APPROVE**, `cx/B-mppi` merged to `main` (--no-ff) 2026-10-22 as **2858db4**. Protocol [docs/CX_B_protocol.md](docs/CX_B_protocol.md) revision 0 with amendments B1 (MPPI samples the NMPC's three inputs), B2 (the low-pass MPPI as the primary after the development gate failed for the white-noise MPPI) and B3 (the gate failed at the real-time budget K = 1 024 too; the benchmark ran with the gate waived) – all approved. MPPI solves the NMPC's own problem. **H-B1 holds in `nmpc`'s favour** (50 new test cars: `mppi_lp` 0, `nmpc` 12 completed; 0 : 12, p = 0.0005 – a confirmatory test of the development result, limited to this CPU / numpy / K 1 024 budget); **H-B2 not testable** (no car completed by both); four times the samples (offline) 2 / 50; the tuning was uninformative (0 of 10 for every setting). Secondary conditions uncorrected; `nmpc` / `mpcc` reproduce phase A exactly on CXD / CXL8. Not cross-checked in a second environment. **Phase C (tube MPC) done** – final review of 3dec001 **APPROVE**, `cx/C-tube` merged to `main` (--no-ff) 2026-10-22 as **4b4e091**. Protocol [docs/CX_C_protocol.md](docs/CX_C_protocol.md) revision 0 (clarifications; amendment C1, the numerical-path control) – `nmpc`'s problem with its constraints tightened by an error tube propagated from the L2 parameter box (a linearisation-based approximation, no guarantee). **H-C1 holds in `tube`'s favour** (50 new test cars: `tube` 49, `nmpc` 16 completed; 33 : 0, p = 2.3 · 10⁻¹⁰; the numerical-path control 2 : 0); **H-C2 holds** (`tube` slower by 1.61 s on the 16 cars both complete, p = 3 · 10⁻⁵); against a uniform 12 % reserve 28 : 0 and 1.39 s faster; secondary conditions (uncorrected): steering lag 45 vs 4 of 50, L3 disturbances 28 vs 4 of 30, new tracks 4 of 4 each (tube +3.6 s). The tube knows the parameter ranges of the test cars (a favourable prior). Not cross-checked in a second environment. **Phase D (adaptive MPC / online identification) done** – final review of 2dd4405 **APPROVE**, `cx/D-adaptive` merged to `main` (--no-ff) 2026-10-22 as **ae87f41**; from 4b4e091: literature §D and protocol [docs/CX_D_protocol.md](docs/CX_D_protocol.md) revision 0 **APPROVE** (314129d) with clarifications m1–m3, n1–n3 – an NMPC that updates its model and plan from an online estimate (RLS with forgetting primary; a model bank secondary; the MHE moved to the frontier list) against `nmpc` on new L2 cars, constant (H-D1) and with the grip falling during the lap (H-D2); code and the development checks done (gate passed; the model bank does not identify the cars – kept as registered), packet 2 **APPROVE**; CDV tuning done (`adaptive` λ_μ 0.995, 19 of 20; `adaptive_bank` W 0.2, 1 of 20; packet 3 **APPROVE**). CDT / CDD (707 laps, cd484bd): **H-D1 holds** (50 new L2 cars: `adaptive` 42, `nmpc` 15; 27 : 0, p 1.5 · 10⁻⁸), **H-D2 holds** (the same cars with the grip falling during the lap: 34 : 0, p 1.2 · 10⁻¹⁰); the numerical-path control 0 : 0; the oracle completes 42 / 44, the offline identification with a full re-plan 50 / 0 – the friction-only plan library limits the adaptive architecture, the online controller loses 0 : 8 to the exploration lap on CDT. A drift-start artefact (the first 10 ms at the final grip) found after the report changes no outcome (re-run check). Not cross-checked in a second environment. Packet 4 **APPROVE**. Secondary conditions (uncorrected): under measurement noise every NMPC fails, the oracle included, and only the tube holds (CDT_NOISE `adaptive` 0, `tube` 33 of 50); an unmodelled lag `adaptive` 6 against `tube` 44; a grip wrong from the start `adaptive` 3 against the oracle's 44 (the excitation limit); patches 1 against 10. **Phase E (learning MPC, lap to lap) done – closed without the benchmark**: final review of 6ebaed0 **APPROVE**, `cx/E-lmpc` merged to `main` (--no-ff) 2026-10-22 as **490d33e**; from ae87f41: literature §E and protocol [docs/CX_E_protocol.md](docs/CX_E_protocol.md) revision 0 **APPROVE** (d1c2043) with clarifications m1–m4, n1, n2 – the NMPC with a terminal set and cost-to-go from the car's own laps and a local error regression, ten learning laps per car, against the one-lap identification with a full re-plan (H-E1 lap time, H-E2 completions). Code and development checks done: **the development gate failed** – no sequence completes more than one learning lap with the registered settings (the best diagnostic variant three laps on the nominal car); **amendment E1, option 1**: phase E closed without the benchmark – **H-E1 and H-E2 not tested**; the negative result concerns this implementation (a nonlinear NLP with a convex hull of stored states as the terminal set), not E.2 / E.4's LMPC as such; its suspected cause is a hypothesis; follow-ups listed (E.2's ATV-QP first); the never-run stages of `scripts/run_cx_e.py` and `scripts/report_cx_e.py` are marked as not validated. **Phase F (safety-critical control) done**: packet 3 and the final review of 85be4ad **APPROVE**, `cx/F-safety` merged to `main` (--no-ff) 2026-10-23 as **1922066**; from 490d33e: literature §F.1–F.12 and the protocol [docs/CX_F_protocol.md](docs/CX_F_protocol.md) revision 0 **APPROVE** (e2bacfd) with clarifications m1–m3, n1, n2 – L7's main filter with retries (H-F2, against L7's records) and a backup shield (model predictive shielding with the tracker at 0.85 of the plan speed as the backup, H-F1, against no filter), the nine L5 finals on L7's T1 + T2; an audit of L7's filter records. Code and development checks done: **the development gate failed** – the shield's backup (the tracker at reduced speed, or on a slower plan) does not drive the cars (the 0.85 speed factor leaves the track even on the nominal car; with a slower-plan backup the selected policy completes 3–7 of 12 development laps instead of 12); **amendment F1, option 1** (review of packet 2, g1–g4): the shield part closed without the benchmark – **H-F1 not tested** (the negative result concerns an MPS shield with the L1 tracker as backup and a nominal-model check, on these cars, not shields in general); H-F2 (`retry` against `main`) the only primary test at α 0.05. **T1 + T2** (e0f0b0a, 2 332 laps, 0 errors): `none` reproduces L7 lap by lap, the `main` reproduction check 22 of 22; **H-F2 holds** – L7's main filter with retried solves completes more laps than without retries for eight of the nine finals (median +14, exact p = 0.0078; not cross-checked), its fallbacks fall to 0.05–0.3 % of the decisions; it still completes fewer laps than no filter for eight of the nine (534 against 702 over the nine; L7's filter 423) – the remaining failures follow interventions, false interventions 0.89–0.995; every decision of both filters exceeds 50 ms under load (an idealisation of the simulation, phase G); the bare tracker completes 6 / 14 / 22 laps (none / main / retry) – the filter helps the weak proposer and costs the policies (a description). **Phase G (real-time)** on `cx/G-realtime` (from 1922066): literature §G.1–G.9 and the protocol [docs/CX_G_protocol.md](docs/CX_G_protocol.md) revision 0 **APPROVE** (277e122) with clarifications m1–m3, n1, n2 – L9's RTI `lag_identified` with a model-based prediction over the known delay against L9's uncompensated RTI, at a 20 ms sensor delay (H-G1; L9: 79 → 0) and without one (H-G2), on a new test set under L9's real-time rule (WSL, acados). Next: code and development checks (packet 2).

## Preparation for L7 (2026-10-14, new machine)

In the order the user set: (1) STATUS for the new machine (above); (2) numerical robustness –
the L6 failures located and reproduced first, then fixed: [docs/prep_robustness.md](docs/prep_robustness.md)
(`FrenetMPC(retry=True, fallback="brake")`, `planning/replan.py`; `scripts/prep_robustness.py`,
results in `results/prep`); (3) the solve-time benchmark of the candidate safety filters, decision
rule written before measuring: [docs/L7_filter_benchmark.md](docs/L7_filter_benchmark.md)
(`control/safety_filter.py`, `FrenetMPC(mode="filter")`, `scripts/l7_filter_bench.py`, results in
`results/L7_bench`); (4) the L7 protocol: [docs/L7_protocol.md](docs/L7_protocol.md); (5) L7 runs
start only after the reviewer has confirmed the protocol.

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
| 2026-10-19 | 598df40 | L9 final review (whole layer vs main) | **APPROVE** | 15 commits, 26 files, +3224 / −6; existing files changed: STATUS.md, docs/L7.md, docs/L8.md only, no test changed; pytest .venv (4 skips, gymnasium) and .venv-rl pass on a clean copy, WSL 59 passed (builder); no course material (test + manual scan); no track data or results tracked; all parts reproduced in earlier rounds; open: second environment (L7, L8, L9), L10 protocol first – merged to main |
| 2026-10-19 | 00527f2 | L9 results and report | **APPROVE** | input SHA-256 (14 files) recomputed in the run worktree; timeline: offline and real-time steps never overlap; H-RT1 88 : 0 (p 6.46·10⁻²⁷) and H-RT2 79 : 0 (p 3.31·10⁻²⁴) recomputed from the records; the completion table, repeat, delay, offline exploration, hold_last and Part C counts checked; the DP signed-rank checked against brute-force enumeration; the error record of the crashed RL job kept; minors l9r-m1 (latency averaging stated) and l9r-m2 (the reviewer's light Windows-side commands during 3a) – fixed |
| 2026-10-19 | ef2d1c7 / e7a5f3e | L9 run driver v2 / v3 (outside the run worktree) | accepted | v2: error gate on each job's last record, the crashed job (exit −11) re-run once, a further segfault would stop the runs; v3: logs appended, trainers waited for by pid after v2 hung; src unchanged, RUN_COMMIT stays a881aee |
| 2026-10-18 | a881aee | L9 A2.6 / A2.7 (repeat includes the A2 arm; smoke runs before / after the a2-M1 fix) | **APPROVE** | runs released at this commit |
| 2026-10-18 | 0d52ade | L9 a2-M1 fix (the compensated arm busy until the plan's alignment time) | **APPROVE** | two text additions asked (→ a881aee) |
| 2026-10-18 | 0649ae9 | L9 amendment A2 (delay-compensated IPOPT arm) | text passes; code: a2-M1 | a solve finished early could overwrite a plan not yet started; run order adjusted |
| 2026-10-18 | 66f7627 / f203cd8 | L9 amendment A1 and code part 2 (RL, Part C) | **APPROVE** | A2 agreed as a secondary arm, primary tests unchanged |
| 2026-10-18 | d075dc7 + 9ae4c50 | L9 code part 1 (acados OCP, equivalence check, runner, report code) | code fine, equivalence met | the real-time rule to be fixed by amendment A1 before any T1new / T2new lap |
| 2026-10-18 | 0af2b60 | L9 protocol revision 1 and literature correction | **APPROVE** | protocol binding |
| 2026-10-18 | 50e2a8d | L9 protocol revision 0 | CHANGES_REQUESTED | l9-M1 (timing fidelity: load, interleaved batches), l9-m1 to l9-m4 |
| 2026-10-18 | 3e8573c | Literature addendum (Song & Schneider; Hildisch et al.) to docs/L7.md and docs/L8.md | **APPROVE** | one correction (ICRA 2022), made in 0af2b60 |
| 2026-10-18 | b908686 | L8 final review (whole layer vs main) | **APPROVE** | 9 commits, 10 files as listed, no existing src module or test changed; pytest .venv (4 skips, gymnasium) and .venv-rl pass on a clean copy; no course material (test + manual scan); no track data or results tracked, fetch_tracks only adds SHA entries; all parts reproduced in earlier rounds; open: second environment (L7, L8), two unread works, L9 protocol first – merged to main |
| 2026-10-18 | b019a0a | L8 results | **APPROVE** | input SHA-256 (38 files) recomputed; H-G 100 : 0 (p 1.58·10⁻³⁰) and H-D 42 : 0 (p 4.55·10⁻¹³), Holm and stricter reading; the fixed-set table recomputed cell by cell; l8-M1 (no identification / plan failures); RL distribution and signed-rank recomputed; Part C counts; primary section identical to report_primary; the RL loop with policies checked against L7 (12 laps identical); minor l8r-m1 (curvature statement descriptive) – fixed |
| 2026-10-17 | e21806b | L8 evaluation / report code, job list | **APPROVE** | order download → SHA → plans checked by timestamps; Norisring's failure and the replacement by Austin; wet / patches / exploration / identification / l8-M1 handling; report rules; nits: t_max 120 vs 121 s, SHA list of the inputs, Part C after the primary report |
| 2026-10-17 | 277fa02 | L8 protocol revision 1 | **APPROVE** | l8-M1 and minors fixed; nit: L7 Part C reference source (fixed in 335fcdf); protocol binding |
| 2026-10-17 | fc53b4f | L8 protocol revision 0 | CHANGES_REQUESTED | draw and untouched test tracks verified; l8-M1 failure handling of the identification controllers; minors: car 18 at 0.696, download → SHA → plans, fine-tuning cars and timing, A0 sources; scope of H-G / H-D |
| 2026-10-17 | 328d5ab | L7 final review (whole layer vs main) | **APPROVE** | 21 files as listed; pytest .venv (4 skips, gymnasium) and .venv-rl pass on a clean copy; no course material (test + manual scan); no track data or results tracked, fetch_tracks unchanged; all parts reproduced in earlier rounds; A10 records the user's deferral of section 8; literature check spot-checked; open: second environment, two unread works – merged to main |
| 2026-10-17 | bd09291 | L7 Part C report (T1 + T2) | **APPROVE** | SHA-256 of the 12 checkpoints, median seeds, S1 (4 : 34, p = 6.04·10⁻⁷, Holm and stricter reading), 5 × 5, S3, secondary tables, Spearman 0.933 and F's pool recomputed; docs/L7.md restore checked (no content lost); minors pc2-m1 (Reading generalised the lap-level S1 to the method – seed-level comparison added: Mann–Whitney p = 0.69, S1's p excludes seed variance), pc2-m2 (selected checkpoints not 'the most reliable result') – fixed in docs/L7.md; L7 not closed: section 8 second environment (user's decision pending), literature check of section 1, final layer review |
| 2026-10-16 | 97a393d | L7 Part C evaluation and report code, job list | **APPROVE** | holm(), ranking(), S1 / S3 rules, select() checked; the selection's later-checkpoint rule is L5's own, only the seed tie-break is new; Holm's literal reading accepted (H-B's null rejected in the opposite direction → S1 at 0.05), the stricter decision (0.025) reported alongside; S3 (b) point estimate of the mean; tracker and baselines reused from Part B; A9 before any Part C test result; SHA-256 lists of the copied checkpoints |
| 2026-10-16 | f6bdc5b | L7 Part B step 2 (T1 + T2) | **APPROVE** | all 3 465 RL-loop laps, McNemar counts, H-B (exact p = 0.0039), pb-m2 attribution and false-intervention rates recomputed from the raw data; the 550 unfiltered laps on T1 (τ = 0) and T2 identical to L5; baselines consistent with L6; minors pb2-m1 scope of the conclusion (this filter, no retry; retry not evaluated), pb2-m2 reading marked as interpretation, pb2-m3 limit of the 3 s false-intervention criterion, pb2-m4 consistency checks and baselines by τ, pb2-m5 overlap of fallback runs and interventions – fixed in docs/L7.md; Part C started before the review: accepted, recorded here |
| 2026-10-14 | 36c6fdb | L7 Part A (revised) | **APPROVE** | all 286 extended part-(i) laps identical to the originals decision by decision; A4c recomputed; the reviewer's earlier rough numbers withdrawn (truncation); nit: wording of the check |
| 2026-10-14 | 735c348 | L7 Part A | CHANGES_REQUESTED | PA-M1 A4 (Q − G on failed laps) a selection effect → A4c over all laps; PA-M2 "rare states" contradicts μ ~ U(0.92, 1.03) → two explanations (value error / observability); pa-m1 six of nine; pa-m2 statistics |
| 2026-10-14 | 8bc7605 | amendment A2 | **APPROVE** | A-M1: Part A diagnoses the L7 test laps (T1 at τ = 0, T2) → findings split (i) / (ii), only (i) may change Part C |
| 2026-10-14 | 58ad42e | L7 protocol revision 1.1 | **APPROVE** (check of the three minors) | r-m1 smoothing recomputed (symmetric periodic kernel, h with raw widths, 42.9 % reproduced), r-m2, r-m3 correct; protocol binding; nits (per-group checkpoint-lap cost, width test) fixed as amendment A1 |
| 2026-10-14 | 39b62c4 | L7 protocol revision 1 | **APPROVE** | merge a4739c2 = 89f81fb tree; new CBF barrier derivation checked; 45.6 % recomputed; minors before the first K run: r-m1 w″ spikes from the piecewise-linear widths (code), r-m2 attainability of the H-B test, r-m3 filters see actuator-limited commands; nit: cost of the checkpoint laps – revision 1.1; Part A may start now |
| 2026-10-14 | d1fa964 | L7 protocol (design) | CHANGES_REQUESTED | P-M1 CBF-QP not realisable (a_y beyond the steering's reach, CG-only barrier) → fixed before calibrating; P-M2 main filter and a test of H-B missing, closed loops of K / A unnamed; P-M3 median of three seeds too weak for S1 → five seeds for P and M; minors: V used by the curriculum, touched test laps undisclosed, CBF band heuristic, acceptance k = 10 and 20, slack-using decisions, prep-rule reference; nit: source of the RL gap – all in revision 1 |
| 2026-10-14 | 89f81fb | preparation (STATUS, numerical robustness, filter benchmark) | **APPROVE** | tests, standing checks, nmpc.py default path, replan_robust direct step, report tables and McNemar recomputed, CBF derivation checked; the prep rule's timeline corrected (two retry-only trials finished before it; conclusion unaffected, m1); benchmark decision insensitive to the threshold (≤ 180 ms admissible, NMPC filter 219–235 ms); minors m2/m3 (CBF realisability, axle points, dw/ds), n1–n3 – fixed on `layer/L7` |
| 2026-10-10 | 284a0e3 | L6 results | **APPROVE** | held-out coverage recomputed in Python 3.11 (identical for 75/75 cases); the 12 lag_nominal failures = the 12 L4 failures; tables checked; minors: no-go basis as amendment A11 + held-out near-limit coverage (m1), SMPC 60 ms only on the nominal car (m2), closed-loop coverage valid but secondary (m3), conformal calibration separate from L7 evaluation laps and round-2 replay before using the residual mean (m4) – fixed in 7f744d3 (table only) and the next commit |
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

## L6–L10 (started 2026-10-09 on the old machine, user away; decisions recorded here)

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
* **2026-10-10 18:12: all L6 runs finished** (round 4: 25/25 cars in the first pass). All results
  merged in `results/L6_final/` of this worktree (git-ignored; `SOURCES.txt` lists where each part
  ran), tables `python scripts/run_l6.py report --results results/L6_final`, docs/L6.md written
  (3c057a1). **Waiting for the user's decision on replays** of records produced during the overload:
  round 2 (52 of 75 jobs with solver fallbacks or retried re-plans; or all 75), round 3 cars 6 and 7
  (least squares, fallbacks), round 1 at 10 ms: 5 laps not yet replayed (lag_nominal cars 16, 18,
  22, 23; nmpc car 13). Then the results review request to the reviewer.
* 2026-10-10 evening, user: send for review now; if the existing results support the conclusions,
  update the strategy instead of re-running. Done: docs/L6.md separates what the overload cannot have
  changed (held-out coverage → no-go) from what it can (round-2 laps, not claimed); review request
  `docs/reviews/L6_results_review_request.md` (3a79314), scope up to 284a0e3 (roadmap section 7,
  strategy update for L7). The reviewer's session was offline after the reboot: the message is
  queued (Remote Control session "独立审查员身份和职责" [13ef37]); if it does not answer, the user may
  need to reopen it. No re-runs until the review asks for them.
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
