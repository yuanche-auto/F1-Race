# Control expansion – status (resume point)

Last updated: 2026-10-23 (phase G: protocol revision 0 APPROVE with clarifications m1–m3, n1, n2; code next). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** G – real-time control, branch `cx/G-realtime` from `main` 1922066 (the merge of phase F):
  literature §G.1–G.9 and the protocol [CX_G_protocol.md](CX_G_protocol.md) revision 0 **APPROVE** (277e122) with
  clarifications m1–m3, n1, n2 (H-G2 tests alignment against a later application; H-G1 on top of an undelayed
  identification; the repeat of every primary arm as the noise floor; the offline equivalence in the gate; the WSL
  environment is not the second environment). Next: code and the development checks (packet 2). Design: L9's real-time rule (WSL, acados SQP-RTI, one vCPU, 50 ms); `lag_identified` RTI
  with a model-based prediction over the known delay (`rti_dc`: from the delayed measurement over the sensor delay with
  its own applied commands, and over a fixed 10 ms command latency) against L9's uncompensated RTI, with the
  exploration, identification and plan shared per case; H-G1 at a 20 ms sensor delay (L9: 79 → 0 of 105), H-G2 without
  one; new test set GT (seeds 371, 7700–7729); secondary: 40 ms and a mis-specified delay, `nmpc`, the RL policy, a
  repeat. Advanced-step MPC, learned warm starts, anytime IPOPT, deadline-aware fallback: not in phase G (protocol 3.2).
* **Phase F:** done – packet 3 and the final review of 85be4ad **APPROVE**; `cx/F-safety` merged to `main` (--no-ff)
  2026-10-23 as **1922066**. Phase F result: **H-F2 holds** – L7's main filter with retried solves completes more T1 + T2 laps than L7's
  filter for eight of the nine L5 finals (median +14, exact p = 0.0078; not cross-checked), its solver fallbacks fall from
  0.5–6.8 % to 0.05–0.3 % of the decisions – yet it still completes fewer laps than no filter for eight of the nine (534
  against 702 over the nine; L7's filter 423): L7's negative result stands, its remaining failures follow interventions;
  **H-F1 not tested** – the shield part closed by amendment F1 (no tracker-based backup drives these cars).
  History of phase F:
  F – safety-critical control, branch `cx/F-safety` from `main` 490d33e (the merge of phase E):
  literature §F.1–F.12 and the protocol [CX_F_protocol.md](CX_F_protocol.md) revision 0 **APPROVE** (e2bacfd) with the
  clarifications m1–m3, n1, n2 (the applied action in the observation; lag makes the shield optimistic – failures after
  interventions by τ; the backup's completions on FD as background; the tie rule's reason; one false-intervention procedure).
  Code 7634e2d … b94e775 (`experiments/cx_f.py`, l7 options `retry` / `log_commands`, `scripts/{run,report}_cx_f.py`,
  `scripts/cx_f_dev.py`, `tests/test_cx_f.py`). **Development gate failed:** the backup a_b (the tracker at 0.85 of the
  plan speed) leaves the track on the nominal car (the tracker completes NOM only at factor 1.0), the shield intervenes
  in 87 % of the decisions; a development variant (the tracker on a slower plan as backup) completes at most 9 of 12 FD
  laps alone and turns `B_s1_500k`'s 12 of 12 FD laps into 3–7; `retry` completes NOM. The audit of L7's records ran
  (reproduces L7's false-intervention counts and failure attribution). **Amendment F1, option 1** (the review of packet 2,
  with g1–g4): the shield part closed without the benchmark – **H-F1 not tested** (the negative result concerns an MPS
  shield with the L1 tracker as backup and a nominal-model check, on these cars); H-F2 the only primary test (α 0.05);
  no FV, `shield` or `backup` lap. **T1 + T2** (worktree `F1-Race-wt-CXF-run` at e0f0b0a, 2 332 laps, 0 errors): the
  `none` regression reproduces L7 lap by lap, the `main` reproduction check 22 of 22 (L7's records used); **H-F2 holds**
  – over the nine finals `retry` completes more laps than `main` (+8, +14, +19, +18, +23, +12, +14, +4, −1; p = 0.0078,
  significant at 0.05 and 0.025; not cross-checked); fallbacks 0.05–0.3 % of the decisions (`main` 0.5–6.8 %); but
  `retry` still completes fewer laps than `none` for eight of the nine finals (534 against 702; L7's `main` 423) – the
  remaining failures follow interventions (469 of 524), false interventions 0.89–0.995. Packet 3 and the final review
  of 85be4ad **APPROVE** (the tracker's 6 / 14 / 22 laps added to the Reading as a description). Design: on L7's test sets T1 + T2 with the nine final L5 checkpoints as proposers – `retry`
  (L7's main filter with retried solves) against L7's `main` records (H-F2, L7's open question), and `shield` (model
  predictive shielding: the policy's action passes while a nominal-model rollout of the tracker at 0.85 of the plan
  speed still recovers the car; margin m tuned on the new set FV) against `none` (H-F1); `backup` alone; an audit of
  L7's per-decision filter records. HOCBF on the single-track model, robust and backup CBFs, reachability: not in
  phase F (reasons in the protocol, section 3.2).
* **Phase E:** done – closed without the benchmark (amendment E1, option 1); final review of 6ebaed0 **APPROVE**
  (f1: the unvalidated scripts marked, cd71350), `cx/E-lmpc` merged to `main` (--no-ff) 2026-10-22 as
  **490d33e**. Phase E result: the
  learning MPC as implemented here (a nonlinear NLP with the convex hull of stored states as the terminal set) does not
  complete ten learning laps on the nominal car or the development cars – the development gate failed; **H-E1 and H-E2
  not tested**; the result concerns this implementation, its suspected cause is a hypothesis; follow-ups listed
  (E.2's ATV-QP first).
  History of phase E:
  E – learning MPC (lap to lap), branch `cx/E-lmpc` from `main` ae87f41: literature §E.1–E.9 and the
    protocol [CX_E_protocol.md](CX_E_protocol.md) revision 0 **APPROVE** (d1c2043) with clarifications m1–m4, n1, n2
    (H-E2's unequal exposure, H-E1 conditional, the line, initialisation failures apart). Code d64eb47 … 5068328
    (`control/lmpc.py`, `experiments/cx_e.py`, runner / report / dev scripts, `tests/test_lmpc.py`). **Development gate
    failed:** no sequence completes more than one learning lap with the registered settings (K = 20 off on the first lap
    on every car; the best diagnostic variant three laps on NOM); mechanism – the local convex hull is not a feasible set
    of the nonlinear model near the limit, and the first lap plans at the nominal model's η 0.95 on mismatched cars.
    **Amendment E1, option 1** (the review of packet 2, with e1–e4): phase E closed without the benchmark – **H-E1 and
    H-E2 not tested**; the negative result concerns this implementation, its cause a hypothesis; follow-ups listed (E.2's
    ATV-QP first). No CEV / CET / secondary lap. Final review of `cx/E-lmpc` requested.
    Design: `lmpc` = the NMPC's model and constraints with a local safe set and cost-to-go from the car's own laps (E.2) and
    a local regression of the model error (E.4); two initialisation laps (plans 0.73, 0.79), ten learning laps per car from
    the same initial state; H-E1 lap time and H-E2 completions of the tenth learning lap against `identified` on 50 new L2
    cars (CET, seed 342); `lmpc_nominal` (no regression) and `nmpc` as comparisons.
* **Phase D:** done – final review of 2dd4405 **APPROVE**, `cx/D-adaptive` merged to `main` (--no-ff) 2026-10-22 as
  **ae87f41**. Phase D result: `nmpc` with its model and plan updated online (RLS with forgetting on the friction, a
  friction-indexed plan library) completes 42 of 50 new L2 cars against `nmpc`'s 15 – **H-D1 holds** (27 : 0, p 1.5 · 10⁻⁸)
  – and 34 of the same 50 with the grip falling during the lap against 0 – **H-D2 holds** (34 : 0, p 1.2 · 10⁻¹⁰); the
  two are not independent (the same cars); the numerical-path control 0 : 0. The oracle of the same architecture
  completes 42 / 44, the offline identification with a full re-plan 50 / 0 (`adaptive` 0 : 8 against it on CDT); the
  model bank 1 / 0; secondary: noise, an unmodelled lag, a wet start and short patches defeat online identification
  (the tube holds under noise and lag). Exact states and model structure in the primary sets (an upper bound); a known
  flaw of the drift field (the first 10 ms at f_end) changed no outcome; not cross-checked.
  History of phase D:
  D – adaptive MPC / online identification, branch `cx/D-adaptive` from `main` 4b4e091: literature
    §D.1–D.14 and the protocol [CX_D_protocol.md](CX_D_protocol.md) revision 0 **APPROVE** (314129d) with clarifications
    m1–m3, n1–n3 (the MHE removed, CDT_NOISE added, the drifts reported before CDV). Design: `adaptive` = `nmpc` with its
    model and plan updated online (RLS with vector-type forgetting on the simulator's step model; a friction-indexed plan
    library); `adaptive_bank` (after LLA-MPC), `adaptive_oracle`, `nmpc_param` (numerical path), `tube`, `identified`
    (L8's pipeline) as comparisons; H-D1 on 50 new constant L2 cars (CDT, seed 332), H-D2 on the same cars with a
    friction drift (CDD, seed 333). Code 6f54f30 … ea4ac82 (`FrenetMPC(param_model)` with a plan library – the existing
    NLPs unchanged to the bit; `estimation/rls.py`, `estimation/bank.py`, `control/adaptive_mpc.py`, `experiments/cx_d.py`,
    scripts, tests 8 + 5); 13-plan library (8 new plans); development checks of §9 done – **gate passed** (`adaptive_oracle`
    NOM and 5 of 5 DEV drifts; `adaptive` NOM with every λ_μ; DEV_C `nmpc` 2 / `adaptive` 5 of 5, DEV_D 0 / 5 of 5); the
    model bank does not identify the cars (replays, §9 item 4); packet 2 (cd484bd) **APPROVE** – the tie-break accepted,
    `adaptive_bank` kept as registered. CDV tuning 11:05–11:28 in `F1-Race-worktrees/F1-Race-wt-CXD-run` at cd484bd (120 laps,
    no error; `tune.jsonl` f59485d0fb5c…, `selected.json` b7e631a95b6f…): `adaptive` λ_μ 0.995 – 19 of 20 (0.99: 17, 0.999: 14);
    `adaptive_bank` W 0.2 – 1 of 20 (0.1 and 0.5: 0; most laps end at turn 1, ~463 m). Selections sent for review; CDT /
    CDD (707 laps) wait for it. Packet 3 **APPROVE**. CDT + NOM and CDD 11:30–14:45 at cd484bd (707 laps, no error;
    `test.jsonl` d7f85902eb45…): **H-D1 holds** (27 : 0, `adaptive` 42 / `nmpc` 15 of 50, p 1.5 · 10⁻⁸), **H-D2 holds**
    (34 : 0, 34 / 0, p 1.2 · 10⁻¹⁰); `nmpc_param` 0 : 0 on both; `adaptive_oracle` 42 (CDT) and 44 (CDD), `identified` 50
    and 0, `tube` 46 and 2, `adaptive_bank` 1 and 0. The drift's start (the start state projects to s = L − 1e-11: the first
    10 ms at f_end, the onset diagnostics from the lap start) – found after the report, diagnostics fixed (c8035cb),
    re-run check: no car's outcome changes; packet 4 (report, re-run check, benchmark text) **APPROVE** – `DriftField` kept
    as run (a known flaw, stated in protocol §2; the fix listed for later phases). Secondary conditions 16:05–17:52 at
    cd484bd (840 laps, no error; `secondary.jsonl` a4a39006e632…): measurement noise breaks every NMPC including the oracle
    (CDT_NOISE `adaptive` 0, oracle 1, `nmpc` 3, `tube` 33 of 50; CXD 2 / 7 / 4 / 28 of 30), the unmodelled lag `adaptive` 6
    against `tube` 44, the wet start `adaptive` 3 against the oracle's 44 (turn 1 – the excitation limit), patches 1 / 10 / 1
    / 7 of 30; the RLS's μ bias under noise +0.08; `nmpc` reproduces phase C on CXD 30 of 30. Packet 5 and the final review
    of `cx/D-adaptive` requested.
* **Phase C:** done – final review of 3dec001 **APPROVE**, `cx/C-tube` merged to `main` (--no-ff) 2026-10-22 as
  **4b4e091**. Phase C result: `nmpc`'s constraints tightened by an error tube from the L2 parameter box complete 49 of 50 new
  test cars against `nmpc`'s 16 – **H-C1 holds** (33 : 0, p 2.3 · 10⁻¹⁰; numerical-path control 2 : 0), **H-C2 holds**
  (+1.61 s, p 3 · 10⁻⁵); the tube knows the test cars' parameter ranges (favourable prior); not cross-checked.
  History of phase C:
  C – robust tube MPC, branch `cx/C-tube` from `main` 2858db4: protocol revision 0 **APPROVE** (9006b4a)
    with clarifications m1–m3, n1–n3 (f0b25fb); `control/tube.py`, `experiments/cx_c.py`, runner / report / dev scripts and
    tests (0689c41, db3b075); development checks of §9 done – gate passed (NOM 6 of 6 settings; CXDEV `nmpc` 2 of 5,
    `tube` 5 of 5); packet 2 (635c61f) **APPROVE**; c1 zero-tightening control (76.454 vs 76.400 s, the parameterised
    NLP's numerical path; 8b6b40d). CXV tuning 05:57–06:13 in `F1-Race-worktrees/F1-Race-wt-CXC-run` at 8b6b40d (60 laps,
    no error): `tube` 10 of 10 with every c_R → c_R 1; `tube_track` 3 of 10 → c_R 0.1; packet 3 **APPROVE** with
    **amendment C1** (`nmpc_param`, the numerical-path control; eab7c63). CCT (306 laps) started 06:17 in the same worktree
    at eab7c63 (RUN_COMMIT_tune 8b6b40d kept); done 07:06 (306 laps, `test.jsonl` bcf5e9f743d8…): **H-C1 holds for `tube`**
    (49 vs 16 of 50, 33 : 0, p 2.3 · 10⁻¹⁰), **H-C2 holds** (`tube` +1.61 s, p 3 · 10⁻⁵); noise floor `nmpc_param` 2 : 0;
    packet 4 **APPROVE** (m1 text fixed in 810e69f). Secondary conditions 07:27–08:23 (420 laps, `secondary.jsonl`
    16d58eb9e999…): LAG30 `tube` 45 vs `nmpc` 4 of 50, CXD 28 vs 4 of 30, CXL8 4 vs 4; `nmpc` reproduces phase B 34 of 34.
    Packet 5 and the final review of `cx/C-tube` requested.
* **Phase B:** done – final review of b276f56 **APPROVE**, `cx/B-mppi` merged to `main` (--no-ff) 2026-10-22 as **2858db4**.
  Phase B result: MPPI on the NMPC's problem at the real-time CPU budget (numpy, K 1 024) completes 0 of 50 test cars
  against the NMPC's 12 – **H-B1 holds for `nmpc`** (0 : 12, p 0.0005; confirmatory, after the development gate failed
  and was waived by amendment B3); H-B2 not testable; 4 K samples offline 2 of 50; not cross-checked.
  History of phase B:
  B – MPPI, branch `cx/B-mppi` from `main` b68d0a4: literature (§B of the literature review) and
    protocol revision 0 (`docs/CX_B_protocol.md`) **APPROVE** (0b94d1a) with clarifications m1, m2, n1–n4 written in;
    `control/mppi.py`, `experiments/cx_b.py`, the runner / report and their tests written (57146e4, d80e1a6); development
    checks 9.1 and 9.2 done – 9.2 found that the NMPC drives and brakes at once in its braking zones, so **amendment B1**
    (MPPI samples the NMPC's three inputs) was proposed (8d86ff2) and approved (c8806c1). K timing: K = 2 048. Development
    gate (9.6) **failed** for `mppi` (white noise: 0 of 17 nominal laps); `mppi_lp` 2 of 8 – **amendment B2** (the
    low-pass MPPI as the primary) proposed in eea5a49; packet 2 (c3fca85) and B2 **APPROVE**; review c1: the K rule
    on `mppi_lp` gives K_RT = 1 024 (K 2 048: median 44.6 ms); c2 traces recorded; c3 scope. Packet 3 (770cd7b) **APPROVE**;
    g1: the gate at K_RT fails for `mppi_lp` too (0 of 9 nominal laps) → **amendment B3** (e9fb7e1) **APPROVE**, option 1:
    the pre-registered tuning and benchmark run with the gate waived (d1, d4 in d93b5e9). CXV tuning step 1 started
    2026-10-22 01:56 in `F1-Race-worktrees/F1-Race-wt-CXB-run` at d93b5e9 (inputs identical to the main checkout and to
    phase A's run). CXV tuning done (01:56–01:59; 120 laps, no error record): every setting 0 of 10 in both steps – the
    rule falls to the grid centre, `mppi_lp` λ 1, c_σ 1, f_c 2 Hz, K 1 024 (an uninformative tuning, review d3);
    packet 4 (the selections) **APPROVE**. CBT 02:00–02:22 (408 laps, `test.jsonl` f869d47d3632…): **H-B1 holds for `nmpc`**
    (0 : 12, p 0.0005), H-B2 not testable; packet 5 **APPROVE**. Secondary conditions 02:25–02:56 (504 laps,
    `secondary.jsonl` 2219e5634a68…); phase-A reproducibility 68 of 68 (after the NaN-comparison fix baa69cb). Final review
    of `cx/B-mppi` requested. A done (final review of e2cea5d
    APPROVE; `cx/A-mpcc` merged to `main` --no-ff as **b68d0a4**, 2026-10-21).
* **Original reviewer:** session `local_30f94a40-03cb-490d-85b4-12ad1d351d3a` ("F1 Race 阶段审查"), the
  reviewer of L7–L9 – reached by SendMessage with this id (its pipe address changes after restarts).
* **L10:** PAUSED — Preserved for Future Development (branch `layer/L10`, protocol revision 1 binding,
  pause note 3ba9087). Nothing of L10 is touched by the expansion.

## Modules

| phase | module | developed | unit-tested | closed-loop checked | reviewed | accepted |
|---|---|---|---|---|---|---|
| A | `control/mpcc.py` (`DynamicMPCC`, `MPCCController`, `path_errors`) | yes (0061e79) | yes – `tests/test_mpcc.py`, 8 tests | development laps; CXV tuning; CXT benchmark; secondary conditions | **APPROVE** (f4d1ee9 … e2cea5d) | **yes – phase A completed** |
| A | runner and report (`experiments/cx_a.py`, `scripts/run_cx_a.py`, `scripts/report_cx_a.py`) | yes | yes – `tests/test_cx_a.py`, 7 tests (5 at 6bcc6ef) | smoke on the nominal car; the CXV / CXT / secondary runs | **APPROVE** (6bcc6ef … e2cea5d) | **yes** |
| A | `mpcc_ca` (CiMPCC-based, amendment A1) and the secondary conditions; soft speed bound (amendment A2) | yes | yes – `tests/test_mpcc.py` 8, `tests/test_cx_a.py` 7 | smoke on cases outside every set (`results/CX/A/dev/secondary_smoke.jsonl`); A2 diagnostic on CXV cars 2 / 5 / 7; CXT | **APPROVE** (94cba68 … e2cea5d) | **yes** |
| B | `control/mppi.py` (`MPPI`), `experiments/cx_b.py`, `scripts/run_cx_b.py`, `scripts/report_cx_b.py`, `scripts/cx_b_dev.py` | yes (57146e4 … baa69cb) | yes – `tests/test_mppi.py` 12, `tests/test_cx_b.py` 6 | development checks 9.1–9.8 on the nominal car; CXV tuning; CBT benchmark; secondary conditions | protocol rev 0, B1, B2, B3 and packets 2–6 **APPROVE**; final review b276f56 **APPROVE** | **yes – phase B completed** |
| C | `control/tube.py` (`Tube`, `TubeMPCController`), `experiments/cx_c.py`, `scripts/{run,report}_cx_c.py`, `scripts/cx_c_dev.py` | yes (0689c41 … eab7c63) | yes – `tests/test_tube.py` 8, `tests/test_cx_c.py` 5 | development checks §9 on NOM and CXDEV; CXV tuning; CCT; secondary conditions | protocol rev 0, C1 and packets 2–5 **APPROVE**; final review 3dec001 **APPROVE** | **yes – phase C completed** |
| D | `estimation/{rls,bank}.py`, `control/adaptive_mpc.py`, `FrenetMPC(param_model=True)`, `experiments/cx_d.py`, `scripts/{run,report}_cx_d.py`, `scripts/cx_d_{dev,plans}.py` | yes (6f54f30 … ea4ac82) | yes – `tests/test_adaptive.py` 8, `tests/test_cx_d.py` 5 | development checks §9 on NOM and DEV (gate passed) | protocol rev 0 and literature §D **APPROVE** (packet 1, m1–m3 / n1–n3); packets 2–5 **APPROVE**; final review 2dd4405 **APPROVE** | **yes – phase D completed** |
| E | `control/lmpc.py`, `experiments/cx_e.py`, `scripts/cx_e_dev.py`; `scripts/run_cx_e.py` (only its `dev` stage run) and `scripts/report_cx_e.py` – **not validated: never run end to end** (review f1) | yes (d64eb47 … 5068328) | yes – `tests/test_lmpc.py` 5 | development checks §9: NLP, regression, 36 sequences – **gate failed** | protocol rev 0 **APPROVE** (packet 1); packet 2 **APPROVE**, amendment E1 option 1 (closed, no benchmark); final review 6ebaed0 **APPROVE** (f1) | **yes – phase E completed (closed without the benchmark)** |
| F | `experiments/cx_f.py`, l7 options `retry` / `log_commands`, `scripts/{run,report}_cx_f.py`, `scripts/cx_f_dev.py` | yes (7634e2d … e0f0b0a) | yes – `tests/test_cx_f.py` 5 | development checks §9 on NOM and FD – **gate failed** (the shield part closed, F1); audit of L7's records; T1 + T2: `none` regression, `main` reproduction, `retry` (2 332 laps) | literature §F and protocol rev 0 **APPROVE** (packet 1, m1–m3 / n1, n2); packet 2 **APPROVE**, amendment F1 option 1 (g1–g4); packet 3 and final review 85be4ad **APPROVE** | **yes – phase F completed (H-F2 holds; H-F1 not tested)** |
| G | – (code next) | – | – | – | literature §G and protocol rev 0 **APPROVE** (packet 1, m1–m3 / n1, n2) | – |
| F–G | – | not started | – | – | – | – |

## Review record

| date | commit | scope | verdict | findings / fixes |
|---|---|---|---|---|
| 2026-10-21 | f4d1ee9 | phase A packet 1: audit and roadmap docs, MPCC code and tests, development checks, CX-A protocol revision 0 | code, tests, math and workflow conventions **APPROVE**; protocol rev 0 CHANGES_REQUESTED | cx-M1 tuning asymmetry (nmpc untuned) → nmpc gets a 6-setting grid with the same rule; cx-M2 the speed is bounded by the plan → question and scope rewritten, speed-bound activity and speed / plan-speed ratio reported; cx-m1 q_c 0.05 kept in the grid, stated; cx-m2 H-A2 conditional on completion, discordant cars listed; cx-m3 mpcc_ca amendment before CXT; cx-m4 sections renumbered (development checks now §9) |
| 2026-10-21 | e8a5cd5 | CX-A protocol revision 1 | **APPROVE** (binding) | nit: state the grid sizes (24 / 6) and the selection bias in the scope – done; stage-0 bound activity is a measured quantity (the bound acts on stages 1 … N) |
| 2026-10-21 | 6bcc6ef | phase A packet 2: runner and report code, tests, nominal-car smoke | **APPROVE** | m1 the full tie-break order and the skip rule written into protocol §4 before CXV (done, protocol clarification); before the CXT report: m2 the report refuses missing cars / error records and checks the settings against selected.json, m3 the missing §5 columns, m4 the force-increment definition and a 50 ms difference column, n1 fallback solves out of the stage statistics, n2 the same cars for every setting asserted; secondary conditions and the mpcc_ca amendment reviewed before they run |
| 2026-10-21 | 94cba68 | phase A packet 3: CXV tuning results (8babe7b), review fixes m2–m4 / n1 / n2 (a1f0a73), amendment A1 (mpcc_ca after CiMPCC) and the secondary-condition code (549e4bd), amendment A2 (soft speed bound, re-tuning of the MPCCs on CXV) | **APPROVE** (conditions c1–c3, done in c7c802f; nits n3–n5 done) | the first CXV tuning accepted as the basis of A2; A2 and A1 agreed |
| 2026-10-21 | c7c802f (run) | phase A packet 4: A2 re-tuning of the MPCCs (480 laps) and the mpcc_ca tuning (60 laps) on CXV | **APPROVE** | report to state: choices on grid edges, mpcc_ca ties at noise level, hard cars / low power of H-A1, CXV lap times selection-biased; nit: timeline lacks 'start tune_ca' – hand note added |
| 2026-10-21 | c7c802f (run) | phase A packet 5: CXT (130 laps, test.jsonl 2e560c407137…) and its report; the secondary conditions running | **APPROVE** (all records and statistics recomputed) | m1 mpcc_centre's failure places on CXT vs the nominal car, n1 reference-path statement limited, n2 'all five controllers here', n3 mean laps on different cars – fixed in d7ea54d |
| 2026-10-21 | e2cea5d | phase A packet 6: the secondary conditions (236 laps, secondary.jsonl f3a3a5399e0a…; the missing CXL8 inputs and the 4 re-run jobs) and the final phase-A review request | **APPROVE** | the missing CXL8 inputs: failures before any simulation, inputs identical to the main checkout and the L8 run, handling and disclosure accepted |
| 2026-10-21 | e2cea5d | **phase A final review** (whole branch vs main c010887: 14 commits, 16 files, +2404 / −2; only STATUS.md among the existing files) | **APPROVE** | the three standing checks pass (.venv 70 + 4 skips, .venv-rl all); nit: status lines updated before the merge |
| 2026-10-21 | b68d0a4 | merge check of phase A (main) | correct | nit: the runner row's test count (5 vs 7) – fixed in d75f037 |
| 2026-10-21 | 0b94d1a | **phase B packet 1**: literature B.1–B.11, CX-B protocol revision 0 (no code) | **APPROVE** (binding) | m1 (a) no valid sample → U only shifted, counted, unit-tested; (b) K rule selects nothing → amendment, timing at the grid centre; m2 K₀ = max(1, ⌊αK⌋); n1 seed 10 (T1new) in the list; n2 mppi_lp filter transient; n3 CXD / CXL8 re-runs expected identical; n4 scope: CPU numpy implementation – all written into the protocol before any timing or development lap |
| 2026-10-22 | 8d86ff2 | amendment B1 (MPPI samples the NMPC's three inputs) | **APPROVE** (decision) | c1 status before the K timing (c8806c1), c2 clamp tests, c3 overlap for both controllers with the left-foot-braking remark; disclose the informal timings in packet 2 |
| 2026-10-22 | c3fca85 | **phase B packet 2**: MPPI code, runner / report, tests, development checks 9.1–9.6 and the post-gate diagnostics; amendment B2 | **APPROVE**; B2 **APPROVE** | c1 re-time K on mppi_lp (→ K_RT 1 024); c2 traces as records; c3 scope (12 vs 6 settings, primary changed after the gate); n1 median budget, n2 nominal-U limitation, n3 common random numbers |
| 2026-10-22 | 770cd7b | phase B packet 3: the B2 roles, K_RT 1 024 (c1), traces (c2), c3 / n1 / n3 | **APPROVE**; g1 blocking | g1 the 9.6 gate at K_RT (0 of 9, 99955f4); n1 budget numbers at 1 024, n2 track fixture (99955f4) |
| 2026-10-22 | e9fb7e1 | amendment B3 (the gate failed at K_RT) | **APPROVE**, option 1 | d1 H-B1 confirmatory, budget-limited; d2 report wording; d3 tuning by the rule even if uninformative; d4 frontier candidates (d93b5e9) |
| 2026-10-22 | worktree d93b5e9 | phase B packet 4: CXV tuning (120 laps; 0 of 10 everywhere, the grid centre) | **APPROVE** | report: the choice is the closeness rule, the better direction (c_σ < 0.5) lies outside the grid; CXV step times carry the load |
| 2026-10-22 | worktree d93b5e9; 82c31f9 … 78877f6 | phase B packet 5: CBT (408 laps) and its report / benchmark text | **APPROVE** | n1 mppi_big reaches the 2.7 / 3.83 km corners on 10 cars (fixed in 5fccae5) |
| 2026-10-22 | worktree d93b5e9; d813143, fc109a0, baa69cb | phase B packet 6: secondary conditions (504 laps), reproducibility 68 of 68 (after the NaN-comparison fix) | **APPROVE** | – |
| 2026-10-22 | b276f56 | **phase B final review** (whole branch vs main b68d0a4: 30 commits, 13 files, +2825 / −14) | **APPROVE** | the three standing checks pass (.venv 88 + 4 skips, .venv-rl 92); nit: status lines marked done before the merge |
| 2026-10-22 | 9006b4a | **phase C packet 1**: literature C.1–C.10, CX-C protocol revision 0 (no code) | **APPROVE** (binding) | m1 K shapes the tube only (not applied), coverage compares two systems; m2 η_k also tightens the axle-force limits; m3 secondary controllers, nmpc as the reproducibility check; n1 mismatch at perturbed points not covered; n2 nmpc_reserve a variant of L2's; n3 tuning asymmetry reversed – written into the protocol in f0b25fb |
| 2026-10-22 | 635c61f | phase C packet 2: tube code, tests, development checks (gate passed), runner / report | **APPROVE** | c1 zero-tightening control (done: 0.054 s, the parameterised NLP; 8b6b40d); n1 timing sources, n2 η capping in the Reading, n3 CXDEV as background |
| 2026-10-22 | 8b6b40d (run) | phase C packet 3: c1 and the CXV tuning (tube c_R 1, tube_track c_R 0.1) | **APPROVE** | amendment C1: `nmpc_param` as the numerical-path noise floor (eab7c63) |
| 2026-10-22 | eab7c63 (run); c52897c, 02bc302 | phase C packet 4: CCT (306 laps) and its report / benchmark text | **APPROVE** | m1 the capped-stage shares of the fast sections (fixed in 810e69f) |
| 2026-10-22 | eab7c63 (run); 306d39f | phase C packet 5: secondary conditions (420 laps), nmpc reproduces phase B 34 of 34 | **APPROVE** | n1 the new-tracks sentence (tube_track 3 of 4) – fixed before the merge |
| 2026-10-22 | 3dec001 | **phase C final review** (whole branch vs main 2858db4: 19 commits, 12 files, +1913 / −13) | **APPROVE** | the three standing checks pass; status lines marked done before the merge |
| 2026-10-22 | 314129d | phase D packet 1: literature D.1–D.14, protocol revision 0 | **APPROVE** | m1 the ceiling reading (gap to the oracle) and CDT_NOISE; m2 the drawn drifts before CDV, H-D1 / H-D2 dependent; m3 the MHE removed; n1–n3 |
| 2026-10-22 | cd484bd (6f54f30 …) | phase D packet 2: code, tests, development checks (gate passed) | **APPROVE** | (a) tie-break to the smaller setting accepted; (b) `adaptive_bank` kept as registered; n1 read too-fast with completion; n2 RUN_COMMIT + timeline for the runs |
| 2026-10-22 | cd484bd (run) | phase D packet 3: CDV tuning (120 laps; `adaptive` λ_μ 0.995, `adaptive_bank` W 0.2) | **APPROVE** | report notes: the CDD failures against the drift's progress, the bank as not applicable, CDV background only |
| 2026-10-22 | cd484bd (run); c8035cb, 8d27088 | phase D packet 4: CDT / CDD (707 laps), H-D1 27 : 0 and H-D2 34 : 0 hold; the drift-start artefact and its re-run check (decision rule fixed before it) | **APPROVE** | `DriftField` kept as run; the flaw stated in protocol §2; the start fix listed for later phases |
| 2026-10-22 | cd484bd (run); 2dd4405 | phase D packet 5: secondary conditions (840 laps), nmpc reproduces phase C 30 of 30 | **APPROVE** | – |
| 2026-10-22 | 2dd4405 | **phase D final review** (whole branch vs main 4b4e091: 12 commits, 17 files, +3017 / −45) | **APPROVE** | the three standing checks pass (.venv and .venv-rl); status lines marked done before the merge |
| 2026-10-22 | d1c2043 | phase E packet 1: literature E.1–E.9, protocol revision 0 | **APPROVE** | m1 H-E2's exposure, m2 H-E1 conditional, m3 the line and δ, m4 initialisation failures apart; n1 survivors, n2 w_T |
| 2026-10-22 | 727297f (d64eb47 …) | phase E packet 2: code, tests, development checks – the gate failed; amendment E1 | **APPROVE**; **E1 option 1** (close without the benchmark) | e1 the result about this implementation, the cause a hypothesis; e2 the observations incl. the 75.9 s lap; e3 follow-ups; e4 the code merged |
| 2026-10-22 | 6ebaed0 | **phase E final review** (whole branch vs main ae87f41: 7 commits, 12 files, +1751 / −13) | **APPROVE** | f1 the never-run scripts marked (cd71350); the three standing checks pass (.venv and .venv-rl) |
| 2026-10-22 | (this commit) | phase F packet 1: literature §F.1–F.12, protocol CX_F revision 0; the phase-E merge hash 490d33e recorded; the phase-E closing dates corrected (2026-10-23 → 2026-10-22, the commits' date) | **APPROVE** | m1 the applied a_b in the policy's observation (unlike L7's filters); m2 the nominal rollout ignores lag – failures within 3 s after interventions by τ; m3 `backup` on FD as background, < 6 of 12 → a more conservative a_b discussed before FV; n1 the tie rule's reason; n2 one false-intervention procedure for all; the skip count on 490d33e is 4 (my count of 3 from the progress dots was wrong) |
| 2026-10-22 | (this commit; code 7634e2d … b94e775) | phase F packet 2: code, tests, development checks – the shield's gate failed; amendment F1; the audit of L7's records | **APPROVE**; **F1 option 1** (the shield part closed) | g1 the scope (an MPS shield with a tracker backup and a nominal check, on these cars) and the m-grid flaw stated; g2 H-F2 alone at α 0.05, 0.025 reported, H-F1 withdrawn before any T1 / T2 result; g3 none regression, main reproduction, retry 1 155 laps from a pinned worktree; g4 the shield code merged as development only, follow-ups in METHOD_MATRIX |
| 2026-10-23 | (this commit; run e0f0b0a) | phase F packet 3: T1 + T2 – the `none` regression, the `main` reproduction check, `retry`; H-F2 holds; the benchmark section – with the **final review** of `cx/F-safety` | **APPROVE** (both) | the bare tracker's 6 / 14 / 22 laps (none / main / retry) added to the Reading as a description |
| 2026-10-23 | (this commit) | phase G packet 1: literature §G.1–G.9, protocol CX_G revision 0; the phase-F merge hash 1922066 recorded | **APPROVE** | m1 H-G2 tests the alignment against a later application, application delays reported; m2 H-G1's scope: an undelayed, shared identification (L9's restriction); m3 every primary arm repeated, flips per arm next to the discordant counts; n1 the offline equivalence a gate condition; n2 WSL is not the second environment |

## Tests

`tests/test_mpcc.py` (8) and `tests/test_cx_a.py` (7) at e2cea5d (6 and 5 at 6bcc6ef), and the full suite – see the latest review packet for the command and output.

## Results

`results/CX/A/dev/` – the 8 development laps at 0061e79 and the secondary-path smoke (549e4bd); `results/CX/A/smoke/` –
the runner's smoke on the nominal car. CXV tuning (8babe7b): worktree `F1-Race-worktrees/F1-Race-wt-CXA-tune`,
`results/CX/A/{tune.jsonl, selected.json, report/tune.md, RUN_COMMIT, SHA256_inputs*.txt, timeline.tsv}`.
The first CXV selection is kept as `selected_first.json` and `report/tune_first.md` when the A2 re-tuning
writes `selected.json` / `report/tune_a2.md` (review c1). CXT: `test.jsonl`, `report/CX_A.md` (2026-10-21 21:33–21:50); secondary conditions: `secondary.jsonl`,
`report/CX_A_secondary.md` (21:51–22:16). All git-ignored.

Phase B: `results/CX/B/dev/` (main checkout) – the development checks of CX-B §9 (`checks.json`, `nmpc_nom.npz`,
`timing.json`, `timing_lp.json`, `gap.json`, `laps.jsonl`, `nom_diagnostic*.jsonl`, `traces.*`, `gate_k1024.jsonl`) and
the runner smoke (`smoke/`). Benchmark: worktree `F1-Race-worktrees/F1-Race-wt-CXB-run` at d93b5e9, `results/CX/B/`
(`tune.jsonl`, `tune_lp.jsonl`, `selected*.json`, `test.jsonl`, `secondary.jsonl`, `report/`, `RUN_COMMIT`,
`SHA256_inputs*.txt`, `timeline.tsv`). All git-ignored.

Phase C: `results/CX/C/dev/` (main checkout) – the development checks of CX-C §9 (`wcheck.json`, `laps.jsonl`, `cxdev.jsonl`,
`null.jsonl`, `null_cause.json`, `capmap.json`) and the runner smoke (`smoke/`). Benchmark: worktree
`F1-Race-worktrees/F1-Race-wt-CXC-run` (tuning at 8b6b40d, CCT and secondary at eab7c63), `results/CX/C/` (`tune.jsonl`,
`selected.json`, `test.jsonl`, `secondary.jsonl`, `report/`, `RUN_COMMIT*`, `SHA256_inputs*.txt`, `timeline.tsv`).
All git-ignored.

Phase F: `results/CX/F/dev/` (main checkout) – the development checks of CX-F §9 (`monitor.json`, `laps.jsonl`,
`gate.json`, `backup_plans.jsonl`, `monitor_variants.json`, `variants.jsonl`, `meta_*.json`), decision records
`results/CX/F/dev_dec/`, `devvar_dec/`; the audit of L7's records `results/CX/F/report/audit_l7.*` (b94e775). T1 + T2:
worktree `F1-Race-worktrees/F1-Race-wt-CXF-run` at e0f0b0a, `results/CX/F/` (`test.jsonl` 248c584b…, `test_dec/`,
`report/CX_F.{md,json}`, `RUN_COMMIT_test`, `SHA256_inputs_{run,main_checkout}.txt`, `timeline.tsv`, `log_test.txt`,
`log_report.txt`, `launch_test.sh`). All git-ignored.

Phase E: `results/CX/E/dev/` (main checkout) – the development checks of CX-E §9 (`laps.jsonl` with `laps_meta.json`,
`diag.jsonl` with `diag_meta.json`, `nlp.json`, `regression.json`, `summary.json`). Git-ignored.

Phase D: `results/CX/D/plans/` (main checkout) – the new plans of the library (`plans_cxd.pkl`, `plans_cxd_log.json`);
`results/CX/D/dev/` – the development checks of CX-D §9 (`unchanged.json`, `step.json`, `nlp.json`, `plans.json`,
`replay.json` with `replay_laps/`, `laps.jsonl`, `gate.json`, `drifts.json`; after the CDT / CDD report
`cdd_failures.json`, `cdd_rerun.jsonl`, `cdd_rerun.json`, `replay_v1_onset_at_lap_start.json`) and the runner smoke
(`smoke/`). Benchmark: worktree `F1-Race-worktrees/F1-Race-wt-CXD-run` at cd484bd, `results/CX/D/` (`tune.jsonl`,
`selected.json`, `test.jsonl`, `report/`, `RUN_COMMIT_tune`, `RUN_COMMIT_test`, `SHA256_inputs*.txt`, `timeline.tsv`,
`identified_cache/`). All git-ignored.

## Known blockers

* The curvature-aware MPCC needs the full text of Ma et al. (Control Engineering Practice 171, 2026),
  which may be behind a paywall (phase A used CiMPCC instead).
* The second Python environment (deferred by the user) – primary results stay "not cross-checked".

## Next actions

1. Phase G: the code of protocol section 8.2 and the development checks of section 9 (packet 2); no GT lap before its
   APPROVE.
2. Follow-up candidates (CONTROL_METHOD_MATRIX.md) stay unscheduled – among them E.2's ATV-QP learning MPC.
