# Control expansion – status (resume point)

Last updated: 2026-11-24 (phase J opened – the user chose robust adaptive MPC; literature §J and the protocol CX_J revision 0 written, packet 1 for review; no code). Phase I is done (merged 70a51f6). Roadmap: [CONTROL_EXPANSION_ROADMAP.md](CONTROL_EXPANSION_ROADMAP.md).

* **Current phase:** J – frontier 3, robust adaptive MPC: a tube around an adaptive model (the user's choice,
  2026-11-24). Branch `cx/J-robust-adaptive` from `main` 85059dc; the merge of phase I is `main` **70a51f6** (recorded in
  the branch's first commit).
  * Literature §J and [CX_J_protocol.md](CX_J_protocol.md) revision 0 were reviewed as packet 1 (2187b53):
    **CHANGES_REQUESTED** (M1 the box is not phase C's, M2 the residual term inflates the tube, m1–m5, n1, n2). The second version
    works them in; packet 1b is for review. **No phase-J code exists.**
  * **Design:**
    * phase D's online estimate (RLS) as the model of phase C's tube;
    * the tube's box shrunk by the estimate's covariance, under a symmetric cap that contains phase C's box;
    * an initial state-error term and a small per-stage term, both from the controller's own past prediction errors.
  * **Primaries:**
    * H-J1 – under a falling grip, `ra` against `adaptive`;
    * H-J2 – under noise and lag, `ra` against `tube` (it can come out against `ra`).
  * The gate on DEV is in the protocol's section 9. If it fails, the phase closes without a benchmark by default.
  * **Corrections to phase I's text (merged in 70a51f6)**, found by the builder on re-reading and made on this branch: CX_I
    10.9.1 (the v_c laps' usage peaks are not all on the straights) and 10.9.5 (538 s is not a learning lap).

* **Phase I is done:**
  * The final review of f2e4263 was **APPROVE**, and `cx/I-lmpc-followups` was merged to `main` (--no-ff) on
    2026-11-01. The merge hash is recorded in the next branch's first commit.
  * **Phase I was closed without a benchmark**, because the user closed the LMPC line after the reproduction check
    (CX_I section 11):
    * I-1's gate failed, and I-2 and I-3 were never registered;
    * H-I1 and H-I2 are not tested.
  * The negative result's scope is this implementation – phase E's NLP with a soft hull of the K nearest stored states
    – on these simulated cars, with one run per arm on NOM for the check. It was not cross-checked.
  * The next frontier method is the user's choice, with literature and a protocol first. The LMPC line's open points are
    in CONTROL_METHOD_MATRIX.md ("From phase I"); they are not scheduled.

  History of phase I:
  I – frontier 2, the learning MPC's follow-ups. The user's instruction (2026-10-28): try the method
  matrix's follow-ups (CX-H review i3) in order, and discuss if they still fail. Branch `cx/I-lmpc-followups` from `main`
  98f8134 (the merge of phase H); literature §I.1–I.4 and the protocol [CX_I_protocol.md](CX_I_protocol.md) revision 0
  **APPROVE** (bdc02d0) with clarifications m1 (the benchmark registered only after stages A and B pass), m2 (the first
  learning lap's bound and its expected lap time; the reference past the line), n1 (ε tuned, K 20), n2 (`pb_nominal`
  descriptive in the gate). I-1's code 62fc41d (`LearningMPC(speed_bound=True)`, `LapStore.v_ref`,
  `LMPCController(eps)`, `experiments/cx_i.py`, `scripts/{run,report}_cx_i.py` (dev only), `scripts/cx_i_dev.py`,
  `tests/test_cx_i.py`). **I-1's gate failed:**
  * stage A passed – NOM's first learning lap completed at every ε (81.8, 80.8, 79.1 s), as clarification m2 expected;
  * stage B failed – `pb` left on lap 2 on NOM and on lap 1 or 2 on every DEV car (0 of 5);
  * `pb_nominal` (no regression; descriptive) improved lap by lap to 77.3–77.7 s and left on laps 3–5, NOM on lap 5;
  * phase E's NLP reproduces exactly with the bound off.
  I-1 is closed for this phase (rule 4); packet 2 reports it. No IV or IT lap. Packet 2 **APPROVE** (2026-10-29): the
  review asks that an I-2 / I-3 revision first state how it addresses the failure near the limit on NOM with an exact
  model (passed to the user as written).
  * **The user's decision (2026-10-29):** discuss before I-2. After reading how the published racing LMPCs were run
    (literature §E, §I, I.5), the user asked for **a reproduction check first** – protocol **revision 1, section 10**
    (packet 3 **APPROVE**, 1e2bf96, with clarifications m1–m3, n1–n3): phase E's NLP without the regression (R1; R2
    with phase E's start; R3 phase H's ATV-QP) on NOM under the published conditions – two centre-line laps at a
    constant 8 m/s, consecutive laps, 40 learning laps; readings fixed in advance ((c) split into (c1) / (c2)). A
    diagnostic: no gate, no benchmark; I-2 and I-3 not registered. Code b34eb97; diagnostics 418bf3a, bca7802, aae7f30.
  * **The check's results (section 10.9, packet 4 APPROVE with the clarifications of its review):**
    * r0 reproduces three NOM records exactly.
    * r2: both v_c laps completed (538.5 and 538.8 s). Their tyre usage reached 0.95, longitudinally (the tracking
      NMPC's drive and brake), not the "about 0.5" of section 10.2.
    * **R1 reads (c1).** Its learning laps took 150.8, 88.6 and 80.4 s, and it left on learning lap 4 at 3 973 m.
      * IPOPT solved, but the plan itself crossed the edge: the soft track slack reached 1.55 m, and the terminal
        slack grew lap by lap.
      * This is phases E's and I's failure near the limit. The slow start and the consecutive laps only delay it.
    * **R2 reads (c2), examined.** K 20 fails on learning lap 1 from phase E's start too. With K 10, the consecutive
      laps repeat phase E's (76.72 s, then off at 471 m).
    * **R3 reads (c2), examined.** OSQP hit its 20 000 iterations on 533 of 538 QPs from the first solve. Shifting the
      time-to-go does not help; ten times the iterations solves the first QP in 2.4 s.
      * This is an implementation limit (phase H's solver settings), unrelated to the learning.
      * So E.2's QP form remains untested under the published conditions.
    * **Also disclosed: the run-out approximation is poor after the slow start.**
      * The run-outs fall back to ~8 m/s.
      * Learning laps 2 and 3 braked on the straight before the line (to 35.0 and 61.4 m/s), slowing the next laps'
        start. This corrects packet 4's "crossed the line at speed".
      * The direct path to R1's failure is ruled out; the indirect one is not quantified.
      * So (b) against (c1) is open. The conclusion is unchanged: the same failure while approaching the limit.
    * The mechanism is an explanation consistent with the replay (a soft terminal hull without recursive feasibility
      once the car is faster than its data), not a tested cause.
  * **The user's decision (2026-11-01): the LMPC line is closed** – protocol section 11. Phase I ends without a
    benchmark: H-I1 and H-I2 are not tested, and no IV or IT lap was run. The open points go to the method matrix
    ("From phase I"), not scheduled. The benchmark document's phase I section records the negative result and its
    scope.
  * Three candidates in a fixed order, each with its own development gate: **I-1** a speed bound per lap on phase E's
    NLP (registered now); **I-2** E.2's own regression model with a trust region; **I-3** consecutive laps (I-2 and I-3
    by revision, if reached).
  * The first candidate to pass gets the benchmark, registered only after its NOM first-lap stage (review i3). If all
    three fail, the user decides.
* **Phase H:** done – the final review of 99732d5 **APPROVE**; `cx/H-atv-lmpc` merged to `main` (--no-ff) 2026-10-28 as
  98f8134. **Phase H was closed without the
  benchmark** (amendment H1): the development gate failed, and H-H1 and H-H2 are not tested; the negative result concerns
  the ATV-QP form of phase E's problem with OSQP and E.2's candidate on these cars (not cross-checked). The user chose the
  matrix's LMPC follow-ups next (phase I).
  History of phase H:
  H – the first frontier method (the user's choice, 2026-10-23): the ATV-QP learning MPC, phase
  E's first follow-up candidate; branch `cx/H-atv-lmpc` from `main` c157738 (the merge of phase G); literature §H.1–H.3
  and the protocol [CX_H_protocol.md](CX_H_protocol.md) revision 0 **APPROVE** (1a5c538) with clarifications m1–m3, n1, n2 (the gate judged on the registered
  development laps only, a failed gate closes the phase by default; the candidate's edges; attribution of H.1 / H.2; the
  linearisation error per lap; the reason for E's K grid). Code 3c69eb0 … 8b17829 (`ATVLearningMPC`,
  `ATVLMPCController`, `cx_e.learn`'s solver option, `experiments/cx_h.py`, `scripts/{run,report}_cx_h.py`,
  `scripts/cx_h_dev.py`, `tests/test_atv_lmpc.py`). **Development gate failed:** with the registered OSQP the QPs stop
  at the iteration limit and `atv` / `atv_nominal` leave the track on learning lap 1 on NOM and all five DEV cars
  (101–465 m); phase E's NLP reproduces exactly. A diagnostic (qpOASES and a box on the input step; not the registered
  design) still leaves on learning lap 1:
  * at turn 1 on four DEV cars, where E's NLP leaves too, with no QP failures (`nmpc` leaves three DEV cars at ~3.83 km,
    not at turn 1 – *corrected 2026-10-28 on the phase-I branch*);
  * at ~2.7 km on NOM and DEV_car1, with 17–18 QP failures.

  Packet 2 (d66cc2b) **APPROVE**, and **amendment H1 accepted** with i1–i3 (m1's default): phase H is closed without
  the benchmark; **H-H1 and H-H2 are not tested**.
  * The negative result concerns the ATV-QP form of E's problem with OSQP and E.2's candidate on these cars. It is not
    about LMPC as such, nor about E.2's own implementation.
  * e1 is weakened, not refuted.
  * Follow-ups are in the matrix. A later LMPC phase must first show on NOM that its first learning lap completes
    reliably.
  * n1: the development report has no hypothesis names (cfe57ad).
  * The benchmark section was written. No HV or HT lap was run. Final review of 99732d5 **APPROVE**; merged.
  Design: phase E's optimal control problem row for row, solved as one QP per step on the model linearised along E.2's
  candidate (the shifted solution, its terminal point from the previous λ's successors); phase E's development gate
  first; then H-H1 / H-H2 (`atv`'s tenth learning lap against `identified`, lap time and completions) on new test cars
  HT (seed 382); secondary: phase E's NLP as the formulation ablation, `atv_nominal`, lag and wet conditions.
* **Phase G:** done – packet 3 and the final review of 380124a **APPROVE**; `cx/G-realtime` merged to `main` (--no-ff)
  2026-10-23 as **c157738**. **The seven core phases A–G are complete.** Phase G result: **H-G1 holds** – a model-based prediction over the known sensor delay lets the RTI
  `lag_identified` complete 73 of 105 GT laps at a 20 ms delay against 5 for L9's RTI (not cross-checked); **H-G2 not
  shown** (72 vs 72 without a delay); with a wrong delay the compensation fails (3 of 105); on these simulated cars the
  identified model is nearly exact, so a real car's unmodelled dynamics are not measured.
  History of phase G:
  G – real-time control, branch `cx/G-realtime` from `main` 1922066 (the merge of phase F):
  literature §G.1–G.9 and the protocol [CX_G_protocol.md](CX_G_protocol.md) revision 0 **APPROVE** (277e122) with
  clarifications m1–m3, n1, n2 (H-G2 tests alignment against a later application; H-G1 on top of an undelayed
  identification; the repeat of every primary arm as the noise floor; the offline equivalence in the gate; the WSL
  environment is not the second environment). Code 2833681 … bdfa2a7 (the `comp` option of `experiments/l9`, `experiments/cx_g.py`,
  `scripts/{run,report}_cx_g.py`, `scripts/cx_g_dev.py`, `tests/test_cx_g.py`). **Development gate passed** (NOM + GD, WSL):
  the default path reproduces L9's offline records exactly; the offline equivalence is bit-identical; with the exact
  model the prediction error is 0; laps – `rti` 8 / 13 without and 1 / 13 with a 20 ms delay, `rti_dc` 9 / 13 and 8 / 13
  (development data); mean latency with the prediction 3.17 ms. Two code incidents on the way (an acados name, a
  summary function), kept and fixed. Packet 2 **APPROVE**. **GT** (WSL worktree `F1-Race-wt-CXG-run` at 5e11634,
  2026-10-23 04:00–05:39, 0 errors): **H-G1 holds** – at a 20 ms sensor delay the compensated RTI completes 73 of 105
  against 5 (68 : 0, p 6.8 · 10⁻²¹; not cross-checked); **H-G2 not shown** – 72 against 72 without a delay (7 : 7, p 1; the
  repeat flipped 0 of 20 in the d = 0 arms, so whether the discordant cars are noise is not known – review h1); at 40 ms 74 against 0, with a wrong delay (20 assumed) 3; latency with
  the prediction 3.2–3.5 ms. Packet 3 and the final review of 380124a **APPROVE** (h1: the H-G2 noise-floor wording
  corrected). Design: L9's real-time rule (WSL, acados SQP-RTI, one vCPU, 50 ms); `lag_identified` RTI
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
| G | `comp` option of `experiments/l9`, `experiments/cx_g.py`, `scripts/{run,report}_cx_g.py`, `scripts/cx_g_dev.py` | yes (2833681 … bdfa2a7) | yes – `tests/test_cx_g.py` 4 | development checks §9 on NOM and GD (WSL) – gate passed; GT (1 340 laps, WSL) | literature §G and protocol rev 0 **APPROVE** (packet 1, m1–m3 / n1, n2); packet 2 **APPROVE**; packet 3 and final review 380124a **APPROVE** (h1) | **yes – phase G completed (H-G1 holds; H-G2 not shown)** |
| H | `ATVLearningMPC`, `ATVLMPCController` (`control/lmpc.py`), `cx_e.learn` solver option, `experiments/cx_h.py`, `scripts/cx_h_dev.py`; `scripts/run_cx_h.py` (only `dev`) and `scripts/report_cx_h.py` (only `dev`) – the other stages never run | yes (3c69eb0 … 8b17829) | yes – `tests/test_atv_lmpc.py` 4 | development checks §9 on NOM and DEV – **gate failed**; a diagnostic variant | literature §H and protocol rev 0 **APPROVE** (packet 1, m1–m3 / n1, n2); packet 2 **APPROVE**, amendment H1 accepted (closed, no benchmark); final review 99732d5 **APPROVE** | **yes – phase H completed (closed without the benchmark)** |
| I | I-1: `LearningMPC(speed_bound=True)`, `LapStore.v_ref`, `LMPCController(eps)` (`control/lmpc.py`), `experiments/cx_i.py`, `scripts/{run,report}_cx_i.py`, `scripts/cx_i_dev.py`; revision 1: `cx_e.learn` options `consecutive`, `start` "centre", `on_lap`, `cx_i.REPRO` / `reading` | yes (62fc41d; b34eb97; diagnostics 418bf3a, bca7802, aae7f30) | yes – `tests/test_cx_i.py` 13 | I-1's development checks §9 on NOM and DEV – stage A passed, **stage B failed** (gate failed); the reproduction check §10.9 on NOM – R1 (c1), R2 and R3 (c2), examined | literature §I and protocol rev 0 **APPROVE** (packet 1, m1, m2 / n1, n2); packet 2 (I-1's results) **APPROVE** – I-1 closed; packet 3 (revision 1) **APPROVE** (m1–m3, n1–n3); packet 4 (the check's results) **APPROVE** (m1–m3, n1–n3); the user closed the LMPC line; final review f2e4263 **APPROVE** | **yes – phase I completed (closed without a benchmark; the LMPC line closed)** |
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
| 2026-10-23 | (this commit; code 2833681 … bdfa2a7) | phase G packet 2: code, tests, development checks – the gate passed | **APPROVE** | GT order and logging: the arms of a case in one batch (batch numbers in the records, batches.tsv), errors re-run by the last-record rule; the Reading to correct m1 (both arms act at 10 ms at d = 0) |
| 2026-10-23 | (this commit; run 5e11634) | phase G packet 3: GT – H-G1 holds, H-G2 not shown; S1–S4; the benchmark section – with the **final review** of `cx/G-realtime` | **APPROVE** (both) | h1: H-G2 is a symmetric 7 : 7, no difference shown; the repeat (0 of 20 flips at d = 0) does not estimate the noise there, so noise and cancelling per-car differences cannot be told apart – the wording corrected |
| 2026-10-23 | (this commit) | phase H packet 1: literature §H.1–H.3, protocol CX_H revision 0 (the ATV-QP learning MPC, the user's choice of the first frontier method); the phase-G merge hash c157738 recorded | **APPROVE** (2026-10-28) | m1 the gate only on the registered development laps, a failed gate closes the phase by default (as E1 option 1); m2 the stored laps' last points excluded from the safe set, a failed first QP applies the candidate's first input; m3 attribution of H.1 / H.2 in the docstring; n1 the linearisation error per lap with the outcome; n2 why the K grid stays E's |
| 2026-10-28 | d66cc2b (code 3c69eb0 … 8b17829) | phase H packet 2: code, tests, development checks – the gate failed; a diagnostic variant; amendment H1 (close, m1's default) | **APPROVE**; **H1 accepted** (closed without the benchmark) | i1 the scope (ATV-QP of E's problem, OSQP, E.2's candidate – not LMPC, not E.2's own implementation); i2 e1 weakened, not refuted, the η 0.95 lap an observation; i3 follow-ups in order and the NOM first-lap precondition; n1 no hypothesis names in the dev report (cfe57ad); n2 the never-run stages marked |
| 2026-10-28 | 99732d5 (cfe57ad …) | phase H: the decision (i1–i3, n1) in the protocol, the benchmark section, the matrix follow-ups; the diagnostic's QP failure counts added – **final review** of `cx/H-atv-lmpc` | **APPROVE** | the disclosed correction verified (qpOASES failures 18 / 1003 on NOM, 17 / 967 on DEV_car1, 0 elsewhere); n1 checked against `report_before_n1/`; STATUS and this file to say phase H is closed without the benchmark at the merge |
| 2026-10-28 | (this commit) | phase I packet 1: literature §I.1–I.4, protocol CX_I revision 0 (the LMPC follow-ups in a fixed order – I-1 a speed bound per lap on phase E's NLP; I-2, I-3 by revision); the phase-H merge hash 98f8134 recorded; corrections to phase H's (and E's) text on where `nmpc` leaves the DEV cars | **APPROVE** (2026-10-28) | m1 the benchmark registered only after stages A and B pass (A passing and B failing is a failed gate); m2 the first learning lap's bound (~1.02–1.08 × the η_p 0.79 lap) and its expected lap time, v_ref past the line from the run-out records; n1 ε the tuned parameter, K 20; n2 `pb_nominal` descriptive in stage B |

| 2026-10-29 | 437ee9e (code 62fc41d) | phase I packet 2: I-1's code, tests, development checks – stage A passed, stage B failed (the gate failed); I-1 closed for the phase (rule 4) | **APPROVE** | records recounted (26 jobs) and E's path checked; observation 2 kept as an untested explanation; the I-2 / I-3 revisions must first state how they address the failure near the limit on NOM with an exact model – to the user as written |
| 2026-10-29 | 1e2bf96 | phase I packet 3: protocol revision 1, section 10 – a reproduction check under the published LMPCs' conditions on NOM (the user's request after the discussion; a diagnostic); literature §I.5 (E.2 re-read for its lap set-up) | **APPROVE** | v_c = 8 m/s re-computed (max\|κ\| a real ~14 m hairpin peak; q99.9 gives 8 too); m1 reading (c) split into (c1) approaching the limit (best ≤ 84.04 s) / (c2) far from it; m2 the initialisation laps consecutive, R2's learning lap 1 from the 0.79 lap's end, the init controller's settings; m3 the run-out approximation checked per lap, the flying start as phase E's; n1 w_T 100 unscaled; n2 failure classes after solver failures / solved; n3 no wall-clock limit – written in as clarifications before the code |
| 2026-10-30 | ff50e41 (run b34eb97; diagnostics 418bf3a, bca7802, aae7f30) | phase I packet 4: the reproduction check's results (section 10.9) – r0–r2; R1 (c1), R2 and R3 (c2) examined (R2's neighbours, R3's and R1's failed laps driven again from snapshots) | **APPROVE** | m1 R3's (c2) is an implementation limit (phase H's solver settings), E.2's QP form untested – the heading corrected; m2 the mechanism an explanation consistent with the replay, the conclusion qualified (NOM, one run per arm, this implementation); m3 the run-out's effect on the reading – direct path ruled out, an indirect path via the start speed found while checking it (packet 4's "crossed the line at speed" corrected), (b) against (c1) open, conclusion unchanged; n1 IPOPT's statuses separately; n2 the learning speed against E.2 in section 10.6; n3 for the user: a real test of E.2's QP form or of the papers' past-line data each needs a revision |
| 2026-11-01 | f2e4263 | phase I: the user's decision (the LMPC line closed) in the protocol (section 11), the benchmark section of phase I, the matrix's open points – **final review** of `cx/I-lmpc-followups` | **APPROVE** | 13 commits from 98f8134, a trial merge without conflicts; pytest summaries and the branch tree (no results / data, no CR) checked; "every NOM LMPC sequence of phases E–I left by its fifth learning lap" re-checked over 19 sequences; the faster-than-`nmpc` laps only in phase E's unregistered diagnostics (as CX_E §9); nit: "The sequences' fastest laps" – done before the merge |
| 2026-11-24 | 2187b53 | phase J packet 1: literature §J and protocol CX_J revision 0 | **CHANGES_REQUESTED** | M1 the L2 box is not centred on the nominal car, so θ̂ ± h is not C's box (61 % of the L2 cars would start outside it); M2 the per-stage residual inflates the tube to its caps under noise, and the grid had no small values; m1 the 2 × 2 ablation; m2 the gate's reference may be uninformative; m3 H-J2 by car; m4 the cold-solve check; m5 NOM's seeds, the replay's conditions; n1, n2. Worked into the second version |
| 2026-11-24 | (reviewer's correction of the phase-I reviews) | packets 4 and final review, phase I | – | *As stated by the reviewer:* the code changes from b34eb97 on had not been read at packet 4 and the final review (records and tests only); they have now read the changes of `cx_e`, `cx_i` and `run_cx_i.py` against 437ee9e and found no defect, but not `cx_i_dev.py`, `report_cx_i.py` and `tests/test_cx_i.py`. r0 was checked only by `all_equal` then; its record and now were compared since: equal. The count of curvature points above 0.08 (packet 3) and the remark that the builder's correction was recorded are unverified / not true. **Phase I's conclusion is unchanged; its reviews do not cover these three files.** |
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

Phase G: `results/CX/G/dev/` (main checkout, WSL) – the development checks of CX-G §9 (`regress.json`, `explore.jsonl`,
`plans.jsonl`, `eval.jsonl`, `equivalence.json`, `predictor.json`, `gate.json`, `load.jsonl`, `meta_*.json`,
`RUN_COMMIT_dev*`, `attempt1_failed/`, `report/`). GT: worktree `F1-Race-worktrees/F1-Race-wt-CXG-run` at 5e11634,
`results/CX/G/test/` (`explore.jsonl`, `plans.jsonl`, `eval.jsonl` 1c5f7884…, `repeat_eval.jsonl`, `rl.jsonl`,
`load.jsonl`, `batches.tsv`, `explore/`, `plans/`, `report/CX_G.{md,json}`, `RUN_COMMIT_test`, `SHA256_*.txt`,
`timeline.tsv`, `cxg_run.sh`, `log_*.txt`). All git-ignored.

Phase F: `results/CX/F/dev/` (main checkout) – the development checks of CX-F §9 (`monitor.json`, `laps.jsonl`,
`gate.json`, `backup_plans.jsonl`, `monitor_variants.json`, `variants.jsonl`, `meta_*.json`), decision records
`results/CX/F/dev_dec/`, `devvar_dec/`; the audit of L7's records `results/CX/F/report/audit_l7.*` (b94e775). T1 + T2:
worktree `F1-Race-worktrees/F1-Race-wt-CXF-run` at e0f0b0a, `results/CX/F/` (`test.jsonl` 248c584b…, `test_dec/`,
`report/CX_F.{md,json}`, `RUN_COMMIT_test`, `SHA256_inputs_{run,main_checkout}.txt`, `timeline.tsv`, `log_test.txt`,
`log_report.txt`, `launch_test.sh`). All git-ignored.

Phase I: `results/CX/I/` (main checkout). The records:
* `dev/` – I-1's development checks of CX-I §9, at 62fc41d (`laps.jsonl`, `nlp_repro.json`, `meta_*.json`,
  `report/`);
* `smoke/` and `repro_smoke/` – the disclosed scratch smokes;
* `repro/` – the reproduction check of CX-I §10, at b34eb97:
  * `laps.jsonl`, `progress_R*.jsonl`, `r0.json`, `r2.json`, `report/`;
  * the diagnostics: `r2_usage.*` (418bf3a), `r2_diag.json` and `snap/` (bca7802), `replay_*.json` (aae7f30);
* `identified_cache/`;
* `final/` – the pytest logs.

All git-ignored.

Phase H: `results/CX/H/dev/` (main checkout) – the development checks of CX-H §9 (`qp.json`, `nlp_repro.json`,
`laps.jsonl`, `diag.jsonl`, `meta_*.json`, `RUN_COMMIT_dev`, `diag_attempt1_failed/`, `report/` regenerated at cfe57ad
with `REPORT_COMMIT`, `report_before_n1/`), `results/CX/H/identified_cache/`, the final test logs `results/CX/H/final/`.
Git-ignored.

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

1. Phase J: review of the protocol revision 0, second version (packet 1b). Code only after its APPROVE.
   * The LMPC line is closed (CX_I section 11). Its open points are listed in the matrix ("From phase I") and are not
     scheduled.
