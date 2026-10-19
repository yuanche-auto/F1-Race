#!/bin/bash
# L9 runs at RUN_COMMIT in the pinned worktree (docs/L9_protocol.md section 11; the reviewer's order):
# 1 offline (model controllers, RL) -> 2 real-time main (--dc) and RL -> 3 repeat, delay, offline_explore,
# hold_last -> 4 Part C (train 3 students, then offline, then real-time). One step at a time, never two;
# stops at the first error record. Usage: bash l9_run.sh [first step id, default 1a]
source ~/f1race-env.sh
W="/mnt/c/Users/yuanche/F1-Race-worktrees/F1-Race-wt-L9-run"
cd "$W" || exit 1
R=results/L9
T=$R/timeline.tsv
FROM=${1:-1a}
log() { printf '%s\t%s\n' "$(date -Iseconds)" "$*" >> "$T"; }

errors() {   # jobs whose last record has a top-level "error" key (load.jsonl excluded; a job re-run after an
             # error appends a new record, the error record stays in the file)
  python - "$R/$1" <<'EOF'
import json, pathlib, sys
n = 0
for f in pathlib.Path(sys.argv[1]).glob("*.jsonl"):
    if f.name != "load.jsonl":
        last = {}
        for l in f.read_text().splitlines():
            if l.strip():
                r = json.loads(l)
                last[r["job"]] = r
        n += sum("error" in r for r in last.values())
print(n)
EOF
}

progress() { # job completion times every 10 s (maps the once-a-minute load samples to batches)
  while [ -f "$R/.running" ]; do
    line="$(date -Iseconds)"
    for f in "$R"/*/*.jsonl; do [ -f "$f" ] && line="$line	${f#$R/}=$(wc -l < "$f")"; done
    echo "$line" >> "$R/progress.tsv"
    sleep 10
  done
}

ACTIVE=0
step() {     # id dir args...
  local id=$1 dir=$2; shift 2
  [ "$id" = "$FROM" ] && ACTIVE=1
  [ $ACTIVE = 1 ] || return 0
  log "start $id ($dir): run_l9.py $*"
  python scripts/run_l9.py "$@" > "$R/log_$id.txt" 2>&1
  local rc=$? n
  n=$(errors "$dir")
  log "end $id rc=$rc errors=$n"
  if [ $rc -ne 0 ] || [ "$n" != "0" ]; then log "STOP at $id"; rm -f "$R/.running"; exit 1; fi
}

train() {    # Part C: three students at once, each on its own vCPU (untimed; nothing timed runs)
  [ "4a" = "$FROM" ] && ACTIVE=1
  [ $ACTIVE = 1 ] || return 0
  log "start 4a: train-student --seed 1..3"
  for s in 1 2 3; do taskset -c $((s - 1)) python scripts/run_l9.py train-student --seed $s > "$R/log_4a_s$s.txt" 2>&1 & done
  wait
  local ok=1
  for s in 1 2 3; do [ -f "$R/partC/student_s$s.pt" ] || ok=0; done
  log "end 4a students_ok=$ok"
  if [ $ok != 1 ]; then log "STOP at 4a"; rm -f "$R/.running"; exit 1; fi
}

[ -f "$R/RUN_COMMIT" ] || exit 1             # HEAD = RUN_COMMIT and a clean tree are checked on Windows first
touch "$R/.running"
progress &
log "session start from $FROM; RUN_COMMIT $(cat "$R/RUN_COMMIT"); $(python -c 'import sys; sys.path.insert(0, "src"); import f1race; print(f1race.__file__)')"

step 1a offline          run --mode offline --slots 20
step 1b offline          run --part rl --mode offline --slots 20
step 2a realtime         run --mode realtime --dc --slots 8
step 2b realtime         run --part rl --mode realtime --slots 8
step 3a repeat           run --mode realtime --dc --repeat 20 --tag repeat --slots 8
step 3b delay            run --mode realtime --kinds lag_identified --delay 0.02 --tag delay --slots 8
step 3c delay            run --part rl --mode realtime --policies B_s1_500k,tracker --delay 0.02 --tag delay --slots 8
step 3d offline_explore  run --mode realtime --kinds lag_identified --explore-mode offline --plan-dir results/L9/offline --tag offline_explore --slots 8
step 3e hold_last        run --mode realtime --hold last --tag hold_last --slots 8
train
step 4b offline          run --part student --mode offline --slots 20
step 4c realtime         run --part student --mode realtime --slots 8

log "all steps done"
rm -f "$R/.running"
wait
