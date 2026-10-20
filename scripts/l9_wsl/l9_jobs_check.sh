#!/bin/bash
# Job counts of every L9 step at the pinned worktree (no laps run).
source ~/f1race-env.sh
cd "/mnt/c/Users/yuanche/F1-Race-worktrees/F1-Race-wt-L9-run" || exit 1
env | grep -E "THREADS|L6_"
python -c 'import sys; sys.path.insert(0, "src"); import f1race; print(f1race.__file__)'
J() { echo "## $*"; python scripts/run_l9.py jobs "$@" | cut -c1-60; }
J --mode offline
J --part rl --mode offline
J --mode realtime --dc
J --part rl --mode realtime
J --mode realtime --dc --repeat 20
J --mode realtime --kinds lag_identified --delay 0.02
J --part rl --mode realtime --policies B_s1_500k,tracker --delay 0.02
J --mode realtime --kinds lag_identified --explore-mode offline --plan-dir results/L9/offline
J --mode realtime --hold last
J --part student --mode offline
