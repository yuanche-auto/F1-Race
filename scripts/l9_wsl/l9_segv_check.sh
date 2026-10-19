#!/bin/bash
# Reproduce the crashed real-time RL job (exit -11) of step 2b outside the run directory, with stderr and
# faulthandler; nothing else runs. Usage: bash l9_segv_check.sh [repeats, default 5]
source ~/f1race-env.sh
cd "/mnt/c/Users/yuanche/F1-Race-worktrees/F1-Race-wt-L9-run" || exit 1
D=results/L9/debug_segv
mkdir -p $D
echo "== kernel log"
dmesg 2>&1 | grep -iE "segfault|python" | tail -n 5
SPEC='{"mode": "realtime", "delay": 0.0, "explore_mode": "realtime", "rep": 0, "plan_dir": null, "hold": "plan", "stage": "rl", "kind": "B_s3", "impl": "policy", "case": "T2new_dist6024"}'
for i in $(seq 1 ${1:-5}); do
  taskset -c 0 python -X faulthandler scripts/run_l9.py job --spec "$SPEC" --dir $D > $D/out_$i.txt 2> $D/err_$i.txt
  rc=$?
  echo "run $i: exit $rc | error record: $(grep -c '"error"' $D/out_$i.txt) | stderr lines $(wc -l < $D/err_$i.txt)"
done
echo "== stderr of the first failing run"
for i in $(seq 1 ${1:-5}); do [ -s $D/err_$i.txt ] && { tail -n 40 $D/err_$i.txt; break; }; done
