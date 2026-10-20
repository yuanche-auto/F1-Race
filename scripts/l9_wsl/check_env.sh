#!/bin/bash
# L9: checks of the WSL environment – an acados example (code generation, compilation, solve) and
# the project's tests (RL tests skip without torch / gymnasium).
source "$HOME/f1race-env.sh"
cd "$HOME/acados/examples/acados_python/getting_started" || exit 1
mkdir -p /tmp/acados_check && cd /tmp/acados_check
cp "$HOME/acados/examples/acados_python/getting_started/"*.py . 2>/dev/null
cp -r "$HOME/acados/examples/acados_python/pendulum_on_cart/common" . 2>/dev/null
timeout 600 python minimal_example_ocp.py 2>&1 | tail -n 5
echo "acados example exit: $?"
cd "/mnt/c/Users/yuanche/F1 Race" && timeout 1800 python -m pytest -q -p no:cacheprovider 2>&1 | tail -n 4
