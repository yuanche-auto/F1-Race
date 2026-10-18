#!/bin/bash
# L9: the WSL (Ubuntu 24.04) Python environment for acados (user's approval 2026-10-18):
# a pip-less venv, pip from its PyPI wheel, the project lock file, acados_template (editable).
set -e
V="$HOME/f1race-venv"
LOCK="/mnt/c/Users/yuanche/F1 Race/requirements-lock.txt"
"$V/bin/python" /tmp/pip-25.2-py3-none-any.whl/pip install -q /tmp/pip-25.2-py3-none-any.whl
"$V/bin/pip" --version
"$V/bin/pip" install -q -r "$LOCK"
"$V/bin/pip" install -q -e "$HOME/acados/interfaces/acados_template"
cat > "$HOME/f1race-env.sh" <<'EOF'
# source this before running L9 code in WSL
export ACADOS_SOURCE_DIR="$HOME/acados"
export LD_LIBRARY_PATH="$HOME/acados/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export L6_REPLAN_SLOTS=3      # one re-plan needs ~7.4 GB; WSL has 31 GB
source "$HOME/f1race-venv/bin/activate"
EOF
"$V/bin/pip" freeze
