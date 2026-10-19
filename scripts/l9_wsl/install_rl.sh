#!/bin/bash
# L9: torch 2.14.1+cpu from the PyTorch CPU index and the RL extras from PyPI into the WSL venv
# (user's choice, 2026-10-18); every package pinned, no dependency resolution beyond the pins.
set -e
V="$HOME/f1race-venv"
W=/tmp/l9_torch_wheel
mkdir -p "$W"
"$V/bin/pip" download --no-deps -d "$W" "torch==2.14.1+cpu" --index-url https://download.pytorch.org/whl/cpu
sha256sum "$W"/torch-*.whl
"$V/bin/pip" install --no-deps "$W"/torch-*.whl
"$V/bin/pip" install --no-deps -r "/mnt/c/Users/yuanche/F1 Race/scripts/l9_wsl/requirements-rl-wsl.txt"
"$V/bin/pip" check
"$V/bin/python" -c "import torch, stable_baselines3, gymnasium; print('torch', torch.__version__, 'threads', torch.get_num_threads(), '| sb3', stable_baselines3.__version__, '| gymnasium', gymnasium.__version__)"
