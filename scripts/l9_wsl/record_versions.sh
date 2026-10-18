#!/bin/bash
# L9 (l9-m3): the exact commits of acados and its submodules as built in WSL, and the SHA-256 of the
# downloaded binaries / wheels (the pip wheel re-downloaded from PyPI and checked against the hash
# PyPI publishes); writes scripts/l9_wsl/acados_versions.txt.
OUT="/mnt/c/Users/yuanche/F1 Race/scripts/l9_wsl/acados_versions.txt"
WHL=/tmp/pip-25.2-py3-none-any.whl
[ -f "$WHL" ] || curl -sSL -o "$WHL" https://files.pythonhosted.org/packages/py3/p/pip/pip-25.2-py3-none-any.whl
PUBLISHED=$(curl -sS https://pypi.org/pypi/pip/25.2/json | python3 -c "import json,sys; print([u['digests']['sha256'] for u in json.load(sys.stdin)['urls'] if u['filename'].endswith('py3-none-any.whl')][0])")
{
  echo "# acados as built in WSL (Ubuntu 24.04) on 2026-10-18 – tag v0.6.0"
  echo "acados $(git -C "$HOME/acados" rev-parse HEAD)"
  echo "# submodules (git submodule status)"
  git -C "$HOME/acados" submodule status
  echo "# binaries and wheels (sha256sum)"
  sha256sum "$HOME/acados/bin/t_renderer" | sed "s|$HOME|~|"
  sha256sum "$WHL"
  echo "# pip 25.2 wheel, SHA-256 published by PyPI: $PUBLISHED"
  echo "# python in the venv"
  "$HOME/f1race-venv/bin/python" --version
  "$HOME/f1race-venv/bin/pip" --version | cut -d" " -f1,2
} > "$OUT"
cat "$OUT"
