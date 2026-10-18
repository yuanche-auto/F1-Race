#!/bin/bash
# L9: acados in WSL (Ubuntu 24.04), as built on 2026-10-18 with the user's approval.
# acados v0.6.0 (released 2026-08-06) with its submodules, built with qpOASES and DAQP and installed
# into the source tree (no sudo); tera_renderer v0.2.1 (released 2026-07-31), the version
# acados_template 0.6.0 expects (TERA_DEFAULT_VERSION).
set -e
cd "$HOME"
git clone --branch v0.6.0 --depth 1 --recurse-submodules --shallow-submodules https://github.com/acados/acados.git
cd acados && mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON ..
make install -j16
mkdir -p "$HOME/acados/bin" && cd "$HOME/acados/bin"
curl -sSL -o t_renderer https://github.com/acados/tera_renderer/releases/download/v0.2.1/t_renderer-v0.2.1-linux-amd64
echo "64a0a0f8d85be0b92234231fd1a9ff50c9d8f13d6208063707e56c9354a976d3  t_renderer" | sha256sum -c
chmod +x t_renderer
# Python: python3-venv is not installed (apt needs sudo), so a pip-less venv and pip from its PyPI wheel:
python3 -m venv --without-pip "$HOME/f1race-venv"
curl -sSL -o /tmp/pip-25.2-py3-none-any.whl https://files.pythonhosted.org/packages/py3/p/pip/pip-25.2-py3-none-any.whl
echo "6d67a2b4e7f14d8b31b8b52648866fa717f45a1eb70e83002f4331d07e953717  /tmp/pip-25.2-py3-none-any.whl" | sha256sum -c
# the commits actually built: scripts/l9_wsl/acados_versions.txt (scripts/l9_wsl/record_versions.sh; a tag can move)
# then: bash scripts/l9_wsl/setup_venv.sh (from the repository, as /mnt/c/...)
