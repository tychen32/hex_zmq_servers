#!/usr/bin/env bash
set -Eeuo pipefail
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-05-26
################################################################

CUR_DIR="$(pwd)"
SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "CUR_DIR: $CUR_DIR"
echo "SCRIPT_DIR: $SCRIPT_DIR"

cd $SCRIPT_DIR

if ! command -v uv >/dev/null 2>&1; then
  echo "Error: uv not found. Please install uv first." >&2
  exit 1
fi

if [ ! -d .venv ]; then
  uv venv --python 3.11
fi
source .venv/bin/activate

# Install hex_zmq_servers
rm -rf dist build *.egg-info
uv pip uninstall hex_zmq_servers || true
uv pip install -e .

# install requirements for advanced examples
uv pip install -r examples/adv/requirements.txt

cd $CUR_DIR