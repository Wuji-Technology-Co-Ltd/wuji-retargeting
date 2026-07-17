#!/usr/bin/env bash
set -euo pipefail

CONDA_ROOT="${CONDA_ROOT:-/home/zxc/miniconda3}"
CONDA_ENV_NAME="${WUJI_TELEOP_CONDA_ENV:-wuji-ros2}"

if [[ ! -f "${CONDA_ROOT}/etc/profile.d/conda.sh" ]]; then
  echo "Conda activation script not found: ${CONDA_ROOT}/etc/profile.d/conda.sh" >&2
  exit 1
fi

source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV_NAME}"

# Keep this runtime independent from packages accidentally installed in
# ~/.local. The launcher below applies the same isolation.
export PYTHONNOUSERSITE=1

python -m pip install \
  "numpy>=1.21,<2" \
  "scipy>=1.11,<2" \
  "nlopt==2.10.0" \
  "pin==3.8.0" \
  "PyYAML==6.0.3" \
  "websockets==16.0" \
  "colorama==0.4.6" \
  "h5py>=3.8,<4" \
  "filterpy==1.4.5" \
  "transforms3d==0.4.2"

echo "Unified teleop dependencies are installed in ${CONDA_PREFIX}."
echo "Run commands through scripts/run_in_unified_teleop_env.sh."
