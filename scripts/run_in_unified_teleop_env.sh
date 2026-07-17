#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONDA_ROOT="${CONDA_ROOT:-/home/zxc/miniconda3}"
CONDA_ENV_NAME="${WUJI_TELEOP_CONDA_ENV:-wuji-ros2}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
ASTRIBOT_SDK_ROOT="${ASTRIBOT_SDK_ROOT:-/home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master}"
WUJIHAND_ROS2_SETUP="${WUJIHAND_ROS2_SETUP:-/home/zxc/Desktop/wuji/wuji-teleop/wujihandros2/install/setup.bash}"

if [[ $# -eq 0 ]]; then
  echo "Usage: $0 COMMAND [ARG ...]" >&2
  exit 2
fi
if [[ ! -f "${CONDA_ROOT}/etc/profile.d/conda.sh" ]]; then
  echo "Conda activation script not found: ${CONDA_ROOT}/etc/profile.d/conda.sh" >&2
  exit 1
fi
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS2 setup not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${ASTRIBOT_SDK_ROOT}/env.sh" ]]; then
  echo "Astribot SDK environment not found: ${ASTRIBOT_SDK_ROOT}/env.sh" >&2
  exit 1
fi

source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda activate "${CONDA_ENV_NAME}"
export PYTHONNOUSERSITE=1

# ROS/colcon setup scripts read optional variables that may be unset.
set +u
source "${ROS_SETUP}"
source "${ASTRIBOT_SDK_ROOT}/env.sh"
if [[ -f "${WUJIHAND_ROS2_SETUP}" ]]; then
  source "${WUJIHAND_ROS2_SETUP}"
fi
set -u

# Astribot prepends its Pinocchio 3.7 Python bindings. Those bindings need
# eigenpy/urdfdom shared libraries supplied by the retargeting environment.
# Prepending only cmeel's library directory preserves Astribot's Python/API
# version while allowing the real Wuji Retargeter to load in the same process.
CMEEL_PREFIX="$(python - <<'PY'
import sysconfig
from pathlib import Path

print(Path(sysconfig.get_paths()["purelib"]) / "cmeel.prefix")
PY
)"
if [[ ! -d "${CMEEL_PREFIX}/lib" ]]; then
  echo "cmeel runtime not found under ${CMEEL_PREFIX}; run bootstrap_unified_teleop_env.sh" >&2
  exit 1
fi
export LD_LIBRARY_PATH="${CMEEL_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

cd "${REPO_ROOT}"
exec "$@"
