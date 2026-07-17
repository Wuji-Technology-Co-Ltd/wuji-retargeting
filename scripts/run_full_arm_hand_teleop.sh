#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_in_unified_teleop_env.sh" \
  python -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_full_control_pc_stack \
  "$@"
