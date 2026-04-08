#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  bash src/tools/tau_v_fit/run_tau_v_fit.sh [duration_s] [out_dir] [prefix] [topic]

Examples:
  bash src/tools/tau_v_fit/run_tau_v_fit.sh 90
  bash src/tools/tau_v_fit/run_tau_v_fit.sh 120 log/tau_v_fit ramp_case /nmpc/speed_observation
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

DURATION="${1:-60}"
OUT_DIR="${2:-log/tau_v_fit}"
PREFIX="${3:-tau_v_test}"
TOPIC="${4:-/nmpc/speed_observation}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[tau_v_fit] ros2 command not found. Please source ROS environment first."
  exit 2
fi

mkdir -p "${OUT_DIR}"
mkdir -p "${OUT_DIR}/ros_logs"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY_SCRIPT="${SCRIPT_DIR}/tau_v_logger_fit.py"

if [ ! -f "${PY_SCRIPT}" ]; then
  echo "[tau_v_fit] Missing script: ${PY_SCRIPT}"
  exit 2
fi

PY_BIN=""
if /usr/bin/python3 -c "import rclpy" >/dev/null 2>&1; then
  PY_BIN="/usr/bin/python3"
elif python3 -c "import rclpy" >/dev/null 2>&1; then
  PY_BIN="python3"
else
  echo "[tau_v_fit] Cannot import rclpy."
  echo "[tau_v_fit] Please run after 'source /opt/ros/humble/setup.bash' and avoid incompatible conda python."
  exit 2
fi

echo "[tau_v_fit] python=${PY_BIN} topic=${TOPIC} duration=${DURATION}s out_dir=${OUT_DIR} prefix=${PREFIX}"
"${PY_BIN}" "${PY_SCRIPT}" \
  --topic "${TOPIC}" \
  --duration "${DURATION}" \
  --out-dir "${OUT_DIR}" \
  --prefix "${PREFIX}"
