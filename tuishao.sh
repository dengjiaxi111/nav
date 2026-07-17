#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TUISHAO_WS="${ROOT_DIR}/tuishao_ws"
BAG_DIR="${ROOT_DIR}/bags_tuishao"
LIDAR_TOPIC="/livox/lidar_192_168_1_199"
IMU_TOPIC="/livox/imu_192_168_1_199"
DEFAULT_ACADOS_SOURCE_DIR="/home/dengjiaxi/dependency/acados"

SERIAL_PID=""
NAV_PID=""
BAG_PID=""

export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-${DEFAULT_ACADOS_SOURCE_DIR}}"
if [[ -d "${ACADOS_SOURCE_DIR}/lib" ]]; then
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
fi

build_tuishao_ws() {
  colcon build --symlink-install \
    --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
}

cleanup() {
  local exit_code=$?
  trap - INT TERM EXIT

  echo
  echo "[tuishao.sh] stopping..."

  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi

  if [[ -n "${NAV_PID}" ]] && kill -0 "${NAV_PID}" 2>/dev/null; then
    kill -INT "${NAV_PID}" 2>/dev/null || true
    wait "${NAV_PID}" 2>/dev/null || true
  fi

  if [[ -n "${SERIAL_PID}" ]] && kill -0 "${SERIAL_PID}" 2>/dev/null; then
    kill -INT "${SERIAL_PID}" 2>/dev/null || true
    wait "${SERIAL_PID}" 2>/dev/null || true
  fi

  echo "[tuishao.sh] stopped."
  exit "${exit_code}"
}

usage() {
  cat <<'EOF'
Usage:
  ./tuishao.sh           Run the legged-robot workspace.
  ./tuishao.sh --build   Build tuishao_ws first, then run it.
  ./tuishao.sh --build-only
                         Build tuishao_ws and exit.

This script does not use or modify start.sh. start.sh remains the wheel-robot entrypoint.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! -d "${TUISHAO_WS}/src" ]]; then
  echo "[tuishao.sh] missing copied workspace: ${TUISHAO_WS}"
  exit 1
fi

cd "${TUISHAO_WS}"

if [[ "${1:-}" == "--build" ]]; then
  echo "[tuishao.sh] building tuishao workspace..."
  build_tuishao_ws
elif [[ "${1:-}" == "--build-only" ]]; then
  echo "[tuishao.sh] building tuishao workspace only..."
  build_tuishao_ws
  exit 0
elif [[ -n "${1:-}" ]]; then
  usage
  exit 1
fi

if [[ ! -f "${TUISHAO_WS}/install/setup.bash" ]]; then
  echo "[tuishao.sh] missing ${TUISHAO_WS}/install/setup.bash"
  echo "[tuishao.sh] build first with:"
  echo "  ${ROOT_DIR}/tuishao.sh --build-only"
  echo "or run:"
  echo "  ${ROOT_DIR}/tuishao.sh --build"
  exit 1
fi

trap cleanup INT TERM EXIT

set +u
source "${TUISHAO_WS}/install/setup.bash"
set -u

echo "Record lidar and IMU bag?"
echo "  0) No"
echo "  1) Yes: ${LIDAR_TOPIC} ${IMU_TOPIC}"
read -r -p "Select [0/1, default 0]: " RECORD_CHOICE
RECORD_CHOICE="${RECORD_CHOICE:-0}"

echo "[tuishao.sh] launching tuishao serial..."
ros2 launch myserial myserial.launch.py &
SERIAL_PID=$!

sleep 1

echo "[tuishao.sh] launching tuishao navigation, lidar_mount_mode=fixed..."
ros2 launch nav_bringup run.launch.py lidar_mount_mode:=fixed &
NAV_PID=$!

case "${RECORD_CHOICE}" in
  1)
    mkdir -p "${BAG_DIR}"
    BAG_PATH="${BAG_DIR}/tuishao_$(date +%Y%m%d_%H%M%S)"
    echo "[tuishao.sh] recording lidar and IMU to ${BAG_PATH}"
    ros2 bag record "${LIDAR_TOPIC}" "${IMU_TOPIC}" -o "${BAG_PATH}" &
    BAG_PID=$!
    ;;
  0)
    echo "[tuishao.sh] skip bag recording."
    ;;
  *)
    echo "[tuishao.sh] unknown option '${RECORD_CHOICE}', skip bag recording."
    ;;
esac

echo "[tuishao.sh] running. Press Ctrl+C to stop all processes."
wait -n "${SERIAL_PID}" "${NAV_PID}" ${BAG_PID:+"${BAG_PID}"}
