#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${WORKSPACE_DIR}/bags"
LIDAR_TOPIC="/livox/lidar_192_168_1_195"
IMU_TOPIC="/livox/imu_192_168_1_195"

SERIAL_PID=""
NAV_PID=""
BAG_PID=""

cleanup() {
  local exit_code=$?
  trap - INT TERM EXIT

  echo
  echo "[start.sh] stopping..."

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

  echo "[start.sh] stopped."
  exit "${exit_code}"
}

trap cleanup INT TERM EXIT

cd "${WORKSPACE_DIR}"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "[start.sh] missing install/setup.bash, please build first: ./build.sh"
  exit 1
fi

set +u
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "请选择是否录包："
echo "  0) 不录包"
echo "  1) 只录雷达和 IMU: ${LIDAR_TOPIC} ${IMU_TOPIC}"
read -r -p "请输入选项 [0/1，默认 0]: " RECORD_CHOICE
RECORD_CHOICE="${RECORD_CHOICE:-0}"

echo "[start.sh] launching serial..."
ros2 launch myserial myserial.launch.py &
SERIAL_PID=$!

sleep 1

echo "[start.sh] launching navigation..."
ros2 launch mybringup run_nav.launch.py &
NAV_PID=$!

case "${RECORD_CHOICE}" in
  1)
    mkdir -p "${BAG_DIR}"
    BAG_PATH="${BAG_DIR}/nav_$(date +%Y%m%d_%H%M%S)"
    echo "[start.sh] recording lidar and IMU to ${BAG_PATH}"
    ros2 bag record "${LIDAR_TOPIC}" "${IMU_TOPIC}" -o "${BAG_PATH}" &
    BAG_PID=$!
    ;;
  0)
    echo "[start.sh] skip bag recording."
    ;;
  *)
    echo "[start.sh] unknown option '${RECORD_CHOICE}', skip bag recording."
    ;;
esac

echo "[start.sh] running. Press Ctrl+C to stop all processes."
wait -n "${SERIAL_PID}" "${NAV_PID}" ${BAG_PID:+"${BAG_PID}"}
