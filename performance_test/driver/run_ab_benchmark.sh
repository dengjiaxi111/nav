#!/usr/bin/env bash
set -euo pipefail

# Automated A/B benchmark for livox_ros_driver2
# Collects:
#   1) latency csv exported by driver (enable_timestamp_logging must be true)
#   2) ros2 topic hz / bw
#   3) pidstat CPU stats
# Then generates per-round metrics and an A/B summary.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${SCRIPT_DIR}/ab_results_${TIMESTAMP}"
mkdir -p "${OUT_DIR}"

BASELINE_SETUP=""
OPT_SETUP=""
BASELINE_CMD=""
OPT_CMD=""
TOPIC="/livox/lidar"
LATENCY_FILE="/tmp/livox_timestamp.csv"
ROUNDS=10
WARMUP_SEC=30
MEASURE_SEC=180
STARTUP_TIMEOUT_SEC=40
IDLE_AFTER_STOP_SEC=2

print_help() {
  cat <<EOF
Usage:
  $(basename "$0") \
    --baseline-setup /path/to/baseline/install/setup.bash \
    --opt-setup /path/to/opt/install/setup.bash \
    --baseline-cmd "ros2 launch livox_ros_driver2 msg_MID360_launch.py" \
    --opt-cmd "ros2 launch livox_ros_driver2 msg_MID360_launch.py" \
    [--topic /livox/lidar] \
    [--latency-file /tmp/livox_timestamp.csv] \
    [--rounds 10] [--warmup 30] [--measure 180]

Required precondition:
  - Launch command must enable timestamp logging in driver:
      enable_timestamp_logging:=true
      timestamp_log_file:=<latency-file>
  - System has: pidstat, python3, ros2 CLI

Outputs:
  - Raw logs:   ${OUT_DIR}/<variant>/round_*/
  - Metrics:    ${OUT_DIR}/round_metrics.csv
  - Summary:    ${OUT_DIR}/ab_summary.txt
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --baseline-setup) BASELINE_SETUP="$2"; shift 2 ;;
    --opt-setup) OPT_SETUP="$2"; shift 2 ;;
    --baseline-cmd) BASELINE_CMD="$2"; shift 2 ;;
    --opt-cmd) OPT_CMD="$2"; shift 2 ;;
    --topic) TOPIC="$2"; shift 2 ;;
    --latency-file) LATENCY_FILE="$2"; shift 2 ;;
    --rounds) ROUNDS="$2"; shift 2 ;;
    --warmup) WARMUP_SEC="$2"; shift 2 ;;
    --measure) MEASURE_SEC="$2"; shift 2 ;;
    --help|-h) print_help; exit 0 ;;
    *)
      echo "Unknown arg: $1"
      print_help
      exit 1
      ;;
  esac
done

if [[ -z "${BASELINE_SETUP}" || -z "${OPT_SETUP}" || -z "${BASELINE_CMD}" || -z "${OPT_CMD}" ]]; then
  echo "Missing required args"
  print_help
  exit 1
fi

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1"
    exit 1
  fi
}

require_cmd python3
require_cmd pidstat
require_cmd ros2
require_cmd timeout

wait_for_livox_pid() {
  local launch_pid="$1"
  local waited=0
  while [[ ${waited} -lt ${STARTUP_TIMEOUT_SEC} ]]; do
    if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
      echo ""
      return
    fi
    local pid
    pid="$(pgrep -f "livox_driver_node|livox_ros_driver2" | head -n1 || true)"
    if [[ -n "${pid}" ]]; then
      echo "${pid}"
      return
    fi
    sleep 1
    waited=$((waited + 1))
  done
  echo ""
}

start_variant_round() {
  local variant="$1"
  local setup_file="$2"
  local launch_cmd="$3"
  local round="$4"

  local round_dir="${OUT_DIR}/${variant}/round_${round}"
  mkdir -p "${round_dir}"

  local launch_log="${round_dir}/launch.log"
  local pidstat_log="${round_dir}/pidstat.log"
  local hz_log="${round_dir}/topic_hz.log"
  local bw_log="${round_dir}/topic_bw.log"
  local latency_copy="${round_dir}/latency.csv"

  rm -f "${LATENCY_FILE}"

  echo "[$(date +%H:%M:%S)] ${variant} round ${round}: launching driver..."
  set +e
  bash -lc "source '${setup_file}' && ${launch_cmd}" >"${launch_log}" 2>&1 &
  local launch_pid=$!
  set -e

  local livox_pid
  livox_pid="$(wait_for_livox_pid "${launch_pid}")"
  if [[ -z "${livox_pid}" ]]; then
    echo "[$(date +%H:%M:%S)] ${variant} round ${round}: livox process not found, saving logs and aborting round"
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    return 1
  fi

  echo "[$(date +%H:%M:%S)] ${variant} round ${round}: livox pid=${livox_pid}, warmup ${WARMUP_SEC}s"
  sleep "${WARMUP_SEC}"

  echo "[$(date +%H:%M:%S)] ${variant} round ${round}: collect ${MEASURE_SEC}s metrics"
  pidstat -t -p "${livox_pid}" 1 >"${pidstat_log}" 2>&1 &
  local pidstat_pid=$!

  timeout "${MEASURE_SEC}" bash -lc "source '${setup_file}' && ros2 topic hz '${TOPIC}'" >"${hz_log}" 2>&1 &
  local hz_pid=$!

  timeout "${MEASURE_SEC}" bash -lc "source '${setup_file}' && ros2 topic bw '${TOPIC}'" >"${bw_log}" 2>&1 &
  local bw_pid=$!

  sleep "${MEASURE_SEC}"

  wait "${hz_pid}" >/dev/null 2>&1 || true
  wait "${bw_pid}" >/dev/null 2>&1 || true
  kill "${pidstat_pid}" >/dev/null 2>&1 || true
  wait "${pidstat_pid}" >/dev/null 2>&1 || true

  echo "[$(date +%H:%M:%S)] ${variant} round ${round}: stopping launch"
  kill -INT "${launch_pid}" >/dev/null 2>&1 || true
  sleep 2
  kill -TERM "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  sleep "${IDLE_AFTER_STOP_SEC}"

  if [[ -f "${LATENCY_FILE}" ]]; then
    cp "${LATENCY_FILE}" "${latency_copy}"
  else
    echo "[$(date +%H:%M:%S)] ${variant} round ${round}: warning latency file not found: ${LATENCY_FILE}" | tee -a "${round_dir}/warnings.log"
    : > "${latency_copy}"
  fi

  return 0
}

run_variant() {
  local variant="$1"
  local setup_file="$2"
  local launch_cmd="$3"

  mkdir -p "${OUT_DIR}/${variant}"
  for ((i=1; i<=ROUNDS; i++)); do
    if ! start_variant_round "${variant}" "${setup_file}" "${launch_cmd}" "${i}"; then
      echo "${variant} round ${i} failed" | tee -a "${OUT_DIR}/${variant}/failed_rounds.log"
    fi
  done
}

echo "Output dir: ${OUT_DIR}"
run_variant "baseline" "${BASELINE_SETUP}" "${BASELINE_CMD}"
run_variant "opt" "${OPT_SETUP}" "${OPT_CMD}"

echo "[$(date +%H:%M:%S)] analyzing metrics..."
python3 - "${OUT_DIR}" <<'PY'
import csv
import math
import pathlib
import re
import statistics
import sys

out_dir = pathlib.Path(sys.argv[1])
round_metrics_csv = out_dir / "round_metrics.csv"
summary_txt = out_dir / "ab_summary.txt"


def percentile(values, p):
    if not values:
        return math.nan
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    k = (len(values) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return values[int(k)]
    return values[f] * (c - k) + values[c] * (k - f)


def parse_hz(path):
    if not path.exists():
        return math.nan
    text = path.read_text(errors="ignore")
    m = re.findall(r"average rate:\s*([0-9.]+)", text)
    return float(m[-1]) if m else math.nan


def to_bps(value, unit):
    unit = unit.strip().lower()
    scale = {
        "b/s": 1,
        "kb/s": 1024,
        "mb/s": 1024**2,
        "gb/s": 1024**3,
    }
    return value * scale.get(unit, 1)


def parse_bw(path):
    if not path.exists():
        return math.nan
    text = path.read_text(errors="ignore")
    # e.g. average:  12.34MB/s
    m = re.findall(r"average:\s*([0-9.]+)\s*([kKmMgG]?[bB]/s)", text)
    if not m:
        return math.nan
    val, unit = m[-1]
    return to_bps(float(val), unit)


def parse_pidstat_cpu(path):
    if not path.exists():
        return math.nan
    lines = path.read_text(errors="ignore").splitlines()
    header_idx = None
    cpu_col = None
    for i, line in enumerate(lines):
        if "%CPU" in line:
            header = line.split()
            try:
                cpu_col = header.index("%CPU")
                header_idx = i
                break
            except ValueError:
                continue
    if header_idx is None or cpu_col is None:
        return math.nan

    vals = []
    for line in lines[header_idx + 1:]:
        s = line.strip()
        if not s or s.startswith("Average"):
            continue
        cols = line.split()
        if len(cols) <= cpu_col:
            continue
        try:
            vals.append(float(cols[cpu_col]))
        except ValueError:
            pass
    return statistics.mean(vals) if vals else math.nan


def parse_latency(path):
    if not path.exists() or path.stat().st_size == 0:
        return {
            "latency_mean_ms": math.nan,
            "latency_p50_ms": math.nan,
            "latency_p95_ms": math.nan,
            "latency_p99_ms": math.nan,
            "latency_max_ms": math.nan,
            "latency_count": 0,
        }
    vals = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                vals.append(float(row.get("latency_ms", "nan")))
            except ValueError:
                pass
    vals = [x for x in vals if not math.isnan(x)]
    if not vals:
        return {
            "latency_mean_ms": math.nan,
            "latency_p50_ms": math.nan,
            "latency_p95_ms": math.nan,
            "latency_p99_ms": math.nan,
            "latency_max_ms": math.nan,
            "latency_count": 0,
        }
    return {
        "latency_mean_ms": statistics.mean(vals),
        "latency_p50_ms": percentile(vals, 50),
        "latency_p95_ms": percentile(vals, 95),
        "latency_p99_ms": percentile(vals, 99),
        "latency_max_ms": max(vals),
        "latency_count": len(vals),
    }


rows = []
for variant in ("baseline", "opt"):
    vdir = out_dir / variant
    if not vdir.exists():
        continue
    for rd in sorted(vdir.glob("round_*")):
        try:
            round_id = int(rd.name.split("_")[-1])
        except Exception:
            continue
        latency = parse_latency(rd / "latency.csv")
        hz = parse_hz(rd / "topic_hz.log")
        bw_bps = parse_bw(rd / "topic_bw.log")
        cpu = parse_pidstat_cpu(rd / "pidstat.log")
        cpu_per_mb = math.nan
        if bw_bps and not math.isnan(bw_bps) and bw_bps > 0 and not math.isnan(cpu):
            cpu_per_mb = cpu / (bw_bps / (1024**2))

        row = {
            "variant": variant,
            "round": round_id,
            "topic_hz": hz,
            "topic_bw_bps": bw_bps,
            "cpu_percent_mean": cpu,
            "cpu_per_mb": cpu_per_mb,
            **latency,
        }
        rows.append(row)

if not rows:
    summary_txt.write_text("No rounds parsed. Check logs.\n")
    print(f"No valid rows parsed. See {out_dir}")
    sys.exit(0)

fieldnames = [
    "variant", "round", "topic_hz", "topic_bw_bps", "cpu_percent_mean", "cpu_per_mb",
    "latency_mean_ms", "latency_p50_ms", "latency_p95_ms", "latency_p99_ms", "latency_max_ms", "latency_count"
]
with round_metrics_csv.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(rows)


def agg(variant, key):
    vals = [r[key] for r in rows if r["variant"] == variant and not math.isnan(r[key])]
    if not vals:
        return (math.nan, math.nan, 0)
    mean = statistics.mean(vals)
    std = statistics.pstdev(vals) if len(vals) > 1 else 0.0
    return (mean, std, len(vals))


def improve(baseline, opt):
    if math.isnan(baseline) or math.isnan(opt) or baseline == 0:
        return math.nan
    return (baseline - opt) / baseline * 100.0

metrics = [
    "topic_hz", "topic_bw_bps", "cpu_percent_mean", "cpu_per_mb",
    "latency_mean_ms", "latency_p50_ms", "latency_p95_ms", "latency_p99_ms", "latency_max_ms"
]

lines = []
lines.append(f"A/B benchmark summary in: {out_dir}")
lines.append("")
lines.append("Per-metric aggregate (mean ± std):")
for m in metrics:
    b_mean, b_std, b_n = agg("baseline", m)
    o_mean, o_std, o_n = agg("opt", m)
    imp = improve(b_mean, o_mean)
    lines.append(
        f"- {m}: baseline={b_mean:.6g}±{b_std:.6g} (n={b_n}), "
        f"opt={o_mean:.6g}±{o_std:.6g} (n={o_n}), "
        f"improvement={imp:.2f}%"
    )

# Main KPIs
b_p95, _, _ = agg("baseline", "latency_p95_ms")
o_p95, _, _ = agg("opt", "latency_p95_ms")
b_cpu_mb, _, _ = agg("baseline", "cpu_per_mb")
o_cpu_mb, _, _ = agg("opt", "cpu_per_mb")

lines.append("")
lines.append("KPI checks:")
if not math.isnan(b_p95) and not math.isnan(o_p95):
    imp_p95 = improve(b_p95, o_p95)
    lines.append(f"- latency_p95 improvement: {imp_p95:.2f}% (target >= 15%)")
else:
    lines.append("- latency_p95 improvement: N/A")

if not math.isnan(b_cpu_mb) and not math.isnan(o_cpu_mb):
    imp_cpu = improve(b_cpu_mb, o_cpu_mb)
    lines.append(f"- cpu_per_mb improvement: {imp_cpu:.2f}% (target >= 10%)")
else:
    lines.append("- cpu_per_mb improvement: N/A")

summary_txt.write_text("\n".join(lines) + "\n")
print("\n".join(lines))
print(f"\nRound metrics: {round_metrics_csv}")
print(f"Summary: {summary_txt}")
PY

echo "[$(date +%H:%M:%S)] done"
echo "Results dir: ${OUT_DIR}"
