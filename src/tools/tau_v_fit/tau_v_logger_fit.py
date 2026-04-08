#!/usr/bin/env python3
import argparse
import csv
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


@dataclass
class Sample:
    t: float
    cmd_v: float
    meas_v: float
    pred_v: float
    a_cmd: float
    tau_v: float
    vcmd_pred: float


def is_finite(x: float) -> bool:
    return not (math.isnan(x) or math.isinf(x))


class SpeedObserver(Node):
    def __init__(self, topic: str):
        super().__init__('tau_v_logger_fit')
        self.samples: List[Sample] = []
        self._sub = self.create_subscription(TwistStamped, topic, self._cb, 50)

    def _cb(self, msg: TwistStamped) -> None:
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t <= 0.0:
            t = time.time()

        self.samples.append(
            Sample(
                t=t,
                cmd_v=float(msg.twist.linear.x),
                meas_v=float(msg.twist.linear.y),
                pred_v=float(msg.twist.linear.z),
                a_cmd=float(msg.twist.angular.x),
                tau_v=float(msg.twist.angular.y),
                vcmd_pred=float(msg.twist.angular.z),
            )
        )


def fit_tau(samples: List[Sample], min_excitation: float) -> Optional[float]:
    if len(samples) < 3:
        return None

    s_xx = 0.0
    s_xy = 0.0

    for i in range(len(samples) - 1):
        a = samples[i]
        b = samples[i + 1]
        if not (is_finite(a.cmd_v) and is_finite(a.meas_v) and is_finite(b.meas_v)):
            continue

        dt = b.t - a.t
        if dt <= 1e-4 or dt > 0.2:
            continue

        x = a.cmd_v - a.meas_v
        if abs(x) < min_excitation:
            continue

        y = (b.meas_v - a.meas_v) / dt
        s_xx += x * x
        s_xy += x * y

    if s_xx <= 1e-12:
        return None

    slope = s_xy / s_xx  # slope ~= 1/tau
    if slope <= 1e-6:
        return None

    return 1.0 / slope


def rmse_pred_vs_meas(samples: List[Sample]) -> Optional[float]:
    err2 = []
    for s in samples:
        if is_finite(s.pred_v) and is_finite(s.meas_v):
            e = s.pred_v - s.meas_v
            err2.append(e * e)
    if not err2:
        return None
    return math.sqrt(sum(err2) / len(err2))


def rmse_one_step_model(samples: List[Sample], tau: float) -> Optional[float]:
    if tau <= 0.0:
        return None

    err2 = []
    for i in range(len(samples) - 1):
        a = samples[i]
        b = samples[i + 1]
        if not (is_finite(a.cmd_v) and is_finite(a.meas_v) and is_finite(b.meas_v)):
            continue
        dt = b.t - a.t
        if dt <= 1e-4 or dt > 0.2:
            continue

        v_next_hat = a.meas_v + dt * (a.cmd_v - a.meas_v) / tau
        e = v_next_hat - b.meas_v
        err2.append(e * e)

    if not err2:
        return None
    return math.sqrt(sum(err2) / len(err2))


def write_csv(path: str, samples: List[Sample]) -> None:
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            't', 'cmd_v', 'meas_v', 'pred_v_1step', 'a_cmd', 'tau_v_param', 'vcmd_pred_1step'
        ])
        for s in samples:
            w.writerow([s.t, s.cmd_v, s.meas_v, s.pred_v, s.a_cmd, s.tau_v, s.vcmd_pred])


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Record /nmpc/speed_observation and fit equivalent first-order tau_v.'
    )
    parser.add_argument('--topic', default='/nmpc/speed_observation', help='Input topic')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Record duration in seconds; <=0 means until Ctrl+C')
    parser.add_argument('--out-dir', default='log/tau_v_fit', help='Output directory')
    parser.add_argument('--prefix', default='tau_v_test', help='Output file prefix')
    parser.add_argument('--min-excitation', type=float, default=0.05,
                        help='Minimum |cmd_v - meas_v| to use for fitting')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    ts = time.strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join(args.out_dir, f'{args.prefix}_{ts}.csv')
    report_path = os.path.join(args.out_dir, f'{args.prefix}_{ts}_report.txt')

    rclpy.init()
    node = SpeedObserver(args.topic)

    stop = {'flag': False}

    def _sigint_handler(_sig, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGINT, _sigint_handler)

    t0 = time.time()
    print(f'[tau_v_fit] Recording topic: {args.topic}')
    if args.duration > 0:
        print(f'[tau_v_fit] Duration: {args.duration:.1f}s')
    else:
        print('[tau_v_fit] Duration: until Ctrl+C')

    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                break
    finally:
        samples = node.samples
        node.destroy_node()
        rclpy.shutdown()

    write_csv(csv_path, samples)

    tau = fit_tau(samples, args.min_excitation)
    rmse_pred = rmse_pred_vs_meas(samples)
    rmse_model = rmse_one_step_model(samples, tau) if tau is not None else None

    valid_meas = sum(1 for s in samples if is_finite(s.meas_v))
    valid_pred = sum(1 for s in samples if is_finite(s.pred_v))

    lines = []
    lines.append(f'samples_total={len(samples)}')
    lines.append(f'samples_meas_valid={valid_meas}')
    lines.append(f'samples_pred_valid={valid_pred}')
    lines.append(f'csv={csv_path}')

    if tau is not None:
        lines.append(f'tau_v_fit={tau:.6f}')
    else:
        lines.append('tau_v_fit=NaN (insufficient excitation/invalid data)')

    if rmse_pred is not None:
        lines.append(f'rmse_vpred_vs_vmeas={rmse_pred:.6f}')
    else:
        lines.append('rmse_vpred_vs_vmeas=NaN')

    if rmse_model is not None:
        lines.append(f'rmse_one_step_model={rmse_model:.6f}')
    else:
        lines.append('rmse_one_step_model=NaN')

    with open(report_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print('[tau_v_fit] Done')
    for l in lines:
        print('[tau_v_fit]', l)
    print(f'[tau_v_fit] report={report_path}')

    # Return non-zero only when no samples were recorded.
    return 0 if len(samples) > 0 else 2


if __name__ == '__main__':
    sys.exit(main())
