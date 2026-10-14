#!/usr/bin/env python3
# Copyright 2026 capstone_project
#
# Phân tích bag (chỉ đọc, không cần ROS graph): độ trễ lệnh -> vận tốc đo,
# gia tốc/phanh thực, quay lố.
#
#   ros2 bag record -o ~/rot_debug /cmd_vel_out /cmd_vel_nav /odometry/filtered \
#       /wheel_feedback /imu_odom /amcl_pose /CustomDWB/state
#   python3 analyze_cmd_lag.py ~/rot_debug [--axis w|v] [--cmd /cmd_vel_out] \
#       [--meas /odometry/filtered]
#
# Thời gian = thời điểm bag nhận tin (Twist của twist_mux không có stamp), nên mọi topic
# cùng một đồng hồ. Kết quả dùng để đặt CustomDWB.control_latency, acc_lim_theta /
# decel_lim_theta (giá trị thực đo được).
"""Estimate command -> measured velocity latency and real accel/decel from a rosbag2."""

import argparse
import bisect
import math
import sys

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def read_bag(path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    out = {t: [] for t in topics}
    msg_types = {t: get_message(types[t]) for t in topics if t in types}
    while reader.has_next():
        name, data, stamp_ns = reader.read_next()
        out[name].append((stamp_ns * 1e-9, rclpy.serialization.deserialize_message(
            data, msg_types[name])))
    return out, missing


def twist_of(msg):
    if hasattr(msg, 'twist') and hasattr(msg.twist, 'twist'):   # Odometry
        return msg.twist.twist
    if hasattr(msg, 'twist'):                                   # TwistStamped
        return msg.twist
    return msg                                                  # Twist


def series(samples, axis):
    t, x = [], []
    for stamp, msg in samples:
        tw = twist_of(msg)
        t.append(stamp)
        x.append(tw.angular.z if axis == 'w' else tw.linear.x)
    return t, x


def hold(t, x, query):
    """Zero-order hold of (t, x) at time query (lệnh giữ đến lệnh kế tiếp)."""
    i = bisect.bisect_right(t, query) - 1
    return x[i] if i >= 0 else 0.0


def estimate_delay(tc, xc, tm, xm, max_delay=2.0, step=0.01):
    """Delay d minimising sum (meas(t) - cmd(t - d))^2 over the measured samples."""
    best_d, best_e = 0.0, float('inf')
    d = 0.0
    while d <= max_delay + 1e-9:
        e, n = 0.0, 0
        for t, x in zip(tm, xm):
            if t - d < tc[0]:
                continue
            e += (x - hold(tc, xc, t - d)) ** 2
            n += 1
        if n > 10 and e / n < best_e:
            best_d, best_e = d, e / n
        d += step
    return best_d, math.sqrt(best_e) if best_e < float('inf') else float('nan')


def accel_stats(tm, xm, thresh):
    """Median of |dx/dt| while speeding up / slowing down (|x| > thresh), 5-sample windows."""
    acc, dec = [], []
    for i in range(5, len(tm)):
        dt = tm[i] - tm[i - 5]
        if dt <= 0.0:
            continue
        a = (xm[i] - xm[i - 5]) / dt
        mid = 0.5 * (xm[i] + xm[i - 5])
        if abs(mid) < thresh or abs(a) < 1e-3:
            continue
        (acc if a * mid > 0 else dec).append(abs(a))

    def med(v):
        return sorted(v)[len(v) // 2] if v else float('nan')
    return med(acc), med(dec), len(acc), len(dec)


def overshoot_events(tc, xc, tm, xm, thresh):
    """Angle/distance travelled after each stop command (|cmd| falls below thresh)."""
    events = []
    for i in range(1, len(tc)):
        if abs(xc[i - 1]) >= thresh and abs(xc[i]) < thresh:
            t0 = tc[i]
            integ, last_t = 0.0, None
            for t, x in zip(tm, xm):
                if t < t0:
                    continue
                if last_t is not None:
                    integ += x * (t - last_t)
                last_t = t
                if t > t0 + 0.3 and abs(x) < 0.02:
                    events.append((t0, t - t0, integ))
                    break
    return events


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('--axis', choices=['w', 'v'], default='w')
    ap.add_argument('--cmd', default='/cmd_vel_out')
    ap.add_argument('--meas', default='/odometry/filtered')
    ap.add_argument('--thresh', type=float, default=None,
                    help='|x| coi như đang chuyển động (mặc định 0.05 rad/s hoặc 0.02 m/s)')
    a = ap.parse_args()
    thresh = a.thresh if a.thresh is not None else (0.05 if a.axis == 'w' else 0.02)
    unit = 'rad/s' if a.axis == 'w' else 'm/s'

    data, missing = read_bag(a.bag, [a.cmd, a.meas])
    if missing:
        print(f'missing topics in bag: {missing}', file=sys.stderr)
        return 1
    tc, xc = series(data[a.cmd], a.axis)
    tm, xm = series(data[a.meas], a.axis)
    if len(tc) < 10 or len(tm) < 10:
        print('not enough samples', file=sys.stderr)
        return 1
    print(f'{a.cmd}: {len(tc)} msgs, {a.meas}: {len(tm)} msgs, axis {a.axis} [{unit}]')
    print(f'  |cmd| max {max(abs(x) for x in xc):.3f}, |meas| max {max(abs(x) for x in xm):.3f}')

    d, rms = estimate_delay(tc, xc, tm, xm)
    print(f'effective delay cmd -> meas (best fit, gồm cả ramp của driver): {d:.2f} s '
          f'(rms residual {rms:.3f} {unit})  -> CustomDWB.control_latency')
    acc, dec, na, nd = accel_stats(tm, xm, thresh)
    print(f'measured |accel| median {acc:.3f} {unit}/s ({na} windows), '
          f'|decel| median {dec:.3f} {unit}/s ({nd} windows)')
    cacc, cdec, _, _ = accel_stats(tc, xc, thresh)
    print(f'commanded |accel| median {cacc:.3f}, |decel| median {cdec:.3f} '
          '(lệnh dạng bậc thang cho số lớn/vô nghĩa; chỉ so sánh khi lệnh là dốc của DWB)')
    ev = overshoot_events(tc, xc, tm, xm, thresh)
    if ev:
        what = 'angle [rad]' if a.axis == 'w' else 'distance [m]'
        print(f'after-stop {what} (từ lúc lệnh về ~0 tới lúc đo về ~0):')
        for t0, dur, integ in ev:
            print(f'  t={t0 - tc[0]:7.2f} s  stop time {dur:.2f} s  travelled {integ:+.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
