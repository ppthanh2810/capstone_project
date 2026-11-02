#!/usr/bin/env python3
"""Lưu vị trí hiện tại của robot SIM làm vị trí spawn cho lần mở sau (chỉ sim).

Đọc /sim/ground_truth/odom (pose thật trong world Gazebo = frame map) và ghi
  ~/.ros/stage4_gazebo/spawn_<env>.yaml   (x, y, yaw)
s2_slam.launch.py env:=<env> dùng file này làm vị trí spawn khi không truyền x/y/yaw. Chỉ là vị trí trong
Gazebo: định vị vẫn bằng 2D Pose Estimate trong RViz (hoặc init_pose:=true). Trạng thái riêng của máy, không
nằm trong repo.

  ros2 run stage4_gazebo save_pose.py --env warehouse            # lưu vị trí hiện tại
  ros2 run stage4_gazebo save_pose.py --env warehouse --pose 5.6 2.18 -3.136   # ghi tay (m, m, rad)
  ros2 run stage4_gazebo save_pose.py --env warehouse --clear    # xóa -> lần sau spawn (0, 0, 0)
  ros2 run stage4_gazebo save_pose.py --env warehouse --show
"""

import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry

STATE_DIR = os.path.join(os.path.expanduser('~'), '.ros', 'stage4_gazebo')


def pose_file(env):
    return os.path.join(STATE_DIR, f'spawn_{env}.yaml')


def load_pose(env):
    """(x, y, yaw) đã lưu cho env, hoặc None."""
    path = pose_file(env)
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    try:
        pose = (float(d['x']), float(d['y']), float(d['yaw']))
    except (KeyError, TypeError, ValueError):
        return None
    return pose if all(math.isfinite(v) for v in pose) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--env', default='warehouse', help='tên môi trường (warehouse | map)')
    ap.add_argument('--clear', action='store_true', help='xóa vị trí đã lưu')
    ap.add_argument('--show', action='store_true', help='in vị trí đã lưu')
    ap.add_argument('--pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                    help='ghi tay vị trí (m, m, rad) thay vì đọc từ sim')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    path = pose_file(args.env)

    if args.show:
        p = load_pose(args.env)
        print(f'{path}: ' + (f'x {p[0]:+.3f} y {p[1]:+.3f} yaw {math.degrees(p[2]):+.1f}°' if p else 'chưa lưu'))
        return 0
    if args.clear:
        if os.path.isfile(path):
            os.remove(path)
        print(f'Đã xóa {path} -> lần sau spawn (0, 0, 0)')
        return 0

    if args.pose:
        write_pose(path, args.env, *args.pose)
        return 0

    rclpy.init()
    node = rclpy.create_node('save_pose')
    got = []
    node.create_subscription(Odometry, '/sim/ground_truth/odom', got.append, 10)
    t0 = time.monotonic()
    while rclpy.ok() and not got and time.monotonic() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print('Không nhận được /sim/ground_truth/odom (sim có đang chạy, cùng ROS_DOMAIN_ID?)', file=sys.stderr)
        return 2

    p = got[-1].pose.pose
    q = p.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    write_pose(path, args.env, p.position.x, p.position.y, yaw)
    return 0


def write_pose(path, env, x, y, yaw):
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    os.makedirs(STATE_DIR, exist_ok=True)
    with open(path, 'w') as f:
        f.write('# Vị trí spawn của robot sim (save_pose.py), toạ độ world Gazebo = frame map\n')
        yaml.safe_dump({'env': env, 'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5),
                        'saved': time.strftime('%Y-%m-%d %H:%M:%S')}, f, sort_keys=False, allow_unicode=True)
    print(f'Đã lưu {path}: x {x:+.3f} y {y:+.3f} yaw {math.degrees(yaw):+.1f}°')


if __name__ == '__main__':
    sys.exit(main())
