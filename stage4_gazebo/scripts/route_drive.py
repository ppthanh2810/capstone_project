#!/usr/bin/env python3
"""Lái robot SIM tự động theo tuyến điểm (công cụ thay người cầm tay cầm, dùng khi vẽ map / chạy kịch bản).

CHỈ DÙNG TRONG SIM. Lái bằng pose thật /sim/ground_truth/odom (như người lái nhìn thấy xe), phát /cmd_vel_nav
(prio 10, tay cầm PS4 vẫn đè được). Không cung cấp gì cho EKF / SLAM / AMCL: các thuật toán chỉ thấy
wheel odom + IMU + LiDAR như trên xe thật.

Đường đi: A* 8 hướng trên bản đồ tham chiếu (toạ độ world Gazebo), ô không trắng = vật cản, nới
`--inflate` m (bán kính xe 0.20 + lề), cộng phí gần vật cản để đi giữa lối. Điểm tuyến rơi vào vùng cấm
được dời tới ô trống gần nhất. Bám đường: pure pursuit (nhìn trước 0.5 m), quay tại chỗ khi lệch > 0.6 rad.
Tại mỗi điểm (tùy chọn) quay 360° để LiDAR (chỉ còn nửa trước sau laser_filters) quét đủ quanh.
Kẹt (đi < 5 cm sau 6 s CỘNG DỒN đang lệnh tiến, không tính lúc quay tại chỗ) -> dừng, báo lỗi.
--log-slam: in sai số TF map->base_footprint (SLAM/AMCL) so với ground truth tại mỗi điểm (chỉ để đánh giá).

  route_drive.py --preset warehouse            # tuyến vẽ map kho AWS (bản đồ tham chiếu AWS maps/005)
  route_drive.py --map <yaml> --route "x,y;x,y" [--no-spin] [--speed 0.25]
"""

import argparse
import heapq
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

AWS_DIR = os.environ.get('AWS_WAREHOUSE_DIR', os.path.expanduser('~/capstone_ws/third_party/aws_small_warehouse'))
PRESETS = {
    # Kho AWS: giữa kho -> đáy -> cột trái (lên) -> góc trên trái -> giữa -> 2 lối giữa các dãy kệ -> về gốc
    'warehouse': {
        'map': os.path.join(AWS_DIR, 'maps', '005', 'map.yaml'),
        'route': [(0.0, 0.0), (0.0, -6.5), (-5.0, -8.5), (-5.0, -3.0), (-5.0, 3.0), (-5.0, 8.5),
                  (1.5, 3.0), (3.6, -1.0), (3.6, -8.5), (5.6, -8.5), (5.6, -1.0), (5.0, 1.5), (0.0, 0.0)],
    },
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Grid:
    def __init__(self, yaml_path, inflate):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = np.asarray(Image.open(img_path).convert('L'), dtype=np.float64)
        p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.h, self.w = img.shape
        blocked = p > 0.1                      # chỉ ô "trắng" (free chắc chắn) mới đi được
        # khoảng cách (ô) tới vật cản gần nhất, tối đa R + 6 ô (đủ cho phí đi giữa lối)
        r_hard = int(math.ceil(inflate / self.res))
        r_max = r_hard + 6
        dist = np.full(blocked.shape, float(r_max + 1))
        dist[blocked] = 0.0
        for dy in range(-r_max, r_max + 1):
            for dx in range(-r_max, r_max + 1):
                d = math.hypot(dx, dy)
                if d == 0 or d > r_max:
                    continue
                shifted = np.zeros_like(blocked)
                ys = slice(max(dy, 0), self.h + min(dy, 0))
                yd = slice(max(-dy, 0), self.h + min(-dy, 0))
                xs = slice(max(dx, 0), self.w + min(dx, 0))
                xd = slice(max(-dx, 0), self.w + min(-dx, 0))
                shifted[yd, xd] = blocked[ys, xs]
                dist[shifted] = np.minimum(dist[shifted], d)
        self.free = dist > r_hard
        # phí gần vật cản: 0 khi cách >= r_max ô, tăng dần khi lại gần vùng cấm
        self.penalty = np.clip(r_max - dist, 0, None) * 0.5

    def cell(self, x, y):
        return self.h - 1 - int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def xy(self, rc):
        r, c = rc
        return self.ox + (c + 0.5) * self.res, self.oy + (self.h - 1 - r + 0.5) * self.res

    def nearest_free(self, rc):
        rr, cc = np.nonzero(self.free)
        i = np.argmin((rr - rc[0]) ** 2 + (cc - rc[1]) ** 2)
        return int(rr[i]), int(cc[i])

    def astar(self, start, goal):
        if not self.free[start]:
            start = self.nearest_free(start)
        if not self.free[goal]:
            goal = self.nearest_free(goal)
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        g = {start: 0.0}
        parent = {start: None}
        heap = [(0.0, start)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(self.xy(cur))
                    cur = parent[cur]
                return path[::-1]
            for dr, dc, cost in steps:
                nb = (cur[0] + dr, cur[1] + dc)
                if not (0 <= nb[0] < self.h and 0 <= nb[1] < self.w) or not self.free[nb]:
                    continue
                ng = g[cur] + cost + self.penalty[nb]
                if ng < g.get(nb, math.inf):
                    g[nb] = ng
                    parent[nb] = cur
                    heapq.heappush(heap, (ng + math.hypot(goal[0] - nb[0], goal[1] - nb[1]), nb))
        return None


class RouteDriver(Node):
    def __init__(self, speed, log_slam=False):
        super().__init__('route_drive', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer() if log_slam else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if log_slam else None
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 50)
        self.pose = None
        self.speed = speed

    def on_gt(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def send(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def step(self, dt=0.1):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < dt:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self, settle=1.5):
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < settle:
            self.send(0.0, 0.0)
            self.step()

    def log_slam(self, tag):
        if self.tf_buffer is None:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=1.0)).transform
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f'[{tag}] không có TF map->base_footprint: {ex}')
            return
        x, y, yaw = self.pose
        err = math.hypot(t.translation.x - x, t.translation.y - y)
        dyaw = math.degrees(wrap(yaw_of(t.rotation) - yaw))
        self.get_logger().info(f'[{tag}] map->base_footprint sai {err * 1000:.0f} mm, {dyaw:+.2f}° so với ground truth')

    def spin360(self, w=0.4):
        turned, prev = 0.0, self.pose[2]
        t0 = self.now()
        while rclpy.ok() and turned < 2 * math.pi and self.now() - t0 < 40.0:
            self.send(0.0, w)
            self.step()
            turned += abs(wrap(self.pose[2] - prev))
            prev = self.pose[2]
        self.stop()

    def follow(self, path, lookahead=0.5):
        idx = 0
        moving, last_pos = 0.0, self.pose[:2]   # thời gian cộng dồn đang lệnh tiến
        while rclpy.ok():
            x, y, yaw = self.pose
            # điểm gần nhất phía trước (không lùi chỉ số), rồi điểm nhìn trước
            while idx + 1 < len(path) and math.hypot(path[idx + 1][0] - x, path[idx + 1][1] - y) <= \
                    math.hypot(path[idx][0] - x, path[idx][1] - y):
                idx += 1
            goal = path[-1]
            if math.hypot(goal[0] - x, goal[1] - y) < 0.10:
                break
            j = idx
            while j + 1 < len(path) and math.hypot(path[j][0] - x, path[j][1] - y) < lookahead:
                j += 1
            tx, ty = path[j]
            err = wrap(math.atan2(ty - y, tx - x) - yaw)
            if abs(err) > 0.6:
                v, w = 0.0, math.copysign(0.4, err)
            else:
                dist_goal = math.hypot(goal[0] - x, goal[1] - y)
                v = min(self.speed, 0.5 * dist_goal + 0.05) * max(0.2, math.cos(err))
                w = max(-0.6, min(0.6, 1.5 * err))
            self.send(v, w)
            t_step = self.now()
            self.step()
            if v > 0.0:
                moving += self.now() - t_step
            if moving > 6.0:
                if math.hypot(self.pose[0] - last_pos[0], self.pose[1] - last_pos[1]) < 0.05:
                    self.stop()
                    return False
                moving, last_pos = 0.0, self.pose[:2]
        self.stop()
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--preset', choices=sorted(PRESETS))
    ap.add_argument('--map', help='bản đồ tham chiếu (yaml) ở toạ độ world Gazebo')
    ap.add_argument('--route', help='"x,y;x,y;..." (m, world)')
    ap.add_argument('--inflate', type=float, default=0.40, help='nới vật cản (m)')
    ap.add_argument('--speed', type=float, default=0.25, help='tốc độ tối đa (m/s)')
    ap.add_argument('--no-spin', action='store_true', help='không quay 360° tại mỗi điểm')
    ap.add_argument('--spin-speed', type=float, default=0.4, help='tốc độ quay 360° (rad/s)')
    ap.add_argument('--log-slam', action='store_true', help='in sai số TF map->base_footprint tại mỗi điểm')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    map_path = args.map or (PRESETS[args.preset]['map'] if args.preset else None)
    if args.route:
        route = [tuple(float(v) for v in p.split(',')) for p in args.route.split(';') if p.strip()]
    elif args.preset:
        route = PRESETS[args.preset]['route']
    else:
        ap.error('cần --preset hoặc --map + --route')
    if not map_path:
        ap.error('cần --map')

    grid = Grid(map_path, args.inflate)
    rclpy.init()
    node = RouteDriver(args.speed, args.log_slam)
    t_wall = time.monotonic()
    while rclpy.ok() and node.pose is None and time.monotonic() - t_wall < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        node.get_logger().error('Không nhận được /sim/ground_truth/odom')
        return 2

    t0 = node.now()
    total = 0.0
    ok = True
    if not args.no_spin:
        node.spin360(args.spin_speed)
    for i, (gx, gy) in enumerate(route):
        start = grid.cell(*node.pose[:2])
        path = grid.astar(start, grid.cell(gx, gy))
        if path is None:
            node.get_logger().error(f'Không có đường tới điểm {i} ({gx:.2f}, {gy:.2f})')
            ok = False
            break
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
        node.get_logger().info(f'Điểm {i + 1}/{len(route)} ({gx:+.2f}, {gy:+.2f}): đường {length:.2f} m')
        if length > 0.15 and not node.follow(path):
            node.get_logger().error(f'Kẹt khi tới điểm {i} tại ({node.pose[0]:+.2f}, {node.pose[1]:+.2f})')
            ok = False
            break
        total += length
        if not args.no_spin and length > 0.15:
            node.spin360(args.spin_speed)
        node.log_slam(f'điểm {i + 1}')
    node.stop()
    node.get_logger().info(f"{'Xong' if ok else 'DỪNG'}: {total:.1f} m, {node.now() - t0:.0f} s sim")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
