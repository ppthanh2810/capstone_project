#!/usr/bin/env python3
# Copyright 2026 capstone_project.
# Fixture offline cho Stage 3.2 — KHÔNG điều khiển phần cứng.
"""
Fixture mô phỏng động học tối giản cho controller_server (CustomDWB / Nav2 DWB), OFFLINE.

Thay thế Stage 2 khi chạy navigation.launch.py stage2:=false. Node này publish:
  /map (OccupancyGrid, transient_local; chỉ vật cản "mapped"),
  TF map->odom (đơn vị, 20 Hz), odom->base_footprint (30 Hz),
  base_footprint->laser_frame (tĩnh, theo URDF),
  /odometry/filtered (30 Hz, twist có nhiễu Gauss theo seed),
  /scan_filtered (10 Hz, 360 tia, góc 0 = phía trước, nửa sau (pi-1.57, pi+1.57) = NaN như
  laser_filters),
  tùy chọn camera:=true (M2): base_footprint->camera_depth_optical_frame (tĩnh, URDF) và
  /camera/camera/depth/color/points (15 Hz, PointCloud2 không texture như camera.launch.py,
  lưới tia 160x90 trong FOV 87x58 độ, tầm 6 m: sàn + vật cản 2D (cao 2 m) + hộp 3D 'unmapped_3d'),
và subscribe topic lệnh (mặc định /cmd_vel_dwb). Mô hình robot: lệnh tới bánh sau `latency` giây,
vận tốc bám lệnh với giới hạn gia tốc, lệnh cũ hơn `cmd_timeout` = 0 (mô phỏng timeout 0.5 s của
twist_mux), tích phân 200 Hz.
Ghi CSV quỹ đạo thật + khoảng hở (bán kính thân thật) ra `log_file`.

Hộp 3D (scenario 'unmapped_3d': [xmin, ymin, xmax, ymax, zmin, zmax]): LiDAR chỉ thấy khi mặt quét
(0.347 m) nằm trong [zmin, zmax]; camera thấy theo hình học 3D;
va chạm tính khi zmin < robot_height.

Giới hạn: không trượt bánh, không động lực học, localization hoàn hảo,
LiDAR/camera không nhiễu/rơi tia.
KHÔNG dùng để kết luận an toàn trên robot.
"""

import csv
import json
import math
import random
import time

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

LASER_XYZ = (0.15, 0.0, 0.1135 + 0.23365)   # base_footprint -> laser_frame (URDF)
CAMERA_XYZ = (0.17, 0.0, 0.1135 + 0.1625)    # base_footprint -> camera_link (URDF)
# camera_link -> camera_depth_optical_frame: rpy (-pi/2, 0, -pi/2) -> quaternion (x, y, z, w)
OPTICAL_Q = (-0.5, 0.5, -0.5, 0.5)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def step_toward(cur, target, acc, decel, h):
    """Bám target trong h giây: tăng |x| <= acc, giảm |x| <= decel (như dwa_core stepToward)."""
    def upper(x):
        if x >= 0.0:
            return x + acc * h
        t0 = -x / decel
        return x + decel * h if t0 >= h else acc * (h - t0)
    lo, hi = -upper(-cur), upper(cur)
    return min(max(target, lo), hi)


def ray_boxes_3d(o, d, boxes):
    """Khoảng cách tia 3D (o: (3,), d: (N, 3) đơn vị) tới hộp gần nhất (slab); inf nếu trượt."""
    best = np.full(d.shape[0], np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for b in boxes:
            lo, hi = b[[0, 1, 4]], b[[2, 3, 5]]
            t0 = (lo - o) / d
            t1 = (hi - o) / d
            tmin = np.nanmax(np.minimum(t0, t1), axis=1)
            tmax = np.nanmin(np.maximum(t0, t1), axis=1)
            hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.0)
            best = np.minimum(best, np.where(hit, tmin, np.inf))
    return best


class World:
    def __init__(self, circles, boxes, boxes3d=()):
        self.circles = np.array(circles, dtype=float).reshape(-1, 3)
        self.boxes = np.array(boxes, dtype=float).reshape(-1, 4)
        self.boxes3d = np.array(boxes3d, dtype=float).reshape(-1, 6)

    def distance(self, x, y, height=math.inf):
        """Khoảng cách 2D tới vật cản; hộp 3D chỉ tính khi zmin < height (robot chạm được)."""
        best = math.inf
        for x0, y0, x1, y1, z0, _ in self.boxes3d:
            if z0 >= height:
                continue
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            d = math.hypot(dx, dy)
            if dx == 0.0 and dy == 0.0:
                d = -min(x - x0, x1 - x, y - y0, y1 - y)
            best = min(best, d)
        for cx, cy, r in self.circles:
            best = min(best, math.hypot(x - cx, y - cy) - r)
        for x0, y0, x1, y1 in self.boxes:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            d = math.hypot(dx, dy)
            if dx == 0.0 and dy == 0.0:
                d = -min(x - x0, x1 - x, y - y0, y1 - y)
            best = min(best, d)
        return best

    def raycast(self, ox, oy, angles, max_range, z=None):
        """
        Khoảng cách tia (numpy) tới hình tròn/hình hộp gần nhất; inf nếu không chạm.

        z: độ cao mặt quét; hộp 3D có zmin <= z <= zmax được tính (None: bỏ qua hộp 3D).
        """
        dx, dy = np.cos(angles), np.sin(angles)
        best = np.full(angles.shape, np.inf)
        if z is not None:
            flat = [b for b in self.boxes3d if b[4] <= z <= b[5]]
            if flat:
                d3 = np.stack([dx, dy, np.zeros_like(dx)], axis=1)
                best = np.minimum(best, ray_boxes_3d(np.array([ox, oy, z]), d3, flat))
        for cx, cy, r in self.circles:
            fx, fy = ox - cx, oy - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            ok = disc >= 0.0
            t = -b - np.sqrt(np.where(ok, disc, 0.0))
            t = np.where(ok & (t > 0.0), t, np.inf)
            best = np.minimum(best, t)
        with np.errstate(divide='ignore', invalid='ignore'):
            for x0, y0, x1, y1 in self.boxes:
                tx0, tx1 = (x0 - ox) / dx, (x1 - ox) / dx
                ty0, ty1 = (y0 - oy) / dy, (y1 - oy) / dy
                tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
                tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
                hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.0)
                best = np.minimum(best, np.where(hit, tmin, np.inf))
        best[best > max_range] = np.inf
        return best


class OfflineSim(Node):
    def __init__(self):
        super().__init__('offline_sim')
        p = self.declare_parameter
        scen_file = p('scenario_file', '').value
        name = p('scenario', 'straight').value
        self.cmd_topic = p('cmd_topic', '/cmd_vel_dwb').value
        self.latency = p('latency', 0.1).value
        self.acc_x = p('acc_x', 0.5).value
        self.decel_x = p('decel_x', 0.5).value
        self.acc_th = p('acc_theta', 1.0).value
        self.decel_th = p('decel_theta', 1.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.noise = p('odom_vel_noise', 0.002).value
        self.radius = p('robot_radius', 0.20).value
        # Chiều cao robot kể cả laptop (va chạm với hộp 3D); NOT_VERIFIED,
        # = max_obstacle_height - 0.1.
        self.height = p('robot_height', 0.6).value
        self.camera = p('camera', False).value
        self.log_file = p('log_file', '').value
        random.seed(p('seed', 1).value)

        with open(scen_file) as f:
            sc = json.load(f)['scenarios'][name]
        self.sc = sc
        mapped = sc.get('mapped', {})
        unmapped = sc.get('unmapped', {})
        self.map_world = World(mapped.get('circles', []), mapped.get('boxes', []))
        self.world = World(mapped.get('circles', []) + unmapped.get('circles', []),
                           mapped.get('boxes', []) + unmapped.get('boxes', []),
                           sc.get('unmapped_3d', []))
        self.x, self.y = 0.0, 0.0
        self.th = sc.get('start_yaw', 0.0)
        self.v, self.w = 0.0, 0.0
        self.active = (0.0, 0.0)
        self.active_t = None
        self.queue = []
        self.t0 = time.monotonic()
        self.odom_stop = sc.get('odom_stop_after')
        self.scan_stop = sc.get('scan_stop_after')
        # Thời điểm runner gửi FollowPath (/sim/kick): mốc cho odom/scan_stop_after.
        self.go_time = None
        self.min_clear = math.inf
        self.collisions = 0

        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'base_footprint'
        st.child_frame_id = 'laser_frame'
        tr = st.transform.translation
        tr.x, tr.y, tr.z = LASER_XYZ
        st.transform.rotation.w = 1.0
        statics = [st]
        if self.camera:
            ct = TransformStamped()
            ct.header.stamp = st.header.stamp
            ct.header.frame_id = 'base_footprint'
            ct.child_frame_id = 'camera_depth_optical_frame'
            tr = ct.transform.translation
            tr.x, tr.y, tr.z = CAMERA_XYZ
            r = ct.transform.rotation
            r.x, r.y, r.z, r.w = OPTICAL_Q
            statics.append(ct)
        self.static_pub.sendTransform(statics)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        if self.camera:
            self.cloud_pub = self.create_publisher(
                PointCloud2, '/camera/camera/depth/color/points', 5)
            naz, nel = 160, 90
            hf, vf = math.radians(87.0), math.radians(58.0)
            az = -hf / 2 + hf * (np.arange(naz) + 0.5) / naz
            el = -vf / 2 + vf * (np.arange(nel) + 0.5) / nel
            self.cam_az, self.cam_el = [a.ravel() for a in np.meshgrid(az, el)]
            self.create_timer(1.0 / 15.0, self.publish_cloud)
        self.create_subscription(Float64, '/sim/kick', self.on_kick, 10)
        self.publish_map()

        self.csv = None
        if self.log_file:
            self.csv_f = open(self.log_file, 'w', newline='')
            self.csv = csv.writer(self.csv_f)
            self.csv.writerow(['mono', 'x', 'y', 'theta', 'v', 'w', 'cmd_v', 'cmd_w', 'clearance'])
        self.h = 0.005
        self.last_step = time.monotonic()
        self.create_timer(self.h, self.physics)
        self.create_timer(1.0 / 30.0, self.publish_odom_tf)
        self.create_timer(0.05, self.publish_map_odom)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(0.05, self.log_row)

    # ---------------------------------------------------------------- lệnh
    def now_s(self):
        return time.monotonic() - self.t0

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().error('non-finite command received')
            v, w = 0.0, 0.0
        self.queue.append((self.now_s() + self.latency, (v, w)))

    def on_kick(self, msg):
        # Mô phỏng: PS4 vừa thả khi robot đang chạy msg.data m/s (vận tốc + lệnh đang hiệu lực).
        self.go_time = self.now_s()
        if msg.data > 0.0:
            self.v = msg.data
            self.active = (msg.data, 0.0)
            self.active_t = self.now_s()

    # ---------------------------------------------------------------- vật lý
    def physics(self):
        now = time.monotonic()
        dt_total = min(now - self.last_step, 0.05)
        self.last_step = now
        t = self.now_s()
        while self.queue and self.queue[0][0] <= t:
            self.active = self.queue.pop(0)[1]
            self.active_t = t
        target = self.active
        if self.active_t is not None and t - self.active_t > self.cmd_timeout:
            target = (0.0, 0.0)   # twist_mux: lệnh cũ hơn timeout bị bỏ
        n = max(1, int(round(dt_total / 0.002)))
        h = dt_total / n
        for _ in range(n):
            v1 = step_toward(self.v, target[0], self.acc_x, self.decel_x, h)
            w1 = step_toward(self.w, target[1], self.acc_th, self.decel_th, h)
            vm, wm = 0.5 * (self.v + v1), 0.5 * (self.w + w1)
            if abs(wm) < 1e-9:
                self.x += vm * h * math.cos(self.th)
                self.y += vm * h * math.sin(self.th)
            else:
                self.x += vm / wm * (math.sin(self.th + wm * h) - math.sin(self.th))
                self.y -= vm / wm * (math.cos(self.th + wm * h) - math.cos(self.th))
                self.th = math.atan2(math.sin(self.th + wm * h), math.cos(self.th + wm * h))
            self.v, self.w = v1, w1
        clr = self.world.distance(self.x, self.y, self.height) - self.radius
        self.min_clear = min(self.min_clear, clr)
        if clr < 0.0:
            self.collisions += 1
        self.target = target

    def log_row(self):
        if self.csv:
            tgt = getattr(self, 'target', (0.0, 0.0))
            clr = self.world.distance(self.x, self.y, self.height) - self.radius
            self.csv.writerow([
                f'{time.monotonic():.3f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.th:.4f}',
                f'{self.v:.4f}', f'{self.w:.4f}', f'{tgt[0]:.4f}', f'{tgt[1]:.4f}',
                f'{clr:.4f}' if math.isfinite(clr) else 'inf'])
            self.csv_f.flush()

    # ---------------------------------------------------------------- cảm biến
    def stopped(self, after):
        if after is None or self.go_time is None:
            return False
        return self.now_s() - self.go_time > after

    def publish_odom_tf(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        q = yaw_to_quat(self.th)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = q
        self.tf_pub.sendTransform(tf)
        if self.stopped(self.odom_stop):
            return
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x = self.v + random.gauss(0.0, self.noise)
        o.twist.twist.angular.z = self.w + random.gauss(0.0, self.noise)
        self.odom_pub.publish(o)

    def publish_map_odom(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(tf)

    def publish_scan(self):
        if self.stopped(self.scan_stop):
            return
        n = 360
        ang = np.arange(n) * (2.0 * math.pi / n)
        lx = self.x + LASER_XYZ[0] * math.cos(self.th)
        ly = self.y + LASER_XYZ[0] * math.sin(self.th)
        r = self.world.raycast(lx, ly, ang + self.th, 12.0, z=LASER_XYZ[2])
        rear = (ang > math.pi - 1.57) & (ang < math.pi + 1.57)
        r[rear] = np.nan
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'laser_frame'
        s.angle_min = 0.0
        s.angle_increment = 2.0 * math.pi / n
        s.angle_max = s.angle_increment * (n - 1)
        s.time_increment = 0.0
        s.scan_time = 0.1
        s.range_min = 0.05
        s.range_max = 12.0
        s.ranges = [float(v) for v in r]
        self.scan_pub.publish(s)

    def publish_cloud(self):
        """Point cloud độ sâu (frame camera_depth_optical_frame) như RealSense không texture."""
        c, s_ = math.cos(self.th), math.sin(self.th)
        o = np.array([self.x + CAMERA_XYZ[0] * c, self.y + CAMERA_XYZ[0] * s_, CAMERA_XYZ[2]])
        ce, se = np.cos(self.cam_el), np.sin(self.cam_el)
        ang = self.cam_az + self.th
        # Tường / vật 2D (cao 2 m): khoảng cách ngang -> 3D, chỉ khi điểm chạm cao 0..2 m.
        th_ = self.world.raycast(o[0], o[1], ang, 6.0)
        t = th_ / ce
        zt = o[2] + t * se
        t = np.where((zt >= 0.0) & (zt <= 2.0), t, np.inf)
        d = np.stack([ce * np.cos(ang), ce * np.sin(ang), se], axis=1)
        if len(self.world.boxes3d):
            t = np.minimum(t, ray_boxes_3d(o, d, self.world.boxes3d))
        with np.errstate(divide='ignore'):
            t = np.minimum(t, np.where(se < 0.0, -o[2] / se, np.inf))   # sàn
        ok = np.isfinite(t) & (t <= 6.0)
        # Hướng tia trong frame robot (x trước, y trái, z lên) -> frame quang học (x phải, y xuống,
        # z trước).
        rx, ry, rz = ce * np.cos(self.cam_az), ce * np.sin(self.cam_az), se
        tt = t[ok]
        pts = np.stack([-ry[ok] * tt, -rz[ok] * tt, rx[ok] * tt], axis=1).astype(np.float32)
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'camera_depth_optical_frame'
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

    def publish_map(self):
        res, x0, y0, w, h = 0.05, -3.0, -3.0, 240, 160
        g = OccupancyGrid()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.info.resolution = res
        g.info.width, g.info.height = w, h
        g.info.origin.position.x, g.info.origin.position.y = x0, y0
        g.info.origin.orientation.w = 1.0
        data = []
        for j in range(h):
            for i in range(w):
                d = self.map_world.distance(x0 + (i + 0.5) * res, y0 + (j + 0.5) * res)
                data.append(100 if d <= 0.5 * res else 0)
        g.data = data
        self.map_pub.publish(g)

    def summary(self):
        return {'min_clearance': self.min_clear, 'collision_steps': self.collisions}


def main():
    rclpy.init()
    node = OfflineSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.csv:
            node.csv_f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
