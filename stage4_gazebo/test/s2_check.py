#!/usr/bin/env python3
"""Kiểm tra nghiệm thu S2 trên sim (KHÔNG dùng cho robot thật).

Chạy khi s2_slam.launch.py đang chạy (khuyến nghị joy:=false gui:=false rviz:=false, ROS_DOMAIN_ID riêng,
cùng IGN_PARTITION với launch để gọi được service Gazebo):
  ros2 run stage4_gazebo s2_check.py --mode localization [--map maps/map.yaml]
  ros2 run stage4_gazebo s2_check.py --mode mapping [--save <stage4_gazebo/maps/sim_map>]

Chung: định dạng /scan_raw (quy ước sllidar), tần số, nửa sau /scan_filtered = NaN, scan khớp ray-cast trên
bản đồ dựng world (từ pose ground truth; so với giả thuyết xoay 180° / lật gương), hộp thấp hơn mặt quét
không xuất hiện trong scan còn hộp cao thì có.
localization: gửi 2D Pose Estimate lệch (0.15, -0.10 m, +5°) -> chờ /localization/status = LOCALIZED,
  so /amcl_pose với ground truth, lái thử rồi so lại.
mapping: /map lớn dần khi quay 360° + đi thẳng, pose map->base_footprint của slam_toolbox so với ground truth,
  chỉ slam_toolbox phát map->odom, (tùy chọn) lưu map bằng map_saver_cli.
World sinh từ bản đồ có toạ độ world = frame map, nên ground truth (sim_world) so thẳng được với frame map.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# map_to_world.py nằm cạnh file này trong lib/stage4_gazebo (symlink-install: __file__ chưa resolve symlink)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_to_world  # noqa: E402
from s0_check import S0Check, wrap, yaw_of  # noqa: E402

LIDAR_X = 0.15          # base_link -> lidar_link (robot.urdf), base_link = base_footprint theo x/y
LIDAR_Z = 0.1135 + 0.23365
REAR_LOWER, REAR_UPPER = 1.5715927, 4.7115927   # stage2_slam/config/angular_filter.yaml


class GridRaycaster:
    def __init__(self, yaml_path):
        self.occ, self.res, self.ox, self.oy, _ = map_to_world.load_map(yaml_path)
        self.h, self.w = self.occ.shape

    def cast(self, x, y, angle, max_range=16.0, step=0.005):
        c, s = math.cos(angle), math.sin(angle)
        r = 0.0
        while r <= max_range:
            col = int(math.floor((x + r * c - self.ox) / self.res))
            row = self.h - 1 - int(math.floor((y + r * s - self.oy) / self.res))
            if 0 <= row < self.h and 0 <= col < self.w and self.occ[row, col]:
                return r
            r += step
        return math.inf


def beam_angles(scan):
    return scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment


class S2Check(S0Check):

    def __init__(self):
        super().__init__('s2_check')
        self.scans = {k: None for k in ('/scan_raw', '/scan', '/scan_filtered')}
        self.stamps = {k: [] for k in self.scans}
        for topic in self.scans:
            self.create_subscription(LaserScan, topic, lambda m, t=topic: self.on_scan(t, m),
                                     qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.loc_status = None      # (t, message, consistency)
        self.csm_status = None
        self.amcl = None            # (t, x, y, yaw)
        self.map_msg = None
        self.tf_edges = set()
        self.create_subscription(DiagnosticStatus, '/localization/status', self.on_loc, latched)
        self.create_subscription(DiagnosticStatus, '/scan_match/status', self.on_csm, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.guess_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose_guess', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.infos = []

    # ---------- callbacks ----------
    def on_scan(self, topic, msg):
        self.scans[topic] = msg
        self.stamps[topic].append(self.now_s())

    def on_loc(self, msg):
        vals = {v.key: v.value for v in msg.values}
        self.loc_status = (self.now_s(), msg.message, vals.get('scan_consistency', 'n/a'))

    def on_csm(self, msg):
        self.csm_status = (self.now_s(), msg.message, {v.key: v.value for v in msg.values})

    def on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl = (self.now_s(), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_map(self, msg):
        self.map_msg = msg

    def on_tf(self, msg):
        for t in msg.transforms:
            self.tf_edges.add((t.header.frame_id, t.child_frame_id))

    # ---------- helpers ----------
    def gt_pose(self):
        return self.gt[1:4]

    def free_ahead(self, caster):
        """Quãng trống phía trước cho CẢ THÂN XE (Ø0.40): ray-cast 5 tia song song lệch ngang ±0.25 m từ
        tâm xe. Một tia giữa không đủ: bản đồ thật có khe nhỏ trên tường mảnh, tia lọt qua nhưng xe thì không."""
        x, y, yaw = self.gt_pose()
        c, s = math.cos(yaw), math.sin(yaw)
        return min(caster.cast(x - off * s, y + off * c, yaw) for off in (-0.25, -0.125, 0.0, 0.125, 0.25))

    def lidar_pose(self):
        x, y, yaw = self.gt_pose()
        return x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw), yaw

    def wait_until(self, cond, timeout):
        start = self.now_s()
        while rclpy.ok() and self.now_s() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if cond():
                return True
        return False

    def wait_ready_s2(self, timeout=90.0):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt is not None and all(self.scans.values()) and self.feedback:
                return True
        return False

    def fresh_scan(self, topic='/scan_filtered'):
        n = len(self.stamps[topic])
        self.wait_until(lambda: len(self.stamps[topic]) > n + 1, 3.0)
        return self.scans[topic]

    def front_range(self, half_width_deg=3.0):
        scan = self.fresh_scan()
        a = np.mod(beam_angles(scan), 2 * math.pi)
        r = np.asarray(scan.ranges)
        sel = (np.minimum(a, 2 * math.pi - a) <= math.radians(half_width_deg)) & np.isfinite(r)
        return float(r[sel].min()) if sel.any() else math.inf

    def info(self, name, detail):
        self.infos.append((name, detail))
        self.get_logger().info(f'[INFO] {name}: {detail}')

    # ---------- bài thử chung ----------
    def test_scan_format_and_rates(self, duration=10.0):
        n0 = {k: len(v) for k, v in self.stamps.items()}
        t0 = self.now_s()
        self.spin_for(duration)
        dt = self.now_s() - t0
        hz = {k: (len(v) - n0[k]) / dt for k, v in self.stamps.items()}
        raw = self.scans['/scan_raw']
        ok = (raw.header.frame_id == 'laser_raw' and len(raw.ranges) == 720
              and abs(raw.angle_min + math.pi) < 1e-4 and abs(raw.angle_max - math.pi) < 1e-4
              and abs(raw.range_min - 0.05) < 1e-6 and abs(raw.range_max - 16.0) < 1e-6
              and not any(math.isnan(r) or r < 0.05 for r in raw.ranges))
        self.check('scan_raw_format', ok,
                   f'frame {raw.header.frame_id}, {len(raw.ranges)} tia, góc [{raw.angle_min:+.4f}, '
                   f'{raw.angle_max:+.4f}], inc {raw.angle_increment:.6f} (2pi/719 = {2 * math.pi / 719:.6f}), '
                   f'range [{raw.range_min}, {raw.range_max}], scan_time {raw.scan_time:.3f} s')
        sc = self.scans['/scan']
        self.check('scan_frames', sc.header.frame_id == 'laser_frame'
                   and self.scans['/scan_filtered'].header.frame_id == 'laser_frame'
                   and abs(sc.angle_min) < 1e-4 and abs(sc.angle_max - 2 * math.pi) < 1e-4,
                   f'/scan frame {sc.header.frame_id} góc [{sc.angle_min:.4f}, {sc.angle_max:.4f}]')
        self.check('scan_rates', all(9.5 <= v <= 10.5 for v in hz.values()),
                   ', '.join(f'{k} {v:.2f} Hz' for k, v in hz.items()) + ' (sim time)')

    def test_rear_nan(self):
        scan = self.fresh_scan()
        a = beam_angles(scan)
        r = np.asarray(scan.ranges)
        rear = (a > REAR_LOWER) & (a < REAR_UPPER)
        front = ~rear
        self.check('rear_nan', bool(np.isnan(r[rear]).all()) and not np.isnan(r[front]).any(),
                   f'nửa sau {int(np.isnan(r[rear]).sum())}/{int(rear.sum())} tia NaN, '
                   f'nửa trước {int(np.isnan(r[front]).sum())} NaN, {int(np.isfinite(r[front]).sum())} tia hữu hạn')

    def test_scan_vs_map(self, caster):
        scan = self.fresh_scan()
        lx, ly, yaw = self.lidar_pose()
        a = beam_angles(scan)
        r = np.asarray(scan.ranges)

        def errors(world_angle_of):
            err = []
            for ai, ri in zip(a, r):
                if not math.isfinite(ri):
                    continue
                re_ = caster.cast(lx, ly, world_angle_of(ai))
                if math.isfinite(re_) and re_ < 15.0:
                    err.append(ri - re_)
            return np.asarray(err)

        e = errors(lambda ai: yaw + ai)
        e_rot = errors(lambda ai: yaw + ai + math.pi)
        e_mir = errors(lambda ai: yaw - ai)
        med = float(np.median(np.abs(e)))
        within = float(np.mean(np.abs(e) < 0.05))
        # Tia lệch > 5 cm nằm ở chỗ khoảng cách nhảy (mép góc, khe chéo giữa 2 ô tường dày 1 ô, tầm xa):
        # dưới 2 m ~97 % < 5 cm, 10–16 m ~69 % (đo 2026-11-02) -> tiêu chí theo trung vị + 85 %.
        ok = (len(e) > 50 and med < 0.03 and within > 0.85
              and np.median(np.abs(e_rot)) > 0.5 and np.median(np.abs(e_mir)) > 0.5)
        self.check('scan_vs_map', ok,
                   f'{len(e)} tia: |sai| trung vị {med * 1000:.1f} mm, {within * 100:.1f} % < 5 cm '
                   f'(giả thuyết xoay 180°: {np.median(np.abs(e_rot)) * 1000:.0f} mm, '
                   f'lật gương: {np.median(np.abs(e_mir)) * 1000:.0f} mm)')

    def world_name(self):
        # `ign service -l` có thể trả danh sách thiếu khi discovery chưa xong -> thử lại
        for _ in range(10):
            out = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True, timeout=10).stdout
            names = sorted(set(re.findall(r'^/world/([^/]+)/create$', out, re.M)))
            if len(names) == 1:
                return names[0]
            time.sleep(1.0)
        return None

    def spawn_box(self, world, name, x, y, height):
        sdf = (f'<sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
               f'<collision name="c"><geometry><box><size>0.2 0.2 {height}</size></box></geometry></collision>'
               f'<visual name="v"><geometry><box><size>0.2 0.2 {height}</size></box></geometry>'
               '<material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual></link></model></sdf>')
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', world, '-name', name, '-string', sdf,
                        '-x', str(x), '-y', str(y), '-z', str(height / 2.0)],
                       capture_output=True, timeout=30)

    def remove_box(self, world, name):
        subprocess.run(['ign', 'service', '-s', f'/world/{world}/remove', '--reqtype', 'ignition.msgs.Entity',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                        '--req', f'name: "{name}" type: MODEL'], capture_output=True, timeout=10)

    def test_low_box(self, caster, dist=0.8):
        world = self.world_name()
        lx, ly, yaw = self.lidar_pose()
        free = caster.cast(lx, ly, yaw)
        if world is None or free < dist + 0.6:
            self.check('low_box', False, f'không thử được (world {world}, phía trước trống {free:.2f} m)')
            return
        bx, by = lx + dist * math.cos(yaw), ly + dist * math.sin(yaw)
        before = self.front_range()
        res = {}
        for name, height in (('s2_box_low', LIDAR_Z - 0.05), ('s2_box_tall', LIDAR_Z + 0.15)):
            self.spawn_box(world, name, bx, by, height)
            self.spin_for(1.0)
            res[name] = self.front_range()
            self.remove_box(world, name)
            self.spin_for(1.0)
        after = self.front_range()
        def same(a, b):  # phía trước có thể trống hẳn (inf)
            return (math.isinf(a) and math.isinf(b)) or abs(a - b) < 0.05

        ok = (same(res['s2_box_low'], before) and abs(res['s2_box_tall'] - (dist - 0.1)) < 0.05
              and same(after, before))
        self.check('low_box', ok,
                   f'phía trước ±3°: không hộp {before:.3f} m; hộp cao {LIDAR_Z - 0.05:.3f} m (dưới mặt quét '
                   f'{LIDAR_Z:.3f}) {res["s2_box_low"]:.3f} m; hộp cao {LIDAR_Z + 0.15:.3f} m '
                   f'{res["s2_box_tall"]:.3f} m (mặt hộp {dist - 0.1:.2f} m); sau khi xóa {after:.3f} m')

    # ---------- localization ----------
    def amcl_error(self):
        if self.amcl is None:
            return math.inf, math.inf
        x, y, yaw = self.gt_pose()
        return math.hypot(self.amcl[1] - x, self.amcl[2] - y), math.degrees(wrap(self.amcl[3] - yaw))

    def test_localize(self, csm=True, timeout=40.0):
        x, y, yaw = self.gt_pose()
        dx, dy, dyaw = 0.15, -0.10, math.radians(5.0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x + dx
        msg.pose.pose.position.y = y + dy
        msg.pose.pose.orientation.z = math.sin((yaw + dyaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos((yaw + dyaw) / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25     # giống RViz 2D Pose Estimate
        msg.pose.covariance[35] = 0.06853891945200942
        t_pub = self.now_s()
        (self.guess_pub if csm else self.init_pub).publish(msg)
        ok = self.wait_until(lambda: self.loc_status is not None and self.loc_status[0] > t_pub + 1.0
                             and self.loc_status[1] == 'LOCALIZED', timeout)
        t_loc = self.now_s() - t_pub
        self.spin_for(2.0)
        pos, ang = self.amcl_error()
        csm_txt = (f"CSM {self.csm_status[1]} score {self.csm_status[2].get('score', 'n/a')}"
                   if self.csm_status else 'CSM không có trạng thái')
        status = self.loc_status[1] if self.loc_status else 'không có'
        self.check('localized', ok and pos <= 0.10 and abs(ang) <= 3.0,
                   f'đoán lệch 18 cm / 5° -> {status} sau {t_loc:.1f} s; {csm_txt}; AMCL sai {pos * 1000:.0f} mm, '
                   f'{ang:+.2f}°; consistency {self.loc_status[2] if self.loc_status else "n/a"}')

    def drive_tracking(self, v, w, done, timeout, track):
        msg = self.twist(v, w)
        start = self.now_s()
        next_pub = start
        while rclpy.ok() and self.now_s() - start < timeout and not done():
            if self.now_s() >= next_pub:
                self.nav_pub.publish(msg)
                next_pub += 0.1
            rclpy.spin_once(self, timeout_sec=0.002)
            track()
        self.spin_for(0.5, [(self.nav_pub, self.twist())])
        self.wait_stopped(timeout=5.0)
        self.spin_for(2.0)

    def test_localized_drive(self, caster):
        states = {}
        worst = [0.0, 0.0]

        def track():
            if self.loc_status:
                states[self.loc_status[1]] = states.get(self.loc_status[1], 0) + 1
            if self.amcl and self.amcl[0] > self.now_s() - 0.2:
                pos, ang = self.amcl_error()
                worst[0] = max(worst[0], pos)
                worst[1] = max(worst[1], abs(ang))

        dist = min(1.5, self.free_ahead(caster) - 0.8)
        g0 = self.gt_pose()
        if dist > 0.3:
            self.drive_tracking(0.2, 0.0, lambda: math.hypot(self.gt[1] - g0[0], self.gt[2] - g0[1]) >= dist,
                                dist / 0.2 + 10.0, track)
        y0 = self.gt_pose()[2]
        self.drive_tracking(0.0, 0.4, lambda: abs(wrap(self.gt[3] - y0)) >= math.radians(88.0), 15.0, track)
        pos, ang = self.amcl_error()
        moved = math.hypot(self.gt[1] - g0[0], self.gt[2] - g0[1])
        status = self.loc_status[1] if self.loc_status else 'không có'
        # 0.15 m (không phải 0.10): amcl_params.yaml dùng mô hình laser mặc định Nav2 (sigma_hit 0.2 m,
        # 60 tia) -> khi lái trên bản đồ khớp hoàn hảo, sai AMCL dao động ~4–12 cm (đo 2026-11-02).
        self.check('localized_drive', status == 'LOCALIZED' and pos <= 0.15 and abs(ang) <= 3.0,
                   f'đi thẳng {moved:.2f} m + quay 90°: cuối {status}, AMCL sai {pos * 1000:.0f} mm {ang:+.2f}°; '
                   f'lớn nhất {worst[0] * 1000:.0f} mm / {worst[1]:.2f}°; số mẫu trạng thái {states}')

    # ---------- mapping ----------
    def known_cells(self):
        if self.map_msg is None:
            return 0
        return int(np.count_nonzero(np.asarray(self.map_msg.data, dtype=np.int16) >= 0))

    def slam_pose_error(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', Time(), Duration(seconds=1.0))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f'TF map->base_footprint: {ex}')
            return math.inf, math.inf
        x, y, yaw = self.gt_pose()
        tr = t.transform
        return (math.hypot(tr.translation.x - x, tr.translation.y - y),
                math.degrees(wrap(yaw_of(tr.rotation) - yaw)))

    def test_mapping(self, caster, save):
        self.wait_until(lambda: self.map_msg is not None, 20.0)
        k0 = self.known_cells()
        y_prev = self.gt_pose()[2]
        turned = [0.0]

        def track_turn():
            nonlocal y_prev
            turned[0] += abs(wrap(self.gt[3] - y_prev))
            y_prev = self.gt[3]

        self.drive_tracking(0.0, 0.4, lambda: turned[0] >= 2 * math.pi, 30.0, track_turn)
        dist = min(1.5, self.free_ahead(caster) - 0.8)
        g0 = self.gt_pose()
        if dist > 0.3:
            self.drive_tracking(0.2, 0.0, lambda: math.hypot(self.gt[1] - g0[0], self.gt[2] - g0[1]) >= dist,
                                dist / 0.2 + 10.0, lambda: None)
        self.spin_for(3.0)
        k1 = self.known_cells()
        m = self.map_msg
        self.check('map_grows', m is not None and k1 > k0 > 0,
                   f'ô đã biết {k0} -> {k1}; map {m.info.width}x{m.info.height} @ {m.info.resolution:.3f} m'
                   if m else 'không nhận được /map')
        pos, ang = self.slam_pose_error()
        self.check('slam_pose', pos <= 0.10 and abs(ang) <= 3.0,
                   f'sau quay 360° + đi {math.hypot(self.gt[1] - g0[0], self.gt[2] - g0[1]):.2f} m: '
                   f'map->base_footprint sai {pos * 1000:.0f} mm, {ang:+.2f}° so với ground truth')
        self.tf_edges.clear()
        self.spin_for(3.0)
        to_odom = sorted(e for e in self.tf_edges if e[1] == 'odom')
        pubs = sorted(i.node_name for i in self.get_publishers_info_by_topic('/tf'))
        self.check('map_odom_source', to_odom == [('map', 'odom')] and 'slam_toolbox' in pubs
                   and 'amcl' not in pubs,
                   f'cạnh tới odom {to_odom}; node có publisher /tf {pubs}')
        if save:
            os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
            r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save,
                                '--ros-args', '-p', 'use_sim_time:=true'],
                               capture_output=True, text=True, timeout=60)
            ok = r.returncode == 0 and os.path.isfile(save + '.yaml') and os.path.isfile(save + '.pgm')
            self.check('map_saved', ok, f'{save}.yaml/.pgm' if ok else (r.stderr or r.stdout)[-300:])


def main():
    default_map = os.path.expanduser('~/capstone_ws/capstone_project/maps/map.yaml')
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('localization', 'mapping'), default='localization')
    ap.add_argument('--map', default=default_map, help='bản đồ đã dùng để sinh world (world_map của launch)')
    ap.add_argument('--no-csm', action='store_true', help='launch chạy csm:=false (gửi thẳng /initialpose)')
    ap.add_argument('--save', default='', help='mapping: lưu map (đường dẫn không đuôi)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = S2Check()
    caster = GridRaycaster(args.map)
    if not node.wait_ready_s2():
        node.get_logger().error('Không nhận được ground truth / /scan_raw / /scan / /scan_filtered / /wheel_feedback')
        return 2
    node.spin_for(2.0)
    node.test_scan_format_and_rates()
    node.test_rear_nan()
    node.test_scan_vs_map(caster)
    node.test_low_box(caster)
    if args.mode == 'localization':
        node.test_localize(csm=not args.no_csm)
        node.test_localized_drive(caster)
    else:
        node.test_mapping(caster, args.save)

    passed = sum(1 for _, ok, _ in node.results if ok)
    print(f'\n===== S2 ({args.mode}): {passed}/{len(node.results)} PASS =====')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(node.results) else 1


if __name__ == '__main__':
    sys.exit(main())
