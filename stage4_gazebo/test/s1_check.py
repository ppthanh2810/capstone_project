#!/usr/bin/env python3
"""Kiểm tra nghiệm thu S1 trên sim (KHÔNG dùng cho robot thật).

Chạy khi s1_odom.launch.py đang chạy (khuyến nghị joy:=false gui:=false, ROS_DOMAIN_ID riêng):
  ros2 run stage4_gazebo s1_check.py [--distance 2.0] [--angle 90]

Phát lệnh lên /cmd_vel_nav, so /odometry/filtered (EKF) và /wheel_odom với /sim/ground_truth/odom
(mỗi nguồn quy về hệ robot lúc bắt đầu bài thử), đo tần số các topic và nguồn /tf. In PASS/FAIL.
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

from s0_check import S0Check, wrap, yaw_of


def pose_of(msg):
    p = msg.pose.pose
    return (p.position.x, p.position.y, yaw_of(p.orientation))


def delta_local(p0, p1):
    """Độ dời p0 -> p1 trong hệ robot tại p0: (dx, dy, dyaw)."""
    dx_w, dy_w = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(p0[2]), math.sin(p0[2])
    return (c * dx_w + s * dy_w, -s * dx_w + c * dy_w, wrap(p1[2] - p0[2]))


class S1Check(S0Check):

    def __init__(self):
        super().__init__('s1_check')
        self.last = {}                       # topic -> pose mới nhất
        self.stamps = {k: [] for k in ('/odometry/filtered', '/wheel_odom', '/imu_odom', '/imu/data')}
        self.imu_frames = set()
        self.imu_odom_yaw = None
        self.tf_edges = set()                # (parent, child) thấy trên /tf
        self.odom_tf_count = 0
        self.infos = []                      # bài thử tham khảo, không tính PASS/FAIL
        for topic in ('/odometry/filtered', '/wheel_odom', '/imu_odom'):
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.on_odom(t, m), 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.zero_yaw = self.create_client(Empty, '/imu_odom/zero_yaw')

    def on_odom(self, topic, msg):
        self.stamps[topic].append(self.now_s())
        self.last[topic] = pose_of(msg)
        if topic == '/imu_odom':
            self.imu_odom_yaw = self.last[topic][2]
            self.last['/imu_odom_frames'] = (msg.header.frame_id, msg.child_frame_id)

    def on_imu(self, msg):
        self.stamps['/imu/data'].append(self.now_s())
        self.imu_frames.add(msg.header.frame_id)

    def on_tf(self, msg):
        for t in msg.transforms:
            self.tf_edges.add((t.header.frame_id, t.child_frame_id))
            if t.child_frame_id == 'base_footprint':
                self.odom_tf_count += 1

    def gt_pose(self):
        return self.gt[1:4]

    def snapshot(self):
        return {'gt': self.gt_pose(), 'ekf': self.last['/odometry/filtered'], 'wheel': self.last['/wheel_odom']}

    def wait_ready_s1(self, timeout=60.0):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.gt is not None and '/odometry/filtered' in self.last and '/wheel_odom' in self.last:
                return True
        return False

    def settle(self):
        """Chờ robot dừng hẳn + EKF bắt kịp (vx filter ~0.2 s)."""
        self.wait_stopped(timeout=5.0)
        self.spin_for(1.0)

    def drive_until(self, v, w, done, timeout):
        """Phát (v, w) lên /cmd_vel_nav 10 Hz tới khi done(snapshot gt) đúng."""
        msg = self.twist(v, w)
        start = self.now_s()
        next_pub = start
        while rclpy.ok() and self.now_s() - start < timeout and not done():
            if self.now_s() >= next_pub:
                self.nav_pub.publish(msg)
                next_pub += 0.1
            rclpy.spin_once(self, timeout_sec=0.002)
        self.spin_for(0.5, [(self.nav_pub, self.twist())])  # lệnh 0 tường minh: chỉ còn ramp decel
        self.settle()

    def compare(self, s0, s1):
        g = delta_local(s0['gt'], s1['gt'])
        e = delta_local(s0['ekf'], s1['ekf'])
        wh = delta_local(s0['wheel'], s1['wheel'])
        return g, e, wh

    # ---------- bài thử ----------
    def test_rates(self, duration=10.0):
        n0 = {k: len(v) for k, v in self.stamps.items()}
        t0 = self.now_s()
        s0 = self.snapshot()
        self.spin_for(duration)
        dt = self.now_s() - t0
        hz = {k: (len(v) - n0[k]) / dt for k, v in self.stamps.items()}
        _, e, _ = self.compare(s0, self.snapshot())
        self.check('ekf_rate', 28.5 <= hz['/odometry/filtered'] <= 31.5,
                   f"/odometry/filtered {hz['/odometry/filtered']:.2f} Hz (sim time)")
        self.check('imu_rate', 45.0 <= hz['/imu_odom'] <= 55.0 and 45.0 <= hz['/imu/data'] <= 55.0,
                   f"/imu_odom {hz['/imu_odom']:.2f} Hz, /imu/data {hz['/imu/data']:.2f} Hz")
        self.check('wheel_odom_rate', 18.0 <= hz['/wheel_odom'] <= 22.0, f"/wheel_odom {hz['/wheel_odom']:.2f} Hz")
        self.check('ekf_still', math.hypot(e[0], e[1]) < 0.002 and abs(e[2]) < math.radians(0.1),
                   f'đứng yên {duration:.0f} s: EKF trôi {math.hypot(e[0], e[1]) * 1000:.2f} mm, '
                   f'{math.degrees(e[2]):+.3f}°')
        frames = self.last.get('/imu_odom_frames')
        self.check('imu_frames', self.imu_frames == {'imu_link'} and frames == ('odom', 'base_footprint'),
                   f'/imu/data frame {sorted(self.imu_frames)}, /imu_odom {frames}')

    def test_tf_sources(self, duration=5.0):
        # zlac8015d_odom luôn tạo TransformBroadcaster (có publisher /tf) nhưng chỉ gửi khi publish_tf:=true
        # -> kiểm tra bằng các cạnh thực sự xuất hiện và tần số cạnh odom -> base_footprint (EKF 30 Hz).
        pubs = sorted(i.node_name for i in self.get_publishers_info_by_topic('/tf'))
        self.odom_tf_count = 0
        self.tf_edges.clear()
        t0 = self.now_s()
        self.spin_for(duration)
        hz = self.odom_tf_count / (self.now_s() - t0)
        odom_edges = sorted(e for e in self.tf_edges if e[1] == 'base_footprint')
        ok = odom_edges == [('odom', 'base_footprint')] and 28.5 <= hz <= 31.5
        self.check('tf_sources', ok, f'cạnh tới base_footprint {odom_edges} {hz:.2f} Hz (chỉ EKF = 30); '
                   f'tất cả cạnh động {sorted(self.tf_edges)}; node có publisher /tf {pubs}')

    def test_straight(self, distance):
        s0 = self.snapshot()
        self.drive_until(0.2, 0.0,
                         lambda: math.hypot(*delta_local(s0['gt'], self.gt_pose())[:2]) >= distance,
                         timeout=distance / 0.2 + 10.0)
        g, e, wh = self.compare(s0, self.snapshot())
        err = math.hypot(e[0] - g[0], e[1] - g[1])
        err_w = math.hypot(wh[0] - g[0], wh[1] - g[1])
        self.check('straight', err <= 0.05,
                   f'thật dx {g[0]:+.4f} dy {g[1]:+.4f} m, yaw {math.degrees(g[2]):+.2f}°; '
                   f'EKF dx {e[0]:+.4f} dy {e[1]:+.4f}; sai EKF {err * 1000:.1f} mm '
                   f'(wheel_odom {err_w * 1000:.1f} mm), sai yaw EKF {math.degrees(wrap(e[2] - g[2])):+.2f}°')

    def test_rotate(self, angle_deg, sign):
        target = math.radians(angle_deg)
        s0 = self.snapshot()
        self.drive_until(0.0, sign * 0.4,
                         lambda: abs(delta_local(s0['gt'], self.gt_pose())[2]) >= target - math.radians(2.0),
                         timeout=target / 0.4 + 10.0)
        g, e, wh = self.compare(s0, self.snapshot())
        err = math.degrees(wrap(e[2] - g[2]))
        err_w = math.degrees(wrap(wh[2] - g[2]))
        name = f"rotate_{'ccw' if sign > 0 else 'cw'}_{angle_deg:.0f}"
        self.check(name, abs(err) <= 3.0,
                   f'thật {math.degrees(g[2]):+.2f}°, EKF {math.degrees(e[2]):+.2f}° -> sai {err:+.2f}° '
                   f'(wheel_odom {err_w:+.2f}°); dịch tâm thật {math.hypot(g[0], g[1]) * 1000:.1f} mm, '
                   f'EKF {math.hypot(e[0], e[1]) * 1000:.1f} mm')

    def test_arc(self, duration=8.0):
        s0 = self.snapshot()
        self.spin_for(duration, [(self.nav_pub, self.twist(0.2, 0.3))])
        self.spin_for(0.5, [(self.nav_pub, self.twist())])
        self.settle()
        g, e, wh = self.compare(s0, self.snapshot())
        err = math.hypot(e[0] - g[0], e[1] - g[1])
        err_w = math.hypot(wh[0] - g[0], wh[1] - g[1])
        err_yaw = math.degrees(wrap(e[2] - g[2]))
        # Không phải tiêu chí S1 (chỉ thẳng 2 m + quay 90°). Ghi lại để so với xe thật: EKF tích phân vx đã lọc
        # thông thấp (zlac8015d_odom α 0.2 @ 20 Hz, trễ ~0.2 s) theo yaw IMU không trễ -> trên cung tròn có
        # khởi động/dừng, sai vị trí ≈ v·τ·2·sin(Δyaw/2) (~7 cm ở bài này); wheel_odom (tích phân quãng đường) không bị.
        self.info('arc', err <= 0.05 and abs(err_yaw) <= 3.0,
                   f'v 0.2 w 0.3 trong {duration:.0f} s: thật ({g[0]:+.3f}, {g[1]:+.3f}, '
                   f'{math.degrees(g[2]):+.1f}°); EKF ({e[0]:+.3f}, {e[1]:+.3f}); sai EKF {err * 1000:.1f} mm, '
                   f'{err_yaw:+.2f}° (wheel_odom {err_w * 1000:.1f} mm, {math.degrees(wrap(wh[2] - g[2])):+.2f}°)')

    def info(self, name, ok, detail):
        self.infos.append((name, ok, detail))
        self.get_logger().info(f"[INFO {'ok' if ok else 'vượt 5 cm / 3°'}] {name}: {detail}")

    def test_zero_yaw(self):
        if not self.zero_yaw.wait_for_service(timeout_sec=5.0):
            self.check('zero_yaw_service', False, 'không thấy /imu_odom/zero_yaw')
            return
        before = self.imu_odom_yaw
        fut = self.zero_yaw.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.spin_for(0.5)
        after = self.imu_odom_yaw
        self.check('zero_yaw_service', fut.done() and abs(after) < math.radians(0.1),
                   f'/imu_odom yaw {math.degrees(before):+.2f}° -> {math.degrees(after):+.3f}°')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--distance', type=float, default=2.0, help='quãng đường thẳng (m)')
    ap.add_argument('--angle', type=float, default=90.0, help='góc quay (độ)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = S1Check()
    if not node.wait_ready_s1():
        node.get_logger().error('Không nhận được /sim/ground_truth/odom, /odometry/filtered hoặc /wheel_odom')
        return 2
    node.spin_for(2.0)
    node.test_rates()
    node.test_tf_sources()
    node.test_straight(args.distance)
    node.test_rotate(args.angle, +1)
    node.test_rotate(args.angle, -1)
    node.test_arc()
    node.test_zero_yaw()  # cuối cùng: đổi gốc yaw của /imu_odom làm EKF nhảy yaw

    passed = sum(1 for _, ok, _ in node.results if ok)
    print(f'\n===== S1: {passed}/{len(node.results)} PASS =====')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    for name, ok, detail in node.infos:
        print(f"INFO  {name} ({'≤' if ok else '>'} 5 cm / 3°, không tính): {detail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(node.results) else 1


if __name__ == '__main__':
    sys.exit(main())
