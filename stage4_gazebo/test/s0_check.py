#!/usr/bin/env python3
"""Kiểm tra nghiệm thu S0 trên sim (KHÔNG dùng cho robot thật).

Chạy khi s0_control.launch.py đang chạy (khuyến nghị joy:=false gui:=false, ROS_DOMAIN_ID riêng):
  ros2 run stage4_gazebo s0_check.py [--still 60] [--csv /đường/dẫn/step.csv]

Phát lệnh lên /cmd_vel_nav và /cmd_vel_joy (đầu vào twist_mux), đo /wheel_feedback,
/sim/zlac/debug và /sim/ground_truth/odom theo sim time, in số liệu + PASS/FAIL.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

WHEEL_RADIUS = 0.0535
WHEEL_DISTANCE = 0.34
RPM_FACTOR = 30.0 / math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def expected_rpm(v, w):
    left = round((v - WHEEL_DISTANCE / 2.0 * w) / WHEEL_RADIUS * RPM_FACTOR)
    right = round((v + WHEEL_DISTANCE / 2.0 * w) / WHEEL_RADIUS * RPM_FACTOR)
    return left, right


class S0Check(Node):

    def __init__(self):
        super().__init__('s0_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.joy_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.create_subscription(Float64MultiArray, '/wheel_feedback', self.on_feedback, 50)
        self.create_subscription(Float64MultiArray, '/sim/zlac/debug', self.on_debug, 200)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 50)
        self.feedback = []   # (t, l_rpm, r_rpm, l_trav, r_trav)
        self.debug = []      # (t, tgt_l, tgt_r, out_l, out_r, joint_l, joint_r)
        self.gt = None       # (t, x, y, yaw, vx)
        self.results = []

    # ---------- callbacks ----------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_feedback(self, msg):
        self.feedback.append((self.now_s(), *msg.data))

    def on_debug(self, msg):
        self.debug.append(tuple(msg.data))

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (self.now_s(), p.position.x, p.position.y, yaw_of(p.orientation),
                   msg.twist.twist.linear.x)

    # ---------- helpers ----------
    def spin_for(self, duration, commands=None, rate=10.0):
        """Spin trong `duration` giây sim; commands = [(publisher, Twist)] phát với `rate` Hz."""
        start = self.now_s()
        next_pub = start
        last_pub = None
        while rclpy.ok() and self.now_s() - start < duration:
            if commands and self.now_s() >= next_pub:
                for pub, msg in commands:
                    pub.publish(msg)
                last_pub = self.now_s()
                next_pub += 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.002)
        return last_pub

    def wait_ready(self, timeout=60.0):
        import time
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.now_s() > 0.0 and self.gt is not None and self.feedback:
                return True
        return False

    def wait_stopped(self, timeout=3.0, tol=0.5):
        """Chờ tới khi |rpm| hai bánh < tol, trả về thời điểm sim (hoặc None)."""
        start = self.now_s()
        while rclpy.ok() and self.now_s() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.002)
            if self.feedback and self.feedback[-1][0] > start:
                _, l, r, _, _ = self.feedback[-1]
                if abs(l) < tol and abs(r) < tol:
                    return self.feedback[-1][0]
        return None

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        self.get_logger().info(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    @staticmethod
    def twist(v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        return msg

    # ---------- bài thử ----------
    def test_still(self, duration):
        g0 = self.gt
        n0 = len(self.feedback)
        t0 = self.now_s()
        self.spin_for(duration)
        g1 = self.gt
        fb = self.feedback[n0:]
        drift = math.hypot(g1[1] - g0[1], g1[2] - g0[2])
        dyaw = abs(wrap(g1[3] - g0[3]))
        max_rpm = max((max(abs(f[1]), abs(f[2])) for f in fb), default=float('nan'))
        self.check('still', drift < 0.005 and dyaw < 0.005 and max_rpm < 0.5,
                   f'{duration:.0f} s: trôi {drift * 1000:.2f} mm, yaw {math.degrees(dyaw):.3f}°, '
                   f'|rpm| max {max_rpm:.1f}')
        hz = len(fb) / (self.now_s() - t0)
        self.check('feedback_rate', 18.0 <= hz <= 22.0, f'/wheel_feedback {hz:.2f} Hz (sim time)')

    def drive_and_measure(self, name, v, w, duration, pub=None, extra=None):
        pub = pub or self.nav_pub
        g0 = self.gt
        n0 = len(self.feedback)
        commands = [(pub, self.twist(v, w))] + (extra or [])
        last = self.spin_for(duration, commands)
        g1 = self.gt
        fb = [f for f in self.feedback[n0:] if last - 0.5 < f[0] <= last]  # 0.5 s cuối, đã ổn định
        l_rpm = sum(f[1] for f in fb) / len(fb)
        r_rpm = sum(f[2] for f in fb) / len(fb)
        stop_t = self.wait_stopped()
        # quay về hệ robot lúc bắt đầu
        dx_w, dy_w = g1[1] - g0[1], g1[2] - g0[2]
        c, s = math.cos(g0[3]), math.sin(g0[3])
        dx, dy = c * dx_w + s * dy_w, -s * dx_w + c * dy_w
        dyaw = wrap(g1[3] - g0[3])
        return dict(dx=dx, dy=dy, dyaw=dyaw, l_rpm=l_rpm, r_rpm=r_rpm,
                    stop_delay=None if stop_t is None else stop_t - last)

    def test_directions(self):
        v = 0.2
        el, er = expected_rpm(v, 0.0)
        m = self.drive_and_measure('forward', v, 0.0, 3.0)
        self.check('forward', m['dx'] > 0.3 and abs(m['dy']) < 0.02 and m['l_rpm'] > 0 and m['r_rpm'] > 0,
                   f"dx {m['dx']:+.3f} m, dy {m['dy']:+.3f} m, rpm L {m['l_rpm']:+.1f} R {m['r_rpm']:+.1f} "
                   f'(lệnh {el:+d}/{er:+d})')
        self.check('watchdog_stop', m['stop_delay'] is not None and 0.9 <= m['stop_delay'] <= 1.3,
                   f"ngừng phát lệnh -> |rpm|<0.5 sau {m['stop_delay'] or float('nan'):.2f} s "
                   '(kỳ vọng ~0.5 twist_mux/watchdog + ≤0.1 timer + 0.5 decel)')

        m = self.drive_and_measure('backward', -v, 0.0, 3.0)
        self.check('backward', m['dx'] < -0.3 and abs(m['dy']) < 0.02 and m['l_rpm'] < 0 and m['r_rpm'] < 0,
                   f"dx {m['dx']:+.3f} m, dy {m['dy']:+.3f} m, rpm L {m['l_rpm']:+.1f} R {m['r_rpm']:+.1f}")

        w = 0.4
        el, er = expected_rpm(0.0, w)
        m = self.drive_and_measure('rotate_ccw', 0.0, w, 3.0)
        self.check('rotate_ccw', m['dyaw'] > 0.6 and m['l_rpm'] < 0 < m['r_rpm']
                   and math.hypot(m['dx'], m['dy']) < 0.02,
                   f"dyaw {math.degrees(m['dyaw']):+.1f}°, dịch {math.hypot(m['dx'], m['dy']) * 1000:.1f} mm, "
                   f"rpm L {m['l_rpm']:+.1f} R {m['r_rpm']:+.1f} (lệnh {el:+d}/{er:+d})")

        m = self.drive_and_measure('rotate_cw', 0.0, -w, 3.0)
        self.check('rotate_cw', m['dyaw'] < -0.6 and m['l_rpm'] > 0 > m['r_rpm'],
                   f"dyaw {math.degrees(m['dyaw']):+.1f}°, rpm L {m['l_rpm']:+.1f} R {m['r_rpm']:+.1f}")

    def test_priority(self):
        # nav muốn tiến 0.2, joy (prio 100) muốn lùi 0.1 -> phải lùi
        m = self.drive_and_measure('priority', 0.2, 0.0, 3.0,
                                   extra=[(self.joy_pub, self.twist(-0.1, 0.0))])
        el, _ = expected_rpm(-0.1, 0.0)
        self.check('priority_joy_over_nav', m['dx'] < -0.1 and m['l_rpm'] < 0 and m['r_rpm'] < 0,
                   f"dx {m['dx']:+.3f} m, rpm L {m['l_rpm']:+.1f} R {m['r_rpm']:+.1f} (joy lệnh {el:+d})")

    def test_step(self, csv_path):
        v = 0.3
        target, _ = expected_rpm(v, 0.0)
        n_dbg = len(self.debug)
        n_fb = len(self.feedback)
        t_step = self.now_s()
        last = self.spin_for(2.5, [(self.nav_pub, self.twist(v, 0.0))], rate=20.0)
        gt_v = self.gt[4]
        self.wait_stopped()
        dbg = [d for d in self.debug[n_dbg:] if d[0] >= t_step]
        fb = [f for f in self.feedback[n_fb:] if f[0] >= t_step]

        def first_time(series, idx, level):
            for row in series:
                if row[idx] >= level:
                    return row[0] - t_step
            return float('nan')

        # đầu ra ramp (debug, 200 Hz) và feedback (20 Hz) — so với 10/90/100 % mức đích
        t_cmd = first_time(dbg, 1, target - 0.5)
        t10 = first_time(dbg, 3, 0.1 * target) - t_cmd
        t90 = first_time(dbg, 3, 0.9 * target) - t_cmd
        t100 = first_time(dbg, 3, target - 1e-6) - t_cmd
        fb90 = first_time(fb, 1, 0.9 * target) - t_cmd
        steady = [f[1] for f in fb if last - 0.5 < f[0] <= last]  # 0.5 s cuối trước khi ngừng lệnh
        final = sum(steady) / max(1, len(steady))
        exp_v = target / RPM_FACTOR * WHEEL_RADIUS
        ok = (abs(t100 - 0.5) <= 0.03 and abs((t90 - t10) - 0.4) <= 0.03
              and abs(final - target) <= 0.5 and abs(gt_v - exp_v) < 0.01)
        self.check('step_0_to_0.3', ok,
                   f'đích {target} rpm; ramp: 10→90 % {t90 - t10:.3f} s, tới đích {t100:.3f} s '
                   f'(kỳ vọng 0.400 / 0.500 với accel 500 ms); feedback 90 % sau {fb90:.3f} s; '
                   f'RPM ổn định {final:.1f}; v thật {gt_v:.4f} m/s (kỳ vọng {exp_v:.4f})')
        if csv_path:
            with open(csv_path, 'w') as f:
                f.write('t,target_l,target_r,ramp_l,ramp_r,joint_rpm_l,joint_rpm_r\n')
                for d in dbg:
                    f.write(','.join(f'{x:.4f}' for x in (d[0] - t_step, *d[1:])) + '\n')
            self.get_logger().info(f'Đã ghi {csv_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--still', type=float, default=60.0, help='thời gian đứng yên (s sim)')
    ap.add_argument('--csv', default='', help='ghi đáp ứng bước ra CSV')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = S0Check()
    if not node.wait_ready():
        node.get_logger().error('Không nhận được /clock, /sim/ground_truth/odom hoặc /wheel_feedback')
        return 2
    node.spin_for(2.0)  # robot ổn định sau spawn
    node.test_still(args.still)
    node.test_directions()
    node.test_priority()
    node.test_step(args.csv)

    passed = sum(1 for _, ok, _ in node.results if ok)
    print(f'\n===== S0: {passed}/{len(node.results)} PASS =====')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(node.results) else 1


if __name__ == '__main__':
    sys.exit(main())
