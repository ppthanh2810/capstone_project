#!/usr/bin/env python3
"""Kiểm tra nghiệm thu S3 trên sim (KHÔNG dùng cho robot thật). Kịch bản cho world kho AWS (env:=warehouse).

Chạy khi s3_navigation.launch.py đang chạy (joy:=false gui:=false rviz:=false, ROS_DOMAIN_ID riêng, cùng
IGN_PARTITION với launch), robot spawn tại (0, 0) hướng -90°:
  ros2 launch stage4_gazebo s3_navigation.launch.py gui:=false rviz:=false joy:=false x:=0 y:=0 yaw:=-1.5708 \
      [camera:=true]
  ros2 run stage4_gazebo s3_check.py [--camera]

Bài thử (ground truth chỉ để chấm điểm, không đưa vào thuật toán):
  localized   2D Pose Estimate lệch -> CSM -> AMCL -> LOCALIZED (như S2)
  goal_*      2D Goal Pose -> A* -> CustomDWB -> SUCCEEDED; sai số cuối theo ground truth. Goal đặt ở LỐI ĐI
              thật: kệ AWS (ShelfD/E, x 2.77..6.69) hở ở độ cao LiDAR nên map chỉ có chân kệ, A* coi gầm kệ là
              trống; camera thấy tầng đáy kệ -> goal dưới gầm kệ bị CustomDWB từ chối (đúng)
  detour      hộp 0.3x0.3x0.5 m (LiDAR thấy) chắn giữa đường A* -> vòng qua, tới đích, không chạm
  blocked     tường 8 m chắn ngang -> dừng trước tường rồi ABORTED/CANCELED (A* không replan), không chạm;
              gỡ tường, chờ hết decay_time 10 s của FusionLayer (camera không xóa được ô gần hơn min_range 0.3 m)
  camera_*    (--camera) point cloud trong camera_depth_optical_frame đúng trục (hộp ở vị trí biết trước, qua TF),
              hộp thấp hơn mặt quét LiDAR không có trong /scan nhưng có trong local costmap; hộp thấp chắn
              đường -> vòng qua không chạm
Khoảng hở thân = khoảng cách (ground truth) từ tâm xe tới hộp - 0.20 m (Ø0.40); < 0 = va chạm.
"""

import argparse
import math
import os
import subprocess
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from s0_check import wrap  # noqa: E402
from s2_check import LIDAR_X, LIDAR_Z, S2Check  # noqa: E402

ROBOT_R = 0.20
TERMINAL = ('SUCCEEDED', 'ABORTED', 'CANCELED', 'REFUSED', 'REJECTED')
XY_TOL, YAW_TOL = 0.15, 0.25          # SimpleGoalChecker (nav2_params.yaml), so với pose ƯỚC LƯỢNG (AMCL)
GT_XY_TOL = 0.20                      # theo ground truth: + sai số định vị (~0.1 m ở góc trái map kho, xem PROGRESS)


def qmat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class S3Check(S2Check):

    def __init__(self):
        super().__init__()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.follow_state = None       # (t, text)
        self.path = None
        self.cloud = None
        self.costmap = None
        self.create_subscription(String, '/goal_path_bridge/follow_state', self.on_follow, latched)
        self.create_subscription(Path, '/global_path', lambda m: setattr(self, 'path', m), latched)
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points',
                                 lambda m: setattr(self, 'cloud', m), 5)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'costmap', m),
                                 latched)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.boxes = {}                # name -> (x, y, sx, sy)
        self.world = None

    def on_follow(self, msg):
        self.follow_state = (self.now_s(), msg.data)

    # ---------- vật cản ----------
    def add_box(self, name, x, y, sx, sy, sz):
        sdf = (f'<sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
               f'<collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
               f'<visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
               '<material><diffuse>0.9 0.2 0.2 1</diffuse></material></visual></link></model></sdf>')
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world, '-name', name, '-string', sdf,
                        '-x', str(x), '-y', str(y), '-z', str(sz / 2.0)], capture_output=True, timeout=30)
        self.boxes[name] = (x, y, sx, sy)
        self.spin_for(1.0)

    def del_box(self, name):
        self.remove_box(self.world, name)
        self.boxes.pop(name, None)
        self.spin_for(1.0)

    def path_clearance(self, name):
        """Khoảng hở thân nhỏ nhất nếu robot đi đúng đường A* (< 0: hộp chắn đường)."""
        bx, by, sx, sy = self.boxes[name]
        best = math.inf
        for ps in (self.path.poses if self.path else []):
            dx = max(abs(ps.pose.position.x - bx) - sx / 2, 0.0)
            dy = max(abs(ps.pose.position.y - by) - sy / 2, 0.0)
            best = min(best, math.hypot(dx, dy) - ROBOT_R)
        return best

    def box_clearance(self):
        x, y, _ = self.gt_pose()
        best = math.inf
        for bx, by, sx, sy in self.boxes.values():
            dx, dy = max(abs(x - bx) - sx / 2, 0.0), max(abs(y - by) - sy / 2, 0.0)
            best = min(best, math.hypot(dx, dy) - ROBOT_R)
        return best

    # ---------- điều hướng ----------
    def navigate(self, x, y, yaw_deg, timeout=180.0):
        """2D Goal Pose -> chờ trạng thái cuối của goal_path_bridge. Trả dict kết quả."""
        yaw = math.radians(yaw_deg)
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        t0 = self.now_s()
        self.goal_pub.publish(g)
        res = {'state': 'none', 'clear': math.inf, 'dist': 0.0}
        last = self.gt_pose()[:2]
        end_seen = None
        while rclpy.ok() and self.now_s() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            p = self.gt_pose()[:2]
            res['dist'] += math.hypot(p[0] - last[0], p[1] - last[1])
            last = p
            res['clear'] = min(res['clear'], self.box_clearance())
            fs = self.follow_state
            if fs and fs[0] > t0 + 0.05 and fs[1].startswith(TERMINAL):
                if end_seen is None:
                    end_seen = self.now_s()
                    res['state'], res['t'] = fs[1], self.now_s() - t0
                elif self.now_s() - end_seen > 1.5:      # để robot dừng hẳn (ramp 0.5 s)
                    break
        res.setdefault('t', self.now_s() - t0)
        gx, gy, gyaw = self.gt_pose()
        res['gt_err'] = math.hypot(gx - x, gy - y)
        res['gt_yaw'] = math.degrees(wrap(gyaw - yaw))
        if self.amcl:
            res['amcl_err'] = math.hypot(self.amcl[1] - x, self.amcl[2] - y)
            res['amcl_gt'] = math.hypot(self.amcl[1] - gx, self.amcl[2] - gy)
        else:
            res['amcl_err'] = res['amcl_gt'] = math.inf
        res['path'] = (sum(math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)
                           for a, b in zip(self.path.poses, self.path.poses[1:])) if self.path else math.nan)
        return res

    @staticmethod
    def fmt(r):
        clr = f", hở thân tới vật {r['clear']:.3f} m" if math.isfinite(r['clear']) else ''
        return (f"{r['state']} sau {r['t']:.1f} s; A* {r['path']:.2f} m, đi {r['dist']:.2f} m; GT sai "
                f"{r['gt_err']:.3f} m / {r['gt_yaw']:+.1f}°; AMCL tới đích {r['amcl_err']:.3f} m, AMCL-GT "
                f"{r['amcl_gt']:.3f} m{clr}")

    def test_goal(self, name, x, y, yaw_deg):
        r = self.navigate(x, y, yaw_deg)
        ok = r['state'] == 'SUCCEEDED' and r['gt_err'] <= GT_XY_TOL and abs(math.radians(r['gt_yaw'])) <= YAW_TOL
        self.check(name, ok, self.fmt(r))
        return r

    def test_detour(self):
        self.add_box('s3_obs', 0.0, -2.2, 0.3, 0.3, 0.5)
        r = self.navigate(0.0, -5.5, -90)
        on_path = self.path_clearance('s3_obs')
        self.del_box('s3_obs')
        self.check('detour', r['state'] == 'SUCCEEDED' and r['clear'] > 0.0 and on_path < 0.0,
                   self.fmt(r) + f'; nếu bám đúng A*: hở {on_path:.3f} m (< 0 = hộp chắn đường)')

    def test_blocked(self):
        self.add_box('s3_wall', 0.0, -3.0, 8.0, 0.2, 0.6)
        r = self.navigate(0.0, 0.0, 90, timeout=90.0)
        y = self.gt_pose()[1]
        self.del_box('s3_wall')
        self.spin_for(11.0)     # decay_time 10 s của FusionLayer (camera_fusion.yaml)
        ok = r['state'].startswith(('ABORTED', 'CANCELED')) and r['clear'] > 0.0 and y < -3.1
        self.check('blocked', ok, self.fmt(r) + f'; dừng ở y {y:.2f} (tường y -3.1..-2.9)')

    # ---------- camera ----------
    def cloud_in_base(self):
        n0 = self.cloud
        self.wait_until(lambda: self.cloud is not None and self.cloud is not n0, 3.0)
        m = self.cloud
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'))
        t = self.tf_buffer.lookup_transform('base_footprint', m.header.frame_id, Time(), Duration(seconds=2.0))
        tr = t.transform.translation
        return m.header.frame_id, a @ qmat(t.transform.rotation).T + np.array([tr.x, tr.y, tr.z])

    def costmap_max(self, bx, by, half):
        """Giá trị lớn nhất của local costmap trong ô vuông ±half quanh điểm (base_footprint)."""
        m = self.costmap
        t = self.tf_buffer.lookup_transform(m.header.frame_id, 'base_footprint', Time(), Duration(seconds=2.0))
        p = qmat(t.transform.rotation) @ np.array([bx, by, 0.0])
        cx, cy = p[0] + t.transform.translation.x, p[1] + t.transform.translation.y
        info, best = m.info, 0
        steps = np.arange(-half, half + 1e-6, info.resolution)
        for dx in steps:
            for dy in steps:
                i = int((cx + dx - info.origin.position.x) / info.resolution)
                j = int((cy + dy - info.origin.position.y) / info.resolution)
                if 0 <= i < info.width and 0 <= j < info.height:
                    best = max(best, m.data[j * info.width + i])
        return best

    def scan_points_near(self, bx, by, half):
        scan = self.fresh_scan('/scan')
        c = 0
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r):
                a = scan.angle_min + i * scan.angle_increment
                if abs(LIDAR_X + r * math.cos(a) - bx) < half and abs(r * math.sin(a) - by) < half:
                    c += 1
        return c

    def test_camera(self):
        if not self.wait_until(lambda: self.cloud is not None and self.costmap is not None, 10.0):
            self.check('camera_frame', False, 'không có /camera/camera/depth/color/points hoặc /local_costmap/costmap')
            return
        # hộp 0.2 x 0.2 x 0.3 (thấp hơn mặt quét LiDAR) tại (1.0, 0.3) trong base_footprint; vùng ±0.3 m quanh
        # đó phải trống trước khi đặt hộp (robot ở (0, 0) hướng -90° sau goal_after_abort)
        fx, fy, h = 1.0, 0.3, 0.3
        _, pts0 = self.cloud_in_base()
        if ((pts0[:, 2] > 0.03) & (np.abs(pts0[:, 0] - fx) < 0.3) & (np.abs(pts0[:, 1] - fy) < 0.3)).sum() > 20:
            self.check('camera_frame', False, 'vùng đặt hộp không trống (robot không ở (0, 0, -90°)?)')
            return
        x, y, yaw = self.gt_pose()
        bx, by = x + fx * math.cos(yaw) - fy * math.sin(yaw), y + fx * math.sin(yaw) + fy * math.cos(yaw)
        self.add_box('s3_cam', bx, by, 0.2, 0.2, h)
        self.spin_for(2.0)
        frame, pts = self.cloud_in_base()
        sel = pts[(pts[:, 2] > 0.03) & (np.abs(pts[:, 0] - fx) < 0.3) & (np.abs(pts[:, 1] - fy) < 0.3)]
        floor = pts[(np.hypot(pts[:, 0], pts[:, 1]) < 2.0) & (pts[:, 2] < 0.03)]
        if len(sel) > 100:
            lo, hi = sel.min(0), sel.max(0)
            exp_lo, exp_hi = np.array([fx - 0.1, fy - 0.1, 0.0]), np.array([fx + 0.1, fy + 0.1, h])
            ok = frame == 'camera_depth_optical_frame' and np.all(np.abs(lo - exp_lo) < 0.05) and \
                np.all(np.abs(hi - exp_hi) < 0.05) and abs(floor[:, 2].mean()) < 0.01
            detail = (f'frame {frame}; hộp qua TF: min {np.round(lo, 3).tolist()} max {np.round(hi, 3).tolist()} '
                      f'(kỳ vọng {exp_lo.tolist()}..{exp_hi.tolist()}); sàn z {floor[:, 2].mean():+.4f} m')
        else:
            ok, detail = False, f'frame {frame}; chỉ {len(sel)} điểm trên sàn trước robot'
        self.check('camera_frame', ok, detail)
        self.spin_for(1.0)
        n_scan = self.scan_points_near(fx, fy, 0.15)
        cost = self.costmap_max(fx, fy, 0.10)
        self.check('camera_low_box', n_scan == 0 and cost >= 99,
                   f'hộp cao {h} m < LiDAR {LIDAR_Z:.3f} m: /scan {n_scan} điểm, local costmap max {cost} (lethal 100)')
        self.del_box('s3_cam')

    def test_low_box_nav(self):
        self.add_box('s3_low', 0.0, -2.2, 0.25, 0.25, 0.2)
        r = self.navigate(0.0, -5.5, -90)
        self.del_box('s3_low')
        self.check('camera_low_box_nav', r['state'] == 'SUCCEEDED' and r['clear'] > 0.0, self.fmt(r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', action='store_true', help='launch chạy camera:=true: thêm các bài thử camera')
    ap.add_argument('--no-csm', action='store_true')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = S3Check()
    if not node.wait_ready_s2():
        node.get_logger().error('Không nhận được ground truth / scan / /wheel_feedback')
        return 2
    node.world = node.world_name()
    if node.world is None:
        node.get_logger().error('Không tìm được world Gazebo (IGN_PARTITION?)')
        return 2
    node.spin_for(2.0)
    node.test_localize(csm=not args.no_csm)
    node.test_goal('goal_far', 4.5, -0.33, 0)        # lối đi giữa kệ E1 (y 0.14..1.02) và D1 (y -1.68..-0.80)
    node.test_goal('goal_back', 0.0, 0.0, -90)
    node.test_detour()
    node.test_blocked()
    node.test_goal('goal_after_abort', 0.0, 0.0, -90)
    if args.camera:
        node.test_camera()
        node.test_low_box_nav()

    passed = sum(1 for _, ok, _ in node.results if ok)
    print(f'\n===== S3 ({"camera" if args.camera else "LiDAR"}): {passed}/{len(node.results)} PASS =====')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(node.results) else 1


if __name__ == '__main__':
    sys.exit(main())
