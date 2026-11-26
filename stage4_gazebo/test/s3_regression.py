#!/usr/bin/env python3
"""Regression Navigation (System Characterization, bước 3 Master Plan) trên sim — KHÔNG dùng cho robot thật.

World kho AWS (env:=warehouse), robot spawn (0, 0) hướng -90°, mọi kịch bản đi (0, 0) -> (0, -5.5) rồi quay về.
Chạy khi s3_navigation.launch.py đang chạy (cùng ROS_DOMAIN_ID / IGN_PARTITION, joy:=false gui:=false rviz:=false):
  ros2 launch stage4_gazebo s3_navigation.launch.py gui:=false rviz:=false joy:=false x:=0 y:=0 yaw:=-1.5708
  ros2 run stage4_gazebo s3_regression.py [--only R2,R4] [--out result.json]

Kịch bản (hộp tĩnh, cao 0.6 m > mặt quét LiDAR 0.347 m; ground truth chỉ để chấm điểm):
  R1 normal        không vật cản                                  -> SUCCEEDED, 0 replan
  R2 detour        hộp 0.3x0.3 chắn đường                        -> SUCCEEDED, DWB tự né, 0 replan
  R3 carrot        hộp 0.4x1.2 dọc đường (cà rốt 0.8 m rơi vào)  -> SUCCEEDED, carrot_shift > 0
  R4 replan        tường 3.0x0.3 chắn ngang (lệch > max_path_deviation) -> SUCCEEDED, >= 1 replan
  R5 narrow        khe 0.65 m giữa 2 hộp trên đường               -> SUCCEEDED
  R6 multi         3 hộp zigzag                                   -> SUCCEEDED
  R7 no_path       đích bị rào kín (S1)                           -> FAILED (Safe Stop), đứng yên, không chạm
  R8 gap050        khe 0.50 m (A* qua được, DWB không) — THÔNG TIN (kiểm tra nhất quán footprint)
Kiểm tra chung: không va chạm (hở thân > 0), hai lần REPLANNING cách nhau >= min_interval, số replan <= giới hạn.
"""

import argparse
import json
import math
import os
import subprocess
import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import s3_check  # noqa: E402
from s3_check import S3Check  # noqa: E402

# FAILED = Safe Stop của goal_path_bridge (replan hết đường) cũng là trạng thái cuối.
s3_check.TERMINAL = s3_check.TERMINAL + ('FAILED',)
ROBOT_NAME = 'ace492'       # sim_world.launch.py

START = (0.0, 0.0, -90)
GOAL = (0.0, -5.5, -90)
GOALS = {'R6b': (0.0, -6.3, -90)}
MIN_INTERVAL = 2.0          # goal_path_bridge replan.min_interval (nav2_params.yaml)
MAX_REPLANS = 10 + 3        # replan.max_obstacle_replans + max_abort_replans


def ring(cx, cy, half, t=0.2, h=0.6):
    """4 tường bao quanh (cx, cy), cạnh trong 2*half."""
    L = 2 * half + 2 * t
    return [('top', cx, cy + half + t / 2, L, t, h), ('bot', cx, cy - half - t / 2, L, t, h),
            ('lft', cx - half - t / 2, cy, t, L, h), ('rgt', cx + half + t / 2, cy, t, L, h)]


# Vật cản "mới" đặt NGOÀI tầm LiDAR lúc gửi goal (obstacle_max_range 2.5 m; LiDAR ở y -0.15 khi robot ở gốc hướng
# -90° -> chỉ thấy tới y ~ -2.65): nếu không, obstacle_layer của global_costmap đã thấy vật cản và A* ban đầu lập đường
# vòng ngay (lần chạy 1, 2026-11-25: R3/R4 không cần cà rốt/replan). Cùng quy tắc cho Scenario Generator.
SCENARIOS = {
    'R1': ('normal', []),
    'R2': ('detour', [('b', 0.0, -3.6, 0.3, 0.3, 0.6)]),
    'R3': ('carrot', [('b', 0.0, -3.9, 0.4, 1.2, 0.6)]),
    'R4': ('replan', [('w', 0.0, -3.6, 3.0, 0.3, 0.6)]),
    'R5': ('narrow', [('l', -0.575, -3.6, 0.5, 0.5, 0.6), ('r', 0.575, -3.6, 0.5, 0.5, 0.6)]),
    'R6': ('multi', [('a', 0.3, -3.2, 0.4, 0.4, 0.6), ('b', -0.3, -4.0, 0.4, 0.4, 0.6),
                     ('c', 0.3, -4.8, 0.4, 0.4, 0.6)]),
    # Rào cạnh trong 1.2 m. Lần chạy 1 (cạnh 1.8 m): tường trái rào cách vật trên map (x -2..-1.25, y ~-6) chỉ
    # 0.15 m -> robot vào hốc, kết thúc ở ô mà global costmap coi là lethal, không lập đường về được (không lùi).
    # R6 khe mép-mép giữa các hộp chỉ 0.4 m (< khe DWB 0.55): lượt 3 -> abort loop -> ABORTED (phát hiện).
    # R6b đúng quy tắc C3 v2 (khe mép-mép >= 0.60 m): tâm cách 1.2 m -> khe 0.8 m, đích (0, -6.3).
    'R6b': ('multi_spec', [('a', 0.3, -3.0, 0.4, 0.4, 0.6), ('b', -0.3, -4.2, 0.4, 0.4, 0.6),
                           ('c', 0.3, -5.4, 0.4, 0.4, 0.6)]),
    'R7': ('no_path', ring(0.0, -5.5, 0.6)),
    'R8': ('gap050', [('l', -0.5, -3.6, 0.5, 0.5, 0.6), ('r', 0.5, -3.6, 0.5, 0.5, 0.6)]),
}


class Regression(S3Check):

    def __init__(self):
        super().__init__()
        self.states = []               # (t, text) mọi trạng thái bridge
        self.dwb = {}
        self.max_shift = 0.0
        self.n_blocked = 0
        self.create_subscription(DiagnosticStatus, '/CustomDWB/state', self.on_dwb, 10)
        self.clear_clients = [self.create_client(ClearEntireCostmap, s) for s in (
            '/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap')]
        self.path_lens = []            # độ dài mọi /global_path (đường đầu = A* ban đầu, sau = replan)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/global_path', self.on_path_len, latched)

    def on_path_len(self, m):
        if m.poses:
            self.path_lens.append(round(sum(math.hypot(a.pose.position.x - b.pose.position.x,
                                                       a.pose.position.y - b.pose.position.y)
                                            for a, b in zip(m.poses, m.poses[1:])), 2))

    def on_follow(self, msg):
        super().on_follow(msg)
        self.states.append((self.now_s(), msg.data))

    def on_dwb(self, msg):
        kv = {k.key: k.value for k in msg.values}
        try:
            self.max_shift = max(self.max_shift, abs(float(kv.get('carrot_shift', '0'))))
        except ValueError:
            pass
        if kv.get('carrot_blocked') == 'true':
            self.n_blocked += 1

    def clear_costmaps(self):
        # Client rclpy (lần chạy 2: `ros2 service call` treo > 30 s khi discovery chậm).
        for cli in self.clear_clients:
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(ClearEntireCostmap.Request())
                self.wait_until(lambda: fut.done(), 10.0)
            else:
                self.get_logger().warn(f'{cli.srv_name} không sẵn sàng')
        self.spin_for(1.0)

    def set_bridge_param(self, name, value):
        cli = self.create_client(SetParameters, '/goal_path_bridge/set_parameters')
        if not cli.wait_for_service(timeout_sec=10.0):
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        fut = cli.call_async(req)
        return self.wait_until(lambda: fut.done(), 10.0) and fut.result().results[0].successful

    def reset_robot(self):
        """Teleport robot về START (Gazebo set_pose) + xóa costmap + định vị lại (CSM/AMCL như 2D Pose Estimate).
        Chỉ dùng để tách các kịch bản (benchmark cũng cần reset); không phải hành vi điều hướng."""
        x, y, yaw = START[0], START[1], math.radians(START[2])
        req = (f'name: "{ROBOT_NAME}" position {{x: {x} y: {y} z: 0.0}} '
               f'orientation {{z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
        cancel = self.create_client(Trigger, '/goal_path_bridge/cancel')
        if cancel.wait_for_service(timeout_sec=5.0):
            fut = cancel.call_async(Trigger.Request())
            self.wait_until(lambda: fut.done(), 10.0)
        self.spin_for(1.5)
        subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                       '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, timeout=10)
        self.spin_for(1.0)
        self.test_localize(csm=True)
        self.results.pop()          # kết quả định vị lại không tính vào regression
        self.clear_costmaps()

    def hold_displacement(self, t_settle=1.5, window=2.0):
        """Đứng yên sau Safe Stop: chờ ramp driver rồi đo quãng dịch chuyển (ground truth)."""
        self.spin_for(t_settle)
        x0, y0, a0 = self.gt_pose()
        self.spin_for(window)
        x1, y1, a1 = self.gt_pose()
        return math.hypot(x1 - x0, y1 - y0), abs(math.degrees(math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))))

    def run(self, sid):
        name, boxes = SCENARIOS[sid]
        for b in boxes:
            self.add_box(f'reg_{sid}_{b[0]}', *b[1:])
        self.states, self.max_shift, self.n_blocked, self.path_lens = [], 0.0, 0, []
        t0 = self.now_s()
        r = self.navigate(*GOALS.get(sid, GOAL), timeout=200.0)
        hold = self.hold_displacement() if r['state'].startswith('FAILED') else (math.nan, math.nan)
        rp = [t for t, s in self.states if s.startswith('REPLANNING')]
        gaps = [b - a for a, b in zip(rp, rp[1:])]
        res = dict(id=sid, name=name, state=r['state'], t=round(r['t'], 1), clear=round(r['clear'], 3),
                   gt_err=round(r['gt_err'], 3), dist=round(r['dist'], 2), path=round(r['path'], 2)
                   if not math.isnan(r['path']) else None, replans=len(rp),
                   min_replan_gap=round(min(gaps), 2) if gaps else None, carrot_shift_max=round(self.max_shift, 3),
                   carrot_blocked_msgs=self.n_blocked, paths=list(self.path_lens), hold_disp=hold[0], hold_yaw=hold[1],
                   states=[(round(t - t0, 1), s) for t, s in self.states])
        for b in boxes:
            self.del_box(f'reg_{sid}_{b[0]}')
        self.clear_costmaps()
        res['verdict'] = self.verdict(res)
        back = self.navigate(*START, timeout=150.0)
        res['return'] = back['state']
        if back['state'] != 'SUCCEEDED' or back['gt_err'] > 0.25:
            self.reset_robot()
            res['return'] += ' -> reset'
        self.check(f'{sid}_{name}', res['verdict'] in ('PASS', 'INFO'), json.dumps(
            {k: v for k, v in res.items() if k != 'states'}, ensure_ascii=False))
        return res

    @staticmethod
    def verdict(r):
        no_collision = r['clear'] > 0.0
        ok_gap = r['min_replan_gap'] is None or r['min_replan_gap'] >= MIN_INTERVAL - 0.3
        common = no_collision and ok_gap and r['replans'] <= MAX_REPLANS
        succ = r['state'] == 'SUCCEEDED'
        cond = {
            'R1': succ and r['replans'] == 0,
            'R2': succ and r['replans'] == 0,
            'R3': succ and r['carrot_shift_max'] > 0.0,
            'R4': succ and r['replans'] >= 1,
            'R5': succ, 'R6': succ, 'R6b': succ,
            'R7': r['state'].startswith('FAILED') and r['hold_disp'] < 0.02,
        }.get(r['id'])
        if cond is None:
            return 'INFO' if no_collision else 'FAIL'
        return 'PASS' if (cond and common) else 'FAIL'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='', help='vd R2,R4 (mặc định tất cả)')
    ap.add_argument('--out', default='')
    ap.add_argument('--no-csm', action='store_true')
    ap.add_argument('--no-require-localized', action='store_true',
                    help='goal_path_bridge require_localized=false (tách Navigation khỏi localization_monitor)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Regression()
    if not node.wait_ready_s2():
        node.get_logger().error('Không nhận được ground truth / scan / /wheel_feedback')
        return 2
    node.world = node.world_name()
    node.spin_for(2.0)
    if args.no_require_localized:
        node.get_logger().info(f"require_localized=false: {node.set_bridge_param('require_localized', False)}")
    node.reset_robot()               # bắt đầu từ START đã định vị, costmap sạch
    ids = [s for s in (args.only.split(',') if args.only else SCENARIOS)]
    results = [node.run(s) for s in ids]
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(results, f, ensure_ascii=False, indent=1)
    passed = sum(1 for _, ok, _ in node.results if ok)
    print(f'\n===== Navigation regression: {passed}/{len(node.results)} PASS/INFO =====')
    for name, ok, detail in node.results:
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(node.results) else 1


if __name__ == '__main__':
    sys.exit(main())
