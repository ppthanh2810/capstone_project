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
  R8 gap050        khe 0.50 m — THÔNG TIN. Từ 2026-11-26 inscribed global = local 0.25 -> A* không qua nữa
  R9 occluded      tường mới 2.4 m chắn đường + hộp KHUẤT ngay sau đầu tường (góc khuất)  -> SUCCEEDED, không chạm
  R10 gap060       khe mép-mép 0.60 m (cận dưới quy tắc C1)       -> SUCCEEDED
  R11 multi5       5 hộp 0.3 m trong hành lang ±1 m, khe >= 0.60   -> SUCCEEDED
  R12 stale_scan   LiDAR ngừng (SIGSTOP lidar_sim_adapter) khi đang chạy -> robot dừng <= 1.5 s, không chạm;
                   LiDAR chạy lại -> điều hướng tiếp tục hoặc kết thúc có lý do (không treo)
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
from geometry_msgs.msg import PoseStamped
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
TERMINAL_ALL = s3_check.TERMINAL
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
    # Góc khuất: tường x -2.0..0.4 ở y -3.2 (ngoài tầm lúc gửi goal) + hộp 0.4 khuất ngay sau tường trên đường cũ.
    # Từ START tia tới hộp đi qua tường -> chỉ thấy hộp khi robot vòng qua đầu tường (x > 0.4).
    'R9': ('occluded', [('w', -0.8, -3.2, 2.4, 0.3, 0.6), ('h', 0.0, -4.2, 0.4, 0.4, 0.6)]),
    'R10': ('gap060', [('l', -0.55, -3.6, 0.5, 0.5, 0.6), ('r', 0.55, -3.6, 0.5, 0.5, 0.6)]),
    'R11': ('multi5', [('a', 0.35, -3.0, 0.3, 0.3, 0.6), ('b', -0.35, -3.8, 0.3, 0.3, 0.6),
                       ('c', 0.40, -4.6, 0.3, 0.3, 0.6), ('d', -0.75, -4.6, 0.3, 0.3, 0.6),
                       ('e', -0.05, -5.6, 0.3, 0.3, 0.6)]),
    'R12': ('stale_scan', []),
}
GOALS['R11'] = (0.0, -6.8, -90)


class Regression(S3Check):

    def __init__(self):
        super().__init__()
        self.states = []               # (t, text) mọi trạng thái bridge
        self.loc_events = []           # (t, state, reasons, consistency, blocked_ratio) khi != LOCALIZED
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

    def on_loc(self, msg):
        super().on_loc(msg)
        vals = {v.key: v.value for v in msg.values}
        if msg.message != 'LOCALIZED':
            self.loc_events.append((round(self.now_s(), 1), msg.message, vals.get('reasons', ''),
                                    vals.get('scan_consistency', ''), vals.get('blocked_ratio', '')))

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
        self.loc_events = []
        t0 = self.now_s()
        stale = None
        if sid == 'R12':
            stale = self.stale_scan_episode()
            r = stale.pop('nav')
        else:
            r = self.navigate(*GOALS.get(sid, GOAL), timeout=200.0)
        hold = self.hold_displacement() if r['state'].startswith('FAILED') else (math.nan, math.nan)
        rp = [t for t, s in self.states if s.startswith('REPLANNING')]
        gaps = [b - a for a, b in zip(rp, rp[1:])]
        res = dict(id=sid, name=name, state=r['state'], t=round(r['t'], 1), clear=round(r['clear'], 3),
                   gt_err=round(r['gt_err'], 3), dist=round(r['dist'], 2), path=round(r['path'], 2)
                   if not math.isnan(r['path']) else None, replans=len(rp),
                   min_replan_gap=round(min(gaps), 2) if gaps else None, carrot_shift_max=round(self.max_shift, 3),
                   carrot_blocked_msgs=self.n_blocked, paths=list(self.path_lens), hold_disp=hold[0], hold_yaw=hold[1],
                   states=[(round(t - t0, 1), s) for t, s in self.states],
                   loc_events=[(round(e[0] - t0, 1),) + e[1:] for e in self.loc_events[:40]])
        if stale:
            res.update(stale)
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

    def stale_scan_episode(self, freeze_after=4.0, freeze_for=6.0):
        """R12: đang chạy thì đóng băng lidar_sim_adapter (SIGSTOP) -> /scan_raw ngừng -> local costmap không
        current -> controller_server ngừng phát -> twist_mux timeout 0.5 s + driver ramp -> robot dừng.
        Đo thời gian/quãng tới khi |v| < 0.02 (GT). Sau freeze_for giây SIGCONT."""
        yaw = math.radians(GOAL[2])
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = GOAL[0], GOAL[1]
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(g)
        self.spin_for(freeze_after)
        pid = subprocess.run(['pgrep', '-f', 'lidar_sim_adapter'], capture_output=True, text=True).stdout.split()
        x0, y0, _ = self.gt_pose()
        subprocess.run(['kill', '-STOP'] + pid)
        t_freeze = self.now_s()
        stop_t, stop_d, last, prev_t = None, None, (x0, y0), self.now_s()
        while self.now_s() - t_freeze < freeze_for:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, _ = self.gt_pose()
            now = self.now_s()
            if now - prev_t >= 0.1:
                v = math.hypot(x - last[0], y - last[1]) / (now - prev_t)
                last, prev_t = (x, y), now
                if stop_t is None and v < 0.02 and now - t_freeze > 0.15:
                    stop_t, stop_d = now - t_freeze, math.hypot(x - x0, y - y0)
        subprocess.run(['kill', '-CONT'] + pid)
        # Không gửi goal mới: chờ phiên hiện tại tự kết thúc (tiếp tục bám / abort -> replan / FAILED).
        r = {'state': 'none', 'clear': math.inf, 'dist': 0.0, 'path': math.nan}
        t_cont = self.now_s()
        while rclpy.ok() and self.now_s() - t_cont < 150.0:
            rclpy.spin_once(self, timeout_sec=0.02)
            fs = self.follow_state
            if fs and fs[0] > t_freeze - 0.5 and fs[1].startswith(TERMINAL_ALL):
                r['state'] = fs[1]
                break
        self.spin_for(1.5)
        r['t'] = self.now_s() - t_freeze + freeze_after
        gx, gy, _ = self.gt_pose()
        r['gt_err'] = math.hypot(gx - GOAL[0], gy - GOAL[1])
        return {'nav': r, 'stale_pid': pid, 'stop_time': None if stop_t is None else round(stop_t, 2),
                'stop_dist': None if stop_d is None else round(stop_d, 3)}

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
            'R9': succ, 'R10': succ, 'R11': succ,
            # stale: dừng <= 1.5 s sau khi LiDAR ngừng; kết thúc có trạng thái (không treo); không chạm
            'R12': r.get('stop_time') is not None and r['stop_time'] <= 1.5 and r['state'] != 'none',
        }.get(r['id'])
        if cond is None:
            return 'INFO' if no_collision else 'FAIL'
        return 'PASS' if (cond and common) else 'FAIL'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='', help='vd R2,R4 (mặc định tất cả)')
    ap.add_argument('--out', default='')
    ap.add_argument('--repeat', type=int, default=1, help='số lượt lặp mỗi kịch bản (gate: 3)')
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
    results = []
    for k in range(args.repeat):
        for s in ids:
            res = node.run(s)
            res['round'] = k + 1
            results.append(res)
            if args.out:                 # ghi dần (lượt dài, tránh mất dữ liệu)
                with open(args.out, 'w') as f:
                    json.dump(results, f, ensure_ascii=False, indent=1)
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
