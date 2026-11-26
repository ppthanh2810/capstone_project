#!/usr/bin/env python3
"""Chạy scenario của manifest trên Gazebo (toàn stack S3 thật: AMCL, costmap Nav2, controller_server, bridge) và chấm
bằng CÙNG Episode Evaluator với Fast 2D (Master Plan v3 B6, C2). CHỈ SIM.

Mỗi layout (map M) cần một world -> script tự launch s3_navigation (env:=map, world_map = M, spawn = start) headless,
ROS_DOMAIN_ID / IGN_PARTITION riêng, ROS_LOCALHOST_ONLY=1; rồi với mỗi bộ tham số DWA: spawn Δ, ros2 param set,
gửi goal, ghi GT 10 Hz + trạng thái bridge, chấm, gỡ Δ, teleport về start, định vị lại.

  gazebo_episode.py --manifest DIR --ids a,b,c --param-sets sets.json --out out.jsonl [--mode planner_gt|planner|full]
  sets.json: {"base": {}, "set1": {"obstacle_scale": 2.0, "sim_time": 2.5}, ...}  (tên CustomDWB.*)
mode planner_gt (V4, Gazebo Planner Benchmark) = TF map->odom từ ground truth (localization_source:=ground_truth),
  không AMCL/monitor -> so ghép cặp với Fast 2D (định vị hoàn hảo);
mode planner = AMCL + require_localized false (v3); full = AMCL + monitor + require_localized (Full-system).
Khoảng hở GT: CÙNG hàm C++ World::clearance với Fast 2D (biên ô chính xác + Δ), lấy mẫu mỗi tin GT (50 Hz).
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
import warnings

warnings.filterwarnings('ignore')

import numpy as np  # noqa: E402

from stage5_benchmark import config as C  # noqa: E402
from stage5_benchmark import evaluator as E  # noqa: E402
from stage5_benchmark import fast2d  # noqa: E402
from stage5_benchmark import runner as RN  # noqa: E402
from stage5_benchmark import scenario_io as IO  # noqa: E402

ROBOT = 'ace492'
ACTION_KEYS = ('path_distance_scale', 'carrot_distance_scale', 'obstacle_scale', 'heading_scale', 'speed_scale',
               'sim_time')


def quat_z(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class GzEpisodes:
    def __init__(self, args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry, Path
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('gazebo_episode', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        n = self.node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.gt = None
        self.gt_t = None
        self.cur_world = None
        self.ep_min_clr = math.inf
        self.robot_radius = 0.20
        self.state = None
        self.loc = None
        self.scan_t = None
        self.wall_clock = []
        n.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_gt, 50)
        n.create_subscription(String, '/goal_path_bridge/follow_state', self.on_state, latched)
        n.create_subscription(DiagnosticStatus, '/localization/status', lambda m: setattr(self, 'loc', m.message),
                              latched)
        n.create_subscription(LaserScan, '/scan_filtered', lambda m: setattr(self, 'scan_t', self.now()), 5)
        n.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.plan_lengths = []
        n.create_subscription(Path, '/global_path', self.on_path, latched)
        self.amcl = None
        n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, latched)
        self.goal_pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
        self.init_pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.PoseStamped, self.PoseCov = PoseStamped, PoseWithCovarianceStamped
        self.states = []

    # ------------------------------------------------------------------ ROS helpers
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_clock(self, m):
        self.wall_clock.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9))
        if len(self.wall_clock) > 400:
            self.wall_clock = self.wall_clock[-200:]

    def rtf(self, window=5.0):
        w = [p for p in self.wall_clock if p[0] > time.monotonic() - window]
        if len(w) < 2 or w[-1][0] - w[0][0] < 1.0:
            return None
        return (w[-1][1] - w[0][1]) / (w[-1][0] - w[0][0])

    def on_gt(self, m):
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gt = (p.x, p.y, yaw, tw.linear.x, tw.angular.z)
        self.gt_t = self.now()
        if self.cur_world is not None:
            self.ep_min_clr = min(self.ep_min_clr, self.cur_world.clearance(p.x, p.y) - self.robot_radius)

    def on_amcl(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.amcl = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_path(self, m):
        if len(m.poses) >= 2:
            p = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])
            self.plan_lengths.append(float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])))))

    def on_state(self, m):
        self.state = m.data
        self.states.append((self.now(), m.data))

    def spin(self, dt):
        t_end = time.monotonic() + dt
        while time.monotonic() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_sim(self, dt):
        t0 = self.now()
        w0 = time.monotonic()
        while self.now() - t0 < dt and time.monotonic() - w0 < dt * 5 + 5:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if cond():
                return True
        return False

    def set_params(self, node_name, params):
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        cli = self.node.create_client(SetParametersAtomically, f'{node_name}/set_parameters_atomically')
        if not cli.wait_for_service(timeout_sec=10.0):
            return 'service not available'
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in params.items()]
        fut = cli.call_async(req)
        self.wait(lambda: fut.done(), 10.0)
        if not fut.done():
            return 'timeout'
        r = fut.result().result
        return '' if r.successful else r.reason

    def call_trigger(self, srv):
        from std_srvs.srv import Trigger
        cli = self.node.create_client(Trigger, srv)
        if cli.wait_for_service(timeout_sec=5.0):
            fut = cli.call_async(Trigger.Request())
            self.wait(lambda: fut.done(), 10.0)

    def clear_costmaps(self):
        from nav2_msgs.srv import ClearEntireCostmap
        for s in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            cli = self.node.create_client(ClearEntireCostmap, s)
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(ClearEntireCostmap.Request())
                self.wait(lambda: fut.done(), 10.0)

    # ------------------------------------------------------------------ Gazebo helpers
    def world_name(self):
        import re
        for _ in range(20):
            out = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True, timeout=10).stdout
            names = sorted(set(re.findall(r'^/world/([^/]+)/create$', out, re.M)))
            if names:
                return names[0]
            time.sleep(1.0)
        return None

    def spawn(self, world, name, shp, h=0.6):
        if shp['type'] == 'circle':
            geo = f'<cylinder><radius>{shp["r"]}</radius><length>{h}</length></cylinder>'
            yaw = 0.0
        else:
            geo = f'<box><size>{shp["sx"]} {shp["sy"]} {h}</size></box>'
            yaw = shp.get('yaw', 0.0)
        sdf = (f'<sdf version="1.8"><model name="{name}"><static>true</static><link name="link">'
               f'<collision name="c"><geometry>{geo}</geometry></collision>'
               f'<visual name="v"><geometry>{geo}</geometry><material><diffuse>0.9 0.2 0.2 1</diffuse></material>'
               '</visual></link></model></sdf>')
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', world, '-name', name, '-string', sdf,
                        '-x', str(shp['x']), '-y', str(shp['y']), '-z', str(h / 2), '-Y', str(yaw)],
                       capture_output=True, timeout=30)

    def remove(self, world, name):
        subprocess.run(['ign', 'service', '-s', f'/world/{world}/remove', '--reqtype', 'ignition.msgs.Entity',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000',
                        '--req', f'name: "{name}" type: MODEL'], capture_output=True, timeout=10)

    def teleport(self, world, x, y, yaw):
        qz, qw = quat_z(yaw)
        req = f'name: "{ROBOT}" position {{x: {x} y: {y} z: 0.0}} orientation {{z: {qz:.6f} w: {qw:.6f}}}'
        subprocess.run(['ign', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, timeout=10)

    def init_pose(self, x, y, yaw):
        m = self.PoseCov()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = quat_z(yaw)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.0025
        m.pose.covariance[35] = 0.0076
        for _ in range(3):
            self.init_pub.publish(m)
            self.spin(0.3)

    # ------------------------------------------------------------------ episode
    def episode(self, sc, world, M, ev, sim_cfg, localization):
        shapes = sc['obstacles']
        self.robot_radius = sim_cfg['robot_radius']
        w_gt = fast2d.World(RN.scenario_to_fast2d(sc))
        names = [f'd_{sc["id"]}_{k}' for k in range(len(shapes))]
        for nm, s in zip(names, shapes):
            self.spawn(world, nm, s)
        self.spin(1.5)
        self.states = []
        self.plan_lengths = []
        gx, gy, gyaw = sc['goal']
        g = self.PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = gx, gy
        g.pose.orientation.z, g.pose.orientation.w = quat_z(gyaw)
        t0 = self.now()
        self.ep_min_clr = math.inf
        self.cur_world = w_gt
        self.goal_pub.publish(g)
        T = sc['timeout']
        trace, events = [], []
        terminal, t_term, min_clr, infra = 'none', -1.0, math.inf, ''
        loc_err = None
        last_row = -1.0
        low_rtf_since = None
        hist = []
        while True:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            t = self.now() - t0
            if self.gt is None:
                continue
            x, y, yaw, v, w = self.gt
            if t - last_row >= 0.1:
                last_row = t
                c = w_gt.clearance(x, y) - sim_cfg['robot_radius']
                min_clr = min(min_clr, c, self.ep_min_clr)
                trace.append([t, x, y, yaw, v, w, c])
                hist.append((t, x, y))
            while self.states:
                ts, s = self.states.pop(0)
                events.append((round(ts - t0, 2), s))
                if terminal == 'none' and ts > t0 and s.startswith(('SUCCEEDED', 'ABORTED', 'CANCELED', 'FAILED',
                                                                   'REFUSED', 'REJECTED')):
                    terminal, t_term = s, ts - t0
                    # sai số định vị (AMCL - GT) lúc kết thúc: tách lỗi định vị khỏi lỗi planner
                    loc_err = math.hypot(self.amcl[0] - x, self.amcl[1] - y) if self.amcl else None
            # dừng
            if terminal != 'none' and t - t_term >= 2.5:
                break
            if min_clr < -ev['collision_tolerance'] and t > 0.5 and trace and trace[-1][6] < 0:
                if not any(e[1] == 'COLLISION (GT)' for e in events):
                    events.append((round(t, 2), 'COLLISION (GT)'))
                    t_coll = t
                elif t - t_coll > 0.5:
                    break
            if t > T:
                break
            if hist and t - hist[0][0] > ev['stuck_window']:
                hist = [h for h in hist if t - h[0] <= ev['stuck_window']]
                if max(math.hypot(h[1] - x, h[2] - y) for h in hist) < ev['stuck_radius'] and terminal == 'none':
                    break
            if t > 15.0 and not events:
                infra = 'bridge published no state in 15 s (map / localization / planner not up)'
                break
            if self.gt_t is not None and self.now() - self.gt_t > ev['sensor_gap_error']:
                infra = 'ground truth odom silent'
                break
            if self.scan_t is not None and self.now() - self.scan_t > ev['sensor_gap_error']:
                infra = 'scan silent > 2 s'
                break
            r = self.rtf()
            if r is not None and r < ev['rtf_min']:
                low_rtf_since = low_rtf_since or time.monotonic()
                if time.monotonic() - low_rtf_since > ev['rtf_window']:
                    infra = f'RTF {r:.2f} < {ev["rtf_min"]}'
                    break
            else:
                low_rtf_since = None
        self.cur_world = None
        min_clr = min(min_clr, self.ep_min_clr)
        self.call_trigger('/goal_path_bridge/cancel')
        for nm in names:
            self.remove(world, nm)
        rec = {'sim': 'gazebo', 'feasibility': sc['feasibility'], 'goal': sc['goal'], 'timeout': T,
               'scenario_id': sc['id'], 'terminal_state': terminal, 't_terminal': t_term,
               'trace': np.array(trace), 'min_clearance': min_clr, 'infra_error': infra, 'timed_out': t > T,
               'replans': sum(1 for e in events if e[1].startswith('REPLANNING')), 'events': events,
               'plan_lengths': list(self.plan_lengths), 'loc_error_at_terminal': loc_err,
               'initial_path_length': self.plan_lengths[0] if self.plan_lengths else 0.0,
               'localization': localization}
        rec['eval'] = E.evaluate(rec, ev)
        return rec

    def reset_to(self, world, start):
        self.call_trigger('/goal_path_bridge/cancel')
        self.spin(1.0)
        self.teleport(world, *start)
        self.spin(1.0)
        self.init_pose(*start)
        self.spin(2.0)
        self.clear_costmaps()
        self.spin(2.0)


def launch_stack(map_yaml, start, domain, mode, log_path):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', IGN_PARTITION=f'gzep{domain}')
    monitor = 'true' if mode == 'full' else 'false'
    source = 'ground_truth' if mode == 'planner_gt' else 'amcl'
    cmd = ['ros2', 'launch', 'stage4_gazebo', 's3_navigation.launch.py', 'env:=map', f'world_map:={map_yaml}',
           f'localization_source:={source}',
           'gui:=false', 'rviz:=false', 'joy:=false', 'csm:=false', f'monitor:={monitor}', 'init_pose:=true',
           f'x:={start[0]}', f'y:={start[1]}', f'yaw:={start[2]}']
    f = open(log_path, 'w')
    return subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT, preexec_fn=os.setsid), env


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--ids', required=True)
    ap.add_argument('--param-sets', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--mode', default='planner_gt', choices=['planner_gt', 'planner', 'full'])
    ap.add_argument('--domain', type=int, default=91)
    ap.add_argument('--repeat', type=int, default=1)
    a = ap.parse_args()
    cfg = C.load()
    ev = cfg['eval']
    sets = json.load(open(a.param_sets)) if a.param_sets else {'base': {}}
    pool = {s['id']: s for s in IO.load_split(a.manifest)}
    ids = a.ids.split(',')
    os.environ.update(ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY='1', IGN_PARTITION=f'gzep{a.domain}')
    out = open(a.out, 'a')
    tmp = os.path.join(os.path.dirname(os.path.abspath(a.out)), 'gz_maps')
    for sid in ids:
        sc = pool[sid]
        M = sc['map']
        map_yaml = IO.write_map_yaml(M, tmp, sc['map_id'])
        proc, env = launch_stack(map_yaml, sc['start'], a.domain, a.mode, os.path.join(tmp, f'{sid}_launch.log'))
        g = GzEpisodes(a)
        try:
            ok = g.wait(lambda: g.gt is not None and g.scan_t is not None, 120.0)
            world = g.world_name() if ok else None
            if not ok or world is None:
                rec = {'scenario_id': sid, 'sim': 'gazebo', 'eval': {'label': 'SYSTEM_ERROR',
                                                                     'sub_reason': 'stack did not start'}}
                out.write(json.dumps(rec) + '\n')
                out.flush()
                continue
            g.spin(8.0)                                  # AMCL nhận pose spawn (init_pose:=true)
            if a.mode in ('planner', 'planner_gt'):
                g.set_params('/goal_path_bridge', {'require_localized': False,
                                                   'follow_timeout': float(sc['timeout'] + 10.0)})
            else:
                g.set_params('/goal_path_bridge', {'follow_timeout': float(sc['timeout'] + 10.0)})
            for rep in range(a.repeat):
                for sname, params in sets.items():
                    base = {f'CustomDWB.{k}': float(cfg['dwb'][k]) for k in ACTION_KEYS}
                    base.update({f'CustomDWB.{k}': float(v) for k, v in params.items()})
                    why = g.set_params('/controller_server', base)
                    g.reset_to(world, sc['start'])
                    t0 = time.monotonic()
                    rec = g.episode(sc, world, M, ev, cfg['sim'],
                                    'perfect' if a.mode == 'planner_gt' else 'amcl')
                    rec.update({'param_set': sname, 'params': params, 'param_error': why, 'repeat': rep,
                                'wall_s': time.monotonic() - t0, 'mode': a.mode})
                    tr = rec.pop('trace')
                    rec['trace_len'] = len(tr)
                    # V4: lưu trace gọn (t, x, y, yaw, v, w, hở) 10 Hz để chấm lại khi evaluator đổi
                    rec['trace'] = np.round(np.asarray(tr, float), 4).tolist() if len(tr) else []
                    out.write(json.dumps(rec, default=float) + '\n')
                    out.flush()
                    print(sid, sname, rep, rec['eval']['label'], rec['eval'].get('sub_reason', ''),
                          rec['terminal_state'][:40], round(rec['min_clearance'], 3), flush=True)
        finally:
            g.node.destroy_node()
            g.rclpy.shutdown()
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            time.sleep(3.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
