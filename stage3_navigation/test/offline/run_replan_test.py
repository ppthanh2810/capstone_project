#!/usr/bin/env python3
# Copyright 2026 capstone_project.
# Kiểm thử replanning OFFLINE — KHÔNG điều khiển phần cứng.
"""
Kiểm thử replanning của goal_path_bridge trong graph ROS cách ly (không phần cứng).

Mỗi kịch bản (scenarios_replan.json):
  ros2 launch stage3_navigation navigation.launch.py stage2:=false rviz:=false monitor:=false
      navigation_enabled:=false auto_follow:=true replan:=<true|false>
  + offline_sim.py (bản đồ, TF, odom, scan mô phỏng; nghe /cmd_vel_dwb)
  -> kiểm graph (không node phần cứng, /cmd_vel_nav không có publisher) -> publish /goal_pose
  -> ghi /goal_path_bridge/follow_state, /global_path tới trạng thái cuối -> phân tích sim.csv.
ROS_DOMAIN_ID riêng (mặc định 161) + ROS_LOCALHOST_ONLY=1.

  python3 run_replan_test.py --out <dir> [--scenario replan_detour ...] [--replan false]
Mã thoát != 0 nếu một kịch bản sai kỳ vọng (trạng thái cuối, số lần replan, va chạm).
"""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
FORBIDDEN_NODES = ('zlac8015d_move', 'twist_mux', 'joy_node', 'joy_teleop', 'teleop', 'sllidar',
                   'imu_odom', 'ekf_filter_node', 'amcl', 'slam_toolbox', 'zlac8015d_odom')
TERMINAL = ('SUCCEEDED', 'FAILED', 'CANCELED', 'ABORTED', 'REJECTED', 'REFUSED')


def kill_group(p, sig=signal.SIGINT, wait=15.0):
    if p.poll() is not None:
        return
    try:
        os.killpg(p.pid, sig)
        p.wait(timeout=wait)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait()
    except ProcessLookupError:
        pass


def client(name, sc, timeout):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from nav_msgs.msg import Path as PathMsg
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    out = {'scenario': name, 'states': [], 'paths': [], 'status': 'setup_failed'}
    rclpy.init()
    node = rclpy.create_node('replan_test_client')
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    t0 = time.monotonic()
    node.create_subscription(
        String, '/goal_path_bridge/follow_state',
        lambda m: out['states'].append((round(time.monotonic() - t0, 2), m.data)), latched)

    def on_path(m):
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                b.pose.position.y - a.pose.position.y)
                     for a, b in zip(m.poses, m.poses[1:]))
        out['paths'].append((round(time.monotonic() - t0, 2), len(m.poses), round(length, 2)))
    node.create_subscription(PathMsg, '/global_path', on_path, latched)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    follow = ActionClient(node, FollowPath, '/follow_path')
    plan = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        tw = time.monotonic()
        while not (follow.server_is_ready() and plan.server_is_ready()) and \
                time.monotonic() - tw < 60.0:
            spin_for(0.2)
        if not (follow.server_is_ready() and plan.server_is_ready()):
            out['status'] = 'setup_failed: actions not available'
            return out
        # Chờ lifecycle active (Fast DDS đôi khi làm mất phản hồi change_state lúc khởi động
        # -> planner_server không bao giờ active).
        from lifecycle_msgs.srv import GetState
        for srv in ('/planner_server/get_state', '/controller_server/get_state'):
            gc = node.create_client(GetState, srv)
            active = False
            while not active and time.monotonic() - tw < 60.0:
                if gc.wait_for_service(timeout_sec=1.0):
                    f = gc.call_async(GetState.Request())
                    while not f.done():
                        spin_for(0.02)
                    active = f.result().current_state.label == 'active'
                if not active:
                    spin_for(0.5)
            if not active:
                out['status'] = f'setup_failed: {srv} not active'
                return out
        spin_for(4.0)   # costmap nhận scan, bridge nhận /map + costmap
        names = node.get_node_names()
        bad = [n for n in names if any(f in n for f in FORBIDDEN_NODES)]
        pubs_nav = node.count_publishers('/cmd_vel_nav')
        pubs_dwb = [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel_dwb')]
        out['graph'] = {'forbidden': bad, 'cmd_vel_nav_publishers': pubs_nav,
                        'cmd_vel_dwb_publishers': pubs_dwb,
                        'domain': os.environ.get('ROS_DOMAIN_ID'),
                        'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY')}
        if bad or pubs_nav != 0 or pubs_dwb != ['controller_server'] or \
                os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            out['status'] = 'setup_failed: graph check'
            return out

        gx, gy, gyaw = sc['goal']
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = node.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(gx), float(gy)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        n_before = len(out['states'])
        out['goal_time'] = round(time.monotonic() - t0, 2)
        goal_pub.publish(g)
        out['status'] = 'timeout'
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            spin_for(0.1)
            new = [s for _, s in out['states'][n_before:]]
            if any(s.startswith('FOLLOW') for s in new) and new and \
                    new[-1].startswith(TERMINAL):
                out['status'] = 'done'
                break
            if new and new[-1].startswith('REFUSED'):
                out['status'] = 'done'
                break
        spin_for(1.0)
        return out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def analyze(sc, res, csv_path):
    rows = []
    if csv_path.exists():
        with open(csv_path) as f:
            rows = list(csv.DictReader(f))
    clear = [float(r['clearance']) for r in rows if r['clearance'] != 'inf']
    states = [s for _, s in res.get('states', [])]
    final = states[-1] if states else ''
    obstacle_replans = sum(1 for s in states if s.startswith('REPLANNING: obstacle'))
    all_replans = sum(1 for s in states if s.startswith('REPLANNING'))
    gx, gy, _ = sc['goal']
    last = rows[-1] if rows else None
    dist_goal = math.hypot(float(last['x']) - gx, float(last['y']) - gy) if last else math.nan
    max_v = max((abs(float(r['cmd_v'])) for r in rows), default=0.0)
    min_v = min((float(r['cmd_v']) for r in rows), default=0.0)
    max_w = max((abs(float(r['cmd_w'])) for r in rows), default=0.0)
    m = {
        'status': res.get('status'), 'final_state': final,
        'obstacle_replans': obstacle_replans, 'all_replans': all_replans,
        'paths_published': len(res.get('paths', [])),
        'min_clearance': round(min(clear), 3) if clear else None,
        'collision': bool(clear) and min(clear) < 0.0,
        'final_dist_to_goal': round(dist_goal, 3),
        'max_cmd_v': round(max_v, 3), 'min_cmd_v': round(min_v, 3), 'max_cmd_w': round(max_w, 3),
        'duration': round(res['states'][-1][0] - res.get('goal_time', 0.0), 1)
        if res.get('states') else None,
    }
    ok = (res.get('status') == 'done' and not m['collision'] and
          any(final.startswith(p) for p in sc['expect_state'].split('|')) and
          obstacle_replans >= sc.get('min_replans', 0) and
          obstacle_replans <= sc.get('max_replans', 10 ** 6) and
          max_v <= 0.15 + 1e-6 and min_v >= -1e-6 and max_w <= 0.5 + 1e-6)
    m['ok'] = ok
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--scenario', nargs='*')
    ap.add_argument('--scenario-file', default=str(HERE / 'scenarios_replan.json'))
    ap.add_argument('--replan', default='true')
    ap.add_argument('--domain', default='161')
    ap.add_argument('--timeout', type=float, default=150.0)
    ap.add_argument('--client', help=argparse.SUPPRESS)
    ap.add_argument('--client-out', help=argparse.SUPPRESS)
    args = ap.parse_args()
    scen = json.load(open(args.scenario_file))['scenarios']

    if args.client:
        res = client(args.client, scen[args.client], args.timeout)
        Path(args.client_out).write_text(json.dumps(res, indent=1))
        return 0

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=args.domain, ROS_LOCALHOST_ONLY='1', PYTHONUNBUFFERED='1')
    names = args.scenario or list(scen)
    summary = {}
    for name in names:
        for attempt in range(3):
            summary[name] = run_one(args, env, out, name, scen[name])
            if not str(summary[name]['status']).startswith('setup_failed'):
                break
            print(name, 'setup failed, retrying', flush=True)
        print(name, json.dumps(summary[name]), flush=True)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    return 0 if all(m['ok'] for m in summary.values()) else 1


def run_one(args, env, out, name, sc):
    rd = out / name
    rd.mkdir(exist_ok=True)
    (rd / 'client.json').unlink(missing_ok=True)
    launch_log = open(rd / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'stage3_navigation', 'navigation.launch.py', 'stage2:=false',
         'rviz:=false', 'monitor:=false', 'navigation_enabled:=false', 'auto_follow:=true',
         f'replan:={args.replan}'],
        stdout=launch_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    sim = subprocess.Popen(
        [sys.executable, str(HERE / 'offline_sim.py'), '--ros-args',
         '-p', f'scenario:={name}', '-p', f'scenario_file:={args.scenario_file}',
         '-p', f'log_file:={rd / "sim.csv"}', '-p', 'latency:=0.2'],
        stdout=open(rd / 'sim.log', 'w'), stderr=subprocess.STDOUT, env=env,
        start_new_session=True)
    try:
        subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), '--out', str(out),
             '--client', name, '--client-out', str(rd / 'client.json'),
             '--scenario-file', args.scenario_file, '--timeout', str(args.timeout)],
            env=env, timeout=args.timeout + 90, stdout=open(rd / 'client.log', 'w'),
            stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        pass
    finally:
        kill_group(sim)
        kill_group(launch)
        launch_log.close()
    try:
        res = json.loads((rd / 'client.json').read_text())
    except (OSError, ValueError):
        res = {'status': 'client_failed'}
    return analyze(sc, res, rd / 'sim.csv')


if __name__ == '__main__':
    sys.exit(main())
