#!/usr/bin/env python3
# Copyright 2026 capstone_project.
# Benchmark offline Stage 3.2 — KHÔNG điều khiển phần cứng.
"""
Benchmark OFFLINE kín vòng: CustomDWB vs Nav2 DWB (Standard, LimitedAccel) trên controller_server.

Mỗi lần chạy (scenario x profile x run) dựng một graph ROS MỚI, cách ly:
  ROS_DOMAIN_ID riêng cho mỗi worker + ROS_LOCALHOST_ONLY=1,
  ros2 launch stage3_navigation navigation.launch.py stage2:=false rviz:=false
      navigation_enabled:=false auto_follow:=false
      params_file:=<bản sao nav2_params.yaml + profile FollowPathLA>
  + offline_sim.py (TF, /odometry/filtered, /scan_filtered, /map; robot đọc /cmd_vel_dwb)
  + tiến trình client (chính file này với --client): kiểm graph (không có node motor/teleop/
    driver), gửi FollowPath trực tiếp tới /follow_path với controller_id của profile, ghi lệnh.
KHÔNG có stage2 (không LiDAR/IMU/Modbus/PS4), KHÔNG có twist_mux / zlac8015d_move trong graph.

  python3 run_benchmark.py --out <dir> [--runs 5] [--workers 3]
      [--profiles custom,nav2_dwb,nav2_dwb_la] [--scenarios straight,l_turn,...]
Kết quả: <dir>/runs.csv, <dir>/summary.json, <dir>/summary.md, <dir>/nav2_params_benchmark.yaml,
  <dir>/raw/<scenario>__<profile>__<run>/{sim.csv,client.json,launch.log}
"""

import argparse
import concurrent.futures as cf
import copy
import csv
import json
import math
import os
from pathlib import Path
import platform
import signal
import statistics
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PROFILES = {'custom': 'CustomDWB', 'nav2_dwb': 'FollowPath', 'nav2_dwb_la': 'FollowPathLA'}
FORBIDDEN_NODES = ('zlac8015d_move', 'twist_mux', 'joy_node', 'joy_teleop', 'teleop', 'sllidar',
                   'imu_odom', 'ekf_filter_node', 'amcl', 'slam_toolbox', 'zlac8015d_odom',
                   'realsense')
LIMITS = {'max_vel_x': 0.15, 'max_vel_theta': 0.5}


# ============================================================================== đường đi
def build_path(sc):
    if sc.get('s_curve'):
        wp = [(0.1 * i, 0.5 * math.sin(0.1 * i * math.pi / 1.5)) for i in range(46)]
    else:
        wp = [(float(p[0]), float(p[1])) for p in sc['waypoints']]
    pts = [wp[0]]
    for (x0, y0), (x1, y1) in zip(wp[:-1], wp[1:]):
        n = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / 0.05))
        pts += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1)]
    return pts


def dist_to_polyline(x, y, pts):
    best = math.inf
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        u = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        best = min(best, math.hypot(ax + u * dx - x, ay + u * dy - y))
    return best


# ======================================================================= client (graph cách ly)
def client_main(args):
    import rclpy
    from action_msgs.msg import GoalStatus
    from diagnostic_msgs.msg import DiagnosticStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry, Path as PathMsg
    from rclpy.action import ActionClient
    from std_msgs.msg import Float64

    sc = json.load(open(args.scenario_file))['scenarios'][args.scenario]
    out = {'scenario': args.scenario, 'profile': args.profile, 'run': args.run, 'cmds': [],
           'compute_ms': [], 'phases': [], 'status': 'setup_failed', 'graph_ok': False}
    rclpy.init()
    node = rclpy.create_node('benchmark_client')
    node.create_subscription(
        Twist, '/cmd_vel_dwb',
        lambda m: out['cmds'].append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    odom_seen = []
    node.create_subscription(Odometry, '/odometry/filtered', lambda m: odom_seen.append(1), 10)

    def on_state(m):
        kv = {e.key: e.value for e in m.values}
        if 'compute_ms' in kv and kv.get('phase') != 'BLOCKED':
            out['compute_ms'].append(float(kv['compute_ms']))
        out['phases'].append((time.monotonic(), kv.get('phase', ''), m.message[:160]))
    node.create_subscription(DiagnosticStatus, '/CustomDWB/state', on_state, 50)
    kick = node.create_publisher(Float64, '/sim/kick', 10)
    client = ActionClient(node, FollowPath, '/follow_path')

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        t_wait = time.monotonic()
        while not client.server_is_ready() and time.monotonic() - t_wait < 60.0:
            spin_for(0.2)
        if not client.server_is_ready():
            out['status'] = 'setup_failed: /follow_path not available'
            return out
        spin_for(3.0)   # costmap nhận vài scan
        names = node.get_node_names()
        bad = [n for n in names if any(f in n for f in FORBIDDEN_NODES)]
        pubs_nav = node.count_publishers('/cmd_vel_nav')
        pubs_dwb = [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel_dwb')]
        out['graph'] = {'nodes': sorted(names), 'forbidden': bad,
                        'cmd_vel_nav_publishers': pubs_nav,
                        'cmd_vel_dwb_publishers': pubs_dwb,
                        'domain': os.environ.get('ROS_DOMAIN_ID'),
                        'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY')}
        out['graph_ok'] = (not bad and pubs_nav == 0 and pubs_dwb == ['controller_server'] and
                           os.environ.get('ROS_LOCALHOST_ONLY') == '1')
        if not out['graph_ok']:
            out['status'] = 'setup_failed: graph check'
            return out
        if not odom_seen:
            out['status'] = 'setup_failed: no odometry'
            return out
        # Ghi lại cấu hình THỰC SỰ đã nạp trong controller_server (không chỉ YAML).
        from rcl_interfaces.srv import GetParameters
        pc = node.create_client(GetParameters, '/controller_server/get_parameters')
        plugin = PROFILES[args.profile]
        keys_q = ('plugin', 'trajectory_generator_name', 'sim_period', 'max_vel_x', 'acc_lim_x',
                  'control_latency', 'sim_time')
        names_q = [f'{plugin}.{k}' for k in keys_q]
        out['loaded_params'] = {}
        if pc.wait_for_service(timeout_sec=5.0):
            # Hỏi từng tham số: một tham số chưa khai báo làm rỗng cả phản hồi.
            for name in names_q:
                pf = pc.call_async(GetParameters.Request(names=[name]))
                while not pf.done():
                    spin_for(0.02)
                for v in pf.result().values:
                    out['loaded_params'][name] = {
                        1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                        4: v.string_value}.get(v.type, None)

        path = PathMsg()
        path.header.frame_id = 'map'
        path.header.stamp = node.get_clock().now().to_msg()
        pts = build_path(sc)
        for i, (x, y) in enumerate(pts):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            yaw = sc['goal_yaw'] if i == len(pts) - 1 else 0.0
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = PROFILES[args.profile]
        goal.goal_checker_id = 'general_goal_checker'

        kick.publish(Float64(data=float(sc.get('kick_v', 0.0))))
        spin_for(0.05)
        out['t_goal'] = time.monotonic()
        fut = client.send_goal_async(goal)
        while not fut.done():
            spin_for(0.02)
        handle = fut.result()
        if not handle.accepted:
            out['status'] = 'rejected'
            return out
        res_fut = handle.get_result_async()
        timeout = 25.0 if 'scan_stop_after' in sc else 120.0
        cancel_after = sc.get('cancel_after')
        canceled = False
        while not res_fut.done():
            spin_for(0.02)
            el = time.monotonic() - out['t_goal']
            if cancel_after is not None and not canceled and el > cancel_after:
                out['t_cancel'] = time.monotonic()
                handle.cancel_goal_async()
                canceled = True
            if el > timeout and not canceled:
                out['t_cancel'] = time.monotonic()
                out['cancel_reason'] = 'benchmark timeout'
                handle.cancel_goal_async()
                canceled = True
            if el > timeout + 10.0:
                break
        if res_fut.done():
            st = res_fut.result().status
            out['status'] = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                             GoalStatus.STATUS_ABORTED: 'aborted',
                             GoalStatus.STATUS_CANCELED: 'canceled'}.get(st, f'status_{st}')
        else:
            out['status'] = 'hung (no action result, cancel not processed)'
        out['t_result'] = time.monotonic()
        spin_for(3.0)   # ghi tiếp lệnh 0 cuối
        return out
    finally:
        with open(args.client_out, 'w') as f:
            json.dump(out, f)
        node.destroy_node()
        rclpy.shutdown()


# ============================================================================== một lần chạy
def derived_params(out_dir):
    import yaml
    src = Path(os.environ.get('STAGE3_PARAMS', '')) if os.environ.get('STAGE3_PARAMS') else None
    if not src:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('stage3_navigation'))
        src = share / 'config' / 'nav2_params.yaml'
    d = yaml.safe_load(open(src))
    cs = d['controller_server']['ros__parameters']
    la = copy.deepcopy(cs['FollowPath'])
    la['trajectory_generator_name'] = 'dwb_plugins::LimitedAccelGenerator'
    la['sim_period'] = 0.1   # = 1 / controller_frequency (10 Hz)
    cs['FollowPathLA'] = la
    cs['controller_plugins'] = ['FollowPath', 'FollowPathLA', 'CustomDWB']
    cs['CustomDWB']['debug_publish_rate'] = 10.0       # state mỗi chu kỳ (đo compute_ms)
    cs['CustomDWB']['publish_trajectories'] = False    # không tốn thời gian vẽ marker
    dst = Path(out_dir) / 'nav2_params_benchmark.yaml'
    with open(dst, 'w') as f:
        f.write(f'# Sinh tự động bởi run_benchmark.py từ {src}\n')
        yaml.safe_dump(d, f, sort_keys=False)
    return dst, src


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


def run_job(job):
    rd = Path(job['raw_dir'])
    rd.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(job['domain']), ROS_LOCALHOST_ONLY='1',
               PYTHONUNBUFFERED='1')
    launch_log = open(rd / 'launch.log', 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'stage3_navigation', 'navigation.launch.py', 'stage2:=false',
         'rviz:=false', 'navigation_enabled:=false', 'auto_follow:=false', 'camera:=false',
         f'camera_layer:={job["camera_layer"]}', f'params_file:={job["params"]}'],
        stdout=launch_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    sim = subprocess.Popen(
        [sys.executable, str(HERE / 'offline_sim.py'), '--ros-args',
         '-p', f'scenario:={job["scenario"]}', '-p', f'scenario_file:={HERE / "scenarios.json"}',
         '-p', f'seed:={job["seed"]}', '-p', f'log_file:={rd / "sim.csv"}',
         '-p', f'camera:={job["camera_layer"] != "none"}'],
        stdout=open(rd / 'sim.log', 'w'), stderr=subprocess.STDOUT, env=env,
        start_new_session=True)
    try:
        subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), '--client',
             '--scenario', job['scenario'], '--profile', job['profile'], '--run', str(job['run']),
             '--scenario-file', str(HERE / 'scenarios.json'),
             '--client-out', str(rd / 'client.json')],
            env=env, timeout=240, stdout=open(rd / 'client.log', 'w'), stderr=subprocess.STDOUT)
    except subprocess.TimeoutExpired:
        pass
    finally:
        kill_group(sim)
        kill_group(launch)
        launch_log.close()
    return analyze(job)


# ============================================================================== phân tích
def pct(v, q):
    if not v:
        return float('nan')
    v = sorted(v)
    i = q * (len(v) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (i - lo) * (v[hi] - v[lo])


def analyze(job):
    rd = Path(job['raw_dir'])
    sc = job['sc']
    m = {'scenario': job['scenario'], 'profile': job['profile'], 'run': job['run'],
         'seed': job['seed'], 'expect': sc['expect']}
    try:
        c = json.load(open(rd / 'client.json'))
    except (OSError, ValueError):
        m['status'] = 'no client output'
        return m
    m['status'] = c['status']
    m['graph_ok'] = c.get('graph_ok', False)
    rows = []
    try:
        with open(rd / 'sim.csv') as f:
            rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    except (OSError, ValueError):
        pass
    t_goal = c.get('t_goal')
    t_end = c.get('t_result', t_goal)
    if not rows or t_goal is None:
        return m
    pts = build_path(sc)
    gx, gy = pts[-1]
    act = [r for r in rows if t_goal <= r['mono'] <= t_end]
    clear = [r['clearance'] for r in rows if r['mono'] >= t_goal]
    m['collision'] = any(x < 0.0 for x in clear)
    m['min_clearance_m'] = min(clear) if clear else float('inf')
    ct = [dist_to_polyline(r['x'], r['y'], pts) for r in act]
    m['cross_track_rms_m'] = math.sqrt(sum(d * d for d in ct) / len(ct)) if ct else float('nan')
    m['cross_track_max_m'] = max(ct) if ct else float('nan')
    m['time_s'] = t_end - t_goal
    last = rows[-1]
    m['final_xy_error_m'] = math.hypot(last['x'] - gx, last['y'] - gy)
    m['final_yaw_error_rad'] = abs(math.atan2(math.sin(sc['goal_yaw'] - last['theta']),
                                              math.cos(sc['goal_yaw'] - last['theta'])))
    cmds = [x for x in c['cmds'] if x[0] >= t_goal]
    m['n_cmds'] = len(cmds)
    m['nonfinite_cmds'] = sum(1 for _, v, w in cmds if not (math.isfinite(v) and math.isfinite(w)))
    m['limit_violations'] = sum(
        1 for _, v, w in cmds
        if v > LIMITS['max_vel_x'] + 1e-6 or v < -1e-6 or abs(w) > LIMITS['max_vel_theta'] + 1e-6)
    m['reverse_cmds'] = sum(1 for _, v, _w in cmds if v < -1e-6)
    flips, last_s = 0, 0
    for _, _v, w in cmds:
        s = 0 if abs(w) <= 0.05 else (1 if w > 0 else -1)
        if s and last_s and s != last_s:
            flips += 1
        last_s = s or last_s
    m['w_sign_flips'] = flips
    period = [b[0] - a[0] for a, b in zip(cmds[:-1], cmds[1:]) if a[0] < t_end]
    m['cmd_period_p50_ms'] = 1000 * pct(period, 0.5)
    m['cmd_period_p99_ms'] = 1000 * pct(period, 0.99)
    m['cmd_period_max_ms'] = 1000 * pct(period, 1.0)
    ms = c.get('compute_ms', [])
    m['compute_p50_ms'] = pct(ms, 0.5)
    m['compute_p95_ms'] = pct(ms, 0.95)
    m['compute_p99_ms'] = pct(ms, 0.99)
    m['compute_max_ms'] = pct(ms, 1.0)
    m['compute_over_100ms'] = sum(1 for x in ms if x > 100.0)
    log = (rd / 'launch.log').read_text(errors='replace')
    m['missed_rate_warnings'] = log.count('missed its desired rate')
    # Sự kiện dừng: cancel / dữ liệu ngừng / kết quả (abort).
    # Đo tới lệnh != 0 cuối cùng và tới khi robot dừng.
    event = None
    if 'cancel_after' in sc and c.get('t_cancel'):
        event = c['t_cancel']
    elif 'odom_stop_after' in sc:
        event = t_goal + sc['odom_stop_after']
    elif 'scan_stop_after' in sc:
        event = t_goal + sc['scan_stop_after']
    elif m['status'] == 'aborted':
        event = t_end
    if event is not None:
        nz = [t for t, v, w in cmds if t >= event and (abs(v) > 1e-6 or abs(w) > 1e-6)]
        m['last_nonzero_cmd_after_event_s'] = (max(nz) - event) if nz else 0.0
        stop = next((r['mono'] for r in rows if r['mono'] >= event and abs(r['v']) < 0.01 and
                     abs(r['w']) < 0.01), None)
        m['robot_stop_after_event_s'] = (stop - event) if stop else float('nan')
        ev_row = min(rows, key=lambda r: abs(r['mono'] - event))
        st_row = next((r for r in rows if stop and r['mono'] >= stop), rows[-1])
        m['stop_distance_m'] = math.hypot(st_row['x'] - ev_row['x'], st_row['y'] - ev_row['y'])
    ok = {'success': m['status'] == 'succeeded', 'stop': m['status'] != 'succeeded',
          'any': True}[sc['expect']]
    m['meets_expectation'] = bool(ok and not m['collision'] and m['limit_violations'] == 0 and
                                  m['nonfinite_cmds'] == 0 and m['graph_ok'])
    return m


def summarize(results, out):
    keys = sorted({(r['scenario'], r['profile']) for r in results})
    table, js = [], []
    for sc, pr in keys:
        rs = [r for r in results if r['scenario'] == sc and r['profile'] == pr]

        def vals(k):
            return [r[k] for r in rs if isinstance(r.get(k), (int, float)) and math.isfinite(r[k])]

        def mean(k):
            v = vals(k)
            return statistics.mean(v) if v else float('nan')
        row = {'scenario': sc, 'profile': pr, 'runs': len(rs),
               'succeeded': sum(r.get('status') == 'succeeded' for r in rs),
               'statuses': sorted({r.get('status', '?') for r in rs}),
               'meets_expectation': sum(bool(r.get('meets_expectation')) for r in rs),
               'collisions': sum(bool(r.get('collision')) for r in rs),
               'min_clearance_m': min(vals('min_clearance_m'), default=float('nan')),
               'mean_time_s': mean('time_s'), 'mean_ct_rms_m': mean('cross_track_rms_m'),
               'max_ct_m': max(vals('cross_track_max_m'), default=float('nan')),
               'mean_w_flips': mean('w_sign_flips'),
               'limit_violations': sum(r.get('limit_violations', 0) for r in rs),
               'mean_final_xy_m': mean('final_xy_error_m'),
               'max_last_nonzero_after_event_s': max(vals('last_nonzero_cmd_after_event_s'),
                                                     default=float('nan')),
               'max_robot_stop_after_event_s': max(vals('robot_stop_after_event_s'),
                                                   default=float('nan')),
               'max_stop_distance_m': max(vals('stop_distance_m'), default=float('nan')),
               'compute_p99_ms_max': max(vals('compute_p99_ms'), default=float('nan')),
               'compute_max_ms': max(vals('compute_max_ms'), default=float('nan')),
               'cmd_period_p99_ms_max': max(vals('cmd_period_p99_ms'), default=float('nan')),
               'missed_rate_warnings': sum(r.get('missed_rate_warnings', 0) for r in rs)}
        js.append(row)
        table.append(row)
    with open(Path(out) / 'summary.json', 'w') as f:
        machine = ' '.join((platform.node(), platform.machine(), platform.release()))
        json.dump({'machine': machine, 'python': sys.version.split()[0], 'rows': js}, f, indent=1)

    def f(x, d=2):
        return 'n/a' if not isinstance(x, (int, float)) or math.isnan(x) else f'{x:.{d}f}'
    lines = ['| scenario | profile | ok/exp | succeeded | statuses | coll | min clr m | t s | '
             'ct rms m | ct max m | ω flips | stop after event s | stop dist m | '
             'compute p99/max ms | cmd period p99 ms |',
             '|' + '---|' * 15]
    for r in table:
        lines.append(
            f"| {r['scenario']} | {r['profile']} | {r['meets_expectation']}/{r['runs']} | "
            f"{r['succeeded']} | "
            f"{', '.join(r['statuses'])} | {r['collisions']} | {f(r['min_clearance_m'], 3)} | "
            f"{f(r['mean_time_s'], 1)} | {f(r['mean_ct_rms_m'], 3)} | {f(r['max_ct_m'], 3)} | "
            f"{f(r['mean_w_flips'], 1)} | {f(r['max_robot_stop_after_event_s'])} | "
            f"{f(r['max_stop_distance_m'], 3)} | "
            f"{f(r['compute_p99_ms_max'])}/{f(r['compute_max_ms'])} | "
            f"{f(r['cmd_period_p99_ms_max'], 0)} |")
    (Path(out) / 'summary.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--client', action='store_true')
    ap.add_argument('--scenario')
    ap.add_argument('--profile')
    ap.add_argument('--run', type=int, default=0)
    ap.add_argument('--scenario-file', default=str(HERE / 'scenarios.json'))
    ap.add_argument('--client-out')
    ap.add_argument('--out', default='stage3_benchmark')
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--domain-base', type=int, default=150)
    ap.add_argument('--profiles', default='custom,nav2_dwb,nav2_dwb_la')
    ap.add_argument('--scenarios', default='')
    # M2: lớp camera trong local_costmap (camera giả lập của offline_sim, KHÔNG chạy RealSense).
    ap.add_argument('--camera-layer', default='none', choices=('none', 'fusion', 'voxel'))
    args = ap.parse_args()
    if args.client:
        client_main(args)
        return
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    params, src = derived_params(out)
    scen = json.load(open(HERE / 'scenarios.json'))['scenarios']
    names = [s for s in args.scenarios.split(',') if s] or list(scen)
    jobs = []
    for name in names:
        for prof in args.profiles.split(','):
            for r in range(args.runs):
                jobs.append({'scenario': name, 'profile': prof, 'run': r, 'seed': 1000 + r,
                             'sc': scen[name], 'params': str(params),
                             'camera_layer': args.camera_layer,
                             'raw_dir': str(out / 'raw' / f'{name}__{prof}__{r}')})
    meta = {'started': time.strftime('%Y-%m-%dT%H:%M:%S'), 'params_source': str(src),
            'jobs': len(jobs),
            'workers': args.workers, 'camera_layer': args.camera_layer,
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'machine': platform.node() + ' ' + platform.machine(), 'cpu_count': os.cpu_count(),
            'note': 'parallel workers share the CPU: timing metrics include that load'}
    (out / 'meta.json').write_text(json.dumps(meta, indent=1))
    results = []
    with cf.ThreadPoolExecutor(max_workers=args.workers) as ex:
        slots = list(range(args.workers))
        futs = {}
        pending = list(jobs)
        while pending or futs:
            while pending and slots:
                job = pending.pop(0)
                job['domain'] = args.domain_base + slots.pop(0)
                futs[ex.submit(run_job, job)] = job
            done, _ = cf.wait(futs, return_when=cf.FIRST_COMPLETED)
            for fu in done:
                job = futs.pop(fu)
                slots.append(job['domain'] - args.domain_base)
                try:
                    r = fu.result()
                except Exception as e:  # noqa: B902
                    r = {'scenario': job['scenario'], 'profile': job['profile'], 'run': job['run'],
                         'status': f'error: {e}'}
                results.append(r)
                print(f"[{len(results)}/{len(jobs)}] {r['scenario']} {r['profile']} #{r['run']}: "
                      f"{r.get('status')} clr={r.get('min_clearance_m')} "
                      f"ok={r.get('meets_expectation')}",
                      flush=True)
    keys = sorted({k for r in results for k in r})
    with open(out / 'runs.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in sorted(results, key=lambda r: (r['scenario'], r['profile'], r['run'])):
            w.writerow(r)
    summarize(results, out)


if __name__ == '__main__':
    main()
