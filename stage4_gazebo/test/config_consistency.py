#!/usr/bin/env python3
"""Kiểm tra nhất quán cấu hình giữa robot thật (stage0–3), Nav2 và Gazebo (stage4). Chỉ đọc file, không chạy ROS.

  python3 stage4_gazebo/test/config_consistency.py [--repo ~/capstone_ws/capstone_project] [--json out.json]

Mỗi dòng: PASS (khớp) / WARN (lệch có chủ đích hoặc cần quyết định) / FAIL (mâu thuẫn). Mã thoát 1 nếu có FAIL.
Các đại lượng dẫn xuất (khe tối thiểu, T_sim tối đa, quãng phanh) in ra để dùng chung cho generator / evaluator.
"""

import argparse
import json
import math
import os
import re
import sys

import yaml


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def cpp_default(path, name):
    """declare_parameter<...>("name", value) -> value (số)."""
    txt = open(path).read()
    m = re.search(r'declare_parameter<[^>]+>\(\s*"%s"\s*,\s*([-0-9.eE]+)' % re.escape(name), txt)
    return float(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=os.path.expanduser('~/capstone_ws/capstone_project'))
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    R = a.repo
    nav = load_yaml(f'{R}/stage3_navigation/config/nav2_params.yaml')
    zsim = load_yaml(f'{R}/stage4_gazebo/config/s0_control/zlac_sim.yaml')['zlac_sim']['ros__parameters']
    gc = nav['global_costmap']['global_costmap']['ros__parameters']
    lc = nav['local_costmap']['local_costmap']['ros__parameters']
    cs = nav['controller_server']['ros__parameters']
    dwb = cs['CustomDWB']
    br = nav['goal_path_bridge']['ros__parameters']
    move = f'{R}/stage0_ps4_control/src/zlac8015d_move.cpp'
    odom = f'{R}/stage1_odom/src/zlac8015d_odom.cpp'
    xacro = open(f'{R}/stage4_gazebo/urdf/robot_sim.urdf.xacro').read()
    hpp = open(f'{R}/hardware/zlac8015d_hw/include/zlac8015d_hw/zlac8015d.hpp').read() \
        if os.path.exists(f'{R}/hardware/zlac8015d_hw/include/zlac8015d_hw/zlac8015d.hpp') else ''

    rows, derived = [], {}

    def row(level, item, detail):
        rows.append((level, item, detail))

    def same(item, vals, tol=1e-9):
        v = [x for x in vals.values() if x is not None]
        ok = len(v) == len(vals) and max(v) - min(v) <= tol
        row('PASS' if ok else 'FAIL', item, ', '.join(f'{k}={x}' for k, x in vals.items()))

    xr = re.search(r'name="wheel_radius" value="([0-9.]+)"', xacro)
    same('wheel_radius', {'move.cpp': cpp_default(move, 'wheel_radius'), 'odom.cpp': cpp_default(odom, 'wheel_radius'),
                          'zlac_sim': zsim['wheel_radius'], 'sim xacro': float(xr.group(1)) if xr else None})
    same('wheel_distance', {'move.cpp': cpp_default(move, 'wheel_distance'),
                            'odom.cpp': cpp_default(odom, 'wheel_distance'), 'zlac_sim': zsim['wheel_distance']})
    m = re.search(r'TRAVEL_IN_ONE_REV\s*=?\s*([0-9.]+)', hpp)
    if m:
        circ = 2 * math.pi * zsim['wheel_radius']
        dev = abs(float(m.group(1)) / circ - 1)
        row('PASS' if dev < 0.005 else 'WARN', 'TRAVEL_IN_ONE_REV vs 2πr', f'{m.group(1)} vs {circ:.4f} (lệch {100 * (float(m.group(1)) / circ - 1):+.1f} %,'
            ' encoder dùng hằng số, lệnh dùng r)')
    same('accel_time_ms (code thật / sim)', {'move.cpp': cpp_default(move, 'accel_time_ms'),
                                            'zlac_sim': float(zsim['accel_time_ms'])})
    same('decel_time_ms (code thật / sim)', {'move.cpp': cpp_default(move, 'decel_time_ms'),
                                            'zlac_sim': float(zsim['decel_time_ms'])})

    # ---- footprint / costmap ----
    R0 = gc['robot_radius']
    same('robot_radius global/local', {'global': R0, 'local': lc['robot_radius']})
    pad_g = gc.get('footprint_padding', 0.01)   # mặc định Nav2 Humble 0.01
    pad_l = lc.get('footprint_padding', 0.01)
    ins_g, ins_l = R0 + pad_g, lc['robot_radius'] + pad_l
    res = lc['resolution']
    gap_astar = 2 * ins_g + res          # tâm phải ở ô < INSCRIBED (ô ~ res)
    gap_dwb = 2 * ins_l + res
    derived.update(R=R0, inscribed_global=ins_g, inscribed_local=ins_l,
                   gap_min_astar=round(gap_astar, 3), gap_min_dwb=round(gap_dwb, 3))
    row('PASS' if abs(ins_g - ins_l) < 1e-9 else 'WARN', 'bán kính nội tiếp global/local',
        f'{ins_g:.2f} / {ins_l:.2f} m -> khe tối thiểu A* ~{gap_astar:.2f} m, DWB ~{gap_dwb:.2f} m'
        + ('' if abs(ins_g - ins_l) < 1e-9 else ' (A* có thể lập đường qua khe DWB không qua được)'))
    infl_l = lc['inflation_layer']['inflation_radius']
    infl_g = gc['inflation_layer']['inflation_radius']
    row('PASS' if infl_l > ins_l else 'FAIL', 'local inflation > inscribed', f'{infl_l} > {ins_l:.2f}')
    row('PASS' if infl_g > ins_g else 'FAIL', 'global inflation > inscribed', f'{infl_g} > {ins_g:.2f}')
    has_obs_g = 'obstacle_layer' in gc['plugins']
    row('PASS' if has_obs_g else 'WARN', 'global_costmap có obstacle_layer (replan thấy vật cản mới)',
        str(gc['plugins']))
    if has_obs_g:
        og, ol = gc['obstacle_layer']['scan'], lc['obstacle_layer']['scan']
        same('obstacle/raytrace range global/local', {'g_obs': og['obstacle_max_range'], 'l_obs': ol['obstacle_max_range'],
                                                      'g_ray': og['raytrace_max_range'] - 0.5,
                                                      'l_ray': ol['raytrace_max_range'] - 0.5})
        same('scan topic global/local', {'g': hash(og['topic']), 'l': hash(ol['topic'])})
        same('replan.lookahead = obstacle_max_range', {'lookahead': br['replan']['lookahead_distance'],
                                                       'obs_range': og['obstacle_max_range']})

    # ---- dynamic window / horizon ----
    half = min(lc['width'], lc['height']) / 2.0
    v, dec = dwb['max_vel_x'], dwb['decel_lim_x']
    lat, dt = dwb['control_latency'], 1.0 / cs['controller_frequency']
    brake = v * (lat + dt) + v * v / (2 * dec) + dwb['stopping_margin']
    t_max = (half - res) / v
    derived.update(braking_distance=round(brake, 3), sim_time_max_costmap=round(t_max, 2),
                   stop_time=round(lat + dt + v / dec, 2))
    row('PASS' if v * dwb['sim_time'] <= half - res else 'FAIL', 'max_vel_x·sim_time trong local costmap',
        f'{v}·{dwb["sim_time"]} = {v * dwb["sim_time"]:.2f} m <= {half - res:.2f} m (T_sim tối đa {t_max:.2f} s '
        f'ở {v} m/s; vượt -> ứng viên nhanh bị loại OFF_MAP)')
    row('PASS' if brake < half - ins_l else 'FAIL', 'quãng phanh (latency + dt + v²/2a + margin)',
        f'{brake:.3f} m < {half - ins_l:.2f} m')
    gt = cs['general_goal_checker']
    same('xy goal tolerance checker/CustomDWB', {'checker': gt['xy_goal_tolerance'], 'dwb': dwb['xy_goal_tolerance']})
    same('yaw goal tolerance checker/CustomDWB', {'checker': gt['yaw_goal_tolerance'], 'dwb': dwb['yaw_goal_tolerance']})
    pc = cs['progress_checker']
    rp = br.get('replan', {})
    if rp:
        row('PASS' if rp['stall_time'] < pc['movement_time_allowance'] else 'FAIL',
            'replan.stall_time < progress checker', f'{rp["stall_time"]} < {pc["movement_time_allowance"]} s')
        row('PASS' if rp['stall_progress'] < pc['required_movement_radius'] else 'WARN',
            'replan.stall_progress < required_movement_radius',
            f'{rp["stall_progress"]} < {pc["required_movement_radius"]} m')
        row('PASS' if br['follow_timeout'] > rp['wait_timeout'] else 'FAIL', 'follow_timeout > replan.wait_timeout',
            f'{br["follow_timeout"]} > {rp["wait_timeout"]}')
    row('PASS' if dwb['max_path_deviation'] <= half else 'WARN', 'max_path_deviation <= nửa local costmap',
        f'{dwb["max_path_deviation"]} <= {half}')
    row('PASS' if dwb['carrot_distance'] < half else 'WARN', 'carrot_distance < nửa local costmap',
        f'{dwb["carrot_distance"]} < {half}')
    # ---- trễ ----
    row('WARN' if abs(zsim['command_delay']) < 1e-9 else 'PASS', 'trễ lệnh sim vs CustomDWB.control_latency',
        f'zlac_sim.command_delay {zsim["command_delay"]} s; control_latency {lat} s (đo bag 2026-10-14, accel 200 ms)')
    row('WARN' if cpp_default(move, 'accel_time_ms') != 200 else 'PASS', 'control_latency đo ở accel 200 ms',
        f'driver hiện {cpp_default(move, "accel_time_ms"):.0f} ms -> trễ thật có thể > {lat} s (chưa đo lại)')

    width = max(len(r[1]) for r in rows)
    for lvl, item, det in rows:
        print(f'{lvl:4s}  {item:<{width}}  {det}')
    print('\nDẫn xuất:', json.dumps(derived, ensure_ascii=False))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({'rows': rows, 'derived': derived}, f, ensure_ascii=False, indent=1)
    return 1 if any(r[0] == 'FAIL' for r in rows) else 0


if __name__ == '__main__':
    sys.exit(main())
