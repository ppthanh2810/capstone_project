"""Cấu hình benchmark — MỘT nguồn: stage3_navigation/config/nav2_params.yaml (CustomDWB, replan, costmap, controller)
+ stage5_benchmark/config/benchmark.yaml (mô phỏng, episode, ngưỡng evaluator, generator).

Fast 2D và Gazebo đọc cùng hai file này, nên cùng tham số điều hướng với robot thật.
"""

import copy
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def _find(rel_candidates):
    for p in rel_candidates:
        if os.path.exists(p):
            return p
    raise FileNotFoundError(rel_candidates)


def repo_root():
    env = os.environ.get('CAPSTONE_REPO')
    if env:
        return env
    return os.path.expanduser('~/capstone_ws/capstone_project')


def nav2_params_path():
    return _find([os.path.join(repo_root(), 'stage3_navigation', 'config', 'nav2_params.yaml')])


def benchmark_yaml_path():
    return _find([os.path.join(repo_root(), 'stage5_benchmark', 'config', 'benchmark.yaml')])


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


DWB_SKIP = {'plugin', 'publish_trajectories', 'trajectory_marker_stride', 'debug_publish_rate'}
REPLAN_SKIP = {'costmap_timeout', 'speed_limit_topic', 'clear_costmap_on_failure', 'clear_keep_distance', 'clear_costmap_service',
               'controller_state_topic'}


def load(profile=None, overrides=None):
    """Trả về dict {dwb, replan, sim, eval, gen, profile} sẵn cho fast2d.Episode.

    profile: tên trong benchmark.yaml `profiles` (vd. 'planner', 'local4x4'); overrides: dict lồng {dwb: {...}, ...}.
    """
    nav = load_yaml(nav2_params_path())
    bm = load_yaml(benchmark_yaml_path())
    cs = nav['controller_server']['ros__parameters']
    dwb = {k: v for k, v in cs['CustomDWB'].items() if k not in DWB_SKIP}
    for k in ('velocity_reference', 'collision_check'):
        dwb[k] = str(dwb[k])
    for k in ('carrot_validation', 'unknown_is_obstacle'):
        dwb[k] = 1.0 if dwb[k] else 0.0
    br = nav['goal_path_bridge']['ros__parameters']
    replan = {k: v for k, v in br['replan'].items() if k not in REPLAN_SKIP}
    replan['enabled'] = 1.0 if replan['enabled'] else 0.0
    lc = nav['local_costmap']['local_costmap']['ros__parameters']
    gc = nav['global_costmap']['global_costmap']['ros__parameters']
    gt = cs['general_goal_checker']
    pc = cs['progress_checker']
    sim = dict(bm['sim'])
    sim.update({
        'local_size': float(lc['width']),
        'resolution': float(gc['resolution']),
        'local_resolution': float(lc['resolution']),
        'footprint_radius': float(lc['robot_radius']),
        'local_padding': float(lc.get('footprint_padding', 0.01)),
        'global_padding': float(gc.get('footprint_padding', 0.01)),
        'local_inflation': float(lc['inflation_layer']['inflation_radius']),
        'global_inflation': float(gc['inflation_layer']['inflation_radius']),
        'cost_scaling': float(lc['inflation_layer']['cost_scaling_factor']),
        'local_track_unknown': 1.0 if lc['track_unknown_space'] else 0.0,
        'inf_is_valid': 1.0 if lc['obstacle_layer']['scan']['inf_is_valid'] else 0.0,
        'obstacle_max_range': float(lc['obstacle_layer']['scan']['obstacle_max_range']),
        'raytrace_max_range': float(lc['obstacle_layer']['scan']['raytrace_max_range']),
        'local_update_rate': float(lc['update_frequency']),
        'global_update_rate': float(gc['update_frequency']),
        'expected_update_rate': float(lc['obstacle_layer']['scan']['expected_update_rate']),
        'global_obstacle_layer': 1.0 if 'obstacle_layer' in gc['plugins'] else 0.0,
        'local_footprint_clearing': 1.0 if lc['obstacle_layer'].get('footprint_clearing_enabled', True) else 0.0,
        'global_footprint_clearing': 1.0 if gc.get('obstacle_layer', {}).get('footprint_clearing_enabled', True)
        else 0.0,
        'controller_rate': float(cs['controller_frequency']),
        'failure_tolerance': float(cs['failure_tolerance']),
        'progress_radius': float(pc['required_movement_radius']),
        'progress_time': float(pc['movement_time_allowance']),
        'xy_goal_tolerance': float(gt['xy_goal_tolerance']),
        'yaw_goal_tolerance': float(gt['yaw_goal_tolerance']),
        'follow_timeout': float(br['follow_timeout']),
        'max_start_distance': float(br['max_start_distance']),
    })
    cfg = {'dwb': dwb, 'replan': replan, 'sim': sim, 'eval': dict(bm['eval']), 'gen': copy.deepcopy(bm['gen']),
           'profile': profile or 'default'}
    if profile:
        prof = bm.get('profiles', {}).get(profile)
        if prof is None:
            raise KeyError(f'profile {profile} không có trong benchmark.yaml')
        _merge(cfg, prof)
    if overrides:
        _merge(cfg, overrides)
    return cfg


def _merge(cfg, upd):
    for sec, vals in upd.items():
        if sec in ('dwb', 'replan', 'sim', 'eval') and isinstance(vals, dict):
            cfg[sec].update(vals)
        elif sec == 'gen' and isinstance(vals, dict):
            cfg['gen'].update(vals)


def sim_for_fast2d(sim):
    """Chỉ các khóa số mà fast2d.SimConfig hiểu."""
    return {k: float(v) for k, v in sim.items() if not isinstance(v, str)}
