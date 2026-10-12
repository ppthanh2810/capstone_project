"""Kiểm tra localization.launch.py (KHÔNG chạy phần cứng).

1. Tĩnh: mặc định csm:=false / monitor:=false không bật node mới, không remap, và 4 include
   (scan, move_twist_mux, ekf, nav2 localization) giữ nguyên launch file + tham số như baseline.
2. Hành vi: SetRemap('/initialpose' -> '/initialpose_guess', condition csm) thực sự đổi topic của
   một node publish '/initialpose' (node giả bằng rclpy, thay cho RViz). Chạy trong ROS_DOMAIN_ID
   riêng + ROS_LOCALHOST_ONLY=1 để không ảnh hưởng robot thật.
"""

import importlib.util
import os
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node, SetRemap
import pytest


LAUNCH = Path(get_package_share_directory('stage2_slam')) / 'launch' / 'localization.launch.py'


def load_description():
    spec = importlib.util.spec_from_file_location('localization_launch', str(LAUNCH))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


def context(**configs):
    ctx = LaunchContext()
    defaults = {
        'serial_port': '/dev/ttyUSB2', 'map': '/tmp/map.yaml', 'rviz': 'true',
        'csm': 'false', 'monitor': 'false'}
    defaults.update(configs)
    for k, v in defaults.items():
        ctx.launch_configurations[k] = v
    return ctx


def group_parts(group):
    actions = group.get_sub_entities()
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    remaps = [a for a in actions if isinstance(a, SetRemap)]
    return includes, remaps


def enabled(entity, ctx):
    return entity.condition is None or entity.condition.evaluate(ctx)


def test_launch_arguments_default_off():
    ld = load_description()
    args = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    assert perform_substitutions(LaunchContext(), args['csm'].default_value) == 'false'
    assert perform_substitutions(LaunchContext(), args['monitor'].default_value) == 'false'
    # Tham số cũ giữ nguyên mặc định
    assert perform_substitutions(LaunchContext(), args['serial_port'].default_value) == '/dev/ttyUSB2'
    assert perform_substitutions(LaunchContext(), args['rviz'].default_value) == 'true'


@pytest.mark.parametrize('csm,monitor', [('false', 'false'), ('true', 'false'),
                                         ('false', 'true'), ('true', 'true')])
def test_conditional_entities(csm, monitor):
    ld = load_description()
    ctx = context(csm=csm, monitor=monitor)
    names = {}
    for n in ld.entities:
        if isinstance(n, Node):
            names[n._Node__node_name] = n  # noqa: SLF001 (tên khai báo trong launch)
    assert set(names) == {'scan_match_initializer', 'localization_monitor'}
    assert enabled(names['scan_match_initializer'], ctx) == (csm == 'true')
    assert enabled(names['localization_monitor'], ctx) == (monitor == 'true')

    groups = [e for e in ld.entities if isinstance(e, GroupAction)]
    remaps = [r for g in groups for r in group_parts(g)[1]]
    assert len(remaps) == 1
    remap = remaps[0]
    assert perform_substitutions(ctx, remap.src) == '/initialpose'
    assert perform_substitutions(ctx, remap.dst) == '/initialpose_guess'
    assert enabled(remap, ctx) == (csm == 'true')


def test_baseline_includes_unchanged():
    """4 include của luồng AMCL cũ: không có condition, cùng file và cùng tham số."""
    ld = load_description()
    ctx = context()
    groups = [e for e in ld.entities if isinstance(e, GroupAction)]
    assert len(groups) == 4
    stage2 = get_package_share_directory('stage2_slam')
    expected = [
        (str(Path(stage2) / 'launch' / 'scan.launch.py'),
         {'serial_port': '/dev/ttyUSB2', 'rviz': 'true',
          'rviz_config': str(Path(stage2) / 'rviz' / 'localization.rviz')}),
        (str(Path(get_package_share_directory('stage0_ps4_control')) / 'launch'
             / 'move_twist_mux.launch.py'), {}),
        (str(Path(get_package_share_directory('stage1_odom')) / 'launch' / 'ekf.launch.py'),
         {'rviz': 'false'}),
        (str(Path(get_package_share_directory('nav2_bringup')) / 'launch'
             / 'localization_launch.py'),
         {'map': '/tmp/map.yaml',
          'params_file': str(Path(stage2) / 'config' / 'amcl_params.yaml'),
          'use_sim_time': 'false', 'autostart': 'true', 'use_composition': 'False'}),
    ]
    for group, (path, args) in zip(groups, expected):
        assert group.condition is None
        includes, _ = group_parts(group)
        assert len(includes) == 1
        inc = includes[0]
        assert inc.condition is None
        # location chỉ được điền sau khi include thực thi; đọc danh sách substitution gốc.
        src = inc.launch_description_source
        loc = src._LaunchDescriptionSource__location  # noqa: SLF001
        assert perform_substitutions(ctx, loc) == path
        got = {perform_substitutions(ctx, normalize_to_list_of_substitutions(k)):
               perform_substitutions(ctx, normalize_to_list_of_substitutions(v))
               for k, v in inc.launch_arguments}
        assert got == args


PUBLISHER = textwrap.dedent('''
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init()
    node = rclpy.create_node('fake_rviz_pose_tool')
    node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    rclpy.spin(node)
''')

COUNTER = textwrap.dedent('''
    import sys, time, rclpy
    rclpy.init()
    node = rclpy.create_node('remap_probe')
    end = time.time() + 6.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        a = node.count_publishers('/initialpose')
        b = node.count_publishers('/initialpose_guess')
        if a + b > 0:
            break
    print(a, b)
''')

LAUNCH_TEMPLATE = textwrap.dedent('''
    import sys
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, GroupAction
    from launch.conditions import IfCondition
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node, SetRemap

    def generate_launch_description():
        return LaunchDescription([
            DeclareLaunchArgument('csm', default_value='false'),
            GroupAction(scoped=True, actions=[
                SetRemap(src='/initialpose', dst='/initialpose_guess',
                         condition=IfCondition(LaunchConfiguration('csm'))),
                Node(executable=sys.executable, arguments=[{publisher!r}],
                     name='fake_rviz_pose_tool'),
            ]),
        ])
''')


@pytest.mark.parametrize('csm,expected', [('true', (0, 1)), ('false', (1, 0))])
def test_setremap_redirects_initialpose(csm, expected):
    env = dict(os.environ, ROS_DOMAIN_ID='143', ROS_LOCALHOST_ONLY='1')
    with tempfile.TemporaryDirectory() as d:
        pub = Path(d) / 'pub.py'
        pub.write_text(PUBLISHER)
        launch_file = Path(d) / 'remap_test.launch.py'
        launch_file.write_text(LAUNCH_TEMPLATE.format(publisher=str(pub)))
        proc = subprocess.Popen(
            ['ros2', 'launch', str(launch_file), f'csm:={csm}'], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            time.sleep(1.5)
            out = subprocess.run(
                [sys.executable, '-c', COUNTER], env=env, capture_output=True, text=True,
                timeout=30)
            counts = tuple(int(x) for x in out.stdout.split())
        finally:
            os.killpg(proc.pid, 15)
            proc.wait(timeout=10)
    assert counts == expected
