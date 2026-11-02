"""Gazebo Fortress + robot sim: world, spawn, /clock bridge, robot_state_publisher.

Đây là TẦNG NỀN: chỉ có world + robot, KHÔNG có driver / teleop nên robot không lái được.
Để lái: ros2 launch stage4_gazebo s0_control.launch.py (include file này).

Không mở cổng phần cứng nào. Mọi node chạy use_sim_time:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROBOT_NAME = 'ace492'


def running_gazebo_servers():
    """PID các `ign gazebo server` cùng IGN_PARTITION với launch này (vd. còn sót từ lần trước)."""
    partition = os.environ.get('IGN_PARTITION', '')
    pids = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore')
            if 'gazebo' not in cmd or not ('server' in cmd or ' -s' in cmd):
                continue
            if not cmd.startswith(('ign gazebo', 'ruby /usr/bin/ign gazebo')):
                continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(e.split('=', 1) for e in f.read().decode(errors='ignore').split('\0') if '=' in e)
            if env.get('IGN_PARTITION', '') == partition:
                pids.append(pid)
        except OSError:
            continue
    return pids


def check_no_old_gazebo(_context):
    # Hai server cùng partition + cùng tên world -> spawn / bridge / GUI có thể nối nhầm world cũ.
    pids = running_gazebo_servers()
    if pids:
        raise RuntimeError(
            'Đang có Gazebo server khác chạy cùng partition (PID ' + ' '.join(pids) + '), có thể còn sót '
            'từ lần trước. Tắt nó trước: kill ' + ' '.join(pids))
    return []


def generate_launch_description():
    pkg = get_package_share_directory('stage4_gazebo')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    bridge_config = LaunchConfiguration('bridge_config')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'robot_sim.urdf.xacro')]),
        value_type=str)

    # Server và GUI là 2 tiến trình riêng, gọi thẳng `ign gazebo` (không qua gz_sim.launch.py: bản đó
    # chạy shell=True, còn `ign gazebo` có GUI tự sinh 2 tiến trình con -> Ctrl+C dễ bỏ sót Gazebo).
    # -r: chạy ngay; -v 2: lỗi + cảnh báo.
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '-v', '2', world],
        name='gz_server',
        output='screen',
        on_exit=Shutdown(reason='Gazebo server đã thoát'),
    )

    # Đóng cửa sổ GUI -> tắt cả launch. Camera gần robot (config/gui.config).
    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g', '-v', '2', '--gui-config', LaunchConfiguration('gui_config')],
        name='gz_gui',
        output='screen',
        on_exit=Shutdown(reason='Đã đóng cửa sổ Gazebo'),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='log',  # tránh 10 dòng "got segment" mỗi lần chạy
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ace492',
        output='log',
        arguments=['-topic', 'robot_description', '-name', ROBOT_NAME,
                   '-x', x, '-y', y, '-z', '0.002', '-Y', yaw],
    )

    # Camera GUI bám theo robot sau khi spawn (thử lại tới khi GUI sẵn sàng, tối đa ~20 s)
    follow_cmd = (
        'for i in $(seq 40); do '
        f'ign service -s /gui/follow --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean '
        f'--timeout 500 --req \'data: "{ROBOT_NAME}"\' 2>/dev/null | grep -q "data: true" && '
        'ign service -s /gui/follow/offset --reqtype ignition.msgs.Vector3d --reptype ignition.msgs.Boolean '
        '--timeout 500 --req "x: -1.6, y: -1.2, z: 1.3" >/dev/null 2>&1 && exit 0; sleep 0.5; done; '
        'echo "[sim_world] Không bật được camera bám robot (GUI chưa sẵn sàng)"')
    camera_follow = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[ExecuteProcess(cmd=['bash', '-c', follow_cmd], name='camera_follow', output='screen',
                                condition=IfCondition(LaunchConfiguration('follow')))],
    ))

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='log',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sim',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    standalone_hint = LogInfo(
        msg='[sim_world] Chỉ có world + robot, KHÔNG có driver/teleop -> robot không lái được. '
            'Để lái bằng PS4: ros2 launch stage4_gazebo s0_control.launch.py',
        condition=UnlessCondition(LaunchConfiguration('included')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='true: mở cửa sổ Gazebo; false: headless'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_floor.sdf'),
                              description='Đường dẫn file world .sdf'),
        DeclareLaunchArgument('x', default_value='0.0', description='Vị trí spawn x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Vị trí spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Hướng spawn (rad)'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='true: camera GUI bám robot (chặn xoay camera bằng chuột)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Mở RViz'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 's0.rviz')),
        DeclareLaunchArgument('gui_config', default_value=os.path.join(pkg, 'config', 'gui.config'),
                              description='File cấu hình GUI Gazebo'),
        DeclareLaunchArgument('bridge_config',
                              default_value=PathJoinSubstitution([pkg, 'config', 's0_control', 'bridge_s0.yaml']),
                              description='File cấu hình ros_gz_bridge (mỗi giai đoạn dùng file riêng)'),
        DeclareLaunchArgument('included', default_value='false',
                              description='true khi được launch giai đoạn include (ẩn lời nhắc)'),
        # Gazebo tìm mesh package://robot_description/meshes/... trong các thư mục share này
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               os.pathsep.join([os.path.dirname(pkg),
                                                os.path.dirname(get_package_share_directory('robot_description')),
                                                os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])),
        OpaqueFunction(function=check_no_old_gazebo),
        standalone_hint,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn,
        camera_follow,
        bridge,
        rviz,
    ])
