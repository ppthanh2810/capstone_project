"""S0 — điều khiển: sim_world + zlac_sim + PS4 (joy thật) + joy_teleop + twist_mux.

Thay zlac8015d_move (Modbus) bằng zlac_sim. joy_teleop / twist_mux chạy trực tiếp với YAML gốc của
stage0_ps4_control (không include launch stage0 vì nó mở driver motor thật).
  /joy -> joy_teleop -> /cmd_vel_joy (prio 100) ┐
                         /cmd_vel_nav (prio 10) ┴-> twist_mux -> /cmd_vel_out -> zlac_sim -> Gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('stage4_gazebo')
    stage0 = get_package_share_directory('stage0_ps4_control')

    sim_time = {'use_sim_time': True}

    sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim_world.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'world': LaunchConfiguration('world'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'rviz': LaunchConfiguration('rviz'),
            'follow': LaunchConfiguration('follow'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'bridge_config': LaunchConfiguration('bridge_config'),
            'included': 'true',
        }.items(),
    )

    zlac_sim = Node(
        package='stage4_gazebo',
        executable='zlac_sim',
        name='zlac_sim',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 's0_control', 'zlac_sim.yaml'),
            {'accel_time_ms': LaunchConfiguration('accel_time_ms'),
             'decel_time_ms': LaunchConfiguration('decel_time_ms')},
            sim_time,
        ],
    )

    # Tay cầm PS4 thật (chỉ đọc /dev/input/js*, không động tới motor)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    joy_teleop = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[os.path.join(stage0, 'config', 'ps4.yaml'), sim_time],
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(stage0, 'config', 'twist_mux.yaml'), sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_floor.sdf')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 's0.rviz')),
        DeclareLaunchArgument('bridge_config',
                              default_value=os.path.join(pkg, 'config', 's0_control', 'bridge_s0.yaml'),
                              description='ros_gz_bridge config (s1+ truyền file của giai đoạn mình)'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='true: camera GUI bám robot (chặn xoay camera bằng chuột)'),
        DeclareLaunchArgument('joy', default_value='true',
                              description='true: joy_node + joy_teleop (PS4 thật); false: không tay cầm'),
        DeclareLaunchArgument('accel_time_ms', default_value='500'),
        DeclareLaunchArgument('decel_time_ms', default_value='500'),
        sim_world,
        zlac_sim,
        joy_node,
        joy_teleop,
        twist_mux,
    ])
