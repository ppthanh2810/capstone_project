"""S1 — odometry: S0 + imu_sim_odom + zlac8015d_odom + EKF gốc của stage1.

Thay imu_odom (UART ESP32/BNO055) bằng imu_sim_odom (IMU Gazebo). zlac8015d_odom và EKF chạy nguyên bản
(cùng tham số với stage1_odom/launch/ekf.launch.py, chỉ thêm use_sim_time). Không include ekf.launch.py
vì nó mở cổng serial IMU.
  /wheel_feedback (zlac_sim) -> zlac8015d_odom -> /wheel_odom ┐
  /sim/imu (Gazebo) -> imu_sim_odom -> /imu/data, /imu_odom ───┴-> ekf_filter_node -> /odometry/filtered
                                                                                     + TF odom -> base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('stage4_gazebo')
    stage1 = get_package_share_directory('stage1_odom')

    sim_time = {'use_sim_time': True}

    common_args = ['gui', 'world', 'x', 'y', 'yaw', 'rviz', 'follow', 'joy',
                   'accel_time_ms', 'decel_time_ms', 'rviz_config', 'bridge_config']
    s0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 's0_control.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in common_args}.items(),
    )

    imu_sim_odom = Node(
        package='stage4_gazebo',
        executable='imu_sim_odom',
        name='imu_odom',  # cùng tên node với imu_odom thật
        output='screen',
        parameters=[{'frame_id': 'imu_link'}, sim_time],
    )

    # Giống ekf.launch.py
    wheel_odom = Node(
        package='stage1_odom',
        executable='zlac8015d_odom',
        name='zlac8015d_odom',
        parameters=[{'wheel_radius': 0.0535, 'wheel_distance': 0.34, 'publish_tf': False}, sim_time],
    )

    # YAML gốc stage1, use_sim_time đặt SAU để ghi đè
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(stage1, 'config', 'ekf.yaml'), sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_floor.sdf')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 's1.rviz')),
        DeclareLaunchArgument('follow', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('accel_time_ms', default_value='500'),
        DeclareLaunchArgument('decel_time_ms', default_value='500'),
        DeclareLaunchArgument('bridge_config',
                              default_value=os.path.join(pkg, 'config', 's1_odom', 'bridge_s1.yaml')),
        s0,
        imu_sim_odom,
        wheel_odom,
        ekf,
    ])
