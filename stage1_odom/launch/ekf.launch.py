"""Stage1: wheel odom + IMU odom + EKF -> /odometry/filtered + TF odom -> base_footprint.

Mở cổng serial IMU (/dev/ttyUSB1). Không chạy motor: /wheel_odom chỉ có dữ liệu khi
zlac8015d_move (stage0) đang phát /wheel_feedback.

rviz:=true (mặc định) chạy thêm robot_state_publisher (URDF) + RViz odom.rviz
(Fixed Frame odom: /wheel_odom, /imu_odom, /odometry/filtered).
mapping/localization.launch.py include file này với rviz:=false vì scan.launch.py đã chạy
robot_state_publisher và RViz riêng (tránh trùng node).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    stage1_share = get_package_share_directory("stage1_odom")
    robot_share = get_package_share_directory("robot_description")

    config = os.path.join(stage1_share, "config", "ekf.yaml")
    rviz_config = os.path.join(stage1_share, "rviz", "odom.rviz")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Mở RViz2 (odom.rviz) + robot_state_publisher. false khi include từ stage2",
    )

    # ============================================================
    # WHEEL ODOMETRY
    # ============================================================

    wheel_odom = Node(
        package="stage1_odom",
        executable="zlac8015d_odom",
        name="zlac8015d_odom",
        parameters=[{
            "wheel_radius": 0.0535,
            "wheel_distance": 0.34,
            "publish_tf": False
        }]
    )

    # ============================================================
    # IMU ODOMETRY
    # ============================================================

    imu_odom = Node(
        package="stage1_odom",
        executable="imu_odom",
        name="imu_odom",
        parameters=[{
            "port": "/dev/ttyUSB1",
            "frame_id": "imu_link"
        }]
    )

    # ============================================================
    # EXTENDED KALMAN FILTER
    # ============================================================

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[config]
    )

    # ============================================================
    # VIEW: URDF (base_footprint -> sensor frames) + RVIZ2
    # ============================================================

    # GroupAction(scoped=True): ở Humble, launch_arguments của include ghi đè launch
    # configuration toàn cục -> không scope thì "rviz": "false" tắt luôn RViz odom bên dưới.
    robot_state = GroupAction(
        scoped=True,
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_share, "launch", "display.launch.py")
                ),
                launch_arguments={
                    "rviz": "false",
                    "publish_default_joint_states": "false",
                }.items(),
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_odom",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([
        rviz_arg,
        wheel_odom,
        imu_odom,
        ekf,
        robot_state,
        rviz,
    ])
