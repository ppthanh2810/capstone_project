
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("stage1_odom"),
        "config",
        "ekf.yaml"
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
    # LAUNCH
    # ============================================================

    return LaunchDescription([
        wheel_odom,
        imu_odom,
        ekf
    ])