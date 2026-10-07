from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_share = Path(get_package_share_directory("robot_description"))
    lidar_share = Path(get_package_share_directory("sllidar_ros2"))
    stage2_share = Path(get_package_share_directory("stage2_slam"))

    robot_launch = robot_share / "launch" / "display.launch.py"
    c1_launch = lidar_share / "launch" / "sllidar_c1_launch.py"
    filter_config = stage2_share / "config" / "angular_filter.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB1",
            description="Cổng USB của RPLidar C1",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Có mở RViz2 hay không",
        ),

        # URDF phát TF base_link -> lidar_link -> laser_frame.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_launch)),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "publish_default_joint_states": "false",
            }.items(),
        ),

        # Driver C1 phát dữ liệu gốc trên /scan.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(c1_launch)),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "frame_id": "laser_frame",
            }.items(),
        ),

        # Đọc /scan và phát phần góc phía trước lên /scan_filtered.
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            output="screen",
            parameters=[str(filter_config)],
            remappings=[
                ("scan", "/scan"),
                ("scan_filtered", "/scan_filtered"),
            ],
        ),
    ])