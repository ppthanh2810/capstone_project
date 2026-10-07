from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, launch_file, arguments=None):
    launch_path = (
        Path(get_package_share_directory(package_name))
        / "launch"
        / launch_file
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    slam_share = Path(get_package_share_directory("slam_toolbox"))

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

        # C1 + URDF TF + bộ lọc /scan_filtered + RViz2.
        include_launch(
            "stage2_slam",
            "scan.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "rviz": LaunchConfiguration("rviz"),
            },
        ),

        # PS4 + twist_mux + động cơ phát /wheel_feedback.
        include_launch(
            "stage0_ps4_control",
            "move_twist_mux.launch.py",
        ),

        # Wheel odom + IMU odom + EKF phát odom -> base_footprint.
        include_launch(
            "stage1_odom",
            "ekf.launch.py",
        ),

        # SLAM chỉ dùng scan đã loại vùng phía sau.
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                str(
                    slam_share
                    / "config"
                    / "mapper_params_online_async.yaml"
                ),
                {
                    "use_sim_time": False,
                    "scan_topic": "/scan_filtered",
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": "base_footprint",
                    "mode": "mapping",
                    "min_laser_range": 0.10,
                    "max_laser_range": 16.0,
                    # Mặc định slam_toolbox: vẽ lại /map mỗi 5 s, chỉ thêm scan khi đi 0.5 m
                    # hoặc quay 0.5 rad -> bản đồ cập nhật chậm với robot nhỏ, đi chậm.
                    "map_update_interval": 1.0,
                    "minimum_travel_distance": 0.2,
                    "minimum_travel_heading": 0.2,
                },
            ],
        ),
    ])