from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, launch_file, arguments=None):
    path = (
        Path(get_package_share_directory(package_name))
        / "launch"
        / launch_file
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    slam_share = Path(get_package_share_directory("slam_toolbox"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB2",
            description="Cổng USB của RPLidar C1",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
        ),

        # RPLidar C1 + robot_state_publisher + RViz2.
        include_launch(
            "stage2_slam",
            "scan.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "rviz": LaunchConfiguration("rviz"),
            },
        ),

        # PS4 + twist_mux + zlac8015d_move.
        # zlac8015d_move phát /wheel_feedback kể cả khi xe đứng yên.
        include_launch(
            "stage0_ps4_control",
            "move_twist_mux.launch.py",
        ),

        # /wheel_feedback -> /wheel_odom;
        # IMU -> /imu_odom; EKF -> /odometry/filtered và odom TF.
        include_launch(
            "stage1_odom",
            "ekf.launch.py",
        ),

        # /scan + odom TF -> /map và map -> odom.
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
                    "scan_topic": "/scan",
                    "map_frame": "map",
                    "odom_frame": "odom",
                    "base_frame": "base_footprint",
                    "mode": "mapping",
                },
            ],
        ),
    ])