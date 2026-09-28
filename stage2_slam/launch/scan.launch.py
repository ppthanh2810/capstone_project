from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_share = Path(get_package_share_directory("robot_description"))
    lidar_share = Path(get_package_share_directory("sllidar_ros2"))

    robot_launch = robot_share / "launch" / "display.launch.py"
    c1_launch = lidar_share / "launch" / "sllidar_c1_launch.py"

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Cổng USB của RPLidar C1",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
        ),

        # URDF hiện có chứa base_link -> lidar_link -> laser_frame.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_launch)),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "publish_default_joint_states": "false",
            }.items(),
        ),

        # Driver C1 phát sensor_msgs/LaserScan trên /scan.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(c1_launch)),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
                "frame_id": "laser_frame",
            }.items(),
        ),
    ])