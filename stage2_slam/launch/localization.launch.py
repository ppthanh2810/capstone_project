from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name, launch_file, arguments=None):
    launch_path = (
        Path(get_package_share_directory(package_name))
        / "launch"
        / launch_file
    )

    # GroupAction(scoped=True): ở Humble, launch_arguments của include ghi đè launch
    # configuration toàn cục (vd. "rviz": "false" cho ekf sẽ tắt luôn RViz của scan).
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments=(arguments or {}).items(),
            ),
        ],
    )


def generate_launch_description():
    stage2_share = Path(get_package_share_directory("stage2_slam"))

    amcl_config = stage2_share / "config" / "amcl_params.yaml"

    default_map = (
        Path.home()
        / "capstone_ws"
        / "capstone_project"
        / "maps"
        / "map.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB2",
            description="Cổng USB của RPLidar C1",
        ),

        DeclareLaunchArgument(
            "map",
            default_value=str(default_map),
            description="Đường dẫn tới file bản đồ site_map.yaml",
        ),

        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Có mở RViz2 hay không",
        ),

        # RPLidar C1 + robot_state_publisher + RViz2 (localization.rviz: /map, AMCL, /scan_filtered).
        include_launch(
            "stage2_slam",
            "scan.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "rviz": LaunchConfiguration("rviz"),
                "rviz_config": str(stage2_share / "rviz" / "localization.rviz"),
            },
        ),

        # Tay PS4 + twist_mux + điều khiển động cơ.
        # zlac8015d_move phát /wheel_feedback.
        include_launch(
            "stage0_ps4_control",
            "move_twist_mux.launch.py",
        ),

        # Odometry bánh xe + IMU + EKF.
        # EKF phát TF odom -> base_footprint.
        # Tắt RViz odom của ekf.launch.py: chỉ mở 1 RViz (của scan.launch.py).
        include_launch(
            "stage1_odom",
            "ekf.launch.py",
            {"rviz": "false"},
        ),

        # Nạp site_map.yaml, chạy Map Server và AMCL.
        # AMCL phát TF map -> odom sau khi đặt vị trí ban đầu.
        include_launch(
            "nav2_bringup",
            "localization_launch.py",
            {
                "map": LaunchConfiguration("map"),
                "params_file": str(amcl_config),
                "use_sim_time": "false",
                "autostart": "true",
                "use_composition": "False",
            },
        ),
    ])