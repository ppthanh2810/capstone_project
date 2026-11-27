from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _isolate_remaps(context):
    # Humble: SetRemap append vào CHÍNH list "ros_remaps" của scope cha (GroupAction scoped chỉ copy
    # nông dict). Khi file này được include dưới một SetRemap khác (localization.launch.py csm:=true),
    # remap "scan" -> "/scan_raw" bị rò ra ngoài group -> laser_filters/RViz đọc /scan_raw (laser_raw).
    # Tạo list riêng cho scope hiện tại trước khi SetRemap.
    context.launch_configurations["ros_remaps"] = list(
        context.launch_configurations.get("ros_remaps", []))


def generate_launch_description():
    robot_share = Path(get_package_share_directory("robot_description"))
    lidar_share = Path(get_package_share_directory("sllidar_ros2"))
    stage2_share = Path(get_package_share_directory("stage2_slam"))

    robot_launch = robot_share / "launch" / "display.launch.py"
    c1_launch = lidar_share / "launch" / "sllidar_c1_launch.py"
    filter_config = stage2_share / "config" / "angular_filter.yaml"
    scan_rviz = stage2_share / "rviz" / "scan.rviz"

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
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(scan_rviz),
            description="File RViz (mapping/localization truyền slam.rviz / localization.rviz)",
        ),

        # URDF phát TF base_link -> lidar_link -> laser_frame (cả hai cùng hướng base_footprint).
        # RViz của robot_description tắt: launch này mở RViz riêng (rviz_config) bên dưới.
        # GroupAction(scoped=True): ở Humble, launch_arguments của include ghi đè launch
        # configuration toàn cục -> không scope thì "rviz": "false" tắt luôn RViz của scan.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(robot_launch)),
                    launch_arguments={
                        "rviz": "false",
                        "publish_default_joint_states": "false",
                    }.items(),
                ),
            ],
        ),

        # Driver C1 phát dữ liệu gốc trên /scan_raw (remap từ "scan").
        # Quy ước sllidar_ros2: góc = pi - góc thiết bị -> góc 0 của /scan_raw hướng ra SAU robot.
        # frame_id "laser_raw" không có trong TF: /scan_raw chỉ dành cho scan_rotate, không hiển thị trực tiếp.
        GroupAction(
            scoped=True,
            actions=[
                OpaqueFunction(function=_isolate_remaps),
                SetRemap(src="scan", dst="/scan_raw"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(c1_launch)),
                    launch_arguments={
                        "serial_port": LaunchConfiguration("serial_port"),
                        "frame_id": "laser_raw",
                    }.items(),
                ),
            ],
        ),

        # Xoay scan pi (angle_min/max += pi, giữ nguyên ranges) -> /scan trong laser_frame,
        # laser_frame = lidar_link cùng hướng base_footprint (X trước, Y trái, Z lên).
        Node(
            package="stage2_slam",
            executable="scan_rotate",
            name="scan_rotate",
            output="screen",
            parameters=[{"angle_offset": 3.141592653589793, "frame_id": "laser_frame"}],
            remappings=[
                ("scan_in", "/scan_raw"),
                ("scan_out", "/scan"),
            ],
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

        # RViz2: mặc định scan.rviz (Fixed Frame base_footprint, /scan + /scan_filtered).
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])