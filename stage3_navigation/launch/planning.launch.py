from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Lệnh thống nhất: Stage 2 (LiDAR + motor/teleop + wheel odom + IMU + EKF + localization hoặc SLAM)
# + Stage 3 (planner_server A* + global_costmap + goal_path_bridge) + MỘT RViz2 (planning.rviz).
#   mode:=localization (mặc định): map_server + AMCL (+ CSM, monitor) -> TF map -> odom
#   mode:=mapping                 : slam_toolbox                     -> TF map -> odom
# Mỗi mode chỉ include MỘT bringup Stage 2 -> chỉ một nguồn map -> odom, một motor driver, một EKF.
# CẢNH BÁO: Stage 2 chạy zlac8015d_move (Modbus, motor) như mapping/localization.launch.py.
# Stage 3 không publish TF.
# Stage 3.2 (controller:=true): controller_server + local_costmap + DWB. Vận tốc DWB:
#   navigation_enabled:=false (mặc định) -> /cmd_vel_dwb: KHÔNG node nào nghe,
#                                           cách ly khỏi twist_mux/motor
#   navigation_enabled:=true -> /cmd_vel_nav -> twist_mux (prio 10, PS4 prio 100 thắng) -> motor
# DWB chỉ chạy khi goal_path_bridge gửi FollowPath: service /goal_path_bridge/follow
# hoặc auto_follow:=true.
# stage2:=false: không include Stage 2 (không phần cứng) -> chỉ để kiểm thử offline, khi /map, TF
# map -> odom -> base_footprint, /scan_filtered, /odometry/filtered do nguồn khác cung cấp.
VALID_MODES = ("localization", "mapping")


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _check_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in VALID_MODES:
        raise RuntimeError(f"mode:={mode} không hợp lệ, chọn một trong {VALID_MODES}")
    controller = _is_true(context, "controller")
    for flag in ("navigation_enabled", "auto_follow"):
        if _is_true(context, flag) and not controller:
            raise RuntimeError(f"{flag}:=true cần controller:=true")
    if not controller:
        return [LogInfo(msg="[stage3] planning only: không có controller_server, "
                            "không có lệnh vận tốc")]
    if _is_true(context, "navigation_enabled"):
        return [LogInfo(msg="[stage3] !!! navigation_enabled:=true: DWB -> /cmd_vel_nav -> "
                            "twist_mux -> MOTOR. Kê bánh / giữ E-stop. Dừng: ros2 service call "
                            "/goal_path_bridge/cancel std_srvs/srv/Trigger (giữ deadman PS4 chỉ "
                            "ghi đè tạm thời: thả ra là DWB chạy tiếp)")]
    return [LogInfo(msg="[stage3] DWB test mode: vận tốc DWB -> /cmd_vel_dwb "
                        "(cách ly, không tới motor)")]


def include_stage2(launch_file, arguments, mode):
    launch_path = Path(get_package_share_directory("stage2_slam")) / "launch" / launch_file
    # GroupAction(scoped=True): ở Humble, launch_arguments của include ghi đè launch configuration
    # toàn cục (vd. "rviz": "false" cho Stage 2 sẽ tắt luôn RViz của planning).
    return GroupAction(
        scoped=True,
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mode"), "' == '", mode, "' and '",
            LaunchConfiguration("stage2"), "'.lower() == 'true'",
        ])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments=arguments.items(),
            ),
        ],
    )


def generate_launch_description():
    stage3_share = Path(get_package_share_directory("stage3_navigation"))

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # Cùng mặc định với stage2_slam localization.launch.py.
    default_map = Path.home() / "capstone_ws" / "capstone_project" / "maps" / "map.yaml"

    # 2D Pose Estimate: localization + csm -> /initialpose_guess (CSM -> /initialpose -> AMCL),
    # còn lại -> /initialpose (localization csm:=false: thẳng vào AMCL;
    # mapping: không node nào dùng).
    initialpose_topic = PythonExpression([
        "'/initialpose_guess' if '", LaunchConfiguration("mode"), "' == 'localization' and '",
        LaunchConfiguration("csm"), "'.lower() == 'true' else '/initialpose'",
    ])

    # Đầu ra vận tốc của controller_server (topic "cmd_vel" của nó).
    cmd_vel_topic = PythonExpression([
        "'/cmd_vel_nav' if '", LaunchConfiguration("navigation_enabled"),
        "'.lower() == 'true' else '/cmd_vel_dwb'",
    ])

    # Chỉ đòi LOCALIZED khi có localization_monitor (mode:=localization monitor:=true).
    require_localized = PythonExpression([
        "'", LaunchConfiguration("mode"), "' == 'localization' and '",
        LaunchConfiguration("monitor"), "'.lower() == 'true'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="localization",
            description="localization: map_server + AMCL (+ CSM/monitor). mapping: slam_toolbox",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB2",
            description="Cổng USB của RPLidar C1",
        ),
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/ttyUSB1",
            description="Cổng USB-UART của ESP32 (IMU), truyền xuống Stage 2 / ekf.launch.py",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=str(default_map),
            description="mode:=localization: file bản đồ .yaml cho map_server",
        ),
        DeclareLaunchArgument(
            "csm",
            default_value="true",
            description="mode:=localization: 2D Pose Estimate -> /initialpose_guess -> CSM "
                        "-> /initialpose. false: RViz gửi thẳng /initialpose",
        ),
        DeclareLaunchArgument(
            "monitor",
            default_value="true",
            description="mode:=localization: chạy localization_monitor (/localization/status)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(stage3_share / "config" / "nav2_params.yaml"),
            description="Tham số planner_server / global_costmap / lifecycle manager / bridge",
        ),
        DeclareLaunchArgument(
            "allow_unknown",
            default_value="false",
            description="true: A* được đi qua vùng unknown (hữu ích khi mapping). "
                        "false: chỉ qua vùng free",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="false",
            description="true: chạy controller_server (DWB) + local_costmap. "
                        "false: chỉ lập kế hoạch A*",
        ),
        DeclareLaunchArgument(
            "navigation_enabled",
            default_value="false",
            description="true: vận tốc DWB -> /cmd_vel_nav -> twist_mux -> MOTOR. "
                        "false: -> /cmd_vel_dwb (cách ly). Cần controller:=true",
        ),
        DeclareLaunchArgument(
            "auto_follow",
            default_value="false",
            description="true: gửi FollowPath ngay khi A* tìm được đường. "
                        "false: chỉ khi gọi /goal_path_bridge/follow. Cần controller:=true",
        ),
        DeclareLaunchArgument(
            "stage2",
            default_value="true",
            description="false: KHÔNG include Stage 2 (không LiDAR/motor/EKF/localization). "
                        "Chỉ dùng cho kiểm thử offline với nguồn /map, TF, scan, odom khác",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Mở RViz2 (planning.rviz). RViz của Stage 2 luôn tắt",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Mức log của planner_server / lifecycle manager / bridge",
        ),

        OpaqueFunction(function=_check_mode),

        # ---- Stage 2 (rviz:=false: chỉ mở RViz của Stage 3) ----
        include_stage2(
            "localization.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "imu_port": LaunchConfiguration("imu_port"),
                "map": LaunchConfiguration("map"),
                "csm": LaunchConfiguration("csm"),
                "monitor": LaunchConfiguration("monitor"),
                "rviz": "false",
            },
            "localization",
        ),
        include_stage2(
            "mapping.launch.py",
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "imu_port": LaunchConfiguration("imu_port"),
                "rviz": "false",
            },
            "mapping",
        ),

        # ---- Stage 3: A* global planning ----
        # A* (nav2_navfn_planner/NavfnPlanner, use_astar: true)
        # + global_costmap (static + inflation).
        # Action: /compute_path_to_pose. planner_server còn tự publish /plan khi thành công.
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "GridBased.allow_unknown": ParameterValue(
                        LaunchConfiguration("allow_unknown"), value_type=bool),
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # ---- Stage 3.2: DWB local controller (controller:=true) ----
        # controller_server (FollowPath = dwb_core::DWBLocalPlanner)
        # + local_costmap (obstacle + inflation).
        # Action: /follow_path. Không có BT navigator: FollowPath chỉ do goal_path_bridge gửi.
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
            remappings=[("cmd_vel", cmd_vel_topic)],
            arguments=["--ros-args", "--log-level", log_level],
            condition=IfCondition(LaunchConfiguration("controller")),
        ),

        # MỘT lifecycle manager cho Stage 3 (map_server/AMCL có lifecycle_manager_localization của
        # nav2_bringup). Hai khai báo loại trừ nhau theo controller: chỉ một node được chạy.
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planning",
            output="screen",
            parameters=[params_file],
            arguments=["--ros-args", "--log-level", log_level],
            condition=UnlessCondition(LaunchConfiguration("controller")),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planning",
            output="screen",
            parameters=[params_file, {"node_names": ["planner_server", "controller_server"]}],
            arguments=["--ros-args", "--log-level", log_level],
            condition=IfCondition(LaunchConfiguration("controller")),
        ),

        # /goal_pose (RViz Goal Pose) -> ComputePathToPose -> /global_path
        # (+ FollowPath khi được kích hoạt). Không phát lệnh vận tốc.
        Node(
            package="stage3_navigation",
            executable="goal_path_bridge",
            name="goal_path_bridge",
            output="screen",
            parameters=[
                params_file,
                {
                    "auto_follow": ParameterValue(
                        LaunchConfiguration("auto_follow"), value_type=bool),
                    "require_localized": ParameterValue(require_localized, value_type=bool),
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # RViz2 duy nhất: map, scan, AMCL, CSM, global/local costmap, Goal Pose, /global_path, DWB.
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_planning",
            output="log",
            arguments=["-d", str(stage3_share / "rviz" / "planning.rviz")],
            parameters=[{"use_sim_time": False}],
            remappings=[("/initialpose", initialpose_topic)],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
