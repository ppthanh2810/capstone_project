from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Stage 3.2: planning.launch.py (Stage 2 + A* + global_costmap + RViz) + bộ điều khiển cục bộ:
# controller_server (CustomDWB / Nav2 DWB) + local_costmap + goal_path_bridge có FollowPath.
# CẢNH BÁO: như planning.launch.py, Stage 2 chạy zlac8015d_move (Modbus, motor) và teleop PS4.
# Vận tốc DWB:
#   navigation_enabled:=false (mặc định) -> /cmd_vel_dwb: KHÔNG node nào nghe,
#                                           cách ly khỏi twist_mux/motor
#   navigation_enabled:=true -> /cmd_vel_nav -> twist_mux (prio 10, PS4 prio 100 thắng) -> motor
# DWB chỉ chạy khi goal_path_bridge gửi FollowPath: service /goal_path_bridge/follow
# hoặc auto_follow:=true.
# Bộ điều khiển cục bộ (cả hai được nạp trong controller_server;
# goal_path_bridge chọn bằng controller_id):
#   custom   -> "CustomDWB"  = stage3_navigation::DwbController (tự viết, src/dwb_controller.cpp)
#   nav2_dwb -> "FollowPath" = dwb_core::DWBLocalPlanner (Nav2)
LOCAL_PLANNERS = {"custom": "CustomDWB", "nav2_dwb": "FollowPath"}

# Lớp camera trong local_costmap (M2): file YAML nạp CHỒNG lên params_file cho controller_server.
#   fusion -> config/camera_fusion.yaml (stage3_navigation::FusionLayer, hợp nhất camera + LiDAR)
#   voxel  -> config/camera_voxel.yaml  (nav2_costmap_2d::VoxelLayer, đối chứng)
#   none   -> chỉ LiDAR (như trước)
#   auto   -> fusion nếu camera:=true, ngược lại none
CAMERA_LAYERS = {"fusion": "camera_fusion.yaml", "voxel": "camera_voxel.yaml", "none": None}

# Tham số chuyển nguyên xuống planning.launch.py (mode được planning.launch.py kiểm tra).
PLANNING_ARGS = ("mode", "serial_port", "imu_port", "map", "csm", "monitor", "params_file",
                 "allow_unknown", "stage2", "rviz", "log_level")


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _camera_layer(context):
    layer = LaunchConfiguration("camera_layer").perform(context)
    if layer == "auto":
        return "fusion" if _is_true(context, "camera") else "none"
    if layer not in CAMERA_LAYERS:
        raise RuntimeError(
            f"camera_layer:={layer} không hợp lệ, chọn một trong "
            f"{('auto',) + tuple(CAMERA_LAYERS)}")
    return layer


def _controller_server(context):
    # controller_server (CustomDWB + FollowPath = dwb_core::DWBLocalPlanner)
    # + local_costmap (obstacle [+ camera] + inflation).
    # Action: /follow_path. Không có BT navigator: FollowPath chỉ do goal_path_bridge gửi.
    params = [LaunchConfiguration("params_file")]
    # Đầu ra vận tốc của controller_server (topic "cmd_vel" của nó).
    cmd_vel_topic = "/cmd_vel_nav" if _is_true(context, "navigation_enabled") else "/cmd_vel_dwb"
    layer = _camera_layer(context)
    actions = []
    if CAMERA_LAYERS[layer]:
        overlay = (Path(get_package_share_directory("stage3_navigation")) / "config" /
                   CAMERA_LAYERS[layer])
        params.append(str(overlay))
        actions.append(LogInfo(
            msg=f"[stage3] local_costmap + lớp camera '{layer}' ({overlay.name})"))
        if not _is_true(context, "camera"):
            actions.append(LogInfo(
                msg="[stage3] camera:=false: không chạy driver RealSense, lớp camera cần nguồn "
                    "point cloud khác (vd. bag), nếu không costmap "
                    "không current"))
    actions.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=params,
        remappings=[("cmd_vel", cmd_vel_topic)],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    ))
    return actions


def _check_args(context):
    planner = LaunchConfiguration("local_planner").perform(context)
    if planner not in LOCAL_PLANNERS:
        raise RuntimeError(
            f"local_planner:={planner} không hợp lệ, chọn một trong {tuple(LOCAL_PLANNERS)}")
    _camera_layer(context)
    warnings = []
    if _is_true(context, "navigation_enabled"):
        return warnings + [LogInfo(
            msg="[stage3] !!! navigation_enabled:=true: DWB -> /cmd_vel_nav -> "
                "twist_mux -> MOTOR. Kê bánh / giữ E-stop. Dừng: ros2 service call "
                "/goal_path_bridge/cancel std_srvs/srv/Trigger (giữ deadman PS4 chỉ "
                "ghi đè tạm thời: thả ra là DWB chạy tiếp)")]
    return warnings + [LogInfo(
        msg=f"[stage3] local planner '{planner}' ({LOCAL_PLANNERS[planner]}), "
            "test mode: vận tốc -> /cmd_vel_dwb (cách ly, không tới motor)")]


def generate_launch_description():
    stage3_share = Path(get_package_share_directory("stage3_navigation"))

    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")

    # Cùng mặc định với planning.launch.py / stage2_slam localization.launch.py.
    default_map = Path.home() / "capstone_ws" / "capstone_project" / "maps" / "map.yaml"

    # controller_id mà goal_path_bridge gửi trong FollowPath (chọn plugin của controller_server).
    controller_id = PythonExpression([
        "'FollowPath' if '", LaunchConfiguration("local_planner"),
        "' == 'nav2_dwb' else 'CustomDWB'",
    ])

    # Chỉ đòi LOCALIZED khi có localization_monitor (mode:=localization monitor:=true).
    require_localized = PythonExpression([
        "'", LaunchConfiguration("mode"), "' == 'localization' and '",
        LaunchConfiguration("monitor"), "'.lower() == 'true'",
    ])

    planning_launch = stage3_share / "launch" / "planning.launch.py"
    planning_arguments = {name: LaunchConfiguration(name) for name in PLANNING_ARGS}
    # Bridge chạy ở dưới (có FollowPath), không chạy bản của planning.launch.py.
    planning_arguments["goal_path_bridge"] = "false"

    return LaunchDescription([
        # ---- Tham số chung với planning.launch.py ----
        DeclareLaunchArgument(
            "mode",
            default_value="localization",
            description="localization: map_server + AMCL (+ CSM/monitor). mapping: slam_toolbox",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB1",
            description="Cổng USB của RPLidar C1",
        ),
        DeclareLaunchArgument(
            "imu_port",
            default_value="/dev/ttyUSB0",
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
            description="mode:=localization: chạy localization_monitor (/localization/status); "
                        "true -> bridge chỉ bám đường khi LOCALIZED",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=str(stage3_share / "config" / "nav2_params.yaml"),
            description="Tham số planner/controller server, costmap, lifecycle manager, bridge",
        ),
        DeclareLaunchArgument(
            "allow_unknown",
            default_value="false",
            description="true: A* được đi qua vùng unknown (hữu ích khi mapping). "
                        "false: chỉ qua vùng free",
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
            description="Mở RViz2 (planning.rviz: map, costmap, đường đi; "
                        "+ ảnh màu và point cloud camera khi camera:=true). "
                        "RViz của Stage 2 luôn tắt",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Mức log của các node Stage 3",
        ),
        # ---- Tham số riêng Stage 3.2 ----
        DeclareLaunchArgument(
            "local_planner",
            default_value="custom",
            description="custom: bộ điều khiển Dynamic Window tự viết "
                        "(CustomDWB, dwb_controller.cpp). "
                        "nav2_dwb: dwb_core::DWBLocalPlanner của Nav2 (FollowPath)",
        ),
        DeclareLaunchArgument(
            "navigation_enabled",
            default_value="false",
            description="true: vận tốc DWB -> /cmd_vel_nav -> twist_mux -> MOTOR. "
                        "false: -> /cmd_vel_dwb (cách ly)",
        ),
        DeclareLaunchArgument(
            "camera",
            default_value="false",
            description="true: chạy driver RealSense D435 (camera.launch.py, THIẾT BỊ THẬT) "
                        "và mặc định thêm lớp hợp nhất camera + LiDAR vào local_costmap",
        ),
        DeclareLaunchArgument(
            "camera_layer",
            default_value="auto",
            description="Lớp camera trong local_costmap: auto (fusion nếu camera:=true, ngược lại "
                        "none) | fusion (FusionLayer) | voxel (VoxelLayer, đối chứng) | none",
        ),
        DeclareLaunchArgument(
            "auto_follow",
            default_value="false",
            description="true: gửi FollowPath ngay khi A* tìm được đường. "
                        "false: chỉ khi gọi /goal_path_bridge/follow",
        ),

        OpaqueFunction(function=_check_args),

        # ---- Stage 2 + A* (planner_server, global_costmap, lifecycle_manager_planning, RViz) ----
        # scoped: launch_arguments của include không ghi đè launch configuration của file này.
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(planning_launch)),
                    launch_arguments=planning_arguments.items(),
                ),
            ],
        ),

        # ---- Camera RealSense (M2), chỉ khi camera:=true: THIẾT BỊ THẬT ----
        # Ảnh màu luôn bật để xem trong planning.rviz (lớp camera chỉ dùng point cloud độ sâu).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(stage3_share / "launch" / "camera.launch.py")),
            launch_arguments={"enable_color": "true"}.items(),
            condition=IfCondition(LaunchConfiguration("camera")),
        ),

        # ---- Stage 3.2: bộ điều khiển cục bộ (controller_server + local_costmap) ----
        OpaqueFunction(function=_controller_server),

        # Lifecycle manager riêng cho controller_server
        # (planner_server thuộc lifecycle_manager_planning của planning.launch.py).
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[params_file],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        # /goal_pose -> ComputePathToPose -> /global_path, + FollowPath khi được kích hoạt
        # (/goal_path_bridge/follow hoặc auto_follow:=true). Không phát lệnh vận tốc.
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
                    "controller_id": controller_id,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
    ])
