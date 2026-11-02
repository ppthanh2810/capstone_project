"""S3 — điều hướng (+ camera): S2 (sim + LiDAR + định vị) + stage3 A* + CustomDWB + goal_path_bridge.

Giống stage3_navigation/launch/navigation.launch.py nhưng KHÔNG include nó (nó include stage2 thật -> motor,
LiDAR, IMU). Chạy lại trực tiếp các node của stage3 với nav2_params.yaml gốc + use_sim_time (RewrittenYaml cho
mọi node con: global_costmap, local_costmap, lifecycle manager):
  planner_server (NavFn A*) + global_costmap      -- lifecycle_manager_planning
  controller_server (CustomDWB | FollowPath) + local_costmap [+ lớp camera]  -- lifecycle_manager_navigation
  goal_path_bridge: /goal_pose -> A* -> /global_path -> FollowPath
Khác với xe thật (mặc định của sim, đổi được bằng tham số):
  navigation_enabled:=true  -> controller_server -> /cmd_vel_nav -> twist_mux (prio 10; PS4 100 vẫn thắng)
                               -> /cmd_vel_out -> zlac_sim. false = /cmd_vel_dwb (cách ly như xe thật).
  auto_follow:=true         -> 2D Goal Pose là robot chạy luôn. false = gọi /goal_path_bridge/follow như xe thật.
  env:=warehouse            -> kho AWS + maps/warehouse.yaml (S2b).
camera:=true: depth_camera D435 trong Gazebo + cloud_sim_adapter (-> /camera/camera/depth/color/points,
camera_depth_optical_frame) thay realsense2_camera; camera_layer:=auto -> fusion (config/camera_fusion.yaml).

Thao tác: RViz 2D Pose Estimate (định vị, như S2) -> chờ /localization/status LOCALIZED -> 2D Goal Pose.
Dừng: ros2 service call /goal_path_bridge/cancel std_srvs/srv/Trigger.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

LOCAL_PLANNERS = {'custom': 'CustomDWB', 'nav2_dwb': 'FollowPath'}
CAMERA_LAYERS = {'fusion': 'camera_fusion.yaml', 'voxel': 'camera_voxel.yaml', 'none': None}

# Tham số chuyển nguyên xuống s2_slam.launch.py
S2_ARGS = ('mode', 'env', 'aws_dir', 'world_map', 'map', 'world', 'wall_height', 'csm', 'monitor', 'gui',
           'x', 'y', 'yaw', 'init_pose', 'follow', 'joy', 'accel_time_ms', 'decel_time_ms', 'camera')


def is_true(value):
    return value.strip().lower() in ('true', '1', 'yes')


def launch_setup(context):
    pkg = get_package_share_directory('stage4_gazebo')
    stage3 = get_package_share_directory('stage3_navigation')

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    planner = arg('local_planner')
    if planner not in LOCAL_PLANNERS:
        raise RuntimeError(f'local_planner:={planner} không hợp lệ, chọn một trong {tuple(LOCAL_PLANNERS)}')
    camera = is_true(arg('camera'))
    layer = arg('camera_layer')
    if layer == 'auto':
        layer = 'fusion' if camera else 'none'
    if layer not in CAMERA_LAYERS:
        raise RuntimeError(f"camera_layer:={layer} không hợp lệ, chọn auto | {' | '.join(CAMERA_LAYERS)}")
    if layer != 'none' and not camera:
        raise RuntimeError(f'camera_layer:={layer} cần camera:=true (không có nguồn point cloud -> '
                           'local costmap không current -> controller dừng)')

    mode = arg('mode')
    localization = mode == 'localization'
    nav_enabled = is_true(arg('navigation_enabled'))
    require_localized = localization and is_true(arg('monitor'))
    log_level = arg('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]

    # nav2_params.yaml gốc, use_sim_time: True ở MỌI node (kể cả node con costmap)
    def sim_yaml(path):
        return RewrittenYaml(source_file=path, param_rewrites={'use_sim_time': 'True'}, convert_types=True)

    nav_params = sim_yaml(os.path.join(stage3, 'config', 'nav2_params.yaml'))
    controller_params = [nav_params]
    if CAMERA_LAYERS[layer]:
        controller_params.append(sim_yaml(os.path.join(stage3, 'config', CAMERA_LAYERS[layer])))

    actions = []

    # ---- S2: sim + S1 + LiDAR + định vị / mapping (RViz của S2 tắt, mở planning.rviz ở dưới) ----
    s2_args = {k: arg(k) for k in S2_ARGS}
    s2_args.update({
        'rviz': 'false',
        'bridge_config': os.path.join(pkg, 'config', 's3_navigation', 'bridge_s3.yaml'),
    })
    actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 's2_slam.launch.py')),
        launch_arguments=s2_args.items())]))

    # ---- Camera (thay realsense2_camera): depth_camera Gazebo -> optical frame ----
    if camera:
        actions.append(Node(
            package='stage4_gazebo',
            executable='cloud_sim_adapter',
            name='cloud_sim_adapter',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))

    # ---- A*: planner_server + global_costmap ----
    actions.append(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params, {'GridBased.allow_unknown': is_true(arg('allow_unknown'))}],
        arguments=ros_args,
    ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[nav_params],
        arguments=ros_args,
    ))

    # ---- Điều khiển cục bộ: controller_server (CustomDWB + FollowPath) + local_costmap ----
    cmd_vel_topic = '/cmd_vel_nav' if nav_enabled else '/cmd_vel_dwb'
    actions.append(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=controller_params,
        remappings=[('cmd_vel', cmd_vel_topic)],
        arguments=ros_args,
    ))
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav_params],
        arguments=ros_args,
    ))

    # ---- goal_path_bridge (A* một lần mỗi goal, không replan) ----
    actions.append(Node(
        package='stage3_navigation',
        executable='goal_path_bridge',
        name='goal_path_bridge',
        output='screen',
        parameters=[nav_params, {
            'use_sim_time': True,
            'auto_follow': is_true(arg('auto_follow')),
            'require_localized': require_localized,
            'controller_id': LOCAL_PLANNERS[planner],
        }],
        arguments=ros_args,
    ))

    # ---- RViz: planning.rviz của stage3 (map, costmap, đường A*, local plan, quỹ đạo DWB) ----
    if is_true(arg('rviz')):
        csm = localization and is_true(arg('csm'))
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_planning',
            output='log',
            arguments=['-d', arg('rviz_config')],
            parameters=[{'use_sim_time': True}],
            remappings=[('/initialpose', '/initialpose_guess')] if csm else [],
        ))

    actions.append(LogInfo(msg=(
        f'[s3] local planner {planner} ({LOCAL_PLANNERS[planner]}), lớp camera: {layer}, '
        + ('vận tốc -> /cmd_vel_nav -> twist_mux -> zlac_sim (robot SIM chạy)' if nav_enabled
           else 'vận tốc -> /cmd_vel_dwb (cách ly, robot không chạy)')
        + (', auto_follow: 2D Goal Pose là chạy' if is_true(arg('auto_follow'))
           else ', bám đường: ros2 service call /goal_path_bridge/follow std_srvs/srv/Trigger')
        + ('. Đặt 2D Pose Estimate trước, chờ LOCALIZED' if localization else ''))))
    return actions


def generate_launch_description():
    stage3 = get_package_share_directory('stage3_navigation')
    return LaunchDescription([
        # ---- chuyển xuống S2 ----
        DeclareLaunchArgument('mode', default_value='localization',
                              description='localization (map_server + AMCL + CSM + monitor) | mapping (slam_toolbox)'),
        DeclareLaunchArgument('env', default_value='warehouse',
                              description='warehouse: kho AWS + maps/warehouse.yaml | map: world sinh từ world_map'),
        DeclareLaunchArgument('aws_dir', default_value=os.path.join(os.path.expanduser('~'), 'capstone_ws',
                                                                    'third_party', 'aws_small_warehouse')),
        DeclareLaunchArgument('world_map', default_value=os.path.join(os.path.expanduser('~'), 'capstone_ws',
                                                                      'capstone_project', 'maps', 'map.yaml')),
        DeclareLaunchArgument('map', default_value='', description='Map cho AMCL; rỗng = mặc định của env'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('wall_height', default_value='1.0'),
        DeclareLaunchArgument('csm', default_value='true'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='true: bridge chỉ bám đường khi LOCALIZED (như xe thật)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('init_pose', default_value='false',
                              description='true: tự gửi pose spawn thay 2D Pose Estimate'),
        DeclareLaunchArgument('follow', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('accel_time_ms', default_value='500'),
        DeclareLaunchArgument('decel_time_ms', default_value='500'),
        DeclareLaunchArgument('camera', default_value='false',
                              description='true: depth camera D435 sim + cloud_sim_adapter (tốn GPU)'),
        # ---- riêng S3 ----
        DeclareLaunchArgument('local_planner', default_value='custom', description='custom (CustomDWB) | nav2_dwb'),
        DeclareLaunchArgument('navigation_enabled', default_value='true',
                              description='true: DWB -> /cmd_vel_nav -> twist_mux -> zlac_sim. false: /cmd_vel_dwb'),
        DeclareLaunchArgument('auto_follow', default_value='true',
                              description='true: bám đường ngay khi A* có đường. false: /goal_path_bridge/follow'),
        DeclareLaunchArgument('camera_layer', default_value='auto',
                              description='auto (fusion nếu camera:=true) | fusion | voxel | none'),
        DeclareLaunchArgument('allow_unknown', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(stage3, 'rviz', 'planning.rviz')),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_setup),
    ])
