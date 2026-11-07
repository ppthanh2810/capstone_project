"""S2 — LiDAR + SLAM / định vị: S1 + gpu_lidar + lidar_sim_adapter + scan_rotate + laser_filters
+ (mode:=mapping) slam_toolbox  |  (mode:=localization, mặc định) map_server + AMCL + CSM + localization_monitor.

World mặc định sinh từ bản đồ thật (`world_map`, mặc định maps/map.yaml) bằng scripts/map_to_world.py:
ô occupied đùn thành tường cao `wall_height` m, toạ độ world = frame map -> pose AMCL so thẳng được với
/sim/ground_truth/odom. Spawn mặc định (0, 0, 0) = điểm bắt đầu vẽ map trên xe (ô trống).

Thay sllidar_ros2 bằng gpu_lidar + lidar_sim_adapter (/scan_raw cùng quy ước driver: góc 0 ra sau, laser_raw).
scan_rotate, laser_filters, slam_toolbox, AMCL, CSM, monitor chạy nguyên bản với YAML gốc stage2 + use_sim_time.
Không include launch stage2 (chúng mở LiDAR / motor / IMU thật).
  /sim/scan -> lidar_sim_adapter -> /scan_raw -> scan_rotate -> /scan -> laser_filters -> /scan_filtered
  mapping:      /scan_filtered -> slam_toolbox -> /map + TF map->odom
  localization: map_server /map + AMCL (TF map->odom); RViz 2D Pose Estimate -> /initialpose_guess (csm:=true)
                -> scan_match_initializer -> /initialpose; localization_monitor -> /localization/status
Vị trí spawn: lưu vị trí hiện tại bằng `ros2 run stage4_gazebo save_pose.py --env <env>` -> lần sau mở env đó
robot spawn tại đó (x/y/yaw truyền tay vẫn ưu tiên). Định vị vẫn bằng 2D Pose Estimate trong RViz như xe thật;
init_pose:=true để tự gửi pose spawn thay cho thao tác đó (mặc định false).
Lưu map (mapping): ros2 run nav2_map_server map_saver_cli -f <stage4_gazebo/maps/tên>
                   --ros-args -p use_sim_time:=true

env:=warehouse: world kho AWS RoboMaker Small Warehouse (worlds/aws_small_warehouse.sdf, model ngoài repo:
scripts/fetch_aws_warehouse.sh -> ~/capstone_ws/third_party/aws_small_warehouse, đổi bằng aws_dir:=).
Map định vị mặc định stage4_gazebo/maps/warehouse.yaml (vẽ bằng mode:=mapping env:=warehouse rồi lưu tên đó).
env:=house: nhà AWS RoboMaker Small House (worlds/aws_small_house.sdf; scripts/fetch_aws_house.sh ->
~/capstone_ws/third_party/aws_small_house), map mặc định stage4_gazebo/maps/house.yaml.
env:=office: văn phòng Husarion (worlds/husarion_office.sdf; scripts/fetch_husarion_office.sh ->
~/capstone_ws/third_party/husarion_gz_worlds + mesh Fuel trong ~/.ignition/fuel), map stage4_gazebo/maps/office.yaml.
aws_dir rỗng = thư mục mặc định của env.
"""

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = Path.home() / 'capstone_ws' / 'capstone_project'
DEFAULT_MAP = REPO / 'maps' / 'map.yaml'          # giống default của stage2 localization.launch.py
SIM_MAPS_DIR = REPO / 'stage4_gazebo' / 'maps'    # map vẽ trong sim lưu ở đây (không ghi đè maps/map.yaml)
WAREHOUSE_MAP = SIM_MAPS_DIR / 'warehouse.yaml'    # map chính của world kho AWS
THIRD_PARTY = Path.home() / 'capstone_ws' / 'third_party'
# env có model ngoài repo (AWS, Husarion): thư mục model mặc định, script tải, world (trong share/worlds),
# map định vị mặc định, thư mục cần thêm vào IGN_GAZEBO_RESOURCE_PATH (tương đối aws_dir)
AWS_ENVS = {
    'warehouse': dict(dir=THIRD_PARTY / 'aws_small_warehouse', fetch='fetch_aws_warehouse.sh',
                      world='aws_small_warehouse.sdf', map=WAREHOUSE_MAP,
                      resource=('', 'models')),   # mesh kho dùng file://models/...
    'house': dict(dir=THIRD_PARTY / 'aws_small_house', fetch='fetch_aws_house.sh',
                  world='aws_small_house.sdf', map=SIM_MAPS_DIR / 'house.yaml',
                  resource=('models',)),
    # Văn phòng Husarion: model://<Tên> trong <dir>/models, đồ đạc OpenRobotics là URI Fuel (cache ~/.ignition/fuel)
    'office': dict(dir=THIRD_PARTY / 'husarion_gz_worlds', fetch='fetch_husarion_office.sh',
                   world='husarion_office.sdf', map=SIM_MAPS_DIR / 'office.yaml',
                   resource=('models',)),
}
SPAWN_STATE_DIR = Path.home() / '.ros' / 'stage4_gazebo'   # vị trí spawn đã lưu (scripts/save_pose.py)


def load_saved_pose(env):
    """(x, y, yaw) lưu bằng `ros2 run stage4_gazebo save_pose.py --env <env>`, hoặc None."""
    import yaml
    path = SPAWN_STATE_DIR / f'spawn_{env}.yaml'
    try:
        d = yaml.safe_load(path.read_text()) or {}
        return float(d['x']), float(d['y']), float(d['yaw'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def load_map_to_world():
    path = os.path.join(get_package_prefix('stage4_gazebo'), 'lib', 'stage4_gazebo', 'map_to_world.py')
    spec = importlib.util.spec_from_file_location('map_to_world', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def is_true(value):
    return value.strip().lower() in ('true', '1', 'yes')


def launch_setup(context):
    pkg = get_package_share_directory('stage4_gazebo')
    stage2 = get_package_share_directory('stage2_slam')
    slam_share = get_package_share_directory('slam_toolbox')
    sim_time = {'use_sim_time': True}

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mode = arg('mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError(f"mode phải là 'mapping' hoặc 'localization' (nhận '{mode}')")
    localization = mode == 'localization'
    csm = localization and is_true(arg('csm'))
    monitor = localization and is_true(arg('monitor'))

    env = arg('env')
    world = arg('world')
    if env == 'map':
        world_map = os.path.expanduser(arg('world_map'))
        if not world:
            world = load_map_to_world().generate(world_map, height=float(arg('wall_height')))
        loc_map = os.path.expanduser(arg('map')) or world_map
    elif env in AWS_ENVS:
        cfg = AWS_ENVS[env]
        aws = os.path.expanduser(arg('aws_dir')) or str(cfg['dir'])
        if not os.path.isdir(os.path.join(aws, 'models')):
            raise RuntimeError(f'Chưa có model của env:={env} ở {aws}. Chạy: '
                               f'{os.path.join(get_package_prefix("stage4_gazebo"), "lib", "stage4_gazebo")}'
                               f'/{cfg["fetch"]}')
        # sim_world.launch.py (include bên dưới) ghép thêm giá trị đang có của biến này.
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.pathsep.join(
            [os.path.join(aws, r) if r else aws for r in cfg['resource']]
            + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
        world = world or os.path.join(pkg, 'worlds', cfg['world'])
        loc_map = os.path.expanduser(arg('map')) or str(cfg['map'])
        if localization and not os.path.isfile(loc_map):
            raise RuntimeError(f'Chưa có map {loc_map}. Vẽ trước: ros2 launch stage4_gazebo s2_slam.launch.py '
                               f'env:={env} mode:=mapping, rồi lưu bằng map_saver_cli -f '
                               f'{Path(cfg["map"]).with_suffix("")}')
    else:
        raise RuntimeError(f"env phải là 'map', {', '.join(repr(k) for k in AWS_ENVS)} (nhận '{env}')")

    # Vị trí spawn: x/y/yaw truyền tay > vị trí đã lưu cho env (save_pose.py) > (0, 0, 0)
    given = [arg(k) for k in ('x', 'y', 'yaw')]
    saved = load_saved_pose(env)
    if any(given):
        spawn = tuple(float(v) if v else 0.0 for v in given)
        spawn_src = 'x/y/yaw'
    elif saved:
        spawn, spawn_src = saved, f'đã lưu {SPAWN_STATE_DIR}/spawn_{env}.yaml'
    else:
        spawn, spawn_src = (0.0, 0.0, 0.0), 'mặc định'
    init_pose = arg('init_pose').strip().lower()
    send_init = localization and init_pose in ('true', '1', 'yes')

    common = ['gui', 'follow', 'joy', 'accel_time_ms', 'decel_time_ms', 'camera']
    s1_args = {k: arg(k) for k in common}
    s1_args.update({
        'x': str(spawn[0]), 'y': str(spawn[1]), 'yaw': str(spawn[2]),
        'world': world,
        'rviz': 'false',  # RViz mở ở dưới (cần remap /initialpose khi csm)
        'bridge_config': arg('bridge_config') or os.path.join(pkg, 'config', 's2_slam', 'bridge_s2.yaml'),
    })
    # GroupAction(scoped): ở Humble launch_arguments của include rò ra scope ngoài (vd. rviz:=false)
    s1 = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 's1_odom.launch.py')),
        launch_arguments=s1_args.items())])

    actions = [
        LogInfo(msg=f'[s2_slam] spawn ({spawn[0]:+.3f}, {spawn[1]:+.3f}, {spawn[2]:+.3f} rad) từ {spawn_src}'
                    + (' | tự gửi pose ban đầu' if send_init
                       else (' | đặt 2D Pose Estimate trong RViz' if localization else ''))),
        LogInfo(msg=f'[s2_slam] env={env} | mode={mode} | world={world} | ' +
                (f'map={loc_map} | csm={csm} | monitor={monitor}' if localization else
                 f'lưu map: ros2 run nav2_map_server map_saver_cli -f {SIM_MAPS_DIR}/<tên> '
                 '--ros-args -p use_sim_time:=true')),
        s1,
        Node(
            package='stage4_gazebo',
            executable='lidar_sim_adapter',
            name='lidar_sim_adapter',
            output='screen',
            parameters=[sim_time],
        ),
        # Giống stage2_slam/launch/scan.launch.py
        Node(
            package='stage2_slam',
            executable='scan_rotate',
            name='scan_rotate',
            output='screen',
            parameters=[{'angle_offset': 3.141592653589793, 'frame_id': 'laser_frame'}, sim_time],
            remappings=[('scan_in', '/scan_raw'), ('scan_out', '/scan')],
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',
            output='screen',
            parameters=[os.path.join(stage2, 'config', 'angular_filter.yaml'), sim_time],
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
        ),
    ]

    if not localization:
        # Giống stage2_slam/launch/mapping.launch.py (chỉ đổi use_sim_time)
        actions.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml'),
                {
                    'use_sim_time': True,
                    'scan_topic': '/scan_filtered',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_footprint',
                    'mode': 'mapping',
                    'min_laser_range': 0.10,
                    'max_laser_range': 16.0,
                    'map_update_interval': 1.0,
                    'minimum_travel_distance': 0.2,
                    'minimum_travel_heading': 0.2,
                },
            ],
        ))
    else:
        # Giống stage2_slam/launch/localization.launch.py; nav2_bringup tự RewrittenYaml use_sim_time
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': loc_map,
                'params_file': os.path.join(stage2, 'config', 'amcl_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_composition': 'False',
            }.items())]))
        if csm:
            actions.append(Node(
                package='stage2_slam',
                executable='scan_match_initializer',
                name='scan_match_initializer',
                output='screen',
                parameters=[os.path.join(stage2, 'config', 'scan_match_initializer.yaml'), sim_time],
            ))
        if monitor:
            actions.append(Node(
                package='stage2_slam',
                executable='localization_monitor',
                name='localization_monitor',
                output='screen',
                parameters=[os.path.join(stage2, 'config', 'localization_monitor.yaml'), sim_time],
            ))

    if send_init:
        # Thay thao tác 2D Pose Estimate: world Gazebo = frame map nên vị trí spawn chính là pose ban đầu
        actions.append(ExecuteProcess(
            cmd=[os.path.join(get_package_prefix('stage4_gazebo'), 'lib', 'stage4_gazebo', 'initial_pose.py'),
                 '--x', str(spawn[0]), '--y', str(spawn[1]), '--yaw', str(spawn[2]),
                 '--topic', '/initialpose_guess' if csm else '/initialpose'],
            name='initial_pose', output='screen'))

    if is_true(arg('rviz')):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_sim',
            arguments=['-d', arg('rviz_config')],
            parameters=[sim_time],
            # csm:=true: 2D Pose Estimate -> /initialpose_guess -> CSM (như stage2/stage3)
            remappings=[('/initialpose', '/initialpose_guess')] if csm else [],
        ))
    return actions


def generate_launch_description():
    stage2 = get_package_share_directory('stage2_slam')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localization', description='localization | mapping'),
        DeclareLaunchArgument('env', default_value='map',
                              description='map: world sinh từ world_map | warehouse: kho AWS Small Warehouse | '
                                          'house: nhà AWS Small House | office: văn phòng Husarion'),
        DeclareLaunchArgument('aws_dir', default_value='',
                              description='Thư mục model ngoài repo (env:=warehouse|house|office); rỗng = '
                                          '~/capstone_ws/third_party/aws_small_<env> | husarion_gz_worlds'),
        DeclareLaunchArgument('world_map', default_value=str(DEFAULT_MAP),
                              description='env:=map: bản đồ (.yaml) dùng để SINH world Gazebo'),
        DeclareLaunchArgument('map', default_value='',
                              description='Bản đồ cho map_server/AMCL (mode:=localization); rỗng = world_map '
                                          '(env:=map) hoặc stage4_gazebo/maps/<env>.yaml '
                                          '(env:=warehouse|house|office)'),
        DeclareLaunchArgument('world', default_value='',
                              description='File world .sdf; rỗng = sinh từ world_map'),
        DeclareLaunchArgument('wall_height', default_value='1.0',
                              description='Chiều cao tường sinh từ bản đồ (m)'),
        DeclareLaunchArgument('csm', default_value='true',
                              description='true: 2D Pose Estimate -> CSM -> /initialpose (như stage3)'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='true: localization_monitor -> /localization/status'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='', description='Spawn x (m); rỗng = vị trí đã lưu hoặc 0'),
        DeclareLaunchArgument('y', default_value='', description='Spawn y (m)'),
        DeclareLaunchArgument('yaw', default_value='', description='Spawn yaw (rad)'),
        DeclareLaunchArgument('init_pose', default_value='false',
                              description='localization: false = tự đặt 2D Pose Estimate trong RViz (như xe thật); '
                                          'true = tự gửi pose spawn thay thao tác đó'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(stage2, 'rviz', 'localization.rviz'),
                              description='Mặc định RViz localization của stage2 (Fixed Frame map)'),
        DeclareLaunchArgument('follow', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('accel_time_ms', default_value='500'),
        DeclareLaunchArgument('decel_time_ms', default_value='500'),
        DeclareLaunchArgument('camera', default_value='false', description='true: depth camera D435 (S3)'),
        DeclareLaunchArgument('bridge_config', default_value='',
                              description='ros_gz_bridge config; rỗng = config/s2_slam/bridge_s2.yaml (S3 truyền file riêng)'),
        OpaqueFunction(function=launch_setup),
    ])
