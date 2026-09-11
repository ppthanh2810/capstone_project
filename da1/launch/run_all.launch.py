from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('da1')
    joy_config = os.path.join(pkg_share, 'config', 'config.yaml')

    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    detect_node = Node(
        package='da1',
        executable='detect',
        name='yolo_person_percent_node',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'model': '/home/yolo11n.onnx',
            'conf': 0.25,
            'iou': 0.45,
            'provider': 'cpu',
            'execution_mode': 'sequential',
            'intra_op_num_threads': 2,
            'inter_op_num_threads': 1,
            'image_queue_depth': 1,
            'warmup_runs': 2,
            'opencv_num_threads': 1,
            'log_percent_period_sec': 0.0,
            'publish_debug_image': True,
            'show_window': False,
            'detections_topic': '/person_detections',
            'percent_topic': '/PT/percent',
            'no_person_percent': 0.0,
        }],
        output='screen',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.08,
            'autorepeat_rate': 15.0
        }],
        prefix='nice -n 5',
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[joy_config],
        prefix='nice -n 5',
    )

    m_to_m_node = Node(
        package='da1',
        executable='m_to_m',
        name='m_to_m',
        prefix='nice -n 8',
    )

    move_node = Node(
        package='da1',
        executable='move',
        name='move',
        prefix='nice -n 8',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'align_depth.enable': 'true'
            }.items()
        ),

        # Cho camera tiến trình khởi động, sau đó bật detect càng sớm càng tốt.
        TimerAction(
            period=0.5,
            actions=[detect_node],
        ),

        # Chỉ bật teleop/control sau khi detect đã start và có thời gian warm-up model.
        RegisterEventHandler(
            OnProcessStart(
                target_action=detect_node,
                on_start=[
                    LogInfo(msg='detect đã start -> trì hoãn teleop/control để ưu tiên inference'),
                    TimerAction(
                        period=2.0,
                        actions=[joy_node, joy_teleop_node],
                    ),
                    TimerAction(
                        period=4.0,
                        actions=[m_to_m_node, move_node],
                    ),
                ],
            )
        ),
    ])
