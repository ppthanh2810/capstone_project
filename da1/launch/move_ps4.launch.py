from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('da1')
    joy_config = os.path.join(pkg_share, 'config', 'config.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.08,
            'autorepeat_rate': 15.0
        }],
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[joy_config],
    )

    m_to_m_node = Node(
        package='da1',
        executable='m_to_m',
        name='m_to_m',
    )

    move_node = Node(
        package='da1',
        executable='move',
        name='move',
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        m_to_m_node,
        move_node,
    ])