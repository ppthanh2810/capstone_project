from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'stage0_ps4_control')

    ps4_config = os.path.join(
        pkg_share,
        'config',
        'ps4.yaml')

    twist_mux_config = os.path.join(
        pkg_share,
        'config',
        'twist_mux.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[ps4_config],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ],
    )

    m_to_m_node = Node(
        package='stage0_ps4_control',
        executable='m_to_m',
        name='m_to_m',
        remappings=[
            ('/cmd_vel_joy', '/cmd_vel'),
        ],
    )

    move_node = Node(
        package='stage0_ps4_control',
        executable='move',
        name='move',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        twist_mux_node,
        m_to_m_node,
        move_node,
    ])