#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    m_to_m_node = Node(
        package='da1',
        executable='m_to_m_test_move',
        name='m_to_m',
        # output='screen',
        # emulate_tty=True
    )

    move_node = Node(
        package='da1',
        executable='move',
        name='move',
        # output='screen',
        # emulate_tty=True
    )

    return LaunchDescription([
        m_to_m_node,
        move_node
    ])
