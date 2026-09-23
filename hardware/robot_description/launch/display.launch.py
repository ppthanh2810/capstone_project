"""Visualize AMR URDF and publish its TF tree (ROS 2 Humble).

For real hardware pass publish_default_joint_states:=false and publish actual
left_wheel_joint/right_wheel_joint positions on /joint_states instead.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_description'))
    model = (package_share / 'urdf' / 'robot.urdf').read_text(encoding='utf-8')
    rviz_config = str(package_share / 'rviz' / 'robot_description.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz2.'),
        DeclareLaunchArgument('publish_default_joint_states', default_value='true',
                              description='Publish dummy wheel joint positions for TF demo only.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': model,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_default_joint_states')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
