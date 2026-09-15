from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory('imu_bno055')

    config = os.path.join(
        pkg_share,
        'config',
        'bno055.yaml'
    )

    bno055_node = Node(
        package='imu_bno055',
        executable='bno055_node',
        name='bno055_node',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([
        bno055_node
    ])