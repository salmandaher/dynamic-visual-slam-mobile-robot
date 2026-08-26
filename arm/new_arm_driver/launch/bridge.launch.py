import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('new_arm_driver'), 'config', 'calibration.yaml')

    return LaunchDescription([
        Node(
            package='new_arm_driver',
            executable='arm_bridge',
            name='arm_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
