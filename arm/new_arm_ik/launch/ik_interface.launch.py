from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare('new_arm_description')
    urdf_path = PathJoinSubstitution([desc_share, 'urdf', 'new_arm.urdf'])
    rviz_config = PathJoinSubstitution([desc_share, 'rviz', 'new_arm.rviz'])

    robot_description = Command(['xacro ', urdf_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='new_arm_ik',
            executable='ik_gui_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
