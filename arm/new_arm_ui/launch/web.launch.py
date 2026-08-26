"""Launch the new_arm web UI (rclnodejs Node server).

This only starts the web server. Bring up the planning/execution stack separately:

    ros2 launch new_arm_moveit_config arm_bringup.launch.py        # move_group + arm_bridge (+RViz)
    ros2 launch new_arm_ui web.launch.py                           # this UI

Then open http://localhost:8080 .

Prerequisites (once):
  * Node 18 or 20 LTS (rclnodejs does NOT build on Node 25):
        nvm install 20 && nvm use 20
  * install Node deps with a ROS 2 env sourced so rclnodejs can generate the
    message bindings:
        cd $(ros2 pkg prefix new_arm_ui)/share/new_arm_ui/web && npm install

NOTE: this launch calls `node` from PATH, so run it from a shell where
`nvm use 20` is active (or set node_bin:= to a specific node binary).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    web_dir = os.path.join(get_package_share_directory('new_arm_ui'), 'web')

    port_arg = DeclareLaunchArgument('port', default_value='8080',
                                     description='HTTP port for the web UI.')
    node_bin_arg = DeclareLaunchArgument('node_bin', default_value='node',
                                         description='node binary to use (Node 18/20).')

    server = ExecuteProcess(
        cmd=[LaunchConfiguration('node_bin'), os.path.join(web_dir, 'server.js')],
        cwd=web_dir,
        additional_env={'PORT': LaunchConfiguration('port')},
        output='screen',
    )

    return LaunchDescription([port_arg, node_bin_arg, server])
