"""
Self-contained demo launch for new_arm MoveIt 2.

Unlike the Setup-Assistant convention (which splits things across rsp.launch.py,
move_group.launch.py, etc.), this file declares every node inline so the package
has no hidden launch-file dependencies. It brings up:

  - robot_state_publisher          (from config/new_arm.urdf.xacro)
  - static TF  world -> arm_base_link   (the SRDF virtual joint)
  - ros2_control_node              (mock hardware)
  - joint_state_broadcaster        (publishes /joint_states)
  - new_arm_controller             (executes MoveIt trajectories)
  - move_group                     (planning, IK, collision checking)
  - rviz2                          (MotionPlanning panel)

Run:  ros2 launch new_arm_moveit_config demo.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("new_arm", package_name="new_arm_moveit_config")
        .robot_description(file_path="config/new_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/new_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    pkg_share = get_package_share_directory("new_arm_moveit_config")
    rviz_config = os.path.join(pkg_share, "config", "moveit.rviz")
    ros2_controllers = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    # --- robot_state_publisher: publishes the URDF + link TFs ---
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # --- static TF for the 'world' virtual joint (world -> arm_base_link) ---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "arm_base_link"],
    )

    # --- ros2_control: mock hardware + controller manager ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers],
        output="both",
    )

    # --- controller spawners ---
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["new_arm_controller", "-c", "/controller_manager"],
    )

    # --- move_group: the MoveIt planning server ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # --- RViz with the MotionPlanning panel ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Spawn controllers in order: broadcaster first, then the arm controller,
    # so /joint_states is available before trajectory execution starts.
    delay_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_spawner],
        )
    )

    return LaunchDescription(
        [
            rsp_node,
            static_tf,
            ros2_control_node,
            jsb_spawner,
            delay_arm_after_jsb,
            move_group_node,
            rviz_node,
        ]
    )
