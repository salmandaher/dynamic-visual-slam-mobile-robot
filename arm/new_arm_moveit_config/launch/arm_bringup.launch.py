"""Bring up MoveIt + RViz wired to the REAL arm (via new_arm_driver/arm_bridge).

Difference from demo.launch.py: demo uses mock ros2_control hardware (pure sim).
This launch instead routes BOTH state and execution through arm_bridge, so:

  * /joint_states comes from arm_bridge  -> RViz mirrors the real servo feedback
    (and echoes commanded positions when no arm is attached, so RViz still works);
  * MoveIt's FollowJointTrajectory goes to arm_bridge's action server, which
    streams waypoints to the ESP32 over /arm/servo_command.

Result: plan in RViz (drag the interactive marker OR type a goal) -> Plan & Execute
-> the trajectory plays in RViz AND on the real arm at the same time.

Usage
-----
  # sim-only (no arm): RViz animates from echoed commands
  ros2 launch new_arm_moveit_config arm_bringup.launch.py

  # with the real arm: start the micro-ROS agent too
  ros2 launch new_arm_moveit_config arm_bringup.launch.py \
       micro_ros_agent:=true serial_device:=/dev/ttyUSB0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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

    moveit_pkg = get_package_share_directory("new_arm_moveit_config")
    driver_pkg = get_package_share_directory("new_arm_driver")
    rviz_config = os.path.join(moveit_pkg, "config", "moveit.rviz")
    calibration = os.path.join(driver_pkg, "config", "calibration.yaml")

    # --- launch args ---
    micro_ros_agent_arg = DeclareLaunchArgument(
        "micro_ros_agent", default_value="false",
        description="Also start the micro-ROS agent for the ESP32 (real arm).")
    serial_device_arg = DeclareLaunchArgument(
        "serial_device", default_value="/dev/ttyUSB0",
        description="Serial device the ESP32 is on (when micro_ros_agent:=true).")

    # --- robot_state_publisher: URDF + link TFs (ros2_control tags ignored) ---
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

    # --- our host driver: /joint_states + FollowJointTrajectory action server.
    #     Replaces ros2_control entirely; it is the single source of joint state. ---
    arm_bridge_node = Node(
        package="new_arm_driver",
        executable="arm_bridge",
        name="arm_bridge",
        output="screen",
        parameters=[calibration],
    )

    # --- move_group: planning, IK, collision checking ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # --- RViz with the MotionPlanning panel (drag marker / set goals) ---
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

    # --- optional: micro-ROS agent for the ESP32 (only with the real arm) ---
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", LaunchConfiguration("serial_device")],
        condition=IfCondition(LaunchConfiguration("micro_ros_agent")),
    )

    return LaunchDescription([
        micro_ros_agent_arg,
        serial_device_arg,
        rsp_node,
        static_tf,
        arm_bridge_node,
        move_group_node,
        rviz_node,
        micro_ros_agent_node,
    ])
