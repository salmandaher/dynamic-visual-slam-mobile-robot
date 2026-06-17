#!/usr/bin/env python3
"""new_arm Isaac Sim digital twin — a drop-in replacement for the real ESP32 arm.

The real arm is driven like this:

    MoveIt / CLI / web  ->  arm_bridge  --/arm/servo_command (pulses)-->  ESP32 -> servos
                                        <--/arm/servo_feedback (pulses)-- ESP32 (reads servos)

This script makes **Isaac Sim play the ESP32 + servos**: it subscribes to
`/arm/servo_command`, converts the Lobot pulses to radians with the SAME calibration
as the firmware, drives the simulated articulation (3 actuated joints + the
mechanically-coupled passive wrist), reads the joints back, converts to pulses, and
publishes `/arm/servo_feedback`. The ENTIRE existing stack then drives the twin with
zero changes — run the bringup WITHOUT the micro-ROS agent and run this instead:

    # terminal 1 (no real arm): move_group + arm_bridge + RViz
    ros2 launch new_arm_moveit_config arm_bringup.launch.py
    # terminal 2: the Isaac twin (this script) -- replaces the ESP32
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    source setup_ros_env.sh
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/digital_twin.py

--publish-joint-states also publishes sensor_msgs/JointState on /joint_states
directly (bypassing arm_bridge), so RViz/MoveIt can point straight at Isaac.

ROS is done with rclpy IN this process (spun once per sim step) rather than the
OmniGraph ROS2 bridge — keeping the exact Float32MultiArray contract the rest of the
stack uses, with no new message types.

Verified API (Isaac Sim 5.1.0): SimulationApp; World (isaacsim.core.api);
add_reference_to_stage (isaacsim.core.utils.stage); SingleArticulation
(isaacsim.core.prims); ArticulationAction (isaacsim.core.utils.types).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac import kinematics as kin  # noqa: E402

parser = argparse.ArgumentParser(description="new_arm Isaac digital twin (ESP32 replacement)")
parser.add_argument("--headless", action="store_true", help="run without the Isaac GUI")
parser.add_argument("--publish-joint-states", action="store_true",
                    help="also publish sensor_msgs/JointState on /joint_states directly")
parser.add_argument("--usd", default=config.USD_PATH, help="USD asset (default: config.USD_PATH)")
args, _ = parser.parse_known_args()

# ---- SimulationApp MUST come first ----
from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402


class TwinBridge(Node):
    """rclpy side of the twin: pulse command in, pulse feedback out."""

    def __init__(self, publish_joint_states):
        super().__init__("new_arm_isaac_twin")
        self.n = len(config.ACTUATED_JOINTS)
        self._cmd_pulses = None
        self._publish_js = publish_joint_states

        self.create_subscription(Float32MultiArray, "/arm/servo_command", self._on_command, 10)
        self.fb_pub = self.create_publisher(Float32MultiArray, "/arm/servo_feedback", 10)
        if self._publish_js:
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.get_logger().info(
            "new_arm Isaac twin up: sub /arm/servo_command, pub /arm/servo_feedback"
            + (" + /joint_states" if self._publish_js else ""))

    def _on_command(self, msg: Float32MultiArray):
        if len(msg.data) >= self.n:
            self._cmd_pulses = [float(msg.data[i]) for i in range(self.n)]

    def desired_thetas(self):
        if self._cmd_pulses is None:
            return None
        return [config.pulse_to_theta(i, self._cmd_pulses[i]) for i in range(self.n)]

    def publish_feedback(self, thetas):
        pulses = [config.theta_to_pulse(i, thetas[i]) for i in range(self.n)]
        self.fb_pub.publish(Float32MultiArray(data=[float(p) for p in pulses]))

    def publish_joint_states(self, thetas, passive):
        if not self._publish_js:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(config.ACTUATED_JOINTS) + [config.PASSIVE_JOINT]
        js.position = [float(t) for t in thetas] + [float(passive)]
        self.js_pub.publish(js)


def main():
    if not os.path.exists(args.usd):
        print(f"[twin] ERROR: USD not found: {args.usd}\n"
              f"       Run scripts/import_urdf.py first.", file=sys.stderr)
        return

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=args.usd, prim_path=config.ROBOT_PRIM)
    robot = SingleArticulation(prim_path=config.ROBOT_PRIM, name="new_arm")
    world.scene.add(robot)
    world.reset()

    dof_names = list(robot.dof_names)
    print(f"[twin] articulation dof_names = {dof_names}")
    try:
        act_idx = [dof_names.index(j) for j in config.ACTUATED_JOINTS]
        pas_idx = dof_names.index(config.PASSIVE_JOINT)
    except ValueError as e:
        print(f"[twin] ERROR: joint name mismatch ({e}).\n"
              f"       Edit config.ACTUATED_JOINTS/PASSIVE_JOINT to match the USD.",
              file=sys.stderr)
        return

    rclpy.init()
    bridge = TwinBridge(publish_joint_states=args.publish_joint_states)

    full_target = np.array(robot.get_joint_positions(), dtype=float)
    print("[twin] running. Drive it via the normal stack (arm_bridge -> /arm/servo_command).")

    while simulation_app.is_running():
        rclpy.spin_once(bridge, timeout_sec=0.0)

        thetas = bridge.desired_thetas()
        if thetas is not None:
            passive = kin.passive_wrist(thetas[1], thetas[2])
            for k, idx in enumerate(act_idx):
                full_target[idx] = thetas[k]
            full_target[pas_idx] = passive
            robot.apply_action(ArticulationAction(joint_positions=full_target))

        world.step(render=not args.headless)

        meas = np.array(robot.get_joint_positions(), dtype=float)
        meas_act = [float(meas[idx]) for idx in act_idx]
        bridge.publish_feedback(meas_act)
        bridge.publish_joint_states(meas_act, float(meas[pas_idx]))

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
