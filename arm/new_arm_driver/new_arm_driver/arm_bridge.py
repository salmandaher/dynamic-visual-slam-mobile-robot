#!/usr/bin/env python3
"""Host driver bridging ROS/MoveIt to the ESPMax ESP32 firmware.

The host owns the kinematics; the ESP32 (espmax_passthrough) only moves servos to
commanded pulses. This node is the SINGLE source of /joint_states, so RViz and the
real arm never disagree -- RViz simply mirrors whatever this node reports.

It:
  * converts joint radians <-> Lobot pulses via a per-joint affine calibration,
  * publishes /joint_states, computing the passive 4th joint (mechanically-coupled
    wrist) from the actuated shoulder + elbow,
  * executes MoveIt trajectories via a FollowJointTrajectory action server, by
    streaming each waypoint to /arm/servo_command with per-segment timing,
  * accepts a simple /arm/joint_command (Float32MultiArray, radians) for manual
    jogging / calibration.

Hardware vs sim is automatic:
  * if /arm/servo_feedback is arriving (ESP32 connected via micro-ROS agent),
    /joint_states reflects REAL servo positions;
  * if no feedback is seen within `feedback_timeout`, the node ECHOES the last
    commanded positions so RViz still animates with no arm attached.

Topics:
  sub  /arm/servo_feedback  std_msgs/Float32MultiArray  [p1, p2, p3]   (pulses)
  pub  /arm/servo_command   std_msgs/Float32MultiArray  [p1, p2, p3, time_ms]
  sub  /arm/joint_command   std_msgs/Float32MultiArray  [q1, q2, q3, (time_ms)]
  pub  /joint_states        sensor_msgs/JointState
  action server  <follow_joint_trajectory_action>  control_msgs/FollowJointTrajectory
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class ArmBridge(Node):
    def __init__(self):
        super().__init__('arm_bridge')
        p = self.declare_parameter

        self.actuated = list(p('actuated_joints',
                               ['after_base_full_joint', 'link1_joint', 'link2_joint']).value)
        self.scale = list(p('scale', [238.7324, -238.7324, 238.7324]).value)
        self.offset = list(p('offset', [625.0, 875.0, 500.0]).value)
        self.pulse_min = list(p('pulse_min', [0.0, 0.0, 470.0]).value)
        self.pulse_max = list(p('pulse_max', [1000.0, 700.0, 1000.0]).value)

        self.passive_joint = p('passive_joint', 'end_effector_joint').value
        self.passive_offset = float(p('passive_offset', 4.712389).value)
        self.passive_c_sh = float(p('passive_coeff_shoulder', -1.0).value)
        self.passive_c_el = float(p('passive_coeff_elbow', -1.0).value)
        self.passive_min = float(p('passive_min', -1.571).value)
        self.passive_max = float(p('passive_max', 1.571).value)

        action_name = p('follow_joint_trajectory_action',
                        '/new_arm_controller/follow_joint_trajectory').value
        self.default_move_ms = float(p('default_move_time_ms', 1000.0).value)
        self.min_seg_ms = float(p('min_segment_time_ms', 200.0).value)
        fb_hz = float(p('feedback_rate_hz', 20.0).value)
        self.feedback_timeout = float(p('feedback_timeout', 0.5).value)
        self.initial_positions = list(p('initial_positions', [0.0, 0.0, 0.0]).value)

        n = len(self.actuated)
        if not (len(self.scale) == len(self.offset) == len(self.pulse_min)
                == len(self.pulse_max) == n):
            raise ValueError('calibration arrays must match actuated_joints length')
        if len(self.initial_positions) != n:
            self.initial_positions = [0.0] * n

        # shoulder = 2nd actuated (link1_joint), elbow = 3rd (link2_joint)
        self.sh_idx = 1 if n > 1 else 0
        self.el_idx = 2 if n > 2 else 0

        # last REAL feedback (pulses) and when it arrived; None until first message
        self._fb_pulses = None
        self._fb_time = None
        # last COMMANDED actuated angles (rad); used to echo when no feedback
        self._commanded = list(self.initial_positions)

        cb = ReentrantCallbackGroup()
        self.servo_cmd_pub = self.create_publisher(Float32MultiArray, '/arm/servo_command', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(Float32MultiArray, '/arm/servo_feedback',
                                 self._on_feedback, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/arm/joint_command',
                                 self._on_joint_command, 10, callback_group=cb)
        self.create_timer(1.0 / fb_hz, self._publish_joint_states, callback_group=cb)

        self._action = ActionServer(
            self, FollowJointTrajectory, action_name,
            execute_callback=self._execute_trajectory,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _c: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f'arm_bridge up. actuated={self.actuated} passive={self.passive_joint} '
            f'action={action_name}. Publishing /joint_states (echo until feedback).')

    # ---------------- calibration ----------------
    def theta_to_pulse(self, i, theta):
        pulse = self.scale[i] * theta + self.offset[i]
        return max(self.pulse_min[i], min(self.pulse_max[i], pulse))

    def pulse_to_theta(self, i, pulse):
        if self.scale[i] == 0.0:
            return 0.0
        return (pulse - self.offset[i]) / self.scale[i]

    def passive_value(self, thetas):
        sh = thetas[self.sh_idx]
        el = thetas[self.el_idx]
        v = self.passive_offset + self.passive_c_sh * sh + self.passive_c_el * el
        # The coupling formula lives in a mechanical frame whose zero differs from
        # the URDF joint zero, so the raw value can be e.g. 270deg. Wrap it into
        # the principal range (-pi, pi] -- that is the same physical orientation,
        # expressed as a valid URDF joint angle (270deg -> -90deg).
        wrapped = math.atan2(math.sin(v), math.cos(v))
        # Safety clamp to the joint's URDF limits (won't fire for wrapped values
        # inside +/-pi, but guards if someone widens passive_offset oddly).
        clamped = max(self.passive_min, min(self.passive_max, wrapped))
        if abs(clamped - wrapped) > 1e-3:
            self.get_logger().warn(
                f'passive joint {wrapped:.3f} clamped to '
                f'[{self.passive_min:.3f}, {self.passive_max:.3f}] -- check formula',
                throttle_duration_sec=5.0)
        return clamped

    # ---------------- feedback -> /joint_states ----------------
    def _on_feedback(self, msg: Float32MultiArray):
        if len(msg.data) >= len(self.actuated):
            # negative pulse = servo read error; keep last good value for that joint
            pulses = list(msg.data[:len(self.actuated)])
            good = self._fb_pulses if self._fb_pulses else [None] * len(self.actuated)
            self._fb_pulses = [pulses[i] if pulses[i] >= 0 else good[i]
                               if good[i] is not None else 0.0
                               for i in range(len(self.actuated))]
            self._fb_time = self.get_clock().now()

    def _feedback_fresh(self):
        if self._fb_time is None or self._fb_pulses is None:
            return False
        age = (self.get_clock().now() - self._fb_time).nanoseconds * 1e-9
        return age <= self.feedback_timeout

    def _publish_joint_states(self):
        if self._feedback_fresh():
            thetas = [self.pulse_to_theta(i, self._fb_pulses[i])
                      for i in range(len(self.actuated))]
        else:
            thetas = list(self._commanded)   # echo: lets RViz animate with no arm
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.actuated) + [self.passive_joint]
        js.position = thetas + [self.passive_value(thetas)]
        self.joint_state_pub.publish(js)

    # ---------------- manual jog ----------------
    def _on_joint_command(self, msg: Float32MultiArray):
        n = len(self.actuated)
        if len(msg.data) < n:
            self.get_logger().warn(f'/arm/joint_command needs >= {n} values; ignoring.')
            return
        move_ms = self.default_move_ms
        if len(msg.data) > n and msg.data[n] > 0.0:
            move_ms = max(self.min_seg_ms, float(msg.data[n]))
        self._send_thetas([float(msg.data[i]) for i in range(n)], move_ms)

    def _send_thetas(self, thetas, move_ms):
        """Command actuated joints (rad) -> pulses, and record for echo state."""
        pulses = [self.theta_to_pulse(i, thetas[i]) for i in range(len(self.actuated))]
        out = Float32MultiArray()
        out.data = [float(x) for x in pulses] + [float(move_ms)]
        self.servo_cmd_pub.publish(out)
        self._commanded = list(thetas)   # so /joint_states reflects the goal in sim

    # ---------------- trajectory execution ----------------
    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        # map incoming joint order -> our actuated order (ignore non-actuated, e.g.
        # the passive joint that the 4-DOF MoveIt group still includes)
        name_to_idx = {nm: i for i, nm in enumerate(traj.joint_names)}
        col = [name_to_idx.get(j) for j in self.actuated]
        missing = [self.actuated[k] for k, c in enumerate(col) if c is None]
        if missing:
            self.get_logger().error(f'trajectory missing actuated joints {missing}; aborting.')
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_JOINTS)

        result = FollowJointTrajectory.Result()
        prev_ms = 0.0
        for pt in traj.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('trajectory canceled.')
                return result
            t_ms = pt.time_from_start.sec * 1000.0 + pt.time_from_start.nanosec / 1e6
            seg_ms = max(self.min_seg_ms, t_ms - prev_ms)
            prev_ms = t_ms
            thetas = [float(pt.positions[col[i]]) for i in range(len(self.actuated))]
            self._send_thetas(thetas, seg_ms)
            time.sleep(seg_ms / 1000.0)   # let the servos reach the waypoint

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('trajectory complete.')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
