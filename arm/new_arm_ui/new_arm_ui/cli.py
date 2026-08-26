#!/usr/bin/env python3
"""arm_cli -- command-line interface for the new_arm.

Two command modes, matching the web UI:

  Cartesian (planned through MoveIt):
    ros2 run new_arm_ui arm_cli goto X Y Z [--time T] [--frame world]
    ros2 run new_arm_ui arm_cli named home|ready

  Raw joint jog (straight to the driver, no planning):
    ros2 run new_arm_ui arm_cli jog J1 J2 J3 [--time T]

  Read state:
    ros2 run new_arm_ui arm_cli state

`goto` calls MoveIt's /compute_ik (position-only, 3-DOF group) then executes the
solution via the FollowJointTrajectory action served by new_arm_driver/arm_bridge,
so it moves in RViz/web AND on the real arm. `jog` publishes radians to
/arm/joint_command (arm_bridge converts to servo pulses). Needs move_group running
for `goto`/`named` (e.g. `ros2 launch new_arm_moveit_config arm_bringup.launch.py`).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK


ACTUATED = ['after_base_full_joint', 'link1_joint', 'link2_joint']
NAMED = {'home': [0.0, 0.0, 0.0], 'ready': [0.0, 0.4, -0.6]}
GROUP = 'new_arm'
IK_LINK = 'end_effector_link'
ACTION = '/new_arm_controller/follow_joint_trajectory'


def _duration(seconds):
    return DurationMsg(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


class ArmCli(Node):
    def __init__(self):
        super().__init__('arm_cli')
        self.jog_pub = self.create_publisher(Float32MultiArray, '/arm/joint_command', 10)
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_cli = ActionClient(self, FollowJointTrajectory, ACTION)

    # -------- Cartesian via MoveIt --------
    def goto(self, x, y, z, frame, move_time):
        joints = self._solve_ik(x, y, z, frame)
        if joints is None:
            return 1
        print(f'IK solution: {dict(zip(ACTUATED, [round(j, 4) for j in joints]))}')
        return self._execute(joints, move_time, f'XYZ ({x}, {y}, {z})')

    def named(self, name, move_time):
        if name not in NAMED:
            print(f"Unknown named pose '{name}'. Known: {list(NAMED)}", file=sys.stderr)
            return 2
        return self._execute(NAMED[name], move_time, f"named '{name}'")

    def _solve_ik(self, x, y, z, frame):
        if not self.ik_cli.wait_for_service(timeout_sec=5.0):
            print('ERROR: /compute_ik unavailable -- is move_group running?', file=sys.stderr)
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = IK_LINK
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = _duration(2.0)
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = ps

        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None:
            print('ERROR: IK call timed out.', file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f'ERROR: no IK solution (code {res.error_code.val}); '
                  f'target likely out of reach.', file=sys.stderr)
            return None
        pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        try:
            return [pos[j] for j in ACTUATED]
        except KeyError as e:
            print(f'ERROR: IK solution missing joint {e}.', file=sys.stderr)
            return None

    def _execute(self, joints, move_time, label):
        if not self.traj_cli.wait_for_server(timeout_sec=5.0):
            print('ERROR: controller action unavailable -- is arm_bridge running?',
                  file=sys.stderr)
            return 3
        traj = JointTrajectory()
        traj.joint_names = list(ACTUATED)
        pt = JointTrajectoryPoint()
        pt.positions = [float(q) for q in joints]
        pt.time_from_start = _duration(move_time)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f'Executing {label} over {move_time:.1f}s...')
        send_fut = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut, timeout_sec=5.0)
        handle = send_fut.result()
        if handle is None or not handle.accepted:
            print('ERROR: trajectory goal rejected.', file=sys.stderr)
            return 4
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=move_time + 10.0)
        result = res_fut.result()
        if result is None:
            print('ERROR: no result from controller.', file=sys.stderr)
            return 5
        code = result.result.error_code
        if code == 0:
            print(f'Done: reached {label}.')
            return 0
        print(f'Execution finished with error_code {code}.', file=sys.stderr)
        return 6

    # -------- Raw joint jog --------
    def jog(self, joints, move_time):
        msg = Float32MultiArray()
        msg.data = [float(j) for j in joints] + [float(move_time * 1000.0)]
        self.jog_pub.publish(msg)
        # give the message time to leave before the process exits
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        print(f'Jog sent: {dict(zip(ACTUATED, [round(j, 4) for j in joints]))} '
              f'over {move_time:.1f}s.')
        return 0

    # -------- State --------
    def state(self):
        got = {}

        def cb(msg):
            got['msg'] = msg
        self.create_subscription(JointState, '/joint_states', cb, 10)
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.05)
            if 'msg' in got:
                break
        if 'msg' not in got:
            print('ERROR: no /joint_states received.', file=sys.stderr)
            return 7
        m = got['msg']
        for n, p in zip(m.name, m.position):
            print(f'  {n:24s} {p:+.4f} rad ({p * 57.2958:+.1f} deg)')
        return 0


def build_parser():
    p = argparse.ArgumentParser(prog='arm_cli', description='new_arm command-line interface')
    sub = p.add_subparsers(dest='cmd', required=True)

    g = sub.add_parser('goto', help='Cartesian goal via MoveIt')
    g.add_argument('x', type=float)
    g.add_argument('y', type=float)
    g.add_argument('z', type=float)
    g.add_argument('--frame', default='world')
    g.add_argument('--time', type=float, default=2.0, help='move duration (s)')

    n = sub.add_parser('named', help='named pose via MoveIt')
    n.add_argument('name', choices=list(NAMED))
    n.add_argument('--time', type=float, default=2.0)

    j = sub.add_parser('jog', help='raw joint jog to /arm/joint_command (radians)')
    j.add_argument('j1', type=float)
    j.add_argument('j2', type=float)
    j.add_argument('j3', type=float)
    j.add_argument('--time', type=float, default=1.0)

    sub.add_parser('state', help='print current /joint_states once')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = ArmCli()
    try:
        if args.cmd == 'goto':
            rc = node.goto(args.x, args.y, args.z, args.frame, args.time)
        elif args.cmd == 'named':
            rc = node.named(args.name, args.time)
        elif args.cmd == 'jog':
            rc = node.jog([args.j1, args.j2, args.j3], args.time)
        elif args.cmd == 'state':
            rc = node.state()
        else:
            rc = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
