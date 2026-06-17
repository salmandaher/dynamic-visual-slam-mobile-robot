import os
import threading
import tkinter as tk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from new_arm_ik.kinematics import parse_urdf_chain, IKSolver


class IKGUINode(Node):
    def __init__(self):
        super().__init__('ik_gui_node')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('description_package', 'new_arm_description')
        self.declare_parameter('urdf_filename', 'new_arm.urdf')
        self.declare_parameter('root_link', 'arm_base_link')
        self.declare_parameter('tip_link', 'end_effector_link')

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if not urdf_path:
            desc_share = get_package_share_directory(
                self.get_parameter('description_package').get_parameter_value().string_value)
            urdf_path = os.path.join(
                desc_share, 'urdf',
                self.get_parameter('urdf_filename').get_parameter_value().string_value)

        root_link = self.get_parameter('root_link').get_parameter_value().string_value
        tip_link = self.get_parameter('tip_link').get_parameter_value().string_value

        self.get_logger().info(f'Loading URDF from {urdf_path}')
        self.chain = parse_urdf_chain(urdf_path, root_link, tip_link)
        self.solver = IKSolver(self.chain)
        self.joint_names = [j['name'] for j in self.chain.joints]
        self.current_q = np.zeros(len(self.joint_names))

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self._publish_state)

        self.get_logger().info(f'IK chain ({len(self.joint_names)} joints): {self.joint_names}')

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_q.tolist()
        self.pub.publish(msg)

    def current_ee_position(self):
        return self.chain.fk(self.current_q)[:3, 3]

    def solve_to(self, target_xyz):
        q, err = self.solver.solve(target_xyz, q_init=self.current_q)
        self.current_q = q
        return q, err

    def set_home(self):
        self.current_q = np.zeros(len(self.joint_names))


def run_gui(node):
    root = tk.Tk()
    root.title('New_arm IK Interface')
    root.geometry('420x320')

    frm = tk.Frame(root, padx=10, pady=10)
    frm.pack(fill='both', expand=True)

    tk.Label(frm, text='Target end-effector position (meters)',
             font=('TkDefaultFont', 10, 'bold')).grid(row=0, column=0, columnspan=3, sticky='w')

    entries = {}
    for i, axis in enumerate(['X', 'Y', 'Z']):
        tk.Label(frm, text=f'{axis}:').grid(row=1 + i, column=0, sticky='e', pady=2)
        e = tk.Entry(frm, width=12)
        default = '0.15' if axis == 'X' else ('0.10' if axis == 'Z' else '0.0')
        e.insert(0, default)
        e.grid(row=1 + i, column=1, sticky='w', pady=2)
        entries[axis] = e

    status_var = tk.StringVar(value='Ready.')
    joints_var = tk.StringVar(value='Joints (deg): -')
    ee_var = tk.StringVar(value='Current EE: -')

    def refresh_ee():
        p = node.current_ee_position()
        ee_var.set(f'Current EE (m): x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}')
        root.after(200, refresh_ee)

    def do_solve():
        try:
            tgt = [float(entries['X'].get()),
                   float(entries['Y'].get()),
                   float(entries['Z'].get())]
        except ValueError:
            status_var.set('Bad numeric input.')
            return
        q, err = node.solve_to(tgt)
        status_var.set(f'Solved. Position residual = {err * 1000:.2f} mm')
        joints_var.set('Joints (deg): ' +
                       ', '.join(f'{np.degrees(a):+7.2f}' for a in q))

    def do_home():
        node.set_home()
        status_var.set('Sent home pose (all zeros).')
        joints_var.set('Joints (deg): ' + ', '.join(['+0.00'] * len(node.joint_names)))

    btn_frame = tk.Frame(frm)
    btn_frame.grid(row=5, column=0, columnspan=3, pady=8, sticky='w')
    tk.Button(btn_frame, text='Solve IK', width=10, command=do_solve).pack(side='left', padx=4)
    tk.Button(btn_frame, text='Home',     width=10, command=do_home ).pack(side='left', padx=4)

    tk.Label(frm, textvariable=status_var, fg='blue').grid(
        row=6, column=0, columnspan=3, sticky='w', pady=(8, 0))
    tk.Label(frm, textvariable=ee_var, font=('TkFixedFont', 9)).grid(
        row=7, column=0, columnspan=3, sticky='w')
    tk.Label(frm, textvariable=joints_var, font=('TkFixedFont', 9)).grid(
        row=8, column=0, columnspan=3, sticky='w')

    tk.Label(frm, text='Tip: this arm is ~0.25 m reach. Targets outside that\n'
                       'workspace will return a non-zero residual.',
             fg='gray', justify='left').grid(row=9, column=0, columnspan=3, sticky='w', pady=(10, 0))

    refresh_ee()

    def on_close():
        root.quit()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


def main():
    rclpy.init()
    node = IKGUINode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_gui(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
