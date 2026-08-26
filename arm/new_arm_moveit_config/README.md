# new_arm_moveit_config

MoveIt 2 motion-planning configuration for the **new_arm** (4-DOF arm), ROS 2 Humble.

This package was hand-authored (not via the Setup Assistant GUI) so it is minimal,
readable, and reproducible. It wraps the existing `new_arm_description` URDF and
adds everything MoveIt needs to plan and visualize trajectories in RViz with mock
hardware.

## Prerequisites

MoveIt and ros2_control are not installed by default. Install once:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster
```

## Build

The package lives in `~/Documents/our_work/` and is symlinked into the workspace
at `~/ros2_ws/src/new_arm_moveit_config`. Build it together with the description
(which provides the meshes and base URDF):

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select new_arm_description new_arm_moveit_config
source install/setup.bash
```

> `new_arm_description` **must** be built (installed) so MoveIt can resolve
> `$(find new_arm_description)` in the xacro. Building from source alone is not
> enough.

## Run

```bash
ros2 launch new_arm_moveit_config demo.launch.py
```

This starts: `robot_state_publisher`, `ros2_control_node` (mock hardware),
`joint_state_broadcaster`, `new_arm_controller`, `move_group`, the `world` static
TF, and RViz with the MotionPlanning panel.

## Test planning in RViz

1. In the **MotionPlanning** display, open the **Planning** tab.
2. Under *Query*, set **Goal State** to `ready` (or drag the orange interactive
   marker on the end-effector to a new pose).
3. Click **Plan** — a preview trajectory animates.
4. Click **Execute** (or **Plan & Execute**) — the mock robot follows it and
   `/joint_states` updates.
5. Add an obstacle: **Scene Objects** tab → import/append a box → **Publish** →
   plan again to see MoveIt avoid it.

## What each file does

| File | Purpose |
|---|---|
| `config/new_arm.urdf.xacro` | Wraps the description URDF and adds the `ros2_control` block (mock hardware). Single source of geometry. |
| `config/new_arm.srdf` | Semantics: the `new_arm` planning group (chain `arm_base_link`→`end_effector_link`), named poses (`home`, `ready`), the fixed `world` virtual joint, and collision exclusions. |
| `config/kinematics.yaml` | IK solver: KDL with `position_only_ik: true` (the arm is 4-DOF, so orientation is left free). |
| `config/joint_limits.yaml` | Velocity/acceleration limits for trajectory timing. |
| `config/moveit_controllers.yaml` | MoveIt → ros2_control handoff: a `FollowJointTrajectory` controller named `new_arm_controller`. |
| `config/ros2_controllers.yaml` | The actual controllers: `joint_state_broadcaster` + `new_arm_controller` (JointTrajectoryController). |
| `config/ompl_planning.yaml` | OMPL planner presets (RRTConnect default). |
| `config/pilz_cartesian_limits.yaml` | Cartesian limits for the optional Pilz planner. |
| `config/initial_positions.yaml` | Mock hardware start pose (home). |
| `config/sensors_3d.yaml` | 3D sensor / Octomap config (empty — no depth camera). |
| `config/moveit.rviz` | RViz layout with the MotionPlanning panel pre-configured. |
| `launch/demo.launch.py` | Brings up the whole stack via `MoveItConfigsBuilder`. |

## Customizing later

- **Add a gripper:** define a second `<group>` in the SRDF (e.g. `hand`), add an
  `<end_effector>`, and create a controller for its joint(s).
- **Use real hardware:** in `new_arm.urdf.xacro`, set `use_mock_hardware:=false`
  and replace the hardware `<plugin>` with your interface; keep the same
  controllers.
- **Swap IK to trac_ik:** install `ros-humble-trac-ik-kinematics-plugin` and set
  `kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` in
  `kinematics.yaml`.
- **Wrap your scipy IK solver:** requires writing a C++ `KinematicsBase` plugin —
  a separate, larger task.
