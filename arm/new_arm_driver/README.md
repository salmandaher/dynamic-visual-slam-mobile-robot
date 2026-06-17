# new_arm_driver

Lightweight host driver (layer 2) bridging ROS/MoveIt to the ESP32
`espmax_passthrough` firmware. The host owns the kinematics; the ESP32 only moves
servos. ros2_control is deferred (layer 2b) until this is validated on hardware.

## What it does

- **rad ↔ pulse** per-joint calibration (`config/calibration.yaml`).
- Publishes **`/joint_states`** from `/arm/servo_feedback`, computing the passive
  4th joint: `θ₄ = 2π − π/2 − θ_shoulder − θ_elbow`.
- **FollowJointTrajectory action server** (`/arm_controller/follow_joint_trajectory`)
  — streams MoveIt waypoints to `/arm/servo_command` with per-segment timing.
- **`/arm/joint_command`** (`Float32MultiArray` `[q1,q2,q3,(time_ms)]`, radians) for
  manual jogging / calibration before MoveIt is wired.

## Bring-up

```bash
# add to workspace (symlink so edits stay live -- see project memory)
ln -s ~/Documents/our_work/new_arm_driver ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select new_arm_driver && source install/setup.bash

# 1) micro-ROS agent (talks to the flashed ESP32)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
# 2) the bridge
ros2 launch new_arm_driver bridge.launch.py
# 3) confirm feedback -> joint states
ros2 topic echo /joint_states
```

## Calibration procedure (do this on hardware first)

The `scale`/`offset` in `calibration.yaml` are **derived seeds**, not measured.
To verify/fix each joint:

1. Command a raw pulse and read it back:
   ```bash
   ros2 topic pub --once /arm/servo_command std_msgs/Float32MultiArray "{data: [500,350,735,1500]}"
   ros2 topic echo --once /arm/servo_feedback
   ```
2. For two known joint angles θ_a, θ_b note the pulses p_a, p_b, then
   `scale = (p_b − p_a)/(θ_b − θ_a)`, `offset = p_a − scale·θ_a`. Update the YAML.
3. Re-check `/joint_states` against the real pose in RViz.

> Start slow and within `pulse_min/pulse_max` — these mirror the firmware's
> mechanical guards (servo2 ≤ 700, servo3 ≥ 470).

## Drive it from MoveIt + RViz

`new_arm_moveit_config/launch/arm_bringup.launch.py` runs MoveIt + RViz wired to
this bridge (instead of mock hardware). Drag the interactive marker OR type a goal
in the MotionPlanning panel, hit **Plan & Execute**, and the motion plays in RViz
**and** on the real arm at once (RViz mirrors `/joint_states` from this node).

```bash
# sim only (no arm): RViz animates from echoed commands
ros2 launch new_arm_moveit_config arm_bringup.launch.py
# with the real arm (ESP32 flashed with espmax_passthrough):
ros2 launch new_arm_moveit_config arm_bringup.launch.py \
     micro_ros_agent:=true serial_device:=/dev/ttyUSB0
```

## 3-DOF planning (done)

MoveIt plans the **3 actuated joints** only; `end_effector_joint` is declared
`<passive_joint>` in the SRDF. The planning tip stays at `end_effector_link`
because that link's frame *origin* is the wrist pivot, whose **position is
independent of the passive joint angle** (rotating the joint only spins the frame
in place). With position-only IK, the planned wrist position is exact and the
solver uses only the 3 actuated joints. Verified headless: `/compute_ik` on group
`new_arm` returns SUCCESS with the 3 actuated joints solved and the wrist held
passive.

The bridge reports the coupled wrist value in `/joint_states`, so RViz shows the
tool correctly leveled.

> Targets refer to the **wrist pivot** (`end_effector_link` origin). To aim a
> gripper tip beyond the pivot, ask me to add a fixed tool frame.

## Not done yet

- **Tool-tip frame** (optional): fixed tool offset so targets aim the gripper tip.
- **Layer 2b**: ros2_control `SystemInterface` once the bridge is proven.
