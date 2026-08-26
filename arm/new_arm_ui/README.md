# new_arm_ui

Two interfaces for the new_arm, both with **both command modes**:

- **Cartesian goal** — planned through MoveIt (`/compute_ik` → `FollowJointTrajectory`).
- **Raw joint jog** — straight to the driver (`/arm/joint_command`), no planning.

Either way, motion runs through `new_arm_driver/arm_bridge`, so it plays in
RViz/web **and** on the real arm (when the ESP32/micro-ROS agent is connected).

## 1. CLI

```bash
# Cartesian (needs move_group running)
ros2 run new_arm_ui arm_cli goto 0.135 0.0 0.156 --time 2.5
ros2 run new_arm_ui arm_cli named ready

# raw joint jog (radians) -- works without move_group
ros2 run new_arm_ui arm_cli jog 0.2 -0.3 0.4 --time 1.5

# read current joints
ros2 run new_arm_ui arm_cli state
```

## 2. Web UI (rclnodejs)

> Note: you asked for "rosnodejs", but that library is **ROS 1 only**. The ROS 2
> Node.js client is **rclnodejs**, which this uses. The Node process is itself a
> ROS 2 node — no rosbridge required.

The browser app gives you: XYZ goal entry (+ click-to-set on the live schematic),
named poses, three joint-jog sliders (with optional **live** mode), a **Stop**
(cancels the running goal), a live 2D top/side schematic driven by `/joint_states`,
and a status log.

### One-time setup

Install Node deps **with a ROS 2 env sourced** (rclnodejs builds message bindings
from it):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd $(ros2 pkg prefix new_arm_ui)/share/new_arm_ui/web
npm install            # pulls rclnodejs, express, ws
```

### Run

```bash
# terminal 1 - planning + execution + RViz
ros2 launch new_arm_moveit_config arm_bringup.launch.py
#   ...add  micro_ros_agent:=true serial_device:=/dev/ttyUSB0  for the real arm

# terminal 2 - the web UI
ros2 launch new_arm_ui web.launch.py        # or: PORT=9090 ros2 launch ... port:=9090
```

Open **http://localhost:8080**.

## How it fits together

```
  Browser ──ws──> server.js (rclnodejs ROS2 node) ──┬─ /compute_ik ─────> move_group
                                                     ├─ FollowJointTrajectory ─> arm_bridge ─> arm
                                                     ├─ /arm/joint_command ───> arm_bridge ─> arm
                                                     └─ /joint_states  <─────── arm_bridge
  CLI (arm_cli, rclpy) ── same /compute_ik + action + /arm/joint_command ───────┘
```

## Notes / caveats

- **Calibrate first.** Raw jogs and executed goals move the real servos via the
  seeded rad↔pulse calibration in `new_arm_driver/config/calibration.yaml`. Verify
  it on hardware before trusting motion (see `new_arm_driver/README.md`).
- The 2D schematic uses the URDF link geometry (`web/fk.js`); it mirrors
  `/joint_states`, so with no arm attached it shows the *commanded* pose (echoed by
  the bridge).
- Targets are the **wrist pivot** (`end_effector_link`), consistent with the 3-DOF
  planning group.
