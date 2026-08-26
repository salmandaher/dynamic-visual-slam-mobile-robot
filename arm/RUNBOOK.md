# new_arm — Runbook (all terminal commands)

Everything you need to build, simulate, drive from RViz, the web UI, the CLI, raw
topics, and the real ESP32 arm. Copy-paste per section.

- **Packages** live in `~/Documents/our_work`, symlinked into `~/ros2_ws/src`.
- **ROS**: Humble. **Always source** in every new terminal (see §0).
- **Two gotchas on this machine:**
  - `python3` is miniconda 3.13 and **cannot** import ROS. The `ros2` CLI is fine.
    For any custom rclpy script use `/usr/bin/python3`.
  - The web UI needs **Node 20** (`nvm use 20`); the default Node 25 won't run it.

---

## 0. Every terminal: source first

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

> If `ros2 launch ... new_arm_*` says "package not found", you forgot this (or the
> package isn't built yet — see §1).

---

## 1. Build (only after code changes, or first time)

```bash
# add packages to the workspace (one-time; safe to re-run)
ln -sf ~/Documents/our_work/new_arm_description    ~/ros2_ws/src/
ln -sf ~/Documents/our_work/new_arm_moveit_config  ~/ros2_ws/src/
ln -sf ~/Documents/our_work/new_arm_driver         ~/ros2_ws/src/
ln -sf ~/Documents/our_work/new_arm_ui             ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select new_arm_description new_arm_moveit_config new_arm_driver new_arm_ui
source ~/ros2_ws/install/setup.bash
```

> A harmless `Failed to extract project name from 'CMakeLists.txt'` line comes from
> a stray file in `~/ros2_ws/` — ignore it; the packages still build.

---

## 2. Run the stack — SIMULATION (no arm)

One command brings up move_group + RViz + the bridge. RViz animates from echoed
commands (no hardware needed).

```bash
ros2 launch new_arm_moveit_config arm_bringup.launch.py
```

In RViz → **MotionPlanning** panel: drag the marker on the arm tip **or** type a
goal, then **Plan & Execute**.

---

## 3. Run the stack — REAL ARM (ESP32 connected)

Same launch, plus it starts the micro-ROS agent for the ESP32. RViz now mirrors the
real servo feedback.

```bash
# adjust the serial device if needed (see §8 to find it)
ros2 launch new_arm_moveit_config arm_bringup.launch.py \
     micro_ros_agent:=true serial_device:=/dev/ttyUSB0
```

> The ESP32 must already be flashed with `espmax_passthrough` (see §7).

---

## 4. Web interface

The web server needs **Node 20**. First-time setup builds its dependencies.

### 4a. One-time setup

```bash
nvm use 20
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd $(ros2 pkg prefix new_arm_ui)/share/new_arm_ui/web
npm install          # builds rclnodejs (~2 min)
```

### 4b. Run it (in a terminal where the stack from §2 or §3 is already running)

```bash
nvm use 20
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch new_arm_ui web.launch.py            # default port 8080
# or a different port:
ros2 launch new_arm_ui web.launch.py port:=9000
```

Open **http://localhost:8080**. The page has: XYZ goal entry (+ click-to-set on the
schematic), named poses, joint-jog sliders (with a **live** toggle), **Stop**, a
live 2D schematic, and a status log.

> The launch runs `node` from your PATH, so that terminal must have `nvm use 20`
> active. To pin a specific binary instead: `... web.launch.py node_bin:=/path/to/node`.

---

## 5. CLI interface

Run in a terminal with the stack (§2/§3) already up.

```bash
# Cartesian goal, planned through MoveIt (metres, base/world frame)
ros2 run new_arm_ui arm_cli goto 0.135 0.0 0.156
ros2 run new_arm_ui arm_cli goto 0.12 0.05 0.18 --time 2.5    # slower move

# named pose (defined in the SRDF)
ros2 run new_arm_ui arm_cli named home
ros2 run new_arm_ui arm_cli named ready

# raw joint jog — RADIANS, [j1 j2 j3] = base, shoulder, elbow (no planning)
ros2 run new_arm_ui arm_cli jog 0.2 -0.3 0.4 --time 1.5

# print current joint positions once
ros2 run new_arm_ui arm_cli state
```

---

## 6. Send things directly from the terminal (raw `ros2` — no CLI/UI)

These talk to the **host bridge** (`arm_bridge`), which converts to servo pulses.
Joint values are **radians**; the 3 actuated joints are base, shoulder, elbow.

```bash
# --- Raw joint jog: [j1, j2, j3, time_ms] ---
ros2 topic pub --once /arm/joint_command std_msgs/msg/Float32MultiArray \
  "{data: [0.3, -0.2, 0.25, 1500.0]}"

# --- Read current state ---
ros2 topic echo --once /joint_states

# --- Plan+execute a Cartesian goal via MoveIt's controller action directly ---
# (joint-space goal to the controller; values in radians)
ros2 action send_goal /new_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [after_base_full_joint, link1_joint, link2_joint],
     points: [{positions: [0.5, 0.3, -0.4], time_from_start: {sec: 2}}]}}"

# --- Ask MoveIt to solve IK for an XYZ (no motion), just to see the joints ---
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: new_arm, ik_link_name: end_effector_link,
     pose_stamped: {header: {frame_id: world},
       pose: {position: {x: 0.12, y: 0.05, z: 0.18}, orientation: {w: 1.0}}},
     timeout: {sec: 2}}}"

# --- Inspect the graph ---
ros2 node list
ros2 topic list
ros2 action list
```

---

## 7. Flash the ESP32 (`espmax_passthrough` firmware)

The sketch is at `~/Documents/our_work/new_arm_firmware/espmax_passthrough/`. The
company servo driver (`LobotSerialServoControl.{h,cpp}`) is already copied in.

**Arduino IDE:**
1. Install the **ESP32 boards** package (a **2.0.x** core — newer 3.x may not link
   the micro-ROS lib) and the **`micro_ros_arduino`** library (Humble branch).
2. Open `espmax_passthrough/espmax_passthrough.ino`, select your ESP32 board + port,
   **Upload**.

**Or arduino-cli:**
```bash
arduino-cli core install esp32:esp32@2.0.2
# install micro_ros_arduino (humble) as a library, then:
cd ~/Documents/our_work/new_arm_firmware
arduino-cli compile --fqbn esp32:esp32:esp32 espmax_passthrough
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 espmax_passthrough
```

---

## 8. Talk to the real ESP32 arm

### 8a. Start the micro-ROS agent (bridges ESP32 ↔ ROS graph)

Either let the bringup launch do it (§3, `micro_ros_agent:=true`), or run it alone:

```bash
# find the device first
ls /dev/ttyUSB* /dev/ttyACM*          # usually /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

Once connected, the ESP32 node appears and these topics go live:

```bash
ros2 node list                         # expect the espmax_joint_node
ros2 topic echo /arm/servo_feedback    # [p1, p2, p3] pulses, ~20 Hz
```

### 8b. Send DIRECTLY to the ESP32 (bypasses the bridge — raw servo PULSES)

⚠️ These are **raw Lobot pulses 0–1000**, NOT radians, and skip the bridge's
calibration. Use small moves first. Format: `[p1, p2, p3, time_ms]`.

```bash
# move all three servos to mid-ish pulses over 1.5 s
ros2 topic pub --once /arm/servo_command std_msgs/msg/Float32MultiArray \
  "{data: [500.0, 350.0, 735.0, 1500.0]}"

# read back where the servos actually are (pulses)
ros2 topic echo --once /arm/servo_feedback
```

**Firmware safety clamps (applied on the ESP32 itself):** every pulse is clamped to
`[0, 1000]`, plus servo2 ≤ 700 and servo3 ≥ 470 to protect the linkage.

### 8c. Normal path (recommended): go through the bridge in radians

When the stack (§3) is running, prefer `/arm/joint_command` (radians, §6) or the
CLI/UI — the bridge applies calibration and publishes proper `/joint_states`.

---

## 9. Calibrate before trusting real motion

The rad↔pulse values in `new_arm_driver/config/calibration.yaml` are **derived
seeds, not measured**. Procedure:

```bash
# 1) command a known raw pulse, read it back
ros2 topic pub --once /arm/servo_command std_msgs/msg/Float32MultiArray "{data: [500,350,735,1500]}"
ros2 topic echo --once /arm/servo_feedback

# 2) for two known joint angles note the pulses, then per joint:
#       scale  = (p_b - p_a) / (theta_b - theta_a)
#       offset =  p_a - scale * theta_a
# 3) edit new_arm_driver/config/calibration.yaml, rebuild (§1), recheck /joint_states.
```

---

## 10. Shut down / cleanup

`Ctrl-C` in each launch terminal. If something lingers:

```bash
pkill -f move_group; pkill -f arm_bridge; pkill -f micro_ros_agent
pkill -f rviz2; pkill -f robot_state_publisher; pkill -f server.js
```

---

## Topic / interface reference

| Interface | Type | Direction | Payload |
|-----------|------|-----------|---------|
| `/arm/servo_command` | `std_msgs/Float32MultiArray` | host/you → ESP32 | `[p1,p2,p3,time_ms]` pulses 0–1000 |
| `/arm/servo_feedback` | `std_msgs/Float32MultiArray` | ESP32 → host | `[p1,p2,p3]` pulses, ~20 Hz |
| `/arm/joint_command` | `std_msgs/Float32MultiArray` | you → bridge | `[j1,j2,j3,(time_ms)]` **radians** |
| `/joint_states` | `sensor_msgs/JointState` | bridge → all | 3 actuated + passive wrist |
| `/new_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt/you → bridge | joint-space trajectory |
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | you → move_group | XYZ → joints |

Joints (actuated, in order): `after_base_full_joint` (base), `link1_joint`
(shoulder), `link2_joint` (elbow). Passive: `end_effector_joint` (coupled wrist,
no motor). Cartesian targets refer to the **wrist pivot** (`end_effector_link`).
