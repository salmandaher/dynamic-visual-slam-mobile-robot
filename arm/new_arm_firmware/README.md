# new_arm_firmware

Our firmware for driving the real ESPMax arm from ROS 2 via the **direct servo
driver** path. The ESP32 becomes a dumb actuator bridge; the ROS host owns all
kinematics and calibration. The company's original `kinematics_move.ino` (onboard
IK + `/arm_position`) is left untouched as reference.

## espmax_passthrough

ESP32 micro-ROS sketch. Contract with the host:

| dir | topic | type | data |
|-----|-------|------|------|
| sub | `/arm/servo_command`  | `std_msgs/Float32MultiArray` | `[p1, p2, p3, time_ms]` — Lobot pulses 0–1000 for servos 1/2/3, move duration ms |
| pub | `/arm/servo_feedback` | `std_msgs/Float32MultiArray` | `[p1, p2, p3]` — pulses read back, ~20 Hz |

Safety: pulses clamped to `[0,1000]` and to the company's mechanical guards
(servo2 ≤ 700, servo3 ≥ 470). No IK on the device.

### Build / flash

1. Copy the company servo driver next to the sketch (it's the HAL, not the IK).
   **Already done** — `LobotSerialServoControl.{h,cpp}` are now in the sketch
   folder. To redo:
   ```bash
   cp kinematics_move/LobotSerialServoControl.{h,cpp} new_arm_firmware/espmax_passthrough/
   ```
2. Arduino IDE: install `micro_ros_arduino` (Humble branch), select the ESP32
   board, open `espmax_passthrough/espmax_passthrough.ino`, flash.
3. Run the micro-ROS agent on the host so the node appears on the ROS graph:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```

### Compile status

A real ESP32 toolchain build was **not** possible in the dev sandbox (the
`~/.arduino15` toolchain lives on an unmounted external drive and the system disk
is full). What *was* verified: a **host-side syntax parse** of the sketch with
`g++` (Arduino/micro-ROS APIs stubbed, the real `LobotSerialServoControl.h`
included) — **0 errors**. That catches our own typos, type and signature mistakes,
and bad array indexing, but does **not** validate the micro-ROS/ESP32 ABIs or
linking. The genuine `arduino-cli compile` / IDE build is still a bench step.

When you do build on the workstation, version pairing matters — `micro_ros_arduino`
is locked to an ESP32 core version. A known-good combo is **esp32 core 2.0.x +
micro_ros_arduino humble**; a 3.x core may fail to link the precompiled lib.

## Why pulses, not radians

The rad↔pulse calibration depends on each servo's mechanical zero/direction. We
keep that in the **host driver** (a small YAML, tuned against the real arm), so the
firmware never has to guess the company's internal angle frame. Initial calibration
hypothesis comes from the company `pulse_to_deg`/`deg_to_pulse` in `_espmax.cpp`:

- servo1 (base):     `angle_deg = pulse * 240/1000`
- servo2 (shoulder): `angle_deg = -pulse * 240/1000 + 210`
- servo3 (elbow):    `angle_deg = pulse * 240/1000 - 120`

These map the company's *mechanical* angle frame, not our URDF joint frame — treat
as a starting point and **verify on hardware**.

## Next layers (not built yet)

- Host driver node: rad↔pulse calibration, computes passive `end_effector_joint`
  (`θ₄ = 2π − π/2 − θ_shoulder − θ_elbow`), publishes `/joint_states`, executes
  MoveIt trajectories by streaming waypoints to `/arm/servo_command`.
- Rework `new_arm_description` + `new_arm_moveit_config` to a 3-DOF planning group
  with the 4th joint passive.
