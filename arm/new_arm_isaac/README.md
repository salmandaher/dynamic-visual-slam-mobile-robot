# new_arm_isaac

The new_arm recreated in **Isaac Sim 5.1.0** + **IsaacLab 0.54.2**: analytic FK/IK,
a physics sim, a ROS 2 **digital twin** that drops into the existing stack, and an
**RL reach task**. Same arm model and math as the rest of `our_work`, so the twin
matches the real robot.

> Isaac install: `/media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64`
> (use this DataDrive1 path — the install's own `_isaac_sim` symlink points at an
> unmounted drive). IsaacLab: `~/Documents/grad_robot/IsaacLab`.
> **The system disk is full** — everything writes USD/caches to
> `/media/salman/DataDrive1/new_arm_isaac` (see `config.py`).

## Layout

```
new_arm_isaac/
  new_arm_isaac/
    config.py            paths, joint names, rad<->pulse calibration, named poses
    kinematics.py        analytic 3-DOF FK/IK (numpy) + faithful firmware port
    kinematics_torch.py  batched FK + passive coupling (torch) for RL
    usd_camera.py        bakes the EE Logitech HD RGB camera into a USD stage
    tasks/new_arm_reach_env.py   IsaacLab DirectRLEnv  (NewArm-Reach-v0)
    agents/rsl_rl_ppo_cfg.py     PPO runner config
    __init__.py          registers the gym task
  scripts/
    import_urdf.py       URDF -> USD (run once); also bakes the EE camera
    add_ee_camera.py     add/refresh the EE camera on an existing USD (no re-import)
    digital_twin.py      ROS 2 twin = drop-in ESP32 replacement
    standalone_control.py  no-ROS Cartesian/named-pose demo
```

## 0. One-time: convert the URDF to USD

```bash
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/import_urdf.py
# -> /media/salman/DataDrive1/new_arm_isaac/usd/new_arm.usd
```
It prints the imported joint names — confirm they match `config.ACTUATED_JOINTS`
(`after_base_full_joint`, `link1_joint`, `link2_joint`) and `PASSIVE_JOINT`
(`end_effector_joint`). If the importer renamed them, edit `config.py`. It also bakes
the end-effector RGB camera (below).

## 0b. End-effector RGB camera (Logitech C920 HD Pro)

A Logitech HD webcam is baked onto `end_effector_link` as a child prim `logitech_camera`,
so the twin, standalone demo, and every cloned RL env inherit it for free. It is mounted
**+14 cm forward (EE +X)** — past the gripper mesh, which otherwise blocks the view —
**and +4 cm up (EE +Z)** of the wrist pivot, pitched **45° forward and down** and rolled
**90° about the optical axis** so the image is **portrait** and upright. RGB only
(no depth). C920 1080p pinhole (78° diagonal FOV); in portrait that is HFOV 43.3° ×
VFOV 70.4° (apertures + resolution swapped to keep square pixels). All values live in `config.py`
(`CAMERA_*`).

`import_urdf.py` bakes it during conversion. To add or update it on an **existing** USD
without a full re-import (idempotent):
```bash
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
PKG=$(ls -d extscache/omni.usd.libs-*lx64.r.cp311 | head -1)
PYTHONPATH="$PWD/$PKG:$PYTHONPATH" LD_LIBRARY_PATH="$PWD/$PKG/bin:$LD_LIBRARY_PATH" \
  ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/add_ee_camera.py --verify
```
The baked prim is geometry+intrinsics only. To harvest `rgb` in RL, set
`enable_camera=True` on `NewArmReachEnvCfg` and launch the runner with `--enable_cameras`;
the env attaches a `TiledCamera` (`spawn=None`, reusing the baked prim) and exposes
`env.camera_rgb()`. This is OFF by default so the verified 12-dim reach policy is
unchanged.

## 1. Quick visual check (no ROS)

```bash
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/standalone_control.py
```
Drives a tour of named poses + Cartesian goals via the analytic IK, printing the FK
error for each. `--headless` to skip the window.

## 2. ROS 2 digital twin (the main event)

The twin **plays the ESP32**: subscribes to `/arm/servo_command` (pulses) and
publishes `/arm/servo_feedback` (pulses), using the same calibration as the firmware.
So the *whole existing stack* drives the simulated arm unchanged.

```bash
# terminal 1 — the normal stack WITHOUT the micro-ROS agent
ros2 launch new_arm_moveit_config arm_bringup.launch.py

# terminal 2 — the Isaac twin (replaces the real arm)
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
source setup_ros_env.sh            # ROS_DISTRO=humble + bridge libs
./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/digital_twin.py
```

Now everything you already built drives Isaac:
- CLI: `ros2 run new_arm_ui arm_cli goto 0.135 0 0.156`
- Web UI, MoveIt "Plan & Execute", or raw `/arm/joint_command`.

Flags: `--headless`; `--publish-joint-states` makes the twin also publish
`sensor_msgs/JointState` on `/joint_states` directly (point RViz/MoveIt straight at
Isaac, no arm_bridge).

## 3. RL reach task (IsaacLab)

```bash
cd /home/salman/Documents/grad_robot/IsaacLab
# register the package once
./isaaclab.sh -p -m pip install -e /home/salman/Documents/our_work/new_arm_isaac
# train (headless, many envs)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task NewArm-Reach-v0 --headless
# watch a trained policy
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task NewArm-Reach-v0 --num_envs 16
```

Task: drive the wrist tip to a random reachable target. Action = 3 joint deltas (no
IK in the loop); obs = `[pos_error(3), q(3), dq(3), prev_action(3)]`; the passive
wrist is commanded to its mechanical coupling every step.

## What's verified vs not

**Verified here (pure Python, ran):**
- FK matches independent homogeneous composition to **5.6e-17 m**.
- Analytic IK (Newton-polished) round-trips over ~4500 random reachable targets:
  **worst 0.21 mm, typically 0**, zero failures (>1 mm).
- Passive wrist at home = **0 rad** (level; coupling offset `2*pi`, with the firmware's
  `- pi/2` term dropped so the wrist sits level at home — see `kinematics.PASSIVE_OFFSET`).
  Firmware FK/IK port (independent of that offset) round-trips to ~1.7e-13 mm.
- Torch FK matches numpy FK to **~1e-13 m** (run in the IsaacLab env's torch).
- Calibration maps home -> pulses **(625, 875, 500)**, matching `calibration.yaml`.
- Package imports without IsaacLab (the gym.register is guarded); all modules compile.

**NOT yet run (needs the GPU box + Isaac, and the disk freed):**
- `import_urdf.py`, `digital_twin.py`, `standalone_control.py` inside Isaac Sim.
- The IsaacLab task actually training.
The Isaac API calls were written against the **installed 5.1.0 source** and the
bundled `standalone_examples`, but have not been executed. First run is a bench step
— watch for: (a) post-import joint-name renames (the script prints them),
(b) `package://` mesh resolution in the URDF importer, (c) disk/cache pressure.

## Caveats

- The twin uses the **seeded** calibration; calibrate on the real arm
  (`new_arm_driver/README.md`) for the twin's pulses to match hardware exactly.
- **Calibration finding (affects the real arm too):** with the current seeded
  `calibration.yaml`, the shoulder maps URDF range `[0.733, 3.665] rad` to pulses
  `[700, 0]`, so **URDF-home (shoulder = 0) is *below* the servo range** —
  `theta_to_pulse(shoulder, 0) = 875` clamps to the `700` max (= 0.733 rad). The
  twin reproduces this faithfully through the pulse path (`digital_twin.py`), so home
  commanded through the full stack lands the shoulder at ~0.733 rad, not 0 — exactly
  as the real arm would under this calibration. `standalone_control.py` bypasses
  calibration (drives URDF radians directly), so it shows a clean home. The proper
  fix is **hardware calibration**, not changing the numbers (that would make the twin
  diverge from the real bridge).
- Targets are the **wrist pivot** (`end_effector_link` origin), consistent with the
  3-DOF MoveIt group.
