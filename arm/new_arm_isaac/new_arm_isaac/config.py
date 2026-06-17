#!/usr/bin/env python3
"""Shared paths and constants for the new_arm Isaac Sim package.

Centralised so every script agrees on joint order, prim paths, calibration, and
(critically on this machine) keeps all heavy USD/cache output on DataDrive1 — the
internal disk is full.
"""

import os

# --- Isaac Sim 5.1.0 standalone (use the MOUNTED DataDrive1 path; the install's own
#     _isaac_sim symlink points at an unmounted drive and is broken). ---
ISAAC_ROOT = "/media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64"

# --- Our robot description (single source of truth) ---
WORKSPACE = "/home/salman/Documents/our_work"
URDF_PATH = os.path.join(WORKSPACE, "new_arm_description/urdf/new_arm.urdf")

# --- Output / scratch (MUST live off the full root disk) ---
DATA_DIR = "/media/salman/DataDrive1/new_arm_isaac"
USD_PATH = os.path.join(DATA_DIR, "usd", "new_arm.usd")
SOCKET_USD_PATH = os.path.join(DATA_DIR, "usd", "socket.usd")  # standalone socket (for the demo + SDG)
SOCKET_MARKER_USD_PATH = os.path.join(DATA_DIR, "usd", "socket_marker.usd")  # hole-centred socket for per-env RL markers
OV_CACHE = os.path.join(DATA_DIR, "ov_cache")

# --- Articulation prim path inside the stage ---
ROBOT_PRIM = "/World/NewArm"

# --- Joints (must match URDF + the rest of the stack) ---
ACTUATED_JOINTS = ["after_base_full_joint", "link1_joint", "link2_joint"]
PASSIVE_JOINT = "end_effector_joint"
ALL_JOINTS = ACTUATED_JOINTS + [PASSIVE_JOINT]
TIP_LINK = "end_effector_link"
BASE_LINK = "arm_base_link"

# --- End-effector RGB camera: Logitech C920 HD Pro webcam, baked onto end_effector_link ---
# Mounted as a child prim of the EE link so it rigidly follows the gripper. At home the
# EE frame is axis-aligned with the base (+X forward/reach, +Y lateral, +Z up). The
# camera is offset +14 cm forward (past the end-effector/gripper mesh, which otherwise
# blocks the view) and +4 cm up, then pitched 45 deg forward-and-down AND rolled 90 deg
# about its optical axis so the image is PORTRAIT and upright (before the roll the
# eye-in-hand "up" pointed sideways/lateral). Optical -Z stays along (+X,-Z)/sqrt2.
# RGB only (a plain webcam, no depth). C920 1080p pinhole, 78 deg diagonal FOV; in this
# portrait orientation that is HFOV 43.3 deg (lateral) x VFOV 70.4 deg (forward-up).
CAMERA_MODEL = "Logitech C920 HD Pro"
CAMERA_PRIM_NAME = "logitech_camera"
CAMERA_PARENT_LINK = TIP_LINK                 # end_effector_link
CAMERA_OFFSET_XYZ = (0.20, 0.0, 0.05)         # metres, EE frame: at the gripper TIP (gripper ends ~0.20), slightly up
CAMERA_PITCH_DEG = -60.0                        # RotateY: look forward + ~30deg down at the plug tip + socket ahead (gripper behind)
CAMERA_ROLL_DEG = -90.0                        # RotateZ about optical axis -> portrait, upright image
# Isaac/USD pinhole intrinsics (mm-equivalent; FOV = 2*atan(aperture/(2*focal))).
# Apertures are swapped for the 90 deg portrait roll so pixels stay square: the wide
# 70.4 deg C920 FOV is now VERTICAL (forward-up) and the 43.3 deg is horizontal (lateral).
CAMERA_FOCAL_LENGTH = 14.84588                 # focal unchanged
CAMERA_HORIZONTAL_APERTURE = 11.78719          # -> HFOV 2*atan(11.78719/(2*14.84588)) = 43.3 deg
CAMERA_VERTICAL_APERTURE = 20.955              # -> VFOV 70.4 deg
CAMERA_CLIPPING_RANGE = (0.01, 1000.0)         # metres; plain render frustum (webcam, no depth)
CAMERA_FOCUS_DISTANCE = 40.0                   # irrelevant at f_stop=0 (pinhole, no DoF)
CAMERA_RESOLUTION = (1080, 1920)               # (w, h) native C920 1080p, portrait
CAMERA_RL_RESOLUTION = (72, 128)               # (w, h) small 9:16 portrait render for batched RL

# --- Socket (wall outlet) + plug for the pick-and-insert / CenterPose perception scene ---
# Procedural geometry (pure pxr, like the camera). The SOCKET is static world geometry on a
# stand IN FRONT of the arm, facing back toward the base; the PLUG is a rigid child of
# end_effector_link (mirrors the camera bake) pointing forward along EE +X so it can insert.
# Socket face placed at a wrist-reachable pose (IK-verified: target (0.22,0,0.12) -> q ok,
# level wrist). The faceplate front face sits at SOCKET_POSE_XYZ[0] - SOCKET_FACEPLATE_HALF[0].
SOCKET_PRIM = "/World/Socket"
SOCKET_SEMANTIC_CLASS = "socket"               # Replicator/CenterPose semantic label
SOCKET_POSE_XYZ = (0.334, 0.0, 0.12)           # faceplate CENTRE (front face at 0.33; sits past the 0.20 m gripper)
SOCKET_FACEPLATE_HALF = (0.004, 0.043, 0.043)  # half-extents (X depth, Y width, Z height) -> 8x86x86 mm
SOCKET_HOLE_RADIUS = 0.0065                     # m, the two round pin holes
SOCKET_HOLE_SPACING = 0.019                     # m, centre-to-centre (EU type-C/F)
SOCKET_HOLE_DEPTH = 0.012                       # m, recess depth into the faceplate
SOCKET_STAND_HALF = (0.02, 0.02, 0.06)         # half-extents of the support post below the faceplate

PLUG_PRIM_NAME = "plug"
PLUG_PARENT_LINK = TIP_LINK                     # end_effector_link (same parent as the camera)
PLUG_OFFSET_XYZ = (0.18, 0.0, 0.0)             # m, EE frame: body starts near the gripper tip (gripper -> 0.20)
PLUG_BODY_LENGTH = 0.04                         # m, plug body (cylinder, axis = EE +X) -> emerges to 0.22
PLUG_BODY_RADIUS = 0.013                        # m
PLUG_PRONG_LENGTH = 0.03                        # m, the two pins -> tip at 0.25 (clearly past the gripper)
PLUG_PRONG_RADIUS = 0.0035                      # m
PLUG_PRONG_SPACING = 0.019                      # m, must match SOCKET_HOLE_SPACING

# --- ROS 2 topics (match new_arm_driver/arm_bridge.py exactly) ---
TOPIC_SERVO_COMMAND = "/arm/servo_command"     # pulses in  (twin plays ESP32)
TOPIC_SERVO_FEEDBACK = "/arm/servo_feedback"   # pulses out (twin plays ESP32)
TOPIC_JOINT_STATES = "/joint_states"

# --- Named poses (from SRDF; 3 actuated values) ---
NAMED_POSES = {
    "home": [0.0, 0.0, 0.0],
    "ready": [0.0, 0.4, -0.6],
}

# --- rad <-> Lobot-pulse calibration (MUST mirror
#     new_arm_driver/config/calibration.yaml so the twin behaves like the firmware).
#     pulse = scale * theta_rad + offset, clamped to [pulse_min, pulse_max]. ---
CAL_SCALE = [238.7324, -238.7324, 238.7324]
CAL_OFFSET = [625.0, 875.0, 500.0]
CAL_PULSE_MIN = [0.0, 0.0, 470.0]
CAL_PULSE_MAX = [1000.0, 700.0, 1000.0]


def theta_to_pulse(i, theta):
    """Actuated joint i (rad) -> Lobot pulse, clamped (matches arm_bridge)."""
    p = CAL_SCALE[i] * theta + CAL_OFFSET[i]
    return max(CAL_PULSE_MIN[i], min(CAL_PULSE_MAX[i], p))


def pulse_to_theta(i, pulse):
    """Lobot pulse -> actuated joint i (rad) (matches arm_bridge)."""
    if CAL_SCALE[i] == 0.0:
        return 0.0
    return (pulse - CAL_OFFSET[i]) / CAL_SCALE[i]


def ensure_dirs():
    """Create the DataDrive1 output dirs (safe to call repeatedly)."""
    for d in (DATA_DIR, os.path.dirname(USD_PATH), OV_CACHE):
        os.makedirs(d, exist_ok=True)
