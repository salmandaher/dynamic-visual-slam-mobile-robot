#!/usr/bin/env python3
"""Drive the mobile base AND publish its sensors/state to ROS 2 — Isaac Sim 5.1.0.

Opens the finished scene (new_built_robot.usd: robot_base articulation + 4 wheel
drives + welded D435i + GroundPlane + PhysicsScene + obstacles) and does two
things every simulation step:

  1. MOVES the robot, smoothly. Three drive modes (one authoritative writer of
     wheel velocity targets via isaacsim.core.prims.Articulation, so they never
     fight): default built-in demo "patrol"; --teleop follows geometry_msgs/Twist
     on /cmd_vel; --keyboard drives with the ARROW KEYS in the GUI (Up/Down =
     forward/back, Left/Right = turn) plus a small on-screen status window.
     Smoothness: wheel targets are slew-rate limited (--accel) so commands ramp
     instead of stepping, and physics runs at --physics-hz (default 120, i.e. 2
     substeps/render frame). The scene itself was tuned for smooth rolling by
     apply_scene_fixes.py (explicit wheel/base masses, solver iterations,
     restitution=0, SDF colliders kept) — run that once if you haven't.

  2. PUBLISHES to ROS 2 from ONE OmniGraph (all heavy work on the C++ side of
     isaacsim.ros2.bridge, so the sim stays real-time):
        /clock                          rosgraph_msgs/Clock
        /joint_states                   sensor_msgs/JointState   <- WHEEL ENCODERS
                                          (position rad, velocity rad/s, effort)
        /tf                             tf2_msgs/TFMessage       (robot link tree)
        /odom                           nav_msgs/Odometry        (base pose+twist)
        /camera/color/image_raw         sensor_msgs/Image
        /camera/color/camera_info       sensor_msgs/CameraInfo
        /camera/depth/image_rect_raw    sensor_msgs/Image   (32FC1 metres)
        /camera/depth/camera_info       sensor_msgs/CameraInfo
        /camera/depth/points            sensor_msgs/PointCloud2  (only with --pointcloud)
     and SUBSCRIBES /cmd_vel (geometry_msgs/Twist) for --teleop.

DEPTH IN RVIZ: the depth image is 32FC1 (float metres). RViz's Image display does
NOT normalize float images, so it looks blank even though it IS publishing (RGB is
rgb8 and renders fine). To SEE the depth: use `rqt_image_view` (it normalizes float
to grayscale), or run with --pointcloud and add an RViz PointCloud2 display on
/camera/depth/points with Fixed Frame = World (--pointcloud also publishes the
camera frames to /tf so the cloud resolves).

Every node type id and every inputs:/outputs: attribute below was verified
against the on-disk Isaac Sim 5.1.0 .ogn / Template.usda / .rst schemas.

RUN (source ROS 2 Humble first so the bridge links system rclcpp):
    source /opt/ros/humble/setup.bash
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/simsim/robot_drive_and_publish.py
    #   (default = demo patrol)
    #   --keyboard   drive with the arrow keys in the GUI (click the viewport first)
    #   --teleop     follow /cmd_vel (e.g. ros2 run teleop_twist_keyboard teleop_twist_keyboard)
    #   --headless   no GUI (demo/teleop only; keyboard needs the GUI)

VERIFY (another terminal, plain Humble):
    ros2 topic list
    ros2 topic hz /joint_states
    ros2 topic echo /joint_states --once          # wheel encoder positions/velocities
    ros2 topic hz /camera/color/image_raw
    ros2 topic echo /odom --once
    ros2 run tf2_tools view_frames                 # robot_base -> wheel frames
    # drive it yourself (needs --teleop):
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

FRAME-ID NOTE: /tf frame names come from the USD prim names — the base link is
"robot_base" and the wheels are "right_back / left_back / right_front /
left_front". /odom uses chassisFrameId="robot_base" to match. The D435i optical
frames (camera_color_optical_frame, camera_depth_optical_frame) are NOT in /tf
(the camera is a separate welded body); publish a static_transform_publisher
robot_base -> camera_*_optical_frame if you need RViz to place the images.
"""

import argparse
import math
import os
import sys

# --- scene + prim paths -----------------------------------------------------
DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot_gui.usd"
ART_ROOT = "/World/robot/robot_base"   # carries PhysicsArticulationRootAPI
CAMERA_ROOT = "/World/robot/d435i_camera"  # welded D435i body (camera_link/{color,depth}_camera)
COLOR_PRIM = "/World/robot/d435i_camera/camera_link/color_camera"
DEPTH_PRIM = "/World/robot/d435i_camera/camera_link/depth_camera"
# ROS optical frames: child Xforms of each USD camera, rotated 180 deg about X so
# they match the ROS optical convention (+Z forward, +Y down) the bridge publishes
# images/clouds in. Authored at runtime and published in /tf so RViz can place the
# depth image AND the point cloud (previously their frames were absent from /tf).
DEPTH_OPT_PRIM = DEPTH_PRIM + "/camera_depth_optical_frame"
COLOR_OPT_PRIM = COLOR_PRIM + "/camera_color_optical_frame"
GRAPH_PATH = "/RobotDriveAndPublishGraph"

# Wheel revolute-joint DOF names == joint prim names in the USD.
# DIFFERENTIAL DRIVE on the two BACK wheels (right_back, left_back). The two front
# "dual omni" wheels are passive casters (joint drives zeroed), so they free-spin
# and are never commanded. Order = the velocity command array.
DRIVEN = ["right_back", "left_back"]
# Differential drive: forward = both back wheels SAME sign (translation); turn =
# OPPOSITE signs (rotation).

# --- TETRIX MAX TorqueNADO DC gearmotor spec (torquenado_dcmotorspecs.pdf) ----
# Each driven wheel is one TorqueNADO (12 VDC brushed, 8.7 A stall). Per gearbox:
#   gear : (output no-load rpm, output stall torque oz-in, encoder cpr)
OZIN_TO_NM = 0.00706155
RPM_TO_RADS = math.pi / 30.0
TORQUENADO = {60: (100, 700, 1440), 40: (150, 466, 960), 20: (300, 233, 480)}
BACK_JOINTS = [
    "/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]


# --- args -------------------------------------------------------------------
parser = argparse.ArgumentParser(
    description="Drive the base + publish camera/encoders/tf/odom to ROS 2 (Isaac Sim 5.1.0)")
parser.add_argument("--headless", action="store_true",
                    help="run without the Kit GUI (cameras still render off-screen)")
parser.add_argument("--scene", default=DEFAULT_SCENE, help="USD scene to open")
parser.add_argument("--robot-prefix", default="/World/robot",
                    help="prim-path prefix of the robot GROUP (the parent of robot_base, the wheel xforms, "
                         "and d435i_camera). Default '/World/robot'. Wrapper scenes that payload the robot "
                         "under another prim need this: e.g. lizard.usd payloads new_built_robot_ros.usd, so "
                         "pass --robot-prefix /World/new_built_robot_ros/robot")
parser.add_argument("--teleop", action="store_true",
                    help="follow /cmd_vel (geometry_msgs/Twist) instead of the built-in demo motion")
parser.add_argument("--keyboard", action="store_true",
                    help="drive with the ARROW KEYS in the Isaac GUI (needs the GUI; overrides --teleop). "
                         "Up/Down = forward/back, Left/Right = turn; release to coast to a stop")
# motion tuning (linear m/s, angular rad/s for the demo AND keyboard/cmd_vel)
parser.add_argument("--wheel-radius", type=float, default=0.049,
                    help="EFFECTIVE rolling radius in m for the real 4-inch wheel (measured ~0.049 by "
                         "diagnose_motion.py). Maps v,w -> back-wheel rad/s")
parser.add_argument("--track-width", type=float, default=0.56,
                    help="EFFECTIVE turn track in m for the v,w mapping (measured ~0.56 by diagnose_motion.py)")
parser.add_argument("--lin-step", type=float, default=0.4,
                    help="linear speed for keyboard Up/Down and the demo, in m/s (default 0.4; the 4-in wheel "
                         "at 100 rpm tops out near 0.53 m/s, so the motor caps this)")
parser.add_argument("--ang-step", type=float, default=2.5,
                    help="angular speed for keyboard Left/Right and the demo, in rad/s (default 2.5; high so a "
                         "turn commands near-full motor torque)")
parser.add_argument("--accel", type=float, default=3.0,
                    help="wheel-target slew-rate limit in rad/s^2 — ramps commands so motion is smooth and "
                         "the wheels don't spin up faster than they can grip (default 3; lower = gentler)")
parser.add_argument("--physics-hz", type=int, default=120,
                    help="physics solver rate in Hz; >60 substeps each render frame for smoother contacts "
                         "(default 120)")
parser.add_argument("--gear", type=int, default=60, choices=[60, 40, 20],
                    help="TorqueNADO gearbox ratio: 60:1 stock (100 rpm/700 oz-in), 40:1 (150/466), "
                         "20:1 (300/233). Sets the wheel motor's free-speed cap + stall-torque limit")
parser.add_argument("--ideal-drive", action="store_true",
                    help="bypass the TorqueNADO model and use a stiff ideal velocity servo (old behavior)")
parser.add_argument("--cpu-dynamics", action="store_true",
                    help="do NOT force GPU dynamics on. By default the launch forces GPU dynamics + GPU "
                         "broadphase because the SDF mesh wheel colliders are GPU-only in PhysX; without it "
                         "they don't collide and sink through the ground (front casters drag). Use this only "
                         "to reproduce the broken behavior.")
parser.add_argument("--reverse", action="store_true",
                    help="flip forward/back if the base drives the wrong way (negates linear v)")
parser.add_argument("--flip-turn", action="store_true",
                    help="flip turn direction if Left/Right are reversed (negates angular w)")
parser.add_argument("--no-pointcloud", dest="pointcloud", action="store_false",
                    help="disable the depth PointCloud2 on /camera/depth/points (it is ON by default so the "
                         "depth is viewable in RViz as a 3D cloud)")
parser.set_defaults(pointcloud=True)
parser.add_argument("--warmup", type=int, default=12,
                    help="render steps to settle the pipeline before driving")
# ROS 2 topics / namespaces
parser.add_argument("--robot-namespace", default="",
                    help="namespace for /joint_states /tf /odom /cmd_vel /clock (default root)")
parser.add_argument("--camera-namespace", default="camera",
                    help="namespace for the camera topics (default 'camera')")
parser.add_argument("--joint-topic", default="joint_states")
parser.add_argument("--odom-topic", default="odom")
parser.add_argument("--cmd-vel-topic", default="cmd_vel")
parser.add_argument("--base-frame", default="robot_base",
                    help="odom child frame id (should match the /tf base link prim name)")
parser.add_argument("--odom-frame", default="odom")
parser.add_argument("--color-topic", default="color/image_raw")
parser.add_argument("--color-info-topic", default="color/camera_info")
parser.add_argument("--depth-topic", default="depth/image_rect_raw")
parser.add_argument("--depth-info-topic", default="depth/camera_info")
parser.add_argument("--pcl-topic", default="depth/points",
                    help="depth PointCloud2 topic (only with --pointcloud)")
parser.add_argument("--color-frame", default="camera_color_optical_frame")
parser.add_argument("--depth-frame", default="camera_depth_optical_frame")
parser.add_argument("--color-width", type=int, default=1280)
parser.add_argument("--color-height", type=int, default=720)
parser.add_argument("--depth-width", type=int, default=848)
parser.add_argument("--depth-height", type=int, default=480)
parser.add_argument("--no-opencv-calib", action="store_true",
                    help="skip authoring OpenCV-pinhole intrinsics (re-introduces Isaac's "
                         "'Forcing fy to fx' camera_info warning)")
args, _ = parser.parse_known_args()

# --- derive every robot prim path from --robot-prefix --------------------------
# The module-level constants above are the defaults for /World/robot; rebind them
# from the prefix so ONE driver works on both the flat scenes (new_built_robot_*)
# and wrapper scenes that payload the robot under a different prim (lizard.usd ->
# /World/new_built_robot_ros/robot). Functions read these as globals at call time,
# so reassigning here takes effect everywhere.
_P = args.robot_prefix.rstrip("/")
ART_ROOT = _P + "/robot_base"
CAMERA_ROOT = _P + "/d435i_camera"
COLOR_PRIM = CAMERA_ROOT + "/camera_link/color_camera"
DEPTH_PRIM = CAMERA_ROOT + "/camera_link/depth_camera"
DEPTH_OPT_PRIM = DEPTH_PRIM + "/camera_depth_optical_frame"
COLOR_OPT_PRIM = COLOR_PRIM + "/camera_color_optical_frame"
BACK_JOINTS = [
    _P + "/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    _P + "/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]

if not os.path.isfile(args.scene):
    sys.exit(f"[drive] scene not found: {args.scene}")

# --- Isaac Sim must boot before any omni / isaacsim import ------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting"})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni  # noqa: E402
import omni.usd  # noqa: E402
import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402


def log(m):
    print(f"[drive] {m}", flush=True)


def author_optical_frames(stage):
    """Create the ROS optical frames as child Xforms of the USD cameras, rotated
    180 deg about X (USD camera = -Z forward / +Y up  ->  ROS optical = +Z forward
    / +Y down). Publishing THESE in /tf is what makes the depth image and the point
    cloud actually appear in RViz — their frame_ids resolve to a pose under world.
    Authored at runtime (not saved), so it works for any scene file."""
    from pxr import UsdGeom, Gf
    q180x = Gf.Quatf(0.0, 1.0, 0.0, 0.0)   # w,x,y,z = 180 deg about X
    for cam_path, leaf in ((DEPTH_PRIM, "camera_depth_optical_frame"),
                           (COLOR_PRIM, "camera_color_optical_frame")):
        p = cam_path + "/" + leaf
        xf = UsdGeom.Xform.Define(stage, p)
        # fresh stage each run -> the op won't pre-exist; set the optical rotation
        xf.ClearXformOpOrder()
        xf.AddOrientOp().Set(q180x)
    log("optical frames authored (camera_depth/color_optical_frame, 180deg about X)")


def apply_motor_model(stage, gear):
    """Make each driven back-wheel joint behave like a TorqueNADO DC gearmotor.
    A PhysX velocity drive computes torque = damping*(target-omega) clamped to
    maxForce; setting damping = stall/free (back-EMF slope) and maxForce = stall
    reproduces the brushed-DC torque-speed line, and the wheel can't exceed the
    motor's free speed. Returns free_speed (rad/s) used to cap wheel commands."""
    from pxr import Sdf
    rpm, ozin, cpr = TORQUENADO[gear]
    w_free = rpm * RPM_TO_RADS
    t_stall = ozin * OZIN_TO_NM
    k = t_stall / w_free
    for jp in BACK_JOINTS:
        j = stage.GetPrimAtPath(jp)
        if not j or not j.IsValid():
            log(f"WARNING: motor-model joint missing: {jp}")
            continue
        for name, val in (("drive:angular:physics:stiffness", 0.0),
                          ("drive:angular:physics:damping", float(k)),
                          ("drive:angular:physics:maxForce", float(t_stall))):
            a = j.GetAttribute(name) or j.CreateAttribute(name, Sdf.ValueTypeNames.Float)
            a.Set(val)
    log(f"TorqueNADO {gear}:1 motor: free {w_free:.2f} rad/s ({rpm} rpm), "
        f"stall {t_stall:.2f} N*m ({ozin} oz-in), damping {k:.3f} N*m/(rad/s), encoder {cpr} cpr")
    return w_free


def author_opencv_pinhole(stage, cam_path, width, height):
    """Author an exact OpenCV-pinhole lens model (square fx==fy, zero distortion)
    so ROS2CameraInfoHelper reports precise intrinsics and Isaac emits no
    'Forcing fy to fx' / 'Unsupported distortion model' warnings. fx is derived
    from the lens so horizontal FOV is preserved. (Same proven routine as
    robot_base_camera_ros.py.)"""
    from pxr import Sdf, Gf
    prim = stage.GetPrimAtPath(cam_path)
    focal = float(prim.GetAttribute("focalLength").Get())
    h_ap = float(prim.GetAttribute("horizontalAperture").Get())
    fx = width * focal / h_ap
    P = "omni:lensdistortion:opencvPinhole:"
    vals = {
        "omni:lensdistortion:model": (Sdf.ValueTypeNames.Token, "opencvPinhole"),
        P + "imageSize": (Sdf.ValueTypeNames.Int2, Gf.Vec2i(int(width), int(height))),
        P + "cx": (Sdf.ValueTypeNames.Float, width * 0.5),
        P + "cy": (Sdf.ValueTypeNames.Float, height * 0.5),
        P + "fx": (Sdf.ValueTypeNames.Float, fx),
        P + "fy": (Sdf.ValueTypeNames.Float, fx),
    }
    for c in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"):
        vals[P + c] = (Sdf.ValueTypeNames.Float, 0.0)
    for name, (vtype, val) in vals.items():
        attr = prim.GetAttribute(name)
        if not attr:
            attr = prim.CreateAttribute(name, vtype)
        attr.Set(val)
    return fx


# --- motion helpers ---------------------------------------------------------
def demo_twist(t, v_max, w_max):
    """Return a demo body twist (v m/s, w rad/s) for sim time t (loops)."""
    phases = [
        (v_max,        0.0,    4.0),   # forward
        (0.6 * v_max,  w_max,  3.0),   # forward + arc left
        (v_max,        0.0,    4.0),   # forward
        (0.6 * v_max, -w_max,  3.0),   # forward + arc right
        (0.0,          w_max,  2.5),   # spin left in place
        (v_max,        0.0,    3.0),   # forward
        (0.0,          0.0,    1.0),   # brief pause
    ]
    total = sum(p[2] for p in phases)
    tt = t % total
    acc = 0.0
    for v, w, dur in phases:
        if tt < acc + dur:
            return v, w
        acc += dur
    return 0.0, 0.0


def diff_drive(v, w, radius, track, reverse=False, flip_turn=False):
    """Differential drive: body twist (v m/s, w rad/s) -> back-wheel velocity
    targets in rad/s, ordered [right_back, left_back] == DRIVEN.

    SIGN CONVENTION (verified empirically by direction_check.py against the wheel
    mounting in this scene): +v drives toward the robot FRONT (+X, where the D435i
    looks) and +w turns LEFT (CCW). Both wheel targets are NEGATED relative to the
    naive formula because a +joint-velocity on these wheels rolls the base toward
    -X. Forward = both wheels SAME sign (translate); turn = OPPOSITE signs (rotate).
    Use --reverse / --flip-turn to invert if you remount the wheels."""
    if reverse:
        v = -v
    if flip_turn:
        w = -w
    omega_r = -(v + w * track * 0.5) / radius
    omega_l = -(v - w * track * 0.5) / radius
    return np.array([omega_r, omega_l], dtype=np.float32)   # order = DRIVEN [right_back, left_back]


def read_cmd_vel():
    """Read the latest /cmd_vel from the SubscribeTwist node outputs; (v, w)."""
    try:
        lin = og.Controller.attribute(f"{GRAPH_PATH}/Twist.outputs:linearVelocity").get()
        ang = og.Controller.attribute(f"{GRAPH_PATH}/Twist.outputs:angularVelocity").get()
        return float(lin[0]), float(ang[2])
    except Exception:
        return 0.0, 0.0


def slew(current, desired, max_delta):
    """Move `current` toward `desired` by at most max_delta per element (smoothing)."""
    return current + np.clip(desired - current, -max_delta, max_delta)


class KeyboardTeleop:
    """Arrow-key teleop for the Isaac GUI + a small omni.ui status window.

    Builds a live (v, w) command from the SET of currently-held arrow keys
    (recomputed on every press/release so a missed event can't desync it).
    All Kit imports are local so a headless / non-keyboard run never touches them.
    Verified against standalone_examples/.../anymal_standalone.py (Isaac 5.1.0)."""

    def __init__(self, lin_step, ang_step):
        self.lin_step = lin_step
        self.ang_step = ang_step
        self.v = 0.0
        self.w = 0.0
        self._pressed = set()
        self._input = None
        self._keyboard = None
        self._sub = None
        self._window = None
        self._label = None

    def setup(self):
        import carb.input
        import omni.appwindow
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._sub = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_key)
        self._build_ui()

    def _on_key(self, event, *args, **kwargs):
        import carb.input
        K = carb.input.KeyboardInput
        T = carb.input.KeyboardEventType
        if event.input in (K.UP, K.DOWN, K.LEFT, K.RIGHT,
                           K.W, K.S, K.A, K.D):
            if event.type == T.KEY_PRESS:
                self._pressed.add(event.input)
            elif event.type == T.KEY_RELEASE:
                self._pressed.discard(event.input)
            self._recompute()
        return True  # callback must return bool

    def _recompute(self):
        import carb.input
        K = carb.input.KeyboardInput
        p = self._pressed
        v = w = 0.0
        if K.UP in p or K.W in p:    v += self.lin_step
        if K.DOWN in p or K.S in p:  v -= self.lin_step
        if K.LEFT in p or K.A in p:  w += self.ang_step   # +angular = turn left (CCW)
        if K.RIGHT in p or K.D in p: w -= self.ang_step
        self.v, self.w = v, w

    def _build_ui(self):
        try:
            import omni.ui as ui
            self._window = ui.Window("Robot Teleop", width=340, height=170)
            with self._window.frame:
                with ui.VStack(spacing=6):
                    ui.Label("Arrow keys (or WASD) to drive:")
                    ui.Label("  Up / Down    = forward / reverse")
                    ui.Label("  Left / Right = turn left / right")
                    ui.Label("Release keys to stop. Click the viewport first.")
                    self._label = ui.Label("v = +0.00 m/s   w = +0.00 rad/s")
        except Exception as e:  # noqa: BLE001 - UI is optional
            print(f"[drive] teleop UI window skipped: {e!r}", flush=True)

    def update_ui(self):
        if self._label is not None:
            self._label.text = f"v = {self.v:+.2f} m/s   w = {self.w:+.2f} rad/s"

    def teardown(self):
        try:
            if self._sub is not None:
                self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub)
                self._sub = None
            if self._window is not None:
                self._window.destroy()
                self._window = None
        except Exception:  # noqa: BLE001
            pass


def build_graph():
    """One OmniGraph: clock, camera color+depth (+info), joint encoders, tf, odom,
    and a /cmd_vel subscriber. All exec driven by OnPlaybackTick."""
    keys = og.Controller.Keys
    cam_ns = args.camera_namespace
    rob_ns = args.robot_namespace

    create_nodes = [
        ("OnTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("RpColor", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("RpDepth", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("ColorImg", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("ColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("DepthImg", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("JointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("Twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ]

    set_values = [
        # render products (relationship inputs -> list-wrapped)
        ("RpColor.inputs:cameraPrim", [COLOR_PRIM]),
        ("RpColor.inputs:width", args.color_width),
        ("RpColor.inputs:height", args.color_height),
        ("RpDepth.inputs:cameraPrim", [DEPTH_PRIM]),
        ("RpDepth.inputs:width", args.depth_width),
        ("RpDepth.inputs:height", args.depth_height),
        # color camera
        ("ColorImg.inputs:type", "rgb"),
        ("ColorImg.inputs:topicName", args.color_topic),
        ("ColorImg.inputs:frameId", args.color_frame),
        ("ColorImg.inputs:nodeNamespace", cam_ns),
        ("ColorInfo.inputs:topicName", args.color_info_topic),
        ("ColorInfo.inputs:frameId", args.color_frame),
        ("ColorInfo.inputs:nodeNamespace", cam_ns),
        # depth camera (type="depth" -> 32FC1 metres)
        ("DepthImg.inputs:type", "depth"),
        ("DepthImg.inputs:topicName", args.depth_topic),
        ("DepthImg.inputs:frameId", args.depth_frame),
        ("DepthImg.inputs:nodeNamespace", cam_ns),
        ("DepthInfo.inputs:topicName", args.depth_info_topic),
        ("DepthInfo.inputs:frameId", args.depth_frame),
        ("DepthInfo.inputs:nodeNamespace", cam_ns),
        # clock
        ("Clock.inputs:topicName", "clock"),
        ("Clock.inputs:nodeNamespace", rob_ns),
        # wheel encoders (targetPrim relationship -> articulation root, list-wrapped)
        ("JointState.inputs:topicName", args.joint_topic),
        ("JointState.inputs:targetPrim", [ART_ROOT]),
        ("JointState.inputs:nodeNamespace", rob_ns),
        # tf: the robot link tree (articulation root expands it) PLUS the two camera
        # optical frames, so /camera/depth/image_rect_raw and /camera/depth/points
        # (frame camera_*_optical_frame) resolve under world in RViz. These are
        # republished every tick, so they track the robot as it drives.
        ("TF.inputs:topicName", "tf"),
        ("TF.inputs:targetPrims", [ART_ROOT, DEPTH_OPT_PRIM, COLOR_OPT_PRIM]),
        ("TF.inputs:nodeNamespace", rob_ns),
        # odometry: compute from the chassis, publish body-frame twist as-is
        ("ComputeOdom.inputs:chassisPrim", [ART_ROOT]),
        ("PublishOdom.inputs:topicName", args.odom_topic),
        ("PublishOdom.inputs:odomFrameId", args.odom_frame),
        ("PublishOdom.inputs:chassisFrameId", args.base_frame),
        ("PublishOdom.inputs:publishRawVelocities", True),  # ComputeOdom emits LOCAL vel; publish as-is
        ("PublishOdom.inputs:nodeNamespace", rob_ns),
        # /cmd_vel subscriber (read in Python for --teleop)
        ("Twist.inputs:topicName", args.cmd_vel_topic),
        ("Twist.inputs:nodeNamespace", rob_ns),
    ]

    connect = [
        # OnPlaybackTick drives every publisher/subscriber + render products
        ("OnTick.outputs:tick", "Clock.inputs:execIn"),
        ("OnTick.outputs:tick", "RpColor.inputs:execIn"),
        ("OnTick.outputs:tick", "RpDepth.inputs:execIn"),
        ("OnTick.outputs:tick", "JointState.inputs:execIn"),
        ("OnTick.outputs:tick", "TF.inputs:execIn"),
        ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
        ("OnTick.outputs:tick", "Twist.inputs:execIn"),
        # sim-time stamps
        ("ReadTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
        ("ReadTime.outputs:simulationTime", "JointState.inputs:timeStamp"),
        ("ReadTime.outputs:simulationTime", "TF.inputs:timeStamp"),
        ("ReadTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
        # color render product -> color helpers
        ("RpColor.outputs:execOut", "ColorImg.inputs:execIn"),
        ("RpColor.outputs:execOut", "ColorInfo.inputs:execIn"),
        ("RpColor.outputs:renderProductPath", "ColorImg.inputs:renderProductPath"),
        ("RpColor.outputs:renderProductPath", "ColorInfo.inputs:renderProductPath"),
        # depth render product -> depth helpers
        ("RpDepth.outputs:execOut", "DepthImg.inputs:execIn"),
        ("RpDepth.outputs:execOut", "DepthInfo.inputs:execIn"),
        ("RpDepth.outputs:renderProductPath", "DepthImg.inputs:renderProductPath"),
        ("RpDepth.outputs:renderProductPath", "DepthInfo.inputs:renderProductPath"),
        # odometry compute -> publish (types match 1:1; both quats IJKR)
        ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
        ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
        ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
        ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
        ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
    ]

    # Depth PointCloud2 off the SAME depth render product (3D-viewable in RViz),
    # ON by default. frameId is the ROS optical frame (now published in /tf), and the
    # bridge emits the cloud in that exact convention, so it places & orients right.
    if args.pointcloud:
        create_nodes.append(("DepthPcl", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        set_values += [
            ("DepthPcl.inputs:type", "depth_pcl"),
            ("DepthPcl.inputs:topicName", args.pcl_topic),
            ("DepthPcl.inputs:frameId", args.depth_frame),  # camera_depth_optical_frame (in /tf)
            ("DepthPcl.inputs:nodeNamespace", cam_ns),
        ]
        connect += [
            ("RpDepth.outputs:execOut", "DepthPcl.inputs:execIn"),
            ("RpDepth.outputs:renderProductPath", "DepthPcl.inputs:renderProductPath"),
        ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: create_nodes, keys.SET_VALUES: set_values, keys.CONNECT: connect},
    )


def main():
    # 1) ROS 2 bridge on, then let the extension settle.
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    # 2) Open the complete scene and wait for it to load.
    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update()
    simulation_app.update()
    log(f"loading {args.scene} ...")
    while is_stage_loading():
        simulation_app.update()
    log("stage loaded")

    stage = omni.usd.get_context().get_stage()
    for p in (ART_ROOT, COLOR_PRIM, DEPTH_PRIM):
        if not stage.GetPrimAtPath(p).IsValid():
            carb.log_error(f"[drive] prim missing: {p}")
            log(f"WARNING: {p} not found — related output will be empty")

    # Optical TF frames so the depth image + point cloud are placeable in RViz.
    author_optical_frames(stage)

    # TorqueNADO DC-gearmotor behaviour on the driven wheels (real torque/speed
    # limits), unless --ideal-drive asks for the old stiff velocity servo.
    motor_free_speed = None
    if not args.ideal_drive:
        motor_free_speed = apply_motor_model(stage, args.gear)
    else:
        log("--ideal-drive: using stiff ideal velocity servo (no motor limits)")

    # 2b) Exact OpenCV-pinhole intrinsics (square fx==fy, zero distortion).
    if not args.no_opencv_calib:
        try:
            fxc = author_opencv_pinhole(stage, COLOR_PRIM, args.color_width, args.color_height)
            fxd = author_opencv_pinhole(stage, DEPTH_PRIM, args.depth_width, args.depth_height)
            log(f"OpenCV-pinhole intrinsics authored: color fx={fxc:.2f}, depth fx={fxd:.2f}")
        except Exception as e:  # noqa: BLE001
            log(f"WARNING: could not author intrinsics ({e!r}); camera_info falls back to lens-derived")

    # 3) SimulationContext: run physics faster than render (substeps) for smoother
    #    contacts. physics_dt also authors /physicsScene timeStepsPerSecond.
    render_dt = 1.0 / 60.0
    sim = SimulationContext(physics_dt=1.0 / max(1, args.physics_hz),
                            rendering_dt=render_dt, stage_units_in_meters=1.0)
    log(f"physics @ {args.physics_hz} Hz, render @ 60 Hz "
        f"({max(1, int(args.physics_hz / 60))} substeps/frame)")

    # 3b) SDF mesh colliders are GPU-ONLY in PhysX. lizard.usd payloads the robot's
    #     /World subtree but NOT the sibling /physicsScene that authored
    #     enableGPUDynamics=True, so the run falls back to a default scene with GPU
    #     dynamics OFF — the SDF wheels then don't collide and sink through the ground
    #     (front casters drag at ~0). Force GPU dynamics + GPU broadphase here, BEFORE
    #     play(), so the SDF colliders cook and collide. This is a SIM/scene setting in
    #     the launch script; it does NOT modify the robot USD or its physics props.
    if not args.cpu_dynamics:
        try:
            pc = sim.get_physics_context()
            before = (pc.is_gpu_dynamics_enabled(), pc.get_broadphase_type())
            pc.enable_gpu_dynamics(True)
            pc.set_broadphase_type("GPU")
            log(f"GPU dynamics for SDF colliders: was {before} -> now "
                f"({pc.is_gpu_dynamics_enabled()}, '{pc.get_broadphase_type()}')")
        except Exception as e:  # noqa: BLE001
            log(f"WARNING: could not force GPU dynamics ({e!r}); SDF wheels may sink through the ground")
    else:
        log("--cpu-dynamics: leaving GPU dynamics as-authored (SDF colliders may not collide → wheels sink)")

    # 4) Build the publish/subscribe OmniGraph.
    log("building OmniGraph ...")
    build_graph()
    log("OmniGraph built (clock, color+depth+info, joint_states, tf, odom, cmd_vel)")

    # 5) Wrap the articulation, then play -> update -> initialize (handles need a
    #    running sim). Re-initialize after any Stop+Play.
    art = Articulation(prim_paths_expr=ART_ROOT, name="robot_base")
    sim.play()
    simulation_app.update()

    wheel_idx = None
    try:
        art.initialize()
        # Only the back wheels are commanded; the front omni wheels free-spin.
        wheel_idx = np.array([art.get_dof_index(n) for n in DRIVEN], dtype=np.int32)
        log(f"articulation ready; dof order = {list(art.dof_names)}")
        log(f"driven (back) wheel dof indices {dict(zip(DRIVEN, wheel_idx.tolist()))}")
    except Exception as e:  # noqa: BLE001 - keep publishing even if drive init fails
        log(f"WARNING: articulation init failed ({e!r}); publishing only, no driving")

    # 5b) Resolve the drive mode. Keyboard needs the GUI; fall back if headless.
    keyboard = None
    if args.keyboard and args.headless:
        log("WARNING: --keyboard needs the GUI; ignoring it under --headless")
    elif args.keyboard:
        keyboard = KeyboardTeleop(args.lin_step, args.ang_step)
        keyboard.setup()
    mode = ("keyboard (arrow keys)" if keyboard else
            "teleop (/cmd_vel)" if args.teleop else "demo patrol")

    # 6) Hold the back wheels still, then warm up the render/ROS pipeline before
    #    the controller takes over.
    current = np.zeros(len(DRIVEN), dtype=np.float32)  # ramped back-wheel targets (rad/s)
    if wheel_idx is not None:
        # KILL THE STARTUP LURCH. The back joints carry a baked drive
        # targetVelocity=100 rad/s from the original robot; left alone it shoves the
        # base forward at startup. We override it at the CONTROL level only (the same
        # set_joint_velocity_targets API we drive with — NOT a physics-prop edit):
        # command zero and step until the wheels have actually stopped, then proceed.
        zero = current.reshape(1, -1)
        try:
            for _ in range(120):
                art.set_joint_velocity_targets(zero, joint_indices=wheel_idx)
                sim.step(render=False)
            vmax = float(np.max(np.abs(np.asarray(art.get_joint_velocities())[0][wheel_idx])))
            log(f"startup target neutralized; back-wheel speed settled to {vmax:.3f} rad/s")
        except Exception as e:  # noqa: BLE001
            log(f"WARNING: startup-settle skipped ({e!r})")
    for _ in range(max(0, args.warmup)):
        sim.step(render=True)

    log(f"driving mode: {mode}")
    log("publishing — Ctrl-C or close the window to stop")
    max_delta = max(1e-3, args.accel) * render_dt   # per-frame slew limit (rad/s)
    t = 0.0

    # 7) Main loop: pick a body twist (v, w), convert to back-wheel targets, SLEW
    #    them (smooth), write them (single authoritative writer), then step+publish.
    while simulation_app.is_running():
        # Keep the run LIVE: if the timeline ever stops (e.g. it reaches the stage's
        # end time and PhysX tears down the simulation view), resume play and re-grab
        # the articulation handles so publishers + driving continue.
        if not sim.is_playing():
            try:
                sim.play()
                art.initialize()
                wheel_idx = np.array([art.get_dof_index(n) for n in DRIVEN], dtype=np.int32)
                current = np.zeros(len(DRIVEN), dtype=np.float32)
                log("timeline had stopped — resumed play() and re-initialized")
            except Exception as e:  # noqa: BLE001
                carb.log_warn(f"[drive] resume failed: {e!r}")
        if wheel_idx is not None:
            if keyboard is not None:
                v, w = keyboard.v, keyboard.w
                if v == 0.0 and w == 0.0:   # no arrow key held -> accept /cmd_vel too
                    v, w = read_cmd_vel()    # arrows AND the /cmd_vel subscriber both drive
            elif args.teleop:
                v, w = read_cmd_vel()
            else:
                v, w = demo_twist(t, args.lin_step, args.ang_step)
            desired = diff_drive(v, w, args.wheel_radius, args.track_width,
                                 reverse=args.reverse, flip_turn=args.flip_turn)
            if motor_free_speed is not None:   # a TorqueNADO can't spin past free speed
                desired = np.clip(desired, -motor_free_speed, motor_free_speed)
            current = slew(current, desired, max_delta)   # ramp -> no jerk
            try:
                art.set_joint_velocity_targets(current.reshape(1, -1), joint_indices=wheel_idx)
            except Exception as e:  # noqa: BLE001
                carb.log_warn(f"[drive] set velocity failed: {e!r}")
        if keyboard is not None:
            keyboard.update_ui()   # keep the status window live every frame
        sim.step(render=True)
        t += render_dt

    if keyboard is not None:
        keyboard.teardown()
    sim.stop()
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    except BaseException as exc:  # noqa: BLE001 - fastShutdown can eat stdout tracebacks
        import traceback
        tb = traceback.format_exc()
        log("FATAL: " + repr(exc))
        print(tb, flush=True)
        try:
            errf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "robot_drive_and_publish.error.log")
            with open(errf, "w") as fh:
                fh.write(tb)
            log("traceback written to " + errf)
        except Exception:
            pass
    finally:
        simulation_app.close()
    sys.exit(rc)
