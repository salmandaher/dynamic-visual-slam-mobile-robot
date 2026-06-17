#!/usr/bin/env python3
"""Publish the robot-base Intel RealSense D435i (COLOR + DEPTH) to ROS 2 from Isaac Sim 5.1.0.

Opens the finished mobile-base scene (new_built_robot.usd — robot + welded d435i_camera +
GroundPlane + PhysicsScene) and builds ONE OmniGraph that streams the two on-board cameras to
ROS 2. The heavy frame copy happens on the C++ side of isaacsim.ros2.bridge (NOT in a Python
rclpy loop), so the sim stays real-time. Mirrors the proven single-camera bridge in
new_arm_isaac/scripts/ros_camera_bridge.py, doubled for the D435i's two sensors, and uses the
SimulationContext load/play lifecycle from NVIDIA's own standalone_examples/.../carter_stereo.py.

Published (defaults; all overridable — see --help):
    /camera/color/image_raw        sensor_msgs/Image      frame camera_color_optical_frame
    /camera/color/camera_info      sensor_msgs/CameraInfo
    /camera/depth/image_rect_raw   sensor_msgs/Image      frame camera_depth_optical_frame
    /camera/depth/camera_info      sensor_msgs/CameraInfo
    /camera/depth/points           sensor_msgs/PointCloud2  (only with --pointcloud)
    /clock                         rosgraph_msgs/Clock

NOTE on depth encoding: Isaac's ROS2CameraHelper type="depth" emits 32FC1 (metres). The real
realsense2_camera driver publishes 16UC1 (millimetres) on .../image_rect_raw. Consumers that
assume mm must scale; most perception stacks accept 32FC1 metres directly.

RUN (ROS 2 Humble must be sourced first so the bridge links the system rclcpp):
    source /opt/ros/humble/setup.bash
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/simsim/robot_base_camera_ros.py --headless

VERIFY (another terminal, plain Humble — no Isaac ROS / Docker needed):
    ros2 topic list | grep camera
    ros2 topic hz /camera/color/image_raw
    ros2 topic echo /camera/color/camera_info --once
    ros2 run rqt_image_view rqt_image_view        # pick the color/depth image topics
    rviz2                                          # add Image + (with --pointcloud) PointCloud2

CONFIRM the camera prim paths (base python3 + usd-core; Usd.Stage.Open SEGFAULTS on this crate,
so use Sdf.Layer):
    python3 -c "from pxr import Sdf; print(Sdf.Layer.FindOrOpen('/home/salman/Documents/simsim/new_built_robot.usd').ExportToString())" | grep -i camera
"""

import argparse
import os
import sys

# --- defaults ---------------------------------------------------------------
DEFAULT_SCENE = "/home/salman/Documents/simsim/new_built_robot.usd"
# Composed-stage camera prim paths: d435i.usd (defaultPrim "d435i") is referenced at
# /World/robot/d435i_camera, so its cameras compose under camera_link. Confirmed in
# d435i.usd + preview_d435i.py.
COLOR_PRIM = "/World/robot/d435i_camera/camera_link/color_camera"
DEPTH_PRIM = "/World/robot/d435i_camera/camera_link/depth_camera"

parser = argparse.ArgumentParser(
    description="Publish the robot-base D435i color+depth to ROS 2 (Isaac Sim 5.1.0)")
parser.add_argument("--headless", action="store_true",
                    help="run without the Kit GUI (cameras still render off-screen)")
parser.add_argument("--scene", default=DEFAULT_SCENE, help="USD scene to open")
parser.add_argument("--namespace", default="camera",
                    help="ROS 2 node namespace, prepended to every topic (default 'camera')")
# topic names are RELATIVE to --namespace (e.g. ns=camera + color/image_raw -> /camera/color/image_raw)
parser.add_argument("--color-topic", default="color/image_raw")
parser.add_argument("--color-info-topic", default="color/camera_info")
parser.add_argument("--depth-topic", default="depth/image_rect_raw")
parser.add_argument("--depth-info-topic", default="depth/camera_info")
parser.add_argument("--pcl-topic", default="depth/points")
parser.add_argument("--pointcloud", action="store_true",
                    help="also publish a PointCloud2 from the depth camera (type=depth_pcl)")
# frame ids are REP-103 optical frames (z-forward, x-right, y-down); NOT namespace-prefixed
parser.add_argument("--color-frame", default="camera_color_optical_frame")
parser.add_argument("--depth-frame", default="camera_depth_optical_frame")
# D435i native stream sizes (overridable). Color up to 1920x1080; depth optimal 848x480.
parser.add_argument("--color-width", type=int, default=1280)
parser.add_argument("--color-height", type=int, default=720)
parser.add_argument("--depth-width", type=int, default=848)
parser.add_argument("--depth-height", type=int, default=480)
parser.add_argument("--warmup", type=int, default=8,
                    help="render steps to settle the pipeline before the main loop")
parser.add_argument("--no-opencv-calib", action="store_true",
                    help="skip authoring OpenCV-pinhole intrinsics; let Isaac derive camera_info "
                         "from the lens apertures (re-introduces the harmless 'Forcing fy to fx' warning)")
args, _ = parser.parse_known_args()

if not os.path.isfile(args.scene):
    sys.exit(f"[base_cam] scene not found: {args.scene}")

# --- Isaac Sim must boot before any omni / isaacsim import ------------------
from isaacsim import SimulationApp  # noqa: E402

# RaytracedLighting matches carter_stereo.py; headless still renders the render products.
simulation_app = SimulationApp({"headless": args.headless, "renderer": "RaytracedLighting"})

import carb  # noqa: E402
import omni  # noqa: E402
import omni.usd  # noqa: E402
import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402


def log(m):
    print(f"[base_cam] {m}", flush=True)


def author_opencv_pinhole(stage, cam_path, width, height):
    """Author an OpenCV-pinhole lens-distortion model on the camera so ROS2CameraInfoHelper
    reports exact, square (fx == fy) intrinsics with a zero-distortion plumb_bob model. This
    clears Isaac's two camera_info warnings: 'Forcing fy to fx' (an exact float compare on the
    independent h/v apertures) and 'Unsupported physical distortion model None'. fx is derived
    from the lens (width * focalLength / horizontalAperture) so the horizontal FOV is preserved;
    fy is set EXACTLY equal to fx (square pixels). imageSize tracks the chosen render resolution,
    so camera_info stays correct for whatever --*-width/height is used. (See Isaac
    isaacsim.ros2.bridge/impl/camera_info_utils.py read_camera_info: the opencvPinhole branch.)"""
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
        P + "fy": (Sdf.ValueTypeNames.Float, fx),  # == fx -> no 'Forcing fy to fx' warning
    }
    # k4,k5,k6 == 0 -> read_camera_info publishes plumb_bob with d = [k1,k2,p1,p2,k3] = zeros.
    for c in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"):
        vals[P + c] = (Sdf.ValueTypeNames.Float, 0.0)
    for name, (vtype, val) in vals.items():
        attr = prim.GetAttribute(name)
        if not attr:
            attr = prim.CreateAttribute(name, vtype)
        attr.Set(val)
    return fx


def main():
    # 1) ROS 2 bridge on, then let the extension settle.
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    # 2) Open the existing complete scene (do NOT re-add ground/robot) and wait for it to load.
    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update()
    simulation_app.update()
    log(f"loading {args.scene} ...")
    while is_stage_loading():
        simulation_app.update()
    log("stage loaded")

    stage = omni.usd.get_context().get_stage()
    for p in (COLOR_PRIM, DEPTH_PRIM):
        if not stage.GetPrimAtPath(p).IsValid():
            carb.log_error(f"[base_cam] camera prim missing: {p}")
            log(f"WARNING: {p} not found in stage — published frames would be blank")

    # 2b) Author exact OpenCV-pinhole intrinsics (square fx==fy, zero distortion) so camera_info
    #     is precise and Isaac emits no fy!=fx / unsupported-distortion warnings.
    if not args.no_opencv_calib:
        try:
            fxc = author_opencv_pinhole(stage, COLOR_PRIM, args.color_width, args.color_height)
            fxd = author_opencv_pinhole(stage, DEPTH_PRIM, args.depth_width, args.depth_height)
            log(f"OpenCV-pinhole intrinsics authored: color fx=fy={fxc:.3f}, depth fx=fy={fxd:.3f}")
        except Exception as e:  # noqa: BLE001 - never let calibration block publishing
            log(f"WARNING: could not author OpenCV-pinhole intrinsics ({e!r}); "
                "camera_info will fall back to lens-derived values")

    # 3) SimulationContext over the opened stage (scene metersPerUnit == 1.0).
    log("creating SimulationContext ...")
    sim = SimulationContext(stage_units_in_meters=1.0)
    log("SimulationContext ready")

    # 4) Build the publishing OmniGraph. One render product per camera (different FOV/resolution),
    #    each feeding its OWN image + camera_info helper — no cross-wiring. Node type ids and every
    #    inputs:/outputs: attribute are verified against the on-disk Isaac Sim 5.1.0 .ogn schemas.
    keys = og.Controller.Keys

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
    ]
    set_values = [
        # cameraPrim is a USD relationship target -> pass as a list.
        ("RpColor.inputs:cameraPrim", [COLOR_PRIM]),
        ("RpColor.inputs:width", args.color_width),
        ("RpColor.inputs:height", args.color_height),
        ("RpDepth.inputs:cameraPrim", [DEPTH_PRIM]),
        ("RpDepth.inputs:width", args.depth_width),
        ("RpDepth.inputs:height", args.depth_height),
        ("ColorImg.inputs:type", "rgb"),
        ("ColorImg.inputs:topicName", args.color_topic),
        ("ColorImg.inputs:frameId", args.color_frame),
        ("ColorImg.inputs:nodeNamespace", args.namespace),
        ("ColorInfo.inputs:topicName", args.color_info_topic),
        ("ColorInfo.inputs:frameId", args.color_frame),
        ("ColorInfo.inputs:nodeNamespace", args.namespace),
        ("DepthImg.inputs:type", "depth"),
        ("DepthImg.inputs:topicName", args.depth_topic),
        ("DepthImg.inputs:frameId", args.depth_frame),
        ("DepthImg.inputs:nodeNamespace", args.namespace),
        ("DepthInfo.inputs:topicName", args.depth_info_topic),
        ("DepthInfo.inputs:frameId", args.depth_frame),
        ("DepthInfo.inputs:nodeNamespace", args.namespace),
    ]
    connect = [
        # OnPlaybackTick fires once per rendered frame -> (re)create both render products.
        # (No IsaacRunOneSimulationFrame: that node id does not exist in Isaac Sim 5.1.0; the
        # per-frame publish gating is handled internally by each ROS2CameraHelper's own
        # IsaacSimulationGate, per the camera_periodic.py reference.)
        ("OnTick.outputs:tick", "RpColor.inputs:execIn"),
        ("OnTick.outputs:tick", "RpDepth.inputs:execIn"),
        # /clock
        ("OnTick.outputs:tick", "Clock.inputs:execIn"),
        ("ReadTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
        # color render product -> color helpers (exec + render product path)
        ("RpColor.outputs:execOut", "ColorImg.inputs:execIn"),
        ("RpColor.outputs:execOut", "ColorInfo.inputs:execIn"),
        ("RpColor.outputs:renderProductPath", "ColorImg.inputs:renderProductPath"),
        ("RpColor.outputs:renderProductPath", "ColorInfo.inputs:renderProductPath"),
        # depth render product -> depth helpers
        ("RpDepth.outputs:execOut", "DepthImg.inputs:execIn"),
        ("RpDepth.outputs:execOut", "DepthInfo.inputs:execIn"),
        ("RpDepth.outputs:renderProductPath", "DepthImg.inputs:renderProductPath"),
        ("RpDepth.outputs:renderProductPath", "DepthInfo.inputs:renderProductPath"),
    ]

    # Optional depth point cloud (PointCloud2) off the SAME depth render product.
    if args.pointcloud:
        create_nodes.append(("DepthPcl", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        set_values += [
            ("DepthPcl.inputs:type", "depth_pcl"),
            ("DepthPcl.inputs:topicName", args.pcl_topic),
            ("DepthPcl.inputs:frameId", args.depth_frame),
            ("DepthPcl.inputs:nodeNamespace", args.namespace),
        ]
        connect += [
            ("RpDepth.outputs:execOut", "DepthPcl.inputs:execIn"),
            ("RpDepth.outputs:renderProductPath", "DepthPcl.inputs:renderProductPath"),
        ]

    log("building OmniGraph ...")
    og.Controller.edit(
        {"graph_path": "/RobotBaseCameraGraph", "evaluator_name": "execution"},
        {keys.CREATE_NODES: create_nodes, keys.SET_VALUES: set_values, keys.CONNECT: connect},
    )
    log("OmniGraph built")

    ns = f"/{args.namespace}" if args.namespace else ""
    log(f"color -> {ns}/{args.color_topic} (+ {ns}/{args.color_info_topic}) "
        f"@ {args.color_width}x{args.color_height}, frame '{args.color_frame}'")
    log(f"depth -> {ns}/{args.depth_topic} (+ {ns}/{args.depth_info_topic}) "
        f"@ {args.depth_width}x{args.depth_height}, frame '{args.depth_frame}'"
        + (f"  + pcl {ns}/{args.pcl_topic}" if args.pointcloud else ""))
    log("verify: ros2 topic hz /camera/color/image_raw ; ros2 topic echo /camera/color/camera_info --once")

    # 5) Play the timeline (so OnPlaybackTick fires), warm up, then stream.
    sim.play()
    for _ in range(max(0, args.warmup)):
        sim.step(render=True)
    log("publishing — Ctrl-C or close the window to stop")
    while simulation_app.is_running():
        sim.step(render=True)
    sim.stop()
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    except BaseException as exc:  # noqa: BLE001 - diagnostic: fastShutdown can eat stdout tracebacks
        import traceback
        tb = traceback.format_exc()
        log("FATAL: " + repr(exc))
        print(tb, flush=True)
        # Isaac's default --/app/fastShutdown=True can truncate the stdout traceback, so also
        # persist it next to the script.
        try:
            errf = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "robot_base_camera_ros.error.log")
            with open(errf, "w") as fh:
                fh.write(tb)
            log("traceback written to " + errf)
        except Exception:
            pass
    finally:
        simulation_app.close()
    sys.exit(rc)
