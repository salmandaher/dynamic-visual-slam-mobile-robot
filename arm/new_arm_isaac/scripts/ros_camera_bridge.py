#!/usr/bin/env python3
"""Stream the wrist camera (RGB + camera_info) to ROS 2 so a perception node (isaac_ros_centerpose)
can consume it, with the socket in the scene. Inference-time companion to socket_demo.py.

Loads the arm (wrist camera + plug) + the socket, enables the Isaac Sim ROS 2 bridge, and builds an
OmniGraph that publishes the wrist camera. The heavy frame copy happens on the C++ side (NOT in a
Python rclpy loop), so the sim stays real-time. CenterPose then subscribes to /rgb + /camera_info.

    # ROS 2 Humble sourced in the shell first:  source /opt/ros/humble/setup.bash
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/ros_camera_bridge.py --headless
    # verify in another terminal (plain Humble, no Isaac ROS needed):
    #   ros2 topic hz /rgb ; ros2 topic echo /camera_info --once ; ros2 run rqt_image_view rqt_image_view

This publishing path needs NO Isaac ROS / Docker / TensorRT — only Isaac's own isaacsim.ros2.bridge
and ROS 2 Humble. isaac_ros_centerpose (which DOES need the Isaac ROS container) is a separate
subscriber; see CENTERPOSE_RUNBOOK.md.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402

parser = argparse.ArgumentParser(description="Publish the wrist camera to ROS 2 (for CenterPose)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--rgb-topic", default="/rgb")
parser.add_argument("--info-topic", default="/camera_info")
parser.add_argument("--frame-id", default="wrist_cam")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
simulation_app = SimulationApp({"headless": args.headless})

import omni.graph.core as og  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402


def log(m):
    print(f"[ros_cam] {m}", flush=True)


def main():
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=config.USD_PATH, prim_path=config.ROBOT_PRIM)
    if os.path.exists(config.SOCKET_USD_PATH):
        add_reference_to_stage(usd_path=config.SOCKET_USD_PATH, prim_path=config.SOCKET_PRIM)
    world.reset()

    cam_prim = f"{config.ROBOT_PRIM}/{config.CAMERA_PARENT_LINK}/{config.CAMERA_PRIM_NAME}"
    w, h = config.CAMERA_RESOLUTION

    # OmniGraph: OnPlaybackTick -> CreateRenderProduct(cam) -> ROS2CameraHelper(rgb) +
    # ROS2CameraInfoHelper, plus a /clock publisher. Node type ids are the Isaac Sim 5.1.0
    # isaacsim.* identifiers. (No IsaacRunOneSimulationFrame: that node id does NOT exist in
    # Isaac Sim 5.1.0 -> OmniGraphError; OnPlaybackTick already fires once per rendered frame and
    # each ROS2CameraHelper gates its own publish rate via an internal IsaacSimulationGate.)
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/CameraGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [cam_prim]),
                ("RenderProduct.inputs:width", w),
                ("RenderProduct.inputs:height", h),
                ("CameraHelperRgb.inputs:type", "rgb"),
                ("CameraHelperRgb.inputs:topicName", args.rgb_topic),
                ("CameraHelperRgb.inputs:frameId", args.frame_id),
                ("CameraHelperInfo.inputs:topicName", args.info_topic),
                ("CameraHelperInfo.inputs:frameId", args.frame_id),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelperRgb.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelperInfo.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelperRgb.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelperInfo.inputs:renderProductPath"),
                ("OnTick.outputs:tick", "Clock.inputs:execIn"),
                ("ReadTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ],
        },
    )
    log(f"publishing camera {cam_prim} -> {args.rgb_topic} (+ {args.info_topic}), frame '{args.frame_id}'")
    log("verify: `ros2 topic hz /rgb`, `ros2 topic echo /camera_info --once`")

    while simulation_app.is_running():
        world.step(render=True)
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    finally:
        simulation_app.close()
    sys.exit(rc)
