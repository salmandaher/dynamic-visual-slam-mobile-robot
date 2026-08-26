#!/usr/bin/env python3
"""Synthetic Data Generation (SDG) for training an isaac_ros_centerpose model of the socket.

Uses Omniverse Replicator INSIDE Isaac Sim 5.1.0 (standalone, NO Docker) to render the procedural
socket (config.SOCKET_USD_PATH) from many randomized viewpoints with domain randomization, writing
images + the per-frame 6-DoF pose / 3D-cuboid ground truth CenterPose training needs.

    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/sdg_centerpose.py \
        --num-frames 2000 --out /media/salman/DataDrive1/new_arm_isaac/sdg/socket

Output goes to DataDrive1 (system disk is full). This is the DATA step; training itself is done
separately with NVIDIA TAO (needs Docker) — see CENTERPOSE_RUNBOOK.md. CenterPose is category-level,
so for one specific socket a few thousand DR frames is a reasonable start; scale up if mAP is low.

NOTE: this is a scaffold. The exact CenterPose label schema TAO expects (the keypoints/cuboid JSON,
the known 'AR_data' friction) is documented in the runbook; this script writes the standard
Replicator BasicWriter outputs (rgb + camera_params + bounding_box_3d + semantics) plus a per-frame
cuboid dump, which the runbook's converter step massages into the CenterPose format.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402

parser = argparse.ArgumentParser(description="Replicator SDG for the socket (CenterPose training)")
parser.add_argument("--num-frames", type=int, default=500)
parser.add_argument("--out", default=os.path.join(config.DATA_DIR, "sdg", "socket"))
parser.add_argument("--socket-usd", default=config.SOCKET_USD_PATH)
parser.add_argument("--res", type=int, nargs=2, default=list(config.CAMERA_RESOLUTION),
                    help="render WxH; default matches the wrist camera (portrait)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
os.makedirs(args.out, exist_ok=True)
simulation_app = SimulationApp({"headless": True})

import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402


def log(m):
    print(f"[sdg] {m}", flush=True)


def main():
    if not os.path.exists(args.socket_usd):
        log(f"ERROR: socket USD not found: {args.socket_usd}; run add_socket_plug.py first.")
        return 1

    # Reference the socket into the stage and (re)assert its CenterPose semantic class so the
    # Replicator annotators label it (the pure-pxr bake could not set Semantics standalone).
    add_reference_to_stage(usd_path=args.socket_usd, prim_path=config.SOCKET_PRIM)
    socket = rep.get.prim_at_path(config.SOCKET_PRIM)
    with socket:
        rep.modify.semantics([("class", config.SOCKET_SEMANTIC_CLASS)])

    # A camera matching the wrist webcam intrinsics (so the trained model matches deployment).
    cam = rep.create.camera(
        focal_length=config.CAMERA_FOCAL_LENGTH,
        horizontal_aperture=config.CAMERA_HORIZONTAL_APERTURE,
        clipping_range=tuple(config.CAMERA_CLIPPING_RANGE),
    )
    rp = rep.create.render_product(cam, tuple(args.res))

    # Dome-light + background randomization for domain randomization.
    dome = rep.create.light(light_type="Dome")

    with rep.trigger.on_frame(num_frames=args.num_frames):
        # Randomize camera pose on a shell around the socket (radius/elevation/azimuth), always
        # looking at the socket face — mimics the wrist camera approaching from varied angles.
        with cam:
            rep.modify.pose(
                position=rep.distribution.uniform((0.05, -0.20, 0.05), (0.45, 0.20, 0.45)),
                look_at=config.SOCKET_PRIM,
            )
        with dome:
            rep.modify.attribute("inputs:intensity", rep.distribution.uniform(300, 2000))
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        with socket:
            # small in-plane jitter of the socket so pose is not trivially constant
            rep.modify.pose(
                rotation=rep.distribution.uniform((-8, -8, -8), (8, 8, 8)))

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=args.out,
        rgb=True,
        bounding_box_3d=True,         # 3D cuboid (CenterPose target)
        camera_params=True,          # intrinsics/extrinsics per frame
        semantic_segmentation=True,
    )
    writer.attach([rp])

    log(f"rendering {args.num_frames} frames @ {args.res} -> {args.out}")
    for i in range(args.num_frames):
        rep.orchestrator.step()
        if (i + 1) % 50 == 0:
            log(f"  {i + 1}/{args.num_frames}")
    rep.orchestrator.wait_until_complete()
    log(f"done. dataset at {args.out}. Convert to CenterPose format + train per CENTERPOSE_RUNBOOK.md")
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    finally:
        simulation_app.close()
    sys.exit(rc)
