#!/usr/bin/env python3
"""Headless RTX render of the imported new_arm USD to a PNG (so you can SEE it).

Loads the converted USD (scripts/import_urdf.py output), poses the arm, frames it
with a 3/4 camera, renders a few frames so the raytracer converges, and writes a PNG
to DataDrive1 (the system disk is full). No GUI required.

    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    PYTHONUNBUFFERED=1 ./python.sh \
        /home/salman/Documents/our_work/new_arm_isaac/scripts/render_usd.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac import kinematics as kin  # noqa: E402

OUT_DIR = os.path.join(config.DATA_DIR, "renders")

# ---- SimulationApp first, with an offscreen RTX renderer ----
from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
os.makedirs(OUT_DIR, exist_ok=True)
simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
    "renderer": "RaytracedLighting",
})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
import isaacsim.core.utils.prims as prim_utils  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file  # noqa: E402


def log(m):
    print(f"[render] {m}", flush=True)


# Camera views: (filename, eye, target). Target ~ middle of the arm.
VIEWS = [
    ("arm_3quarter.png", (0.45, -0.42, 0.40), (0.12, 0.0, 0.13)),
    ("arm_side.png",     (0.02, -0.55, 0.18), (0.12, 0.0, 0.13)),
]

# Pose to display (actuated, radians) — "ready" so it's visibly articulated.
POSE = config.NAMED_POSES["ready"]


def render():
    if not os.path.exists(config.USD_PATH):
        log(f"ERROR USD missing: {config.USD_PATH}"); return 1

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=config.USD_PATH, prim_path=config.ROBOT_PRIM)

    # A dome light so the arm is well lit regardless of the default scene lighting.
    prim_utils.create_prim(
        "/World/DomeLight", "DomeLight",
        attributes={"inputs:intensity": 1500.0, "inputs:texture:format": "latlong"},
    )

    robot = SingleArticulation(prim_path=config.ROBOT_PRIM, name="new_arm")
    world.scene.add(robot)
    world.reset()

    dof = list(robot.dof_names)
    log(f"dof_names = {dof}")
    act_idx = [dof.index(j) for j in config.ACTUATED_JOINTS]
    pas_idx = dof.index(config.PASSIVE_JOINT)

    target = np.array(robot.get_joint_positions(), dtype=float)
    for k, idx in enumerate(act_idx):
        target[idx] = POSE[k]
    target[pas_idx] = kin.passive_wrist(POSE[1], POSE[2])

    # settle the arm into the commanded pose
    for _ in range(120):
        robot.apply_action(ArticulationAction(joint_positions=target))
        world.step(render=True)
    meas = np.array(robot.get_joint_positions(), dtype=float)
    log(f"posed actuated={np.round([meas[i] for i in act_idx],3)} wrist={meas[pas_idx]:.3f}")

    vp = get_active_viewport()
    for name, eye, tgt in VIEWS:
        set_camera_view(eye=list(eye), target=list(tgt))
        # let the raytracer accumulate a clean frame
        for _ in range(60):
            world.step(render=True)
        path = os.path.join(OUT_DIR, name)
        capture_viewport_to_file(vp, path)
        # pump updates so the async capture flushes to disk
        for _ in range(60):
            simulation_app.update()
        ok = os.path.exists(path)
        log(f"wrote {path}: {ok} ({os.path.getsize(path) if ok else 0} bytes)")

    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = render() or 0
    finally:
        simulation_app.close()
    sys.exit(rc)
