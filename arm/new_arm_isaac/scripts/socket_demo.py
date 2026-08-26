#!/usr/bin/env python3
"""Socket + plug demo: load the arm (wrist camera + plug baked in) and the socket, then drive the
analytic 3-DOF IK to approach the socket while the eye-in-hand camera watches it.

This is the STATIC perception-scene stage 0: it validates that the socket is reachable, the plug
is on the gripper, and the wrist camera sees the socket along the approach (the input the
isaac_ros_centerpose pipeline will consume once the model is trained — see CENTERPOSE_RUNBOOK.md).

GUI on your screen (the docked "EE Camera" panel shows the wrist feed):
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/socket_demo.py
Headless (no window):  add --headless   |  one approach pass then hold:  default loops in GUI.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac import kinematics as kin  # noqa: E402

parser = argparse.ArgumentParser(description="new_arm socket + plug approach demo")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--shot", default=None, help="headless: save the wrist-cam PNG at the final pose, then exit")
parser.add_argument("--shots", default=None, help="headless: save a wrist-cam frame SEQUENCE (approach->seated) to this DIR")
parser.add_argument("--shots_every", type=int, default=8, help="save a sequence frame every N steps")
parser.add_argument("--vision", action="store_true", help="run the wrist-cam vision-as-force-surrogate detector live each step and log alignment/seated")
parser.add_argument("--vision_every", type=int, default=8, help="run the detector every N steps")
parser.add_argument("--hold", type=int, default=150, help="sim steps to hold each waypoint")
parser.add_argument("--view_camera", action="store_true", default=True)
parser.add_argument("--no_view_camera", dest="view_camera", action="store_false")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

# Approach waypoints, COMPUTED so the PLUG TIP (not the wrist) reaches the socket front face.
# The wrist target = socket_front_x - plug_tip_x (EE is kept level by the passive coupling, so the
# plug points horizontally along +X). All targets stay inside the 0.266 m wrist reach.
def _approach_waypoints():
    tip_x = config.PLUG_OFFSET_XYZ[0] + config.PLUG_BODY_LENGTH + config.PLUG_PRONG_LENGTH
    front_x = config.SOCKET_POSE_XYZ[0] - config.SOCKET_FACEPLATE_HALF[0]
    sy, sz = config.SOCKET_POSE_XYZ[1], config.SOCKET_POSE_XYZ[2]
    wx = front_x - tip_x  # wrist x so the plug tip lands at the socket front face
    # Approach from above -> descend to hole height -> push forward in (no overshoot through
    # the socket): keep the plug tip a touch behind the face until the final push.
    return [
        ("home", None),
        ("lift", (wx - 0.02, sy, sz + 0.10)),
        ("descend", (wx - 0.02, sy, sz)),
        ("insert", (wx, sy, sz)),
    ]


WAYPOINTS = _approach_waypoints()


def log(m):
    print(f"[socket_demo] {m}", flush=True)


def main():
    if not os.path.exists(config.USD_PATH):
        log(f"ERROR: robot USD not found: {config.USD_PATH}; run import_urdf.py first.")
        return
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_reference_to_stage(usd_path=config.USD_PATH, prim_path=config.ROBOT_PRIM)
    if os.path.exists(config.SOCKET_USD_PATH):
        add_reference_to_stage(usd_path=config.SOCKET_USD_PATH, prim_path=config.SOCKET_PRIM)
        log(f"socket referenced at {config.SOCKET_PRIM} from {config.SOCKET_USD_PATH}")
    else:
        log(f"WARNING: {config.SOCKET_USD_PATH} missing; run add_socket_plug.py (no socket shown)")

    robot = SingleArticulation(prim_path=config.ROBOT_PRIM, name="new_arm")
    world.scene.add(robot)
    world.reset()

    dof = list(robot.dof_names)
    act_idx = [dof.index(j) for j in config.ACTUATED_JOINTS]
    pas_idx = dof.index(config.PASSIVE_JOINT)
    log(f"dof_names = {dof}")

    if args.view_camera and not args.headless:
        try:
            from omni.kit.viewport.utility import create_viewport_window
            cw, ch = config.CAMERA_RESOLUTION
            sc = 480.0 / max(cw, ch)
            win = create_viewport_window("EE Camera", width=max(1, int(cw * sc)), height=max(1, int(ch * sc)))
            win.viewport_api.camera_path = (
                f"{config.ROBOT_PRIM}/{config.CAMERA_PARENT_LINK}/{config.CAMERA_PRIM_NAME}")
            log("docked 'EE Camera' viewport bound to the wrist camera")
        except Exception as e:  # noqa: BLE001
            log(f"could not open camera viewport ({e}); use Viewport > Cameras menu")

    # wrist-cam: needed for sequence capture (--shots) and/or the live vision detector (--vision)
    shot_cam = None
    if args.shots or args.vision:
        cam_prim = f"{config.ROBOT_PRIM}/{config.CAMERA_PARENT_LINK}/{config.CAMERA_PRIM_NAME}"
        try:
            from isaacsim.sensors.camera import Camera
        except Exception:
            from omni.isaac.sensor import Camera  # older namespace fallback
        shot_cam = Camera(prim_path=cam_prim, resolution=tuple(config.CAMERA_RESOLUTION))
        shot_cam.initialize()
        if args.shots:
            os.makedirs(args.shots, exist_ok=True)
        for _ in range(20):
            world.step(render=True)  # warm up the render product
        from PIL import Image as _Image
    detect_alignment = None
    if args.vision:
        from new_arm_isaac.vision_align import detect_alignment

    full = np.array(robot.get_joint_positions(), dtype=float)
    gstep = 0
    for name, tgt in WAYPOINTS:
        if tgt is None:
            thetas = [0.0, 0.0, 0.0]
        else:
            q, ok = kin.inverse_kinematics(tgt, elbow_up=True)
            thetas = list(q)
            err = np.linalg.norm(kin.forward_kinematics(q) - np.array(tgt)) * 1000.0
            log(f"{name} {tgt} -> q {np.round(q, 3)}  ok={ok}  FKerr {err:.2f} mm")
        passive = kin.passive_wrist(thetas[1], thetas[2])
        for k, idx in enumerate(act_idx):
            full[idx] = thetas[k]
        full[pas_idx] = passive
        for _ in range(args.hold):
            robot.apply_action(ArticulationAction(joint_positions=full))
            world.step(render=(shot_cam is not None) or (not args.headless))
            save_now = args.shots and gstep % args.shots_every == 0
            vis_now = args.vision and gstep % args.vision_every == 0
            if shot_cam is not None and (save_now or vis_now):
                rgba = np.asarray(shot_cam.get_rgba())
                if rgba.size:
                    rgb = (rgba[:, :, :3] * (255 if rgba.max() <= 1.0 else 1)).astype("uint8")
                    if save_now:
                        _Image.fromarray(rgb).save(
                            os.path.join(args.shots, f"frame_{gstep:04d}_{name}.png"))
                    if vis_now and detect_alignment is not None:
                        r = detect_alignment(rgb)
                        lp = f"{r['lateral_px']:.0f}px" if r["lateral_px"] is not None else "  --"
                        log(f"[vision] {name:11s} step {gstep:4d}: holes={len(r['holes'])} "
                            f"docked={r['docked_holes']}/{len(r['holes'])} lateral={lp:>5s} "
                            f"aligned={int(r['aligned'])} SEATED={int(r['seated'])}")
            gstep += 1
            if not simulation_app.is_running():
                return
    log("approach complete")
    if args.shots:
        log(f"saved wrist-cam sequence to {args.shots}")
        return

    if args.shot:
        cam_prim = f"{config.ROBOT_PRIM}/{config.CAMERA_PARENT_LINK}/{config.CAMERA_PRIM_NAME}"
        try:
            from isaacsim.sensors.camera import Camera
        except Exception:
            from omni.isaac.sensor import Camera  # older namespace fallback
        import numpy as _np
        cam = Camera(prim_path=cam_prim, resolution=tuple(config.CAMERA_RESOLUTION))
        cam.initialize()
        for _ in range(40):
            world.step(render=True)
        rgba = cam.get_rgba()
        rgb = (_np.asarray(rgba)[:, :, :3] * (255 if _np.asarray(rgba).max() <= 1.0 else 1)).astype("uint8")
        os.makedirs(os.path.dirname(args.shot), exist_ok=True)
        try:
            from PIL import Image
            Image.fromarray(rgb).save(args.shot)
        except Exception:
            _np.save(args.shot + ".npy", rgb)
        log(f"saved wrist-cam shot {args.shot} shape={rgb.shape} nonzero={(rgb > 0).sum()}")
        return

    log("holding at the socket (close the window to stop)")
    while simulation_app.is_running() and not args.headless:
        world.step(render=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
