#!/usr/bin/env python3
"""Convert new_arm.urdf -> USD for Isaac Sim 5.1.0 (standalone).

Run with the Isaac python:
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/import_urdf.py

Produces a SELF-CONTAINED USD articulation at config.USD_PATH (on DataDrive1) with a
FIXED base (arm_base_link bolted to the world) and position-driven revolute joints.

Passive wrist: on the real arm end_effector_joint is mechanically coupled, no motor.
The sim/twin uses theta4 = 2*pi - theta2 - theta3 (the firmware's original "- pi/2"
term is intentionally dropped so the wrist is LEVEL (0) at home — see
kinematics.PASSIVE_OFFSET). Sim has no such mechanism, so we KEEP a position drive on
it and always *command* it to the coupled value (computed by the twin / RL each step).
RL still only *acts* on the 3 actuated joints.

IMPORTANT (learned the hard way): do NOT pass dest_path to URDFParseAndImportFile and
THEN call save_as_stage — that writes a multi-file package and the subsequent
save_as_stage overwrites the wrapper with the (empty) viewport stage, leaving a USD
with no defaultPrim and no robot. Instead we follow the canonical
standalone_examples/.../urdf_import.py pattern: import into the CURRENT stage (no
dest_path), edit it live, set the defaultPrim, then save_as_stage exactly once.

Verified against Isaac Sim 5.1.0:
  exts/isaacsim.asset.importer.urdf (URDFCreateImportConfig / URDFParseAndImportFile)
  standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac.usd_camera import bake_ee_camera  # noqa: E402
from new_arm_isaac.usd_socket_plug import bake_plug  # noqa: E402

# --- SimulationApp MUST be created before any other isaac/omni import ---
from isaacsim import SimulationApp  # noqa: E402

config.ensure_dirs()
simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType  # noqa: E402
from pxr import Sdf, UsdPhysics  # noqa: E402

# Position-drive gains baked into the USD (used by the standalone/twin
# SingleArticulation path; the IsaacLab env overrides via ImplicitActuatorCfg).
DRIVE_STIFFNESS = 1.0e7
DRIVE_DAMPING = 1.0e5


def log(msg):
    print(f"[import_urdf] {msg}", flush=True)


def main():
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    # All set by DIRECT ASSIGNMENT — verified against the official test
    # exts/isaacsim.asset.importer.urdf/.../tests/test_urdf.py and the canonical
    # standalone_examples/.../urdf_import.py.
    import_config.fix_base = True                  # arm_base_link is the root -> fixed
    import_config.merge_fixed_joints = False
    import_config.import_inertia_tensor = True
    import_config.distance_scale = 1.0             # URDF already metres (mesh scale 0.001)
    import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_position_drive_damping = DRIVE_DAMPING
    import_config.self_collision = False
    # make_default_prim: importer marks the robot root as the stage defaultPrim, so a
    # later add_reference_to_stage(@USD@<defaultPrim>) resolves. Belt-and-suspenders;
    # we also set it explicitly below.
    try:
        import_config.make_default_prim = True
    except Exception:
        pass

    log(f"importing {config.URDF_PATH}")
    # NOTE: no dest_path -> import lands in the current in-memory stage.
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=config.URDF_PATH,
        import_config=import_config,
        get_articulation_root=True,
    )
    log(f"status={status}  imported articulation prim: {prim_path}")
    if not prim_path:
        log("ERROR: importer returned no prim_path"); return 1

    stage = omni.usd.get_context().get_stage()

    # The stage defaultPrim MUST be a ROOT-LEVEL prim (direct child of the pseudo-root)
    # or USD silently refuses to set it. get_articulation_root returns the nested
    # ArticulationRootAPI prim (e.g. /new_arm/root_joint), so walk up to its top-level
    # ancestor (/new_arm) — the Xform that actually holds every link + joint.
    top_path = "/" + prim_path.strip("/").split("/")[0]
    top_prim = stage.GetPrimAtPath(top_path)
    if top_prim and top_prim.IsValid():
        stage.SetDefaultPrim(top_prim)
        log(f"defaultPrim set to {top_path} (artroot at {prim_path})")
    else:
        log(f"WARNING: top prim {top_path} not valid on stage; defaultPrim not set")

    # Bake the end-effector Logitech HD RGB camera onto end_effector_link so the
    # twin, standalone, and cloned RL envs all inherit it (single source of truth).
    bake_ee_camera(stage, top_path, log=log)
    # Bake the plug onto end_effector_link too (the socket is separate world geometry —
    # see scripts/add_socket_plug.py / socket.usd).
    bake_plug(stage, top_path, log=log)

    # Set angular position-drive stiffness/damping on every revolute joint via the
    # stable pxr UsdPhysics API (the carter standalone example's pattern).
    revolute, applied = [], 0
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsRevoluteJoint":
            revolute.append(prim.GetName())
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(DRIVE_STIFFNESS)
            drive.CreateDampingAttr(DRIVE_DAMPING)
            applied += 1
    log(f"revolute joints: {revolute}")
    log(f"set angular drive (k={DRIVE_STIFFNESS:.0e}, d={DRIVE_DAMPING:.0e}) on {applied} joints")

    # Validate it's a real physics articulation before trusting the asset (mirrors the
    # carter example): play one step, init an Articulation view, check the handle.
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()
    try:
        art = Articulation(prim_path)
        art.initialize()
        dof_names = list(art.dof_names) if art.dof_names is not None else []
        log(f"articulation dof_names = {dof_names}")
        log(f"(verify names match config.ACTUATED_JOINTS={config.ACTUATED_JOINTS} "
            f"+ PASSIVE_JOINT={config.PASSIVE_JOINT})")
    except Exception as e:
        log(f"WARNING: could not introspect articulation view: {e}")
    omni.timeline.get_timeline_interface().stop()

    # Save the now-populated stage exactly once as a self-contained USD.
    ok = omni.usd.get_context().save_as_stage(config.USD_PATH)
    log(f"save_as_stage -> {ok}: {config.USD_PATH}")
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main() or 0
    finally:
        simulation_app.close()
    sys.exit(rc)
