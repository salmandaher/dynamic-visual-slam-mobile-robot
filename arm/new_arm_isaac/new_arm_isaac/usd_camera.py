#!/usr/bin/env python3
"""Bake the end-effector RGB camera (Logitech C920 HD Pro webcam) into a USD stage.

Pure-pxr (no omni / SimulationApp), so it is shared by:
  * scripts/import_urdf.py    — bakes the camera while generating new_arm.usd, and
  * scripts/add_ee_camera.py  — patches an EXISTING new_arm.usd without a full re-import.

pxr is imported lazily INSIDE the function so this module (and the package) still import
without Isaac/USD present, matching the rest of new_arm_isaac.

Geometry / intrinsics all come from new_arm_isaac.config (single source of truth). The
camera is created as a child prim of `<top>/end_effector_link`, so it inherits the EE
link's pose and rigidly follows the gripper through every USD reference / Fabric clone
(the digital twin at /World/NewArm/..., the cloned RL envs at /World/envs/env_*/NewArm/...).

It is a plain RGB camera (UsdGeom.Camera). RGB vs depth is a render-product choice made by
the consumer (e.g. the RL TiledCamera); the baked prim itself is just geometry+intrinsics.
"""

import math

from new_arm_isaac import config


def bake_ee_camera(stage, top_prim_path, log=print):
    """Create (or refresh) the EE camera prim under <top_prim_path>/end_effector_link.

    Idempotent: any pre-existing camera prim at the same path is removed first so repeated
    runs do not stack duplicate xformOps. Returns the camera prim path, or None if the EE
    link prim is missing.
    """
    from pxr import Gf, Sdf, UsdGeom

    ee_path = f"{top_prim_path}/{config.CAMERA_PARENT_LINK}"
    ee = stage.GetPrimAtPath(ee_path)
    if not (ee and ee.IsValid()):
        log(f"[camera] WARNING: EE link prim {ee_path} not found; camera NOT baked")
        return None

    cam_path = f"{ee_path}/{config.CAMERA_PRIM_NAME}"
    # Idempotency: drop a previous camera so AddTranslateOp/AddRotateYOp start fresh.
    if stage.GetPrimAtPath(cam_path).IsValid():
        stage.RemovePrim(Sdf.Path(cam_path))

    cam = UsdGeom.Camera.Define(stage, cam_path)
    xf = UsdGeom.Xformable(cam.GetPrim())
    # IsaacLab's camera pose code REQUIRES the canonical xformOpOrder
    # [translate, orient, scale] and rejects euler ops (e.g. rotateY). So express the
    # orientation as a single quaternion in an orient op. Orientation = RotateY(pitch) *
    # RotateZ(roll): the pitch aims the optical -Z forward+down, then a roll about the
    # (local) optical axis makes the image upright/portrait. q_total = q_pitch * q_roll
    # corresponds to matrix Ry @ Rz. Local transform = Translate * Orient (rotated about
    # its own origin; translate unaffected). Double-precision to match IsaacLab's writes.
    py = math.radians(float(config.CAMERA_PITCH_DEG)) / 2.0
    pz = math.radians(float(getattr(config, "CAMERA_ROLL_DEG", 0.0))) / 2.0
    q_pitch = Gf.Quatd(math.cos(py), Gf.Vec3d(0.0, math.sin(py), 0.0))  # about +Y
    q_roll = Gf.Quatd(math.cos(pz), Gf.Vec3d(0.0, 0.0, math.sin(pz)))   # about +Z (optical)
    quat = q_pitch * q_roll
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(v) for v in config.CAMERA_OFFSET_XYZ]))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))

    cam.CreateFocalLengthAttr(float(config.CAMERA_FOCAL_LENGTH))
    cam.CreateHorizontalApertureAttr(float(config.CAMERA_HORIZONTAL_APERTURE))
    cam.CreateVerticalApertureAttr(float(config.CAMERA_VERTICAL_APERTURE))
    cam.CreateClippingRangeAttr(Gf.Vec2f(*[float(v) for v in config.CAMERA_CLIPPING_RANGE]))
    cam.CreateFocusDistanceAttr(float(config.CAMERA_FOCUS_DISTANCE))
    cam.CreateFStopAttr(0.0)  # pinhole: no depth-of-field blur, only the FOV ratio matters
    cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)

    hfov = 2.0 * math.degrees(math.atan(
        config.CAMERA_HORIZONTAL_APERTURE / (2.0 * config.CAMERA_FOCAL_LENGTH)))
    log(f"[camera] baked {config.CAMERA_MODEL} (RGB) at {cam_path}: xyz "
        f"{config.CAMERA_OFFSET_XYZ} m, pitch {config.CAMERA_PITCH_DEG} / roll "
        f"{getattr(config, 'CAMERA_ROLL_DEG', 0.0)} deg, HFOV {hfov:.1f} deg, "
        f"clip {config.CAMERA_CLIPPING_RANGE} m")
    return cam_path
