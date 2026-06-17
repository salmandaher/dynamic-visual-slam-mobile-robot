#!/usr/bin/env python3
"""Procedural socket (wall outlet) + plug geometry for the pick-and-insert / CenterPose scene.

Pure-pxr (no omni / SimulationApp), shared the same way as usd_camera.py:
  * bake_plug(stage, top_prim_path)  -> rigid child of end_effector_link (baked into new_arm.usd
    by import_urdf.py / scripts/add_socket_plug.py, so it follows the gripper everywhere).
  * bake_socket(stage, prim_path)    -> static world geometry (a faceplate with recessed holes on
    a stand) placed at a wrist-reachable pose; used to build the standalone socket.usd that the
    demo / Replicator SDG load.

pxr is imported lazily inside the functions so the package still imports without Isaac, matching
the rest of new_arm_isaac. All geometry/poses come from new_arm_isaac.config (single source).

The socket carries a `class=socket` semantic label (best-effort) so Omniverse Replicator SDG and
isaac_ros_centerpose have a labelled target. Geometry is deliberately simple but recognisable
(white faceplate, two dark recessed holes; plug = body + two prongs).
"""

import math

from new_arm_isaac import config


def _set_color(gprim, rgb):
    gprim.CreateDisplayColorAttr([tuple(float(c) for c in rgb)])


def _apply_semantics(prim, class_name, log):
    """Best-effort `class=<name>` semantic label (Replicator/CenterPose). Never fatal."""
    try:
        from pxr import Semantics
        sem = Semantics.SemanticsAPI.Apply(prim, "Semantics")
        sem.CreateSemanticTypeAttr().Set("class")
        sem.CreateSemanticDataAttr().Set(class_name)
    except Exception as e:  # noqa: BLE001
        log(f"[socket] note: semantics not applied ({e}); apply via Replicator at SDG time")


def bake_plug(stage, top_prim_path, log=print):
    """Create (or refresh) the plug as a rigid child of <top>/end_effector_link.

    Visual-only (no collider) so the analytic-IK approach is not perturbed by contact; enable
    collision later for a real insertion task. Idempotent. Returns the plug prim path or None.
    """
    from pxr import Gf, Sdf, UsdGeom

    ee_path = f"{top_prim_path}/{config.PLUG_PARENT_LINK}"
    ee = stage.GetPrimAtPath(ee_path)
    if not (ee and ee.IsValid()):
        log(f"[plug] WARNING: EE link prim {ee_path} not found; plug NOT baked")
        return None

    plug_path = f"{ee_path}/{config.PLUG_PRIM_NAME}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(Sdf.Path(plug_path))

    plug = UsdGeom.Xform.Define(stage, plug_path)
    xf = UsdGeom.Xformable(plug.GetPrim())
    # Canonical [translate, orient, scale] (IsaacLab-friendly); identity orient so the plug
    # points along EE +X (the reach/insertion direction).
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(v) for v in config.PLUG_OFFSET_XYZ]))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, Gf.Vec3d(0, 0, 0)))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))

    bl, br = config.PLUG_BODY_LENGTH, config.PLUG_BODY_RADIUS
    pl, pr = config.PLUG_PRONG_LENGTH, config.PLUG_PRONG_RADIUS
    sp = config.PLUG_PRONG_SPACING / 2.0

    body = UsdGeom.Cylinder.Define(stage, f"{plug_path}/body")
    body.CreateAxisAttr("X")
    body.CreateHeightAttr(bl)
    body.CreateRadiusAttr(br)
    body.CreateExtentAttr([(-bl / 2, -br, -br), (bl / 2, br, br)])
    UsdGeom.Xformable(body.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(bl / 2.0, 0.0, 0.0))
    _set_color(body, (0.08, 0.08, 0.10))  # dark plug body

    for sign, name in ((+1.0, "prong_l"), (-1.0, "prong_r")):
        pin = UsdGeom.Cylinder.Define(stage, f"{plug_path}/{name}")
        pin.CreateAxisAttr("X")
        pin.CreateHeightAttr(pl)
        pin.CreateRadiusAttr(pr)
        pin.CreateExtentAttr([(-pl / 2, -pr, -pr), (pl / 2, pr, pr)])
        UsdGeom.Xformable(pin.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(bl + pl / 2.0, sign * sp, 0.0))
        _set_color(pin, (0.95, 0.45, 0.05))  # bright orange pins (high contrast vs blue socket)

    tip_x = config.PLUG_OFFSET_XYZ[0] + bl + pl
    log(f"[plug] baked at {plug_path}: offset {config.PLUG_OFFSET_XYZ} m (EE +X), body {bl} m + "
        f"2 prongs {pl} m @ +/-{sp:.4f} m -> tip ~{tip_x:.3f} m ahead of wrist (EE frame)")
    return plug_path


def bake_socket(stage, prim_path=None, log=print, marker=False):
    """Create (or refresh) a socket (faceplate + 2 recessed holes [+ stand]) at prim_path.

    marker=False (default): a STATIC world socket on a stand, faceplate centre at
      SOCKET_POSE_XYZ, front face faces -X (toward the arm), static collider (no RigidBodyAPI).
    marker=True: a HOLE-CENTRED, stand-less, collider-less socket for use as a per-env RL
      VisualizationMarker — the front face (the hole plane) sits at the prim ORIGIN so that
      placing the marker at the plug-tip target puts the hole exactly on the target. The marker
      system supplies the translation/orientation, so the prim's own Xform translate is 0.
    Idempotent. Returns the socket prim path.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    prim_path = prim_path or config.SOCKET_PRIM
    if stage.GetPrimAtPath(prim_path).IsValid():
        stage.RemovePrim(Sdf.Path(prim_path))

    hx, hy, hz = config.SOCKET_FACEPLATE_HALF
    # marker: shift geometry +hx so the front face (was at -hx) lands at the origin.
    sx = hx if marker else 0.0

    socket = UsdGeom.Xform.Define(stage, prim_path)
    socket_xyz = (0.0, 0.0, 0.0) if marker else config.SOCKET_POSE_XYZ
    UsdGeom.Xformable(socket.GetPrim()).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*[float(v) for v in socket_xyz]))

    # Faceplate: a unit Cube (size 2 -> half-extent 1) scaled to the half-extents.
    plate = UsdGeom.Cube.Define(stage, f"{prim_path}/faceplate")
    plate.CreateSizeAttr(2.0)
    pxf = UsdGeom.Xformable(plate.GetPrim())
    pxf.AddTranslateOp().Set(Gf.Vec3d(sx, 0.0, 0.0))
    pxf.AddScaleOp().Set(Gf.Vec3d(hx, hy, hz))
    _set_color(plate, (0.16, 0.28, 0.55))  # blue faceplate (high contrast vs orange plug)
    if not marker:
        UsdPhysics.CollisionAPI.Apply(plate.GetPrim())  # static collider (no RigidBodyAPI)

    # Two dark recessed holes flush with the front face, going inward.
    hr, hd, sp = config.SOCKET_HOLE_RADIUS, config.SOCKET_HOLE_DEPTH, config.SOCKET_HOLE_SPACING / 2.0
    for sign, name in ((+1.0, "hole_l"), (-1.0, "hole_r")):
        hole = UsdGeom.Cylinder.Define(stage, f"{prim_path}/{name}")
        hole.CreateAxisAttr("X")
        hole.CreateHeightAttr(hd)
        hole.CreateRadiusAttr(hr)
        hole.CreateExtentAttr([(-hd / 2, -hr, -hr), (hd / 2, hr, hr)])
        UsdGeom.Xformable(hole.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(sx - hx + hd / 2.0, sign * sp, 0.0))  # front face at x = sx-hx
        # bright GREEN holes: a vision aid so the wrist-cam detector can robustly track the
        # hole targets (distinct from yellow prongs + blue plate). On the real arm, retune the
        # detector to the real hole appearance (dark recess) or use a small learned detector.
        _set_color(hole, (0.10, 0.90, 0.20))

    if not marker:
        # Support stand: a post filling ground (z=0) up to the faceplate bottom (static collider).
        sxh, syh = config.SOCKET_STAND_HALF[0], config.SOCKET_STAND_HALF[1]
        faceplate_bottom = config.SOCKET_POSE_XYZ[2] - hz          # world z of faceplate bottom
        stand_hz = max(0.005, faceplate_bottom / 2.0)              # half-height
        stand = UsdGeom.Cube.Define(stage, f"{prim_path}/stand")
        stand.CreateSizeAttr(2.0)
        sxf = UsdGeom.Xformable(stand.GetPrim())
        sxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, faceplate_bottom / 2.0 - config.SOCKET_POSE_XYZ[2]))
        sxf.AddScaleOp().Set(Gf.Vec3d(sxh, syh, stand_hz))
        _set_color(stand, (0.3, 0.3, 0.32))
        UsdPhysics.CollisionAPI.Apply(stand.GetPrim())

    _apply_semantics(socket.GetPrim(), config.SOCKET_SEMANTIC_CLASS, log)

    front_x = config.SOCKET_POSE_XYZ[0] - hx
    log(f"[socket] baked at {prim_path}: faceplate centre {config.SOCKET_POSE_XYZ} m, front face "
        f"x={front_x:.3f} m (faces -X / the arm), 2 holes r={hr} @ +/-{sp:.4f} m, class="
        f"{config.SOCKET_SEMANTIC_CLASS}")
    return prim_path
