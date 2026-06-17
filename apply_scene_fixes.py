#!/usr/bin/env python3
"""Author the scene fixes + environment into new_built_robot.usd, in place.

This uses the pxr **Sdf** API only — NOT Usd.Stage — because Usd.Stage.Open
SEGFAULTS on this particular crate (same workaround the other scripts use).
Run with the miniconda python that has usd-core:

    python3 /home/salman/Documents/simsim/apply_scene_fixes.py

What it does (idempotent — safe to re-run; a second run is a no-op):
  1. Camera units fix: /World/robot/d435i_camera xformOp:transform diagonal
     0.01 (cm) -> 0.001 (mm). d435i.usd is authored in mm, so 0.01 over-scaled
     the camera's internal link offsets 10x (stereo baseline ~150 mm instead of
     15 mm). The 4th-row translate (1.5226209275265952, 0, 1.7192, 1) is left
     untouched so the mount position does not move.
     IMPORTANT pxr gotcha: `m[0][0] = v` is a no-op on Gf.Matrix4d (the index
     returns a temporary row copy) -> we MUST rewrite whole rows with SetRow().
  2. PhysicsScene at /physicsScene with Earth gravity for a Z-up, metres scene
     (gravityDirection=(0,0,-1), gravityMagnitude=9.81). The saved file had no
     explicit PhysicsScene, so gravity/solver settings were left to a runtime
     default.
  3. Environment under /World/Obstacles: static collider cubes/cylinders + two
     low walls for the robot to drive around (collision-only, no rigid body, so
     they stay put). Plus a DomeLight at /World/DomeLight for ambient fill (the
     scene previously had only a single DistantLight).

A backup (new_built_robot.usd.bak_before_drive_env_ros) was made before running.
"""
from pxr import Sdf, Gf

TARGET = "/home/salman/Documents/simsim/new_built_robot.usd"

# Static colliders => apiSchemas EXACTLY ["PhysicsCollisionAPI","PhysxCollisionAPI"]
# and NO rigid-body API (schema: "If there is no body in the parent hierarchy,
# this collider is considered static"). physics:approximation is omitted on
# analytic Cube/Cylinder shapes (it only applies to Mesh colliders).
SRC_USDA = r'''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def PhysicsScene "physicsScene"
{
    vector3f physics:gravityDirection = (0, 0, -1)
    float physics:gravityMagnitude = 9.81
}

def Xform "World"
{
    def Xform "Obstacles"
    {
        def Cube "Obstacle_box_red" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            double size = 1.0
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.85, 0.2, 0.2)]
            double3 xformOp:translate = (7, 0, 0.5)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cube "Obstacle_box_green" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            double size = 1.0
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.2, 0.8, 0.3)]
            double3 xformOp:translate = (-6, 5, 0.5)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cube "Obstacle_box_yellow" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            double size = 1.0
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.9, 0.8, 0.15)]
            double3 xformOp:translate = (5, -7, 0.5)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cylinder "Obstacle_cyl_blue" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            uniform token axis = "Z"
            double height = 1.2
            double radius = 0.4
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.2, 0.45, 0.9)]
            double3 xformOp:translate = (0, 8, 0.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cylinder "Obstacle_cyl_orange" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            uniform token axis = "Z"
            double height = 1.2
            double radius = 0.4
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.95, 0.5, 0.1)]
            double3 xformOp:translate = (-8, -3, 0.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Cube "Wall_north" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            double size = 1.0
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.6, 0.6, 0.65)]
            double3 xformOp:translate = (12, 0, 0.75)
            float3 xformOp:scale = (0.3, 8, 1.5)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }

        def Cube "Wall_east" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]
        )
        {
            double size = 1.0
            bool physics:collisionEnabled = 1
            float physxCollision:torsionalPatchRadius = 0.1
            color3f[] primvars:displayColor = [(0.6, 0.6, 0.65)]
            double3 xformOp:translate = (0, 12, 0.75)
            float3 xformOp:scale = (8, 0.3, 1.5)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }
    }

    def DomeLight "DomeLight"
    {
        float inputs:intensity = 1000
        color3f inputs:color = (1, 1, 1)
    }
}
'''


def ensure_xform(layer, path):
    """Make sure an Xform PrimSpec exists at `path` (create parents as needed)."""
    if layer.GetPrimAtPath(path) is None:
        parent = path.GetParentPath()
        if parent != Sdf.Path.absoluteRootPath:
            ensure_xform(layer, parent)
        pspec = (layer.GetPrimAtPath(parent)
                 if parent != Sdf.Path.absoluteRootPath else layer.pseudoRoot)
        Sdf.PrimSpec(pspec, path.name, Sdf.SpecifierDef, "Xform")
    return layer.GetPrimAtPath(path)


# --- physics-tuning helpers (operate on existing prims in the crate) --------
# Wheel rigid-body roots and their collision meshes.
WHEEL_ROOTS = [
    "/World/robot/_9055_txm_4_inch_wheel",
    "/World/robot/_9055_txm_4_inch_wheel_01",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
]
BASE = "/World/robot/robot_base"
WHEEL_MAT = "/World/Looks/wheels"
SCENE = "/physicsScene"


def set_attr(prim, name, vtype, value, uniform=False):
    """Create-or-overwrite an attribute default on a PrimSpec (idempotent)."""
    a = prim.properties.get(name)
    if a is None:
        var = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
        a = Sdf.AttributeSpec(prim, name, vtype, var)
    a.default = value
    return a


def del_attr(prim, name):
    """Remove an attribute spec if present (idempotent). Used to UNauthor
    diagonalInertia so PhysX auto-computes the tensor from the (convex-hull)
    collider geometry scaled to physics:mass — correct for ANY wheel size,
    instead of a hand-typed value that was right only for a 0.05 m wheel."""
    a = prim.properties.get(name)
    if a is not None:
        prim.RemoveProperty(a)
        return True
    return False


def add_api(prim, schema):
    """Append an applied API schema to a PrimSpec's apiSchemas list (idempotent),
    preserving the existing prepended/appended items of the token listOp."""
    listop = prim.GetInfo("apiSchemas")
    pre = list(listop.prependedItems)
    app = list(listop.appendedItems)
    exp = list(listop.explicitItems)
    if schema in pre or schema in app or schema in exp:
        return False
    newop = Sdf.TokenListOp()
    newop.prependedItems = pre + [schema]
    newop.appendedItems = app
    newop.deletedItems = list(listop.deletedItems)
    prim.SetInfo("apiSchemas", newop)
    return True


# Back (driven) revolute-joint drives.
BACK_JOINTS = [
    "/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]
# Front "dual omni" wheels: revolute-joint drives + collider meshes.
FRONT_JOINTS = [
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel/node_/mesh_/right_front",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01/node_/mesh_/left_front",
]
FRONT_MESHES = [
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel/node_/mesh_",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01/node_/mesh_",
]
CASTERS_MAT = "/World/Looks/casters"


def bind_physics_material(prim, mat_path):
    """Add a purpose='physics' material binding (material:binding:physics) to a
    collider PrimSpec — overrides the friction material WITHOUT touching render."""
    rel = prim.relationships.get("material:binding:physics")
    if rel is None:
        rel = Sdf.RelationshipSpec(prim, "material:binding:physics", False)
    rel.targetPathList.explicitItems = [Sdf.Path(mat_path)]


def make_front_casters(dst):
    """Turn the two FRONT dual-omni wheels into passive casters: zero their joint
    drives (stiffness=damping=0 -> zero torque -> free-spinning) so they are NOT
    driven, and bind a low-friction physics material so they don't resist turning.
    The two BACK wheels keep their drives (differential drive). Idempotent."""
    F = Sdf.ValueTypeNames.Float
    T = Sdf.ValueTypeNames.Token

    # 1) Zero the front joint drives -> passive free-spinning casters.
    for jp in FRONT_JOINTS:
        j = dst.GetPrimAtPath(jp)
        assert j is not None, f"front joint missing: {jp}"
        set_attr(j, "drive:angular:physics:targetVelocity", F, 0.0)
        set_attr(j, "drive:angular:physics:damping", F, 0.0)
        set_attr(j, "drive:angular:physics:stiffness", F, 0.0)

    # 2) Low-friction physics material for the casters (omni wheels slide laterally).
    if dst.GetPrimAtPath(CASTERS_MAT) is None:
        looks = dst.GetPrimAtPath("/World/Looks")
        assert looks is not None, "/World/Looks missing"
        mp = Sdf.PrimSpec(looks, "casters", Sdf.SpecifierDef, "Material")
        op = Sdf.TokenListOp()
        op.prependedItems = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
        mp.SetInfo("apiSchemas", op)
    mp = dst.GetPrimAtPath(CASTERS_MAT)
    # Low friction so the fixed (non-swivelling) front wheels skid easily when the
    # base turns — less turn resistance => turning closer to ideal diff-drive.
    set_attr(mp, "physics:staticFriction", F, 0.10)
    set_attr(mp, "physics:dynamicFriction", F, 0.06)
    set_attr(mp, "physics:restitution", F, 0.0)
    set_attr(mp, "physxMaterial:frictionCombineMode", T, "min", uniform=True)
    set_attr(mp, "physxMaterial:restitutionCombineMode", T, "min", uniform=True)

    # 3) Bind that material (physics purpose only) on the front wheel colliders.
    for mpath in FRONT_MESHES:
        mesh = dst.GetPrimAtPath(mpath)
        assert mesh is not None, f"front collider missing: {mpath}"
        bind_physics_material(mesh, CASTERS_MAT)

    print("front omni wheels -> passive casters (drives zeroed + low-friction material)")


def tune_physics(dst):
    """Make the articulation roll smoothly and CORRECTLY for the wheels' REAL size.

    Diagnostics (diagnose_motion.py / kinematics_probe.py / geom_inspect.py) showed
    the wheels are ~0.976 m diameter (0.488 m radius) and the base is ~2.9x3.7 m —
    i.e. the robot is genuinely metres-scale. The previous tuning was written for an
    imagined 0.05 m wheel: inertia 0.0008 (~100x too small) and SDF mesh colliders,
    which gave erratic, asymmetric, "slippy" rolling. This version:
      * collider sdf -> CONVEX HULL (a near-perfect cylinder of the round wheel mesh
        rolls smoothly & symmetrically; SDF contact on a big round mesh jitters),
      * UNauthors diagonalInertia so PhysX auto-computes the correct tensor from the
        convex hull scaled to physics:mass (right for any wheel size),
      * zeroes the back-wheel drive targets (the file had baked +/-200 rad/s),
      * higher wheel friction + a non-"min" combine so traction is real,
    plus the solver/scene settings as before. Idempotent."""
    F = Sdf.ValueTypeNames.Float
    I = Sdf.ValueTypeNames.Int
    U = Sdf.ValueTypeNames.UInt
    B = Sdf.ValueTypeNames.Bool
    T = Sdf.ValueTypeNames.Token

    # 1) PhysxSceneAPI on /physicsScene: TGS, stabilization, GPU, 120 Hz substeps.
    scene = dst.GetPrimAtPath(SCENE)
    assert scene is not None, "physicsScene missing — run the scene-build step first"
    add_api(scene, "PhysxSceneAPI")
    set_attr(scene, "physxScene:solverType", T, "TGS", uniform=True)
    set_attr(scene, "physxScene:enableStabilization", B, True)
    set_attr(scene, "physxScene:enableGPUDynamics", B, True)
    set_attr(scene, "physxScene:bounceThreshold", F, 0.5)
    set_attr(scene, "physxScene:timeStepsPerSecond", U, 120)

    # 2) Base: mass + AUTO inertia + articulation solver tuning + gentle depen.
    base = dst.GetPrimAtPath(BASE)
    assert base is not None, "robot_base missing"
    add_api(base, "PhysicsMassAPI")
    set_attr(base, "physics:mass", F, 30.0)
    del_attr(base, "physics:diagonalInertia")   # auto-compute (was 2.5: ~15x too small)
    # PhysxArticulationAPI already applied on robot_base -> just author the attrs.
    set_attr(base, "physxArticulation:solverPositionIterationCount", I, 32)
    set_attr(base, "physxArticulation:solverVelocityIterationCount", I, 4)
    set_attr(base, "physxArticulation:stabilizationThreshold", F, 0.001)
    set_attr(base, "physxArticulation:sleepThreshold", F, 0.0)
    set_attr(base, "physxArticulation:enabledSelfCollisions", B, False)
    # PhysxRigidBodyAPI already applied -> resolve any spawn intersection slowly.
    set_attr(base, "physxRigidBody:maxDepenetrationVelocity", F, 1.0)

    # 3) Each wheel rigid body: mass + AUTO inertia + CONVEX-HULL rolling collider.
    for wp in WHEEL_ROOTS:
        w = dst.GetPrimAtPath(wp)
        assert w is not None, f"wheel missing: {wp}"
        add_api(w, "PhysicsMassAPI")
        set_attr(w, "physics:mass", F, 0.6)
        del_attr(w, "physics:diagonalInertia")   # auto-compute (was 0.0008: ~100x too small)
        # PhysxRigidBodyAPI already applied on the wheel root.
        set_attr(w, "physxRigidBody:maxDepenetrationVelocity", F, 1.0)
        set_attr(w, "physxRigidBody:sleepThreshold", F, 0.0)           # a slow wheel must not sleep
        del_attr(w, "physxRigidBody:contactSlopCoefficient")          # was an SDF-rolling hack
        # collider lives on <wheel>/node_/mesh_ — switch SDF -> convex hull.
        mesh = dst.GetPrimAtPath(wp + "/node_/mesh_")
        assert mesh is not None, f"wheel collider missing: {wp}/node_/mesh_"
        add_api(mesh, "PhysxConvexHullCollisionAPI")
        set_attr(mesh, "physics:approximation", T, "convexHull", uniform=True)
        set_attr(mesh, "physxConvexHullCollision:hullVertexLimit", I, 128)  # round -> smooth roll
        set_attr(mesh, "physxCollision:contactOffset", F, 0.02)
        set_attr(mesh, "physxCollision:restOffset", F, 0.0)

    # 3b) Zero the BACK-wheel drive targets (the crate had baked +/-200 rad/s) — the
    #     Python controller is the single authoritative writer of velocity targets.
    #     Keep a stiff velocity drive (stiffness 0, damping for velocity tracking).
    for jp in BACK_JOINTS:
        j = dst.GetPrimAtPath(jp)
        assert j is not None, f"back joint missing: {jp}"
        set_attr(j, "drive:angular:physics:targetVelocity", F, 0.0)
        set_attr(j, "drive:angular:physics:stiffness", F, 0.0)
        set_attr(j, "drive:angular:physics:damping", F, 1500.0)

    # 4) Wheel material: high friction, no bounce, and a combine mode that does NOT
    #    collapse to the lowest surface (was "min", which throws away traction).
    mat = dst.GetPrimAtPath(WHEEL_MAT)
    assert mat is not None, "wheel material missing"
    set_attr(mat, "physics:staticFriction", F, 1.2)
    set_attr(mat, "physics:dynamicFriction", F, 1.0)
    set_attr(mat, "physics:restitution", F, 0.0)
    set_attr(mat, "physxMaterial:frictionCombineMode", T, "multiply", uniform=True)
    set_attr(mat, "physxMaterial:restitutionCombineMode", T, "min", uniform=True)
    # 4b) Ground material: high friction too (the drive wheels contact this).
    ground = dst.GetPrimAtPath("/World/Looks/ground")
    if ground is not None:
        set_attr(ground, "physics:staticFriction", F, 1.2)
        set_attr(ground, "physics:dynamicFriction", F, 1.0)
        set_attr(ground, "physxMaterial:frictionCombineMode", T, "multiply", uniform=True)
    print("physics tuned: convex-hull wheels, AUTO inertia, back targets zeroed, "
          "friction 1.2/1.0 (multiply), TGS/stabilized 120 Hz scene")


def main():
    src = Sdf.Layer.CreateAnonymous(".usda")
    src.ImportFromString(SRC_USDA)

    dst = Sdf.Layer.FindOrOpen(TARGET)
    assert dst is not None, f"could not open target layer: {TARGET}"

    changed = False

    # 1) PhysicsScene -> top level (idempotent)
    if dst.GetPrimAtPath("/physicsScene") is None:
        Sdf.CopySpec(src, Sdf.Path("/physicsScene"), dst, Sdf.Path("/physicsScene"))
        print("added /physicsScene (gravity (0,0,-1) * 9.81)")
        changed = True
    else:
        print("/physicsScene already present — skipped")

    # 2) Obstacles + DomeLight UNDER the existing /World (never CopySpec /World
    #    itself, that would clobber the real scene). Copy per-child so re-runs
    #    only add what's missing.
    ensure_xform(dst, Sdf.Path("/World"))
    if dst.GetPrimAtPath("/World/Obstacles") is None:
        Sdf.CopySpec(src, Sdf.Path("/World/Obstacles"), dst, Sdf.Path("/World/Obstacles"))
        print("added /World/Obstacles (5 props + 2 walls, static colliders)")
        changed = True
    else:
        for child in src.GetPrimAtPath("/World/Obstacles").nameChildren:
            dpath = Sdf.Path("/World/Obstacles").AppendChild(child.name)
            if dst.GetPrimAtPath(dpath) is None:
                Sdf.CopySpec(src, child.path, dst, dpath)
                print(f"added {dpath}")
                changed = True

    if dst.GetPrimAtPath("/World/DomeLight") is None:
        Sdf.CopySpec(src, Sdf.Path("/World/DomeLight"), dst, Sdf.Path("/World/DomeLight"))
        print("added /World/DomeLight (intensity 1000)")
        changed = True
    else:
        print("/World/DomeLight already present — skipped")

    # 3) Camera units fix: diagonal 0.01 -> 0.001, translate row 3 unchanged.
    CAM_ATTR = Sdf.Path("/World/robot/d435i_camera.xformOp:transform")
    attr = dst.GetAttributeAtPath(CAM_ATTR)
    assert attr is not None, "camera xformOp:transform attribute missing"
    m = attr.default
    if isinstance(m, Gf.Matrix4d) and abs(m[0][0] - 0.01) < 1e-9:
        n = Gf.Matrix4d(m)
        n.SetRow(0, Gf.Vec4d(0.001, m[0][1], m[0][2], m[0][3]))
        n.SetRow(1, Gf.Vec4d(m[1][0], 0.001, m[1][2], m[1][3]))
        n.SetRow(2, Gf.Vec4d(m[2][0], m[2][1], 0.001, m[2][3]))
        attr.default = n
        print(f"camera fix applied: diagonal 0.01 -> 0.001, translate row = {tuple(n.GetRow(3))}")
        changed = True
    else:
        print(f"camera fix skipped (diagonal already {m[0][0]})")

    # 4) Physics tuning for smooth rolling (idempotent — always re-applied).
    tune_physics(dst)
    # 5) Front omni wheels -> passive casters; differential drive on the back wheels.
    make_front_casters(dst)
    changed = True

    if changed:
        dst.Save()
        print("SAVED:", TARGET)
    else:
        print("no changes needed — file already up to date")


if __name__ == "__main__":
    main()
