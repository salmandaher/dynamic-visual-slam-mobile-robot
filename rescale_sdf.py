#!/usr/bin/env python3
"""Scale the robot to real 4-inch wheels + set real masses, via Sdf (offline,
robust). s = 4in / current_wheel_diameter (= 0.1016 / 0.976 = 0.10410).

Uniformly scales every robot prim's translate + unitsResolve scale, scales the
camera MOUNT but keeps its internal scale, sets masses (base 4.5 kg, back wheels
1.332 kg, front omni 0.206 kg) with auto inertia, and lowers the robot so the
small wheels rest on the ground."""
from pxr import Sdf, Gf

# IMPORTANT: operate on the COPY, never the user's new_built_robot.usd, and KEEP
# the SDF mesh colliders (the user's deliberate choice).
SCENE = "/home/salman/Documents/simsim/new_built_robot_gui.usd"
S = 0.1016 / 0.976                       # = 0.104098...
ROBOT = "/World/robot"
BASE = "/World/robot/robot_base"
WHEELS = [
    "/World/robot/_9055_txm_4_inch_wheel",
    "/World/robot/_9055_txm_4_inch_wheel_01",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
]
BACK_JOINTS = [
    "/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]
CAMERA = "/World/robot/d435i_camera"
MASSES = {                               # user spec
    BASE: 4.5,
    "/World/robot/_9055_txm_4_inch_wheel": 0.132,        # back wheels 132 g each
    "/World/robot/_9055_txm_4_inch_wheel_01": 0.132,
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel": 0.203,   # front omni 203 g each
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01": 0.203,
}
F = Sdf.ValueTypeNames.Float


def scale_vec(prim, name):
    a = prim.properties.get(name)
    if a is None or a.default is None:
        return False
    v = a.default
    a.default = type(v)(v[0]*S, v[1]*S, v[2]*S)
    return True


def main():
    L = Sdf.Layer.FindOrOpen(SCENE)
    assert L, SCENE
    print(f"scale ratio S = {S:.5f}  (wheel 0.976 -> {0.976*S:.4f} m)")

    # 1) base + wheels: scale position + geometry
    for path in [BASE] + WHEELS:
        p = L.GetPrimAtPath(path)
        assert p is not None, path
        t = scale_vec(p, "xformOp:translate")
        u = scale_vec(p, "xformOp:scale:unitsResolve")
        print(f"  {path.split('/')[-1]:34s} translate={t} unitsResolve={u}")

    # 2) camera: scale only the MOUNT position; keep xformOp:scale (0.001 = real
    #    D435i internal size — a real sensor doesn't shrink with the chassis).
    cam = L.GetPrimAtPath(CAMERA)
    scale_vec(cam, "xformOp:translate")
    tv = cam.properties.get("xformOp:translate").default
    sc = cam.properties.get("xformOp:scale").default
    print(f"  camera mount -> ({tv[0]:.3f},{tv[1]:.3f},{tv[2]:.3f}) m, internal scale kept {tuple(sc)}")

    # 3) masses + auto inertia
    for path, mass in MASSES.items():
        p = L.GetPrimAtPath(path)
        ma = p.properties.get("physics:mass") or Sdf.AttributeSpec(p, "physics:mass", F, Sdf.VariabilityVarying)
        ma.default = float(mass)
        di = p.properties.get("physics:diagonalInertia")
        if di is not None:
            p.RemoveProperty(di)
        print(f"  {path.split('/')[-1]:34s} mass={mass} kg, inertia auto")

    # 3b) fix contact params for the small scale. contactOffset must be wide enough
    #     to catch the wheel before it passes through in one 1/120 s step (a settling
    #     0.05 m wheel can move several mm/step); too small and the wheel sinks. Keep
    #     restOffset 0 and a fast depenetration velocity. 1 cm contactOffset on a 5 cm
    #     wheel is generous but stable.
    for path in WHEELS + [BASE]:
        mesh = L.GetPrimAtPath(path + "/node_/mesh_")
        if mesh is not None:
            co = mesh.properties.get("physxCollision:contactOffset") or \
                 Sdf.AttributeSpec(mesh, "physxCollision:contactOffset", F, Sdf.VariabilityVarying)
            co.default = 0.01
            ro = mesh.properties.get("physxCollision:restOffset")
            if ro is not None:
                ro.default = 0.0
        body = L.GetPrimAtPath(path)
        a = body.properties.get("physxRigidBody:maxDepenetrationVelocity")
        if a is not None and a.default is not None:
            a.default = 1.0   # fast correction so the wheel doesn't stay sunk
    print("  contact offsets -> 0.01 m, restOffset 0, maxDepenetrationVelocity 1.0")
    print("  colliders KEPT as SDF (user requirement)")

    # 3c) zero the baked back-wheel targetVelocity so it doesn't lurch at startup.
    for jp in BACK_JOINTS:
        j = L.GetPrimAtPath(jp)
        if j is None:
            continue
        a = j.properties.get("drive:angular:physics:targetVelocity")
        if a is not None:
            a.default = 0.0

    # 4) lower the robot so the (now small) wheels rest on the ground.
    # wheel bottom was at robot-local z 0.1692 (rested at world 0 with robot at -0.1692);
    # after scaling that local z -> 0.1692*S, so set robot z to put it ~3 mm above ground.
    rob = L.GetPrimAtPath(ROBOT)
    rt = rob.properties.get("xformOp:translate")
    v = rt.default
    new_z = 0.003 - 0.1692 * S
    rt.default = Gf.Vec3d(v[0], v[1], new_z)
    print(f"  robot z {v[2]:.4f} -> {new_z:.4f}")

    L.Save()
    print("SAVED:", SCENE)


if __name__ == "__main__":
    main()
