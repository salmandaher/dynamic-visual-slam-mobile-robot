#!/usr/bin/env python3
"""Convert new_built_robot_4wd.usd into a 4-WHEEL-DRIVE mobile base.

The two front "dual omni" wheels are REMOVED (their payload is swapped from the
omni-wheel mesh to the same regular wheel mesh the back wheels use), turned into
normal GRIPPY DRIVEN wheels, and driven together with the back wheels (skid
steer). No more frictionless casters -> full traction, not slippery.

Sdf-only (run with miniconda usd-core):
    python3 build_4wd.py
"""
from pxr import Sdf

TARGET = "/home/salman/Documents/simsim/new_built_robot_4wd.usd"
OMNI_ASSET = "36466_txm-4 inch dual omni wheel.usd"
REG_ASSET = "39055_txm-4 inch wheel.usd"

FRONT_ROOTS = [
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01",
]
FRONT_MESHES = [r + "/node_/mesh_" for r in FRONT_ROOTS]
FRONT_JOINTS = [
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel/node_/mesh_/right_front",
    "/World/robot/_6466_txm_4_inch_dual_omni_wheel_01/node_/mesh_/left_front",
]
WHEELS_MAT = "/World/Looks/wheels"
F = Sdf.ValueTypeNames.Float
T = Sdf.ValueTypeNames.Token
B = Sdf.ValueTypeNames.Bool


def set_attr(prim, name, vtype, value, uniform=False):
    a = prim.properties.get(name)
    if a is None:
        var = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
        a = Sdf.AttributeSpec(prim, name, vtype, var)
    a.default = value
    return a


def swap_payload(spec):
    pl = spec.payloadList
    def fix(items):
        out = []
        for p in items:
            ap = p.assetPath.replace(OMNI_ASSET, REG_ASSET)
            out.append(Sdf.Payload(ap, p.primPath, p.layerOffset))
        return out
    for which in ("prependedItems", "explicitItems", "appendedItems"):
        items = getattr(pl, which)
        if items:
            setattr(pl, which, fix(items))


def main():
    L = Sdf.Layer.FindOrOpen(TARGET)
    assert L is not None, TARGET

    # 1) swap front omni mesh -> regular wheel mesh
    for root in FRONT_ROOTS:
        spec = L.GetPrimAtPath(root)
        assert spec is not None, root
        before = [p.assetPath for p in spec.payloadList.prependedItems] or \
                 [p.assetPath for p in spec.payloadList.explicitItems]
        swap_payload(spec)
        after = [p.assetPath for p in spec.payloadList.prependedItems] or \
                [p.assetPath for p in spec.payloadList.explicitItems]
        spec.SetInfo("displayName", "39055_txm-4 inch wheel (front, driven)")
        print(f"{root}\n   payload {before} -> {after}")

    # 2) front colliders: grippy wheel material + convex hull + collision on
    for mp in FRONT_MESHES:
        m = L.GetPrimAtPath(mp)
        assert m is not None, mp
        rel = m.relationships.get("material:binding:physics")
        if rel is None:
            rel = Sdf.RelationshipSpec(m, "material:binding:physics", False)
        rel.targetPathList.explicitItems = [Sdf.Path(WHEELS_MAT)]
        set_attr(m, "physics:collisionEnabled", B, True)
        set_attr(m, "physics:approximation", T, "convexHull", uniform=True)
        print(f"{mp}: material->wheels, convexHull, collision on")

    # 3) front joints: become driven (match the back wheels' velocity drive)
    for jp in FRONT_JOINTS:
        j = L.GetPrimAtPath(jp)
        assert j is not None, jp
        set_attr(j, "drive:angular:physics:stiffness", F, 0.0)
        set_attr(j, "drive:angular:physics:damping", F, 1500.0)
        set_attr(j, "drive:angular:physics:targetVelocity", F, 0.0)
        print(f"{jp}: driven (damping 1500)")

    L.Save()
    print("SAVED:", TARGET)


if __name__ == "__main__":
    main()
